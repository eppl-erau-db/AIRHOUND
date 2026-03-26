#!/usr/bin/env python3
"""
AIRHOUND Post-Flight Analysis

Extracts all paper metrics from a recorded rosbag and generates plots + CSV.
Run after every flight to get immediate results.

Usage:
    python3 analyze_flight.py ~/airhound_bags/flight_20260219_143000
    python3 analyze_flight.py ~/airhound_bags/sim_20260218_222227 --output-dir results/sim_01
    python3 analyze_flight.py --batch data/hitl_bags/ --compare

Produces (per bag):
    - yaw_tracking_error.pdf    (yaw rate command over time)
    - detection_timeline.pdf    (detection vs dropout intervals)
    - tracking_mode.pdf         (which fallback mode was active when)
    - latency_histogram.pdf     (inference latency distribution)
    - yaw_ground_truth.pdf      (PX4 yaw + command overlay)
    - metrics_summary.csv       (all scalar metrics for paper tables)
    - flight_summary.txt        (human-readable flight report)

Produces (batch --compare):
    - comparison/latency_boxplot.pdf
    - comparison/fps_bar.pdf
    - comparison/detection_rate_bar.pdf
    - comparison/tracking_mode_stacked.pdf
    - comparison/comparison.csv
    - comparison/table.tex

Supports SQLite3 (.db3/.db3.zstd) and MCAP bag formats.
"""

import argparse
import csv
import os
import sqlite3
import struct
import subprocess
import sys
import tempfile
from pathlib import Path

try:
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError:
    print("ERROR: numpy/matplotlib not installed.")
    sys.exit(1)


# ---------------------------------------------------------------------------
# Bag file discovery
# ---------------------------------------------------------------------------

def find_bag_file(bag_path: Path) -> Path:
    """Find the data file (.db3, .db3.zstd, or .mcap) inside a bag directory."""
    if bag_path.is_file():
        return bag_path

    # Check for SQLite3 bags first (both compressed and uncompressed)
    db3_zstd = sorted(bag_path.glob('*.db3.zstd'))
    if db3_zstd:
        return db3_zstd[0]
    db3 = sorted(bag_path.glob('*.db3'))
    if db3:
        return db3[0]
    mcaps = sorted(bag_path.glob('*.mcap'))
    if mcaps:
        return mcaps[0]

    print(f"ERROR: No .db3, .db3.zstd, or .mcap file found in {bag_path}")
    sys.exit(1)


# ---------------------------------------------------------------------------
# SQLite3 extraction (ROS2 bag format)
# ---------------------------------------------------------------------------

def extract_all_messages_db3(db3_path: Path) -> tuple:
    """
    Read a ROS2 SQLite3 bag file.
    Returns ({topic: [(timestamp_ns, raw_data), ...]}, {topic_id: (topic_name, type_name)}).
    Handles truncated/malformed databases gracefully (common with kill -9 recordings).
    """
    conn = sqlite3.connect(str(db3_path))
    cursor = conn.cursor()

    # Read topic table
    cursor.execute("SELECT id, name, type FROM topics")
    topics = {}  # id -> (name, type)
    for tid, name, ttype in cursor.fetchall():
        topics[tid] = (name, ttype)

    # Read all messages grouped by topic
    messages = {}  # topic_name -> [(timestamp_ns, data)]
    for tid, (tname, _) in topics.items():
        messages[tname] = []

    # Try direct read first, fall back to .recover if needed
    try:
        cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp ASC")
        for topic_id, timestamp, data in cursor:
            info = topics.get(topic_id)
            if info:
                tname, _ = info
                messages[tname].append((timestamp, bytes(data)))
        conn.close()
    except sqlite3.DatabaseError:
        conn.close()
        # Database is malformed -- use sqlite3 .recover
        recovered_path = recover_db3(db3_path)
        try:
            return extract_all_messages_db3(recovered_path)
        finally:
            recovered_path.unlink()

    # Convert topics dict to channel-like format for compatibility
    channels = {tid: (name, ttype) for tid, (name, ttype) in topics.items()}
    return messages, channels


def decompress_zstd(zstd_path: Path) -> Path:
    """Decompress a .db3.zstd file to a temp .db3 file. Returns path to temp file.
    Uses zstd CLI (falls back to zstandard Python module)."""
    raw_tmp = tempfile.NamedTemporaryFile(suffix='.db3', delete=False)
    raw_tmp.close()

    # Try zstd CLI first (handles truncated streams better than Python module)
    result = subprocess.run(
        ['zstd', '-d', '-f', str(zstd_path), '-o', raw_tmp.name],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        # CLI may fail on truncated streams — try Python module as fallback
        try:
            import zstandard
            dctx = zstandard.ZstdDecompressor()
            with open(zstd_path, 'rb') as fin, open(raw_tmp.name, 'wb') as fout:
                reader = dctx.stream_reader(fin)
                while True:
                    chunk = reader.read(65536)
                    if not chunk:
                        break
                    fout.write(chunk)
        except Exception:
            pass  # truncated stream — we got what we got

    # Check if the decompressed db3 is readable
    try:
        conn = sqlite3.connect(raw_tmp.name)
        # Use row-by-row read — aggregate queries fail on corrupt indexes
        cur = conn.cursor()
        cur.execute("SELECT COUNT(*) FROM topics")
        cur.close()
        conn.close()
        return Path(raw_tmp.name)
    except sqlite3.DatabaseError:
        pass  # need recovery

    # Recover corrupt db using sqlite3 .recover
    recovered_tmp = tempfile.NamedTemporaryFile(suffix='.db3', delete=False)
    recovered_tmp.close()
    print("  Recovering truncated database...")
    subprocess.run(
        f'sqlite3 {raw_tmp.name} ".recover" | sqlite3 {recovered_tmp.name}',
        shell=True, capture_output=True, text=True
    )
    os.unlink(raw_tmp.name)

    # Verify recovery worked
    try:
        conn = sqlite3.connect(recovered_tmp.name)
        count = conn.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
        conn.close()
        print(f"  Recovered {count} messages")
        return Path(recovered_tmp.name)
    except Exception as e:
        os.unlink(recovered_tmp.name)
        print(f"ERROR: Could not recover database: {e}")
        sys.exit(1)


# ---------------------------------------------------------------------------
# MCAP extraction (handles truncated files)
# ---------------------------------------------------------------------------

def extract_all_messages_mcap(mcap_path: Path) -> tuple:
    """
    Stream-read an MCAP file and return {topic: [(timestamp_ns, raw_data), ...]}.
    Works on truncated files where the footer is corrupt.
    """
    from mcap.stream_reader import StreamReader

    channels = {}   # channel_id -> (topic, schema_name)
    schemas = {}    # schema_id -> schema_name
    messages = {}   # topic -> [(ts_ns, raw_bytes)]

    with open(mcap_path, 'rb') as f:
        reader = StreamReader(f)
        try:
            for record in reader.records:
                rtype = type(record).__name__
                if rtype == 'Schema':
                    schemas[record.id] = record.name
                elif rtype == 'Channel':
                    sname = schemas.get(record.schema_id, '?')
                    channels[record.id] = (record.topic, sname)
                    if record.topic not in messages:
                        messages[record.topic] = []
                elif rtype == 'Message':
                    info = channels.get(record.channel_id)
                    if info:
                        topic, _ = info
                        messages[topic].append((record.log_time, record.data))
        except Exception:
            pass  # truncated file -- we got what we got

    return messages, channels


def recover_db3(db3_path: Path) -> Path:
    """Run sqlite3 .recover on a malformed db3 file. Returns path to recovered temp file."""
    recovered_tmp = tempfile.NamedTemporaryFile(suffix='.db3', delete=False)
    recovered_tmp.close()
    print("  Recovering malformed database...")
    subprocess.run(
        f'sqlite3 {db3_path} ".recover" | sqlite3 {recovered_tmp.name}',
        shell=True, capture_output=True, text=True
    )
    try:
        conn = sqlite3.connect(recovered_tmp.name)
        count = conn.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
        conn.close()
        print(f"  Recovered {count} messages")
        return Path(recovered_tmp.name)
    except Exception as e:
        os.unlink(recovered_tmp.name)
        print(f"ERROR: Could not recover database: {e}")
        sys.exit(1)


def extract_messages(bag_file: Path) -> tuple:
    """Auto-dispatch to the right extractor based on file extension."""
    suffix = ''.join(bag_file.suffixes)  # e.g. '.db3.zstd' or '.db3' or '.mcap'
    tmp_path = None
    try:
        if suffix.endswith('.db3.zstd'):
            print(f"  Decompressing {bag_file.name}...")
            tmp_path = decompress_zstd(bag_file)
            return extract_all_messages_db3(tmp_path)
        elif suffix.endswith('.db3'):
            return extract_all_messages_db3(bag_file)
        elif suffix.endswith('.mcap'):
            return extract_all_messages_mcap(bag_file)
        else:
            print(f"ERROR: Unknown bag format: {suffix}")
            sys.exit(1)
    finally:
        if tmp_path and tmp_path.exists():
            tmp_path.unlink()


# ---------------------------------------------------------------------------
# CDR deserialization helpers (minimal, no rosbags dependency)
# ---------------------------------------------------------------------------

def decode_float64(data: bytes) -> float:
    """Decode a std_msgs/Float64 CDR message."""
    # CDR: 4-byte header + 8-byte float64
    if len(data) < 12:
        return float('nan')
    return struct.unpack_from('<d', data, 4)[0]


def decode_float32(data: bytes) -> float:
    """Decode a std_msgs/Float32 CDR message."""
    if len(data) < 8:
        return float('nan')
    return struct.unpack_from('<f', data, 4)[0]


def decode_string(data: bytes) -> str:
    """Decode a std_msgs/String CDR message."""
    if len(data) < 8:
        return ''
    strlen = struct.unpack_from('<I', data, 4)[0]
    return data[8:8 + strlen - 1].decode('utf-8', errors='replace')


def decode_quaternion_yaw(data: bytes) -> float:
    """Decode yaw from px4_msgs/VehicleAttitude CDR (q field at known offset)."""
    # VehicleAttitude: timestamp(8) + timestamp_sample(8) + q[4](32) + ...
    # CDR header is 4 bytes, then padding to 8-byte alignment
    # q starts at offset 4 (header) + 4 (pad) + 8 (timestamp) + 8 (timestamp_sample) = 24
    if len(data) < 56:
        return float('nan')
    offset = 4 + 4 + 8 + 8  # CDR header + pad + 2 timestamps = 24
    w = struct.unpack_from('<f', data, offset)[0]
    x = struct.unpack_from('<f', data, offset + 4)[0]
    y = struct.unpack_from('<f', data, offset + 8)[0]
    z = struct.unpack_from('<f', data, offset + 12)[0]
    yaw_rad = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(yaw_rad)


def decode_detection_count(data: bytes) -> int:
    """Decode detection count from vision_msgs/Detection2DArray CDR.
    Just counts the array length field."""
    if len(data) < 20:
        return 0
    # Skip CDR encapsulation (4 bytes)
    # Header: stamp (sec:4 + nanosec:4) + frame_id (len:4 + string + padding)
    off = 4  # CDR header
    # stamp
    off += 8  # sec + nanosec
    # frame_id string
    if off + 4 > len(data):
        return 0
    fid_len = struct.unpack_from('<I', data, off)[0]
    off += 4 + fid_len
    # Align to 4 bytes
    off = (off + 3) & ~3
    # detections sequence length
    if off + 4 > len(data):
        return 0
    count = struct.unpack_from('<I', data, off)[0]
    # Sanity check
    return count if count < 1000 else 0


def decode_float64_multiarray(data: bytes) -> list:
    """Decode std_msgs/Float64MultiArray -- extract the data array."""
    if len(data) < 12:
        return []
    off = 4  # CDR header
    # MultiArrayLayout: dim sequence (uint32 len + entries), data_offset (uint32)
    if off + 4 > len(data):
        return []
    dim_len = struct.unpack_from('<I', data, off)[0]
    off += 4
    # Skip dim entries (each: label string + size uint32 + stride uint32)
    for _ in range(dim_len):
        if off + 4 > len(data):
            return []
        slen = struct.unpack_from('<I', data, off)[0]
        off += 4 + slen
        off = (off + 3) & ~3  # align
        off += 8  # size + stride
    # data_offset
    off += 4
    # Align to 8 for float64 array
    off = (off + 7) & ~7
    # data sequence: uint32 length then float64[]
    if off + 4 > len(data):
        return []
    arr_len = struct.unpack_from('<I', data, off)[0]
    off += 4
    off = (off + 7) & ~7  # align to 8
    values = []
    for i in range(arr_len):
        if off + 8 > len(data):
            break
        values.append(struct.unpack_from('<d', data, off)[0])
        off += 8
    return values


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def compute_dropout_intervals(det_times, det_counts, threshold_sec=0.1):
    """Find intervals where no detection was present for > threshold_sec."""
    if len(det_times) == 0:
        return []
    dropouts = []
    in_dropout = False
    dropout_start = 0.0
    for i in range(len(det_times)):
        if det_counts[i] == 0:
            if not in_dropout:
                in_dropout = True
                dropout_start = det_times[i]
        else:
            if in_dropout:
                duration = det_times[i] - dropout_start
                if duration >= threshold_sec:
                    dropouts.append((dropout_start, det_times[i], duration))
                in_dropout = False
    return dropouts


def analyze_bag(bag_path, output_dir):
    """Main analysis pipeline. Returns metrics dict (or None on failure)."""
    bag_path = Path(bag_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    bag_file = find_bag_file(bag_path)
    print(f"Analyzing: {bag_file}")
    print(f"Output:    {output_dir}")

    messages, channels = extract_messages(bag_file)

    # Report topics
    print(f"\nTopics in bag: {len(messages)}")
    for topic in sorted(messages.keys()):
        schema = '?'
        for cid, (t, s) in channels.items():
            if t == topic:
                schema = s
                break
        print(f"  {topic}: {len(messages[topic])} msgs ({schema})")

    # ---- Extract typed data ----
    print("\nExtracting data...")

    def to_timeseries(topic, decoder):
        raw = messages.get(topic, [])
        if not raw:
            return np.array([]), np.array([])
        times = np.array([ts * 1e-9 for ts, _ in raw])
        values = np.array([decoder(d) for _, d in raw])
        mask = ~np.isnan(values)
        return times[mask], values[mask]

    yaw_t, yaw_cmd = to_timeseries('/yaw_command', decode_float64)
    print(f"  /yaw_command: {len(yaw_t)} messages")

    mode_t, mode_val = to_timeseries('/tracking/mode', decode_float64)
    print(f"  /tracking/mode: {len(mode_t)} messages")

    att_t, att_yaw = to_timeseries('/fmu/out/vehicle_attitude', decode_quaternion_yaw)
    print(f"  /fmu/out/vehicle_attitude: {len(att_t)} messages")

    # Detections
    det_raw = messages.get('/detections', [])
    det_t = np.array([ts * 1e-9 for ts, _ in det_raw])
    det_counts = np.array([decode_detection_count(d) for _, d in det_raw])
    print(f"  /detections: {len(det_t)} messages")

    lat_t, lat_ms = to_timeseries('/perception/latency_ms', decode_float32)
    print(f"  /perception/latency_ms: {len(lat_t)} messages")

    fps_t, fps_val = to_timeseries('/perception/fps', decode_float32)
    print(f"  /perception/fps: {len(fps_t)} messages")

    # Kalman state
    kalman_raw = messages.get('/tracking/kalman_state', [])
    kalman_t = []
    kalman_states = []
    for ts, d in kalman_raw:
        vals = decode_float64_multiarray(d)
        if vals:
            kalman_t.append(ts * 1e-9)
            kalman_states.append(vals)

    # ---- Compute metrics ----
    print("\nComputing metrics...")
    metrics = {}
    metrics['bag_name'] = bag_path.name

    all_times = [t for t in [yaw_t, att_t, det_t, lat_t] if len(t) > 0]
    if not all_times:
        print("ERROR: No data found in bag. Check topic names.")
        return None
    t0 = min(t[0] for t in all_times)
    flight_duration = max(t[-1] for t in all_times) - t0
    metrics['flight_duration_sec'] = round(flight_duration, 1)

    # Normalize time
    if len(yaw_t) > 0: yaw_t -= t0
    if len(att_t) > 0: att_t -= t0
    if len(det_t) > 0: det_t -= t0
    if len(lat_t) > 0: lat_t -= t0
    if len(fps_t) > 0: fps_t -= t0
    if len(mode_t) > 0: mode_t -= t0
    if kalman_t:
        kalman_t = [t - t0 for t in kalman_t]

    # Yaw command stats
    if len(yaw_cmd) > 0:
        yaw_cmd_deg = np.degrees(yaw_cmd)
        metrics['yaw_cmd_mean_deg_s'] = round(float(np.mean(np.abs(yaw_cmd_deg))), 2)
        metrics['yaw_cmd_max_deg_s'] = round(float(np.max(np.abs(yaw_cmd_deg))), 2)
        metrics['yaw_cmd_std_deg_s'] = round(float(np.std(yaw_cmd_deg)), 2)

    # Detection stats
    if len(det_counts) > 0:
        detection_rate = np.mean(det_counts > 0) * 100
        metrics['detection_rate_pct'] = round(float(detection_rate), 1)
        dropouts = compute_dropout_intervals(det_t, det_counts)
        metrics['num_dropouts'] = len(dropouts)
        if dropouts:
            durations = [d[2] for d in dropouts]
            metrics['dropout_mean_sec'] = round(float(np.mean(durations)), 3)
            metrics['dropout_max_sec'] = round(float(np.max(durations)), 3)
            metrics['dropout_total_sec'] = round(float(np.sum(durations)), 2)

    # Latency stats
    if len(lat_ms) > 0:
        metrics['latency_mean_ms'] = round(float(np.mean(lat_ms)), 1)
        metrics['latency_p50_ms'] = round(float(np.percentile(lat_ms, 50)), 1)
        metrics['latency_p95_ms'] = round(float(np.percentile(lat_ms, 95)), 1)
        metrics['latency_p99_ms'] = round(float(np.percentile(lat_ms, 99)), 1)
        metrics['latency_max_ms'] = round(float(np.max(lat_ms)), 1)

    # FPS stats
    if len(fps_val) > 0:
        metrics['fps_mean'] = round(float(np.mean(fps_val)), 1)
        metrics['fps_min'] = round(float(np.min(fps_val)), 1)

    # Tracking mode breakdown
    # Mode encoding: 0=zero/no_target, 1=detection, 2=kalman, 3=pinn, 4=hold, 5=decay
    if len(mode_val) > 0:
        mode_labels = {0: 'zero', 1: 'detection', 2: 'kalman', 3: 'pinn', 4: 'hold', 5: 'decay'}
        for code, label in mode_labels.items():
            pct = np.mean(mode_val == code) * 100
            metrics[f'mode_{label}_pct'] = round(float(pct), 1)

    # Store raw latency values for comparison plots
    metrics['_latency_raw'] = lat_ms

    # ---- Generate plots ----
    print("\nGenerating plots...")
    plt.rcParams.update({'font.size': 10, 'figure.dpi': 150})

    # 1. Yaw rate command over time
    if len(yaw_cmd) > 0:
        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(yaw_t, np.degrees(yaw_cmd), linewidth=0.5, color='steelblue')
        ax.axhline(y=0, color='gray', linewidth=0.5, linestyle='--')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw Command (\u00b0/s)')
        ax.set_title('Yaw Rate Command')
        ax.set_xlim(0, flight_duration)
        fig.tight_layout()
        fig.savefig(output_dir / 'yaw_tracking_error.pdf')
        plt.close(fig)
        print("  -> yaw_tracking_error.pdf")

    # 2. Detection timeline
    if len(det_t) > 0:
        fig, ax = plt.subplots(figsize=(10, 2))
        detected = det_counts > 0
        ax.fill_between(det_t, 0, 1, where=detected,
                         color='green', alpha=0.4, label='Detected')
        ax.fill_between(det_t, 0, 1, where=~detected,
                         color='red', alpha=0.4, label='Dropout')
        ax.set_xlabel('Time (s)')
        ax.set_yticks([])
        ax.set_title('Detection Timeline')
        ax.legend(loc='upper right', fontsize=8)
        ax.set_xlim(0, flight_duration)
        fig.tight_layout()
        fig.savefig(output_dir / 'detection_timeline.pdf')
        plt.close(fig)
        print("  -> detection_timeline.pdf")

    # 3. Tracking mode
    if len(mode_val) > 0:
        fig, ax = plt.subplots(figsize=(10, 2))
        colors = {0: 'gray', 1: 'green', 2: 'orange', 3: 'blue', 4: 'red', 5: 'salmon'}
        labels = {0: 'Zero', 1: 'Detection', 2: 'Kalman', 3: 'PINN', 4: 'Hold', 5: 'Decay'}
        for code in sorted(colors.keys()):
            mask = mode_val == code
            if np.any(mask):
                ax.fill_between(mode_t, 0, 1, where=mask,
                                 color=colors[code], alpha=0.5, label=labels[code])
        ax.set_xlabel('Time (s)')
        ax.set_yticks([])
        ax.set_title('Tracking Mode')
        ax.legend(loc='upper right', fontsize=8)
        ax.set_xlim(0, flight_duration)
        fig.tight_layout()
        fig.savefig(output_dir / 'tracking_mode.pdf')
        plt.close(fig)
        print("  -> tracking_mode.pdf")

    # 4. Latency histogram
    if len(lat_ms) > 0:
        fig, ax = plt.subplots(figsize=(6, 3))
        ax.hist(lat_ms, bins=50, color='steelblue', edgecolor='white', linewidth=0.3)
        ax.axvline(np.mean(lat_ms), color='red', linewidth=1,
                    label=f'Mean: {np.mean(lat_ms):.1f} ms')
        ax.axvline(np.percentile(lat_ms, 95), color='orange', linewidth=1,
                    linestyle='--', label=f'P95: {np.percentile(lat_ms, 95):.1f} ms')
        ax.set_xlabel('Inference Latency (ms)')
        ax.set_ylabel('Count')
        ax.set_title('Detection Inference Latency')
        ax.legend(fontsize=8)
        fig.tight_layout()
        fig.savefig(output_dir / 'latency_histogram.pdf')
        plt.close(fig)
        print("  -> latency_histogram.pdf")

    # 5. Ground truth yaw + command overlay
    if len(att_yaw) > 0 and len(yaw_cmd) > 0:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
        ax1.plot(att_t, att_yaw, linewidth=0.5, color='darkblue')
        ax1.set_ylabel('Drone Yaw (\u00b0)')
        ax1.set_title('Ground Truth Yaw (PX4)')
        ax2.plot(yaw_t, np.degrees(yaw_cmd), linewidth=0.5, color='steelblue')
        ax2.axhline(y=0, color='gray', linewidth=0.5, linestyle='--')
        ax2.set_ylabel('Yaw Command (\u00b0/s)')
        ax2.set_xlabel('Time (s)')
        ax2.set_title('Yaw Rate Command')
        ax1.set_xlim(0, flight_duration)
        fig.tight_layout()
        fig.savefig(output_dir / 'yaw_ground_truth.pdf')
        plt.close(fig)
        print("  -> yaw_ground_truth.pdf")

    # ---- Write CSV ----
    csv_path = output_dir / 'metrics_summary.csv'
    csv_metrics = {k: v for k, v in metrics.items() if not k.startswith('_')}
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['metric', 'value'])
        for k, v in sorted(csv_metrics.items()):
            writer.writerow([k, v])
    print(f"\n  -> metrics_summary.csv ({len(csv_metrics)} metrics)")

    # ---- Write summary ----
    summary_path = output_dir / 'flight_summary.txt'
    with open(summary_path, 'w') as f:
        f.write(f"AIRHOUND Flight Analysis\n")
        f.write(f"{'=' * 50}\n")
        f.write(f"Bag: {bag_path}\n")
        f.write(f"Duration: {metrics.get('flight_duration_sec', '?')} s\n\n")

        f.write("Detection:\n")
        f.write(f"  Rate:        {metrics.get('detection_rate_pct', '?')}%\n")
        f.write(f"  Dropouts:    {metrics.get('num_dropouts', '?')}\n")
        f.write(f"  Max dropout: {metrics.get('dropout_max_sec', '?')} s\n\n")

        f.write("Latency:\n")
        f.write(f"  Mean:  {metrics.get('latency_mean_ms', '?')} ms\n")
        f.write(f"  P95:   {metrics.get('latency_p95_ms', '?')} ms\n")
        f.write(f"  P99:   {metrics.get('latency_p99_ms', '?')} ms\n\n")

        f.write("FPS:\n")
        f.write(f"  Mean:  {metrics.get('fps_mean', '?')}\n")
        f.write(f"  Min:   {metrics.get('fps_min', '?')}\n\n")

        f.write("Control:\n")
        f.write(f"  Mean |yaw cmd|: {metrics.get('yaw_cmd_mean_deg_s', '?')} deg/s\n")
        f.write(f"  Max  |yaw cmd|: {metrics.get('yaw_cmd_max_deg_s', '?')} deg/s\n\n")

        f.write("Tracking Mode Breakdown:\n")
        for mode in ['detection', 'kalman', 'pinn', 'hold', 'decay', 'zero']:
            f.write(f"  {mode:12s}: {metrics.get(f'mode_{mode}_pct', '?')}%\n")

    print(f"  -> flight_summary.txt")
    print(f"\nDone. {len(csv_metrics)} metrics extracted.")

    # Print summary to stdout
    print(f"\n{'=' * 50}")
    print(f"FLIGHT SUMMARY")
    print(f"{'=' * 50}")
    with open(summary_path) as f:
        print(f.read())

    return metrics


# ---------------------------------------------------------------------------
# Batch + comparison
# ---------------------------------------------------------------------------

def run_batch(bags_dir: Path, do_compare: bool):
    """Run analysis on all bag subdirectories and optionally generate comparison."""
    bags_dir = Path(bags_dir)
    subdirs = sorted([d for d in bags_dir.iterdir() if d.is_dir() and d.name != 'comparison'])

    if not subdirs:
        print(f"ERROR: No subdirectories found in {bags_dir}")
        sys.exit(1)

    print(f"Found {len(subdirs)} bags in {bags_dir}\n")

    all_metrics = []
    for sd in subdirs:
        print(f"\n{'#' * 60}")
        print(f"# {sd.name}")
        print(f"{'#' * 60}\n")
        output_dir = sd / 'analysis'
        m = analyze_bag(sd, output_dir)
        if m is not None:
            all_metrics.append(m)

    if not all_metrics:
        print("\nNo bags produced metrics.")
        return

    # Write combined comparison CSV
    comp_dir = bags_dir / 'comparison'
    comp_dir.mkdir(parents=True, exist_ok=True)

    # Collect all scalar keys across bags
    all_keys = set()
    for m in all_metrics:
        all_keys.update(k for k in m.keys() if not k.startswith('_'))
    col_order = ['bag_name', 'flight_duration_sec',
                 'detection_rate_pct', 'num_dropouts', 'dropout_max_sec',
                 'latency_mean_ms', 'latency_p50_ms', 'latency_p95_ms', 'latency_p99_ms',
                 'fps_mean', 'fps_min',
                 'yaw_cmd_mean_deg_s', 'yaw_cmd_max_deg_s',
                 'mode_detection_pct', 'mode_kalman_pct', 'mode_pinn_pct',
                 'mode_hold_pct', 'mode_decay_pct', 'mode_zero_pct']
    # Add any keys not in our preferred order
    extra = sorted(all_keys - set(col_order))
    columns = [c for c in col_order if c in all_keys] + extra

    csv_path = comp_dir / 'comparison.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        for m in all_metrics:
            row = {k: v for k, v in m.items() if not k.startswith('_')}
            writer.writerow(row)
    print(f"\n-> {csv_path}")

    if do_compare:
        generate_comparison(all_metrics, comp_dir)


def generate_comparison(all_metrics: list, comp_dir: Path):
    """Generate cross-bag comparison plots and LaTeX table."""
    plt.rcParams.update({'font.size': 10, 'figure.dpi': 150})

    names = [m['bag_name'] for m in all_metrics]
    # Shorten names for plot labels: use last component, trim common prefix
    short_names = []
    for n in names:
        # e.g. sim_20260309_134700 -> sim_0309
        # e.g. flight_20260311_145227 -> flight_145227
        parts = n.split('_')
        if len(parts) >= 3:
            prefix = parts[0]  # sim or flight
            time_part = parts[-1]  # last segment (time)
            short_names.append(f"{prefix}_{time_part}")
        else:
            short_names.append(n)

    # 1. Latency box plot
    latency_data = []
    latency_labels = []
    for m, sn in zip(all_metrics, short_names):
        raw = m.get('_latency_raw', np.array([]))
        if len(raw) > 0:
            latency_data.append(raw)
            latency_labels.append(sn)

    if latency_data:
        fig, ax = plt.subplots(figsize=(max(8, len(latency_data) * 1.5), 4))
        bp = ax.boxplot(latency_data, tick_labels=latency_labels, patch_artist=True,
                        showfliers=False, medianprops=dict(color='red', linewidth=1.5))
        colors_cycle = ['#4C72B0', '#55A868', '#C44E52', '#8172B2', '#CCB974', '#64B5CD']
        for i, patch in enumerate(bp['boxes']):
            patch.set_facecolor(colors_cycle[i % len(colors_cycle)])
            patch.set_alpha(0.7)
        ax.set_ylabel('Latency (ms)')
        ax.set_title('Inference Latency Distribution by Configuration')
        plt.xticks(rotation=30, ha='right')
        fig.tight_layout()
        fig.savefig(comp_dir / 'latency_boxplot.pdf')
        plt.close(fig)
        print("  -> latency_boxplot.pdf")

    # 2. FPS bar chart
    fps_data = [(sn, m.get('fps_mean', 0)) for m, sn in zip(all_metrics, short_names)
                if 'fps_mean' in m]
    if fps_data:
        fig, ax = plt.subplots(figsize=(max(7, len(fps_data) * 1.3), 4))
        fps_names, fps_vals = zip(*fps_data)
        bars = ax.bar(fps_names, fps_vals, color='#4C72B0', alpha=0.8, edgecolor='white')
        for bar, val in zip(bars, fps_vals):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.3,
                    f'{val:.1f}', ha='center', va='bottom', fontsize=9)
        ax.set_ylabel('Mean FPS')
        ax.set_title('Mean Detection FPS by Configuration')
        plt.xticks(rotation=30, ha='right')
        fig.tight_layout()
        fig.savefig(comp_dir / 'fps_bar.pdf')
        plt.close(fig)
        print("  -> fps_bar.pdf")

    # 3. Detection rate bar chart
    det_data = [(sn, m.get('detection_rate_pct', 0)) for m, sn in zip(all_metrics, short_names)
                if 'detection_rate_pct' in m]
    if det_data:
        fig, ax = plt.subplots(figsize=(max(7, len(det_data) * 1.3), 4))
        det_names, det_vals = zip(*det_data)
        bars = ax.bar(det_names, det_vals, color='#55A868', alpha=0.8, edgecolor='white')
        for bar, val in zip(bars, det_vals):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.5,
                    f'{val:.1f}%', ha='center', va='bottom', fontsize=9)
        ax.set_ylabel('Detection Rate (%)')
        ax.set_title('Detection Rate by Configuration')
        ax.set_ylim(0, 105)
        plt.xticks(rotation=30, ha='right')
        fig.tight_layout()
        fig.savefig(comp_dir / 'detection_rate_bar.pdf')
        plt.close(fig)
        print("  -> detection_rate_bar.pdf")

    # 4. Tracking mode stacked bar
    mode_keys = ['mode_detection_pct', 'mode_kalman_pct', 'mode_pinn_pct',
                 'mode_hold_pct', 'mode_decay_pct', 'mode_zero_pct']
    mode_labels_plot = ['Detection', 'Kalman', 'PINN', 'Hold', 'Decay', 'Zero']
    mode_colors = ['#55A868', '#F0A030', '#4C72B0', '#C44E52', '#E8A0A0', '#BBBBBB']

    has_mode = any(any(k in m for k in mode_keys) for m in all_metrics)
    if has_mode:
        fig, ax = plt.subplots(figsize=(max(8, len(all_metrics) * 1.5), 4))
        x = np.arange(len(short_names))
        bottoms = np.zeros(len(short_names))
        for mk, ml, mc in zip(mode_keys, mode_labels_plot, mode_colors):
            vals = np.array([m.get(mk, 0) for m in all_metrics])
            ax.bar(x, vals, bottom=bottoms, label=ml, color=mc, alpha=0.8, edgecolor='white')
            bottoms += vals
        ax.set_xticks(x)
        ax.set_xticklabels(short_names, rotation=30, ha='right')
        ax.set_ylabel('Time (%)')
        ax.set_title('Tracking Mode Breakdown by Configuration')
        ax.legend(loc='upper right', fontsize=8)
        ax.set_ylim(0, 105)
        fig.tight_layout()
        fig.savefig(comp_dir / 'tracking_mode_stacked.pdf')
        plt.close(fig)
        print("  -> tracking_mode_stacked.pdf")

    # 5. LaTeX table
    tex_path = comp_dir / 'table.tex'
    with open(tex_path, 'w') as f:
        f.write("% Auto-generated by analyze_flight.py --compare\n")
        f.write("\\begin{table}[htbp]\n")
        f.write("\\centering\n")
        f.write("\\caption{HITL Test Configuration Comparison}\n")
        f.write("\\label{tab:hitl-comparison}\n")
        f.write("\\begin{tabular}{l r r r r r r}\n")
        f.write("\\toprule\n")
        f.write("Configuration & Duration & Det.~Rate & Latency & P95 & FPS & Dropouts \\\\\n")
        f.write(" & (s) & (\\%) & (ms) & (ms) & (mean) & \\\\\n")
        f.write("\\midrule\n")
        for m, sn in zip(all_metrics, short_names):
            dur = m.get('flight_duration_sec', '--')
            det = m.get('detection_rate_pct', '--')
            lat = m.get('latency_mean_ms', '--')
            p95 = m.get('latency_p95_ms', '--')
            fps = m.get('fps_mean', '--')
            ndo = m.get('num_dropouts', '--')
            sn_tex = sn.replace('_', '\\_')
            f.write(f"{sn_tex} & {dur} & {det} & {lat} & {p95} & {fps} & {ndo} \\\\\n")
        f.write("\\bottomrule\n")
        f.write("\\end{tabular}\n")
        f.write("\\end{table}\n")
    print(f"  -> table.tex")

    print(f"\nComparison outputs in {comp_dir}")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='AIRHOUND post-flight analysis')
    parser.add_argument('bag_path', nargs='?', default=None,
                        help='Path to rosbag directory or file')
    parser.add_argument('--output-dir', '-o', default=None,
                        help='Output directory (default: <bag_path>/analysis)')
    parser.add_argument('--batch', metavar='DIR',
                        help='Run analysis on all bag subdirectories in DIR')
    parser.add_argument('--compare', action='store_true',
                        help='Generate cross-bag comparison plots (requires --batch)')
    args = parser.parse_args()

    if args.batch:
        run_batch(Path(args.batch), args.compare)
    elif args.bag_path:
        output = args.output_dir or os.path.join(args.bag_path, 'analysis')
        analyze_bag(args.bag_path, output)
    else:
        parser.print_help()
        sys.exit(1)
