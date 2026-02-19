#!/usr/bin/env python3
"""
AIRHOUND Post-Flight Analysis

Extracts all paper metrics from a recorded rosbag and generates plots + CSV.
Run after every flight to get immediate results.

Usage:
    python3 analyze_flight.py ~/airhound_bags/flight_20260219_143000
    python3 analyze_flight.py ~/airhound_bags/sim_20260218_222227 --output-dir results/sim_01

Produces:
    - yaw_tracking_error.pdf    (yaw rate command over time)
    - detection_timeline.pdf    (detection vs dropout intervals)
    - tracking_mode.pdf         (which fallback mode was active when)
    - latency_histogram.pdf     (inference latency distribution)
    - yaw_ground_truth.pdf      (PX4 yaw + command overlay)
    - metrics_summary.csv       (all scalar metrics for paper tables)
    - flight_summary.txt        (human-readable flight report)

Supports both clean and truncated MCAP bags (handles kill -9 gracefully).
"""

import argparse
import csv
import os
import struct
import sys
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
# MCAP extraction (handles truncated files)
# ---------------------------------------------------------------------------

def find_mcap_file(bag_path: Path) -> Path:
    """Find the .mcap file inside a bag directory."""
    if bag_path.is_file() and bag_path.suffix == '.mcap':
        return bag_path
    mcaps = sorted(bag_path.glob('*.mcap'))
    if not mcaps:
        print(f"ERROR: No .mcap file found in {bag_path}")
        sys.exit(1)
    return mcaps[0]


def extract_all_messages(mcap_path: Path) -> dict:
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
            pass  # truncated file — we got what we got

    return messages, channels


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
    # header (stamp + frame_id) then detections sequence
    # This is tricky with CDR. Simpler: count is encoded as uint32 before the array.
    # For Detection2DArray: header + sequence<Detection2D>
    # Rough: scan for sequence length after the header
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
    """Decode std_msgs/Float64MultiArray — extract the data array."""
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
    """Main analysis pipeline."""
    bag_path = Path(bag_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    mcap_path = find_mcap_file(bag_path)
    print(f"Analyzing: {mcap_path}")
    print(f"Output:    {output_dir}")

    messages, channels = extract_all_messages(mcap_path)

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

    all_times = [t for t in [yaw_t, att_t, det_t, lat_t] if len(t) > 0]
    if not all_times:
        print("ERROR: No data found in bag. Check topic names.")
        return
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
    if len(mode_val) > 0:
        mode_labels = {0: 'hold', 1: 'detection', 2: 'kalman', 3: 'pinn'}
        for code, label in mode_labels.items():
            pct = np.mean(mode_val == code) * 100
            metrics[f'mode_{label}_pct'] = round(float(pct), 1)

    # ---- Generate plots ----
    print("\nGenerating plots...")
    plt.rcParams.update({'font.size': 10, 'figure.dpi': 150})

    # 1. Yaw rate command over time
    if len(yaw_cmd) > 0:
        fig, ax = plt.subplots(figsize=(10, 3))
        ax.plot(yaw_t, np.degrees(yaw_cmd), linewidth=0.5, color='steelblue')
        ax.axhline(y=0, color='gray', linewidth=0.5, linestyle='--')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Yaw Command (°/s)')
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
        colors = {0: 'red', 1: 'green', 2: 'orange', 3: 'blue'}
        labels = {0: 'Hold', 1: 'Detection', 2: 'Kalman', 3: 'PINN'}
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
        ax1.set_ylabel('Drone Yaw (°)')
        ax1.set_title('Ground Truth Yaw (PX4)')
        ax2.plot(yaw_t, np.degrees(yaw_cmd), linewidth=0.5, color='steelblue')
        ax2.axhline(y=0, color='gray', linewidth=0.5, linestyle='--')
        ax2.set_ylabel('Yaw Command (°/s)')
        ax2.set_xlabel('Time (s)')
        ax2.set_title('Yaw Rate Command')
        ax1.set_xlim(0, flight_duration)
        fig.tight_layout()
        fig.savefig(output_dir / 'yaw_ground_truth.pdf')
        plt.close(fig)
        print("  -> yaw_ground_truth.pdf")

    # ---- Write CSV ----
    csv_path = output_dir / 'metrics_summary.csv'
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['metric', 'value'])
        for k, v in sorted(metrics.items()):
            writer.writerow([k, v])
    print(f"\n  -> metrics_summary.csv ({len(metrics)} metrics)")

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
        for mode in ['detection', 'kalman', 'pinn', 'hold']:
            f.write(f"  {mode:12s}: {metrics.get(f'mode_{mode}_pct', '?')}%\n")

    print(f"  -> flight_summary.txt")
    print(f"\nDone. {len(metrics)} metrics extracted.")

    # Print summary to stdout
    print(f"\n{'=' * 50}")
    print(f"FLIGHT SUMMARY")
    print(f"{'=' * 50}")
    with open(summary_path) as f:
        print(f.read())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='AIRHOUND post-flight analysis')
    parser.add_argument('bag_path', help='Path to rosbag directory or .mcap file')
    parser.add_argument('--output-dir', '-o', default=None,
                        help='Output directory (default: <bag_path>/analysis)')
    args = parser.parse_args()

    output = args.output_dir or os.path.join(args.bag_path, 'analysis')
    analyze_bag(args.bag_path, output)
