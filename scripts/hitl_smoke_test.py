#!/usr/bin/env python3
"""
AIRHOUND HITL Smoke Test

Sends HIL_SENSOR + HIL_GPS, confirms PX4 responds with HIL_ACTUATOR_CONTROLS.
IMPORTANT: Use udpout: not udp: for pymavlink connection string.
"""

import time
import sys
from pymavlink import mavutil

PX4_ADDR = "udpout:192.168.0.4:14550"
DURATION = 5  # seconds

LAT = 29.1868
LON = -81.0489
ALT = 10.0
GRAVITY = 9.81
MAG_X, MAG_Y, MAG_Z = 0.21, 0.0, 0.42
PRESSURE = 1013.25
TEMP = 25.0


def main():
    print("=" * 60)
    print("  AIRHOUND HITL Smoke Test")
    print(f"  Jetson 192.168.0.3:14551 <-> PX4 192.168.0.4:14550")
    print("=" * 60)

    # Step 1: Send heartbeats
    print("\n[1] Sending initial heartbeat to wake PX4 MAVLink ...")
    mav = mavutil.mavlink_connection(PX4_ADDR, source_system=255, source_component=0)
    for _ in range(20):
        mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        time.sleep(0.05)

    # Step 2: Connection
    print("[2] Opening MAVLink connection ...")

    # Step 3: Wait for heartbeat
    print("[3] Waiting for PX4 heartbeat (5 s timeout) ...")
    hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    hil_flag = False
    if hb:
        hil_flag = bool(hb.base_mode & 0x20)
        print(f"    Heartbeat OK  base_mode=0x{hb.base_mode:02x}  HIL_flag={'SET' if hil_flag else 'NOT SET'}")
    else:
        print("    FAIL - no heartbeat received")
        print("\n  OVERALL: FAIL - no PX4 heartbeat")
        sys.exit(1)

    # Step 4: Stream HIL data
    print(f"[4] Streaming HIL_SENSOR + HIL_GPS for {DURATION} s ...")
    t0 = time.monotonic()
    sensor_count = 0
    gps_count = 0
    actuator_received = False

    while time.monotonic() - t0 < DURATION:
        elapsed = time.monotonic() - t0
        us = int(elapsed * 1e6)

        # Heartbeat every 1s
        if sensor_count % 100 == 0:
            mav.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )

        # HIL_SENSOR
        mav.mav.hil_sensor_send(
            us, 0.0, 0.0, -GRAVITY,
            0.0, 0.0, 0.0,
            MAG_X, MAG_Y, MAG_Z,
            PRESSURE, 0.0,
            (44330.0 * (1.0 - (PRESSURE / 1013.25) ** 0.19029)),
            TEMP, 0x1FFF
        )
        sensor_count += 1

        # HIL_GPS at 10 Hz
        if sensor_count % 10 == 0:
            mav.mav.hil_gps_send(
                us, 3,
                int(LAT * 1e7), int(LON * 1e7), int(ALT * 1000),
                20, 20, 0, 0, 0, 0, 0, 12
            )
            gps_count += 1

        # Check for actuator response
        while True:
            try:
                m = mav.recv_msg()
            except TypeError:
                break
            if m is None:
                break
            if m.get_type() == 'HIL_ACTUATOR_CONTROLS':
                actuator_received = True

        # ~100 Hz
        next_tick = t0 + sensor_count * 0.01
        sleep_time = next_tick - time.monotonic()
        if sleep_time > 0:
            time.sleep(sleep_time)

    # Results
    print()
    print("=" * 60)
    print("  Results")
    print("=" * 60)
    print(f"  MAVLink / Ethernet:    PASS")
    print(f"  HIL mode flag:         {'SET' if hil_flag else 'NOT SET'}")
    print(f"  Actuator response:     {'PASS - HIL_ACTUATOR_CONTROLS received' if actuator_received else 'FAIL - no actuator response'}")
    print(f"  HIL packets sent:      {sensor_count} sensor + {gps_count} GPS")

    overall = "PASS" if (hil_flag and actuator_received) else "FAIL"
    print(f"\n  OVERALL: {overall} - {'HITL loop confirmed working over Ethernet' if overall == 'PASS' else 'Check configuration'}")
    print("=" * 60)

    sys.exit(0 if overall == "PASS" else 1)


if __name__ == "__main__":
    main()
