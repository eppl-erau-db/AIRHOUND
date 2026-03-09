#!/usr/bin/env python3
"""
HIL sensor feeder — sends HIL_SENSOR (100 Hz) + HIL_GPS (10 Hz) to PX4.

Sends heartbeats first to wake PX4 MAVLink unicast.
Reference location: Daytona Beach FL (29.1868, -81.0489)

IMPORTANT: Use udpout: not udp: for pymavlink connection string.
"""

import time
import sys
from pymavlink import mavutil

PX4_ADDR = "udpout:192.168.0.4:14550"

# Reference location
LAT = 29.1868
LON = -81.0489
ALT = 10.0  # metres MSL

# Sensor constants
GRAVITY = 9.81
MAG_X, MAG_Y, MAG_Z = 0.21, 0.0, 0.42  # gauss
PRESSURE = 1013.25  # mbar
TEMP = 25.0  # deg C


def main():
    print(f"Connecting to PX4 at {PX4_ADDR} ...")
    mav = mavutil.mavlink_connection(PX4_ADDR, source_system=255, source_component=0)

    # Send heartbeats to wake PX4 MAVLink
    print("Sending heartbeats to wake PX4 ...")
    for _ in range(20):
        mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        time.sleep(0.05)

    # Wait for PX4 heartbeat
    print("Waiting for PX4 heartbeat ...")
    msg = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if msg:
        print(f"  Got heartbeat: base_mode=0x{msg.base_mode:02x}")
    else:
        print("  WARNING: No heartbeat received (continuing anyway)")

    print("Streaming HIL_SENSOR (100 Hz) + HIL_GPS (10 Hz) ... Ctrl+C to stop")

    t0 = time.monotonic()
    sensor_count = 0
    gps_count = 0
    hb_last = 0

    try:
        while True:
            now = time.monotonic()
            elapsed = now - t0
            us = int(elapsed * 1e6)

            # Heartbeat every 1s
            if int(elapsed) > hb_last:
                hb_last = int(elapsed)
                mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )

            # HIL_SENSOR at 100 Hz
            mav.mav.hil_sensor_send(
                us,           # time_usec
                0.0, 0.0, -GRAVITY,  # xacc, yacc, zacc (m/s^2)
                0.0, 0.0, 0.0,       # xgyro, ygyro, zgyro (rad/s)
                MAG_X, MAG_Y, MAG_Z, # xmag, ymag, zmag (gauss)
                PRESSURE,             # abs_pressure (mbar)
                0.0,                  # diff_pressure
                (44330.0 * (1.0 - (PRESSURE / 1013.25) ** 0.19029)),  # pressure_alt
                TEMP,                 # temperature (degC)
                0x1FFF                # fields_updated (all)
            )
            sensor_count += 1

            # HIL_GPS at 10 Hz
            if sensor_count % 10 == 0:
                mav.mav.hil_gps_send(
                    us,                          # time_usec
                    3,                           # fix_type (3D fix)
                    int(LAT * 1e7),              # lat (degE7)
                    int(LON * 1e7),              # lon (degE7)
                    int(ALT * 1000),             # alt (mm MSL)
                    20,                          # eph (cm)
                    20,                          # epv (cm)
                    0,                           # vel (cm/s)
                    0, 0, 0,                     # vn, ve, vd (cm/s)
                    0,                           # cog (cdeg)
                    12                           # satellites_visible
                )
                gps_count += 1

            # Print status every 5s
            if sensor_count % 500 == 0:
                print(f"  [{elapsed:.0f}s] sensor={sensor_count} gps={gps_count}")

            # Drain incoming messages
            while True:
                try:
                    m = mav.recv_msg()
                except TypeError:
                    # pymavlink Python 3.14 bug
                    break
                if m is None:
                    break

            # Sleep to maintain ~100 Hz
            next_tick = t0 + sensor_count * 0.01
            sleep_time = next_tick - time.monotonic()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print(f"\nStopped. Sent {sensor_count} sensor + {gps_count} GPS packets in {elapsed:.1f}s")


if __name__ == "__main__":
    main()
