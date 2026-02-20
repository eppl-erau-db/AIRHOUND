# HITL Simulation Setup

> **Hardware-in-the-Loop (HITL)** runs the real PX4 flight stack on the Pixhawk while a simulator
> supplies synthetic sensor data. The drone's motors do not spin; only the firmware logic executes.

---

## Confirmed Hardware Configuration

| Component | Details |
|-----------|---------|
| Flight Controller | Auterion PX4 FMU v6X.x (`px4_fmu-v6x`) |
| Companion Computer | NVIDIA Jetson Orin Nano 16GB |
| Baseboard | Auterion (provides Jetson↔Pixhawk Ethernet) |
| Jetson Ethernet IP | `192.168.0.3` (interface `eno1`) |
| Pixhawk Ethernet IP | `192.168.0.4` (static, configured on SD card) |
| MAVLink port | UDP 14550 |
| uXRCE-DDS port | UDP 8888 |

---

## Overview

The HITL loop runs entirely over Ethernet — no USB data link is required during simulation:

```
Jetson (192.168.0.3)  ←── Auterion baseboard Ethernet ───→  Pixhawk (192.168.0.4)
       │                                                              │
  sends HIL_SENSOR                                        runs full PX4 flight stack
  sends HIL_GPS                                           returns HIL_ACTUATOR_CONTROLS
  runs uXRCE-DDS Agent (port 8888)
  exposes MAVLink GCS (port 14550)
```

---

## Step 1: Build HITL Firmware

HITL requires the `pwm_out_sim` module. The `px4_fmu-v6x_default` target overflows flash;
use the `multicopter` configuration instead, which removes fixed-wing and VTOL modules.

**On the Jetson** (or any machine with PX4-Autopilot source):

```bash
cd ~/PX4-Autopilot

# Edit boards/px4/fmu-v6x/multicopter.px4board
# Add this line at the bottom:
#   CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y
nano boards/px4/fmu-v6x/multicopter.px4board
```

The final `multicopter.px4board` should contain (among existing lines):

```
CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y
```

Build:

```bash
make px4_fmu-v6x_multicopter
# Result: build/px4_fmu-v6x_multicopter/px4_fmu-v6x_multicopter.px4
# Expected flash usage: ~97% (fits within 1920KB limit)
```

---

## Step 2: Flash the Firmware

Use the official PX4 upload tool. The Pixhawk must be connected via USB to the machine
running the flash command (USB for power/flashing only — all runtime data uses Ethernet).

```bash
# Stop ModemManager first — it grabs /dev/ttyACM0 and blocks flashing
sudo systemctl stop ModemManager

# Flash (adjust port if needed; fmu-v6x appears as /dev/ttyACM0)
python3 ~/PX4-Autopilot/Tools/px_uploader.py \
    /dev/ttyACM0 \
    ~/PX4-Autopilot/build/px4_fmu-v6x_multicopter/px4_fmu-v6x_multicopter.px4
```

The uploader reboots the Pixhawk into bootloader mode automatically. Wait for `Done` before
disconnecting USB.

> **Note:** ModemManager re-grabs the port after every Pixhawk reboot. Run
> `sudo systemctl stop ModemManager` before each NSH session.
> To disable permanently: `sudo systemctl disable ModemManager`

---

## Step 3: Configure PX4 Parameters via NSH

Open the PX4 NSH console using `mavlink_shell.py` (available in
`~/PX4-Autopilot/Tools/mavlink_shell.py`):

```bash
sudo systemctl stop ModemManager
python3 ~/PX4-Autopilot/Tools/mavlink_shell.py /dev/ttyACM0
```

Set the required parameters (type each line and press Enter):

```
param set SYS_AUTOSTART 1001
param set COM_RC_IN_MODE 1
param set NAV_RCL_ACT 0
param save
reboot
```

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SYS_AUTOSTART` | `1001` | HIL Quadcopter X airframe |
| `COM_RC_IN_MODE` | `1` | Joystick / no RC checks |
| `NAV_RCL_ACT` | `0` | Disable RC loss failsafe |

---

## Step 4: Configure Pixhawk Ethernet (net.cfg)

The fmu-v6x uses a static network config file on the SD card. Set this via NSH:

```bash
# Open NSH console
python3 ~/PX4-Autopilot/Tools/mavlink_shell.py /dev/ttyACM0
```

In the NSH shell:

```
echo "DEVICE=eth0" > /fs/microsd/net.cfg
echo "BOOTPROTO=static" >> /fs/microsd/net.cfg
echo "NETMASK=255.255.255.0" >> /fs/microsd/net.cfg
echo "IPADDR=192.168.0.4" >> /fs/microsd/net.cfg
echo "ROUTER=192.168.0.254" >> /fs/microsd/net.cfg
echo "DNS=192.168.0.254" >> /fs/microsd/net.cfg
netman update
```

The Pixhawk will reboot and come up at `192.168.0.4`.

> **Note:** The fmu-v6x does **not** use `NET_IP0`–`NET_IP3` parameters —
> those are for other boards. Use `netman` + the SD card file.

---

## Step 5: Configure uXRCE-DDS and MAVLink over Ethernet

Back in NSH, set the DDS and MAVLink parameters:

```
param set UXRCE_DDS_CFG 1000
param set UXRCE_DDS_PRT 8888
param set UXRCE_DDS_AG_IP -1062731773
param set MAV_2_CONFIG 1000
param set MAV_2_UDP_PRT 14550
param set MAV_2_REMOTE_PRT 14550
param save
reboot
```

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `UXRCE_DDS_CFG` | `1000` | Enable uXRCE-DDS on Ethernet |
| `UXRCE_DDS_PRT` | `8888` | DDS UDP port |
| `UXRCE_DDS_AG_IP` | `-1062731773` | Agent IP = 192.168.0.3 (Jetson) as signed int32 |
| `MAV_2_CONFIG` | `1000` | Enable MAVLink instance 2 on Ethernet |
| `MAV_2_UDP_PRT` | `14550` | MAVLink listen port |
| `MAV_2_REMOTE_PRT` | `14550` | MAVLink GCS port |

> **IP encoding:** `192.168.0.3` as a signed 32-bit integer = `-1062731773`.
> Formula: `int.from_bytes(bytes([192,168,0,3]), 'big', signed=True)`

---

## Step 6: Configure Jetson Route

The Jetson's `eno1` is configured as `/32` by default, so it can't route to the Pixhawk
subnet. Add a persistent host route:

```bash
# On the Jetson
sudo nmcli connection modify 'Wired connection 2' \
    +ipv4.routes '192.168.0.4/32'

# Verify
ping -c 3 192.168.0.4   # Should show <1ms RTT
```

---

## Step 7: Run the HITL Smoke Test

The smoke test script is saved on the Jetson at `/tmp/hitl_smoke_test.py`.
It sends HIL_SENSOR + HIL_GPS packets and confirms `HIL_ACTUATOR_CONTROLS` are received.

**From the Jetson:**

```bash
python3 /tmp/hitl_smoke_test.py
```

Expected output (confirmed passing):

```
============================================================
  AIRHOUND HITL Smoke Test
  Jetson 192.168.0.3:14551 <-> PX4 192.168.0.4:14550
============================================================

[1] Sending initial heartbeat to wake PX4 MAVLink ...
[2] Opening MAVLink connection ...
[3] Waiting for PX4 heartbeat (5 s timeout) ...
    Heartbeat OK  base_mode=0x3d  HIL_flag=SET
[4] Streaming HIL_SENSOR + HIL_GPS for 5 s ...

============================================================
  Results
============================================================
  MAVLink / Ethernet:    PASS
  HIL mode flag:         SET
  Actuator response:     PASS - HIL_ACTUATOR_CONTROLS received
  HIL packets sent:      474 sensor + 474 GPS

  OVERALL: PASS - HITL loop confirmed working over Ethernet
============================================================
```

> **HIL flag note:** `MAV_MODE_FLAG_HIL_ENABLED = 0x20` (bit 5).
> `base_mode=0x3d` has this bit set. The flag is **not** `0x80`.

---

## Starting the DDS Agent (for ROS2 integration)

Once Pixhawk is on the network, start the uXRCE-DDS agent on the Jetson:

```bash
# On the Jetson
MicroXRCEAgent udp4 -p 8888
```

PX4 ROS2 topics will appear under `/fmu/` once the session connects.

---

## Troubleshooting

### `mavlink_shell.py` hangs indefinitely

ModemManager grabbed the port. Fix:

```bash
sudo systemctl stop ModemManager
# Then retry mavlink_shell.py
```

### `NET_IP0 not found` in NSH

Expected — fmu-v6x does not use those parameters. Use `netman` + `net.cfg` instead.

### Can't ping Pixhawk at 192.168.0.4

1. Confirm `net.cfg` was written correctly: in NSH run `cat /fs/microsd/net.cfg`
2. Confirm Jetson route exists: `ip route show | grep 192.168.0.4`
3. Add route if missing: `sudo nmcli connection modify 'Wired connection 2' +ipv4.routes '192.168.0.4/32'`

### No heartbeat from PX4 in smoke test

PX4 MAVLink UDP is unicast — it only responds after receiving a packet from the GCS.
The smoke test sends initial heartbeats to trigger this. If still failing:
- Confirm `MAV_2_CONFIG=1000` is set (not 0)
- Confirm Pixhawk Ethernet is up: `ping 192.168.0.4` from Jetson

### jMAVSim on Jetson (ARM64) fails

jMAVSim's `jssc` Java serial library has no ARM64 native binaries and will crash with
`UnsatisfiedLinkError`. Use the pymavlink smoke test (`hitl_smoke_test.py`) instead.

---

## Reference: IP Address Encoding

To encode an IP address as the signed int32 that PX4 expects for `UXRCE_DDS_AG_IP`:

```python
import struct
ip = "192.168.0.3"
parts = [int(x) for x in ip.split('.')]
val = struct.unpack('>i', bytes(parts))[0]
print(val)  # -1062731773
```

---

*Last updated: February 2026 — HITL confirmed working on Auterion fmu-v6x hardware*
