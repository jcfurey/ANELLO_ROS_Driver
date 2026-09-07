# ANELLO Ethernet Setup Guide (ROS2 Jazzy and later)

End-to-end instructions for running an ANELLO EVK / Ground INS over
**Ethernet (UDP)** as the primary data link into a ROS2 Jazzy (or later)
stack: physical mounting, wiring, one-time unit configuration with the
official [ANELLO user tool](https://github.com/Anello-Photonics/user_tool),
host networking, driver launch, and stack integration.

Ethernet is the recommended link for robot integrations: it supports the
full 200 Hz output rate (RS-232 caps RTCM at 100 Hz and ASCII at 50 Hz),
avoids USB-serial adapter latency/jitter, and carries the data, config,
and odometer channels on one cable.

## Table of Contents

1. [How the Ethernet interface works](#1-how-the-ethernet-interface-works)
2. [Mounting](#2-mounting)
3. [Wiring](#3-wiring)
4. [One-time unit configuration (ANELLO user tool)](#4-one-time-unit-configuration-anello-user-tool)
5. [ROS host network setup](#5-ros-host-network-setup)
6. [Launching the driver in Ethernet mode](#6-launching-the-driver-in-ethernet-mode)
7. [Vehicle configuration (lever arms)](#7-vehicle-configuration-lever-arms)
8. [Integration into the ROS2 stack](#8-integration-into-the-ros2-stack)
9. [Verification](#9-verification)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. How the Ethernet interface works

The unit exchanges UDP datagrams with one configured host. Three logical
channels use **fixed port numbers on the unit** and **configurable port
numbers on the computer** (set on the unit as `rport1/2/3`):

| Channel | Unit port (fixed) | Computer port (unit config) | Driver parameter | Direction / content |
|---|---|---|---|---|
| Data | 1 | `rport1` (default example 1111) | `local_data_port` | Unit → host: APIMU/APINS/APGPS/…; Host → unit: RTCM corrections |
| Configuration | 2 | `rport2` (default example 2222) | `local_config_port` | Host → unit: `#APCFG`/`#APVEH`/`#APRST` commands; unit → host: responses |
| Odometer | 3 | `rport3` (default example 3333) | `local_odometer_port` | Host → unit: `#APODO` wheel-speed input (UDP-only channel) |

The unit only talks to the **one computer IP** configured as `rip`, and
the driver only accepts datagrams from the configured `remote_ip` —
both ends filter by address, so every value in section 4 must be
consistent or you will see silence.

> **Note:** Ethernet cannot be configured over Ethernet from scratch —
> the initial setup in section 4 happens over USB/serial once, then the
> cable can stay in a drawer.

## 2. Mounting

Mechanical setup determines INS quality; do this before worrying about
software.

1. **Rigid-mount the unit** to the vehicle frame (no rubber isolation —
   the EKF models vehicle dynamics, not mount flex). The recommended
   location is the **center of the rear axle**, unit **X-axis pointing
   forward** in the direction of travel. The body frame is
   **X forward, Y right, Z down (FRD)**, origin at the unit center.
2. If the unit cannot be mounted in the default orientation, configure
   the installed orientation (`orn`, one of 8 right-hand-rule frames,
   default `+X+Y+Z`) and fine misalignment angles (`aln`, roll/pitch/yaw
   in degrees) — both settable from the user tool or the
   `anello/send_cmd` service.
3. **GNSS antennas** (EVK / Ground INS): mount with clear sky view on a
   ground plane ≥ 10 cm diameter. For dual-antenna heading, separate the
   antennas by **at least 0.6 m** (longer baseline = better heading:
   σ_heading ≈ position accuracy / baseline). ANT1 is the primary.
4. **Measure lever arms** (you will enter them in section 7): vectors
   *from the unit center* to ANT1 (`g1x,g1y,g1z`), ANT2 (`g2x,g2y,g2z`),
   rear-axle center (`cnx,cny,cnz`), desired output point (`ocx,ocy,ocz`),
   and odometer sensor (`wsx,wsy,wsz`) — all in **meters, in the unit's
   FRD frame**. Centimeter accuracy pays off directly in INS accuracy.
5. After installation, firmware v1.2.6+ requires a **baseline
   calibration** for dual-antenna heading (auto-calibrates in 2–5 min of
   driving, or enter the measured `bsl` manually), and firmware v1.3.24+
   wants a one-time **2-minute completely undisturbed stationary period**
   (ZUPT calibration).

## 3. Wiring

EVK:

1. **Power** per the EVK quick-start (bench supply or vehicle power
   harness).
2. **Ethernet**: RJ45 from the EVK to the robot's switch or directly to
   the ROS host NIC.
3. **GNSS antennas** to ANT1 (and ANT2 for dual-antenna heading).
4. **USB-C** to the host — *only needed for the one-time configuration*
   in section 4 (it enumerates 4 virtual COM ports) and firmware updates.

Ground INS: same idea with the 20-pin Molex MX150 harness — power,
Ethernet pairs, and the two RS-232 ports (RS232-2 can serve as the
config link for initial setup). Consult the pinout in the
[ANELLO Developer Manual](https://docs-a1.readthedocs.io/en/latest/)
for your harness revision.

Keep the Ethernet run away from motor/inverter cabling where practical;
the optical gyro is EMI-immune but the GNSS antenna cables are not.

## 4. One-time unit configuration (ANELLO user tool)

Use ANELLO's official configuration tool over USB to set up the
Ethernet personality. On the ROS host (or any laptop):

```bash
git clone https://github.com/Anello-Photonics/user_tool.git
cd user_tool
pip install -r requirements.txt
python board_tools/user_program.py
```

In the tool: **Connect → COM** (it auto-detects the config port), then
**Unit Configuration**, and set:

| Tool menu name | Code | Value for a typical robot |
|---|---|---|
| Ethernet Output | `eth` | `on` |
| DHCP (auto assign ip address) | `dhcp` | `off` (static is strongly recommended on robots) |
| UDP A-1 IP | `lip` | the unit's static IP, e.g. `192.168.1.111` |
| UDP Computer IP | `rip` | the ROS host's IP, e.g. `192.168.1.100` |
| UDP Computer Port 1 (data) | `rport1` | `1111` (driver default `local_data_port`) |
| UDP Computer Port 2 (configuration) | `rport2` | `2222` (driver default `local_config_port`) |
| UDP Computer Port 3 (odometer) | `rport3` | `3333` (driver default `local_odometer_port`) |
| Message Format | `mfm` | `4` (RTCM binary, default — compact, full-rate; use `1`/ASCII only for debugging by eye) |
| Output Data Rate (Hz) | `odr` | `100` or `200` (Ethernet supports 200; requires reset) |
| Odometer Units | `odo` | `mps` (if you will feed wheel speed) |
| Serial Output | `uart` | optional `off` once Ethernet is verified |

Save to **flash** (the tool's save option / upper-case `W` writes), then
reset the unit (the tool's reset, or `#APRST,0*58`). `odr`, baud, and
sync changes only take effect after reset.

Verify from the tool itself: **Connect → UDP**, enter the unit IP and
the computer data/config ports (1111/2222) — if the tool connects and
streams, the Ethernet personality is correct.

> Units delivered before 2023-04-13 cannot output the RTCM binary
> format — set `mfm` to ASCII (`1`) on those, or contact ANELLO support.
> The driver parses both formats automatically.

## 5. ROS host network setup

1. Give the host NIC a **static IP matching `rip`** on the same subnet
   as the unit, e.g. with netplan:

   ```yaml
   # /etc/netplan/60-anello.yaml
   network:
     version: 2
     ethernets:
       enp2s0:
         addresses: [192.168.1.100/24]
   ```

   ```bash
   sudo netplan apply
   ```

2. Open the inbound UDP ports if a firewall is active:

   ```bash
   sudo ufw allow in proto udp from 192.168.1.111 to any port 1111:3333
   ```

3. Confirm packets are arriving before involving ROS at all:

   ```bash
   sudo tcpdump -i enp2s0 -c 5 udp and src host 192.168.1.111
   ```

   You should see a steady stream to port 1111 the moment the unit is
   powered (after the configuration in section 4).

## 6. Launching the driver in Ethernet mode

ROS2 Jazzy or later, with the workspace built and sourced
(see the [readme](../readme.md) for build instructions):

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
    com_type:=ETH \
    remote_ip:=192.168.1.111 \
    local_data_port:=1111 \
    local_config_port:=2222 \
    local_odometer_port:=3333
```

What the driver does in ETH mode:

- binds the three local UDP ports and addresses the unit at
  `remote_ip` ports 1/2/3 (data/config/odometer);
- sends an `#APPNG` handshake on the config channel at startup;
- forwards `ntrip_client/rtcm` corrections to the **data** channel and
  `anello/odo` wheel speed to the dedicated **odometer** channel;
- drops datagrams from any source other than `remote_ip`.

Recommended companion parameters for Ethernet operation:

- `timestamp_source` — `mcu` (the default) removes network/OS arrival
  jitter from inter-message timing (see the readme's Timestamping
  section). Newer firmware also offers PTP (`ptp` unit config) for
  hardware time sync if your network is PTP-capable.
- NTRIP for RTK: add `ntrip_host`/`ntrip_mountpoint`/credentials launch
  arguments; corrections flow host → unit over the data channel, so no
  serial link is needed (leave the unit's own `ntrip` input channel
  `off`/serial-default — the ROS client replaces it).

## 7. Vehicle configuration (lever arms)

Enter the lever arms measured in section 2. Either use the user tool's
vehicle configuration menu, or do it live from ROS via the command
service (values in meters, FRD, from the unit center; upper-case `W`
persists to flash):

```bash
# Antenna 1 lever arm: 0.50 m forward, 0.20 m right, 0.30 m up (-Z)
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
    "{command: 'APVEH,W,g1x,0.50,g1y,0.20,g1z,-0.30'}"

# Rear axle center and output point
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
    "{command: 'APVEH,W,cnx,-0.80,cny,0.0,cnz,0.15'}"
```

The driver computes and appends the checksum automatically. Read back
any value with the `r`/`R` forms (e.g. `APVEH,R,bsl`). Set
`heading_baseline` on the driver to your measured antenna separation so
the health monitor can validate the dual-antenna solution.

## 8. Integration into the ROS2 stack

With the driver up, integration is identical to serial operation — see
the [integration guide](integration_guide.md) for the full treatment.
The short version:

1. **TF/URDF:** add the unit to your URDF at its measured mounting pose.
   Driver TF defaults to disabled; the robot estimator owns
   `map -> odom -> base_link`. Optional standalone TF is
   `anello_local -> ins_link`, and its child must have no other parent.
2. **Choose the measurement sources:** use `ins/odometry` as a globally
   corrected measurement with verified covariance and datum alignment,
   or fuse `imu/data_raw` and `gps/fix` with the heading reference required
   by your estimator. Follow the integration guide's correlation and
   heading guidance; INS output and its raw inputs are not independent.
3. **Odometer input:** publish signed speed (m/s, negative in reverse)
   on `anello/odo`; in ETH mode the driver routes it to the unit's
   UDP-only odometer channel. Strongly recommended for GNSS-denied
   stretches and required for reliable reverse detection.
4. **Run the bench checks** in integration guide §3 (rates, accel sign
   check, INS status progression) before the first drive, then the
   initialization drive: > 2 m/s for ~30 s with good sky view, and 2–5
   minutes of driving for full Kalman convergence.

## 9. Verification

These are the link-level basics; for the full bench → static → drive
acceptance sequence, run the
[Hardware Validation Checklist](hardware_validation_checklist.md).

```bash
# All topics alive
ros2 topic list | grep -E "anello|imu|gps|ins"

# Data rate matches the configured odr (e.g. ~100 Hz)
ros2 topic hz /anello/imu_raw

# GNSS fix present and sensible
ros2 topic echo /gps/fix --once

# Config channel round-trip works
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
    "{command: 'APPNG'}"   # expect '#APPNG,0*54'

# Health monitor content
ros2 topic echo /anello/health --once
```

## 10. Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| No data at all | `rip` on the unit ≠ host IP, host IP not static, or firewall. Check with `tcpdump` (section 5) first — if no packets, the problem is unit config or cabling, not ROS. |
| `tcpdump` shows packets but no ROS topics | Port mismatch: `rport1/2/3` on the unit must equal `local_data_port`/`local_config_port`/`local_odometer_port`. Also check another process isn't bound to the port (`ss -ulpn | grep 1111`). |
| Data flows, config service times out | `rport2` mismatch, or another host on the subnet configured as `rip`. The unit answers only the configured computer IP. |
| Driver logs "Ethernet bind failed" | Port already in use, or a second driver instance running. |
| Messages arrive but parse-fail | `mfm` mismatch is harmless (driver reads both), but pre-2023-04-13 units cannot emit binary — set `mfm` to `1` (ASCII). |
| Rates below configured `odr` | `odr` change without reset, or host NIC power management (`ethtool`, disable EEE) — and keep `timestamp_source` at its `mcu` default so jitter doesn't masquerade as rate variation. |
| Unit unreachable after a bad IP config | Connect over USB with the user tool and fix `lip`/`rip` — USB always works regardless of the Ethernet personality. |
