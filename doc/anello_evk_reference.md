# ANELLO EVK (with FOG) — Driver Reference

Working notes for targeting the **ANELLO EVK** (Evaluation Kit with SiPhOG
optical gyro) with this ROS2 driver. Condensed from the official
[ANELLO Developer Manual](https://docs-a1.readthedocs.io/en/latest/),
checked June 2026. Always cross-check against the manual for your
firmware version.

## Communication settings (EVK)

| Setting | EVK value | Driver parameter |
|---------|-----------|------------------|
| Serial baud rate | **921600** (8N1) | `baud_rate` (driver default is 230400 — set `baud_rate:=921600` for the EVK) |
| Virtual COM ports | 4 enumerate over USB-C | — |
| Data port | **lowest** port (e.g. `/dev/ttyUSB0`) — main message output, RTCM corrections input | `uart_data_port` |
| Config port | **highest** port (e.g. `/dev/ttyUSB3`) — configuration + odometer input, NMEA output | `uart_config_port` |
| UDP remote port 1 | Data output | fixed on unit |
| UDP remote port 2 | Configuration input / NMEA output | fixed on unit |
| UDP remote port 3 | **Odometer input (UDP only)** | fixed on unit |
| Local UDP ports | selectable | `local_data_port`, `local_config_port`, `local_odometer_port` |

Ethernet must first be configured over USB (`dhcp`, `lip` static IP,
`rip` computer IP, `rport1/2/3` local ports). Output format `mfm`:
`1` = ASCII, `4` = RTCM (default). Output rate `odr`: 20/50/100/200 Hz
(reset required). Reset command: `#APRST,0*58` (config port).

## Vehicle frame and conventions

Officially documented as **X forward, Y right, Z down (FRD)** in the
vehicle frame, origin at the ANELLO unit center. Attitude is standard
aerospace 3-2-1 (yaw-pitch-roll) Euler angles in **NED**: roll about
body X, pitch about body Y, heading about body Z (clockwise from
north). Velocities are NED (VN, VE, VD).

This matches the driver's conversion layer: `anello/*` topics carry
device-native NED/FRD values; `imu/data`, `gps/fix`, and TF are
converted to REP-103 ENU/FLU.

The FOG (SiPhOG) measures the Z axis: APIMU's `OG_WZ` field is the
optical-gyro z-rate alongside the MEMS `WZ` (the driver's gyro health
check compares the two).

## Message reference (EVK & Ground INS)

### APIMU (ASCII field order, current firmware)

`#APIMU, Time(ms), T_Sync(ms), AX, AY, AZ (g), WX, WY, WZ (deg/s), OG_WZ (deg/s), ODO (m/s), ODO_Time (ms), Temp (C)`

T_Sync is present on current EVK firmware (the driver detects it by
field count).

### APINS (100 Hz)

`#APINS, Time(ms), PPS_Time(ns), Status, Lat(deg), Lon(deg), Height(m), VN, VE, VD (m/s), Roll, Pitch, Heading (deg), ZUPT`

**INS status enumeration:**

| Value | Meaning |
|-------|---------|
| 0 | Attitude only |
| 1 | Position and attitude |
| 2 | Position, attitude, and heading |
| 3 | RTK Float |
| 4 | RTK Fix |
| 8 | Attitude only (GPS disabled) |
| 9 | Position and attitude (GPS disabled) |
| 10 | Position, attitude, and heading (GPS disabled) |

### APGPS (4 Hz)

Fix type: `0` no fix, `2` 2D, `3` 3D, `5` time only.
RTK status: `0` single point, `1` RTK float, `2` RTK fixed
(the driver maps these to GGA quality 1/5/4).

### APHDG status flag bits

Bit 0 gnssFixOK, bit 1 diffSoln, bit 2 relPosValid, bits 3..4 carrSoln,
bit 5 isMoving, bit 6 refPosMiss, bit 7 refObsMiss,
bit 8 relPosHeadingValid, bit 9 relPosNormalized.

### APODO (odometer input)

`#APODO,<dir>,<speed>*CK` — direction `-` reverse / `+` forward
(optional; negative speed also indicates reverse). Units configurable
(m/s default). Send to the **config port** over serial, or **UDP
port 3** over ethernet. Reverse indication persists until a forward
packet arrives.

### RTCM binary subtypes (message type 4058)

1 = IMU, 2 = GPS PVT, 3 = HDG, 4 = INS, 6 = IM1 (Ground IMU),
8 = AHRS (Ground IMU). (APCOV / subtype 10 is supported by newer
firmware but not yet in the public manual.)

## Setup / initialization checklist (drive testing)

1. Mount unit, measure **lever arms** in vehicle frame (FRD, meters,
   unit center origin): `g1x/g1y/g1z` (ANT1), `g2x/g2y/g2z` (ANT2),
   `ocx/ocy/ocz` output center, `wsx/wsy/wsz` odometer, `cnx/cny/cnz`
   rear axle. Configured via APVEH.
2. **Antenna baseline** (`bsl`): required for dual-antenna heading
   (fw ≥ 1.2.6). Auto-calibrate (2–5 min, full sky view), manual entry
   (< 2 cm accuracy), or derived from lever arms. ≥ 0.6 m separation
   required for stationary heading init. Set the driver's
   `heading_baseline` parameter to the same value to enable the
   heading health check.
3. **ZUPT calibration** (fw ≥ 1.3.24): unit on, stationary,
   undisturbed for 2 minutes (via ANELLO Python tool).
4. **Heading initialization**: stationary (dual antenna) or drive
   forward > 2 m/s for ~30 s.
5. Kalman filter converges within 2–5 minutes of driving; allow this
   before GNSS-denied operation. Provide odometer input for extended
   GNSS-denied driving.

## Known limitations (from the manual)

- Algorithms designed for wheeled land vehicles.
- CAN interface not yet available (serial/ethernet only).
- Binary messaging requires units delivered after 2023-04-13 or a
  firmware update.
- Dual-antenna static heading requires units shipped after 2022-11-01
  with fw ≥ 1.0.
- Minor timing jitter expected in GPS/GP2/HDG packets; IMU/INS
  timestamp jitter on fw < 1.1.3.

## Sources

- [ANELLO Developer Manual](https://docs-a1.readthedocs.io/en/latest/)
- [EVK Getting Started Guide](https://docs-a1.readthedocs.io/en/latest/getting_started_quick.html)
- [Communication & Messaging](https://docs-a1.readthedocs.io/en/latest/communication_messaging.html)
- [Unit Configurations](https://docs-a1.readthedocs.io/en/latest/unit_configuration.html)
- [Vehicle Configuration](https://docs-a1.readthedocs.io/en/latest/vehicle_configuration.html)
- [Drive Testing Best Practices](https://docs-a1.readthedocs.io/en/latest/drive_testing.html)
- [Known Issues and Limitations](https://docs-a1.readthedocs.io/en/latest/known_limitations.html)
- [EVK product page](https://www.anellophotonics.com/products/gnss-ins-evaluation-kit)
