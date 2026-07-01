# ANELLO ROS2 Driver

[![CI](https://github.com/jcfurey/ANELLO_ROS_Driver/actions/workflows/ci.yml/badge.svg)](https://github.com/jcfurey/ANELLO_ROS_Driver/actions/workflows/ci.yml)

ROS2 driver for [ANELLO Photonics](https://www.anellophotonics.com/) GNSS/INS devices.

**New to ANELLO?** See the [Integration Guide](doc/integration_guide.md) for
step-by-step instructions on wiring, configuring, and integrating the ANELLO
EVK with your robotics platform (including `robot_localization` and Nav2).
Using Ethernet as the primary link (recommended for robots)? Follow the
[Ethernet Setup Guide](doc/ethernet_setup_guide.md) for mounting, unit
configuration with the ANELLO user tool, and wiring into a ROS2 Jazzy stack.
Before trusting outputs from a new install or driver upgrade, run the
[Hardware Validation Checklist](doc/hardware_validation_checklist.md).

## Supported Products

| Product | Firmware | Baud Rate |
|---------|----------|-----------|
| ANELLO GNSS INS | >= v1.1.1 | 230400 (default) |
| ANELLO EVK | >= v1.1.1 | 921600 |
| ANELLO IMU+ | >= v1.1.1 | 230400 (default) |

## Package Overview

This repository contains three ROS2 packages:

| Package | Description |
|---------|-------------|
| `anello_ros_driver` | C++ driver node that communicates with ANELLO hardware over UART or Ethernet |
| `anello_interfaces` | Custom ROS2 message and service definitions |
| `ntrip_client` | Python NTRIP client for receiving RTK corrections |

## Installation

### Prerequisites

- A supported ROS2 distro - see the [ROS2 installation guide](https://docs.ros.org/en/rolling/Installation.html)
- `rtcm_msgs` and `nmea_msgs` packages

#### Supported ROS2 distros

| Distro          | Status                                     |
|-----------------|--------------------------------------------|
| Humble          | Supported                                  |
| Jazzy           | Supported (Ubuntu 24.04 / Python 3.12)     |
| Lyrical/Kilted  | Build-verified; hardware testing pending   |

### Build

```bash
# Clone into your workspace
cd ~/ros2_ws/src
git clone https://github.com/Anello-Photonics/ANELLO_ROS_Driver.git

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select anello_interfaces anello_ros_driver ntrip_client

# Source the workspace
source install/setup.bash
```

## Quick Start

### Launch with Defaults (UART, auto-detect ports)

```bash
ros2 launch anello_ros_driver anello_driver.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
  com_type:=UART \
  baud_rate:=921600 \
  uart_data_port:=/dev/ttyUSB0 \
  uart_config_port:=/dev/ttyUSB3
```

### Launch with Ethernet

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
  com_type:=ETH \
  remote_ip:=192.168.1.111
```

### Launch with NTRIP Corrections

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
  ntrip_host:=caster.example.com \
  ntrip_port:=2101 \
  ntrip_mountpoint:=MOUNTPOINT \
  ntrip_authenticate:=true \
  ntrip_username:=user \
  ntrip_password:=pass
```

## Configuration

### Driver Parameters

All parameters can be set via the launch file or on the command line.

#### Communication

| Parameter | Default | Description |
|-----------|---------|-------------|
| `com_type` | `UART` | Communication type: `UART` or `ETH` |
| `uart_data_port` | `AUTO` | UART data port path, or `AUTO` for auto-detection |
| `uart_config_port` | `AUTO` | UART config port path, `AUTO`, or `OFF` to disable |
| `baud_rate` | `230400` | Serial baud rate (`115200`, `230400`, `460800`, `921600`). The EVK ships at `921600`; the Ground INS/IMU default is `230400` — see [doc/anello_evk_reference.md](doc/anello_evk_reference.md) |
| `remote_ip` | `192.168.1.111` | Device IP address (ethernet mode). For full Ethernet setup — unit configuration with the ANELLO user tool, host networking, port mapping — see the [Ethernet Setup Guide](doc/ethernet_setup_guide.md) |
| `local_data_port` | `1111` | Local UDP data port (ethernet mode) |
| `local_config_port` | `2222` | Local UDP config port (ethernet mode) |
| `local_odometer_port` | `3333` | Local UDP odometer port (ethernet mode) |

#### Frame IDs

| Parameter | Default | Description |
|-----------|---------|-------------|
| `frame_id.imu` | `imu_link` | Frame ID for IMU messages |
| `frame_id.ins` | `ins_link` | Frame ID for INS messages |
| `frame_id.gnss` | `gnss_link` | Frame ID for GNSS messages |
| `frame_id.hdg` | `gnss_link` | Frame ID for dual-antenna heading messages |

#### TF Broadcasting

| Parameter | Default | Description |
|-----------|---------|-------------|
| `publish_tf` | `true` | Publish TF transform from parent frame to `ins_link` |
| `tf_parent_frame` | `odom` | Parent frame for TF broadcast |

#### Polling

| Parameter | Default | Description |
|-----------|---------|-------------|
| `poll_interval_ms` | `5` | Main loop polling interval in milliseconds |

#### Timestamping

| Parameter | Default | Description |
|-----------|---------|-------------|
| `timestamp_source` | `arrival` | `arrival` = host time captured at the port read. `mcu` = device MCU time translated to host time with a minimum-offset filter (Olson, IROS 2010): inter-message timing then follows the device clock instead of carrying serial/OS arrival jitter (sub-ms typical, multi-ms outliers). The translated stamps keep a small constant offset (≈ the minimum link latency); the raw `mcu_time`/`gps_time` fields remain in every `anello/*` message for offline use |

Timestamping tips: at 230400 baud a full message spends 4–5 ms on the
wire — prefer 921600 baud or UDP when stamp latency matters. With FTDI
USB-serial adapters, lower the adapter's `latency_timer` from its 16 ms
default (`/sys/bus/usb-serial/devices/*/latency_timer`) to 1 ms.

#### Health Monitoring

| Parameter | Default | Description |
|-----------|---------|-------------|
| `heading_baseline` | `0.0` | Dual-antenna baseline length in meters, used to validate APHDG heading in the health monitor. When left at `0.0` the driver queries the device (`APVEH,R,bsl`) once at startup; if that also fails, the baseline check is skipped |

#### Standard IMU Output (REP-145)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `use_fog_wz` | `true` | Use the optical gyro (`OG_WZ`) for the z angular rate in `imu/data` and `imu/data_raw` instead of the MEMS `WZ`. Set `false` if the FOG is disabled on the unit (`APCFG fog off`) |
| `flip_accel_sign` | `false` | Negate all `linear_acceleration` axes in `imu/data`/`imu/data_raw`. The device's at-rest accelerometer sign convention is not in the public manual — verify on the bench: stationary `linear_acceleration.z` must read **+9.8**; if it reads −9.8, set this `true` (see the [integration guide](doc/integration_guide.md)) |
| `covariance.angular_velocity` | `[7.6e-7, 7.6e-7, 2.1e-8]` | Diagonal angular velocity covariance `[x, y, z]` in (rad/s)². Defaults derived from the ANELLO datasheet ARW specs at 100 Hz (MEMS X/Y: 0.3°/√hr; optical Z: 0.05°/√hr) |
| `covariance.linear_acceleration` | `[2.5e-5, 2.5e-5, 2.5e-5]` | Diagonal linear acceleration covariance `[x, y, z]` in (m/s²)². Default derived from the 0.03 m/s/√hr VRW spec at 100 Hz |

The defaults follow the standard white-noise model (per-sample variance =
noise-density² × sample rate) at 100 Hz ODR; they scale linearly with ODR,
so double them at 200 Hz or halve at 50 Hz. Datasheet noise densities are
not conservative across all timescales (bias instability and temperature
drift are excluded) — for tight fusion tuning, characterize the actual unit
with an Allan-variance run (e.g. `allan_variance_ros`) and override these
parameters.

### NTRIP Client Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ntrip_host` | (empty) | NTRIP caster hostname |
| `ntrip_port` | `2101` | NTRIP caster port |
| `ntrip_mountpoint` | (empty) | NTRIP mountpoint name |
| `ntrip_authenticate` | `false` | Enable NTRIP authentication |
| `ntrip_username` | (empty) | NTRIP username |
| `ntrip_password` | (empty) | NTRIP password |

Additional NTRIP parameters (`ssl`, `cert`, `key`, `ca_cert`, `reconnect_attempt_max`, `reconnect_attempt_wait_seconds`, `rtcm_timeout_seconds`, `ntrip_version`, `nmea_min_interval_seconds`) can be passed directly to the `ntrip_client` node.

The client speaks NTRIP rev1 by default. Set `ntrip_version` to `Ntrip/2.0`
for rev2 casters: the request is then sent as HTTP/1.1 with the required
`Host` and `Ntrip-Version` headers, and chunked transfer encoding is
decoded automatically. GGA sentences forwarded to the caster are
rate-limited to one per `nmea_min_interval_seconds` (default 10 s, `0`
disables the throttle), per standard NTRIP caster practice (5–60 s).

## Topics

### Published Topics

#### ANELLO Custom Messages

All custom messages include a `std_msgs/Header` with timestamp and frame ID. Message definitions are in `anello_interfaces/msg/`.

| Topic | Type | Description |
|-------|------|-------------|
| `anello/imu_raw` | `anello_interfaces/APIMU` | Raw IMU data (accel, gyro, FOG gyro, odometer, temp) |
| `anello/im1` | `anello_interfaces/APIM1` | IMU data without odometer (with sync time) |
| `anello/ins` | `anello_interfaces/APINS` | INS solution (position, velocity, attitude, status) |
| `anello/gps` | `anello_interfaces/APGPS` | Primary GNSS receiver (position, velocity, accuracy, RTK status) |
| `anello/gps2` | `anello_interfaces/APGPS` | Secondary GNSS receiver |
| `anello/hdg` | `anello_interfaces/APHDG` | Dual-antenna heading and baseline |
| `anello/cov` | `anello_interfaces/APCOV` | Covariance matrices (position, velocity, attitude) |
| `anello/health` | `anello_interfaces/APHEALTH` | Device health status (1 Hz) |

#### Standard ROS2 Messages

| Topic | Type | Description |
|-------|------|-------------|
| `imu/data_raw` | `sensor_msgs/Imu` | Accelerometer + gyroscope at the sensor rate (every APIMU/APIM1), no orientation (`orientation_covariance[0] = -1` per REP-145) |
| `imu/data` | `sensor_msgs/Imu` | Same as `imu/data_raw` plus the INS orientation quaternion, published at the INS rate |
| `gps/fix` | `sensor_msgs/NavSatFix` | Raw GNSS fix from APGPS (4 Hz), covariance approximated from the receiver accuracy estimates. Feed this (not `ins/fix`) to `navsat_transform_node` — fusing the INS position back in would double-count the IMU |
| `ins/fix` | `sensor_msgs/NavSatFix` | INS-fused position at the INS rate, with the full EKF covariance from APCOV when available |
| `odom` | `nav_msgs/Odometry` | INS solution at the INS rate: pose in a local ENU frame anchored at the first valid fix (frame `tf_parent_frame` → `frame_id.ins`), twist in the body (FLU) frame. Pose covariance from APCOV position/orientation; twist linear covariance from the APCOV velocity covariance rotated into the body frame; angular covariance from the `covariance.angular_velocity` parameter |
| `ntrip_client/nmea` | `nmea_msgs/Sentence` | GGA sentence forwarded to NTRIP caster |

**Frame conventions:** the `anello/*` custom topics carry values in the
device-native convention (NED attitude with heading clockwise from north,
FRD body axes), exactly as reported by the unit. The standard interfaces —
`imu/data`, `gps/fix`, and the TF broadcast — are converted by the driver
to REP-103 (ENU orientation, FLU body axes), so they can be consumed
directly by tools like `robot_localization` and Nav2.

#### TF Transforms

| Parent | Child | Description |
|--------|-------|-------------|
| `odom` (configurable) | `ins_link` (configurable) | INS orientation from roll/pitch/heading |

#### Diagnostics

The driver publishes to `/diagnostics` via `diagnostic_updater` with:
- Position accuracy status (cm / m / >1m)
- Heading stability (stable / unstable)
- Gyro health (good / bad)
- Decoded message rate and error rate over a 5 s sliding window (a
  device that stops streaming raises ERROR instead of reporting the
  last known health flags)
- Lifetime message / checksum-failure / parse-failure counters
- Active port names

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `anello/odo` | `anello_interfaces/APODO` | Odometer speed input to the ANELLO device |
| `ntrip_client/rtcm` | `rtcm_msgs/Message` | RTCM correction data from NTRIP client |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `anello/send_cmd` | `anello_interfaces/CmdAndRsp` | Send an ASCII command to the device and receive the response |

**Example:**

```bash
# Ping the device
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp "{command: 'APPNG'}"

# Read baseline configuration
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp "{command: 'APVEH,R,bsl'}"
```

## Message Definitions

### APIMU

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `header` | `Header` | | Timestamp and frame ID |
| `mcu_time` | `float64` | ms | MCU time since power-on |
| `ax`, `ay`, `az` | `float64` | g | Linear acceleration |
| `wx`, `wy`, `wz` | `float64` | deg/s | Angular rate (MEMS) |
| `wz_fog` | `float64` | deg/s | High-precision z-axis angular rate (optical) |
| `odometer_speed` | `float64` | m/s | Odometer speed |
| `odometer_time` | `float64` | s | Odometer timestamp (seconds, unlike `mcu_time`) |
| `temp` | `float64` | C | Temperature |

### APINS

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `header` | `Header` | | Timestamp and frame ID |
| `mcu_time` | `float64` | ms | MCU time |
| `gps_time` | `float64` | ns | GPS time (GTOW) |
| `ins_status` | `uint8` | | 0=Att only, 1=Pos+Att, 2=Pos+Hdg+Att, 3=RTK Float, 4=RTK Fix |
| `lat`, `lon` | `float64` | deg | Latitude, longitude |
| `alt_ellipsoid` | `float64` | m | Altitude (ellipsoid) |
| `vn`, `ve`, `vd` | `float64` | m/s | NED velocity |
| `roll`, `pitch`, `heading` | `float64` | deg | Attitude |
| `zupt` | `uint8` | | 1=stationary, 0=moving |

### APGPS

| Field | Type | Unit | Description |
|-------|------|------|-------------|
| `header` | `Header` | | Timestamp and frame ID |
| `mcu_time` | `float64` | ms | MCU time |
| `gps_time` | `float64` | ns | GPS time |
| `lat`, `lon` | `float64` | deg | Position |
| `alt_ellipsoid`, `alt_msl` | `float64` | m | Altitude |
| `speed` | `float64` | m/s | Ground speed |
| `heading` | `float64` | deg | Heading |
| `hacc`, `vacc` | `float64` | m | Horizontal/vertical accuracy |
| `pdop` | `float64` | | Position dilution of precision |
| `fix_type` | `uint8` | | 0=No Fix, 2=2D, 3=3D, 5=Time only |
| `sat_num` | `uint8` | | Number of satellites |
| `speed_accuracy` | `float64` | m/s | Speed accuracy |
| `heading_accuracy` | `float64` | deg | Heading accuracy |
| `rtk_fix_status` | `uint8` | | 0=SPP, 1=RTK Float, 2=RTK Fix |

### APHEALTH

| Field | Type | Values |
|-------|------|--------|
| `header` | `Header` | Timestamp |
| `position_acc_flag` | `uint8` | 0=cm-level, 1=m-level, 2=>1m |
| `heading_health_flag` | `uint8` | 0=stable, 1=unstable |
| `gyro_health_flag` | `uint8` | 0=good, 1=bad |

Full message definitions for `APIM1`, `APHDG`, `APCOV`, and `APODO` can be found in `anello_interfaces/msg/`.

## Node Composition

The driver is built as a composable node and can be loaded into a component container:

```bash
ros2 run rclcpp_components component_container
```

```bash
ros2 component load /ComponentManager anello_ros_driver anello::AnelloRosDriver \
  -p com_type:=UART -p baud_rate:=230400
```

## Using with Your Code

### C++

```cpp
#include "anello_interfaces/msg/apins.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Subscribe to the standard IMU message
auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(),
    [](sensor_msgs::msg::Imu::SharedPtr msg) {
        // Use msg->orientation, msg->angular_velocity, msg->linear_acceleration
    });

// Or subscribe to the ANELLO-specific INS message
auto sub2 = node->create_subscription<anello_interfaces::msg::APINS>(
    "anello/ins", rclcpp::SensorDataQoS(),
    [](anello_interfaces::msg::APINS::SharedPtr msg) {
        // Use msg->lat, msg->lon, msg->heading, etc.
    });
```

### Python

```python
from sensor_msgs.msg import Imu, NavSatFix
from anello_interfaces.msg import APINS

# Standard messages work with any ROS2 tool
self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)

# ANELLO-specific messages provide additional fields
self.create_subscription(APINS, 'anello/ins', self.ins_callback, 10)
```

## Testing

```bash
cd ~/ros2_ws
colcon test --packages-select anello_interfaces anello_ros_driver ntrip_client
colcon test-result --verbose
```

This runs `ament_lint_auto` (cppcheck, flake8, pep257, xmllint) and any additional tests. Copyright and formatting-only linters (copyright, cpplint, uncrustify) are skipped: the codebase predates them and a bulk reformat would obscure history.

## Architecture

```
                    ANELLO Device (GNSS INS / EVK / IMU+)
                         |                    |
                    UART / Ethernet      UART / Ethernet
                    (Data Port)          (Config Port)
                         |                    |
              +----------+--------------------+----------+
              |          anello_ros_driver (C++)          |
              |                                          |
              |  Communication Layer                     |
              |    serial_interface / ethernet_interface  |
              |    anello_data_port / anello_config_port  |
              |                                          |
              |  Decoding Layer                          |
              |    ASCII decoder (#AP messages)           |
              |    RTCM decoder (binary type 4058)        |
              |                                          |
              |  Publishing Layer                        |
              |    Custom ANELLO messages (anello/*)      |
              |    Standard messages (imu/data, gps/fix)  |
              |    TF2 transforms                        |
              |    Diagnostics                           |
              +------------------------------------------+
                         |                    ^
                    ROS2 DDS               ROS2 DDS
                         |                    |
              +----------+--------------------+----------+
              |          ntrip_client (Python)            |
              |                                          |
              |  NTRIP caster <-> RTCM corrections       |
              |  ntrip_client/nmea (GGA) -> caster       |
              |  caster -> ntrip_client/rtcm -> driver   |
              +------------------------------------------+
```

## Further Reading

- [Integration Guide](doc/integration_guide.md) - Hardware setup, URDF, robot_localization, Nav2, NTRIP, odometer input, troubleshooting
- [ANELLO Developer Manual](https://docs-a1.readthedocs.io/en/latest/) - Firmware documentation, ASCII/RTCM protocol reference

## License

MIT License

Copyright (c) 2023 ANELLO Photonics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

## Author Information

This driver was created by [ANELLO Photonics](https://www.anellophotonics.com/).
