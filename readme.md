# ANELLO ROS2 Driver

ROS2 driver for [ANELLO Photonics](https://www.anellophotonics.com/) GNSS/INS devices.

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

- ROS2 (Humble or later recommended) - see the [ROS2 installation guide](https://docs.ros.org/en/rolling/Installation.html)
- `mavros_msgs` and `nmea_msgs` packages

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
| `baud_rate` | `230400` | Serial baud rate (`115200`, `230400`, `460800`, `921600`) |
| `remote_ip` | `192.168.1.111` | Device IP address (ethernet mode) |
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

### NTRIP Client Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ntrip_host` | (empty) | NTRIP caster hostname |
| `ntrip_port` | `2101` | NTRIP caster port |
| `ntrip_mountpoint` | (empty) | NTRIP mountpoint name |
| `ntrip_authenticate` | `false` | Enable NTRIP authentication |
| `ntrip_username` | (empty) | NTRIP username |
| `ntrip_password` | (empty) | NTRIP password |

Additional NTRIP parameters (`ssl`, `cert`, `key`, `ca_cert`, `reconnect_attempt_max`, `reconnect_attempt_wait_seconds`, `rtcm_timeout_seconds`) can be passed directly to the `ntrip_client` node.

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
| `imu/data` | `sensor_msgs/Imu` | Standard IMU message with orientation quaternion, angular velocity, linear acceleration, and covariance |
| `gps/fix` | `sensor_msgs/NavSatFix` | Standard GNSS fix with position covariance |
| `ntrip_client/nmea` | `nmea_msgs/Sentence` | GGA sentence forwarded to NTRIP caster |

#### TF Transforms

| Parent | Child | Description |
|--------|-------|-------------|
| `odom` (configurable) | `ins_link` (configurable) | INS orientation from roll/pitch/heading |

#### Diagnostics

The driver publishes to `/diagnostics` via `diagnostic_updater` with:
- Position accuracy status (cm / m / >1m)
- Heading stability (stable / unstable)
- Gyro health (good / bad)
- Active port names

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `anello/odo` | `anello_interfaces/APODO` | Odometer speed input to the ANELLO device |
| `ntrip_client/rtcm` | `mavros_msgs/RTCM` | RTCM correction data from NTRIP client |

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
| `odometer_time` | `float64` | ms | Odometer timestamp |
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

This runs `ament_lint_auto` (copyright, cppcheck, cpplint, flake8, pep257, xmllint) and any additional tests.

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
