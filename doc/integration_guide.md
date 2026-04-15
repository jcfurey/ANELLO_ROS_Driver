# ANELLO EVK Integration Guide for Robotics Platforms

This guide walks through integrating the ANELLO EVK (Evaluation Kit) with a
ROS2-based robotics platform. It covers hardware wiring, driver configuration,
integration with `robot_localization` and Nav2, URDF setup, and
troubleshooting.

## Table of Contents

1. [Hardware Setup](#1-hardware-setup)
2. [Driver Installation and Launch](#2-driver-installation-and-launch)
3. [Verifying Data Flow](#3-verifying-data-flow)
4. [URDF / TF Integration](#4-urdf--tf-integration)
5. [Integration with robot_localization](#5-integration-with-robot_localization)
6. [Integration with Nav2](#6-integration-with-nav2)
7. [NTRIP / RTK Setup](#7-ntrip--rtk-setup)
8. [Odometer Input](#8-odometer-input)
9. [Device Configuration via Service](#9-device-configuration-via-service)
10. [Performance Tuning](#10-performance-tuning)
11. [Common Issues and Troubleshooting](#11-common-issues-and-troubleshooting)

---

## 1. Hardware Setup

### 1.1 Connections

The ANELLO EVK exposes two USB-serial ports when connected via USB:

| Port | Function | Typical Device |
|------|----------|----------------|
| Data port | Streams IMU/INS/GPS/HDG messages | `/dev/ttyUSB0` |
| Config port | Accepts commands, returns responses | `/dev/ttyUSB3` |

The exact device paths depend on your system. The driver's `AUTO` mode will
detect the correct ports automatically.

**USB connection:**

```
Robot Computer ──USB──> ANELLO EVK
```

**Ethernet connection (alternative):**

```
Robot Computer ──Ethernet──> ANELLO EVK
  (local IP)                 (192.168.1.111 default)
```

For Ethernet, the ANELLO unit must be configured with your computer's IP
address as its "Computer IP" target. Refer to the ANELLO Developer Manual.

### 1.2 USB Permissions

Add your user to the `dialout` group so the driver can access serial ports
without `sudo`:

```bash
sudo usermod -aG dialout $USER
# Log out and back in for this to take effect
```

Or create a udev rule for persistent device names:

```bash
# /etc/udev/rules.d/99-anello.rules
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6011", \
  SYMLINK+="anello_evk_%s{bInterfaceNumber}", MODE="0666"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 1.3 Mounting

Mount the EVK rigidly to your robot chassis. Record the position offset from
your robot's base frame origin to the IMU reference point:

- **X**: forward (meters)
- **Y**: left (meters)
- **Z**: up (meters)

You will need these offsets for the URDF (Section 4).


---

## 2. Driver Installation and Launch

### 2.1 Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/Anello-Photonics/ANELLO_ROS_Driver.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select anello_interfaces anello_ros_driver ntrip_client
source install/setup.bash
```

### 2.2 Launch (USB, EVK defaults)

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
  baud_rate:=921600
```

The EVK uses 921600 baud by default. The GNSS INS and IMU+ use 230400.

### 2.3 Launch (Ethernet)

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
  com_type:=ETH \
  remote_ip:=192.168.1.111
```

### 2.4 Launch with Custom Frame IDs

If your robot's URDF uses different frame names:

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
  baud_rate:=921600 \
  frame_id_imu:=anello_imu \
  frame_id_ins:=anello_ins \
  frame_id_gnss:=anello_gnss \
  publish_tf:=false
```

Set `publish_tf:=false` if another node (e.g., `robot_localization`) is
responsible for publishing the TF tree.

---

## 3. Verifying Data Flow

### 3.1 Check Topics

```bash
ros2 topic list | grep -E "anello|imu|gps"
```

Expected output:

```
/anello/cov
/anello/gps
/anello/gps2
/anello/hdg
/anello/health
/anello/im1
/anello/imu_raw
/anello/ins
/gps/fix
/imu/data
```

### 3.2 Inspect Data

```bash
# Standard IMU (orientation + angular velocity + linear acceleration)
ros2 topic echo /imu/data --once

# Standard GPS fix
ros2 topic echo /gps/fix --once

# ANELLO INS solution (lat/lon/alt + velocity + attitude + status)
ros2 topic echo /anello/ins --once

# Health status
ros2 topic echo /anello/health
```

### 3.3 Check Message Rates

```bash
ros2 topic hz /imu/data
ros2 topic hz /anello/ins
ros2 topic hz /anello/gps
```

Typical rates: IMU at 100-200 Hz, INS at 100 Hz, GPS at 5-20 Hz.

### 3.4 Diagnostics

```bash
ros2 topic echo /diagnostics
```

Or use `rqt_runtime_monitor` for a GUI view.

### 3.5 INS Status Codes

The `anello/ins` message includes an `ins_status` field:

| Value | Meaning |
|-------|---------|
| 255 | Uninitialized |
| 0 | Attitude only |
| 1 | Position + Attitude |
| 2 | Position + Heading + Attitude |
| 3 | RTK Float |
| 4 | RTK Fix |

You should see the status progress from 255 -> 0 -> 1 -> 2 as the device
initializes. With NTRIP corrections, it will reach 3 (RTK Float) and
eventually 4 (RTK Fix).


---

## 4. URDF / TF Integration

### 4.1 Adding the ANELLO to Your URDF

Add a link and joint for the ANELLO unit in your robot's URDF/xacro. Adjust
the `xyz` and `rpy` values to match your physical mounting:

```xml
<!-- ANELLO EVK mounted on the robot -->
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.06 0.06 0.03"/>
    </geometry>
    <material name="dark_grey">
      <color rgba="0.3 0.3 0.3 1.0"/>
    </material>
  </visual>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <!-- Adjust to your mounting position (meters, radians) -->
  <origin xyz="0.1 0.0 0.15" rpy="0 0 0"/>
</joint>

<!-- GNSS antenna link (if antenna is separate from IMU body) -->
<link name="gnss_link"/>

<joint name="gnss_joint" type="fixed">
  <parent link="base_link"/>
  <child link="gnss_link"/>
  <!-- Adjust to your antenna position -->
  <origin xyz="0.0 0.0 0.5" rpy="0 0 0"/>
</joint>

<!-- INS solution reference frame (typically same as IMU) -->
<link name="ins_link"/>

<joint name="ins_joint" type="fixed">
  <parent link="imu_link"/>
  <child link="ins_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

### 4.2 TF Tree

The driver publishes a transform from `odom` -> `ins_link` by default. In a
typical setup with `robot_localization`, you want the driver's TF disabled
and let `robot_localization` handle the full TF tree:

```
map ──(robot_localization)--> odom ──(robot_localization)--> base_link
                                                               |
                                                          (static, URDF)
                                                               |
                                                           imu_link
                                                           gnss_link
                                                           ins_link
```

Set `publish_tf:=false` when using `robot_localization`.

### 4.3 Frame ID Conventions

Make sure the `frame_id.*` parameters match the link names in your URDF:

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
  frame_id_imu:=imu_link \
  frame_id_ins:=ins_link \
  frame_id_gnss:=gnss_link \
  publish_tf:=false
```


---

## 5. Integration with robot_localization

The `robot_localization` package (EKF/UKF) fuses IMU and GPS data into a
smooth odometry estimate. The ANELLO driver publishes the standard message
types that `robot_localization` expects.

### 5.1 Install

```bash
sudo apt install ros-${ROS_DISTRO}-robot-localization
```

### 5.2 EKF Configuration (local odometry)

Create `config/ekf_local.yaml`:

```yaml
ekf_local:
  ros__parameters:
    frequency: 50.0
    sensor_timeout: 0.1
    two_d_mode: false

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    # IMU from ANELLO (orientation, angular velocity, linear acceleration)
    imu0: /imu/data
    imu0_config: [false, false, false,   # x, y, z position
                  true,  true,  true,    # roll, pitch, yaw
                  false, false, false,   # x, y, z velocity
                  true,  true,  true,    # roll rate, pitch rate, yaw rate
                  true,  true,  true]    # x, y, z acceleration
    imu0_differential: false
    imu0_relative: false
    imu0_remove_gravitational_acceleration: true
    imu0_queue_size: 10

    publish_tf: true
    publish_acceleration: true
```

### 5.3 NavSat Transform (GPS -> odometry)

For outdoor GPS-aided navigation, use `navsat_transform_node` to convert
`sensor_msgs/NavSatFix` into odometry:

Create `config/navsat.yaml`:

```yaml
navsat_transform:
  ros__parameters:
    frequency: 30.0
    delay: 3.0
    magnetic_declination_radians: 0.0
    yaw_offset: 0.0
    zero_altitude: false
    broadcast_utm_transform: true
    publish_filtered_gps: true
    use_odometry_yaw: false
    wait_for_datum: false
```

### 5.4 Launch Everything Together

```python
# my_robot_nav.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    anello_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('anello_ros_driver'),
                'launch', 'anello_driver.launch.py'
            )
        ),
        launch_arguments={
            'baud_rate': '921600',
            'publish_tf': 'false',
        }.items()
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local',
        parameters=[os.path.join('config', 'ekf_local.yaml')],
    )

    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[os.path.join('config', 'navsat.yaml')],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/filtered'),
        ],
    )

    return LaunchDescription([
        anello_launch,
        ekf_node,
        navsat_node,
    ])
```


---

## 6. Integration with Nav2

Nav2 requires a TF tree (`map -> odom -> base_link`) and odometry. With the
ANELLO + `robot_localization` setup above, Nav2 gets what it needs
automatically.

### 6.1 Key Requirements

| Nav2 Needs | Source |
|-----------|--------|
| `map -> odom` transform | `robot_localization` (with GPS via `navsat_transform`) or AMCL |
| `odom -> base_link` transform | `robot_localization` EKF |
| `/odom` topic (`nav_msgs/Odometry`) | `robot_localization` EKF publishes this |
| `/imu/data` topic | ANELLO driver |

### 6.2 Sensor Timeout

If you experience Nav2 controller timeouts ("Sensor data too old"), confirm
that the ANELLO driver is publishing at the expected rate:

```bash
ros2 topic hz /imu/data
```

If rates are lower than expected, check your USB connection and the
`poll_interval_ms` parameter.

---

## 7. NTRIP / RTK Setup

RTK corrections dramatically improve position accuracy (from meters to
centimeters). The ANELLO driver includes a built-in NTRIP client.

### 7.1 Launch with NTRIP

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
  baud_rate:=921600 \
  ntrip_host:=ntrip.example.com \
  ntrip_port:=2101 \
  ntrip_mountpoint:=NEAR_ME \
  ntrip_authenticate:=true \
  ntrip_username:=myuser \
  ntrip_password:=mypass
```

### 7.2 Verifying RTK Status

```bash
# Check the RTK fix status on the GPS topic
ros2 topic echo /anello/gps --field rtk_fix_status
# 0 = SPP (no corrections), 1 = RTK Float, 2 = RTK Fix

# Check INS solution status
ros2 topic echo /anello/ins --field ins_status
# 3 = RTK Float, 4 = RTK Fix

# Check position accuracy
ros2 topic echo /anello/gps --field hacc
# Should drop from ~2.0 (SPP) to ~0.02 (RTK Fix) meters

# Check health
ros2 topic echo /anello/health
# position_acc_flag: 0 means centimeter-level
```

### 7.3 Using an External NTRIP Client

If you prefer a standalone NTRIP client (e.g., `str2str` from RTKLIB), you
can disable the built-in one and pipe RTCM data directly:

```bash
# Launch driver without NTRIP
ros2 launch anello_ros_driver anello_driver.launch.py baud_rate:=921600

# In another terminal, publish RTCM data to the driver
ros2 topic pub /ntrip_client/rtcm mavros_msgs/msg/RTCM \
  "{header: {frame_id: 'odom'}, data: [...]}"
```

---

## 8. Odometer Input

The ANELLO INS solution improves significantly with odometer aiding,
especially during GPS outages. If your robot has wheel encoders, forward
the speed to the ANELLO unit:

```bash
# Publish odometer speed (m/s) to the driver
ros2 topic pub /anello/odo anello_interfaces/msg/APODO \
  "{odo_speed: 1.5}" --rate 10
```

### 8.1 Python Example

```python
from anello_interfaces.msg import APODO

class OdoForwarder(Node):
    def __init__(self):
        super().__init__('odo_forwarder')
        self.pub = self.create_publisher(APODO, 'anello/odo', 1)
        # Subscribe to your robot's wheel odometry
        self.create_subscription(Odometry, 'wheel_odom', self.callback, 10)

    def callback(self, msg):
        speed = math.sqrt(
            msg.twist.twist.linear.x ** 2 +
            msg.twist.twist.linear.y ** 2
        )
        odo_msg = APODO()
        odo_msg.odo_speed = speed
        self.pub.publish(odo_msg)
```


---

## 9. Device Configuration via Service

The `anello/send_cmd` service lets you read and write device configuration
at runtime without restarting the driver.

### 9.1 Common Commands

```bash
# Ping the device
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
  "{command: 'APPNG'}"

# Read the antenna baseline
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
  "{command: 'APVEH,R,bsl'}"

# Read the current output data rate
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
  "{command: 'APOUT,R,odr'}"

# Set heading (degrees, for initialization)
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
  "{command: 'APINI,W,hdg,90.0'}"
```

The command string corresponds to the ASCII body without the `#` prefix,
`*` separator, or checksum. The driver computes and appends these
automatically.

---

## 10. Performance Tuning

### 10.1 Polling Interval

The `poll_interval_ms` parameter controls how often the driver reads from
the serial/ethernet port. Lower values reduce latency but increase CPU
usage.

| Setting | Latency | CPU | Use Case |
|---------|---------|-----|----------|
| `1` | ~1 ms | High | Low-latency control loops |
| `5` (default) | ~5 ms | Moderate | Most robotics applications |
| `10` | ~10 ms | Low | Data logging, slow platforms |

```bash
ros2 launch anello_ros_driver anello_driver.launch.py poll_interval_ms:=1
```

### 10.2 QoS Compatibility

The driver publishes sensor data with `SensorDataQoS` (best-effort
reliability, volatile durability). If your subscriber uses a different QoS
profile, you may not receive messages. Match your subscriber:

```cpp
auto sub = node->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data", rclcpp::SensorDataQoS(), callback);
```

```python
from rclpy.qos import qos_profile_sensor_data
self.create_subscription(Imu, 'imu/data', callback, qos_profile_sensor_data)
```

### 10.3 USB Latency

The default USB-serial latency on Linux is 16 ms. Reduce it for lower
latency:

```bash
# Check current latency
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

# Set to 1 ms (requires root)
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

Make this persistent with a udev rule:

```bash
# /etc/udev/rules.d/99-anello-latency.rules
ACTION=="add", SUBSYSTEM=="usb-serial", ATTR{latency_timer}="1"
```

---

## 11. Common Issues and Troubleshooting

### No data on topics

| Symptom | Check |
|---------|-------|
| No topics at all | Is the node running? `ros2 node list` |
| Topics exist but no data | Check serial connection: `ls /dev/ttyUSB*` |
| "Config port init failed" warning | Config port not found. Set `uart_config_port:=OFF` if not needed |
| Data appears then stops | USB cable issue or baud rate mismatch |

### Wrong baud rate

If you see garbled data or checksum failures in the logs:

```
[WARN] Checksum fail: ...
```

Verify the baud rate matches your device:
- EVK: `baud_rate:=921600`
- GNSS INS / IMU+: `baud_rate:=230400`

### Port permission denied

```
[ERROR] Failed to open serial port: /dev/ttyUSB0 (Permission denied)
```

Add your user to `dialout`:

```bash
sudo usermod -aG dialout $USER
```

### GPS accuracy is poor

- Ensure the GNSS antenna has a clear sky view.
- Enable NTRIP for RTK corrections (Section 7).
- Check `ros2 topic echo /anello/health` — `position_acc_flag` should be `0`
  (centimeter-level) with RTK.

### INS heading is wrong at startup

The INS needs a few seconds of driving to converge heading. You can also
initialize heading manually:

```bash
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
  "{command: 'APINI,W,hdg,180.0'}"
```

### robot_localization not receiving IMU data

Confirm QoS compatibility. The ANELLO driver uses `SensorDataQoS`
(best-effort). If `robot_localization` uses reliable QoS, messages will be
dropped. Check with:

```bash
ros2 topic info /imu/data --verbose
```

### High CPU usage

Increase the polling interval:

```bash
ros2 launch anello_ros_driver anello_driver.launch.py poll_interval_ms:=10
```

---

## Appendix: Full Parameter Reference

See the [main README](../readme.md#configuration) for the complete parameter
table covering all driver and NTRIP parameters.
