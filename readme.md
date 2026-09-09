# ANELLO ROS 2 driver — 4.0.0

ROS 2 component and standalone driver for ANELLO EVK, Ground INS, and Ground IMU devices, with UART/UDP transport and an optional NTRIP correction client. Device configuration and physical calibration remain the responsibility of the installation.

Version 4 changes navigation frames, validity handling, and covariance defaults. Read [the migration guide](doc/migration_v4.md) before updating an existing launch or fusion configuration.

## Build and test

From the containing ROS workspace, with the desired ROS distribution sourced:

```bash
rosdep install --from-paths src/ANELLO_ROS_Driver --ignore-src -r -y
colcon build --packages-up-to anello_ros_driver ntrip_client
source install/setup.bash
colcon test --packages-select anello_interfaces anello_ros_driver ntrip_client \
  --python-testing pytest --return-code-on-test-failure
colcon test-result --verbose
python3 src/ANELLO_ROS_Driver/tools/check_test_results.py build
```

The CI matrix covers Humble, Jazzy, Kilted, and Lyrical. The September 2026 remediation was built and exercised locally on **Lyrical**; other distributions require their CI results before being called validated. The service API uses the Humble QoS profile overload where needed.

The suites cover full ASCII/RTCM frames, numerical conversion, covariance and coordinate math, clock resets, pseudo-terminal reconnection/backpressure, simulated casters, and the installed driver/launch files on localhost. They do not establish physical sensor accuracy. `colcon.pkg` selects pytest for the Python package when `colcon-metadata` is installed; the explicit flag above also works without that extension. CI checks for missing or empty result files. An ament cppcheck run that skips checks is not static-analysis coverage.

## Launch

```bash
# Stable symlinks are preferred; AUTO scans ttyUSB* candidates.
ros2 launch anello_ros_driver anello_driver.launch.py \
  uart_data_port:=/dev/serial/by-id/DEVICE_DATA \
  uart_config_port:=/dev/serial/by-id/DEVICE_CONFIG

# EVK factory baud rate
ros2 launch anello_ros_driver anello_driver.launch.py baud_rate:=921600

# Ethernet: host ports must agree with the device configuration.
ros2 launch anello_ros_driver anello_driver.launch.py com_type:=ETH \
  remote_ip:=192.168.1.111 local_data_port:=1111 \
  local_config_port:=2222 local_odometer_port:=3333

# All startup parameters can be supplied in a normal ROS parameter file.
ros2 launch anello_ros_driver anello_driver.launch.py params_file:=/absolute/path/anello.yaml
```

Unspecified launch arguments preserve the parameter file or node default. Launch arguments for dotted parameters replace `.` with `_`, for example `frame_id_imu` and `covariance_angular_velocity`. String arguments including `OFF` and numeric credentials keep their string type. The legacy XML entry point includes the Python implementation and accepts the former `host`, `username`, `ssl`, and related NTRIP aliases.

Both UART channels recover after a device replacement. Config `OFF` stays disabled. AUTO configuration probing runs incrementally; startup does not wait for a device. A command sent while the channel is unavailable returns an error, and odometer/correction transmission failures appear in diagnostics. The standalone executable uses a multithreaded executor. Use `component_container_mt` when loading `anello::AnelloRosDriver` so bounded command/transmit waits cannot hold the receive callback.

Serial ports are exclusively claimed before changing settings or flushing. Data/config aliases of the same device are rejected; corrections wait for a confirmed data stream and cannot follow a stale port generation. See [EVK device input protection](doc/evk_device_inputs.md) for the outbound limits and remaining firmware validation.

## Standard interfaces

Data and command topic names are relative to the node namespace. Diagnostics and TF use the usual global ROS topics. Sensor publishers use `SensorDataQoS` (best effort, depth 5). RTCM input/output is reliable with depth 10; health is reliable and transient local with depth 1.

| Topic | Type | Contract |
|---|---|---|
| `imu/data_raw` | `sensor_msgs/Imu` | APIMU/APIM1 acceleration and selected gyro, SI units, FLU axes; orientation unavailable |
| `imu/data` | `sensor_msgs/Imu` | APINS attitude in current geodetic ENU; body fields included only when fresh and in the same declared frame |
| `gps/fix` | `sensor_msgs/NavSatFix` | Primary GNSS receiver solution; 2D height is NaN; accuracy-derived covariance is approximated |
| `ins/fix` | `sensor_msgs/NavSatFix` | Fused INS position; no-fix status and NaN coordinates when position is unavailable |
| `ins/odometry` | `nav_msgs/Odometry` | Globally corrected INS solution: ECEF-derived local ENU pose, body-frame twist; emitted only with valid position |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Stream ages, health, framing errors, transport failures, clock resets; status names identify the node namespace |
| `ntrip_client/rtcm` | `rtcm_msgs/Message` | Checksum-verified corrections sent to the device data channel |
| `ntrip_client/nmea` | `nmea_msgs/Sentence` | GGA generated from primary GNSS for the caster |

Device-native topics are `anello/{imu_raw,im1,ins,gps,gps2,hdg,cov,ahrs,health}` using the matching `anello_interfaces` message definitions. Units remain documented in those messages: acceleration in g, rates and angles in degrees, MCU time in milliseconds; APIMU odometer time is seconds. IMU/INS/covariance/AHRS native frame identifiers have `_frd` appended to distinguish them from converted FLU output. They are not substitutes for standard ROS sensor messages. GPS2 identifies the secondary antenna separately.

APAHRS/subtype 8 is supported on `anello/ahrs`. Its yaw can be relative; it is not automatically presented as a north-referenced `imu/data` orientation. Legacy APIMU/APIM1 ASCII layouts without synchronization time remain supported. RTCM APIMU supports both old and current layouts; other binary layouts must match the documented structure exactly. Unknown extensions are rejected and counted.

## Principal startup parameters

Parameters are read-only while running; restart to change them.

| Parameter | Default | Meaning |
|---|---|---|
| `com_type` | `UART` | `UART` or `ETH` |
| `uart_data_port`, `uart_config_port` | `AUTO` | Serial path or discovery; config also accepts `OFF` |
| `baud_rate` | `230400` | 115200, 230400, 460800, or 921600 |
| `remote_ip` | `192.168.1.111` | Device IPv4 address |
| `local_data_port`, `local_config_port`, `local_odometer_port` | 1111, 2222, 3333 | Host UDP ports; device destination ports are 1, 2, 3 |
| `frame_id.imu`, `frame_id.ins` | `ins_link` | Actual output origins/axes; see the integration contract |
| `frame_id.gnss`, `frame_id.gnss2`, `frame_id.hdg` | `gnss_link`, `gnss2_link`, `gnss_link` | Antenna/heading reference frames |
| `publish_tf` | `false` | Optional standalone INS transform |
| `tf_parent_frame`, `tf_child_frame` | `anello_local`, `ins_link` | Global local-Cartesian frame and firmware output frame; child must equal `frame_id.ins` |
| `poll_interval_ms` | 5 | Nonblocking read cadence, up to 16 buffers per callback |
| `timestamp_source` | `mcu` | Device-to-host translation after 100 valid samples; `arrival` uses read time |
| `use_fog_wz`, `flip_accel_sign` | `true`, `false` | Select optical Z gyro; optionally invert all acceleration axes |
| `imu_max_age`, `covariance.max_age` | 0.05, 0.2 s | Maximum acquisition-time separation and steady-clock cache age |
| `covariance.device_convention` | `unknown` | Enable `verified_m2_deg2_euler` only after obtaining the firmware contract |
| `covariance.unknown_variance` | 1e6 | Conservative odometry diagonal fallback when uncertainty/fields are unavailable |
| `covariance.angular_velocity`, `covariance.linear_acceleration` | `[]` | Three nonnegative finite SI variances; empty means unknown uncertainty |
| `imu_output_rate_hz` | 100 | Legacy rate hint; does not configure device rate or determine covariance |
| `expected_streams` | `[imu, ins, gps]` | Streams required for healthy diagnostics; Ground IMU normally uses `[imu]` |
| `stream_timeout` | 2 s | Maximum silence for each expected stream |
| `heading_baseline` | 0 m | Measured dual-antenna separation; zero skips baseline comparison |
| `gps_utc_leap_seconds` | 18 | GPS minus UTC seconds for GGA; update from IERS announcements |
| `gnss_service_mask` | 0 | Configured constellation mask; zero means unspecified |
| `accel_sign_check_upright` | `false` | Opt-in startup gravity check for a known upright installation |
| `publish_custom_messages` | `true` | Publish the device-native topics |
| `command_mode` | `read_only` | Queries and echo only; `unrestricted` explicitly permits configuration, reset, and other device commands |
| `rtcm.max_bytes_per_second` | 8192 | Admission budget, burst 4096 bytes; UART also caps it at baud/20 bytes/s |
| `rtcm.max_frames_per_second` | 100 | Admission budget, burst 16 complete RTCM frames |
| `odometer.max_speed_mps` | 100 | Reject larger absolute speeds; configurable up to 1000 m/s |
| `odometer.max_rate_hz` | 50 | Send at most this rate, no burst; configurable up to 100 Hz |

Empty covariance arrays now publish all-zero IMU covariance (unknown), for both FOG and MEMS selection. Earlier revisions inferred small variances from one model's datasheet random-walk specifications and the requested output rate; those assumptions did not establish this unit's bandwidth or accuracy. Supply explicit measured SI variances for the selected gyro source, unit, output rate, and filter settings. Explicit values are already in the published FLU axes and final SI units and are not scaled with `imu_output_rate_hz`. Odometry uses the configured conservative fallback for unknown uncertainty.

## NTRIP and commands

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
  ntrip_host:=caster.example.org ntrip_port:=2101 ntrip_mountpoint:=MOUNT \
  ntrip_authenticate:=true ntrip_username:=USER ntrip_password:=PASSWORD \
  ntrip_version:=Ntrip/2.0 ntrip_ssl:=true

ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp "{command: 'APPNG'}"
ros2 topic pub -r 10 /anello/odo anello_interfaces/msg/APODO '{odo_speed: 1.25}'
```

For credentials, prefer a protected ROS parameter file supplied via `ntrip_params_file`, with `ntrip_enable:=true`. All NTRIP node parameters below have launch overrides prefixed `ntrip_`, except `ntrip_version` itself: `host`, `port`, `mountpoint`, `authenticate`, `username`, `password`, `ssl`, `cert`, `key`, `ca_cert`, `rtcm_frame_id`, `reconnect_attempt_wait_seconds`, `rtcm_timeout_seconds`, `nmea_min_interval_seconds`, and `nmea_max_age_seconds`.

NTRIP starts even when the caster is unavailable and retries indefinitely. Default retry delay is 5 s; the first/next valid-correction deadline is 4 s. GGA sending defaults to every 10 s and stops when the last valid sentence is older than 30 s. Network I/O runs outside ROS callbacks with bounded correction buffering; expired queued corrections are discarded. NTRIP diagnostics report connection state, valid-frame age, and queue drops/expiry. The deprecated `reconnect_attempt_max` parameter is accepted for compatibility but no longer stops retries. HTTP/1.0, HTTP/1.1, ICY, TLS certificate validation, and bounded chunk decoding are supported.

Commands accept a 5..128-byte body such as `APVEH,R,bsl`; the driver adds framing/checksum and returns a checksum-verified reply with the matching message identifier (or APERR). `command_mode=read_only` is the default. Start with `command_mode:=unrestricted` only when intentionally configuring or resetting the device. Commands are limited to two per second. The response wait is bounded to 500 ms; an explicitly permitted `APRST,0` returns `SENT` without claiming acknowledgement, because that command has no reply. The driver never retries it automatically. Concurrent identical unsolicited responses cannot be correlated more precisely because the protocol has no transaction ID. `InitHeading`/`UpdHeading` interfaces remain for source compatibility but have no advertised service; use documented device commands through `send_cmd`.

See [integration](doc/integration_guide.md), [hardware validation](doc/hardware_validation_checklist.md), [Ethernet setup](doc/ethernet_setup_guide.md), and [the EVK reference](doc/anello_evk_reference.md).
