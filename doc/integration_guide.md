# Integrating the ANELLO driver

Use the [version 4 migration guide](migration_v4.md) when replacing an earlier driver. [The readme](../readme.md) lists startup parameters and launch commands; [the Ethernet guide](ethernet_setup_guide.md) covers host/device networking.

## Frames and the physical output point

Standard body vectors use FLU (forward, left, up). APINS aerospace roll/pitch/heading are converted from NED/FRD into a quaternion taking FLU body vectors into current geodetic ENU. Geographic east is zero yaw and north is +π/2. Native custom topics retain vendor units/conventions and distinguish FRD frame names.

`frame_id.ins` identifies the point and axes actually described by the firmware INS solution. By default that is `ins_link`; the driver does not infer a base-link mounting transform or shift an INS output point from a URDF joint. `tf_child_frame` must equal `frame_id.ins`. If firmware output-centre settings move the solution to the vehicle base, verify its axes and `ocx/ocy/ocz` configuration before naming that frame `base_link`. A name change alone cannot apply a lever arm. If the firmware has already applied mounting rotation, applying it again in ROS is wrong.

Raw IMU vectors identify their actual sensing/output point with `frame_id.imu`. The default assumes the raw body measurements and INS attitude describe the same axes/reference point. If the firmware output centre differs from the raw IMU origin, set distinct frame IDs. `imu/data` then remains an orientation measurement in `frame_id.ins`, with body fields unavailable. `imu/data_raw` still supplies the fresh raw vectors in their actual frame. A downstream estimator can use verified static extrinsics; this driver does not silently combine measurements at different points.

For a robot with a URDF, mount `ins_link` under `base_link` using measured extrinsics, and leave driver TF disabled. The localization system owns `map → odom → base_link`. For standalone visualization, enable driver TF only when the INS output frame has no other parent. The driver then publishes `anello_local → ins_link` from exactly the pose in `ins/odometry`.

`anello_local` is a fixed WGS84 ECEF-derived ENU tangent frame anchored at the first INS solution with valid position and absolute heading. Position uses latitude, longitude, and ellipsoid height. Pose orientation and covariance rotate current local-level axes into the anchor axes; linear twist and its covariance are expressed in the child frame. The anchor survives reconnects/device reboots but is chosen again when the node restarts. Record or establish the datum/alignment in the localization system if repeatability across node restarts is required.

The fused INS position can jump when GNSS corrections or reinitialization change the solution. Accordingly, the default topic is `ins/odometry`, the parent is `anello_local`, and `odom` is rejected as the parent. These measurements do not implement the continuous-pose contract of REP-105 `odom`. A global estimator must establish the relation between this local datum and its map.

## Validity and timing

Checksum/schema validation precedes every state update and publication. Missing required fields, unknown enums, non-finite/overflowing numbers, invalid geodetic coordinates, and invalid covariance matrices are rejected and counted. The accepted INS statuses are 0, 1, 2, 3, 4, 8, 9, and 10. Position and absolute heading have separate availability rules:

| APINS status | `ins/fix` position | `imu/data` ENU orientation | `ins/odometry` and driver TF |
|---|---|---|---|
| 0, 8 | No fix; NaN coordinates | Unavailable | Withheld |
| 1, 9 | Available | Unavailable | Withheld |
| 2, 3, 4, 10 | Available | Available | Published |

Without absolute heading, the standard IMU uses a neutral identity quaternion and `orientation_covariance[0]=-1`; fresh acceleration/gyro remain independently available. Native `anello/ins` retains the reported angles and status. Missing optional velocity is retained as unavailable, with conservative odometry twist uncertainty.

INS-derived IMU output combines raw body fields only if acquisition times differ by at most `imu_max_age` and the cached sample is no older than that duration on a steady clock. Covariance uses the equivalent `covariance.max_age` checks. Cache contents are invalidated on transport-generation changes, a device-time regression exceeding one second, or ROS clock discontinuity. A large device-time regression starts an inferred clock epoch: it can also result from a sufficiently delayed/replayed packet and is not proof of a device reboot.

Within a clock epoch, each logical stream rejects duplicate or older device timestamps before updating caches, health windows, or the clock translator. APIMU and APIM1 share the `imu` stream; INS, GPS, GPS2, heading, covariance, and AHRS have independent ordering. This permits interleaved GNSS latency within the one-second clock-reset tolerance. Rejected packets cannot keep a stream fresh. Multiple newer packets in one read retain distinct ROS stamps, nudged by 1 ns when necessary. A non-increasing ROS stamp across reads is rejected and counted.

`timestamp_source=mcu` uses a minimum-offset device-to-host filter after 100 validated messages that pass device-time ordering. Host epoch nanoseconds stay integral; only relative time is fitted. This estimates transport offset and drift; it does not provide hardware synchronization or eliminate minimum link latency. Forward/backward ROS-clock steps and clock-source changes reset translation and ordering state together. With `use_sim_time`, stamps use ROS arrival time, zero time suppresses publication, and each stream is suppressed while the simulated clock is paused. Time resets start a new timestamp epoch rather than continuing the old timeline.

For a missing or rejected IMU vector, all three components are NaN and the applicable covariance element 0 is -1. An available estimate with unknown uncertainty uses an all-zero IMU covariance matrix. Odometry has no corresponding covariance marker: unavailable twist vectors are NaN, and missing uncertainty/fields use the configured `covariance.unknown_variance` diagonal (default 1e6). Consumers must discard unavailable fields rather than interpreting them as measured zero velocity.

An enabled FOG returning a stuck/zero sequence degrades health. A selected FOG near its range limit (MEMS Z magnitude at least 180 deg/s or optical Z magnitude at least 200 deg/s) immediately makes the standard gyro vector unavailable, sets APHEALTH gyro status to bad, and reports the range-guard reason in diagnostics. This describes measurement usability, not a firmware BIT or proof of hardware failure. Acceleration and native raw values remain available. There is no automatic source switch; set `use_fog_wz=false` to select MEMS Z, including when the optical gyro is disabled in firmware. The optical range guard does not gate selected MEMS output.

During the initial ten-sample health window, APHEALTH reports gyro assessment unavailable and diagnostics warn. Range-valid measurements may still be published; the window being incomplete is not a detected sensor fault. `gyro_reason` explains that distinction, and `gyro_measurement_available` reports whether the latest raw gyro sample remains usable within `stream_timeout`. Fused IMU fields additionally require the tighter cache/frame checks above.

### Continuity diagnostics

For each observed stream, diagnostics expose accepted, duplicate, out-of-order, ROS-stamp rejection, and gap-event totals, plus the recent accepted rate and last/maximum device interval. A gap event means successive accepted samples exceed `stream_timeout` in device time or steady arrival time; it is not an exact missing-sample count. The accepted rate uses a five-second window. Counters and maximum intervals survive clock epochs; recent rates and current interval state reset. The aggregate `messages_total`/`message_rate_hz_recent` still count checksum/schema-valid packets, including packets later rejected by ordering.

`transport_generation_changes_total`, `device_time_regressions_total`, `ros_clock_changes_total`, and `clock_mode_changes_total` distinguish reset causes; `last_measurement_reset_reason` names them. Initial transport acquisition counts as a generation change. Legacy `clock_resets` counts all measurement-state resets, including transport changes, and must not be interpreted as a hardware reboot counter.

## Covariance and physical accuracy

The default `covariance.device_convention=unknown` leaves APCOV available on the custom topic but does not claim its matrices are known ROS uncertainties. Set `verified_m2_deg2_euler` only after confirming all of this for the device/model/firmware:

- Position covariance is in local north/east/up metres squared, in lat/lon/alt field order, including the signs of cross terms.
- Velocity covariance is NED in (m/s)².
- Attitude covariance is aerospace roll/pitch/heading **Euler-angle covariance in degrees squared**, rather than quaternion/small-angle covariance in another basis.
- The covariances refer to the reported output centre and timestamps.

That mode permutes/signs the position and velocity blocks, converts attitude units, applies the Euler-to-fixed-axis Jacobian at the current attitude, and rotates blocks into their published frames. APCOV does not provide all position-attitude/velocity-attitude cross correlations; the 6×6 odometry representation uses a block-diagonal approximation. Stale blocks or blocks with any zero diagonal do not claim perfect accuracy. Position, velocity, and attitude availability are checked independently, so an unavailable block does not discard the other estimates. Other firmware covariance conventions require an explicit new conversion with fixtures; do not infer units from magnitude alone.

Raw GNSS `gps/fix` covariance uses Hacc² on east/north and Vacc² on up, marked **APPROXIMATED** because the receiver confidence definition must be confirmed. A 2D fix has NaN altitude and unknown covariance; invalid fixes do not become RTK-valid from a retained RTK field. `gnss_service_mask=0` avoids claiming unverified constellations. Configure the actual NavSatStatus mask when known.

Verify acceleration sign in six static orientations and gyro sign with known positive rotations. The startup gravity warning is opt-in for a known upright mount; it cannot diagnose an arbitrary orientation. Empty `covariance.angular_velocity` and `covariance.linear_acceleration` arrays leave uncertainty unknown (all zeros). No per-sample noise is inferred from the output rate or a model datasheet. Supply three finite nonnegative diagonal variances in published FLU axes and final SI units from measurements of the actual unit, selected FOG/MEMS channel, output rate, and filter bandwidth. `imu_output_rate_hz` remains accepted for compatibility but does not determine covariance. A partially zero explicit array means known zero variance on those axes; use an entirely zero array for unknown uncertainty.

## Choosing measurements for localization

For a system that uses the onboard INS solution, consume `ins/odometry` as a globally corrected measurement, with verified covariance and map/datum alignment. Let the robot estimator manage continuous odometry and the robot TF tree. Do not add the driver’s standalone transform to a sensor already parented by URDF.

For a system that performs its own fusion, `gps/fix` supplies the primary receiver solution and `imu/data_raw` supplies the fresh body measurements. `imu/data` contains onboard INS attitude and may be correlated with GNSS/INS output. Account for those correlations; feeding fused INS position together with the same raw GNSS/IMU as independent measurements can overstate confidence. `navsat_transform_node` requires a correctly referenced orientation and valid extrinsics/datum; relative APAHRS yaw alone does not provide that reference.

For Ground IMU set `expected_streams: [imu]` (or `[imu, ahrs]` for the AHRS upgrade). INS deployments normally expect `[imu, ins, gps]`; add `heading`, `gps2`, or `covariance` only when those streams are configured. Diagnostics report each expected stream’s age. APHEALTH is withheld while required streams are stale; its transient-local history is not a heartbeat. Monitor diagnostics age as well as status, and configure the measured `heading_baseline` explicitly.

## Sources and validation

The conversions and contracts follow [REP-103](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst), [REP-105](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst), and the [Imu](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/sensor_msgs/msg/Imu.msg), [NavSatFix](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/sensor_msgs/msg/NavSatFix.msg), and [Odometry](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/nav_msgs/msg/Odometry.msg) definitions. [REP-145](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0145.rst) supplies useful IMU-driver guidance and remains Draft.

The GGA UTC offset is configurable; [IERS Bulletin C](https://datacenter.iers.org/data/latestVersion/bulletinC.txt) supplies current leap-second announcements. APGPS PDOP is not used as GGA HDOP.

The manufacturer documents [message layouts and status](https://docs-a1.readthedocs.io/en/latest/communication_messaging.html) and [vehicle orientation/output centre](https://docs-a1.readthedocs.io/en/latest/vehicle_configuration.html). Its public messaging reference does not establish the APCOV contract. Use the [hardware checklist](hardware_validation_checklist.md) to close those installation-dependent questions; software test success is not a substitute for that evidence.
