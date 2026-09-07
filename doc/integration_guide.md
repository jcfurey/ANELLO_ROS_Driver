# Integrating the ANELLO driver

Use the [version 4 migration guide](migration_v4.md) when replacing an earlier driver. [The readme](../readme.md) lists startup parameters and launch commands; [the Ethernet guide](ethernet_setup_guide.md) covers host/device networking.

## Frames and the physical output point

Standard body vectors use FLU (forward, left, up). APINS aerospace roll/pitch/heading are converted from NED/FRD into a quaternion taking FLU body vectors into current geodetic ENU. Geographic east is zero yaw and north is +π/2. Native custom topics retain vendor units/conventions and distinguish FRD frame names.

`frame_id.ins` identifies the point and axes actually described by the firmware INS solution. By default that is `ins_link`; the driver does not infer a base-link mounting transform or shift an INS output point from a URDF joint. `tf_child_frame` must equal `frame_id.ins`. If firmware output-centre settings move the solution to the vehicle base, verify its axes and `ocx/ocy/ocz` configuration before naming that frame `base_link`. A name change alone cannot apply a lever arm. If the firmware has already applied mounting rotation, applying it again in ROS is wrong.

Raw IMU vectors identify their actual sensing/output point with `frame_id.imu`. The default assumes the raw body measurements and INS attitude describe the same axes/reference point. If the firmware output centre differs from the raw IMU origin, set distinct frame IDs. `imu/data` then remains an orientation measurement in `frame_id.ins`, with body fields unavailable. `imu/data_raw` still supplies the fresh raw vectors in their actual frame. A downstream estimator can use verified static extrinsics; this driver does not silently combine measurements at different points.

For a robot with a URDF, mount `ins_link` under `base_link` using measured extrinsics, and leave driver TF disabled. The localization system owns `map → odom → base_link`. For standalone visualization, enable driver TF only when the INS output frame has no other parent. The driver then publishes `anello_local → ins_link` from exactly the pose in `ins/odometry`.

`anello_local` is a fixed WGS84 ECEF-derived ENU tangent frame anchored at the first valid INS latitude, longitude, and ellipsoid height. Position uses all three coordinates. Pose orientation and covariance rotate current local-level axes into the anchor axes; linear twist and its covariance are expressed in the child frame. The anchor survives reconnects/device reboots but is chosen again when the node restarts. Record or establish the datum/alignment in the localization system if repeatability across node restarts is required.

The fused INS position can jump when GNSS corrections or reinitialization change the solution. Accordingly, the default topic is `ins/odometry`, the parent is `anello_local`, and `odom` is rejected as the parent. These measurements do not implement the continuous-pose contract of REP-105 `odom`. A global estimator must establish the relation between this local datum and its map.

## Validity and timing

Checksum/schema validation precedes every state update and publication. Missing required fields, unknown enums, non-finite/overflowing numbers, invalid geodetic coordinates, and invalid covariance matrices are rejected and counted. The accepted INS statuses are 0, 1, 2, 3, 4, 8, 9, and 10. Status 0/8 carries attitude only: `ins/fix` reports no fix with NaN coordinates, and no position-bearing odometry or TF is emitted. Missing optional velocity is retained as unavailable, with conservative odometry twist uncertainty.

INS-derived IMU output combines raw body fields only if acquisition times differ by at most `imu_max_age` and the cached sample is no older than that duration on a steady clock. Covariance uses the equivalent `covariance.max_age` checks. Cache contents are invalidated on transport-generation changes, detected device reset, or ROS clock discontinuity. Reordered acquisition times within the one-second reset tolerance do not cause device reboot detection. Lower/duplicate INS epochs are discarded; multiple newer INS epochs in one arrival-time read retain distinct stamps, nudged by 1 ns when required for TF ordering.

`timestamp_source=mcu` uses a minimum-offset device-to-host filter after 100 validated messages. Host epoch nanoseconds stay integral; only relative time is fitted. This estimates transport offset and drift; it does not provide hardware synchronization or eliminate minimum link latency. Forward/backward ROS-clock steps and clock-source changes reset translation and ordering state together. With `use_sim_time`, stamps use ROS arrival time, zero time suppresses publication, and each stream is suppressed while the simulated clock is paused. Time resets start a new timestamp epoch rather than continuing the old timeline.

For a missing IMU estimate, the applicable covariance element 0 is -1. Unknown orientation uncertainty is an all-zero matrix, as specified by `sensor_msgs/Imu`. Odometry has no corresponding unknown marker: unavailable uncertainty/fields use the configured `covariance.unknown_variance` diagonal (default 1e6). Consumers must treat those as weak estimates, not measured zero velocity.

An enabled FOG returning a stuck/zero sequence degrades health. A selected FOG near its range limit (MEMS magnitude at least 180 deg/s or optical magnitude at least 200 deg/s) is marked unavailable on standard gyro output; its nominal small noise variance cannot describe saturation. Acceleration remains usable when gyro availability alone is lost. Set `use_fog_wz=false` when the optical gyro is disabled in firmware.

## Covariance and physical accuracy

The default `covariance.device_convention=unknown` leaves APCOV available on the custom topic but does not claim its matrices are known ROS uncertainties. Set `verified_m2_deg2_euler` only after confirming all of this for the device/model/firmware:

- Position covariance is in local north/east/up metres squared, in lat/lon/alt field order, including the signs of cross terms.
- Velocity covariance is NED in (m/s)².
- Attitude covariance is aerospace roll/pitch/heading **Euler-angle covariance in degrees squared**, rather than quaternion/small-angle covariance in another basis.
- The covariances refer to the reported output centre and timestamps.

That mode permutes/signs the position and velocity blocks, converts attitude units, applies the Euler-to-fixed-axis Jacobian at the current attitude, and rotates blocks into their published frames. APCOV does not provide all position-attitude/velocity-attitude cross correlations; the 6×6 odometry representation uses a block-diagonal approximation. Stale or entirely zero blocks do not claim perfect accuracy. Other firmware covariance conventions require an explicit new conversion with fixtures; do not infer units from magnitude alone.

Raw GNSS `gps/fix` covariance uses Hacc² on east/north and Vacc² on up, marked **APPROXIMATED** because the receiver confidence definition must be confirmed. A 2D fix has NaN altitude and unknown covariance; invalid fixes do not become RTK-valid from a retained RTK field. `gnss_service_mask=0` avoids claiming unverified constellations. Configure the actual NavSatStatus mask when known.

Verify acceleration sign in six static orientations and gyro sign with known positive rotations. The startup gravity warning is opt-in for a known upright mount; it cannot diagnose an arbitrary orientation. Automatic covariance estimates assume the configured sample rate and white noise. Use measured variance/bandwidth for accuracy work.

## Choosing measurements for localization

For a system that uses the onboard INS solution, consume `ins/odometry` as a globally corrected measurement, with verified covariance and map/datum alignment. Let the robot estimator manage continuous odometry and the robot TF tree. Do not add the driver’s standalone transform to a sensor already parented by URDF.

For a system that performs its own fusion, `gps/fix` supplies the primary receiver solution and `imu/data_raw` supplies the fresh body measurements. `imu/data` contains onboard INS attitude and may be correlated with GNSS/INS output. Account for those correlations; feeding fused INS position together with the same raw GNSS/IMU as independent measurements can overstate confidence. `navsat_transform_node` requires a correctly referenced orientation and valid extrinsics/datum; relative APAHRS yaw alone does not provide that reference.

For Ground IMU set `expected_streams: [imu]` (or `[imu, ahrs]` for the AHRS upgrade). INS deployments normally expect `[imu, ins, gps]`; add `heading`, `gps2`, or `covariance` only when those streams are configured. Diagnostics report each expected stream’s age. APHEALTH is withheld while required streams are stale; its transient-local history is not a heartbeat. Monitor diagnostics age as well as status, and configure the measured `heading_baseline` explicitly.

## Sources and validation

The conversions and contracts follow [REP-103](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0103.rst), [REP-105](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst), and the [Imu](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/sensor_msgs/msg/Imu.msg), [NavSatFix](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/sensor_msgs/msg/NavSatFix.msg), and [Odometry](https://raw.githubusercontent.com/ros2/common_interfaces/rolling/nav_msgs/msg/Odometry.msg) definitions. [REP-145](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0145.rst) supplies useful IMU-driver guidance and remains Draft.

The GGA UTC offset is configurable; [IERS Bulletin C](https://datacenter.iers.org/data/latestVersion/bulletinC.txt) supplies current leap-second announcements. APGPS PDOP is not used as GGA HDOP.

The manufacturer documents [message layouts and status](https://docs-a1.readthedocs.io/en/latest/communication_messaging.html) and [vehicle orientation/output centre](https://docs-a1.readthedocs.io/en/latest/vehicle_configuration.html). Its public messaging reference does not establish the APCOV contract. Use the [hardware checklist](hardware_validation_checklist.md) to close those installation-dependent questions; software test success is not a substitute for that evidence.
