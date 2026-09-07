# Hardware validation after the software audit

The automated suites cover decoding, conversion math, clocks, transport recovery, and ROS publication using simulated devices. The checks below require the actual model, firmware, mounting, and receiver configuration. None should be marked complete solely because the software suite passed.

Record the device model/serial, firmware version, parameter file, output rates/filter settings, firmware input/output rotations, output-centre offsets, antenna offsets, temperature, and rosbag/log paths for each run.

## Bench

- [ ] Check `imu/data_raw`, `anello/imu_raw` or `anello/im1`, and the streams named in `expected_streams`. Rates must match configured device output, with no sustained framing-error growth in `/diagnostics`.
- [ ] Verify `anello/send_cmd` with `{command: 'APPNG'}` and record the checksum-valid reply. For Ethernet, check configured host/device ports; for UART, use stable symlinks where available.
- [ ] Power-cycle and replace the device while the node remains running. Both data and config service/APODO transmission must recover. Verify cached IMU/covariance is unavailable during loss, and no position-bearing output is published without a valid INS solution.
- [ ] With config `OFF`, confirm it remains disabled and commands report unavailable rather than succeeding silently.
- [ ] Test **six static orientations**, placing each positive and negative body axis upward. Converted specific-force acceleration should be approximately +g on the upward axis, with the other components near zero. Record whether `flip_accel_sign` is required. One level pose cannot establish all three signs or arbitrary mounting alignment.
- [ ] Apply known positive rotations around each FLU body axis and verify gyro signs. Confirm whether firmware has already applied vehicle mounting correction; do not apply the same correction twice in TF.
- [ ] Validate the physical output point and axes for each raw/INS stream. Check `ocx/ocy/ocz` and antenna/IMU lever arms against surveyed values. Distinct output points require distinct frame IDs or a verified transformation in the estimator.
- [ ] Confirm the selected FOG/MEMS channel and measure its variance at the actual output rate/filter bandwidth. FOG-disabled units use `use_fog_wz=false`. Check that enabled stuck/zero FOG output and saturation are not treated as precise angular measurements.
- [ ] Collect enough stationary/dynamic data to assess health thresholds; the gyro window is ten samples and its limits are heuristics, not a statistical guarantee of zero false alarms.
- [ ] Validate translated versus arrival stamps against a hardware reference where available. Expect residual minimum transport delay; the software clock filter is not PPS/PTP synchronization. Check reset behavior on device reboot and host-clock adjustments.

## Firmware covariance contract

- [ ] Obtain the exact APCOV units, field order, error basis, cross-term signs, acquisition timing, and output-centre relationship from ANELLO for this firmware.
- [ ] Leave `covariance.device_convention=unknown` until those facts are established. Values that look small do not prove degree-squared units; values that look plausible do not prove metre-squared units.
- [ ] Use `verified_m2_deg2_euler` only if all assumptions in the integration guide match. Different Euler/small-angle conventions require different conversion code and fixtures.
- [ ] Compare the transformed covariance to measured residuals and a trusted reference, including nonzero roll/pitch, turns, and output offsets. The odometry blocks omit cross correlations not supplied by APCOV.
- [ ] Check cache expiry by disabling APCOV alone while other streams continue; it must not remain known indefinitely.

## Outdoor and dynamic tests

- [ ] Confirm raw GNSS FixType and RTK status agree with `gps/fix`; time-only/no-fix must not appear as a valid fix. Two-dimensional height must be NaN.
- [ ] Record the actual constellations and configure `gnss_service_mask`. Obtain the confidence definitions for Hacc/Vacc before relying on Hacc²/Vacc²; published covariance is marked approximated.
- [ ] Check geographic east/north headings and turns against an independent reference. APINS attitude and relative APAHRS yaw have different heading contracts.
- [ ] If using dual antennas, survey baseline length and set `heading_baseline`. Verify APHDG flags, heading, and reference point.
- [ ] Exercise NTRIP with the real caster: startup outage, no first correction, link loss, server recovery, TLS verification, and VRS GGA requirements. Check NTRIP diagnostics and driver transmission counters as well as receipt on the ROS RTCM topic.
- [ ] Confirm GGA time, ellipsoid/MSL height relationship, and caster acceptance of unavailable HDOP. APGPS supplies PDOP, which the driver does not relabel as HDOP. Review `gps_utc_leap_seconds` against current IERS announcements.
- [ ] Compare local Cartesian position against surveyed points, including altitude and a route long enough to expose tangent-plane curvature. Record the first-fix origin and its relationship to the robot map.
- [ ] Deliberately lose position validity after acquiring a fix. INS attitude may remain available; `ins/odometry` and driver TF must stop until position is valid again.
- [ ] Verify signed wheel-speed aiding in forward and reverse and after a UART config-channel reconnect.
- [ ] Check estimator consistency, message freshness, and uncertainty through starts/stops/turns and GNSS degradation. Avoid treating raw GNSS/IMU and their fused INS output as independent observations.
- [ ] Verify a single owner for every TF child. Normally the estimator owns `map → odom → base_link`, URDF owns static sensor transforms, and driver TF stays disabled. Its optional `anello_local → ins_link` transform is for a standalone topology without another parent.

Use [the migration guide](migration_v4.md) to update old topic/frame/parameter expectations. Record hardware acceptance results separately from software test results.
