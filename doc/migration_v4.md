# Migrating to driver 4.0.0

This release implements the September 2026 driver audit fixes. Rebuild all three packages, including consumers of `anello_interfaces`, and review startup configuration before restarting the driver. No firmware settings are changed automatically.

## Changes that affect existing consumers

| Previous behavior | Version 4 behavior / migration |
|---|---|
| `odom` topic, parent `odom`, child `base_link`, TF enabled | `ins/odometry`, parent `anello_local`, child `ins_link`, TF disabled. This is a globally corrected solution, not continuous odometry. |
| Changing `tf_child_frame` relabelled the output | Child must equal `frame_id.ins`. Use the real firmware output point/axes; configure and verify the firmware before declaring a base-link output. |
| Separate default `imu_link`/`ins_link` labels combined without checking | Both default to `ins_link`. If raw IMU and INS use different output points/axes, configure distinct frames; fused IMU body fields then remain unavailable. |
| Native FRD data shared FLU frame labels | Native IMU/INS/covariance/AHRS frame names receive `_frd`; GPS2 uses `frame_id.gnss2=gnss2_link`. |
| Position-bearing messages before a valid fix and after fix loss | No odometry/TF without valid INS position; no-fix `ins/fix` coordinates are NaN. Attitude remains separate. |
| Old IMU/APCOV samples reused indefinitely | Acquisition and steady-clock age limits, plus cache resets on reconnect/reboot/clock changes. |
| APCOV assumed to be metres²/degrees² | Default is `covariance.device_convention=unknown`. Opt into `verified_m2_deg2_euler` only with firmware evidence; see integration guide. |
| Zero odometry covariance used before measurements | Configurable conservative fallback variance (default 1e6). Missing IMU estimates use covariance[0]=-1. |
| Fixed FOG covariance even with MEMS selected | Empty covariance arrays select source/rate-dependent estimates. Explicit three-element arrays override them and must be finite/nonnegative. |
| Approximate latitude/longitude scaling | WGS84 ECEF-to-ENU position, including altitude, with consistent anchor/current-axis rotations and attitude covariance Jacobian. |
| All four GNSS constellations claimed | `gnss_service_mask=0` unless configured. Two-dimensional height is unavailable; Hacc/Vacc-derived covariance is APPROXIMATED. |
| GGA PDOP placed in HDOP | HDOP is left empty, geoid separation comes from ellipsoid minus MSL height, and `gps_utc_leap_seconds` makes UTC conversion configurable. |
| Startup gravity warning assumed upright orientation | Opt in with `accel_sign_check_upright=true` only for a known upright installation. Six-face/rotation verification is still required. |
| Config UART required restart after device loss | Fixed-path reopen and incremental AUTO probing recover it. `OFF` stays disabled. Initial device absence is tolerated. |
| Startup queried antenna baseline | Configure `heading_baseline` explicitly. Zero skips the check; it no longer adds a blocking startup query. |
| Aggregate traffic hid missing streams; zero FOG samples ignored | Explicit `expected_streams`, per-stream ages, unavailable health states, and enabled-FOG zero/stuck detection. Ground IMU normally selects `[imu]`. |
| FOG saturation retained its small nominal variance | Selected saturated FOG rate is unavailable on standard output; acceleration remains usable. |
| Clock translation crept across host clock steps | Translation, caches, and ordering reset together; paused/uninitialized simulated time does not create advancing measurements. |
| Partial/failed writes were not visible to callers | Complete bounded serial writes and checked UDP sends; failures counted in diagnostics; RTCM uses its own callback group. |
| NTRIP exited when startup connection failed | Background persistent retries, first/valid-frame deadlines, bounded queue, fresh GGA resend, expired correction rejection, and diagnostics. |
| `reconnect_attempt_max` stopped recovery | Accepted as deprecated compatibility input; the worker continues retrying until shutdown. |
| Python/XML launch values inferred through YAML | Typed overrides; parameter-file support; XML delegates to Python. Unspecified overrides preserve file/node values. |
| `tests_require` selected pytest | Supported extras metadata plus `colcon.pkg`; CI/docs also explicitly select pytest and check for missing results. |
| Arbitrary bytes on the RTCM subscription reached the device | Entire bundles must pass RTCM envelope/CRC checks; output telemetry, ASCII, partial/corrupt frames, and trailing junk are rejected. Bundles are split into individual frames. |
| Corrections followed an unconfirmed UART scan candidate | Corrections require validated telemetry on the same open-port generation. Serial ownership prevents data/config collisions. |
| Unbounded odometer values and outbound traffic | Finite speed/rate limits and RTCM byte/frame budgets reject excess input, with diagnostic counters. |
| Any AP command body could change or reset firmware | `command_mode=read_only` by default; intentional configuration/reset needs `unrestricted`. Commands are bounded to 128 body bytes and two per second. Reset transmission no longer reports an expected missing reply as a timeout. |

For example, a parameter file for an INS at its sensor reference point is:

```yaml
anello_ros_driver:
  ros__parameters:
    com_type: UART
    uart_data_port: /dev/serial/by-id/DEVICE_DATA
    uart_config_port: /dev/serial/by-id/DEVICE_CONFIG
    baud_rate: 230400
    frame_id.imu: ins_link
    frame_id.ins: ins_link
    tf_child_frame: ins_link
    tf_parent_frame: anello_local
    publish_tf: false
    expected_streams: [imu, ins, gps]
    timestamp_source: mcu
    use_fog_wz: true
    covariance.device_convention: unknown
```

Pass it using `params_file:=/absolute/path/anello.yaml`. ROS parameter files must quote string values such as `'OFF'` themselves; launch argument substitutions preserve their string type automatically. An external component container needs a multithreaded executor to get independent receive/configuration/transmit execution.

## Original audit disposition

| Finding | Implementation and regression evidence |
|---|---|
| 1 — numeric/schema validation | Shared complete-frame decoder, exact layouts/enums/ranges, locale-independent checked numbers, covariance PSD validation; C++ and live ROS malformed-input regressions |
| 2 — invalid INS position | Per-sample publication gate; live tests before anchoring and after fix loss, including TF |
| 3 — stale cached measurements | Sample acquisition/arrival times and reset invalidation; unit/live tests, including APIM1 |
| 4 — output point/frame mismatch | Correct sensor-point default, enforced frame contract, no silent lever-arm relabelling; invalid-configuration regression; physical output-centre verification remains installation work |
| 5 — REP-105 continuity | Explicit global local-Cartesian frame/topic, no TF by default, `odom` parent rejected, integration guidance |
| 6 — config-channel recovery | Incremental AUTO/fixed-path recovery; stable-symlink and fragmented AUTO-probe pseudoterminal regressions, explicit OFF test |
| 7 — stream/FOG health | Expected-stream watchdogs, unavailable states, GPS fix gating, APIM1 health, zero/stuck/saturation handling; synthetic and live tests |
| 8 — launch types | Typed mappings in shared Python implementation; installed Python/XML tests with OFF and numeric credentials |
| 9 — configuration exposure | Driver/NTRIP parameter files and typed overrides, with file-precedence regression |
| 10 — clock discontinuities | Integral host epoch, device/host/source-reset detection, simulation pause/reset policy; unit/live tests |
| 11 — NTRIP recovery | Worker, persistent startup retries, monotonic valid-frame deadlines, fresh GGA resend and bounded shutdown; caster/worker/live-launch tests |
| 12 — NTRIP framing | Incremental exact status/header handling, ICY payload preservation, bounded strict chunks/EOF; fragmented caster regressions |
| 13 — Humble API | Version-compatible service QoS call and Humble CI job; local verification remains Lyrical only |
| 14 — Python test discovery | `colcon.pkg` pytest selection, extras metadata, explicit CI command and required-suite checks; ordinary colcon discovery verified locally |
| 15 — local projection | WGS84 ECEF/ENU helper and rotations, curvature/altitude/antimeridian/Jacobian tests |
| 16 — covariance source/rate | MEMS/FOG defaults, rate scaling, explicit overrides, invalid-parameter and published-message regressions |
| 17 — transport writes | Bounded partial-write handling, fd ownership, callback separation, send results/counters; backpressure, completion, descriptor-zero, and reconnect tests |

Additional fixes from the continued review include overflow-safe GGA conversion, correct dilution/height metadata, saturated-rate availability, distinct diagnostic instance names, retained batched INS output, validated command replies, parameter range checks before narrowing, rejection of undersized RTCM corrections and embedded NMEA line endings, and expiry of corrections delayed in the publication queue. Obsolete unvalidated ASCII decoders and the C++11 makefile were removed; protocol records, clock/sample state, navigation math, publisher types, and transport headers are separated from the public node factory.

The command/recovery review also caught a CRLF-boundary mismatch in the legacy checksum helper. A shared bounded checker now validates both stripped telemetry frames and complete device replies. AUTO probing skips corrupt lines before a valid fragmented reply. A live UART service regression verifies reply matching/checksums and continued IMU reception during the command wait.

APAHRS/subtype 8 now has a native custom message. Its relative yaw is deliberately not promoted into absolute ENU attitude. Existing unused heading service definitions are retained with deprecation notes for source compatibility.

## Validation boundaries

Software tests exercise deterministic frames, pseudo-terminals, local sockets, installed ROS nodes, and launch files. They cannot establish firmware APCOV units/basis, mounting/output-centre configuration, six-axis sign conventions, receiver confidence definitions, or field accuracy. Keep device covariance unverified until those are established. Follow the [hardware checklist](hardware_validation_checklist.md) and [integration guide](integration_guide.md).

The implementation targets Linux little-endian ROS platforms. The binary protocol uses packed little-endian device structures; unsupported host endianness fails compilation instead of publishing misdecoded values. CI is configured for Humble/Jazzy/Kilted/Lyrical; this workspace's actual local results are from Lyrical. No hardware configuration, external deployment, or firmware update is performed by this release.

The [EVK device-input review](evk_device_inputs.md) explains the later outbound hardening. Its traffic and command limits are driver policy, not manufacturer-certified firmware limits.
