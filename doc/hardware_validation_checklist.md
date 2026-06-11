# Hardware Validation Checklist

Run this checklist against a real ANELLO unit after building a new
driver version (and once after merging driver changes), before trusting
the outputs in the field. The driver's automated tests cover build and
lint only — everything here needs hardware.

Items are ordered so each stage validates the prerequisites of the
next: bench (powered, stationary, indoors ok) → static outdoor →
drive. Each item lists the command, the expected result, and what to do
on failure.

## 1. Bench checks (unit powered, stationary)

### 1.1 Link and topics

```bash
ros2 topic list | grep -E "anello|imu|gps|ins"
ros2 topic hz /anello/imu_raw      # ASCII/RTCM IMU stream
ros2 topic hz /anello/ins          # INS solution
```

- [ ] All topics present: `anello/{imu_raw,ins,gps,gps2,hdg,cov,health}`,
      `imu/data`, `imu/data_raw`, `gps/fix`, `ins/fix`.
- [ ] `imu_raw` rate matches the configured `odr` (±2%); `ins` is
      ~100 Hz on EVK/Ground INS.
- **If not:** serial — check port/baud; Ethernet — work through the
  [Ethernet Setup Guide](ethernet_setup_guide.md) troubleshooting table
  (`tcpdump` first).

### 1.2 Config channel round-trip

```bash
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
    "{command: 'APPNG'}"
```

- [ ] Response is `#APPNG,0*54`.
- **If timeout:** config port not detected (serial `uart_config_port`)
  or `rport2`/`local_config_port` mismatch (Ethernet).

### 1.3 Accelerometer sign convention (CRITICAL, once per unit/firmware)

```bash
ros2 topic echo /imu/data --once --field linear_acceleration
```

With the unit stationary and level:

- [ ] `z` reads approximately **+9.8** m/s² and `x`,`y` are near 0.
- **If z reads −9.8:** the firmware reports the opposite at-rest sign
  convention — relaunch with `flip_accel_sign:=true` and record that in
  your platform config. Getting this wrong inverts gravity for every
  downstream consumer (the public ANELLO docs do not state the
  convention; this check is the only arbiter).

### 1.4 Gyro channels at rest

```bash
ros2 topic echo /anello/imu_raw --once
```

- [ ] `wz` (MEMS) and `wz_fog` (optical) are both small (≪ 0.1 deg/s)
      and not byte-identical constants over repeated echoes.
- [ ] `imu/data_raw` `orientation_covariance[0]` is `-1` (orientation
      unreported on the raw topic, REP-145).
- [ ] If the FOG is disabled (`fog off`): `wz_fog` is exactly 0 — then
      launch with `use_fog_wz:=false`.

### 1.5 Health monitor at rest

```bash
ros2 topic echo /anello/health
```

Watch for ~2 minutes:

- [ ] `gyro_health_flag` stays 0 (the discrepancy gate is ~50σ above
      sensor noise — any flag at rest indicates a real channel problem
      or a stuck/dead output).
- [ ] No repeated `Checksum fail` warnings in the driver log
      (occasional ones on startup are normal while mid-message).

### 1.6 Covariance plumbing

```bash
ros2 topic echo /anello/cov --once          # requires APCOV-capable firmware
ros2 topic echo /ins/fix --once --field position_covariance
```

- [ ] Driver log shows **no** "APCOV ... suspiciously small for m^2"
      warning. If it appears, the firmware is reporting deg²-scale
      values and `ins/fix` covariance is misscaled — report to ANELLO
      support and do not feed `ins/fix` covariance to a fuser until
      resolved.
- [ ] `ins/fix` covariance diagonal is plausible m² (e.g. 1e-4–10
      depending on fix quality), not ~1e-10.

### 1.7 Timestamping

```bash
ros2 topic delay /anello/imu_raw     # with default timestamp_source:=arrival
```

- [ ] Delay is small and positive (ms-scale).
- [ ] If using `timestamp_source:=mcu`: stamps are strictly increasing,
      inter-message dt is uniform (plot `header.stamp` deltas — jitter
      should collapse vs `arrival` mode), and stamps stay within a few
      ms of wall time after the ~100-message warm-up.
- [ ] Power-cycle the unit while the driver runs: stamps must recover
      (translator resets on MCU time going backwards), not jump.

## 2. Static outdoor checks (clear sky view)

### 2.1 GNSS fix and raw topic semantics

```bash
ros2 topic echo /gps/fix --once
ros2 topic echo /anello/gps --once
```

- [ ] `gps/fix.status.status` is `-1` (NO_FIX) until the receiver
      reports a 2D/3D fix, then `0` (FIX); with RTK corrections, `2`
      (GBAS_FIX).
- [ ] Position covariance diagonal ≈ `hacc²`/`vacc²` from
      `anello/gps`, and `position_covariance_type` is 2
      (DIAGONAL_KNOWN).
- [ ] `gps/fix` rate is ~4 Hz (it is the **raw** GNSS stream; the
      fused position is on `ins/fix` at the INS rate).

### 2.2 NTRIP / RTK (if used)

```bash
ros2 topic hz /ntrip_client/rtcm
ros2 topic echo /anello/gps --once    # watch rtk_fix_status
```

- [ ] RTCM messages flow after connect; no chunked-framing warnings
      with an NTRIP v2 caster (`ntrip_version: 'Ntrip/2.0'`).
- [ ] GGA upload is accepted: a VRS/NEAR caster starts streaming within
      seconds (GGA is sent on first fix, then every
      `nmea_min_interval_seconds`, default 10 s).
- [ ] `rtk_fix_status` progresses 0 → 1 (float) → 2 (fixed) within a
      few minutes on a clean sky.

### 2.3 Dual-antenna heading (if ANT2 fitted)

```bash
ros2 topic echo /anello/hdg --once
```

- [ ] `rel_pos_length` matches the measured antenna separation within
      a few cm (set the same value as the driver's `heading_baseline`).
- [ ] `gnss_fix_ok`, `rel_pos_valid`, `rel_pos_heading_valid` all 1 and
      `carrier_solution` is 2 once RTK-fixed.
- [ ] `heading_health_flag` on `anello/health` stays 0 while static.

### 2.4 ZUPT calibration (firmware ≥ v1.3.24, once per installation)

- [ ] With the vehicle completely undisturbed for 2 minutes, the
      APINS `zupt` flag reads 1 throughout (`ros2 topic echo
      /anello/ins --field zupt`).

## 3. Drive tests

### 3.1 INS initialization

Drive forward at > 2 m/s for ~30 s with sky view.

- [ ] `anello/ins` `ins_status` progresses 0 → 1 → 2 (and 3/4 with
      RTK). Values 8–10 mean GPS aiding is off — fix the unit config.
- [ ] `imu/data` orientation yaw matches reality: pointing geographic
      **east → yaw ≈ 0**, **north → yaw ≈ +π/2** (ENU, east-zero —
      what `navsat_transform` assumes with `yaw_offset: 0`).

### 3.2 Health monitor under dynamics

Drive 5–10 minutes including turns, stops, and slow maneuvering
(< 2 m/s):

- [ ] `heading_health_flag` stays 0 during slow maneuvering (the GPS
      heading comparison is speed-gated at 2 m/s — false streaks here
      would indicate a regression).
- [ ] `gyro_health_flag` stays 0 through aggressive turns (the
      discrepancy check suspends above 180 deg/s where the FOG
      saturates).
- [ ] `zupt` flag: 1 at every full stop, 0 while moving, and reported
      velocity ≈ 0 whenever it reads 1.

### 3.3 Fused vs raw position

- [ ] `ins/fix` tracks `gps/fix` within the reported accuracies in open
      sky, and keeps dead-reckoning smoothly through a brief antenna
      occlusion (underpass, tree cover) while `gps/fix` degrades.

### 3.4 Odometer and reverse (if wheel speed is wired)

```bash
ros2 topic pub -r 10 /anello/odo anello_interfaces/msg/APODO \
    "{odo_speed: <signed speed m/s>}"
```

- [ ] `anello/imu_raw` `odometer_speed` echoes the input (m/s).
- [ ] Reverse at low speed: INS position moves backwards (signed
      input, or dual-antenna heading, is required — unsigned input
      corrupts heading in reverse).

### 3.5 Stack integration (whichever topology you picked)

- [ ] **Trust-the-INS:** downstream consumes `ins/fix` + driver TF;
      driver launched with `publish_tf:=true`; no second EKF runs.
- [ ] **Re-fuse:** `ekf_node` + `navsat_transform_node` consume
      `imu/data` + `gps/fix`; driver launched with `publish_tf:=false`;
      `ins/fix` is NOT fused; `/odometry/filtered` is smooth and
      `navsat_transform` initializes its datum after the first valid
      fix.

## 4. After-upgrade regression notes

When upgrading from a pre-v3.1 driver, re-check the consumers affected
by intentional behavior changes:

- [ ] Anything that consumed the fused position from `gps/fix` now
      reads `ins/fix`.
- [ ] Downstream packages using `anello_interfaces/APIMU` are rebuilt
      (the message gained `t_sync`).
- [ ] `imu/data` z-rate now defaults to the optical gyro
      (`use_fog_wz`); set `false` for `fog off` units.
- [ ] NTRIP VRS casters still stream with the 10 s GGA interval (set
      `nmea_min_interval_seconds` lower if your caster requires faster
      updates).
