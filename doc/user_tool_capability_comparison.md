# ANELLO user tool and ROS device configuration

The ROS driver already transports configuration commands, but it does not
provide the vendor tool's discovery, editing, export, or restart workflow.
ROS parameter YAML configures the host driver; it does not apply settings to
the ANELLO unit. A useful next step is a structured configuration interface
over `anello/send_cmd`, with explicit RAM/flash selection and readback.
This document records the baseline comparison and proposed sequence. The
first discovery/export phase is now implemented: see the
[read-only export guide](device_configuration_export.md). The tables below
describe the baseline driver commit cited here; apply and action workflows
remain proposed.

## Evidence and scope

Reviewed on 2026-09-21 against:

- ANELLO [`user_tool` commit `cd5e069b34daad183e251ac88b5e716971e38c3b`](https://github.com/Anello-Photonics/user_tool/tree/cd5e069b34daad183e251ac88b5e716971e38c3b).
- This driver's `ros2_develop_cam` commit
  `bb857c0d8d98012dcc6213e692da5138041917d9`.
- The workspace's archived `IMU-A1_v1.3.53` HEX, source SHA-256
  `6accd6d205e1389cb1ddf8feeabac830e6f86a699a54790781d05f4f939e2d2e`.
  Its readout is under `firmware_analysis/IMU-A1_v1.3.53/` in the parent
  workspace repository, rather than this driver submodule.

The upstream sources establish host-tool behavior. They do not establish that
every field works on the archived image or a connected unit. The firmware
strings contain `APCFG`, `APVEH`, `APINI`, `APUPD`, and configuration keys;
device identity, supported keys, and active values still require live replies.
No device was queried, configured, reset, or flashed during this comparison.

Source references used below:

- [Vendor UI and configuration workflows](https://github.com/Anello-Photonics/user_tool/blob/cd5e069b34daad183e251ac88b5e716971e38c3b/user_program.py):
  `set_cfg`, `read_all_configs`, `vehicle_configure`, `save_configurations`,
  `initialize_and_update_menu`, `reset`, and the `form_*_string_prompt` helpers.
- [Vendor field names and menu choices](https://github.com/Anello-Photonics/user_tool/blob/cd5e069b34daad183e251ac88b5e716971e38c3b/board_tools/user_program_config.py).
- [Vendor board API](https://github.com/Anello-Photonics/user_tool/blob/cd5e069b34daad183e251ac88b5e716971e38c3b/board_tools/src/tools/board.py):
  `get_cfg`, `get_cfg_flash`, `set_cfg`, `set_cfg_flash`, `get_veh_flash`,
  `set_veh_flash`, `set_veh_terminal_interface`, and `reset_with_waits`.
- [Wire constants and vehicle fields](https://github.com/Anello-Photonics/user_tool/blob/cd5e069b34daad183e251ac88b5e716971e38c3b/board_tools/src/tools/class_configs/readable_scheme_config.py)
  and [configuration framing/parser](https://github.com/Anello-Photonics/user_tool/blob/cd5e069b34daad183e251ac88b5e716971e38c3b/board_tools/src/tools/readable_scheme.py).
- [Vendor configuration export](https://github.com/Anello-Photonics/user_tool/blob/cd5e069b34daad183e251ac88b5e716971e38c3b/board_tools/log_config.py):
  `log_board_config` writes a host-side text file.
- ROS [command admission](../anello_node/src/anello_ros_driver/device_input.h),
  [service and parameters](../anello_node/src/anello_ros_driver/main_anello_ros_driver.cpp),
  [UART](../anello_node/src/anello_ros_driver/comm/serial_interface.cpp), and
  [UDP](../anello_node/src/anello_ros_driver/comm/ethernet_interface.cpp).

## Capability comparison

| Capability | Vendor tool | Baseline ROS driver | Work needed for equivalent configuration support |
|---|---|---|---|
| Product, serial, version, hardware identity | Board queries including `APPID`, `APSER`, `APVER`, `APIHW`, `APFHW`, `APFSN` | Most available as raw replies through `send_cmd`; `APIHW` is outside the default read-only allowlist | Structured identity and raw-response capture; assess adding the documented `APIHW` query |
| Discover supported unit settings | Reads all flash configuration, then probes RAM-only controls; UI offers fields returned by the device | Keyed `APCFG,r/R,...` reads allowed; keyless read-all rejected in read-only mode | Admit exact read-all forms and parse discovered keys without assuming a model-wide fixed list |
| Read/write unit settings | RAM and flash API; UI writes most fields to flash, `azupt`/`ahdg` to RAM | Raw keyed reads; writes require `command_mode=unrestricted` | Named storage selection, value validation, explicit writes, and readback |
| Vehicle geometry and tuning | Flash reads/writes for antenna, axle, output, odometer, baseline, wheel, and calibration settings | Raw `APVEH` commands with the same command-policy restrictions | Structured values with units and a mapping to ROS frame/baseline assumptions |
| Baseline calibration | Auto, derive from lever arms, cancel before manual baseline; reads progress | Raw commands only | Separate action/status workflow; distinguish accepted command from completed calibration |
| ZUPT calibration | Start/reset controls and visible calibration status/data | Raw commands only | Explicit action and status reporting |
| Save configuration | Exports unit flash settings, firmware version, and vehicle settings to host text | No structured device snapshot; `ros2 param dump` captures host parameters only | Portable snapshot with identity, RAM/flash distinction, raw replies, errors, and timestamps |
| Restart and changed baud rates | Reads stored data/config bauds, resets, changes host bauds, and reconnects | `APRST,0` reports transmission; ports reopen with the original ROS settings | Reconnection plan and post-restart identity/configuration verification |
| Position/heading initialization and heading update | Builds `APINI`/`APUPD` with values and uncertainty | Raw service can carry them in unrestricted mode; `InitHeading`/`UpdHeading` types have no advertised service | Typed inputs and parsed device result if included in a later phase |
| NTRIP and odometer input | Caster client and `APODO` input APIs/demos | NTRIP node, RTCM forwarding, and `/anello/odo` already exist | Verify device input-channel selection and odometer units alongside host settings |
| Logging/monitoring | Raw logs, export, plots, and GUI | ROS topics and diagnostics; recordings can use ROS tools | Configuration snapshot should accompany recordings |
| Firmware update | Bootloader workflow and HEX selection | No firmware flashing interface; workspace readout script analyzes an existing HEX | Treat firmware maintenance as a separate workflow from device configuration |

## Configuration wire contract

Case selects the storage operation. These are command bodies; the ROS service
adds `#`, checksum, and CRLF.

| Operation | Unit command | Vehicle command in vendor board API |
|---|---|---|
| Read RAM | `APCFG,r,odr,mfm` | No corresponding public RAM helper |
| Write RAM | `APCFG,w,odr,100` | No corresponding public RAM helper |
| Read flash | `APCFG,R,odr,mfm` | `APVEH,R,g1x,g1y,g1z` |
| Write flash | `APCFG,W,odr,100` | `APVEH,W,g1x,0.5,g1y,0,g1z,-0.3` |
| Read all | `APCFG,r` / `APCFG,R` | `APVEH,R` |

The read-all bodies have **no trailing comma**. The baseline ROS read-only
allowlist requires at least one key, so all three are blocked. The unrestricted
mode can transmit them, but that is not a suitable prerequisite for discovery.

The vendor response parser expects name/value pairs after `APCFG` or `APVEH`,
without the request's operation letter: for example,
`APCFG,odr,100,mfm,1` before framing/checksum. `APERR` is a device rejection,
even when its frame passes checksum validation. A future structured client
must also distinguish it from driver-generated `ERROR: ...` strings.

The current service admits bodies of 5–128 printable bytes and at most two
commands per second, with a one-command burst. It waits about 500 ms for a
matching checksum-valid response. It matches the message identifier, not the
requested key/value set, and the device protocol has no transaction ID.
Clients must serialize and pace requests, check returned keys, and report
ambiguous or incomplete results rather than treating a matching prefix as
configuration success.

The baseline also has a transport limit for bulk discovery:
its service passes a 511-byte buffer limit to the UDP reader, which reserves
one byte for termination. A configuration response exceeding **510 bytes in
one UDP datagram is discarded**, not assembled across reads. Serial reads can
accumulate fragments, subject to the service's bounded response buffer. Bulk
read support needs a bounded whole-datagram receive path and tests for realistic
full configuration responses as well as oversized input. The implemented
export phase resolves this with a 4096-byte reply limit and explicit oversize
errors; see the export guide for the current behavior.

## Device settings that must agree with ROS

The following mappings describe the inspected code, not automatic
synchronization. Driver parameters are read at startup and declared read-only.

| Device fields | ROS relationship |
|---|---|
| `odr`, `lpa`, `lpw`, `lpo` | Device output rate and filters. `imu_output_rate_hz` is only a legacy host hint; it changes none of these. Configure stream expectations/age limits for the actual output and retain measured covariance for that rate/filter setup. |
| `mfm` | Vendor labels `0` binary, `1` ASCII, `4` RTCM. The ROS decoder implements ASCII and RTCM; proprietary binary `0` is not an equivalent supported selection. |
| `gps1`, `gps2`, `ahrs`, `nmea`, `nmea_rate` | Device output/features. Set `expected_streams` from observed supported messages. Its entries do not enable device output. Vendor NMEA bits are 0=GGA, 1=GSA, 2=RMC. |
| `fog` | Device FOG control. `use_fog_wz` selects the ROS gyro source; it does not enable the optical gyro in firmware. |
| `orn`, `aln` | Device orientation and alignment. `frame_id.*` names frames and does not configure mounting. Confirm the resulting axes and avoid applying mounting correction twice. |
| `g1x/y/z`, `g2x/y/z`, `cnx/y/z`, `ocx/y/z`, `wsx/y/z` | Antenna, rear-axle, output, and odometer lever arms. ROS frame names/TF do not write these values or move the firmware output point. Preserve the measured units and verified frame convention with each setting. |
| `bsl`, `bcal` | Device antenna baseline and calibration. `heading_baseline` is the host health-check reference; setting it does not configure or calibrate the device. |
| `odo`, `tic`, `rad` | Device odometer units, ticks/revolution, and wheel radius. `/anello/odo` supplies speed in m/s; the driver has no conversion for firmware configured in mph, kph, or fps. |
| `nhc`, `dir_det`, `rmin`, `zcal` and ZUPT calibration values | Firmware navigation/calibration settings; no matching driver parameter or typed configuration workflow. |
| `bau`, `bau_input` | Separate data/config bauds in the vendor tool. ROS uses one `baud_rate` for both ports, accepting 115200, 230400, 460800, or 921600. The vendor also lists 19200/57600 and restricts its IMU/GNSS RS-232 menu to at most 230400. Port `AUTO` discovery in ROS does not scan baud rates. |
| `eth`, `uart`, `dhcp`, `lip`, `rip`, `rport1/2/3` | Device networking/output controls. ROS `remote_ip` is the unit IP (`lip` for static addressing); the host interface IP must match device `rip`. `local_data_port`, `local_config_port`, `local_odometer_port` must match `rport1/2/3`. Unit destination ports remain 1/2/3. `com_type` only chooses the host transport. |
| `ntrip` | Vendor labels `0` off, `1` serial, `2` Ethernet as the correction input channel. ROS caster settings configure the host NTRIP client, not that device selector. Verify the selected channel on the actual firmware. |
| `sync`, `ptp` | Device timing controls. `timestamp_source=mcu` translates MCU timestamps to host time; selecting it does not configure device sync/PTP or establish PTP synchronization. |
| `azupt`, `ahdg` | RAM-only AHRS controls in the vendor UI. The tool encodes `ahdg` in integer millidegrees, not the decimal degrees displayed to users. |
| `min` | Device configuration-print interval; no corresponding host parameter. Unsolicited configuration responses matter when correlating command replies. |

Some settings need more than a generic `key=value` serializer:

- `aln` is three signed degree values concatenated without commas; the vendor
  formats each to six decimal places, e.g. `+0.000000-1.500000+90.000000`.
- Before setting odometer units, the vendor UI writes `odo=on`, then the
  selected unit. A ROS workflow needs to preserve that sequence and use m/s
  when forwarding the current `APODO` input.
- Manual baseline entry first sends `APVEH,W,bcal,99`, then writes `bsl`.
  Baseline auto/from-levers use `bcal=1/2`; `bcal=0` indicates completion in
  the vendor status table. ZUPT uses `zcal=1` to start and `zcal=3` to reset.
  These are actions with state transitions, not ordinary values to replay
  from a snapshot.
- Readback of stored flash values establishes storage contents. Whether a
  setting is active immediately or after restart must be established for the
  specific firmware. Baud/network changes need a reconnection procedure.

## Read-only inspection available today

Enable the distinct UART config port or use the configured Ethernet endpoint.
Leave `command_mode=read_only`. Run each call after the preceding one finishes,
allowing at least 0.5 seconds between transmissions; another command client
can still consume the driver's rate budget. A namespace changes the service
path shown here.

```bash
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
  "{command: 'APVER'}"
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
  "{command: 'APCFG,R,odr,mfm,odo'}"
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
  "{command: 'APCFG,r,odr,mfm,odo'}"
ros2 service call /anello/send_cmd anello_interfaces/srv/CmdAndRsp \
  "{command: 'APVEH,R,bsl,bcal'}"
```

These are keyed inspection examples, not a complete dump. Unsupported fields
may return `APERR`; preserve that reply instead of substituting defaults.

## Proposed implementation sequence

1. **Discovery and export (implemented):** support documented read-all queries under the
   read-only policy, resolve UDP reply sizing, then capture identity and
   supported unit RAM, unit flash, and vehicle flash settings. Include raw
   checksum-verified replies, timestamps, unsupported/error results, driver
   revision, and host ROS parameters. Never label a partial export complete.
2. **Reviewable configuration:** define a device profile separate from
   `ros__parameters`. Produce a proposed-change list using device discovery,
   with storage, units, expected before/after values, and affected host
   settings. Keep snapshots separate from apply profiles so calibration
   status cannot be replayed as a command.
3. **Explicit apply and verification:** validate the full profile first,
   serialize commands within the driver budget, apply only requested changes,
   parse `APERR`, and read back the appropriate storage. Record partial
   progress on failure; do not blindly retry writes or claim rollback.
4. **Actions and reconnection:** add calibration/status and restart as explicit
   operations. Handle separate baud rates and network endpoint changes before
   claiming those edits can be completed through ROS. Confirm identity and
   active configuration after reconnecting.

The first deliverable should be discovery/export: it directly supports a
documentable device setup and the firmware archive. An apply interface can
then build on that evidence. Firmware flashing, GUI replication, and implicit
startup writes are outside this proposed first implementation.

Software validation for a later implementation should use simulated serial
and UDP devices for case-sensitive storage modes, full replies over 510 bytes,
invalid checksums, `APERR`, wrong/missing keys, rate limits, partial writes, and
reconnection. Physical validation must retain `APVER`/identity replies and
before/after settings on the actual unit; source strings alone cannot supply
that evidence.
