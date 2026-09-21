# Read-only device configuration export

`anello_config export` captures the device's identity, unit RAM settings,
unit flash settings, vehicle flash settings, and the running ROS driver's
parameters in a JSON snapshot. It sends queries through `anello/send_cmd`;
the running driver owns the device connection. The command does not write
settings, calibrate, restart, or flash the unit.

Build and source the workspace, then start the driver with its normal
connection parameters and `command_mode=read_only`. For serial operation,
enable the distinct configuration port; `uart_config_port=OFF` cannot serve
queries. For Ethernet, the device's address and host configuration port must
already match the driver settings. See the [connection examples](../readme.md#launch).

```bash
ros2 run anello_ros_driver anello_config export --output anello-device.json
```

For a driver in a namespace or with a different node name:

```bash
ros2 run anello_ros_driver anello_config export \
  --node /robot/anello_ros_driver --output robot-anello-device.json
```

The default command service is `<node namespace>/anello/send_cmd`. Supply
`--service /fully/qualified/service` if it was remapped. The exporter checks
that the selected node advertises that command service before querying it,
so host parameters are associated with the intended driver. `--timeout`
sets the wall-clock timeout per ROS service call (default 3 seconds); it does
not change the driver's approximately 500 ms device-response timeout.

The destination must be a new file in an existing directory. Snapshot creation
is atomic and refuses to replace an existing file. A normal export takes about
8 seconds plus ROS discovery time. Queries run sequentially, with at least
0.55 seconds between a response and the next command. Other command clients
can still consume the shared driver rate budget; resulting errors are recorded.

## Captured evidence

The top-level format identifier is `anello_device_snapshot_v1`.

| Field | Contents |
|---|---|
| `identity` | `APPID`, `APSER`, `APVER`, `APIHW`, `APFHW`, `APFSN` replies |
| `configuration.unit_ram` | Read-all `APCFG,r` reply |
| `configuration.unit_flash` | Read-all `APCFG,R` reply |
| `configuration.vehicle_flash` | Read-all `APVEH,R` reply |
| `ram_controls` | Separate `APCFG,r,azupt` and `APCFG,r,ahdg` probes, matching the vendor UI's handling of RAM-only AHRS controls |
| `identity_after` | Repeated product, serial, and firmware version queries to detect an identity change during capture |
| `host_parameters` | Typed parameters from the selected running driver's ROS parameter services |
| `driver_node`, `command_service` | Fully qualified ROS endpoints selected for capture |
| `started_utc`, `finished_utc` | Overall capture times |
| `exporter_sha256` | SHA-256 of the exporter script used for this capture |
| `complete`, `identity_consistent` | Overall query success and before/after identity comparison |

Each device query retains the command, exact service response (including CRLF
escaped in JSON), start/end timestamps, elapsed time, validation status, and
either an identity value, configuration key/value map, or error details.
Device values remain strings: serial leading zeros, signed alignment strings,
numeric formatting, unknown keys, and empty values are preserved. Unit RAM and
flash are kept separate even when they contain the same keys.

The exporter verifies framing, checksum, response identifier, key/value
structure, duplicate keys, and requested keys for the two targeted RAM probes.
It does not infer units or convert stored values into ROS frame conventions.
For example, `ahdg` retains its wire value in millidegrees; it is not silently
converted to degrees. Consult the [vendor-tool mapping](user_tool_capability_comparison.md)
when interpreting device settings.

The host parameters include `driver_build_revision` and
`driver_build_source_sha256`. These are supplied by the running driver and
ignore parameter-file overrides. Revision is the Git commit recorded when
CMake configured the build, with `-dirty` when there were local changes, or
`unknown` without Git metadata. The source fingerprint hashes the sorted
relative paths and SHA-256 hashes of C++ sources/headers, `CMakeLists.txt`,
`package.xml`, and the build-info template. It identifies those configured
inputs, not a reproducible binary or dependency/toolchain fingerprint.
Source edits trigger CMake reconfiguration; reconfigure after changing only
Git metadata if an updated commit label is needed.

## Partial captures and limits

Exit codes are:

| Code | Meaning |
|---|---|
| `0` | All requested device queries and host-parameter capture succeeded, and identity matched before/after |
| `1` | A snapshot was written but is partial or identity could not be confirmed |
| `2` | Invalid arguments, unavailable Python dependencies, or another failure prevented a successful export operation |

`complete` is deliberately strict. An older unit rejecting an optional hardware
or AHRS query still yields a useful snapshot with `complete=false`. Successful
sections remain available. Status `unsupported` records device `APERR` codes
6, 7, or 11; other codes are `device_error`. `driver_error`, `transport_error`,
and `invalid_reply` distinguish host rejection/timeouts from malformed service
responses. Inspect these statuses before using the snapshot as evidence.

The driver now admits the exact keyless read-all forms in read-only mode and
accepts framed configuration replies up to **4096 bytes**, including framing
and CRLF. Larger UDP datagrams are discarded as a whole and produce an explicit
service error. Serial replies can arrive in fragments but use the same maximum
line length. The existing command-body limit and rate policy remain in force.
`APIHW` is also admitted as a no-argument identity query.

Raw evidence is the exact response returned by the service. Bad-checksum or
unrelated frames filtered by the driver, and oversized datagrams discarded by
the receiver, cannot appear as raw device frames in the export; their outcome
is the service error. This is not a packet capture.

The snapshot covers the requested query set over an interval, not an atomic
device-memory read. Before/after identity comparison does not prove that
configuration was unchanged during capture, or that a device with the same
identity did not reboot. The protocol has no transaction IDs: concurrent or
unsolicited responses with the same identifier cannot always be distinguished.
Readback of flash establishes stored values, not activation after a restart.

Keep this snapshot alongside the host launch YAML, recording, and firmware
archive. It is evidence for configuration review; it is not an apply profile.
Write/apply, calibration actions, and reconnection after changing device baud
or network settings remain later work in the
[configuration implementation sequence](user_tool_capability_comparison.md#proposed-implementation-sequence).
