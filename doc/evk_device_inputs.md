# EVK device input protection

The host driver cannot prove that the EVK firmware never crashes. The September 2026 follow-up review covers bytes and control operations the driver can send. Firmware, power, USB hardware, and receiver behavior still require the actual unit and firmware version. All fault injection in this review used simulated serial ports and localhost ROS.

## Outbound paths

| Path | Behavior |
|---|---|
| Startup/recovery | Data discovery listens for validated telemetry. AUTO config probing sends only `APPNG`, at most once per 500 ms; it does not reset the unit or change configuration. Data-port reopen attempts are also spaced by at least 500 ms. |
| RTCM subscription | Validate every complete envelope, reserved bits, payload length, and CRC before sending any part of a bundle. Reject ASCII, partial/corrupt frames, trailing junk, reserved message ID zero, and ANELLO output telemetry ID 4058. |
| RTCM transport | Require a confirmed UART data-port generation. Send one frame per write/datagram, at most 1029 bytes, avoiding bundle-induced fragmentation at ordinary Ethernet MTUs. A failed write stops the bundle; it is not replayed. |
| RTCM admission | Default 8192 bytes/s with a 4096-byte burst, additionally capped at baud/20 on UART. Default 100 frames/s with a 16-frame burst limits small-frame floods. Oversized bursts/excess input are dropped without delayed replay. |
| Odometer subscription | Reject non-finite values and speeds beyond the configured absolute limit (default 100 m/s). Format with a fixed decimal point and send at most the configured rate (default 50 Hz), without a burst or delayed replay. Firmware odometer units must be m/s. |
| Command service | Default `read_only` permits bounded queries and echo; configuration, reset, and unknown commands are blocked. All modes reject controls/framing injection and bodies longer than 128 bytes. Maximum command rate is two per second. |
| Explicit reset | Only `command_mode=unrestricted` permits `APRST,0`. A successful transport write returns `SENT`, not an acknowledged reset; no reply is expected and the driver never automatically retries it. |

Read-only query identifiers are `APPNG`, `APVER`, `APSER`, `APSTA`, `APPID`, `APFSN`, and `APFHW`, with no arguments; `APECH` accepts printable echo text. `APCFG`/`APVEH` permit only `r`/`R` operations and nonempty parameter names. Unsupported firmware queries may still return APERR.

The byte/frame limits are conservative, configurable host admission policies, not vendor-certified device capacity. Byte rates can be configured up to 65536/s, frame rates up to 1000/s, odometer rate up to 100 Hz, and absolute odometer speed up to 1000 m/s. Raising limits or enabling `unrestricted` changes the protection contract. A CRC-valid RTCM envelope is not proof that every message's payload, correction reference frame, or firmware support is correct.

## Serial ownership and control lines

The driver claims an advisory lock and Linux exclusive-open mode before changing termios or flushing. This prevents the data/config channels, aliases, and other cooperating driver processes from consuming each other's bytes or changing each other's baud rate. Writers on one interface serialize complete write attempts within the same 100 ms deadline; stale generations cannot send to a replacement device. Cleanup releases ownership even when initialization fails.

Exclusive-open mode does not evict a noncooperating program that already has the TTY open, and privileged programs may bypass it. Stop other serial applications before starting the driver. AUTO cannot establish which ports belong together when several units are attached; choose the explicit data/config paths for that EVK. These limits follow the [Linux exclusive-TTY API](https://man7.org/linux/man-pages/man2/TIOCEXCL.2const.html).

The termios setup clears inherited `HUPCL`; the driver never explicitly pulses DTR/RTS or sends a serial break. It also disables `IXON`/`IXOFF`/`IXANY`, so inherited software flow control cannot inject XON/XOFF into the device input. These settings remove the inherited behavior described by [termios](https://man7.org/linux/man-pages/man3/termios.3.html), but do not establish the EVK's electrical wiring or guarantee that the adapter/kernel never changes a modem line when opening a port.

## EVK startup and bench follow-up

For an EVK using its factory serial settings, supply 921600 baud and its actual stable port identities:

```bash
ros2 launch anello_ros_driver anello_driver.launch.py \
  baud_rate:=921600 \
  uart_data_port:=/dev/serial/by-id/EVK_DATA \
  uart_config_port:=/dev/serial/by-id/EVK_CONFIG \
  command_mode:=read_only
```

ANELLO documents the data/config port roles, factory baud, odometer format, correction input, and reset-without-response behavior in its [communication manual](https://docs-a1.readthedocs.io/en/latest/communication_messaging.html). The paths above are placeholders for the identified unit.

Monitor `device_input_rejections_total`, `device_input_rate_drops_total`, and `transmission_failures_total` in `/diagnostics`. Rejection/rate counters are lifetime totals; they distinguish rejected input from transport failures but do not prove firmware execution or acknowledgement. Existing decoded-stream ages and MCU/clock resets help distinguish a silent ROS connection from a device reboot, though reset detection also includes host-clock changes.

Record firmware version, input rates, physical connection, power conditions, MCU uptime, and logs for a long run using valid corrections and odometer data. Link loss or a timed-out write can still truncate a physical transmission. Firmware must tolerate normal fragmentation/link loss; synthetic host tests cannot demonstrate that. Follow the [hardware checklist](hardware_validation_checklist.md) before asserting stability of the unit itself.
