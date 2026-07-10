# px4-param-audit

PX4 parameter audit and explicit write tool for multirotors.

The first target is Pixhawk/Holybro over MAVLink serial. Starling/VOXL support is
intentionally out of scope for the MVP.

## Current MVP

- Connects to PX4 over MAVLink serial.
- Reads heartbeat vehicle identity.
- Requests `AUTOPILOT_VERSION`.
- Requests the full parameter list.
- Parses PX4 firmware defaults from a local `PX4-Autopilot` checkout.
- Parses matching airframe defaults from `SYS_AUTOSTART` or `--sys-autostart`.
- Opens a scrollable/searchable terminal UI listing every parameter read from
  the device.
- Can write explicit numeric parameter values when requested.

Normal browse/audit mode does not write parameters. Writes only happen through
write flags and require confirmation unless `--yes` is supplied.

## Build

Build a compiled executable:

```bash
cargo build --release
```

The binary will be at:

```bash
target/release/px4-param-audit
```

Optional local install:

```bash
install -m 0755 target/release/px4-param-audit "$HOME/.local/bin/px4-param-audit"
```

## Usage

Close QGroundControl before using direct serial. QGC holds the Pixhawk USB serial
port open and the audit tool cannot connect while QGC owns it.

```bash
px4-param-audit --connect serial:/dev/cu.usbmodem01:57600
```

If `--connect` is omitted, the tool tries `/dev/cu.usbmodem01` first.

Use a specific PX4 source checkout for baseline defaults:

```bash
px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --px4-source /Users/gonk/git/PX4-Autopilot
```

If the device reports `SYS_AUTOSTART=0`, the tool cannot infer a PX4 airframe
baseline from the vehicle. You can still compare read-only against a known PX4
airframe ID:

```bash
px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --sys-autostart 4019
```

PX4 airframe `4019` is `Holybro X500 V2` in the upstream airframe list.

For non-interactive output:

```bash
px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --sys-autostart 4019 \
  --plain
```

You can also run through Cargo during development:

```bash
cargo run -- --connect serial:/dev/cu.usbmodem01:57600 --sys-autostart 4019
```

## Writes

Write a single numeric parameter:

```bash
px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --set SYS_HAS_GPS=1
```

Write multiple explicit parameters:

```bash
px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --set SYS_HAS_GPS=1 \
  --set EKF2_GPS_CTRL=7
```

Write all numeric diffs that have a known PX4 baseline:

```bash
px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --sys-autostart 4019 \
  --write-diffs
```

The tool prints the planned writes and prompts for `yes`. Add `--yes` only when
you intentionally want non-interactive writes.

String-like PX4 metadata defaults such as `GPS1` are not written by
`--write-diffs`; use an explicit numeric `--set PARAM=VALUE` once the correct
MAVLink parameter value is known.

## TUI Keys

- `q`: quit
- `/`: edit search
- `Enter`: leave search mode
- `Esc`: clear search
- `Up` / `Down` or `k` / `j`: move selection
- `PageUp` / `PageDown` or `Ctrl-u` / `Ctrl-d`: page
- `g` / `G`: top / bottom

The table lists every parameter returned by PX4. Search matches parameter name,
device value, PX4 baseline value, source, or status.

## Baseline Model

The baseline is derived from PX4, not from a hand-written project profile:

```text
PX4 firmware YAML defaults
+ matching airframe defaults from SYS_AUTOSTART
```

If `--sys-autostart` is supplied, that airframe ID is used for the comparison
baseline instead of the device's reported `SYS_AUTOSTART`. Supplying a baseline
only changes comparison output; it does not write by itself.

If a parameter default cannot be found in local PX4 source, the table reports
`<unknown>` rather than guessing.
