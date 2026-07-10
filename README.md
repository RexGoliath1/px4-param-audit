# px4-param-audit

PX4 parameter audit and explicit write tool for multirotors.

The first target is Pixhawk/Holybro over MAVLink serial. MAVLink UDP/TCP is
also supported for remote links. Starling/VOXL support still needs a ModalAI
baseline provider.

## Current MVP

- Connects to PX4 over MAVLink serial, UDP, or TCP.
- Reads heartbeat vehicle identity.
- Requests `AUTOPILOT_VERSION`.
- Requests the full parameter list.
- Parses PX4 firmware defaults from the pinned `vendor/PX4-Autopilot`
  submodule by default.
- Parses matching airframe defaults from `SYS_AUTOSTART` or `--sys-autostart`.
- Opens a scrollable/searchable terminal UI listing every parameter read from
  the device.
- Can write explicit numeric parameter values when requested.

Normal browse/audit mode does not write parameters. Writes only happen through
write flags and require confirmation unless `--yes` is supplied.

## Build

Initialize the pinned PX4 baseline source:

```bash
git submodule update --init --depth 1 vendor/PX4-Autopilot
```

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

Supported connection strings:

```text
serial:/dev/ttyACM0:57600
udp-listen:0.0.0.0:14550
udp-connect:192.168.8.1:14550
udp:192.168.8.1:14550
tcp:192.168.8.1:5760
```

Use `udp-listen` when the drone or MAVLink router is configured to send
MAVLink to this computer. Use `udp-connect`/`udp` or `tcp` when this tool should
initiate traffic to a known endpoint.

If `--connect` is omitted, the tool autodiscovers a PX4/Pixhawk USB serial
port and uses baud `57600`.

On macOS this commonly resolves to:

```text
serial:/dev/cu.usbmodem01:57600
```

On Linux this commonly resolves to a `ttyACM` or `/dev/serial/by-id` device,
for example:

```text
serial:/dev/ttyACM0:57600
```

List detected serial ports and their discovery scores:

```bash
px4-param-audit --list-ports
```

Use a specific PX4 source checkout for baseline defaults:

```bash
px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --px4-source /path/to/PX4-Autopilot
```

If `--px4-source` is omitted, the tool uses `vendor/PX4-Autopilot`. You can
also set `PX4_PARAM_AUDIT_PX4_SOURCE=/path/to/PX4-Autopilot`.

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

## TUI Editing

The TUI can edit device values directly:

- Select a parameter row.
- Press `e` or `Enter`.
- Type a numeric value.
- Press `Enter` to write it with MAVLink `PARAM_SET`.
- Press `Esc` to cancel editing.

After PX4 confirms the write, the device value and status update in the table.

## CLI Writes

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
- `e` / `Enter`: edit selected device value
- `/`: edit search
- `Enter`: leave search mode
- `Esc`: clear search or cancel editing
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

The selected PX4 checkout path and git commit are printed at startup. Treat
diffs as authoritative only when the baseline source matches the PX4 version you
intend to compare against.

If a parameter default cannot be found in local PX4 source, the table reports
`<unknown>` rather than guessing.
