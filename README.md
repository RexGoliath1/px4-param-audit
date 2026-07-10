# px4-param-audit

Read-only PX4 parameter audit tool for multirotors.

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

The tool does not write parameters.

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

## Protected Parameters

Rangefinder/SF11-related params are marked as `diff protected` when they differ
from the PX4 baseline:

- `EKF2_HGT_REF`
- `EKF2_RNG_CTRL`
- `SENS_EN_SF1XX`
- `SF1XX_MODE`
- `SF1XX_ROT`

## Baseline Model

The baseline is derived from PX4, not from a hand-written project profile:

```text
PX4 firmware YAML defaults
+ matching airframe defaults from SYS_AUTOSTART
```

If `--sys-autostart` is supplied, that airframe ID is used for the comparison
baseline instead of the device's reported `SYS_AUTOSTART`. This is still
read-only and does not change the vehicle.

If a parameter default cannot be found in local PX4 source, the table reports
`<unknown>` rather than guessing.
