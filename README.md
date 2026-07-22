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
- Parses PX4 firmware defaults and metadata from the pinned `vendor/PX4-Autopilot`
  submodule by default.
- Parses matching airframe defaults from `SYS_AUTOSTART` or `--sys-autostart`.
- Vendors ModalAI VOXL source and parameter repos for VOXL-specific baseline
  support without copying their generated defaults into this repository.
- Opens a scrollable/searchable terminal UI listing every parameter read from
  the device.
- Can write explicit numeric parameter values when requested.

Normal browse/audit mode does not write parameters. Writes only happen through
write flags and require confirmation unless `--yes` is supplied.

## Dependencies

Common requirements:

- Rust toolchain with Cargo. Install with `rustup` unless your package manager
  already provides a current Rust release.
- `git` and `make`.
- The baseline source submodules, or another PX4 checkout passed with
  `--px4-source`.
- Access to a MAVLink endpoint: serial, UDP, or TCP.

macOS:

```bash
brew install rust git make
```

USB telemetry radios and Pixhawk USB ports normally appear as `/dev/cu.*`.
Use `./px4-param-audit --list-ports` to see the exact device path.

Linux:

```bash
sudo apt-get update
sudo apt-get install -y build-essential pkg-config libudev-dev git
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Serial devices commonly appear as `/dev/ttyACM*`, `/dev/ttyUSB*`, or
`/dev/serial/by-id/*`. If the port is permission-denied, add your user to the
serial device group and start a new login session:

```bash
sudo usermod -aG dialout "$USER"
```

## Build

Initialize the pinned baseline sources:

```bash
git submodule update --init \
  vendor/PX4-Autopilot \
  vendor/voxl-px4 \
  vendor/voxl-px4-params
git -C vendor/voxl-px4 submodule update --init px4-firmware
```

Build a compiled executable:

```bash
make
```

This creates both:

```bash
target/release/px4-param-audit
./px4-param-audit
```

The root-level binary is ignored by git. Cargo still keeps its normal build
artifacts under `target/`.

If you only want Cargo's normal build output:

```bash
cargo build --release
```

Remove the root-level binary and Cargo build artifacts:

```bash
make clean
```

Optional local install:

```bash
install -m 0755 target/release/px4-param-audit "$HOME/.local/bin/px4-param-audit"
```

The same install step is available through:

```bash
make install
```

The examples below assume you are running from the repository root after
`make`, so they use `./px4-param-audit`. If you install the binary onto your
`PATH`, you can omit the `./` prefix.

## Vendored Baseline Sources

Baseline data is referenced from source repositories at runtime; parameter
metadata and defaults are not copied into this project.

- `vendor/PX4-Autopilot`: upstream PX4 source. This is the default source for
  generic MAVLink/Pixhawk use.
- `vendor/voxl-px4`: ModalAI's VOXL PX4 wrapper. Its nested
  `px4-firmware` submodule is the PX4 firmware tree to use for VOXL firmware
  metadata and defaults.
- `vendor/voxl-px4-params`: ModalAI's platform and helper parameter bundles,
  including Starling platform files such as
  `params/v1.14/platforms/D0012_Starling_2_Max.params`.

When `--voxl-ssh` or `--voxl-adb` is supplied, the tool discovers the VOXL
hostname, SKU, installed `voxl-px4` package, installed `voxl-px4-params`
package, installed `/usr/share/modalai/px4_params/v*` trees, and the ordered
parameter stack from `voxl-configure-px4-params`. It then applies that stack on
top of the PX4 firmware and airframe defaults. This keeps generic PX4
parameters visible when a VOXL platform file does not override them, while
preferring ModalAI's platform and helper defaults when they exist.

The selected VOXL platform, params stack size, installed package versions, and
local vendored source versions are shown in the TUI Baseline panel. Version rows
are marked `match`, `mismatch`, or `unknown` so it is clear when the drone and
local sources are not aligned.

For example, a Starling 2 Max with SKU `MRB-D0012-...` and `voxl-px4
1.14.0-...` can resolve to an ordered stack like:

```text
params/v1.14/platforms/VOXL_2_defaults.params
params/v1.14/EKF2_helpers/vio_gps_baro.params
params/v1.14/platforms/D0012_Starling_2_Max.params
params/v1.14/EKF2_helpers/exposed_baro.params
params/v1.14/battery_helpers/Amprius_18650_4s.params
params/v1.14/other_helpers/starling_2_max_outdoor_position.params
params/v1.14/radio_helpers/Commando_8.params
```

MAVLink remains the transport for reading and writing PX4 parameters. SSH or
ADB is only used for VOXL baseline discovery, so generic MAVLink-only
PX4/Holybro use does not require SSH or ADB.

To inspect against the ModalAI firmware tree manually without VOXL discovery:

```bash
./px4-param-audit \
  --connect udp-listen:0.0.0.0:14550 \
  --px4-source vendor/voxl-px4/px4-firmware \
  --sys-autostart 4001
```

## Usage

Close QGroundControl before using direct serial. QGC holds the Pixhawk USB serial
port open and the audit tool cannot connect while QGC owns it.

```bash
./px4-param-audit --connect serial:/dev/cu.usbmodem01:57600
```

To keep QGC open, do not have QGC own the USB serial port directly. Put a
MAVLink router/proxy in front of the Pixhawk and connect both QGC and this tool
to separate UDP/TCP endpoints from that router. The tool already supports
`udp-listen`, `udp-connect`/`udp`, and `tcp` connection strings for that setup.

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

## Starling / VOXL UDP

Starling 2 normally appears over USB as a VOXL/ADB device, not as a Pixhawk
MAVLink serial port on the host computer. In that setup, use UDP from
`voxl-mavlink-server`.

Confirm ADB can see the VOXL:

```bash
adb devices -l
```

Check whether `voxl-mavlink-server` is running:

```bash
adb shell 'voxl-inspect-services | grep voxl-mavlink-server'
```

Find the GCS IPs and MAVLink UDP ports configured on the VOXL:

```bash
adb shell 'cat /etc/modalai/voxl-mavlink-server.conf'
adb shell 'ss -lunpt | grep -E "14550|1455"'
```

In `voxl-mavlink-server.conf`, `primary_static_gcs_ip` and
`secondary_static_gcs_ip` are the host IPs that VOXL will send GCS MAVLink to.
`gcs_port_from_autopilot` is the VOXL-side receive path from PX4; the host
usually listens on the GCS UDP port shown by `ss`, commonly `14550`.

Find the PX4 airframe ID if you want to sanity-check the generic PX4 baseline:

```bash
adb shell 'px4-param show SYS_AUTOSTART'
```

Then run the tool on the host. For example, if VOXL sends GCS MAVLink to this
computer on UDP `14550` and the VOXL is reachable over SSH:

```bash
./px4-param-audit \
  --connect udp-listen:0.0.0.0:14550 \
  --voxl-ssh root@100.116.80.54 \
  --param-timeout 45
```

If the VOXL is reachable over ADB instead:

```bash
./px4-param-audit \
  --connect udp-listen:0.0.0.0:14550 \
  --voxl-adb \
  --param-timeout 45
```

For multiple ADB devices, pass the ADB serial after `--voxl-adb`:

```bash
./px4-param-audit \
  --connect udp-listen:0.0.0.0:14550 \
  --voxl-adb 0123456789abcdef \
  --param-timeout 45
```

Example Starling 2 session searching for `GPS_CTRL` and opening the metadata
popup:

![Starling 2 GPS_CTRL search demo](docs/assets/starling-gps-ctrl.gif)

If QGroundControl or another process is already bound to `14550`, configure a
second VOXL GCS endpoint or use a MAVLink router/proxy so each program has its
own local UDP port.

For another computer, the same split applies: SSH or ADB only discovers the
VOXL baseline, while MAVLink must be sent to the computer running this tool.
Update `primary_static_gcs_ip` or `secondary_static_gcs_ip` in
`voxl-mavlink-server.conf` to that computer's IP, or route MAVLink to it with a
router/proxy.

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
./px4-param-audit --list-ports
```

Use a specific PX4 source checkout for baseline defaults:

```bash
./px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --px4-source /path/to/PX4-Autopilot
```

If `--px4-source` is omitted, the tool uses `vendor/PX4-Autopilot`. You can
also set `PX4_PARAM_AUDIT_PX4_SOURCE=/path/to/PX4-Autopilot`.

If the device reports `SYS_AUTOSTART=0`, the tool cannot infer a PX4 airframe
baseline from the vehicle. You can still compare read-only against a known PX4
airframe ID:

```bash
./px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --sys-autostart 4019
```

PX4 airframe `4019` is `Holybro X500 V2` in the upstream airframe list.

For non-interactive output:

```bash
./px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --sys-autostart 4019 \
  --plain
```

You can also run through Cargo during development:

```bash
cargo run -- --connect serial:/dev/cu.usbmodem01:57600 --sys-autostart 4019
```

## TUI Editing

The TUI can edit or reset device values directly:

- Select a parameter row.
- Press `e` or `Enter`.
- Type a numeric value.
- Press `Enter` to write it with MAVLink `PARAM_SET`.
- Press `Esc` to cancel editing.

For parameters with PX4 metadata, the selected-row detail panel shows the
QGroundControl-style type information from the selected PX4 source. Enum
parameters list their numeric values and labels, bitmask parameters list every
known bit and the active bits for the current value, and boolean parameters show
the `0=Disabled` / `1=Enabled` mapping.
This metadata is parsed from the selected PX4 source checkout at runtime; it is
not copied into this repository.

To reset one selected parameter to the PX4 baseline value, press `d`.

To reset every numeric non-default value with a known PX4 baseline, press `A`.
The TUI shows a warning prompt before writing. Press `y` to confirm or `n` /
`Esc` to cancel. PX4 serial-port labels such as `GPS1` are resolved to their
numeric MAVLink parameter values when PX4's serial metadata is available.
Other string-like defaults are skipped because the tool only writes single
numeric MAVLink parameter values.

After PX4 confirms the write, the device value and status update in the table.

## CLI Writes

Write a single numeric parameter:

```bash
./px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --set SYS_HAS_GPS=1
```

Write multiple explicit parameters:

```bash
./px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --set SYS_HAS_GPS=1 \
  --set EKF2_GPS_CTRL=7
```

Write all numeric diffs that have a known PX4 baseline:

```bash
./px4-param-audit \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --sys-autostart 4019 \
  --write-diffs
```

The tool prints the planned writes and prompts for `yes`. Add `--yes` only when
you intentionally want non-interactive writes.

PX4 serial-port labels such as `GPS1` are resolved to their numeric MAVLink
parameter values when PX4's serial metadata is available. Other string-like
defaults are not written by `--write-diffs`; use an explicit numeric
`--set PARAM=VALUE` once the correct MAVLink parameter value is known.

## TUI Keys

- `q`: quit
- `?` / `m`: show a popup with all enum, boolean, or bitmask choices for the selected parameter
- `f`: toggle between all parameters and differing parameters only
- `e` / `Enter`: edit selected device value
- `d`: write the selected parameter's numeric PX4 baseline value
- `A`: prompt, then write all numeric diffs back to PX4 baseline values
- `/`: edit search
- `Enter`: leave search mode
- `Esc`: clear search or cancel editing
- `Up` / `Down` or `k` / `j`: move selection
- `PageUp` / `PageDown` or `Ctrl-u` / `Ctrl-d`: page
- `g` / `G`: top / bottom

The table lists every parameter returned by PX4. Search matches parameter name,
device value, PX4 baseline value, source, or status.

## Baseline Model

The baseline is derived from PX4 and VOXL source repositories, not from a
hand-written project profile:

```text
PX4 firmware defaults and metadata
+ matching airframe defaults from SYS_AUTOSTART
+ ordered VOXL parameter stack when --voxl-ssh or --voxl-adb is used
```

The layers are merged in that order. Generic PX4 firmware defaults fill in
parameters that the vehicle-specific files do not mention, and later VOXL
helper/platform files override earlier defaults when ModalAI supplies a more
specific value.

If `--sys-autostart` is supplied, that airframe ID is used for the comparison
baseline instead of the device's reported `SYS_AUTOSTART`. Supplying a baseline
only changes comparison output; it does not write by itself.

If the airframe cannot be selected or does not match any reported parameters,
the tool falls back to QGC-style firmware defaults only. Fallback rows are shown
as `fallback match` or `fallback diff` and are highlighted as warnings because
airframe-specific defaults were not applied. Fallback baselines are display-only
for default reset actions.

The selected PX4 checkout path and git commit are printed at startup. Treat
diffs as authoritative only when the baseline source matches the PX4 version you
intend to compare against.

If a parameter default cannot be found in local PX4 source, the table reports
`<unknown>` rather than guessing.
