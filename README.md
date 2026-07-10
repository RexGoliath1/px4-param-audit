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
- Parses selected airframe defaults from `SYS_AUTOSTART`.
- Prints a read-only watched-parameter table.

The tool does not write parameters.

## Usage

Close QGroundControl before using direct serial. QGC holds the Pixhawk USB serial
port open and the audit tool cannot connect while QGC owns it.

```bash
cargo run -- --connect serial:/dev/cu.usbmodem01:57600
```

If `--connect` is omitted, the tool tries `/dev/cu.usbmodem01` first.

Use a specific PX4 source checkout for baseline defaults:

```bash
cargo run -- \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --px4-source /Users/gonk/git/PX4-Autopilot
```

If the device reports `SYS_AUTOSTART=0`, the tool cannot infer a PX4 airframe
baseline from the vehicle. You can still compare read-only against a known PX4
airframe ID:

```bash
cargo run -- \
  --connect serial:/dev/cu.usbmodem01:57600 \
  --sys-autostart 4019
```

PX4 airframe `4019` is `Holybro X500 V2` in the upstream airframe list.

## Watched Parameters

The first watched set is focused on GPS and SF11/rangefinder safety:

- `SYS_AUTOSTART`
- `MAV_TYPE`
- `GPS_1_CONFIG`
- `GPS_2_CONFIG`
- `SYS_HAS_GPS`
- `EKF2_GPS_CTRL`
- `EKF2_GPS_CHECK`
- `EKF2_HGT_REF`
- `EKF2_RNG_CTRL`
- `SENS_EN_SF1XX`
- `SF1XX_MODE`
- `SF1XX_ROT`

Rangefinder/SF11-related params are marked as protected in the table when they
differ from the PX4 baseline.

## Baseline Model

The baseline is derived from PX4, not from a hand-written project profile:

```text
PX4 firmware YAML defaults
+ selected airframe defaults from SYS_AUTOSTART
```

If `--sys-autostart` is supplied, that airframe ID is used for the comparison
baseline instead of the device's reported `SYS_AUTOSTART`. This is still
read-only and does not change the vehicle.

If a parameter default cannot be found in local PX4 source, the table reports
`<unknown>` rather than guessing.
