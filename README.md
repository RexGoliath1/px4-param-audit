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

If a parameter default cannot be found in local PX4 source, the table reports
`<unknown>` rather than guessing.
