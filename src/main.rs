use anyhow::{Context, Result, bail};
use clap::Parser;
use comfy_table::{Cell, Color, Table};
use mavlink::dialects::common;
use mavlink::{MavHeader, MavlinkVersion, Message, MessageData};
use serde_yaml::Value;
use serialport::SerialPort;
use std::collections::{HashMap, HashSet};
use std::fs;
use std::io::{ErrorKind, Read};
use std::path::{Path, PathBuf};
use std::thread;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(version, about = "Read-only PX4 parameter audit for multirotors")]
struct Args {
    /// MAVLink connection string. Use `serial:/dev/cu.usbmodem01:57600` for Pixhawk USB.
    #[arg(short, long)]
    connect: Option<String>,

    /// PX4-Autopilot source checkout used to derive firmware defaults and airframe defaults.
    #[arg(long, default_value = "/Users/gonk/git/PX4-Autopilot")]
    px4_source: PathBuf,

    /// Compare against this PX4 airframe ID instead of the device SYS_AUTOSTART.
    #[arg(long)]
    sys_autostart: Option<i32>,

    /// Seconds to wait for the full parameter list.
    #[arg(long, default_value_t = 15)]
    param_timeout: u64,

    /// Print every received watched parameter plus basic vehicle identity.
    #[arg(long)]
    verbose: bool,
}

#[derive(Debug, Clone)]
struct ParamValue {
    value: f32,
    mav_type: common::MavParamType,
}

#[derive(Debug, Clone)]
struct Recommendation {
    value: String,
    source: String,
}

#[derive(Debug)]
struct VehicleIdentity {
    system_id: u8,
    component_id: u8,
    mav_type: common::MavType,
    autopilot: common::MavAutopilot,
    version: Option<common::AUTOPILOT_VERSION_DATA>,
}

const WATCHED_PARAMS: &[&str] = &[
    "SYS_AUTOSTART",
    "MAV_TYPE",
    "GPS_1_CONFIG",
    "GPS_2_CONFIG",
    "SYS_HAS_GPS",
    "EKF2_GPS_CTRL",
    "EKF2_GPS_CHECK",
    "EKF2_HGT_REF",
    "EKF2_RNG_CTRL",
    "SENS_EN_SF1XX",
    "SF1XX_MODE",
    "SF1XX_ROT",
];

const PROTECTED_PARAMS: &[&str] = &[
    "SENS_EN_SF1XX",
    "SF1XX_MODE",
    "SF1XX_ROT",
    "EKF2_RNG_CTRL",
    "EKF2_HGT_REF",
];

fn main() -> Result<()> {
    let args = Args::parse();
    let connect = args.connect.unwrap_or_else(default_connection);

    println!("connecting: {connect}");
    let mut conn =
        MavSerial::open(&connect).with_context(|| format!("failed to connect to {connect}"))?;

    let identity = wait_for_heartbeat(&mut conn, Duration::from_secs(10))?;
    request_autopilot_version(&mut conn, &identity)?;
    let version = wait_for_autopilot_version(&mut conn, Duration::from_secs(3));
    let identity = VehicleIdentity {
        version,
        ..identity
    };

    print_identity(&identity);

    request_params(&mut conn, &identity)?;
    let params = collect_params(&mut conn, Duration::from_secs(args.param_timeout))?;
    println!("received {} parameters", params.len());

    let device_sys_autostart = params.get("SYS_AUTOSTART").map(|p| p.value.round() as i32);
    let baseline_sys_autostart = args.sys_autostart.or(device_sys_autostart);
    let firmware_defaults = load_firmware_defaults(&args.px4_source, WATCHED_PARAMS)?;
    let airframe_defaults = if let Some(id) = baseline_sys_autostart {
        load_airframe_defaults(&args.px4_source, id, WATCHED_PARAMS)?
    } else {
        HashMap::new()
    };

    let recommendations = build_recommendations(firmware_defaults, airframe_defaults);
    print_baseline(device_sys_autostart, args.sys_autostart, &args.px4_source)?;
    print_audit_table(&params, &recommendations);

    if args.verbose {
        println!("\nPX4 source: {}", args.px4_source.display());
    }

    Ok(())
}

fn default_connection() -> String {
    for port in ["/dev/cu.usbmodem01", "/dev/tty.usbmodem01"] {
        if Path::new(port).exists() {
            return format!("serial:{port}:57600");
        }
    }
    "serial:/dev/cu.usbmodem01:57600".to_string()
}

struct MavSerial {
    port: Box<dyn SerialPort>,
    rx_buf: Vec<u8>,
    sequence: u8,
}

impl MavSerial {
    fn open(connect: &str) -> Result<Self> {
        let (path, baud) = parse_serial_connection(connect)?;
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(10))
            .open()?;
        Ok(Self {
            port,
            rx_buf: Vec::with_capacity(4096),
            sequence: 0,
        })
    }

    fn send(&mut self, msg: &common::MavMessage) -> Result<()> {
        let header = MavHeader {
            system_id: 255,
            component_id: 190,
            sequence: self.sequence,
        };
        self.sequence = self.sequence.wrapping_add(1);
        mavlink::write_versioned_msg(&mut self.port, MavlinkVersion::V2, header, msg)?;
        Ok(())
    }

    fn recv(&mut self) -> Option<(MavHeader, common::MavMessage)> {
        self.read_available();
        self.decode_next_frame()
    }

    fn read_available(&mut self) {
        let Ok(available) = self.port.bytes_to_read() else {
            return;
        };
        if available == 0 {
            return;
        }

        let mut buf = vec![0_u8; (available as usize).min(8192)];
        match self.port.read(&mut buf) {
            Ok(n) => self.rx_buf.extend_from_slice(&buf[..n]),
            Err(err) if matches!(err.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock) => {}
            Err(_) => {}
        }
    }

    fn decode_next_frame(&mut self) -> Option<(MavHeader, common::MavMessage)> {
        loop {
            let start = self.rx_buf.iter().position(|b| matches!(*b, 0xfe | 0xfd))?;
            if start > 0 {
                self.rx_buf.drain(..start);
            }

            let magic = self.rx_buf[0];
            let payload_len = *self.rx_buf.get(1)? as usize;
            let frame_len = match magic {
                0xfe => 6 + payload_len + 2,
                0xfd => {
                    let incompat_flags = *self.rx_buf.get(2)?;
                    10 + payload_len + 2 + if incompat_flags & 0x01 != 0 { 13 } else { 0 }
                }
                _ => unreachable!(),
            };

            if self.rx_buf.len() < frame_len {
                return None;
            }

            let frame: Vec<u8> = self.rx_buf.drain(..frame_len).collect();
            if let Some(decoded) = decode_mavlink_frame(&frame) {
                return Some(decoded);
            }
        }
    }
}

fn decode_mavlink_frame(frame: &[u8]) -> Option<(MavHeader, common::MavMessage)> {
    match frame.first().copied()? {
        0xfe => {
            let payload_len = frame[1] as usize;
            let header = MavHeader {
                sequence: frame[2],
                system_id: frame[3],
                component_id: frame[4],
            };
            let msgid = frame[5] as u32;
            let payload = &frame[6..6 + payload_len];
            let msg = common::MavMessage::parse(MavlinkVersion::V1, msgid, payload).ok()?;
            Some((header, msg))
        }
        0xfd => {
            let payload_len = frame[1] as usize;
            let header = MavHeader {
                sequence: frame[4],
                system_id: frame[5],
                component_id: frame[6],
            };
            let msgid = (frame[7] as u32) | ((frame[8] as u32) << 8) | ((frame[9] as u32) << 16);
            let payload = &frame[10..10 + payload_len];
            let msg = common::MavMessage::parse(MavlinkVersion::V2, msgid, payload).ok()?;
            Some((header, msg))
        }
        _ => None,
    }
}

fn parse_serial_connection(connect: &str) -> Result<(&str, u32)> {
    let Some(rest) = connect.strip_prefix("serial:") else {
        bail!("only serial: connection strings are implemented in the MVP");
    };
    let Some((path, baud)) = rest.rsplit_once(':') else {
        bail!("serial connection must look like serial:/dev/cu.usbmodem01:57600");
    };
    Ok((path, baud.parse()?))
}

fn wait_for_heartbeat(conn: &mut MavSerial, timeout: Duration) -> Result<VehicleIdentity> {
    let deadline = Instant::now() + timeout;
    let mut next_gcs_heartbeat = Instant::now();
    while Instant::now() < deadline {
        if Instant::now() >= next_gcs_heartbeat {
            send_gcs_heartbeat(conn)?;
            next_gcs_heartbeat = Instant::now() + Duration::from_millis(900);
        }
        match conn.recv() {
            Some((header, common::MavMessage::HEARTBEAT(heartbeat))) => {
                if heartbeat.autopilot == common::MavAutopilot::MAV_AUTOPILOT_INVALID {
                    continue;
                }
                return Ok(VehicleIdentity {
                    system_id: header.system_id,
                    component_id: header.component_id,
                    mav_type: heartbeat.mavtype,
                    autopilot: heartbeat.autopilot,
                    version: None,
                });
            }
            Some(_) => {}
            None => thread::sleep(Duration::from_millis(5)),
        }
    }
    bail!("timed out waiting for MAVLink heartbeat");
}

fn send_gcs_heartbeat(conn: &mut MavSerial) -> Result<()> {
    let msg = common::MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: common::MavType::MAV_TYPE_GCS,
        autopilot: common::MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: common::MavModeFlag::empty(),
        system_status: common::MavState::MAV_STATE_ACTIVE,
        mavlink_version: 3,
    });
    conn.send(&msg)?;
    Ok(())
}

fn request_autopilot_version(conn: &mut MavSerial, identity: &VehicleIdentity) -> Result<()> {
    let msg = common::MavMessage::COMMAND_LONG(common::COMMAND_LONG_DATA {
        target_system: identity.system_id,
        target_component: identity.component_id,
        command: common::MavCmd::MAV_CMD_REQUEST_MESSAGE,
        confirmation: 0,
        param1: common::AUTOPILOT_VERSION_DATA::ID as f32,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
    });
    conn.send(&msg)?;
    Ok(())
}

fn wait_for_autopilot_version(
    conn: &mut MavSerial,
    timeout: Duration,
) -> Option<common::AUTOPILOT_VERSION_DATA> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match conn.recv() {
            Some((_, common::MavMessage::AUTOPILOT_VERSION(version))) => return Some(version),
            Some(_) => {}
            None => thread::sleep(Duration::from_millis(5)),
        }
    }
    None
}

fn request_params(conn: &mut MavSerial, identity: &VehicleIdentity) -> Result<()> {
    let msg = common::MavMessage::PARAM_REQUEST_LIST(common::PARAM_REQUEST_LIST_DATA {
        target_system: identity.system_id,
        target_component: identity.component_id,
    });
    conn.send(&msg)?;
    Ok(())
}

fn collect_params(conn: &mut MavSerial, timeout: Duration) -> Result<HashMap<String, ParamValue>> {
    let deadline = Instant::now() + timeout;
    let mut params = HashMap::new();
    while Instant::now() < deadline {
        match conn.recv() {
            Some((_, common::MavMessage::PARAM_VALUE(param))) => {
                let name = param.param_id.to_str().unwrap_or("").to_string();
                if !name.is_empty() {
                    let expected_count = param.param_count as usize;
                    params.insert(
                        name,
                        ParamValue {
                            value: param.param_value,
                            mav_type: param.param_type,
                        },
                    );
                    if expected_count > 0 && params.len() >= expected_count {
                        return Ok(params);
                    }
                }
            }
            Some(_) => {}
            None => thread::sleep(Duration::from_millis(5)),
        }
    }

    if params.is_empty() {
        bail!("timed out waiting for PARAM_VALUE messages");
    }
    Ok(params)
}

fn print_identity(identity: &VehicleIdentity) {
    println!(
        "vehicle: sys={} comp={} autopilot={:?} mav_type={:?}",
        identity.system_id, identity.component_id, identity.autopilot, identity.mav_type
    );
    if let Some(version) = &identity.version {
        println!(
            "px4 version: {} board={} vendor={} product={} git={}",
            format_px4_version(version.flight_sw_version),
            version.board_version,
            version.vendor_id,
            version.product_id,
            format_hash(&version.flight_custom_version),
        );
    } else {
        println!("px4 version: unavailable via AUTOPILOT_VERSION");
    }
}

fn print_baseline(
    device_sys_autostart: Option<i32>,
    override_sys_autostart: Option<i32>,
    px4_source: &Path,
) -> Result<()> {
    match (device_sys_autostart, override_sys_autostart) {
        (Some(device), Some(override_id)) if device != override_id => {
            println!(
                "baseline: PX4 airframe {override_id} ({}) via --sys-autostart; device SYS_AUTOSTART={device}",
                airframe_name(px4_source, override_id)?
                    .unwrap_or_else(|| "unknown airframe".into())
            );
        }
        (_, Some(override_id)) => {
            println!(
                "baseline: PX4 airframe {override_id} ({}) via --sys-autostart",
                airframe_name(px4_source, override_id)?
                    .unwrap_or_else(|| "unknown airframe".into())
            );
        }
        (Some(device), None) if device != 0 => {
            println!(
                "baseline: PX4 airframe {device} ({}) from device SYS_AUTOSTART",
                airframe_name(px4_source, device)?.unwrap_or_else(|| "unknown airframe".into())
            );
        }
        (Some(device), None) => {
            println!(
                "baseline: firmware defaults only; device SYS_AUTOSTART={device} does not select an airframe"
            );
        }
        (None, None) => {
            println!("baseline: firmware defaults only; device SYS_AUTOSTART was not received");
        }
    }
    Ok(())
}

fn airframe_name(px4_source: &Path, sys_autostart: i32) -> Result<Option<String>> {
    let airframes = px4_source.join("ROMFS/px4fmu_common/init.d/airframes");
    let Some(path) = find_airframe_script(&airframes, sys_autostart)? else {
        return Ok(None);
    };
    let text = fs::read_to_string(path)?;
    Ok(text.lines().find_map(|line| {
        line.trim()
            .strip_prefix("# @name ")
            .map(|name| name.to_string())
    }))
}

fn format_px4_version(encoded: u32) -> String {
    let major = (encoded >> 24) & 0xff;
    let minor = (encoded >> 16) & 0xff;
    let patch = (encoded >> 8) & 0xff;
    let release_type = encoded & 0xff;
    format!("{major}.{minor}.{patch} type={release_type}")
}

fn format_hash(bytes: &[u8; 8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

fn load_firmware_defaults(px4_source: &Path, watched: &[&str]) -> Result<HashMap<String, String>> {
    let watched: HashSet<&str> = watched.iter().copied().collect();
    let mut defaults = HashMap::new();
    let src = px4_source.join("src");
    visit_yaml_files(&src, &mut |path| {
        if let Ok(text) = fs::read_to_string(path) {
            if let Ok(yaml) = serde_yaml::from_str::<Value>(&text) {
                collect_yaml_defaults(&yaml, &watched, &mut defaults);
            }
        }
    })?;
    Ok(defaults)
}

fn visit_yaml_files(dir: &Path, f: &mut dyn FnMut(&Path)) -> Result<()> {
    if !dir.exists() {
        return Ok(());
    }
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            visit_yaml_files(&path, f)?;
        } else if path
            .extension()
            .is_some_and(|ext| ext == "yaml" || ext == "yml")
        {
            f(&path);
        }
    }
    Ok(())
}

fn collect_yaml_defaults(
    value: &Value,
    watched: &HashSet<&str>,
    defaults: &mut HashMap<String, String>,
) {
    match value {
        Value::Mapping(map) => {
            if let Some(Value::Mapping(definitions)) = map.get(Value::String("definitions".into()))
            {
                for (key, body) in definitions {
                    if let Some(name) = key.as_str() {
                        if watched.contains(name) {
                            if let Some(default) = body
                                .as_mapping()
                                .and_then(|m| m.get(Value::String("default".into())))
                            {
                                defaults.insert(name.to_string(), yaml_scalar_to_string(default));
                            }
                        }
                    }
                }
            }
            if let Some(Value::Mapping(port_config)) =
                map.get(Value::String("port_config_param".into()))
            {
                let name = port_config
                    .get(Value::String("name".into()))
                    .and_then(Value::as_str);
                let default = port_config.get(Value::String("default".into()));
                if let (Some(name), Some(default)) = (name, default) {
                    if watched.contains(name) {
                        defaults.insert(name.to_string(), yaml_scalar_to_string(default));
                    }
                }
            }
            for child in map.values() {
                collect_yaml_defaults(child, watched, defaults);
            }
        }
        Value::Sequence(seq) => {
            for child in seq {
                collect_yaml_defaults(child, watched, defaults);
            }
        }
        _ => {}
    }
}

fn yaml_scalar_to_string(value: &Value) -> String {
    match value {
        Value::Number(n) => n.to_string(),
        Value::String(s) => s.clone(),
        Value::Bool(b) => b.to_string(),
        _ => format!("{value:?}"),
    }
}

fn load_airframe_defaults(
    px4_source: &Path,
    sys_autostart: i32,
    watched: &[&str],
) -> Result<HashMap<String, String>> {
    let watched: HashSet<&str> = watched.iter().copied().collect();
    let airframes = px4_source.join("ROMFS/px4fmu_common/init.d/airframes");
    let Some(path) = find_airframe_script(&airframes, sys_autostart)? else {
        return Ok(HashMap::new());
    };
    let mut visited = HashSet::new();
    let mut defaults = parse_airframe_script(px4_source, &path, &watched, &mut visited)?;
    if watched.contains("SYS_AUTOSTART") {
        defaults.insert("SYS_AUTOSTART".into(), sys_autostart.to_string());
    }
    Ok(defaults)
}

fn find_airframe_script(dir: &Path, sys_autostart: i32) -> Result<Option<PathBuf>> {
    if !dir.exists() {
        return Ok(None);
    }
    let prefix = format!("{sys_autostart}_");
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if path
            .file_name()
            .and_then(|n| n.to_str())
            .is_some_and(|name| name.starts_with(&prefix))
        {
            return Ok(Some(path));
        }
    }
    Ok(None)
}

fn parse_airframe_script(
    px4_source: &Path,
    path: &Path,
    watched: &HashSet<&str>,
    visited: &mut HashSet<PathBuf>,
) -> Result<HashMap<String, String>> {
    if !visited.insert(path.to_path_buf()) {
        return Ok(HashMap::new());
    }

    let text = fs::read_to_string(path)?;
    let mut defaults = HashMap::new();
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        let parts: Vec<&str> = line.split_whitespace().collect();
        if parts.len() >= 4 && parts[0] == "param" && parts[1] == "set-default" {
            let name = parts[2];
            if watched.contains(name) {
                defaults.insert(name.to_string(), parts[3].to_string());
            }
        } else if parts.len() >= 2 && parts[0] == "." {
            if let Some(source_path) = resolve_px4_init_source(px4_source, parts[1]) {
                defaults.extend(parse_airframe_script(
                    px4_source,
                    &source_path,
                    watched,
                    visited,
                )?);
            }
        }
    }
    Ok(defaults)
}

fn resolve_px4_init_source(px4_source: &Path, source: &str) -> Option<PathBuf> {
    source
        .strip_prefix("${R}etc/init.d/")
        .map(|relative| px4_source.join("ROMFS/px4fmu_common/init.d").join(relative))
}

fn build_recommendations(
    firmware: HashMap<String, String>,
    airframe: HashMap<String, String>,
) -> HashMap<String, Recommendation> {
    let mut out = HashMap::new();
    for (name, value) in firmware {
        out.insert(
            name,
            Recommendation {
                value,
                source: "firmware default".into(),
            },
        );
    }
    for (name, value) in airframe {
        out.insert(
            name,
            Recommendation {
                value,
                source: "airframe default".into(),
            },
        );
    }
    out
}

fn print_audit_table(
    params: &HashMap<String, ParamValue>,
    recommendations: &HashMap<String, Recommendation>,
) {
    let protected: HashSet<&str> = PROTECTED_PARAMS.iter().copied().collect();
    let mut table = Table::new();
    table.set_header(vec!["Param", "Device", "PX4 baseline", "Source", "Status"]);

    for name in WATCHED_PARAMS {
        let current = params
            .get(*name)
            .map(format_param)
            .unwrap_or_else(|| "<missing>".into());
        let rec = recommendations.get(*name);
        let baseline = rec.map(|r| r.value.as_str()).unwrap_or("<unknown>");
        let source = rec.map(|r| r.source.as_str()).unwrap_or("not found");
        let status = classify_status(*name, &current, baseline, protected.contains(name));
        let color = match status.as_str() {
            "match" => Color::Green,
            "diff" => Color::Red,
            "diff protected" => Color::Yellow,
            "unknown" => Color::DarkGrey,
            _ => Color::White,
        };
        table.add_row(vec![
            Cell::new(*name),
            Cell::new(current),
            Cell::new(baseline),
            Cell::new(source),
            Cell::new(status).fg(color),
        ]);
    }

    println!("\n{table}");
}

fn format_param(param: &ParamValue) -> String {
    match param.mav_type {
        common::MavParamType::MAV_PARAM_TYPE_UINT8
        | common::MavParamType::MAV_PARAM_TYPE_INT8
        | common::MavParamType::MAV_PARAM_TYPE_UINT16
        | common::MavParamType::MAV_PARAM_TYPE_INT16
        | common::MavParamType::MAV_PARAM_TYPE_UINT32
        | common::MavParamType::MAV_PARAM_TYPE_INT32 => format!("{}", param.value.round() as i64),
        _ => {
            let s = format!("{:.6}", param.value);
            s.trim_end_matches('0').trim_end_matches('.').to_string()
        }
    }
}

fn classify_status(name: &str, current: &str, baseline: &str, protected: bool) -> String {
    if baseline == "<unknown>" || current == "<missing>" {
        return "unknown".into();
    }
    let matches = values_equivalent(current, baseline);
    match (matches, protected) {
        (true, _) => "match".into(),
        (false, true) => "diff protected".into(),
        (false, false) => {
            if name.starts_with("GPS") || name.contains("GPS") {
                "diff".into()
            } else {
                "diff".into()
            }
        }
    }
}

fn values_equivalent(a: &str, b: &str) -> bool {
    if let (Ok(af), Ok(bf)) = (a.parse::<f64>(), b.parse::<f64>()) {
        (af - bf).abs() < 0.0001
    } else {
        a == b
    }
}
