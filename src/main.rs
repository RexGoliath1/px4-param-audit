use anyhow::{Context, Result, bail, ensure};
use clap::Parser;
use comfy_table::{Cell, Color, Table as PlainTable};
use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::execute;
use crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use mavlink::dialects::common;
use mavlink::{MavHeader, MavlinkVersion, Message, MessageData};
use ratatui::Terminal;
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout};
use ratatui::style::Style;
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Cell as TuiCell, Paragraph, Row, Table, TableState};
use serde_yaml::Value;
use serialport::{SerialPort, SerialPortInfo, SerialPortType, UsbPortInfo};
use std::collections::{HashMap, HashSet};
use std::fs;
use std::io::{self, ErrorKind, IsTerminal, Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs, UdpSocket};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};

#[derive(Parser, Debug)]
#[command(
    version,
    about = "PX4 parameter audit and explicit write tool for multirotors"
)]
struct Args {
    /// MAVLink connection string. Supports serial, udp-listen, udp-connect/udp, and tcp.
    #[arg(short, long)]
    connect: Option<String>,

    /// PX4-Autopilot source checkout used to derive firmware defaults and airframe defaults.
    #[arg(long)]
    px4_source: Option<PathBuf>,

    /// Compare against this PX4 airframe ID instead of the device SYS_AUTOSTART.
    #[arg(long)]
    sys_autostart: Option<i32>,

    /// Seconds to wait for the full parameter list.
    #[arg(long, default_value_t = 15)]
    param_timeout: u64,

    /// Print extra collection context before showing the audit.
    #[arg(long)]
    verbose: bool,

    /// Print a static table instead of opening the interactive TUI.
    #[arg(long)]
    plain: bool,

    /// Set one parameter to a numeric value, for example SYS_HAS_GPS=1. May be repeated.
    #[arg(long = "set", value_name = "PARAM=VALUE")]
    set_params: Vec<String>,

    /// Write every numeric diff with a known PX4 baseline value.
    #[arg(long)]
    write_diffs: bool,

    /// Skip interactive confirmation for writes.
    #[arg(long)]
    yes: bool,

    /// Seconds to wait for each write confirmation from PX4.
    #[arg(long, default_value_t = 3)]
    write_timeout: u64,

    /// List detected serial ports and exit.
    #[arg(long)]
    list_ports: bool,
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

#[derive(Debug, Clone)]
struct AuditRow {
    name: String,
    current: String,
    baseline: String,
    source: String,
    status: String,
    mav_type: common::MavParamType,
}

#[derive(Debug, Clone)]
struct WriteRequest {
    name: String,
    value: f32,
    mav_type: common::MavParamType,
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

fn main() -> Result<()> {
    let args = Args::parse();
    if args.list_ports {
        print_serial_ports()?;
        return Ok(());
    }

    let connect = match args.connect.clone() {
        Some(connect) => connect,
        None => default_connection()?,
    };
    let px4_source = resolve_px4_source(args.px4_source.as_deref())?;

    println!("connecting: {connect}");
    let mut conn =
        open_transport(&connect).with_context(|| format!("failed to connect to {connect}"))?;

    let identity = wait_for_heartbeat(&mut *conn, Duration::from_secs(10))?;
    request_autopilot_version(&mut *conn, &identity)?;
    let version = wait_for_autopilot_version(&mut *conn, Duration::from_secs(3));
    let identity = VehicleIdentity {
        version,
        ..identity
    };

    print_identity(&identity);

    request_params(&mut *conn, &identity)?;
    let params = collect_params(&mut *conn, Duration::from_secs(args.param_timeout))?;
    println!("received {} parameters", params.len());

    let device_sys_autostart = params.get("SYS_AUTOSTART").map(|p| p.value.round() as i32);
    let baseline_sys_autostart = args.sys_autostart.or(device_sys_autostart);
    let wanted_params: HashSet<String> = params.keys().cloned().collect();
    let firmware_defaults = load_firmware_defaults(&px4_source, &wanted_params)?;
    let airframe_defaults = if let Some(id) = baseline_sys_autostart {
        load_airframe_defaults(&px4_source, id, &wanted_params)?
    } else {
        HashMap::new()
    };

    let recommendations = build_recommendations(firmware_defaults, airframe_defaults);
    print_baseline(device_sys_autostart, args.sys_autostart, &px4_source)?;
    print_px4_source(&px4_source);
    let rows = build_audit_rows(&params, &recommendations);

    if args.write_diffs || !args.set_params.is_empty() {
        let writes = build_write_plan(&args, &params, &rows)?;
        confirm_writes(&writes, args.yes)?;
        apply_writes(
            &mut *conn,
            &identity,
            writes,
            Duration::from_secs(args.write_timeout),
        )?;
        println!("writes complete");
        return Ok(());
    }

    if args.verbose {
        println!("\nPX4 source: {}", px4_source.display());
    }

    if args.plain || !io::stdout().is_terminal() {
        print_audit_table(&rows);
    } else {
        run_tui(
            &mut *conn,
            &identity,
            rows,
            Duration::from_secs(args.write_timeout),
        )?;
    }

    Ok(())
}

const DEFAULT_BAUD: u32 = 57600;

fn default_connection() -> Result<String> {
    if let Some(port) = discover_px4_serial_port()? {
        return Ok(format!("serial:{port}:{DEFAULT_BAUD}"));
    }

    for port in fallback_serial_paths() {
        if Path::new(&port).exists() {
            return Ok(format!("serial:{port}:{DEFAULT_BAUD}"));
        }
    }

    bail!("could not autodiscover a PX4 USB serial port; pass --connect or run --list-ports")
}

fn resolve_px4_source(explicit: Option<&Path>) -> Result<PathBuf> {
    if let Some(path) = explicit {
        return validate_px4_source(path);
    }

    if let Ok(path) = std::env::var("PX4_PARAM_AUDIT_PX4_SOURCE") {
        return validate_px4_source(Path::new(&path));
    }

    for candidate in [
        PathBuf::from("vendor/PX4-Autopilot"),
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("vendor/PX4-Autopilot"),
    ] {
        if candidate.exists() {
            return validate_px4_source(&candidate);
        }
    }

    bail!(
        "could not find PX4-Autopilot baseline source; initialize the vendor/PX4-Autopilot submodule or pass --px4-source"
    )
}

fn validate_px4_source(path: &Path) -> Result<PathBuf> {
    let source = path.to_path_buf();
    ensure!(
        source.join("src").is_dir() && source.join("ROMFS/px4fmu_common/init.d/airframes").is_dir(),
        "{} does not look like a PX4-Autopilot checkout",
        source.display()
    );
    Ok(source)
}

fn print_px4_source(px4_source: &Path) {
    let git = Command::new("git")
        .arg("-C")
        .arg(px4_source)
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .and_then(|output| {
            output
                .status
                .success()
                .then(|| String::from_utf8_lossy(&output.stdout).trim().to_string())
        })
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".into());
    println!("PX4 baseline source: {} git={git}", px4_source.display());
}

fn discover_px4_serial_port() -> Result<Option<String>> {
    let mut candidates = serialport::available_ports()
        .context("failed to list serial ports")?
        .into_iter()
        .filter_map(|port| {
            let score = score_serial_port(&port);
            (score > 0).then_some((score, port.port_name))
        })
        .collect::<Vec<_>>();

    candidates.sort_by(|a, b| b.0.cmp(&a.0).then_with(|| a.1.cmp(&b.1)));
    Ok(candidates.into_iter().next().map(|(_, port)| port))
}

fn score_serial_port(port: &SerialPortInfo) -> i32 {
    let mut score = score_port_name(&port.port_name);
    if let SerialPortType::UsbPort(usb) = &port.port_type {
        score += score_usb_info(usb);
    }
    score
}

fn score_usb_info(usb: &UsbPortInfo) -> i32 {
    let mut score = 0;
    let text = format!(
        "{} {}",
        usb.manufacturer.as_deref().unwrap_or_default(),
        usb.product.as_deref().unwrap_or_default()
    )
    .to_lowercase();

    if usb.vid == 12677 {
        score += 800;
    }
    if text.contains("px4")
        || text.contains("pixhawk")
        || text.contains("fmu")
        || text.contains("auterion")
        || text.contains("holybro")
    {
        score += 1000;
    }
    score
}

fn score_port_name(port_name: &str) -> i32 {
    let name = port_name.to_lowercase();
    let mut score = 0;
    if name.contains("usbmodem") {
        score += 300;
    }
    if name.contains("ttyacm") {
        score += 250;
    }
    if name.contains("/dev/serial/by-id") {
        score += 220;
    }
    if name.contains("ttyusb") || name.contains("usbserial") {
        score += 100;
    }
    if score > 0 && name.contains("/dev/cu.") {
        score += 20;
    }
    score
}

fn fallback_serial_paths() -> Vec<String> {
    let mut paths = vec![
        "/dev/cu.usbmodem01".to_string(),
        "/dev/tty.usbmodem01".to_string(),
        "/dev/ttyACM0".to_string(),
        "/dev/ttyACM1".to_string(),
        "/dev/ttyUSB0".to_string(),
        "/dev/ttyUSB1".to_string(),
    ];

    if let Ok(entries) = fs::read_dir("/dev/serial/by-id") {
        let mut by_id = entries
            .flatten()
            .map(|entry| entry.path().display().to_string())
            .collect::<Vec<_>>();
        by_id.sort();
        paths.extend(by_id);
    }

    paths
}

fn print_serial_ports() -> Result<()> {
    let ports = serialport::available_ports().context("failed to list serial ports")?;
    if ports.is_empty() {
        println!("no serial ports detected");
        return Ok(());
    }

    for port in ports {
        let score = score_serial_port(&port);
        match port.port_type {
            SerialPortType::UsbPort(usb) => {
                println!(
                    "{} score={} usb vid={:#06x} pid={:#06x} manufacturer={} product={} serial={}",
                    port.port_name,
                    score,
                    usb.vid,
                    usb.pid,
                    usb.manufacturer.as_deref().unwrap_or("-"),
                    usb.product.as_deref().unwrap_or("-"),
                    usb.serial_number.as_deref().unwrap_or("-"),
                );
            }
            other => println!("{} score={} {:?}", port.port_name, score, other),
        }
    }
    Ok(())
}

trait MavTransport {
    fn send(&mut self, msg: &common::MavMessage) -> Result<()>;
    fn recv(&mut self) -> Option<(MavHeader, common::MavMessage)>;
}

struct FrameDecoder {
    rx_buf: Vec<u8>,
}

impl FrameDecoder {
    fn new() -> Self {
        Self {
            rx_buf: Vec::with_capacity(4096),
        }
    }

    fn extend(&mut self, bytes: &[u8]) {
        self.rx_buf.extend_from_slice(bytes);
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

struct MavSerial {
    port: Box<dyn SerialPort>,
    decoder: FrameDecoder,
    sequence: u8,
}

impl MavSerial {
    fn open(path: &str, baud: u32) -> Result<Self> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(10))
            .open()?;
        Ok(Self {
            port,
            decoder: FrameDecoder::new(),
            sequence: 0,
        })
    }
}

impl MavTransport for MavSerial {
    fn send(&mut self, msg: &common::MavMessage) -> Result<()> {
        write_mavlink_message(&mut self.port, &mut self.sequence, msg)?;
        Ok(())
    }

    fn recv(&mut self) -> Option<(MavHeader, common::MavMessage)> {
        self.read_available();
        self.decoder.decode_next_frame()
    }
}

impl MavSerial {
    fn read_available(&mut self) {
        let Ok(available) = self.port.bytes_to_read() else {
            return;
        };
        if available == 0 {
            return;
        }

        let mut buf = vec![0_u8; (available as usize).min(8192)];
        match self.port.read(&mut buf) {
            Ok(n) => self.decoder.extend(&buf[..n]),
            Err(err) if matches!(err.kind(), ErrorKind::TimedOut | ErrorKind::WouldBlock) => {}
            Err(_) => {}
        }
    }
}

struct MavTcp {
    stream: TcpStream,
    decoder: FrameDecoder,
    sequence: u8,
}

impl MavTcp {
    fn open(addr: &str) -> Result<Self> {
        let stream = TcpStream::connect(resolve_socket_addr(addr)?)?;
        stream.set_nonblocking(true)?;
        Ok(Self {
            stream,
            decoder: FrameDecoder::new(),
            sequence: 0,
        })
    }

    fn read_available(&mut self) {
        let mut buf = [0_u8; 4096];
        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => self.decoder.extend(&buf[..n]),
                Err(err) if matches!(err.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {
                    break;
                }
                Err(_) => break,
            }
        }
    }
}

impl MavTransport for MavTcp {
    fn send(&mut self, msg: &common::MavMessage) -> Result<()> {
        self.stream.set_nonblocking(false)?;
        let result = write_mavlink_message(&mut self.stream, &mut self.sequence, msg);
        self.stream.set_nonblocking(true)?;
        result
    }

    fn recv(&mut self) -> Option<(MavHeader, common::MavMessage)> {
        self.read_available();
        self.decoder.decode_next_frame()
    }
}

struct MavUdp {
    socket: UdpSocket,
    peer: Option<SocketAddr>,
    decoder: FrameDecoder,
    sequence: u8,
}

impl MavUdp {
    fn listen(addr: &str) -> Result<Self> {
        let socket = UdpSocket::bind(addr)?;
        socket.set_nonblocking(true)?;
        Ok(Self {
            socket,
            peer: None,
            decoder: FrameDecoder::new(),
            sequence: 0,
        })
    }

    fn connect(addr: &str) -> Result<Self> {
        let peer = resolve_socket_addr(addr)?;
        let socket = UdpSocket::bind("0.0.0.0:0")?;
        socket.set_nonblocking(true)?;
        Ok(Self {
            socket,
            peer: Some(peer),
            decoder: FrameDecoder::new(),
            sequence: 0,
        })
    }

    fn read_available(&mut self) {
        let mut buf = [0_u8; 4096];
        loop {
            match self.socket.recv_from(&mut buf) {
                Ok((n, addr)) => {
                    self.peer = Some(addr);
                    self.decoder.extend(&buf[..n]);
                }
                Err(err) if err.kind() == ErrorKind::WouldBlock => break,
                Err(_) => break,
            }
        }
    }
}

impl MavTransport for MavUdp {
    fn send(&mut self, msg: &common::MavMessage) -> Result<()> {
        let Some(peer) = self.peer else {
            return Ok(());
        };
        let mut buf = Vec::with_capacity(280);
        write_mavlink_message(&mut buf, &mut self.sequence, msg)?;
        self.socket.send_to(&buf, peer)?;
        Ok(())
    }

    fn recv(&mut self) -> Option<(MavHeader, common::MavMessage)> {
        self.read_available();
        self.decoder.decode_next_frame()
    }
}

fn write_mavlink_message<W: Write>(
    writer: &mut W,
    sequence: &mut u8,
    msg: &common::MavMessage,
) -> Result<()> {
    let header = MavHeader {
        system_id: 255,
        component_id: 190,
        sequence: *sequence,
    };
    *sequence = sequence.wrapping_add(1);
    mavlink::write_versioned_msg(writer, MavlinkVersion::V2, header, msg)?;
    Ok(())
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

fn open_transport(connect: &str) -> Result<Box<dyn MavTransport>> {
    if connect.starts_with("serial:") {
        let (path, baud) = parse_serial_connection(connect)?;
        return Ok(Box::new(MavSerial::open(path, baud)?));
    }
    if let Some(addr) = connect.strip_prefix("udp-listen:") {
        return Ok(Box::new(MavUdp::listen(addr)?));
    }
    if let Some(addr) = connect.strip_prefix("udp-connect:") {
        return Ok(Box::new(MavUdp::connect(addr)?));
    }
    if let Some(addr) = connect.strip_prefix("udp:") {
        return Ok(Box::new(MavUdp::connect(addr)?));
    }
    if let Some(addr) = connect.strip_prefix("tcp:") {
        return Ok(Box::new(MavTcp::open(addr)?));
    }
    bail!(
        "unsupported connection string; use serial:<path>:<baud>, udp-listen:<ip>:<port>, udp-connect:<host>:<port>, udp:<host>:<port>, or tcp:<host>:<port>"
    )
}

fn parse_serial_connection(connect: &str) -> Result<(&str, u32)> {
    let Some(rest) = connect.strip_prefix("serial:") else {
        bail!("serial connection must start with serial:");
    };
    let Some((path, baud)) = rest.rsplit_once(':') else {
        bail!("serial connection must look like serial:/dev/cu.usbmodem01:57600");
    };
    Ok((path, baud.parse()?))
}

fn resolve_socket_addr(addr: &str) -> Result<SocketAddr> {
    addr.to_socket_addrs()?
        .next()
        .with_context(|| format!("could not resolve socket address {addr}"))
}

fn wait_for_heartbeat(conn: &mut dyn MavTransport, timeout: Duration) -> Result<VehicleIdentity> {
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

fn send_gcs_heartbeat(conn: &mut dyn MavTransport) -> Result<()> {
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

fn request_autopilot_version(
    conn: &mut dyn MavTransport,
    identity: &VehicleIdentity,
) -> Result<()> {
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
    conn: &mut dyn MavTransport,
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

fn request_params(conn: &mut dyn MavTransport, identity: &VehicleIdentity) -> Result<()> {
    let msg = common::MavMessage::PARAM_REQUEST_LIST(common::PARAM_REQUEST_LIST_DATA {
        target_system: identity.system_id,
        target_component: identity.component_id,
    });
    conn.send(&msg)?;
    Ok(())
}

fn collect_params(
    conn: &mut dyn MavTransport,
    timeout: Duration,
) -> Result<HashMap<String, ParamValue>> {
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

fn build_write_plan(
    args: &Args,
    params: &HashMap<String, ParamValue>,
    rows: &[AuditRow],
) -> Result<Vec<WriteRequest>> {
    let mut writes: HashMap<String, WriteRequest> = HashMap::new();

    if args.write_diffs {
        for row in rows {
            if row.status != "diff" {
                continue;
            }
            let Ok(value) = parse_write_value(&row.baseline) else {
                continue;
            };
            writes.insert(
                row.name.clone(),
                WriteRequest {
                    name: row.name.clone(),
                    value,
                    mav_type: row.mav_type,
                    source: format!("PX4 baseline ({})", row.source),
                },
            );
        }
    }

    for spec in &args.set_params {
        let (name, value) = parse_set_spec(spec)?;
        let current = params.get(&name).with_context(|| {
            format!("cannot set unknown parameter {name}; it was not reported by PX4")
        })?;
        writes.insert(
            name.clone(),
            WriteRequest {
                name,
                value,
                mav_type: current.mav_type,
                source: "--set".into(),
            },
        );
    }

    let mut writes: Vec<_> = writes.into_values().collect();
    writes.sort_by(|a, b| a.name.cmp(&b.name));
    ensure!(
        !writes.is_empty(),
        "no writable parameters selected; known diffs may have nonnumeric baselines"
    );
    Ok(writes)
}

fn parse_set_spec(spec: &str) -> Result<(String, f32)> {
    let Some((name, value)) = spec.split_once('=') else {
        bail!("--set must look like PARAM=VALUE");
    };
    let name = name.trim();
    ensure!(!name.is_empty(), "--set parameter name cannot be empty");
    ensure!(
        name.len() <= 16,
        "MAVLink parameter names are at most 16 bytes: {name}"
    );
    Ok((name.to_string(), parse_write_value(value.trim())?))
}

fn parse_write_value(value: &str) -> Result<f32> {
    ensure!(
        value != "<unknown>" && !value.contains(','),
        "not a single numeric value: {value}"
    );
    Ok(value.parse::<f32>()?)
}

fn confirm_writes(writes: &[WriteRequest], yes: bool) -> Result<()> {
    println!("planned writes:");
    for write in writes {
        println!(
            "  {} = {} ({:?}, {})",
            write.name, write.value, write.mav_type, write.source
        );
    }

    if yes {
        return Ok(());
    }

    ensure!(
        io::stdin().is_terminal(),
        "refusing to prompt on non-interactive stdin; pass --yes to confirm writes"
    );

    print!(
        "write {} parameter(s) to PX4? Type 'yes' to continue: ",
        writes.len()
    );
    io::stdout().flush()?;
    let mut answer = String::new();
    io::stdin().read_line(&mut answer)?;
    ensure!(answer.trim() == "yes", "write cancelled");
    Ok(())
}

fn apply_writes(
    conn: &mut dyn MavTransport,
    identity: &VehicleIdentity,
    writes: Vec<WriteRequest>,
    timeout: Duration,
) -> Result<()> {
    for write in writes {
        send_param_set(conn, identity, &write)?;
        let confirmed = wait_for_param_confirmation(conn, &write.name, timeout)
            .with_context(|| format!("PX4 did not confirm {}", write.name))?;
        println!("set {} = {}", write.name, format_param(&confirmed));
    }
    Ok(())
}

fn send_param_set(
    conn: &mut dyn MavTransport,
    identity: &VehicleIdentity,
    write: &WriteRequest,
) -> Result<()> {
    let msg = common::MavMessage::PARAM_SET(common::PARAM_SET_DATA {
        target_system: identity.system_id,
        target_component: identity.component_id,
        param_id: write.name.as_str().into(),
        param_value: write.value,
        param_type: write.mav_type,
    });
    conn.send(&msg)?;
    Ok(())
}

fn wait_for_param_confirmation(
    conn: &mut dyn MavTransport,
    name: &str,
    timeout: Duration,
) -> Result<ParamValue> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match conn.recv() {
            Some((_, common::MavMessage::PARAM_VALUE(param))) => {
                let param_name = param.param_id.to_str().unwrap_or("");
                if param_name == name {
                    return Ok(ParamValue {
                        value: param.param_value,
                        mav_type: param.param_type,
                    });
                }
            }
            Some(_) => {}
            None => thread::sleep(Duration::from_millis(5)),
        }
    }
    bail!("timed out waiting for PARAM_VALUE");
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

fn load_firmware_defaults(
    px4_source: &Path,
    wanted: &HashSet<String>,
) -> Result<HashMap<String, String>> {
    let mut defaults = HashMap::new();
    let src = px4_source.join("src");
    visit_yaml_files(&src, &mut |path| {
        if let Ok(text) = fs::read_to_string(path) {
            if let Ok(yaml) = serde_yaml::from_str::<Value>(&text) {
                collect_yaml_defaults(&yaml, wanted, &mut defaults);
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
    wanted: &HashSet<String>,
    defaults: &mut HashMap<String, String>,
) {
    match value {
        Value::Mapping(map) => {
            if let Some(Value::Mapping(definitions)) = map.get(Value::String("definitions".into()))
            {
                for (key, body) in definitions {
                    if let Some(name) = key.as_str() {
                        if wanted.contains(name) {
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
                    if wanted.contains(name) {
                        defaults.insert(name.to_string(), yaml_scalar_to_string(default));
                    }
                }
            }
            for child in map.values() {
                collect_yaml_defaults(child, wanted, defaults);
            }
        }
        Value::Sequence(seq) => {
            for child in seq {
                collect_yaml_defaults(child, wanted, defaults);
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
        Value::Sequence(seq) => seq
            .iter()
            .map(yaml_scalar_to_string)
            .collect::<Vec<_>>()
            .join(","),
        _ => format!("{value:?}"),
    }
}

fn load_airframe_defaults(
    px4_source: &Path,
    sys_autostart: i32,
    wanted: &HashSet<String>,
) -> Result<HashMap<String, String>> {
    let airframes = px4_source.join("ROMFS/px4fmu_common/init.d/airframes");
    let Some(path) = find_airframe_script(&airframes, sys_autostart)? else {
        return Ok(HashMap::new());
    };
    let mut visited = HashSet::new();
    let mut defaults = parse_airframe_script(px4_source, &path, wanted, &mut visited)?;
    if wanted.contains("SYS_AUTOSTART") {
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
    wanted: &HashSet<String>,
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
            if wanted.contains(name) {
                defaults.insert(name.to_string(), parts[3].to_string());
            }
        } else if parts.len() >= 2 && parts[0] == "." {
            if let Some(source_path) = resolve_px4_init_source(px4_source, parts[1]) {
                defaults.extend(parse_airframe_script(
                    px4_source,
                    &source_path,
                    wanted,
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

fn build_audit_rows(
    params: &HashMap<String, ParamValue>,
    recommendations: &HashMap<String, Recommendation>,
) -> Vec<AuditRow> {
    let mut rows: Vec<AuditRow> = params
        .iter()
        .map(|(name, value)| {
            let current = format_param(value);
            let rec = recommendations.get(name);
            let baseline = rec
                .map(|r| r.value.clone())
                .unwrap_or_else(|| "<unknown>".into());
            let source = rec
                .map(|r| r.source.clone())
                .unwrap_or_else(|| "not found".into());
            let status = classify_status(&current, &baseline);
            AuditRow {
                name: name.clone(),
                current,
                baseline,
                source,
                status,
                mav_type: value.mav_type,
            }
        })
        .collect();
    rows.sort_by(|a, b| a.name.cmp(&b.name));
    rows
}

fn print_audit_table(rows: &[AuditRow]) {
    let mut table = PlainTable::new();
    table.set_header(vec!["Param", "Device", "PX4 baseline", "Source", "Status"]);

    for row in rows {
        let color = match row.status.as_str() {
            "match" => Color::Green,
            "diff" => Color::Red,
            "unknown" => Color::DarkGrey,
            _ => Color::White,
        };
        table.add_row(vec![
            Cell::new(&row.name),
            Cell::new(&row.current),
            Cell::new(&row.baseline),
            Cell::new(&row.source),
            Cell::new(&row.status).fg(color),
        ]);
    }

    println!("\n{table}");
}

struct TuiApp {
    rows: Vec<AuditRow>,
    filtered: Vec<usize>,
    query: String,
    edit_value: String,
    mode: TuiMode,
    selected: usize,
    status_message: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TuiMode {
    Browse,
    Search,
    Edit,
}

enum TuiAction {
    Continue,
    Quit,
    Write(WriteRequest),
}

impl TuiApp {
    fn new(rows: Vec<AuditRow>) -> Self {
        let filtered = (0..rows.len()).collect();
        Self {
            rows,
            filtered,
            query: String::new(),
            edit_value: String::new(),
            mode: TuiMode::Browse,
            selected: 0,
            status_message: "Browse: e/Enter edit selected value, / search, q quit".into(),
        }
    }

    fn selected_row(&self) -> Option<&AuditRow> {
        self.filtered
            .get(self.selected)
            .and_then(|index| self.rows.get(*index))
    }

    fn selected_row_mut(&mut self) -> Option<&mut AuditRow> {
        let index = *self.filtered.get(self.selected)?;
        self.rows.get_mut(index)
    }

    fn apply_filter(&mut self) {
        let query = self.query.to_lowercase();
        self.filtered = self
            .rows
            .iter()
            .enumerate()
            .filter_map(|(index, row)| {
                if query.is_empty()
                    || row.name.to_lowercase().contains(&query)
                    || row.current.to_lowercase().contains(&query)
                    || row.baseline.to_lowercase().contains(&query)
                    || row.source.to_lowercase().contains(&query)
                    || row.status.to_lowercase().contains(&query)
                {
                    Some(index)
                } else {
                    None
                }
            })
            .collect();
        self.clamp_selection();
    }

    fn clamp_selection(&mut self) {
        if self.filtered.is_empty() {
            self.selected = 0;
        } else if self.selected >= self.filtered.len() {
            self.selected = self.filtered.len() - 1;
        }
    }

    fn move_down(&mut self, amount: usize) {
        if !self.filtered.is_empty() {
            self.selected = (self.selected + amount).min(self.filtered.len() - 1);
        }
    }

    fn move_up(&mut self, amount: usize) {
        self.selected = self.selected.saturating_sub(amount);
    }

    fn start_edit(&mut self) {
        if let Some(row) = self.selected_row() {
            let name = row.name.clone();
            let current = row.current.clone();
            self.edit_value = current;
            self.mode = TuiMode::Edit;
            self.status_message = format!("Editing {name}. Enter writes, Esc cancels.");
        }
    }

    fn update_selected_value(&mut self, confirmed: ParamValue) {
        if let Some(row) = self.selected_row_mut() {
            row.current = format_param(&confirmed);
            row.mav_type = confirmed.mav_type;
            row.status = classify_status(&row.current, &row.baseline);
            self.status_message = format!("Set {} = {}", row.name, row.current);
        }
        self.apply_filter();
    }
}

fn run_tui(
    conn: &mut dyn MavTransport,
    identity: &VehicleIdentity,
    rows: Vec<AuditRow>,
    write_timeout: Duration,
) -> Result<()> {
    enable_raw_mode()?;
    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;
    let result = run_tui_loop(
        &mut terminal,
        conn,
        identity,
        TuiApp::new(rows),
        write_timeout,
    );
    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;
    result
}

fn run_tui_loop(
    terminal: &mut Terminal<CrosstermBackend<io::Stdout>>,
    conn: &mut dyn MavTransport,
    identity: &VehicleIdentity,
    mut app: TuiApp,
    write_timeout: Duration,
) -> Result<()> {
    loop {
        terminal.draw(|frame| draw_tui(frame, &app))?;

        if !event::poll(Duration::from_millis(100))? {
            continue;
        }

        let Event::Key(key) = event::read()? else {
            continue;
        };
        match handle_key(&mut app, key) {
            TuiAction::Continue => {}
            TuiAction::Quit => break,
            TuiAction::Write(write) => {
                app.status_message = format!("Writing {} = {}...", write.name, write.value);
                terminal.draw(|frame| draw_tui(frame, &app))?;
                match send_param_set(conn, identity, &write).and_then(|_| {
                    wait_for_param_confirmation(conn, &write.name, write_timeout)
                        .with_context(|| format!("PX4 did not confirm {}", write.name))
                }) {
                    Ok(confirmed) => {
                        app.update_selected_value(confirmed);
                        app.mode = TuiMode::Browse;
                        app.edit_value.clear();
                    }
                    Err(err) => {
                        app.status_message = format!("Write failed: {err:#}");
                        app.mode = TuiMode::Browse;
                    }
                }
            }
        }
    }
    Ok(())
}

fn handle_key(app: &mut TuiApp, key: KeyEvent) -> TuiAction {
    if app.mode == TuiMode::Search {
        match key.code {
            KeyCode::Esc => {
                app.query.clear();
                app.mode = TuiMode::Browse;
                app.apply_filter();
            }
            KeyCode::Enter => app.mode = TuiMode::Browse,
            KeyCode::Backspace => {
                app.query.pop();
                app.apply_filter();
            }
            KeyCode::Char('u') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                app.query.clear();
                app.apply_filter();
            }
            KeyCode::Char(c) => {
                app.query.push(c);
                app.apply_filter();
            }
            _ => {}
        }
        return TuiAction::Continue;
    }

    if app.mode == TuiMode::Edit {
        match key.code {
            KeyCode::Esc => {
                app.mode = TuiMode::Browse;
                app.edit_value.clear();
                app.status_message = "Edit cancelled".into();
            }
            KeyCode::Enter => {
                let Some(row) = app.selected_row() else {
                    return TuiAction::Continue;
                };
                match parse_write_value(app.edit_value.trim()) {
                    Ok(value) => {
                        return TuiAction::Write(WriteRequest {
                            name: row.name.clone(),
                            value,
                            mav_type: row.mav_type,
                            source: "TUI edit".into(),
                        });
                    }
                    Err(err) => app.status_message = format!("Invalid value: {err:#}"),
                }
            }
            KeyCode::Backspace => {
                app.edit_value.pop();
            }
            KeyCode::Char('u') if key.modifiers.contains(KeyModifiers::CONTROL) => {
                app.edit_value.clear();
            }
            KeyCode::Char(c) => {
                app.edit_value.push(c);
            }
            _ => {}
        }
        return TuiAction::Continue;
    }

    match key.code {
        KeyCode::Char('q') => TuiAction::Quit,
        KeyCode::Char('/') => {
            app.mode = TuiMode::Search;
            TuiAction::Continue
        }
        KeyCode::Enter | KeyCode::Char('e') => {
            app.start_edit();
            TuiAction::Continue
        }
        KeyCode::Char('g') => {
            app.selected = 0;
            TuiAction::Continue
        }
        KeyCode::Char('G') => {
            if !app.filtered.is_empty() {
                app.selected = app.filtered.len() - 1;
            }
            TuiAction::Continue
        }
        KeyCode::Down | KeyCode::Char('j') => {
            app.move_down(1);
            TuiAction::Continue
        }
        KeyCode::Up | KeyCode::Char('k') => {
            app.move_up(1);
            TuiAction::Continue
        }
        KeyCode::PageDown => {
            app.move_down(20);
            TuiAction::Continue
        }
        KeyCode::PageUp => {
            app.move_up(20);
            TuiAction::Continue
        }
        KeyCode::Char('d') if key.modifiers.contains(KeyModifiers::CONTROL) => {
            app.move_down(20);
            TuiAction::Continue
        }
        KeyCode::Char('u') if key.modifiers.contains(KeyModifiers::CONTROL) => {
            app.move_up(20);
            TuiAction::Continue
        }
        KeyCode::Esc => {
            app.query.clear();
            app.apply_filter();
            TuiAction::Continue
        }
        _ => TuiAction::Continue,
    }
}

fn draw_tui(frame: &mut ratatui::Frame, app: &TuiApp) {
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Min(6),
            Constraint::Length(4),
        ])
        .split(frame.area());

    let input_title = match app.mode {
        TuiMode::Browse => "Search (/), Edit selected value (e/Enter)",
        TuiMode::Search => "Search (Enter done, Esc clear)",
        TuiMode::Edit => "Edit value (Enter writes, Esc cancels)",
    };
    let input_style = match app.mode {
        TuiMode::Search => Style::default().yellow(),
        TuiMode::Edit => Style::default().cyan(),
        TuiMode::Browse => Style::default(),
    };
    let input_value = match app.mode {
        TuiMode::Edit => app.edit_value.as_str(),
        _ => app.query.as_str(),
    };
    let input = Paragraph::new(input_value)
        .block(Block::default().borders(Borders::ALL).title(input_title))
        .style(input_style);
    frame.render_widget(input, chunks[0]);

    let rows = app.filtered.iter().filter_map(|index| {
        let row = app.rows.get(*index)?;
        Some(Row::new(vec![
            TuiCell::from(row.name.clone()),
            TuiCell::from(row.current.clone()),
            TuiCell::from(row.baseline.clone()),
            TuiCell::from(row.source.clone()),
            TuiCell::from(row.status.clone()).style(status_style(&row.status)),
        ]))
    });

    let title = format!(
        "PX4 Params - {} of {} shown",
        app.filtered.len(),
        app.rows.len()
    );
    let table = Table::new(
        rows,
        [
            Constraint::Length(22),
            Constraint::Length(14),
            Constraint::Length(18),
            Constraint::Length(18),
            Constraint::Min(14),
        ],
    )
    .header(
        Row::new(vec!["Param", "Device", "PX4 baseline", "Source", "Status"])
            .style(Style::default().bold())
            .bottom_margin(1),
    )
    .block(Block::default().borders(Borders::ALL).title(title))
    .row_highlight_style(Style::default().reversed())
    .highlight_symbol(">> ");

    let mut state = TableState::default();
    if !app.filtered.is_empty() {
        state.select(Some(app.selected));
    }
    frame.render_stateful_widget(table, chunks[1], &mut state);

    let detail = app
        .selected_row()
        .map(|row| {
            vec![
                Line::from(vec![
                    Span::styled("Selected: ", Style::default().bold()),
                    Span::raw(row.name.as_str()),
                    Span::raw("  "),
                    Span::styled(row.status.as_str(), status_style(&row.status)),
                ]),
                Line::from(app.status_message.as_str()),
                Line::from("Keys: q quit | e/Enter edit | / search | arrows/j/k scroll | PgUp/PgDn | g/G top/bottom"),
            ]
        })
        .unwrap_or_else(|| {
            vec![
                Line::from("No matching parameters"),
                Line::from(app.status_message.as_str()),
                Line::from("Keys: q quit | / search | Esc clear search"),
            ]
        });

    frame.render_widget(
        Paragraph::new(detail).block(Block::default().borders(Borders::ALL).title("Status")),
        chunks[2],
    );
}

fn status_style(status: &str) -> Style {
    match status {
        "match" => Style::default().green(),
        "diff" => Style::default().red(),
        "unknown" => Style::default().dark_gray(),
        _ => Style::default(),
    }
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

fn classify_status(current: &str, baseline: &str) -> String {
    if baseline == "<unknown>" || current == "<missing>" {
        return "unknown".into();
    }
    let matches = values_equivalent(current, baseline);
    if matches {
        "match".into()
    } else {
        "diff".into()
    }
}

fn values_equivalent(a: &str, b: &str) -> bool {
    if let (Ok(af), Ok(bf)) = (a.parse::<f64>(), b.parse::<f64>()) {
        (af - bf).abs() < 0.0001
    } else {
        a == b
    }
}
