use anyhow::{Context, Result, bail, ensure};
use clap::Parser;
use comfy_table::{Cell, Color, Table as PlainTable};
use crossterm::cursor::Show;
use crossterm::event::{self, Event, KeyCode, KeyEvent, KeyModifiers};
use crossterm::execute;
use crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use mavlink::dialects::common;
use mavlink::{MavHeader, MavlinkVersion, Message, MessageData};
use ratatui::Terminal;
use ratatui::backend::CrosstermBackend;
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::Style;
use ratatui::text::{Line, Span};
use ratatui::widgets::{
    Block, Borders, Cell as TuiCell, Clear, Paragraph, Row, Table, TableState, Wrap,
};
use serde_yaml::Value;
use serialport::{SerialPort, SerialPortInfo, SerialPortType, UsbPortInfo};
use std::collections::{HashMap, HashSet};
use std::fmt;
use std::fs;
use std::io::{self, ErrorKind, IsTerminal, Read, Write};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs, UdpSocket};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};

macro_rules! statusln {
    ($($arg:tt)*) => {
        write_output_line(format_args!($($arg)*))
    };
}

fn write_output_line(args: fmt::Arguments<'_>) {
    write_output_text(&format!("{args}\n"));
}

fn write_output_block(text: &str) {
    let mut text = text.to_string();
    if !text.ends_with('\n') {
        text.push('\n');
    }
    write_output_text(&text);
}

fn write_output_text(text: &str) {
    let mut stdout = io::stdout().lock();
    if io::stdout().is_terminal() {
        let _ = stdout.write_all(text.replace('\n', "\r\n").as_bytes());
    } else {
        let _ = stdout.write_all(text.as_bytes());
    }
    let _ = stdout.flush();
}

#[derive(Parser, Debug)]
#[command(version, about = "PX4 parameter audit tool for multirotors")]
struct Args {
    /// MAVLink connection string. Supports serial, udp-listen, udp-connect/udp, and tcp.
    #[arg(short, long)]
    connect: Option<String>,

    /// PX4-Autopilot source checkout used to derive firmware defaults and airframe defaults.
    #[arg(long)]
    px4_source: Option<PathBuf>,

    /// SSH target for VOXL-specific baseline discovery, for example root@100.116.80.54.
    #[arg(long)]
    voxl_ssh: Option<String>,

    /// Discover VOXL-specific baseline data over ADB. Optionally pass an ADB serial.
    #[arg(long, num_args = 0..=1, default_missing_value = "", value_name = "SERIAL")]
    voxl_adb: Option<Option<String>>,

    /// voxl-px4-params checkout used for VOXL platform baseline defaults.
    #[arg(long)]
    voxl_params_source: Option<PathBuf>,

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
    encoding: ParamEncoding,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum ParamEncoding {
    CCast,
    Bytewise,
}

#[derive(Debug, Clone)]
struct Recommendation {
    value: String,
    source: String,
    fallback: bool,
}

#[derive(Debug, Clone)]
struct BaselineLayer {
    defaults: HashMap<String, String>,
    source: String,
    fallback: bool,
}

#[derive(Debug)]
struct FirmwareDefaults {
    defaults: HashMap<String, String>,
    metadata: HashMap<String, ParamMetadata>,
    serial_config_params: HashSet<String>,
}

#[derive(Debug, Clone, Default)]
struct ParamMetadata {
    param_type: Option<String>,
    short_description: Option<String>,
    unit: Option<String>,
    min: Option<String>,
    max: Option<String>,
    decimal: Option<String>,
    reboot_required: bool,
    values: Vec<ParamChoice>,
    bits: Vec<ParamChoice>,
}

#[derive(Debug, Clone)]
struct ParamChoice {
    value: i64,
    label: String,
}

#[derive(Debug, Clone)]
struct AuditRow {
    name: String,
    current: String,
    baseline: String,
    source: String,
    status: String,
    baseline_fallback: bool,
    mav_type: common::MavParamType,
    metadata: Option<ParamMetadata>,
}

#[derive(Debug, Clone)]
struct WriteRequest {
    name: String,
    value: f32,
    mav_type: common::MavParamType,
    encoding: ParamEncoding,
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

#[derive(Debug, Clone)]
struct VoxlDiscovery {
    transport: String,
    hostname: Option<String>,
    sku: String,
    platform_code: String,
    px4_package: Option<String>,
    params_package: Option<String>,
    params_tree: String,
    params_source: PathBuf,
    platform_key: Option<String>,
    platform_file: PathBuf,
    param_files: Vec<PathBuf>,
    local_px4_package: Option<LocalRepoVersion>,
    local_params_package: Option<LocalRepoVersion>,
}

#[derive(Debug, Clone)]
struct LocalRepoVersion {
    version: String,
    git: String,
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
    let px4_source = resolve_px4_source(&args)?;
    print_voxl_discovery_start(&args);
    let voxl_discovery = discover_voxl_baseline(&args, &px4_source)?;
    print_voxl_discovery_result(voxl_discovery.as_ref());

    statusln!("connecting: {connect}");
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
    let mut params = collect_params(
        &mut *conn,
        &identity,
        Duration::from_secs(args.param_timeout),
    )?;
    statusln!("received {} parameters", params.len());

    let wanted_params: HashSet<String> = params.keys().cloned().collect();
    let firmware_defaults = load_firmware_defaults(&px4_source, &wanted_params)?;
    let serial_port_indexes = load_serial_port_indexes(&px4_source)?;
    let voxl_layers = voxl_discovery
        .as_ref()
        .map(|discovery| load_voxl_param_layers(discovery, &wanted_params))
        .transpose()?
        .unwrap_or_default();
    let encoding_layers = voxl_layers.clone();
    let encoding_recommendations = build_recommendations(
        firmware_defaults.defaults.clone(),
        HashMap::new(),
        &encoding_layers,
        &firmware_defaults.serial_config_params,
        &serial_port_indexes,
        None,
    );
    let preferred_encoding = identity
        .version
        .as_ref()
        .and_then(integer_encoding_from_autopilot_version);
    let integer_encoding = infer_param_encodings(
        &mut params,
        &firmware_defaults.metadata,
        &encoding_recommendations,
        preferred_encoding,
    );
    let device_sys_autostart = decoded_sys_autostart(&params);
    let baseline_sys_autostart = args.sys_autostart.or(device_sys_autostart);
    let airframe_defaults = if let Some(id) = baseline_sys_autostart.filter(|id| *id != 0) {
        load_airframe_defaults(&px4_source, id, &wanted_params)?
    } else {
        HashMap::new()
    };
    let fallback_reason =
        baseline_fallback_reason(baseline_sys_autostart, &airframe_defaults, &wanted_params);
    let final_airframe_defaults = if fallback_reason.is_some() {
        HashMap::new()
    } else {
        airframe_defaults
    };
    let recommendations = build_recommendations(
        firmware_defaults.defaults.clone(),
        final_airframe_defaults,
        &voxl_layers,
        &firmware_defaults.serial_config_params,
        &serial_port_indexes,
        fallback_reason.as_deref(),
    );
    infer_param_encodings(
        &mut params,
        &firmware_defaults.metadata,
        &recommendations,
        Some(integer_encoding),
    );
    statusln!("MAVLink integer parameter encoding: {integer_encoding:?}");
    let baseline_line = baseline_status_line(
        device_sys_autostart,
        args.sys_autostart,
        &px4_source,
        fallback_reason.as_deref(),
    )?;
    statusln!("{baseline_line}");
    let voxl_lines = voxl_status_lines(voxl_discovery.as_ref());
    for line in &voxl_lines {
        statusln!("{line}");
    }
    let px4_source_line = px4_source_status_line(&px4_source);
    statusln!("{px4_source_line}");
    let mut tui_context_lines = vec![
        format!("MAVLink integer encoding: {integer_encoding:?}"),
        baseline_line,
    ];
    tui_context_lines.extend(voxl_lines);
    tui_context_lines.push(px4_source_line);
    let rows = build_audit_rows(
        &params,
        &recommendations,
        &firmware_defaults.metadata,
        integer_encoding,
    );

    if args.write_diffs {
        let writes = build_write_plan(&args, &rows, integer_encoding)?;
        confirm_writes(&writes, args.yes)?;
        apply_writes(
            &mut *conn,
            &identity,
            writes,
            Duration::from_secs(args.write_timeout),
        )?;
        statusln!("writes complete");
        return Ok(());
    }

    if args.verbose {
        write_output_block(&format!("\nPX4 source: {}", px4_source.display()));
    }

    if args.plain || !io::stdout().is_terminal() {
        print_audit_table(&rows);
    } else {
        run_tui(
            &mut *conn,
            &identity,
            rows,
            tui_context_lines,
            integer_encoding,
            Duration::from_secs(args.write_timeout),
        )?;
    }

    Ok(())
}

const DEFAULT_BAUD: u32 = 57600;
const DEFAULT_VOXL_PX4_SOURCE: &str = "vendor/voxl-px4/px4-firmware";
const DEFAULT_VOXL_PARAMS_SOURCE: &str = "vendor/voxl-px4-params";
const VOXL_DISCOVERY_SCRIPT: &str = r#"hostname 2>/dev/null | sed 's/^/hostname=/'
if [ -f /data/modalai/sku.txt ]; then sed 's/^/sku=/' /data/modalai/sku.txt; fi
voxl-version 2>/dev/null | awk '$1=="voxl-px4"{print "voxl_px4="$2} $1=="voxl-px4-params"{print "voxl_px4_params="$2}'
for d in /usr/share/modalai/px4_params/v*; do [ -d "$d" ] && basename "$d" | sed 's/^/param_tree=/'; done
if command -v voxl-configure-px4-params >/dev/null 2>&1; then python3 - <<'PY' 2>/dev/null
import ast
import re

script = open('/usr/bin/voxl-configure-px4-params', encoding='utf-8').read()
sku = open('/data/modalai/sku.txt', encoding='utf-8').read().strip()
match = re.search(r'PLATFORM_DIC\s*=\s*(\{.*?\n\})\n\nCAL_PARAMS_PATH', script, re.S)
if match:
    platform_dic = ast.literal_eval(match.group(1))
    for key in sorted(platform_dic, key=len, reverse=True):
        if sku.startswith(key):
            print(f'platform_key={key}')
            for path in platform_dic[key]:
                print(f'param_file={path}')
            break
PY
fi
"#;

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

fn resolve_px4_source(args: &Args) -> Result<PathBuf> {
    if let Some(path) = args.px4_source.as_deref() {
        return validate_px4_source(path);
    }

    if args.voxl_requested() {
        let default_voxl_source = PathBuf::from(DEFAULT_VOXL_PX4_SOURCE);
        if default_voxl_source.exists() {
            return validate_px4_source(&default_voxl_source);
        }
        let manifest_voxl_source =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(DEFAULT_VOXL_PX4_SOURCE);
        if manifest_voxl_source.exists() {
            return validate_px4_source(&manifest_voxl_source);
        }
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

impl Args {
    fn voxl_requested(&self) -> bool {
        self.voxl_ssh.is_some() || self.voxl_adb.is_some()
    }
}

fn discover_voxl_baseline(args: &Args, px4_source: &Path) -> Result<Option<VoxlDiscovery>> {
    if args.voxl_ssh.is_some() && args.voxl_adb.is_some() {
        bail!("pass only one of --voxl-ssh or --voxl-adb");
    }

    let Some((transport, text)) = run_voxl_discovery(args)? else {
        return Ok(None);
    };
    let fields = parse_key_value_lines(&text);
    let sku = fields
        .get("sku")
        .cloned()
        .filter(|value| !value.is_empty())
        .context("VOXL discovery did not report /data/modalai/sku.txt")?;
    let platform_code = extract_modalai_platform_code(&sku)
        .with_context(|| format!("could not derive ModalAI platform code from SKU {sku:?}"))?;
    let px4_package = fields.get("voxl_px4").cloned();
    let params_package = fields.get("voxl_px4_params").cloned();
    let param_trees = fields
        .get("param_tree")
        .into_iter()
        .flat_map(|value| value.split(','))
        .map(str::trim)
        .filter(|value| !value.is_empty())
        .map(ToOwned::to_owned)
        .collect::<Vec<_>>();
    let params_tree = derive_voxl_params_tree(px4_package.as_deref(), &param_trees)?;
    let params_source = resolve_voxl_params_source(args.voxl_params_source.as_deref())?;
    let platform_key = fields.get("platform_key").cloned();
    let discovered_param_files = fields
        .get("param_file")
        .into_iter()
        .flat_map(|value| value.split(','))
        .map(str::trim)
        .filter(|value| !value.is_empty())
        .map(ToOwned::to_owned)
        .collect::<Vec<_>>();
    let param_files = resolve_voxl_param_stack(
        &params_source,
        &params_tree,
        &platform_code,
        &discovered_param_files,
    )?;
    let platform_file = param_files
        .iter()
        .find(|path| {
            path.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| name.starts_with(&platform_code))
        })
        .cloned()
        .unwrap_or_else(|| param_files[0].clone());
    let local_px4_package = infer_voxl_wrapper_source(px4_source)
        .and_then(|path| local_repo_version(&path, px4_package.as_deref()));
    let local_params_package = local_repo_version(&params_source, params_package.as_deref());

    Ok(Some(VoxlDiscovery {
        transport,
        hostname: fields.get("hostname").cloned(),
        sku,
        platform_code,
        px4_package,
        params_package,
        params_tree,
        params_source,
        platform_key,
        platform_file,
        param_files,
        local_px4_package,
        local_params_package,
    }))
}

fn print_voxl_discovery_start(args: &Args) {
    match (args.voxl_ssh.as_deref(), args.voxl_adb.as_ref()) {
        (Some(target), None) => statusln!("VOXL discovery: probing ssh:{target}"),
        (None, Some(Some(serial))) if !serial.is_empty() => {
            statusln!("VOXL discovery: probing adb:{serial}")
        }
        (None, Some(_)) => statusln!("VOXL discovery: probing adb"),
        (None, None) => statusln!("VOXL discovery: disabled (generic PX4 baseline mode)"),
        (Some(_), Some(_)) => {}
    }
}

fn print_voxl_discovery_result(discovery: Option<&VoxlDiscovery>) {
    if let Some(discovery) = discovery {
        let hostname = discovery.hostname.as_deref().unwrap_or("unknown");
        statusln!(
            "VOXL discovery: found hostname={} sku={} platform={} params_tree={} stack_files={} platform_file={}",
            hostname,
            discovery.sku,
            discovery.platform_code,
            discovery.params_tree,
            discovery.param_files.len(),
            discovery.platform_file.display()
        );
    }
}

fn run_voxl_discovery(args: &Args) -> Result<Option<(String, String)>> {
    if let Some(target) = args.voxl_ssh.as_deref() {
        let output = Command::new("ssh")
            .args(["-o", "BatchMode=yes", "-o", "ConnectTimeout=8", target])
            .arg(VOXL_DISCOVERY_SCRIPT)
            .output()
            .with_context(|| format!("failed to run VOXL SSH discovery against {target}"))?;
        ensure!(
            output.status.success(),
            "VOXL SSH discovery failed for {target}: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
        return Ok(Some((
            format!("ssh:{target}"),
            String::from_utf8_lossy(&output.stdout).to_string(),
        )));
    }

    if let Some(adb_serial) = args.voxl_adb.as_ref() {
        let mut command = Command::new("adb");
        if let Some(serial) = adb_serial.as_deref().filter(|serial| !serial.is_empty()) {
            command.args(["-s", serial]);
        }
        let output = command
            .args(["shell", VOXL_DISCOVERY_SCRIPT])
            .output()
            .context("failed to run VOXL ADB discovery")?;
        ensure!(
            output.status.success(),
            "VOXL ADB discovery failed: {}",
            String::from_utf8_lossy(&output.stderr).trim()
        );
        let transport = adb_serial
            .as_deref()
            .filter(|serial| !serial.is_empty())
            .map(|serial| format!("adb:{serial}"))
            .unwrap_or_else(|| "adb".into());
        return Ok(Some((
            transport,
            String::from_utf8_lossy(&output.stdout).to_string(),
        )));
    }

    Ok(None)
}

fn parse_key_value_lines(text: &str) -> HashMap<String, String> {
    let mut fields: HashMap<String, String> = HashMap::new();
    for line in text.lines() {
        let Some((key, value)) = line.trim().trim_end_matches('\r').split_once('=') else {
            continue;
        };
        let value = value.trim();
        if value.is_empty() {
            continue;
        }
        fields
            .entry(key.trim().to_string())
            .and_modify(|existing| {
                existing.push(',');
                existing.push_str(value);
            })
            .or_insert_with(|| value.to_string());
    }
    fields
}

fn extract_modalai_platform_code(sku: &str) -> Option<String> {
    let bytes = sku.as_bytes();
    bytes.windows(5).find_map(|window| {
        (window[0] == b'D' && window[1..].iter().all(u8::is_ascii_digit))
            .then(|| String::from_utf8_lossy(window).to_string())
    })
}

fn derive_voxl_params_tree(px4_package: Option<&str>, param_trees: &[String]) -> Result<String> {
    if let Some(package) = px4_package {
        let version = package.split('-').next().unwrap_or(package);
        let parts = version.split('.').collect::<Vec<_>>();
        if parts.len() >= 2
            && parts[0].chars().all(|c| c.is_ascii_digit())
            && parts[1].chars().all(|c| c.is_ascii_digit())
        {
            let candidate = format!("v{}.{}", parts[0], parts[1]);
            if param_trees.is_empty() || param_trees.iter().any(|tree| tree == &candidate) {
                return Ok(candidate);
            }
        }
    }

    let mut trees = param_trees.to_vec();
    trees.sort();
    trees
        .pop()
        .context("VOXL discovery did not report any installed px4_params version directories")
}

fn resolve_voxl_params_source(explicit: Option<&Path>) -> Result<PathBuf> {
    if let Some(path) = explicit {
        return validate_voxl_params_source(path);
    }

    for candidate in [
        PathBuf::from(DEFAULT_VOXL_PARAMS_SOURCE),
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(DEFAULT_VOXL_PARAMS_SOURCE),
    ] {
        if candidate.exists() {
            return validate_voxl_params_source(&candidate);
        }
    }

    bail!(
        "could not find voxl-px4-params source; initialize vendor/voxl-px4-params or pass --voxl-params-source"
    )
}

fn validate_voxl_params_source(path: &Path) -> Result<PathBuf> {
    ensure!(
        path.join("params").is_dir(),
        "{} does not look like a voxl-px4-params checkout",
        path.display()
    );
    Ok(path.to_path_buf())
}

fn find_voxl_platform_file(
    params_source: &Path,
    params_tree: &str,
    platform_code: &str,
) -> Result<PathBuf> {
    let dir = params_source
        .join("params")
        .join(params_tree)
        .join("platforms");
    ensure!(
        dir.is_dir(),
        "VOXL params tree {} was not found under {}",
        params_tree,
        params_source.display()
    );

    let mut matches = fs::read_dir(&dir)?
        .filter_map(|entry| entry.ok().map(|entry| entry.path()))
        .filter(|path| path.extension().is_some_and(|ext| ext == "params"))
        .filter(|path| {
            path.file_name()
                .and_then(|name| name.to_str())
                .is_some_and(|name| name.starts_with(platform_code))
        })
        .collect::<Vec<_>>();
    matches.sort();

    match matches.len() {
        0 => bail!(
            "no VOXL platform params file matching {} in {}",
            platform_code,
            dir.display()
        ),
        1 => Ok(matches.remove(0)),
        _ => bail!(
            "multiple VOXL platform params files match {} in {}: {}",
            platform_code,
            dir.display(),
            matches
                .iter()
                .filter_map(|path| path.file_name().and_then(|name| name.to_str()))
                .collect::<Vec<_>>()
                .join(", ")
        ),
    }
}

fn resolve_voxl_param_stack(
    params_source: &Path,
    params_tree: &str,
    platform_code: &str,
    discovered_param_files: &[String],
) -> Result<Vec<PathBuf>> {
    if !discovered_param_files.is_empty() {
        let mut paths = Vec::new();
        for relative in discovered_param_files {
            let path = params_source
                .join("params")
                .join(params_tree)
                .join(relative);
            ensure!(
                path.is_file(),
                "VOXL discovered params file {} was not found in local source",
                path.display()
            );
            paths.push(path);
        }
        return Ok(paths);
    }

    Ok(vec![find_voxl_platform_file(
        params_source,
        params_tree,
        platform_code,
    )?])
}

fn infer_voxl_wrapper_source(px4_source: &Path) -> Option<PathBuf> {
    if px4_source.file_name().and_then(|name| name.to_str()) == Some("px4-firmware") {
        let parent = px4_source.parent()?;
        if parent.join(".git").exists() || parent.join(".gitmodules").exists() {
            return Some(parent.to_path_buf());
        }
    }

    let default = PathBuf::from(DEFAULT_VOXL_PX4_SOURCE);
    default.parent().and_then(|parent| {
        (parent.join(".git").exists() || parent.join(".gitmodules").exists())
            .then(|| parent.to_path_buf())
    })
}

fn local_repo_version(
    path: &Path,
    preferred_remote_version: Option<&str>,
) -> Option<LocalRepoVersion> {
    let git = git_output(path, &["rev-parse", "--short", "HEAD"])?;
    let tags = git_output(path, &["tag", "--points-at", "HEAD"])
        .unwrap_or_default()
        .lines()
        .map(ToOwned::to_owned)
        .collect::<Vec<_>>();
    let version = choose_local_version(&tags, preferred_remote_version)
        .or_else(|| git_output(path, &["describe", "--tags", "--always", "--dirty"]))
        .unwrap_or_else(|| git.clone());

    Some(LocalRepoVersion { version, git })
}

fn git_output(path: &Path, args: &[&str]) -> Option<String> {
    let output = Command::new("git")
        .arg("-C")
        .arg(path)
        .args(args)
        .output()
        .ok()?;
    output
        .status
        .success()
        .then(|| String::from_utf8_lossy(&output.stdout).trim().to_string())
        .filter(|value| !value.is_empty())
}

fn choose_local_version(tags: &[String], preferred_remote_version: Option<&str>) -> Option<String> {
    if let Some(preferred) = preferred_remote_version {
        if let Some(tag) = tags
            .iter()
            .find(|tag| versions_match(Some(preferred), Some(tag.as_str())))
        {
            return Some(tag.clone());
        }
    }

    tags.iter()
        .find(|tag| tag.starts_with('v'))
        .or_else(|| tags.first())
        .cloned()
}

fn versions_match(device: Option<&str>, local: Option<&str>) -> bool {
    match (device, local) {
        (Some(device), Some(local)) => normalize_version(device) == normalize_version(local),
        _ => false,
    }
}

fn normalize_version(version: &str) -> &str {
    version.trim().strip_prefix('v').unwrap_or(version.trim())
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

fn px4_source_status_line(px4_source: &Path) -> String {
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
    format!("PX4 baseline source: {} git={git}", px4_source.display())
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
        statusln!("no serial ports detected");
        return Ok(());
    }

    for port in ports {
        let score = score_serial_port(&port);
        match port.port_type {
            SerialPortType::UsbPort(usb) => {
                statusln!(
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
            other => statusln!("{} score={} {:?}", port.port_name, score, other),
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
    identity: &VehicleIdentity,
    timeout: Duration,
) -> Result<HashMap<String, ParamValue>> {
    let deadline = Instant::now() + timeout;
    let mut params = HashMap::new();
    let mut received_indices = HashSet::new();
    let mut expected_count = 0usize;
    while Instant::now() < deadline {
        match conn.recv() {
            Some((_, common::MavMessage::PARAM_VALUE(param))) => {
                update_param_collection(
                    param,
                    &mut params,
                    &mut received_indices,
                    &mut expected_count,
                );
                if expected_count > 0 && received_indices.len() >= expected_count {
                    return Ok(params);
                }
            }
            Some(_) => {}
            None => thread::sleep(Duration::from_millis(5)),
        }
    }

    if expected_count > 0 && received_indices.len() < expected_count {
        recover_missing_params(
            conn,
            identity,
            &mut params,
            &mut received_indices,
            expected_count,
            timeout.min(Duration::from_secs(15)),
        )?;
    }

    if params.is_empty() {
        bail!("timed out waiting for PARAM_VALUE messages");
    }
    Ok(params)
}

fn update_param_collection(
    param: common::PARAM_VALUE_DATA,
    params: &mut HashMap<String, ParamValue>,
    received_indices: &mut HashSet<u16>,
    expected_count: &mut usize,
) {
    let name = param.param_id.to_str().unwrap_or("").to_string();
    if name.is_empty() {
        return;
    }
    if param.param_count > 0 {
        *expected_count = (*expected_count).max(param.param_count as usize);
    }
    received_indices.insert(param.param_index);
    params.insert(
        name,
        ParamValue {
            value: param.param_value,
            mav_type: param.param_type,
            encoding: ParamEncoding::CCast,
        },
    );
}

fn recover_missing_params(
    conn: &mut dyn MavTransport,
    identity: &VehicleIdentity,
    params: &mut HashMap<String, ParamValue>,
    received_indices: &mut HashSet<u16>,
    expected_count: usize,
    timeout: Duration,
) -> Result<()> {
    let deadline = Instant::now() + timeout;
    let mut next_request = Instant::now();
    let mut missing_cursor = 0usize;

    while Instant::now() < deadline && received_indices.len() < expected_count {
        if Instant::now() >= next_request {
            request_missing_param_indexes(
                conn,
                identity,
                expected_count,
                received_indices,
                &mut missing_cursor,
            )?;
            next_request = Instant::now() + Duration::from_millis(700);
        }

        match conn.recv() {
            Some((_, common::MavMessage::PARAM_VALUE(param))) => {
                let mut observed_count = expected_count;
                update_param_collection(param, params, received_indices, &mut observed_count);
            }
            Some(_) => {}
            None => thread::sleep(Duration::from_millis(5)),
        }
    }

    Ok(())
}

fn request_missing_param_indexes(
    conn: &mut dyn MavTransport,
    identity: &VehicleIdentity,
    expected_count: usize,
    received_indices: &HashSet<u16>,
    cursor: &mut usize,
) -> Result<()> {
    let count = expected_count.min(i16::MAX as usize);
    let mut sent = 0usize;

    for offset in 0..count {
        let index = (*cursor + offset) % count;
        if received_indices.contains(&(index as u16)) {
            continue;
        }
        request_param_by_index(conn, identity, index as i16)?;
        sent += 1;
        if sent >= 50 {
            *cursor = (index + 1) % count;
            return Ok(());
        }
    }

    *cursor = 0;
    Ok(())
}

fn request_param_by_index(
    conn: &mut dyn MavTransport,
    identity: &VehicleIdentity,
    index: i16,
) -> Result<()> {
    let msg = common::MavMessage::PARAM_REQUEST_READ(common::PARAM_REQUEST_READ_DATA {
        target_system: identity.system_id,
        target_component: identity.component_id,
        param_id: "".into(),
        param_index: index,
    });
    conn.send(&msg)?;
    Ok(())
}

fn build_write_plan(
    args: &Args,
    rows: &[AuditRow],
    write_encoding: ParamEncoding,
) -> Result<Vec<WriteRequest>> {
    let mut writes: HashMap<String, WriteRequest> = HashMap::new();

    if args.write_diffs {
        for row in rows {
            if row.status != "diff" {
                continue;
            }
            if row.baseline_fallback {
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
                    encoding: write_encoding_for_type(row.mav_type, write_encoding),
                    source: format!("PX4 baseline ({})", row.source),
                },
            );
        }
    }

    let mut writes: Vec<_> = writes.into_values().collect();
    writes.sort_by(|a, b| a.name.cmp(&b.name));
    ensure!(
        !writes.is_empty(),
        "no writable parameters selected; known diffs may have nonnumeric baselines"
    );
    Ok(writes)
}

fn parse_write_value(value: &str) -> Result<f32> {
    ensure!(
        value != "<unknown>" && !value.contains(','),
        "not a single numeric value: {value}"
    );
    Ok(value.parse::<f32>()?)
}

fn confirm_writes(writes: &[WriteRequest], yes: bool) -> Result<()> {
    statusln!("planned writes:");
    for write in writes {
        statusln!(
            "  {} = {} ({:?}, {:?}, {})",
            write.name,
            write.value,
            write.mav_type,
            write.encoding,
            write.source
        );
    }

    if yes {
        return Ok(());
    }

    ensure!(
        io::stdin().is_terminal(),
        "refusing to prompt on non-interactive stdin; pass --yes to confirm writes"
    );

    write_output_text(&format!(
        "write {} parameter(s) to PX4? Type 'yes' to continue: ",
        writes.len()
    ));
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
        let confirmed = wait_for_param_confirmation(conn, &write, timeout)
            .with_context(|| format!("PX4 did not confirm {}", write.name))?;
        statusln!("set {} = {}", write.name, format_param(&confirmed));
    }
    Ok(())
}

fn send_param_set(
    conn: &mut dyn MavTransport,
    identity: &VehicleIdentity,
    write: &WriteRequest,
) -> Result<()> {
    let param_value = encode_param_set_value(write.value, write.mav_type, write.encoding)
        .with_context(|| format!("cannot encode {} for {:?}", write.name, write.mav_type))?;
    let msg = common::MavMessage::PARAM_SET(common::PARAM_SET_DATA {
        target_system: identity.system_id,
        target_component: identity.component_id,
        param_id: write.name.as_str().into(),
        param_value,
        param_type: write.mav_type,
    });
    conn.send(&msg)?;
    Ok(())
}

fn wait_for_param_confirmation(
    conn: &mut dyn MavTransport,
    write: &WriteRequest,
    timeout: Duration,
) -> Result<ParamValue> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        match conn.recv() {
            Some((_, common::MavMessage::PARAM_VALUE(param))) => {
                let param_name = param.param_id.to_str().unwrap_or("");
                if param_name == write.name {
                    let confirmed = ParamValue {
                        value: param.param_value,
                        mav_type: param.param_type,
                        encoding: write.encoding,
                    };
                    if write_confirmation_matches(write, &confirmed) {
                        return Ok(confirmed);
                    }
                }
            }
            Some(_) => {}
            None => thread::sleep(Duration::from_millis(5)),
        }
    }
    bail!("timed out waiting for PARAM_VALUE");
}

fn write_confirmation_matches(write: &WriteRequest, confirmed: &ParamValue) -> bool {
    let confirmed = ParamValue {
        value: confirmed.value,
        mav_type: write.mav_type,
        encoding: write.encoding,
    };
    let expected = ParamValue {
        value: write.value,
        mav_type: write.mav_type,
        encoding: ParamEncoding::CCast,
    };
    values_equivalent(&format_param(&confirmed), &format_param(&expected))
}

fn print_identity(identity: &VehicleIdentity) {
    statusln!(
        "vehicle: sys={} comp={} autopilot={:?} mav_type={:?}",
        identity.system_id,
        identity.component_id,
        identity.autopilot,
        identity.mav_type
    );
    if let Some(version) = &identity.version {
        statusln!(
            "px4 version: {} board={} vendor={} product={} git={}",
            format_px4_version(version.flight_sw_version),
            version.board_version,
            version.vendor_id,
            version.product_id,
            format_hash(&version.flight_custom_version),
        );
    } else {
        statusln!("px4 version: unavailable via AUTOPILOT_VERSION");
    }
}

fn baseline_status_line(
    device_sys_autostart: Option<i32>,
    override_sys_autostart: Option<i32>,
    px4_source: &Path,
    fallback_reason: Option<&str>,
) -> Result<String> {
    if let Some(reason) = fallback_reason {
        return Ok(format!(
            "Baseline: QGC-style firmware defaults only; airframe-specific defaults not applied ({reason})"
        ));
    }

    Ok(match (device_sys_autostart, override_sys_autostart) {
        (Some(device), Some(override_id)) if device != override_id => {
            format!(
                "Baseline: PX4 airframe {override_id} ({}) via --sys-autostart; device SYS_AUTOSTART={device}",
                airframe_name(px4_source, override_id)?
                    .unwrap_or_else(|| "unknown airframe".into())
            )
        }
        (_, Some(override_id)) => {
            format!(
                "Baseline: PX4 airframe {override_id} ({}) via --sys-autostart",
                airframe_name(px4_source, override_id)?
                    .unwrap_or_else(|| "unknown airframe".into())
            )
        }
        (Some(device), None) if device != 0 => {
            format!(
                "Baseline: PX4 airframe {device} ({}) from device SYS_AUTOSTART",
                airframe_name(px4_source, device)?.unwrap_or_else(|| "unknown airframe".into())
            )
        }
        (Some(device), None) => format!(
            "Baseline: firmware defaults only; device SYS_AUTOSTART={device} does not select an airframe"
        ),
        (None, None) => {
            "Baseline: firmware defaults only; device SYS_AUTOSTART was not received".into()
        }
    })
}

fn voxl_status_lines(discovery: Option<&VoxlDiscovery>) -> Vec<String> {
    let Some(discovery) = discovery else {
        return vec!["VOXL discovery: disabled; using generic PX4 baseline sources".into()];
    };
    let hostname = discovery.hostname.as_deref().unwrap_or("unknown");
    let platform_key = discovery.platform_key.as_deref().unwrap_or("unknown");
    let platform_file = discovery
        .platform_file
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("unknown");
    vec![
        format!(
            "VOXL: {} host={} sku={} platform={} key={} tree={}",
            discovery.transport,
            hostname,
            discovery.sku,
            discovery.platform_code,
            platform_key,
            discovery.params_tree
        ),
        format!(
            "VOXL params: {} file(s); platform source={}",
            discovery.param_files.len(),
            platform_file,
        ),
        voxl_version_status_line(
            "voxl-px4",
            discovery.px4_package.as_deref(),
            discovery.local_px4_package.as_ref(),
        ),
        voxl_version_status_line(
            "voxl-px4-params",
            discovery.params_package.as_deref(),
            discovery.local_params_package.as_ref(),
        ),
    ]
}

fn voxl_version_status_line(
    name: &str,
    device_version: Option<&str>,
    local: Option<&LocalRepoVersion>,
) -> String {
    let device = device_version.unwrap_or("unknown");
    let local_version = local
        .map(|version| version.version.as_str())
        .unwrap_or("unknown");
    let status = if versions_match(
        device_version,
        local.map(|version| version.version.as_str()),
    ) {
        "match"
    } else if device_version.is_some() && local.is_some() {
        "mismatch"
    } else {
        "unknown"
    };
    let local_git = local
        .map(|version| version.git.as_str())
        .unwrap_or("unknown");
    format!("{name}: status={status} device={device} local={local_version} git={local_git}")
}

fn decoded_sys_autostart(params: &HashMap<String, ParamValue>) -> Option<i32> {
    params.get("SYS_AUTOSTART").and_then(|param| {
        if is_integer_param_type(param.mav_type) {
            decode_integer_value(param.value, param.mav_type, param.encoding)
                .and_then(|value| i32::try_from(value).ok())
        } else {
            let value = param.value.round();
            (param.value.is_finite() && (param.value - value).abs() <= 0.0001)
                .then_some(value as i32)
        }
    })
}

fn baseline_fallback_reason(
    baseline_sys_autostart: Option<i32>,
    airframe_defaults: &HashMap<String, String>,
    wanted_params: &HashSet<String>,
) -> Option<String> {
    match baseline_sys_autostart {
        None => Some("device SYS_AUTOSTART was not received".into()),
        Some(0) => Some("SYS_AUTOSTART=0 does not select an airframe".into()),
        Some(id) => {
            if airframe_defaults.is_empty() {
                Some(format!("PX4 airframe {id} was not found"))
            } else if !airframe_defaults
                .keys()
                .any(|name| name != "SYS_AUTOSTART" && wanted_params.contains(name))
            {
                Some(format!(
                    "PX4 airframe {id} has no matching parameter defaults"
                ))
            } else {
                None
            }
        }
    }
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

fn load_firmware_defaults(px4_source: &Path, wanted: &HashSet<String>) -> Result<FirmwareDefaults> {
    let mut defaults = HashMap::new();
    let mut metadata = HashMap::new();
    let mut serial_config_params = HashSet::new();
    let src = px4_source.join("src");
    visit_param_source_files(&src, &mut |path| {
        if let Ok(text) = fs::read_to_string(path) {
            collect_c_param_data(&text, wanted, &mut defaults, &mut metadata);
        }
    })?;
    visit_yaml_files(&src, &mut |path| {
        if let Ok(text) = fs::read_to_string(path) {
            if let Ok(yaml) = serde_yaml::from_str::<Value>(&text) {
                collect_yaml_param_data(
                    &yaml,
                    wanted,
                    &mut defaults,
                    &mut metadata,
                    &mut serial_config_params,
                );
            }
        }
    })?;
    Ok(FirmwareDefaults {
        defaults,
        metadata,
        serial_config_params,
    })
}

fn visit_param_source_files(dir: &Path, f: &mut dyn FnMut(&Path)) -> Result<()> {
    if !dir.exists() {
        return Ok(());
    }
    for entry in fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            visit_param_source_files(&path, f)?;
        } else if path
            .extension()
            .is_some_and(|ext| matches!(ext.to_str(), Some("c" | "cc" | "cpp" | "h" | "hpp")))
        {
            f(&path);
        }
    }
    Ok(())
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

fn collect_yaml_param_data(
    value: &Value,
    wanted: &HashSet<String>,
    defaults: &mut HashMap<String, String>,
    metadata: &mut HashMap<String, ParamMetadata>,
    serial_config_params: &mut HashSet<String>,
) {
    match value {
        Value::Mapping(map) => {
            if let Some(Value::Mapping(definitions)) = map.get(Value::String("definitions".into()))
            {
                for (key, body) in definitions {
                    if let Some(name) = key.as_str() {
                        if wanted.contains(name) {
                            if let Some(body) = body.as_mapping() {
                                if let Some(default) = body.get(Value::String("default".into())) {
                                    defaults
                                        .insert(name.to_string(), yaml_scalar_to_string(default));
                                }
                                metadata.insert(name.to_string(), parse_param_metadata(body));
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
                if let Some(name) = name {
                    if wanted.contains(name) {
                        serial_config_params.insert(name.to_string());
                        if let Some(default) = default {
                            defaults.insert(name.to_string(), yaml_scalar_to_string(default));
                        }
                    }
                }
            }
            for child in map.values() {
                collect_yaml_param_data(child, wanted, defaults, metadata, serial_config_params);
            }
        }
        Value::Sequence(seq) => {
            for child in seq {
                collect_yaml_param_data(child, wanted, defaults, metadata, serial_config_params);
            }
        }
        _ => {}
    }
}

fn parse_param_metadata(body: &serde_yaml::Mapping) -> ParamMetadata {
    let mut metadata = ParamMetadata {
        param_type: yaml_field_string(body, "type"),
        short_description: body
            .get(Value::String("description".into()))
            .and_then(Value::as_mapping)
            .and_then(|description| yaml_field_string(description, "short")),
        unit: yaml_field_string(body, "unit"),
        min: yaml_field_string(body, "min"),
        max: yaml_field_string(body, "max"),
        decimal: yaml_field_string(body, "decimal"),
        reboot_required: body
            .get(Value::String("reboot_required".into()))
            .and_then(Value::as_bool)
            .unwrap_or(false),
        values: parse_param_choices(body.get(Value::String("values".into()))),
        bits: parse_param_choices(
            body.get(Value::String("bit".into()))
                .or_else(|| body.get(Value::String("bitmask".into()))),
        ),
    };
    metadata.values.sort_by_key(|choice| choice.value);
    metadata.bits.sort_by_key(|choice| choice.value);
    metadata
}

fn yaml_field_string(map: &serde_yaml::Mapping, key: &str) -> Option<String> {
    map.get(Value::String(key.into()))
        .map(yaml_scalar_to_string)
}

fn parse_param_choices(value: Option<&Value>) -> Vec<ParamChoice> {
    let Some(Value::Mapping(map)) = value else {
        return Vec::new();
    };

    map.iter()
        .filter_map(|(key, label)| {
            let value = yaml_key_to_i64(key)?;
            Some(ParamChoice {
                value,
                label: yaml_scalar_to_string(label),
            })
        })
        .collect()
}

fn yaml_key_to_i64(value: &Value) -> Option<i64> {
    match value {
        Value::Number(number) => number.as_i64(),
        Value::String(s) => s.parse().ok(),
        _ => None,
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

fn collect_c_param_data(
    text: &str,
    wanted: &HashSet<String>,
    defaults: &mut HashMap<String, String>,
    metadata: &mut HashMap<String, ParamMetadata>,
) {
    let mut in_comment = false;
    let mut comment = Vec::new();
    let mut pending_comment = Vec::new();

    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("/**") {
            in_comment = true;
            comment.clear();
            let rest = trimmed.trim_start_matches("/**").trim();
            if !rest.is_empty() && rest != "*/" {
                comment.push(clean_c_comment_line(rest));
            }
            if trimmed.ends_with("*/") {
                in_comment = false;
                pending_comment = comment.clone();
            }
            continue;
        }

        if in_comment {
            let ends = trimmed.ends_with("*/");
            let content = trimmed.trim_end_matches("*/").trim();
            if !content.is_empty() {
                comment.push(clean_c_comment_line(content));
            }
            if ends {
                in_comment = false;
                pending_comment = comment.clone();
            }
            continue;
        }

        if let Some((macro_type, name, default)) = parse_c_param_define(trimmed) {
            if wanted.contains(&name) {
                defaults.insert(name.clone(), default);
                metadata.insert(name, parse_c_param_metadata(&pending_comment, &macro_type));
            }
            pending_comment.clear();
        }
    }
}

fn clean_c_comment_line(line: &str) -> String {
    line.trim().trim_start_matches('*').trim().to_string()
}

fn parse_c_param_define(line: &str) -> Option<(String, String, String)> {
    let start = line.find("PARAM_DEFINE_")?;
    let rest = &line[start + "PARAM_DEFINE_".len()..];
    let (macro_type, args) = rest.split_once('(')?;
    let args = args.split_once(')').map(|(args, _)| args)?;
    let mut parts = args.splitn(3, ',').map(str::trim);
    let name = parts.next()?.to_string();
    let default = normalize_c_default_value(parts.next()?);
    (!name.is_empty()).then(|| (macro_type.to_ascii_lowercase(), name, default))
}

fn normalize_c_default_value(value: &str) -> String {
    value
        .trim()
        .trim_end_matches(';')
        .trim_end_matches('f')
        .trim_end_matches('F')
        .trim()
        .to_string()
}

fn parse_c_param_metadata(comment: &[String], macro_type: &str) -> ParamMetadata {
    let mut metadata = ParamMetadata {
        param_type: Some(macro_type.to_string()),
        ..ParamMetadata::default()
    };
    let mut description = Vec::new();
    let mut seen_tag = false;

    for line in comment {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            if !description.is_empty() && !seen_tag {
                seen_tag = true;
            }
            continue;
        }

        if let Some(tag) = trimmed.strip_prefix('@') {
            seen_tag = true;
            parse_c_param_tag(tag, &mut metadata);
        } else if !seen_tag {
            description.push(trimmed.to_string());
        }
    }

    if !description.is_empty() {
        metadata.short_description = Some(description.join(" "));
    }
    if !metadata.bits.is_empty() {
        metadata.param_type = Some("bitmask".into());
    } else if !metadata.values.is_empty() {
        metadata.param_type = Some("enum".into());
    }
    metadata.bits.sort_by_key(|choice| choice.value);
    metadata.values.sort_by_key(|choice| choice.value);
    metadata
}

fn parse_c_param_tag(tag: &str, metadata: &mut ParamMetadata) {
    let (name, value) = tag.split_once(char::is_whitespace).unwrap_or((tag, ""));
    let value = value.trim();
    match name {
        "min" => metadata.min = Some(value.to_string()),
        "max" => metadata.max = Some(value.to_string()),
        "unit" => metadata.unit = Some(value.to_string()),
        "decimal" => metadata.decimal = Some(value.to_string()),
        "reboot_required" => metadata.reboot_required = value == "true",
        "bit" => {
            if let Some(choice) = parse_c_param_choice(value) {
                metadata.bits.push(choice);
            }
        }
        "value" => {
            if let Some(choice) = parse_c_param_choice(value) {
                metadata.values.push(choice);
            }
        }
        _ => {}
    }
}

fn parse_c_param_choice(value: &str) -> Option<ParamChoice> {
    let mut parts = value.splitn(2, char::is_whitespace);
    let choice_value = parts.next()?.parse().ok()?;
    let label = parts.next().unwrap_or("").trim().to_string();
    Some(ParamChoice {
        value: choice_value,
        label,
    })
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

fn load_voxl_param_layers(
    discovery: &VoxlDiscovery,
    wanted: &HashSet<String>,
) -> Result<Vec<BaselineLayer>> {
    let mut layers = Vec::new();
    for path in &discovery.param_files {
        let defaults = parse_voxl_params_file(path, wanted)?;
        if defaults.is_empty() {
            continue;
        }
        layers.push(BaselineLayer {
            defaults,
            source: format!(
                "VOXL {} default ({})",
                discovery.platform_code,
                voxl_param_stack_label(discovery, path)
            ),
            fallback: false,
        });
    }
    ensure!(
        !layers.is_empty(),
        "VOXL params stack starting at {} has no defaults matching reported parameters",
        discovery.platform_file.display()
    );
    Ok(layers)
}

fn voxl_param_stack_label(discovery: &VoxlDiscovery, path: &Path) -> String {
    path.strip_prefix(
        discovery
            .params_source
            .join("params")
            .join(&discovery.params_tree),
    )
    .ok()
    .and_then(|path| path.to_str())
    .map(ToOwned::to_owned)
    .or_else(|| {
        path.file_name()
            .and_then(|name| name.to_str())
            .map(ToOwned::to_owned)
    })
    .unwrap_or_else(|| "unknown".into())
}

fn parse_voxl_params_file(
    path: &Path,
    wanted: &HashSet<String>,
) -> Result<HashMap<String, String>> {
    let text = fs::read_to_string(path)
        .with_context(|| format!("failed to read VOXL params file {}", path.display()))?;
    Ok(parse_voxl_params_text(&text, wanted))
}

fn parse_voxl_params_text(text: &str, wanted: &HashSet<String>) -> HashMap<String, String> {
    let mut defaults = HashMap::new();
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let parts = line.split_whitespace().collect::<Vec<_>>();
        if parts.len() < 5 {
            continue;
        }
        let name = parts[2];
        if wanted.contains(name) {
            defaults.insert(name.to_string(), parts[3].to_string());
        }
    }
    defaults
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

fn load_serial_port_indexes(px4_source: &Path) -> Result<HashMap<String, String>> {
    let path = px4_source.join("Tools/serial/generate_config.py");
    if !path.exists() {
        return Ok(HashMap::new());
    }
    let text = fs::read_to_string(path)?;
    Ok(parse_serial_port_indexes(&text))
}

fn parse_serial_port_indexes(text: &str) -> HashMap<String, String> {
    let mut indexes = HashMap::new();
    let mut current_tag: Option<String> = None;

    for line in text.lines() {
        let line = line.trim();
        if line.starts_with('#') {
            continue;
        }

        if let Some((tag, _)) = line
            .strip_prefix('"')
            .and_then(|rest| rest.split_once('"'))
            .filter(|(_, rest)| {
                rest.trim_start()
                    .strip_prefix(':')
                    .is_some_and(|rest| rest.trim_start().starts_with('{'))
            })
        {
            current_tag = Some(tag.to_string());
            continue;
        }

        let Some(tag) = current_tag.as_ref() else {
            continue;
        };
        let Some(raw_index) = line
            .strip_prefix("\"index\"")
            .and_then(|rest| rest.trim_start().strip_prefix(':'))
        else {
            continue;
        };
        let index = raw_index
            .split(',')
            .next()
            .unwrap_or("")
            .trim()
            .trim_matches('\'')
            .trim_matches('"');
        if !index.is_empty() && index.chars().all(|c| c.is_ascii_digit()) {
            indexes.insert(tag.clone(), index.to_string());
        }
    }

    indexes
}

fn build_recommendations(
    firmware: HashMap<String, String>,
    airframe: HashMap<String, String>,
    extra_layers: &[BaselineLayer],
    serial_config_params: &HashSet<String>,
    serial_port_indexes: &HashMap<String, String>,
    fallback_reason: Option<&str>,
) -> HashMap<String, Recommendation> {
    let mut out = HashMap::new();
    let fallback = fallback_reason.is_some();
    let firmware_source = if fallback {
        "QGC-style firmware default"
    } else {
        "firmware default"
    };
    for (name, value) in firmware {
        let (value, label) =
            normalize_default_value(&name, value, serial_config_params, serial_port_indexes);
        out.insert(
            name,
            Recommendation {
                value,
                source: recommendation_source(firmware_source, label.as_deref()),
                fallback,
            },
        );
    }
    for (name, value) in airframe {
        let (value, label) =
            normalize_default_value(&name, value, serial_config_params, serial_port_indexes);
        out.insert(
            name,
            Recommendation {
                value,
                source: recommendation_source("airframe default", label.as_deref()),
                fallback: false,
            },
        );
    }
    for layer in extra_layers {
        for (name, value) in &layer.defaults {
            let (value, label) = normalize_default_value(
                name,
                value.clone(),
                serial_config_params,
                serial_port_indexes,
            );
            out.insert(
                name.clone(),
                Recommendation {
                    value,
                    source: recommendation_source(&layer.source, label.as_deref()),
                    fallback: layer.fallback,
                },
            );
        }
    }
    out
}

fn normalize_default_value(
    name: &str,
    value: String,
    serial_config_params: &HashSet<String>,
    serial_port_indexes: &HashMap<String, String>,
) -> (String, Option<String>) {
    if !serial_config_params.contains(name) {
        return (value, None);
    }

    let trimmed = value.trim();
    match serial_port_indexes.get(trimmed) {
        Some(index) => (index.clone(), Some(trimmed.to_string())),
        None => (value, None),
    }
}

fn recommendation_source(base: &str, label: Option<&str>) -> String {
    match label {
        Some(label) => format!("{base} ({label})"),
        None => base.into(),
    }
}

fn infer_param_encodings(
    params: &mut HashMap<String, ParamValue>,
    metadata: &HashMap<String, ParamMetadata>,
    recommendations: &HashMap<String, Recommendation>,
    preferred: Option<ParamEncoding>,
) -> ParamEncoding {
    let global = preferred
        .unwrap_or_else(|| infer_global_integer_encoding(params, metadata, recommendations));
    for param in params.values_mut() {
        if !is_integer_param_type(param.mav_type) {
            param.encoding = ParamEncoding::CCast;
            continue;
        }
        param.encoding = global;
    }
    global
}

fn integer_encoding_from_autopilot_version(
    version: &common::AUTOPILOT_VERSION_DATA,
) -> Option<ParamEncoding> {
    let bytewise = version
        .capabilities
        .contains(common::MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE);
    let ccast = version
        .capabilities
        .contains(common::MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST);

    match (bytewise, ccast) {
        (true, false) => Some(ParamEncoding::Bytewise),
        (false, true) => Some(ParamEncoding::CCast),
        _ => None,
    }
}

fn infer_global_integer_encoding(
    params: &HashMap<String, ParamValue>,
    metadata: &HashMap<String, ParamMetadata>,
    recommendations: &HashMap<String, Recommendation>,
) -> ParamEncoding {
    let mut bytewise_score = 0_i32;
    let mut ccast_score = 0_i32;

    for (name, param) in params {
        if !is_integer_param_type(param.mav_type) {
            continue;
        }
        let (byte_score, cast_score) =
            encoding_scores(param, metadata.get(name), recommendations.get(name));
        let delta = byte_score - cast_score;
        if delta >= 4 {
            bytewise_score += delta;
        } else if delta <= -4 {
            ccast_score += -delta;
        }
    }

    if bytewise_score > ccast_score {
        ParamEncoding::Bytewise
    } else {
        ParamEncoding::CCast
    }
}

fn encoding_scores(
    param: &ParamValue,
    metadata: Option<&ParamMetadata>,
    recommendation: Option<&Recommendation>,
) -> (i32, i32) {
    let bytewise = decode_integer_value(param.value, param.mav_type, ParamEncoding::Bytewise);
    let ccast = decode_integer_value(param.value, param.mav_type, ParamEncoding::CCast);
    let mut byte_score = bytewise.map(|_| 1).unwrap_or(-10);
    let mut cast_score = ccast.map(|_| 1).unwrap_or(-10);
    let range = metadata.and_then(param_integer_range);

    if let Some((min, max)) = range {
        score_range(&mut byte_score, bytewise, min, max);
        score_range(&mut cast_score, ccast, min, max);
    }

    if let Some(baseline) = recommendation
        .and_then(|rec| parse_integral_display_value(&rec.value))
        .filter(|_| range.is_some() || metadata.is_some())
    {
        if bytewise == Some(baseline) {
            byte_score += 4;
        }
        if ccast == Some(baseline) {
            cast_score += 4;
        }
    }

    if param.value.is_finite() && (param.value - param.value.round()).abs() <= f32::EPSILON {
        cast_score += 2;
    }
    if param.value != 0.0 && param.value.abs() < f32::MIN_POSITIVE {
        byte_score += 2;
    }

    (byte_score, cast_score)
}

fn score_range(score: &mut i32, value: Option<i64>, min: i64, max: i64) {
    match value {
        Some(value) if value >= min && value <= max => *score += 6,
        Some(_) => *score -= 6,
        None => *score -= 6,
    }
}

fn param_integer_range(metadata: &ParamMetadata) -> Option<(i64, i64)> {
    let min = metadata
        .min
        .as_deref()
        .and_then(parse_integral_display_value)
        .unwrap_or(i64::MIN);
    let max = metadata
        .max
        .as_deref()
        .and_then(parse_integral_display_value)
        .unwrap_or(i64::MAX);
    (min <= max).then_some((min, max))
}

fn build_audit_rows(
    params: &HashMap<String, ParamValue>,
    recommendations: &HashMap<String, Recommendation>,
    metadata: &HashMap<String, ParamMetadata>,
    integer_encoding: ParamEncoding,
) -> Vec<AuditRow> {
    let mut rows: Vec<AuditRow> = params
        .iter()
        .map(|(name, value)| {
            let metadata = metadata.get(name).cloned();
            let mav_type = effective_mav_type(value.mav_type, metadata.as_ref());
            let encoding =
                if is_integer_param_type(mav_type) && !is_integer_param_type(value.mav_type) {
                    integer_encoding
                } else {
                    value.encoding
                };
            let current = format_param(&ParamValue {
                value: value.value,
                mav_type,
                encoding,
            });
            let rec = recommendations.get(name);
            let baseline = rec
                .map(|r| r.value.clone())
                .unwrap_or_else(|| "<unknown>".into());
            let source = rec
                .map(|r| r.source.clone())
                .unwrap_or_else(|| "not found".into());
            let baseline_fallback = rec.map(|r| r.fallback).unwrap_or(false);
            let status = classify_status(&current, &baseline);
            AuditRow {
                name: name.clone(),
                current,
                baseline,
                source,
                status,
                baseline_fallback,
                mav_type,
                metadata,
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
        let color = match display_status(row).as_str() {
            "match" => Color::Green,
            "diff" | "fallback match" | "fallback diff" => Color::Red,
            "unknown" => Color::DarkGrey,
            _ => Color::White,
        };
        table.add_row(vec![
            Cell::new(&row.name),
            Cell::new(&row.current),
            Cell::new(&row.baseline),
            Cell::new(&row.source),
            Cell::new(display_status(row)).fg(color),
        ]);
    }

    write_output_block(&format!("\n{table}"));
}

struct TuiApp {
    rows: Vec<AuditRow>,
    filtered: Vec<usize>,
    context_lines: Vec<String>,
    write_encoding: ParamEncoding,
    query: String,
    edit_value: String,
    pending_writes: Vec<WriteRequest>,
    show_diffs_only: bool,
    mode: TuiMode,
    selected: usize,
    status_message: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum TuiMode {
    Browse,
    Search,
    Edit,
    ConfirmDefaults,
    MetadataPopup,
}

enum TuiAction {
    Continue,
    Quit,
    Write(WriteRequest),
    WriteMany(Vec<WriteRequest>),
}

impl TuiApp {
    fn new(rows: Vec<AuditRow>, context_lines: Vec<String>, write_encoding: ParamEncoding) -> Self {
        let filtered = (0..rows.len()).collect();
        Self {
            rows,
            filtered,
            context_lines,
            write_encoding,
            query: String::new(),
            edit_value: String::new(),
            pending_writes: Vec::new(),
            show_diffs_only: false,
            mode: TuiMode::Browse,
            selected: 0,
            status_message: "Browse: f diffs/all, e/Enter edit, d default selected, A default all"
                .into(),
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
                if self.row_is_visible(row) && row_matches_query(row, &query) {
                    Some(index)
                } else {
                    None
                }
            })
            .collect();
        self.clamp_selection();
    }

    fn row_is_visible(&self, row: &AuditRow) -> bool {
        !self.show_diffs_only || row.status == "diff"
    }

    fn view_label(&self) -> &'static str {
        if self.show_diffs_only {
            "diffs only"
        } else {
            "all params"
        }
    }

    fn toggle_diffs_only(&mut self) {
        self.show_diffs_only = !self.show_diffs_only;
        self.apply_filter();
        self.status_message = if self.show_diffs_only {
            format!("Showing {} differing parameter(s)", self.filtered.len())
        } else {
            format!("Showing all {} parameter(s)", self.rows.len())
        };
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
        let write_encoding = self.write_encoding;
        if let Some(row) = self.selected_row_mut() {
            let mav_type = effective_mav_type(confirmed.mav_type, row.metadata.as_ref());
            let encoding = write_encoding_for_type(mav_type, write_encoding);
            row.current = format_param(&ParamValue {
                value: confirmed.value,
                mav_type,
                encoding,
            });
            row.mav_type = mav_type;
            row.status = classify_status(&row.current, &row.baseline);
            self.status_message = format!("Set {} = {}", row.name, row.current);
        }
        self.apply_filter();
    }

    fn update_value_by_name(&mut self, name: &str, confirmed: ParamValue) {
        let write_encoding = self.write_encoding;
        if let Some(row) = self.rows.iter_mut().find(|row| row.name == name) {
            let mav_type = effective_mav_type(confirmed.mav_type, row.metadata.as_ref());
            let encoding = write_encoding_for_type(mav_type, write_encoding);
            row.current = format_param(&ParamValue {
                value: confirmed.value,
                mav_type,
                encoding,
            });
            row.mav_type = mav_type;
            row.status = classify_status(&row.current, &row.baseline);
        }
        self.apply_filter();
    }

    fn selected_default_write(&mut self) -> Option<WriteRequest> {
        let Some(row) = self.selected_row() else {
            return None;
        };
        let name = row.name.clone();
        let baseline = row.baseline.clone();
        let mav_type = row.mav_type;
        let encoding = write_encoding_for_type(row.mav_type, self.write_encoding);
        let source = row.source.clone();
        let current = row.current.clone();

        if values_equivalent(&current, &baseline) {
            self.status_message = format!("{name} already matches PX4 baseline");
            return None;
        }

        if row.baseline_fallback {
            self.status_message =
                format!("Refusing to write fallback QGC-style baseline for {name}");
            return None;
        }

        match parse_write_value(&baseline) {
            Ok(value) => Some(WriteRequest {
                name,
                value,
                mav_type,
                encoding,
                source: format!("TUI default selected ({source})"),
            }),
            Err(_) => {
                self.status_message = format!("No single numeric PX4 baseline for {name}");
                None
            }
        }
    }

    fn default_diff_writes(&self) -> Vec<WriteRequest> {
        self.rows
            .iter()
            .filter(|row| row.status == "diff")
            .filter(|row| !row.baseline_fallback)
            .filter_map(|row| {
                let value = parse_write_value(&row.baseline).ok()?;
                Some(WriteRequest {
                    name: row.name.clone(),
                    value,
                    mav_type: row.mav_type,
                    encoding: write_encoding_for_type(row.mav_type, self.write_encoding),
                    source: format!("TUI default all ({})", row.source),
                })
            })
            .collect()
    }

    fn start_default_all_confirmation(&mut self) {
        self.pending_writes = self.default_diff_writes();
        if self.pending_writes.is_empty() {
            self.status_message =
                "No numeric non-default values have a known PX4 baseline to write".into();
            return;
        }
        self.mode = TuiMode::ConfirmDefaults;
        self.status_message = format!(
            "WARNING: reset {} numeric non-default value(s) on the device? y=yes n=no",
            self.pending_writes.len()
        );
    }
}

fn row_matches_query(row: &AuditRow, query: &str) -> bool {
    query.is_empty()
        || row.name.to_lowercase().contains(query)
        || row.current.to_lowercase().contains(query)
        || row.baseline.to_lowercase().contains(query)
        || row.source.to_lowercase().contains(query)
        || row.status.to_lowercase().contains(query)
}

fn run_tui(
    conn: &mut dyn MavTransport,
    identity: &VehicleIdentity,
    rows: Vec<AuditRow>,
    context_lines: Vec<String>,
    write_encoding: ParamEncoding,
    write_timeout: Duration,
) -> Result<()> {
    let _guard = TuiTerminalGuard::enter()?;
    let backend = CrosstermBackend::new(io::stdout());
    let mut terminal = Terminal::new(backend)?;
    let result = run_tui_loop(
        &mut terminal,
        conn,
        identity,
        TuiApp::new(rows, context_lines, write_encoding),
        write_timeout,
    );
    terminal.show_cursor()?;
    result
}

struct TuiTerminalGuard;

impl TuiTerminalGuard {
    fn enter() -> Result<Self> {
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        if let Err(err) = execute!(stdout, EnterAlternateScreen) {
            let _ = disable_raw_mode();
            return Err(err.into());
        }
        Ok(Self)
    }
}

impl Drop for TuiTerminalGuard {
    fn drop(&mut self) {
        let _ = disable_raw_mode();
        let mut stdout = io::stdout();
        let _ = execute!(stdout, LeaveAlternateScreen, Show);
    }
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
                    wait_for_param_confirmation(conn, &write, write_timeout)
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
            TuiAction::WriteMany(writes) => {
                let total = writes.len();
                let mut written = 0usize;
                app.mode = TuiMode::Browse;
                app.pending_writes.clear();
                for write in writes {
                    app.status_message = format!(
                        "Writing {}/{}: {} = {}...",
                        written + 1,
                        total,
                        write.name,
                        write.value
                    );
                    terminal.draw(|frame| draw_tui(frame, &app))?;
                    match send_param_set(conn, identity, &write).and_then(|_| {
                        wait_for_param_confirmation(conn, &write, write_timeout)
                            .with_context(|| format!("PX4 did not confirm {}", write.name))
                    }) {
                        Ok(confirmed) => {
                            written += 1;
                            app.update_value_by_name(&write.name, confirmed);
                        }
                        Err(err) => {
                            app.status_message = format!(
                                "Stopped after {written}/{total} writes. {} failed: {err:#}",
                                write.name
                            );
                            break;
                        }
                    }
                }
                if written == total {
                    app.status_message = format!("Reset {written} parameter(s) to PX4 baseline");
                }
            }
        }
    }
    Ok(())
}

fn handle_key(app: &mut TuiApp, key: KeyEvent) -> TuiAction {
    if app.mode == TuiMode::MetadataPopup {
        match key.code {
            KeyCode::Char('q') => return TuiAction::Quit,
            KeyCode::Esc | KeyCode::Enter | KeyCode::Char('?') | KeyCode::Char('m') => {
                app.mode = TuiMode::Browse;
                app.status_message = "Metadata popup closed".into();
            }
            _ => {}
        }
        return TuiAction::Continue;
    }

    if app.mode == TuiMode::ConfirmDefaults {
        match key.code {
            KeyCode::Char('y') | KeyCode::Char('Y') => {
                let writes = app.pending_writes.clone();
                return TuiAction::WriteMany(writes);
            }
            KeyCode::Char('n') | KeyCode::Char('N') | KeyCode::Esc => {
                app.pending_writes.clear();
                app.mode = TuiMode::Browse;
                app.status_message = "Default-all write cancelled".into();
            }
            _ => {}
        }
        return TuiAction::Continue;
    }

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
                            encoding: write_encoding_for_type(row.mav_type, app.write_encoding),
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
        KeyCode::Char('?') | KeyCode::Char('m') => {
            if app.selected_row().is_some_and(row_has_metadata_choices) {
                app.mode = TuiMode::MetadataPopup;
            } else {
                app.status_message =
                    "No enum, boolean, or bitmask metadata for selected parameter".into();
            }
            TuiAction::Continue
        }
        KeyCode::Char('f') if key.modifiers.is_empty() => {
            app.toggle_diffs_only();
            TuiAction::Continue
        }
        KeyCode::Enter | KeyCode::Char('e') => {
            app.start_edit();
            TuiAction::Continue
        }
        KeyCode::Char('d') if key.modifiers.is_empty() => match app.selected_default_write() {
            Some(write) => TuiAction::Write(write),
            None => TuiAction::Continue,
        },
        KeyCode::Char('A') if key.modifiers.is_empty() || key.modifiers == KeyModifiers::SHIFT => {
            app.start_default_all_confirmation();
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
    let context_height = if app.context_lines.len() >= 6 {
        10
    } else {
        (app.context_lines.len() as u16 + 2).clamp(3, 6)
    };
    let status_height = if context_height >= 10 { 5 } else { 8 };
    let chunks = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),
            Constraint::Length(context_height),
            Constraint::Min(6),
            Constraint::Length(status_height),
        ])
        .split(frame.area());

    let input_title = match app.mode {
        TuiMode::Browse => {
            "Search (/), Toggle diffs/all (f), Edit (e/Enter), Default selected (d), Default all (A)"
        }
        TuiMode::Search => "Search (Enter done, Esc clear)",
        TuiMode::Edit => "Edit value (Enter writes, Esc cancels)",
        TuiMode::ConfirmDefaults => "Reset all numeric diffs to PX4 baseline? y/n",
        TuiMode::MetadataPopup => "Metadata choices (Enter/Esc closes)",
    };
    let input_style = match app.mode {
        TuiMode::Search => Style::default().yellow(),
        TuiMode::Edit => Style::default().cyan(),
        TuiMode::ConfirmDefaults => Style::default().red().bold(),
        TuiMode::MetadataPopup => Style::default().green(),
        TuiMode::Browse => Style::default(),
    };
    let input_value = match app.mode {
        TuiMode::Edit => app.edit_value.as_str(),
        TuiMode::ConfirmDefaults => {
            "WARNING: all numeric non-default values will be reset on the device"
        }
        TuiMode::MetadataPopup => "Enum/bitmask metadata popup is open",
        _ => app.query.as_str(),
    };
    let input = Paragraph::new(input_value)
        .block(Block::default().borders(Borders::ALL).title(input_title))
        .style(input_style);
    frame.render_widget(input, chunks[0]);

    let context = app
        .context_lines
        .iter()
        .map(|line| baseline_context_line(line))
        .collect::<Vec<_>>();
    frame.render_widget(
        Paragraph::new(context)
            .block(Block::default().borders(Borders::ALL).title("Baseline"))
            .wrap(Wrap { trim: false }),
        chunks[1],
    );

    let rows = app.filtered.iter().filter_map(|index| {
        let row = app.rows.get(*index)?;
        Some(Row::new(vec![
            TuiCell::from(row.name.clone()),
            TuiCell::from(row.current.clone()),
            TuiCell::from(row.baseline.clone()),
            TuiCell::from(row.source.clone()).style(source_style(row)),
            TuiCell::from(display_status(row)).style(status_style_for_row(row)),
        ]))
    });

    let title = format!(
        "PX4 Params - {} - {} of {} shown",
        app.view_label(),
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
    frame.render_stateful_widget(table, chunks[2], &mut state);

    let detail = app
        .selected_row()
        .map(|row| selected_detail_lines(row, &app.status_message))
        .unwrap_or_else(|| {
            vec![
                Line::from("No matching parameters"),
                Line::from(app.status_message.as_str()),
                Line::from("Keys: q quit | f diffs/all | / search | Esc clear search"),
            ]
        });

    frame.render_widget(
        Paragraph::new(detail).block(Block::default().borders(Borders::ALL).title("Status")),
        chunks[3],
    );

    if app.mode == TuiMode::MetadataPopup {
        draw_metadata_popup(frame, app);
    }
}

fn baseline_context_line(line: &str) -> Line<'static> {
    let style = if line.contains("status=match") {
        Style::default().green()
    } else if line.contains("status=mismatch") {
        Style::default().red().bold()
    } else if line.contains("status=unknown") {
        Style::default().dark_gray()
    } else {
        Style::default()
    };
    Line::from(Span::styled(line.to_string(), style))
}

fn draw_metadata_popup(frame: &mut ratatui::Frame, app: &TuiApp) {
    let Some(row) = app.selected_row() else {
        return;
    };
    let area = centered_rect(84, 70, frame.area());
    let lines = metadata_popup_lines(row);
    let title = format!("{} metadata - ?/Enter/Esc close", row.name);
    let popup = Paragraph::new(lines)
        .block(Block::default().borders(Borders::ALL).title(title))
        .wrap(Wrap { trim: false });
    frame.render_widget(Clear, area);
    frame.render_widget(popup, area);
}

fn centered_rect(percent_x: u16, percent_y: u16, area: Rect) -> Rect {
    let vertical = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Percentage((100 - percent_y) / 2),
            Constraint::Percentage(percent_y),
            Constraint::Percentage((100 - percent_y) / 2),
        ])
        .split(area);
    let horizontal = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage((100 - percent_x) / 2),
            Constraint::Percentage(percent_x),
            Constraint::Percentage((100 - percent_x) / 2),
        ])
        .split(vertical[1]);
    horizontal[1]
}

fn row_has_metadata_choices(row: &AuditRow) -> bool {
    row.metadata.as_ref().is_some_and(|metadata| {
        !metadata.bits.is_empty()
            || !metadata.values.is_empty()
            || metadata.param_type.as_deref() == Some("boolean")
    })
}

fn metadata_popup_lines(row: &AuditRow) -> Vec<Line<'static>> {
    let mut lines = Vec::new();
    let Some(metadata) = &row.metadata else {
        return vec![Line::from("No PX4 metadata for this parameter")];
    };

    lines.push(Line::from(vec![
        Span::styled("Current: ", Style::default().bold()),
        Span::raw(row.current.clone()),
        Span::raw("  "),
        Span::styled("Baseline: ", Style::default().bold()),
        Span::raw(row.baseline.clone()),
    ]));
    if let Some(summary) = metadata_summary_line(metadata) {
        lines.push(Line::from(summary));
    }
    if let Some(description) = &metadata.short_description {
        lines.push(Line::from(format!("Description: {description}")));
    }
    lines.push(Line::from(""));

    if !metadata.bits.is_empty() {
        let current = parse_integral_display_value(&row.current).unwrap_or_default();
        lines.push(Line::from("Bits:"));
        for choice in &metadata.bits {
            let active = bit_enabled(current, choice.value);
            lines.push(choice_line(choice.value, &choice.label, active));
        }
    } else if !metadata.values.is_empty() {
        let current = parse_integral_display_value(&row.current).unwrap_or_default();
        lines.push(Line::from("Values:"));
        for choice in &metadata.values {
            lines.push(choice_line(
                choice.value,
                &choice.label,
                choice.value == current,
            ));
        }
    } else if metadata.param_type.as_deref() == Some("boolean") {
        let current = parse_integral_display_value(&row.current).unwrap_or_default();
        lines.push(Line::from("Values:"));
        lines.push(choice_line(0, "Disabled", current == 0));
        lines.push(choice_line(1, "Enabled", current != 0));
    } else {
        lines.push(Line::from(
            "No enum, boolean, or bitmask choices for this parameter",
        ));
    }

    lines
}

fn choice_line(value: i64, label: &str, active: bool) -> Line<'static> {
    let prefix = if active { "  * " } else { "    " };
    let style = if active {
        Style::default().green().bold()
    } else {
        Style::default()
    };
    Line::from(vec![
        Span::styled(prefix, style),
        Span::styled(format!("{value}: "), style),
        Span::styled(label.to_string(), style),
    ])
}

fn selected_detail_lines(row: &AuditRow, status_message: &str) -> Vec<Line<'static>> {
    let mut lines = vec![Line::from(vec![
        Span::styled("Selected: ", Style::default().bold()),
        Span::raw(row.name.clone()),
        Span::raw("  "),
        Span::styled(display_status(row), status_style_for_row(row)),
    ])];

    if let Some(metadata) = &row.metadata {
        if let Some(line) = metadata_summary_line(metadata) {
            lines.push(Line::from(line));
        }
        if let Some(line) = metadata_current_line(row, metadata) {
            lines.push(Line::from(line));
        }
        if let Some(line) = metadata_choices_line(metadata) {
            lines.push(Line::from(line));
        }
        if let Some(description) = &metadata.short_description {
            lines.push(Line::from(format!(
                "Description: {}",
                compact_line(description, 150)
            )));
        }
    }

    if row.baseline_fallback {
        lines.push(Line::from(vec![Span::styled(
            "Fallback baseline: firmware metadata/default only; airframe-specific defaults were not applied and default writes are disabled for this row.",
            Style::default().red().bold(),
        )]));
    }

    lines.push(Line::from(status_message.to_string()));
    lines.push(Line::from(
        "Keys: q quit | ? metadata | f diffs/all | e/Enter edit | d default selected | A default all | / search",
    ));
    lines
}

fn metadata_summary_line(metadata: &ParamMetadata) -> Option<String> {
    let mut parts = Vec::new();
    if let Some(param_type) = &metadata.param_type {
        parts.push(param_type.clone());
    }
    match (&metadata.min, &metadata.max) {
        (Some(min), Some(max)) => parts.push(format!("range {min}..{max}")),
        (Some(min), None) => parts.push(format!("min {min}")),
        (None, Some(max)) => parts.push(format!("max {max}")),
        (None, None) => {}
    }
    if let Some(unit) = &metadata.unit {
        parts.push(format!("unit {unit}"));
    }
    if let Some(decimal) = &metadata.decimal {
        parts.push(format!("decimal {decimal}"));
    }
    if metadata.reboot_required {
        parts.push("reboot required".into());
    }

    (!parts.is_empty()).then(|| format!("PX4 metadata: {}", parts.join(" | ")))
}

fn metadata_current_line(row: &AuditRow, metadata: &ParamMetadata) -> Option<String> {
    if !metadata.bits.is_empty() {
        let value = parse_integral_display_value(&row.current)?;
        let active = metadata
            .bits
            .iter()
            .filter(|choice| bit_enabled(value, choice.value))
            .map(|choice| format!("{}={}", choice.value, choice.label))
            .collect::<Vec<_>>();
        let active = if active.is_empty() {
            "none".into()
        } else {
            compact_line(&active.join(" | "), 150)
        };
        return Some(format!("Active bits for {}: {}", row.current, active));
    }

    if !metadata.values.is_empty() {
        let value = parse_integral_display_value(&row.current)?;
        let label = metadata
            .values
            .iter()
            .find(|choice| choice.value == value)
            .map(|choice| choice.label.as_str())
            .unwrap_or("unlisted value");
        return Some(format!("Current meaning: {value} = {label}"));
    }

    if metadata.param_type.as_deref() == Some("boolean") {
        let value = parse_integral_display_value(&row.current)?;
        let label = if value == 0 { "Disabled" } else { "Enabled" };
        return Some(format!("Current meaning: {value} = {label}"));
    }

    None
}

fn metadata_choices_line(metadata: &ParamMetadata) -> Option<String> {
    if !metadata.bits.is_empty() {
        return Some(format!("Bits: {}", format_choice_list(&metadata.bits, 150)));
    }
    if !metadata.values.is_empty() {
        return Some(format!(
            "Values: {}",
            format_choice_list(&metadata.values, 150)
        ));
    }
    if metadata.param_type.as_deref() == Some("boolean") {
        return Some("Values: 0=Disabled | 1=Enabled".into());
    }
    None
}

fn format_choice_list(choices: &[ParamChoice], max_len: usize) -> String {
    compact_line(
        &choices
            .iter()
            .map(|choice| format!("{}={}", choice.value, choice.label))
            .collect::<Vec<_>>()
            .join(" | "),
        max_len,
    )
}

fn compact_line(s: &str, max_len: usize) -> String {
    if s.chars().count() <= max_len {
        return s.to_string();
    }
    let keep = max_len.saturating_sub(3);
    format!("{}...", s.chars().take(keep).collect::<String>())
}

fn parse_integral_display_value(value: &str) -> Option<i64> {
    value
        .parse::<i64>()
        .ok()
        .or_else(|| value.parse::<f64>().ok().map(|value| value.round() as i64))
}

fn bit_enabled(value: i64, bit: i64) -> bool {
    if !(0..63).contains(&bit) {
        return false;
    }
    (value & (1_i64 << bit)) != 0
}

fn source_style(row: &AuditRow) -> Style {
    if row.baseline_fallback {
        Style::default().red().bold()
    } else {
        Style::default()
    }
}

fn status_style_for_row(row: &AuditRow) -> Style {
    if row.baseline_fallback && row.status != "unknown" {
        Style::default().red().bold()
    } else {
        status_style(&row.status)
    }
}

fn display_status(row: &AuditRow) -> String {
    if row.baseline_fallback && row.status != "unknown" {
        format!("fallback {}", row.status)
    } else {
        row.status.clone()
    }
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
    if is_integer_param_type(param.mav_type) {
        if let Some(value) = decode_integer_value(param.value, param.mav_type, param.encoding) {
            return value.to_string();
        }
    }
    format_float_param(param.value)
}

fn format_float_param(value: f32) -> String {
    let s = format!("{value:.6}");
    s.trim_end_matches('0').trim_end_matches('.').to_string()
}

fn encode_param_set_value(
    value: f32,
    mav_type: common::MavParamType,
    encoding: ParamEncoding,
) -> Result<f32> {
    if !is_integer_param_type(mav_type) {
        return Ok(value);
    }

    if encoding == ParamEncoding::CCast {
        checked_integer_for_type(value, mav_type)?;
        return Ok(value);
    }

    match mav_type {
        common::MavParamType::MAV_PARAM_TYPE_UINT8 => {
            Ok(f32::from_bits(
                checked_integer(value, 0, u8::MAX as i64)? as u8 as u32,
            ))
        }
        common::MavParamType::MAV_PARAM_TYPE_INT8 => {
            Ok(f32::from_bits(
                checked_integer(value, i8::MIN as i64, i8::MAX as i64)? as i8 as u8 as u32,
            ))
        }
        common::MavParamType::MAV_PARAM_TYPE_UINT16 => {
            Ok(f32::from_bits(
                checked_integer(value, 0, u16::MAX as i64)? as u16 as u32,
            ))
        }
        common::MavParamType::MAV_PARAM_TYPE_INT16 => {
            Ok(f32::from_bits(
                checked_integer(value, i16::MIN as i64, i16::MAX as i64)? as i16 as u16 as u32,
            ))
        }
        common::MavParamType::MAV_PARAM_TYPE_UINT32 => {
            Ok(f32::from_bits(
                checked_integer(value, 0, u32::MAX as i64)? as u32
            ))
        }
        common::MavParamType::MAV_PARAM_TYPE_INT32 => {
            Ok(f32::from_bits(
                checked_integer(value, i32::MIN as i64, i32::MAX as i64)? as i32 as u32,
            ))
        }
        _ => unreachable!("integer MAVLink parameter type was already checked"),
    }
}

fn write_encoding_for_type(
    mav_type: common::MavParamType,
    integer_encoding: ParamEncoding,
) -> ParamEncoding {
    if is_integer_param_type(mav_type) {
        integer_encoding
    } else {
        ParamEncoding::CCast
    }
}

fn effective_mav_type(
    reported: common::MavParamType,
    metadata: Option<&ParamMetadata>,
) -> common::MavParamType {
    metadata
        .and_then(metadata_mav_type)
        .filter(|metadata_type| {
            *metadata_type != common::MavParamType::MAV_PARAM_TYPE_REAL32
                || reported == common::MavParamType::MAV_PARAM_TYPE_REAL32
        })
        .unwrap_or(reported)
}

fn metadata_mav_type(metadata: &ParamMetadata) -> Option<common::MavParamType> {
    let raw = metadata.param_type.as_deref()?.to_ascii_lowercase();
    let normalized = raw
        .trim()
        .trim_start_matches("param_type_")
        .trim_start_matches("mav_param_type_");
    match normalized {
        "uint8" => Some(common::MavParamType::MAV_PARAM_TYPE_UINT8),
        "int8" => Some(common::MavParamType::MAV_PARAM_TYPE_INT8),
        "uint16" => Some(common::MavParamType::MAV_PARAM_TYPE_UINT16),
        "int16" => Some(common::MavParamType::MAV_PARAM_TYPE_INT16),
        "uint32" => Some(common::MavParamType::MAV_PARAM_TYPE_UINT32),
        "int32" | "enum" | "bitmask" | "boolean" | "bool" => {
            Some(common::MavParamType::MAV_PARAM_TYPE_INT32)
        }
        "float" | "real32" => Some(common::MavParamType::MAV_PARAM_TYPE_REAL32),
        _ => None,
    }
}

fn checked_integer_for_type(value: f32, mav_type: common::MavParamType) -> Result<i64> {
    match mav_type {
        common::MavParamType::MAV_PARAM_TYPE_UINT8 => checked_integer(value, 0, u8::MAX as i64),
        common::MavParamType::MAV_PARAM_TYPE_INT8 => {
            checked_integer(value, i8::MIN as i64, i8::MAX as i64)
        }
        common::MavParamType::MAV_PARAM_TYPE_UINT16 => checked_integer(value, 0, u16::MAX as i64),
        common::MavParamType::MAV_PARAM_TYPE_INT16 => {
            checked_integer(value, i16::MIN as i64, i16::MAX as i64)
        }
        common::MavParamType::MAV_PARAM_TYPE_UINT32 => checked_integer(value, 0, u32::MAX as i64),
        common::MavParamType::MAV_PARAM_TYPE_INT32 => {
            checked_integer(value, i32::MIN as i64, i32::MAX as i64)
        }
        _ => bail!("not an integer MAVLink parameter type: {mav_type:?}"),
    }
}

fn is_integer_param_type(mav_type: common::MavParamType) -> bool {
    matches!(
        mav_type,
        common::MavParamType::MAV_PARAM_TYPE_UINT8
            | common::MavParamType::MAV_PARAM_TYPE_INT8
            | common::MavParamType::MAV_PARAM_TYPE_UINT16
            | common::MavParamType::MAV_PARAM_TYPE_INT16
            | common::MavParamType::MAV_PARAM_TYPE_UINT32
            | common::MavParamType::MAV_PARAM_TYPE_INT32
    )
}

fn decode_integer_value(
    value: f32,
    mav_type: common::MavParamType,
    encoding: ParamEncoding,
) -> Option<i64> {
    match encoding {
        ParamEncoding::CCast => decode_integer_ccast(value, mav_type),
        ParamEncoding::Bytewise => decode_integer_bytewise(value, mav_type),
    }
}

fn decode_integer_ccast(value: f32, mav_type: common::MavParamType) -> Option<i64> {
    if !value.is_finite() {
        return None;
    }
    let rounded = value.round();
    if (value - rounded).abs() > 0.0001 {
        return None;
    }
    let integer = rounded as i64;
    checked_integer_for_type(integer as f32, mav_type).ok()?;
    Some(integer)
}

fn decode_integer_bytewise(value: f32, mav_type: common::MavParamType) -> Option<i64> {
    let bits = value.to_bits();
    let integer = match mav_type {
        common::MavParamType::MAV_PARAM_TYPE_UINT8 => (bits & 0xff) as i64,
        common::MavParamType::MAV_PARAM_TYPE_INT8 => ((bits as u8) as i8) as i64,
        common::MavParamType::MAV_PARAM_TYPE_UINT16 => (bits & 0xffff) as i64,
        common::MavParamType::MAV_PARAM_TYPE_INT16 => ((bits as u16) as i16) as i64,
        common::MavParamType::MAV_PARAM_TYPE_UINT32 => bits as i64,
        common::MavParamType::MAV_PARAM_TYPE_INT32 => (bits as i32) as i64,
        _ => return None,
    };
    Some(integer)
}

fn checked_integer(value: f32, min: i64, max: i64) -> Result<i64> {
    ensure!(value.is_finite(), "integer parameter value must be finite");
    let rounded = value.round();
    ensure!(
        (value - rounded).abs() <= f32::EPSILON,
        "integer parameter value must be whole: {value}"
    );
    let integer = rounded as i64;
    ensure!(
        integer >= min && integer <= max,
        "integer parameter value {integer} is outside [{min}, {max}]"
    );
    Ok(integer)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_px4_serial_port_indexes() {
        let text = r#"
serial_ports = {
    "TEL1": {
        "label": "TELEM 1",
        "index": 101,
        },
    "GPS1": {
        "label": "GPS 1",
        "index": 201,
        },
}
"#;

        let indexes = parse_serial_port_indexes(text);

        assert_eq!(indexes.get("TEL1").map(String::as_str), Some("101"));
        assert_eq!(indexes.get("GPS1").map(String::as_str), Some("201"));
    }

    #[test]
    fn normalizes_serial_label_defaults_only_for_serial_config_params() {
        let serial_config_params = HashSet::from(["GPS_1_CONFIG".to_string()]);
        let serial_port_indexes = HashMap::from([("GPS1".to_string(), "201".to_string())]);

        assert_eq!(
            normalize_default_value(
                "GPS_1_CONFIG",
                "GPS1".into(),
                &serial_config_params,
                &serial_port_indexes
            ),
            ("201".into(), Some("GPS1".into()))
        );
        assert_eq!(
            normalize_default_value(
                "SOME_OTHER_PARAM",
                "GPS1".into(),
                &serial_config_params,
                &serial_port_indexes
            ),
            ("GPS1".into(), None)
        );
    }

    #[test]
    fn parses_px4_param_metadata_choices() {
        let text = r#"
parameters:
- group: EKF2
  definitions:
    EKF2_GPS_CTRL:
      description:
        short: GNSS sensor aiding
      type: bitmask
      bit:
        0: Lon/lat
        1: Altitude
        2: 3D velocity
        3: Dual antenna heading
      default: 7
      min: 0
      max: 15
    EKF2_GPS_MODE:
      type: enum
      values:
        0: Automatic
        1: Dead-reckoning
      default: 0
"#;
        let yaml = serde_yaml::from_str::<Value>(text).expect("yaml");
        let wanted = HashSet::from(["EKF2_GPS_CTRL".to_string(), "EKF2_GPS_MODE".to_string()]);
        let mut defaults = HashMap::new();
        let mut metadata = HashMap::new();
        let mut serial_config_params = HashSet::new();

        collect_yaml_param_data(
            &yaml,
            &wanted,
            &mut defaults,
            &mut metadata,
            &mut serial_config_params,
        );

        assert_eq!(defaults.get("EKF2_GPS_CTRL").map(String::as_str), Some("7"));
        let gps_ctrl = metadata.get("EKF2_GPS_CTRL").expect("metadata");
        assert_eq!(gps_ctrl.param_type.as_deref(), Some("bitmask"));
        assert_eq!(
            gps_ctrl.short_description.as_deref(),
            Some("GNSS sensor aiding")
        );
        assert_eq!(gps_ctrl.bits.len(), 4);
        assert_eq!(gps_ctrl.bits[3].label, "Dual antenna heading");
        let gps_mode = metadata.get("EKF2_GPS_MODE").expect("metadata");
        assert_eq!(gps_mode.values[1].label, "Dead-reckoning");
    }

    #[test]
    fn formats_bitmask_detail_lines() {
        let row = AuditRow {
            name: "EKF2_GPS_CTRL".into(),
            current: "7".into(),
            baseline: "7".into(),
            source: "firmware default".into(),
            status: "match".into(),
            baseline_fallback: false,
            mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
            metadata: Some(ParamMetadata {
                param_type: Some("bitmask".into()),
                bits: vec![
                    ParamChoice {
                        value: 0,
                        label: "Lon/lat".into(),
                    },
                    ParamChoice {
                        value: 1,
                        label: "Altitude".into(),
                    },
                    ParamChoice {
                        value: 2,
                        label: "3D velocity".into(),
                    },
                    ParamChoice {
                        value: 3,
                        label: "Dual antenna heading".into(),
                    },
                ],
                ..ParamMetadata::default()
            }),
        };

        let metadata = row.metadata.as_ref().expect("metadata");

        assert_eq!(
            metadata_current_line(&row, metadata).as_deref(),
            Some("Active bits for 7: 0=Lon/lat | 1=Altitude | 2=3D velocity")
        );
        assert_eq!(
            metadata_choices_line(metadata).as_deref(),
            Some("Bits: 0=Lon/lat | 1=Altitude | 2=3D velocity | 3=Dual antenna heading")
        );
    }

    #[test]
    fn toggles_diff_only_view_with_search_filter() {
        let mut app = TuiApp::new(
            vec![
                test_row("EKF2_GPS_CTRL", "diff"),
                test_row("EKF2_REQ_EPH", "match"),
                test_row("GPS_1_CONFIG", "diff"),
            ],
            vec!["Baseline: test".into()],
            ParamEncoding::Bytewise,
        );

        assert_eq!(app.filtered.len(), 3);
        assert_eq!(app.view_label(), "all params");

        app.toggle_diffs_only();
        assert_eq!(app.view_label(), "diffs only");
        assert_eq!(app.filtered.len(), 2);
        assert_eq!(
            app.selected_row().map(|row| row.name.as_str()),
            Some("EKF2_GPS_CTRL")
        );

        app.query = "GPS".into();
        app.apply_filter();
        let filtered_names = app
            .filtered
            .iter()
            .map(|index| app.rows[*index].name.as_str())
            .collect::<Vec<_>>();
        assert_eq!(filtered_names, vec!["EKF2_GPS_CTRL", "GPS_1_CONFIG"]);

        app.toggle_diffs_only();
        assert_eq!(app.view_label(), "all params");
        let filtered_names = app
            .filtered
            .iter()
            .map(|index| app.rows[*index].name.as_str())
            .collect::<Vec<_>>();
        assert_eq!(filtered_names, vec!["EKF2_GPS_CTRL", "GPS_1_CONFIG"]);
    }

    #[test]
    fn encodes_px4_int32_param_sets_bytewise() {
        let encoded = encode_param_set_value(
            7.0,
            common::MavParamType::MAV_PARAM_TYPE_INT32,
            ParamEncoding::Bytewise,
        )
        .expect("encode int32");

        assert_eq!(encoded.to_bits(), 7);
        assert_ne!(encoded.to_bits(), 7.0f32.to_bits());
    }

    #[test]
    fn encodes_px4_int32_param_sets_ccast() {
        let encoded = encode_param_set_value(
            7.0,
            common::MavParamType::MAV_PARAM_TYPE_INT32,
            ParamEncoding::CCast,
        )
        .expect("encode int32");

        assert_eq!(encoded, 7.0);
    }

    #[test]
    fn formats_px4_int32_param_values_for_selected_encoding() {
        let param = ParamValue {
            value: f32::from_bits(7),
            mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
            encoding: ParamEncoding::Bytewise,
        };

        assert_eq!(format_param(&param), "7");

        let param = ParamValue {
            value: 2047.0,
            mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
            encoding: ParamEncoding::CCast,
        };

        assert_eq!(format_param(&param), "2047");
    }

    #[test]
    fn infers_ccast_integer_encoding_from_px4_metadata_range() {
        let mut params = HashMap::from([(
            "EKF2_GPS_CHECK".into(),
            ParamValue {
                value: 2047.0,
                mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
                encoding: ParamEncoding::CCast,
            },
        )]);
        let metadata = HashMap::from([(
            "EKF2_GPS_CHECK".into(),
            ParamMetadata {
                min: Some("0".into()),
                max: Some("4095".into()),
                ..ParamMetadata::default()
            },
        )]);
        let recommendations = HashMap::from([(
            "EKF2_GPS_CHECK".into(),
            Recommendation {
                value: "2047".into(),
                source: "firmware default".into(),
                fallback: false,
            },
        )]);

        infer_param_encodings(&mut params, &metadata, &recommendations, None);

        let param = params.get("EKF2_GPS_CHECK").expect("param");
        assert_eq!(param.encoding, ParamEncoding::CCast);
        assert_eq!(format_param(param), "2047");
    }

    #[test]
    fn infers_bytewise_integer_encoding_from_px4_metadata_range() {
        let mut params = HashMap::from([(
            "EKF2_GPS_CHECK".into(),
            ParamValue {
                value: f32::from_bits(2047),
                mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
                encoding: ParamEncoding::CCast,
            },
        )]);
        let metadata = HashMap::from([(
            "EKF2_GPS_CHECK".into(),
            ParamMetadata {
                min: Some("0".into()),
                max: Some("4095".into()),
                ..ParamMetadata::default()
            },
        )]);
        let recommendations = HashMap::from([(
            "EKF2_GPS_CHECK".into(),
            Recommendation {
                value: "2047".into(),
                source: "firmware default".into(),
                fallback: false,
            },
        )]);

        infer_param_encodings(&mut params, &metadata, &recommendations, None);

        let param = params.get("EKF2_GPS_CHECK").expect("param");
        assert_eq!(param.encoding, ParamEncoding::Bytewise);
        assert_eq!(format_param(param), "2047");
    }

    #[test]
    fn rejects_fractional_integer_writes() {
        let err = encode_param_set_value(
            7.5,
            common::MavParamType::MAV_PARAM_TYPE_INT32,
            ParamEncoding::CCast,
        )
        .expect_err("fractional int write should fail");

        assert!(err.to_string().contains("must be whole"));
    }

    #[test]
    fn decodes_sys_autostart_after_bytewise_encoding_inference() {
        let mut params = HashMap::from([(
            "SYS_AUTOSTART".into(),
            ParamValue {
                value: f32::from_bits(4019),
                mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
                encoding: ParamEncoding::CCast,
            },
        )]);
        let metadata = HashMap::from([(
            "SYS_AUTOSTART".into(),
            ParamMetadata {
                min: Some("0".into()),
                max: Some("999999".into()),
                ..ParamMetadata::default()
            },
        )]);
        let recommendations = HashMap::from([(
            "SYS_AUTOSTART".into(),
            Recommendation {
                value: "4019".into(),
                source: "airframe default".into(),
                fallback: false,
            },
        )]);

        infer_param_encodings(&mut params, &metadata, &recommendations, None);

        assert_eq!(
            params.get("SYS_AUTOSTART").expect("param").encoding,
            ParamEncoding::Bytewise
        );
        assert_eq!(decoded_sys_autostart(&params), Some(4019));
    }

    #[test]
    fn marks_qgc_style_fallback_recommendations() {
        let recommendations = build_recommendations(
            HashMap::from([("EKF2_GPS_CTRL".into(), "7".into())]),
            HashMap::new(),
            &[],
            &HashSet::new(),
            &HashMap::new(),
            Some("SYS_AUTOSTART=0 does not select an airframe"),
        );

        let rec = recommendations
            .get("EKF2_GPS_CTRL")
            .expect("recommendation");
        assert_eq!(rec.source, "QGC-style firmware default");
        assert!(rec.fallback);
    }

    #[test]
    fn skips_fallback_baselines_for_default_writes() {
        let row = AuditRow {
            baseline_fallback: true,
            ..test_row("EKF2_GPS_CTRL", "diff")
        };
        let writes = build_write_plan(
            &Args {
                connect: None,
                px4_source: None,
                voxl_ssh: None,
                voxl_adb: None,
                voxl_params_source: None,
                sys_autostart: None,
                param_timeout: 15,
                verbose: false,
                plain: false,
                write_diffs: true,
                yes: false,
                write_timeout: 3,
                list_ports: false,
            },
            &[row],
            ParamEncoding::Bytewise,
        )
        .expect_err("fallback baseline should not be writable");

        assert!(
            writes
                .to_string()
                .contains("no writable parameters selected")
        );
    }

    #[test]
    fn default_writes_use_global_bytewise_encoding_for_integer_params() {
        let mut row = test_row("EKF2_OF_CTRL", "diff");
        row.current = "1065353216".into();
        row.baseline = "1".into();
        row.mav_type = common::MavParamType::MAV_PARAM_TYPE_INT32;

        let writes = build_write_plan(
            &Args {
                connect: None,
                px4_source: None,
                voxl_ssh: None,
                voxl_adb: None,
                voxl_params_source: None,
                sys_autostart: None,
                param_timeout: 15,
                verbose: false,
                plain: false,
                write_diffs: true,
                yes: false,
                write_timeout: 3,
                list_ports: false,
            },
            &[row],
            ParamEncoding::Bytewise,
        )
        .expect("write plan");

        assert_eq!(writes[0].encoding, ParamEncoding::Bytewise);
        assert_eq!(
            encode_param_set_value(writes[0].value, writes[0].mav_type, writes[0].encoding)
                .expect("encoded")
                .to_bits(),
            1
        );
    }

    #[test]
    fn px4_metadata_supplies_integer_write_type_when_mavlink_reports_float() {
        let params = HashMap::from([(
            "EKF2_OF_CTRL".into(),
            ParamValue {
                value: f32::from_bits(1),
                mav_type: common::MavParamType::MAV_PARAM_TYPE_REAL32,
                encoding: ParamEncoding::CCast,
            },
        )]);
        let recommendations = HashMap::from([(
            "EKF2_OF_CTRL".into(),
            Recommendation {
                value: "0".into(),
                source: "firmware default".into(),
                fallback: false,
            },
        )]);
        let metadata = HashMap::from([(
            "EKF2_OF_CTRL".into(),
            ParamMetadata {
                param_type: Some("int32".into()),
                ..ParamMetadata::default()
            },
        )]);

        let rows = build_audit_rows(
            &params,
            &recommendations,
            &metadata,
            ParamEncoding::Bytewise,
        );

        assert_eq!(rows[0].current, "1");
        assert_eq!(rows[0].mav_type, common::MavParamType::MAV_PARAM_TYPE_INT32);
    }

    #[test]
    fn write_confirmation_requires_matching_value() {
        let write = WriteRequest {
            name: "EKF2_OF_CTRL".into(),
            value: 1.0,
            mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
            encoding: ParamEncoding::Bytewise,
            source: "test".into(),
        };
        let stale = ParamValue {
            value: f32::from_bits(0),
            mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
            encoding: ParamEncoding::Bytewise,
        };
        let confirmed = ParamValue {
            value: f32::from_bits(1),
            mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
            encoding: ParamEncoding::Bytewise,
        };

        assert!(!write_confirmation_matches(&write, &stale));
        assert!(write_confirmation_matches(&write, &confirmed));
    }

    #[test]
    fn parses_voxl_params_file_ignoring_commented_defaults() {
        let text = r#"
# 1 1 MC_PITCHRATE_D 0.0015 9
1 1 MC_PITCHRATE_D 0.0011 9
1 1 SYS_AUTOSTART 4001 6 # inline comment
"#;
        let wanted = HashSet::from(["MC_PITCHRATE_D".to_string(), "SYS_AUTOSTART".to_string()]);
        let defaults = parse_voxl_params_text(text, &wanted);

        assert_eq!(
            defaults.get("MC_PITCHRATE_D").map(String::as_str),
            Some("0.0011")
        );
        assert_eq!(
            defaults.get("SYS_AUTOSTART").map(String::as_str),
            Some("4001")
        );
    }

    #[test]
    fn derives_voxl_platform_baseline_from_starling_sku() {
        assert_eq!(
            extract_modalai_platform_code("MRB-D0012-4-V2-C29-T9-M1-X8").as_deref(),
            Some("D0012")
        );
        assert_eq!(
            derive_voxl_params_tree(Some("1.14.0-2.0.142"), &["v1.14".into(), "v1.18".into()])
                .as_deref()
                .expect("params tree"),
            "v1.14"
        );
    }

    #[test]
    fn voxl_platform_layer_overrides_firmware_default() {
        let recommendations = build_recommendations(
            HashMap::from([("MC_PITCHRATE_D".into(), "0.003".into())]),
            HashMap::new(),
            &[BaselineLayer {
                defaults: HashMap::from([("MC_PITCHRATE_D".into(), "0.0011".into())]),
                source: "VOXL D0012 default (platforms/D0012_Starling_2_Max.params)".into(),
                fallback: false,
            }],
            &HashSet::new(),
            &HashMap::new(),
            None,
        );

        let rec = recommendations
            .get("MC_PITCHRATE_D")
            .expect("recommendation");
        assert_eq!(rec.value, "0.0011");
        assert_eq!(
            rec.source,
            "VOXL D0012 default (platforms/D0012_Starling_2_Max.params)"
        );
        assert!(!rec.fallback);
    }

    #[test]
    fn voxl_platform_baseline_keeps_px4_bitmask_metadata() {
        let params = HashMap::from([(
            "EKF2_GPS_CTRL".into(),
            ParamValue {
                value: 7.0,
                mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
                encoding: ParamEncoding::CCast,
            },
        )]);
        let recommendations = HashMap::from([(
            "EKF2_GPS_CTRL".into(),
            Recommendation {
                value: "7".into(),
                source: "VOXL D0012 default (EKF2_helpers/vio_gps_baro.params)".into(),
                fallback: false,
            },
        )]);
        let metadata = HashMap::from([(
            "EKF2_GPS_CTRL".into(),
            ParamMetadata {
                param_type: Some("bitmask".into()),
                bits: vec![
                    ParamChoice {
                        value: 0,
                        label: "Lon/lat".into(),
                    },
                    ParamChoice {
                        value: 1,
                        label: "Altitude".into(),
                    },
                ],
                ..ParamMetadata::default()
            },
        )]);

        let rows = build_audit_rows(
            &params,
            &recommendations,
            &metadata,
            ParamEncoding::Bytewise,
        );
        let row = rows.first().expect("row");

        assert!(row_has_metadata_choices(row));
        assert_eq!(
            metadata_choices_line(row.metadata.as_ref().expect("metadata")).as_deref(),
            Some("Bits: 0=Lon/lat | 1=Altitude")
        );
    }

    #[test]
    fn parses_px4_c_param_bitmask_metadata() {
        let text = r#"
/**
 * Integer bitmask controlling GPS fusion.
 *
 * @group EKF2
 * @min 0
 * @max 15
 * @bit 0 Lon/lat
 * @bit 1 Altitude
 * @bit 2 3D velocity
 * @bit 3 Dual antenna heading
 */
PARAM_DEFINE_INT32(EKF2_GPS_CTRL, 7);
"#;
        let wanted = HashSet::from(["EKF2_GPS_CTRL".to_string()]);
        let mut defaults = HashMap::new();
        let mut metadata = HashMap::new();

        collect_c_param_data(text, &wanted, &mut defaults, &mut metadata);

        assert_eq!(defaults.get("EKF2_GPS_CTRL").map(String::as_str), Some("7"));
        let metadata = metadata.get("EKF2_GPS_CTRL").expect("metadata");
        assert_eq!(metadata.param_type.as_deref(), Some("bitmask"));
        assert_eq!(metadata.min.as_deref(), Some("0"));
        assert_eq!(metadata.max.as_deref(), Some("15"));
        assert_eq!(metadata.bits.len(), 4);
        assert_eq!(metadata.bits[3].label, "Dual antenna heading");
    }

    #[test]
    fn reports_voxl_package_version_mismatch() {
        let local = LocalRepoVersion {
            version: "v0.9.16".into(),
            git: "f289135".into(),
        };

        let line = voxl_version_status_line("voxl-px4-params", Some("0.9.15"), Some(&local));

        assert!(line.contains("device=0.9.15"));
        assert!(line.contains("local=v0.9.16"));
        assert!(line.contains("status=mismatch"));
    }

    fn test_row(name: &str, status: &str) -> AuditRow {
        AuditRow {
            name: name.into(),
            current: "1".into(),
            baseline: "0".into(),
            source: "firmware default".into(),
            status: status.into(),
            baseline_fallback: false,
            mav_type: common::MavParamType::MAV_PARAM_TYPE_INT32,
            metadata: None,
        }
    }
}
