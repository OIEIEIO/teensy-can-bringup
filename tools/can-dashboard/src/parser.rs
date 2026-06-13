// ============================================================================
// File: parser.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/tools/can-dashboard/src/parser.rs
// Version: v0.3.3-no-frame-elapsed
// Purpose:
//   Parser for Teensy CAN bring-up firmware log records. Scans each input line
//   for every CAN record it contains, resynchronizes on merged serial/log
//   records, updates low-rate dashboard state, and sends compact CANFRAME
//   records only to the live frame stream.
// Changes from v0.3.0:
//   - clean_scalar: adds \r and \n to trimmed chars. Fixes CTRL1 and any
//     other last-field-on-line values being silently dropped because
//     BufRead::lines() strips \n but leaves \r attached when the firmware
//     uses \r\n line endings, causing from_str_radix to fail on the value.
// Changes from v0.3.1:
//   - parse_canroute, parse_canpath, parse_canlayout: truncate rest at the
//     first '[' character before storing. Fixes layout/path/route fields
//     bleeding into [INFO teensy_can_bringup::app]: prefix of the next line
//     when the USB CDC driver delivers merged lines in a single read buffer.
// Changes from v0.3.2:
//   - parse_canframe: keeps requiring elapsed_ms/elapsed in CANFRAME records,
//     but no longer stores a per-frame elapsed copy after the UI elapsed
//     display moved to the shared timing/header state.
// Created: 2026-06-10
// Timestamp: 2026-06-13
// ============================================================================

use crate::model::{CanDashboardModel, CanFrameStreamEntry};

const RECORD_TAGS: [&str; 50] = [
    "CANFRAME",
    "CANPROVEN",
    "CANLAYOUT",
    "CANFAULT",
    "CANROUTE",
    "CANBOOT",
    "CANCFG",
    "CANPATH",
    "CANTEST",
    "CANSTAT",
    "CANRATE",
    "CANMASK",
    "CANRXD",
    "CANTXD",
    "CANRX",
    "CANTX",
    "CANIO",
    "CANERR",
    "CANMUX1",
    "CANMUX2",
    "CANEN1",
    "CANEN2",
    "CANFRZ1",
    "CANFRZ2",
    "CANUFZ1",
    "CANUFZ2",
    "CANMAX1",
    "CANMAX2",
    "CANCTRL1",
    "CANCTRL2",
    "CANCTRL",
    "CANMB0ERR",
    "CANMB1",
    "CANMB2",
    "CANEXP",
    "CANRXSEL",
    "CANMATCH",
    "CANTXCS",
    "CANRXCS",
    "CANPRE1",
    "CANPRE2",
    "CANFINAL1",
    "CANFINAL2",
    "CANFINAL",
    "CANFRZ",
    "CANUFZ",
    "CANMAX",
    "CANMUX",
    "CANEN",
    "CANMB",
];

pub(crate) fn parse_line(model: &mut CanDashboardModel, line: &str) {
    let mut search_from = 0usize;

    while let Some((record_start, tag)) = find_next_record(line, search_from) {
        let after_tag = record_start + tag.len();
        let record_end = find_next_record(line, after_tag)
            .map(|(next_start, _)| next_start)
            .unwrap_or(line.len());

        let record = line[record_start..record_end].trim();
        parse_record(model, record);

        search_from = record_end;
    }
}

fn find_next_record(line: &str, from: usize) -> Option<(usize, &'static str)> {
    for (index, _) in line.char_indices() {
        if index < from {
            continue;
        }

        if !record_boundary_before(line, index) {
            continue;
        }

        for tag in RECORD_TAGS {
            if line[index..].starts_with(tag) && record_boundary_after(line, index + tag.len()) {
                return Some((index, tag));
            }
        }
    }

    None
}

fn record_boundary_before(line: &str, index: usize) -> bool {
    if index == 0 {
        return true;
    }

    line[..index]
        .chars()
        .next_back()
        .map(|ch| !(ch.is_ascii_alphanumeric() || ch == '_'))
        .unwrap_or(true)
}

fn record_boundary_after(line: &str, index: usize) -> bool {
    if index >= line.len() {
        return true;
    }

    line[index..]
        .chars()
        .next()
        .map(|ch| ch.is_ascii_whitespace())
        .unwrap_or(true)
}

fn parse_record(model: &mut CanDashboardModel, record: &str) {
    let mut parts = record.split_whitespace();
    let Some(tag) = parts.next() else {
        return;
    };

    let rest = record[tag.len()..].trim();

    match tag {
        "CANBOOT" => parse_canboot(model, rest),
        "CANCFG" => parse_cancfg(model, rest),
        "CANROUTE" => parse_canroute(model, rest),
        "CANPATH" => parse_canpath(model, rest),
        "CANLAYOUT" => parse_canlayout(model, rest),
        "CANTEST" => parse_cantest(model, rest),
        "CANSTAT" => parse_canstat(model, rest),
        "CANIO" => parse_canio(model, rest),
        "CANTX" => parse_cantx(model, rest),
        "CANTXD" => parse_cantxd(model, rest),
        "CANRX" => parse_canrx(model, rest),
        "CANRXD" => parse_canrxd(model, rest),
        "CANFRAME" => parse_canframe(model, rest),
        "CANERR" => parse_canerr(model, rest),
        "CANRATE" => parse_canrate(model, rest),
        "CANPROVEN" => parse_canproven(model, rest),
        "CANFAULT" => parse_canfault(model, rest),
        "CANMASK" => parse_canmask(model, rest),
        _ => {}
    }
}

fn parse_canboot(model: &mut CanDashboardModel, rest: &str) {
    if let Some(version) = field(rest, "version") {
        model.config.boot_version = Some(version.to_string());
    }

    if let Some(board) = field(rest, "board") {
        model.config.board = Some(board.to_string());
    }

    if let Some(mode) = field(rest, "mode") {
        model.config.mode = Some(mode.to_string());
    }
}

fn parse_cancfg(model: &mut CanDashboardModel, rest: &str) {
    if let Some(clock_hz) = field_u32(rest, "clk_hz") {
        model.config.clock_hz = Some(clock_hz);
    }

    if let Some(ctrl1) = field_u32(rest, "ctrl1") {
        model.config.ctrl1 = Some(ctrl1);
    }

    if let Some(presdiv) = field_u32(rest, "presdiv") {
        model.config.presdiv = Some(presdiv);
    }

    if let Some(pseg1) = field_u32(rest, "pseg1") {
        model.config.pseg1 = Some(pseg1);
    }

    if let Some(pseg2) = field_u32(rest, "pseg2") {
        model.config.pseg2 = Some(pseg2);
    }

    if let Some(rjw) = field_u32(rest, "rjw") {
        model.config.rjw = Some(rjw);
    }

    if let Some(smp) = field_u32(rest, "smp") {
        model.config.smp = Some(smp);
    }

    if let Some(bitrate_bps) = field_u32(rest, "bitrate") {
        model.timing.bitrate_bps = Some(bitrate_bps);
    }

    if let Some(loop_delay_ms) = field_u32(rest, "loop_ms") {
        model.timing.loop_delay_ms = Some(loop_delay_ms);
    }
}

fn parse_canroute(model: &mut CanDashboardModel, rest: &str) {
    model.config.route = Some(trim_to_bracket(rest).to_string());
}

fn parse_canpath(model: &mut CanDashboardModel, rest: &str) {
    let clean = trim_to_bracket(rest);
    model.config.path = Some(clean.to_string());
    model.push_event(format!("CANPATH {}", clean));
}

fn parse_canlayout(model: &mut CanDashboardModel, rest: &str) {
    model.config.layout = Some(trim_to_bracket(rest).to_string());
}

// Truncate a raw rest string at the first '[' character. Prevents [INFO ...]
// log prefixes from bleeding into stored string fields when the USB CDC driver
// delivers multiple log lines concatenated in a single read buffer.
fn trim_to_bracket(rest: &str) -> &str {
    match rest.find('[') {
        Some(pos) => rest[..pos].trim_end(),
        None => rest.trim_end(),
    }
}

fn parse_cantest(model: &mut CanDashboardModel, rest: &str) {
    if let Some(id) = field_u32(rest, "id") {
        model.config.test_id = Some(id);
    }

    if let Some(dlc) = field_u32(rest, "dlc") {
        model.config.test_dlc = Some(dlc);
    }

    if let Some(dw0) = field_u32(rest, "dw0") {
        model.config.test_dw0 = Some(dw0);
    }

    if let Some(dw1) = field_u32(rest, "dw1") {
        model.config.test_dw1 = Some(dw1);
    }

    if let Some(mask) = field_u32(rest, "mask") {
        model.config.rx_mask = Some(mask);
    }
}

fn parse_canstat(model: &mut CanDashboardModel, rest: &str) {
    if let Some(cycle) = field_u32(rest, "cycle") {
        model.cycle = Some(cycle);
    }

    if let Some(pass_count) = field_u32(rest, "pass") {
        model.pass_count = pass_count;
    }

    if let Some(fail_count) = field_u32(rest, "fail") {
        model.fail_count = fail_count;
    }

    if let Some(last_pass) = field_bool(rest, "last") {
        model.last_pass = Some(last_pass);
    }

    if let Some(frame_match) = field_bool(rest, "match") {
        model.frame_match = Some(frame_match);
    }

    model.push_event(format!(
        "CANSTAT cycle={} pass={} fail={} last={} match={}",
        model.cycle.unwrap_or(0),
        model.pass_count,
        model.fail_count,
        bool_digit(model.last_pass.unwrap_or(false)),
        bool_digit(model.frame_match.unwrap_or(false))
    ));
}

fn parse_canio(model: &mut CanDashboardModel, rest: &str) {
    if let Some(cycle) = field_u32(rest, "cycle") {
        model.cycle = Some(cycle);
    }

    if let Some(tx_done) = field_bool(rest, "tx") {
        model.io.tx_done = Some(tx_done);
    }

    if let Some(rx_done) = field_bool(rest, "rx") {
        model.io.rx_done = Some(rx_done);
    }

    if let Some(late_rx) = field_bool(rest, "late") {
        model.io.late_rx = Some(late_rx);
    }

    if let Some(tx_attempt) = field_u32(rest, "txa") {
        model.io.tx_attempt = Some(tx_attempt);
    }

    if let Some(rx_attempt) = field_u32(rest, "rxa") {
        model.io.rx_attempt = Some(rx_attempt);
    }

    if let Some(clear_can1_iflag) = field_u32(rest, "clr1") {
        model.io.clear_can1_iflag = Some(clear_can1_iflag);
    }

    if let Some(clear_can2_iflag) = field_u32(rest, "clr2") {
        model.io.clear_can2_iflag = Some(clear_can2_iflag);
    }
}

fn parse_cantx(model: &mut CanDashboardModel, rest: &str) {
    if let Some(cycle) = field_u32(rest, "cycle") {
        model.cycle = Some(cycle);
    }

    if let Some(id) = field_u32(rest, "id") {
        model.tx_frame.id = Some(id);
    }

    if let Some(dlc) = field_u32(rest, "dlc") {
        model.tx_frame.dlc = Some(dlc);
    }

    if let Some(cs) = field_u32(rest, "cs") {
        model.tx_frame.cs = Some(cs);
    }
}

fn parse_cantxd(model: &mut CanDashboardModel, rest: &str) {
    if let Some(cycle) = field_u32(rest, "cycle") {
        model.cycle = Some(cycle);
    }

    if let Some(dw0) = field_u32(rest, "dw0") {
        model.tx_frame.dw0 = Some(dw0);
    }

    if let Some(dw1) = field_u32(rest, "dw1") {
        model.tx_frame.dw1 = Some(dw1);
    }
}

fn parse_canrx(model: &mut CanDashboardModel, rest: &str) {
    if let Some(cycle) = field_u32(rest, "cycle") {
        model.cycle = Some(cycle);
    }

    if let Some(id) = field_u32(rest, "id") {
        model.rx_frame.id = Some(id);
    }

    if let Some(dlc) = field_u32(rest, "dlc") {
        model.rx_frame.dlc = Some(dlc);
    }

    if let Some(code) = field_u32(rest, "code") {
        model.rx_frame.code = Some(code);
    }
}

fn parse_canrxd(model: &mut CanDashboardModel, rest: &str) {
    if let Some(cycle) = field_u32(rest, "cycle") {
        model.cycle = Some(cycle);
    }

    if let Some(dw0) = field_u32(rest, "dw0") {
        model.rx_frame.dw0 = Some(dw0);
    }

    if let Some(dw1) = field_u32(rest, "dw1") {
        model.rx_frame.dw1 = Some(dw1);
    }
}

fn parse_canframe(model: &mut CanDashboardModel, rest: &str) {
    let Some(bus) = field(rest, "bus") else {
        return;
    };
    let Some(dir) = field(rest, "dir") else {
        return;
    };
    let Some(cycle) = field_u32(rest, "cycle") else {
        return;
    };
    let Some(id) = field_u32(rest, "id") else {
        return;
    };
    let Some(dlc) = field_u32(rest, "dlc") else {
        return;
    };
    let Some(dw0) = field_u32(rest, "dw0") else {
        return;
    };
    let Some(dw1) = field_u32(rest, "dw1") else {
        return;
    };

    if field_u32(rest, "elapsed_ms")
        .or_else(|| field_u32(rest, "elapsed"))
        .is_none()
    {
        return;
    }

    let decode = field(rest, "decode").unwrap_or("--");

    let bytes = words_to_bytes(dw0, dw1);
    let dlc = dlc.min(8);

    model.cycle = Some(cycle);

    if dir.eq_ignore_ascii_case("TX") {
        model.tx_frame.id = Some(id);
        model.tx_frame.dlc = Some(dlc);
        model.tx_frame.dw0 = Some(dw0);
        model.tx_frame.dw1 = Some(dw1);
    } else if dir.eq_ignore_ascii_case("RX") {
        model.rx_frame.id = Some(id);
        model.rx_frame.dlc = Some(dlc);
        model.rx_frame.dw0 = Some(dw0);
        model.rx_frame.dw1 = Some(dw1);
    }

    model.push_frame(CanFrameStreamEntry {
        cycle,
        bus: normalize_bus(bus),
        dir: normalize_dir(dir),
        id,
        dlc,
        bytes,
        decode: decode.to_string(),
    });
}

fn parse_canerr(model: &mut CanDashboardModel, rest: &str) {
    if let Some(cycle) = field_u32(rest, "cycle") {
        model.cycle = Some(cycle);
    }

    if let Some(can1_tx_err) = field_u32(rest, "c1tx") {
        model.errors.can1_tx_err = Some(can1_tx_err);
    }

    if let Some(can1_rx_err) = field_u32(rest, "c1rx") {
        model.errors.can1_rx_err = Some(can1_rx_err);
    }

    if let Some(can2_tx_err) = field_u32(rest, "c2tx") {
        model.errors.can2_tx_err = Some(can2_tx_err);
    }

    if let Some(can2_rx_err) = field_u32(rest, "c2rx") {
        model.errors.can2_rx_err = Some(can2_rx_err);
    }

    if let Some(can1_esr1) = field_u32(rest, "e1") {
        model.errors.can1_esr1 = Some(can1_esr1);
    }

    if let Some(can2_esr1) = field_u32(rest, "e2") {
        model.errors.can2_esr1 = Some(can2_esr1);
    }
}

fn parse_canrate(model: &mut CanDashboardModel, rest: &str) {
    if let Some(cycle) = field_u32(rest, "cycle") {
        model.cycle = Some(cycle);
    }

    if let Some(pass_percent) = field_u32(rest, "pct") {
        model.timing.pass_percent = Some(pass_percent);
    }

    if let Some(rate_x100) = field_u32(rest, "rate_x100") {
        model.timing.rate_x100 = Some(rate_x100);
    }

    if let Some(elapsed_ms) = field_u32(rest, "elapsed_ms") {
        model.timing.elapsed_ms = Some(elapsed_ms);
    }

    if let Some(util_x100) = field_u32(rest, "util_x100") {
        model.timing.util_x100 = Some(util_x100);
    }
}

fn parse_canproven(model: &mut CanDashboardModel, rest: &str) {
    model.proven = true;

    if let Some(cycle) = field_u32(rest, "cycle") {
        model.cycle = Some(cycle);
        model.proven_cycle = Some(cycle);
    }

    if let Some(path) = field(rest, "path") {
        model.proven_path = Some(path.to_string());
        model.push_event(format!(
            "CANPROVEN cycle={} path={}",
            model.proven_cycle.unwrap_or(0),
            path
        ));
        return;
    }

    if let (Some(id), Some(dlc), Some(dw0), Some(dw1)) = (
        field_u32(rest, "id"),
        field_u32(rest, "dlc"),
        field_u32(rest, "dw0"),
        field_u32(rest, "dw1"),
    ) {
        model.push_event(format!(
            "CANPROVEN cycle={} id=0x{:03X} dlc={} dw0=0x{:08X} dw1=0x{:08X}",
            model.proven_cycle.unwrap_or(0),
            id,
            dlc,
            dw0,
            dw1
        ));
        return;
    }

    if let Some(data) = field(rest, "data") {
        model.push_event(format!(
            "CANPROVEN cycle={} data={}",
            model.proven_cycle.unwrap_or(0),
            data
        ));
    }
}

fn parse_canfault(model: &mut CanDashboardModel, rest: &str) {
    if let Some(cycle) = field_u32(rest, "cycle") {
        model.cycle = Some(cycle);
    }

    if let Some(stage) = field(rest, "stage") {
        model.last_fault_stage = Some(stage.to_string());
    }

    if let Some(reason) = field(rest, "reason") {
        model.last_fault_reason = Some(reason.to_string());
    }

    model.push_event(format!(
        "CANFAULT cycle={} stage={} reason={}",
        model.cycle.unwrap_or(0),
        model.last_fault_stage.as_deref().unwrap_or("unknown"),
        model.last_fault_reason.as_deref().unwrap_or("unknown")
    ));
}

fn parse_canmask(model: &mut CanDashboardModel, rest: &str) {
    if let Some(mask) = field_u32(rest, "mg") {
        model.config.rx_mask = Some(mask);
    }
}

fn field<'a>(rest: &'a str, key: &str) -> Option<&'a str> {
    for token in rest.split_whitespace() {
        let Some((token_key, token_value)) = token.split_once('=') else {
            continue;
        };

        if token_key == key {
            return Some(clean_scalar(token_value));
        }
    }

    None
}

fn field_u32(rest: &str, key: &str) -> Option<u32> {
    parse_u32(field(rest, key)?)
}

fn field_bool(rest: &str, key: &str) -> Option<bool> {
    match field(rest, key)? {
        "0" => Some(false),
        "1" => Some(true),
        "false" | "FALSE" => Some(false),
        "true" | "TRUE" => Some(true),
        _ => None,
    }
}

fn parse_u32(value: &str) -> Option<u32> {
    let trimmed = clean_scalar(value);

    if let Some(hex) = trimmed
        .strip_prefix("0x")
        .or_else(|| trimmed.strip_prefix("0X"))
    {
        u32::from_str_radix(hex, 16).ok()
    } else if trimmed.chars().all(|ch| ch.is_ascii_hexdigit())
        && trimmed.chars().any(|ch| ch.is_ascii_hexdigit() && !ch.is_ascii_digit())
    {
        u32::from_str_radix(trimmed, 16).ok()
    } else {
        trimmed.parse::<u32>().ok()
    }
}

fn clean_scalar(value: &str) -> &str {
    value.trim_matches(|ch: char| {
        ch == ','
            || ch == ';'
            || ch == '"'
            || ch == '\''
            || ch == '['
            || ch == ']'
            || ch == '\r'
            || ch == '\n'
    })
}

fn words_to_bytes(dw0: u32, dw1: u32) -> [u8; 8] {
    [
        ((dw0 >> 24) & 0xFF) as u8,
        ((dw0 >> 16) & 0xFF) as u8,
        ((dw0 >> 8) & 0xFF) as u8,
        (dw0 & 0xFF) as u8,
        ((dw1 >> 24) & 0xFF) as u8,
        ((dw1 >> 16) & 0xFF) as u8,
        ((dw1 >> 8) & 0xFF) as u8,
        (dw1 & 0xFF) as u8,
    ]
}

fn normalize_bus(value: &str) -> String {
    match value {
        "1" | "can1" | "CAN1" => "CAN1".to_string(),
        "2" | "can2" | "CAN2" => "CAN2".to_string(),
        _ => value.to_string(),
    }
}

fn normalize_dir(value: &str) -> String {
    if value.eq_ignore_ascii_case("tx") {
        "TX".to_string()
    } else if value.eq_ignore_ascii_case("rx") {
        "RX".to_string()
    } else {
        value.to_string()
    }
}

fn bool_digit(value: bool) -> u32 {
    if value { 1 } else { 0 }
}

// ============================================================================
// Footer
// File: parser.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/tools/can-dashboard/src/parser.rs
// Version: v0.3.3-no-frame-elapsed
// Created: 2026-06-10
// Timestamp: 2026-06-13
// End of file
// ============================================================================