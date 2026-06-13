// ============================================================================
// File: ui.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/tools/can-dashboard/src/ui.rs
// Version: v0.2.13-watch-panel-stable-layout
// Purpose:
//   Ratatui terminal renderer for the Teensy CAN dashboard. Displays parsed
//   status, compact CAN1/CAN2 node panels, timing with CTRL1 bit fields,
//   split CAN1/CAN2 error counters, human-readable route/config lines,
//   live CAN frame byte grid, watched decoded voltage_distribution message,
//   and bounded event history.
//
//   Layout:
//   - Header.
//   - Status / I/O / Timing.
//   - Compact CAN1 TX / CAN2 RX / Errors row.
//   - Live CAN Frame Grid with right-side watched-message panel.
//   - Route / Config and Event Log bottom row.
//
//   Changes from v0.2.12:
//   - Keeps compact CAN1/CAN2/Error panels.
//   - Keeps row-two height compact for grid real estate.
//   - Keeps Elapsed removed from Live CAN Frame Grid.
//   - Builds frame pairs once per render and shares that snapshot between the
//     grid and watched-message panel.
//   - Gives the watched-message panel fixed row count and fixed spacing.
//   - Removes wrapping from the watched-message panel to reduce visible redraw
//     flicker.
//   - Keeps watched-message decode for CAN ID 0x145 voltage_distribution.
// Created: 2026-06-10
// Timestamp: 2026-06-12
// ============================================================================

use crate::model::{CanDashboardModel, CanFrameStreamEntry};
use crate::units::{
    bool_text, bytes_text, elapsed_text, hex11_text, hex32_text, percent_text,
    rate_x100_text, str_text, u32_text, util_x100_text,
};
use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::text::Line;
use ratatui::widgets::{Block, Borders, Paragraph, Wrap};
use ratatui::Frame;

const WATCH_ID: u32 = 0x145;
const WATCH_NAME: &str = "voltage_distribution";

#[derive(Clone)]
struct FramePair {
    cycle: u32,
    id: u32,
    tx: Option<CanFrameStreamEntry>,
    rx: Option<CanFrameStreamEntry>,
}

pub(crate) fn render_dashboard(frame: &mut Frame<'_>, model: &CanDashboardModel) {
    let root = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(3),   // header
            Constraint::Length(11),  // status / io / timing
            Constraint::Length(6),   // compact can1 / can2 / errors
            Constraint::Min(10),     // live frame grid / watched message
            Constraint::Length(8),   // route / config / event log
        ])
        .split(frame.area());

    render_header(frame, root[0], model);

    let row_one = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(34),
            Constraint::Percentage(33),
            Constraint::Percentage(33),
        ])
        .split(root[1]);

    render_status(frame, row_one[0], model);
    render_io(frame, row_one[1], model);
    render_timing(frame, row_one[2], model);

    let row_two = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([
            Constraint::Percentage(34),
            Constraint::Percentage(33),
            Constraint::Percentage(33),
        ])
        .split(root[2]);

    render_can1_node(frame, row_two[0], model);
    render_can2_node(frame, row_two[1], model);
    render_errors(frame, row_two[2], model);

    render_frame_and_watch(frame, root[3], model);

    let row_bottom = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(40), Constraint::Percentage(60)])
        .split(root[4]);

    render_route(frame, row_bottom[0], model);
    render_events(frame, row_bottom[1], model);
}

fn render_header(frame: &mut Frame<'_>, area: Rect, model: &CanDashboardModel) {
    let proven = if model.proven { "PROVEN" } else { "WAITING" };

    let line = format!(
        "CAN Dashboard | {} | board={} | mode={} | version={}",
        proven,
        str_text(model.config.board.as_deref()),
        str_text(model.config.mode.as_deref()),
        str_text(model.config.boot_version.as_deref())
    );

    let widget = Paragraph::new(vec![Line::from(line)])
        .block(Block::default().title(" Teensy CAN Bring-Up ").borders(Borders::ALL));

    frame.render_widget(widget, area);
}

fn render_status(frame: &mut Frame<'_>, area: Rect, model: &CanDashboardModel) {
    let lines = vec![
        Line::from(format!("cycle:       {}", u32_text(model.cycle))),
        Line::from(format!("pass:        {}", model.pass_count)),
        Line::from(format!("fail:        {}", model.fail_count)),
        Line::from(format!("total:       {}", model.total_runs())),
        Line::from(format!("last pass:   {}", bool_text(model.last_pass))),
        Line::from(format!("match:       {}", bool_text(model.frame_match))),
        Line::from(format!("proven:      {}", if model.proven { "yes" } else { "no" })),
        Line::from(format!("proven cyc:  {}", u32_text(model.proven_cycle))),
        Line::from(format!(
            "last fault:  {}/{}",
            str_text(model.last_fault_stage.as_deref()),
            str_text(model.last_fault_reason.as_deref())
        )),
    ];

    let widget = Paragraph::new(lines)
        .block(Block::default().title(" Status ").borders(Borders::ALL))
        .wrap(Wrap { trim: true });

    frame.render_widget(widget, area);
}

fn render_io(frame: &mut Frame<'_>, area: Rect, model: &CanDashboardModel) {
    let lines = vec![
        Line::from(format!("CAN1 TX done: {}", bool_text(model.io.tx_done))),
        Line::from(format!("CAN2 RX done: {}", bool_text(model.io.rx_done))),
        Line::from(format!("late RX:      {}", bool_text(model.io.late_rx))),
        Line::from(format!("TX attempt:   {}", u32_text(model.io.tx_attempt))),
        Line::from(format!("RX attempt:   {}", u32_text(model.io.rx_attempt))),
        Line::from(format!("clr CAN1:     {}", hex32_text(model.io.clear_can1_iflag))),
        Line::from(format!("clr CAN2:     {}", hex32_text(model.io.clear_can2_iflag))),
    ];

    let widget = Paragraph::new(lines)
        .block(Block::default().title(" I/O ").borders(Borders::ALL))
        .wrap(Wrap { trim: true });

    frame.render_widget(widget, area);
}

fn render_timing(frame: &mut Frame<'_>, area: Rect, model: &CanDashboardModel) {
    let lines = vec![
        Line::from(format!("bitrate:   {} bps", u32_text(model.timing.bitrate_bps))),
        Line::from(format!("loop delay:{} ms", u32_text(model.timing.loop_delay_ms))),
        Line::from(format!("pass rate: {}", percent_text(model.timing.pass_percent))),
        Line::from(format!("frame rate:{}", rate_x100_text(model.timing.rate_x100))),
        Line::from(format!("elapsed:   {}", elapsed_text(model.timing.elapsed_ms))),
        Line::from(format!("util:      {}", util_x100_text(model.timing.util_x100))),
        Line::from(format!("CAN clk:   {} Hz", u32_text(model.config.clock_hz))),
        Line::from(format!("CTRL1:     {}", hex32_text(model.config.ctrl1))),
        Line::from(format!(
            "PRESDIV={} PSEG1={} PSEG2={} RJW={}",
            u32_text(model.config.presdiv),
            u32_text(model.config.pseg1),
            u32_text(model.config.pseg2),
            u32_text(model.config.rjw),
        )),
    ];

    let widget = Paragraph::new(lines)
        .block(Block::default().title(" Timing ").borders(Borders::ALL))
        .wrap(Wrap { trim: true });

    frame.render_widget(widget, area);
}

fn render_can1_node(frame: &mut Frame<'_>, area: Rect, model: &CanDashboardModel) {
    let lines = vec![
        Line::from(format!(
            "id:     {:<12} dlc:      {}",
            hex11_text(model.tx_frame.id),
            u32_text(model.tx_frame.dlc)
        )),
        Line::from(format!(
            "dw0:    {:<12} cs:       {}",
            hex32_text(model.tx_frame.dw0),
            hex32_text(model.tx_frame.cs)
        )),
        Line::from(format!(
            "dw1:    {:<12} TX done:  {}",
            hex32_text(model.tx_frame.dw1),
            bool_text(model.io.tx_done)
        )),
    ];

    let widget = Paragraph::new(lines)
        .block(Block::default().title(" CAN1 TX ").borders(Borders::ALL))
        .wrap(Wrap { trim: true });

    frame.render_widget(widget, area);
}

fn render_can2_node(frame: &mut Frame<'_>, area: Rect, model: &CanDashboardModel) {
    let lines = vec![
        Line::from(format!(
            "id:     {:<12} dlc:      {}",
            hex11_text(model.rx_frame.id),
            u32_text(model.rx_frame.dlc)
        )),
        Line::from(format!(
            "dw0:    {:<12} code:     {}",
            hex32_text(model.rx_frame.dw0),
            u32_text(model.rx_frame.code)
        )),
        Line::from(format!(
            "dw1:    {:<12} RX done:  {}",
            hex32_text(model.rx_frame.dw1),
            bool_text(model.io.rx_done)
        )),
        Line::from(format!("late RX: {}", bool_text(model.io.late_rx))),
    ];

    let widget = Paragraph::new(lines)
        .block(Block::default().title(" CAN2 RX ").borders(Borders::ALL))
        .wrap(Wrap { trim: true });

    frame.render_widget(widget, area);
}

fn render_errors(frame: &mut Frame<'_>, area: Rect, model: &CanDashboardModel) {
    let lines = vec![
        Line::from("CAN1                       | CAN2"),
        Line::from(format!(
            "txerr: {:<18} | txerr: {}",
            u32_text(model.errors.can1_tx_err),
            u32_text(model.errors.can2_tx_err)
        )),
        Line::from(format!(
            "rxerr: {:<18} | rxerr: {}",
            u32_text(model.errors.can1_rx_err),
            u32_text(model.errors.can2_rx_err)
        )),
        Line::from(format!(
            "ESR1:  {:<18} | ESR1:  {}",
            hex32_text(model.errors.can1_esr1),
            hex32_text(model.errors.can2_esr1)
        )),
    ];

    let widget = Paragraph::new(lines)
        .block(Block::default().title(" Errors ").borders(Borders::ALL))
        .wrap(Wrap { trim: true });

    frame.render_widget(widget, area);
}

fn render_route(frame: &mut Frame<'_>, area: Rect, model: &CanDashboardModel) {
    let route = model.config.route.as_deref();
    let layout = model.config.layout.as_deref();

    let line1 = format!(
        "pins:    CAN1 TX={} RX={} | CAN2 TX={} RX={} | ACK={}",
        field_text(route_field(route, "c1tx")),
        field_text(route_field(route, "c1rx")),
        field_text(route_field(route, "c2tx")),
        field_text(route_field(route, "c2rx")),
        field_text(route_field(route, "ack")),
    );

    let line2 = format!("path:    {}", path_text(model.config.path.as_deref()));

    let line3 = format!(
        "mailbox: CAN1 {} | CAN2 {} | MAXMB={}",
        field_text(route_field(layout, "c1_mb")),
        field_text(route_field(layout, "c2_mb")),
        field_text(route_field(layout, "maxmb")),
    );

    let line4 = format!(
        "test:    id={} dlc={} mask={}",
        hex11_text(model.config.test_id),
        u32_text(model.config.test_dlc),
        hex32_text(model.config.rx_mask),
    );

    let line5 = format!("traffic: {}", traffic_text(model.config.mode.as_deref()));

    let widget = Paragraph::new(vec![
        Line::from(line1),
        Line::from(line2),
        Line::from(line3),
        Line::from(line4),
        Line::from(line5),
    ])
    .block(Block::default().title(" Route / Config ").borders(Borders::ALL))
    .wrap(Wrap { trim: true });

    frame.render_widget(widget, area);
}

fn route_field<'a>(text: Option<&'a str>, key: &str) -> Option<&'a str> {
    let text = text?;

    for token in text.split_whitespace() {
        let Some((token_key, token_value)) = token.split_once('=') else {
            continue;
        };

        if token_key == key {
            return Some(clean_route_scalar(token_value));
        }
    }

    None
}

fn clean_route_scalar(value: &str) -> &str {
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

fn field_text(value: Option<&str>) -> &str {
    value.unwrap_or("--")
}

fn path_text(path: Option<&str>) -> String {
    let Some(path) = path else {
        return "--".to_string();
    };

    path.replace("CANH_CANL", "CANH/CANL")
        .replace("XCVRA", "XCVR A")
        .replace("XCVRB", "XCVR B")
        .replace('-', " -> ")
}

fn traffic_text(mode: Option<&str>) -> &'static str {
    match mode {
        Some("drone_catalog_64_one_rx_mb") => {
            "drone64 catalog | 64 frames/cycle | one reused RX mailbox"
        }
        Some(_) => "mode-specific traffic | see live frame grid",
        None => "--",
    }
}

fn render_frame_and_watch(frame: &mut Frame<'_>, area: Rect, model: &CanDashboardModel) {
    let pairs = build_frame_pairs(model);

    let row = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Min(141), Constraint::Length(46)])
        .split(area);

    render_frame_stream(frame, row[0], model, &pairs);
    render_watch_message(frame, row[1], &pairs);
}

fn render_frame_stream(
    frame: &mut Frame<'_>,
    area: Rect,
    model: &CanDashboardModel,
    pairs: &[FramePair],
) {
    let mut lines = Vec::new();

    lines.push(Line::from(
        " Cyc   Match | TX Bus TX ID   TX Decode             B0 B1 B2 B3 B4 B5 B6 B7 DLC | RX Bus RX ID   RX Decode             B0 B1 B2 B3 B4 B5 B6 B7 DLC"
            .to_string(),
    ));
    lines.push(Line::from(
        " ----  ----- | ------ ------  --------------------  -- -- -- -- -- -- -- -- --- | ------ ------  --------------------  -- -- -- -- -- -- -- -- ---"
            .to_string(),
    ));

    if model.frame_stream.is_empty() {
        lines.push(Line::from(" waiting for CANFRAME records...".to_string()));
    } else {
        let complete_pairs: Vec<&FramePair> = pairs
            .iter()
            .filter(|pair| pair.tx.is_some() && pair.rx.is_some())
            .collect();

        if complete_pairs.is_empty() {
            lines.push(Line::from(" waiting for complete TX/RX frame pairs...".to_string()));
        } else {
            let inner_height = area.height.saturating_sub(4) as usize;
            let pair_count = complete_pairs.len();
            let skip = pair_count.saturating_sub(inner_height);

            for pair in complete_pairs.iter().skip(skip) {
                lines.push(Line::from(format_frame_pair(*pair)));
            }
        }
    }

    let widget = Paragraph::new(lines)
        .block(Block::default().title(" Live CAN Frame Grid ").borders(Borders::ALL));

    frame.render_widget(widget, area);
}

fn render_watch_message(frame: &mut Frame<'_>, area: Rect, pairs: &[FramePair]) {
    let watched = pairs
        .iter()
        .rev()
        .find(|pair| pair.id == WATCH_ID && pair.tx.is_some() && pair.rx.is_some());

    let lines = if let Some(pair) = watched {
        let tx = pair.tx.as_ref();
        let rx = pair.rx.as_ref();

        let tx_bytes = match tx {
            Some(tx) => bytes_text(&tx.bytes, tx.dlc),
            None => "--".to_string(),
        };

        let rx_bytes = match rx {
            Some(rx) => bytes_text(&rx.bytes, rx.dlc),
            None => "--".to_string(),
        };

        let (main_battery_raw, rail_5v_raw, rail_3v3_raw, current_raw) = match rx {
            Some(rx) => (
                u16_le_field(rx, 0),
                u16_le_field(rx, 2),
                u16_le_field(rx, 4),
                u16_le_field(rx, 6),
            ),
            None => (None, None, None, None),
        };

        vec![
            Line::from(format!("id:      0x{:03X}", WATCH_ID)),
            Line::from(format!("name:    {}", WATCH_NAME)),
            Line::from(format!("age:     cycle {} / live", pair.cycle)),
            Line::from(""),
            Line::from(format!("tx:      {}", tx_bytes)),
            Line::from(format!("rx:      {}", rx_bytes)),
            Line::from(""),
            Line::from("field           raw      value"),
            watch_value_line("main battery", main_battery_raw, main_battery_raw.map(voltage_mv_text)),
            watch_value_line("rail 5V", rail_5v_raw, rail_5v_raw.map(voltage_mv_text)),
            watch_value_line("rail 3V3", rail_3v3_raw, rail_3v3_raw.map(voltage_mv_text)),
            watch_value_line("current", current_raw, current_raw.map(current_ca_text)),
        ]
    } else {
        vec![
            Line::from(format!("id:      0x{:03X}", WATCH_ID)),
            Line::from(format!("name:    {}", WATCH_NAME)),
            Line::from("age:     --"),
            Line::from(""),
            Line::from("tx:      -- -- -- -- -- -- -- --"),
            Line::from("rx:      -- -- -- -- -- -- -- --"),
            Line::from(""),
            Line::from("field           raw      value"),
            watch_value_line("main battery", None, None),
            watch_value_line("rail 5V", None, None),
            watch_value_line("rail 3V3", None, None),
            watch_value_line("current", None, None),
        ]
    };

    let widget = Paragraph::new(lines)
        .block(Block::default().title(" Watched Message ").borders(Borders::ALL));

    frame.render_widget(widget, area);
}

fn build_frame_pairs(model: &CanDashboardModel) -> Vec<FramePair> {
    let mut pairs: Vec<FramePair> = Vec::new();

    for entry in model.frame_stream.iter() {
        let mut placed = false;

        for pair in pairs.iter_mut().rev() {
            if pair.cycle == entry.cycle && pair.id == entry.id {
                if entry.dir.eq_ignore_ascii_case("TX") && pair.tx.is_none() {
                    pair.tx = Some(entry.clone());
                    placed = true;
                    break;
                }

                if entry.dir.eq_ignore_ascii_case("RX") && pair.rx.is_none() {
                    pair.rx = Some(entry.clone());
                    placed = true;
                    break;
                }
            }
        }

        if !placed {
            let mut pair = FramePair {
                cycle: entry.cycle,
                id: entry.id,
                tx: None,
                rx: None,
            };

            if entry.dir.eq_ignore_ascii_case("TX") {
                pair.tx = Some(entry.clone());
            } else if entry.dir.eq_ignore_ascii_case("RX") {
                pair.rx = Some(entry.clone());
            }

            pairs.push(pair);
        }
    }

    pairs
}

fn format_frame_pair(pair: &FramePair) -> String {
    format!(
        " {:>4}  {:>5} | {} | {}",
        pair.cycle,
        frame_pair_match_text(pair),
        frame_side_text(pair.tx.as_ref()),
        frame_side_text(pair.rx.as_ref())
    )
}

fn frame_pair_match_text(pair: &FramePair) -> &'static str {
    let (Some(tx), Some(rx)) = (pair.tx.as_ref(), pair.rx.as_ref()) else {
        return "--";
    };

    if tx.id == rx.id
        && tx.dlc == rx.dlc
        && bytes_text(&tx.bytes, tx.dlc) == bytes_text(&rx.bytes, rx.dlc)
    {
        "yes"
    } else {
        "no"
    }
}

fn frame_side_text(entry: Option<&CanFrameStreamEntry>) -> String {
    let Some(entry) = entry else {
        return format!(
            "{:>6} {:>6}  {:<20}  {:>2} {:>2} {:>2} {:>2} {:>2} {:>2} {:>2} {:>2} {:>3}",
            "--", "--", "--", "--", "--", "--", "--", "--", "--", "--", "--", "--"
        );
    };

    format!(
        "{:>6} 0x{:03X}  {:<20}  {:>2} {:>2} {:>2} {:>2} {:>2} {:>2} {:>2} {:>2} {:>3}",
        entry.bus,
        entry.id & 0x7FF,
        entry.decode,
        byte_cell(entry, 0),
        byte_cell(entry, 1),
        byte_cell(entry, 2),
        byte_cell(entry, 3),
        byte_cell(entry, 4),
        byte_cell(entry, 5),
        byte_cell(entry, 6),
        byte_cell(entry, 7),
        entry.dlc.min(8)
    )
}

fn byte_cell(entry: &CanFrameStreamEntry, index: usize) -> String {
    if index < (entry.dlc as usize).min(8) {
        format!("{:02X}", entry.bytes[index])
    } else {
        "--".to_string()
    }
}

fn u16_le_field(entry: &CanFrameStreamEntry, offset: usize) -> Option<u16> {
    if (entry.dlc as usize) < offset + 2 {
        return None;
    }

    let lo = *entry.bytes.get(offset)? as u16;
    let hi = *entry.bytes.get(offset + 1)? as u16;

    Some((hi << 8) | lo)
}

fn watch_value_line(label: &str, raw: Option<u16>, value: Option<String>) -> Line<'static> {
    let raw_text = match raw {
        Some(raw) => format!("0x{:04X}", raw),
        None => "--".to_string(),
    };

    let value_text = value.unwrap_or_else(|| "--".to_string());

    Line::from(format!("{:<15} {:<8} {}", label, raw_text, value_text))
}

fn voltage_mv_text(raw: u16) -> String {
    let mv = raw as u32;
    let whole = mv / 1000;
    let frac = mv % 1000;

    format!("{}.{:03} V", whole, frac)
}

fn current_ca_text(raw: u16) -> String {
    let centiamps = raw as i16;
    let centiamps_i32 = centiamps as i32;
    let sign = if centiamps_i32 < 0 { "-" } else { "" };
    let abs = centiamps_i32.abs();
    let whole = abs / 100;
    let frac = abs % 100;

    format!("{}{}.{:02} A", sign, whole, frac)
}

fn render_events(frame: &mut Frame<'_>, area: Rect, model: &CanDashboardModel) {
    let mut lines = Vec::new();

    if model.event_log.is_empty() {
        lines.push(Line::from("waiting for CAN records on stdin"));
    } else {
        let inner_height = area.height.saturating_sub(2) as usize;
        let log_len = model.event_log.len();
        let skip = log_len.saturating_sub(inner_height);

        for event in model.event_log.iter().skip(skip) {
            lines.push(Line::from(event.clone()));
        }
    }

    let widget = Paragraph::new(lines)
        .block(Block::default().title(" Event Log ").borders(Borders::ALL))
        .wrap(Wrap { trim: true });

    frame.render_widget(widget, area);
}

// ============================================================================
// Footer
// File: ui.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/tools/can-dashboard/src/ui.rs
// Version: v0.2.13-watch-panel-stable-layout
// Created: 2026-06-10
// Timestamp: 2026-06-12
// End of file
// ============================================================================