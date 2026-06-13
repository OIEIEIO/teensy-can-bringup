// ============================================================================
// File: model.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/tools/can-dashboard/src/model.rs
// Version: v0.2.1-no-frame-elapsed
// Purpose:
//   Host-side dashboard state for the Teensy CAN bring-up log stream. Stores
//   the latest parsed CAN status, I/O state, frame data, error counters,
//   timing values, route/config text, bounded event history, and live frame
//   stream entries from CANFRAME records.
// Changes from v0.1.0:
//   - CanConfigView: adds presdiv, pseg1, pseg2, rjw, smp from CANCFG.
//   - CanTimingView: adds util_x100 from CANRATE.
//   - CanFrameStreamEntry: new struct for one CANFRAME record.
//   - CanDashboardModel: adds frame_stream VecDeque bounded to FRAME_STREAM_LIMIT.
// Changes from v0.2.0:
//   - CanFrameStreamEntry: removes unused elapsed_ms field after UI elapsed
//     display moved to the shared timing/header state.
// ============================================================================

use std::collections::VecDeque;

pub(crate) const EVENT_LOG_LIMIT: usize = 16;
pub(crate) const FRAME_STREAM_LIMIT: usize = 64;

#[derive(Clone, Debug, Default)]
pub(crate) struct CanFrameView {
    pub(crate) id: Option<u32>,
    pub(crate) dlc: Option<u32>,
    pub(crate) code: Option<u32>,
    pub(crate) cs: Option<u32>,
    pub(crate) dw0: Option<u32>,
    pub(crate) dw1: Option<u32>,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct CanIoView {
    pub(crate) tx_done: Option<bool>,
    pub(crate) rx_done: Option<bool>,
    pub(crate) late_rx: Option<bool>,
    pub(crate) tx_attempt: Option<u32>,
    pub(crate) rx_attempt: Option<u32>,
    pub(crate) clear_can1_iflag: Option<u32>,
    pub(crate) clear_can2_iflag: Option<u32>,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct CanErrorView {
    pub(crate) can1_tx_err: Option<u32>,
    pub(crate) can1_rx_err: Option<u32>,
    pub(crate) can2_tx_err: Option<u32>,
    pub(crate) can2_rx_err: Option<u32>,
    pub(crate) can1_esr1: Option<u32>,
    pub(crate) can2_esr1: Option<u32>,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct CanTimingView {
    pub(crate) bitrate_bps: Option<u32>,
    pub(crate) loop_delay_ms: Option<u32>,
    pub(crate) pass_percent: Option<u32>,
    pub(crate) rate_x100: Option<u32>,
    pub(crate) elapsed_ms: Option<u32>,
    pub(crate) util_x100: Option<u32>,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct CanConfigView {
    pub(crate) boot_version: Option<String>,
    pub(crate) board: Option<String>,
    pub(crate) mode: Option<String>,
    pub(crate) clock_hz: Option<u32>,
    pub(crate) ctrl1: Option<u32>,
    pub(crate) presdiv: Option<u32>,
    pub(crate) pseg1: Option<u32>,
    pub(crate) pseg2: Option<u32>,
    pub(crate) rjw: Option<u32>,
    pub(crate) smp: Option<u32>,
    pub(crate) route: Option<String>,
    pub(crate) path: Option<String>,
    pub(crate) layout: Option<String>,
    pub(crate) test_id: Option<u32>,
    pub(crate) test_dlc: Option<u32>,
    pub(crate) test_dw0: Option<u32>,
    pub(crate) test_dw1: Option<u32>,
    pub(crate) rx_mask: Option<u32>,
}

// One entry in the live frame stream, built from a CANFRAME record.
#[derive(Clone, Debug)]
pub(crate) struct CanFrameStreamEntry {
    pub(crate) cycle: u32,
    pub(crate) bus: String,
    pub(crate) dir: String,
    pub(crate) id: u32,
    pub(crate) dlc: u32,
    pub(crate) bytes: [u8; 8],
    pub(crate) decode: String,
}

#[derive(Clone, Debug, Default)]
pub(crate) struct CanDashboardModel {
    pub(crate) cycle: Option<u32>,
    pub(crate) pass_count: u32,
    pub(crate) fail_count: u32,
    pub(crate) last_pass: Option<bool>,
    pub(crate) frame_match: Option<bool>,

    pub(crate) config: CanConfigView,
    pub(crate) io: CanIoView,
    pub(crate) tx_frame: CanFrameView,
    pub(crate) rx_frame: CanFrameView,
    pub(crate) errors: CanErrorView,
    pub(crate) timing: CanTimingView,

    pub(crate) proven: bool,
    pub(crate) proven_cycle: Option<u32>,
    pub(crate) proven_path: Option<String>,

    pub(crate) last_fault_stage: Option<String>,
    pub(crate) last_fault_reason: Option<String>,

    pub(crate) event_log: VecDeque<String>,
    pub(crate) frame_stream: VecDeque<CanFrameStreamEntry>,
}

impl CanDashboardModel {
    pub(crate) fn new() -> Self {
        Self::default()
    }

    pub(crate) fn total_runs(&self) -> u32 {
        self.pass_count.wrapping_add(self.fail_count)
    }

    pub(crate) fn push_event(&mut self, event: impl Into<String>) {
        self.event_log.push_back(event.into());
        while self.event_log.len() > EVENT_LOG_LIMIT {
            let _ = self.event_log.pop_front();
        }
    }

    pub(crate) fn push_frame(&mut self, entry: CanFrameStreamEntry) {
        self.frame_stream.push_back(entry);
        while self.frame_stream.len() > FRAME_STREAM_LIMIT {
            let _ = self.frame_stream.pop_front();
        }
    }
}

// ============================================================================
// Footer
// File: model.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/tools/can-dashboard/src/model.rs
// Version: v0.2.1-no-frame-elapsed
// Creation date: 2026-06-12
// Timestamp: 2026-06-13
// ============================================================================