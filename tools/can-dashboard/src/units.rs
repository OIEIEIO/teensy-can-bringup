// ============================================================================
// File: units.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/tools/can-dashboard/src/units.rs
// Version: v0.2.0-canframe-stream
// Purpose:
//   Display formatting helpers for the Teensy CAN dashboard. Converts optional
//   parsed values into stable text for terminal rendering without changing
//   parser state or firmware record formats.
// Changes from v0.1.0:
//   - util_x100_text: formats bus utilization from util_x100 field.
//   - bytes_text: formats a byte slice as space-separated hex for frame stream.
// ============================================================================

pub(crate) fn bool_text(value: Option<bool>) -> &'static str {
    match value {
        Some(true) => "yes",
        Some(false) => "no",
        None => "--",
    }
}

pub(crate) fn u32_text(value: Option<u32>) -> String {
    match value {
        Some(number) => number.to_string(),
        None => "--".to_string(),
    }
}

pub(crate) fn percent_text(value: Option<u32>) -> String {
    match value {
        Some(number) => format!("{}%", number),
        None => "--".to_string(),
    }
}

pub(crate) fn hex11_text(value: Option<u32>) -> String {
    match value {
        Some(number) => format!("0x{:03X}", number & 0x7FF),
        None => "--".to_string(),
    }
}

pub(crate) fn hex32_text(value: Option<u32>) -> String {
    match value {
        Some(number) => format!("0x{:08X}", number),
        None => "--".to_string(),
    }
}

pub(crate) fn rate_x100_text(value: Option<u32>) -> String {
    match value {
        Some(number) => {
            let whole = number / 100;
            let frac = number % 100;
            format!("{}.{:02} frame/s", whole, frac)
        }
        None => "--".to_string(),
    }
}

// Format util_x100 as a percentage with two decimal places.
// util_x100=2 -> "0.02%", util_x100=314 -> "3.14%"
pub(crate) fn util_x100_text(value: Option<u32>) -> String {
    match value {
        Some(number) => {
            let whole = number / 100;
            let frac = number % 100;
            format!("{}.{:02}%", whole, frac)
        }
        None => "--".to_string(),
    }
}

pub(crate) fn elapsed_text(value_ms: Option<u32>) -> String {
    let Some(value_ms) = value_ms else {
        return "--".to_string();
    };

    let total_seconds = value_ms / 1000;
    let millis = value_ms % 1000;
    let minutes = total_seconds / 60;
    let seconds = total_seconds % 60;

    format!("{}:{:02}.{:03}", minutes, seconds, millis)
}

pub(crate) fn str_text(value: Option<&str>) -> String {
    match value {
        Some(text) if !text.is_empty() => text.to_string(),
        _ => "--".to_string(),
    }
}

// Format up to dlc bytes from a byte array as space-separated uppercase hex.
// dlc=8, bytes=[0xDE,0xAD,...] -> "DE AD BE EF CA FE F0 0D"
pub(crate) fn bytes_text(bytes: &[u8; 8], dlc: u32) -> String {
    let count = (dlc as usize).min(8);
    bytes[..count]
        .iter()
        .map(|b| format!("{:02X}", b))
        .collect::<Vec<_>>()
        .join(" ")
}

// ============================================================================
// Footer
// File: units.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/tools/can-dashboard/src/units.rs
// Version: v0.2.0-canframe-stream
// Creation date: 2026-06-12
// Timestamp: 2026-06-12T01:24:36Z
// ============================================================================