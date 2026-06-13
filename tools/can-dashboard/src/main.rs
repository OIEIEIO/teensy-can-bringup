// ============================================================================
// File: main.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/tools/can-dashboard/src/main.rs
// Version: v0.4.0-fixed-rate-render-loop
// Purpose:
//   Linux host entry point for the Teensy CAN dashboard. Reads firmware log
//   records from piped stdin, parses records into the dashboard model, and
//   redraws the ratatui terminal at a fixed rate instead of drawing once per
//   input line. This prevents high-rate CANFRAME records from starving status,
//   event log updates, keyboard exit handling, or terminal redraw behavior.
// Created: 2026-06-10
// Timestamp: 2026-06-11
// ============================================================================

mod model;
mod parser;
mod ui;
mod units;

use crate::model::CanDashboardModel;
use crossterm::event::{self, Event, KeyCode, KeyModifiers};
use crossterm::execute;
use crossterm::terminal::{
    disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen,
};
use ratatui::backend::CrosstermBackend;
use ratatui::Terminal;
use std::io::{self, BufRead, IsTerminal, Stdout};
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

const UI_TICK: Duration = Duration::from_millis(50);
const KEY_POLL: Duration = Duration::from_millis(5);
const MAX_LINES_PER_TICK: usize = 512;
const INPUT_CHANNEL_DEPTH: usize = 4096;

enum AppEvent {
    Line(String),
    Eof,
}

fn main() -> io::Result<()> {
    if io::stdin().is_terminal() {
        eprintln!("Usage: cat /dev/ttyACM0 | can-dashboard");
        eprintln!("stdin must be piped firmware log output, not a TTY.");
        std::process::exit(1);
    }

    let mut terminal = start_terminal()?;
    let run_result = run_app(&mut terminal);
    let stop_result = stop_terminal(&mut terminal);

    run_result.and(stop_result)
}

fn start_terminal() -> io::Result<Terminal<CrosstermBackend<Stdout>>> {
    enable_raw_mode()?;

    let mut stdout = io::stdout();
    execute!(stdout, EnterAlternateScreen)?;

    let backend = CrosstermBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;
    terminal.clear()?;

    Ok(terminal)
}

fn stop_terminal(terminal: &mut Terminal<CrosstermBackend<Stdout>>) -> io::Result<()> {
    disable_raw_mode()?;
    execute!(terminal.backend_mut(), LeaveAlternateScreen)?;
    terminal.show_cursor()?;

    Ok(())
}

fn run_app(terminal: &mut Terminal<CrosstermBackend<Stdout>>) -> io::Result<()> {
    let (tx, rx) = mpsc::sync_channel::<AppEvent>(INPUT_CHANNEL_DEPTH);

    thread::spawn(move || {
        let stdin = io::stdin();

        for line_result in stdin.lock().lines() {
            match line_result {
                Ok(line) => {
                    if tx.send(AppEvent::Line(line)).is_err() {
                        return;
                    }
                }
                Err(_) => break,
            }
        }

        let _ = tx.send(AppEvent::Eof);
    });

    let mut model = CanDashboardModel::new();
    let mut last_draw = Instant::now();
    let mut redraw_needed = true;
    let mut eof_seen = false;

    terminal.draw(|frame| ui::render_dashboard(frame, &model))?;

    loop {
        if event::poll(KEY_POLL)? {
            if let Event::Key(key) = event::read()? {
                let quit = match key.code {
                    KeyCode::Char('q') | KeyCode::Esc => true,
                    KeyCode::Char('c') if key.modifiers.contains(KeyModifiers::CONTROL) => true,
                    _ => false,
                };

                if quit {
                    return Ok(());
                }
            }
        }

        let mut lines_processed = 0usize;

        while lines_processed < MAX_LINES_PER_TICK {
            match rx.try_recv() {
                Ok(AppEvent::Line(line)) => {
                    parser::parse_line(&mut model, &line);
                    redraw_needed = true;
                    lines_processed += 1;
                }
                Ok(AppEvent::Eof) => {
                    eof_seen = true;
                    break;
                }
                Err(mpsc::TryRecvError::Empty) => break,
                Err(mpsc::TryRecvError::Disconnected) => {
                    eof_seen = true;
                    break;
                }
            }
        }

        if redraw_needed && last_draw.elapsed() >= UI_TICK {
            terminal.draw(|frame| ui::render_dashboard(frame, &model))?;
            last_draw = Instant::now();
            redraw_needed = false;
        }

        if eof_seen {
            if redraw_needed {
                terminal.draw(|frame| ui::render_dashboard(frame, &model))?;
            }

            return Ok(());
        }
    }
}

// ============================================================================
// Footer
// File: main.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/tools/can-dashboard/src/main.rs
// Version: v0.4.0-fixed-rate-render-loop
// Created: 2026-06-10
// Timestamp: 2026-06-11
// End of file
// ============================================================================