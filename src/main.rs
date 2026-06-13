// ============================================================================
// File: main.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/src/main.rs
// Version: v0.11.3-voltage-distribution-decode
// Purpose:
//   Teensy 4.0 CAN bring-up staging firmware for external CAN1 -> CAN2
//   testing through two transceiver modules.
//
//   Test mode:
//   - Single Teensy.
//   - CAN1 acts as simulated drone CAN transmitter.
//   - CAN2 acts as receiver/verifier.
//   - Sends a 64-message drone-style catalog every cycle.
//   - Uses one reusable CAN2 RX mailbox for deterministic send/read/verify.
//   - Keeps the current dashboard CANFRAME log contract.
//
//   Traffic target:
//   - CAN bitrate: 500000 bps.
//   - Catalog size: 64 full DLC=8 frames.
//   - Cycle delay: 100 ms.
//   - Target frame rate from scheduler: 640 CAN frames/sec.
//   - Dashboard log output: 1280 CANFRAME records/sec because TX + RX.
//
//   Keeps:
//   - USB logging.
//   - Pin 13 LED heartbeat.
//   - CAN1 -> CAN2 external transceiver path.
//   - RX lock-safe polling rule.
//   - RX mailbox read sequence: CS -> ID -> DW0 -> DW1 -> TIMER unlock.
//   - Clean dashboard log contract.
//
//   Changes from v0.11.2:
//   - Keeps dashboard refresh behavior, CANFRAME log contract, 64-frame
//     catalog size, CAN1/CAN2 setup, mailbox handling, and pass/fail logic.
//   - Changes only ID 0x145 voltage_distribution payload packing.
//   - Packs 0x145 as four little-endian 16-bit fields for dashboard decoding:
//     B0-B1 main_battery_mv, B2-B3 rail_5v_mv, B4-B5 rail_3v3_mv,
//     B6-B7 current_ca.
// Created: 2026-06-10
// Timestamp: 2026-06-12
// ============================================================================

#![no_std]
#![no_main]

use teensy4_panic as _;

#[rtic::app(device = teensy4_bsp, peripherals = true, dispatchers = [KPP])]
mod app {
    use bsp::board;
    use bsp::ral::can;
    use bsp::ral::ccm;
    use bsp::ral::iomuxc;
    use teensy4_bsp as bsp;

    use imxrt_log as logging;
    use rtic_monotonics::systick::*;

    struct DroneMessage {
        id: u32,
        decode: &'static str,
    }

    const CCM_CSCMR2_ADDR: usize = 0x400F_C020;

    const CCM_CSCMR2_CAN_CLK_FIELD_MASK: u32 = 0x0000_03FC;
    const CCM_CSCMR2_CAN_CLK_SEL_24MHZ: u32 = 1 << 8;
    const CCM_CSCMR2_CAN_CLK_PODF_DIV1: u32 = 0 << 2;
    const CCM_CSCMR2_CAN_CLK_24MHZ_FIELD: u32 =
        CCM_CSCMR2_CAN_CLK_SEL_24MHZ | CCM_CSCMR2_CAN_CLK_PODF_DIV1;

    const CCM_CCGR0_CAN1_FULL_GATE_MASK: u32 = 0x0003_C000;
    const CCM_CCGR0_CAN2_FULL_GATE_MASK: u32 = 0x003C_0000;
    const CCM_CCGR0_CAN12_FULL_GATE_MASK: u32 =
        CCM_CCGR0_CAN1_FULL_GATE_MASK | CCM_CCGR0_CAN2_FULL_GATE_MASK;

    const FLEXCAN1_MB_RAM_BASE: u32 = 0x401D_0080;
    const FLEXCAN2_MB_RAM_BASE: u32 = 0x401D_4080;

    const CAN1_MB0_RESERVED_OFFSET: usize = 0;
    const CAN1_MB1_TX_OFFSET: usize = 4;
    const CAN1_MB2_INACTIVE_OFFSET: usize = 8;

    const CAN2_MB0_RESERVED_OFFSET: usize = 0;
    const CAN2_MB1_INACTIVE_OFFSET: usize = 4;
    const CAN2_MB2_RX_OFFSET: usize = 8;

    const MCR_MDIS: u32 = 1 << 31;
    const MCR_RFEN: u32 = 1 << 29;
    const MCR_FRZACK: u32 = 1 << 24;
    const MCR_LPMACK: u32 = 1 << 20;
    const MCR_IRMQ_MASK: u32 = 1 << 16;
    const MCR_MAXMB_MASK: u32 = 0x0000_007F;
    const MCR_MAXMB_TARGET_CAN1: u32 = 2;
    const MCR_MAXMB_TARGET_CAN2: u32 = 2;

    const CTRL1_LPB_MASK: u32 = 1 << 12;
    const CTRL1_OLD_WRONG_LPB_MASK: u32 = 1 << 6;

    // 24 MHz CAN clock / ((PRESDIV + 1) * TQ)
    // PRESDIV=2, PROPSEG=5, PSEG1=4, PSEG2=3
    // TQ = 1 + (PROPSEG + 1) + (PSEG1 + 1) + (PSEG2 + 1) = 16
    // bitrate = 24_000_000 / (3 * 16) = 500_000 bps
    const CTRL1_TIMING: u32 = (2 << 24) | (1 << 22) | (4 << 19) | (3 << 16) | (5 << 0);
    const CAN_BITRATE_BPS_DERIVED: u32 = 500_000;

    const CTRL1_PRESDIV: u32 = (CTRL1_TIMING >> 24) & 0xFF;
    const CTRL1_RJW: u32 = (CTRL1_TIMING >> 22) & 0x3;
    const CTRL1_PSEG1: u32 = (CTRL1_TIMING >> 19) & 0x7;
    const CTRL1_PSEG2: u32 = (CTRL1_TIMING >> 16) & 0x7;
    const CTRL1_SMP: u32 = (CTRL1_TIMING >> 7) & 0x1;

    const BITS_PER_STANDARD_DLC8_FRAME: u32 = 111;

    const MB_CODE_TX_INACTIVE: u32 = 0x8 << 24;
    const MB_CODE_RX_INACTIVE: u32 = 0x0 << 24;
    const MB_CODE_RX_EMPTY: u32 = 0x4 << 24;
    const MB_CODE_TX_DATA: u32 = 0xC << 24;

    const RX_CODE_FULL: u32 = 0x2;
    const CAN2_RXMGMASK_EXACT: u32 = 0x1FFF_FFFF;

    const DRONE_DLC: u32 = 8;
    const DRONE_MESSAGE_COUNT: u32 = 64;

    const LOOP_DELAY_MS: u32 = 100;
    const DASHBOARD_REFRESH_CYCLES: u32 = 100;
    const CAN2_RX_MB2_IFLAG: u32 = 1 << 2;

    const IOMUXC_MUX_CAN1_TX_P22_AD_B1_08_ALT2: u32 = 0x0000_0002;
    const IOMUXC_MUX_CAN1_RX_P23_AD_B1_09_ALT2: u32 = 0x0000_0002;
    const IOMUXC_MUX_CAN2_TX_P1_AD_B0_02_ALT0: u32 = 0x0000_0000;
    const IOMUXC_MUX_CAN2_RX_P0_AD_B0_03_ALT0: u32 = 0x0000_0000;
    const IOMUXC_DAISY_CAN1_RX_P23_AD_B1_09_ALT2: u32 = 0x0000_0002;
    const IOMUXC_DAISY_CAN2_RX_P0_AD_B0_03_ALT0: u32 = 0x0000_0001;

    const DRONE_MESSAGES: [DroneMessage; 64] = [
        DroneMessage { id: 0x100, decode: "system_heartbeat" },
        DroneMessage { id: 0x101, decode: "system_time_sync" },
        DroneMessage { id: 0x102, decode: "system_health" },
        DroneMessage { id: 0x103, decode: "system_fault_summary" },
        DroneMessage { id: 0x104, decode: "scheduler_status" },
        DroneMessage { id: 0x105, decode: "comms_status" },
        DroneMessage { id: 0x106, decode: "node_temperature" },
        DroneMessage { id: 0x107, decode: "boot_counter" },

        DroneMessage { id: 0x110, decode: "rc_channels_0_3" },
        DroneMessage { id: 0x111, decode: "rc_channels_4_7" },
        DroneMessage { id: 0x112, decode: "rc_switches_modes" },
        DroneMessage { id: 0x113, decode: "rc_link_status" },
        DroneMessage { id: 0x114, decode: "control_roll_pitch" },
        DroneMessage { id: 0x115, decode: "control_yaw_throttle" },
        DroneMessage { id: 0x116, decode: "arming_state" },
        DroneMessage { id: 0x117, decode: "failsafe_state" },

        DroneMessage { id: 0x120, decode: "motor1_cmd" },
        DroneMessage { id: 0x121, decode: "motor1_status" },
        DroneMessage { id: 0x122, decode: "motor1_electrical" },
        DroneMessage { id: 0x123, decode: "motor1_temperature" },
        DroneMessage { id: 0x124, decode: "motor1_timing" },
        DroneMessage { id: 0x125, decode: "motor1_limits" },
        DroneMessage { id: 0x126, decode: "motor1_fault" },
        DroneMessage { id: 0x127, decode: "motor1_debug" },

        DroneMessage { id: 0x128, decode: "motor2_cmd" },
        DroneMessage { id: 0x129, decode: "motor2_status" },
        DroneMessage { id: 0x12A, decode: "motor2_electrical" },
        DroneMessage { id: 0x12B, decode: "motor2_temperature" },
        DroneMessage { id: 0x12C, decode: "motor2_timing" },
        DroneMessage { id: 0x12D, decode: "motor2_limits" },
        DroneMessage { id: 0x12E, decode: "motor2_fault" },
        DroneMessage { id: 0x12F, decode: "motor2_debug" },

        DroneMessage { id: 0x130, decode: "motor3_cmd" },
        DroneMessage { id: 0x131, decode: "motor3_status" },
        DroneMessage { id: 0x132, decode: "motor3_electrical" },
        DroneMessage { id: 0x133, decode: "motor3_temperature" },
        DroneMessage { id: 0x134, decode: "motor3_timing" },
        DroneMessage { id: 0x135, decode: "motor3_limits" },
        DroneMessage { id: 0x136, decode: "motor3_fault" },
        DroneMessage { id: 0x137, decode: "motor3_debug" },

        DroneMessage { id: 0x138, decode: "motor4_cmd" },
        DroneMessage { id: 0x139, decode: "motor4_status" },
        DroneMessage { id: 0x13A, decode: "motor4_electrical" },
        DroneMessage { id: 0x13B, decode: "motor4_temperature" },
        DroneMessage { id: 0x13C, decode: "motor4_timing" },
        DroneMessage { id: 0x13D, decode: "motor4_limits" },
        DroneMessage { id: 0x13E, decode: "motor4_fault" },
        DroneMessage { id: 0x13F, decode: "motor4_debug" },

        DroneMessage { id: 0x140, decode: "battery_main" },
        DroneMessage { id: 0x141, decode: "battery_cells_0_3" },
        DroneMessage { id: 0x142, decode: "power_rails" },
        DroneMessage { id: 0x143, decode: "power_limits" },
        DroneMessage { id: 0x144, decode: "current_distribution" },
        DroneMessage { id: 0x145, decode: "voltage_distribution" },
        DroneMessage { id: 0x146, decode: "bec_status" },
        DroneMessage { id: 0x147, decode: "power_faults" },

        DroneMessage { id: 0x150, decode: "imu_gyro" },
        DroneMessage { id: 0x151, decode: "imu_accel" },
        DroneMessage { id: 0x152, decode: "attitude_estimate" },
        DroneMessage { id: 0x153, decode: "altitude_estimate" },
        DroneMessage { id: 0x154, decode: "gps_position" },
        DroneMessage { id: 0x155, decode: "gps_velocity" },
        DroneMessage { id: 0x156, decode: "rangefinder_flow" },
        DroneMessage { id: 0x157, decode: "magnetometer_baro" },
    ];

    #[local]
    struct Local {
        poller: logging::Poller,
        led: board::Led,
        can1: can::CAN1,
        can2: can::CAN2,
    }

    #[shared]
    struct Shared {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        let board::Resources {
            usb,
            pins,
            mut gpio2,
            ..
        } = board::t40(cx.device);

        let led = board::led(&mut gpio2, pins.p13);

        let _can2_rx_p0 = pins.p0;
        let _can2_tx_p1 = pins.p1;
        let _can1_tx_p22 = pins.p22;
        let _can1_rx_p23 = pins.p23;

        let can1 = unsafe { can::CAN1::instance() };
        let can2 = unsafe { can::CAN2::instance() };

        let poller = logging::log::usbd(usb, logging::Interrupts::Enabled).unwrap();

        Systick::start(
            cx.core.SYST,
            board::ARM_FREQUENCY,
            rtic_monotonics::create_systick_token!(),
        );

        flexcan1_to_flexcan2::spawn().unwrap();

        led.set();

        (
            Shared {},
            Local {
                poller,
                led,
                can1,
                can2,
            },
        )
    }

    unsafe fn read_mb_word(base: u32, word_index: usize) -> u32 {
        let ptr = (base as *const u32).add(word_index);
        unsafe { core::ptr::read_volatile(ptr) }
    }

    unsafe fn write_mb_word(base: u32, word_index: usize, value: u32) {
        let ptr = (base as *mut u32).add(word_index);
        unsafe { core::ptr::write_volatile(ptr, value) };
    }

    #[task(local = [led, can1, can2])]
    async fn flexcan1_to_flexcan2(cx: flexcan1_to_flexcan2::Context) {
        let flexcan1_to_flexcan2::LocalResources {
            led,
            can1,
            can2,
            ..
        } = cx.local;

        let mut cycle = 0u32;
        let mut total_pass = 0u32;
        let mut total_fail = 0u32;
        let mut total_frame_pass = 0u32;
        let mut total_frame_fail = 0u32;

        log::info!("CANBOOT version=v0.11.3-voltage-distribution-decode file=src/main.rs");
        log::info!("CANBOOT board=teensy40 mcu=imxrt1062 mode=drone_catalog_64_one_rx_mb");
        log::info!("CANCFG clk_hz=24000000 ctrl1=0x{:08X}", CTRL1_TIMING);
        log::info!("CANCFG bitrate={} loop_ms={}", CAN_BITRATE_BPS_DERIVED, LOOP_DELAY_MS);
        log::info!(
            "CANCFG presdiv={} pseg1={} pseg2={} rjw={} smp={}",
            CTRL1_PRESDIV,
            CTRL1_PSEG1,
            CTRL1_PSEG2,
            CTRL1_RJW,
            CTRL1_SMP
        );
        log::info!("CANROUTE c1tx=P22 c1rx=P23 c2tx=P1 c2rx=P0 ack=CAN2");
        log::info!("CANPATH CAN1-XCVRA-CANH_CANL-XCVRB-CAN2");
        log::info!("CANLAYOUT c1_mb=0r_1tx_2off c2_mb=0r_1off_2rx_reuse maxmb=2");
        log::info!("CANTEST id=0x{:03X} dlc={} dw0=0x00000000", DRONE_MESSAGES[0].id, DRONE_DLC);
        log::info!("CANTEST dw1=0x00000000 mask=0x{:08X}", CAN2_RXMGMASK_EXACT);
        log::info!(
            "CANMSG catalog=drone64 frames_per_cycle={} rx_mode=one_reused_mailbox",
            DRONE_MESSAGE_COUNT
        );

        Systick::delay(500.millis()).await;

        loop {
            led.toggle();

            let ccm = unsafe { ccm::CCM::instance() };
            let iomuxc_regs = unsafe { iomuxc::IOMUXC::instance() };

            let cscmr2_before = unsafe { core::ptr::read_volatile(CCM_CSCMR2_ADDR as *const u32) };
            let cscmr2_target = (cscmr2_before & !CCM_CSCMR2_CAN_CLK_FIELD_MASK)
                | CCM_CSCMR2_CAN_CLK_24MHZ_FIELD;

            if (cscmr2_before & CCM_CSCMR2_CAN_CLK_FIELD_MASK)
                != CCM_CSCMR2_CAN_CLK_24MHZ_FIELD
            {
                unsafe {
                    core::ptr::write_volatile(CCM_CSCMR2_ADDR as *mut u32, cscmr2_target);
                }

                Systick::delay(1.millis()).await;
            }

            let cscmr2_after = unsafe { core::ptr::read_volatile(CCM_CSCMR2_ADDR as *const u32) };
            let clock_ok =
                (cscmr2_after & CCM_CSCMR2_CAN_CLK_FIELD_MASK) == CCM_CSCMR2_CAN_CLK_24MHZ_FIELD;

            if !clock_ok {
                total_fail = total_fail.wrapping_add(1);

                log::info!("CANFAULT cycle={} stage=clock reason=clk_root", cycle);
                log::info!("CANFAULT cscmr2_before=0x{:08X}", cscmr2_before);
                log::info!("CANFAULT cscmr2_target=0x{:08X}", cscmr2_target);
                log::info!("CANFAULT cscmr2_after=0x{:08X}", cscmr2_after);

                Systick::delay(LOOP_DELAY_MS.millis()).await;
                cycle = cycle.wrapping_add(1);
                continue;
            }

            let ccgr0_before = bsp::ral::read_reg!(ccm, ccm, CCGR0);
            let ccgr0_write = ccgr0_before | CCM_CCGR0_CAN12_FULL_GATE_MASK;

            bsp::ral::write_reg!(ccm, ccm, CCGR0, ccgr0_write);

            let ccgr0_after = bsp::ral::read_reg!(ccm, ccm, CCGR0);
            let gate_ok =
                (ccgr0_after & CCM_CCGR0_CAN12_FULL_GATE_MASK) == CCM_CCGR0_CAN12_FULL_GATE_MASK;

            if !gate_ok {
                total_fail = total_fail.wrapping_add(1);

                log::info!("CANFAULT cycle={} stage=gate reason=ccgr0", cycle);
                log::info!("CANFAULT ccgr0_before=0x{:08X}", ccgr0_before);
                log::info!("CANFAULT ccgr0_write=0x{:08X}", ccgr0_write);
                log::info!("CANFAULT ccgr0_after=0x{:08X}", ccgr0_after);

                Systick::delay(LOOP_DELAY_MS.millis()).await;
                cycle = cycle.wrapping_add(1);
                continue;
            }

            bsp::ral::write_reg!(
                iomuxc,
                iomuxc_regs,
                SW_MUX_CTL_PAD_GPIO_AD_B1_08,
                IOMUXC_MUX_CAN1_TX_P22_AD_B1_08_ALT2
            );
            bsp::ral::write_reg!(
                iomuxc,
                iomuxc_regs,
                SW_MUX_CTL_PAD_GPIO_AD_B1_09,
                IOMUXC_MUX_CAN1_RX_P23_AD_B1_09_ALT2
            );
            bsp::ral::write_reg!(
                iomuxc,
                iomuxc_regs,
                FLEXCAN1_RX_SELECT_INPUT,
                IOMUXC_DAISY_CAN1_RX_P23_AD_B1_09_ALT2
            );
            bsp::ral::write_reg!(
                iomuxc,
                iomuxc_regs,
                SW_MUX_CTL_PAD_GPIO_AD_B0_02,
                IOMUXC_MUX_CAN2_TX_P1_AD_B0_02_ALT0
            );
            bsp::ral::write_reg!(
                iomuxc,
                iomuxc_regs,
                SW_MUX_CTL_PAD_GPIO_AD_B0_03,
                IOMUXC_MUX_CAN2_RX_P0_AD_B0_03_ALT0
            );
            bsp::ral::write_reg!(
                iomuxc,
                iomuxc_regs,
                FLEXCAN2_RX_SELECT_INPUT,
                IOMUXC_DAISY_CAN2_RX_P0_AD_B0_03_ALT0
            );

            let mux_can1_tx =
                bsp::ral::read_reg!(iomuxc, iomuxc_regs, SW_MUX_CTL_PAD_GPIO_AD_B1_08);
            let mux_can1_rx =
                bsp::ral::read_reg!(iomuxc, iomuxc_regs, SW_MUX_CTL_PAD_GPIO_AD_B1_09);
            let daisy_can1_rx =
                bsp::ral::read_reg!(iomuxc, iomuxc_regs, FLEXCAN1_RX_SELECT_INPUT);
            let mux_can2_tx =
                bsp::ral::read_reg!(iomuxc, iomuxc_regs, SW_MUX_CTL_PAD_GPIO_AD_B0_02);
            let mux_can2_rx =
                bsp::ral::read_reg!(iomuxc, iomuxc_regs, SW_MUX_CTL_PAD_GPIO_AD_B0_03);
            let daisy_can2_rx =
                bsp::ral::read_reg!(iomuxc, iomuxc_regs, FLEXCAN2_RX_SELECT_INPUT);

            let iomux_ok = mux_can1_tx == IOMUXC_MUX_CAN1_TX_P22_AD_B1_08_ALT2
                && mux_can1_rx == IOMUXC_MUX_CAN1_RX_P23_AD_B1_09_ALT2
                && daisy_can1_rx == IOMUXC_DAISY_CAN1_RX_P23_AD_B1_09_ALT2
                && mux_can2_tx == IOMUXC_MUX_CAN2_TX_P1_AD_B0_02_ALT0
                && mux_can2_rx == IOMUXC_MUX_CAN2_RX_P0_AD_B0_03_ALT0
                && daisy_can2_rx == IOMUXC_DAISY_CAN2_RX_P0_AD_B0_03_ALT0;

            if !iomux_ok {
                total_fail = total_fail.wrapping_add(1);

                log::info!("CANFAULT cycle={} stage=iomux reason=readback", cycle);
                log::info!("CANMUX1 tx=0x{:08X} rx=0x{:08X}", mux_can1_tx, mux_can1_rx);
                log::info!("CANMUX1 daisy=0x{:08X}", daisy_can1_rx);
                log::info!("CANMUX2 tx=0x{:08X} rx=0x{:08X}", mux_can2_tx, mux_can2_rx);
                log::info!("CANMUX2 daisy=0x{:08X}", daisy_can2_rx);

                Systick::delay(LOOP_DELAY_MS.millis()).await;
                cycle = cycle.wrapping_add(1);
                continue;
            }

            let can1_mcr_stage1 = bsp::ral::read_reg!(can, can1, MCR);
            let can2_mcr_stage1 = bsp::ral::read_reg!(can, can2, MCR);

            if (can1_mcr_stage1 & MCR_MDIS) != 0 {
                bsp::ral::modify_reg!(can, can1, MCR, MDIS: 0);
            }

            if (can2_mcr_stage1 & MCR_MDIS) != 0 {
                bsp::ral::modify_reg!(can, can2, MCR, MDIS: 0);
            }

            Systick::delay(1.millis()).await;

            let can1_mcr_enabled = bsp::ral::read_reg!(can, can1, MCR);
            let can2_mcr_enabled = bsp::ral::read_reg!(can, can2, MCR);
            let can1_mdis = (can1_mcr_enabled & MCR_MDIS) >> 31;
            let can2_mdis = (can2_mcr_enabled & MCR_MDIS) >> 31;
            let can1_lpmack = (can1_mcr_enabled & MCR_LPMACK) >> 20;
            let can2_lpmack = (can2_mcr_enabled & MCR_LPMACK) >> 20;

            let mdis_ok = can1_mdis == 0 && can2_mdis == 0;

            if !mdis_ok {
                total_fail = total_fail.wrapping_add(1);

                log::info!("CANFAULT cycle={} stage=enable reason=mdis", cycle);
                log::info!(
                    "CANEN1 before=0x{:08X} after=0x{:08X}",
                    can1_mcr_stage1,
                    can1_mcr_enabled
                );
                log::info!("CANEN1 mdis={} lpmack={}", can1_mdis, can1_lpmack);
                log::info!(
                    "CANEN2 before=0x{:08X} after=0x{:08X}",
                    can2_mcr_stage1,
                    can2_mcr_enabled
                );
                log::info!("CANEN2 mdis={} lpmack={}", can2_mdis, can2_lpmack);

                Systick::delay(LOOP_DELAY_MS.millis()).await;
                cycle = cycle.wrapping_add(1);
                continue;
            }

            bsp::ral::modify_reg!(can, can1, MCR, FRZ: 1, HALT: 1);
            bsp::ral::modify_reg!(can, can2, MCR, FRZ: 1, HALT: 1);

            let mut can1_freeze_ok = false;
            let mut can2_freeze_ok = false;
            let mut can1_freeze_attempt = 0u32;
            let mut can2_freeze_attempt = 0u32;
            let mut can1_mcr_freeze_final = 0u32;
            let mut can2_mcr_freeze_final = 0u32;

            for attempt in 0..500u32 {
                let can1_mcr_poll = bsp::ral::read_reg!(can, can1, MCR);
                let can2_mcr_poll = bsp::ral::read_reg!(can, can2, MCR);

                can1_mcr_freeze_final = can1_mcr_poll;
                can2_mcr_freeze_final = can2_mcr_poll;

                if !can1_freeze_ok && ((can1_mcr_poll & MCR_FRZACK) != 0) {
                    can1_freeze_ok = true;
                    can1_freeze_attempt = attempt;
                }

                if !can2_freeze_ok && ((can2_mcr_poll & MCR_FRZACK) != 0) {
                    can2_freeze_ok = true;
                    can2_freeze_attempt = attempt;
                }

                if can1_freeze_ok && can2_freeze_ok {
                    break;
                }
            }

            if !(can1_freeze_ok && can2_freeze_ok) {
                total_fail = total_fail.wrapping_add(1);

                log::info!("CANFAULT cycle={} stage=freeze_enter reason=frzack", cycle);
                log::info!(
                    "CANFRZ1 ok={} attempt={}",
                    if can1_freeze_ok { 1 } else { 0 },
                    can1_freeze_attempt
                );
                log::info!("CANFRZ1 mcr=0x{:08X}", can1_mcr_freeze_final);
                log::info!(
                    "CANFRZ2 ok={} attempt={}",
                    if can2_freeze_ok { 1 } else { 0 },
                    can2_freeze_attempt
                );
                log::info!("CANFRZ2 mcr=0x{:08X}", can2_mcr_freeze_final);

                Systick::delay(LOOP_DELAY_MS.millis()).await;
                cycle = cycle.wrapping_add(1);
                continue;
            }

            let can1_mcr_before_maxmb = bsp::ral::read_reg!(can, can1, MCR);
            let can2_mcr_before_maxmb = bsp::ral::read_reg!(can, can2, MCR);
            let can1_mcr_set_maxmb =
                (can1_mcr_before_maxmb & !MCR_MAXMB_MASK & !MCR_RFEN) | MCR_MAXMB_TARGET_CAN1;
            let can2_mcr_set_maxmb =
                (can2_mcr_before_maxmb & !MCR_MAXMB_MASK & !MCR_RFEN) | MCR_MAXMB_TARGET_CAN2;

            bsp::ral::write_reg!(can, can1, MCR, can1_mcr_set_maxmb);
            bsp::ral::write_reg!(can, can2, MCR, can2_mcr_set_maxmb);

            let can1_mcr_after_maxmb = bsp::ral::read_reg!(can, can1, MCR);
            let can2_mcr_after_maxmb = bsp::ral::read_reg!(can, can2, MCR);
            let can1_maxmb_after = can1_mcr_after_maxmb & MCR_MAXMB_MASK;
            let can2_maxmb_after = can2_mcr_after_maxmb & MCR_MAXMB_MASK;
            let can1_rfen_after = (can1_mcr_after_maxmb & MCR_RFEN) >> 29;
            let can2_rfen_after = (can2_mcr_after_maxmb & MCR_RFEN) >> 29;
            let can2_irmq_after = (can2_mcr_after_maxmb & MCR_IRMQ_MASK) >> 16;

            let maxmb_ok = can1_maxmb_after == MCR_MAXMB_TARGET_CAN1
                && can2_maxmb_after == MCR_MAXMB_TARGET_CAN2
                && can1_rfen_after == 0
                && can2_rfen_after == 0;

            if !maxmb_ok {
                total_fail = total_fail.wrapping_add(1);

                log::info!("CANFAULT cycle={} stage=maxmb reason=mcr", cycle);
                log::info!("CANMAX1 before=0x{:08X}", can1_mcr_before_maxmb);
                log::info!("CANMAX1 write=0x{:08X}", can1_mcr_set_maxmb);
                log::info!("CANMAX1 after=0x{:08X}", can1_mcr_after_maxmb);
                log::info!("CANMAX1 maxmb={} rfen={}", can1_maxmb_after, can1_rfen_after);
                log::info!("CANMAX2 before=0x{:08X}", can2_mcr_before_maxmb);
                log::info!("CANMAX2 write=0x{:08X}", can2_mcr_set_maxmb);
                log::info!("CANMAX2 after=0x{:08X}", can2_mcr_after_maxmb);
                log::info!(
                    "CANMAX2 maxmb={} rfen={} irmq={}",
                    can2_maxmb_after,
                    can2_rfen_after,
                    can2_irmq_after
                );

                Systick::delay(LOOP_DELAY_MS.millis()).await;
                cycle = cycle.wrapping_add(1);
                continue;
            }

            bsp::ral::write_reg!(can, can1, CTRL1, CTRL1_TIMING);
            bsp::ral::write_reg!(can, can2, CTRL1, CTRL1_TIMING);

            let can1_ctrl1_rb = bsp::ral::read_reg!(can, can1, CTRL1);
            let can2_ctrl1_rb = bsp::ral::read_reg!(can, can2, CTRL1);
            let can1_lpb12 = (can1_ctrl1_rb & CTRL1_LPB_MASK) >> 12;
            let can2_lpb12 = (can2_ctrl1_rb & CTRL1_LPB_MASK) >> 12;
            let can1_old_lpb6 = (can1_ctrl1_rb & CTRL1_OLD_WRONG_LPB_MASK) >> 6;
            let can2_old_lpb6 = (can2_ctrl1_rb & CTRL1_OLD_WRONG_LPB_MASK) >> 6;

            let ctrl1_ok = can1_ctrl1_rb == CTRL1_TIMING && can2_ctrl1_rb == CTRL1_TIMING;

            if !ctrl1_ok {
                total_fail = total_fail.wrapping_add(1);

                log::info!("CANFAULT cycle={} stage=ctrl1 reason=timing", cycle);
                log::info!("CANCTRL expected=0x{:08X}", CTRL1_TIMING);
                log::info!("CANCTRL1 got=0x{:08X}", can1_ctrl1_rb);
                log::info!("CANCTRL1 lpb12={} old_lpb6={}", can1_lpb12, can1_old_lpb6);
                log::info!("CANCTRL2 got=0x{:08X}", can2_ctrl1_rb);
                log::info!("CANCTRL2 lpb12={} old_lpb6={}", can2_lpb12, can2_old_lpb6);

                Systick::delay(LOOP_DELAY_MS.millis()).await;
                cycle = cycle.wrapping_add(1);
                continue;
            }

            unsafe {
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB0_RESERVED_OFFSET + 0, MB_CODE_TX_INACTIVE);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB0_RESERVED_OFFSET + 1, 0x0000_0000);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB0_RESERVED_OFFSET + 2, 0x0000_0000);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB0_RESERVED_OFFSET + 3, 0x0000_0000);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB0_RESERVED_OFFSET + 0, MB_CODE_TX_INACTIVE);

                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 0, MB_CODE_TX_INACTIVE);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 1, DRONE_MESSAGES[0].id << 18);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 2, 0x0000_0000);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 3, 0x0000_0000);

                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB2_INACTIVE_OFFSET + 0, MB_CODE_TX_INACTIVE);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB2_INACTIVE_OFFSET + 1, 0x0000_0000);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB2_INACTIVE_OFFSET + 2, 0x0000_0000);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB2_INACTIVE_OFFSET + 3, 0x0000_0000);

                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB0_RESERVED_OFFSET + 0, MB_CODE_TX_INACTIVE);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB0_RESERVED_OFFSET + 1, 0x0000_0000);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB0_RESERVED_OFFSET + 2, 0x0000_0000);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB0_RESERVED_OFFSET + 3, 0x0000_0000);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB0_RESERVED_OFFSET + 0, MB_CODE_TX_INACTIVE);

                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB1_INACTIVE_OFFSET + 0, MB_CODE_TX_INACTIVE);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB1_INACTIVE_OFFSET + 1, 0x0000_0000);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB1_INACTIVE_OFFSET + 2, 0x0000_0000);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB1_INACTIVE_OFFSET + 3, 0x0000_0000);

                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 0, MB_CODE_RX_INACTIVE);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 1, DRONE_MESSAGES[0].id << 18);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 2, 0x0000_0000);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 3, 0x0000_0000);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 0, MB_CODE_RX_EMPTY);
            }

            bsp::ral::write_reg!(can, can2, RXMGMASK, CAN2_RXMGMASK_EXACT);

            let can1_mb0_cs =
                unsafe { read_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB0_RESERVED_OFFSET + 0) };
            let can1_mb1_cs =
                unsafe { read_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 0) };
            let can1_mb2_cs =
                unsafe { read_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB2_INACTIVE_OFFSET + 0) };

            let can2_mb0_cs =
                unsafe { read_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB0_RESERVED_OFFSET + 0) };
            let can2_mb1_cs =
                unsafe { read_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB1_INACTIVE_OFFSET + 0) };
            let can2_mb2_cs =
                unsafe { read_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 0) };
            let can2_mb2_id =
                unsafe { read_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 1) };

            let can2_rxmgmask = bsp::ral::read_reg!(can, can2, RXMGMASK);
            let can2_rxfgmask = bsp::ral::read_reg!(can, can2, RXFGMASK);
            let can2_rx14mask = bsp::ral::read_reg!(can, can2, RX14MASK);
            let can2_rx15mask = bsp::ral::read_reg!(can, can2, RX15MASK);
            let can2_imask1 = bsp::ral::read_reg!(can, can2, IMASK1);
            let can2_ctrl2 = bsp::ral::read_reg!(can, can2, CTRL2);
            let can2_mcr_maskdiag = bsp::ral::read_reg!(can, can2, MCR);
            let can2_irmq_maskdiag = (can2_mcr_maskdiag & MCR_IRMQ_MASK) >> 16;

            let mailbox_ok = can1_mb0_cs == MB_CODE_TX_INACTIVE
                && can1_mb1_cs == MB_CODE_TX_INACTIVE
                && can1_mb2_cs == MB_CODE_TX_INACTIVE
                && can2_mb0_cs == MB_CODE_TX_INACTIVE
                && can2_mb1_cs == MB_CODE_TX_INACTIVE
                && can2_mb2_cs == MB_CODE_RX_EMPTY
                && can2_mb2_id == (DRONE_MESSAGES[0].id << 18)
                && can2_rxmgmask == CAN2_RXMGMASK_EXACT;

            if !mailbox_ok {
                total_fail = total_fail.wrapping_add(1);

                log::info!("CANFAULT cycle={} stage=mailbox reason=setup", cycle);
                log::info!("CANMB1 mb0=0x{:08X} mb1=0x{:08X}", can1_mb0_cs, can1_mb1_cs);
                log::info!("CANMB1 mb2=0x{:08X}", can1_mb2_cs);
                log::info!("CANMB2 mb0=0x{:08X} mb1=0x{:08X}", can2_mb0_cs, can2_mb1_cs);
                log::info!("CANMB2 mb2=0x{:08X} id2=0x{:08X}", can2_mb2_cs, can2_mb2_id);
                log::info!("CANMASK mcr=0x{:08X} irmq={}", can2_mcr_maskdiag, can2_irmq_maskdiag);
                log::info!("CANMASK mg=0x{:08X} fg=0x{:08X}", can2_rxmgmask, can2_rxfgmask);
                log::info!("CANMASK r14=0x{:08X} r15=0x{:08X}", can2_rx14mask, can2_rx15mask);
                log::info!("CANMASK imask1=0x{:08X} ctrl2=0x{:08X}", can2_imask1, can2_ctrl2);

                Systick::delay(LOOP_DELAY_MS.millis()).await;
                cycle = cycle.wrapping_add(1);
                continue;
            }

            bsp::ral::write_reg!(can, can1, IFLAG1, 0x0000_0007);
            bsp::ral::write_reg!(can, can2, IFLAG1, 0x0000_0007);

            let can1_iflag_cleared = bsp::ral::read_reg!(can, can1, IFLAG1);
            let can2_iflag_cleared = bsp::ral::read_reg!(can, can2, IFLAG1);

            bsp::ral::modify_reg!(can, can2, MCR, FRZ: 0, HALT: 0);
            Systick::delay(1.millis()).await;
            bsp::ral::modify_reg!(can, can1, MCR, FRZ: 0, HALT: 0);
            Systick::delay(10.millis()).await;

            let mut can1_unfreeze_ok = false;
            let mut can2_unfreeze_ok = false;
            let mut can1_unfreeze_attempt = 0u32;
            let mut can2_unfreeze_attempt = 0u32;
            let mut can1_mcr_unfreeze_final = 0u32;
            let mut can2_mcr_unfreeze_final = 0u32;

            for attempt in 0..10_000u32 {
                let can1_mcr_poll = bsp::ral::read_reg!(can, can1, MCR);
                let can2_mcr_poll = bsp::ral::read_reg!(can, can2, MCR);

                can1_mcr_unfreeze_final = can1_mcr_poll;
                can2_mcr_unfreeze_final = can2_mcr_poll;

                if !can1_unfreeze_ok && ((can1_mcr_poll & MCR_FRZACK) == 0) {
                    can1_unfreeze_ok = true;
                    can1_unfreeze_attempt = attempt;
                }

                if !can2_unfreeze_ok && ((can2_mcr_poll & MCR_FRZACK) == 0) {
                    can2_unfreeze_ok = true;
                    can2_unfreeze_attempt = attempt;
                }

                if can1_unfreeze_ok && can2_unfreeze_ok {
                    break;
                }
            }

            if !(can1_unfreeze_ok && can2_unfreeze_ok) {
                let can1_ctrl1_final = bsp::ral::read_reg!(can, can1, CTRL1);
                let can2_ctrl1_final = bsp::ral::read_reg!(can, can2, CTRL1);
                let can1_esr1_final = bsp::ral::read_reg!(can, can1, ESR1);
                let can2_esr1_final = bsp::ral::read_reg!(can, can2, ESR1);
                let can1_ecr_final = bsp::ral::read_reg!(can, can1, ECR);
                let can2_ecr_final = bsp::ral::read_reg!(can, can2, ECR);

                total_fail = total_fail.wrapping_add(1);

                log::info!("CANFAULT cycle={} stage=freeze_exit reason=frzack", cycle);
                log::info!(
                    "CANUFZ1 ok={} attempt={}",
                    if can1_unfreeze_ok { 1 } else { 0 },
                    can1_unfreeze_attempt
                );
                log::info!(
                    "CANUFZ1 mcr=0x{:08X} ctrl1=0x{:08X}",
                    can1_mcr_unfreeze_final,
                    can1_ctrl1_final
                );
                log::info!("CANUFZ1 esr1=0x{:08X} ecr=0x{:08X}", can1_esr1_final, can1_ecr_final);
                log::info!(
                    "CANUFZ2 ok={} attempt={}",
                    if can2_unfreeze_ok { 1 } else { 0 },
                    can2_unfreeze_attempt
                );
                log::info!(
                    "CANUFZ2 mcr=0x{:08X} ctrl1=0x{:08X}",
                    can2_mcr_unfreeze_final,
                    can2_ctrl1_final
                );
                log::info!("CANUFZ2 esr1=0x{:08X} ecr=0x{:08X}", can2_esr1_final, can2_ecr_final);

                Systick::delay(LOOP_DELAY_MS.millis()).await;
                cycle = cycle.wrapping_add(1);
                continue;
            }

            let can1_pre_mcr = bsp::ral::read_reg!(can, can1, MCR);
            let can1_pre_ctrl1 = bsp::ral::read_reg!(can, can1, CTRL1);
            let can1_pre_iflag1 = bsp::ral::read_reg!(can, can1, IFLAG1);
            let can1_pre_esr1 = bsp::ral::read_reg!(can, can1, ESR1);
            let can1_pre_ecr = bsp::ral::read_reg!(can, can1, ECR);
            let can1_pre_timer = bsp::ral::read_reg!(can, can1, TIMER);
            let can2_pre_mcr = bsp::ral::read_reg!(can, can2, MCR);
            let can2_pre_ctrl1 = bsp::ral::read_reg!(can, can2, CTRL1);
            let can2_pre_iflag1 = bsp::ral::read_reg!(can, can2, IFLAG1);
            let can2_pre_esr1 = bsp::ral::read_reg!(can, can2, ESR1);
            let can2_pre_ecr = bsp::ral::read_reg!(can, can2, ECR);
            let can2_pre_timer = bsp::ral::read_reg!(can, can2, TIMER);

            bsp::ral::write_reg!(can, can1, IFLAG1, 0x0000_0007);
            bsp::ral::write_reg!(can, can2, IFLAG1, 0x0000_0007);

            unsafe {
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB0_RESERVED_OFFSET + 0, MB_CODE_TX_INACTIVE);
                write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB0_RESERVED_OFFSET + 0, MB_CODE_TX_INACTIVE);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB0_RESERVED_OFFSET + 0, MB_CODE_TX_INACTIVE);
                write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB0_RESERVED_OFFSET + 0, MB_CODE_TX_INACTIVE);
            }

            let can1_mb0_errata_rb =
                unsafe { read_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB0_RESERVED_OFFSET + 0) };
            let can2_mb0_errata_rb =
                unsafe { read_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB0_RESERVED_OFFSET + 0) };

            let completed_cycles = cycle.wrapping_add(1);
            let elapsed_ms = completed_cycles.wrapping_mul(LOOP_DELAY_MS);
            let uptime_s = elapsed_ms / 1000;
            let seq = cycle & 0xFFFF;

            let rc_base = 1000u32 + ((cycle.wrapping_mul(11)) % 1001);
            let throttle_base = cycle.wrapping_mul(37) % 1001;
            let voltage_mv = 14800u32.wrapping_add((cycle % 120).wrapping_mul(5));
            let current_ca = 120u32.wrapping_add((throttle_base.wrapping_mul(3)) / 10);
            let total_error_count = total_frame_fail & 0xFFFF;

            let mut cycle_frame_pass = 0u32;
            let mut cycle_frame_fail = 0u32;
            let mut cycle_tx_ok = true;
            let mut cycle_rx_ok = true;
            let mut cycle_match = true;
            let mut any_late = false;
            let mut last_can1_tx_attempt = 0u32;
            let mut last_can2_rx_attempt = 0u32;
            let mut last_tx_id = 0u32;
            let mut last_tx_dlc = 0u32;
            let mut last_tx_cs = 0u32;
            let mut last_tx_dw0 = 0u32;
            let mut last_tx_dw1 = 0u32;
            let mut last_rx_id = 0u32;
            let mut last_rx_dlc = 0u32;
            let mut last_rx_code = 0u32;
            let mut last_rx_dw0 = 0u32;
            let mut last_rx_dw1 = 0u32;
            let mut final_can1_iflag1 = 0u32;
            let mut final_can2_iflag1 = 0u32;
            let mut final_can1_esr1 = 0u32;
            let mut final_can2_esr1 = 0u32;
            let mut final_can1_ecr = 0u32;
            let mut final_can2_ecr = 0u32;

            for frame_index in 0..DRONE_MESSAGE_COUNT {
                let message = &DRONE_MESSAGES[frame_index as usize];
                let tx_id = message.id;
                let decode = message.decode;
                let frame_seq = cycle
                    .wrapping_mul(DRONE_MESSAGE_COUNT)
                    .wrapping_add(frame_index)
                    & 0xFFFF;

                let (tx_dw0, tx_dw1) = match frame_index {
                    0 => {
                        let node_state = 1u32;
                        let dw0 = ((uptime_s & 0xFFFF) << 16) | (node_state & 0xFFFF);
                        let dw1 = ((total_error_count & 0xFFFF) << 16) | seq;
                        (dw0, dw1)
                    }
                    1 => {
                        let dw0 = elapsed_ms;
                        let dw1 = frame_seq;
                        (dw0, dw1)
                    }
                    2 => {
                        let cpu_load_x10 = 230u32.wrapping_add(cycle % 70);
                        let loop_us = LOOP_DELAY_MS.wrapping_mul(1000) & 0xFFFF;
                        let status_flags = 1u32;
                        let dw0 = ((cpu_load_x10 & 0xFFFF) << 16) | loop_us;
                        let dw1 = ((status_flags & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    3 => {
                        let active_faults = 0u32;
                        let latched_faults = total_error_count;
                        let last_fault_code = 0u32;
                        let dw0 = ((active_faults & 0xFFFF) << 16) | (latched_faults & 0xFFFF);
                        let dw1 = ((last_fault_code & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    4 => {
                        let scheduler_load_x10 = 410u32.wrapping_add(cycle % 60);
                        let missed_deadlines = total_error_count;
                        let active_tasks = 8u32;
                        let dw0 = ((scheduler_load_x10 & 0xFFFF) << 16) | (missed_deadlines & 0xFFFF);
                        let dw1 = ((active_tasks & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    5 => {
                        let rx_packets = frame_seq;
                        let tx_packets = frame_seq;
                        let dropped_packets = 0u32;
                        let dw0 = ((rx_packets & 0xFFFF) << 16) | (tx_packets & 0xFFFF);
                        let dw1 = ((dropped_packets & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    6 => {
                        let mcu_temp_cdeg = 360u32.wrapping_add(cycle % 70);
                        let board_temp_cdeg = 340u32.wrapping_add(cycle % 60);
                        let regulator_temp_cdeg = 330u32.wrapping_add(cycle % 55);
                        let dw0 = ((mcu_temp_cdeg & 0xFFFF) << 16) | (board_temp_cdeg & 0xFFFF);
                        let dw1 = ((regulator_temp_cdeg & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    7 => {
                        let boot_count = 1u32;
                        let reset_reason = 0u32;
                        let dw0 = ((boot_count & 0xFFFF) << 16) | (reset_reason & 0xFFFF);
                        let dw1 = frame_seq;
                        (dw0, dw1)
                    }

                    8 => {
                        let ch0 = rc_base;
                        let ch1 = 1000u32 + ((cycle.wrapping_mul(13).wrapping_add(100)) % 1001);
                        let ch2 = 1000u32 + ((cycle.wrapping_mul(17).wrapping_add(200)) % 1001);
                        let ch3 = 1000u32 + ((cycle.wrapping_mul(19).wrapping_add(300)) % 1001);
                        let dw0 = ((ch0 & 0xFFFF) << 16) | (ch1 & 0xFFFF);
                        let dw1 = ((ch2 & 0xFFFF) << 16) | (ch3 & 0xFFFF);
                        (dw0, dw1)
                    }
                    9 => {
                        let ch4 = 1000u32 + ((cycle.wrapping_mul(23).wrapping_add(400)) % 1001);
                        let ch5 = 1000u32 + ((cycle.wrapping_mul(29).wrapping_add(500)) % 1001);
                        let ch6 = 1000u32 + ((cycle.wrapping_mul(31).wrapping_add(600)) % 1001);
                        let ch7 = 1000u32 + ((cycle.wrapping_mul(37).wrapping_add(700)) % 1001);
                        let dw0 = ((ch4 & 0xFFFF) << 16) | (ch5 & 0xFFFF);
                        let dw1 = ((ch6 & 0xFFFF) << 16) | (ch7 & 0xFFFF);
                        (dw0, dw1)
                    }
                    10 => {
                        let arm_state = 1u32;
                        let mode_id = 3u32 + (cycle % 4);
                        let aux_flags = 0x0005u32;
                        let failsafe_state = 0u32;
                        let dw0 = ((arm_state & 0xFFFF) << 16) | (mode_id & 0xFFFF);
                        let dw1 = ((aux_flags & 0xFFFF) << 16) | (failsafe_state & 0xFFFF);
                        (dw0, dw1)
                    }
                    11 => {
                        let rssi = 85u32.saturating_sub(cycle % 20);
                        let link_quality = 95u32.saturating_sub(cycle % 10);
                        let snr_x10 = 280u32.saturating_sub(cycle % 30);
                        let packet_loss = total_error_count;
                        let dw0 = ((rssi & 0xFFFF) << 16) | (link_quality & 0xFFFF);
                        let dw1 = ((snr_x10 & 0xFFFF) << 16) | (packet_loss & 0xFFFF);
                        (dw0, dw1)
                    }
                    12 => {
                        let roll_cdeg = 18_000u32.wrapping_add((cycle.wrapping_mul(3)) % 3600);
                        let pitch_cdeg = 18_000u32.wrapping_add((cycle.wrapping_mul(5)) % 3600);
                        let dw0 = ((roll_cdeg & 0xFFFF) << 16) | (pitch_cdeg & 0xFFFF);
                        let dw1 = frame_seq;
                        (dw0, dw1)
                    }
                    13 => {
                        let yaw_rate_cdeg = 18_000u32.wrapping_add((cycle.wrapping_mul(7)) % 3600);
                        let throttle_x1000 = throttle_base;
                        let dw0 = ((yaw_rate_cdeg & 0xFFFF) << 16) | (throttle_x1000 & 0xFFFF);
                        let dw1 = frame_seq;
                        (dw0, dw1)
                    }
                    14 => {
                        let arm_state = 1u32;
                        let prearm_ok = 1u32;
                        let inhibit_flags = 0u32;
                        let dw0 = ((arm_state & 0xFFFF) << 16) | (prearm_ok & 0xFFFF);
                        let dw1 = ((inhibit_flags & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    15 => {
                        let failsafe_state = 0u32;
                        let failsafe_flags = 0u32;
                        let last_trigger = 0u32;
                        let dw0 = ((failsafe_state & 0xFFFF) << 16) | (failsafe_flags & 0xFFFF);
                        let dw1 = ((last_trigger & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }

                    16..=47 => {
                        let motor_slot = frame_index - 16;
                        let motor_number = (motor_slot / 8) + 1;
                        let motor_kind = motor_slot % 8;

                        let motor_throttle =
                            (throttle_base + motor_number.wrapping_mul(23)) % 1001;
                        let motor_target_rpm = 1200u32
                            .wrapping_add(motor_throttle.wrapping_mul(6))
                            .wrapping_add(motor_number.wrapping_mul(45));
                        let motor_measured_rpm =
                            motor_target_rpm.saturating_sub((cycle + motor_number) % 19);
                        let motor_voltage_mv =
                            voltage_mv.saturating_sub(motor_number.wrapping_mul(8));
                        let motor_current_ca =
                            current_ca.wrapping_add(motor_number.wrapping_mul(11));
                        let esc_temp_cdeg = 320u32
                            .wrapping_add((cycle + motor_number.wrapping_mul(5)) % 100);
                        let motor_temp_cdeg = 340u32
                            .wrapping_add((cycle + motor_number.wrapping_mul(7)) % 120);
                        let motor_angle_deg_x10 = cycle
                            .wrapping_mul(73)
                            .wrapping_add(motor_number.wrapping_mul(90))
                            % 3600;

                        match motor_kind {
                            0 => {
                                let flags = 1u32;
                                let dw0 = ((motor_throttle & 0xFFFF) << 16)
                                    | (motor_target_rpm & 0xFFFF);
                                let dw1 = ((flags & 0xFFFF) << 16) | frame_seq;
                                (dw0, dw1)
                            }
                            1 => {
                                let dw0 = ((motor_measured_rpm & 0xFFFF) << 16)
                                    | (motor_voltage_mv & 0xFFFF);
                                let dw1 = ((motor_current_ca & 0xFFFF) << 16)
                                    | (esc_temp_cdeg & 0xFFFF);
                                (dw0, dw1)
                            }
                            2 => {
                                let bus_power_w = motor_voltage_mv
                                    .wrapping_mul(motor_current_ca)
                                    / 10_000;
                                let phase_current_ca =
                                    motor_current_ca.wrapping_add(motor_number.wrapping_mul(5));
                                let dw0 = ((bus_power_w & 0xFFFF) << 16)
                                    | (phase_current_ca & 0xFFFF);
                                let dw1 = ((motor_voltage_mv & 0xFFFF) << 16)
                                    | (motor_current_ca & 0xFFFF);
                                (dw0, dw1)
                            }
                            3 => {
                                let mosfet_temp_cdeg = esc_temp_cdeg.wrapping_add(20);
                                let board_temp_cdeg = esc_temp_cdeg.wrapping_sub(5);
                                let dw0 = ((esc_temp_cdeg & 0xFFFF) << 16)
                                    | (motor_temp_cdeg & 0xFFFF);
                                let dw1 = ((mosfet_temp_cdeg & 0xFFFF) << 16)
                                    | (board_temp_cdeg & 0xFFFF);
                                (dw0, dw1)
                            }
                            4 => {
                                let commutation_us = 200u32
                                    .saturating_sub((motor_measured_rpm / 200) & 0xFFFF);
                                let pwm_hz = 20_000u32;
                                let control_loop_us = 250u32;
                                let dw0 = ((commutation_us & 0xFFFF) << 16)
                                    | (pwm_hz & 0xFFFF);
                                let dw1 = ((control_loop_us & 0xFFFF) << 16)
                                    | frame_seq;
                                (dw0, dw1)
                            }
                            5 => {
                                let current_limit_ca = 6000u32;
                                let rpm_limit = 9000u32;
                                let temp_limit_cdeg = 850u32;
                                let limit_flags = 0u32;
                                let dw0 = ((current_limit_ca & 0xFFFF) << 16)
                                    | (rpm_limit & 0xFFFF);
                                let dw1 = ((temp_limit_cdeg & 0xFFFF) << 16)
                                    | (limit_flags & 0xFFFF);
                                (dw0, dw1)
                            }
                            6 => {
                                let fault_flags = 0u32;
                                let last_fault_code = 0u32;
                                let dw0 = ((fault_flags & 0xFFFF) << 16)
                                    | (total_error_count & 0xFFFF);
                                let dw1 = ((last_fault_code & 0xFFFF) << 16)
                                    | frame_seq;
                                (dw0, dw1)
                            }
                            _ => {
                                let input_pct_x10 = motor_throttle;
                                let output_pct_x10 =
                                    motor_throttle.saturating_sub((cycle + motor_number) % 15);
                                let dw0 = ((input_pct_x10 & 0xFFFF) << 16)
                                    | (output_pct_x10 & 0xFFFF);
                                let dw1 = ((motor_angle_deg_x10 & 0xFFFF) << 16)
                                    | frame_seq;
                                (dw0, dw1)
                            }
                        }
                    }

                    48 => {
                        let used_mah = cycle.wrapping_mul(3) & 0xFFFF;
                        let dw0 = ((voltage_mv & 0xFFFF) << 16) | (current_ca & 0xFFFF);
                        let dw1 = ((used_mah & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    49 => {
                        let cell1_mv = 3700u32.wrapping_add(cycle % 40);
                        let cell2_mv = 3695u32.wrapping_add((cycle + 3) % 40);
                        let cell3_mv = 3702u32.wrapping_add((cycle + 5) % 40);
                        let cell4_mv = 0u32;
                        let dw0 = ((cell1_mv & 0xFFFF) << 16) | (cell2_mv & 0xFFFF);
                        let dw1 = ((cell3_mv & 0xFFFF) << 16) | (cell4_mv & 0xFFFF);
                        (dw0, dw1)
                    }
                    50 => {
                        let rail5v_mv = 5010u32.wrapping_add(cycle % 12);
                        let rail12v_mv = 12020u32.wrapping_add(cycle % 25);
                        let bec_temp_cdeg = 330u32.wrapping_add(cycle % 80);
                        let rail_flags = 1u32;
                        let dw0 = ((rail5v_mv & 0xFFFF) << 16) | (rail12v_mv & 0xFFFF);
                        let dw1 = ((bec_temp_cdeg & 0xFFFF) << 16) | (rail_flags & 0xFFFF);
                        (dw0, dw1)
                    }
                    51 => {
                        let current_limit_ca = 8000u32;
                        let voltage_limit_mv = 15000u32;
                        let brownout_count = 0u32;
                        let dw0 = ((current_limit_ca & 0xFFFF) << 16)
                            | (voltage_limit_mv & 0xFFFF);
                        let dw1 = ((brownout_count & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    52 => {
                        let motor_current_sum = current_ca.wrapping_mul(4);
                        let avionics_current_ca = 80u32;
                        let payload_current_ca = 40u32;
                        let dw0 = ((motor_current_sum & 0xFFFF) << 16)
                            | (avionics_current_ca & 0xFFFF);
                        let dw1 = ((payload_current_ca & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    53 => {
                        let main_battery_mv = 12_340u32.wrapping_add((cycle % 80).wrapping_mul(2));
                        let rail_5v_mv = 5_001u32.wrapping_add(cycle % 5);
                        let rail_3v3_mv = 3_300u32.wrapping_add(cycle % 3);
                        let current_ca_watch = 42u32.wrapping_add(cycle % 20);

                        let main_battery_raw = main_battery_mv & 0xFFFF;
                        let rail_5v_raw = rail_5v_mv & 0xFFFF;
                        let rail_3v3_raw = rail_3v3_mv & 0xFFFF;
                        let current_raw = current_ca_watch & 0xFFFF;

                        let dw0 = ((main_battery_raw & 0x00FF) << 24)
                            | ((main_battery_raw & 0xFF00) << 8)
                            | ((rail_5v_raw & 0x00FF) << 8)
                            | ((rail_5v_raw & 0xFF00) >> 8);
                        let dw1 = ((rail_3v3_raw & 0x00FF) << 24)
                            | ((rail_3v3_raw & 0xFF00) << 8)
                            | ((current_raw & 0x00FF) << 8)
                            | ((current_raw & 0xFF00) >> 8);
                        (dw0, dw1)
                    }
                    54 => {
                        let bec_temp_cdeg = 330u32.wrapping_add(cycle % 80);
                        let bec_current_ca = 120u32.wrapping_add(cycle % 30);
                        let bec_status = 1u32;
                        let dw0 = ((bec_temp_cdeg & 0xFFFF) << 16) | (bec_current_ca & 0xFFFF);
                        let dw1 = ((bec_status & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    55 => {
                        let power_faults = 0u32;
                        let brownout_count = 0u32;
                        let overcurrent_count = 0u32;
                        let dw0 = ((power_faults & 0xFFFF) << 16) | (brownout_count & 0xFFFF);
                        let dw1 = ((overcurrent_count & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }

                    56 => {
                        let gx_dps_x10 = 10_000u32.wrapping_add((cycle.wrapping_mul(7)) % 2000);
                        let gy_dps_x10 = 10_000u32.wrapping_add((cycle.wrapping_mul(11)) % 2000);
                        let gz_dps_x10 = 10_000u32.wrapping_add((cycle.wrapping_mul(13)) % 2000);
                        let imu_temp_cdeg = 310u32.wrapping_add(cycle % 40);
                        let dw0 = ((gx_dps_x10 & 0xFFFF) << 16) | (gy_dps_x10 & 0xFFFF);
                        let dw1 = ((gz_dps_x10 & 0xFFFF) << 16) | (imu_temp_cdeg & 0xFFFF);
                        (dw0, dw1)
                    }
                    57 => {
                        let ax_mg = 10_000u32.wrapping_add((cycle.wrapping_mul(5)) % 1000);
                        let ay_mg = 10_000u32.wrapping_add((cycle.wrapping_mul(7)) % 1000);
                        let az_mg = 11_000u32.wrapping_add((cycle.wrapping_mul(3)) % 1000);
                        let sample_count = frame_seq;
                        let dw0 = ((ax_mg & 0xFFFF) << 16) | (ay_mg & 0xFFFF);
                        let dw1 = ((az_mg & 0xFFFF) << 16) | (sample_count & 0xFFFF);
                        (dw0, dw1)
                    }
                    58 => {
                        let roll_cdeg = 18_000u32.wrapping_add((cycle.wrapping_mul(3)) % 3600);
                        let pitch_cdeg = 18_000u32.wrapping_add((cycle.wrapping_mul(5)) % 3600);
                        let yaw_cdeg = cycle.wrapping_mul(9) % 36000;
                        let dw0 = ((roll_cdeg & 0xFFFF) << 16) | (pitch_cdeg & 0xFFFF);
                        let dw1 = ((yaw_cdeg & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    59 => {
                        let baro_alt_cm = 10_000u32.wrapping_add(cycle.wrapping_mul(2) % 500);
                        let range_alt_cm = 250u32.wrapping_add(cycle % 100);
                        let climb_rate_cms = 10_000u32.wrapping_add((cycle.wrapping_mul(3)) % 400);
                        let dw0 = ((baro_alt_cm & 0xFFFF) << 16) | (range_alt_cm & 0xFFFF);
                        let dw1 = ((climb_rate_cms & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    60 => {
                        let lat_offset_cm = cycle.wrapping_mul(11) % 10_000;
                        let lon_offset_cm = cycle.wrapping_mul(13) % 10_000;
                        let gps_alt_cm = 10_000u32.wrapping_add(cycle % 1000);
                        let dw0 = ((lat_offset_cm & 0xFFFF) << 16) | (lon_offset_cm & 0xFFFF);
                        let dw1 = ((gps_alt_cm & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    61 => {
                        let vn_cms = 10_000u32.wrapping_add((cycle.wrapping_mul(5)) % 1000);
                        let ve_cms = 10_000u32.wrapping_add((cycle.wrapping_mul(7)) % 1000);
                        let vd_cms = 10_000u32.wrapping_add((cycle.wrapping_mul(3)) % 500);
                        let dw0 = ((vn_cms & 0xFFFF) << 16) | (ve_cms & 0xFFFF);
                        let dw1 = ((vd_cms & 0xFFFF) << 16) | frame_seq;
                        (dw0, dw1)
                    }
                    62 => {
                        let range_cm = 250u32.wrapping_add(cycle % 100);
                        let flow_x = 10_000u32.wrapping_add((cycle.wrapping_mul(2)) % 500);
                        let flow_y = 10_000u32.wrapping_add((cycle.wrapping_mul(3)) % 500);
                        let quality = 200u32.saturating_sub(cycle % 30);
                        let dw0 = ((range_cm & 0xFFFF) << 16) | (flow_x & 0xFFFF);
                        let dw1 = ((flow_y & 0xFFFF) << 16) | (quality & 0xFFFF);
                        (dw0, dw1)
                    }
                    _ => {
                        let mag_x = 10_000u32.wrapping_add((cycle.wrapping_mul(2)) % 800);
                        let mag_y = 10_000u32.wrapping_add((cycle.wrapping_mul(3)) % 800);
                        let mag_z = 10_000u32.wrapping_add((cycle.wrapping_mul(5)) % 800);
                        let baro_temp_cdeg = 300u32.wrapping_add(cycle % 40);
                        let dw0 = ((mag_x & 0xFFFF) << 16) | (mag_y & 0xFFFF);
                        let dw1 = ((mag_z & 0xFFFF) << 16) | (baro_temp_cdeg & 0xFFFF);
                        (dw0, dw1)
                    }
                };

                bsp::ral::write_reg!(can, can1, IFLAG1, 0x0000_0002);
                bsp::ral::write_reg!(can, can2, IFLAG1, CAN2_RX_MB2_IFLAG);

                unsafe {
                    write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 0, MB_CODE_RX_INACTIVE);
                    write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 1, tx_id << 18);
                    write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 2, 0x0000_0000);
                    write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 3, 0x0000_0000);
                    write_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 0, MB_CODE_RX_EMPTY);
                }

                let tx_cs = MB_CODE_TX_DATA | (DRONE_DLC << 16);

                unsafe {
                    write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 0, MB_CODE_TX_INACTIVE);
                    write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 1, tx_id << 18);
                    write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 2, tx_dw0);
                    write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 3, tx_dw1);
                    write_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 0, tx_cs);
                }

                let mut can1_tx_done = false;
                let mut can2_rx_pending = false;
                let mut can1_tx_attempt = 0xFFFF_FFFFu32;
                let mut can2_rx_attempt = 0xFFFF_FFFFu32;

                for attempt in 0..20_000u32 {
                    let can1_iflag1 = bsp::ral::read_reg!(can, can1, IFLAG1);
                    let can2_iflag1 = bsp::ral::read_reg!(can, can2, IFLAG1);
                    let can1_esr1_poll = bsp::ral::read_reg!(can, can1, ESR1);
                    let can2_esr1_poll = bsp::ral::read_reg!(can, can2, ESR1);
                    let can1_ecr_poll = bsp::ral::read_reg!(can, can1, ECR);
                    let can2_ecr_poll = bsp::ral::read_reg!(can, can2, ECR);

                    let can1_mb1_flag = (can1_iflag1 >> 1) & 1;
                    let can2_rx_flag = if (can2_iflag1 & CAN2_RX_MB2_IFLAG) != 0 { 1 } else { 0 };

                    if !can1_tx_done && can1_mb1_flag == 1 {
                        can1_tx_done = true;
                        can1_tx_attempt = attempt;
                    }

                    if !can2_rx_pending && can2_rx_flag == 1 {
                        can2_rx_pending = true;
                        can2_rx_attempt = attempt;
                    }

                    final_can1_iflag1 = can1_iflag1;
                    final_can2_iflag1 = can2_iflag1;
                    final_can1_esr1 = can1_esr1_poll;
                    final_can2_esr1 = can2_esr1_poll;
                    final_can1_ecr = can1_ecr_poll;
                    final_can2_ecr = can2_ecr_poll;

                    if can1_tx_done && can2_rx_pending {
                        break;
                    }
                }

                if !can1_tx_done {
                    let can1_mb1_cs_after_timeout =
                        unsafe { read_mb_word(FLEXCAN1_MB_RAM_BASE, CAN1_MB1_TX_OFFSET + 0) };

                    log::info!("CANFAULT cycle={} stage=tx_poll reason=timeout", cycle);
                    log::info!("CANTX id=0x{:03X} iflag1=0x{:08X}", tx_id, final_can1_iflag1);
                    log::info!("CANTX mb1_cs=0x{:08X}", can1_mb1_cs_after_timeout);
                    log::info!("CANTX esr1=0x{:08X} ecr=0x{:08X}", final_can1_esr1, final_can1_ecr);
                }

                if !can2_rx_pending {
                    log::info!("CANFAULT cycle={} stage=rx_poll reason=no_flag", cycle);
                    log::info!("CANRX id=0x{:03X} iflag1=0x{:08X}", tx_id, final_can2_iflag1);
                    log::info!("CANRX esr1=0x{:08X} ecr=0x{:08X}", final_can2_esr1, final_can2_ecr);
                }

                let rx_cs = unsafe { read_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 0) };
                let rx_id = unsafe { read_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 1) };
                let rx_dw0 = unsafe { read_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 2) };
                let rx_dw1 = unsafe { read_mb_word(FLEXCAN2_MB_RAM_BASE, CAN2_MB2_RX_OFFSET + 3) };

                let can2_timer_unlock_stage = bsp::ral::read_reg!(can, can2, TIMER);
                let can2_iflag_after_unlock = bsp::ral::read_reg!(can, can2, IFLAG1);

                let rx_code = (rx_cs >> 24) & 0xF;
                let rx_srr = (rx_cs >> 22) & 1;
                let rx_ide = (rx_cs >> 21) & 1;
                let rx_rtr = (rx_cs >> 20) & 1;
                let rx_dlc = (rx_cs >> 16) & 0xF;
                let rx_id_val = (rx_id >> 18) & 0x7FF;

                let late_movein_detected =
                    !can2_rx_pending && ((can2_iflag_after_unlock & CAN2_RX_MB2_IFLAG) != 0);

                let code_full = rx_code == RX_CODE_FULL;
                let id_match = rx_id_val == tx_id;
                let dlc_match = rx_dlc == DRONE_DLC;
                let dw0_match = rx_dw0 == tx_dw0;
                let dw1_match = rx_dw1 == tx_dw1;
                let frame_match = code_full && id_match && dlc_match && dw0_match && dw1_match;

                let frame_pass =
                    can1_tx_done && (can2_rx_pending || late_movein_detected) && frame_match;

                if frame_pass {
                    cycle_frame_pass = cycle_frame_pass.wrapping_add(1);
                    total_frame_pass = total_frame_pass.wrapping_add(1);
                } else {
                    cycle_frame_fail = cycle_frame_fail.wrapping_add(1);
                    total_frame_fail = total_frame_fail.wrapping_add(1);
                    cycle_match = false;
                }

                if !can1_tx_done {
                    cycle_tx_ok = false;
                }

                if !(can2_rx_pending || late_movein_detected) {
                    cycle_rx_ok = false;
                }

                if late_movein_detected {
                    any_late = true;
                }

                if can1_tx_attempt > last_can1_tx_attempt && can1_tx_attempt != 0xFFFF_FFFF {
                    last_can1_tx_attempt = can1_tx_attempt;
                }

                if can2_rx_attempt > last_can2_rx_attempt && can2_rx_attempt != 0xFFFF_FFFF {
                    last_can2_rx_attempt = can2_rx_attempt;
                }

                last_tx_id = tx_id;
                last_tx_dlc = DRONE_DLC;
                last_tx_cs = tx_cs;
                last_tx_dw0 = tx_dw0;
                last_tx_dw1 = tx_dw1;
                last_rx_id = rx_id_val;
                last_rx_dlc = rx_dlc;
                last_rx_code = rx_code;
                last_rx_dw0 = rx_dw0;
                last_rx_dw1 = rx_dw1;

                log::info!(
                    "CANFRAME bus=CAN1 dir=TX cycle={} elapsed_ms={} id=0x{:03X} dlc={} dw0=0x{:08X} dw1=0x{:08X} decode={}",
                    cycle,
                    elapsed_ms,
                    tx_id,
                    DRONE_DLC,
                    tx_dw0,
                    tx_dw1,
                    decode
                );
                log::info!(
                    "CANFRAME bus=CAN2 dir=RX cycle={} elapsed_ms={} id=0x{:03X} dlc={} dw0=0x{:08X} dw1=0x{:08X} decode={}",
                    cycle,
                    elapsed_ms,
                    rx_id_val,
                    rx_dlc,
                    rx_dw0,
                    rx_dw1,
                    decode
                );

                if !frame_pass {
                    log::info!("CANFAULT cycle={} stage=frame_compare reason=mismatch", cycle);
                    log::info!(
                        "CANFAULT id=0x{:03X} tx={} rx={} late={}",
                        tx_id,
                        if can1_tx_done { 1 } else { 0 },
                        if can2_rx_pending { 1 } else { 0 },
                        if late_movein_detected { 1 } else { 0 }
                    );
                    log::info!("CANEXP code=0x{:X} id=0x{:03X} dlc={}", RX_CODE_FULL, tx_id, DRONE_DLC);
                    log::info!("CANEXP dw0=0x{:08X} dw1=0x{:08X}", tx_dw0, tx_dw1);
                    log::info!("CANRXSEL code=0x{:X} id=0x{:03X} dlc={}", rx_code, rx_id_val, rx_dlc);
                    log::info!("CANRXSEL dw0=0x{:08X} dw1=0x{:08X}", rx_dw0, rx_dw1);
                    log::info!(
                        "CANMATCH code={} id={} dlc={}",
                        if code_full { 1 } else { 0 },
                        if id_match { 1 } else { 0 },
                        if dlc_match { 1 } else { 0 }
                    );
                    log::info!(
                        "CANMATCH dw0={} dw1={}",
                        if dw0_match { 1 } else { 0 },
                        if dw1_match { 1 } else { 0 }
                    );
                    log::info!("CANRXCS srr={} ide={} rtr={}", rx_srr, rx_ide, rx_rtr);
                    log::info!("CANRXCS timer=0x{:08X}", can2_timer_unlock_stage);
                }

                bsp::ral::write_reg!(can, can1, IFLAG1, 0x0000_0002);
                bsp::ral::write_reg!(can, can2, IFLAG1, CAN2_RX_MB2_IFLAG);
            }

            let can1_esr1 = bsp::ral::read_reg!(can, can1, ESR1);
            let can2_esr1 = bsp::ral::read_reg!(can, can2, ESR1);
            let can1_ecr = bsp::ral::read_reg!(can, can1, ECR);
            let can2_ecr = bsp::ral::read_reg!(can, can2, ECR);
            let can1_tx_err = can1_ecr & 0xFF;
            let can1_rx_err = (can1_ecr >> 8) & 0xFF;
            let can2_tx_err = can2_ecr & 0xFF;
            let can2_rx_err = (can2_ecr >> 8) & 0xFF;

            let cycle_pass = cycle_frame_pass == DRONE_MESSAGE_COUNT
                && cycle_frame_fail == 0
                && cycle_tx_ok
                && cycle_rx_ok
                && cycle_match;

            if cycle_pass {
                total_pass = total_pass.wrapping_add(1);
            } else {
                total_fail = total_fail.wrapping_add(1);
            }

            let total_runs = total_pass.wrapping_add(total_fail);
            let pass_rate_percent = if total_runs == 0 {
                0
            } else {
                total_pass.saturating_mul(100) / total_runs
            };

            let elapsed_ms_u64 = elapsed_ms as u64;

            let test_frame_rate_x100_u64 = if elapsed_ms_u64 == 0 {
                0
            } else {
                (total_frame_pass as u64).saturating_mul(100_000) / elapsed_ms_u64
            };

            let util_x100_u64 = test_frame_rate_x100_u64
                .saturating_mul(BITS_PER_STANDARD_DLC8_FRAME as u64)
                .saturating_mul(100)
                / (CAN_BITRATE_BPS_DERIVED as u64);

            let test_frame_rate_x100 = if test_frame_rate_x100_u64 > (u32::MAX as u64) {
                u32::MAX
            } else {
                test_frame_rate_x100_u64 as u32
            };

            let util_x100 = if util_x100_u64 > (u32::MAX as u64) {
                u32::MAX
            } else {
                util_x100_u64 as u32
            };

            log::info!(
                "CANSTAT cycle={} pass={} fail={} last={} match={} frames={} frame_fail={}",
                cycle,
                total_pass,
                total_fail,
                if cycle_pass { 1 } else { 0 },
                if cycle_match { 1 } else { 0 },
                cycle_frame_pass,
                cycle_frame_fail
            );
            log::info!(
                "CANIO cycle={} tx={} rx={} late={} txa={} rxa={} clr1=0x{:08X} clr2=0x{:08X}",
                cycle,
                if cycle_tx_ok { 1 } else { 0 },
                if cycle_rx_ok { 1 } else { 0 },
                if any_late { 1 } else { 0 },
                last_can1_tx_attempt,
                last_can2_rx_attempt,
                can1_iflag_cleared,
                can2_iflag_cleared
            );
            log::info!(
                "CANTX cycle={} id=0x{:03X} dlc={} cs=0x{:08X}",
                cycle,
                last_tx_id,
                last_tx_dlc,
                last_tx_cs
            );
            log::info!(
                "CANTXD cycle={} dw0=0x{:08X} dw1=0x{:08X}",
                cycle,
                last_tx_dw0,
                last_tx_dw1
            );
            log::info!(
                "CANRX cycle={} id=0x{:03X} dlc={} code=0x{:X}",
                cycle,
                last_rx_id,
                last_rx_dlc,
                last_rx_code
            );
            log::info!(
                "CANRXD cycle={} dw0=0x{:08X} dw1=0x{:08X}",
                cycle,
                last_rx_dw0,
                last_rx_dw1
            );
            log::info!(
                "CANERR cycle={} c1tx={} c1rx={} c2tx={} c2rx={} e1=0x{:08X} e2=0x{:08X}",
                cycle,
                can1_tx_err,
                can1_rx_err,
                can2_tx_err,
                can2_rx_err,
                can1_esr1,
                can2_esr1
            );
            log::info!(
                "CANRATE cycle={} pct={} rate_x100={} elapsed_ms={} util_x100={}",
                cycle,
                pass_rate_percent,
                test_frame_rate_x100,
                elapsed_ms,
                util_x100
            );

            if !cycle_pass {
                log::info!("CANFAULT cycle={} stage=cycle_summary reason=frame_set", cycle);
                log::info!(
                    "CANFAULT frames_pass={} frames_fail={}",
                    cycle_frame_pass,
                    cycle_frame_fail
                );
                log::info!("CANPRE1 mcr=0x{:08X} ctrl1=0x{:08X}", can1_pre_mcr, can1_pre_ctrl1);
                log::info!("CANPRE1 iflag=0x{:08X} timer=0x{:08X}", can1_pre_iflag1, can1_pre_timer);
                log::info!("CANPRE1 esr1=0x{:08X} ecr=0x{:08X}", can1_pre_esr1, can1_pre_ecr);
                log::info!("CANPRE2 mcr=0x{:08X} ctrl1=0x{:08X}", can2_pre_mcr, can2_pre_ctrl1);
                log::info!("CANPRE2 iflag=0x{:08X} timer=0x{:08X}", can2_pre_iflag1, can2_pre_timer);
                log::info!("CANPRE2 esr1=0x{:08X} ecr=0x{:08X}", can2_pre_esr1, can2_pre_ecr);
                log::info!("CANFINAL1 iflag=0x{:08X}", final_can1_iflag1);
                log::info!("CANFINAL1 esr1=0x{:08X} ecr=0x{:08X}", final_can1_esr1, final_can1_ecr);
                log::info!("CANFINAL2 iflag=0x{:08X}", final_can2_iflag1);
                log::info!("CANFINAL2 esr1=0x{:08X} ecr=0x{:08X}", final_can2_esr1, final_can2_ecr);
                log::info!("CANMB0ERR c1=0x{:08X} c2=0x{:08X}", can1_mb0_errata_rb, can2_mb0_errata_rb);
            }

            if (cycle % DASHBOARD_REFRESH_CYCLES) == 0 {
                log::info!("CANBOOT version=v0.11.3-voltage-distribution-decode file=src/main.rs");
                log::info!("CANBOOT board=teensy40 mcu=imxrt1062 mode=drone_catalog_64_one_rx_mb");
                log::info!("CANCFG clk_hz=24000000 ctrl1=0x{:08X}", CTRL1_TIMING);
                log::info!("CANCFG bitrate={} loop_ms={}", CAN_BITRATE_BPS_DERIVED, LOOP_DELAY_MS);
                log::info!(
                    "CANCFG presdiv={} pseg1={} pseg2={} rjw={} smp={}",
                    CTRL1_PRESDIV,
                    CTRL1_PSEG1,
                    CTRL1_PSEG2,
                    CTRL1_RJW,
                    CTRL1_SMP
                );
                log::info!("CANROUTE c1tx=P22 c1rx=P23 c2tx=P1 c2rx=P0 ack=CAN2");
                log::info!("CANPATH CAN1-XCVRA-CANH_CANL-XCVRB-CAN2");
                log::info!("CANLAYOUT c1_mb=0r_1tx_2off c2_mb=0r_1off_2rx_reuse maxmb=2");
                log::info!("CANTEST id=0x{:03X} dlc={} dw0=0x00000000", DRONE_MESSAGES[0].id, DRONE_DLC);
                log::info!("CANTEST dw1=0x00000000 mask=0x{:08X}", CAN2_RXMGMASK_EXACT);
                log::info!(
                    "CANMSG catalog=drone64 frames_per_cycle={} rx_mode=one_reused_mailbox",
                    DRONE_MESSAGE_COUNT
                );
            }

            if cycle_pass
                && can1_tx_err == 0
                && can1_rx_err == 0
                && can2_tx_err == 0
                && can2_rx_err == 0
                && (cycle % DASHBOARD_REFRESH_CYCLES) == 0
            {
                log::info!("CANPROVEN cycle={} path=CAN1-XCVRA-CANH_CANL-XCVRB-CAN2", cycle);
                log::info!(
                    "CANPROVEN cycle={} catalog=drone64 frames_per_cycle={} rx_mode=one_reused_mailbox",
                    cycle,
                    DRONE_MESSAGE_COUNT
                );
            }

            cycle = cycle.wrapping_add(1);
            Systick::delay(LOOP_DELAY_MS.millis()).await;
        }
    }

    #[task(binds = USB_OTG1, local = [poller])]
    fn usb_interrupt(cx: usb_interrupt::Context) {
        cx.local.poller.poll();
    }
}

// ============================================================================
// Footer
// File: main.rs
// Path: ~/teensy-rust-test/teensy-can-bringup/src/main.rs
// Version: v0.11.3-voltage-distribution-decode
// Created: 2026-06-10
// Timestamp: 2026-06-12
// End of file
// ============================================================================