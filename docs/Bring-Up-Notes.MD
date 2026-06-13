# Teensy 4.0 Rust FlexCAN Bring-Up Notes

## External CAN1 → CAN2 Test Using Two SN65HVD230-Style Transceiver Modules

## Standalone Project Update

This bring-up has now been moved out of the original `teensy4-rs/examples` workflow into a standalone project folder:

```text
~/teensy-can-bringup
```

Current standalone build and flash flow:

```bash
cargo build --release --target thumbv7em-none-eabihf
cargo run --release --target thumbv7em-none-eabihf
```

The project now uses the published `teensy4-bsp` crate instead of a sibling `../teensy4-rs` checkout.

The flashing runner is local to this repo:

```text
tools/runner.rs
tools/Cargo.toml
```

The host dashboard is also local to this repo:

```text
tools/can-dashboard
```

The current firmware stream is line-oriented and compatible with ComChan-style serial monitoring. The dashboard consumes the same stream and displays live CAN status, frame grid data, decoded message watch entries, error counters, timing, rate, and utilization.

The historical notes below remain important because they document the original hardware proof, pin routing, mailbox layout, timing, ACK behavior, and the receive-mailbox lock rule that must not be broken during future cleanup.


### Status

This document records the successful bring-up of CAN communication on a Teensy 4.0 using Rust, direct register access through the `teensy4-rs` environment, and two external CAN transceiver modules.

The final checkpoint for this phase is:

`can_bringup.rs v0.6.6-flexcan1-to-flexcan2-external-pass-summary`

The proven physical path is:

`Teensy CAN1 -> transceiver A -> CANH/CANL -> transceiver B -> Teensy CAN2`

The final successful test repeatedly transmitted a standard CAN frame from CAN1 and received it on CAN2:

```text
CAN ID: 0x123
DLC:    8
DATA:   DE AD BE EF CA FE F0 0D
```

The final logs showed:

```text
[CANBUS] tx CAN1 mb=1 id=0x123 dlc=8 data=DE AD BE EF CA FE F0 0D
[CANBUS] rx CAN2 mb=2 id=0x123 dlc=8 data=DE AD BE EF CA FE F0 0D
[CANBUS] result frame_match=true tx_done=true rx_done=true code_full=true id_match=true dlc_match=true data_match=true
[CANBUS] health CAN1_TX_ERR=0 CAN1_RX_ERR=0 CAN2_TX_ERR=0 CAN2_RX_ERR=0
[PROVEN] Teensy CAN1 -> transceiver A -> CANH/CANL -> transceiver B -> Teensy CAN2
```

By the time the test was stopped, the repeated cycles were stable, with total pass counts increasing and total fail remaining zero.

---

## Hardware Setup

### Board

Target board:

```text
Teensy 4.0
MCU: NXP i.MX RT1062
CAN engines used: FlexCAN1 and FlexCAN2
```

### Transceiver modules

Two small blue SN65HVD230-style 3.3 V CAN transceiver modules were used.

Each module had:

```text
VCC
GND
TXD / CTX
RXD / CRX
CANH
CANL
```

The modules translate between the Teensy logic-side CAN pins and the differential CAN bus pair.

### CAN wiring

CAN1 side:

```text
Teensy pin 22 / CAN1 TX -> transceiver A TXD / CTX
Teensy pin 23 / CAN1 RX <- transceiver A RXD / CRX
```

CAN2 side:

```text
Teensy pin 1 / CAN2 TX -> transceiver B TXD / CTX
Teensy pin 0 / CAN2 RX <- transceiver B RXD / CRX
```

Bus side:

```text
transceiver A CANH -> transceiver B CANH
transceiver A CANL -> transceiver B CANL
common GND between Teensy and both transceivers
```

Both transceivers were powered from the Teensy 3.3 V rail for this bench test.

CAN2 TX was intentionally connected and routed even though CAN2 was only the receiver in the test. CAN2 still needs to drive the ACK bit during a valid received CAN frame.

---

## Software Environment

Project location during bring-up:

```text
~/teensy-rust-test/teensy-can-bringup/src/main.rs
```

The test was run as a Teensy example using:

```text
cargo run --release --all-features --example can_bringup --target thumbv7em-none-eabihf
```

The file was built as a no-std RTIC application using:

```text
teensy4_bsp
imxrt_log
rtic_monotonics::systick
direct register access through bsp::ral
```

The test kept the pin 13 LED heartbeat and USB logging active throughout the bring-up.

---

## Confirmed Pin Routing

The final working pin routing was:

```text
CAN1_TX = Teensy pin 22 = GPIO_AD_B1_08 = ALT2
CAN1_RX = Teensy pin 23 = GPIO_AD_B1_09 = ALT2
CAN2_TX = Teensy pin 1  = GPIO_AD_B0_02 = ALT0
CAN2_RX = Teensy pin 0  = GPIO_AD_B0_03 = ALT0
```

The receive daisy-chain selections were:

```text
CAN1_RX_DAISY = 0x00000002
CAN2_RX_DAISY = 0x00000001
```

The final test logged and confirmed these mux settings on every cycle.

---

## Clock Setup

The CAN clock root was configured to 24 MHz.

The observed confirmed register state was:

```text
CSCMR2 = 0x13192D02
CAN_CLK_SEL = 1
CAN_CLK_PODF = 0
```

Both CAN1 and CAN2 clock gates were enabled through `CCGR0`.

The final logs repeatedly showed:

```text
CAN1_bits17_14 = 0b1111
CAN2_bits21_18 = 0b1111
```

This confirmed that both internal CAN message engines were clocked and active.

---

## CAN Timing

The working timing value for both CAN1 and CAN2 was:

```text
CTRL1 = 0x075B0004
```

For the internal loopback tests, the loopback bit was set at bit 12:

```text
CTRL1 loopback = 0x075B1004
LPB12 = 1
```

The external test used loopback off:

```text
CTRL1 = 0x075B0004
LPB12 = 0
```

This distinction was important. Internal loopback was used only to prove CAN2’s own TX/RX mailbox behavior. External CAN1→CAN2 testing used real transceivers and real CANH/CANL bus signaling.

---

## Mailbox Layout

The final successful external test used this mailbox layout:

```text
CAN1:
  MB0 = reserved inactive
  MB1 = transmit mailbox
  MB2 = inactive
  MAXMB = 2

CAN2:
  MB0 = reserved inactive
  MB1 = inactive
  MB2 = receive mailbox
  MAXMB = 2
```

RX FIFO was disabled:

```text
RFEN = 0
```

This kept the test in normal mailbox mode rather than FIFO mode.

The receive mask used for CAN2 was:

```text
RXMGMASK = 0x1FFFFFFF
IRMQ = 0
```

The receive mailbox ID was loaded as:

```text
TEST_ID << 18
```

For the test ID:

```text
TEST_ID = 0x123
encoded ID = 0x048C0000
```

---

## Bring-Up Path

### 1. Basic code execution and logging

The first step was confirming that a simple Teensy example could build, flash, blink the LED, and print USB logs.

This proved:

```text
toolchain works
HalfKay flashing works
RTIC startup works
USB logging works
pin 13 LED heartbeat works
```

### 2. FlexCAN1 internal loopback

The first real CAN proof was internal loopback on CAN1.

A known-good internal loopback version used:

```text
CAN1 MB0 reserved inactive
CAN1 MB1 TX
CAN1 MB2 RX
CTRL1.LPB bit 12 set
RFEN = 0
MAXMB = 2
```

That test proved the mailbox layout, timing, and internal FlexCAN1 operation.

### 3. External CAN1 transmit attempts

The next stage moved out of loopback and tried to transmit through an external transceiver.

CAN1 began completing TX successfully:

```text
CAN1 IFLAG1 bit1 set
CAN1 MB1 CODE returned to 0x8
CAN1 ECR remained zero
```

This mattered because a CAN transmitter does not complete a normal frame successfully unless it sees a valid ACK. CAN1 cannot ACK itself in external mode. When CAN1 completed with no ACK errors, that strongly indicated that another active CAN node was present and acknowledging the frame.

### 4. Initial CAN2 RX failures

The first CAN1→CAN2 external tests failed at the CAN2 receive mailbox.

Typical failure state:

```text
CAN1 TX complete = true
CAN2 IFLAG1 bit2 = 0
CAN2 MB2 CODE = 0x4
CAN2 DLC = 0
CAN2 data = 0
CAN1 ECR = 0
CAN2 ECR = 0
```

This looked at first like a filter, mask, mux, or physical RX problem.

### 5. CAN2 internal loopback

CAN2 was then tested by itself in internal loopback mode.

That test passed cleanly:

```text
CAN2 MB1 TX complete
CAN2 MB2 RX complete
ID matched
DLC matched
data matched
error counters stayed zero
```

This ruled out the main CAN2 mailbox setup as the cause. CAN2 could transmit, receive, match, and store frames internally using the same MB1/MB2 layout.

### 6. Mailbox lock discovery

The important failure mechanism was the FlexCAN receive mailbox lock behavior.

The earlier external polling loop repeatedly read the CAN2 RX mailbox C/S word while waiting for the RX flag.

That was wrong.

The important rule from this bring-up is:

```text
Do not poll an RX mailbox by repeatedly reading its C/S word.
Poll IFLAG only.
Only read the RX mailbox after its IFLAG bit is set, or after timeout for diagnostics.
```

Reading an EMPTY RX mailbox C/S word can lock that mailbox. If the mailbox is locked while a frame is being received, the frame can be held internally and not committed to the visible mailbox until the lock is released.

The corrected receive sequence was:

```text
1. Poll CAN2 IFLAG1 bit2 only.
2. Do not read CAN2 MB2 C/S during the poll loop.
3. After CAN2 IFLAG1 bit2 is set:
   read MB2 CS
   read MB2 ID
   read MB2 DW0
   read MB2 DW1
   read CAN2 TIMER to unlock
4. Only then clear CAN2 IFLAG1 bit2.
```

Once this was done, external CAN1→CAN2 immediately passed.

This was the key lesson of the bring-up.

---

## Correct RX Read Sequence

The successful sequence for a received mailbox was:

```text
rx_cs  = read MB2 CS
rx_id  = read MB2 ID
rx_dw0 = read MB2 WORD0
rx_dw1 = read MB2 WORD1
timer  = read CAN2 TIMER
```

The timer read releases the receive mailbox lock.

The IFLAG bit should not be cleared until after the received frame has been read and the mailbox has been unlocked.

---

## Final Confirmed Result

The final file added CAN-style summary logs.

Representative passing output:

```text
[CANBUS] tx CAN1 mb=1 id=0x123 dlc=8 data=DE AD BE EF CA FE F0 0D

[CANBUS] rx CAN2 mb=2 id=0x123 dlc=8 data=DE AD BE EF CA FE F0 0D

[CANBUS] result frame_match=true tx_done=true rx_done=true code_full=true id_match=true dlc_match=true data_match=true late_movein_detected=false

[CANBUS] health CAN1_TX_ERR=0 CAN1_RX_ERR=0 CAN2_TX_ERR=0 CAN2_RX_ERR=0

[PROVEN] Teensy CAN1 -> transceiver A -> CANH/CANL -> transceiver B -> Teensy CAN2
```

The repeated-pass summary reached:

```text
total_pass = 100
total_fail = 0
external_can1_to_can2_last_pass = true
```

This confirmed stable communication across the full external path.

---

## What Was Proven

The final checkpoint proves:

```text
Teensy 4.0 can configure FlexCAN1 and FlexCAN2 from Rust.
CAN clock root setup works.
CAN1 and CAN2 clock gates work.
Pin mux and RX daisy routing work.
CAN1 can transmit a standard CAN frame externally.
CAN2 can acknowledge the external frame.
CAN2 can store the received frame in an RX mailbox.
Two 3.3 V transceiver modules can communicate over CANH/CANL.
The Rust code can perform a complete external CAN1→CAN2 frame test.
The mailbox-lock-safe receive flow works.
```

The full proven signal path is:

```text
FlexCAN1 message engine
-> Teensy pin 22
-> transceiver A TXD
-> transceiver A CANH/CANL driver
-> CANH/CANL wiring
-> transceiver B CANH/CANL receiver
-> transceiver B RXD
-> Teensy pin 0
-> CAN2 RX input mux/daisy path
-> FlexCAN2 protocol engine
-> CAN2 MB2 receive mailbox
```

The ACK path is also proven:

```text
FlexCAN2 protocol engine
-> Teensy pin 1
-> transceiver B TXD
-> transceiver B CANH/CANL driver during ACK bit
-> CANH/CANL bus
-> transceiver A receiver
-> Teensy pin 23
-> FlexCAN1 sees ACK and completes TX
```

---

## Important Notes for Future Work

### Do not read RX mailbox C/S while polling

This was the major trap.

Bad pattern:

```text
loop:
    read IFLAG
    read RX mailbox C/S
    check if full
```

Good pattern:

```text
loop:
    read IFLAG only

after flag:
    read RX mailbox C/S
    read ID/data
    read TIMER to unlock
```

### CAN TX complete means an ACK was seen

In external mode, CAN1 TX complete with no ACK error is strong evidence that another node acknowledged the frame.

In this setup, that node was CAN2 through transceiver B.

### Zero error counters matter

The final test repeatedly showed:

```text
CAN1_TX_ERR = 0
CAN1_RX_ERR = 0
CAN2_TX_ERR = 0
CAN2_RX_ERR = 0
```

That means the bus was not merely producing some noisy activity. It was completing valid CAN frames.

### RX mailbox CODE value

For a successful receive, CAN2 MB2 C/S showed:

```text
CODE = 0x2
DLC = 8
```

Earlier failures showed:

```text
CODE = 0x4
DLC = 0
```

`CODE=0x4` meant the RX mailbox was still empty.

### Use MB0 reserved inactive

The stable layout kept MB0 reserved/inactive and used MB1/MB2 for actual work. That matched the earlier proven internal loopback pattern.

### Keep logs structured

The successful `v0.6.6` style made the output easier to read:

```text
[CANBUS] tx ...
[CANBUS] rx ...
[CANBUS] result ...
[CANBUS] health ...
[PROVEN] ...
[SUMMARY] ...
```

Future standalone project versions should keep that structure, but move most low-level register diagnostics behind a debug level or compile-time flag.

---

## Suggested Next Standalone Project Shape

When this moves out of the examples folder, the code can be split into layers:

```text
clock setup
pin mux setup
CAN engine setup
mailbox setup
frame TX
frame RX
logging/summary
```

The first standalone version should preserve the known-good behavior before cleanup.

Recommended checkpoint before refactoring:

```text
v0.6.6-flexcan1-to-flexcan2-external-pass-summary
```

After that, cleanup can happen in small steps:

```text
1. Keep full diagnostics, but group them.
2. Add clear debug/info logging levels.
3. Separate register setup from frame send/receive.
4. Keep the lock-safe RX read rule unchanged.
5. Add a small CAN frame struct only after the raw path remains verified.
6. Add reusable send/receive functions only after the standalone copy passes unchanged.
```

The rule for refactoring is:

```text
Do not change the mailbox access order while cleaning up.
```

The exact ordering of RX reads is part of the working behavior, not cosmetic code.

---

## Current Checkpoint

The bring-up is no longer a theory test. It is a working external CAN communication checkpoint.

Final status:

```text
Board: Teensy 4.0
Language: Rust
CAN engines: FlexCAN1 and FlexCAN2
Transceivers: two 3.3 V SN65HVD230-style modules
Mode: external CAN bus, no loopback
Frame: standard ID 0x123, DLC 8
Payload: DE AD BE EF CA FE F0 0D
Result: repeated pass
Errors: zero counters on both CAN engines
```

This should be treated as the base known-good version before moving into a standalone Teensy CAN project.
