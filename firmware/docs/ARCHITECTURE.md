# Firmware architecture

How the normal firmware (`stm32c071-auto-hybrid`) is organized: the signal
path, the main loop, who owns each resource, and the safety mechanisms. The
control laws are in [CLOCK_DISCIPLINE](CLOCK_DISCIPLINE.md) and
[THERMAL_CONTROL](THERMAL_CONTROL.md); the USB interface is in
[COMMANDS](COMMANDS.md).

## Signal path

```text
SiT5801 OCXO --> U5 10 MHz buffer --> 10 MHz outputs
     ^                       \--> divider --> mosaic-G5 T REF_I (receiver runs on the OCXO)
     | I2C2 (frequency word)                       |
     |                                             +--> SBF data, USART2 --> MCU
STM32C071 MCU <------------------------------------+
     |                                             +--> PPS --> U6 buffer --> PPS outputs
     | PB3 --> Q1 --> heater (J18)                                    \--> MCU TIM2 capture
     | I2C2 --> TMP117 temperature sensor (beside the OCXO)
     | USB --> host (telemetry, commands)
```

Because the receiver is clocked by the OCXO, its reported clock bias and drift
measure the OCXO directly against GNSS time. The firmware closes the loop by
writing the OCXO's frequency control word.

## Pins

| Function | Connection |
| --- | --- |
| 10 MHz buffer enable | PA1 → U5 (driven high at boot, kept high) |
| PPS buffer enable | PA0 → U6 (driven high at boot, kept high) |
| Receiver UART | USART2: PA2 TX → receiver RXD1, PA3 RX ← TXD1; 115200 8N1 |
| Receiver reset | PA4 → nRST_IN, left high-impedance |
| PPS capture | U6 output → PA5, TIM2_CH1 |
| I2C2 | PA6 SDA, PA7 SCL, 5.1 kΩ pull-ups; TMP117 at 0x48, SiT5801 at 0x50 |
| Heater | PB3 → R13 → Q1 (AO3400A), R14 pull-down; heater on J18 (5 V, about 1 W) |
| LEDs | LED1 PA8, LED2 PC14, LED3 PC15 (active high) |
| USB | PA11 D−, PA12 D+ (CDC, HSI48 with USB clock recovery) |
| Boot | J17 pulls PA14/BOOT0 high (R9 pull-down); see [first flashing](../../docs/FIRST_FLASHING.md) |

Package pin 20 is shared by PB3–PB6; the firmware selects PB3 explicitly and
sets the other three to analog. The system clock is the internal 48 MHz HSI.

## Main loop and interrupts

A bare-metal cooperative loop runs on the 1 ms SysTick, sleeping between ticks
(no RTOS, no dynamic allocation). Each pass:

1. `thermal_probe_service`: one TMP117 I2C transaction, PID update, thermal
   commands and records.
2. `clock_probe_service`: checks the loop gap and stack guard, then
   - `gnss_probe_service`: drains the receiver UART ring, parses SBF frames and
     ASCII replies, runs receiver command sequences;
   - `timing_probe_service`: assembles receiver epochs and PPS captures;
   - `hybrid_probe_service`: USB commands, controller timeouts, consumes the
     latest epoch, outer thermal loop, telemetry;
   - one paced SiT5801 register read; when a full 8-register read-back
     completes, `hybrid_probe_cycle` updates the controller with the verified
     frequency word and writes the next one;
   - LEDs, `resources` and `gnss_probe` records; feeds the watchdog.

Interrupts do bounded work only:

| Interrupt | Work |
| --- | --- |
| SysTick (1 ms) | Heater PWM frame and safety deadlines |
| DMA1 channel 1 | Counts wraps of the circular USART2 receive ring (DMA keeps receiving while flash writes stall the CPU) |
| USART2 | Error flags |
| TIM2 capture | PPS edge snapshot into a ring |
| USB | CDC transfer completion; each complete command line goes into its subsystem's one-slot mailbox |

Parsing, receiver sequencing, control and formatting all run in the foreground.
Shared state is exchanged through single-producer rings and short
interrupt-masked snapshots.

## Data flow for one second

1. The receiver sends its SBF blocks (PVT, receiver time and status, measurement
   epoch, RF status, DOP) over USART2.
2. `sbf` checks framing and CRC; `sbf_fields` decodes the fields; `timing`
   assembles one epoch from blocks with the same time tag and marks gaps,
   duplicates, resets and receiver-configuration changes.
3. `hybrid_input` decides whether the epoch is usable and which solution type
   it is (PPP/HAS, SBAS, standalone, static), requiring fresh data, RAIM
   success, a clear RF and error status, and the external reference present.
4. `hybrid` updates the controller and, when allowed, proposes a new trim.
5. `hybrid_probe` writes the three frequency-word registers, then waits for the
   paced read-back before the controller accepts the trim as applied. A write
   that is not confirmed by read-back puts the controller into ACTUATOR_UNKNOWN.

## Ownership

| Resource | Owner |
| --- | --- |
| OCXO frequency word | `hybrid_probe`: one arbiter for the controller and the `clock trial` test; at most one write in flight |
| Heater pin | `thermal_probe` and the SysTick heater logic |
| Receiver UART and configuration | `gnss_probe` command sequencer; one transaction at a time |
| I2C2 bus | Shared by the TMP117 and SiT5801 drivers; one bounded transaction per service call each |
| USB transmit | One buffer; records retry on BUSY and are counted as dropped if they cannot be sent |
| Settings flash | `settings_flash`: two 2 KiB pages at `0x0801F000` |

## Safety

- **Heater:** the foreground must renew a 250 ms lease; a missed lease, a stale
  sensor (2.5 s), a raw reading ≥ 70 °C or a control fault switches the heater
  off and latches a fault. Flash writes switch the heater off for their
  duration. HardFault and NMI handlers drive PB3 low directly, without using
  the stack.
- **Watchdog:** a 2 s independent watchdog is fed only by the main loop, and
  only while the loop gap stays ≤ 500 ms and the stack guard is intact. A
  reset leaves the OCXO at its last frequency word; the clock then requalifies.
- **Oscillator:** trim stays within ±1000 ppb of the value read at boot and within ±1 ppb
  (FLL) or ±10 ppb (phase-only) of the last acquired trim, moves at
  most 0.003 ppb/s while tracking, and is only accepted after read-back.
- **Settings:** saved as a CRC-protected record in alternating flash pages,
  committed by a final write, so a power loss during a save keeps the previous
  record.
- **Receiver:** only whitelisted commands are sent; settings changes are read
  back and saved, and the clock is held while the receiver is reconfigured.
