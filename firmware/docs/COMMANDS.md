# USB commands and telemetry

Reference for the normal SimpleClock firmware's USB serial interface. For
everyday use see the [user manual](../../docs/USER_MANUAL.md).

## Connection

USB CDC serial port, 115200 baud, 8N1, no flow control, DTR/RTS asserted.
Commands are ASCII lines ending in CR LF. The board sends newline-terminated
ASCII telemetry continuously; command replies arrive interleaved with it.
`tools/usb-capture.py` records everything and sends commands (`--command`,
repeatable). Only one program can own the port.

## Command replies

A host write is not an acknowledgement. Each command produces a reply line:

| Commands | Reply |
| --- | --- |
| `hybrid …`, `pps …`, `outer …`, `clock trial`, `clock abort`, `system …` | `type=hybrid_command v=1 uptime_ms=… accepted=0/1 command_hex=<command in ASCII hex>` |
| `thermal …`, `fault clear` | `type=thermal_command v=1 … result=<name> command_hex=…` |
| `gnss read/write/exec`, `gnss defaults` | `type=gnss_control` stage events (see [receiver settings](#receiver-settings)) |
| Other `gnss …` profiles | `gnss_probe` fields `command_profile`, `command_state` (0 idle, 1 waiting, 2 OK, 3 rejected, 4 timeout), `readback_ok` |

The firmware holds one pending command and one reply. A command that arrives
while another waits is refused and counted (`clock_actuator.command_denied`);
a summary line `type=hybrid_command accepted=0 status=busy count=<n>` reports
refusals. `hybrid hold` and `clock abort` always take effect, even when
refused this way. Commands still waiting after 1.5 s (thermal commands: 1 s)
are rejected rather than run late. Send the next command only after the previous
one's reply.

## Clock commands

| Command | Effect |
| --- | --- |
| `hybrid status`, `hybrid config` | Send the status and configuration records now; `hybrid_config` includes the firmware `version`. |
| `hybrid hold` | Stop steering; the oscillator keeps its current trim. Heater unaffected. |
| `hybrid resume` | Clear a hold (and an acquisition fault); requalify, then steer. |
| `hybrid params ppp\|sbas\|standalone\|static <Tf> <Tphi>` | Loop pair for one solution type, whole seconds, 0 (off) or 10–86400. Live; not saved. `hybrid params <Tf> <Tphi>` sets PPP. `0 0` holds the trim whenever the receiver reports that type, in a fallback too. |
| `hybrid clip <ps>` | Per-sample bias clip, 1–5000 ps (default 2000). Each sample's bias innovation is clamped to ±clip before use. Live; not saved. |
| `hybrid save` | Save the four loop pairs, clip, PPS policy, GNSS selection and receiver-profile flag to MCU flash. Steering continues. |
| `pps policy continuous\|qualified\|off\|manual` | Receiver PPS output: always on, only while GNSS is usable, off, or left as configured. Live; save with `hybrid save`. |
| `pps status` | Send `hybrid_config` and `gnss_pps` now. |
| `system reboot mcu\|gnss\|all` | Restart the MCU, the receiver (software restart) or both. No save, no erase. |
| `system defaults` | Restore receiver defaults plus the project profile and the compiled loop defaults, and save both. Holds the clock. |
| `clock trial` | Diagnostic oscillator step test: holds discipline, then ramps the trim +1 ppb at ≤ 0.05 ppb/s, holds, and ramps back. Needs ≥ 30 min since boot, usable GNSS and a safe thermal state. |
| `clock abort` | End a `clock trial` and hold. |

`hybrid params`, `hybrid clip` and `hybrid save` wait (up to 1.5 s) for an
oscillator write in progress to be read back, then run. A held clock or a
running `clock trial` refuses `params` and `clip`.

Loop pair semantics: **Tf** is the frequency-loop (FLL) time constant;
**Tphi** the natural time of a critically damped phase PI loop (proportional
gain 2/Tphi, integral gain 1/Tphi², in ppb per ns of phase error). Both on:
the phase loop sets the FLL's target. Only Tphi: the phase loop sets the trim
directly. Only Tf: frequency only. Both 0: hold the current trim. Defaults:
PPP/HAS 0/10, SBAS 10/100, standalone 10/100, static 10/100 s.

## GNSS selection

| Command | Allowed solutions | Clock |
| --- | --- | --- |
| `gnss auto` | PPP/HAS + SBAS + standalone (default) | unchanged |
| `gnss has` | PPP/HAS + standalone | held |
| `gnss sbas` (or `gnss nohas`) | SBAS + standalone | held |
| `gnss standalone` | standalone | held |

These shortcuts select Rover positioning. The receiver uses the best
available solution within the allowed set. Each is checked by readback and
saved in the receiver and the MCU; `hybrid_config gnss_selection` reports
0 auto, 1 SBAS, 2 PPP/HAS, 3 standalone. Send `hybrid resume` after a held
selection to steer again.

A held selection or mask change, and a `gnss write` or clock-holding
`gnss exec`, first waits (up to 3 s) for the clock hold to
take effect, and is refused with the receiver untouched if it does not. If a
selection change fails after reaching the receiver (lost reply, wrong
readback, failed save), the firmware reads the receiver back, adopts the mode
it actually runs and saves that; `gnss_probe auto_fault=1` only while this
runs. Send `hybrid resume` only after the change has completed (the
`gnss_control` done event, or `gnss_selection` showing the new mode): a resume
sent earlier can make the closing save fail, which `auto_fault=1` reports and
a repeated command clears. Check `gnss_selection` to see which mode is in effect. A failed mask
change is followed by a mask readback: `mask_fault=1` then reports the failed
change, and the values shown are the receiver's actual masks.

**Static (fixed-position) PVT** uses the expert setter, with StandAlone or
StandAlone+SBAS (PPP requires Rover):

```text
gnss write setPVTMode,Static,StandAlone,auto
```

A static solution (PVT mode 3) is steered with the `static` loop pair once
the receiver has settled its position.

## Receiver settings

| Command | Effect |
| --- | --- |
| `gnss read <get…/lst… command>` | Run a whitelisted receiver query; the reply is forwarded as `gnss_ascii` chunks. |
| `gnss write <set… command>` | Run a whitelisted setter, check the readback, save the receiver configuration (Current → Boot). Holds the clock. |
| `gnss exec <exe… command>` | Run a whitelisted receiver action. A navigation-filter reset or configuration copy holds the clock. |
| `gnss masks` | Read the PVT and tracking elevation masks. |
| `gnss mask pvt\|tracking <deg>\|off` | Set one elevation mask (−90…90; `off` = −90) and save. Holds the clock. |
| `gnss masks reset` | Both masks to −90 (no exclusion) and save. Holds the clock. |
| `gnss save` | Save the receiver's current configuration to Boot. |
| `gnss defaults` | Restore receiver defaults plus the project profile and save. Holds the clock. |

The whitelist (receiver dynamics, masks, multipath, C/N0 masks, PVT mode,
signal tracking and usage, SBF output and others) and its argument checks are
in `tools/gnss_control_catalog.json`. `tools/gnss-control.py` prints the USB
line for a typed request, and with `--port` runs it and waits for completion:

```text
python tools/gnss-control.py catalog ReceiverDynamics
python tools/gnss-control.py --port <port> get ReceiverDynamics
python tools/gnss-control.py --port <port> set ReceiverDynamics Level=Low
```

Each operation reports `type=gnss_control v=1` events with one `seq` per
operation and `stage` before / write / after / save / done. Success is the
`stage=done` event with `status=complete` (read), `status=saved saved=1`
(write), `status=acknowledged` (exec) or `status=defaults_saved` (defaults),
and `drops=0 rx_loss=0 truncated=0`. Receiver replies arrive as
`type=gnss_ascii v=1 seq offset drops hex`: decode the hex and join chunks by
offset; `ascii_end` on the before and done events bounds the operation's bytes.

Receiver setting changes hold the clock; reads do not. Send `hybrid resume`
after a change to steer again.

## Thermal commands

| Command | Effect |
| --- | --- |
| `thermal status` | Acknowledge only (status is in the periodic records). |
| `thermal off` | Heater off; ends automatic temperature control until restart. |
| `thermal pi <target °C> [seconds]` | Run the PID at a fixed target (0–60 °C), indefinitely or for 1–1800 s. Replaces automatic startup. |
| `thermal manual <duty> <seconds>` | Fixed heater duty 0–1 (up to 3 decimals) for 1–300 s. |
| `thermal continue` | Continue a timed PID run as continuous control. |
| `fault clear` | Clear a latched heater fault once its cause is gone. |
| `outer pause` / `outer resume` | Stop / restart the slow target-centering loop. `outer resume` also clears an outer-loop safety fault. |
| `outer gain <1..20>` | Outer-loop plant gain estimate in K per unit duty; only while paused. |

Replies give `result=` OK, BUSY, NOT_READY, RANGE, EXPIRED, DISARMED,
FAULT_LATCHED or BAD_COMMAND. Thermal commands never affect clock discipline.

## Development profiles

The firmware also accepts `gnss identify`, `survey`, `once`, `stream`, `stop`,
`health`, `e6survey`, `e6stream`, `e6stop`, `pps`, `ppsoff`, `pppsurvey`,
`ppp`, `ppprestore` and `trial`. They were used during development. Several
change the receiver's output or PVT configuration and take over from automatic
operation; they are not needed to run a clock. `gnss health` (reads the
receiver's error log, which also acknowledges a latched software warning) and
`gnss identify` (identity, capabilities, permissions, current configuration)
are safe to run.

## Telemetry

Every line is `type=<name> key=value …`: single spaces between fields, no
spaces inside values, keys unique per line. Parse by key, keep unknown keys,
and treat `NA`, `unknown` or a missing field as unavailable, not zero. Records
are not an atomic snapshot and need not appear exactly once per second.

### Units

| Suffix | Unit |
| --- | --- |
| `_ps` | picoseconds (÷1000 → ns) |
| `_nppb` | 10⁻⁹ ppb (÷1e9 → ppb; 20000000 = 0.020 ppb) |
| `_permille` | ‰ duty (÷10 → %) |
| `_q16` | value × 65536 |
| `_ms`, `_s`, `_c` | milliseconds, seconds, °C |

1 ns/s of phase slope equals 1 ppb; at 10 MHz, 1 ppb is 0.01 Hz.

### Clock records (1 Hz)

| Record | Main fields |
| --- | --- |
| `clock_auto` | `state`, `enabled`, `reason` (bits below), `loop_mode`, `phase_mode`, `source_mode` (PVT mode: 1 standalone, 3 static, 6 SBAS, 10 PPP), `time_system`, `gnss_good`, `phase_enabled`, `relative_lock`, `frequency_settled`, `bias_ps`, `error_ps`, `d_nppb` (measured drift), `v_nppb` (drift target), `I_nppb` (integrator), `source_age_ms` |
| `clock_actuator` | `applied_nppb`, `requested_nppb`, `readback_fcw`, `commanded_fcw`, `pending`, `known`, `writes`, `limiting`, `slew_limited`, `phase_limited`, `tx_drops`, `command_denied` |
| `clock_guard` v3 | `clipped`, `clipped_samples`, `innovation_ps`, `lock_failures` (bits below), `reanchors`, `reanchor_total_ps`, `bridged_gaps`, `bridged_epochs`, `fallback` (below the selected solution: frequency loop only), `settling` (after a return to PPP), `gnss_usable`, `thermal_warning` |
| `hybrid_quality` | `reason`, frequency-monitor statistics (`frequency_mean_drift_nppb`, `frequency_sd_nppb`, …), window statistics, `receiver_config_ready`, `receiver_config_fault` |
| `hybrid_config` v4 | `version` (firmware release), `loop_mode`, all eight loop times (`ppp_fll_time_s` … `static_phase_time_s`), `bias_clip_ps`, `pps_policy`, `gnss_selection`, `settings_saved`, `settings_dirty`, `settings_generation`, `settings_fault` |
| `gnss_dop` | receiver `tdop_x100`, `nr_sv`, `valid`, `age_ms` (information only, never a control input) |

States: 2 QUALIFY, 3 ACQUIRE_FREQUENCY, 4 VERIFY_FREQUENCY, 5 TRACK,
6 PHASE_RECOVERING, 7 HOLDOVER, 8 MANUAL_HOLD, 9 ACTUATOR_UNKNOWN,
10 ACQUISITION_FAULT, 13 FREQUENCY_ONLY, 14 DISCIPLINE_OFF.

### Events

| Record | Sent when | Fields |
| --- | --- | --- |
| `hybrid_transition` | Controller state changes | `seq`, `state`, `reason` |
| `phase_reanchor` | Phase steering restarts on a new reference | `step_ps` (new − old reference), `total_ps`, `generation`, `source_mode` |
| `clock_failure` | Lock lost or a sample rejected/clipped | `flags` (lock-failure bits), `bias_ps`, `drift_nppb`, `trim_delta_nppb`, `drift_innovation_nppb`, `phase_error_ps` |
| `thermal_warning` | A warning bit sets or clears | `bits`, `previous` (1 sensor, 2 startup, 4 regulation, 8 duty near 0 or 100 %) |
| `system_reboot` | Reboot accepted / failed / receiver requalified | `target`, `stage` |

### Reason bits

| Bit | Meaning |
| --- | --- |
| 1 | GNSS input unusable |
| 8 | Oscillator (actuator) state or readback |
| 16 | Epoch or reference discontinuity |
| 32 | Input stale |
| 64 | Frequency innovation rejected |
| 128 | Qualification window not met |
| 256 | Control or envelope limit |
| 512 | Operator hold |

### Lock-failure bits

1 span not yet qualified, 2 phase error too large, 4 bias slope too large,
8 hard limit, 16 GNSS, 32 actuator, 64 epoch, 128 innovation.

### Thermal records

| Record | Main fields |
| --- | --- |
| `thermal_outer` | `target_c`, `raw_c`, `filtered_c`, `error_c`, `duty_permille`, `mean30_permille`, `enabled`, `pause_reason`, `centered`, inner PID terms |
| `thermal_pi` | PID state: `target_c`, `filtered_c`, `requested_permille`, `slope_c_per_min`, `stable`, `led_stable` |
| `thermal_boot` | Startup capture: `samples`, `slope_q16_per_s`, `criterion`, `selected_target_c` |
| `thermal` | Sensor and heater status: `temp_c`, `sample_age_ms`, `faults`, `duty_permille`, `auto_state` |

### Receiver and system records

| Record | Main fields |
| --- | --- |
| `gnss_probe` v4 | UART/SBF counters (`rx_overflow`, `rx_errors`, `crc_errors`, …), `ext_freq`, `auto_ready`, `auto_fault`, `pps_snapshot` (receiver PPS settings, commas only), current command state |
| `gnss_pps` | `policy`, `policy_valid`, `enabled`, `pending`, `fault`, `time_quality` (GNSS_VALID / HOLDOVER / FREE_RUNNING) |
| `epoch` | One assembled receiver epoch: `week`, `tow_ms`, `mode`, `time_system`, ages, `bias_bits`/`drift_bits` (raw IEEE hex), receiver status and error |
| `timing_status` | PPS capture and epoch-assembly counters |
| `resources` | Firmware `profile` and `version`, stack high-water mark, `guard`, `loop_gap_ms`, `reset_flags`, LED states |
| `ocxo_probe` | Oscillator register read-back (`fcw_m/n/l`, `id`, `clip`, `oe`) |

Receiver error 2048 (OUTOFFENCE) is the receiver's motion/geographic
restriction; it clears once the receiver has a position.
