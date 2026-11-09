# SimpleClock user manual

How to run a programmed SimpleClock: what it does on its own, what the LEDs
mean, how to record its data and the everyday commands. To program a new
board first, see [first flashing](FIRST_FLASHING.md). The complete command and
telemetry reference is in [COMMANDS](../firmware/docs/COMMANDS.md).

## 1. What the clock does by itself

SimpleClock needs no computer to run. At power-up (with J17 open) it:

1. **Heats the oscillator.** Once the temperature sensor reads valid data, the
   heater runs at full power. When the temperature rise has slowed to
   0.4 °C/min or less for 20 s, the current temperature becomes the target.
   A PID loop then holds that target, and a slow outer loop moves the target
   (within 0–60 °C, at most 0.004 K/s) until the heater sits near 50 % duty,
   so it keeps headroom in both directions. A raw reading of 70 °C or more
   switches the heater off and latches a fault.
2. **Configures the GNSS receiver.** It reads and checks the receiver settings
   it needs (free-running clock, data streams, the allowed solution types) and
   saves them in the receiver on the first start. Saved user choices are kept.
3. **Disciplines the oscillator.** When the receiver delivers valid solutions,
   the firmware measures the oscillator's frequency offset, corrects it in one
   bounded step, then steers continuously: a frequency loop and/or a phase
   loop, with time constants that depend on the solution type (section 4).

Temperature control and clock discipline are independent: a thermal problem
raises a warning but never stops the clock loop, and a GNSS outage never stops
temperature control.

### When GNSS is lost

- **Up to 10 s** without a usable solution (a lost epoch, one bad fix): the
  trim is held and steering continues afterwards as if nothing happened.
- **Longer:** the clock enters **HOLDOVER**. The oscillator keeps its last trim.
  When solutions return, the firmware requalifies them, corrects the frequency
  again if it moved, and starts phase steering from a **new phase reference**.
  The output stays continuous, but it is not pulled back to the time alignment
  it had before the outage. Each such restart is reported once in a
  `phase_reanchor` record (`step_ps`, running `total_ps`).

### Solution types and handovers

The receiver may use **PPP/HAS** (Galileo High Accuracy Service corrections),
**SBAS**, **standalone** (rover) or, if configured, **static** (fixed
position). The selection (section 5) sets the best type allowed: PPP for
`gnss auto` and `gnss has`, SBAS for `gnss sbas`, standalone for
`gnss standalone`. Steering continues without a pause when the type changes:

- Below the selected type (a **fallback**, such as PPP to SBAS during
  interference), only the frequency loop runs. The receiver's bias jumps by
  several ns between types, but its drift does not.
- Back on SBAS, phase steering restarts after its normal 120 s check.
- Back on PPP, the frequency loop alone continues for at least 1000 s while
  PPP re-converges. Phase steering then restarts from a fresh reference.

The output phase therefore never follows a fallback's bias. It may wander
slightly during the fallback, and that wander is left in place, not pulled
back. `clock_guard` shows `fallback=1` or `settling=1`.

A site that repeatedly sees only a few satellites (about 6) gives a poor PPP
solution. The firmware does not detect this by itself; select SBAS or
standalone for such a site (section 5). The same applies to a site that keeps
falling back, or one without HAS coverage on `gnss auto`. There, the clock would
stay frequency-only.

## 2. The LEDs

| LED | Off | Blinking (1 Hz) | Solid |
| --- | --- | --- | --- |
| **LED1** frequency | — | Frequency not yet settled | Measured frequency settled |
| **LED2** GNSS | No fresh valid fix | Valid fix, not PPP | Valid PPP fix |
| **LED3** thermal | Temperature control not available yet | Regulating, not yet settled | Settled (error ≤ 0.25 K, drift ≤ 0.1 K/min) |

**LED3 double flash** (two short flashes per second) means a fault: heater or
temperature sensor, oscillator control, or an internal check. Read the
telemetry to find which (section 6).

LED1 is solid when the last 50 drift samples average within ±0.05 ppb of zero
and of the requested drift, the last 121 samples scatter by no more than
0.1 ppb, and this has held for 10 s. It stays lit through short data gaps
(up to 120 s). LED1 reflects the receiver's view of the oscillator; it is not
an independent measurement of output stability, and no LED certifies UTC or
PPS accuracy.

## 3. Connect and record

The board appears as a USB serial port: `COM<n>` on Windows, `/dev/ttyACM<n>`
on Linux, `/dev/cu.usbmodem…` on macOS. It sends one line of telemetry per
record type each second, as `type=<name> key=value …`.

Install the one dependency once (`python -m pip install -r tools/requirements.txt`),
then record from the `firmware/` folder:

```text
python tools/usb-capture.py --port <port> --output run-2026-11-01 --seconds 3600
```

The output folder must be new. It receives:

| File | Content |
| --- | --- |
| `transport.log` | Every line exactly as received |
| `status.jsonl` | One JSON object per line: host arrival time, type and fields |
| `events.jsonl` | Commands sent, gaps or resets in the data, stop reason |
| `run_manifest.json` | Port, duration, record count, any failure |

Recording does not affect the clock; closing the program does not stop it.
To end a long recording early, add `--stop-file <path>` and create that file
from another terminal. Only one program can own the port at a time.

## 4. Loop settings

Each solution type has its own pair of times, in whole seconds:

- **Tf**: frequency loop (FLL) time constant;
- **Tphi**: phase loop natural time (a critically damped PI loop: gains
  2/Tphi and 1/Tphi²).

0 switches that loop off; otherwise each is 10–86400 s. Both on runs the
frequency loop with the phase loop on top; both off holds the current trim.
When both are on, Tphi of at least 2.5 × Tf is recommended.

| Solution | Default Tf / Tphi |
| --- | --- |
| PPP/HAS | 0 / 10 s (phase loop only) |
| SBAS | 10 / 100 s |
| Standalone (rover) | 10 / 100 s |
| Static (fixed position) | 10 / 100 s |

A pair applies while its type is the selected one. In a fallback, or while
PPP settles after a return, the frequency loop runs alone with a fixed Tf of
10 s (section 1).

**`0 0` means "never steer on this solution type"**, in a fallback too. The
receiver, not the firmware, picks the solution type, and it uses the best one
allowed. With SBAS set to `0 0` under `gnss auto`, the clock therefore holds
its trim for as long as the receiver stays on SBAS; it does not move on to
standalone. To keep the receiver off SBAS altogether, use `gnss has`
(section 5).

Change them live, then save so they survive a restart:

```text
hybrid params ppp 0 10
hybrid params sbas 10 100
hybrid params standalone 10 100
hybrid params static 10 100
hybrid save
```

A change applies at once, keeps the current trim and needs no hold or resume.
`hybrid save` keeps the clock running.

## 5. Everyday commands

Send commands with `usb-capture.py`, which prints the board's reply:

```text
python tools/usb-capture.py --port <port> --output cmd-1 --seconds 10 --command "hybrid config"
```

Every clock command is answered by a line
`type=hybrid_command … accepted=1` (or `accepted=0`); `command_hex` is the
command text in hexadecimal. A sent command is not done until that reply says
`accepted=1`.

| Command | Effect |
| --- | --- |
| `hybrid status`, `hybrid config` | Report state and settings now; `hybrid config` shows the firmware `version=` |
| `hybrid hold` | Stop steering at the current trim (heater keeps running) |
| `hybrid resume` | Resume steering; requalifies first |
| `hybrid params <type> <Tf> <Tphi>` | Change one loop pair (section 4) |
| `hybrid save` | Save loop pairs, bias clip, PPS policy and GNSS selection |
| `gnss auto` | Allow PPP/HAS + SBAS + standalone (the default) |
| `gnss has` / `gnss sbas` / `gnss standalone` | Allow only that solution set (standalone is always allowed); holds the clock, so send `hybrid resume` afterwards to steer |
| `pps policy continuous` / `qualified` / `off` / `manual` | When the receiver's PPS output is enabled; save with `hybrid save` |
| `system reboot mcu` / `gnss` / `all` | Restart the MCU, the receiver, or both; nothing is erased |
| `system defaults` | Restore receiver and loop defaults and save them; holds the clock, so send `hybrid resume` afterwards |

GNSS selections are saved in the receiver and the MCU. Use static (fixed
position) receiver mode only through the expert receiver commands described
in [COMMANDS](../firmware/docs/COMMANDS.md#receiver-settings).

## 6. Troubleshooting

Start with `hybrid config` and a short recording, then look at these fields:

| Symptom | Check |
| --- | --- |
| LED2 off | Antenna and sky view. In `gnss_probe`: `auto_ready=1` and `auto_fault=0` mean the receiver is configured. |
| LED2 on but the clock is not steering | `clock_auto`: `state`, `reason` (bit meanings in [COMMANDS](../firmware/docs/COMMANDS.md#reason-bits)), `enabled`, `gnss_good`. `enabled=0` means a hold: send `hybrid resume`. |
| LED1 keeps blinking | Normal for several minutes after start or a GNSS outage. `hybrid_quality` shows the frequency statistics. |
| LED3 double flash | `thermal_warning` and `thermal` records name the fault. After fixing the cause, `fault clear` clears a heater fault; `outer resume` clears an outer-loop safety fault. |
| Frequent `phase_reanchor` records | Repeated GNSS outages longer than 10 s, or repeated fallbacks and returns (`clock_guard` `fallback`, `settling`). Check the antenna; for a site that keeps falling back, select SBAS or standalone. |
| Receiver error `2048` in status | The receiver's geographic/motion restriction (OUTOFFENCE); it clears itself once the receiver has a position. |

A board that does not respond over USB can still be reprogrammed: fit J17,
power-cycle, and follow [first flashing](FIRST_FLASHING.md) step 3.

## 7. Known limitations

- **Saving while steering.** A receiver change ends by saving the selection in
  MCU flash, which needs a moment with no oscillator write in progress. The
  firmware pauses the clock for this, but two cases can still fail and report
  `gnss_probe auto_fault=1` with the clock in HOLDOVER:
  - `gnss auto` sent while steering, after loop settings were changed with
    `hybrid params` or `hybrid clip` and not saved;
  - `hybrid resume` sent before a receiver change has completed.

  Sending the command again clears it. To avoid it, send `hybrid hold` first
  (or `hybrid save` after tuning), wait for the change to complete, then
  `hybrid resume`.
- **A refused change leaves the clock held.** If a receiver change is refused
  (`busy`, or no hold within 3 s), the receiver is untouched but the clock
  stays in MANUAL_HOLD until `hybrid resume`.
- **PPP settle needs a visible fallback.** The 1000 s settle starts when the
  receiver returns to PPP from SBAS or standalone. If the receiver reports PPP
  again directly after an outage, only the normal 120 s phase check applies.
- **Not hardware-tested in this release:** the ±1000 ppb boot envelope, the
  ±10 ppb phase-only tracking span, `0 0` pairs in a fallback, and repeated
  `gnss auto` while steering are covered by host tests only.
