# GNSS receiver configuration

The firmware manages a Septentrio **mosaic-G5 T** running application firmware
**1.1.0** (the version that advertises the `PPPGalileoHAS-SIS` capability).
Galileo HAS also needs an antenna that covers E6. Upgrading the receiver is
part of [first flashing](../../docs/FIRST_FLASHING.md).

## What the firmware sets up

At every start the firmware first **queries** the receiver, then applies only
the settings whose read-back does not already match:

| Item | Setting |
| --- | --- |
| Clock synchronization | `setClockSyncThreshold, usec500, on` |
| Timing system | `setTimingSystem, auto` |
| COM1 output | SBF added (`setDataInOut, COM1, ,+SBF`) |
| Stream1, every 1 s | PVTGeodetic, ReceiverTime, ReceiverStatus, MeasEpoch, EndOfPVT, PosCovCartesian, VelCovCartesian, RFStatus, DOP |
| PVT mode | Rover with the allowed solution set (default StandAlone+SBAS+PPP) |
| Receiver dynamics | Low, Static (first start only) |

It also checks the PPS settings, time-sync source, capabilities and signal
tracking/usage. After a successful first setup it saves the receiver's
configuration to Boot and marks the receiver as **managed** in the MCU
settings. From then on startup only reads the configuration and accepts the
user's own choices (for example different dynamics or Static PVT) instead of
overwriting them. Each setting is accepted only after its read-back matches;
`gnss_probe auto_ready=1 auto_fault=0` reports a receiver ready for control.
Configuration retries every 30 s if the receiver does not answer.

`gnss defaults` (and `system defaults`) copies the receiver's factory defaults
to its current configuration, turns PPS off, enables Galileo E6 BC tracking and
navigation use, sets Rover StandAlone+SBAS+PPP and then reapplies the setup
above and saves it.

## When a solution may steer the clock

Every solution must be fresh (≤ 2.5 s), complete, numeric and with successful
RAIM, with a clear RF and receiver error status, the external 10 MHz reference
present and the receiver's own clock steering off. In addition:

| Solution | PVT mode | Extra requirement |
| --- | --- | --- |
| PPP/HAS | 10 | Galileo time, orbit and clock corrections in use, correction age ≤ 30 s |
| SBAS | 6 | Correction age reported |
| Standalone | 1 | — |
| Static | 3 | Static PVT configured and the fixed position settled |

The satellite count is reported (`gnss_dop nr_sv`) but is not a requirement.
A site that keeps only about 6 satellites gives a poor PPP solution; select
SBAS or standalone there (`gnss sbas`, `gnss standalone`). The receiver has no
setting for a minimum satellite count in PPP.

## Static (fixed-position) PVT

Static PVT fixes the antenna position and solves only for time, which suits a
fixed station. Select it with the expert setter:

```text
gnss write setPVTMode,Static,StandAlone,auto
gnss write setPVTMode,Static,StandAlone+SBAS,auto
```

`auto` lets the receiver determine and then fix the position; a stored
Geodetic1–5 or Cartesian1–5 position can be named instead. PPP cannot be
combined with Static (the firmware refuses that combination), so static
operation does not fall back from PPP: choose Rover (with PPP) or Static per
site. Static solutions use the `static` loop pair.

## Receiver errors

- **8 (SOFTWARE)** after a restart: the receiver latches a warning (for example
  "Start up: after PLL unlocked issue"). If this is the only error bit and the
  external reference is present, the firmware reads the receiver's error log
  once after startup, which acknowledges the warning. A warning that returns is
  not acknowledged again; `gnss health` reads the log on request.
- **2048 (OUTOFFENCE)**: the receiver's motion or geographic restriction. It
  clears once the receiver has a position; `gnss read lstInternalFile,RxMessages`
  shows the reason.

Any receiver error blocks steering until it clears.
