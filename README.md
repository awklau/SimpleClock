# SimpleClock

SimpleClock is an open-source GNSS-disciplined 10 MHz reference built around a
Septentrio **mosaic-G5 T** timing receiver, a SiTime **SiT5801** digitally
controlled OCXO and an **STM32C071** microcontroller. It was developed as a
station clock for VLBI.

The OCXO provides the 10 MHz output and also clocks the GNSS receiver. The
microcontroller reads the receiver's clock bias and drift every second and
steers the OCXO's frequency control word, while a heater loop holds the
oscillator's temperature steady. It runs on its own from power-up; a computer
is only needed to program it or read its telemetry.

## Features

- Frequency and phase discipline with separate loop settings for each GNSS
  solution type: PPP/HAS (Galileo High Accuracy Service), SBAS, standalone
  and static (fixed position).
- Continuous steering through solution-type changes and short GNSS gaps
  (≤ 10 s); holdover on the last trim during longer outages.
- Two-stage oven control: a PID loop on the board temperature and a slow outer
  loop that keeps heater power near 50 %.
- USB serial interface with live telemetry, run-time tuning and receiver
  configuration; settings saved in MCU flash.
- Cross-platform tools for flashing, the receiver firmware upgrade and
  recording.

## Repository

| Folder | Content |
| --- | --- |
| [`firmware/`](firmware/README.md) | STM32 firmware, host tests and tools |
| [`hardware/`](hardware/) | Schematic, PCB production files, bill of materials |
| [`docs/`](docs/) | [First flashing](docs/FIRST_FLASHING.md) and [user manual](docs/USER_MANUAL.md) |

## Getting started

1. Build or order the board from [`hardware/`](hardware/).
2. Program it and upgrade the receiver: [first flashing](docs/FIRST_FLASHING.md).
   Pre-built images are attached to each [release](https://github.com/awklau/SimpleClock/releases).
3. Connect the antenna and heater, power it, and read the
   [user manual](docs/USER_MANUAL.md).

The firmware can also be built from source: see [firmware/](firmware/README.md).

## Status

<!-- TODO: release version and hardware-test summary before publishing. -->
Firmware release `<version>`. The control and flashing procedures have been
run on three boards. Tools run on Windows, Linux and macOS; the hardware
procedures have so far been carried out on Windows.

Receiver-derived telemetry and LEDs describe the clock as seen by its own
receiver; they are not an independent measurement of output stability. For
measured performance, see the paper below.

## Citation

If you use SimpleClock, please cite:

<!-- TODO: paper reference and DOI; keep CITATION.cff in step. -->
> `<authors>`, "`<title>`", `<venue>`, `<year>`. DOI: `<doi>`

GitHub's "Cite this repository" button uses [CITATION.cff](CITATION.cff).

## Licences

- Firmware and software: [MIT](LICENSE).
- Hardware design files: <!-- TODO: chosen CERN-OHL variant --> see
  [`hardware/LICENSE`](hardware/LICENSE).
- Third-party sources under `firmware/vendor/` keep their own licences; see
  [firmware/vendor/README.md](firmware/vendor/README.md).
