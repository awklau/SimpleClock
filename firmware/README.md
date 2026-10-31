# SimpleClock firmware

Firmware for the SimpleClock board's STM32C071 microcontroller, plus the host
tools to build, test, flash and operate it. Run all commands below from this
`firmware/` folder.

## Requirements

| For | Needs |
| --- | --- |
| Host tests | CMake ≥ 3.25, Ninja, a C17 compiler (GCC, Clang or MSVC) |
| Firmware images | [Arm GNU Toolchain 14.3.Rel1](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) (`arm-none-eabi`), on PATH |
| Tools and Python tests | Python ≥ 3.11; for live serial I/O also `pip install -r tools/requirements.txt` |
| Flashing | [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) |

On Windows, `. .\tools\env.ps1` in PowerShell selects Visual Studio's build
tools and a toolchain unpacked under `.tools/` (see `tools/toolchain.lock.json`).
Builds never download anything and never program a board.

## Build and test

```text
cmake --preset native
cmake --build --preset native
ctest --preset native
python -m unittest discover -s tests -p "test_*.py"

cmake --preset stm32c071-auto-hybrid
cmake --build --preset stm32c071-auto-hybrid
```

`native-release` runs the host tests in Release. Images, with map, size and
resource reports, are written to `build/<preset>/artifacts/`. The CDC
applications share the file name `stm32c071_tmp117_cdc.bin`: the preset folder
says which image it is.

| Preset | Image |
| --- | --- |
| `stm32c071-auto-hybrid` | **Normal firmware**: temperature control and GNSS clock discipline, autonomous from power-up |
| `stm32c071-gnss-bridge` | Maintenance bridge for the receiver firmware upgrade (`tools/gnss-upgrade.py`) |
| `stm32c071-led1` | Optional board check: blinks LED1 |
| `stm32c071-clock-probe` | Optional board check: heater off, read-only oscillator, sensor and GNSS telemetry |
| `arm-none-eabi` | Cross-compiled core library only |
| `native`, `native-release` | Host build of the core library and all C tests |

The linker script enforces the flash and RAM budgets (normal firmware: 112 KiB
flash, 19 KiB static RAM, 4 KiB stack) and keeps the last two flash pages
(`0x0801F000`, 4 KiB) for saved settings. To flash a board, see
[first flashing](../docs/FIRST_FLASHING.md).

## Configuration

`default_parameters.json` holds the controller defaults and limits (loop time
constants, qualification thresholds, thermal settings). After editing it,
regenerate the header and check it is current:

```text
python tools/generate-hybrid-defaults.py
python tools/generate-hybrid-defaults.py --check
```

Values changed at run time over USB (`hybrid params`, `hybrid save`) override
the compiled defaults; see [COMMANDS](docs/COMMANDS.md).

## Code layout

```text
core/                portable C17, no allocation, no hardware access
  include/simpleclock/   public headers and their contracts
  src/                   implementations
platform/stm32c071/  board support: startup, linker script, drivers, USB, adapters
vendor/              pinned ST CMSIS, HAL and USB device sources (their own licences)
tests/               C tests (host fakes under tests/stubs/) and Python tests
tools/               flashing, receiver upgrade, USB capture, receiver control, build helpers
cmake/               Arm toolchain description
```

| Core module | Responsibility |
| --- | --- |
| `hybrid`, `hybrid_input`, `hybrid_stats` | Clock controller: qualification, frequency acquisition, FLL and phase PI, handovers, holdover; receiver-epoch validity; windowed statistics |
| `frequency_monitor` | Independent frequency-settled indication (LED1); never authorizes steering |
| `sit5801` | SiT5801 oscillator registers and 39-bit frequency control word |
| `thermal_boot`, `thermal_control`, `thermal_outer`, `heater`, `temperature`, `tmp117` | Startup target capture, inner PID, outer centering loop, heater duty and deadlines, TMP117 sensor |
| `sbf`, `sbf_fields`, `timing`, `crc16` | Septentrio SBF framing and fields, receiver-epoch assembly |
| `gnss_command`, `gnss_control`, `gnss_control_lists` | Receiver command sequencing, whitelisted settings, readback checks |
| `settings` | Saved-settings record (two flash slots, CRC, commit word, migration) |
| `status_leds`, `text`, `bridge`, `fcw_trial`, `preheat` | LED patterns, bounded telemetry text, bridge framing, oscillator step test, legacy preheat |

The board adapters in `platform/stm32c071/src/` connect these to hardware:
`hybrid_probe.c` (clock control and its USB commands/telemetry),
`thermal_probe.c` (heater and sensor), `gnss_probe.c` (receiver UART,
configuration and commands), `timing_probe.c` (PPS capture, epochs) and
`clock_probe.c` (main loop service, watchdog, LEDs, resources).
[Architecture](docs/ARCHITECTURE.md) describes how they fit together.

## Rules for changes

- Keep hardware-independent logic in `core/` with units and invalid inputs
  stated; keep board effects in `platform/`.
- Interrupt handlers do bounded transport work only; parsing, command
  sequencing and formatting run in the foreground loop. Shared state uses short
  interrupt-masked snapshots, not `volatile` alone.
- Every telemetry line must follow the format in [COMMANDS](docs/COMMANDS.md#telemetry);
  the C tests check every line they produce (`tests/stubs/telemetry_check.h`).
- New receiver commands need an explicit command list, bounded timeouts and a
  readback rule. There is no free-form passthrough.
- Before a pull request: native tests (Debug and Release), Python tests, all
  firmware presets, `tools/test-resource-gates.py`, the defaults check and
  `python tools/format-code.py --check` (clang-format 19, black). CI runs the
  tests on Linux, macOS and Windows and builds the firmware images on Linux and
  macOS.

## Licences and USB identity

Original firmware and tools are MIT ([LICENSE](../LICENSE)). Files under
`vendor/` keep their upstream licences; the ST USB device middleware is under
ST's SLA0044 terms, which allow use only on ST microcontrollers. See
[vendor/README.md](vendor/README.md).

The USB VID:PID `1209:0001` is the pid.codes test identity, suitable for
development only.
