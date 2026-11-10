# Tools

Run from the `firmware/` folder with Python 3.11 or newer. Live serial I/O on
Linux and macOS (and USB recording or receiver commands on any system) needs
pyserial: `pip install -r tools/requirements.txt`.

| Tool | Purpose |
| --- | --- |
| `program-stm32.py` | Back up, set the boot option bytes and program one board over ROM DFU through STM32CubeProgrammer's CLI. See [first flashing](../../docs/FIRST_FLASHING.md). |
| `gnss-upgrade.py` | Upgrade the mosaic-G5 T to application 1.1.0 through the bridge image (`selftest`, `probe`, `upgrade`). |
| `gnss_bridge.py` | Binary transport used by `gnss-upgrade.py` (Windows ctypes or pyserial). |
| `usb-capture.py` | Record the board's USB telemetry and send commands. |
| `gnss-control.py`, `gnss_control.py`, `gnss_control_catalog.json` | Typed, checked receiver-setting requests (`gnss read/write/exec`); prints the USB line or runs it with `--port`. |
| `generate-hybrid-defaults.py` | Generate `core/include/simpleclock/hybrid_defaults.h` from `default_parameters.json` (`--check` verifies it). |
| `test-resource-gates.py` | Check that the linker's flash and RAM limits reject oversized builds. |
| `format-code.py` | Format or check C (clang-format 19) and Python (black). |
| `env.ps1`, `toolchain.lock.json` | Windows: select Visual Studio's build tools and a locked Arm toolchain under `.tools/`. |

## Flashing

```text
python tools/program-stm32.py backup       --board <DFU serial> --image <file.bin>
python tools/program-stm32.py boot-options --board <DFU serial> --image <file.bin>
python tools/program-stm32.py program      --board <DFU serial> --image <file.bin>
```

`backup` saves flash and option bytes under `tmp/flash/<board>-<image hash>/`
and reports the boot options. `boot-options` sets only `nBOOT_SEL=0` and
`nBOOT1=1` when they differ, so J17 (BOOT0) keeps selecting the bootloader once
flash holds an application, and checks that no other option byte changed.
`program` refuses to write unless those options are correct, then programs,
reads back and checks that option bytes and the settings pages are unchanged.

## Receiver upgrade

```text
python tools/gnss-upgrade.py selftest --port <port> --image <application.suf> --output <new folder>
python tools/gnss-upgrade.py probe    --port <port> --output <new folder>
python tools/gnss-upgrade.py upgrade  --port <port> --image <application.suf> --receiver-serial <serial> --output <new folder>
```

Only the G5-T 1.1.0 application file with the pinned SHA-256 is accepted, and
`upgrade` only runs from application 1.0.0 on the receiver whose serial you
give. The bytes are sent once; the tool never retries or resets the receiver.

## Recording and commands

```text
python tools/usb-capture.py --port <port> --output <new folder> --seconds 600
python tools/usb-capture.py --port <port> --output <new folder> --seconds 10 --command "hybrid config"
```

The recording format is described in the [user manual](../../docs/README.md#3-connect-and-record).
