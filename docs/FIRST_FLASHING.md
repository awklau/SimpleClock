# First flashing of a new SimpleClock board

This guide takes a newly assembled board to running firmware:

1. set the STM32 boot options so the board can always be reprogrammed,
2. upgrade the mosaic-G5 T receiver to application firmware 1.1.0,
3. program the normal SimpleClock firmware.

It takes about 30 minutes, most of it the receiver upgrade. Steps 1 and 3 are
also how you update the firmware later. Read the whole guide once before you
start.

> The tools run on Windows, Linux and macOS. The whole procedure has been
> carried out on Windows. On Linux and macOS it is so far tested only in
> software; run the read-only checks the guide points out before writing
> anything.

## What you need

- A computer running Windows, Linux or macOS.
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html).
  Version 2.23.0 was used to write this guide. The flashing tool finds its
  command-line program (`STM32_Programmer_CLI`) on PATH or at ST's default
  install location; otherwise pass `--programmer <path>`.
- Python 3.11 or newer. On Linux and macOS also install pyserial:
  `python3 -m pip install -r tools/requirements.txt`. Windows needs no extra
  package for the steps below.
- A clone of this repository. Run all commands from its `firmware/` folder.
  The examples use `python`; on Linux and macOS this is usually `python3`.
- The two firmware images, from the GitHub release or built yourself
  ([building](../firmware/README.md)):
  - `simpleclock-gnss-bridge-<version>.bin`: maintenance image for the receiver upgrade;
  - `simpleclock-firmware-<version>.bin`: the normal firmware.
- The **mosaic-G5 T 1.1.0** firmware package from Septentrio. Use only the
  application file inside it, `mosaic-G5 T-1.1.0.suf` (2,951,207 bytes,
  SHA-256 `5e3cdd6a96e553e1ac33684cf576692b6a653983c5cb4d29374fa616d55e48c7`).
  Do not use the failsafe file in the same package. The upgrade tool refuses
  any other file.
- A jumper or short for **J17 (BOOT)**.

Keep the board powered from USB throughout. The heater stays off during steps
1 and 2. It may switch on at full power once the normal firmware runs (step 3).

### USB access on Linux and macOS

- **Linux:** ST's installer adds the USB rules STM32CubeProgrammer needs for the
  bootloader; if the board is not listed, install them as ST describes for
  your distribution. The bridge and the normal firmware appear as
  `/dev/ttyACM0` (or another number); your user needs access to it, usually
  by joining the `dialout` group (`sudo usermod -aG dialout $USER`, then log
  in again).
- **macOS:** the serial port appears as `/dev/cu.usbmodem…`; list them with
  `ls /dev/cu.usbmodem*`.
- **Windows:** the serial port appears as `COM<n>` in Device Manager.

## How the board boots

J17 pulls the STM32's BOOT0 pin (shared with PA14, pulled down by R9) high.
With J17 **closed** at power-up the STM32 starts its built-in USB bootloader
(ROM DFU), which STM32CubeProgrammer talks to. With J17 **open** it runs the
firmware in flash. J17 is read only when the STM32 starts: fitting or removing
it while the board runs changes nothing until the next power-up or reset.
Remove it after flashing; if it stays fitted, the next reset (including a
watchdog reset or `system reboot mcu`) starts the bootloader instead of the
clock firmware.

A factory-new STM32C071 has empty flash and starts the bootloader by itself,
even with J17 open. Once anything is in flash, J17 only works if the option
byte `nBOOT_SEL` is 0 (BOOT0 pin selects the boot mode). If it were left at 1,
the board would always run the flash image and would need an SWD debugger to
recover. The flashing tool checks this before every write and refuses to
program until it is right; step 1 sets it.

## Find the board's DFU serial

Close J17, then connect USB. List the bootloader devices with
STM32CubeProgrammer's command-line program:

```text
STM32_Programmer_CLI -l usb
```

(On Windows, if it is not on PATH:
`& 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe' -l usb`.)
Note the 12-character serial number, for example `0123456789AB`. Every
flashing command below selects the board by this serial, so other boards
connected at the same time are never touched. The examples write it as
`<board>`.

## Step 1: back up and set the boot options

With J17 closed and the board in the bootloader:

```text
python tools/program-stm32.py backup --board <board> --image simpleclock-gnss-bridge-<version>.bin
```

This saves the complete flash (128 KiB) and option bytes under
`tmp/flash/<board>-<image>/` and prints the boot options. If it shows a
warning, set them:

```text
python tools/program-stm32.py boot-options --board <board> --image simpleclock-gnss-bridge-<version>.bin
```

`boot-options` changes only `nBOOT_SEL` to 0 and `nBOOT1` to 1, then reads
every option byte back and fails if anything else changed. It never changes
the read-out protection (RDP) or boot lock. If the boot options are already
correct it writes nothing. The STM32 restarts after an option-byte write; with
J17 closed it returns to the bootloader, where the tool reconnects.

## Step 2: upgrade the GNSS receiver

The receiver is upgraded through the STM32, which acts as a USB-to-receiver
bridge. The commands write the bridge's serial port as `<port>` (for example
`COM5`, `/dev/ttyACM0` or `/dev/cu.usbmodem1101`).

1. Program the bridge (J17 still closed):

   ```text
   python tools/program-stm32.py program --board <board> --image simpleclock-gnss-bridge-<version>.bin
   ```

   It must end with `Programmed and verified`.
2. Unplug USB, **open J17**, plug USB back in. The bridge appears as a USB
   serial port (see [USB access](#usb-access-on-linux-and-macos)).
3. Check the USB path with the full file, without sending anything to the
   receiver:

   ```text
   python tools/gnss-upgrade.py selftest --port <port> --image "mosaic-G5 T-1.1.0.suf" --output receiver-selftest
   ```

4. Read the receiver identity and back up its settings. Note the `serial` and
   `version` in the `receiver_identity` line. If the version is already
   `1.1.0`, skip the upgrade and go to step 3.

   ```text
   python tools/gnss-upgrade.py probe --port <port> --output receiver-probe
   ```

   On Linux and macOS, run `selftest` and `probe` first and check that both
   finish without errors: neither writes anything to the receiver.
5. Upgrade (only from version 1.0.0; the tool refuses other versions):

   ```text
   python tools/gnss-upgrade.py upgrade --port <port> --image "mosaic-G5 T-1.1.0.suf" --receiver-serial <serial from probe> --output receiver-upgrade
   ```

   The transfer takes about 6 minutes and the whole run about 7. **Do not
   unplug the board or stop the program during the upgrade.** The tool sends
   the file once, waits for the receiver to install it and restart, and
   finishes with `upgrade_verified` and version 1.1.0.

Each run needs a new `--output` folder; it keeps a full record of the
receiver's replies. If anything fails, leave the board powered and keep the
output folder. The tool never retries or resets the receiver by itself, and a
receiver that may still be installing must not be power-cycled.

## Step 3: program the normal firmware

1. Unplug USB, **close J17**, plug USB back in (bootloader).
2. Back up, then program:

   ```text
   python tools/program-stm32.py backup  --board <board> --image simpleclock-firmware-<version>.bin
   python tools/program-stm32.py program --board <board> --image simpleclock-firmware-<version>.bin
   ```

   `program` refuses to write unless the boot options from step 1 are in
   place. It ends with `Programmed and verified`, after checking that the
   read-back image matches, the option bytes are unchanged and the settings
   pages at `0x0801F000` were not touched.
3. Unplug USB, **open J17**, connect the GNSS antenna and the heater, plug USB
   back in.

The firmware starts on its own; no computer is needed. On the first start it
configures the receiver (free-running clock, data streams, PPP/HAS + SBAS +
standalone, low/static dynamics), checks every setting by reading it back and
saves it in the receiver. It warms the oscillator at full heater power, then
regulates its temperature and starts disciplining the clock once the GNSS
solution qualifies. See the [user manual](README.md) for the LEDs,
commands and logging.

## Updating the firmware later

Close J17, power up, and run `backup` and `program` with the new image, as in
step 3. Saved settings survive, because programming never writes the settings
pages. The receiver does not need to be upgraded again.

## Optional board checks

Two small images help check a newly assembled board before or instead of the
steps above. Neither is needed to build a working clock. Program them like any
other image (`program`), then open J17 and power-cycle.

- **LED1 check** (`stm32c071-led1`, about 1 KB): blinks LED1 every 0.5 s using
  the STM32's internal clock. It touches no other pin and has no USB, so it
  shows that power, the MCU and programming work.
- **Clock probe** (`stm32c071-clock-probe`, about 20 KB): keeps the heater off
  and never writes to the oscillator. Over USB it reports the temperature
  sensor, the oscillator registers and the GNSS data stream, so you can check
  the board's parts without heating or steering anything.

Return to the normal firmware afterwards with step 3.

## If something goes wrong

- **The board is not listed by `-l usb`:** unplug, check J17 is closed, plug
  in again. Try another USB cable or port. On Linux, check ST's USB rules.
- **A serial port cannot be opened:** another program (a terminal, a logger)
  may hold it; close it. On Linux, check the `dialout` group.
- **The tool stops with an error:** nothing after the failing step was done.
  The evidence folder `tmp/flash/<board>-<image>/` holds every programmer log
  and the backups. `flash-before.bin` is the complete flash as it was before,
  including the settings pages; restore it with STM32CubeProgrammer if needed
  (the flashing tool itself never writes the settings pages).
- **Boot options refused (RDP or BOOT_LOCK):** the chip was configured outside
  this procedure. The tool does not change these; check them in
  STM32CubeProgrammer before going further.
