# SPDX-License-Identifier: MIT
"""Back up, set boot options, program and verify one SimpleClock STM32 over ROM DFU.

Drives STM32CubeProgrammer's CLI (Windows, Linux or macOS). The board must be in
the ROM bootloader: J17/BOOT fitted at power-up, or a factory-new STM32C071 with
empty flash. Every call selects the board by its DFU serial (sn=<board>), so
other boards connected at the same time are never touched.

  backup        save the 128 KiB flash and 116 option bytes; report boot options
  boot-options  set only nBOOT_SEL=0 / nBOOT1=1 when they differ, so J17 keeps
                selecting ROM DFU once flash holds an application; then prove no
                other option byte changed. Never changes RDP or BOOT_LOCK.
  program       refuse unless the boot options are correct; write the image at
                0x08000000, read it back, check option bytes and the settings
                pages (0x0801F000, 4 KiB) are unchanged.

Evidence goes to <evidence>/<board>-<first 12 hex of the image SHA-256>/.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
import re
import shutil
import struct
import subprocess
import sys

FLASH_BASE, FLASH_BYTES = 0x08000000, 131072
SETTINGS_OFFSET, SETTINGS_BYTES = 0x1F000, 4096  # Linker SETTINGS region.
OPTIONS_BASE, OPTIONS_BYTES = 0x1FFF7800, 116
# Boot options that keep ROM DFU reachable through J17 (PA14/BOOT0): BOOT0 pin
# selects boot, BOOT0=1 selects system memory, no boot lock, RDP level 0. Read
# from CubeProgrammer's own '-ob displ' decoding, one line per name.
BOOT_REQUIRED = {"RDP": 0xAA, "nBOOT_SEL": 0, "nBOOT1": 1, "BOOT_LOCK": 0}
WRITABLE = ("nBOOT_SEL", "nBOOT1")
# FLASH_OPTR is word 0 of the option area; nBOOT_SEL is bit 24, nBOOT1 bit 25
# (stm32c071xx.h). boot-options may change no other bit or byte.
OTHER_OPTR_BITS = 0xFCFFFFFF
CLI = "STM32_Programmer_CLI"
CLI_CANDIDATES = (
    r"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
    "~/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI",
    "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/"
    "STM32CubeProgrammer.app/Contents/MacOs/bin/STM32_Programmer_CLI",
)


class FlashError(RuntimeError):
    pass


def find_programmer(explicit=None):
    """Explicit path, then PATH, then ST's default install locations."""
    if explicit:
        return explicit
    found = shutil.which(CLI)
    if found:
        return found
    for candidate in CLI_CANDIDATES:
        path = Path(candidate).expanduser()
        if path.is_file():
            return str(path)
    raise FlashError(f"{CLI} not found; install STM32CubeProgrammer or pass --programmer")


def subprocess_runner(programmer):
    def run(arguments):
        done = subprocess.run([programmer, *arguments], capture_output=True)
        return done.returncode, (done.stdout + done.stderr).decode("utf-8", errors="replace")

    return run


def parse_boot_options(text):
    found = {}
    for name in BOOT_REQUIRED:
        hits = re.findall(rf"(?m)^\s*{name}\s*:\s*0x([0-9A-Fa-f]+)\b", text)
        if len(hits) != 1:
            raise FlashError(f"Option byte {name} not found exactly once in the programmer output")
        found[name] = int(hits[0], 16)
    return found


def boot_ok(found):
    return all(found[name] == value for name, value in BOOT_REQUIRED.items())


def describe(found):
    return " ".join(f"{name}=0x{value:X}" for name, value in found.items())


class Flasher:
    def __init__(self, board, image, evidence, run, port="USB1"):
        self.board, self.image, self.run, self.port = board, Path(image), run, port
        if not self.image.is_file():
            raise FlashError(f"Missing image: {image}")
        self.data = self.image.read_bytes()
        if not 8 <= len(self.data) <= SETTINGS_OFFSET:
            raise FlashError(f"Image size {len(self.data)} B is outside 8..{SETTINGS_OFFSET} B.")
        self.sha256 = hashlib.sha256(self.data).hexdigest()
        self.folder = Path(evidence) / f"{board}-{self.sha256[:12]}"
        self.folder.mkdir(parents=True, exist_ok=True)

    def cube(self, log, arguments):
        code, text = self.run(["-c", f"port={self.port}", f"sn={self.board}", *arguments])
        (self.folder / log).write_text(text, encoding="utf-8")
        if code != 0 or self.board not in text:
            raise FlashError(f"Programmer failed or board {self.board} not identified: {log}")
        return text

    def upload(self, log, address, size, name):
        path = self.folder / name
        self.cube(log, ["-u", f"0x{address:08X}", str(size), str(path)])
        data = path.read_bytes() if path.is_file() else b""
        if len(data) != size:
            raise FlashError(f"Incomplete read: {name}")
        return data

    def boot_options(self, log):
        return parse_boot_options(self.cube(log, ["-ob", "displ"]))

    def backup(self):
        if (self.folder / "flash-before.bin").exists():
            raise FlashError("Backup exists; inspect it instead of overwriting.")
        self.cube("identity.log", [])
        boot = self.boot_options("options-before.log")
        options = self.upload(
            "options-before-read.log", OPTIONS_BASE, OPTIONS_BYTES, "options-before.bin"
        )
        flash = self.upload("backup.log", FLASH_BASE, FLASH_BYTES, "flash-before.bin")
        print(f"flash-before.bin   sha256 {hashlib.sha256(flash).hexdigest()}")
        print(f"options-before.bin sha256 {hashlib.sha256(options).hexdigest()}")
        print("Boot options: " + describe(boot))
        if not boot_ok(boot):
            print(
                "WARNING: boot options do not keep J17 DFU recovery; run boot-options before program."
            )
        print(f"Backup complete. Image to program: {self.sha256} ({len(self.data)} B)")

    def require_backup(self):
        for name in ("flash-before.bin", "options-before.bin"):
            if not (self.folder / name).is_file():
                raise FlashError(f"Run backup first (missing {name}).")

    def set_boot_options(self):
        self.require_backup()
        boot = self.boot_options("boot-options-check.log")
        if boot_ok(boot):
            print(f"Boot options already correct: {describe(boot)}. Nothing written.")
            return
        if boot["RDP"] != BOOT_REQUIRED["RDP"] or boot["BOOT_LOCK"] != BOOT_REQUIRED["BOOT_LOCK"]:
            raise FlashError(
                f"RDP or BOOT_LOCK differs ({describe(boot)}). This tool never changes them; "
                "resolve manually."
            )
        before = self.upload(
            "boot-options-before-read.log", OPTIONS_BASE, OPTIONS_BYTES, "boot-options-before.bin"
        )
        writes = [f"{n}={BOOT_REQUIRED[n]}" for n in WRITABLE if boot[n] != BOOT_REQUIRED[n]]
        # Loading option bytes resets the MCU; with J17 fitted (or empty flash) it
        # returns to ROM DFU, where the programmer reconnects for the checks below.
        self.cube("boot-options-write.log", ["-ob", *writes])
        after_boot = self.boot_options("boot-options-after.log")
        after = self.upload(
            "boot-options-after-read.log", OPTIONS_BASE, OPTIONS_BYTES, "boot-options-after.bin"
        )
        old_optr, new_optr = struct.unpack_from("<I", before)[0], struct.unpack_from("<I", after)[0]
        result = {
            "utc": datetime.now(timezone.utc).isoformat(),
            "board": self.board,
            "written": " ".join(writes),
            "before": describe(boot),
            "after": describe(after_boot),
            "boot_options_correct": boot_ok(after_boot),
            "other_optr_bits_unchanged": ((old_optr ^ new_optr) & OTHER_OPTR_BITS) == 0,
            "other_option_bytes_unchanged": before[4:] == after[4:],
        }
        self.record("boot-options.json", result)
        if not (
            result["boot_options_correct"]
            and result["other_optr_bits_unchanged"]
            and result["other_option_bytes_unchanged"]
        ):
            raise FlashError(
                "Boot option verification failed: keep the board in DFU and inspect the "
                "evidence before any further write."
            )
        print("Boot options set and verified. Next: program.")

    def program(self):
        self.require_backup()
        if (self.folder / "program.log").exists():
            raise FlashError("Programming already attempted; inspect evidence before retrying.")
        self.cube("identity-before-program.log", [])
        boot = self.boot_options("options-before-program.log")
        if not boot_ok(boot):
            raise FlashError(
                f"Boot options would not keep J17 DFU recovery ({describe(boot)}). "
                "Run boot-options first; nothing was written."
            )
        options = self.upload(
            "options-before-program-read.log",
            OPTIONS_BASE,
            OPTIONS_BYTES,
            "options-before-program.bin",
        )
        candidate = self.folder / "candidate.bin"
        candidate.write_bytes(self.data)
        self.cube("program.log", ["-d", str(candidate), f"0x{FLASH_BASE:08X}", "-v"])
        readback = self.upload("readback.log", FLASH_BASE, len(self.data), "readback.bin")
        settings = self.upload(
            "settings-after-read.log",
            FLASH_BASE + SETTINGS_OFFSET,
            SETTINGS_BYTES,
            "settings-after.bin",
        )
        options_after = self.upload(
            "options-after-read.log", OPTIONS_BASE, OPTIONS_BYTES, "options-after.bin"
        )
        backup = (self.folder / "flash-before.bin").read_bytes()
        result = {
            "utc": datetime.now(timezone.utc).isoformat(),
            "board": self.board,
            "image": self.image.name,
            "sha256": hashlib.sha256(readback).hexdigest(),
            "expected_sha256": self.sha256,
            "length": len(self.data),
            "independent_readback_matches": readback == self.data,
            "boot_options": describe(boot),
            "option_bytes_unchanged": options == options_after,
            "settings_pages_unchanged": settings
            == backup[SETTINGS_OFFSET : SETTINGS_OFFSET + SETTINGS_BYTES],
            "application_observed": False,
        }
        self.record("verification.json", result)
        if not (
            result["independent_readback_matches"]
            and result["option_bytes_unchanged"]
            and result["settings_pages_unchanged"]
        ):
            raise FlashError(
                "Verification failed: keep the board in DFU and restore flash-before.bin if needed."
            )
        print("Programmed and verified. Remove J17/BOOT, power-cycle, then check the LEDs.")

    def record(self, name, result):
        text = json.dumps(result, indent=2)
        (self.folder / name).write_text(text + "\n", encoding="utf-8")
        print(text)


def main(argv=None, run=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("phase", choices=("backup", "boot-options", "program"))
    parser.add_argument("--board", required=True, help="12-character DFU serial (CLI -l usb)")
    parser.add_argument("--image", required=True, type=Path, help="application image (.bin)")
    parser.add_argument("--port", default="USB1", help="CubeProgrammer USB port name")
    parser.add_argument("--programmer", help="path to STM32_Programmer_CLI")
    parser.add_argument("--evidence", type=Path, default=Path("tmp/flash"))
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[0-9A-F]{12}", args.board):
        parser.error("--board must be the 12 upper-case hex characters of the DFU serial")
    try:
        if run is None:
            run = subprocess_runner(find_programmer(args.programmer))
        flasher = Flasher(args.board, args.image, args.evidence, run, args.port)
        {
            "backup": flasher.backup,
            "boot-options": flasher.set_boot_options,
            "program": flasher.program,
        }[args.phase]()
    except FlashError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
