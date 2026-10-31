"""Exercise the DFU flasher's boot-option gate and verification against a fake programmer."""

import contextlib
import importlib.util
import io
from pathlib import Path
import struct
import sys
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
spec = importlib.util.spec_from_file_location("program_stm32", TOOLS / "program-stm32.py")
flash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flash)

BOARD = "0123456789AB"
# 116 option bytes read from a production board (FLASH_OPTR 0x3EEFFEAA: RDP 0xAA,
# nBOOT_SEL 0, nBOOT1 1, nBOOT0 1), little-endian words.
OPTIONS = struct.pack(
    "<29I",
    *(
        [0x3EEFFEAA, 0xC1100155, 0x000000FF, 0xFFFFFF00, 0, 0xFFFFFFFF, 0x3F, 0xFFFFFFC0]
        + [0x3F, 0xFFFFFFC0, 0xFF, 0xFFFFFF00, 0, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF]
        + [0xFFFFFFFF] * 12
        + [0]
    ),
)
# Trimmed from STM32CubeProgrammer 2.23.0 '-ob displ'; look-alike names stay.
DISPLAY = """SN          : {sn}
OPTION BYTES BANK: 0
   Read Out Protection:
     RDP          : 0x{rdp:X} (Level 0, no protection)
   User Configuration:
     nBOOT_SEL    : 0x{sel:X} (BOOT0 pin (legacy mode))
     nBOOT1       : 0x{boot1:X} (Boot from Flash if BOOT0 = 1, otherwise system memory)
     nBOOT0       : 0x1 (nBOOT0=1)
     PCROP_RDP    : 0x0 (PCROP zone is kept when RDP is decreased)
     BOOT_LOCK    : 0x{lock:X} (Boot based on the pad/option bit configuration)
"""


class FakeCube:
    """Simulated STM32_Programmer_CLI holding flash and option bytes."""

    def __init__(self, boot_sel=0, boot_lock=0, corrupt_write=False):
        self.options = bytearray(OPTIONS)
        optr = struct.unpack_from("<I", self.options)[0]
        optr = optr | (1 << 24) if boot_sel else optr & ~(1 << 24)
        struct.pack_into("<I", self.options, 0, optr)
        self.flash = bytearray(b"\xa5" * flash.FLASH_BYTES)
        self.boot_lock, self.corrupt_write = boot_lock, corrupt_write
        self.option_writes, self.programs = [], 0
        self.fail_next = False

    def __call__(self, arguments):
        sn = next(a[3:] for a in arguments if a.startswith("sn="))
        if self.fail_next:
            self.fail_next = False
            return 1, f"SN : {sn}\nError: simulated"
        text = f"STM32CubeProgrammer v2.23.0 (fake)\nSN          : {sn}\n"
        if "-ob" in arguments:
            rest = arguments[arguments.index("-ob") + 1 :]
            optr = struct.unpack_from("<I", self.options)[0]
            if rest == ["displ"]:
                return 0, DISPLAY.format(
                    sn=sn,
                    rdp=optr & 0xFF,
                    sel=optr >> 24 & 1,
                    boot1=optr >> 25 & 1,
                    lock=self.boot_lock,
                )
            for item in rest:
                name, value = item.split("=")
                bit = {"nBOOT_SEL": 24, "nBOOT1": 25}[name]
                optr = optr | (1 << bit) if int(value) else optr & ~(1 << bit)
            if self.corrupt_write:
                optr ^= 1 << 16
            struct.pack_into("<I", self.options, 0, optr)
            self.option_writes.append(rest)
            return 0, text + "Option Bytes successfully programmed\n"
        if "-u" in arguments:
            i = arguments.index("-u")
            address, size, path = int(arguments[i + 1], 16), int(arguments[i + 2]), arguments[i + 3]
            if address == flash.OPTIONS_BASE:
                data = bytes(self.options[:size])
            else:
                offset = address - flash.FLASH_BASE
                data = bytes(self.flash[offset : offset + size])
            Path(path).write_bytes(data)
            return 0, text + "Upload done\n"
        if "-d" in arguments:
            data = Path(arguments[arguments.index("-d") + 1]).read_bytes()
            self.flash[: len(data)] = data
            self.programs += 1
            return 0, text + "Download verified successfully\n"
        return 0, text


class FlasherTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        self.evidence = root / "evidence"
        self.image = root / "firmware.bin"
        self.image.write_bytes(bytes(range(256)) * 40)
        self.bridge = root / "bridge.bin"
        self.bridge.write_bytes(b"\x5a" * 13708)

    def phase(self, cube, name, image=None):
        out, err = io.StringIO(), io.StringIO()
        argv = [name, "--board", BOARD, "--image", str(image or self.image)]
        argv += ["--evidence", str(self.evidence)]
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(err):
            code = flash.main(argv, run=cube)
        return code, out.getvalue() + err.getvalue()

    def test_board_with_correct_options(self):
        cube = FakeCube(boot_sel=0)
        code, text = self.phase(cube, "backup")
        self.assertEqual(code, 0, text)
        self.assertIn("nBOOT_SEL=0x0", text)
        self.assertNotIn("WARNING", text)
        code, text = self.phase(cube, "boot-options")
        self.assertEqual((code, cube.option_writes), (0, []), text)
        self.assertIn("already correct", text)
        code, text = self.phase(cube, "program")
        self.assertEqual(code, 0, text)
        self.assertIn("Programmed and verified", text)
        self.assertEqual(bytes(cube.flash[: self.image.stat().st_size]), self.image.read_bytes())

    def test_program_refused_until_boot_options_set(self):
        cube = FakeCube(boot_sel=1)
        code, text = self.phase(cube, "backup")
        self.assertEqual(code, 0, text)
        self.assertIn("WARNING", text)
        code, text = self.phase(cube, "program")
        self.assertEqual((code, cube.programs), (1, 0), text)
        self.assertIn("Run boot-options first", text)
        code, text = self.phase(cube, "boot-options")
        self.assertEqual(code, 0, text)
        self.assertEqual(cube.option_writes, [["nBOOT_SEL=0"]])
        self.assertIn('"other_option_bytes_unchanged": true', text)
        code, text = self.phase(cube, "program")
        self.assertEqual((code, cube.programs), (0, 1), text)

    def test_boot_lock_is_never_touched(self):
        cube = FakeCube(boot_sel=1, boot_lock=1)
        self.phase(cube, "backup")
        code, text = self.phase(cube, "boot-options")
        self.assertEqual((code, cube.option_writes), (1, []), text)
        self.assertIn("never changes them", text)

    def test_unrelated_option_change_is_detected(self):
        cube = FakeCube(boot_sel=1, corrupt_write=True)
        self.phase(cube, "backup")
        code, text = self.phase(cube, "boot-options")
        self.assertEqual(code, 1, text)
        self.assertIn('"other_optr_bits_unchanged": false', text)
        self.assertIn("verification failed", text)

    def test_first_flash_bridge_then_firmware(self):
        cube = FakeCube(boot_sel=1)
        for name in ("backup", "boot-options", "program"):
            code, text = self.phase(cube, name, self.bridge)
            self.assertEqual(code, 0, f"{name}: {text}")
        for name in ("backup", "program"):
            code, text = self.phase(cube, name)
            self.assertEqual(code, 0, f"{name}: {text}")
        self.assertEqual(cube.programs, 2)
        folders = sorted(path.name for path in self.evidence.iterdir())
        self.assertEqual(len(folders), 2)  # One evidence folder per image hash.

    def test_settings_pages_and_oversize_images_protected(self):
        oversize = Path(self.directory.name) / "oversize.bin"
        oversize.write_bytes(bytes(flash.SETTINGS_OFFSET + 1))
        cube = FakeCube()
        code, text = self.phase(cube, "backup", oversize)
        self.assertEqual(code, 1, text)
        self.assertIn("outside 8..126976", text)
        self.assertFalse(self.evidence.exists())

    def test_programmer_failure_stops_before_writing(self):
        cube = FakeCube()
        self.phase(cube, "backup")
        cube.fail_next = True
        code, text = self.phase(cube, "program")
        self.assertEqual((code, cube.programs), (1, 0), text)
        self.assertIn("Programmer failed", text)
        code, text = self.phase(cube, "program")  # A retry is allowed: nothing was written.
        self.assertEqual(code, 0, text)

    def test_wrong_board_output_is_rejected(self):
        cube = FakeCube()
        wrong = lambda arguments: cube(
            [a if not a.startswith("sn=") else "sn=FFFFFFFFFFFF" for a in arguments]
        )
        code, text = self.phase(wrong, "backup")
        self.assertEqual(code, 1, text)
        self.assertIn("not identified", text)

    def test_board_serial_format_is_checked(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            flash.main(
                ["backup", "--board", "0123456789ab", "--image", str(self.image)], run=FakeCube()
            )


if __name__ == "__main__":
    unittest.main()
