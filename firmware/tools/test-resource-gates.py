"""Exercise actual linker flash/static-RAM gates; temporary objects stay in build/."""

from pathlib import Path
import argparse
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--toolchain-bin", type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
out = root / "build/resource-gate-tests"
out.mkdir(parents=True, exist_ok=True)
script = root / "platform/stm32c071/linker/stm32c071_flash.ld"
gcc = args.toolchain_bin / "arm-none-eabi-gcc"
ld = args.toolchain_bin / "arm-none-eabi-ld"
for name, section, size, hybrid, expected in [
    ("within", ".bss", 4, False, None),
    ("static_overflow", ".bss", 14337, False, "static RAM exceeds 14 KiB"),
    (
        "flash_overflow",
        ".rodata",
        49153,
        False,
        "loadable image exceeds reviewed profile flash goal",
    ),
    ("hybrid_within", ".bss", 19000, True, None),
    ("hybrid_static_overflow", ".bss", 19457, True, "static RAM exceeds 14 KiB"),
    (
        "hybrid_flash_overflow",
        ".rodata",
        114689,
        True,
        "loadable image exceeds reviewed profile flash goal",
    ),
]:
    source = out / f"{name}.s"
    source.write_text(
        '.syntax unified\n.thumb\n.section .text.Reset_Handler,"ax",%progbits\n'
        ".global Reset_Handler\n.thumb_func\nReset_Handler:\n b .\n"
        f".section {section}\n.space {size}\n",
        encoding="ascii",
    )
    obj = source.with_suffix(".o")
    subprocess.run(
        [str(gcc), "-mcpu=cortex-m0plus", "-mthumb", "-c", str(source), "-o", str(obj)], check=True
    )
    result = subprocess.run(
        [
            str(ld),
            *(["--defsym=__hybrid_release=1"] if hybrid else []),
            "-T",
            str(script),
            str(obj),
            "-o",
            str(source.with_suffix(".elf")),
        ],
        capture_output=True,
        text=True,
    )
    if expected is None:
        assert result.returncode == 0, result.stderr
    else:
        assert result.returncode != 0 and expected in " ".join(result.stderr.split()), result.stderr
    print(f"{name}: PASS")
