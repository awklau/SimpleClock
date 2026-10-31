"""Format/check first-party C and Python only; never traverse vendor or build output."""

import argparse
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Report drift without editing files")
    parser.add_argument("--clang-format", default="clang-format", help="Path to clang-format 19")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    c_files = sorted(
        str(path.relative_to(root))
        for directory in ("core", "platform", "tests")
        for path in (root / directory).rglob("*")
        if path.suffix in (".c", ".h")
    )
    # Explicit roots keep imported ST files and captured artifacts byte-for-byte intact.
    c_options = ["--dry-run", "--Werror"] if args.check else ["-i"]
    python_options = ["--check"] if args.check else []
    commands = [
        [args.clang_format, "--style=file", *c_options, *c_files],
        [sys.executable, "-m", "black", *python_options, "tools", "tests"],
    ]
    for command in commands:
        try:
            subprocess.run(command, cwd=root, check=True)
        except (OSError, subprocess.CalledProcessError) as error:
            print(f"Formatting failed: {error}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
