# SPDX-License-Identifier: MIT
"""Record SimpleClock USB telemetry and optionally send commands (any OS).

Opens the board's USB serial port (COM5, /dev/ttyACM0, /dev/cu.usbmodem...),
sends each --command once in order, then records every line for --seconds or
until --stop-file appears. Output folder (must be new):

  transport.log      raw lines as received
  status.jsonl       one JSON row per line: host arrival time, type, key=value fields
  events.jsonl       commands sent, uptime gaps/resets, stop reason
  run_manifest.json  what was recorded and how

A sent command is not an acknowledgement: the board answers each one with a
'type=hybrid_command ... accepted=0/1 command_hex=<ASCII hex>' line, which is
printed here. Host timestamps describe arrival, never the measurement epoch.
The port has one owner: stop other loggers on it first.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
import sys
import time
import uuid


def utc():
    return datetime.now(timezone.utc).isoformat()


def fields(line):
    out = {}
    for token in line.split(" "):
        key, sep, value = token.partition("=")
        if sep:
            out[key] = value
    return out


def capture(
    connection, output, seconds, commands, stop_file=None, clock=time.monotonic, echo=print
):
    """Record from an open serial-like connection; returns (records, stop_reason)."""
    started = clock()
    pending = bytearray()
    sent_hex = {command.encode("ascii").hex().upper(): command for command in commands}
    last_uptime, records, shown = {}, 0, 0
    with (
        (output / "transport.log").open("w", encoding="utf-8", newline="\n") as transport,
        (output / "status.jsonl").open("w", encoding="utf-8", newline="\n") as status,
        (output / "events.jsonl").open("w", encoding="utf-8", newline="\n") as events,
    ):

        def event(**record):
            events.write(json.dumps({"host_arrival_utc": utc(), **record}) + "\n")
            events.flush()

        for command in commands:
            connection.write((command + "\r\n").encode("ascii"))
            event(event="command", command=command)
        reason = "duration_elapsed"
        while clock() - started < seconds:
            if stop_file is not None and stop_file.exists():
                reason = "user_requested"
                event(event="capture_stop", reason=reason)
                break
            pending.extend(connection.read(4096))
            while b"\n" in pending:
                raw, _, pending = pending.partition(b"\n")
                line = raw.decode("ascii", errors="replace").rstrip("\r")
                if not line:
                    continue
                arrival, elapsed = utc(), clock() - started
                transport.write(line + "\n")
                data = fields(line)
                kind = data.get("type", "tmp117_legacy")
                row = {"host_arrival_utc": arrival, "host_elapsed_s": elapsed, "type": kind}
                status.write(json.dumps({**row, "fields": data}) + "\n")
                # SBF prefixes can share a completion tick, so skip them here.
                if "uptime_ms" in data and kind != "sbf_prefix" and data["uptime_ms"].isdigit():
                    uptime = int(data["uptime_ms"])
                    previous = last_uptime.get(kind)
                    if previous is not None and (uptime <= previous or uptime - previous > 1500):
                        event(
                            event="gap_or_reset",
                            type=kind,
                            previous_uptime_ms=previous,
                            uptime_ms=uptime,
                        )
                    last_uptime[kind] = uptime
                records += 1
                if kind == "hybrid_command" and data.get("command_hex", "").upper() in sent_hex:
                    echo(line)
                elif shown < 8:
                    echo(line)
                    shown += 1
            transport.flush()
            status.flush()
    return records, reason


def main(argv=None, open_port=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", required=True, help="board USB serial port")
    parser.add_argument("--output", required=True, type=Path, help="new folder for the recording")
    parser.add_argument("--seconds", type=float, default=75.0, help="recording length (default 75)")
    parser.add_argument("--command", action="append", default=[], help="send once; repeatable")
    parser.add_argument("--stop-file", type=Path, help="stop early when this file appears")
    parser.add_argument("--firmware-image", type=Path, help="image file to hash for provenance")
    parser.add_argument("--board-id", default="unknown", help="operator-supplied board serial")
    args = parser.parse_args(argv)
    if not 0 < args.seconds <= 31 * 86400:
        parser.error("--seconds must be in (0, 31 days]")
    if any(not command.isascii() or "\r" in command or "\n" in command for command in args.command):
        parser.error("commands must be single ASCII lines")
    if args.stop_file is not None and args.stop_file.exists():
        parser.error("--stop-file already exists; use a new sentinel path")
    if args.output.exists():
        parser.error("--output exists; use a new folder so no recording is overwritten")
    image_hash = (
        hashlib.sha256(args.firmware_image.read_bytes()).hexdigest()
        if args.firmware_image
        else None
    )
    if open_port is None:

        def open_port(port):
            import serial  # pyserial: pip install -r tools/requirements.txt

            return serial.Serial(port, 115200, timeout=0.25, write_timeout=1)

    args.output.mkdir(parents=True)
    start_utc, started = utc(), time.monotonic()
    records, reason, failure = 0, "capture_failure", None
    try:
        connection = open_port(args.port)
        try:
            records, reason = capture(
                connection, args.output, args.seconds, args.command, args.stop_file
            )
        finally:
            connection.close()
    except Exception as error:  # Keep the manifest; never reconnect or backfill.
        failure = str(error)
    manifest = {
        "schema": 8,
        "capture_id": str(uuid.uuid4()),
        "start_utc": start_utc,
        "end_utc": utc(),
        "requested_duration_s": args.seconds,
        "actual_duration_s": time.monotonic() - started,
        "records": records,
        "failure": failure,
        "stop_reason": reason,
        "stop_file": str(args.stop_file) if args.stop_file else "",
        "port": args.port,
        "commands": args.command,
        "firmware_path": str(args.firmware_image) if args.firmware_image else None,
        "firmware_sha256": image_hash,
        "board_id": args.board_id,
        "board_id_source": (
            "not supplied"
            if args.board_id == "unknown"
            else "operator supplied; not queried over USB"
        ),
        "host_logger": "tools/usb-capture.py",
        "note": "No reconnect or backfill; failures end the recording.",
    }
    (args.output / "run_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", "utf-8")
    print(f"Captured {records} records ({reason}): {args.output}")
    if failure:
        print(f"ERROR: {failure}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
