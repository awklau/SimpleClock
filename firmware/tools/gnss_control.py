# SPDX-License-Identifier: MIT
"""Typed G5-T controls for the normal SimpleClock USB firmware.

Without --port this prints the USB command only. With --port it owns one serial
session, checks installed command descriptions before writes, and records
sanitized replies. A host catalog is a schema, not proof of receiver support.
"""
from __future__ import annotations
import argparse
import csv
from decimal import Decimal, InvalidOperation
import hashlib
import json
from pathlib import Path
import re
import time

CATALOG_PATH = Path(__file__).with_name("gnss_control_catalog.json")
CATALOG = json.loads(CATALOG_PATH.read_text(encoding="utf-8"))
MAX_USB = 192


def satellite(value: str, *, channel=False, usage=False) -> bool:
    match = re.fullmatch(r"([GREFSCJI])(\d{2,3})", value)
    if not match:
        return False
    code, number = match[1], int(match[2])
    ranges = {
        "G": (1, 32 if usage else 36),
        "R": (1, 30),
        "E": (1, 36),
        "S": (120, 158),
        "C": (1, 63),
        "J": (1, 10),
        "I": (1, 14),
    }
    if channel:
        ranges.pop("R")
        ranges["F"] = (1, 14)
    if usage:
        ranges.pop("S")
        ranges.pop("I")
    if code == "G" and number in (38, 39) and not usage:
        return True
    return code in ranges and ranges[code][0] <= number <= ranges[code][1]


def validate_value(setting, field, value):
    if not value or any(ord(c) < 32 or ord(c) > 126 or c in ';\\"' for c in value):
        raise ValueError(f"{field['name']}: invalid text")
    if not field.get("writable", True):
        raise ValueError(f"{field['name']}: preserved field, outside the write interface")
    enum = field.get("enum")
    if enum is not None and value not in enum:
        if not (
            field.get("sbas") and re.fullmatch(r"S\d{3}", value) and 120 <= int(value[1:]) <= 158
        ):
            raise ValueError(f"{field['name']}: choose {', '.join(enum)}")
    if "pattern" in field and not re.fullmatch(field["pattern"], value):
        raise ValueError(f"{field['name']}: invalid selector")
    if "max_length" in field and len(value) > field["max_length"]:
        raise ValueError(f"{field['name']}: maximum {field['max_length']} characters")
    if "min" in field:
        try:
            number = Decimal(value)
        except InvalidOperation as error:
            raise ValueError(f"{field['name']}: expected a number") from error
        if not number.is_finite() or not Decimal(str(field["min"])) <= number <= Decimal(
            str(field["max"])
        ):
            raise ValueError(
                f"{field['name']}: range {field['min']}..{field['max']} {field['unit']}"
            )
        if field.get("integer") and number != number.to_integral_value():
            raise ValueError(f"{field['name']}: expected an integer")
        # No exponents on the wire: preserve the receiver's documented decimal units.
        value = format(number, "f")
    if "items" in field or field.get("satellite_list"):
        relative = value.startswith(("+", "-"))
        if relative and not field.get("list_edits"):
            raise ValueError(f"{field['name']}: relative edits are not allowed")
        if value.startswith("-") and field.get("list_remove") is False:
            raise ValueError(
                f"{field['name']}: this receiver rejects relative removal; "
                "read the current list and supply the complete replacement"
            )
        items = (value[1:] if relative else value).split("+")
        if len(set(items)) != len(items):
            raise ValueError(f"{field['name']}: duplicate items")
        if len(items) > field.get("max_items", 128):
            raise ValueError(f"{field['name']}: too many items")
        if ({"all", "none"} & set(items)) and len(items) != 1:
            raise ValueError("all/none cannot be combined with other items")
        for item in items:
            if "items" in field and item not in field["items"]:
                raise ValueError(f"{field['name']}: unsupported item {item}")
            aliases = {"all", "none", "GPS", "GLONASS", "GALILEO", "BEIDOU", "QZSS"}
            if setting == "SatelliteTracking":
                aliases |= {"SBAS", "NAVIC"}
            if (
                field.get("satellite_list")
                and item not in aliases
                and not satellite(item, usage=setting == "SatelliteUsage")
            ):
                raise ValueError(f"{field['name']}: invalid satellite {item}")
    if (
        field.get("satellite")
        and not (field.get("auto") and value == "auto")
        and not satellite(value, channel=setting == "ChannelAllocation")
    ):
        raise ValueError(f"{field['name']}: invalid satellite")
    return value


def usb_set(name: str, assignments: list[str]) -> str:
    try:
        setting = CATALOG["settings"][name]
    except KeyError as error:
        raise ValueError(f"{name}: no write control in the audit's R/D scope") from error
    fields = {field["name"]: field for field in setting["fields"]}
    values = {}
    for assignment in assignments:
        key, separator, value = assignment.partition("=")
        if not separator or key not in fields or key in values:
            raise ValueError(f"Unknown or duplicate field: {key}")
        values[key] = validate_value(name, fields[key], value)
    if not any(not fields[key].get("selector") for key in values):
        raise ValueError("Supply at least one setting value")
    for field in setting["fields"]:
        if field.get("selector") and field["name"] not in values:
            raise ValueError(f"Supply selector {field['name']}")
    if name == "SignalTracking" and not values["Signal"].startswith(("+", "-")):
        selected = set(values["Signal"].split("+"))
        prefixes = {
            "GPS": "GPS",
            "GLONASS": "GLO",
            "GALILEO": "GAL",
            "SBAS": "GEO",
            "BEIDOU": "BDS",
            "QZSS": "QZS",
            "NAVIC": "NAV",
        }
        if "all" in selected:
            selected = set(CATALOG["signals"])
        for alias, prefix in prefixes.items():
            if alias in selected:
                selected.update(s for s in CATALOG["signals"] if s.startswith(prefix))
        for child, parents in {
            "GPSL2PY": {"GPSL1CA"},
            "GLOL2P": {"GLOL2CA"},
            "GLOL3": {"GLOL1CA"},
            "QZSL6": {"QZSL1CA", "QZSL1CB"},
        }.items():
            if child in selected and not selected.intersection(parents):
                raise ValueError(f"{child} requires {' or '.join(sorted(parents))}")
    if name in ("SBFOutput", "NMEAOutput"):
        if not {"Stream", "Cd", "Messages", "Interval"} <= values.keys():
            raise ValueError("Stream, Cd, Messages and Interval must be explicit")
    native = ["set" + name] + [values.get(field["name"], "") for field in setting["fields"]]
    while native[-1] == "":
        native.pop()
    # Native G5 strings containing whitespace need quotes, even when CSV would
    # leave them bare. Quotes/newlines in user values were rejected above.
    encoded = ",".join(
        '"' + value + '"' if " " in value or "," in value else value for value in native
    )
    return checked_usb("gnss write " + encoded)


def checked_usb(command):
    if len(command.encode("ascii")) > MAX_USB or "\r" in command or "\n" in command:
        raise ValueError("Command exceeds the bounded USB line or contains a line break")
    if command.startswith(("gnss read ", "gnss exec ")) and len(command[10:]) > 181:
        raise ValueError("Native command exceeds 181 characters")
    return command


def sanitize(text):
    # Current/Boot export may contain provisioning commands. Keep these out of
    # shareable records, including abbreviated native access-level setters.
    return "\n".join(
        (
            "[provisioning entry redacted]"
            if re.search(r"(?i)\b(?:setUserAccessLevel|sual|password|privatekey)\b", line)
            else line
        )
        for line in text.splitlines()
    )


def require_installed_catalog(name, identity, mib):
    if not re.search(r"\b1\.1\.0\b", identity) or not re.search(r"G5", identity, re.I):
        raise ValueError(
            "Receiver identity does not match the G5 v1.1.0 catalog; use read-only discovery"
        )
    setting = CATALOG["settings"][name]
    family = re.search(r"(?m)^\s*--\s*" + re.escape(name) + r"\s+ARGUMENTS\s*$", mib, re.I)
    if family is None:
        raise ValueError(f"Installed MIB lacks the {name} ARGUMENTS family")
    remainder = mib[family.end() :]
    end = re.search(r"(?m)^\s*--\s*=+\s*$", remainder)
    argument_block = remainder[: end.start()] if end else remainder
    declarations = list(
        re.finditer(r"(?m)^\s*([A-Za-z][A-Za-z0-9]*)\s+OBJECT-TYPE\s*$", argument_block)
    )
    definitions = {}
    for index, declaration in enumerate(declarations):
        end = (
            declarations[index + 1].start()
            if index + 1 < len(declarations)
            else len(argument_block)
        )
        body = argument_block[declaration.end() : end]
        if re.search(r"(?m)^\s*SYNTAX\s+", body) and re.search(r"(?m)^\s*::=\s*\{", body):
            definitions[declaration.group(1).casefold()] = body
    for field in setting["fields"]:
        symbol = field.get("mib_name")
        if not symbol:
            raise ValueError(
                f"Catalog has no verified installed MIB field for {name}.{field['name']}"
            )
        if symbol.casefold() not in definitions:
            raise ValueError(
                f"Installed MIB lacks {symbol} OBJECT-TYPE in {name} arguments; "
                "update/review the host catalog before writing"
            )
    return definitions


def validate_installed_enum(name, command, definitions):
    """Reject simple scalar values absent from an installed MIB enum."""
    if name != "SignalAuthentication":
        return
    native = command.removeprefix("gnss write ")
    try:
        values = next(csv.reader([native], skipinitialspace=True))
    except (csv.Error, StopIteration):
        return
    setting = CATALOG["settings"][name]
    if not values or values[0].casefold() != ("set" + name).casefold():
        return
    for field, value in zip(setting["fields"], values[1:]):
        symbol = field.get("mib_name")
        if not symbol or not value:
            continue
        definition = definitions.get(symbol.casefold())
        enum = (
            re.search(r"(?is)\bSYNTAX\s+[A-Za-z0-9]+\s*\{(.*?)\}", definition)
            if definition
            else None
        )
        if enum:
            allowed = {
                item.casefold()
                for item in re.findall(r"\b([A-Za-z][A-Za-z0-9_]*)\s*\(", enum.group(1))
            }
            if value.casefold() not in allowed:
                choices = ", ".join(sorted(allowed))
                raise ValueError(
                    f"Installed MIB rejects {name}.{field['name']}={value}; allowed: {choices}"
                )


def fields(line):
    return dict(re.findall(r"(?:^| )([A-Za-z_]+)=([^ ]*)", line))


def exchange(connection, command, timeout=70):
    """One USB command, preserving chunk offsets and explicit incomplete states."""
    native = (
        command.split(" ", 2)[2]
        if command.startswith(("gnss read ", "gnss write ", "gnss exec "))
        else None
    )
    expected_hex = native.encode("ascii").hex() if native is not None else None
    usb_hex = command.encode("ascii").hex()
    connection.write((command + "\r\n").encode("ascii"))
    deadline = time.monotonic() + timeout
    chunks, events, done = [], [], None
    pending = bytearray()
    while time.monotonic() < deadline:
        pending.extend(connection.read(max(1, min(connection.in_waiting, 4096))))
        if len(pending) > 8192:
            raise RuntimeError("Overlong USB telemetry line")
        while b"\n" in pending:
            raw, _, pending = pending.partition(b"\n")
            line = raw.decode("ascii", errors="replace").strip()
            data = fields(line)
            if data.get("type") == "gnss_ascii":
                chunks.append((int(data["offset"]), bytes.fromhex(data.get("hex", ""))))
            elif data.get("type") in ("gnss_control", "hybrid_command", "system_reboot"):
                events.append(data)
                if (
                    data.get("type") == "hybrid_command"
                    and data.get("accepted") == "0"
                    and data.get("command_hex", "").lower() == usb_hex
                ):
                    raise RuntimeError("Firmware rejected USB command")
                if data.get("type") == "gnss_control":
                    if data.get("stage") == "denied":
                        raise RuntimeError(
                            "Firmware busy; inspect the active operation before retrying"
                        )
                    if data.get("stage") == "done":
                        if data.get("kind") == "0" and data.get("status") in ("invalid", "busy"):
                            raise RuntimeError("Firmware rejected command: " + data["status"])
                        # Replies from a previous operation can still be queued
                        # when this port opens. Only this native request can finish it.
                        if (
                            expected_hex is not None
                            and data.get("command_hex", "").lower() == expected_hex
                        ) or (
                            command in ("gnss defaults", "system defaults")
                            and data.get("status") in ("defaults_saved", "defaults_failed")
                        ):
                            done = data
                if (
                    command.startswith("system reboot ")
                    and data.get("type") == "system_reboot"
                    and data.get("stage") == "accepted"
                    and data.get("target") == command.rsplit(" ", 1)[1]
                ):
                    return {
                        "command": command,
                        "events": events,
                        "receiver_text": "",
                        "physical_reboot_verified": False,
                    }
        if done:
            operation_events = [
                event
                for event in events
                if event.get("type") == "gnss_control" and event.get("seq") == done.get("seq")
            ]
            end = int(done["ascii_end"])
            # Completion can be queued ahead of the last ASCII chunks.
            tail = max((offset + len(block) for offset, block in chunks), default=0)
            if tail < end:
                continue
            successful = done["status"] in (
                "complete",
                "saved",
                "unchanged",
                "acknowledged",
                "defaults_saved",
            )
            if any(done.get(key, "0") != "0" for key in ("drops", "rx_loss", "truncated")):
                successful = False
            before = next((e for e in operation_events if e.get("stage") == "before"), None)
            if native is not None and before is None:
                successful = False
            start = int(before["ascii_end"]) if before is not None else end
            selected = []
            cursor = start
            for offset, block in chunks:
                if offset + len(block) <= start or offset >= end:
                    continue
                begin = max(start, offset)
                finish = min(end, offset + len(block))
                if begin != cursor:
                    successful = False
                selected.append(block[begin - offset : finish - offset])
                cursor = finish
            if start != end and cursor != end:
                successful = False
            result = {
                "command": command,
                "events": events,
                "receiver_text": sanitize(b"".join(selected).decode("ascii", errors="replace")),
                "complete": successful,
            }
            if not successful:
                raise RuntimeError(json.dumps(result, indent=2))
            return result
    raise TimeoutError(f"No complete firmware result for {command}")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="Exclusive SimpleClock CDC port; omit to print the command")
    parser.add_argument("--log", type=Path, help="Write sanitized operation evidence as JSON")
    sub = parser.add_subparsers(dest="operation", required=True)
    catalog = sub.add_parser("catalog")
    catalog.add_argument("name", nargs="?")
    get = sub.add_parser("get")
    get.add_argument("name")
    get.add_argument("selectors", nargs="*")
    write = sub.add_parser("set")
    write.add_argument("name")
    write.add_argument("assignments", nargs="+")
    read = sub.add_parser("query")
    read.add_argument("native", help="Read-only get/lst command; use full native names")
    sub.add_parser("reset-pvt")
    defaults = sub.add_parser("defaults")
    defaults.add_argument("--receiver-only", action="store_true")
    reboot = sub.add_parser("reboot")
    reboot.add_argument("target", choices=["mcu", "gnss", "all"])
    diagnostics = sub.add_parser("diagnostics")
    diagnostics.add_argument("profile", choices=["geometry", "rf", "auth", "off"])
    args = parser.parse_args(argv)
    if args.operation == "catalog":
        print(
            json.dumps(
                CATALOG["settings"].get(args.name, CATALOG) if args.name else CATALOG, indent=2
            )
        )
        return
    try:
        if args.operation == "set":
            command = usb_set(args.name, args.assignments)
        elif args.operation == "get":
            if not re.fullmatch(r"[A-Za-z][A-Za-z0-9]*", args.name):
                raise ValueError("Use a full setting name")
            command = checked_usb(
                "gnss read get"
                + args.name
                + ("," + ",".join(args.selectors) if args.selectors else "")
            )
        elif args.operation == "query":
            command = checked_usb("gnss read " + args.native)
        elif args.operation == "reset-pvt":
            command = "gnss exec exeResetNavFilter,PVT"
        elif args.operation == "defaults":
            command = "gnss defaults" if args.receiver_only else "system defaults"
        elif args.operation == "reboot":
            command = "system reboot " + args.target
        else:
            blocks = {
                "geometry": "DOP+ChannelStatus+SatVisibility",
                "rf": "RFStatus+QualityInd",
                "auth": "GALAuthStatus+ReceiverStatus",
                "off": "none",
            }[args.profile]
            command = usb_set(
                "SBFOutput",
                [
                    "Stream=Stream3",
                    "Cd=COM1",
                    "Messages=" + blocks,
                    "Interval=" + ("off" if blocks == "none" else "sec5"),
                ],
            )
    except ValueError as error:
        parser.error(str(error))
    if not args.port:
        print(command)
        return
    import serial  # Optional dependency only for explicitly requested hardware I/O.

    results = []
    with serial.Serial(args.port, 115200, timeout=0.1, write_timeout=2) as connection:
        # A model/permissions-specific catalog key accompanies every live edit.
        if command.startswith("gnss write "):
            for query in (
                "lstInternalFile,Identification",
                "getReceiverCapabilities",
                "lstInternalFile,Permissions",
                "lstMIBDescription," + command[11:].split(",")[0],
            ):
                results.append(exchange(connection, "gnss read " + query))
            name = command[14:].split(",")[0]
            argument_block = require_installed_catalog(
                name, results[0]["receiver_text"], results[-1]["receiver_text"]
            )
            validate_installed_enum(name, command, argument_block)
        results.append(exchange(connection, command))
    result = {
        "catalog_version": CATALOG["version"],
        "catalog_sha256": hashlib.sha256(CATALOG_PATH.read_bytes()).hexdigest(),
        "port": args.port,
        "operations": results,
    }
    if args.log:
        args.log.parent.mkdir(parents=True, exist_ok=True)
        args.log.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
