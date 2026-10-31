"""G5-T 1.1.0 application update through the dedicated SCB1 maintenance image.

probe/selftest never enter receiver upgrade mode; probe reports the receiver
serial. upgrade requires the exact package hash and the operator-stated
--receiver-serial to match the live receiver; bytes are sent once, in
acknowledged 512 B chunks. Never use with the normal text diagnostic, a generic
terminal, or failsafe SUF.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
import time
from gnss_bridge import BridgeError, Client, ECHO, PAYLOAD_MAX, open_serial

APP_SHA256 = "5e3cdd6a96e553e1ac33684cf576692b6a653983c5cb4d29374fa616d55e48c7"
APP_SIZE = 2951207
# The hwplatform element spans lines in the raw reply; [^>]* crosses them.
SERIAL = re.compile(rb'<hwplatform\b[^>]*\bserialnr="([0-9A-Za-z]+)"')


def validate_image(path):
    """Accept only the inspected application file, excluding failsafe/wrong targets."""
    data = path.read_bytes()
    if len(data) != APP_SIZE or hashlib.sha256(data).hexdigest() != APP_SHA256:
        raise BridgeError("File is not the inspected mosaic-G5 T 1.1.0 APPLICATION SUF")
    return data


def identity(client, directory, prefix, expected_serial=None):
    """Save the raw reply; require the G5-T platform/model and, when given, serial.

    Returns (application version, receiver serial).
    """
    data = client.command("lif,Identification")
    (directory / (prefix + "-identification.txt")).write_bytes(data)
    serial = SERIAL.search(data)
    if b"GRB0060" not in data or b'model="KORE"' not in data or not serial:
        raise BridgeError("Receiver identity is not a mosaic-G5 T (GRB0060/KORE)")
    serial = serial.group(1).decode("ascii")
    if expected_serial is not None and serial != expected_serial:
        raise BridgeError(f"Receiver serial {serial} does not match --receiver-serial")
    match = re.search(rb'<firmware version="([^"]+)"', data)
    if not match:
        raise BridgeError("Missing application version in receiver identity")
    return match.group(1).decode("ascii"), serial


def backup(client, directory, prefix):
    """Save receiver settings/permissions; this is not a backup of GNSS flash."""
    # Raw replies preserve every setting, including unsupported-file responses.
    commands = {
        "capabilities": "getReceiverCapabilities",
        "permissions": "lif,Permissions",
        "current": "lstConfigFile,Current",
        "boot": "lstConfigFile,Boot",
        "user1": "lstConfigFile,User1",
        "user2": "lstConfigFile,User2",
        "permanent": "lstConfigFile,Permanent",
        "serial": "getCOMSettings,COM1",
        "pps": "getPPSParameters",
        "signal-tracking": "getSignalTracking",
        "signal-usage": "getSignalUsage",
        "pvt-mode": "getPVTMode",
        "streams": "getSBFOutput",
    }
    for name, command in commands.items():
        reply = client.command(
            command, timeout=30, allow_reject=name in ("user1", "user2", "permanent")
        )
        (directory / (prefix + "-" + name + ".txt")).write_bytes(reply)
    client.status()


def selftest(client, data, event):
    """Echo the full validated image over USB and prove no bytes reached UART."""
    begin = client.status()
    digest = hashlib.sha256()
    for offset in range(0, len(data), PAYLOAD_MAX):
        block = data[offset : offset + PAYLOAD_MAX]
        echoed = client.request(ECHO, block)
        if echoed != block:
            raise BridgeError(f"Binary self-test mismatch at {offset}")
        digest.update(echoed)
    end = client.status()
    if end["uart_tx_bytes"] != begin["uart_tx_bytes"]:
        raise BridgeError("Self-test incorrectly reached receiver UART")
    event("selftest_passed", bytes=len(data), sha256=digest.hexdigest())


def upgrade_receiver(client, data, version, serial, directory, event):
    """Install an already hash-validated SUF after identity/configuration backup.

    The exact order is safety-critical: ready marker, unchanged bytes, restart
    prompt, then queries. No exception handler here may retry a chunk or reset GNSS.
    """
    if version != "1.0.0":
        raise BridgeError(f"Expected installed 1.0.0, found {version}; no upgrade sent")
    # Explicitly stop only the two diagnostic streams and mute PPS.
    for command in (
        "setSBFOutput,Stream1,COM1,none",
        "setSBFOutput,Stream2,COM1,none",
        "setPPSParameters,off",
    ):
        client.command(command)
    client.status()
    start = len(client.received)
    event("enter_upgrade", command="exeResetReceiver, Upgrade, none")
    client.transmit(b"exeResetReceiver, Upgrade, none\r")
    client.wait_for(b"Ready for SUF download", start, 60)
    event("receiver_ready")
    # The manual's 200 s is the start/interruption deadline;
    # the 115200 8N1 wire time for the full file is 256.181 s.
    began_transfer = time.monotonic()
    previous_report = began_transfer
    transfer_start = len(client.received)
    for offset in range(0, len(data), PAYLOAD_MAX):
        block = data[offset : offset + PAYLOAD_MAX]
        client.transmit(block)
        now = time.monotonic()
        if now - previous_report >= 10:
            event("transfer", bytes=offset + len(block), total=len(data))
            client.status()
            previous_report = now
    client.status()
    event(
        "transfer_complete",
        bytes=len(data),
        duration_s=time.monotonic() - began_transfer,
    )
    # Wait for the receiver's own restart/prompt. Sending even
    # a query before this could corrupt an incomplete SUF.
    client.wait_for(b"COM1>", transfer_start, 240)
    event("receiver_prompt_after_transfer")
    new_version, _ = identity(client, directory, "after", serial)
    if new_version != "1.1.0":
        raise BridgeError(f"Upgrade not verified: receiver reports {new_version}")
    backup(client, directory, "after")
    caps = (directory / "after-capabilities.txt").read_bytes()
    event(
        "upgrade_verified",
        version=new_version,
        native_has_capability=b"PPPGalileoHAS-SIS" in caps,
    )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=("probe", "selftest", "upgrade"))
    p.add_argument("--port", required=True, help="bridge serial port (COM5, /dev/ttyACM0, ...)")
    p.add_argument("--image", type=Path)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument(
        "--receiver-serial",
        help="serial reported by probe; upgrade refuses any other receiver",
    )
    args = p.parse_args()
    if args.mode != "probe" and args.image is None:
        p.error(f"--image is required for {args.mode}")
    if args.mode == "upgrade" and not args.receiver_serial:
        p.error("--receiver-serial is required for upgrade (run probe to read it)")
    data = validate_image(args.image) if args.mode != "probe" else None
    args.output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    with (args.output / "events.jsonl").open("w", encoding="utf-8") as events:

        def event(kind, **fields):
            record = dict(type=kind, elapsed_s=round(time.monotonic() - started, 3), **fields)
            line = json.dumps(record)
            events.write(line + "\n")
            events.flush()
            print(line, flush=True)

        event(
            "start",
            mode=args.mode,
            port=args.port,
            image=str(args.image),
            sha256=hashlib.sha256(data).hexdigest() if data else None,
        )
        serial = None
        try:
            serial = open_serial(args.port)
            with (args.output / "receiver.bin").open("wb") as raw:
                client = Client(serial, raw, event)
                client.status(hello=True)
                if args.mode == "selftest":
                    selftest(client, data, event)
                else:
                    version, receiver = identity(
                        client, args.output, "before", args.receiver_serial
                    )
                    event("receiver_identity", version=version, serial=receiver)
                    backup(client, args.output, "before")
                    if args.mode == "upgrade":
                        upgrade_receiver(client, data, version, receiver, args.output, event)
                client.collect(0.5)
                client.status()
                event("complete")
        except Exception as error:
            event(
                "FAILED",
                error=str(error),
                action="Keep board powered; inspect receiver.bin. No automatic retries or reset.",
            )
            return 1
        finally:
            if serial is not None:
                serial.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
