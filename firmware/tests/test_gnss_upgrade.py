"""Exercise upgrade sequencing and stop-on-failure behavior without opening a COM port."""

import importlib.util
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
SERIAL = "1234567890"
sys.path.insert(0, str(TOOLS))
from gnss_bridge import BridgeError, ECHO

spec = importlib.util.spec_from_file_location("gnss_upgrade", TOOLS / "gnss-upgrade.py")
upgrade = importlib.util.module_from_spec(spec)
spec.loader.exec_module(upgrade)


class FakeReceiver:
    """Record I/O order, return canned identity, and optionally fail a data chunk."""

    def __init__(self, version="1.1.0", fail_data=False):
        self.version = version
        self.fail_data = fail_data
        self.received = bytearray()
        self.calls = []
        self.uart_count = 0

    def status(self):
        self.calls.append(("status", None))
        return {"uart_tx_bytes": self.uart_count}

    def command(self, command, **kwargs):
        self.calls.append(("command", command))
        if command == "lif,Identification":
            return (
                f'<hwplatform product="mosaic-G5 T"\r\n'
                f'            name="GRB0060" serialnr="{SERIAL}"\r\n'
                f'            rxfullid="SN2609{SERIAL}">\r\n'
                f'</hwplatform><firmware version="{self.version}"><os model="KORE"/></firmware>'
            ).encode()
        if command == "getReceiverCapabilities":
            return b"PPPGalileoHAS-SIS"
        return b"test configuration reply"

    def transmit(self, data):
        self.calls.append(("transmit", data))
        if self.fail_data and not data.startswith(b"exeResetReceiver"):
            raise BridgeError("simulated missing chunk ACK")
        self.uart_count += len(data)

    def wait_for(self, token, start, timeout):
        self.calls.append(("wait", token))
        self.received.extend(token)
        return token

    def request(self, kind, payload):
        if kind != ECHO:
            raise AssertionError("self-test may only send USB echo requests")
        return payload


class UpgradeTests(unittest.TestCase):
    def test_missing_image_is_usage_error_before_output_or_serial_io(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            for mode in ("selftest", "upgrade"):
                with self.subTest(mode=mode):
                    result = subprocess.run(
                        [
                            sys.executable,
                            str(TOOLS / "gnss-upgrade.py"),
                            mode,
                            "--port",
                            "COM9",
                            "--output",
                            str(output),
                        ],
                        capture_output=True,
                        text=True,
                    )
                    self.assertEqual(result.returncode, 2)
                    self.assertIn("--image is required", result.stderr)
                    self.assertNotIn("Traceback", result.stderr)
                    self.assertFalse(output.exists())

    def test_payload_preserved_and_queries_wait_for_restart(self):
        client = FakeReceiver()
        payload = bytes(range(256)) * 5 + b"\x00\x11\x13\xff\r\n"
        events = []
        with tempfile.TemporaryDirectory() as directory:
            upgrade.upgrade_receiver(
                client,
                payload,
                "1.0.0",
                SERIAL,
                Path(directory),
                lambda kind, **fields: events.append((kind, fields)),
            )
        writes = [value for kind, value in client.calls if kind == "transmit"]
        self.assertEqual(writes[0], b"exeResetReceiver, Upgrade, none\r")
        self.assertEqual(b"".join(writes[1:]), payload)
        self.assertTrue(all(1 <= len(block) <= 512 for block in writes[1:]))
        ready = client.calls.index(("wait", b"Ready for SUF download"))
        first_data = client.calls.index(("transmit", writes[1]))
        restart = client.calls.index(("wait", b"COM1>"))
        identify = client.calls.index(("command", "lif,Identification"))
        self.assertLess(ready, first_data)
        self.assertLess(first_data, restart)
        self.assertLess(restart, identify)
        self.assertEqual(
            events[-1],
            (
                "upgrade_verified",
                {
                    "version": "1.1.0",
                    "native_has_capability": True,
                },
            ),
        )

    def test_wrong_starting_version_performs_no_io(self):
        client = FakeReceiver()
        with self.assertRaises(BridgeError):
            upgrade.upgrade_receiver(client, b"unused", "1.1.0", SERIAL, None, lambda *a, **k: None)
        self.assertEqual(client.calls, [])

    def test_chunk_failure_never_retries_or_queries_receiver(self):
        client = FakeReceiver(fail_data=True)
        with self.assertRaises(BridgeError):
            upgrade.upgrade_receiver(
                client, b"payload", "1.0.0", SERIAL, None, lambda *a, **k: None
            )
        writes = [value for kind, value in client.calls if kind == "transmit"]
        self.assertEqual(writes, [b"exeResetReceiver, Upgrade, none\r", b"payload"])
        self.assertNotIn(("command", "lif,Identification"), client.calls)
        self.assertNotIn(("wait", b"COM1>"), client.calls)

    def test_old_version_after_restart_is_not_success(self):
        client = FakeReceiver(version="1.0.0")
        with tempfile.TemporaryDirectory() as directory, self.assertRaises(BridgeError):
            upgrade.upgrade_receiver(
                client, b"payload", "1.0.0", SERIAL, Path(directory), lambda *a, **k: None
            )

    def test_identity_reads_multiline_serial_and_refuses_another(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            self.assertEqual(upgrade.identity(FakeReceiver(), folder, "probe"), ("1.1.0", SERIAL))
            self.assertEqual(
                upgrade.identity(FakeReceiver(), folder, "check", SERIAL), ("1.1.0", SERIAL)
            )
            with self.assertRaises(BridgeError):
                upgrade.identity(FakeReceiver(), folder, "other", "0000000000")

    def test_upgrade_requires_receiver_serial_before_output_or_serial_io(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "capture"
            result = subprocess.run(
                [
                    sys.executable,
                    str(TOOLS / "gnss-upgrade.py"),
                    "upgrade",
                    "--port",
                    "COM9",
                    "--image",
                    str(Path(directory) / "app.suf"),
                    "--output",
                    str(output),
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(result.returncode, 2)
            self.assertIn("--receiver-serial is required", result.stderr)
            self.assertFalse(output.exists())

    def test_selftest_does_not_transmit_uart(self):
        client = FakeReceiver()
        upgrade.selftest(client, bytes(range(256)) * 5, lambda *a, **k: None)
        self.assertEqual(client.uart_count, 0)

    def test_wrong_image_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            image = Path(directory) / "wrong.suf"
            image.write_bytes(b"not the approved application")
            with self.assertRaises(BridgeError):
                upgrade.validate_image(image)


if __name__ == "__main__":
    unittest.main()
