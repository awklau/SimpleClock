"""Record through a fake port and check the recording format."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest

TOOLS = Path(__file__).resolve().parents[1] / "tools"
spec = importlib.util.spec_from_file_location("usb_capture", TOOLS / "usb-capture.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)

# Current-firmware record formats (clock_guard v2, hybrid_config v4).
LINES = (
    b"type=clock_guard v=2 uptime_ms=1000 clipped=0 clipped_samples=3 innovation_ps=12 "
    b"integral_limited=0 lock_failures=0 thermal_warning=0 reanchors=1 reanchor_total_ps=-1470 "
    b"bridged_gaps=2 bridged_epochs=3 gnss_usable=1 event_drops=0\r\n",
    b"type=hybrid_config v=4 phase_mode=RELATIVE_FIXED pps_policy=CONTINUOUS uptime_ms=1001 "
    b"bias_clip_ps=2000 static_fll_time_s=10 static_phase_time_s=100\r\n",
    b"type=hybrid_command v=1 uptime_ms=1002 accepted=1 command_hex=68796272696420636F6E666967\r\n",
    b"type=clock_guard v=2 uptime_ms=4000 bridged_gaps=2 bridged_epochs=3\r\n",
)


class FakePort:
    """Deliver the lines split across reads; record writes."""

    def __init__(self, stop_file=None):
        self.stop_file = stop_file  # Created once every chunk has been delivered.
        data = b"".join(LINES)
        self.chunks = [data[i : i + 37] for i in range(0, len(data), 37)]
        self.written = []
        self.closed = False

    def read(self, size):
        chunk = self.chunks.pop(0) if self.chunks else b""
        if not self.chunks and self.stop_file is not None:
            self.stop_file.touch()
        return chunk

    def write(self, data):
        self.written.append(data)
        return len(data)

    def close(self):
        self.closed = True


class CaptureTests(unittest.TestCase):
    def test_capture_round_trips_through_analysis(self):
        with tempfile.TemporaryDirectory() as directory:
            stop = Path(directory) / "stop"
            port = FakePort(stop)  # Ends the capture when its data runs out.
            output = Path(directory) / "run"
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                code = capture.main(
                    [
                        "--port",
                        "COM9",
                        "--output",
                        str(output),
                        "--seconds",
                        "60",
                        "--stop-file",
                        str(stop),
                        "--command",
                        "hybrid config",
                    ],
                    open_port=lambda name: port,
                )
            self.assertEqual(code, 0, out.getvalue())
            self.assertEqual(port.written, [b"hybrid config\r\n"])
            self.assertTrue(port.closed)
            self.assertIn("accepted=1 command_hex=68796272696420636F6E666967", out.getvalue())
            rows = [
                json.loads(line)
                for line in (output / "status.jsonl").read_text(encoding="utf-8").splitlines()
            ]
            self.assertEqual(
                [row["type"] for row in rows],
                ["clock_guard", "hybrid_config", "hybrid_command", "clock_guard"],
            )
            for row in rows:
                self.assertEqual(set(row), {"host_arrival_utc", "host_elapsed_s", "type", "fields"})
                self.assertEqual(row["fields"]["type"], row["type"])
            self.assertEqual(
                (rows[0]["fields"]["bridged_gaps"], rows[0]["fields"]["bridged_epochs"]), ("2", "3")
            )
            self.assertEqual(
                (rows[1]["fields"]["static_fll_time_s"], rows[1]["fields"]["static_phase_time_s"]),
                ("10", "100"),
            )
            events = [
                json.loads(line)
                for line in (output / "events.jsonl").read_text(encoding="utf-8").splitlines()
            ]
            self.assertEqual(events[0]["command"], "hybrid config")
            gap = [event for event in events if event["event"] == "gap_or_reset"]
            self.assertEqual((gap[0]["previous_uptime_ms"], gap[0]["uptime_ms"]), (1000, 4000))
            manifest = json.loads((output / "run_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual((manifest["records"], manifest["failure"]), (4, None))
            self.assertEqual(
                (output / "transport.log").read_text(encoding="utf-8").splitlines()[0],
                LINES[0].decode().rstrip("\r\n"),
            )

    def test_existing_output_and_multiline_commands_are_refused(self):
        with tempfile.TemporaryDirectory() as directory:
            for extra in (
                ["--output", directory],
                ["--output", directory + "/x", "--command", "a\nb"],
            ):
                with self.subTest(extra=extra), contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit):
                        capture.main(["--port", "COM9", *extra], open_port=lambda name: FakePort())

    def test_open_failure_still_writes_manifest(self):
        def fail(name):
            raise OSError("port busy")

        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run"
            with (
                contextlib.redirect_stdout(io.StringIO()),
                contextlib.redirect_stderr(io.StringIO()),
            ):
                code = capture.main(["--port", "COM9", "--output", str(output)], open_port=fail)
            manifest = json.loads((output / "run_manifest.json").read_text(encoding="utf-8"))
            self.assertEqual((code, manifest["failure"]), (1, "port busy"))


if __name__ == "__main__":
    unittest.main()
