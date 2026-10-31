"""Protocol failures must stop transmission, never resend firmware bytes."""

import io
from pathlib import Path
import struct
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from gnss_bridge import BridgeError, Client, Decoder, encode


class FakePort:
    def __init__(self, answer):
        self.answer = answer
        self.writes = []
        self.incoming = bytearray()

    def write(self, data):
        self.writes.append(data)
        self.incoming.extend(self.answer(data))

    def read(self):
        data = bytes(self.incoming[:17])
        del self.incoming[:17]
        return data


class BridgeTests(unittest.TestCase):
    def test_c_wire_vector(self):
        self.assertEqual(
            encode(2, 0x12345678, b"\x00\x11\x13\xff").hex(), "534342310200040078563412001113ff4d36"
        )

    def test_all_bytes_fragmented_and_concatenated(self):
        payload = bytes(range(256)) * 2
        stream = encode(4, 0xFFFFFFFF, payload) + encode(1, 0)
        for chunk in (1, 7, 64, 128, 526, 1024):
            decoder = Decoder()
            results = []
            for i in range(0, len(stream), chunk):
                results.extend(decoder.feed(stream[i : i + chunk]))
            self.assertEqual(results, [(4, 0, 0xFFFFFFFF, payload), (1, 0, 0, b"")])
            self.assertFalse(decoder.buffer)

    def test_every_single_bit_error_rejected(self):
        original = encode(4, 19, bytes(range(32)))
        for i in range(len(original)):
            for bit in range(8):
                bad = bytearray(original)
                bad[i] ^= 1 << bit
                # A damaged length can make a frame incomplete; it cannot be accepted.
                try:
                    self.assertEqual(Decoder().feed(bad), [])
                except BridgeError:
                    pass

    def test_rx_interleaved_ack_and_tc_byte_count(self):
        def answer(data):
            _, _, seq, payload = Decoder().feed(data)[0]
            return encode(144, 0, b"progress\x00\xff") + encode(
                130, seq, struct.pack("<I", 9 + len(payload))
            )

        port = FakePort(answer)
        log = io.BytesIO()
        client = Client(port, log)
        client.tx_count = 9
        client.transmit(b"\x00\xff\x11\x13")
        self.assertEqual(client.tx_count, 13)
        self.assertEqual(log.getvalue(), b"progress\x00\xff")
        self.assertEqual(len(port.writes), 1)

    def test_wrong_ack_stops_without_retry(self):
        for response in (
            encode(130, 99, struct.pack("<I", 1)),
            encode(130, 1, struct.pack("<I", 0)),
            encode(130, 1, struct.pack("<I", 1), flags=1),
        ):
            port = FakePort(lambda _, r=response: r)
            client = Client(port, io.BytesIO())
            client.tx_count = 0
            with self.assertRaises(BridgeError):
                client.transmit(b"x")
            self.assertEqual(len(port.writes), 1)

    def test_rx_gap_and_crc_rejected(self):
        for response in (encode(144, 1, b"lost"), encode(144, 0, b"crc")[:-1] + b"\xff"):
            port = FakePort(lambda _, r=response: r)
            client = Client(port, io.BytesIO())
            with self.assertRaises(BridgeError):
                client.request(4, b"x")
            self.assertEqual(len(port.writes), 1)

    def test_no_ack_never_retried(self):
        port = FakePort(lambda _: b"")
        with self.assertRaises(BridgeError):
            Client(port, io.BytesIO()).request(4, b"x", timeout=0.001)
        self.assertEqual(len(port.writes), 1)

    def test_oversize_and_incomplete(self):
        with self.assertRaises(ValueError):
            encode(2, 1, bytes(513))
        frame = encode(2, 1, bytes(512))
        decoder = Decoder()
        self.assertEqual(decoder.feed(frame[:-1]), [])
        self.assertEqual(len(decoder.feed(frame[-1:])), 1)


if __name__ == "__main__":
    unittest.main()
