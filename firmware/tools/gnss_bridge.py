"""Bounded SCB1 transport. No automatic retries or baud/reset side effects."""

from __future__ import annotations
import binascii
import ctypes as ct
import os
import struct
import time

if os.name == "nt":
    from ctypes import wintypes as wt

# Stable SCB1 wire values; mirror core/include/simpleclock/bridge.h.
# Request/reply sequences and byte counters wrap modulo 2^32. A backward
# uptime (including its 49.7-day wrap) conservatively aborts the session: the
# host cannot distinguish that event from a restart with the same session ID.
PROTOCOL_VERSION = 1
PAYLOAD_MAX = 512
UINT32_MASK = 0xFFFFFFFF
HELLO, UART_TX, STATUS, ECHO = 1, 2, 3, 4
REPLY_OFFSET = 128
UART_RX, ERROR = 144, 255
HEADER = struct.Struct("<4sBBHI")
STATUS_PAYLOAD = struct.Struct("<15I")
INFO_KEYS = (
    "protocol",
    "uptime_ms",
    "session",
    "faults",
    "uart_tx_bytes",
    "uart_rx_bytes",
    "uart_rx_dropped",
    "usb_rx_bytes",
    "usb_overflow",
    "protocol_errors",
    "uart_rx_peak",
    "usb_rx_peak",
    "loop_gap_max_ms",
    "stack_peak",
    "guard",
)


class BridgeError(RuntimeError):
    pass


def encode(kind, sequence, payload=b"", flags=0):
    if len(payload) > PAYLOAD_MAX:
        raise ValueError("Payload exceeds bridge buffer")
    body = HEADER.pack(b"SCB1", kind, flags, len(payload), sequence) + payload
    return body + struct.pack("<H", binascii.crc_hqx(body, 0))


class Decoder:
    """Accumulate fragmented USB reads; return complete CRC-checked packets.

    Corruption raises immediately. Searching ahead for another magic word would
    hide a lost frame and is deliberately not a recovery strategy during upgrades.
    """

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        packets = []
        while len(self.buffer) >= HEADER.size:
            magic, kind, flags, length, seq = HEADER.unpack_from(self.buffer)
            if magic != b"SCB1" or length > PAYLOAD_MAX:
                raise BridgeError(
                    "Invalid bridge header (wrong image, dropped bytes, or corruption)"
                )
            size = HEADER.size + length + 2
            if len(self.buffer) < size:
                break
            body = self.buffer[: size - 2]
            if binascii.crc_hqx(body, 0) != struct.unpack_from("<H", self.buffer, size - 2)[0]:
                raise BridgeError("Bridge frame CRC mismatch")
            packets.append((kind, flags, seq, bytes(self.buffer[HEADER.size : size - 2])))
            del self.buffer[:size]
        return packets


class WinSerial:
    """Windows synchronous COM I/O; requires no installed Python packages."""

    def __init__(self, port):
        kernel = self.kernel = ct.WinDLL("kernel32", use_last_error=True)
        kernel.CreateFileW.argtypes = [
            wt.LPCWSTR,
            wt.DWORD,
            wt.DWORD,
            ct.c_void_p,
            wt.DWORD,
            wt.DWORD,
            wt.HANDLE,
        ]
        kernel.CreateFileW.restype = wt.HANDLE
        for name in ("ReadFile", "WriteFile"):
            fn = getattr(kernel, name)
            fn.argtypes = [wt.HANDLE, ct.c_void_p, wt.DWORD, ct.POINTER(wt.DWORD), ct.c_void_p]
            fn.restype = wt.BOOL
        kernel.CloseHandle.argtypes = [wt.HANDLE]
        kernel.CloseHandle.restype = wt.BOOL
        kernel.BuildCommDCBW.argtypes = [wt.LPCWSTR, ct.c_void_p]
        kernel.BuildCommDCBW.restype = wt.BOOL
        kernel.SetCommState.argtypes = [wt.HANDLE, ct.c_void_p]
        kernel.SetCommState.restype = wt.BOOL
        kernel.SetCommTimeouts.argtypes = [wt.HANDLE, ct.c_void_p]
        kernel.SetCommTimeouts.restype = wt.BOOL
        self.handle = kernel.CreateFileW("\\\\.\\" + port, 0xC0000000, 0, None, 3, 0, None)
        if self.handle == ct.c_void_p(-1).value:
            raise ct.WinError(ct.get_last_error())
        try:
            # DCB is 28 bytes in both Win32 and Win64 (Windows DWORD is 32 bits).
            dcb = ct.create_string_buffer(28)
            struct.pack_into("<I", dcb, 0, 28)
            self.check(
                kernel.BuildCommDCBW(
                    "baud=115200 parity=n data=8 stop=1 octs=off " "odsr=off dtr=on rts=on xon=off",
                    dcb,
                )
            )
            # Explicit binary-preserving flags: fBinary, DTR_ENABLE, RTS_ENABLE.
            # No null stripping, error replacement, XON/XOFF, or DSR filtering.
            struct.pack_into("<I", dcb, 8, 0x1011)
            dcb[21], dcb[22] = b"\x11", b"\x13"  # SetCommState requires distinct chars.
            self.check(kernel.SetCommState(self.handle, dcb))
            timeouts = (wt.DWORD * 5)(0xFFFFFFFF, 0, 5, 0, 5000)
            self.check(kernel.SetCommTimeouts(self.handle, timeouts))
        except BaseException:
            self.close()
            raise

    @staticmethod
    def check(result):
        if not result:
            raise ct.WinError(ct.get_last_error())

    def read(self):
        data, count = ct.create_string_buffer(4096), wt.DWORD()
        self.check(self.kernel.ReadFile(self.handle, data, len(data), ct.byref(count), None))
        return data.raw[: count.value]

    def write(self, data):
        count = wt.DWORD()
        buf = ct.create_string_buffer(data)
        self.check(self.kernel.WriteFile(self.handle, buf, len(data), ct.byref(count), None))
        if count.value != len(data):
            raise BridgeError(f"Short USB write: {count.value}/{len(data)}; no retry")

    def close(self):
        if self.handle is not None:
            self.kernel.CloseHandle(self.handle)
            self.handle = None


class PortSerial:
    """pyserial transport for Linux/macOS: raw 115200 8N1, no flow control.

    Same contract as WinSerial: read returns what arrives within about 5 ms,
    write sends everything within 5 s or fails without retry.
    """

    def __init__(self, port):
        import serial  # Needed only for live hardware I/O off Windows.

        self.port = serial.Serial(
            port, 115200, timeout=0.005, write_timeout=5, xonxoff=False, rtscts=False
        )

    def read(self):
        return self.port.read(max(1, min(self.port.in_waiting, 4096)))

    def write(self, data):
        count = self.port.write(data)
        if count != len(data):
            raise BridgeError(f"Short USB write: {count}/{len(data)}; no retry")

    def close(self):
        self.port.close()


def open_serial(port):
    """Windows uses the hardware-proven ctypes path; elsewhere pyserial."""
    return WinSerial(port) if os.name == "nt" else PortSerial(port)


class Client:
    """Single-threaded stop-and-wait session over a serial-like read/write object.

    One request is outstanding. RX frames may arrive while waiting for its ACK;
    they are written to raw_log before interpretation. received holds receiver
    replies/progress on the PC, never the outgoing multi-megabyte SUF.
    """

    def __init__(self, serial, raw_log, event=None):
        self.serial, self.raw_log = serial, raw_log
        self.event = event or (lambda *args, **kwargs: None)
        self.decoder = Decoder()
        self.sequence = 1
        self.rx_sequence = 0
        self.received = bytearray()
        self.tx_count = None
        self.session = None
        self.last_uptime = None

    def read(self):
        replies = []
        for kind, flags, seq, payload in self.decoder.feed(self.serial.read()):
            if kind == UART_RX:
                if seq != self.rx_sequence:
                    raise BridgeError(
                        f"Receiver byte-stream sequence gap {self.rx_sequence}->{seq}"
                    )
                self.rx_sequence = (seq + 1) & UINT32_MASK
                self.raw_log.write(payload)
                self.raw_log.flush()
                self.received.extend(payload)
            else:
                replies.append((kind, seq, payload))
            if flags or kind == ERROR:
                detail = ""
                if len(payload) == STATUS_PAYLOAD.size and kind in (
                    HELLO + REPLY_OFFSET,
                    STATUS + REPLY_OFFSET,
                    ERROR,
                ):
                    detail = str(dict(zip(INFO_KEYS, STATUS_PAYLOAD.unpack(payload))))
                raise BridgeError(f"Bridge fault type={kind} flags={flags} {detail}")
        return replies

    def request(self, kind, payload=b"", timeout=5):
        """Send once and wait timeout seconds; consume interleaved receiver bytes."""
        seq = self.sequence
        self.sequence = (seq + 1) & UINT32_MASK
        self.serial.write(encode(kind, seq, payload))
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            replies = self.read()
            if not replies:
                continue
            if len(replies) != 1 or replies[0][:2] != (kind + REPLY_OFFSET, seq):
                raise BridgeError(f"Unexpected/duplicate bridge acknowledgment: {replies}")
            return replies[0][2]
        raise BridgeError(f"No acknowledgment for request {seq}; transfer stopped without retry")

    def status(self, hello=False):
        """Validate health/session/counts; the first HELLO establishes the baseline."""
        data = self.request(HELLO if hello else STATUS)
        if len(data) != STATUS_PAYLOAD.size:
            raise BridgeError("Wrong status length")
        info = dict(zip(INFO_KEYS, STATUS_PAYLOAD.unpack(data)))
        self.event("bridge_status", **info)
        if info["protocol"] != PROTOCOL_VERSION or info["faults"] or not info["guard"]:
            raise BridgeError(f"Unhealthy bridge: {info}")
        if self.session is not None and (
            info["session"] != self.session or info["uptime_ms"] < self.last_uptime
        ):
            raise BridgeError("Bridge restarted/session changed")
        if self.tx_count is not None and info["uart_tx_bytes"] != self.tx_count:
            raise BridgeError("UART transmit count changed without an acknowledged request")
        self.session, self.last_uptime = info["session"], info["uptime_ms"]
        self.tx_count = info["uart_tx_bytes"]
        return info

    def transmit(self, data):
        """Send one UART chunk and require the count acknowledged after hardware TC."""
        if not data or len(data) > PAYLOAD_MAX or self.tx_count is None:
            raise ValueError("Transmit needs HELLO and 1..512 bytes")
        ack = self.request(UART_TX, data)
        expected = (self.tx_count + len(data)) & UINT32_MASK
        if len(ack) != 4 or struct.unpack("<I", ack)[0] != expected:
            raise BridgeError("UART acknowledgment byte count mismatch")
        self.tx_count = expected

    def collect(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if self.read():
                raise BridgeError("Unsolicited bridge acknowledgment")

    def wait_for(self, token, start, timeout):
        """Search only bytes after start (a received-buffer offset), never old prompts."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if token in self.received[start:]:
                return bytes(self.received[start:])
            if self.read():
                raise BridgeError("Unsolicited bridge acknowledgment")
        raise BridgeError(f"Receiver did not send {token!r} within {timeout} seconds")

    def command(self, command, timeout=15, allow_reject=False):
        """Run a serialized receiver command; optional rejected queries remain evidence.

        This transport-level helper requires a response marker and final prompt.
        Identity checks and the bounded command selection belong to the upgrade CLI.
        """
        self.collect(0.15)  # Drain prior asynchronous bytes before marking this reply.
        start = len(self.received)
        self.event("receiver_command", command=command)
        self.transmit(command.encode("ascii") + b"\r")
        reply = self.wait_for(b"COM1>", start, timeout)
        if b"$R" not in reply or (not allow_reject and b"$R?" in reply):
            raise BridgeError(f"Receiver rejected or failed to acknowledge {command}: {reply!r}")
        return reply
