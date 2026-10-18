"""
rtcm.py

RTCM3 plumbing for RTK: pull corrections from the base station's TCP stream
(STRSVR / str2str) and inject them into the flight controller as MAVLink
GPS_RTCM_DATA.

Pure Python, no ROS imports, so pixhawk_bridge and gps_diagnose can share it.
Only ONE process may own the flight controller's serial port, which is why
the injection runs on the caller's existing mavutil connection rather than
in a separate node.
"""

import socket
import threading
import time
from collections import Counter


# ------------------------------------------------------------ RTCM3 framing
def _crc24q_table():
    t = []
    for i in range(256):
        c = i << 16
        for _ in range(8):
            c <<= 1
            if c & 0x1000000:
                c ^= 0x1864CFB
        t.append(c & 0xFFFFFF)
    return t


_CRC_T = _crc24q_table()


def crc24q(data):
    c = 0
    for b in data:
        c = ((c << 8) & 0xFFFFFF) ^ _CRC_T[(c >> 16) ^ b]
    return c


class RTCM3Framer:
    """Feed arbitrary bytes, get back complete, CRC-valid RTCM3 frames."""

    def __init__(self):
        self.buf = bytearray()
        self.bad_crc = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(0xD3)
            if i < 0:
                self.buf.clear()
                break
            if i:
                del self.buf[:i]          # drop non-RTCM bytes (UBX/NMEA)
            if len(self.buf) < 3:
                break
            if self.buf[1] & 0xFC:        # 6 reserved bits must be zero
                del self.buf[0]
                continue
            total = 6 + (((self.buf[1] & 0x03) << 8) | self.buf[2])
            if len(self.buf) < total:
                break
            frame = bytes(self.buf[:total])
            if crc24q(frame[:-3]) == int.from_bytes(frame[-3:], 'big'):
                out.append(frame)
                del self.buf[:total]
            else:
                self.bad_crc += 1
                del self.buf[0]           # false sync, resync on next 0xD3
        return out


def msg_type(frame):
    return (frame[3] << 4) | (frame[4] >> 4) if len(frame) > 5 else -1


# ------------------------------------------ MAVLink GPS_RTCM_DATA fragmenting
def fragment(frame, seq, max_len=180):
    """Split one RTCM frame into (flags, len, data) tuples for GPS_RTCM_DATA.

    Frames that fit in one message are sent whole.  Larger ones are split
    into up to 4 fragments (flags bit0 = fragmented, bits1-2 = fragment id,
    bits3-7 = sequence).  If the last fragment is full-size a zero-length
    one is appended so the receiver can tell the frame has ended.
    Returns [] for frames too big for the protocol.
    """
    if len(frame) > 4 * max_len:
        return []
    seq_bits = (seq & 0x1F) << 3
    if len(frame) <= max_len:
        return [(seq_bits, len(frame), frame.ljust(max_len, b'\0'))]
    chunks = [frame[i:i + max_len] for i in range(0, len(frame), max_len)]
    if len(chunks) < 4 and len(chunks[-1]) == max_len:
        chunks.append(b'')
    return [(1 | (fid << 1) | seq_bits, len(ch), ch.ljust(max_len, b'\0'))
            for fid, ch in enumerate(chunks)]


class RtcmSender:
    """Injects RTCM frames into a mavutil connection as GPS_RTCM_DATA.

    `lock` should be the same lock the caller uses for every other write on
    this connection, so fragments of one frame are never interleaved with
    other traffic from another thread.
    """
    MAX = 180

    def __init__(self, master, lock=None):
        self.master = master
        self.seq = 0
        self.lock = lock or threading.Lock()

    def send(self, frame):
        parts = fragment(frame, self.seq, self.MAX)
        if not parts:
            return False
        with self.lock:
            for flags, n, data in parts:
                self.master.mav.gps_rtcm_data_send(flags, n, list(data))
            self.seq += 1
        return True


# ------------------------------------------------------- base-station client
class RtcmTcpClient:
    """Daemon thread: connect to the base's RTCM TCP server, reconnect forever.

    on_frame(frame) is called for each CRC-valid RTCM3 frame.  Counters are
    read by the caller for its health log; drain_counts() returns and resets
    the per-message-type tally.
    """

    def __init__(self, host, port, on_frame, log=print):
        self.host = host
        self.port = int(port)
        self.on_frame = on_frame
        self.log = log
        self.connected = False
        self.last_rx = 0.0
        self.total_frames = 0
        self._counts = Counter()
        self._count_lock = threading.Lock()
        self._framer = RTCM3Framer()
        self._stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True

    @property
    def bad_crc(self):
        return self._framer.bad_crc

    def data_age(self):
        return time.time() - self.last_rx if self.last_rx else float('inf')

    def drain_counts(self):
        with self._count_lock:
            counts, self._counts = self._counts, Counter()
        return counts

    def _run(self):
        while not self._stop:
            try:
                with socket.create_connection((self.host, self.port),
                                              timeout=5) as s:
                    s.settimeout(5)
                    self.connected = True
                    self._framer = RTCM3Framer()   # drop any half frame
                    self.log(f'connected to RTCM base stream '
                             f'{self.host}:{self.port}')
                    while not self._stop:
                        data = s.recv(4096)
                        if not data:
                            raise ConnectionError('base closed the connection')
                        self.last_rx = time.time()
                        for f in self._framer.feed(data):
                            self.on_frame(f)
                            self.total_frames += 1
                            with self._count_lock:
                                self._counts[msg_type(f)] += 1
            except Exception as e:
                self.log(f'RTCM base stream: {e}; retrying in 2 s')
            self.connected = False
            if not self._stop:
                time.sleep(2)
