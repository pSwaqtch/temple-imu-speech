"""UDP wire format. Mirrors firmware/main/protocol.h -- keep both in sync."""
from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

MAGIC = 0x554D4954  # b"TIMU"
VERSION = 1
UDP_PORT = 5555
CMD_SUBSCRIBE = b"SUB1"
CMD_UNSUBSCRIBE = b"UNSB"

LAYOUT_ACCEL = 0x01
LAYOUT_GYRO = 0x02

# magic, version, layout, accel_fs, gyro_fs, n_samples, segment, seq,
# first_sample, t_us, nominal_rate_hz, boot_id, fifo_overflows, queue_drops
HEADER = struct.Struct("<IBBBBHHIQQfIII")
assert HEADER.size == 48

ACCEL_LSB_PER_G = (16384.0, 8192.0, 4096.0, 2048.0)
GYRO_LSB_PER_DPS = (131.0, 65.5, 32.8, 16.4)


@dataclass(frozen=True)
class Packet:
    layout: int
    accel_fs: int
    gyro_fs: int
    segment: int
    seq: int
    first_sample: int
    t_us: int
    rate_hz: float
    boot_id: int
    fifo_overflows: int
    queue_drops: int
    raw: np.ndarray  # (n, words) int16 counts: ax ay az [gx gy gz]

    @property
    def n(self) -> int:
        return self.raw.shape[0]

    @property
    def has_gyro(self) -> bool:
        return bool(self.layout & LAYOUT_GYRO)

    def accel_g(self) -> np.ndarray:
        return self.raw[:, 0:3] / ACCEL_LSB_PER_G[self.accel_fs]

    def gyro_dps(self) -> np.ndarray | None:
        return self.raw[:, 3:6] / GYRO_LSB_PER_DPS[self.gyro_fs] if self.has_gyro else None


def words_per_frame(layout: int) -> int:
    return 3 * bool(layout & LAYOUT_ACCEL) + 3 * bool(layout & LAYOUT_GYRO)


def parse(buf: bytes) -> Packet:
    """Raises ValueError on anything malformed."""
    if len(buf) < HEADER.size:
        raise ValueError(f"short packet ({len(buf)} B)")
    (magic, version, layout, accel_fs, gyro_fs, n, segment, seq, first_sample, t_us,
     rate_hz, boot_id, overflows, queue_drops) = HEADER.unpack_from(buf)
    if magic != MAGIC:
        raise ValueError(f"bad magic {magic:#x}")
    if version != VERSION:
        raise ValueError(f"protocol version {version}, expected {VERSION}")
    if not layout & LAYOUT_ACCEL or accel_fs > 3 or gyro_fs > 3:
        raise ValueError("bad sensor config fields")
    words = words_per_frame(layout)
    if len(buf) != HEADER.size + n * words * 2:
        raise ValueError(f"length {len(buf)} does not match n_samples={n}")
    raw = np.frombuffer(buf, dtype="<i2", offset=HEADER.size, count=n * words).reshape(n, words)
    return Packet(layout, accel_fs, gyro_fs, segment, seq, first_sample, t_us, rate_hz,
                  boot_id, overflows, queue_drops, raw)
