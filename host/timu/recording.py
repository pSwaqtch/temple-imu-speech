"""Recording files: raw packets exactly as received, so nothing is lost or pre-processed.

File format (.timu):
    b"TIMUREC1"
    repeated: <u32 packet_len> <f64 host_unix_time> <packet bytes>

Append-only and flushed every second; a crash leaves at most a truncated last
record, which the reader ignores.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

import numpy as np

from . import protocol as P

FILE_MAGIC = b"TIMUREC1"
RECORD = struct.Struct("<Id")


class Writer:
    def __init__(self, path: Path):
        self.path = path
        self._f = open(path, "xb")
        self._f.write(FILE_MAGIC)
        self._last_flush = time.monotonic()

    def write(self, packet: bytes, host_time: float) -> None:
        self._f.write(RECORD.pack(len(packet), host_time))
        self._f.write(packet)
        if time.monotonic() - self._last_flush > 1.0:
            self._f.flush()
            self._last_flush = time.monotonic()

    def close(self) -> None:
        self._f.close()


def iter_records(path: Path) -> Iterator[tuple[float, bytes]]:
    with open(path, "rb") as f:
        if f.read(len(FILE_MAGIC)) != FILE_MAGIC:
            raise ValueError(f"{path} is not a .timu recording")
        while True:
            head = f.read(RECORD.size)
            if len(head) < RECORD.size:
                return
            n, host_time = RECORD.unpack(head)
            body = f.read(n)
            if len(body) < n:
                return
            yield host_time, body


@dataclass
class Recording:
    """Per-sample arrays are aligned; per-packet arrays describe each received packet."""
    rate_hz: float
    sample_index: np.ndarray   # (N,) int64, device sample counter (resets on reboot)
    segment: np.ndarray        # (N,) int, changes where samples were lost on the device
    boot_id: np.ndarray        # (N,) uint32
    accel_g: np.ndarray        # (N, 3)
    gyro_dps: np.ndarray | None  # (N, 3)
    pkt_host_time: np.ndarray  # (P,) unix seconds when the PC received the packet
    pkt_t_us: np.ndarray       # (P,) device time of the packet's last sample
    pkt_first_sample: np.ndarray
    pkt_n: np.ndarray


def load(path: Path) -> Recording:
    pkts, host_times = [], []
    for host_time, body in iter_records(Path(path)):
        try:
            pkts.append(P.parse(body))
            host_times.append(host_time)
        except ValueError:
            continue
    if not pkts:
        raise ValueError(f"{path} contains no valid packets")

    def per_sample(values):
        return np.concatenate([np.full(p.n, v) for p, v in zip(pkts, values)])

    has_gyro = all(p.has_gyro for p in pkts)
    return Recording(
        rate_hz=pkts[0].rate_hz,
        sample_index=np.concatenate([p.first_sample + np.arange(p.n, dtype=np.int64) for p in pkts]),
        segment=per_sample([p.segment for p in pkts]),
        boot_id=per_sample([p.boot_id for p in pkts]).astype(np.uint32),
        accel_g=np.concatenate([p.accel_g() for p in pkts]),
        gyro_dps=np.concatenate([p.gyro_dps() for p in pkts]) if has_gyro else None,
        pkt_host_time=np.array(host_times),
        pkt_t_us=np.array([p.t_us for p in pkts], dtype=np.int64),
        pkt_first_sample=np.array([p.first_sample for p in pkts], dtype=np.int64),
        pkt_n=np.array([p.n for p in pkts]),
    )
