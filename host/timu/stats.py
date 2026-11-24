"""Link-quality and signal statistics over the packet stream."""
from __future__ import annotations

import numpy as np

from . import protocol as P

VOICE_BAND_HZ = (80.0, 500.0)


def band_rms(x: np.ndarray, fs: float, lo: float, hi: float) -> np.ndarray:
    """Per-column RMS of x restricted to [lo, hi] Hz (Parseval on a one-sided FFT)."""
    x = x - x.mean(axis=0)
    spec = np.fft.rfft(x, axis=0)
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    m = (f >= lo) & (f <= hi)
    return np.sqrt(2.0 * np.sum(np.abs(spec[m]) ** 2, axis=0)) / len(x)


class Stats:
    """Totals since creation plus a signal window that resets on every snapshot()."""

    def __init__(self):
        self.samples = self.lost = self.discontinuities = self.reboots = 0
        self.reordered = self.bad = 0
        self.boot_id = self.segment = self.expected = None
        self.last: P.Packet | None = None
        self._window: list[np.ndarray] = []

    def update(self, p: P.Packet) -> None:
        if self.boot_id is not None and p.boot_id != self.boot_id:
            self.reboots += 1
            self.expected = self.segment = None
        self.boot_id = p.boot_id
        if self.segment is not None and p.segment != self.segment:
            self.discontinuities += 1  # device lost an unknown number of samples
        self.segment = p.segment
        if self.expected is not None:
            gap = p.first_sample - self.expected
            if gap > 0:
                self.lost += gap
            elif gap < 0:
                self.reordered += 1
        self.expected = max(self.expected or 0, p.first_sample + p.n)
        self.samples += p.n
        self.last = p
        self._window.append(p.accel_g())

    def snapshot(self, dt: float) -> dict:
        s = {
            "samples": self.samples, "lost": self.lost, "discontinuities": self.discontinuities,
            "reboots": self.reboots, "bad": self.bad, "reordered": self.reordered,
            "rate_hz": None, "accel_mean_g": None, "voice_rms_mg": None,
            "fifo_overflows": self.last.fifo_overflows if self.last else None,
            "queue_drops": self.last.queue_drops if self.last else None,
        }
        if self._window:
            a = np.concatenate(self._window)
            s["rate_hz"] = len(a) / dt
            s["accel_mean_g"] = a.mean(axis=0).round(4).tolist()
            s["voice_rms_mg"] = (band_rms(a, self.last.rate_hz, *VOICE_BAND_HZ) * 1000).round(2).tolist()
        self._window = []
        return s


def format_line(s: dict) -> str:
    if s["rate_hz"] is None:
        sig = "   no data"
    else:
        m, v = s["accel_mean_g"], s["voice_rms_mg"]
        sig = (f"{s['rate_hz']:7.1f} Hz | acc [{m[0]:+.3f} {m[1]:+.3f} {m[2]:+.3f}] g  "
               f"voice-band rms [{v[0]:6.2f} {v[1]:6.2f} {v[2]:6.2f}] mg")
    return (f"{sig} | lost {s['lost']} disc {s['discontinuities']} reboot {s['reboots']} "
            f"bad {s['bad']} | ovf {s['fifo_overflows']} qdrop {s['queue_drops']}")
