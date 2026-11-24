"""Streaming voice-activity detection + pitch (f0) tracking from temple accelerometer data.

Method (per hop of ~28 ms, on a ~225 ms Hann window):
  1. power spectrum of each accel axis, summed (orientation independent)
  2. per-bin noise floor = median of recent clearly-unvoiced frames (adaptive, ~5 s). Frames during
     voicing, for a short hangover after it, or with an onset-level peak are kept out, so the floor
     doesn't learn the voice and erode detection of long utterances.
  3. onset: a frame starts voicing if, in the f0 search band, the strongest bin is
       snr_db above its floor          (something tonal is there)   and
       peakiness_db above the band's median SNR (it's a line, not a broadband tap/bump)   and
       line_db above its neighbouring bins (+-~50 Hz) with a half-power width <= max_width_bins
       (voice is a pure, narrow line; chewing / nodding give broader smeared bumps)       and
       no sample-to-sample jump above transient_ratio x the typical noise jump (taps, cable knocks;
       bone-conducted voice is far too small to produce them)
     sustain: once voiced, only the bins near the current f0 are checked, against the lower
     sustain_snr_db / sustain_peakiness_db (voicing weakens after the attack)
  4. hysteresis: `onset` candidate frames in a row start voicing, `release` misses end it
  5. f0 = peak bin (kept within +-35% of the previous f0 while voiced, to avoid octave slips),
     refined by parabolic interpolation

Offline:  uv run python -m timu.voice recordings/<file>.timu
Live:     the hub runs this and pushes results to the web page / websocket clients.
"""
from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import protocol as P


@dataclass
class VoiceFrame:
    sample: int          # device sample index at the centre of the analysis window
    voiced: bool
    f0_hz: float         # nan when unvoiced
    snr_db: float        # strongest in-band bin over its noise floor
    peakiness_db: float  # that bin over the band's median SNR


class VoiceDetector:
    def __init__(self, fs: float = 1125.0, nfft: int = 256, hop: int = 32,
                 f0_range: tuple[float, float] = (70.0, 350.0), snr_db: float = 9.0,
                 peakiness_db: float = 7.0, sustain_snr_db: float = 5.0, sustain_peakiness_db: float = 4.0,
                 line_db: float = 6.0, max_width_bins: int = 2,
                 transient_ratio: float = 2.5, onset: int = 2, release: int = 3,
                 floor_seconds: float = 5.0, floor_hangover: int = 8):
        self.nominal_fs = fs
        self.nfft, self.hop = nfft, hop
        self.f0_range, self.snr_th, self.peak_th = f0_range, snr_db, peakiness_db
        self.sustain_snr_th, self.sustain_peak_th = sustain_snr_db, sustain_peakiness_db
        self.onset, self.release = onset, release
        self.floor_hangover = floor_hangover
        self.line_th, self.max_width = line_db, max_width_bins
        self._since_voiced = floor_hangover + 1
        self.transient_ratio = transient_ratio
        self.window = np.hanning(nfft)[:, None]
        self.floor_frames: deque[np.ndarray] = deque(maxlen=max(10, int(floor_seconds * fs / hop)))
        self.floor_jumps: deque[float] = deque(maxlen=self.floor_frames.maxlen)
        self._floor: np.ndarray | None = None
        self._floor_dirty = True
        self._reset_stream()
        self.voiced = False
        self._last_f0 = float("nan")
        self._run = 0  # consecutive candidate (if unvoiced) / non-candidate (if voiced) frames
        self._rate_anchor: tuple[int, int] | None = None  # (first_sample, t_us) of current run
        self.rate_hz = fs

    def _reset_stream(self):
        self._buf = np.empty((0, 3))
        self._buf_first = 0  # sample index of _buf[0]
        self._expected: int | None = None

    # -- input ---------------------------------------------------------------

    def push_packet(self, pkt: P.Packet) -> list[VoiceFrame]:
        """Feed packets in arrival order. Gaps, device resets and segment breaks restart the window."""
        key = (pkt.boot_id, pkt.segment)
        if self._expected is None or key != self._key or pkt.first_sample != self._expected:
            if self._expected is not None and pkt.first_sample < self._expected and key == self._key:
                return []  # duplicate / late packet
            self._reset_stream()
            self._buf_first = pkt.first_sample
            self._rate_anchor = None if key != getattr(self, "_key", None) else self._rate_anchor
        self._key = key
        self._expected = pkt.first_sample + pkt.n
        self._update_rate(pkt)
        return self.push(pkt.accel_g())

    def _update_rate(self, pkt: P.Packet):
        """True sample rate from device sample count vs device clock (the nominal ODR is ~1% off)."""
        last = (pkt.first_sample + pkt.n, pkt.t_us)
        if self._rate_anchor is None:
            self._rate_anchor = last
            return
        ds, dt = last[0] - self._rate_anchor[0], (last[1] - self._rate_anchor[1]) / 1e6
        if dt > 2.0:
            self.rate_hz = ds / dt

    def push(self, accel_g: np.ndarray) -> list[VoiceFrame]:
        """Feed contiguous samples (n, 3) in g."""
        self._buf = np.concatenate([self._buf, accel_g[:, :3]])
        out = []
        start = 0
        while len(self._buf) - start >= self.nfft:
            out.append(self._analyze(self._buf[start:start + self.nfft], self._buf_first + start))
            start += self.hop
        self._buf = self._buf[start:]
        self._buf_first += start
        return out

    # -- analysis ------------------------------------------------------------

    def _analyze(self, x: np.ndarray, first: int) -> VoiceFrame:
        fs = self.rate_hz
        jump = float(np.abs(np.diff(x, axis=0)).max())
        x = x - x.mean(axis=0)
        p = np.sum(np.abs(np.fft.rfft(x * self.window, axis=0)) ** 2, axis=1)
        logp = 10 * np.log10(p + 1e-30)
        centre = first + self.nfft // 2
        f = np.arange(len(p)) * fs / self.nfft

        if len(self.floor_frames) < 10:  # bootstrapping the noise floor
            self.floor_frames.append(logp)
            self.floor_jumps.append(jump)
            self._floor_dirty = True
            return VoiceFrame(centre, False, float("nan"), 0.0, 0.0)
        if self._floor_dirty:
            self._floor = np.median(np.array(self.floor_frames), axis=0)
            self._floor_dirty = False

        snr = logp - self._floor
        band = (f >= self.f0_range[0]) & (f <= self.f0_range[1])
        idx = np.flatnonzero(band)
        k = idx[np.argmax(snr[idx])]
        peak = float(snr[k])
        wide_median = float(np.median(snr[(f >= 60) & (f <= 0.97 * fs / 2)]))
        transient = jump > self.transient_ratio * float(np.median(self.floor_jumps))
        onset_level = peak >= self.snr_th and peak - wide_median >= self.peak_th
        if onset_level:  # line shape: prominent over its neighbourhood and narrow
            nb = logp[max(1, k - 12):k + 13]
            half = p[k] / 2
            lo, hi = k, k
            while lo > 1 and p[lo - 1] > half:
                lo -= 1
            while hi < len(p) - 2 and p[hi + 1] > half:
                hi += 1
            onset_level = logp[k] - float(np.median(nb)) >= self.line_th and hi - lo + 1 <= self.max_width

        if self.voiced and not np.isnan(self._last_f0):
            # sustain: follow the current f0 (also prevents octave slips) with lower thresholds
            near = idx[(f[idx] >= 0.8 * self._last_f0) & (f[idx] <= 1.25 * self._last_f0)]
            if len(near):
                k = near[np.argmax(snr[near])]
            peak = float(snr[k])
            candidate = (peak >= self.sustain_snr_th and peak - wide_median >= self.sustain_peak_th
                         and not transient)
        else:
            candidate = onset_level and not transient
        peakiness = peak - wide_median

        # hysteresis
        if candidate != self.voiced:
            self._run += 1
            if self._run >= (self.onset if candidate else self.release):
                self.voiced, self._run = candidate, 0
        else:
            self._run = 0
        self._since_voiced = 0 if self.voiced else self._since_voiced + 1
        if (not self.voiced and not transient and not onset_level
                and self._since_voiced > self.floor_hangover):
            self.floor_frames.append(logp)
            self.floor_jumps.append(jump)
            self._floor_dirty = True

        f0 = float("nan")
        if self.voiced and candidate:
            a, b, c = logp[k - 1], logp[k], logp[k + 1]  # parabolic peak refinement
            d = 0.5 * (a - c) / (a - 2 * b + c) if (a - 2 * b + c) != 0 else 0.0
            f0 = (k + float(np.clip(d, -0.5, 0.5))) * fs / self.nfft
        if not self.voiced:
            self._last_f0 = float("nan")
        elif not np.isnan(f0):
            self._last_f0 = f0  # during brief dropouts keep tracking around the last good f0
        return VoiceFrame(centre, self.voiced, f0, peak, peakiness)


def segments(frames: list[VoiceFrame], fs: float) -> list[tuple[float, float, float]]:
    """(start_s, end_s, median_f0) of voiced runs; times relative to the first frame."""
    out, start, f0s = [], None, []
    t0 = frames[0].sample if frames else 0
    for fr in frames + [VoiceFrame(0, False, float("nan"), 0, 0)]:
        if fr.voiced and start is None:
            start, f0s = fr.sample, []
        if fr.voiced and not np.isnan(fr.f0_hz):
            f0s.append(fr.f0_hz)
        if not fr.voiced and start is not None:
            out.append(((start - t0) / fs, (last - t0) / fs, float(np.median(f0s)) if f0s else float("nan")))
            start = None
        last = fr.sample
    return out


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description="Run the voice detector over a recording.")
    ap.add_argument("files", nargs="+", type=Path)
    args = ap.parse_args(argv)
    from .recording import iter_records
    for path in args.files:
        det, frames = None, []
        for _, body in iter_records(path):
            pkt = P.parse(body)
            det = det or VoiceDetector(fs=pkt.rate_hz)
            frames += det.push_packet(pkt)
        segs = segments(frames, det.rate_hz)
        voiced = sum(fr.voiced for fr in frames)
        print(f"{path.name}: {len(frames)} frames, rate {det.rate_hz:.1f} Hz, "
              f"voiced {100 * voiced / max(1, len(frames)):.1f}%, {len(segs)} segments")
        for s, e, f0 in segs:
            print(f"   {s:6.2f} - {e:6.2f} s   f0 {f0:5.1f} Hz")


if __name__ == "__main__":
    main()
