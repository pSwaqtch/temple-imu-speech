"""Spectrogram of a recording (x+y+z power, 50 Hz high-pass) with the detector's pitch overlaid.

Writes <recording>.png next to the .timu file unless -o is given.

    uv run --with matplotlib --with scipy python -m timu.plot recordings/<file>.timu
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfiltfilt, spectrogram

from . import protocol as P
from .recording import iter_records, load
from .voice import VoiceDetector


def plot(path: Path, out: Path, fmax: float = 560.0) -> None:
    rec = load(path)
    fs = rec.rate_hz
    a = sosfiltfilt(butter(4, 50, "hp", fs=fs, output="sos"), rec.accel_g * 1000, axis=0)
    power = 0
    for k in range(3):
        f, t, s = spectrogram(a[:, k], fs, nperseg=512, noverlap=480, window="hann")
        power = power + s
    db = 10 * np.log10(power + 1e-12)
    m = f <= fmax

    det, frames = None, []
    for _, body in iter_records(path):
        pkt = P.parse(body)
        det = det or VoiceDetector(fs=pkt.rate_hz)
        frames += det.push_packet(pkt)
    s0 = rec.sample_index[0]
    ft = np.array([(fr.sample - s0) / fs for fr in frames])
    f0 = np.array([fr.f0_hz for fr in frames])

    fig, ax = plt.subplots(figsize=(14, 4))
    ax.pcolormesh(t, f[m], db[m], vmin=np.percentile(db[m], 20), vmax=np.percentile(db[m], 99.7),
                  cmap="magma", shading="auto")
    ax.plot(ft, f0, color="#39d0ff", lw=1.5, label="detected pitch (f0)")
    ax.set_title(path.stem)
    ax.set_xlabel("s")
    ax.set_ylabel("Hz")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    plt.close(fig)


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", nargs="+", type=Path)
    ap.add_argument("-o", "--out", type=Path, help="output file (single input only)")
    args = ap.parse_args(argv)
    for path in args.files:
        out = args.out or path.with_suffix(".png")
        plot(path, out)
        print(out)


if __name__ == "__main__":
    main()
