"""The hub: the single process that talks to the device.

    device --UDP--> hub --> recordings/*.timu           (start/stop from the web page or --record)
                        --> ws://<host>:8765/ws         binary msgs = raw device packets (protocol.py)
                                                        text msgs   = JSON: {"type":"status"} once per second,
                                                                      {"type":"voice"} per packet (timu.voice)
                        --> http://<host>:8765/         live view + recording controls

Python processing code connects with timu.client; browsers use the page. Run only one hub:
the device streams to a single subscriber.

    uv run python -m timu.hub                       # then open http://localhost:8765
    uv run python -m timu.hub --host 192.168.1.35 --record bench
"""
from __future__ import annotations

import argparse
import asyncio
import json
import re
import socket
import time
from datetime import datetime
from pathlib import Path

from aiohttp import web

from . import protocol as P
from .recording import Writer
from .stats import Stats, format_line
from .voice import VoiceDetector

BROADCAST = "255.255.255.255"
SUBSCRIBE_EVERY_S = 1.0
REDISCOVER_AFTER_S = 3.0
CLIENT_QUEUE_MSGS = 512  # ~15 s of packets; a slower client loses packets (visible as sample gaps)
WEB_DIR = Path(__file__).parent / "web"


class DeviceLink(asyncio.DatagramProtocol):
    """Subscribes to the device (broadcast discovery unless a host is fixed) and keeps it alive."""

    def __init__(self, on_datagram, host: str | None, port: int):
        self.on_datagram = on_datagram
        self.fixed_host = host
        self.port = port
        self.device = host
        self.last_rx = 0.0
        self.transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport):
        self.transport = transport
        transport.get_extra_info("socket").setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 22)

    def datagram_received(self, data, addr):
        if self.fixed_host and addr[0] != self.fixed_host:
            return
        self.last_rx = time.monotonic()
        if self.device != addr[0]:
            print(f"streaming from {addr[0]}")
            self.device = addr[0]
        self.on_datagram(data, addr[0])

    def error_received(self, exc):
        print(f"udp error: {exc}")

    async def keepalive(self):
        while True:
            if not self.fixed_host and self.device and time.monotonic() - self.last_rx > REDISCOVER_AFTER_S:
                print(f"no data from {self.device}, rediscovering")
                self.device = None
            self.transport.sendto(P.CMD_SUBSCRIBE, (self.device or BROADCAST, self.port))
            await asyncio.sleep(SUBSCRIBE_EVERY_S)

    def close(self):
        if self.transport:
            if self.device:
                self.transport.sendto(P.CMD_UNSUBSCRIBE, (self.device, self.port))
            self.transport.close()


class Recorder:
    def __init__(self, out_dir: Path):
        self.out_dir = out_dir
        self.writer: Writer | None = None
        self.meta: dict = {}
        self.stats = Stats()
        self._meta_path: Path | None = None
        self._t0 = 0.0

    @property
    def active(self) -> bool:
        return self.writer is not None

    def start(self, label: str) -> None:
        self.stop()
        label = re.sub(r"[^A-Za-z0-9_-]+", "_", label.strip())[:64]
        self.out_dir.mkdir(parents=True, exist_ok=True)
        stem = datetime.now().strftime("%Y%m%d-%H%M%S") + (f"_{label}" if label else "")
        self.writer = Writer(self.out_dir / f"{stem}.timu")
        self._meta_path = self.out_dir / f"{stem}.json"
        self.meta = {"label": label, "started": datetime.now().astimezone().isoformat(),
                     "protocol_version": P.VERSION}
        self.stats = Stats()
        self._t0 = time.monotonic()
        self._save_meta()
        print(f"recording to {self.writer.path}")

    def write(self, data: bytes, pkt: P.Packet, host_time: float, ip: str) -> None:
        if not self.writer:
            return
        if "device_ip" not in self.meta:
            self.meta.update(device_ip=ip, boot_id=f"{pkt.boot_id:08x}", nominal_rate_hz=pkt.rate_hz,
                             layout=pkt.layout, accel_fs=pkt.accel_fs, gyro_fs=pkt.gyro_fs)
            self._save_meta()
        self.writer.write(data, host_time)
        self.stats.update(pkt)

    def stop(self) -> None:
        if not self.writer:
            return
        self.writer.close()
        s = self.stats
        self.meta.update(ended=datetime.now().astimezone().isoformat(), samples=s.samples,
                         lost_samples=s.lost, discontinuities=s.discontinuities, reboots=s.reboots)
        self._save_meta()
        print(f"saved {self.writer.path} ({s.samples} samples, {s.lost} lost)")
        self.writer = None

    def status(self) -> dict | None:
        if not self.writer:
            return None
        return {"file": self.writer.path.name, "label": self.meta["label"],
                "seconds": round(time.monotonic() - self._t0, 1), "samples": self.stats.samples,
                "lost": self.stats.lost}

    def _save_meta(self):
        self._meta_path.write_text(json.dumps(self.meta, indent=2))


class Hub:
    def __init__(self, args):
        self.link = DeviceLink(self.on_datagram, args.host, args.port)
        self.recorder = Recorder(args.out)
        self.stats = Stats()
        self.clients: set[asyncio.Queue] = set()
        self.status: dict = {}
        self.voice = VoiceDetector()
        self._voice_start: int | None = None
        self._voice_f0: list[float] = []

    def on_datagram(self, data: bytes, ip: str) -> None:
        host_time = time.time()
        try:
            pkt = P.parse(data)
        except ValueError as e:
            self.stats.bad += 1
            print(f"bad packet from {ip}: {e}")
            return
        self.stats.update(pkt)
        self.recorder.write(data, pkt, host_time, ip)
        self._fanout(data)
        self._run_voice(pkt)

    def _run_voice(self, pkt: P.Packet) -> None:
        frames = self.voice.push_packet(pkt)
        if not frames:
            return
        for fr in frames:  # log each finished utterance to the terminal
            if fr.voiced and self._voice_start is None:
                self._voice_start, self._voice_f0 = fr.sample, []
            if fr.voiced and fr.f0_hz == fr.f0_hz:
                self._voice_f0.append(fr.f0_hz)
            if not fr.voiced and self._voice_start is not None:
                dur = (fr.sample - self._voice_start) / self.voice.rate_hz
                f0 = sorted(self._voice_f0)[len(self._voice_f0) // 2] if self._voice_f0 else float("nan")
                print(f"voice: {dur:.2f} s, f0 {f0:.0f} Hz")
                self._voice_start = None
        self._fanout(json.dumps({
            "type": "voice", "rate_hz": round(self.voice.rate_hz, 2),
            # [sample, voiced, f0 (null if none), snr_db]
            "frames": [[fr.sample, fr.voiced, None if fr.f0_hz != fr.f0_hz else round(fr.f0_hz, 1),
                        round(fr.snr_db, 1)] for fr in frames],
        }))

    def _fanout(self, msg) -> None:
        for q in self.clients:
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                pass  # this client is too slow; it sees the gap via first_sample

    async def status_loop(self):
        last = time.monotonic()
        while True:
            await asyncio.sleep(1.0)
            now = time.monotonic()
            s = self.stats.snapshot(now - last)
            last = now
            self.status = {"type": "status", "device": self.link.device,
                           "receiving": now - self.link.last_rx < 1.5,
                           "recording": self.recorder.status(), **s}
            print(format_line(s) + (f" | REC {self.recorder.status()['seconds']:.0f}s"
                                    if self.recorder.active else ""))
            self._fanout(json.dumps(self.status))

    # --- HTTP -------------------------------------------------------------

    async def ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=10)
        await ws.prepare(request)
        q: asyncio.Queue = asyncio.Queue(maxsize=CLIENT_QUEUE_MSGS)
        self.clients.add(q)

        async def pump():
            while True:
                msg = await q.get()
                if isinstance(msg, str):
                    await ws.send_str(msg)
                else:
                    await ws.send_bytes(msg)

        task = asyncio.create_task(pump())
        try:
            async for _ in ws:  # clients don't send anything; this just waits for close
                pass
        finally:
            self.clients.discard(q)
            task.cancel()
        return ws

    async def record_handler(self, request: web.Request) -> web.Response:
        body = await request.json()
        if body.get("action") == "start":
            self.recorder.start(str(body.get("label", "")))
        elif body.get("action") == "stop":
            self.recorder.stop()
        else:
            raise web.HTTPBadRequest(text="action must be 'start' or 'stop'")
        return web.json_response({"recording": self.recorder.status()})

    async def status_handler(self, request: web.Request) -> web.Response:
        return web.json_response(self.status)

    def app(self) -> web.Application:
        app = web.Application()
        app.router.add_get("/", lambda r: web.FileResponse(WEB_DIR / "index.html"))
        app.router.add_get("/ws", self.ws_handler)
        app.router.add_get("/api/status", self.status_handler)
        app.router.add_post("/api/record", self.record_handler)
        return app


async def run(args) -> None:
    hub = Hub(args)
    loop = asyncio.get_running_loop()
    await loop.create_datagram_endpoint(lambda: hub.link, local_addr=("0.0.0.0", 0), allow_broadcast=True)
    runner = web.AppRunner(hub.app())
    await runner.setup()
    await web.TCPSite(runner, args.bind, args.http).start()
    print(f"web view: http://{'localhost' if args.bind in ('127.0.0.1', '0.0.0.0') else args.bind}:{args.http}"
          f"   python clients: ws://<this-pc>:{args.http}/ws")
    print(f"subscribing to {args.host or 'broadcast'}:{args.port} ... (Ctrl+C to stop)")
    if args.record is not None:
        hub.recorder.start(args.record)
    try:
        await asyncio.gather(hub.link.keepalive(), hub.status_loop())
    finally:
        hub.recorder.stop()
        hub.link.close()
        await runner.cleanup()


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", help="device IP (default: broadcast discovery)")
    ap.add_argument("--port", type=int, default=P.UDP_PORT, help="device UDP port")
    ap.add_argument("--http", type=int, default=8765, help="web/websocket port")
    ap.add_argument("--bind", default="127.0.0.1",
                    help="web bind address; 0.0.0.0 to allow other machines (they can also start/stop recordings)")
    ap.add_argument("--out", type=Path, default=Path(__file__).resolve().parents[1] / "recordings")
    ap.add_argument("--record", metavar="LABEL", help="start recording immediately with this label")
    args = ap.parse_args(argv)
    try:
        asyncio.run(run(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
