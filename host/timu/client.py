"""Read the live stream from a running hub (python -m timu.hub).

Blocking:
    from timu.client import stream
    for pkt in stream():                 # timu.protocol.Packet
        process(pkt.first_sample, pkt.accel_g())

Async:
    async for pkt in packets(): ...

Packets are delivered exactly as the device sent them. Use pkt.first_sample / pkt.segment /
pkt.boot_id to detect gaps (see protocol.h); a client that falls >15 s behind loses packets.
"""
from __future__ import annotations

import asyncio
import queue
import threading
from typing import AsyncIterator, Iterator

import aiohttp

from . import protocol as P

DEFAULT_URL = "ws://localhost:8765/ws"


async def packets(url: str = DEFAULT_URL, reconnect: bool = True) -> AsyncIterator[P.Packet]:
    while True:
        try:
            async with aiohttp.ClientSession() as session:
                async with session.ws_connect(url, heartbeat=10) as ws:
                    async for msg in ws:
                        if msg.type == aiohttp.WSMsgType.BINARY:
                            yield P.parse(msg.data)
        except aiohttp.ClientError as e:
            if not reconnect:
                raise
            print(f"timu.client: {e}; retrying")
        if not reconnect:
            return
        await asyncio.sleep(1.0)


def stream(url: str = DEFAULT_URL, reconnect: bool = True) -> Iterator[P.Packet]:
    """Blocking iterator; the websocket runs in a background thread."""
    q: queue.Queue = queue.Queue(maxsize=4096)
    done = object()

    async def pump():
        try:
            async for pkt in packets(url, reconnect):
                q.put(pkt)
        finally:
            q.put(done)

    threading.Thread(target=lambda: asyncio.run(pump()), daemon=True).start()
    while (item := q.get()) is not done:
        yield item
