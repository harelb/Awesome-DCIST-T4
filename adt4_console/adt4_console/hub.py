"""WebSocket pub/sub hub.

Frames are binary msgpack ``{topic, seq, data}``; ``seq`` counts per topic so a
client can detect gaps (e.g. dropped graph deltas → refetch the snapshot).

``Hub.publish`` is thread-safe: job workers, the ROS executor and the
fleetlink receiver all publish from their own threads.  Each client owns a
bounded queue; when it is full the *oldest frame of the same topic* is dropped
(falling back to the oldest frame overall), and frames of latest-value topics
replace any queued frame of that topic instead of piling up.

Video topics (``video/<robot>/<camera>``, API.md §12) are neither latest-value
nor droppable one by one: a delta frame is useless without every frame since
the last key frame.  :meth:`Hub.publish_video` keeps a GOP cache (frames from
the newest key frame on, capped at ``gop_max_s`` / ``gop_max_bytes``) that is
replayed to a client when it starts wanting the topic, so its decoder can start
at once.  Per client, when the queue backs up, video frames are dropped *until
the next key frame* (:meth:`Client.push_video`), never in the middle of a chain.
Video is not part of the default subscription.
"""

from __future__ import annotations

import asyncio
import collections
import itertools
import threading
import time

import msgpack

# Topics whose frames carry the full current value: only the newest matters.
LATEST_VALUE_PREFIXES = ("fleet/", "jobs", "job/", "robot/", "link/", "grid/", "mission/")
LATEST_VALUE_SUFFIXES = ("/version",)
VIDEO_PREFIX = "video/"


def is_video(topic: str) -> bool:
    return topic.startswith(VIDEO_PREFIX)


def is_latest_value(topic: str) -> bool:
    return topic.startswith(LATEST_VALUE_PREFIXES) or topic.endswith(
        LATEST_VALUE_SUFFIXES
    )


def default_match(topic: str) -> bool:
    """Default subscription: everything except ``graph/*/delta`` and video."""
    if is_video(topic):
        return False
    return not (topic.startswith("graph/") and topic.endswith("/delta"))


def prefixes_match(prefixes: list[str] | None, topic: str) -> bool:
    if prefixes is None:
        return default_match(topic)
    return any(topic.startswith(p) for p in prefixes)


def encode_frame(topic: str, seq: int, data) -> bytes:
    return msgpack.packb({"topic": topic, "seq": seq, "data": data}, use_bin_type=True)


class Client:
    """One WebSocket connection's outbound queue (sync core, async wakeup)."""

    _ids = itertools.count(1)

    def __init__(self, maxlen: int = 256, loop=None, video_maxlen: int = 32):
        self.id = next(self._ids)
        self.maxlen = maxlen
        self.video_maxlen = video_maxlen  # queued frames per video topic
        self.prefixes: list[str] | None = None  # None -> default_match
        self.queue: collections.deque[tuple[str, bytes]] = collections.deque()
        self.dropped = 0
        self.video_dropped = 0
        self.video_wait_key: set[str] = set()  # topics skipping to the next key frame
        self._lock = threading.Lock()
        self._loop = loop
        self._event = asyncio.Event() if loop is not None else None

    def wants(self, topic: str) -> bool:
        return prefixes_match(self.prefixes, topic)

    def push(self, topic: str, frame: bytes) -> None:
        with self._lock:
            if is_latest_value(topic):
                for i, (t, _) in enumerate(self.queue):
                    if t == topic:
                        del self.queue[i]
                        break
            if len(self.queue) >= self.maxlen:
                victim = next(
                    (i for i, (t, _) in enumerate(self.queue) if t == topic), None
                )
                if victim is None:  # never cut a video chain in the middle
                    victim = next(
                        (i for i, (t, _) in enumerate(self.queue) if not is_video(t)),
                        None,
                    )
                if victim is None:
                    self._purge_video(self.queue[0][0])
                else:
                    del self.queue[victim]
                    self.dropped += 1
            self.queue.append((topic, frame))
        self._wake()

    def _purge_video(self, topic: str) -> None:
        """Drop every queued frame of a video topic and wait for its next key."""
        n = len(self.queue)
        self.queue = collections.deque((t, f) for t, f in self.queue if t != topic)
        self.video_dropped += n - len(self.queue)
        self.video_wait_key.add(topic)

    def push_video(self, topic: str, frame: bytes, key: bool) -> bool:
        """Queue one video frame.  A backed-up client drops frames until the next
        key frame, so it never receives a delta whose predecessor it missed.
        Returns False when the frame was dropped."""
        with self._lock:
            if topic in self.video_wait_key:
                if not key:
                    self.video_dropped += 1
                    return False
                self.video_wait_key.discard(topic)
            queued = sum(1 for t, _ in self.queue if t == topic)
            if queued >= self.video_maxlen or len(self.queue) >= self.maxlen:
                if not key:
                    # the queued frames are still a valid chain; skip from here on
                    self.video_dropped += 1
                    self.video_wait_key.add(topic)
                    return False
                # a key frame supersedes the stale chain still in the queue
                self._purge_video(topic)
                self.video_wait_key.discard(topic)
            self.queue.append((topic, frame))
        self._wake()
        return True

    def pop_all(self) -> list[tuple[str, bytes]]:
        with self._lock:
            out = list(self.queue)
            self.queue.clear()
            return out

    def _wake(self) -> None:
        if self._event is None or self._loop is None:
            return
        try:
            self._loop.call_soon_threadsafe(self._event.set)
        except RuntimeError:  # loop closed
            pass

    async def wait(self) -> list[tuple[str, bytes]]:
        while True:
            frames = self.pop_all()
            if frames:
                return frames
            self._event.clear()
            frames = self.pop_all()
            if frames:
                return frames
            await self._event.wait()


class Hub:
    def __init__(
        self,
        client_queue: int = 256,
        gop_max_s: float = 3.0,
        gop_max_bytes: int = 2_000_000,
        clock=time.monotonic,
    ):
        self.client_queue = client_queue
        self.gop_max_s = gop_max_s
        self.gop_max_bytes = gop_max_bytes
        self.clock = clock
        self._clients: dict[int, Client] = {}
        self._seq: dict[str, int] = collections.defaultdict(int)
        self._latest: dict[str, bytes] = {}
        # video topic -> [(frame bytes, key, t, size)] from the newest key frame
        self._gop: dict[str, list[tuple[bytes, bool, float, int]]] = {}
        # called with a video topic when some client starts waiting for a key frame
        self.on_video_starved = None
        self._lock = threading.Lock()

    def add_client(self, loop=None) -> Client:
        client = Client(self.client_queue, loop)
        with self._lock:
            self._clients[client.id] = client
        return client

    def remove_client(self, client: Client) -> None:
        with self._lock:
            self._clients.pop(client.id, None)

    def subscribe(self, client: Client, prefixes: list[str] | None) -> None:
        """Set ``client``'s topic prefixes and replay cached latest values, plus the
        GOP cache of every video topic the client did not want before."""
        old = client.prefixes
        client.prefixes = list(prefixes) if prefixes is not None else None
        with self._lock:
            cached = list(self._latest.items())
            gops = {t: list(g) for t, g in self._gop.items()}
        for topic, frame in cached:
            if client.wants(topic):
                client.push(topic, frame)
        for topic, frames in gops.items():
            if client.wants(topic) and not prefixes_match(old, topic):
                client.video_wait_key.discard(topic)
                for frame, key, _t, _n in frames:
                    client.push_video(topic, frame, key)

    def publish(self, topic: str, data) -> int:
        with self._lock:
            self._seq[topic] += 1
            seq = self._seq[topic]
            clients = list(self._clients.values())
        frame = encode_frame(topic, seq, data)
        if is_latest_value(topic):
            with self._lock:
                self._latest[topic] = frame
        for c in clients:
            if c.wants(topic):
                c.push(topic, frame)
        return seq

    def publish_video(self, topic: str, data, key: bool) -> int:
        """Publish one video frame (see the module docstring); returns its seq."""
        now = self.clock()
        with self._lock:
            self._seq[topic] += 1
            seq = self._seq[topic]
            clients = list(self._clients.values())
        frame = encode_frame(topic, seq, data)
        with self._lock:
            gop = self._gop.get(topic)
            if key:
                self._gop[topic] = [(frame, True, now, len(frame))]
            elif gop:
                gop.append((frame, False, now, len(frame)))
                if (
                    now - gop[0][2] > self.gop_max_s
                    or sum(e[3] for e in gop) > self.gop_max_bytes
                ):
                    # too long to replay: new viewers wait for the next key frame
                    del self._gop[topic]
        starved = False
        for c in clients:
            if c.wants(topic) and not c.push_video(topic, frame, key) and not key:
                starved = True
        if starved and self.on_video_starved is not None:
            # a backed-up browser now waits for a key frame: ask for one now
            # rather than letting it freeze until the next GOP boundary
            try:
                self.on_video_starved(topic)
            except Exception:  # noqa: BLE001
                pass
        return seq

    def clear_video(self, topic: str) -> None:
        """Forget a stopped stream's GOP cache."""
        with self._lock:
            self._gop.pop(topic, None)

    def gop(self, topic: str) -> dict:
        with self._lock:
            g = self._gop.get(topic) or []
            return {"frames": len(g), "bytes": sum(e[3] for e in g)}

    def latest(self, topic: str):
        with self._lock:
            frame = self._latest.get(topic)
        return msgpack.unpackb(frame, raw=False)["data"] if frame else None

    @property
    def n_clients(self) -> int:
        return len(self._clients)
