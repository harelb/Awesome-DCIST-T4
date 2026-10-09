"""Transports for fleetlink: a zenoh one for real deployments and an in-memory
link emulator (bandwidth / latency / loss / bounded queues, virtual clock) for
tests and benchmarks.

Stream classes (API.md §8):

* ``C0`` control  - INTERACTIVE_HIGH, BLOCK, express, reliable
* ``C1`` state    - INTERACTIVE_LOW, DROP
* ``C2`` graph    - DATA_HIGH, BLOCK
* ``C3`` mesh     - DATA_LOW, DROP
* ``C4`` media    - DATA, request/reply only (on-demand camera frames)

Request/reply (``declare_queryable`` / ``query``) carries C4: the base asks,
the robot answers once.  Replies travel above C3 (mesh / grid) in priority,
because an operator is waiting for them, and are rate-limited by the agent's
own C4 budget rather than by the scheduler.
"""

from __future__ import annotations

import heapq
import itertools
import json
import queue
import random
import threading
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

CLASSES = ("C0", "C1", "C2", "C3", "C4")
# strict transmit order on the link (zenoh priority order): C4 sits between
# graph (C2) and mesh (C3)
PRIORITY_ORDER = ("C0", "C1", "C2", "C4", "C3")
PRIORITY = {c: i for i, c in enumerate(PRIORITY_ORDER)}  # lower = more important
BLOCKING = {"C0": True, "C1": False, "C2": True, "C3": False, "C4": False}

Callback = Callable[[str, bytes], None]
# queryable handler: (key, params) -> reply bytes, or None for no reply
QueryHandler = Callable[[str, dict], "bytes | None"]
# query callback: reply bytes, or None on timeout / no replier
ReplyCallback = Callable[["bytes | None"], None]


def encode_params(params: dict | None) -> str:
    """``{"w": 640, "q": 70}`` → ``"w=640;q=70"`` (zenoh selector parameters)."""
    return ";".join(f"{k}={v}" for k, v in (params or {}).items())


def decode_params(text: str) -> dict:
    out = {}
    for part in (text or "").replace("&", ";").split(";"):
        if not part:
            continue
        k, _, v = part.partition("=")
        out[k] = v
    return out


def key_matches(expr: str, key: str) -> bool:
    """Zenoh-style key-expression match supporting ``*`` (one chunk) and ``**``."""
    e, k = expr.split("/"), key.split("/")

    def rec(i: int, j: int) -> bool:
        if i == len(e):
            return j == len(k)
        if e[i] == "**":
            return any(rec(i + 1, jj) for jj in range(j, len(k) + 1))
        if j == len(k):
            return False
        return (e[i] == "*" or e[i] == k[j]) and rec(i + 1, j + 1)

    return rec(0, 0)


class Transport:
    """Minimal pub/sub interface used by the agent and the receiver."""

    def put(self, key: str, data: bytes, cls: str) -> None:
        raise NotImplementedError

    def subscribe(self, key_expr: str, callback: Callback):
        raise NotImplementedError

    def can_send(self, cls: str) -> bool:
        """False when a BLOCK-class put would block (scheduler holds the class back)."""
        return True

    def declare_queryable(self, key_expr: str, handler: QueryHandler):
        """Answer queries on ``key_expr``; ``handler(key, params)`` returns the reply
        bytes (or None for no reply).  It runs on the transport's thread."""
        raise NotImplementedError

    def query(
        self,
        key: str,
        params: dict | None,
        callback: ReplyCallback,
        timeout: float = 5.0,
    ) -> None:
        """Non-blocking request: ``callback(reply_bytes)`` once, or ``callback(None)``
        after ``timeout`` seconds without a reply."""
        raise NotImplementedError

    def close(self) -> None:
        pass


# ----------------------------------------------------------------------------
# Zenoh
# ----------------------------------------------------------------------------


class ZenohTransport(Transport):
    """eclipse-zenoh 1.x transport (client mode towards the local router).

    BLOCK-class puts run on a per-class worker thread with a small queue so a
    congested C2 publisher never stalls the C1 state stream on the caller's
    thread; ``can_send`` reports when that queue is full.
    """

    def __init__(
        self,
        connect: str | list[str] = "tcp/127.0.0.1:7447",
        mode: str = "client",
        session=None,
        block_queue: int = 32,
    ):
        import zenoh

        self._z = zenoh
        if session is None:
            conf = zenoh.Config()
            endpoints = [connect] if isinstance(connect, str) else list(connect)
            conf.insert_json5("mode", json.dumps(mode))
            conf.insert_json5("connect/endpoints", json.dumps(endpoints))
            conf.insert_json5("scouting/multicast/enabled", "false")
            session = zenoh.open(conf)
            self._own_session = True
        else:
            self._own_session = False
        self.session = session
        self._qos = {
            "C0": dict(
                priority=zenoh.Priority.INTERACTIVE_HIGH,
                congestion_control=zenoh.CongestionControl.BLOCK,
                express=True,
                reliability=zenoh.Reliability.RELIABLE,
            ),
            "C1": dict(
                priority=zenoh.Priority.INTERACTIVE_LOW,
                congestion_control=zenoh.CongestionControl.DROP,
            ),
            "C2": dict(
                priority=zenoh.Priority.DATA_HIGH,
                congestion_control=zenoh.CongestionControl.BLOCK,
                reliability=zenoh.Reliability.RELIABLE,
            ),
            "C3": dict(
                priority=zenoh.Priority.DATA_LOW,
                congestion_control=zenoh.CongestionControl.DROP,
                reliability=zenoh.Reliability.BEST_EFFORT,
            ),
            "C4": dict(
                priority=zenoh.Priority.DATA,
                congestion_control=zenoh.CongestionControl.DROP,
            ),
        }
        self._pubs: dict[tuple[str, str], object] = {}
        self._subs = []
        self._lock = threading.Lock()
        self._closed = False
        self._workers: dict[str, tuple[queue.Queue, threading.Thread]] = {}
        for cls in ("C0", "C2"):
            q: queue.Queue = queue.Queue(maxsize=block_queue)
            t = threading.Thread(
                target=self._worker, args=(q,), daemon=True, name=f"fleetlink-{cls}"
            )
            t.start()
            self._workers[cls] = (q, t)

    def _publisher(self, key: str, cls: str):
        with self._lock:
            pub = self._pubs.get((key, cls))
            if pub is None:
                pub = self.session.declare_publisher(key, **self._qos[cls])
                self._pubs[(key, cls)] = pub
            return pub

    def _worker(self, q: queue.Queue) -> None:
        while True:
            item = q.get()
            if item is None:
                return
            key, data, cls = item
            try:
                self._publisher(key, cls).put(data)
            except Exception:  # session closed underneath us
                if self._closed:
                    return

    def put(self, key: str, data: bytes, cls: str) -> None:
        if self._closed:
            return
        if cls in self._workers:
            self._workers[cls][0].put((key, data, cls))  # blocks only if full
        else:
            self._publisher(key, cls).put(data)

    def can_send(self, cls: str) -> bool:
        w = self._workers.get(cls)
        return w is None or not w[0].full()

    def subscribe(self, key_expr: str, callback: Callback):
        def handler(sample):
            callback(str(sample.key_expr), sample.payload.to_bytes())

        sub = self.session.declare_subscriber(key_expr, handler)
        self._subs.append(sub)
        return sub

    def declare_queryable(self, key_expr: str, handler: QueryHandler):
        # replies inherit the query's priority (C4 = DATA, set in ``query``)
        def on_query(query):
            try:
                key = str(query.key_expr)
                data = handler(key, decode_params(str(query.parameters)))
                if data is not None:
                    query.reply(query.key_expr, bytes(data))
            except Exception:  # a bad request must never kill zenoh's thread
                pass

        q = self.session.declare_queryable(key_expr, on_query)
        self._subs.append(q)
        return q

    def query(self, key, params, callback, timeout: float = 5.0) -> None:
        if self._closed:
            callback(None)
            return
        z = self._z
        done = threading.Lock()
        state = {"answered": False}

        def finish(data):
            with done:
                if state["answered"]:
                    return
                state["answered"] = True
            callback(data)

        def on_reply(reply):
            ok = reply.ok
            if ok is not None:
                finish(ok.payload.to_bytes())

        sel = key + ("?" + encode_params(params) if params else "")
        try:
            self.session.get(
                sel,
                z.handlers.Callback(on_reply, lambda: finish(None)),
                timeout=float(timeout),
                priority=self._qos["C4"]["priority"],
            )
        except Exception:
            finish(None)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        for q, t in self._workers.values():
            q.put(None)
            t.join(timeout=2.0)
        for s in self._subs:
            try:
                s.undeclare()
            except Exception:
                pass
        for p in self._pubs.values():
            try:
                p.undeclare()
            except Exception:
                pass
        if self._own_session:
            self.session.close()


# ----------------------------------------------------------------------------
# In-memory link emulator
# ----------------------------------------------------------------------------


class VirtualClock:
    """Manually advanced clock; call it to read the time."""

    def __init__(self, t: float = 0.0):
        self.t = float(t)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> float:
        self.t += dt
        return self.t


@dataclass
class _Msg:
    key: str
    data: bytes
    cls: str
    remaining: int
    lost: bool = False
    extra_delay: float = 0.0
    kind: str = "put"  # put | query | reply
    qid: int = 0
    params: dict | None = None


@dataclass
class LinkStats:
    put_bytes: dict = field(default_factory=lambda: dict.fromkeys(CLASSES, 0))
    delivered_bytes: dict = field(default_factory=lambda: dict.fromkeys(CLASSES, 0))
    wire_bytes: dict = field(default_factory=lambda: dict.fromkeys(CLASSES, 0))
    congestion_drops: dict = field(default_factory=lambda: dict.fromkeys(CLASSES, 0))
    loss_drops: dict = field(default_factory=lambda: dict.fromkeys(CLASSES, 0))
    retransmits: dict = field(default_factory=lambda: dict.fromkeys(CLASSES, 0))


class _Direction:
    """One direction of the link: per-priority tx queues, fragment-level strict
    priority (like zenoh's batching), serialisation at ``bandwidth`` and
    propagation ``latency``.

    Loss is applied per fragment.  Reliable classes retransmit the fragment
    (bandwidth is spent again and delivery is delayed by one RTT); best-effort
    classes lose the whole message.  Under congestion (class queue over
    ``queue_bytes``) DROP classes are dropped at ``put``; BLOCK classes are
    accepted but ``can_send`` turns False, which is what a blocked put would do.
    """

    def __init__(self, link: LinkEmulator, rng: random.Random):
        self.link = link
        self.rng = rng
        self.queues = {c: deque() for c in CLASSES}
        self.queued = dict.fromkeys(CLASSES, 0)
        self.tx_time = 0.0
        self.inflight: list = []
        self.last_delivery = dict.fromkeys(CLASSES, 0.0)
        self.order = itertools.count()
        self.stats = LinkStats()
        self.peer: MemoryTransport | None = None

    def busy(self) -> bool:
        return any(self.queues.values())

    def put(self, key: str, data: bytes, cls: str, now: float, **extra) -> bool:
        n = len(data)
        self.stats.put_bytes[cls] += n
        lk = self.link
        if not BLOCKING[cls] and self.queued[cls] + n > lk.queue_bytes:
            self.stats.congestion_drops[cls] += 1
            return False
        if not self.busy():
            self.tx_time = max(self.tx_time, now)
        self.queues[cls].append(_Msg(key, data, cls, n, **extra))
        self.queued[cls] += n
        return True

    def can_send(self, cls: str) -> bool:
        return self.queued[cls] < self.link.queue_bytes

    def advance(self, t: float) -> list:
        """Transmit until ``t``; returns messages whose delivery time is <= t."""
        lk = self.link
        while self.tx_time < t and self.busy():
            cls = next(c for c in PRIORITY_ORDER if self.queues[c])
            msg = self.queues[cls][0]
            frag = min(lk.mtu, msg.remaining)
            if lk.bandwidth:
                self.tx_time += frag / lk.bandwidth
            self.stats.wire_bytes[cls] += frag
            if lk.loss and self.rng.random() < lk.loss:
                if lk.reliable[cls]:
                    self.stats.retransmits[cls] += 1
                    msg.extra_delay += 2.0 * lk.latency
                    continue  # resend the same fragment
                msg.lost = True
            msg.remaining -= frag
            if msg.remaining > 0:
                continue
            self.queues[cls].popleft()
            self.queued[cls] -= len(msg.data)
            if msg.lost:
                self.stats.loss_drops[cls] += 1
                continue
            when = self.tx_time + lk.latency + msg.extra_delay
            when = max(when, self.last_delivery[cls])  # per-class FIFO
            self.last_delivery[cls] = when
            heapq.heappush(self.inflight, (when, next(self.order), msg))
        if not self.busy():
            self.tx_time = max(self.tx_time, t)
        out = []
        while self.inflight and self.inflight[0][0] <= t:
            _, _, msg = heapq.heappop(self.inflight)
            self.stats.delivered_bytes[msg.cls] += len(msg.data)
            out.append(msg)
        return out


class LinkEmulator:
    """Two :class:`MemoryTransport` endpoints (``.robot`` and ``.base``) joined by
    a simulated radio link.  Deterministic: driven by a :class:`VirtualClock`
    and a seeded RNG; call :meth:`advance` after moving the clock.

    ``bandwidth`` is bytes/s per direction (None = infinite).
    """

    def __init__(
        self,
        clock: Callable[[], float] | None = None,
        bandwidth: float | None = 250_000.0,
        latency: float = 0.15,
        loss: float = 0.0,
        queue_bytes: int = 256 * 1024,
        mtu: int = 8192,
        seed: int = 0,
        reliable: dict | None = None,
    ):
        self.clock = clock or VirtualClock()
        self.bandwidth = bandwidth
        self.latency = latency
        self.loss = loss
        self.queue_bytes = queue_bytes
        self.mtu = mtu
        self.reliable = reliable or {
            "C0": True,
            "C1": True,
            "C2": True,
            "C3": False,
            "C4": True,
        }
        self._lock = threading.RLock()
        rng = random.Random(seed)
        self.robot = MemoryTransport(
            self, _Direction(self, random.Random(rng.random()))
        )
        self.base = MemoryTransport(self, _Direction(self, random.Random(rng.random())))
        self.robot.tx.peer = self.base
        self.base.tx.peer = self.robot

    def advance(self, t: float | None = None) -> None:
        """Move both directions forward to ``t`` (default: the clock) and deliver."""
        t = self.clock() if t is None else t
        with self._lock:
            # deliver in time order across both directions; callbacks may put
            for _ in range(1000):
                batch = []
                for d in (self.robot.tx, self.base.tx):
                    batch.extend((m, d.peer) for m in d.advance(t))
                if not batch:
                    break
                for msg, peer in batch:
                    peer._deliver_msg(msg)
            for end in (self.robot, self.base):
                end._expire_queries(t)

    def stats(self) -> dict:
        return {"up": self.robot.tx.stats, "down": self.base.tx.stats}


class MemoryTransport(Transport):
    """One end of a :class:`LinkEmulator`."""

    def __init__(self, link: LinkEmulator, tx: _Direction):
        self.link = link
        self.tx = tx
        self._subs: list[tuple[str, Callback]] = []
        self._queryables: list[tuple[str, QueryHandler]] = []
        self._queries: dict[int, tuple[float, ReplyCallback]] = {}  # qid -> deadline
        self._qids = itertools.count(1)
        self._closed = False

    def put(self, key: str, data: bytes, cls: str) -> None:
        if self._closed:
            return
        with self.link._lock:
            self.tx.put(key, bytes(data), cls, self.link.clock())

    def can_send(self, cls: str) -> bool:
        return self.tx.can_send(cls)

    def subscribe(self, key_expr: str, callback: Callback):
        handle = (key_expr, callback)
        self._subs.append(handle)
        return handle

    def unsubscribe(self, handle) -> None:
        if handle in self._subs:
            self._subs.remove(handle)

    def declare_queryable(self, key_expr: str, handler: QueryHandler):
        handle = (key_expr, handler)
        self._queryables.append(handle)
        return handle

    def query(self, key, params, callback, timeout: float = 5.0) -> None:
        """The request crosses the link as a small C4 message; the reply comes back
        as C4 on the peer's direction (bandwidth, latency and loss apply).  Expiry
        is checked in :meth:`LinkEmulator.advance`, so it follows the clock."""
        if self._closed:
            callback(None)
            return
        with self.link._lock:
            qid = next(self._qids)
            now = self.link.clock()
            self._queries[qid] = (now + timeout, callback)
            req = (key + "?" + encode_params(params)).encode()
            self.tx.put(key, req, "C4", now, kind="query", qid=qid, params=params)

    def _expire_queries(self, t: float) -> None:
        for qid, (deadline, cb) in list(self._queries.items()):
            if deadline <= t:
                del self._queries[qid]
                cb(None)

    def _deliver_msg(self, msg: _Msg) -> None:
        if self._closed:
            return
        if msg.kind == "put":
            self._deliver(msg.key, msg.data)
        elif msg.kind == "query":
            for expr, handler in list(self._queryables):
                if key_matches(expr, msg.key):
                    try:
                        data = handler(msg.key, dict(msg.params or {}))
                    except Exception:
                        data = None
                    if data is not None:
                        self.tx.put(
                            msg.key,
                            bytes(data),
                            "C4",
                            self.link.clock(),
                            kind="reply",
                            qid=msg.qid,
                        )
                    break
        elif msg.kind == "reply":
            pending = self._queries.pop(msg.qid, None)
            if pending is not None:
                pending[1](msg.data)

    def _deliver(self, key: str, data: bytes) -> None:
        if self._closed:
            return
        for expr, cb in list(self._subs):
            if key_matches(expr, key):
                cb(key, data)

    def close(self) -> None:
        self._closed = True
        self._subs.clear()
        self._queryables.clear()
        for _, cb in self._queries.values():
            cb(None)
        self._queries.clear()

    def reopen(self) -> None:
        """Allow a new agent to reuse this endpoint after ``close`` (restart tests)."""
        self._closed = False
