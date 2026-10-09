"""Base-station side of fleetlink (API.md §8).

``Receiver(transport, on_update)`` mirrors every robot's graph (``GraphState``)
and mesh tiles, tracks link statistics, detects C2 sequence gaps and epoch
changes (→ rate-limited resync request) and sends C0 requests with retry until
acked.  ``on_update(robot, kind, data)`` kinds:

* ``snapshot`` - ``{"epoch", "nodes", "edges"}`` after a snapshot completed
* ``delta``    - ``GraphDelta.encode()`` dict + ``{"seq"}``
* ``tiles``    - ``{"changed": [keys], "removed": [keys]}``
* ``state``    - the robot's C1 state dict
* ``link``     - same as :meth:`Receiver.link` (once per second per robot)
* ``grid``     - ``{hash, frame, width, height, resolution, t}`` (data via :meth:`Receiver.grid`)
* ``paths``    - the paths dict ``{smooth, mlp, target, frame, t, hash}``
* ``video``    - one C5 video frame ``{camera, codec, seq, t, key, width, height,
  fps, data, ...}`` plus ``gap`` (True when frames of that stream were lost
  in between; a delta after a gap cannot be decoded)

Live video is turned on / kept alive / off with :meth:`Receiver.video`
(C0 ``video`` op); :meth:`Receiver.video_stats` has per-stream receive rates.

Camera frames are pulled on demand with :meth:`Receiver.camera` (blocking) or
:meth:`Receiver.query_camera` (callback), over the transport's C4 request/reply.

Time comes from ``clock`` (default ``time.time``); without a background thread
(``threaded=False``) call :meth:`tick` yourself.
"""

from __future__ import annotations

import itertools
import secrets
import threading
import time
from typing import Callable

import msgpack

from fleetlink import codec
from fleetlink.codec import GraphDelta, GraphState
from fleetlink.transport import CLASSES, Transport

STREAM_CLASS = {
    "ctl/ack": "C0",
    "state": "C1",
    "dsg/delta": "C2",
    "mesh/tile": "C3",
    "mesh/manifest": "C3",
    "grid/occupancy": "C3",
    "paths": "C1",
}
RESEND_STREAMS = ("grid", "paths")
VIDEO_PREFIX = "video/"


def stream_class(stream: str) -> str | None:
    if stream.startswith(VIDEO_PREFIX):
        return "C5"
    return STREAM_CLASS.get(stream)


class _VideoRx:
    """Receive-side stats of one video stream."""

    def __init__(self):
        self.frames = 0
        self.keys = 0
        self.bytes = 0
        self.gaps = 0
        self.lost = 0
        self.win_frames = 0
        self.win_bytes = 0
        self.fps = 0.0
        self.kbps = 0.0
        self.last: dict = {}
        self.last_rx: float | None = None

    def stats(self, now: float) -> dict:
        return {
            "frames": self.frames,
            "keys": self.keys,
            "bytes": self.bytes,
            "gaps": self.gaps,
            "lost": self.lost,
            "fps": round(self.fps, 2),
            "kbps": round(self.kbps, 1),
            "age_s": None if self.last_rx is None else now - self.last_rx,
            **self.last,
        }


class _Robot:
    def __init__(self, name: str):
        self.name = name
        self.epoch: str | None = None
        self.seqs: dict[str, int] = {}
        self.graph = GraphState()
        self.tiles: dict[str, dict] = {}
        self.manifest: dict[str, str] = {}
        self.state: dict = {}
        self.last_seen: float | None = None
        self.last_state_rx: float | None = None
        self.latency: float | None = None
        self.rtt: float | None = None
        self.bytes = dict.fromkeys(CLASSES, 0)
        self.rates = dict.fromkeys(CLASSES, 0.0)
        self.win = dict.fromkeys(CLASSES, 0)
        self.win_t: float | None = None
        self.gaps = 0
        self.drops = 0
        self.resyncs = 0  # completed snapshots
        self.resync_requests = 0
        self.epochs_seen = 0
        self.retired: set[str] = set()
        self.last_resync_req = -1e9
        self.in_snapshot = False
        self.tile_req_t: dict[str, float] = {}
        self.t_ping = -1e9
        self.grid: dict | None = None
        self.paths: dict | None = None
        self.mismatch_since: dict[str, float] = {}
        self.resend_req_t: dict[str, float] = {}
        self.video: dict[str, _VideoRx] = {}


class Receiver:
    def __init__(
        self,
        transport: Transport,
        on_update: Callable[[str, str, object], None] | None = None,
        *,
        clock: Callable[[], float] = time.time,
        threaded: bool = True,
        tick_period: float = 0.1,
        retry_s: float = 1.0,
        max_retries: int = 30,
        resync_min_interval: float = 5.0,
        ping_period: float = 1.0,
        tile_rerequest_s: float = 10.0,
        connected_timeout: float = 3.0,
        resend_grace_s: float = 10.0,
    ):
        self.transport = transport
        self.on_update = on_update or (lambda r, k, d: None)
        self.clock = clock
        self.threaded = threaded
        self.tick_period = tick_period
        self.retry_s = retry_s
        self.max_retries = max_retries
        self.resync_min_interval = resync_min_interval
        self.ping_period = ping_period
        self.tile_rerequest_s = tile_rerequest_s
        self.connected_timeout = connected_timeout
        self.resend_grace_s = resend_grace_s
        self._robots: dict[str, _Robot] = {}
        self._pending: dict[str, dict] = {}  # req id -> {robot, req, t, tries}
        self._ids = itertools.count()
        self._session = secrets.token_hex(3)
        self._req_seq = itertools.count()
        self._lock = threading.RLock()
        self._sub = None
        self._thread: threading.Thread | None = None
        self._running = False
        self.failed_requests: list[dict] = []

    # ----------------------------------------------------------- lifecycle

    def start(self) -> None:
        self._sub = self.transport.subscribe("adt4/**", self._on_msg)
        if self.threaded:
            self._running = True
            self._thread = threading.Thread(
                target=self._loop, daemon=True, name="fleetlink-rx"
            )
            self._thread.start()

    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        unsub = getattr(self.transport, "unsubscribe", None)
        if unsub is not None and self._sub is not None:
            unsub(self._sub)
        self._sub = None

    def _loop(self) -> None:
        while self._running:
            try:
                self.tick()
            except Exception:
                pass
            time.sleep(self.tick_period)

    # -------------------------------------------------------------- queries

    def robots(self) -> list[str]:
        with self._lock:
            return sorted(self._robots)

    def graph(self, robot: str) -> GraphState:
        with self._lock:
            return self._robot(robot).graph

    def tiles(self, robot: str) -> dict[str, dict]:
        with self._lock:
            return dict(self._robot(robot).tiles)

    def state(self, robot: str) -> dict:
        with self._lock:
            return dict(self._robot(robot).state)

    def grid(self, robot: str) -> dict | None:
        """Latest occupancy grid wire dict (``data`` = zlib int8), or None."""
        with self._lock:
            r = self._robots.get(robot)
            return dict(r.grid) if r is not None and r.grid is not None else None

    def paths(self, robot: str) -> dict | None:
        with self._lock:
            r = self._robots.get(robot)
            return dict(r.paths) if r is not None and r.paths is not None else None

    def cameras(self, robot: str) -> list[str]:
        """Camera names the robot advertises in its C1 state."""
        with self._lock:
            r = self._robots.get(robot)
            return list((r.state or {}).get("cameras") or []) if r is not None else []

    def query_camera(
        self,
        robot: str,
        name: str,
        callback: Callable[[dict | None], None],
        width: int = 640,
        quality: int = 70,
        timeout: float = 5.0,
    ) -> None:
        """Ask the robot for its newest ``name`` frame.  ``callback(reply)`` with
        ``{format, data, width, height, stamp, age_s, ...}`` or ``{error, ...}``;
        ``callback(None)`` on timeout."""

        def done(data):
            if data is None:
                callback(None)
                return
            try:
                rep = msgpack.unpackb(data, raw=False)
            except Exception:  # noqa: BLE001
                rep = {"error": "bad reply"}
            with self._lock:
                r = self._robot(robot)
                r.bytes["C4"] += len(data)
                r.win["C4"] += len(data)
            callback(rep)

        self.transport.query(
            f"adt4/{robot}/media/camera/{name}",
            {"w": int(width), "q": int(quality)},
            done,
            timeout,
        )

    def camera(self, robot, name, width=640, quality=70, timeout=5.0) -> dict | None:
        """Blocking :meth:`query_camera` (needs a transport that answers on its own
        threads, i.e. zenoh; with a virtual-clock link use ``query_camera``)."""
        ev = threading.Event()
        out = {}

        def cb(rep):
            out["rep"] = rep
            ev.set()

        self.query_camera(robot, name, cb, width, quality, timeout)
        ev.wait(timeout + 1.0)
        return out.get("rep")

    def link(self, robot: str, now: float | None = None) -> dict:
        now = self.clock() if now is None else now
        with self._lock:
            return self._link(self._robot(robot), now)

    def _link(self, r: _Robot, now: float) -> dict:
        stats = (r.state or {}).get("stats") or {}
        staleness = None if r.last_seen is None else now - r.last_seen
        return {
            "connected": staleness is not None and staleness < self.connected_timeout,
            "epoch": r.epoch,
            "last_seen": r.last_seen,
            "staleness_s": staleness,
            "state_age_s": None if r.last_state_rx is None else now - r.last_state_rx,
            "latency_ms": None if r.latency is None else 1000.0 * r.latency,
            "rtt_ms": None if r.rtt is None else 1000.0 * r.rtt,
            "nodes": len(r.graph.records),
            "tiles": len(r.tiles),
            "budgets": stats.get("budgets", {}),
            "rates": {c: r.rates[c] for c in ("C1", "C2", "C3", "C4", "C5")},
            "bytes": dict(r.bytes),
            "drops": r.drops,
            "gaps": r.gaps,
            "resyncs": r.resyncs,
            "resync_requests": r.resync_requests,
            "auto": stats.get("auto", False),
            "robot_queue": stats.get("queue", {}),
        }

    def _robot(self, name: str) -> _Robot:
        r = self._robots.get(name)
        if r is None:
            r = self._robots[name] = _Robot(name)
        return r

    # ------------------------------------------------------------- requests

    def _send(
        self, robot: str, op: str, args: dict | None = None, retry: bool = True
    ) -> str:
        rid = f"{self._session}-{next(self._ids)}"
        req = {"id": rid, "op": op, "args": args or {}}
        with self._lock:
            if retry:
                self._pending[rid] = {
                    "robot": robot,
                    "req": req,
                    "t": self.clock(),
                    "tries": 1,
                }
        self._put_req(robot, req)
        return rid

    def _put_req(self, robot: str, req: dict) -> None:
        env = codec.envelope("base", "ctl/req", next(self._req_seq), self._session, req)
        env["t"] = self.clock()
        self.transport.put(f"adt4/{robot}/ctl/req", codec.pack(env), "C0")

    def set_budget(self, robot: str, cls: str, bps: float) -> str:
        if cls not in ("C1", "C2", "C3", "C4", "C5"):
            raise ValueError(f"unknown class {cls!r}")
        return self._send(robot, "budget", {cls: float(bps)})

    def video(
        self,
        robot: str,
        camera: str,
        enable: bool = True,
        retry: bool = True,
        **params,
    ) -> str:
        """C0 ``video {camera, enable, fps, kbps, width, codec}``: start / update /
        keep alive (repeat at least every 6 s) or stop (``enable=False``) a stream.
        Keepalives can go with ``retry=False``: the next one follows anyway."""
        args = {"camera": camera, "enable": bool(enable)}
        args.update({k: v for k, v in params.items() if v is not None})
        return self._send(robot, "video", args, retry=retry)

    def video_stats(self, robot: str, now: float | None = None) -> dict:
        """``{camera: {frames, keys, bytes, gaps, lost, fps, kbps, age_s, codec,
        width, height}}`` measured here, plus ``robot`` = the robot's own report
        for that stream from its C1 state (absent when it is not streaming)."""
        now = self.clock() if now is None else now
        with self._lock:
            r = self._robots.get(robot)
            if r is None:
                return {}
            reported = (r.state or {}).get("video") or {}
            out = {c: v.stats(now) for c, v in r.video.items()}
            for cam, st in reported.items():
                out.setdefault(cam, {})["robot"] = st
            return out

    def set_auto(self, robot: str, enabled: bool) -> str:
        return self._send(robot, "auto", {"enabled": bool(enabled)})

    def request_resync(self, robot: str) -> str:
        with self._lock:
            r = self._robot(robot)
            r.resync_requests += 1
            r.last_resync_req = self.clock()
        return self._send(robot, "resync")

    def pending_requests(self) -> list[dict]:
        with self._lock:
            return [
                dict(p["req"], robot=p["robot"], tries=p["tries"])
                for p in self._pending.values()
            ]

    def _maybe_resync(self, r: _Robot, now: float, force: bool = False) -> None:
        if force or now - r.last_resync_req >= self.resync_min_interval:
            if any(
                p["req"]["op"] == "resync" and p["robot"] == r.name
                for p in self._pending.values()
            ):
                return
            r.resync_requests += 1
            r.last_resync_req = now
            self._pending_send(r.name, "resync", {})

    def _pending_send(self, robot: str, op: str, args: dict) -> None:
        # called with the lock held; the put itself is fine under the RLock
        rid = f"{self._session}-{next(self._ids)}"
        req = {"id": rid, "op": op, "args": args}
        self._pending[rid] = {"robot": robot, "req": req, "t": self.clock(), "tries": 1}
        self._put_req(robot, req)

    # ------------------------------------------------------------- receive

    def _on_msg(self, key: str, data: bytes) -> None:
        try:
            env = codec.unpack(data)
            robot, stream = env["robot"], env["stream"]
        except Exception:
            return
        cls = stream_class(stream)
        if cls is None:
            return  # e.g. our own ctl/req
        now = self.clock()
        events = []
        with self._lock:
            r = self._robot(robot)
            r.bytes[cls] += len(data)
            r.win[cls] += len(data)
            r.last_seen = now
            lat = now - float(env.get("t", now))
            r.latency = lat if r.latency is None else 0.8 * r.latency + 0.2 * lat
            if env["epoch"] in r.retired:
                return  # late message from a previous agent incarnation
            if r.epoch != env["epoch"]:
                self._new_epoch(r, env["epoch"], now)
            seq = int(env["seq"])
            last = r.seqs.get(stream)
            if last is not None and seq <= last:
                return  # duplicate / reordered
            gap = last is not None and seq > last + 1
            if gap:
                r.drops += seq - last - 1
                if stream == "dsg/delta":
                    r.gaps += 1
                    self._maybe_resync(r, now)
            r.seqs[stream] = seq
            p = env["p"]
            if cls == "C5":
                events.append(("video", self._on_video(r, stream, p, seq, last, now)))
            elif stream == "state":
                r.state = p
                r.last_state_rx = now
                events.append(("state", p))
                self._check_hashes(r, p, now)
            elif stream == "grid/occupancy":
                r.grid = p
                events.append(
                    (
                        "grid",
                        {
                            k: p.get(k)
                            for k in (
                                "hash",
                                "frame",
                                "width",
                                "height",
                                "resolution",
                                "t",
                            )
                        },
                    )
                )
            elif stream == "paths":
                r.paths = p
                events.append(("paths", p))
            elif stream == "dsg/delta":
                events.extend(self._on_graph(r, p, seq))
            elif stream == "mesh/tile":
                r.tiles[p["key"]] = p
                r.tile_req_t.pop(p["key"], None)
                events.append(("tiles", {"changed": [p["key"]], "removed": []}))
            elif stream == "mesh/manifest":
                events.extend(self._on_manifest(r, p, now))
            elif stream == "ctl/ack":
                self._on_ack(r, p, now)
        for kind, d in events:
            self.on_update(robot, kind, d)

    def _on_video(self, r: _Robot, stream: str, p: dict, seq, last, now) -> dict:
        cam = stream[len(VIDEO_PREFIX) :]
        v = r.video.get(cam)
        if v is None:
            v = r.video[cam] = _VideoRx()
        n = len(p.get("data") or b"")
        gap = last is None or seq > last + 1
        if last is not None and seq > last + 1:
            v.gaps += 1
            v.lost += seq - last - 1
        v.frames += 1
        v.keys += int(bool(p.get("key")))
        v.bytes += n
        v.win_frames += 1
        v.win_bytes += n
        v.last_rx = now
        v.last = {k: p.get(k) for k in ("codec", "width", "height")}
        v.last["target_fps"] = p.get("fps")
        # ``gap``: the first frame we see, or frames were lost before this one
        return {**p, "camera": cam, "gap": gap}

    def _new_epoch(self, r: _Robot, epoch: str, now: float) -> None:
        if r.epoch is not None:
            r.retired.add(r.epoch)
        r.epoch = epoch
        r.epochs_seen += 1
        r.seqs.clear()
        r.graph = GraphState()
        r.tiles.clear()
        r.manifest = {}
        r.tile_req_t.clear()
        r.grid = None
        r.paths = None
        r.mismatch_since.clear()
        r.resend_req_t.clear()
        r.video.clear()
        r.in_snapshot = False
        # requests for the previous incarnation are void
        for rid in [k for k, v in self._pending.items() if v["robot"] == r.name]:
            del self._pending[rid]
        self._maybe_resync(r, now, force=True)

    def _on_graph(self, r: _Robot, p: dict, seq: int) -> list:
        kind = p.get("kind", "delta")
        delta = GraphDelta.decode(p)
        if kind == "snapshot_begin":
            r.graph = GraphState()
            r.in_snapshot = True
        r.graph.apply(delta)
        if kind == "snapshot_end":
            r.in_snapshot = False
            r.resyncs += 1
            return [
                (
                    "snapshot",
                    {
                        "epoch": r.epoch,
                        "nodes": len(r.graph.records),
                        "edges": len(r.graph.edges),
                    },
                )
            ]
        if kind == "delta" and not delta.empty():
            return [
                ("delta", {**{k: v for k, v in p.items() if k != "kind"}, "seq": seq})
            ]
        return []

    def _check_hashes(self, r: _Robot, p: dict, now: float) -> None:
        """The state lists the hash of the last grid / paths the robot sent.  If ours
        differs for longer than ``resend_grace_s`` (the C3 / C1 message was dropped),
        ask the robot to send it again (C0 ``resend``)."""
        hashes = p.get("hashes") or {}
        stale = []
        for name in RESEND_STREAMS:
            theirs = hashes.get(name)
            have = getattr(r, name)
            ours = have.get("hash") if have else None
            if theirs is None or theirs == ours:
                r.mismatch_since.pop(name, None)
                continue
            since = r.mismatch_since.setdefault(name, now)
            if (
                now - since >= self.resend_grace_s
                and now - r.resend_req_t.get(name, -1e9) >= self.resend_grace_s
            ):
                stale.append(name)
        if stale:
            for name in stale:
                r.resend_req_t[name] = now
            self._pending_send(r.name, "resend", {"streams": stale})

    def _on_manifest(self, r: _Robot, p: dict, now: float) -> list:
        manifest = p.get("tiles") or {}
        r.manifest = manifest
        removed = [k for k in r.tiles if k not in manifest]
        for k in removed:
            del r.tiles[k]
        missing = [
            k
            for k, h in manifest.items()
            if (k not in r.tiles or r.tiles[k]["hash"] != h)
            and now - r.tile_req_t.get(k, -1e9) >= self.tile_rerequest_s
        ]
        if missing:
            for k in missing:
                r.tile_req_t[k] = now
            self._pending_send(r.name, "tiles", {"keys": missing})
        return [("tiles", {"changed": [], "removed": removed})] if removed else []

    def _on_ack(self, r: _Robot, p: dict, now: float) -> None:
        self._pending.pop(p.get("id"), None)
        res = p.get("result") or {}
        if res.get("t") is not None:  # ping ack (pings are fire-and-forget)
            rtt = now - float(res["t"])
            r.rtt = rtt if r.rtt is None else 0.8 * r.rtt + 0.2 * rtt
        if "budgets" in res and isinstance(r.state, dict):
            r.state.setdefault("stats", {})["budgets"] = {
                c: res["budgets"].get(c) for c in ("C1", "C2", "C3", "C4", "C5")
            }
        if "auto" in res and isinstance(r.state, dict):
            r.state.setdefault("stats", {})["auto"] = res["auto"]

    # ---------------------------------------------------------------- tick

    def tick(self, now: float | None = None) -> None:
        now = self.clock() if now is None else now
        events = []
        with self._lock:
            for rid, pend in list(self._pending.items()):
                if now - pend["t"] < self.retry_s:
                    continue
                if pend["tries"] >= self.max_retries:
                    self.failed_requests.append(pend["req"])
                    del self._pending[rid]
                    continue
                pend["tries"] += 1
                pend["t"] = now
                self._put_req(pend["robot"], pend["req"])
            for r in self._robots.values():
                if r.win_t is None:
                    r.win_t = now
                elif now - r.win_t >= 1.0:
                    dt = now - r.win_t
                    for c in CLASSES:
                        r.rates[c] = 0.6 * r.rates[c] + 0.4 * (r.win[c] / dt)
                        r.win[c] = 0
                    for v in r.video.values():
                        v.fps = 0.5 * v.fps + 0.5 * (v.win_frames / dt)
                        v.kbps = 0.5 * v.kbps + 0.5 * (v.win_bytes * 8 / 1000 / dt)
                        v.win_frames = v.win_bytes = 0
                    r.win_t = now
                    events.append((r.name, "link", self._link(r, now)))
                if r.epoch is not None and now - r.t_ping >= self.ping_period:
                    r.t_ping = now
                    rx = {c: r.rates[c] for c in ("C1", "C2", "C3")}
                    req = {
                        "id": f"{self._session}-p{next(self._ids)}",
                        "op": "ping",
                        "args": {"t": now, "rx": rx, "rtt": r.rtt},
                    }
                    self._put_req(r.name, req)
        for robot, kind, d in events:
            self.on_update(robot, kind, d)
