"""Live image-store mirror over fleetlink (class C6, API.md §13).

Robot side, :class:`StoreTail`: tails the robot's hydra image store (sqlite,
WAL, opened read-only so Hydra keeps writing undisturbed), selects frames
(``keyframes`` = kind 0, what SAM3 search uses, or ``all``) in ``frame_id``
order and encodes each with :func:`hydra_image_store.transfer.encode_frame`
(``none`` | ``lossless`` | ``fast``).  The frame row carries the sensor
calibration, so the base can rebuild a valid store from any frame.

Base side, :class:`StoreMirror`: writes every received frame into
``<root>/<robot>/<store_id>/image_store`` with
:func:`hydra_image_store.transfer.write_frame` (source frame / sensor ids
kept), on a writer thread.

Protocol (nothing lost or duplicated across robot, agent or console restarts):

* the agent's C1 ``state`` carries ``store = {id, active, after, sent, last,
  backlog, ...}``.  ``id`` names the robot's *store* (it changes when Hydra
  starts a new session and rotates the store, since frame ids restart then);
* the base answers a state whose store it does not mirror yet, or whose
  ``active`` is false (agent restarted), with the C0 request ``store {id,
  after}``: ``after`` is the highest ``frame_id`` already in its mirror (None
  = empty).  The agent restarts its send cursor there;
* each C6 message ``adt4/<r>/store/frame`` is ``{id, frame_id, prev, blob}``
  where ``prev`` is the frame sent before it (None for the first after the
  cursor).  The base applies a frame only when ``prev`` equals its watermark;
  otherwise (a frame was lost) it re-sends ``store {id, after: watermark}``,
  and the agent rewinds.  A lost *last* frame shows as ``state.store.sent`` >
  watermark with nothing arriving for ``resend_timeout_s``.
"""

from __future__ import annotations

import hashlib
import logging
import os
import pathlib
import queue
import threading
import time
from typing import Callable

log = logging.getLogger(__name__)

STORE_STREAM = "store/frame"
FRAME_SELECTIONS = ("keyframes", "all")
COMPRESSION_MODES = ("none", "lossless", "fast")


def _transfer():
    from hydra_image_store import transfer

    return transfer


def store_id_of(store_dir: str | os.PathLike, first_ts: int | None) -> str | None:
    """Identity of one Hydra store session: the sqlite file's inode (a rotated
    store is a new file) plus the timestamp of its first frame."""
    db = pathlib.Path(store_dir) / "store.sqlite"
    try:
        st = os.stat(db)
    except OSError:
        return None
    if first_ts is None:
        return None
    h = hashlib.sha1(f"{st.st_dev}:{st.st_ino}:{int(first_ts)}".encode())
    return h.hexdigest()[:12]


# ----------------------------------------------------------------------------
# robot side
# ----------------------------------------------------------------------------


class StoreTail:
    """Read-only tail of a robot image store; pure python, no ROS.

    :meth:`set_cursor` (from the base's ``store`` request) arms it;
    :meth:`take` returns the next frame ids to send (advancing the cursor);
    :meth:`encode` builds a frame's wire payload.
    """

    def __init__(
        self,
        path: str | os.PathLike,
        frames: str = "keyframes",
        compression: str = "fast",
        clock: Callable[[], float] = time.time,
        reopen_s: float = 2.0,
    ):
        if frames not in FRAME_SELECTIONS:
            raise ValueError(f"store_frames must be one of {FRAME_SELECTIONS}")
        if compression not in COMPRESSION_MODES:
            raise ValueError(f"store_compression must be one of {COMPRESSION_MODES}")
        self.path = pathlib.Path(os.path.expanduser(str(path)))
        self.frames = frames
        self.compression = compression
        self.clock = clock
        self.reopen_s = reopen_s
        self._store = None
        self._t_open = -1e9
        self.id: str | None = None
        self.error: str | None = None
        self.cursor: int | None = None  # last frame id handed out (None = start)
        self.after: int | None = None  # the base's watermark when armed
        self.active = False
        self.last_sent: int | None = None
        self.frames_sent = 0
        self.bytes_sent = 0
        self.encode_errors = 0
        self.rewinds = 0
        self._avg_bytes = 60_000.0
        self._lock = threading.RLock()

    # -- store -----------------------------------------------------------

    def _open(self):
        """The reader (opened lazily: Hydra may create the store later)."""
        if self._store is not None:
            return self._store
        now = self.clock()
        if now - self._t_open < self.reopen_s:
            return None
        self._t_open = now
        if not (self.path / "store.sqlite").exists():
            self.error = f"no image store at {self.path}"
            return None
        try:
            from hydra_image_store import ImageStore

            self._store = ImageStore.open(self.path)
            self.error = None
        except Exception as exc:  # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"
            self._store = None
        return self._store

    def _first_ts(self, store) -> int | None:
        row = None
        for _db, r in store._query("SELECT MIN(timestamp_ns) FROM frames"):
            row = r
        return None if row is None or row[0] is None else int(row[0])

    def refresh(self) -> None:
        """Re-check the store identity (Hydra rotates the store per session)."""
        with self._lock:
            store = self._open()
            if store is None:
                return
            try:
                sid = store_id_of(self.path, self._first_ts(store))
            except Exception as exc:  # noqa: BLE001  (store replaced underneath)
                self.error = f"{type(exc).__name__}: {exc}"
                self._close()
                return
            if sid is not None and self.id is not None and sid != self.id:
                self._close()  # new session: reopen the new file
                store = self._open()
                if store is None:
                    return
                sid = store_id_of(self.path, self._first_ts(store))
            if sid != self.id:
                self.id = sid
                self.active = False  # the base must arm the new store
                self.cursor = self.after = self.last_sent = None

    def _close(self) -> None:
        if self._store is not None:
            try:
                self._store.close()
            except Exception:  # noqa: BLE001
                pass
        self._store = None
        self._t_open = -1e9

    # -- cursor ----------------------------------------------------------

    def set_cursor(self, store_id: str | None, after: int | None) -> dict:
        """The base's ``store`` request: resume after ``after`` (None = start)."""
        with self._lock:
            self.refresh()
            if self.id is None:
                raise ValueError(self.error or "store not available yet")
            if store_id != self.id:
                raise ValueError(f"store id {store_id!r} is not current ({self.id})")
            if self.active and after != self.cursor:
                self.rewinds += 1
            self.after = None if after is None else int(after)
            self.cursor = self.after
            self.active = True
            return {"id": self.id, "after": self.after, "backlog": self.backlog()}

    def _select(self, after: int | None, limit: int | None):
        store = self._open()
        if store is None:
            return []
        return _transfer().select_frames(store, self.frames, after=after, limit=limit)

    def take(self, n: int) -> list[tuple[int, int | None]]:
        """Up to ``n`` next ``(frame_id, prev)`` after the cursor (advances it)."""
        with self._lock:
            if not self.active or n <= 0:
                return []
            try:
                frames = self._select(self.cursor, n)
            except Exception as exc:  # noqa: BLE001
                self.error = f"{type(exc).__name__}: {exc}"
                return []
            out = []
            for f in frames:
                out.append((int(f.frame_id), self.cursor))
                self.cursor = int(f.frame_id)
            return out

    def encode(self, frame_id: int, prev: int | None) -> dict | None:
        """Wire payload of one frame.  A frame that cannot be encoded (file gone,
        corrupt image) is sent as a skip (``blob`` None, ``error`` set) so the
        ``prev`` chain stays intact and the base moves its watermark past it."""
        with self._lock:
            store = self._open()
            if store is None:
                return None  # store vanished: nothing sent, the base re-arms later
            try:
                blob = _transfer().encode_frame(store, int(frame_id), self.compression)
            except Exception as exc:  # noqa: BLE001
                self.encode_errors += 1
                self.error = f"encode {frame_id}: {type(exc).__name__}: {exc}"
                log.warning("store mirror: %s", self.error)
                return {"id": self.id, "frame_id": int(frame_id), "prev": prev,
                        "blob": None, "error": self.error}  # fmt: skip
            self._avg_bytes = 0.8 * self._avg_bytes + 0.2 * len(blob)
            return {"id": self.id, "frame_id": int(frame_id), "prev": prev, "blob": blob}

    def mark_sent(self, frame_id: int, nbytes: int) -> None:
        with self._lock:
            self.last_sent = int(frame_id)
            self.frames_sent += 1
            self.bytes_sent += int(nbytes)

    @property
    def avg_bytes(self) -> float:
        return self._avg_bytes

    def backlog(self) -> int:
        """Selected frames after the cursor (not yet handed out)."""
        store = self._open()
        if store is None:
            return 0
        sql = "SELECT COUNT(*) FROM frames WHERE (rgb_path IS NOT NULL OR depth_path IS NOT NULL)"
        params: list = []
        if self.frames == "keyframes":
            sql += " AND kind = 0"
        if self.cursor is not None:
            sql += " AND frame_id > ?"
            params.append(self.cursor)
        try:
            return int(sum(r[0] for _db, r in store._query(sql, params)))
        except Exception:  # noqa: BLE001
            return 0

    def last_frame(self) -> int | None:
        store = self._open()
        if store is None:
            return None
        sql = "SELECT MAX(frame_id) FROM frames WHERE (rgb_path IS NOT NULL OR depth_path IS NOT NULL)"
        if self.frames == "keyframes":
            sql += " AND kind = 0"
        try:
            vals = [r[0] for _db, r in store._query(sql) if r[0] is not None]
        except Exception:  # noqa: BLE001
            return None
        return max(vals) if vals else None

    def state(self) -> dict:
        with self._lock:
            return {
                "id": self.id,
                "active": self.active,
                "after": self.after,
                "sent": self.last_sent,
                "last": self.last_frame(),
                "backlog": self.backlog() if self.active else None,
                "frames_sent": self.frames_sent,
                "bytes_sent": self.bytes_sent,
                "avg_frame_bytes": round(self._avg_bytes),
                "frames": self.frames,
                "compression": self.compression,
                "rewinds": self.rewinds,
                "error": self.error,
            }

    def close(self) -> None:
        with self._lock:
            self._close()


# ----------------------------------------------------------------------------
# base side
# ----------------------------------------------------------------------------


class _Mirror:
    def __init__(self, robot: str, store_id: str, path: pathlib.Path):
        self.robot = robot
        self.id = store_id
        self.path = path
        self.writer = None
        self.last: int | None = None  # watermark: highest frame id accepted
        self.frames = 0
        self.bytes = 0
        self.dups = 0
        self.skipped = 0
        self.gaps = 0
        self.errors = 0
        self.last_error: str | None = None
        self.t_req = -1e9
        self.req_after: object = "never"  # after of the last request sent
        self.t_rx = None
        self.robot_state: dict = {}


class StoreMirror:
    """Base-side writer of live store mirrors (one per robot store).

    ``on_state`` / ``on_frame`` return the C0 requests to send as
    ``[(robot, "store", args)]``; the receiver sends (and retries) them.
    Frames are written on a worker thread (``threaded=False``: inline, tests).
    """

    def __init__(
        self,
        root: str | os.PathLike,
        clock: Callable[[], float] = time.time,
        threaded: bool = True,
        request_interval_s: float = 2.0,
        resend_timeout_s: float = 5.0,
    ):
        self.root = pathlib.Path(os.path.expanduser(str(root)))
        self.clock = clock
        self.threaded = threaded
        self.request_interval_s = request_interval_s
        self.resend_timeout_s = resend_timeout_s
        self._mirrors: dict[str, _Mirror] = {}  # robot -> current mirror
        self._lock = threading.RLock()
        self._q: queue.Queue = queue.Queue()
        self._thread: threading.Thread | None = None
        if threaded:
            self._thread = threading.Thread(
                target=self._loop, daemon=True, name="fleetlink-store-mirror"
            )
            self._thread.start()

    # -- mirrors ---------------------------------------------------------

    def path_for(self, robot: str, store_id: str) -> pathlib.Path:
        return self.root / robot / store_id / "image_store"

    def _mirror(self, robot: str, store_id: str) -> _Mirror:
        m = self._mirrors.get(robot)
        if m is not None and m.id == store_id:
            return m
        if m is not None:
            self._q.put(("close", m, None))
        m = _Mirror(robot, store_id, self.path_for(robot, store_id))
        m.last = self._existing_watermark(m.path)
        self._mirrors[robot] = m
        return m

    @staticmethod
    def _existing_watermark(path: pathlib.Path) -> int | None:
        db = path / "store.sqlite"
        if not db.exists():
            return None
        try:
            from hydra_image_store import ImageStore

            with ImageStore.open(path) as s:
                vals = [r[0] for _d, r in s._query("SELECT MAX(frame_id) FROM frames")]
            return vals[0] if vals and vals[0] is not None else None
        except Exception:  # noqa: BLE001
            log.exception("store mirror: cannot read %s", path)
            return None

    def _request(self, m: _Mirror, now: float, force: bool = False) -> list:
        if not force and now - m.t_req < self.request_interval_s:
            return []
        m.t_req = now
        m.req_after = m.last
        return [(m.robot, "store", {"id": m.id, "after": m.last})]

    # -- inputs (receiver thread) ----------------------------------------

    def on_state(self, robot: str, st: dict | None, now: float | None = None) -> list:
        """The ``store`` part of the agent's state."""
        if not st or not st.get("id"):
            return []
        now = self.clock() if now is None else now
        with self._lock:
            m = self._mirror(robot, str(st["id"]))
            m.robot_state = dict(st)
            if not st.get("active") or m.req_after == "never":
                # agent not armed (agent restart / new store), or armed by an
                # earlier console: tell it where this mirror stands
                return self._request(m, now)
            sent = st.get("sent")
            if (
                sent is not None
                and (m.last is None or int(sent) > m.last)
                and (m.t_rx is None or now - m.t_rx >= self.resend_timeout_s)
                and now - m.t_req >= self.resend_timeout_s
            ):
                m.gaps += 1  # the last frame(s) were lost
                return self._request(m, now, force=True)
            return []

    def on_frame(self, robot: str, p: dict, now: float | None = None) -> list:
        now = self.clock() if now is None else now
        with self._lock:
            m = self._mirrors.get(robot)
            if m is None or p.get("id") != m.id:
                return []  # not armed for this store (yet)
            m.t_rx = now
            fid = int(p["frame_id"])
            if m.last is not None and fid <= m.last:
                m.dups += 1
                return []
            if p.get("prev") != m.last:
                m.gaps += 1
                return self._request(m, now)
            m.last = fid
            if p.get("blob") is None:  # the robot could not encode it
                m.skipped += 1
                return []
            m.frames += 1
            m.bytes += len(p["blob"])
            item = ("write", m, p["blob"])
        if self.threaded:
            self._q.put(item)
        else:
            self._apply(*item)
        return []

    # -- writing ---------------------------------------------------------

    def _loop(self) -> None:
        while True:
            item = self._q.get()
            if item is None:
                return
            self._apply(*item)

    def _apply(self, op: str, m: _Mirror, blob) -> None:
        if op == "flush":
            blob.set()
            return
        if op == "close":
            if m.writer is not None:
                try:
                    m.writer.close()
                except Exception:  # noqa: BLE001
                    pass
                m.writer = None
            return
        try:
            t = _transfer()
            if m.writer is None:
                from hydra_image_store import ImageStore

                m.path.mkdir(parents=True, exist_ok=True)
                m.writer = ImageStore.open_writer(m.path)
            t.write_frame(m.writer, t.decode_frame(blob))
        except Exception as exc:  # noqa: BLE001
            with self._lock:
                m.errors += 1
                m.last_error = f"{type(exc).__name__}: {exc}"
            log.warning("store mirror %s: %s", m.robot, m.last_error)

    def flush(self, timeout: float = 10.0) -> None:
        """Wait until queued frames are written (tests / shutdown)."""
        if not self.threaded:
            return
        done = threading.Event()
        self._q.put(("flush", None, done))
        done.wait(timeout)

    def stats(self, robot: str | None = None) -> dict:
        with self._lock:
            out = {}
            for r, m in self._mirrors.items():
                st = m.robot_state
                last = st.get("last")
                out[r] = {
                    "store_id": m.id,
                    "path": str(m.path),
                    "watermark": m.last,
                    "frames": m.frames,
                    "bytes": m.bytes,
                    "avg_frame_bytes": round(m.bytes / m.frames) if m.frames else None,
                    "backlog": st.get("backlog"),
                    "robot_last": last,
                    "duplicates": m.dups,
                    "skipped": m.skipped,
                    "gaps": m.gaps,
                    "write_errors": m.errors,
                    "last_error": m.last_error,
                    "frames_selection": st.get("frames"),
                    "compression": st.get("compression"),
                    "queued_writes": self._q.qsize(),
                }
            return out if robot is None else out.get(robot, {})

    def close(self) -> None:
        if self._thread is not None:
            self.flush()
            for m in list(self._mirrors.values()):
                self._q.put(("close", m, None))
            self._q.put(None)
            self._thread.join(timeout=5.0)
            self._thread = None
        else:
            for m in list(self._mirrors.values()):
                self._apply("close", m, None)
