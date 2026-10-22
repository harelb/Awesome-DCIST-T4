"""Console side of fleetlink: owns the ``fleetlink.receiver.Receiver`` (API.md §8).

The receiver is imported lazily; with ``--no-fleetlink`` (or no zenoh /
fleetlink) ``receiver`` stays ``None`` and live endpoints return empty.

``ZenohTransport`` never fails for want of a router: it retries the open in
the background (and reconnects after router restarts), so the receiver is
created at once and :attr:`LiveService.status` reports ``waiting`` until the
session is open, then ``connected``.

Live image-store mirror (fleetlink class C6, API.md §13): the receiver writes
the robots' store frames into ``<live_store_root>/<robot>/<store_id>/image_store``
(default ``<output_root>/live``; an empty ``live_store_root`` disables it), a
valid hydra image store that SAM3 search can load while the run goes on.

Lidar (fleetlink C3 ``lidar/points``, off by default): :meth:`LiveService.set_lidar`
asks a robot to stream it, :meth:`LiveService.lidar` is the latest frame.  Each
read (the console polls ``GET .../lidar`` ~1 Hz while the toggle is on) is the
keepalive: unread for ``LIDAR_IDLE_S`` the receiver sends ``enabled: false``
itself, so a closed page does not leave the robot streaming.  The agent also
stops on its own when the base's refreshes stop (``lidar_ttl_s``, 60 s).
"""

from __future__ import annotations

import logging
import os
import threading

from .video_service import VideoRelay

log = logging.getLogger(__name__)
LIDAR_IDLE_S = 15.0  # no lidar GET for this long after an enable: turn it off


class LiveService:
    def __init__(self, settings, hub=None, receiver=None):
        self.settings = settings
        self.hub = hub
        self.receiver = receiver
        self.graph = None  # GraphService, wired by the app
        self.error: str | None = None  # the receiver could not be created at all
        self.camera_timeout_s = 5.0
        self._stop = threading.Event()
        self.video = VideoRelay(self, hub)  # live camera video (API.md §12)

    def start(self) -> None:
        if self.receiver is not None:
            self.receiver.start()
        elif self.settings.fleetlink:
            self._connect()

    def _connect(self) -> bool:
        try:
            from fleetlink.receiver import Receiver
            from fleetlink.transport import ZenohTransport

            # opens at once or keeps retrying in the background (zenohd usually
            # comes up after the console)
            transport = ZenohTransport(
                connect=self.settings.zenoh_connect or "tcp/127.0.0.1:7447"
            )
            receiver = Receiver(
                transport,
                self.on_update,
                store_root=self.store_root,
                lidar_idle_s=LIDAR_IDLE_S,
            )
            receiver.start()
        except Exception as e:  # noqa: BLE001  (e.g. zenoh / fleetlink missing)
            self.error = f"{type(e).__name__}: {e}"
            log.warning("fleetlink unavailable: %s", self.error)
            return False
        self.receiver = receiver
        self.error = None
        opened = getattr(transport, "opened", None)
        if opened is not None and not opened.is_set():
            log.info(
                "fleetlink waiting for a zenoh router at %s",
                ", ".join(getattr(transport, "endpoints", []) or []),
            )
            threading.Thread(
                target=self._log_when_open,
                args=(opened,),
                name="fleetlink-open-wait",
                daemon=True,
            ).start()
        else:
            log.info("fleetlink receiver connected")
        return True

    def _log_when_open(self, opened: threading.Event) -> None:
        while not self._stop.is_set():
            if opened.wait(1.0):
                log.info("fleetlink receiver connected")
                return

    @property
    def store_root(self) -> str | None:
        """Where live store mirrors go (None = mirroring disabled)."""
        root = getattr(self.settings, "live_store_root", None)
        if root is None:
            out = getattr(self.settings, "output_root", None) or "~/adt4_output"
            root = os.path.join(out, "live")
        return os.path.expanduser(root) if root else None

    def store_stats(self) -> dict:
        """``{root, robots: {robot: mirror stats}}`` (robots empty when off)."""
        fn = getattr(self.receiver, "store_stats", None)
        robots = {}
        if fn is not None:
            try:
                robots = fn() or {}
            except Exception:  # noqa: BLE001
                robots = {}
        enabled = getattr(self.receiver, "store", None) is not None
        return {"root": self.store_root if enabled else None, "robots": robots}

    @property
    def status(self) -> dict:
        """``{state, endpoints, error}``; state is ``disabled`` (--no-fleetlink),
        ``error`` (no receiver: ``error`` says why), ``waiting`` (zenoh session
        not open yet: no router reachable) or ``connected``."""
        if self.receiver is None:
            if self.error is not None:
                return {"state": "error", "endpoints": [], "error": self.error}
            return {"state": "disabled", "endpoints": [], "error": None}
        tr = self.receiver.transport
        endpoints = list(getattr(tr, "endpoints", []) or [])
        opened = getattr(tr, "opened", None)
        if opened is not None and not opened.is_set():
            where = ", ".join(endpoints) or "?"
            err = getattr(tr, "open_error", None)
            msg = f"waiting for a zenoh router at {where}"
            return {
                "state": "waiting",
                "endpoints": endpoints,
                "error": f"{msg} ({err})" if err else msg,
            }
        return {"state": "connected", "endpoints": endpoints, "error": None}

    def stop(self) -> None:
        self._stop.set()
        try:
            self.video.close()
        except Exception:  # noqa: BLE001
            pass
        if self.receiver is not None:
            try:
                self.receiver.stop()
                close = getattr(self.receiver.transport, "close", None)
                if close:
                    close()
            except Exception:  # noqa: BLE001
                pass

    @property
    def available(self) -> bool:
        """The receiver exists (live endpoints work, possibly still empty)."""
        return self.receiver is not None

    @property
    def connected(self) -> bool:
        return self.status["state"] == "connected"

    def robots(self) -> list[str]:
        return list(self.receiver.robots()) if self.receiver is not None else []

    def on_update(self, robot: str, kind: str, data) -> None:
        """Receiver callback (receiver thread)."""
        try:
            if kind == "video":
                self.video.on_frame(robot, data)
            elif kind in ("snapshot", "delta", "tiles"):
                if self.graph is not None:
                    self.graph.on_live(robot, kind, data)
                if kind == "snapshot":
                    self._clear_stale_overlays(robot)
            elif kind == "state" and self.hub is not None:
                self.hub.publish(f"robot/{robot}/state", data)
            elif kind == "link" and self.hub is not None:
                self.hub.publish(f"link/{robot}", data)
            elif kind == "grid" and self.hub is not None:
                self.hub.publish(f"grid/{robot}", data)
            elif kind == "paths" and self.hub is not None:
                self.hub.publish(f"robot/{robot}/paths", data)
        except Exception:  # noqa: BLE001
            log.exception("live update %s/%s failed", robot, kind)

    def _clear_stale_overlays(self, robot: str) -> None:
        """After a (re)sync — e.g. the robot's agent restarted with a new epoch —
        the hub still caches the previous incarnation's latest paths/grid and
        would replay them to every new browser; overwrite them with "none"."""
        if self.hub is None or self.receiver is None:
            return
        if self.receiver.paths(robot) is None:
            self.hub.publish(f"robot/{robot}/paths", None)
        if self.receiver.grid(robot) is None:
            self.hub.publish(f"grid/{robot}", {"hash": None})

    def links(self) -> dict:
        if self.receiver is None:
            return {
                "robots": {},
                "transport": self.status,
                "store": {"root": None, "robots": {}},
            }
        return {
            "robots": {r: self.receiver.link(r) for r in self.receiver.robots()},
            "transport": self.status,
            "store": self.store_stats(),
        }

    def state(self, robot: str) -> dict | None:
        if self.receiver is None:
            return None
        try:
            st = self.receiver.state(robot)
        except Exception:  # noqa: BLE001
            return None
        if st is None:
            return None
        st = dict(st)
        st["paths"] = self.paths(robot)
        return st

    def _call(self, name: str, robot: str):
        fn = getattr(self.receiver, name, None) if self.receiver is not None else None
        if fn is None:
            return None
        try:
            return fn(robot)
        except Exception:  # noqa: BLE001
            return None

    def grid(self, robot: str) -> dict | None:
        """Latest occupancy grid wire dict (``data`` = zlib int8 bytes)."""
        return self._call("grid", robot)

    def paths(self, robot: str) -> dict | None:
        return self._call("paths", robot)

    def lidar(self, robot: str) -> dict | None:
        """Latest lidar frame (``Receiver.lidar``), or None."""
        return self._call("lidar", robot)

    def set_lidar(self, robot: str, enabled: bool) -> dict:
        """Turn the robot's lidar stream on / off; ``{request, status}``."""
        req = self.receiver.set_lidar(robot, bool(enabled))
        return {"request": req, "status": self.receiver.lidar_status(robot)}

    def cameras(self, robot: str) -> list[str]:
        return list(self._call("cameras", robot) or [])

    def camera(self, robot: str, name: str, width: int, quality: int) -> dict | None:
        """Blocking C4 request; None on timeout."""
        return self.receiver.camera(
            robot, name, width=width, quality=quality, timeout=self.camera_timeout_s
        )
