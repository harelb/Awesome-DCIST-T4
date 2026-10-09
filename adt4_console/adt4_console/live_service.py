"""Console side of fleetlink: owns the ``fleetlink.receiver.Receiver`` (API.md §8).

The receiver is imported lazily; with ``--no-fleetlink`` (or no zenoh /
fleetlink) ``receiver`` stays ``None`` and live endpoints return empty.
"""

from __future__ import annotations

import logging
import threading

log = logging.getLogger(__name__)


class LiveService:
    def __init__(self, settings, hub=None, receiver=None):
        self.settings = settings
        self.hub = hub
        self.receiver = receiver
        self.graph = None  # GraphService, wired by the app
        self.error: str | None = None
        self.retry_s = 10.0
        self.camera_timeout_s = 5.0
        self._stop = threading.Event()

    def start(self) -> None:
        if self.receiver is not None:
            self.receiver.start()
        elif self.settings.fleetlink and not self._connect():
            # zenohd usually comes up after the console (fleet zenohd / base-station
            # session): keep retrying in the background.
            threading.Thread(
                target=self._retry, name="fleetlink-connect", daemon=True
            ).start()

    def _connect(self) -> bool:
        try:
            from fleetlink.receiver import Receiver
            from fleetlink.transport import ZenohTransport

            transport = ZenohTransport(
                connect=self.settings.zenoh_connect or "tcp/127.0.0.1:7447"
            )
            receiver = Receiver(transport, self.on_update)
            receiver.start()
        except Exception as e:  # noqa: BLE001
            self.error = f"{type(e).__name__}: {e}"
            log.warning("fleetlink unavailable: %s", self.error)
            return False
        self.receiver = receiver
        self.error = None
        log.info("fleetlink receiver connected")
        return True

    def _retry(self) -> None:
        while not self._stop.wait(self.retry_s):
            if self._connect():
                return

    def stop(self) -> None:
        self._stop.set()
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
        return self.receiver is not None

    def robots(self) -> list[str]:
        return list(self.receiver.robots()) if self.receiver is not None else []

    def on_update(self, robot: str, kind: str, data) -> None:
        """Receiver callback (receiver thread)."""
        try:
            if kind in ("snapshot", "delta", "tiles"):
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
            return {"robots": {}}
        return {"robots": {r: self.receiver.link(r) for r in self.receiver.robots()}}

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

    def cameras(self, robot: str) -> list[str]:
        return list(self._call("cameras", robot) or [])

    def camera(self, robot: str, name: str, width: int, quality: int) -> dict | None:
        """Blocking C4 request; None on timeout."""
        return self.receiver.camera(
            robot, name, width=width, quality=quality, timeout=self.camera_timeout_s
        )
