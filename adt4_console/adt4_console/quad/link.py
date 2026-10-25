"""The console side of the Penn quad link (API.md §16).

``send_waypoint`` keeps heracles_agents' behaviour (PUSH *bind*, ``send_pyobj``
of a PennQuadCommand dict) but holds one socket open instead of binding per
call.  ``request_image`` is a REQ client of the (mock-schema) image server with
the ZMQ "lazy pirate" recovery: a REQ socket that timed out is closed and
replaced, and the request retried.
"""

from __future__ import annotations

import logging
import threading
import time

from . import protocol

log = logging.getLogger(__name__)


class QuadUnavailable(Exception):
    pass


class QuadLink:
    def __init__(self, cmd_uri: str | None, image_uri: str | None, frame: protocol.QuadFrame,
                 timeout_s: float = 3.0, retries: int = 2):  # fmt: skip
        self.cmd_uri = cmd_uri
        self.image_uri = image_uri
        self.frame = frame
        self.timeout_s = timeout_s
        self.retries = retries
        self.connect_wait_s = 1.0
        self._lock = threading.Lock()
        self._ctx = None
        self._push = None
        self._req = None
        self.last_image: dict | None = None  # header of the newest image (pose = telemetry)
        self.last_command: dict | None = None

    @property
    def configured(self) -> bool:
        return bool(self.cmd_uri or self.image_uri)

    def info(self) -> dict:
        pose = None
        if self.last_image and self.last_image.get("pose"):
            p = self.last_image["pose"]
            mx, my = self.frame.quad_to_map(p["x"], p["y"])
            pose = {"x": mx, "y": my, "z": p.get("z"), "yaw": self.frame.yaw_quad_to_map(p.get("yaw", 0.0)),
                    "stamp_ns": self.last_image.get("stamp_ns"), "t": self.last_image.get("t")}  # fmt: skip
        return {
            "cmd_uri": self.cmd_uri,
            "image_uri": self.image_uri,
            "local_utm_origin": list(self.frame.origin),
            "map_offset": list(self.frame.offset),
            "last_command": self.last_command,
            "pose": pose,
        }

    def _context(self):
        if self._ctx is None:
            try:
                import zmq
            except ImportError as e:
                raise QuadUnavailable(f"pyzmq unavailable: {e}") from None
            self._ctx = zmq.Context.instance()
        return self._ctx

    # ------------------------------------------------------------ waypoints
    def start(self) -> None:
        """Bind the command socket now, so the quad can connect before the first waypoint."""
        if self.cmd_uri:
            try:
                with self._lock:
                    self._bind()
            except Exception as e:  # noqa: BLE001 — never block console startup
                log.warning("quad command socket %s: %s", self.cmd_uri, e)

    def stop(self) -> None:
        self.close()

    def _bind(self):
        import zmq

        if self._push is None:
            s = self._context().socket(zmq.PUSH)
            s.setsockopt(zmq.LINGER, 0)
            s.setsockopt(zmq.SNDHWM, 4)
            s.bind(self.cmd_uri)
            self._push = s
        return self._push

    def send_waypoint(self, x: float, y: float, query: str = "") -> dict:
        """Map-frame ``(x, y)`` → PennQuadCommand in the quad frame."""
        if not self.cmd_uri:
            raise QuadUnavailable("no quad command URI configured (--quad-cmd-uri)")
        import zmq

        qx, qy = self.frame.map_to_quad(x, y)
        cmd = protocol.quad_command(qx, qy, 0, query)
        with self._lock:
            push = self._bind()
            # PUSH has no peer until the quad connected: wait a little, then give up
            if not push.poll(int(self.connect_wait_s * 1000), zmq.POLLOUT):
                raise QuadUnavailable(f"the quad is not connected to {self.cmd_uri}")
            try:
                push.send_pyobj(cmd, flags=zmq.NOBLOCK)
            except zmq.Again:
                raise QuadUnavailable("the quad is not connected to the command socket") from None
        self.last_command = {"map": [x, y], "quad": cmd, "t": time.time()}
        return {"ok": True, "sent": cmd}

    # --------------------------------------------------------------- images
    def request_image(self, stamp_ns: int | None = None) -> tuple[dict, bytes]:
        """(header with ``pose_map`` added, jpeg)."""
        if not self.image_uri:
            raise QuadUnavailable("no quad image URI configured (--quad-image-uri)")
        import zmq

        with self._lock:
            for attempt in range(self.retries + 1):
                if self._req is None:
                    s = self._context().socket(zmq.REQ)
                    s.setsockopt(zmq.LINGER, 0)
                    s.connect(self.image_uri)
                    self._req = s
                self._req.send_multipart(protocol.image_request(stamp_ns))
                if self._req.poll(int(self.timeout_s * 1000), zmq.POLLIN):
                    frames = self._req.recv_multipart()
                    break
                log.info("quad image request timed out (attempt %d)", attempt + 1)
                self._req.close()
                self._req = None
            else:
                raise QuadUnavailable(f"quad image server {self.image_uri} not answering")
        header, jpeg = protocol.parse_image_reply(frames)
        if jpeg is None:
            raise QuadUnavailable(f"quad image server: {header.get('error')}")
        header["t"] = time.time()
        p = header["pose"]
        mx, my = self.frame.quad_to_map(p["x"], p["y"])
        header["pose_map"] = {"x": mx, "y": my, "z": float(p.get("z", 0.0)),
                              "yaw": self.frame.yaw_quad_to_map(float(p.get("yaw", 0.0)))}  # fmt: skip
        self.last_image = header
        return header, jpeg

    def close(self):
        with self._lock:
            for s in (self._push, self._req):
                if s is not None:
                    s.close()
            self._push = self._req = None
