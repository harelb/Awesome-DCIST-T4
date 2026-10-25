"""Mock Penn quadrotor: the other end of :mod:`.link` for offline rehearsals.

* connects a PULL socket to the console's command URI and flies (kinematically)
  toward each PennQuadCommand waypoint;
* binds a REP image server (mock schema, :mod:`.protocol`) answering with a
  nadir image rendered from the satellite tile cache under the quad, with
  planted mock objects painted in.

    python -m adt4_console.quad.mock_server --anchor <run>/geo_anchor.json \\
        --cmd tcp://127.0.0.1:6555 --image tcp://127.0.0.1:6556 \\
        --start 10 -20 --plant crate:35,-60 --plant car:-20,-90
"""

from __future__ import annotations

import argparse
import logging
import math
import threading
import time

from . import camera, protocol

log = logging.getLogger("mock_penn_quad")


class MockQuad:
    def __init__(self, frame: protocol.QuadFrame, mosaic, planted, start=(0.0, 0.0),
                 speed: float = 4.0, altitude: float = 20.0):  # fmt: skip
        self.frame = frame
        self.mosaic = mosaic
        self.planted = list(planted)
        self.x, self.y = float(start[0]), float(start[1])  # map frame
        self.yaw = 0.0
        self.altitude = altitude
        self.speed = speed
        self.goal = None
        self.lock = threading.Lock()

    def command(self, cmd: dict) -> None:
        gx, gy = self.frame.quad_to_map(cmd["x"], cmd["y"])
        with self.lock:
            self.goal = (gx, gy)
        log.info("waypoint quad (%.1f, %.1f) = map (%.1f, %.1f) query=%r",
                 cmd["x"], cmd["y"], gx, gy, cmd.get("query", ""))  # fmt: skip

    def step(self, dt: float) -> None:
        with self.lock:
            if self.goal is None:
                return
            dx, dy = self.goal[0] - self.x, self.goal[1] - self.y
            d = math.hypot(dx, dy)
            if d < 0.05:
                self.goal = None
                return
            self.yaw = math.atan2(dy, dx)
            k = min(1.0, self.speed * dt / d)
            self.x += dx * k
            self.y += dy * k

    def image(self) -> list[bytes]:
        with self.lock:
            pose_map = {"x": self.x, "y": self.y, "z": self.altitude, "yaw": self.yaw}
        jpeg, _ = camera.render_nadir(pose_map, self.mosaic, self.planted)
        qx, qy = self.frame.map_to_quad(self.x, self.y)
        header = {
            "stamp_ns": time.time_ns(),
            "frame": "quad",
            "pose": {"x": qx, "y": qy, "z": self.altitude, "yaw": self.frame.yaw_map_to_quad(self.yaw)},
            "K": camera.intrinsics(),
            "width": camera.WIDTH,
            "height": camera.HEIGHT,
            "nadir": True,
        }
        return protocol.image_reply(header, jpeg)


def serve(quad: MockQuad, cmd_uri: str, image_uri: str, stop: threading.Event) -> None:
    import zmq

    ctx = zmq.Context.instance()
    pull = ctx.socket(zmq.PULL)
    pull.setsockopt(zmq.LINGER, 0)
    pull.connect(cmd_uri)
    rep = ctx.socket(zmq.REP)
    rep.setsockopt(zmq.LINGER, 0)
    rep.bind(image_uri)
    poller = zmq.Poller()
    poller.register(pull, zmq.POLLIN)
    poller.register(rep, zmq.POLLIN)
    last = time.monotonic()
    try:
        while not stop.is_set():
            events = dict(poller.poll(50))
            now = time.monotonic()
            quad.step(now - last)
            last = now
            if pull in events:
                # the console's side pickles (heracles_agents compatibility); this
                # mock only ever talks to a console we started ourselves
                quad.command(pull.recv_pyobj())
            if rep in events:
                frames = rep.recv_multipart()
                try:
                    import json

                    req = json.loads(frames[0].decode())
                    if req.get("op") != "get_image":
                        rep.send_multipart(protocol.error_reply(f"unknown op {req.get('op')!r}"))
                    else:
                        rep.send_multipart(quad.image())
                except Exception as e:  # noqa: BLE001
                    rep.send_multipart(protocol.error_reply(str(e)))
    finally:
        pull.close()
        rep.close()


def _plant(spec: str) -> dict:
    label, _, xy = spec.partition(":")
    x, y = (float(v) for v in xy.split(","))
    return {"label": label.strip(), "pos": [x, y], "size_m": 1.5}


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--anchor", help="geo_anchor.json of the map (no anchor: map frame = quad frame)")
    p.add_argument("--tiles-dir", default="~/.adt4_console/tiles")
    p.add_argument("--layer", default="esri_world_imagery")
    p.add_argument("--zoom", type=int, default=19)
    p.add_argument("--cmd", default="tcp://127.0.0.1:6555", help="the console's PUSH (bound) URI")
    p.add_argument("--image", default="tcp://127.0.0.1:6556", help="image server URI to bind")
    p.add_argument("--utm-origin", type=float, nargs=2, default=(0.0, 0.0))
    p.add_argument("--map-offset", type=float, nargs=2, default=(0.0, 0.0))
    p.add_argument("--start", type=float, nargs=2, default=(0.0, 0.0), help="map x y")
    p.add_argument("--speed", type=float, default=4.0)
    p.add_argument("--altitude", type=float, default=20.0)
    p.add_argument("--plant", action="append", default=[], metavar="LABEL:X,Y", help="mock object (map frame)")
    args = p.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")

    from ..geo import GeoAnchor, TileCache

    anchor = GeoAnchor.load(args.anchor) if args.anchor else None
    frame = protocol.QuadFrame(anchor, args.utm_origin, args.map_offset)
    mosaic = camera.TileMosaic(TileCache(args.tiles_dir), args.layer, anchor, args.zoom) if anchor else None
    quad = MockQuad(frame, mosaic, [_plant(s) for s in args.plant], args.start, args.speed, args.altitude)
    stop = threading.Event()
    log.info("mock quad: commands from %s, images on %s", args.cmd, args.image)
    try:
        serve(quad, args.cmd, args.image, stop)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
