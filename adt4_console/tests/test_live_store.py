"""Live image-store mirror (fleetlink C6) in the console: ``/api/live`` reports
the per-robot mirror (frames, bytes, backlog, path under ``live_store_root``)."""

import dataclasses

import numpy as np
import pytest

from .conftest import make_client

transfer = pytest.importorskip("hydra_image_store.transfer")
cv2 = pytest.importorskip("cv2")


def _robot_store(root, n):
    from hydra_image_store import FrameKind, ImageStore

    src = root.parent / "src"
    src.mkdir(parents=True)
    w = ImageStore.open_writer(root, robot_id=1)
    sid = w.add_sensor("camera", 32, 24, 30.0, 30.0, 16, 12, 0.001, np.eye(4))
    ids = []
    for i in range(n):
        ts = 1_000_000_000 + i * 100_000_000
        rp, dp = src / f"{ts}_rgb.jpg", src / f"{ts}_depth.png"
        cv2.imwrite(str(rp), np.full((24, 32, 3), i * 20, np.uint8))
        cv2.imwrite(str(dp), np.full((24, 32), 1000 + i, np.uint16))
        ids.append(
            w.add_frame_ref(sid, ts, kind=FrameKind.KEYFRAME, rgb=rp, depth=dp, link="copy")
        )
    w.close()
    return ids


def test_live_reports_store_mirror(settings, tmp_path):
    from fleetlink.agent import AgentCore
    from fleetlink.receiver import Receiver
    from fleetlink.store_mirror import StoreMirror
    from fleetlink.transport import LinkEmulator, VirtualClock

    st = dataclasses.replace(settings, live_store_root=str(tmp_path / "live"))
    ids = _robot_store(tmp_path / "robot" / "image_store", 3)
    clock = VirtualClock()
    link = LinkEmulator(clock=clock, bandwidth=None, latency=0.01)
    core = AgentCore(
        "hamilton", link.robot,
        {"store_path": str(tmp_path / "robot" / "image_store"), "store_poll_s": 0.2},
        clock=clock,
    )  # fmt: skip
    rx = Receiver(
        link.base, clock=clock, threaded=False,
        store_mirror=StoreMirror(tmp_path / "live", clock=clock, threaded=False),
    )  # fmt: skip
    client, svc = make_client(st, receiver=rx)
    with client:
        assert svc.live.store_root == str(tmp_path / "live")
        for _ in range(80):
            clock.advance(0.05)
            core.tick()
            link.advance()
            rx.tick()
        live = client.get("/api/live").json()
        mirror = live["store"]["robots"]["hamilton"]
        assert mirror["frames"] == 3 and mirror["watermark"] == ids[-1]
        assert mirror["backlog"] == 0 and mirror["bytes"] > 0
        assert mirror["path"].startswith(str(tmp_path / "live" / "hamilton"))
        assert live["robots"]["hamilton"]["store"]["frames"] == 3
        assert live["robots"]["hamilton"]["budgets"]["C6"] == 150_000
        r = client.post("/api/live/hamilton/budget", json={"cls": "C6", "bps": 1000})
        assert r.status_code == 200


def test_store_root_default_and_disabled(settings):
    from adt4_console.live_service import LiveService

    assert LiveService(settings).store_root.endswith("/out/live")
    off = dataclasses.replace(settings, live_store_root="")
    assert LiveService(off).store_root is None
