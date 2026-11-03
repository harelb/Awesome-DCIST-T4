"""Object images for the map search + Inspect: ``/imaged`` (which nodes have
thumbnails), image variants (``w=`` resize, ``mask=1`` overlay, ``crop=1``),
the render cache, Neo4j Observation rows and live mirror stores."""

import io
import shutil

import numpy as np
import pytest

from .conftest import make_client
from .synthetic import build_run

pytest.importorskip("spark_dsg")
PIL = pytest.importorskip("PIL.Image")

W, H = 64, 48


def _png_free_jpeg(path, value):
    img = np.zeros((H, W, 3), np.uint8)
    img[...] = value
    PIL.fromarray(img).save(path, "JPEG", quality=95)


def _mask():
    m = np.zeros((H, W), bool)
    m[10:30, 20:44] = True  # 24 x 20 box
    return m


def build_store(store_dir, src):
    """O(0): two kept observations (best one masked); O(2): one unmasked; O(1): none."""
    from hydra_image_store import FrameKind, ImageStore, make_node_id

    src.mkdir(parents=True, exist_ok=True)
    w = ImageStore.open_writer(store_dir, robot_id=0)
    sid = w.add_sensor("cam", W, H, 50.0, 50.0, W / 2, H / 2)
    fids = []
    for k, v in enumerate((40, 90, 160)):
        f = src / f"{k}.jpg"
        _png_free_jpeg(f, v)
        fids.append(w.add_frame_ref(sid, k + 1, kind=FrameKind.OBSERVATION, rgb=f, link="copy"))
    w.bind_track(5, make_node_id("O", 0))
    w.add_observation(5, fids[0], timestamp_ns=1, score=0.9, kept=True, mask=_mask(), bbox2d=(20, 10, 44, 30))
    w.add_observation(5, fids[1], timestamp_ns=2, score=0.4, kept=True)
    w.add_observation(5, fids[2], timestamp_ns=3, score=0.1, kept=False)  # not kept: no image
    w.bind_track(6, make_node_id("O", 2))
    w.add_observation(6, fids[2], timestamp_ns=3, score=0.7, kept=True)
    w.close()


@pytest.fixture
def run_dir(tmp_path):
    run = build_run(tmp_path)
    shutil.rmtree(run / "image_store")
    build_store(run / "image_store", tmp_path / "imgs")
    return run


@pytest.fixture
def gclient(settings, run_dir):
    settings.maps = {"syn": str(run_dir)}
    client, _ = make_client(settings)
    with client:
        yield client


def _img(r):
    assert r.status_code == 200, r.text
    assert r.headers["content-type"] == "image/jpeg"
    return np.asarray(PIL.open(io.BytesIO(r.content)).convert("RGB")).astype(int)


def test_imaged_lists_objects_with_kept_images(gclient):
    r = gclient.get("/api/graph/file:syn/imaged").json()
    assert r["syms"] == ["O(0)", "O(2)"]
    assert gclient.get("/api/graph/file:nope/imaged").status_code == 404


def test_node_detail_reports_masks(gclient):
    d = gclient.get("/api/graph/file:syn/node/O(0)").json()
    assert d["n_images"] == 2 and d["masks"] == [True, False]
    assert [i["kind"] for i in d["image_info"]] == [None, None]  # object images, not agent frames
    assert gclient.get("/api/graph/file:syn/node/O(1)").json()["masks"] == []


def test_image_variants(gclient):
    base = "/api/graph/file:syn/image/O(0)/0"
    raw = _img(gclient.get(base))
    assert raw.shape == (H, W, 3)
    small = _img(gclient.get(base + "?w=32"))
    assert small.shape == (24, 32, 3)
    # mask overlay: inside the mask turns orange, outside stays grey
    ov = _img(gclient.get(base + "?mask=1"))
    assert ov.shape == (H, W, 3)
    inside, outside = ov[20, 32], ov[40, 5]
    assert inside[0] > inside[2] + 40, inside
    assert abs(outside - raw[40, 5]).max() < 12
    edge = ov[10, 32]  # mask boundary row: solid outline colour
    assert edge[0] > 200 and edge[2] < 60, edge
    # crop: around the mask (plus context), narrower than the frame
    cr = _img(gclient.get(base + "?crop=1&mask=1"))
    assert cr.shape[0] <= H and cr.shape[1] < W
    # unmasked observation: mask=1 is a no-op overlay; crop without a mask/box = whole
    plain = _img(gclient.get("/api/graph/file:syn/image/O(0)/1?mask=1&crop=1&w=40"))
    assert plain.shape == (30, 40, 3)
    assert gclient.get("/api/graph/file:syn/image/O(0)/2?w=32").status_code == 404
    assert gclient.get("/api/graph/file:syn/image/O(1)/0?w=32").status_code == 404
    assert gclient.get(base + "?w=0").status_code == 422


def test_render_cache_reuses_variants(gclient):
    from adt4_console.routes import graph as groutes

    n0 = len(groutes._renders)
    a = gclient.get("/api/graph/file:syn/image/O(2)/0?w=20&mask=1").content
    b = gclient.get("/api/graph/file:syn/image/O(2)/0?w=20&mask=1").content
    assert a == b and len(groutes._renders) == n0 + 1


def test_render_cache_byte_bound():
    from adt4_console.image_render import RenderCache

    c = RenderCache(max_bytes=10)
    c.put("a", b"123456")
    c.put("b", b"123456")
    assert c.get("a") is None and c.get("b") == b"123456"


def test_crop_box_square_and_clamped():
    from adt4_console.image_render import crop_box

    m = np.zeros((100, 200), bool)
    m[0:10, 190:200] = True  # in the corner
    x0, y0, x1, y1 = crop_box(m.shape, m, None)
    assert x1 == 200 and y0 == 0 and (x1 - x0) == (y1 - y0)
    assert crop_box(m.shape, None, None) is None
    assert crop_box(m.shape, None, (10, 10, 20, 20)) is not None


def test_neo4j_observation_rows_carry_masks(tmp_path):
    from adt4_console.graph_service import _neo4j_image
    from hydra_image_store.codec import rle_encode

    rgb = tmp_path / "f.jpg"
    _png_free_jpeg(rgb, 100)
    im = _neo4j_image({"p": str(rgb), "rle": rle_encode(_mask()), "bb": [20, 10, 44, 30]})
    assert im.has_mask and im.bbox2d == (20, 10, 44, 30)
    assert im.mask(shape=(H, W)).sum() == _mask().sum()
    mp = tmp_path / "m.png"
    PIL.fromarray(_mask().astype(np.uint8) * 255).save(mp)
    im2 = _neo4j_image({"p": str(rgb), "mp": str(mp)})
    assert im2.has_mask and (im2.mask() == _mask()).all()
    assert not _neo4j_image({"p": str(rgb)}).has_mask


def test_live_source_reads_mirror_store(tmp_path):
    from adt4_console.graph_service import LiveSource

    class _Live:
        store_root = str(tmp_path / "live")
        receiver = None

    src = LiveSource("hamilton", _Live())
    assert src.image_store() is None  # nothing mirrored yet: no images, no error
    assert src.node_detail_extra("O(0)", None) == ({}, [])
    build_store(tmp_path / "live" / "hamilton" / "s1" / "image_store", tmp_path / "imgs")
    src._mirror = None  # skip the rescan interval
    _, images = src.node_detail_extra("O(0)", None)
    assert len(images) == 2 and images[0].has_mask
    assert src._imaged_syms() == {"O(0)", "O(2)"}
    off = LiveSource("hamilton", type("L", (), {"store_root": None, "receiver": None})())
    assert off.image_store() is None


def test_file_source_indexes_a_run_with_only_raw_logs(settings, tmp_path):
    """Hydra/Khronos write keyframe zips + track logs (no store): the file source opens
    the run with ImageStore.open(run), which indexes it once, and shows its images."""
    import os
    import pathlib
    import sys

    os.environ.setdefault("OPENCV_IO_ENABLE_OPENEXR", "1")
    his = pytest.importorskip("hydra_image_store")
    pytest.importorskip("hydra_image_store.tail")
    pytest.importorskip("yaml")
    pytest.importorskip("cv2")
    tests = pathlib.Path(his.__file__).resolve().parents[2] / "tests"
    if not (tests / "raw_logs.py").is_file():
        pytest.skip("hydra_image_store test writers not found")
    sys.path.insert(0, str(tests))
    import raw_logs as R

    run = R.build_run(tmp_path / "raw_run")
    assert not (run / "image_store").exists()
    settings.maps = {"raw": str(run)}
    client, _ = make_client(settings)
    with client:
        syms = client.get("/api/graph/file:raw/imaged").json()["syms"]
        assert syms and (run / "image_store" / "store.sqlite").is_file()
        from hydra_image_store import ImageStore

        with ImageStore.open(run / "image_store", index=False) as s:
            expect = {f"O({n & ((1 << 56) - 1)})" for n in s.object_nodes()
                      if s.observations(n, kept_only=True)}  # fmt: skip
        assert set(syms) == expect
        d = client.get(f"/api/graph/file:raw/node/{syms[0]}").json()
        assert d["n_images"] >= 1
        assert _img(client.get(f"/api/graph/file:raw/image/{syms[0]}/0")).ndim == 3


def test_live_source_opens_only_the_current_mirror(tmp_path):
    """Node ids restart with every Hydra session: only the store session the receiver
    mirrors now (else the newest mirror) is read, never an earlier one."""
    import os
    import time

    from adt4_console.graph_service import LiveSource
    from hydra_image_store import ImageStore

    old = tmp_path / "live" / "hamilton" / "old" / "image_store"
    build_store(old, tmp_path / "imgs_old")
    new = tmp_path / "live" / "hamilton" / "new" / "image_store"
    with ImageStore.open_writer(new, robot_id=0) as w:  # current session: no images yet
        w.add_sensor("cam", W, H, 50.0, 50.0, W / 2, H / 2)
    t = time.time()
    os.utime(old.parent, (t - 100, t - 100))  # the older session's mirror dir

    class _Rx:
        def __init__(self, path):
            self.path = path

        def store_stats(self, robot):
            return {"path": str(self.path)} if self.path else {}

    live = type("L", (), {"store_root": str(tmp_path / "live"), "receiver": _Rx(new)})()
    src = LiveSource("hamilton", live)
    assert src.node_detail_extra("O(0)", None) == ({}, [])  # old session's O(0) not shown
    live.receiver = _Rx(old)
    src._mirror = None
    _, images = src.node_detail_extra("O(0)", None)
    assert len(images) == 2
    live.receiver = None  # no receiver: the most recently written mirror
    src._mirror = None
    assert src.node_detail_extra("O(0)", None) == ({}, [])


def test_store_with_sub_keyframes_is_not_opened_nor_modified(tmp_path, caplog):
    """Stores written before hydra_image_store dropped sub-keyframes (frame kind 1)
    stay hidden until `hydra-image-store-adt4 convert` ran; the console never writes."""
    import hashlib
    import sqlite3

    from hydra_image_store import FrameKind, ImageStore

    from adt4_console.graph_service import needs_convert, open_image_store

    run = tmp_path / "run"
    img = tmp_path / "x.jpg"
    img.write_bytes(b"\xff\xd8\xff\xd9")
    w = ImageStore.open_writer(run / "image_store", robot_id=0)
    sid = w.add_sensor("cam", 4, 4, 1.0, 1.0, 2.0, 2.0)
    w.add_frame_ref(sid, 1, kind=FrameKind.KEYFRAME, rgb=img, link="copy")
    w.add_frame_ref(sid, 2, kind=FrameKind.KEYFRAME, rgb=img, link="copy")
    w.close()
    db = run / "image_store" / "store.sqlite"
    s = open_image_store(run, index=False)
    assert not needs_convert(run) and s is not None
    s.close()
    with sqlite3.connect(db) as c:
        c.execute("UPDATE frames SET kind = 1 WHERE timestamp_ns = 2")
    files = sorted(p for p in db.parent.glob("store.sqlite*") if not p.name.endswith("-shm"))
    digest = [hashlib.sha256(p.read_bytes()).hexdigest() for p in files]
    assert needs_convert(run) and needs_convert(run / "image_store") and needs_convert(db)
    with caplog.at_level("WARNING"):
        assert open_image_store(run) is None
    assert "hydra-image-store-adt4 convert" in caplog.text
    assert [hashlib.sha256(p.read_bytes()).hexdigest() for p in files] == digest
