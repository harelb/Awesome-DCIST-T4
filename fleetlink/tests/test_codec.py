import dataclasses

import numpy as np
from helpers import states_equal, synthetic_mesh, synthetic_records

from fleetlink import codec
from fleetlink.codec import GraphDelta, GraphState, NodeRecord


def test_records_roundtrip():
    recs, _, _ = synthetic_records(30)
    dec = codec.decode_records(
        codec.unpack(codec.pack(codec.encode_records(list(recs.values()))))
    )
    assert [r.sym for r in dec] == list(recs)
    for r in dec:
        o = recs[r.sym]
        assert (r.layer, r.part, r.label, r.color, r.name, r.active) == (
            o.layer,
            o.part,
            o.label,
            o.color,
            o.name,
            o.active,
        )
        assert np.allclose(r.pos, o.pos, atol=1e-4)
        assert (r.bbox is None) == (o.bbox is None)
        if r.bbox:
            assert np.allclose(r.bbox, o.bbox, atol=1e-4)


def test_delta_and_snapshot_roundtrip():
    recs, edges, ls = synthetic_records(40)
    base = GraphState()
    delta = base.diff(recs, edges, ls)
    wire = codec.unpack(codec.pack(delta.encode()))
    mirror = GraphState()
    mirror.apply(GraphDelta.decode(wire))
    target = GraphState()
    target.records, target.edges, target.labelspaces = recs, edges, ls
    assert states_equal(mirror, target) == (True, "")
    assert mirror.labelspaces == ls

    snap = GraphState()
    snap.load_snapshot(codec.unpack(codec.pack(mirror.snapshot())))
    assert states_equal(snap, target) == (True, "")


def test_diff_apply_converges_over_evolution():
    rng = np.random.default_rng(3)
    recs, edges, ls = synthetic_records(60, seed=1)
    robot, mirror = GraphState(), GraphState()
    for step in range(15):
        # mutate: move some, delete some, add some, rewire edges
        recs = dict(recs)
        syms = list(recs)
        for s in rng.choice(syms, 5, replace=False):
            r = recs[s]
            recs[s] = dataclasses.replace(
                r, pos=tuple(np.asarray(r.pos) + rng.normal(0, 0.5, 3))
            )
        for s in rng.choice(syms, 3, replace=False):
            del recs[s]
        for i in range(4):
            sym = f"O({1000 + step * 10 + i})"
            recs[sym] = NodeRecord(sym, 2, 0, tuple(rng.uniform(-5, 5, 3)))
        live = list(recs)
        edges = {e for e in edges if e[0] in recs and e[1] in recs}
        edges |= {(live[i], live[i + 1]) for i in rng.integers(0, len(live) - 1, 5)}
        delta = robot.diff(recs, edges, ls)
        robot.apply(delta)
        mirror.apply(GraphDelta.decode(codec.unpack(codec.pack(delta.encode()))))
        target = GraphState()
        target.records, target.edges = recs, edges
        assert states_equal(mirror, target) == (True, ""), step
        assert robot.diff(recs, edges, ls).empty()


def test_split_delta_closest_first():
    recs = {
        f"O({i})": NodeRecord(f"O({i})", 2, 0, (float(i), 0.0, 0.0)) for i in range(10)
    }
    edges = {("O(0)", "O(1)"), ("O(1)", "O(9)"), ("O(8)", "O(9)")}
    delta = GraphState().diff(recs, edges)
    now, later = codec.split_delta(delta, 3, origin=(9.2, 0, 0))
    assert [r.sym for r in now.upserts] == ["O(9)", "O(8)", "O(7)"]
    assert now.edges_add == [("O(8)", "O(9)")]  # only edges between sent nodes go now
    assert set(later.edges_add) == {("O(0)", "O(1)"), ("O(1)", "O(9)")}
    mirror = GraphState()
    mirror.apply(now)
    mirror.apply(later)
    target = GraphState()
    target.records, target.edges = recs, edges
    assert states_equal(mirror, target) == (True, "")


def test_tile_quantization_error_below_1cm():
    pos, col, faces = synthetic_mesh(900, extent=37.3, seed=5)
    pos = pos + np.float32(0.00371)  # off-grid
    tiles = codec.tile_mesh(pos, col, faces, tile_size=8.0)
    assert len(tiles) > 4
    assert sum(t["nf"] for t in tiles.values()) == len(faces)
    err = 0.0
    tri_orig = pos[faces].mean(axis=1)
    for t in codec.unpack(codec.pack(tiles)).values():
        dpos, dcol, didx = codec.decode_tile(t)
        assert didx.max() < len(dpos)
        tri_dec = dpos[didx].mean(axis=1)
        # match each decoded triangle centroid to the nearest original centroid
        d = np.linalg.norm(tri_dec[:, None, :] - tri_orig[None, :, :], axis=2)
        err = max(err, float(d.min(axis=1).max()))
    assert err <= 0.01
    # vertex-level: every decoded vertex is within 1 cm of an original vertex
    t = next(iter(tiles.values()))
    dpos, _, _ = codec.decode_tile(t)
    d = np.linalg.norm(dpos[:, None, :] - pos[None, :, :], axis=2).min(axis=1)
    assert d.max() <= 0.01
    assert codec.tile_manifest(tiles) == {k: v["hash"] for k, v in tiles.items()}


def test_pack_is_thread_safe():
    # zstd (de)compressors are not thread-safe; the agent packs concurrently from
    # its tick, video and query threads.  A shared instance corrupted the heap
    # ("free(): corrupted unsorted chunks") under live video + mapping.
    import threading

    owners = {}

    def worker(i):
        owners[i] = (codec._zc(), codec._zd())  # keep alive: ids could be reused
        blob = {"i": i, "data": bytes([i % 256]) * 50_000}
        for _ in range(200):
            assert codec.unpack(codec.pack(blob)) == blob

    threads = [threading.Thread(target=worker, args=(i,)) for i in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    assert len({id(c) for c, _ in owners.values()}) == 8  # one per thread
    assert len({id(d) for _, d in owners.values()}) == 8


def _place(sym, n=24, seed=0, **kw):
    rng = np.random.default_rng(seed)
    radii = tuple(round(float(v), 2) for v in rng.uniform(0.05, 4.0, n))
    states = tuple(int(v) for v in rng.integers(0, 4, n))
    return NodeRecord(sym, 3, 1, (1.0, 2.0, 0.5), radii=radii, states=states, **kw)


def test_boundary_roundtrip_and_old_peer_compat():
    plain = NodeRecord("O(1)", 2, 0, (0.0, 0.0, 0.0))
    places = [_place(f"t({i})", n=20 + i, seed=i) for i in range(3)]
    enc = codec.encode_records([plain, *places])
    dec = codec.decode_records(codec.unpack(codec.pack(enc)))
    assert dec[0].radii is None and dec[0].states is None
    for d, o in zip(dec[1:], places):
        assert d.states == o.states
        assert np.allclose(d.radii, o.radii, atol=0.006)
        assert d.key() == o.key()
    # graphs without boundaries keep the old wire keys exactly
    assert "nrad" not in codec.encode_records([plain])
    # an old payload (no boundary keys) still decodes
    old = {k: v for k, v in enc.items() if k not in ("nrad", "radii", "states")}
    assert all(r.radii is None for r in codec.decode_records(old))


def test_boundary_quantised_and_clamped():
    r = NodeRecord("t(1)", 3, 1, (0, 0, 0), radii=(0.123, -1.0, 1e6, float("nan")))
    (d,) = codec.decode_records(codec.encode_records([r]))
    assert d.radii == (0.12, 0.0, 655.35, 0.0)
    assert d.states == (0, 0, 0, 0)  # missing states pad as UNKNOWN


def test_boundary_delta_only_on_change():
    st = GraphState()
    recs = {"t(1)": _place("t(1)"), "t(2)": _place("t(2)", seed=2)}
    st.apply(st.diff(recs, set()))
    # sub-cm radius jitter: no resend
    jit = dataclasses.replace(recs["t(1)"], radii=tuple(v + 0.001 for v in recs["t(1)"].radii))
    assert st.diff({**recs, "t(1)": jit}, set()).empty()
    # a state flip: resend just that node, and the mirror converges
    flipped = list(recs["t(2)"].states)
    flipped[0] = (flipped[0] + 1) % 4
    new = {**recs, "t(2)": dataclasses.replace(recs["t(2)"], states=tuple(flipped))}
    delta = st.diff(new, set())
    assert [r.sym for r in delta.upserts] == ["t(2)"]
    mirror = GraphState()
    mirror.load_snapshot(codec.unpack(codec.pack(st.snapshot())))
    mirror.apply(GraphDelta.decode(codec.unpack(codec.pack(delta.encode()))))
    assert mirror.records["t(2)"].states == tuple(flipped)


def test_node_record_reads_trav_attributes():
    class State:
        def __init__(self, v):
            self.value = v

    class Attrs:
        position = (1.0, 2.0, 3.0)
        radii = [0.5, 1.004, 2.0]
        states = [State(1), State(2)]

    class Node:
        attributes = Attrs()
        layer = 3
        id = "t(7)"

    rec = codec.node_record(Node())
    assert rec.radii == (0.5, 1.0, 2.0)
    assert rec.states == (1, 2, 0)
