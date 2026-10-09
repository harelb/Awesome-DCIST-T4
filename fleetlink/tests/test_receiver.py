from helpers import RecordingTransport, env_bytes, synthetic_records

from fleetlink import codec
from fleetlink.codec import GraphState
from fleetlink.receiver import Receiver
from fleetlink.transport import VirtualClock


def _rx():
    tr = RecordingTransport()
    clock = VirtualClock(100.0)
    events = []
    rx = Receiver(
        tr, lambda r, k, d: events.append((r, k, d)), clock=clock, threaded=False
    )
    rx.start()
    return tr, clock, rx, events


def _reqs(tr, op=None):
    out = [codec.unpack(d)["p"] for k, d, c in tr.sent if k.endswith("/ctl/req")]
    return [r for r in out if op is None or r["op"] == op]


def _delta(records, edges=()):
    return {"kind": "delta", **GraphState().diff(records, set(edges)).encode()}


def test_gap_triggers_rate_limited_resync():
    tr, clock, rx, events = _rx()
    recs, _, _ = synthetic_records(6)
    items = list(recs.items())
    rx._on_msg(
        "adt4/r/dsg/delta",
        env_bytes("r", "dsg/delta", 0, "E1", _delta(dict(items[:2]))),
    )
    assert len(_reqs(tr, "resync")) == 1  # first contact = epoch change
    ack = {"id": _reqs(tr, "resync")[0]["id"], "ok": True, "result": {}}
    rx._on_msg("adt4/r/ctl/ack", env_bytes("r", "ctl/ack", 0, "E1", ack))
    clock.advance(10.0)
    rx._on_msg(
        "adt4/r/dsg/delta",
        env_bytes("r", "dsg/delta", 1, "E1", _delta(dict(items[2:3]))),
    )
    assert len(_reqs(tr, "resync")) == 1  # contiguous: no resync
    rx._on_msg(
        "adt4/r/dsg/delta",
        env_bytes("r", "dsg/delta", 4, "E1", _delta(dict(items[3:4]))),
    )
    assert len(_reqs(tr, "resync")) == 2
    link = rx.link("r")
    assert link["gaps"] == 1 and link["drops"] == 2
    # a second gap right away is rate limited (and the resync is still pending)
    rx._on_msg(
        "adt4/r/dsg/delta",
        env_bytes("r", "dsg/delta", 6, "E1", _delta(dict(items[4:5]))),
    )
    assert len(_reqs(tr, "resync")) == 2
    # deltas are still applied
    assert set(rx.graph("r").records) == {s for s, _ in items[:5]}
    assert [k for _, k, _ in events].count("delta") == 4


def test_epoch_change_resets_and_resyncs():
    tr, clock, rx, events = _rx()
    recs, _, _ = synthetic_records(4)
    rx._on_msg("adt4/r/dsg/delta", env_bytes("r", "dsg/delta", 0, "E1", _delta(recs)))
    tile = {"key": "0_0_0", "hash": "h", "pos": b"", "col": b"", "idx": b""}
    rx._on_msg("adt4/r/mesh/tile/0_0_0", env_bytes("r", "mesh/tile", 0, "E1", tile))
    assert rx.graph("r").records and rx.tiles("r")
    n = len(_reqs(tr, "resync"))
    rx._on_msg("adt4/r/state", env_bytes("r", "state", 0, "E2", {"pose": None}))
    assert len(_reqs(tr, "resync")) == n + 1
    assert rx.link("r")["epoch"] == "E2"
    assert not rx.graph("r").records and not rx.tiles("r")
    # stale messages from the old incarnation are ignored, no flapping
    rx._on_msg("adt4/r/dsg/delta", env_bytes("r", "dsg/delta", 1, "E1", _delta(recs)))
    assert rx.link("r")["epoch"] == "E2" and not rx.graph("r").records
    # snapshot begin/chunk/end rebuilds the mirror
    rx._on_msg(
        "adt4/r/dsg/delta",
        env_bytes("r", "dsg/delta", 0, "E2", {**_delta({}), "kind": "snapshot_begin"}),
    )
    rx._on_msg(
        "adt4/r/dsg/delta",
        env_bytes(
            "r", "dsg/delta", 1, "E2", {**_delta(recs), "kind": "snapshot_chunk"}
        ),
    )
    rx._on_msg(
        "adt4/r/dsg/delta",
        env_bytes("r", "dsg/delta", 2, "E2", {**_delta({}), "kind": "snapshot_end"}),
    )
    assert set(rx.graph("r").records) == set(recs)
    assert rx.link("r")["resyncs"] == 1
    assert events[-1][1] == "snapshot"


def test_requests_retry_until_acked():
    tr, clock, rx, _ = _rx()
    rx._on_msg("adt4/r/state", env_bytes("r", "state", 0, "E1", {}))
    tr.sent.clear()
    rid = rx.set_budget("r", "C3", 20_000)
    for _ in range(3):
        clock.advance(1.1)
        rx.tick()
    budget_reqs = _reqs(tr, "budget")
    assert len(budget_reqs) == 4 and {r["id"] for r in budget_reqs} == {rid}
    ack = {
        "id": rid,
        "ok": True,
        "result": {"budgets": {"C1": 5000, "C2": 50000, "C3": 20000}},
    }
    rx._on_msg("adt4/r/ctl/ack", env_bytes("r", "ctl/ack", 0, "E1", ack))
    clock.advance(5.0)
    rx.tick()
    assert len(_reqs(tr, "budget")) == 4
    assert rx.link("r")["budgets"]["C3"] == 20000
    assert [p["op"] for p in rx.pending_requests()] == [
        "resync"
    ]  # first-contact resync, never acked


def test_manifest_requests_missing_tiles_and_removes_stale():
    tr, clock, rx, events = _rx()
    t1 = {"key": "a", "hash": "1", "pos": b"", "col": b"", "idx": b""}
    t2 = {"key": "b", "hash": "2", "pos": b"", "col": b"", "idx": b""}
    rx._on_msg("k", env_bytes("r", "mesh/tile", 0, "E1", t1))
    rx._on_msg("k", env_bytes("r", "mesh/tile", 1, "E1", t2))
    rx._on_msg(
        "k", env_bytes("r", "mesh/manifest", 0, "E1", {"tiles": {"a": "1", "c": "3"}})
    )
    assert set(rx.tiles("r")) == {"a"}
    assert _reqs(tr, "tiles")[-1]["args"]["keys"] == ["c"]
    assert ("r", "tiles", {"changed": [], "removed": ["b"]}) in events
