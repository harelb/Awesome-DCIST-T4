"""Change-detection reports (C1 ``changes``): AwcdChanges -> payload, prior boxes
for removed objects, change-only sending, receiver storage and resend."""

from types import SimpleNamespace as NS

from helpers import RecordingTransport, env_bytes

from fleetlink import codec
from fleetlink.agent import AgentCore
from fleetlink.changes import PriorBoxes, awcd_to_dict, changes_payload, prior_dsg_file
from fleetlink.receiver import Receiver
from fleetlink.transport import VirtualClock

T = NS(sec=10, nanosec=500_000_000)
V = lambda x, y, z: NS(x=x, y=y, z=z)  # noqa: E731


def _added(i, label=3, name="chair"):
    return NS(
        id=i, stamp=T, change_confidence=0.91, num_frames_observed=25,
        centroid=V(1, 2, 0.5), bbox_center=V(1, 2, 0.5), bbox_dimensions=V(0.5, 0.6, 1.0),
        bbox_orientation=NS(x=0.0, y=0.0, z=0.0, w=1.0), semantic_label=label, name=name,
        confidence=0.7,
    )  # fmt: skip


def _removed(i):
    return NS(id=i, stamp=T, change_confidence=0.85, num_frames_observed=7)


def _msg(added=(), removed=(), robot="r"):
    return NS(
        header=NS(frame_id="map", stamp=T),
        robot_name=robot,
        added_objects=list(added),
        removed_objects=list(removed),
    )


DOOR = {"center": [5, 0, 1], "dims": [1, 0.1, 2], "quat": [0, 0, 0, 1], "label": 12, "name": "door"}


def _prior(boxes=None):
    pb = PriorBoxes("prior.json", loader=lambda _p: boxes if boxes is not None else {77: DOOR})
    pb.start(background=False)
    return pb


def test_awcd_to_dict_keeps_boxes_of_added_only():
    d = awcd_to_dict(_msg([_added(5)], [_removed(77)]))
    assert d["robot"] == "r" and d["frame"] == "map" and d["t"] == 10.5
    (a,) = d["added"]
    assert a["center"] == [1, 2, 0.5] and a["dims"] == [0.5, 0.6, 1.0] and a["name"] == "chair"
    (r,) = d["removed"]
    assert r == {"id": 77, "kind": "removed", "stamp": 10.5, "cc": 0.85, "n": 7}


def test_payload_boxes_removed_from_prior():
    p = changes_payload(awcd_to_dict(_msg([_added(5)], [_removed(77), _removed(99)])), _prior())
    assert p["added"] == 1 and p["removed"] == 2 and p["frame"] == "map"
    by = {(o["kind"], o["id"]): o for o in p["objects"]}
    door = by[("removed", 77)]
    assert door["box"] and door["name"] == "door" and door["center"] == [5, 0, 1]
    assert by[("removed", 99)]["box"] is False  # not in the prior: listed, not drawn
    assert by[("added", 5)]["box"] and by[("added", 5)]["label"] == 3
    assert p["prior"] == {"ready": True, "boxes": 1, "error": None}


def test_payload_hash_ignores_time_and_order():
    a = changes_payload(awcd_to_dict(_msg([_added(5), _added(6)])), None, t=1.0)
    b = changes_payload(awcd_to_dict(_msg([_added(6), _added(5)])), None, t=2.0)
    c = changes_payload(awcd_to_dict(_msg([_added(5)])), None)
    assert a["hash"] == b["hash"] != c["hash"]


def test_prior_load_failure_is_reported():
    def boom(_p):
        raise OSError("no such file")

    pb = PriorBoxes("x.json", loader=boom)
    pb.start(background=False)
    assert not pb.ready and "no such file" in pb.error and pb.lookup(1) is None
    assert PriorBoxes(None).error == "no prior map"


def test_prior_dsg_file_prefers_the_small_graph(tmp_path):
    backend = tmp_path / "hydra" / "backend"
    backend.mkdir(parents=True)
    (backend / "dsg_with_mesh.json").write_text("{}")
    assert prior_dsg_file(str(tmp_path)).endswith("dsg_with_mesh.json")
    (backend / "dsg.json").write_text("{}")
    assert prior_dsg_file(str(tmp_path)).endswith("/dsg.json")
    assert prior_dsg_file("") is None and prior_dsg_file(str(tmp_path / "none")) is None


def _agent():
    tr = RecordingTransport()
    clock = VirtualClock()
    return tr, clock, AgentCore("r", tr, None, clock)


def _run(clock, core, seconds, dt=0.05):
    for _ in range(int(round(seconds / dt))):
        core.tick(clock.advance(dt))


def test_agent_sends_changes_on_change_only():
    tr, clock, core = _agent()
    core.changes_prior = _prior()
    core.on_changes(awcd_to_dict(_msg([_added(5)], [_removed(77)])))
    _run(clock, core, 2.0)
    msgs = tr.messages("adt4/r/changes")
    assert len(msgs) == 1 and msgs[0][2] == "C1"
    assert {o["id"] for o in msgs[0][1]["p"]["objects"]} == {5, 77}
    # the detector republishes the same set: nothing new on the radio
    core.on_changes(awcd_to_dict(_msg([_added(5)], [_removed(77)])))
    _run(clock, core, 2.0)
    assert len(tr.messages("adt4/r/changes")) == 1
    core.on_changes(awcd_to_dict(_msg([], [_removed(77)])))
    _run(clock, core, 2.0)
    msgs = tr.messages("adt4/r/changes")
    assert len(msgs) == 2 and msgs[1][1]["p"]["added"] == 0
    # the state advertises the hash, so a lost message is re-asked for
    state = tr.messages("adt4/r/state")[-1][1]["p"]
    assert state["hashes"]["changes"] == msgs[1][1]["p"]["hash"]


def test_agent_resends_once_the_prior_is_loaded():
    tr, clock, core = _agent()
    loaded = {}
    core.enable_changes("prior.json", background=False)  # real loader fails: no file
    core.changes_prior._loader = lambda _p: loaded
    core.changes_prior._started = False
    core.on_changes(awcd_to_dict(_msg([], [_removed(77)])))
    _run(clock, core, 2.0)
    first = tr.messages("adt4/r/changes")[-1][1]["p"]
    assert first["objects"][0]["box"] is False
    loaded[77] = DOOR
    core.changes_prior.start(background=False)  # on_ready marks the report dirty
    _run(clock, core, 2.0)
    last = tr.messages("adt4/r/changes")[-1][1]["p"]
    assert last["objects"][0]["box"] and last["objects"][0]["name"] == "door"


def test_receiver_stores_changes_and_requests_resend():
    tr = RecordingTransport()
    clock = VirtualClock(100.0)
    events = []
    rx = Receiver(tr, lambda r, k, d: events.append((k, d)), clock=clock, threaded=False)
    rx.start()
    p = changes_payload(awcd_to_dict(_msg([_added(5)])), None)
    tr.inject("adt4/x/changes", env_bytes("x", "changes", 0, "E", p))
    assert rx.changes("x")["objects"][0]["id"] == 5
    assert ("changes", p) in [(k, d) for k, d in events if k == "changes"]
    st = {"hashes": {"changes": "deadbeef"}}
    for i in range(1, 13):
        tr.inject("adt4/x/state", env_bytes("x", "state", i, "E", st))
        clock.advance(1.0)
    reqs = [codec.unpack(d)["p"] for k, d, c in tr.sent if k == "adt4/x/ctl/req"]
    resend = [r for r in reqs if r["op"] == "resend"]
    assert len(resend) == 1 and resend[0]["args"] == {"streams": ["changes"]}
