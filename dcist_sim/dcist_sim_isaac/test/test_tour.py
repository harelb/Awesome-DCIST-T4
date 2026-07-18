from dcist_sim_isaac.scenario import TourWaypoint
from dcist_sim_isaac.tour import DONE, SEND, WAIT, TourSequencer

WPS = [
    TourWaypoint(x=2.0, y=0.0, yaw=0.0),
    TourWaypoint(x=4.0, y=0.0, yaw=0.0, dwell_s=5.0),
]


def test_happy_path_reaches_all():
    seq = TourSequencer(WPS, arrival_tol_m=0.5, waypoint_timeout_s=90.0)
    assert seq.next_action(0.0, (0.0, 0.0)).kind == SEND
    assert seq.next_action(1.0, (1.0, 0.0)).kind == WAIT
    a = seq.next_action(2.0, (2.0, 0.0))     # reached wp0 -> sends wp1
    assert (a.kind, a.waypoint_index) == (SEND, 1)
    a = seq.next_action(3.0, (4.0, 0.0))     # reached wp1 -> dwell 5s
    assert a.kind == WAIT
    assert seq.next_action(7.0, (4.0, 0.0)).kind == WAIT   # still dwelling
    assert seq.next_action(8.1, (4.0, 0.0)).kind == DONE   # dwell over
    assert [r.status for r in seq.results] == ["reached", "reached"]
    assert seq.ok()


def test_timeout_retries_then_skips():
    seq = TourSequencer(WPS, waypoint_timeout_s=10.0, max_retries=1)
    assert seq.next_action(0.0, (0.0, 0.0)).kind == SEND
    a = seq.next_action(11.0, (0.0, 0.0))    # timeout -> retry same wp
    assert (a.kind, a.waypoint_index) == (SEND, 0)
    a = seq.next_action(22.0, (0.0, 0.0))    # second timeout -> skip, send wp1
    assert (a.kind, a.waypoint_index) == (SEND, 1)
    assert seq.results[0].status == "skipped"
    assert seq.results[0].attempts == 2


def test_skip_fraction_fails_run():
    seq = TourSequencer(WPS, waypoint_timeout_s=1.0, max_retries=0,
                        max_skip_fraction=0.3)
    t = 0.0
    while seq.next_action(t, (0.0, 0.0)).kind != DONE:
        t += 10.0
    assert seq.skipped_fraction == 1.0
    assert not seq.ok()


def test_none_odom_waits():
    seq = TourSequencer(WPS)
    assert seq.next_action(0.0, None).kind == SEND
    assert seq.next_action(1.0, None).kind == WAIT


def test_empty_tour_done_not_ok():
    seq = TourSequencer([])
    assert seq.next_action(0.0, (0.0, 0.0)).kind == DONE
    assert not seq.ok()
