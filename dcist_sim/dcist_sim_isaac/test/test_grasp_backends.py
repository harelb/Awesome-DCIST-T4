"""Contract tests for `dcist_sim_isaac.grasp_backends.PhysicsGraspBackend`.

Task 11 introduces this class purely as a Task-13 placeholder: it must be
constructible without raising (a `grasping: physics` scenario that never
attempts a grasp still constructs it unconditionally in `ros_bridge.py`'s
per-robot dispatch) and every grasp/place attempt must fail cleanly rather
than crash or hang a poller. These tests pin exactly that stub contract --
Task 13 replaces the *internals* with a real physics-tier grasp, at which
point the grasp/place/status assertions here (but not the "constructible"/
"step and reset are no-ops for an untouched backend" ones) should be
replaced with real in_progress/succeeded-path coverage.
"""
from dcist_sim_isaac.grasp_backends import PhysicsGraspBackend

_NOT_IMPLEMENTED_MSG = "physics grasping not implemented until Task 13"


class _FakeSpec:
    def __init__(self, name):
        self.name = name


class _FakeRobot:
    def __init__(self, name):
        self.spec = _FakeSpec(name)


def _make_backend():
    # `registry` is never touched by this stub's methods (see grasp_backends.py)
    # -- a sentinel object is enough to prove that.
    return PhysicsGraspBackend([_FakeRobot("hilbert")], registry=object())


def test_constructible_without_raising():
    _make_backend()  # must not raise NotImplementedError or anything else


def test_grasp_returns_not_accepted():
    backend = _make_backend()
    assert backend.grasp("hilbert") == (False, "", _NOT_IMPLEMENTED_MSG)


def test_place_returns_not_accepted():
    backend = _make_backend()
    assert backend.place("hilbert") == (False, _NOT_IMPLEMENTED_MSG)


def test_status_reports_failed():
    backend = _make_backend()
    assert backend.status("hilbert") == ("failed", _NOT_IMPLEMENTED_MSG, "")


def test_step_is_a_noop():
    backend = _make_backend()
    assert backend.step(0.016) is None  # must not raise


def test_reset_is_a_noop_returning_true():
    backend = _make_backend()
    assert backend.reset() is True
