"""State-machine tests for `dcist_sim_isaac.grasp_backends.PhysicsGraspBackend`
(Task 13, replacing the Task-11 placeholder contract tests).

The backend's Isaac arm access sits behind `_ArmInterface`; these tests inject
a scripted `FakeArm` + `FakeRegistry` via the `arm_factory` constructor param,
so the whole grasp/place state machine runs without Isaac (same defer-Isaac
convention as `grasp.py`). Covered per the brief's Step 1:
  * reachable object  -> terminal "succeeded" + attach recorded
  * object outside reach_m -> "failed" with "reach" in the message
  * servo stall -> "failed"
  * place -> `set_kinematic(False)` called + "succeeded"
Plus: async accept contract, the arm-ownership handoff, no-drive-backend
scope-cut failure, and the `PolicyDriveBackend.set_arm_hold` flag logic.
"""
import numpy as np

from dcist_sim_isaac.grasp_backends import PhysicsGraspBackend

_Q_ID = (1.0, 0.0, 0.0, 0.0)


class _FakeSpec:
    def __init__(self, name):
        self.name = name


class _FakeRobot:
    def __init__(self, name, gripper=(0.1, 0.0, 0.4), drive_backend="policy"):
        self.spec = _FakeSpec(name)
        self._gpos = list(map(float, gripper))
        # sentinel so `_default_arm_factory` sees a drive backend; the scope-cut
        # test passes drive_backend=None to exercise the real default factory.
        self.drive_backend = drive_backend

    def gripper_world_pose(self):
        return list(self._gpos), _Q_ID


class _FakeArm:
    """Scripted arm: a 3x6 [I3 | 0] jacobian so `dls_step`'s dq maps its first
    three entries straight to a cartesian gripper delta. `apply_dq` moves the
    robot's gripper (keeping `gripper_world_pose` in sync with the servo) unless
    `movable=False` (used to force a servo stall)."""

    def __init__(self, robot, reach_origin=(0.0, 0.0, 0.5), movable=True):
        self.robot = robot
        self._origin = np.asarray(reach_origin, dtype=float)
        self.movable = movable
        self.owned = False
        self.release_count = 0

    def reach_origin(self):
        return self._origin

    def to_servo_frame(self, world_xyz):
        # identity servo frame in the fake (== world), so target and gripper
        # live in the same frame as the identity jacobian below.
        return np.asarray(world_xyz, dtype=float)

    def gripper_pos(self):
        return np.asarray(self.robot._gpos, dtype=float)

    def jacobian(self):
        J = np.zeros((3, 6))
        J[0, 0] = J[1, 1] = J[2, 2] = 1.0
        return J

    def deploy(self):
        pass                                  # no-op: fake arm stays put

    def apply_dq(self, dq):
        if self.movable:
            self.robot._gpos = list(np.asarray(self.robot._gpos, dtype=float)
                                    + np.asarray(dq, dtype=float)[:3])

    def take_ownership(self):
        self.owned = True

    def release(self):
        self.owned = False
        self.release_count += 1


class _FakeRegistry:
    def __init__(self, objects):
        # objects: {oid: {"pos": (x,y,z), "graspable": bool, "held_by": str|None}}
        self._o = {k: dict(v) for k, v in objects.items()}
        self.kinematic_calls = []           # list of (oid, enabled)

    def selection_snapshot(self):
        return {k: {"pos": v["pos"], "graspable": v["graspable"],
                    "held_by": v["held_by"]} for k, v in self._o.items()}

    def world_pose(self, oid):
        return tuple(self._o[oid]["pos"]), _Q_ID

    def set_world_pose(self, oid, pos, quat):
        self._o[oid]["pos"] = tuple(pos)

    def set_held_by(self, oid, robot):
        self._o[oid]["held_by"] = robot

    def clear_held(self, oid):
        self._o[oid]["held_by"] = None

    def set_kinematic(self, oid, enabled):
        self.kinematic_calls.append((oid, bool(enabled)))


def _run(backend, name, dt=0.1, max_steps=500):
    """Step until the robot reaches a terminal grasp/place status."""
    for _ in range(max_steps):
        state, msg, oid = backend.status(name)
        if state in ("succeeded", "failed"):
            return state, msg, oid
        backend.step(dt)
    return backend.status(name)


def _make(objects, arms):
    robots = [a.robot for a in arms.values()]
    reg = _FakeRegistry(objects)
    backend = PhysicsGraspBackend(
        robots, reg, arm_factory=lambda r: arms[r.spec.name])
    return backend, reg


# --------------------------------------------------------------------------


def test_reachable_object_grasp_succeeds_and_attaches():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5))
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    accepted, oid, _ = backend.grasp("hilbert")
    assert accepted is True
    assert arm.owned is True                      # took arm ownership

    state, _msg, oid = _run(backend, "hilbert")
    assert state == "succeeded"
    assert oid == "cone_0"
    # attach recorded: object suspended (kinematic True) and marked held
    assert (("cone_0", True) in reg.kinematic_calls)
    assert reg._o["cone_0"]["held_by"] == "hilbert"
    assert arm.owned is False                     # ownership returned (re-stow)


def test_out_of_reach_object_fails_with_reach_message():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5))
    objs = {"far": {"pos": (3.0, 0.0, 0.0), "graspable": True,
                    "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    state, msg, _ = _run(backend, "hilbert")
    assert state == "failed"
    assert "reach" in msg
    assert arm.owned is False


def test_servo_stall_fails():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5), movable=False)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    state, msg, _ = _run(backend, "hilbert")
    assert state == "failed"
    assert "servo" in msg.lower()
    assert arm.owned is False


def test_place_detaches_and_succeeds():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot, reach_origin=(0.0, 0.0, 0.5))
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})

    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"

    accepted, _msg = backend.place("hilbert")
    assert accepted is True
    state, _msg, oid = _run(backend, "hilbert")
    assert state == "succeeded"
    assert oid == "cone_0"
    # detach: object made dynamic again + released
    assert (("cone_0", False) in reg.kinematic_calls)
    assert reg._o["cone_0"]["held_by"] is None
    assert arm.owned is False


def test_no_graspable_object_fails():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot)
    objs = {"rock": {"pos": (0.5, 0.0, 0.0), "graspable": False,
                     "held_by": None}}
    backend, _reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    state, msg, _ = _run(backend, "hilbert")
    assert state == "failed"
    assert "graspable" in msg


def test_status_idle_before_any_attempt():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot)
    backend, _reg = _make({}, {"hilbert": arm})
    assert backend.status("hilbert") == ("idle", "", "")


def test_place_without_holding_fails():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot)
    backend, _reg = _make({}, {"hilbert": arm})
    accepted, msg = backend.place("hilbert")
    assert accepted is False
    assert "not holding" in msg


def test_reset_releases_and_restores_dynamics():
    robot = _FakeRobot("hilbert")
    arm = _FakeArm(robot)
    objs = {"cone_0": {"pos": (0.5, 0.0, 0.0), "graspable": True,
                       "held_by": None}}
    backend, reg = _make(objs, {"hilbert": arm})
    backend.grasp("hilbert")
    assert _run(backend, "hilbert")[0] == "succeeded"   # now holding (kinematic)

    assert backend.reset() is True
    # reset restored dynamics (set_kinematic(False)) on the held object
    assert (("cone_0", False) in reg.kinematic_calls)
    assert backend.status("hilbert") == ("idle", "", "")


def test_no_drive_backend_is_clean_failure():
    # Exercises the REAL default arm_factory (no injected fake): a robot with
    # drive_backend=None (locomotion: kinematic, grasping: physics) fails
    # cleanly rather than crashing (documented scope cut).
    robot = _FakeRobot("kin", drive_backend=None)
    reg = _FakeRegistry({"cone_0": {"pos": (0.5, 0.0, 0.0),
                                    "graspable": True, "held_by": None}})
    backend = PhysicsGraspBackend([robot], reg)   # default arm_factory
    accepted, oid, msg = backend.grasp("kin")
    assert accepted is False
    assert "locomotion: policy" in msg
    assert backend.status("kin")[0] == "failed"


def test_unknown_robot():
    backend, _reg = _make({}, {"hilbert": _FakeArm(_FakeRobot("hilbert"))})
    assert backend.grasp("ghost")[0] is False
    assert backend.place("ghost")[0] is False


# -- arm-ownership handoff flag on PolicyDriveBackend (pure, Isaac-free) -----


def test_policy_drive_backend_arm_hold_flag():
    from dcist_sim_isaac.drive_backends import PolicyDriveBackend

    b = PolicyDriveBackend("/World/hilbert", _FakeSpec("hilbert"))
    assert b.arm_hold_enabled() is True           # default: policy holds arm
    b.set_arm_hold(False)
    assert b.arm_hold_enabled() is False          # grasp op owns the arm
    b.set_arm_hold(True)
    assert b.arm_hold_enabled() is True           # returned (re-stow)
