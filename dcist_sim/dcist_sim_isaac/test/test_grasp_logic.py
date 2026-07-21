from dcist_sim_isaac.grasp import GraspBackend, select_grasp_target


def make_reg():
    return {
        "bag_0": {"pos": (5.0, 2.0, 0.1), "graspable": True, "held_by": None},
        "bag_1": {"pos": (50.0, 2.0, 0.1), "graspable": True, "held_by": None},
        "rock_0": {"pos": (5.1, 2.0, 0.0), "graspable": False, "held_by": None},
    }


def test_selects_nearest_graspable_in_radius():
    assert select_grasp_target(make_reg(), gripper_pos=(5.2, 2.0, 0.4), radius=1.5) == "bag_0"


def test_none_when_out_of_radius():
    assert select_grasp_target(make_reg(), gripper_pos=(20.0, 2.0, 0.4), radius=1.5) is None


def test_ignores_held_and_ungraspable():
    reg = make_reg()
    reg["bag_0"]["held_by"] = "hilbert"
    assert select_grasp_target(reg, gripper_pos=(5.2, 2.0, 0.4), radius=1.5) is None


# -- GraspBackend.status()/_last coverage (Task 11) ----------------------
#
# GraspBackend itself defers all Isaac/pxr imports into its methods (see
# grasp.py's module docstring), so it's constructible here with plain
# duck-typed fakes -- no Isaac/ROS needed, same as select_grasp_target above.
# These fakes only implement the subset of ObjectRegistry/robot surface
# GraspBackend's grasp()/place()/teleport()/reset() actually touch.


class _FakeSpec:
    def __init__(self, name, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.name = name
        self.x, self.y, self.z, self.yaw = x, y, z, yaw


class _FakeRobot:
    def __init__(self, name, gripper_pos=(0.0, 0.0, 0.0)):
        self.spec = _FakeSpec(name)
        self._gripper_pos = gripper_pos
        self.teleported = None

    def gripper_world_pose(self):
        return self._gripper_pos, (1.0, 0.0, 0.0, 0.0)

    def teleport(self, x, y, z, yaw):
        self.teleported = (x, y, z, yaw)


class _FakeRegistry:
    """Mirrors `ObjectRegistry`'s surface over a plain dict, Isaac-free."""

    def __init__(self, objects):
        self._objects = objects  # {object_id: {"pos", "graspable", "held_by"}}
        self.collision_calls = []            # (oid, enabled) -- Task 15i

    def selection_snapshot(self):
        return {oid: dict(v) for oid, v in self._objects.items()}

    def world_pose(self, object_id):
        return self._objects[object_id]["pos"], (1.0, 0.0, 0.0, 0.0)

    def set_world_pose(self, object_id, pos, quat):
        self._objects[object_id]["pos"] = pos

    def set_held_by(self, object_id, robot_name):
        self._objects[object_id]["held_by"] = robot_name

    def clear_held(self, object_id):
        self._objects[object_id]["held_by"] = None

    def set_collision_enabled(self, object_id, enabled):
        self.collision_calls.append((object_id, bool(enabled)))

    def reset_all(self):
        pass


def _one_bag_registry(pos=(5.0, 2.0, 0.1)):
    return _FakeRegistry({"bag_0": {"pos": pos, "graspable": True, "held_by": None}})


def test_status_idle_before_any_attempt():
    backend = GraspBackend([_FakeRobot("hilbert")], _one_bag_registry(), grasp_radius=1.5)
    assert backend.status("hilbert") == ("idle", "", "")


def test_status_mirrors_successful_grasp():
    robot = _FakeRobot("hilbert", gripper_pos=(5.0, 2.0, 0.1))
    backend = GraspBackend([robot], _one_bag_registry(), grasp_radius=1.5)
    success, object_id, message = backend.grasp("hilbert")
    assert success is True
    assert backend.status("hilbert") == ("succeeded", message, object_id)
    assert object_id == "bag_0"


def test_status_mirrors_failed_grasp():
    # Gripper far outside grasp_radius of the only object -> select_grasp_
    # target returns None -> grasp() fails.
    robot = _FakeRobot("hilbert", gripper_pos=(500.0, 500.0, 0.1))
    backend = GraspBackend([robot], _one_bag_registry(), grasp_radius=1.5)
    success, object_id, message = backend.grasp("hilbert")
    assert success is False
    assert backend.status("hilbert") == ("failed", message, "")


def test_status_mirrors_place_result():
    robot = _FakeRobot("hilbert", gripper_pos=(5.0, 2.0, 0.1))
    backend = GraspBackend([robot], _one_bag_registry(), grasp_radius=1.5)
    backend.grasp("hilbert")
    success, message = backend.place("hilbert")
    assert success is True
    assert backend.status("hilbert") == ("succeeded", message, "bag_0")


def test_status_does_not_cross_contaminate_between_robots():
    near = _FakeRobot("hilbert", gripper_pos=(5.0, 2.0, 0.1))  # grasps fine
    far = _FakeRobot("newton", gripper_pos=(500.0, 500.0, 0.1))  # out of radius
    backend = GraspBackend([near, far], _one_bag_registry(), grasp_radius=1.5)

    backend.grasp("hilbert")
    backend.grasp("newton")

    hilbert_state, _, hilbert_obj = backend.status("hilbert")
    newton_state, _, newton_obj = backend.status("newton")
    assert hilbert_state == "succeeded"
    assert hilbert_obj == "bag_0"
    assert newton_state == "failed"
    assert newton_obj == ""


def test_reset_clears_last_to_idle():
    robot = _FakeRobot("hilbert", gripper_pos=(5.0, 2.0, 0.1))
    backend = GraspBackend([robot], _one_bag_registry(), grasp_radius=1.5)
    backend.grasp("hilbert")
    assert backend.status("hilbert")[0] == "succeeded"

    assert backend.reset() is True

    assert backend.status("hilbert") == ("idle", "", "")


# -- held-object collision toggle in the magic backend (Task 15i) -----------


def test_magic_grasp_disables_collision_place_reenables():
    robot = _FakeRobot("hilbert", gripper_pos=(5.0, 2.0, 0.1))
    reg = _one_bag_registry()
    backend = GraspBackend([robot], reg, grasp_radius=1.5)
    backend.grasp("hilbert")
    assert reg.collision_calls[-1] == ("bag_0", False)   # disabled while held
    backend.place("hilbert")
    assert reg.collision_calls[-1] == ("bag_0", True)    # re-enabled on place


def test_magic_reset_reenables_collision_while_held():
    robot = _FakeRobot("hilbert", gripper_pos=(5.0, 2.0, 0.1))
    reg = _one_bag_registry()
    backend = GraspBackend([robot], reg, grasp_radius=1.5)
    backend.grasp("hilbert")
    assert reg.collision_calls[-1] == ("bag_0", False)
    assert backend.reset() is True
    assert reg.collision_calls[-1] == ("bag_0", True)    # re-enabled on reset
