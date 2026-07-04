from dcist_sim_isaac.grasp import select_grasp_target


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
