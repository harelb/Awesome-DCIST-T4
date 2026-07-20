"""Pure-python unit coverage for SpotSimRobot's terminal-recovery dispatch
(Task 9 review fix). `_terminal_recovery_reason` is the Isaac-free decision
helper `_step_physics` calls to decide whether a NaN-tripped policy action
or a physical fall (or neither) needs the self-heal (reset_standing +
cancel goal + nav_status='fallen') -- see spot_robot.py's docstring for why
both causes now self-heal identically.
"""
from dcist_sim_isaac.spot_robot import _terminal_recovery_reason


def test_no_recovery_when_neither_tripped():
    assert _terminal_recovery_reason(nan_tripped=False, is_fallen=False) is None


def test_recovery_reason_is_fallen():
    assert _terminal_recovery_reason(nan_tripped=False, is_fallen=True) == "fallen"


def test_recovery_reason_is_nan():
    assert _terminal_recovery_reason(nan_tripped=True, is_fallen=False) == "nan"


def test_nan_takes_priority_when_both_true():
    """Implausible but well-defined: NaN wins the log label if a NaN action
    also happened to leave the robot tilted/low in the same frame."""
    assert _terminal_recovery_reason(nan_tripped=True, is_fallen=True) == "nan"
