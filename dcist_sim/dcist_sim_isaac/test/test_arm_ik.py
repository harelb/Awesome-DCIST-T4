import math

import numpy as np

from dcist_sim_isaac.arm_ik import IkServo, dls_step

L1, L2 = 0.5, 0.4


def _fk(q):
    x = L1 * math.cos(q[0]) + L2 * math.cos(q[0] + q[1])
    y = L1 * math.sin(q[0]) + L2 * math.sin(q[0] + q[1])
    return np.array([x, y, 0.0])


def _jac(q):
    s1, c1 = math.sin(q[0]), math.cos(q[0])
    s12, c12 = math.sin(q[0] + q[1]), math.cos(q[0] + q[1])
    return np.array([[-L1 * s1 - L2 * s12, -L2 * s12],
                     [L1 * c1 + L2 * c12, L2 * c12],
                     [0.0, 0.0]])


def _servo_to(target, q0, servo):
    q = np.array(q0, dtype=float)
    servo.start(now=0.0)
    t = 0.0
    while True:
        t += 0.02
        dq, status = servo.update(target - _fk(q), _jac(q), now=t)
        if status != IkServo.ACTIVE:
            return q, status
        q += dq


def test_dls_step_clamps():
    J = np.array([[1.0, 0.0], [0.0, 1.0], [0.0, 0.0]])
    dq = dls_step(J, np.array([10.0, 0.0, 0.0]), dq_max=0.15)
    assert np.linalg.norm(dq) <= 0.15 + 1e-9


def test_converges_to_reachable_target():
    q, status = _servo_to(np.array([0.6, 0.3, 0.0]), [0.3, 0.5], IkServo())
    assert status == IkServo.CONVERGED
    assert np.linalg.norm(_fk(q)[:2] - [0.6, 0.3]) < 0.02


def test_fails_on_unreachable_target():
    q, status = _servo_to(np.array([2.0, 0.0, 0.0]), [0.3, 0.5],
                          IkServo(timeout_s=3.0))
    assert status == IkServo.FAILED


def test_timeout_path_specifically():
    """Timeout clause hits before stall: disable stall with huge window and eps_m=0."""
    servo = IkServo(timeout_s=1.0, stall_window_s=100.0, stall_eps_m=0.0)
    servo.start(now=0.0)

    q = np.array([0.3, 0.5], dtype=float)
    target = np.array([2.0, 0.0, 0.0])  # unreachable

    # Simulate updates at regular intervals; should timeout before stall
    for i in range(60):  # 60 * 0.02 = 1.2s > timeout_s=1.0
        t = i * 0.02
        err = target - _fk(q)
        dq, status = servo.update(err, _jac(q), now=t)

        if status != IkServo.ACTIVE:
            assert status == IkServo.FAILED
            assert t > 1.0  # Timeout must occur after timeout_s threshold
            return
        q += dq

    raise AssertionError("Servo should have timed out by now")


def test_update_after_converged_is_sticky():
    """Terminal state (CONVERGED) is sticky; further updates return None and keep status."""
    servo = IkServo()
    q, status = _servo_to(np.array([0.6, 0.3, 0.0]), [0.3, 0.5], servo)
    assert status == IkServo.CONVERGED

    # Try to update with a large error; should remain converged
    large_err = np.array([5.0, 5.0, 5.0])
    dq, status = servo.update(large_err, _jac(q), now=100.0)
    assert status == IkServo.CONVERGED
    assert dq is None


def test_servo_reuse_after_terminal():
    """After terminal state, start() resets to ACTIVE and can converge again."""
    servo = IkServo()

    # First episode: converge to [0.6, 0.3]
    q, status = _servo_to(np.array([0.6, 0.3, 0.0]), [0.3, 0.5], servo)
    assert status == IkServo.CONVERGED

    # Reuse same servo with different target
    servo.start(now=0.0)
    q = np.array([0.3, 0.5], dtype=float)
    target = np.array([0.7, 0.2, 0.0])
    t = 0.0

    while True:
        t += 0.02
        err = target - _fk(q)
        dq, status = servo.update(err, _jac(q), now=t)
        if status != IkServo.ACTIVE:
            assert status == IkServo.CONVERGED
            assert np.linalg.norm(_fk(q)[:2] - [0.7, 0.2]) < 0.02
            return
        q += dq
