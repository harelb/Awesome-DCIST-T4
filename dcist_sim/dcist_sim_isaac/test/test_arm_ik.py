import math

import numpy as np
import pytest

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
