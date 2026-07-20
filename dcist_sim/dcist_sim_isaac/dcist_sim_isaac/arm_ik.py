"""Damped-least-squares IK servo, pure numpy (spec §6.1).

Position-only (3D error, 3xN jacobian): the P1 gripper is a single finger
link with no meaningful wrist-orientation contract, so G1 servos position
and leaves orientation free.  Executed closed-loop THROUGH the simulator:
Task 13 reads the live jacobian from the PhysX articulation each control
tick, gets dq here, and applies it as incremental joint position targets.
"""
from __future__ import annotations

import numpy as np


def dls_step(jacobian, err, damping=0.05, gain=0.5, dq_max=0.15):
    J = np.asarray(jacobian, dtype=float)
    e = gain * np.asarray(err, dtype=float)
    JJt = J @ J.T + (damping ** 2) * np.eye(J.shape[0])
    dq = J.T @ np.linalg.solve(JJt, e)
    n = np.linalg.norm(dq)
    if n > dq_max:
        dq *= dq_max / n
    return dq


class IkServo:
    ACTIVE = "active"
    CONVERGED = "converged"
    FAILED = "failed"

    def __init__(self, tol_m=0.02, timeout_s=8.0, stall_window_s=1.5,
                 stall_eps_m=0.005):
        self._tol = tol_m
        self._timeout = timeout_s
        self._stall_window = stall_window_s
        self._stall_eps = stall_eps_m
        self._status = self.ACTIVE

    def start(self, now):
        self._t0 = now
        self._status = self.ACTIVE
        self._best = None        # (best_err_norm, t_of_best)

    def update(self, err, jacobian, now):
        if self._status != self.ACTIVE:
            return None, self._status
        en = float(np.linalg.norm(err))
        if en <= self._tol:
            self._status = self.CONVERGED
            return None, self._status
        if now - self._t0 > self._timeout:
            self._status = self.FAILED
            return None, self._status
        if self._best is None or en < self._best[0] - self._stall_eps:
            self._best = (en, now)
        elif now - self._best[1] > self._stall_window:
            self._status = self.FAILED
            return None, self._status
        return dls_step(jacobian, err), self._status
