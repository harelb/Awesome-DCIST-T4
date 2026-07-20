"""Magic-attach grasp/place/teleport/reset backend for dcist_sim_isaac (Task 9).

Tier A grasping (design spec section "Physics tiers": `grasping: magic`)
never touches PhysX contact/grasp dynamics -- it selects the nearest
graspable object within the gripper's radius and rigidly re-poses it to
the gripper every frame ("kinematic attach"), exactly the way
`spot_robot.py`'s kinematic locomotion tier drives the robot root: we
write world poses ourselves, we never ask PhysX to simulate the
attachment (no fixed joint, no weld).

`select_grasp_target` is deliberately pure (stdlib `math` only, no
Isaac/ROS imports at module scope) so it is unit-testable with plain
`python3` -- see `test/test_grasp_logic.py`. Every Isaac-dependent piece
below (`ObjectRegistry`, `GraspBackend`) defers its `isaacsim`/`pxr`
imports into method bodies, the same pattern `stage.py` and
`spot_robot.py` use, so importing this module never requires Isaac to be
installed.

Kinematic-attach math (`GraspBackend.step`): at grasp time we record the
object's pose *relative to the gripper* (rotate the world-frame offset
into the gripper's local frame, and the object's orientation relative to
the gripper's). Every frame afterwards we re-derive the object's world
pose from the gripper's *current* world pose and that fixed local
offset. This is a full 6-DoF rigid attach (the object rotates and
translates with the gripper, not just a world-frame position clamp),
which is what "ride along" while driving means in the Step 4 manual
verification -- a world-frame-only offset would look wrong the moment
the robot turns.

Place ("detach", design spec line 128): per task-9-brief.md Step 2, "a
fixed drop at gripper z-0.3 clamped >= 0 is acceptable for P1" -- no
raycast-to-ground, no terrain height query. Documented here rather than
implemented more elaborately because Task 10's environment/ground truth
isn't in scope for this task.
"""
from __future__ import annotations

import json
import logging
import math

logger = logging.getLogger(__name__)

# Kept as a literal (not imported from scenario.py) on purpose:
# scenario.py's module docstring states it must stay importable with
# stdlib+pyyaml only and independent of this module; mirroring the
# constant avoids a needless cross-import for one float. Keep both
# definitions in sync if this default ever changes.
DEFAULT_GRASP_RADIUS = 1.5

# task-9-brief.md Step 2: "a fixed drop at gripper z-0.3 clamped >= 0".
PLACE_DROP_OFFSET = 0.3


def select_grasp_target(registry, gripper_pos, radius):
    """Return the object_id of the nearest graspable, unheld object in
    `registry` within euclidean `radius` of `gripper_pos`, or None.

    `registry` maps object_id -> {"pos": (x, y, z), "graspable": bool,
    "held_by": str | None}. Pure function (stdlib `math` only) -- see
    test/test_grasp_logic.py for the TDD tests this satisfies verbatim.
    """
    best_id = None
    best_dist = None
    for object_id, entry in registry.items():
        if not entry["graspable"] or entry["held_by"] is not None:
            continue
        dist = math.dist(entry["pos"], gripper_pos)
        if dist > radius:
            continue
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_id = object_id
    return best_id


# -- quaternion helpers (scalar-first w, x, y, z -- matches spot_robot.py's
# Isaac-side convention; ros_bridge.py converts to/from ROS's scalar-last
# x, y, z, w at the service boundary) -----------------------------------


def _quat_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def _quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _rotate_vector(v, q):
    """Rotate 3-vector `v` by unit quaternion `q` (w, x, y, z)."""
    qv = (0.0, v[0], v[1], v[2])
    _, x, y, z = _quat_mul(_quat_mul(q, qv), _quat_conjugate(q))
    return (x, y, z)


def _to_local_frame(gripper_pos, gripper_quat, obj_pos, obj_quat):
    """Express world-frame `obj_pos`/`obj_quat` relative to the gripper frame."""
    delta = tuple(o - g for o, g in zip(obj_pos, gripper_pos))
    local_pos = _rotate_vector(delta, _quat_conjugate(gripper_quat))
    local_quat = _quat_mul(_quat_conjugate(gripper_quat), obj_quat)
    return local_pos, local_quat


class _ObjectEntry:
    __slots__ = (
        "xform", "prim_path", "label", "graspable", "held_by", "spawn_pos",
        "spawn_quat",
    )

    def __init__(self, xform, prim_path, label, graspable, spawn_pos, spawn_quat):
        self.xform = xform
        # Stored directly rather than re-derived from `xform.prim_paths[0]`
        # at use time (Task 6, `set_kinematic`): `add()` already has the
        # exact string, and it sidesteps depending on `XFormPrim`/`Prim`'s
        # view-style `prim_paths` list property for what is always a
        # single-prim wrapper here.
        self.prim_path = prim_path
        self.label = label
        self.graspable = graspable
        self.held_by = None
        self.spawn_pos = spawn_pos
        self.spawn_quat = spawn_quat


class ObjectRegistry:
    """Live per-object Isaac state: `{object_id: (prim_path, label,
    graspable, held_by)}` per the task-9-brief.md "Produces" line, plus
    the spawn pose ResetScenario restores.

    Built once by `stage.py`'s `_spawn_objects` as each `ObjectSpec`'s
    prim is created. Positions are re-queried from the live stage prim
    on every `selection_snapshot()`/`world_pose()` call rather than
    cached, so grasp selection always sees the object's true current
    pose (including mid-carry).
    """

    def __init__(self):
        self._entries = {}

    def add(self, object_id, prim_path, label, graspable, spawn_pos, spawn_quat):
        from isaacsim.core.prims import XFormPrim

        self._entries[object_id] = _ObjectEntry(
            xform=XFormPrim(prim_path),
            prim_path=prim_path,
            label=label,
            graspable=graspable,
            spawn_pos=tuple(float(v) for v in spawn_pos),
            spawn_quat=tuple(float(v) for v in spawn_quat),
        )

    def __contains__(self, object_id):
        return object_id in self._entries

    def __len__(self):
        return len(self._entries)

    def world_pose(self, object_id):
        positions, orientations = self._entries[object_id].xform.get_world_poses()
        pos = tuple(float(v) for v in positions[0])
        quat = tuple(float(v) for v in orientations[0])
        return pos, quat

    def set_world_pose(self, object_id, pos, quat):
        import numpy as np

        self._entries[object_id].xform.set_world_poses(
            positions=np.array([pos], dtype=float),
            orientations=np.array([quat], dtype=float),
        )

    def selection_snapshot(self):
        """Build the plain-dict registry `select_grasp_target` expects."""
        snapshot = {}
        for object_id, entry in self._entries.items():
            pos, _ = self.world_pose(object_id)
            snapshot[object_id] = {
                "pos": pos,
                "graspable": entry.graspable,
                "held_by": entry.held_by,
            }
        return snapshot

    def set_held_by(self, object_id, robot_name):
        self._entries[object_id].held_by = robot_name

    def clear_held(self, object_id):
        self._entries[object_id].held_by = None

    def status_snapshot(self):
        """`{object_id: held_by_or_null}` for the `/sim/status` debug topic."""
        return {oid: e.held_by for oid, e in self._entries.items()}

    def reset_all(self):
        for object_id, entry in self._entries.items():
            entry.held_by = None
            self.set_world_pose(object_id, entry.spawn_pos, entry.spawn_quat)

    def set_kinematic(self, object_id, enabled):
        """Suspend (True) or restore (False) PhysX dynamics on a dynamic
        object -- used by physics-tier grasp backends (Task 8+) while an
        object is held, so the same "we own the pose, PhysX doesn't"
        invariant `GraspBackend.step`'s magic-attach relies on also holds
        for physics-mode dynamic objects during a hold (spec §6.1).

        Only meaningful for objects spawned dynamic (`stage._make_dynamic`,
        physics_mode); calling this on a kinematic-tier object is harmless
        (it already never has dynamics applied) but pointless.
        """
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(self._entries[object_id].prim_path)
        UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(bool(enabled))


class GraspBackend:
    """Backend for the grasp/place/teleport/reset services (Task 9).

    Owns the `ObjectRegistry` and the per-robot "what am I holding, and
    at what fixed offset" state, and implements ResetScenario's
    restore-to-spawn semantics for both robots and objects.

    All public methods are called synchronously from ROS service
    callbacks executing inside `RosBridge`'s `spin_once` slot on the sim
    main thread (ros_bridge.py's module docstring: service callbacks run
    serialized with stepping, no locks needed, but must return fast).
    Every method here is O(number of objects held / objects total) --
    small scenario counts, no per-frame overhead concern.
    """

    def __init__(self, robots, registry, grasp_radius=DEFAULT_GRASP_RADIUS):
        self.robots = {r.spec.name: r for r in robots}
        self.registry = registry
        self.grasp_radius = grasp_radius
        # robot_name -> (object_id, local_offset_pos[3], local_offset_quat[4])
        self._attached = {}

    def grasp(self, robot_name):
        robot = self.robots.get(robot_name)
        if robot is None:
            return False, "", f"unknown robot '{robot_name}'"
        if robot_name in self._attached:
            held = self._attached[robot_name][0]
            return False, "", f"'{robot_name}' is already holding '{held}'"

        gripper_pos, gripper_quat = robot.gripper_world_pose()
        gripper_pos = tuple(float(v) for v in gripper_pos)
        gripper_quat = tuple(float(v) for v in gripper_quat)

        target_id = select_grasp_target(
            self.registry.selection_snapshot(), gripper_pos, self.grasp_radius
        )
        if target_id is None:
            return False, "", "no graspable object within grasp_radius"

        obj_pos, obj_quat = self.registry.world_pose(target_id)
        offset_pos, offset_quat = _to_local_frame(
            gripper_pos, gripper_quat, obj_pos, obj_quat
        )
        self._attached[robot_name] = (target_id, offset_pos, offset_quat)
        self.registry.set_held_by(target_id, robot_name)
        logger.info("'%s' grasped '%s'", robot_name, target_id)
        return True, target_id, f"grasped '{target_id}'"

    def place(self, robot_name):
        robot = self.robots.get(robot_name)
        if robot is None:
            return False, f"unknown robot '{robot_name}'"
        attached = self._attached.pop(robot_name, None)
        if attached is None:
            return False, f"'{robot_name}' is not holding anything"
        object_id = attached[0]

        gripper_pos, _ = robot.gripper_world_pose()
        drop_z = max(float(gripper_pos[2]) - PLACE_DROP_OFFSET, 0.0)
        _, obj_quat = self.registry.world_pose(object_id)
        self.registry.set_world_pose(
            object_id,
            (float(gripper_pos[0]), float(gripper_pos[1]), drop_z),
            obj_quat,
        )
        self.registry.clear_held(object_id)
        logger.info("'%s' placed '%s' at z=%.3f", robot_name, object_id, drop_z)
        return True, f"placed '{object_id}'"

    def teleport(self, robot_name, x, y, z, yaw):
        robot = self.robots.get(robot_name)
        if robot is None:
            return False
        robot.teleport(x, y, z, yaw)
        return True

    def reset(self):
        for robot in self.robots.values():
            spec = robot.spec
            robot.teleport(spec.x, spec.y, spec.z, spec.yaw)
        self._attached.clear()
        self.registry.reset_all()
        return True

    def status_json(self):
        return json.dumps(self.registry.status_snapshot())

    def step(self, dt):
        """Re-pin every held object to its gripper. Called every sim
        frame from `RosBridge.step()`, after robots have already been
        stepped for this frame (sim_app.py's main loop order), so this
        reads each robot's post-step gripper pose -- the held object
        never lags a frame behind the robot it's riding on."""
        for robot_name, (object_id, offset_pos, offset_quat) in self._attached.items():
            robot = self.robots[robot_name]
            gripper_pos, gripper_quat = robot.gripper_world_pose()
            gripper_pos = tuple(float(v) for v in gripper_pos)
            gripper_quat = tuple(float(v) for v in gripper_quat)

            world_offset = _rotate_vector(offset_pos, gripper_quat)
            new_pos = tuple(g + o for g, o in zip(gripper_pos, world_offset))
            new_quat = _quat_mul(gripper_quat, offset_quat)
            self.registry.set_world_pose(object_id, new_pos, new_quat)
