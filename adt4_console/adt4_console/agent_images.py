"""Images for agent (trajectory keyframe) nodes.

Hydra saves an agent keyframe image only when the robot moved >= 1 m or turned
>= 30 deg since the last one *and* a camera frame lies within 50 ms of the node,
so many agent nodes have no image of their own. The denser sub-keyframes
(0.25 m / 15 deg) are anchored to agent nodes, so for such a node we show the
frame taken closest to its pose: first among the sub-keyframes anchored to it,
then among the frames anchored to neighbouring agent nodes within a few metres.

:class:`AgentFrameIndex` holds every keyframe / sub-keyframe with its world body
pose (anchor pose composed with ``anchor_T_body``, or the odometric pose when the
anchor cannot be resolved) and answers that query; sources build one per graph
version (see ``graph_service``).
"""

from __future__ import annotations

import math
import os
from collections import defaultdict
from dataclasses import dataclass

import numpy as np

NEAR_M = 3.0  # neighbouring frames considered (metres from the node)
YAW_DEG_PER_M = 90.0  # ranking: 90 deg of heading difference weighs like 1 m
MAX_AGENT_IMAGES = 24

KEYFRAME, SUBKEYFRAME = "keyframe", "subkeyframe"


def yaw_of(R) -> float:
    """Heading (rad) of a body rotation matrix (x forward)."""
    return math.atan2(float(R[1][0]), float(R[0][0]))


def _wrap_deg(d):
    return np.abs((d + 180.0) % 360.0 - 180.0)


@dataclass(frozen=True)
class AgentFrame:
    """One frame chosen for an agent node."""

    path: str
    kind: str  # KEYFRAME | SUBKEYFRAME
    own: bool  # the node's own keyframe
    dist_m: float | None  # translation between node and frame body pose
    yaw_deg: float | None  # heading difference (None: a heading is unknown)
    frame_id: int | None
    anchor: str | None  # symbol of the agent node the frame is anchored to


def _sym(node_id: int) -> str:
    return f"{chr((node_id >> 56) & 0xFF)}({node_id & ((1 << 56) - 1)})"


def _category(sym: str | None) -> str:
    return sym[0] if sym else ""


class AgentFrameIndex:
    """Keyframes + sub-keyframes with their world body poses."""

    def __init__(self, rows, node_pose=None):
        """``rows``: iterable of (path, kind, anchor_sym|None, frame_id|None, xyz, yaw_rad|None);
        ``node_pose(sym) -> (xyz, yaw_rad|None) | None`` resolves query nodes."""
        rows = list(rows)
        self.paths = [str(r[0]) for r in rows]
        self.kinds = [r[1] for r in rows]
        self.anchors = [r[2] for r in rows]
        self.frame_ids = [r[3] for r in rows]
        self.pos = np.array([r[4] for r in rows], float).reshape(-1, 3)
        self.yaw = np.array([math.nan if r[5] is None else r[5] for r in rows], float)
        self.by_anchor: dict[str, list[int]] = defaultdict(list)
        for i, a in enumerate(self.anchors):
            if a is not None:
                self.by_anchor[a].append(i)
        self._node_pose = node_pose

    def __len__(self) -> int:
        return len(self.paths)

    def node_pose(self, sym: str):
        return self._node_pose(sym) if self._node_pose is not None else None

    # -- builders -------------------------------------------------------------
    @classmethod
    def from_store(cls, store, lookup=None, positions=None) -> "AgentFrameIndex":
        """Index an ``hydra_image_store.ImageStore``.

        ``lookup(node_id) -> 4x4 | None`` gives (optimized) agent poses, e.g. a
        spark_dsg graph via ``as_pose_lookup``. Without one, ``positions``
        (``sym -> xyz``, e.g. live node records) supply the anchor translation and
        the anchor's heading is recovered from its frames' odometric poses.
        """
        from hydra_image_store import FrameKind

        frames = [
            f
            for f in store.frames(kind=(int(FrameKind.KEYFRAME), int(FrameKind.SUBKEYFRAME)))
            if f.rgb_path is not None
        ]
        canon = {}

        def anchor_of(f):
            if f.anchor_node_id is None:
                return None
            a = int(f.anchor_node_id)
            if a not in canon:
                canon[a] = store.canonical(a)
            return canon[a]

        if lookup is None and positions is not None:
            rot = {}  # anchor id -> odometric anchor rotation (from its frames)
            for f in frames:
                a = anchor_of(f)
                if a is None or a in rot:
                    continue
                aTb = np.eye(4) if f.anchor_T_body is None else f.anchor_T_body
                rot[a] = (f.world_T_body_odom @ np.linalg.inv(aTb))[:3, :3]

            def lookup(nid):
                p = positions.get(_sym(int(nid)))
                if p is None:
                    return None
                T = np.eye(4)
                T[:3, :3] = rot.get(int(nid), np.eye(3))
                T[:3, 3] = p
                return T

            heading_known = rot.__contains__
        else:
            heading_known = None

        rows = []
        for f in frames:
            a = anchor_of(f)
            T = None
            if a is not None and lookup is not None:
                W = lookup(a)
                if W is None and a != f.anchor_node_id:
                    W = lookup(int(f.anchor_node_id))
                if W is not None:
                    aTb = np.eye(4) if f.anchor_T_body is None else f.anchor_T_body
                    T = np.asarray(W, float) @ aTb
            if T is None:
                T = f.world_T_body_odom
            kind = KEYFRAME if int(f.kind) == int(FrameKind.KEYFRAME) else SUBKEYFRAME
            rows.append(
                (f.rgb_path, kind, None if a is None else _sym(a), f.frame_id,
                 T[:3, 3], yaw_of(T[:3, :3]))
            )  # fmt: skip

        def node_pose(sym):
            if lookup is None:
                return None
            try:
                c, i = sym[0], int(sym[2:-1])
            except (ValueError, IndexError):
                return None
            nid = store.canonical((ord(c) << 56) | i)
            T = lookup(nid)
            if T is None:
                return None
            T = np.asarray(T, float)
            known = heading_known is None or heading_known(nid)
            return T[:3, 3], (yaw_of(T[:3, :3]) if known else None)

        return cls(rows, node_pose)

    # -- query ----------------------------------------------------------------
    def select(self, sym: str, pos=None, yaw=None, near_m: float = NEAR_M,
               limit: int = MAX_AGENT_IMAGES, exists=os.path.isfile) -> list[AgentFrame]:  # fmt: skip
        """Frames for agent ``sym``: its own keyframe first, else the nearest frame
        anchored to it, else the nearest frame of a neighbouring agent node within
        ``near_m``; then the other nearby frames by distance. ``pos``/``yaw``
        (rad) default to :meth:`node_pose`."""
        if pos is None:
            p = self.node_pose(sym)
            if p is not None:
                pos, yaw = p
        if not len(self):
            return []
        anchored = self.by_anchor.get(sym, [])
        if pos is not None:
            d = np.linalg.norm(self.pos - np.asarray(pos, float)[None, :], axis=1)
            dy = (
                _wrap_deg(np.degrees(self.yaw - yaw))
                if yaw is not None
                else np.full(len(self), math.nan)
            )
            score = d + np.nan_to_num(dy, nan=0.0) / YAW_DEG_PER_M
            cat = _category(sym)
            near = [
                int(i)
                for i in np.flatnonzero(d <= near_m)
                if _category(self.anchors[i]) == cat
            ]
        else:
            d = dy = None
            score = np.array([float(i) for i in range(len(self))])
            near = []
        own = [i for i in anchored if self.kinds[i] == KEYFRAME]
        by_score = lambda idx: sorted(idx, key=lambda i: (score[i], i))  # noqa: E731
        ordered = by_score(own) + by_score(anchored) + by_score(set(near) | set(anchored))
        out, seen = [], set()
        for i in ordered:
            if i in seen:
                continue
            seen.add(i)
            if exists is not None and not exists(self.paths[i]):
                continue
            out.append(
                AgentFrame(
                    path=self.paths[i],
                    kind=self.kinds[i],
                    own=i in own,
                    dist_m=None if d is None else round(float(d[i]), 3),
                    yaw_deg=(
                        None if dy is None or math.isnan(dy[i]) else round(float(dy[i]), 1)
                    ),
                    frame_id=self.frame_ids[i],
                    anchor=self.anchors[i],
                )
            )
            if len(out) >= limit:
                break
        # the hero is chosen; the strip after it is purely by distance
        return out[:1] + sorted(out[1:], key=lambda f: (f.dist_m is None, f.dist_m or 0.0))
