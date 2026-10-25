"""Path planning for mission previews and the simulator.

Ground agents follow the traversable mesh-places graph (the layer omniplanner's
``LayerPlanner`` plans on), with Dijkstra on Euclidean edge length; places inside
a no-go geofence are removed first.  The node-to-node zig-zag is then smoothed
(shortcuts, then corner rounding) inside a rasterised corridor of traversable
ground around the path, keeping clear of its edges and out of the geofences.
Quads fly straight lines, and cover an ``explore_area`` polygon with a
boustrophedon (lawnmower) pattern whose line spacing comes from the camera
footprint.
"""

from __future__ import annotations

import logging
import math

import numpy as np

log = logging.getLogger(__name__)

OBJECT_LAYER = 2
CAMERA_HFOV_DEG = 70.0  # mock quad camera; spacing = footprint × (1 − overlap)
COVERAGE_OVERLAP = 0.2
# None = hop count (omniplanner's route choice); "weight" = metres (shortest distance)
ROUTE_WEIGHT = None
OFFGRAPH_OK_M = 4.0  # a target this far from the graph is still "on" it
CELL_M = 0.2  # corridor raster resolution
SAMPLE_M = 0.15  # segment check sample spacing
SEAM_CELLS = 2  # seams up to ~2× this many cells wide between places' polygons are closed
BAND_M = 5.0  # corridor = places within this graph distance of the planned path
CLEARANCE_M = 0.5  # smoothed legs keep this far from untraversable ground where they can
Z_TOL_M = 0.6  # a smoothed leg stays on ground within this of its interpolated height
DETOUR_MAX = 2.0  # a shortcut may skip at most this × its length (+2 m) of the raw path
CORNER_CUT_M = 1.5  # most a rounded corner moves off its vertex
ROUND_PASSES = 2


class NoPath(Exception):
    pass


def point_in_polygon(p, poly) -> bool:
    x, y = p[0], p[1]
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i][0], poly[i][1]
        x2, y2 = poly[(i + 1) % n][0], poly[(i + 1) % n][1]
        if (y1 > y) != (y2 > y):
            xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < xi:
                inside = not inside
    return inside


def _seg_intersect(a, b, c, d) -> bool:
    def orient(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def segment_hits_polygon(a, b, poly) -> bool:
    if point_in_polygon(a, poly) or point_in_polygon(b, poly):
        return True
    n = len(poly)
    return any(_seg_intersect(a, b, poly[i], poly[(i + 1) % n]) for i in range(n))


def path_length(pts) -> float:
    return float(sum(math.dist(pts[i][:2], pts[i + 1][:2]) for i in range(len(pts) - 1)))


def straight(a, b) -> list[list[float]]:
    return [[float(a[0]), float(a[1])], [float(b[0]), float(b[1])]]


def footprint_m(altitude: float, hfov_deg: float = CAMERA_HFOV_DEG) -> float:
    return 2.0 * altitude * math.tan(math.radians(hfov_deg) / 2.0)


def lawnmower(poly, spacing: float, start=None) -> list[list[float]]:
    """Boustrophedon sweep of a polygon along its longer bbox axis; each sweep
    line is clipped to the polygon (all crossings, so concave shapes work)."""
    P = np.asarray(poly, dtype=float)[:, :2]
    if len(P) < 3:
        return []
    lo, hi = P.min(0), P.max(0)
    along_x = (hi[0] - lo[0]) >= (hi[1] - lo[1])
    # sweep lines are parallel to the long axis, stepped along the short one
    a, b = (1, 0) if along_x else (0, 1)  # a: stepped axis, b: line axis
    spacing = max(float(spacing), 1.0)
    n_lines = max(1, int(math.ceil((hi[a] - lo[a]) / spacing)))
    first = lo[a] + ((hi[a] - lo[a]) - (n_lines - 1) * spacing) / 2.0
    out: list[list[float]] = []
    for i in range(n_lines):
        c = first + i * spacing
        xs = []
        for j in range(len(P)):
            p, q = P[j], P[(j + 1) % len(P)]
            if (p[a] > c) != (q[a] > c):
                xs.append(p[b] + (c - p[a]) * (q[b] - p[b]) / (q[a] - p[a]))
        xs.sort()
        segs = [(xs[k], xs[k + 1]) for k in range(0, len(xs) - 1, 2)]
        if i % 2:
            segs = [(s1, s0) for s0, s1 in reversed(segs)]
        for s0, s1 in segs:
            for v in (s0, s1):
                pt = [0.0, 0.0]
                pt[a], pt[b] = float(c), float(v)
                out.append(pt)
    if start is not None and out and math.dist(start[:2], out[-1]) < math.dist(start[:2], out[0]):
        out.reverse()
    return out


def _points_in_polygon(P, poly) -> np.ndarray:
    Q = np.asarray(poly, dtype=float)[:, :2]
    x, y = P[:, 0], P[:, 1]
    inside = np.zeros(len(P), dtype=bool)
    for (x1, y1), (x2, y2) in zip(Q, np.roll(Q, -1, axis=0)):
        if y1 == y2:
            continue
        cross = (y1 > y) != (y2 > y)
        xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
        inside ^= cross & (x < xi)
    return inside


def polygon_centroid(poly) -> list[float]:
    P = np.asarray(poly, dtype=float)[:, :2]
    return [float(P[:, 0].mean()), float(P[:, 1].mean())]


class PlaceGraph:
    """Mesh-places graph of one loaded source, plus object positions."""

    def __init__(self, loaded):
        import networkx as nx

        self.G = loaded.G
        self.nx = nx.Graph()
        self.pos: dict[int, np.ndarray] = {}
        self.bounds: dict[int, np.ndarray] = {}
        self.objects: dict[str, dict] = {}
        layer = self._places_layer()
        if layer is not None:
            for n in layer.nodes:
                self.pos[n.id.value] = np.asarray(n.attributes.position[:3], dtype=float)
                b = np.asarray(getattr(n.attributes, "boundary", []), dtype=float)
                if b.ndim == 2 and len(b) >= 3:
                    self.bounds[n.id.value] = b[:, :2]
                self.nx.add_node(n.id.value)
            for e in layer.edges:
                if e.source in self.pos and e.target in self.pos:
                    w = float(np.linalg.norm(self.pos[e.source][:2] - self.pos[e.target][:2]))
                    self.nx.add_edge(e.source, e.target, weight=max(w, 1e-3))
        self.ids = list(self.pos)
        self.P = np.array([self.pos[i][:2] for i in self.ids]) if self.ids else np.zeros((0, 2))
        self._symbols_from_records(loaded)
        self.Z = np.array([self.pos[i][2] for i in self.ids]) if self.ids else np.zeros(0)
        self._index = {v: k for k, v in enumerate(self.ids)}
        self._cells = None
        self._component = {}
        for k, comp in enumerate(nx.connected_components(self.nx)):
            for v in comp:
                self._component[v] = k

    def _places_layer(self):
        """Mesh places (named MESH_PLACES = layer 3 partition 1 in Hydra output),
        else layer 20, else plain places."""
        import spark_dsg as dsg

        G = self.G
        if G is None:
            return None
        for key in (("MESH_PLACES",), (3, 1), (20,), (3,)):
            try:
                if key == ("MESH_PLACES",):
                    L = G.get_layer(dsg.DsgLayers.MESH_PLACES)
                else:
                    L = G.get_layer(*key)
            except Exception:  # noqa: BLE001
                continue
            if L.num_nodes() > 0:
                return L
        return None

    def _symbols_from_records(self, loaded):
        from ..graph_service import canonical_symbol

        for rec in loaded.records.values():
            if rec.layer == OBJECT_LAYER and rec.part == 0:
                ls = loaded.labelspaces.get(f"{rec.layer}/{rec.part}", {})
                label = ls.get(str(rec.label), "") if rec.label >= 0 else ""
                self.objects[canonical_symbol(rec.sym)] = {
                    "pos": [float(v) for v in rec.pos],
                    "label": label or rec.name or "",
                }

    @property
    def empty(self) -> bool:
        return not self.ids

    # ------------------------------------------------------------ lookups
    def symbol_pos(self, sym: str) -> list[float] | None:
        from ..graph_service import canonical_symbol

        try:
            s = canonical_symbol(sym)
        except ValueError:
            return None
        if s in self.objects:
            return list(self.objects[s]["pos"])
        if self.G is not None:
            try:
                import spark_dsg as dsg

                from ..graph_service import parse_symbol

                c, i = parse_symbol(s)
                node = self.G.find_node(dsg.NodeSymbol(c, i))
                if node is not None:
                    return [float(v) for v in node.attributes.position]
            except Exception:  # noqa: BLE001
                return None
        return None

    def nearest(self, p, allowed=None) -> int | None:
        if self.empty:
            return None
        d = np.linalg.norm(self.P - np.asarray(p[:2], dtype=float), axis=1)
        if allowed is not None:
            mask = np.array([i in allowed for i in self.ids])
            if not mask.any():
                return None
            d = np.where(mask, d, np.inf)
        return self.ids[int(np.argmin(d))]

    def nearest_symbol(self, p) -> str | None:
        """A place symbol omniplanner's goto accepts, nearest ``p``."""
        v = self.nearest(p)
        return None if v is None else _sym_str(v)

    def same_component(self, a, b) -> bool:
        va, vb = self.nearest(a), self.nearest(b)
        return va is not None and self._component.get(va) == self._component.get(vb)

    def node_xy(self, v) -> list[float]:
        return [float(self.pos[v][0]), float(self.pos[v][1])]

    # ------------------------------------------------------------- planning
    def plan(self, a, b, fences=()) -> tuple[list[list[float]], float]:
        """``a`` → ``b`` through the places graph, avoiding no-go polygons.

        Returns (path, off-graph metres): a target away from every reachable
        place (e.g. something a quad found where no ground robot has mapped) is
        reached by a straight last leg from the nearest reachable place.
        Raises :class:`NoPath` when no traversable place is left."""
        import networkx as nx

        if self.empty:
            return straight(a, b), math.dist(a[:2], b[:2])
        banned = set()
        for poly in fences:
            for v in self.ids:
                if point_in_polygon(self.pos[v], poly):
                    banned.add(v)
        g = self.nx.subgraph([v for v in self.ids if v not in banned]) if banned else self.nx
        allowed = set(g.nodes)
        # snap the start into the component the robot is in, the goal to the
        # nearest place of that same component
        s = self.nearest(a, allowed)
        if s is None:
            raise NoPath("no traversable places left outside the geofences")
        comp = nx.node_connected_component(g, s)
        t = self.nearest(b, comp)
        # fewest hops, like omniplanner (LayerPlanner.get_shortest_path is an
        # unweighted nx.shortest_path): the preview takes the route the robot will
        node_path = nx.shortest_path(g, s, t, weight=ROUTE_WEIGHT)
        pts = [[float(a[0]), float(a[1])]] + [self.node_xy(v) for v in node_path]
        pts.append([float(b[0]), float(b[1])])
        z = [self.pos[s][2]] + [self.pos[v][2] for v in node_path] + [self.pos[t][2]]
        off = math.dist(self.node_xy(t), b[:2])
        keep = _simplify_idx(pts)
        pts, z = [pts[k] for k in keep], [z[k] for k in keep]
        if len(pts) > 2:
            band = nx.multi_source_dijkstra_path_length(g, set(node_path), cutoff=BAND_M, weight="weight")
            try:
                c = Corridor(self, g.subgraph(band))
            except ImportError:  # no scipy: leave the path as planned
                c = None
            if c is not None and off > OFFGRAPH_OK_M:
                # keep the last place: the off-graph leg starts there (planning drops it)
                pts = smooth(pts[:-1], z[:-1], c, fences) + [pts[-1]]
            elif c is not None:
                pts = smooth(pts, z, c, fences)
        return pts, (off if off > OFFGRAPH_OK_M else 0.0)

    def place_cells(self, vs) -> tuple[np.ndarray, np.ndarray]:
        """Raster cells (global ``CELL_M`` grid) of the places ``vs``' ground and
        each cell's height: a place's boundary polygon, else a disk of the local
        place spacing.  Rasterised for all places at once, on first use."""
        if self._cells is None:
            self._cells = self._rasterise()
        cells, start = self._cells
        k = np.array([self._index[v] for v in vs], dtype=int)
        n = start[k + 1] - start[k]
        rows = np.repeat(start[k] - np.cumsum(n) + n, n) + np.arange(n.sum())
        return cells[rows], np.repeat(self.Z[k], n)

    def _rasterise(self):
        lens = [d["weight"] for _, _, d in self.nx.edges(data=True)]
        r = 0.5 * (float(np.median(lens)) if lens else 1.0)
        ang = np.linspace(0, 2 * math.pi, 16, endpoint=False)
        disk = r * np.column_stack([np.cos(ang), np.sin(ang)])
        polys = [self.bounds[v] if v in self.bounds else self.pos[v][:2] + disk for v in self.ids]
        K = max(len(q) for q in polys)
        V = np.stack([np.vstack([q, np.repeat(q[-1:], K - len(q), 0)]) for q in polys])  # pad: zero-length edges
        lo = np.floor(V.min(1) / CELL_M).astype(int)
        size = np.ceil(V.max(1) / CELL_M).astype(int) - lo + 1
        n = size.prod(1)
        pid = np.repeat(np.arange(len(V)), n)
        k = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
        C = lo[pid] + np.column_stack([k // size[pid, 1], k % size[pid, 1]])
        x, y = ((C + 0.5) * CELL_M).T
        inside = np.zeros(len(C), dtype=bool)
        for e in range(K):
            (x1, y1), (x2, y2) = V[pid, e].T, V[pid, (e + 1) % K].T
            with np.errstate(divide="ignore", invalid="ignore"):
                xi = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            inside ^= ((y1 > y) != (y2 > y)) & (x < xi)
        start = np.concatenate([[0], np.cumsum(np.bincount(pid[inside], minlength=len(V)))])
        return C[inside], start


def _sym_str(v: int) -> str:
    import spark_dsg as dsg

    from ..graph_service import canonical_symbol

    return canonical_symbol(dsg.NodeSymbol(v).str())


def _simplify_idx(pts, eps: float = 0.05) -> list[int]:
    keep = [0]
    for k in range(1, len(pts)):
        if math.dist(pts[k], pts[keep[-1]]) > eps:
            keep.append(k)
    return keep


# ------------------------------------------------------------------ smoothing
class Corridor:
    """Traversable ground around one planned path, rasterised: the ground of
    the allowed places within ``BAND_M`` of it (graph distance, so other levels
    and the far side of walls stay out) plus their graph edges.  Each cell has
    its distance to untraversable ground and the height range of its places."""

    def __init__(self, pg: PlaceGraph, band):
        from scipy import ndimage

        C, zs = pg.place_cells(list(band.nodes))
        E = np.array([(pg.pos[u], pg.pos[v]) for u, v in band.edges]).reshape(-1, 2, 3)
        if len(E):
            n = np.ceil(np.linalg.norm(E[:, 1, :2] - E[:, 0, :2], axis=1) / (CELL_M / 2)).astype(int) + 1
            t = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
            t = t / np.repeat(np.maximum(n - 1, 1), n)
            S = np.repeat(E[:, 0], n, 0) + t[:, None] * np.repeat(E[:, 1] - E[:, 0], n, 0)
            C = np.concatenate([C, np.floor(S[:, :2] / CELL_M).astype(int)])
            zs = np.concatenate([zs, S[:, 2]])
        pad = SEAM_CELLS + 2
        self.o = C.min(0) - pad
        C = C - self.o
        shape = tuple((C.max(0) + pad + 1).tolist())
        self.zlo = np.full(shape, np.inf)
        self.zhi = np.full(shape, -np.inf)
        k = np.argsort(zs)
        self.zhi[C[k, 0], C[k, 1]] = zs[k]  # the last write wins
        k = k[::-1]
        self.zlo[C[k, 0], C[k, 1]] = zs[k]
        # neighbouring places' polygons leave thin seams between them: close those
        r = np.arange(-SEAM_CELLS, SEAM_CELLS + 1)
        disk = r[:, None] ** 2 + r[None, :] ** 2 <= SEAM_CELLS**2
        free = ndimage.binary_closing(np.isfinite(self.zlo), disk)
        self.clear = ndimage.distance_transform_edt(free) * CELL_M

    def _ij(self, S):
        ij = np.floor(S / CELL_M).astype(int) - self.o
        ok = (ij >= 0).all(1) & (ij[:, 0] < self.clear.shape[0]) & (ij[:, 1] < self.clear.shape[1])
        return ij, ok

    def clearance(self, S) -> np.ndarray:
        ij, ok = self._ij(S)
        out = np.zeros(len(S))
        out[ok] = self.clear[ij[ok, 0], ij[ok, 1]]
        return out

    def ok(self, A, B, za, zb, fences) -> np.ndarray:
        """Which segments A[k] → B[k] (heights za → zb) are traversable: every
        sample keeps ``CLEARANCE_M`` from untraversable ground (relaxed, 1 m per
        m, towards an endpoint that has less), stays on ground of about its
        interpolated height, and is outside every fence."""
        L = np.linalg.norm(B - A, axis=1)
        n = np.ceil(L / SAMPLE_M).astype(int) + 1
        seg = np.repeat(np.arange(len(A)), n)
        start = np.cumsum(n) - n
        t = (np.arange(n.sum()) - start[seg]) / np.maximum(n[seg] - 1, 1)
        S = A[seg] + t[:, None] * (B - A)[seg]
        ca, cb = self.clearance(A), self.clearance(B)
        need = np.minimum(CLEARANCE_M, np.minimum(ca[seg] + t * L[seg], cb[seg] + (1 - t) * L[seg]))
        ij, inside = self._ij(S)
        clr = np.zeros(len(S))
        clr[inside] = self.clear[ij[inside, 0], ij[inside, 1]]
        near_end = np.minimum(t, 1 - t) * L[seg] < CELL_M
        good = (clr >= need - CELL_M / 2) & ((clr > 0) | near_end)
        on = inside & (clr > 0)
        on[on] = np.isfinite(self.zlo[ij[on, 0], ij[on, 1]])
        z = za[seg] + t * (zb - za)[seg]
        lo = np.full(len(S), -np.inf)
        hi = np.full(len(S), np.inf)
        lo[on] = self.zlo[ij[on, 0], ij[on, 1]]
        hi[on] = self.zhi[ij[on, 0], ij[on, 1]]
        good &= (z >= lo - Z_TOL_M) & (z <= hi + Z_TOL_M)
        for poly in fences:
            good &= ~_points_in_polygon(S, poly)
        return np.logical_and.reduceat(good, start)


def _no_fence(a, b, fences) -> bool:
    return not any(segment_hits_polygon(a, b, poly) for poly in fences)


def smooth(pts, z, c: Corridor, fences=()) -> list[list[float]]:
    """Shortcut the zig-zag, then round the corners; every new segment passes
    ``Corridor.ok`` and misses the fences.  The endpoints are kept."""
    P = np.asarray(pts, dtype=float)[:, :2]
    Z = np.asarray(z, dtype=float)
    keep = [0]
    while keep[-1] < len(P) - 1:
        keep.append(_farthest(P, Z, keep[-1], c, fences))
    P, Z = P[keep], Z[keep]
    for _ in range(ROUND_PASSES):
        P, Z = _round_corners(P, Z, c, fences)
    return [[float(x), float(y)] for x, y in P]


def _farthest(P, Z, i, c, fences, chunk: int = 16) -> int:
    """The farthest later vertex reachable from P[i] in a straight line, skipping
    no more than ``DETOUR_MAX``× the line's length of the original path."""
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    best = i + 1
    for j0 in range(i + 2, len(P), chunk):
        J = np.arange(j0, min(j0 + chunk, len(P)))
        J = J[arc[J] - arc[i] <= DETOUR_MAX * np.linalg.norm(P[J] - P[i], axis=1) + 2.0]
        if not len(J):
            continue
        ok = c.ok(np.repeat(P[i : i + 1], len(J), 0), P[J], np.repeat(Z[i], len(J)), Z[J], fences)
        if not ok.any():
            break
        for j in J[ok][::-1]:
            if j > best and _no_fence(P[i], P[j], fences):
                best = int(j)
                break
    return best


def _round_corners(P, Z, c, fences):
    """One Chaikin-style pass: each corner is cut by a chord at most
    ``CORNER_CUT_M`` from it, if the chord is traversable."""
    if len(P) < 3:
        return P, Z
    U, V, W = P[:-2], P[1:-1], P[2:]
    du, dw = U - V, W - V
    lu, lw = np.linalg.norm(du, axis=1), np.linalg.norm(dw, axis=1)
    f = np.minimum(CORNER_CUT_M, 0.25 * np.minimum(lu, lw))
    ku, kw = f / np.maximum(lu, 1e-9), f / np.maximum(lw, 1e-9)
    Q1, Q2 = V + du * ku[:, None], V + dw * kw[:, None]
    z1, z2 = Z[1:-1] + (Z[:-2] - Z[1:-1]) * ku, Z[1:-1] + (Z[2:] - Z[1:-1]) * kw
    cosang = np.einsum("ij,ij->i", du, dw) / np.maximum(lu * lw, 1e-12)
    ok = (cosang > -0.9986) & (f > 0.05)  # leave turns under ~3 deg alone
    if ok.any():
        ok[ok] = c.ok(Q1[ok], Q2[ok], z1[ok], z2[ok], fences)
    P2, Z2 = [P[0]], [Z[0]]
    for k in range(len(V)):
        if ok[k] and _no_fence(Q1[k], Q2[k], fences):
            P2 += [Q1[k], Q2[k]]
            Z2 += [z1[k], z2[k]]
        else:
            P2.append(V[k])
            Z2.append(Z[k + 1])
    P2.append(P[-1])
    Z2.append(Z[-1])
    return np.asarray(P2), np.asarray(Z2)
