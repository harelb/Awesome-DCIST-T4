"""FastAPI application factory.

``create_app(settings, services=None)`` — tests inject a :class:`Services` built
with fakes (``build_services(settings, fleet_deps=..., launch_deps=..., receiver=...)``).
"""

from __future__ import annotations

import contextlib
import logging
import pathlib
from dataclasses import dataclass, field

from fastapi import FastAPI, Request
from fastapi.middleware.gzip import GZipMiddleware
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse

from . import __version__
from .auth import TokenAuthMiddleware
from .chat import ChatService
from .fleet_service import FleetDeps, FleetService
from .geo import GeoService
from .graph_service import GraphService, GraphUnavailable, NotFound
from .hub import Hub
from .jobs import JobManager
from .live_service import LiveService
from .manipulation_service import (
    ApprovalError,
    ManipulationApprovalService,
    ros_response_publisher,
)
from .ros_bridge import RosBridge, RosUnavailable
from .settings import Settings
from .teleop_service import TeleopService

log = logging.getLogger(__name__)

# NDJSON streams must flush chunk by chunk; GZipMiddleware would buffer them.
NO_GZIP_PATHS = ("/api/chat",)


class SelectiveGZipMiddleware:
    """GZip everything except streaming endpoints (same idea as agentic_navigation's chat_panel)."""

    def __init__(self, app, minimum_size=1024, exclude_paths=NO_GZIP_PATHS):
        self._plain_app = app
        self._gzip_app = GZipMiddleware(app, minimum_size=minimum_size)
        self._exclude = set(exclude_paths)

    async def __call__(self, scope, receive, send):
        if scope.get("type") == "http" and scope.get("path") in self._exclude:
            await self._plain_app(scope, receive, send)
        else:
            await self._gzip_app(scope, receive, send)


def _fleet_helpers():
    from dcist_launch_system import fleet_helpers

    return fleet_helpers


@dataclass
class Services:
    settings: Settings
    hub: Hub
    jobs: JobManager
    fleet: FleetService
    graph: GraphService
    ros: RosBridge
    live: LiveService
    chat: ChatService
    launch_deps: object = None  # fleet.launch_workflow.LaunchDeps (None → real)
    helpers: object = field(default_factory=_fleet_helpers)  # fleet_helpers (or a fake)
    launch_state: object = None  # launch_state.LaunchState
    teleop: TeleopService | None = None
    geo: GeoService | None = None
    missions: object = None  # mission.service.MissionService
    quad: object = None  # quad.link.QuadLink
    edits: object = None  # edit_service.EditService
    maps: object = None  # map_loader.MapLoader: the live map and switching it
    scene_repair: object = (
        None  # scene_repair.SceneRepair: scene change -> replan (API.md §20)
    )
    manipulation: ManipulationApprovalService | None = None

    def start(self) -> None:
        if self.settings.fleet_poll:
            self.fleet.start()
        robots = self.settings.robots or self.fleet.robots()
        self.ros.start(robots)
        if self.teleop is not None:
            self.teleop.start()
        self.live.start()
        if self.quad is not None:
            self.quad.start()
        self.graph.start()
        self.graph.warm()

    def stop(self) -> None:
        for svc in (
            self.missions,
            self.quad,
            self.teleop,
            self.live,
            self.ros,
            self.graph,
            self.fleet,
        ):
            if svc is None:
                continue
            try:
                svc.stop()
            except Exception:  # noqa: BLE001
                log.exception("stopping %s", type(svc).__name__)


def build_services(
    settings: Settings,
    *,
    fleet_deps: FleetDeps | None = None,
    launch_deps=None,
    receiver=None,
    helpers=None,
    tile_fetch=None,
) -> Services:
    hub = Hub()
    jobs = JobManager(hub, confirm_timeout=settings.confirm_timeout_s)
    fleet = FleetService(settings, hub, fleet_deps)
    live = LiveService(settings, hub, receiver)
    graph = GraphService(settings, hub, live)
    live.graph = graph
    ros = RosBridge(settings, hub, fleet)
    chat = ChatService(settings, graph, live, ros)
    svc = Services(settings, hub, jobs, fleet, graph, ros, live, chat, launch_deps)
    # late-bound so tests can swap svc.ros.publish_now
    svc.teleop = TeleopService(
        lambda topic, t, fields: svc.ros.publish_now(topic, t, fields),
        hub,
        platform_of=lambda robot: svc.ros.platform(robot),
    )
    ros.teleop_latch = svc.teleop.stopped
    svc.geo = GeoService(settings, graph, fetch=tile_fetch)
    from .map_loader import MapLoader

    svc.maps = MapLoader(settings, graph, jobs, hub, svc.geo)
    from .mission.service import MissionService
    from .quad.link import QuadLink
    from .quad.protocol import QuadFrame

    svc.quad = QuadLink(
        settings.quad_cmd_uri,
        settings.quad_image_uri,
        QuadFrame(svc.geo.default_anchor() if settings.quad_cmd_uri or settings.quad_image_uri else None,
                  settings.quad_utm_origin, settings.quad_map_offset),
    )  # fmt: skip
    svc.missions = MissionService(settings, hub, graph, svc.geo, live, ros, svc.quad)
    chat.missions = svc.missions
    # answers go out through the bridge; the bridge hands requests to the service
    svc.manipulation = ManipulationApprovalService(
        ros_response_publisher(ros),
        hub,
        available=lambda: svc.ros.available,
        error=lambda: svc.ros.error,
        default_robot=lambda: (settings.robots or fleet.robots() or ["euclid"])[0],
    )
    ros.manipulation = svc.manipulation
    if helpers is not None:
        svc.helpers = helpers
    from . import launch_state

    svc.launch_state = launch_state.LaunchState(settings.launch_state_path)
    launch_state.restore(svc)
    from .edit_service import EditService

    svc.edits = EditService(settings, graph, hub, jobs,
                            launch_state=lambda: svc.launch_state.get() if svc.launch_state else None)  # fmt: skip
    chat.edits = svc.edits
    from .sget import SgetService

    # plan repair's scene edits (heracles_agents SGET), used while its writer runs
    chat.sget = SgetService(graph, ros, settings)
    svc.missions.edits = svc.edits
    wire_scene_repair(svc)
    return svc


def _live_target(svc):
    try:
        return svc.maps.target()
    except Exception:  # noqa: BLE001 — no Neo4j target
        return None


def _object_row(svc, sym: str) -> dict | None:
    """The live map's Object ``sym`` ({c: class, x, y}) or None."""
    t = _live_target(svc)
    if t is None:
        return None
    from .graph_service import parse_symbol

    try:
        c, i = parse_symbol(sym)
        rows = t.query("MATCH (n:Object) WHERE n.nodeSymbol IN $syms "
                       "RETURN n.class AS c, n.center.x AS x, n.center.y AS y LIMIT 1",
                       syms=[f"{c.upper()}{i}", f"{c.upper()}({i})"])  # fmt: skip
    except Exception:  # noqa: BLE001
        return None
    return dict(rows[0]) if rows else None


def wire_scene_repair(svc) -> None:
    """Scene changes -> replan recommendations (scene_repair), and console edits ->
    SceneChangeMsg on /scene_graph/changes_applied."""
    from .scene_repair import SceneRepair, pose_in_global, scene_changes_from_edit

    def robot_xy(robot):
        """The robot's (x, y) in the planner's map frame: heracles' state updater
        writes it on the live map's Robot node; else the fleetlink live pose,
        aligned to the global frame (frame_align); else unknown."""
        t = _live_target(svc)
        if t is not None:
            try:
                rows = t.query("MATCH (r:Robot {name: $n}) RETURN r.position.x AS x, r.position.y AS y LIMIT 1",
                               n=robot)  # fmt: skip
                if rows and rows[0].get("x") is not None:
                    return (float(rows[0]["x"]), float(rows[0]["y"]))
            except Exception:  # noqa: BLE001 — no Robot node / Neo4j down
                pass
        st = svc.live.state(robot) if svc.live is not None else None
        if not st:
            return None
        fa = st.get("frame_align")
        if fa is None:
            rx = getattr(svc.live, "receiver", None)
            try:
                fa = (
                    (rx.link(robot) or {}).get("frame_align")
                    if rx is not None
                    else None
                )
            except Exception:  # noqa: BLE001
                fa = None
        return pose_in_global(st.get("pose"), fa)

    def position_of(sym):
        r = _object_row(svc, sym)
        return (r["x"], r["y"]) if r and r.get("x") is not None else None

    def invalidate():
        t = _live_target(svc)
        if t is None:
            return
        t.invalidate()
        svc.graph.notify_version(t.id, "scene changes")

    def send(robot, pddl, constraints, engine):
        return svc.ros.send_pddl_goal(
            robot, pddl, constraints=constraints, engine=engine
        )

    def resume(robot):
        from .routes import teleop as teleop_routes  # Resume also clears a STOP latch

        return teleop_routes.resume(svc, svc.ros, robot)

    sr = SceneRepair(
        svc.hub, send=send, resume=resume, active_goals=lambda: svc.ros.active_goals(),
        plan_of=lambda robot: svc.ros.plan(robot),
        class_of=lambda sym: (_object_row(svc, sym) or {}).get("c"),
        position_of=position_of, robot_xy=robot_xy, invalidate=invalidate,
    )  # fmt: skip
    svc.scene_repair = sr
    svc.ros.scene_repair = sr

    def on_commit(src_id, out, prov):
        t = _live_target(svc)
        if t is not None and t.id != src_id:
            return  # an edit of another map: no robot plans on it
        source = f"console/{prov.get('operator') or 'operator'}"
        changes = scene_changes_from_edit(out.get("changes"))
        summary = out.get("summary")
        if isinstance(summary, (list, tuple)):
            summary = "; ".join(str(x) for x in summary)
        sr.on_applied(
            source, out.get("map_version"), changes, summary=summary or None, local=True
        )
        if changes and svc.ros.available:
            import threading

            def go():
                try:
                    svc.ros.publish_scene_changes(
                        source, out.get("map_version"), changes
                    )
                except Exception:  # noqa: BLE001
                    log.exception("publishing scene changes of an edit")

            threading.Thread(target=go, name="edit-scene-changes", daemon=True).start()

    svc.edits.on_commit = on_commit

    def on_stage(src_id, count, summary):
        t = _live_target(svc)
        if t is not None and t.id != src_id:
            return
        sr.on_staged(src_id, count, summary)

    svc.edits.on_stage = on_stage


PLACEHOLDER = """<!doctype html><html><head><meta charset="utf-8"><title>ADT4 Console</title>
<style>body{font-family:system-ui,sans-serif;background:#111;color:#ddd;margin:3rem}
code{color:#8cf}</style></head><body><h1>ADT4 Console</h1>
<p>The web UI is not built yet (<code>adt4_console/adt4_console/static/index.html</code> is missing).</p>
<p>The API is up: try <a href="/api/health">/api/health</a> (add <code>?token=…</code> if a token is set).</p>
</body></html>"""


def create_app(settings: Settings, services: Services | None = None) -> FastAPI:
    services = services or build_services(settings)

    @contextlib.asynccontextmanager
    async def lifespan(app):
        services.start()
        try:
            yield
        finally:
            services.stop()

    app = FastAPI(title="ADT4 Console", version=__version__, lifespan=lifespan)
    app.state.services = services

    @app.exception_handler(NotFound)
    async def _not_found(request: Request, exc: NotFound):
        return JSONResponse({"detail": str(exc)}, status_code=404)

    @app.exception_handler(GraphUnavailable)
    async def _graph_unavailable(request: Request, exc: GraphUnavailable):
        return JSONResponse({"detail": str(exc)}, status_code=503)

    @app.exception_handler(RosUnavailable)
    async def _ros_unavailable(request: Request, exc: RosUnavailable):
        return JSONResponse({"detail": f"ROS unavailable: {exc}"}, status_code=503)

    @app.exception_handler(ApprovalError)
    async def _approval_error(request: Request, exc: ApprovalError):
        return JSONResponse({"detail": exc.detail}, status_code=exc.status_code)

    from .routes import (
        change_detection,
        chat,
        edit,
        fleet,
        geo,
        graph,
        jobs,
        launch,
        live,
        manipulation,
        maps,
        mission,
        network,
        plan,
        quad,
        sam3,
        scene_repair,
        teleop,
        terminal,
        ws,
    )

    for mod in (fleet, launch, jobs, network, maps, graph, plan, live, chat, sam3, teleop, terminal,
                geo, mission, quad, edit, change_detection, scene_repair, manipulation, ws):  # fmt: skip
        app.include_router(mod.router)
    app.include_router(live.robots_router)

    _mount_static(app, pathlib.Path(settings.static_dir))

    app.add_middleware(SelectiveGZipMiddleware, minimum_size=1024)
    app.add_middleware(TokenAuthMiddleware, token=settings.token)
    return app


def _mount_static(app: FastAPI, static_dir: pathlib.Path) -> None:
    """Serve the built SPA with an index.html fallback for client-side routes."""
    root = static_dir.resolve()

    @app.get("/{path:path}", include_in_schema=False)
    async def spa(path: str):
        if path == "api" or path.startswith("api/"):
            return JSONResponse({"detail": "Not Found"}, status_code=404)
        index = root / "index.html"
        if path:
            target = (root / path).resolve()
            if target.is_file() and root in target.parents:
                headers = (
                    {"Cache-Control": "public, max-age=31536000, immutable"}
                    if path.startswith("assets/")
                    else {}
                )
                # not in every Python's mimetypes table; browsers ignore a
                # manifest served as octet-stream (robot models: viewer/RobotModel)
                media = {
                    ".webmanifest": "application/manifest+json",
                    ".glb": "model/gltf-binary",
                }.get(target.suffix)
                return FileResponse(target, headers=headers, media_type=media)
        if index.is_file():
            return FileResponse(index, headers={"Cache-Control": "no-cache"})
        return HTMLResponse(PLACEHOLDER)
