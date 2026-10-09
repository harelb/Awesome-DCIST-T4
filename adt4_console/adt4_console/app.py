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
from .graph_service import GraphService, GraphUnavailable, NotFound
from .hub import Hub
from .jobs import JobManager
from .live_service import LiveService
from .ros_bridge import RosBridge, RosUnavailable
from .settings import Settings

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

    def start(self) -> None:
        if self.settings.fleet_poll:
            self.fleet.start()
        robots = self.settings.robots or self.fleet.robots()
        self.ros.start(robots)
        self.live.start()
        self.graph.start()
        self.graph.warm()

    def stop(self) -> None:
        for svc in (self.live, self.ros, self.graph, self.fleet):
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
    if helpers is not None:
        svc.helpers = helpers
    return svc


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

    from .routes import chat, fleet, graph, jobs, launch, live, maps, network, plan, ws

    for mod in (fleet, launch, jobs, network, maps, graph, plan, live, chat, ws):
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
                # manifest served as octet-stream
                media = "application/manifest+json" if target.suffix == ".webmanifest" else None
                return FileResponse(target, headers=headers, media_type=media)
        if index.is_file():
            return FileResponse(index, headers={"Cache-Control": "no-cache"})
        return HTMLResponse(PLACEHOLDER)
