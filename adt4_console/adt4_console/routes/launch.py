"""Launch / robot control (API.md §5) — wraps ``fleet.launch_workflow``."""

from __future__ import annotations

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..jobs import JobLaunchCallbacks
from ..ros_bridge import RosUnavailable
from . import services

router = APIRouter(prefix="/api")


class LaunchPlanBody(BaseModel):
    robots: list[str]
    session: str
    experiment: str
    mode: str | None = None
    prior_path: str = ""
    prior_source: str | None = None
    record: bool = False
    base_station: str | None = None
    bs_session: str | None = None


def _plan(body: LaunchPlanBody):
    from dcist_launch_system.fleet.launch_workflow import LaunchPlan

    return LaunchPlan(**body.model_dump())


@router.get("/launch/defaults")
def launch_defaults(request: Request, mode: str = "mapping"):
    from dcist_launch_system.fleet.launch_workflow import (
        default_selection,
        list_sessions,
    )

    fleet = services(request).fleet
    with fleet.lock:
        sel = default_selection(
            fleet.topo, fleet.runtime_config, mode, None, fleet.local_ips
        )
    return {**sel, "mode": mode, "sessions": list_sessions()}


@router.post("/launch/dry_run")
def launch_dry_run(body: LaunchPlanBody, request: Request):
    from dcist_launch_system.fleet.launch_workflow import LaunchError, dry_run_lines

    svc = services(request)
    try:
        with svc.fleet.lock:
            lines = dry_run_lines(
                _plan(body), svc.fleet.runtime_config, svc.settings.output_root
            )
    except LaunchError as e:
        raise HTTPException(400, str(e)) from None
    return {"lines": lines}


@router.post("/launch")
def launch(body: LaunchPlanBody, request: Request):
    from dcist_launch_system.fleet.launch_workflow import (
        LaunchError,
        build_robot_commands,
        run_launch,
    )

    svc = services(request)
    fleet = svc.fleet
    plan = _plan(body)
    try:  # validate up front so the UI gets a 400, not a failed job
        build_robot_commands(plan, fleet.runtime_config, svc.settings.output_root)
    except LaunchError as e:
        raise HTTPException(400, str(e)) from None

    def run(job):
        cb = JobLaunchCallbacks(job, fleet.zenohd_state)
        with fleet.lock:
            topo = fleet.topo
            network = fleet.network
            runtime_config = {k: dict(v) for k, v in fleet.runtime_config.items()}
        return run_launch(
            plan,
            topo=topo,
            active_network=network,
            runtime_config=runtime_config,
            output_root=svc.settings.output_root,
            callbacks=cb,
            deps=svc.launch_deps,
        )

    title = f"Launch {plan.experiment} on {', '.join(plan.robots)}"
    job = svc.jobs.start("launch", title, run)
    return {"job_id": job.id}


@router.post("/robots/{robot}/stop_save")
def stop_save(robot: str, request: Request):
    svc = services(request)

    def run(job):
        if svc.ros.available:
            job.log("info", f"Calling /{robot}/shutdown ...")
            try:
                svc.ros.call_service(
                    f"/{robot}/shutdown", "std_srvs/srv/Empty", {}, timeout=10
                )
                job.log(
                    "ok", f"{robot}: shutdown service called (maps are being saved)"
                )
                return {"method": "service"}
            except Exception as e:  # noqa: BLE001
                job.log("warn", f"shutdown service failed: {e}")
        else:
            job.log("dim", "ROS unavailable; using tmux fallback")
        m = svc.fleet.machine(robot)
        if m is None:
            job.log(
                "error", f"{robot} is not in the topology; cannot fall back to tmux"
            )
            return False
        ok, msg = svc.helpers.send_tmux_keys(m["user"], m["ip"])
        job.log("ok" if ok else "error", f"{robot}: tmux Enter on core pane: {msg}")
        return {"method": "tmux"} if ok else False

    job = svc.jobs.start("stop_save", f"Stop & save {robot}", run)
    return {"job_id": job.id}


class SaveDsgBody(BaseModel):
    path: str = ""
    include_mesh: bool = True


@router.post("/robots/{robot}/save_dsg")
def save_dsg(robot: str, request: Request, body: SaveDsgBody | None = None):
    svc = services(request)
    body = body or SaveDsgBody()
    if not svc.ros.available:
        raise RosUnavailable(svc.ros.error or "not running")
    service = f"/{robot}/dsg_saver/save_dsg"

    def run(job):
        job.log("info", f"Calling {service} (include_mesh={body.include_mesh}) ...")
        res = svc.ros.call_service(
            service,
            "dcist_launch_system_msgs/srv/SaveDsg",
            {"save_path": body.path, "include_mesh": body.include_mesh},
            timeout=60,
        )
        ok = bool(res.get("success"))
        job.log("ok" if ok else "error", f"save_dsg success={ok}")
        return res if ok else False

    job = svc.jobs.start("save_dsg", f"Save DSG on {robot}", run)
    return {"job_id": job.id}
