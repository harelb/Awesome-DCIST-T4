"""Launch / robot control (API.md §5) — wraps ``fleet.launch_workflow``."""

from __future__ import annotations

import time

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
                _plan(body),
                svc.fleet.runtime_config,
                svc.settings.output_root,
                svc.fleet.topo,
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


# Stop & save calls the robot stack's two map savers, in this order (the core
# window's shutdown pane runs the same pair, core.yaml).  hydra_ros_node's
# ~/shutdown (ianvs spinAndWait) only flags the node to exit and answers at
# once; hydra then saves the DSG + mesh (minutes with a big mesh) and exits.
# roman/shutdown pickles the ROMAN map before it answers.
HYDRA_SHUTDOWN_TIMEOUT_S = 15.0
ROMAN_SHUTDOWN_TIMEOUT_S = 180.0  # > fleetlink relay timeout_s (170) + 2
HYDRA_EXIT_WAIT_S = 600.0
HYDRA_EXIT_POLL_S = 2.0


def _wait_hydra_exit(svc, job, robot: str) -> bool | None:
    """Router mode: hydra's shutdown service disappears once hydra saved and
    exited.  True = gone, False = still up after HYDRA_EXIT_WAIT_S, None = cannot
    tell (no graph query)."""
    name = f"/{robot}/hydra/shutdown"
    has = getattr(svc.ros, "has_service", None)
    if has is None:
        return None
    t0 = time.monotonic()
    last_note = t0
    while time.monotonic() - t0 < HYDRA_EXIT_WAIT_S:
        try:
            if not has(name):
                return True
        except Exception:  # noqa: BLE001
            return None
        if time.monotonic() - last_note >= 30:
            last_note = time.monotonic()
            job.log("dim", f"hydra still saving ({time.monotonic() - t0:.0f} s) ...")
        time.sleep(HYDRA_EXIT_POLL_S)
    return False


@router.post("/robots/{robot}/stop_save")
def stop_save(robot: str, request: Request):
    svc = services(request)

    def run(job):
        if svc.ros.available:
            fleetlink = svc.ros.link_status(robot) is not None
            hydra = f"/{robot}/hydra/shutdown"
            roman = f"/{robot}/roman/shutdown"
            job.log("info", f"Calling {hydra} ...")
            try:
                svc.ros.call_service(
                    hydra, "std_srvs/srv/Empty", {}, timeout=HYDRA_SHUTDOWN_TIMEOUT_S
                )
            except Exception as e:  # noqa: BLE001
                job.log("warn", f"{hydra} failed: {e}")
            else:
                job.log(
                    "ok",
                    f"{robot}: hydra is shutting down; it saves the scene graph and "
                    "mesh before it exits (can take minutes with a big mesh)",
                )
                if fleetlink:
                    job.log(
                        "dim",
                        "fleetlink mode: the relay answers an Empty call even when the "
                        "robot did not; confirm on the robot's node status",
                    )
                job.log(
                    "info", f"Calling {roman} (saves the ROMAN map, then answers) ..."
                )
                roman_ok = True
                try:
                    svc.ros.call_service(
                        roman,
                        "std_srvs/srv/Empty",
                        {},
                        timeout=ROMAN_SHUTDOWN_TIMEOUT_S,
                    )
                    job.log("ok", f"{robot}: ROMAN map saved")
                except Exception as e:  # noqa: BLE001
                    roman_ok = False
                    job.log(
                        "warn",
                        f"{roman} failed: {e} (ROMAN may not run in this experiment)",
                    )
                hydra_saved = None
                if not fleetlink:  # the bridge serves relayed services forever
                    job.log("info", "Waiting for hydra to finish saving and exit ...")
                    hydra_saved = _wait_hydra_exit(svc, job, robot)
                    if hydra_saved:
                        job.log("ok", f"{robot}: hydra saved and exited")
                    elif hydra_saved is False:
                        job.log(
                            "warn",
                            f"{robot}: hydra still running after "
                            f"{HYDRA_EXIT_WAIT_S:.0f} s; check its pane",
                        )
                return {
                    "method": "service",
                    "hydra": True,
                    "roman": roman_ok,
                    "hydra_exited": hydra_saved,
                }
        else:
            job.log("dim", "ROS unavailable; using tmux fallback")
        m = svc.fleet.machine(robot)
        if m is None:
            job.log(
                "error", f"{robot} is not in the topology; cannot fall back to tmux"
            )
            return False
        job.log("info", f"tmux fallback: running the core shutdown pane on {robot} ...")
        ok, msg = svc.helpers.stop_save_via_tmux(m["user"], m["ip"], robot=robot)
        job.log("ok" if ok else "error", f"{robot}: {msg}")
        return {"method": "tmux", "detail": msg} if ok else False

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
