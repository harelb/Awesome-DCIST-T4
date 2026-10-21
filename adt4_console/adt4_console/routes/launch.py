"""Launch / robot control (API.md §5) — wraps ``fleet.launch_workflow``."""

from __future__ import annotations

import pathlib
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
    # robot <-> base link network (a topology network name); None = topology default
    link_network: str | None = None
    # "router" | "fleetlink" for this launch; None = fleet_defaults.fleet_link
    link_mode: str | None = None


def _plan(body: LaunchPlanBody, topo: dict | None = None):
    from dcist_launch_system.fleet.launch_workflow import LaunchPlan

    if body.link_network and body.link_network not in ((topo or {}).get("networks") or {}):
        raise HTTPException(400, f"unknown link network {body.link_network!r}")
    if body.link_mode:
        from dcist_launch_system.fleet.launch_workflow import LaunchError, with_link_mode

        try:
            with_link_mode(topo or {}, body.link_mode)
        except LaunchError as e:
            raise HTTPException(400, str(e)) from None
    return LaunchPlan(**body.model_dump())


def link_choices(topo: dict | None, active: str | None) -> dict:
    """Networks a launch can put the robot links on, and the default one."""
    from dcist_launch_system.fleet_helpers import FLEET_LINK_MODES, fleet_link_mode, fleetlink_base, link_network

    return {
        "link_networks": list(((topo or {}).get("networks") or {}).keys()),
        "link_network": link_network(topo or {}, None, active),
        "active_network": active,
        "link_modes": list(FLEET_LINK_MODES),
        "link_mode": fleet_link_mode(topo or {}),
        "fleetlink_base": fleetlink_base(topo or {}),
    }


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
        links = link_choices(fleet.topo, fleet.network)
    return {**sel, **links, "mode": mode, "sessions": list_sessions()}


@router.post("/launch/dry_run")
def launch_dry_run(body: LaunchPlanBody, request: Request):
    from dcist_launch_system.fleet.launch_workflow import LaunchError, dry_run_lines

    svc = services(request)
    try:
        with svc.fleet.lock:
            lines = dry_run_lines(
                _plan(body, svc.fleet.topo),
                svc.fleet.runtime_config,
                svc.settings.output_root,
                svc.fleet.topo,
                svc.fleet.network,
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
    plan = _plan(body, fleet.topo)
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
        ok = run_launch(
            plan,
            topo=topo,
            active_network=network,
            runtime_config=runtime_config,
            output_root=svc.settings.output_root,
            callbacks=cb,
            deps=svc.launch_deps,
        )
        if ok:
            try:
                attach_prior(svc, plan, job)
            except Exception as e:  # noqa: BLE001 - the launch itself succeeded
                job.log("warn", f"could not register the prior map in the viewer: {e}")
        return ok

    title = f"Launch {plan.experiment} on {', '.join(plan.robots)}"
    job = svc.jobs.start("launch", title, run)
    return {"job_id": job.id}


def attach_prior(svc, plan, job) -> dict | None:
    """After a launch: remember it, and show its prior map in the viewer.

    The prior is registered from this machine's copy (the prior itself when it lives
    here, else the local run with the same run_id).  Without a local copy the launch
    record keeps ``pending``; transferring the prior later registers it.
    """
    from dcist_launch_system.fleet import run_identity as ri

    from ..launch_state import new_state, register_prior
    from .maps import _endpoint, _is_local, _local_name, _scan_one, find_local_copy

    if svc.launch_state is None:
        return None
    if not plan.prior_path:
        svc.launch_state.set(new_state(plan, None))
        return None
    src_name = plan.prior_source or _local_name(svc)
    src = _endpoint(svc, src_name)
    p = pathlib.PurePosixPath(plan.prior_path)
    entry = _scan_one(svc, src, p.name, str(p.parent))
    if entry is None:
        job.log("warn", f"prior {src_name}:{plan.prior_path} not found; viewer not updated")
        svc.launch_state.set(new_state(plan, None))
        return None
    ident = entry.get("identity") or {}
    origin = entry.get("origin") or {}
    run_id = ri.canonical_id(entry)
    robot = ident.get("robot") or origin.get("robot") or origin.get("source_machine") or src_name
    experiment = ident.get("experiment") or origin.get("experiment") or p.name
    prior = {
        "run_id": run_id,
        "source_machine": src_name,
        "path": plan.prior_path,
        "experiment": experiment,
        "robot": robot,
        "label": f"prior: {experiment} ({robot})",
        "local_path": None,
        "source_id": None,
        "pending": True,
    }
    if _is_local(svc, src):
        prior["local_path"] = entry["abs_path"]
    else:
        local = find_local_copy(svc, run_id) if run_id else None
        if local is not None:
            prior["local_path"] = local["path"]
    if prior["local_path"]:
        prior["source_id"] = register_prior(svc.graph, prior)
        prior["pending"] = prior["source_id"] is None
    svc.launch_state.set(new_state(plan, prior))
    if prior["source_id"]:
        svc.graph.notify_version(prior["source_id"], "registered")
        job.log(
            "ok",
            f"viewer: {prior['label']} from {prior['local_path']} + live "
            + ", ".join(plan.robots),
        )
    else:
        job.log(
            "warn",
            f"prior not on this machine yet: copy {src_name}:{plan.prior_path} here "
            "(Maps → Transfer to base) and the viewer will show it",
        )
    return prior


@router.get("/launch/active")
def launch_active(request: Request):
    """The last successful launch + the viewer's default sources for it."""
    from ..launch_state import default_sources

    svc = services(request)
    state = svc.launch_state.get() if svc.launch_state is not None else None
    return {"active": state, "default_sources": default_sources(state)}


@router.delete("/launch/active")
def launch_active_clear(request: Request):
    svc = services(request)
    if svc.launch_state is not None:
        svc.launch_state.clear()
    return {"active": None, "default_sources": []}


# Stop & save calls the robot stack's two map savers, in this order (the core
# window's shutdown pane runs the same pair, core.yaml).  hydra_ros_node's
# ~/shutdown (ianvs spinAndWait) only flags the node to exit and answers at
# once; hydra then saves the DSG + mesh (minutes with a big mesh) and exits.
# roman/shutdown pickles the ROMAN map before it answers.
HYDRA_SHUTDOWN_TIMEOUT_S = 15.0
ROMAN_SHUTDOWN_TIMEOUT_S = 180.0  # > fleetlink relay timeout_s (170) + 2
HYDRA_EXIT_WAIT_S = 600.0
HYDRA_EXIT_POLL_S = 2.0
# teardown: after Ctrl-C, how long nodes get to exit before the session is killed
TEARDOWN_WAIT_S = 30.0


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


def _wait_hydra_exit_ssh(svc, job, m) -> bool | None:
    """Like _wait_hydra_exit, by looking for the hydra process on the robot."""
    t0 = time.monotonic()
    last_note = t0
    while time.monotonic() - t0 < HYDRA_EXIT_WAIT_S:
        rc, out, _ = svc.helpers.ssh_cmd(
            m["user"], m["ip"], "pgrep -f hydra_ros_node >/dev/null && echo up || echo gone"
        )
        if rc != 0 or out.strip() not in ("up", "gone"):
            return None
        if out.strip() == "gone":
            return True
        if time.monotonic() - last_note >= 30:
            last_note = time.monotonic()
            job.log("dim", f"hydra still saving ({time.monotonic() - t0:.0f} s) ...")
        time.sleep(HYDRA_EXIT_POLL_S)
    return False


def _teardown_stack(svc, job, robot: str, m) -> bool:
    """Ctrl-C every pane of the ADT4 session, give the nodes TEARDOWN_WAIT_S to
    exit, then kill the session."""
    from dcist_launch_system.fleet.launch_workflow import TMUX_SESSION_NAME

    from ..terminal import stop_session

    session = TMUX_SESSION_NAME
    job.log("info", f"Stopping the rest of the stack on {robot} (Ctrl-C every pane of {session}) ...")
    ok = stop_session(
        svc.helpers.ssh_cmd, m["user"], m["ip"], session,
        lambda lvl, text: job.log(lvl, f"{robot}: {text}"),
        wait_s=TEARDOWN_WAIT_S, poll_s=HYDRA_EXIT_POLL_S,
    )  # fmt: skip
    if ok:
        job.log("ok", f"{robot}: {session} stopped and closed")
    return ok


class StopSaveBody(BaseModel):
    # after saving, Ctrl-C and kill the robot's whole ADT4 tmux session
    teardown: bool = True


@router.post("/robots/{robot}/stop_save")
def stop_save(robot: str, request: Request, body: StopSaveBody | None = None):
    svc = services(request)
    teardown = body.teardown if body else True

    def finish(job, result, hydra_exited):
        """Tear the stack down once hydra's save is known to be done."""
        if not teardown or result is False:
            return result
        m = svc.fleet.machine(robot)
        if m is None:
            job.log("warn", f"{robot} is not in the topology; cannot stop its tmux session")
            return result
        if hydra_exited is None:
            job.log("info", "Waiting for hydra to finish saving and exit ...")
            hydra_exited = _wait_hydra_exit_ssh(svc, job, m)
            if hydra_exited:
                job.log("ok", f"{robot}: hydra saved and exited")
        if hydra_exited is False:
            job.log("warn", f"{robot}: hydra is still saving; left the stack running (kill it from Fleet when it is done)")
            return {**result, "teardown": False}
        if hydra_exited is None:
            job.log("warn", f"{robot}: cannot tell whether hydra finished saving; left the stack running")
            return {**result, "teardown": False}
        return {**result, "teardown": _teardown_stack(svc, job, robot, m)}

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
                return finish(
                    job,
                    {
                        "method": "service",
                        "hydra": True,
                        "roman": roman_ok,
                        "hydra_exited": hydra_saved,
                    },
                    hydra_saved,
                )
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
        return finish(job, {"method": "tmux", "detail": msg}, None) if ok else False

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
