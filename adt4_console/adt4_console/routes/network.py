"""Network: zenoh config/router, Silvus radios, iperf (API.md §6)."""

from __future__ import annotations

import json
import urllib.request

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from . import require_machine, services

router = APIRouter(prefix="/api/network")

ZENOH_ADMIN = "http://localhost:8000/@/router/local/link/**"


def _zenoh_links() -> list:
    try:
        with urllib.request.urlopen(ZENOH_ADMIN, timeout=1.0) as resp:
            data = json.loads(resp.read() or b"[]")
            return data if isinstance(data, list) else [data]
    except Exception:  # noqa: BLE001
        return []


@router.get("/zenoh")
def zenoh_status(request: Request):
    from dcist_launch_system.fleet import zenohd

    svc = services(request)
    fleet = svc.fleet
    port = fleet.topo.get("zenoh", {}).get("port", 7447)
    with fleet.lock:
        machines = [dict(m) for m in fleet.runtime_config.values()]
    results = (
        svc.helpers.run_parallel(
            lambda m: svc.helpers.check_zenoh_port(m["ip"], port, timeout=1), machines
        )
        if machines
        else []
    )
    return {
        "local_zenohd": {"running": zenohd.is_running(fleet.zenohd_state)},
        "port": port,
        "links": _zenoh_links(),
        "reachable": {m["name"]: r is True for m, r in results},
    }


class DeployBody(BaseModel):
    machines: list[str] | None = None
    robots: list[str] | None = None


@router.post("/zenoh/deploy")
def zenoh_deploy(request: Request, body: DeployBody | None = None):
    svc = services(request)
    body = body or DeployBody()
    fleet = svc.fleet
    h = svc.helpers

    def run(job):
        with fleet.lock:
            targets = [
                dict(m)
                for n, m in fleet.runtime_config.items()
                if (body.machines is None and m.get("online", True))
                or (body.machines and n in body.machines)
            ]
            topo, network = fleet.topo, fleet.network
        if not targets:
            job.log("warn", "No machines to deploy to.")
            return False

        def deploy(m):
            try:
                eps = h.generate_zenoh_endpoints(
                    topo, network, m["name"], robot_filter=body.robots
                )
            except KeyError:
                return False, f"no address on {network}", 0
            ok, msg = h.deploy_zenoh_config(m["user"], m["ip"], eps)
            return ok, msg, len(eps)

        all_ok = True
        out = {}
        for m, r in h.run_parallel(deploy, targets):
            ok, msg, n = r if isinstance(r, tuple) else (False, str(r), 0)
            all_ok &= ok
            out[m["name"]] = {"ok": ok, "message": msg, "endpoints": n}
            job.log("ok" if ok else "error", f"{m['name']}: {msg} ({n} ep)")
        job.result = out
        return out if all_ok else False

    job = svc.jobs.start("zenoh_deploy", "Deploy zenoh config", run)
    return {"job_id": job.id}


@router.post("/zenohd/start")
def zenohd_start(request: Request):
    from dcist_launch_system.fleet import zenohd

    fleet = services(request).fleet
    with fleet.lock:
        ok, msg = zenohd.start(
            fleet.zenohd_state,
            fleet.topo,
            fleet.network,
            fleet.runtime_config,
            fleet.local_ips,
        )
    return {"ok": ok, "message": msg}


@router.post("/zenohd/stop")
def zenohd_stop(request: Request):
    from dcist_launch_system.fleet import zenohd

    fleet = services(request).fleet
    was = zenohd.is_running(fleet.zenohd_state)
    zenohd.stop(fleet.zenohd_state)
    return {"ok": True, "message": "stopped" if was else "not running"}


@router.get("/silvus")
def silvus(request: Request):
    from dcist_launch_system.fleet.silvus import link_matrix, routing_summary

    svc = services(request)
    details = svc.helpers.get_silvus_radio_details(svc.fleet.topo)
    matrix = {}
    for src, row in link_matrix(details).items():
        matrix[src] = (
            row
            if isinstance(row, str)
            else {
                dst: {"rssi_db": c.rssi_db, "quality": c.quality, "linked": c.linked}
                for dst, c in row.items()
            }
        )
    return {"matrix": matrix, "routing": routing_summary(details)}


@router.get("/silvus/route")
def silvus_route(request: Request):
    return services(request).helpers.check_silvus_route()


class IperfBody(BaseModel):
    machine: str
    seconds: int = 5


@router.post("/iperf")
def iperf(body: IperfBody, request: Request):
    svc = services(request)
    m = require_machine(svc, body.machine)
    h = svc.helpers
    server_ip = h.get_local_ip_for_network(svc.fleet.topo, svc.fleet.network)
    if not server_ip:
        raise HTTPException(400, f"this machine has no address on {svc.fleet.network}")

    def run(job):
        if not h.check_iperf3(m["user"], m["ip"]):
            job.log("error", f"iperf3 not installed on {m['name']}")
            return False
        job.log("info", f"iperf3 {m['name']} -> {server_ip} for {body.seconds}s")

        def interval(t_end, mbps, retr):
            job.set_progress(
                m["name"], min(100.0, 100.0 * t_end / max(1, body.seconds))
            )
            job.log("dim", f"{t_end:4.1f}s  {mbps:8.2f} Mbit/s")

        res = h.run_iperf3_test(
            server_ip, m["user"], m["ip"], duration=body.seconds, interval_cb=interval
        )
        if "error" in res:
            job.log("error", res["error"])
            return False
        job.log(
            "ok",
            f"{res['mbps']:.2f} Mbit/s"
            + (" (ssh-tunneled)" if res.get("tunneled") else ""),
        )
        return res

    job = svc.jobs.start("iperf", f"iperf {m['name']}", run)
    return {"job_id": job.id}
