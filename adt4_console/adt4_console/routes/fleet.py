"""Health, config, fleet machines/status/nodes and topology editing (API.md §3)."""

from __future__ import annotations

import time

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from .. import __version__
from . import services

router = APIRouter(prefix="/api")


@router.get("/time")
def server_time():
    """Console wall clock, for browsers to estimate their clock offset (lag
    figures compare times taken on robot, console and browser)."""
    return {"t": time.time()}


@router.get("/health")
def health(request: Request):
    svc = services(request)
    neo4j = {}
    for s in svc.graph.sources.values():
        if s.kind == "neo4j":
            try:
                s.read_version()
                neo4j[s.name] = True
            except Exception:  # noqa: BLE001
                neo4j[s.name] = False
    return {
        "ok": True,
        "version": __version__,
        "ros": svc.ros.available,
        "ros_error": svc.ros.error,
        "fleetlink": svc.live.available,
        "fleetlink_error": svc.live.error,
        "neo4j": neo4j,
    }


@router.get("/config")
def config(request: Request):
    from dcist_launch_system.fleet.launch_workflow import list_sessions

    svc = services(request)
    s = svc.settings
    fleet = svc.fleet
    local = fleet.local_machine
    return {
        "network": fleet.network,
        "networks": sorted(fleet.topo.get("networks", {})),
        "output_root": s.output_root,
        "local_machine": local["name"] if local else None,
        "neo4j_targets": [
            {"name": n, "uri": u, "prior": n == s.prior_neo4j}
            for n, u in s.neo4j.items()
        ],
        "maps": [
            {"name": src.name, "path": str(src.run_dir)}
            for src in svc.graph.sources.values()
            if src.kind == "file"
        ],
        "sessions": list_sessions(),
        "fleet_defaults": fleet.topo.get("fleet_defaults", {}),
        "robots": s.robots or fleet.robots(),
        "topology_path": fleet.topology_path,
    }


@router.get("/fleet")
def fleet_machines(request: Request):
    return services(request).fleet.machines_payload()


@router.post("/fleet/refresh")
def fleet_refresh(request: Request):
    return services(request).fleet.refresh_reachability()


@router.get("/fleet/status")
def fleet_status(request: Request):
    return services(request).fleet.status


@router.post("/fleet/status/refresh")
def fleet_status_refresh(request: Request):
    svc = services(request)
    if svc.settings.fleet_poll:
        svc.fleet.request_status_refresh()
        return {"ok": True, "queued": True}
    return svc.fleet.refresh_status()


@router.get("/fleet/nodes")
def fleet_nodes(request: Request):
    return services(request).fleet.nodes


# ------------------------------------------------------------------ topology
class MachineEdit(BaseModel):
    name: str
    role: str
    platform: str = ""
    desc: str = ""
    mit_wifi: str = ""
    silvus: str = ""


@router.get("/topology")
def topology(request: Request):
    return services(request).fleet.topo


@router.put("/topology/machines/{name}")
def topology_edit(name: str, edit: MachineEdit, request: Request):
    fleet = services(request).fleet
    original = name if name in fleet.topo.get("machines", {}) else None
    new_name = fleet.edit_machine(original, edit.model_dump())
    return {"ok": True, "name": new_name, "machine": fleet.topo["machines"][new_name]}


@router.delete("/topology/machines/{name}")
def topology_delete(name: str, request: Request):
    fleet = services(request).fleet
    if name not in fleet.topo.get("machines", {}):
        raise HTTPException(404, f"unknown machine {name}")
    fleet.remove_machine(name)
    return {"ok": True}


@router.post("/topology/save")
def topology_save(request: Request):
    path = services(request).fleet.save_topology()
    return {"ok": True, "path": path}
