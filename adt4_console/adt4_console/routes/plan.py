"""Planning goals → omniplanner topics, plan/result history (API.md §9)."""

from __future__ import annotations

from typing import Literal

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..graph_service import canonical_symbol
from ..ros_bridge import NotSupported, RosUnavailable
from . import services

router = APIRouter(prefix="/api")


def _ros(request: Request):
    ros = services(request).ros
    if not ros.available:
        raise RosUnavailable(ros.error or "not running")
    return ros


def _robots(request: Request) -> list[str]:
    svc = services(request)
    return svc.settings.robots or svc.fleet.robots()


@router.get("/plan/robots")
def plan_robots(request: Request):
    ros = services(request).ros
    return [
        {"robot": r, "platform": ros.platform(r), "plugins": ros.plugins_for(r)}
        for r in _robots(request)
    ]


@router.get("/robots/{robot}/plan")
def robot_plan(robot: str, request: Request):
    return services(request).ros.plan(robot)


class LanguageGoal(BaseModel):
    command: str
    # "default" = the planner's configured language domain (LLM -> PDDL);
    # "goto_points" takes node symbols ("O(12) P(850)") and does not use the LLM.
    domain_type: Literal["default", "Pddl", "goto_points"] = "default"
    plugin: str | None = None


class PointsGoal(BaseModel):
    points: list[str]
    plugin: str | None = None


class PddlGoal(BaseModel):
    plugin: str | None = None
    goal: str
    # plan repair v3 constraints to ADD to the robot's persistent set, e.g.
    # [["forbidden-poi", "o14"], ["forbidden-edge", "p1", "p2"]] or
    # [{"predicate": "forbidden-poi", "symbols": ["o14"]}]
    constraints: list | None = None
    # "auto": goal_manager when it runs, else the plugin + a warning;
    # "planner": straight to the plugin; "goal_manager": 409 when it is down
    route: Literal["auto", "planner", "goal_manager"] = "auto"


class ConstraintsBody(BaseModel):
    constraints: list


def _plugin(ros, kind: str, name: str | None) -> dict:
    p = ros.find_plugin(kind, name)
    if p is None:
        raise HTTPException(
            400, f"no {kind} plugin" + (f" named {name}" if name else "")
        )
    return p


def omniplanner_points(points: list[str]) -> list[str]:
    """Node symbols in omniplanner's form: ``str(NodeSymbol)`` *with* parens.

    ``goto_points.get_loc`` parses with ``parse("{}({})")``, so ``"O12"`` /
    ``"o(12)"`` are normalised to ``"O(12)"`` (category case is kept).
    """
    out = []
    for p in points:
        try:
            out.append(canonical_symbol(p))
        except ValueError:
            raise HTTPException(400, f"bad node symbol {p!r}") from None
    if not out:
        raise HTTPException(400, "no points given")
    return out


@router.post("/robots/{robot}/goal/language")
def goal_language(robot: str, body: LanguageGoal, request: Request):
    ros = _ros(request)
    fields = {
        "robot_id": robot,
        "command": body.command,
        "domain_type": body.domain_type,
    }
    return ros.send_goal(robot, _plugin(ros, "language", body.plugin), fields)


@router.post("/robots/{robot}/goal/goto")
def goal_goto(robot: str, body: PointsGoal, request: Request):
    ros = _ros(request)
    fields = {
        "robot_id": robot,
        "point_names_to_visit": omniplanner_points(body.points),
    }
    return ros.send_goal(robot, _plugin(ros, "goto", body.plugin), fields)


@router.post("/robots/{robot}/goal/tsp")
def goal_tsp(robot: str, body: PointsGoal, request: Request):
    ros = _ros(request)
    fields = {
        "robot_id": robot,
        "point_names_to_visit": omniplanner_points(body.points),
    }
    return ros.send_goal(robot, _plugin(ros, "tsp", body.plugin), fields)


@router.post("/robots/{robot}/goal/pddl")
def goal_pddl(robot: str, body: PddlGoal, request: Request):
    """Through plan repair v3's goal_manager when it runs (API.md §9)."""
    ros = _ros(request)
    try:
        return ros.send_pddl_goal(
            robot, body.goal, body.plugin, body.constraints, body.route
        )
    except ValueError as e:  # bad constraint
        raise HTTPException(400, str(e)) from None
    except LookupError as e:  # no plugin / goal_manager required but down
        raise HTTPException(409 if body.route == "goal_manager" else 400, str(e)) from None


@router.get("/robots/{robot}/constraints")
def get_constraints(robot: str, request: Request):
    ros = services(request).ros
    return {"robot": robot, "constraints": ros.constraints(robot)}


@router.post("/robots/{robot}/constraints")
def add_constraints(robot: str, body: ConstraintsBody, request: Request):
    """Add to the persistent set (sent with every goal_manager goal)."""
    ros = services(request).ros
    try:
        return {"robot": robot, "constraints": ros.add_constraints(robot, body.constraints)}
    except ValueError as e:
        raise HTTPException(400, str(e)) from None


@router.delete("/robots/{robot}/constraints")
def clear_constraints(robot: str, request: Request, body: ConstraintsBody | None = None):
    """Clear all (no body) or only the given constraints."""
    ros = services(request).ros
    try:
        facts = body.constraints if body is not None else None
        return {"robot": robot, "constraints": ros.clear_constraints(robot, facts)}
    except ValueError as e:
        raise HTTPException(400, str(e)) from None


@router.get("/plan/repair")
def plan_repair_status(request: Request):
    """Is plan repair v3's goal_manager running (subscribers on commanded_goal)?"""
    ros = services(request).ros
    if not ros.available:
        return {"available": False, "goal_manager": False, "subscribers": 0,
                "topic": None, "error": ros.error}  # fmt: skip
    subs = ros.goal_manager_subscribers()
    return {"available": True, "goal_manager": subs > 0, "subscribers": subs,
            "topic": ros.commanded_goal_topic()}  # fmt: skip


@router.post("/robots/{robot}/executor/{cmd}")
def executor(robot: str, cmd: str, request: Request):
    """Plan execution: pause / resume / stop.  Stop and resume share the teleop
    routes' handling (phoenix STOP also cuts and latches console teleop; a
    Resume clears that latch)."""
    from . import teleop as teleop_routes

    if cmd not in ("pause", "resume", "stop"):
        raise HTTPException(404, f"unknown executor command {cmd}")
    svc = services(request)
    ros = svc.ros
    try:
        if cmd == "stop":
            return teleop_routes.stop(svc, request, robot)
        if cmd == "pause":
            ros.require_spot(robot, cmd)  # phoenix: no pause, only stop / resume
            return _ros(request).executor_cmd(robot, cmd)
        return teleop_routes.resume(svc, _ros(request), robot)
    except NotSupported as e:
        raise HTTPException(400, str(e)) from None


@router.post("/robots/{robot}/take_lease")
def take_lease(robot: str, request: Request):
    """Forcibly take robot's Spot lease (e.g. from the tablet).  The planner may
    then command the robot; the UI asks for an explicit confirmation first.
    Spot only: 400 at once for any other platform."""
    try:
        services(request).ros.require_spot(robot, "take_lease")
    except NotSupported as e:
        raise HTTPException(400, str(e)) from None
    ros = _ros(request)
    try:
        return ros.take_lease(robot)
    except TimeoutError as e:
        raise HTTPException(504, str(e)) from None
