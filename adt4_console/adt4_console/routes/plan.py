"""Planning goals → omniplanner topics, plan/result history (API.md §9)."""

from __future__ import annotations

from typing import Literal

from fastapi import APIRouter, HTTPException, Request
from pydantic import BaseModel

from ..graph_service import canonical_symbol
from ..ros_bridge import RosUnavailable
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
    return [{"robot": r, "plugins": ros.plugins_for(r)} for r in _robots(request)]


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
    ros = _ros(request)
    fields = {"robot_id": robot, "pddl_goal": body.goal}
    return ros.send_goal(robot, _plugin(ros, "pddl", body.plugin), fields)


@router.post("/robots/{robot}/executor/{cmd}")
def executor(robot: str, cmd: str, request: Request):
    if cmd not in ("pause", "resume"):
        raise HTTPException(404, f"unknown executor command {cmd}")
    return _ros(request).executor_cmd(robot, cmd)
