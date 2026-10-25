"""Planner B: omniplanner's own language → PDDL → Fast Downward pipeline, in-process.

The same pieces the base station's ``language_planner`` plugin builds
(``omniplanner_ros.language_planner_ros``: PDDL domain + ``llm_config.yaml`` +
nlu_interface prompt), run on the mission's graph without ROS.  The resulting
per-robot PDDL plans become ground ``goto`` tasks (one per ``goto-*`` action
target), so the preview, the simulator and the live executor treat them like
any other mission.  Ground robots only: omniplanner has no quad or search
actions.
"""

from __future__ import annotations

import logging
import os
import pathlib
import re
import shutil

from .model import Mission, Task, new_id

log = logging.getLogger(__name__)

DEFAULT_DOMAIN = "GotoObjectDomain"
# the robots nlu_interface's planning prompt knows (prompt_pddl_planner.yaml <Robots>)
PROMPT_ROBOTS = ("euclid", "hamilton", "hilbert")
_SYM = re.compile(r"^([a-z])(\d+)$", re.IGNORECASE)


class OmniUnavailable(Exception):
    pass


def _llm_config_path(settings) -> str:
    p = getattr(settings, "omni_llm_config", None)
    if p:
        return os.path.expanduser(os.path.expandvars(p))
    here = pathlib.Path(__file__).resolve()
    for root in here.parents:
        cand = root / "dcist_launch_system" / "config" / "default" / "llm_config.yaml"
        if cand.is_file():
            return str(cand)
    raise OmniUnavailable("no llm_config.yaml (dcist_launch_system/config/default)")


def _interface(settings, domain_name: str):
    try:
        from importlib.resources import as_file, files

        import dsg_pddl.domains
        import nlu_interface.resources
        from dsg_pddl.pddl_grounding import PddlDomain
        from nlu_interface.config import OpenAIConfig
        from nlu_interface_dcist.language_planning_interface import (
            LanguagePddlInterface,
            SimplePddlSceneGraphPrompt,
        )
        from ruamel.yaml import YAML
    except ImportError as e:
        raise OmniUnavailable(f"omniplanner / nlu_interface not importable: {e}") from None
    if shutil.which("fast-downward") is None and shutil.which("fast-downward.py") is None:
        raise OmniUnavailable("Fast Downward is not on PATH")
    yaml = YAML(typ="safe")
    with as_file(files(dsg_pddl.domains).joinpath(domain_name + ".pddl")) as path:
        domain = PddlDomain(pathlib.Path(path).read_text())
    domain.scene_scope = "goal_relevant"
    cfg = yaml.load(pathlib.Path(_llm_config_path(settings)).read_text())
    with as_file(files(nlu_interface.resources).joinpath(cfg["prompt"] + ".yaml")) as path:
        prompt = SimplePddlSceneGraphPrompt(**yaml.load(pathlib.Path(path).read_text()))
    key_var = cfg.get("api_key_env_var", "")
    if not (os.environ.get(key_var) if key_var else os.environ.get("OPENAI_API_KEY")):
        raise OmniUnavailable(f"no OpenAI key in ${key_var or 'OPENAI_API_KEY'}")
    config = OpenAIConfig(
        model=os.environ.get("ADT4_LLM_MODEL") or cfg["model"],
        prompt_mode=cfg.get("mode", "default"),
        prompt_type=cfg.get("prompt_type", "default"),
        num_incontext_examples=cfg["num_incontext_examples"],
        temperature=cfg["temperature"],
        api_timeout=cfg["api_timeout"],
        seed=cfg["seed"],
        api_key_env_var=key_var,
        debug=cfg.get("debug", False),
    )
    return domain, LanguagePddlInterface(config=config, prompt=prompt)


def pddl_symbol_to_node(s: str) -> str | None:
    m = _SYM.match(str(s).strip())
    return f"{m.group(1).upper()}({int(m.group(2))})" if m else None


def plans_by_robot(result, robot=None, out=None) -> dict[str, list[tuple]]:
    """Walk omniplanner's (Robot/MultiRobot-wrapped) result for PddlPlans."""
    out = {} if out is None else out
    if result is None:
        return out
    if hasattr(result, "symbolic_actions"):
        out.setdefault(robot or "?", []).extend(result.symbolic_actions)
        return out
    name = getattr(result, "name", None)
    value = getattr(result, "value", None)
    if value is not None:
        return plans_by_robot(value, name if isinstance(name, str) else robot, out)
    if isinstance(result, dict):
        for k, v in result.items():
            plans_by_robot(v, k if isinstance(k, str) else robot, out)
    elif isinstance(result, (list, tuple)):
        for v in result:
            plans_by_robot(v, robot, out)
    return out


def actions_to_tasks(actions: dict[str, list[tuple]], agents: list[dict]) -> list[Task]:
    ids = [a["id"] for a in agents]
    tasks = []
    for robot, acts in actions.items():
        agent = robot if robot in ids else (ids[0] if len(ids) == 1 else None)
        if agent is None:
            log.warning("omniplanner planned for %r, which is not a mission agent", robot)
            continue
        for act in acts:
            name = str(act[0]).lower()
            if not name.startswith("goto"):
                continue
            target = pddl_symbol_to_node(act[-1])
            if target is None:
                continue
            tasks.append(Task(id=new_id("t"), agent=agent, type="goto", target=target,
                              note=" ".join(str(a) for a in act)))  # fmt: skip
    return tasks


def aliases(agents) -> dict[str, str]:
    """Mission agent id → a robot name the planning prompt knows (agents named
    or bound like a prompt robot keep it; the rest take the free ones)."""
    out, free = {}, [r for r in PROMPT_ROBOTS]
    for a in agents:
        for name in (a.id, a.binding):
            if name in free:
                out[a.id] = name
                free.remove(name)
                break
    for a in agents:
        if a.id not in out:
            if not free:
                raise ValueError(f"omniplanner's prompt knows {len(PROMPT_ROBOTS)} robots; plan at most that many ground agents")
            out[a.id] = free.pop(0)
    return out


def alias_text(text: str, names: dict[str, str]) -> str:
    for aid, alias in sorted(names.items(), key=lambda kv: -len(kv[0])):
        text = re.sub(rf"\b{re.escape(aid)}\b", alias, text, flags=re.IGNORECASE)
    return text


def plan(text: str, agents: list[dict], src: str | None, service, domain_name: str = DEFAULT_DOMAIN) -> Mission:
    # importing the dsg_pddl grounding / planning modules registers their plum
    # dispatch methods (ground_problem / make_plan for PddlDomain)
    import dsg_pddl.dsg_pddl_grounding  # noqa: F401
    import dsg_pddl.dsg_pddl_planning  # noqa: F401
    from omniplanner.language_planner import LanguageDomain, LanguageGoal
    from omniplanner.omniplanner import PlanRequest, full_planning_pipeline

    import numpy as np

    ground = [a for a in agents if a.get("kind", "spot") != "quad"]
    if not ground:
        raise ValueError("omniplanner plans ground robots only")
    m0 = Mission(text=text, src=src, agents=agents, planner="omniplanner")
    world = service.world(m0)
    if world.graph is None or world.graph.G is None:
        raise ValueError("no scene graph to plan on")
    from .planning import start_of

    names = aliases([a for a in m0.agents if a.is_ground])
    back = {v: k for k, v in names.items()}
    domain, llm = _interface(service.settings, domain_name)
    starts = {}
    for aid, alias in names.items():
        s = start_of(m0, world, aid)
        if s is None:
            raise ValueError(f"{aid} has no start")
        starts[alias] = np.array(s[:2])
    req = PlanRequest(
        domain=LanguageDomain("Pddl", domain, llm),
        goal=LanguageGoal(robot_id=names[ground[0]["id"]] if len(ground) == 1 else "", command=alias_text(text, names)),
        robot_states=starts,
    )
    try:
        result = full_planning_pipeline(req, world.graph.G)
    except Exception as e:  # noqa: BLE001 — LLM / grounding / Fast Downward failures
        log.exception("omniplanner failed")
        hint = ""
        if "Unsolvable" in type(e).__name__:
            hint = (" (usually the start or a target snaps to a place-graph island omniplanner "
                    "cannot reach; the chat planner routes around that)")
        raise ValueError(f"omniplanner failed: {type(e).__name__}: {str(e).splitlines()[0][:300]}{hint}") from None
    by_robot = {back.get(r, r): acts for r, acts in plans_by_robot(result).items()}
    tasks = actions_to_tasks(by_robot, [a for a in agents])
    if not tasks:
        raise ValueError("omniplanner returned no goto actions")
    m0.tasks = tasks
    return Mission.model_validate(m0.model_dump())
