# Camp Mission Phase D — Live NL via gpt-mini — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The camp mission ("block the intersection with a cone") passes the strict e2e gate driven from a live natural-language sentence through omniplanner's language planner backed by a gpt-mini model, instead of the scripted PDDL goal.

**Architecture:** `mission_cli --nl` already publishes `LanguageGoalMsg{command, domain_type="Pddl"}` to `/{robot}/omniplanner_node/language_planner/language_goal`; omniplanner's `LanguagePlannerRos` calls `nlu_interface_dcist.LanguagePddlInterface` (OpenAI Responses API) to map the sentence to `{"<robot>": "<pddl goal>"}` and grounds it on `RegionObjectRearrangementDomain` with `scene_scope=goal_relevant`. Phase D closes the three gaps: (1) the domain has no object-in-region concept (add a **derived predicate**, no new actions), (2) the prompt has no block-verb vocabulary (add few-shot examples), (3) the model is `gpt-4.1` (switch isaac_sim config to `gpt-4.1-mini-2025-04-14` via an experiment overlay). A new offline harness (`nl_grounding_check.py`) validates LLM grounding + FD planning against the real `camp_sim_a` graph without GPU before the live gate.

**Tech Stack:** omniplanner (dsg_pddl/PDDL/fast-downward), nlu_interface (+_dcist, OpenAI Responses API), dcist_sim_isaac scripts (spark_env venv + ROS jazzy), dcist_launch_system config_generation, Isaac Sim 6.0 (kinematic tier), Neo4j/heracles.

## Global Constraints

- Push feature branches to **harelb forks only**, never `origin` (MIT-SPARK / robustrobotics); no PRs on origin.
- Generated configs are NEVER hand-edited: change `config_generation/` sources, run `dcist_launch_system/scripts/generate_configs.sh`, sanity via `check_configs.sh` (invoke the repo `adt4-config-generation` skill). NOTE: `check_configs.sh` exits 1 on an unrelated dirty tree — commit or stash first.
- Colcon rebuild after editing installed packages (`omniplanner`, `nlu_interface`): use the repo `building-adt4-workspace` skill. dcist_sim_isaac scripts run in `~/environments/dcist/spark_env` with ROS + workspace sourced and `PYTHONPATH=$PWD/dcist_sim/dcist_sim_isaac:$PYTHONPATH` appended.
- ROMAN stays OFF in sim configs. Kinematic tier only (physics is Phase E).
- The strict verifier (`camp_mission_smoke.py phase_verify`: robot odom pose at the held→released instant must be in-region) must NOT be weakened.
- Gates must not rely on slow FD planning: measure region-domain `goal_relevant` FD wall time offline; budget **< 30 s** (expect seconds).
- Do not modify pinned submodules `heracles` (frame_id defect stays flagged) or `hydra`.
- API key: `ADT4_OPENAI_API_KEY` (exported in `~/.zshrc:133`; tmux panes re-source `~/.zshrc`, so it is present in launched sessions). Never print the key.
- Branches: superproject `feature/isaac_sim_camp_mission` (continue); dcist_sim `feature/camp_mission` (continue); omniplanner **new** `feature/camp_mission_nl` off `0d5f148`; nlu_interface **new** `feature/camp_mission_nl` off `1219c89` (harelb fork must be created — none exists yet).
- Robot/sentence: the sim robot is **hilbert**; the mission sentence is `"Hilbert, block the intersection with a cone"`. (The spec's "Hamilton" is a real-robot name; both are in the prompt roster and the omniplanner adaptor list — the sentence's robot name is what dispatches, because in the `Pddl` branch `LanguageGoalMsg.robot_id` is IGNORED and the robot comes from the LLM output-dict key. `language_planner.py:54-56`.)

## Locked design decisions (do not re-litigate)

1. **Goal form** the LLM emits: `(object-in-region <ObjectID> <RegionID>)` — a NEW **derived predicate** in `RegionObjectRearrangementDomain.pddl` (spec's "no new PDDL actions" honored; derived ≠ action). Rationale: the prompt exposes only Object IDs (+ `parent_region`) and Region IDs — no Place IDs — so `(object-in-place o p)` is un-emittable; `(in-region ?r)` is a *robot*-location predicate; a quantified `exists` goal is fragile LLM output. `goal_relevant` grounding already handles it: `collect_goal_symbol_names` yields `{obj, region}` → object gets its current place + `object-in-place` init fact (`dsg_pddl_grounding.py:486-493`), region gets ≤3 member places + `place-in-region` facts (`:494-509`).
2. **Model**: `gpt-4.1-mini-2025-04-14` (in `OpenAIWrapper.valid_model_names`, `nlu_interface/src/nlu_interface/interface.py:55-62`), scoped to isaac_sim via `experiment_overrides/isaac_sim/llm_config_overlay.yaml` — base `gpt-4.1` for real-robot configs unchanged.
3. **Degenerate-goal hazard**: cone_0 spawns ~3.9 m from the 4.0 m-radius region center — if the LLM names a cone whose current place is already in-region, FD returns an EMPTY plan → no held→released transition → gate fails. The offline harness (Task D2) measures which cone IDs are degenerate on the real graph; the prompt (Task D3) teaches "prefer a matching Object whose `parent_region` is NOT the target Region". Contingency if `parent_region` is `none` on the served graph is written into Task D3.
4. Evidence naming continues the letter scheme: **gateE**, **gateF** (`~/adt4_output/camp_mission_gate{E,F}/`).

---

### Task D1: `object-in-region` derived predicate (omniplanner)

**Files:**
- Modify: `omniplanner/omniplanner/src/dsg_pddl/domains/RegionObjectRearrangementDomain.pddl`
- Modify: `omniplanner/omniplanner/src/dsg_pddl/dsg_pddl_grounding.py:381-389` (`_NON_SYMBOL_TOKENS`)
- Modify: `omniplanner/omniplanner/examples/utils.py` (`build_test_dsg` — add a second region so the new test's goal is not satisfied at init)
- Test: `omniplanner/omniplanner/tests/test_pddl_goal_relevant.py`

**Interfaces:**
- Consumes: existing `ground_problem` / `make_plan` (unchanged signatures).
- Produces: goal predicate `(object-in-region ?o - dsg_object ?r - region)` usable in `PddlGoal.pddl_goal` strings on `RegionObjectRearrangementDomain`; token `object-in-region` recognized as a non-symbol by `collect_goal_symbol_names`. Tasks D2/D3 rely on exactly this goal string form.

- [ ] **Step 1: branch omniplanner**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/omniplanner
git switch -c feature/camp_mission_nl 0d5f148
```

- [ ] **Step 2: Write the failing test**

Append to `omniplanner/omniplanner/tests/test_pddl_goal_relevant.py`:

```python
def test_goal_relevant_object_in_region_plan():
    # o0's current place is a member of r0, so target the OTHER region r1:
    # the plan must pick o0 and place it at a place-in-region of r1.
    G, gp = _ground(
        _load_domain("RegionObjectRearrangementDomain.pddl", "goal_relevant"),
        "(and (object-in-region o0 r1))",
    )
    actions = _plan_action_names(G, gp)
    assert "pick-object" in actions
    assert "place-object" in actions


def test_object_in_region_already_satisfied_yields_empty_plan():
    # Degeneracy guard documented for Phase D: if the object's current place is
    # already in the target region, the goal is initially true -> empty plan.
    G, gp = _ground(
        _load_domain("RegionObjectRearrangementDomain.pddl", "goal_relevant"),
        "(and (object-in-region o0 r0))",
    )
    plan = make_plan(gp, G)
    assert plan.symbolic_actions == []
```

- [ ] **Step 3: Run to verify failure**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/omniplanner/omniplanner
zsh -c "source ~/dcist_ws/install/setup.zsh && source ~/environments/dcist/spark_env/bin/activate && PYTHONPATH=src:examples python -m pytest tests/test_pddl_goal_relevant.py -v"
```
Expected: the two new tests FAIL (unknown predicate `object-in-region` / no region `r1`); the 5 existing tests PASS. (If the existing tests need a different invocation, mirror the file's own header: `PYTHONPATH=omniplanner/src python omniplanner/tests/test_pddl_goal_relevant.py`.)

- [ ] **Step 4: Extend `build_test_dsg` with a second region**

In `omniplanner/omniplanner/examples/utils.py`, after the `O1` insert_edge block (line ~83), add a region `R1` with mirrored place pair `p2`/`P2` at `(5, 0)`:

```python
    room2 = spark_dsg.RoomNodeAttributes()
    room2.position = np.array([5, 0, 0])
    room2.semantic_label = 1  # road
    G.add_node(spark_dsg.DsgLayers.ROOMS, spark_dsg.NodeSymbol("R", 1).value, room2)

    place3 = spark_dsg.PlaceNodeAttributes()
    place3.position = np.array([5, 0, 0])
    G.add_node(spark_dsg.DsgLayers.PLACES, spark_dsg.NodeSymbol("p", 2).value, place3)
    place3_2d = spark_dsg.PlaceNodeAttributes()
    place3_2d.position = np.array([5.1, 0, 0])
    place3_2d.semantic_label = 4  # ground
    G.add_node(
        spark_dsg.DsgLayers.MESH_PLACES, spark_dsg.NodeSymbol("P", 2).value, place3_2d
    )
    G.insert_edge(
        spark_dsg.NodeSymbol("R", 1).value, spark_dsg.NodeSymbol("p", 2).value
    )
    G.insert_edge(
        spark_dsg.NodeSymbol("p", 1).value, spark_dsg.NodeSymbol("p", 2).value
    )
    G.insert_edge(
        spark_dsg.NodeSymbol("P", 1).value, spark_dsg.NodeSymbol("P", 2).value
    )
```

(Additive only — existing tests reference `o0/p1/r0` and `test_goal_relevant_is_smaller_than_full` compares relative sizes, so extra symbols are safe. Verify in Step 7.)

- [ ] **Step 5: Add the derived predicate to the domain**

In `RegionObjectRearrangementDomain.pddl`, add to `(:predicates ...)` after `(place-in-region ?p - place ?r - region)`:

```
        (object-in-region ?o - dsg_object ?r - region)
```

and add after the `(:derived (visited-region ...))` block:

```
    (:derived (object-in-region ?o - dsg_object ?r - region)
        (exists (?p - place) (and (object-in-place ?o ?p) (place-in-region ?p ?r))))
```

- [ ] **Step 6: Register the token**

In `dsg_pddl_grounding.py` `_NON_SYMBOL_TOKENS` (line ~386), extend the predicate line:

```python
    "holding", "hand-full", "object-in-place", "place-in-region",
    "object-in-region",
```

- [ ] **Step 7: Run the full test file — all pass**

Same command as Step 3. Expected: 7 passed. If FD chokes on the derived predicate in the goal, inspect the generated problem (`make_plan` temp dir) — the domain already uses `exists` inside `(:derived (in-region ...))`, so failures here indicate a problem-generation issue, not FD.

- [ ] **Step 8: Also run omniplanner's other test files** (regression):

```bash
zsh -c "source ~/dcist_ws/install/setup.zsh && source ~/environments/dcist/spark_env/bin/activate && cd ~/dcist_ws/src/awesome_dcist_t4/omniplanner/omniplanner && PYTHONPATH=src:examples python -m pytest tests/ -v"
```
Expected: all pass.

- [ ] **Step 9: Rebuild omniplanner into the workspace** (installed copy is what `omniplanner_node` runs): use the `building-adt4-workspace` skill; typically:

```bash
cd ~/dcist_ws && colcon build --packages-select omniplanner
```

- [ ] **Step 10: Commit**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/omniplanner
git add omniplanner/src/dsg_pddl/domains/RegionObjectRearrangementDomain.pddl \
        omniplanner/src/dsg_pddl/dsg_pddl_grounding.py \
        omniplanner/examples/utils.py omniplanner/tests/test_pddl_goal_relevant.py
git commit -m "feat(dsg_pddl): object-in-region derived predicate for NL block-verb goals"
```

---

### Task D2: Offline NL grounding harness `nl_grounding_check.py` (goal-only mode)

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/nl_grounding_check.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_nl_grounding_check.py`

**Interfaces:**
- Consumes: `dcist_sim_isaac.region_injector.augment_dsg_with_regions(G, regions)` (mutates G, raises on zero members); `dcist_sim_isaac.scenario.load_scenario(path)`; omniplanner `PddlDomain/PddlGoal/ground_problem/make_plan` (as in `omniplanner/omniplanner/tests/test_pddl_goal_relevant.py`); Task D1's `(object-in-region ...)`.
- Produces: CLI used by D3/D4 gates: `nl_grounding_check.py --goal-only "<goal>"` (no LLM) and `--sentence "..."` (LLM mode, D3); helper functions `validate_response_dict(text, robot)` and `check_plan(plan)` imported by its pytest.

The harness loads the REAL saved camp map, applies the SAME region augmentation the ingest path uses, and answers: (a) does `(object-in-region <cone> r0)` ground + FD-plan with pick+place, (b) how long does FD take, (c) which cone IDs are **degenerate** (already in-region → empty plan) — the input Task D3's prompt rule needs.

- [ ] **Step 1: Write the failing pytest**

`dcist_sim/dcist_sim_isaac/test/test_nl_grounding_check.py`:

```python
"""Pure-function tests for scripts/nl_grounding_check.py (no ROS/LLM/graph)."""
import pytest

from scripts.nl_grounding_check import (
    ResponseValidationError,
    check_plan,
    validate_response_dict,
)


def test_validate_response_accepts_well_formed_dict():
    goal = validate_response_dict('{"hilbert": "(object-in-region O2 R0)"}', "hilbert")
    assert goal == "(object-in-region O2 R0)"


def test_validate_response_rejects_wrong_robot():
    with pytest.raises(ResponseValidationError, match="hilbert"):
        validate_response_dict('{"hamilton": "(object-in-region O2 R0)"}', "hilbert")


def test_validate_response_rejects_non_dict():
    with pytest.raises(ResponseValidationError):
        validate_response_dict("(object-in-region O2 R0)", "hilbert")


def test_validate_response_rejects_goal_without_object_in_region():
    with pytest.raises(ResponseValidationError, match="object-in-region"):
        validate_response_dict('{"hilbert": "(visited-object O2)"}', "hilbert")


class _FakePlan:
    def __init__(self, actions):
        self.symbolic_actions = actions


def test_check_plan_requires_pick_and_place():
    ok, why = check_plan(_FakePlan([("goto-poi", "pstart", "p0"),
                                    ("pick-object", "o2", "p0"),
                                    ("goto-poi", "p0", "p5"),
                                    ("place-object", "o2", "p5")]))
    assert ok, why


def test_check_plan_flags_empty_plan_as_degenerate():
    ok, why = check_plan(_FakePlan([]))
    assert not ok
    assert "degenerate" in why
```

- [ ] **Step 2: Run to verify failure**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac
python3 -m pytest test/test_nl_grounding_check.py -v
```
Expected: FAIL — `ModuleNotFoundError: scripts.nl_grounding_check`.

- [ ] **Step 3: Write the harness**

`dcist_sim/dcist_sim_isaac/scripts/nl_grounding_check.py`:

```python
#!/usr/bin/env python3
"""Offline NL-grounding check for the camp mission (Phase D).

Loads the SAVED camp map (dsg_with_mesh.json), applies the same region
augmentation ingest_map.py performs, and validates the language-goal path
WITHOUT GPU/ROS:

  --goal-only "<pddl goal>"   skip the LLM; ground + FD-plan the given goal
  --sentence "<NL sentence>"  full path: OpenAI model -> {"robot": goal}
                              -> ground -> FD plan (needs ADT4_OPENAI_API_KEY)
  --degeneracy-report         for every cone Object, plan
                              (object-in-region <obj> <region>) and report
                              empty-plan (degenerate) vs pick+place
  --runs N                    repeat LLM mode N times (stability; temp=0)

Exit 0 iff every requested check passes. Run inside spark_env with the
workspace sourced and PYTHONPATH including dcist_sim/dcist_sim_isaac.
"""
from __future__ import annotations

import argparse
import ast
import os
import sys
import time

DEFAULT_DSG = os.path.expanduser("~/adt4_output/camp_sim_a/dsg_with_mesh.json")
DEFAULT_SCENARIO = os.path.join(
    os.path.dirname(__file__), "..", "..", "scenarios", "camp_smoke.yaml"
)
DEFAULT_LLM_CONFIG = os.path.expandvars(
    "$HOME/dcist_ws/src/awesome_dcist_t4/dcist_launch_system/config/isaac_sim/llm_config.yaml"
)
FD_BUDGET_S = 30.0


class ResponseValidationError(Exception):
    pass


def validate_response_dict(text, robot):
    """Parse the LLM answer text into a single-robot object-in-region goal.

    Mirrors omniplanner's language_planner: ast.literal_eval(text) must be a
    dict; here we additionally require exactly the target robot as key and an
    (object-in-region ...) goal, since that is the Phase D contract.
    """
    try:
        d = ast.literal_eval(text.strip())
    except (ValueError, SyntaxError) as e:
        raise ResponseValidationError(f"response is not a literal dict: {e}")
    if not isinstance(d, dict):
        raise ResponseValidationError(f"response is not a dict: {d!r}")
    if list(d.keys()) != [robot]:
        raise ResponseValidationError(
            f"expected exactly {{'{robot}': goal}}, got keys {list(d.keys())}"
        )
    goal = d[robot]
    if "object-in-region" not in goal:
        raise ResponseValidationError(
            f"goal does not use object-in-region: {goal!r}"
        )
    return goal


def check_plan(plan):
    """(ok, why): plan must contain both pick-object and place-object."""
    names = [a[0] for a in plan.symbolic_actions]
    if not names:
        return False, "degenerate: FD returned an empty plan (goal true at init)"
    if "pick-object" not in names:
        return False, f"no pick-object in plan: {names}"
    if "place-object" not in names:
        return False, f"no place-object in plan: {names}"
    return True, f"plan ok ({len(names)} actions): {names}"


def load_augmented_graph(dsg_path, scenario_path):
    import spark_dsg

    from dcist_sim_isaac.region_injector import augment_dsg_with_regions
    from dcist_sim_isaac.scenario import load_scenario

    G = spark_dsg.DynamicSceneGraph.load(dsg_path)
    scenario = load_scenario(scenario_path)
    summary = augment_dsg_with_regions(G, scenario.regions)
    print(f"[nl_check] region augmentation: {summary}")
    return G, scenario


def ground_and_plan(G, goal_str, robot, robot_xy):
    import numpy as np
    from importlib.resources import as_file, files

    import dsg_pddl.domains
    from dsg_pddl.dsg_pddl_grounding import ground_problem
    from dsg_pddl.dsg_pddl_planning import make_plan
    from dsg_pddl.pddl_grounding import PddlDomain, PddlGoal

    with as_file(
        files(dsg_pddl.domains).joinpath("RegionObjectRearrangementDomain.pddl")
    ) as p:
        domain = PddlDomain(open(p).read())
    domain.scene_scope = "goal_relevant"
    goal = PddlGoal(robot_id=robot, pddl_goal=goal_str)
    t0 = time.monotonic()
    grounded = ground_problem(domain, G, {robot: np.array(robot_xy)}, goal)
    plan = make_plan(grounded.value, G)
    dt = time.monotonic() - t0
    return plan, dt


def cone_objects(G):
    """[(symbol_lower, label, (x, y))] for every cone-class OBJECTS node."""
    import spark_dsg

    out = []
    ls = G.get_labelspace(2, 0)
    for n in G.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes:
        label = ls.get_category(n.attributes.semantic_label) if ls else "?"
        if "cone" in str(label).lower():
            sym = n.id.str(True).lower()
            out.append((sym, label, tuple(n.attributes.position[:2])))
    return out


def region_symbol(G, label):
    import spark_dsg

    ls = G.get_labelspace(4, 0)
    for n in G.get_layer(spark_dsg.DsgLayers.ROOMS).nodes:
        cat = ls.get_category(n.attributes.semantic_label) if ls else "?"
        if str(cat).lower() == label.lower():
            return n.id.str(True).lower()
    raise SystemExit(f"[nl_check] FAIL: no ROOMS node labeled {label!r}")


def request_llm_goal(sentence, G, llm_config_path):
    from ruamel.yaml import YAML

    from nlu_interface.config import OpenAIConfig
    from nlu_interface.interface import load_prompt_from_name
    from nlu_interface_dcist.language_planning_interface import (
        LanguagePddlInterface,
        SimplePddlSceneGraphPrompt,
    )

    yaml = YAML(typ="safe")
    with open(llm_config_path) as f:
        llm = yaml.load(f)
    prompt = SimplePddlSceneGraphPrompt(**load_prompt_from_name(llm["prompt"]))
    cfg = OpenAIConfig(
        model=llm["model"],
        prompt_mode=llm.get("mode", "default"),
        num_incontext_examples=llm["num_incontext_examples"],
        temperature=llm["temperature"],
        api_timeout=llm["api_timeout"],
        seed=llm["seed"],
        api_key_env_var=llm.get("api_key_env_var", ""),
        debug=llm.get("debug", False),
    )
    iface = LanguagePddlInterface(config=cfg, prompt=prompt)
    print(f"[nl_check] model={llm['model']} sentence={sentence!r}")
    t0 = time.monotonic()
    text = iface.request_plan_specification(sentence, G)
    print(f"[nl_check] LLM response ({time.monotonic() - t0:.1f}s): {text}")
    return text


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dsg", default=DEFAULT_DSG)
    ap.add_argument("--scenario", default=DEFAULT_SCENARIO)
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument("--robot-xy", nargs=2, type=float, default=[0.0, 0.0])
    ap.add_argument("--goal-only", default=None, metavar="PDDL_GOAL")
    ap.add_argument("--sentence", default=None)
    ap.add_argument("--llm-config", default=DEFAULT_LLM_CONFIG)
    ap.add_argument("--degeneracy-report", action="store_true")
    ap.add_argument("--runs", type=int, default=1)
    args = ap.parse_args(argv)

    G, scenario = load_augmented_graph(args.dsg, args.scenario)
    failures = 0

    if args.degeneracy_report:
        rsym = region_symbol(G, scenario.regions[0].label)
        print(f"[nl_check] degeneracy report vs region {rsym}:")
        for sym, label, pos in cone_objects(G):
            plan, dt = ground_and_plan(
                G, f"(object-in-region {sym} {rsym})", args.robot, args.robot_xy
            )
            ok, why = check_plan(plan)
            print(f"  {sym} ({label} @ {pos}): FD {dt:.1f}s -> {why}")

    if args.goal_only:
        plan, dt = ground_and_plan(G, args.goal_only, args.robot, args.robot_xy)
        ok, why = check_plan(plan)
        print(f"[nl_check] goal-only: FD {dt:.1f}s -> {why}")
        if not ok or dt > FD_BUDGET_S:
            failures += 1

    if args.sentence:
        for i in range(args.runs):
            try:
                text = request_llm_goal(args.sentence, G, args.llm_config)
                goal = validate_response_dict(text, args.robot)
                plan, dt = ground_and_plan(G, goal, args.robot, args.robot_xy)
                ok, why = check_plan(plan)
                print(f"[nl_check] run {i + 1}/{args.runs}: goal={goal!r} "
                      f"FD {dt:.1f}s -> {why}")
                if not ok or dt > FD_BUDGET_S:
                    failures += 1
            except ResponseValidationError as e:
                print(f"[nl_check] run {i + 1}/{args.runs}: FAIL {e}")
                failures += 1

    print(f"[nl_check] {'PASS' if failures == 0 else f'FAIL ({failures})'}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
```

NOTE on `load_prompt_from_name`: the ROS side loads the prompt YAML by name from `nlu_interface.resources` (`language_planner_ros.py:49-54`). If `nlu_interface.interface` does not export such a helper, inline the same 3 lines the ROS plugin uses (importlib.resources on `nlu_interface.resources`, `yaml.load`, `SimplePddlSceneGraphPrompt(**d)`) — copy them verbatim from `language_planner_ros.py`, do NOT invent a new loading scheme.

- [ ] **Step 4: pytest passes**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac
python3 -m pytest test/test_nl_grounding_check.py -v
```
Expected: 6 passed.

- [ ] **Step 5: Integration — goal-only + degeneracy report against the real map**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
zsh -c "source ~/dcist_ws/install/setup.zsh && source ~/environments/dcist/spark_env/bin/activate && \
  export PYTHONPATH=\$PWD/dcist_sim/dcist_sim_isaac:\$PYTHONPATH && \
  python3 dcist_sim/dcist_sim_isaac/scripts/nl_grounding_check.py --degeneracy-report \
    --goal-only '(object-in-region <NONDEGENERATE_CONE_SYM> r0)'"
```
First run with only `--degeneracy-report` to learn the cone symbols and which are degenerate; then re-run `--goal-only` with a NON-degenerate cone symbol from the report. Expected: report lists 3 cone nodes; `--goal-only` prints `plan ok` with `pick-object` + `place-object`, FD well under 30 s; exit 0. **Record the degeneracy table in the task report — Task D3's prompt rule depends on it.**

- [ ] **Step 6: Full dcist_sim suite stays green**

```bash
zsh -c "source ~/dcist_ws/install/setup.zsh && cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac && python3 -m pytest test/ -v"
```
Expected: 228/228 (222 existing + 6 new).

- [ ] **Step 7: Commit (dcist_sim)**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim
git add dcist_sim_isaac/scripts/nl_grounding_check.py dcist_sim_isaac/test/test_nl_grounding_check.py
git commit -m "feat(scripts): offline NL-grounding harness for Phase D (goal-only + degeneracy report)"
```

---

### Task D3: Block-verb prompt few-shots (nlu_interface) + LLM-mode validation

**Files:**
- Modify: `nlu_interface/nlu_interface/src/nlu_interface/resources/prompt_pnp_pddl_planner.yaml`
- Validation: `nl_grounding_check.py --sentence ... --runs 5` (Task D2's harness; no new test file — the harness IS the test)

**Interfaces:**
- Consumes: Task D1's `(object-in-region ...)` predicate; Task D2's harness.
- Produces: a prompt that maps `"Hilbert, block the intersection with a cone"` → `{"hilbert": "(object-in-region <cone-sym> <region-sym>)"}` reliably at temperature 0.

- [ ] **Step 1: Create the harelb fork + branch** (none exists yet — `gh repo view harelb/nlu_interface` currently 404s):

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/nlu_interface
gh repo fork robustrobotics/nlu_interface --clone=false
git remote add harelb git@github.com:harelb/nlu_interface.git
git switch -c feature/camp_mission_nl 1219c89
```
If the fork fails (permissions), STOP and report — do not push to origin.

- [ ] **Step 2: Extend the `<PDDL Domain>` block**

In `prompt_pnp_pddl_planner.yaml`, after the `object-in-place` paragraph (line ~35), insert:

```
    (object-in-region ?o ?r): This predicate indicates that an Object '?o' must end up inside Region '?r', where '?o' is a placeholder for an Object ID and '?r' is a placeholder for a Region ID.
    The 'object-in-region' predicate is useful for commands that ask a robot to move, put, carry, or place an object into a region, or to block a region with an object. For example, "block the courtyard with a barrel" means some barrel Object must end up inside the courtyard Region. When several Objects match the requested type, prefer an Object whose parent_region is NOT already the target Region (blocking a region with an object that is already there requires no work); among equally valid choices, pick the lowest Object ID.
```

- [ ] **Step 3: Add few-shot examples**

Append to `incontext_examples` (the example scene graph at lines 47-60 stays unchanged; boats exist only in R1, vehicles in R0 and R1):

```yaml
- example_input: "Hamilton, block the parking lot with a boat."
  example_output: "{\"hamilton\": \"(object-in-region O5 R0)\"}"
- example_input: "Hilbert, block the courtyard with a vehicle."
  example_output: "{\"hilbert\": \"(object-in-region O1 R2)\"}"
- example_input: "Euclid, put the tree in the dock area."
  example_output: "{\"euclid\": \"(object-in-region O0 R1)\"}"
```

(Example 1: only boat is O5, in R1 ≠ R0 target. Example 2: vehicles O1 (R0) and O4 (R1) — both outside target R2 → lowest ID O1. Example 3: trees O0 (R0) and O3 (R2) — both outside target R1 → lowest ID O0; also teaches the non-"block" phrasing.)

- [ ] **Step 4: Rebuild nlu_interface** (resources are installed): `building-adt4-workspace` skill; typically

```bash
cd ~/dcist_ws && colcon build --packages-select nlu_interface
```

- [ ] **Step 5: LLM-mode validation with the CURRENT model (gpt-4.1) — isolate prompt changes from the model switch**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
zsh -c "source ~/dcist_ws/install/setup.zsh && source ~/environments/dcist/spark_env/bin/activate && \
  export PYTHONPATH=\$PWD/dcist_sim/dcist_sim_isaac:\$PYTHONPATH && \
  python3 dcist_sim/dcist_sim_isaac/scripts/nl_grounding_check.py \
    --sentence 'Hilbert, block the intersection with a cone' --runs 5"
```
Expected: 5/5 `plan ok` with pick+place, exit 0. **Check the chosen cone symbol against Task D2's degeneracy table**: if the LLM picks a degenerate cone (empty plan), inspect the printed `parent_region` values in the prompt (add a temporary `print` of `scene_graph_to_prompt(G)` in the harness if needed — or run with `debug: true`, which the config already sets).
  - **Contingency (only if `parent_region` is `none` for the in-region cone, so the rule can't fire):** extend `region_to_prompt` in `nlu_interface/nlu_interface_dcist/src/nlu_interface_dcist/language_planning_interface.py:44-52` to include the region's position — `(id=..., type=..., pos=(x,y))` (RoomNodeAttributes has `.position`), update the `<Scene Graph Description>` Region sentence accordingly, change the prompt rule to "prefer the matching Object whose pos is NOT closest to the target Region's pos", rebuild `nlu_interface_dcist` too, and re-run 5/5.

- [ ] **Step 6: Commit (nlu_interface)**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/nlu_interface
git add nlu_interface/src/nlu_interface/resources/prompt_pnp_pddl_planner.yaml
git commit -m "feat(prompt): block-verb / object-in-region few-shots for camp mission NL"
```
(Include `language_planning_interface.py` in the add if the contingency fired.)

---

### Task D4: gpt-mini model switch (isaac_sim-only overlay)

**Files:**
- Create: `dcist_launch_system/config_generation/experiment_overrides/isaac_sim/llm_config_overlay.yaml`
- Regenerated: `dcist_launch_system/config/isaac_sim/llm_config.yaml` (via generator ONLY)

**Interfaces:**
- Consumes: overlay naming rule `<base>_overlay.yaml` → composes onto `base_params/llm_config.yaml` (`generate_configs.py:200-223`); Task D3's validated prompt.
- Produces: `config/isaac_sim/llm_config.yaml` with `model: gpt-4.1-mini-2025-04-14`; all other configs unchanged.

- [ ] **Step 1: Invoke the repo `adt4-config-generation` skill** (it governs sources → generation → checks).

- [ ] **Step 2: Write the overlay**

`dcist_launch_system/config_generation/experiment_overrides/isaac_sim/llm_config_overlay.yaml`:

```yaml
# isaac_sim: Phase D live-NL uses a mini model (fast + cheap; block-verb
# grounding validated 5/5 offline via nl_grounding_check.py, 2026-07-23).
# Real-robot configs keep the base gpt-4.1. Model string must be in
# OpenAIWrapper.valid_model_names (nlu_interface/src/nlu_interface/interface.py).
model: gpt-4.1-mini-2025-04-14
```

- [ ] **Step 3: Regenerate + check** (tree must be otherwise clean — commit D2's dcist_sim work first, which Step 7 of D2 did):

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_launch_system
./scripts/generate_configs.sh && ./scripts/check_configs.sh
```
Expected: `config/isaac_sim/llm_config.yaml` now shows `model: gpt-4.1-mini-2025-04-14` (everything else identical); `git diff --stat` shows ONLY isaac_sim llm_config + the new overlay; `config/default/llm_config.yaml` untouched; check_configs exit 0.

- [ ] **Step 4: Re-run the 5/5 stability check on the mini model** (same command as Task D3 Step 5 — the harness reads the regenerated `config/isaac_sim/llm_config.yaml`). Expected: 5/5 `plan ok`, exit 0. If the mini model is unreliable (<5/5), try `gpt-4.1-nano-2025-04-14`? NO — escalate the few-shots first (add one more targeted example of the failing pattern), and only if still failing document the failure mode and stay on `gpt-4.1-mini-2025-04-14` with the extra examples; do not silently change models.

- [ ] **Step 5: Commit (superproject)**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
git add dcist_launch_system/config_generation/experiment_overrides/isaac_sim/llm_config_overlay.yaml \
        dcist_launch_system/config/isaac_sim/llm_config.yaml
git commit -m "feat(config): isaac_sim llm_config -> gpt-4.1-mini (Phase D live NL)"
```
(If generation touched generated tmux files too, include them — they are generator products.)

---

### Task D5: `mission_cli --nl` robot-name guard + `camp_mission_smoke --nl` mode

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/mission_cli.py:425-434` (NL branch)
- Modify: `dcist_sim/dcist_sim_isaac/scripts/camp_mission_smoke.py` (argparser ~line 501-532; `phase_publish_mission` ~line 333-336)
- Test: `dcist_sim/dcist_sim_isaac/test/test_mission_cli.py`

**Interfaces:**
- Consumes: existing `_publish_language_goal(robot, command)`; smoke's `run_streamed`/Popen helpers and `MISSION_CLI_PY`.
- Produces: pure helper `nl_command(sentence, robot) -> str` in mission_cli (used by tests); smoke flag `--nl`; NL evidence file `<out>/llm_response.txt`.

Rationale: in the `Pddl` branch omniplanner IGNORES `LanguageGoalMsg.robot_id` — the robot is dispatched from the LLM output-dict key, which the model derives from the robot name IN the sentence. A sentence without the robot's name silently dispatches to whatever robot the LLM guesses.

- [ ] **Step 1: Write the failing tests** (append to `test/test_mission_cli.py`):

```python
from scripts.mission_cli import nl_command


def test_nl_command_prefixes_robot_when_absent():
    assert (
        nl_command("block the intersection with a cone", "hilbert")
        == "Hilbert, block the intersection with a cone"
    )


def test_nl_command_keeps_sentence_when_robot_present():
    s = "Hilbert, block the intersection with a cone"
    assert nl_command(s, "hilbert") == s


def test_nl_command_robot_match_is_case_insensitive():
    s = "hilbert please block the intersection with a cone"
    assert nl_command(s, "Hilbert") == s
```

- [ ] **Step 2: Run to verify failure**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac
python3 -m pytest test/test_mission_cli.py -v
```
Expected: 3 new FAIL (ImportError: `nl_command`), 17 existing PASS.

- [ ] **Step 3: Implement `nl_command` + use it in the NL branch**

In `mission_cli.py`, add above `build_arg_parser()`:

```python
def nl_command(sentence, robot):
    """Ensure the robot's name appears in the NL sentence.

    omniplanner's language planner (Pddl branch) dispatches on the robot key
    of the LLM's output dict, NOT on LanguageGoalMsg.robot_id
    (omniplanner/src/omniplanner/language_planner.py:54-56) -- the name in the
    sentence is what routes the mission. Prefix it if missing.
    """
    if robot.lower() in sentence.lower():
        return sentence
    return f"{robot.capitalize()}, {sentence}"
```

and change the NL branch of `main()` (lines 425-434) to:

```python
    if args.nl:
        command = nl_command(args.sentence, args.robot)
        if command != args.sentence:
            print(f"[mission_cli] NL: prefixed robot name -> {command!r}")
        topic = LANGUAGE_GOAL_TOPIC.format(robot=args.robot)
        if args.dry_run:
            print(f"[mission_cli] NL mode: command={command!r}")
            print(f"[mission_cli] topic: {topic}")
            print("[mission_cli] domain_type: Pddl")
            return 0
        _publish_language_goal(args.robot, command)
        print(f"[mission_cli] published LanguageGoalMsg to {topic}")
        return 0
```

Also update the `--nl` help string: drop "(smoke-tested only; tuned in Phase D)" → "(live NL path, Phase D)".

- [ ] **Step 4: Tests pass** (same command as Step 2). Expected: 20 passed.

- [ ] **Step 5: Add `--nl` to camp_mission_smoke**

In `build_arg_parser()` add:

```python
    ap.add_argument(
        "--nl",
        action="store_true",
        help="Phase D: publish the mission via the language planner "
        "(mission_cli --nl -> LanguageGoalMsg -> LLM) instead of the "
        "scripted PddlGoalMsg path",
    )
```

In `phase_publish_mission` (line ~333), pass the flag through and, in NL mode, start an evidence subscriber BEFORE publishing (adapt to the file's existing subprocess helpers — `ros2 topic echo --once` exits after the first message; non-fatal on timeout):

```python
def phase_publish_mission(args, out_dir):
    llm_echo = None
    if args.nl:
        llm_log = open(os.path.join(out_dir, "llm_response.txt"), "w")
        llm_echo = subprocess.Popen(
            ["ros2", "topic", "echo", "--once",
             f"/{args.robot}/omniplanner_node/llm_response",
             "std_msgs/msg/String"],
            stdout=llm_log, stderr=subprocess.STDOUT,
        )
    cmd = [sys.executable, MISSION_CLI_PY, args.mission, "--robot", args.robot]
    if args.nl:
        cmd.append("--nl")
    ...  # existing run_streamed invocation of cmd, unchanged
    if llm_echo is not None:
        try:
            llm_echo.wait(timeout=180.0)
        except subprocess.TimeoutExpired:
            llm_echo.kill()
            print("[smoke] WARN: no llm_response within 180s (evidence only)")
```

Match the function's real current signature/idiom when editing (it may take `args` only — derive `out_dir` the same way the video path is derived). Keep the verifier and every timeout unchanged; the LLM roundtrip fits inside the existing 300 s verify window.

- [ ] **Step 6: Dry-run sanity (no ROS graph needed for --dry-run)**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
zsh -c "source ~/dcist_ws/install/setup.zsh && source ~/environments/dcist/spark_env/bin/activate && \
  python3 dcist_sim/dcist_sim_isaac/scripts/mission_cli.py \
    'block the intersection with a cone' --robot hilbert --nl --dry-run"
```
Expected output includes `NL: prefixed robot name -> 'Hilbert, block the intersection with a cone'` and the language_goal topic.

- [ ] **Step 7: Full suite green + commit (dcist_sim)**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac
zsh -c "source ~/dcist_ws/install/setup.zsh && python3 -m pytest test/ -v"
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim
git add dcist_sim_isaac/scripts/mission_cli.py dcist_sim_isaac/scripts/camp_mission_smoke.py \
        dcist_sim_isaac/test/test_mission_cli.py
git commit -m "feat(mission): --nl mode for camp_mission_smoke + robot-name guard in mission_cli NL path"
```

---

### Task D6: GPU e2e gate — 2 consecutive live-NL mission passes (gateE, gateF)

**Files:** none created (evidence task). Outputs: `~/adt4_output/camp_mission_gateE/`, `~/adt4_output/camp_mission_gateF/` (scene-graph folder + `mission_video/capture.mp4` + `llm_response.txt` each).

**Interfaces:**
- Consumes: everything D1-D5; runbook §13.1 pipeline; strict verifier unchanged.
- Produces: Phase D gate evidence per spec §5 row D.

- [ ] **Step 1: Pre-flight** (runbook §13.4 traps):
  - `nvidia-smi` — no stray SAM3/Isaac processes.
  - Reap orphan launch children between runs: `pgrep -af static_transform_publisher` and kill any stale ones by PID (the 2026-07-21 `/hilbert` orphans class).
  - Neo4j container up (`docker ps`); `HERACLES_NEO4J_*` env set.
  - `echo ${ADT4_OPENAI_API_KEY:+set}` prints `set` (do NOT echo the value); `grep model ~/dcist_ws/src/awesome_dcist_t4/dcist_launch_system/config/isaac_sim/llm_config.yaml` shows the mini model.
  - Confirm rebuilt `omniplanner` + `nlu_interface` are installed (D1 Step 9, D3 Step 4).

- [ ] **Step 2: Run gateE**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
zsh -c "source ~/dcist_ws/install/setup.zsh && source ~/environments/dcist/spark_env/bin/activate && \
  export PYTHONPATH=\$PWD/dcist_sim/dcist_sim_isaac:\$PYTHONPATH && \
  python3 dcist_sim/dcist_sim_isaac/scripts/camp_mission_smoke.py \
    --output-dir ~/adt4_output/camp_mission_gateE --nl"
```
Expected: exit 0 (strict verifier: robot odom pose in-region at the held→released instant, AND `capture.mp4` non-empty). Also verify `llm_response.txt` contains the LLM dict with an `object-in-region` goal for `hilbert`.

- [ ] **Step 3: Reap orphans (Step 1's check again), then run gateF** — same command with `camp_mission_gateF`. Expected: exit 0.

- [ ] **Step 4: Failure protocol** — on any failure, treat it with systematic-debugging (the C2 precedent: 11 attempts were needed before the frame-mismatch root cause). First suspects, in order: (a) LLM picked a degenerate cone (empty plan — check `llm_response.txt` + omniplanner log "Setting DSG!"/grounding lines, cross-check D2's degeneracy table); (b) served-graph labelspace missing → `scene_graph_to_prompt` ValueError in the omniplanner pane (fix belongs in ingest/labelspace embedding, NOT by patching nlu at runtime); (c) region-domain plan latency (compare FD wall time to D2's measurement). Best-guess iterate; do NOT weaken the verifier or timeouts to pass.

- [ ] **Step 5: Note in the task report** — this run also implicitly confirms the final-review Important (identity `map -> <robot>/map` TF in the `hydra_isaac` component under a live GPU session, flagged 2026-07-23 to confirm on next GPU run).

---

### Task D7: Docs, spec close-out, submodule bumps, push

**Files:**
- Modify: `docs/sim_runbook.md` (§13: new §13.5 "Live NL mode (Phase D)" + §13.3 gate table row)
- Modify: `docs/superpowers/specs/2026-07-22-camp-mission-sim-design.md` (status note + §5 row D → GATE MET)
- Modify: `dcist_sim/dcist_sim_isaac/README.md` (one-liner for `nl_grounding_check.py`)
- Superproject: submodule pointer bumps (omniplanner, nlu_interface, dcist_sim) + this plan file.

**Interfaces:** consumes D1-D6 evidence; produces the pushed, documented Phase D close-out.

- [ ] **Step 1: Runbook §13.5** — document: the NL flow (`mission_cli --nl` → `LanguageGoalMsg` → `language_planner` → gpt-4.1-mini → `{"hilbert": "(object-in-region ...)"}` → `RegionObjectRearrangementDomain` goal_relevant grounding → FD), the robot-name-in-sentence dispatch rule, the offline iteration loop (`nl_grounding_check.py` commands incl. `--degeneracy-report`), the model overlay location, and gateE/gateF evidence paths. Update §13.3 with the D row.

- [ ] **Step 2: Spec close-out** — §5 table row D → `GATE MET` (+ caveats if any), status paragraph appended (mirror the Phase A-C close-out style).

- [ ] **Step 3: Push submodules to harelb**

```bash
cd ~/dcist_ws/src/awesome_dcist_t4/omniplanner   && git push harelb feature/camp_mission_nl
cd ~/dcist_ws/src/awesome_dcist_t4/nlu_interface && git push harelb feature/camp_mission_nl
cd ~/dcist_ws/src/awesome_dcist_t4/dcist_sim     && git push harelb feature/camp_mission
```

- [ ] **Step 4: Superproject commit + push** (docs + pointer bumps in ONE commit, after a final whole-diff review):

```bash
cd ~/dcist_ws/src/awesome_dcist_t4
git add docs/sim_runbook.md docs/superpowers/specs/2026-07-22-camp-mission-sim-design.md \
        docs/superpowers/plans/2026-07-23-camp-mission-phaseD-live-nl.md \
        omniplanner nlu_interface dcist_sim
git commit -m "feat(sim): Phase D live-NL camp mission (gpt-4.1-mini) -- gateE/gateF, runbook §13.5"
git push harelb feature/isaac_sim_camp_mission
```
CAUTION: `git add nlu_interface` etc. stage pointer bumps — verify with `git submodule status` that each recorded SHA is the pushed tip. Note `.gitmodules`: nlu_interface's recorded URL stays `robustrobotics` (pointer commits exist only on the harelb fork — flag this in the commit body so a fresh clone knows to add the harelb remote; same pattern as other harelb-only submodule branches).

- [ ] **Step 5: Update memory** (`project_camp_mission.md`): Phase D complete + evidence paths + NEXT: E physics, F fleet.

---

## Self-Review (done at write time)

- **Spec coverage:** spec row D ("same mission from the NL sentence") → D6; gpt-mini backend → D4; prompt block-verb resolution → D3; mission_cli NL publish → existed, hardened in D5; outputs (map folder + video) → smoke defaults, D6.
- **Placeholder scan:** two intentional adaptation points are called out explicitly with their authoritative sources (D2 `load_prompt_from_name` note → copy from `language_planner_ros.py:49-54`; D5 Step 5 "match the function's real current signature") — these are verbatim-copy instructions, not TBDs.
- **Type consistency:** goal string `(object-in-region <obj> <region>)` used identically in D1 test, D2 harness, D3 prompt; `nl_command(sentence, robot)` name consistent across D5 steps; gate dirs gateE/gateF consistent across D6/D7.
