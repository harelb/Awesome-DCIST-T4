#!/usr/bin/env python3
"""A stand-in for GRSTAPS-X's ``itags`` binary: ``fake_itags.py <input> <output>``.

Reads the itags_input.json omniplanner's GRSTAPS-X grounding wrote and writes an
itags_solution.json in the real solver's format, with a simple greedy allocation:
tasks in precedence order; a task demanding robot identities (a ``-by`` pin) gets
those robots; a coalition (ground demand > 1) the nearest robots; any other task
the robot free earliest. Travel is a straight line at 1 m/s. Lets the console's
tests run the whole GRSTAPS-X path without Gurobi. ``FAKE_ITAGS_FAIL=<text>``
prints that as the solver's error and writes no solution.
"""

import json
import math
import os
import sys

TRAIT_DIM, GROUND = 8, 0


def main(inp, out):
    if os.environ.get("FAKE_ITAGS_FAIL"):
        print(f"[2026-11-02 00:00:00.000] [error] {os.environ['FAKE_ITAGS_FAIL']}")
        return 1
    data = json.load(open(inp))
    tasks, robots = data["tasks"], data["robots"]
    n = len(tasks)
    pre = {j: [] for j in range(n)}
    for a, b in data.get("precedence_constraints") or []:
        pre[b].append(a)
    order, seen = [], set()

    def visit(i):
        if i in seen:
            return
        seen.add(i)
        for p in pre[i]:
            visit(p)
        order.append(i)

    for i in range(n):
        visit(i)
    pos = {r["name"]: (r["initial_configuration"]["x"], r["initial_configuration"]["y"]) for r in robots}
    vid = {r["name"]: r["initial_configuration"]["id"] for r in robots}
    free = {r["name"]: 0.0 for r in robots}
    names = [r["name"] for r in robots]
    plans = {r: [] for r in names}
    legs = {r: [] for r in names}
    out_tasks = [None] * n
    finish = {}
    for i in order:
        t = tasks[i]
        traits = t["desired_traits"]
        cfg, end = t["initial_configuration"], t["terminal_configuration"]
        pinned = [names[k - TRAIT_DIM] for k in range(TRAIT_DIM, len(traits))
                  if traits[k] > 0 and k - TRAIT_DIM < len(names)]  # fmt: skip
        need = max(1, int(round(traits[GROUND])), len(pinned))

        def arrive(r):
            return free[r] + math.dist(pos[r], (cfg["x"], cfg["y"]))

        team = list(pinned)
        for r in sorted(names, key=arrive):
            if len(team) >= need:
                break
            if r not in team:
                team.append(r)
        ready = max([finish[p] for p in pre[i]], default=0.0)
        start = max([arrive(r) for r in team] + [ready])
        end_t = start + float(t["duration"])
        for r in team:
            legs[r].append({"path": [{"id": vid[r], "x": pos[r][0], "y": pos[r][1]},
                                     {"id": cfg["id"], "x": cfg["x"], "y": cfg["y"]}]})  # fmt: skip
            plans[r].append(i)
            pos[r], vid[r], free[r] = (end["x"], end["y"]), end["id"], end_t
        finish[i] = end_t
        out_tasks[i] = {"id": i, "name": t["name"], "start_timepoint": start, "finish_timepoint": end_t,
                        "coalition": [names.index(r) for r in team]}  # fmt: skip
    sol = {"makespan": max(finish.values(), default=0.0), "tasks": out_tasks,
           "agents": [{"name": r, "individual_plan": plans[r], "transitions": legs[r]} for r in names],
           "precedence_constraints": data.get("precedence_constraints") or []}  # fmt: skip
    json.dump({"solution": sol, "statistics": {"fake": True}}, open(out, "w"))
    print("[info] fake itags solved")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1], sys.argv[2]))
