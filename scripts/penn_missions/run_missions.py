"""Build each Penn mission on a running console (no ROS):

  python run_missions.py missions.yaml http://127.0.0.1:8099 OUT [--chat] [--only M1,M2]

Per mission writes OUT/<id>/: command.txt, goal.pddl, constraints.json, plan.json
(mission + planner info), preview.json (routes / actions / schedule), schedule.csv,
goal_images/*.jpg (object crops with masks, near-frames for places/regions) and,
with --chat, chat.json (what the console's LLM proposed for the command).
"""
import argparse, csv, json, pathlib, re, sys, urllib.parse, urllib.request
import yaml

SRC = "neo4j:pennmissions"


def call(base, path, body=None, timeout=600, raw=False):
    req = urllib.request.Request(base + path, json.dumps(body).encode() if body is not None else None,
                                 {"content-type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        data = r.read()
    return data if raw else json.loads(data)


def save_img(base, path, dest):
    try:
        dest.write_bytes(call(base, path, raw=True, timeout=120))
        return True
    except Exception as e:  # noqa: BLE001
        print("   image failed", path, e)
        return False


def goal_images(base, d, pv):
    import shutil
    shutil.rmtree(d / "goal_images", ignore_errors=True); (d / "goal_images").mkdir()
    n = 0
    for robot, goals in (pv.get("goals") or {}).items():
        for g in goals:
            sym = g["sym"]
            n += 1
            stem = d / "goal_images" / f"{n:02d}_{robot}_{re.sub(r'[^A-Za-z0-9]', '', sym)}"
            imaged = False
            if sym.startswith("O("):
                imaged = save_img(base, f"/api/graph/{urllib.parse.quote(SRC)}/image/{urllib.parse.quote(sym)}/0?w=640&mask=1&crop=1", stem.with_suffix(".jpg"))
                if imaged:
                    save_img(base, f"/api/graph/{urllib.parse.quote(SRC)}/image/{urllib.parse.quote(sym)}/0?w=960&mask=1", pathlib.Path(str(stem) + "_context.jpg"))
            if not imaged and g.get("xy"):  # places / regions, and objects with no observation image: the view from the goal pose
                x, y = g["xy"]
                q = f"x={x}&y={y}" + (f"&yaw={g['yaw']}" if g.get("yaw") is not None else "") + "&n=1"
                save_img(base, f"/api/graph/{urllib.parse.quote(SRC)}/near/image?{q}&i=0&w=640", stem.with_suffix(".jpg"))


def chat(base, model, agents, m, d):
    """The command through the console's chat (mission planner context); what goal it proposed."""
    ctx = {"roster": agents, "engine": m["engine"]}
    raw = call(base, "/api/chat", {"message": m["text"], "src": SRC, "model": model, "mission_ctx": ctx},
               raw=True, timeout=900).decode()
    events = [json.loads(l) for l in raw.splitlines() if l.strip()]
    (d / "chat.json").write_text(json.dumps(events, indent=1))
    goals = re.findall(r'"pddl_goal"\s*:\s*"([^"]+)"', raw) + re.findall(r'"pddl"\s*:\s*"([^"]+)"', raw)
    usage = call(base, "/api/chat/usage")
    out = {"chat_goal": goals[-1] if goals else None, "llm_spent_usd": usage.get("spent_usd")}
    print(m["id"], "chat goal:", out["chat_goal"], "| spent $%.2f" % (usage.get("spent_usd") or 0), flush=True)
    if (usage.get("spent_usd") or 0) > 9.0:
        raise SystemExit("LLM budget nearly spent; stopping")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("catalog"); ap.add_argument("base"); ap.add_argument("out")
    ap.add_argument("--chat", action="store_true"); ap.add_argument("--only")
    ap.add_argument("--model", default="gpt-6-luna")
    ap.add_argument("--images-only", action="store_true", help="redo goal_images from the saved preview.json")
    ap.add_argument("--chat-only", action="store_true", help="only send each command through the console chat (LLM)")
    a = ap.parse_args()
    cat = yaml.safe_load(open(a.catalog))
    base = a.base.rstrip("/")
    out = pathlib.Path(a.out)
    agents = [{"id": n, "kind": v["kind"], "start": v["start"], "binding": None} for n, v in cat["starts"].items()]
    robots = [{"name": n, "start": v["start"], "kind": v["kind"]} for n, v in cat["starts"].items()]
    only = set(a.only.split(",")) if a.only else None
    index = {}
    for m in cat["missions"]:
        if only and m["id"] not in only:
            continue
        d = out / m["id"]; (d / "goal_images").mkdir(parents=True, exist_ok=True)
        (d / "command.txt").write_text(m["text"] + "\n")
        (d / "goal.pddl").write_text(m["goal"] + "\n")
        cons = m.get("constraints") or []
        (d / "constraints.json").write_text(json.dumps(cons, indent=1))
        if a.chat_only:
            res = json.loads((d / "result.json").read_text()) if (d / "result.json").exists() else {"id": m["id"]}
            res.update(chat(base, a.model, agents, m, d))
            (d / "result.json").write_text(json.dumps(res, indent=1))
            index[m["id"]] = res
            continue
        if a.images_only:
            pv = json.loads((d / "preview.json").read_text())
            goal_images(base, d, pv)
            continue
        body = {"text": m["text"], "agents": agents, "src": SRC, "pddl_goal": m["goal"], "constraints": cons,
                "engine": m["engine"]}
        planned = call(base, "/api/missions/plan/omniplanner", body)
        mission = planned["mission"]
        mid = mission["id"]
        try:  # keep it in the Saved list under a readable name
            call(base, f"/api/missions/{mid}/save", {"name": f"{m['id']} {m['title']}"})
        except Exception as e:  # noqa: BLE001
            print("   save failed", e)
        (d / "plan.json").write_text(json.dumps(planned, indent=1))
        pv = call(base, "/api/missions/plan/preview_pddl", {"pddl_goal": m["goal"], "robots": robots, "src": SRC,
                                                             "engine": m["engine"], "constraints": cons})
        (d / "preview.json").write_text(json.dumps(pv, indent=1))
        with open(d / "schedule.csv", "w", newline="") as f:
            w = csv.writer(f)
            sched = (pv.get("schedule") or {}).get("rows")
            if sched:
                w.writerow(["robot", "action", "target", "start_s", "finish_s", "coalition"])
                for r in sched:
                    w.writerow([r["robot"], r["action"], r["target"], r["start"], r["finish"], " ".join(r.get("coalition") or [])])
            else:  # Fast Downward: the mission preview's per-task timing
                w.writerow(["robot", "task", "target", "start_s", "end_s", "move_m"])
                ptasks = planned["preview"]["tasks"]
                for t in mission["tasks"]:
                    p = ptasks.get(t["id"], {})
                    w.writerow([t["agent"], t["type"], t["target"], p.get("t_start"), p.get("t_end"), round(p.get("length_m") or 0, 1)])
        goal_images(base, d, pv)
        res = {"id": m["id"], "mission_id": mid, "engine": m["engine"], "driven_m": pv["driven_m"],
               "makespan_s": (pv.get("schedule") or {}).get("makespan") or planned["preview"].get("makespan"),
               "issues": planned["preview"].get("issues"), "tasks": len(mission["tasks"])}
        if a.chat:
            res.update(chat(base, a.model, agents, m, d))
        (d / "result.json").write_text(json.dumps(res, indent=1))
        index[m["id"]] = res
        print(m["id"], mid, res["engine"], "makespan", res["makespan_s"], "driven", res["driven_m"], "issues",
              [i.get("msg") for i in res["issues"] or [] if i.get("level") == "error"])
    prev = json.loads((out / "index.json").read_text()) if (out / "index.json").exists() else {}
    prev.update(index)
    (out / "index.json").write_text(json.dumps(prev, indent=1))


if __name__ == "__main__":
    sys.exit(main())
