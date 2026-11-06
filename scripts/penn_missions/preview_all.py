"""Preview every mission's canonical goal on a running console (no LLM): python preview_all.py missions.yaml URL OUT"""
import json, sys, urllib.request, pathlib, yaml

cat = yaml.safe_load(open(sys.argv[1]))
base = sys.argv[2].rstrip("/")
out = pathlib.Path(sys.argv[3]); out.mkdir(parents=True, exist_ok=True)
robots = [{"name": n, "start": v["start"], "kind": v["kind"]} for n, v in cat["starts"].items()]
only = set(sys.argv[4].split(",")) if len(sys.argv) > 4 else None
for m in cat["missions"]:
    if only and m["id"] not in only:
        continue
    body = {"pddl_goal": m["goal"], "robots": robots, "src": "neo4j:pennmissions", "engine": m["engine"],
            "constraints": m.get("constraints") or []}
    req = urllib.request.Request(base + "/api/missions/plan/preview_pddl", json.dumps(body).encode(), {"content-type": "application/json"})
    try:
        r = json.load(urllib.request.urlopen(req, timeout=300))
    except urllib.error.HTTPError as e:
        print(m["id"], "FAIL", e.read().decode()[:500]); continue
    (out / f"{m['id']}_preview.json").write_text(json.dumps(r, indent=1))
    acts = [a["action"] for a in r["actions"] if not a["action"].startswith("goto")]
    mk = (r.get("schedule") or {}).get("makespan")
    print(m["id"], m["engine"], "driven", r["driven_m"], "makespan", mk, "left_out", r.get("left_out"))
    for a in r["actions"]:
        print("    ", a["robot"], a["action"])
