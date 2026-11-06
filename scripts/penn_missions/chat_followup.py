"""Second chat turn for missions whose first turn stopped for operator input (open-set approval, unknown names):
python chat_followup.py missions.yaml URL MISSIONS_DIR ID 'operator reply' [model]"""
import json, re, sys, pathlib, urllib.request
import yaml

cat = yaml.safe_load(open(sys.argv[1])); base = sys.argv[2].rstrip("/"); md = pathlib.Path(sys.argv[3])
mid, reply = sys.argv[4], sys.argv[5]; model = sys.argv[6] if len(sys.argv) > 6 else "gpt-6-luna"
m = next(x for x in cat["missions"] if x["id"] == mid)
d = md / mid
ev = json.loads((d / "chat.json").read_text())
history = next(e for e in reversed(ev) if e.get("type") == "transcript")["items"]
agents = [{"id": n, "kind": v["kind"], "start": v["start"], "binding": None} for n, v in cat["starts"].items()]
body = {"message": reply, "src": "neo4j:pennmissions", "model": model, "history": history,
        "mission_ctx": {"roster": agents, "engine": m["engine"]}}
req = urllib.request.Request(base + "/api/chat", json.dumps(body).encode(), {"content-type": "application/json"})
raw = urllib.request.urlopen(req, timeout=900).read().decode()
ev2 = [json.loads(l) for l in raw.splitlines() if l.strip()]
(d / "chat_turn2.json").write_text(json.dumps(ev2, indent=1))
goals = re.findall(r'"pddl_goal"\s*:\s*"([^"]+)"', raw) + re.findall(r'"pddl"\s*:\s*"([^"]+)"', raw)
res = json.loads((d / "result.json").read_text())
res["chat_followup"] = reply
res["chat_goal"] = goals[-1] if goals else res.get("chat_goal")
usage = json.load(urllib.request.urlopen(base + "/api/chat/usage"))
res["llm_spent_usd"] = usage.get("spent_usd")
(d / "result.json").write_text(json.dumps(res, indent=1))
for e in ev2:
    if e.get("type") == "text":
        print("TEXT", e["text"][:300])
print(mid, "goal:", res["chat_goal"], "| spent $%.2f" % usage["spent_usd"])
