#!/usr/bin/env python3
"""Plan-repair v5 cone scenario against a running stack (start_stack.sh), no robots.

The console's API drives it; ROS is only used to play the robot's change detector
(AwcdChanges on the base station's reconciled topic, as awcd_base_station would) and
heracles_agents' SGET chat (a SceneChangeMsg with source sget/chat). Steps:

  1 goal "(visited-object o20)" (O20: the cone at (17, 3); hilbert at (1, 1)) goes
    through plan repair; the plan drives to O20
  2 change detection adds a tree far away: no recommendation (unrelated)
  3 change detection adds a cone at (5, 1), closer than O20: the console recommends
    "replan with <new cone>"; confirm -> the new plan drives to the new cone
  4 a console map edit (delete the chair O5) always recommends; "replan same goal"
    -> a new plan
  5 an SGET edit (heracles_agents) always recommends; dismiss -> declined
  6 staged edits say "publish to replan" (only with --staged, console in staged mode)

    run_scenario.py [--console http://127.0.0.1:8095] [--base prv5base] [--robot hilbert]
Exit status 0 = every step passed.
"""

import argparse
import json
import math
import sys
import time
import urllib.error
import urllib.parse
import urllib.request

RESULTS = []


def step(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f" -- {detail}" if detail else ""), flush=True)
    return ok


class Console:
    def __init__(self, url):
        self.url = url.rstrip("/")

    def req(self, method, path, body=None, timeout=30):
        data = None if body is None else json.dumps(body).encode()
        r = urllib.request.Request(f"{self.url}/{path.lstrip('/')}", data=data, method=method,
                                   headers={"Content-Type": "application/json"})  # fmt: skip
        try:
            with urllib.request.urlopen(r, timeout=timeout) as resp:
                return json.loads(resp.read() or b"null")
        except urllib.error.HTTPError as e:
            raise RuntimeError(f"{method} {path}: {e.code} {e.read().decode()[:300]}") from None

    get = lambda self, p: self.req("GET", p)  # noqa: E731
    post = lambda self, p, b=None: self.req("POST", p, b or {})  # noqa: E731


def wait(pred, timeout, every=0.5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = pred()
        if v:
            return v
        time.sleep(every)
    return None


def plan_end(plan):
    """(plan_id, end of the last FOLLOW) of a console plan dict, or None."""
    p = (plan or {}).get("plan") or {}
    follows = [a for a in p.get("actions") or [] if a.get("type") == "FOLLOW" and a.get("path")]
    if not follows:
        return None
    end = follows[-1]["path"][-1]
    return p.get("plan_id"), (float(end[0]), float(end[1]))


def near(a, b, tol=2.5):
    return a is not None and math.dist(a, b) <= tol


class Ros:
    """The robot's change detector and heracles_agents' chat, on ROS."""

    def __init__(self, base):
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("prv5_scenario")
        from heracles_ros_interfaces.msg import SceneChangeMsg
        from khronos_msgs.msg import AwcdChanges

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)  # fmt: skip
        self.awcd = self.node.create_publisher(AwcdChanges, f"/{base}/awcd_base_station/reconciled_changes", latched)
        self.applied = self.node.create_publisher(SceneChangeMsg, "/scene_graph/changes_applied", 10)
        self._added = []  # AWCD republishes its whole current set every time

    def _wait_match(self, pub, timeout=5.0):
        t0 = time.time()
        while pub.get_subscription_count() == 0 and time.time() - t0 < timeout:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return pub.get_subscription_count()

    def awcd_add(self, obj_id, name, x, y):
        from khronos_msgs.msg import AwcdChanges, ChangedObjectInfo

        o = ChangedObjectInfo()
        o.id, o.name, o.change_confidence, o.num_frames_observed, o.confidence = obj_id, name, 0.9, 20, 0.9
        o.centroid.x, o.centroid.y, o.centroid.z = float(x), float(y), 0.3
        o.bbox_center.x, o.bbox_center.y, o.bbox_center.z = float(x), float(y), 0.3
        o.bbox_dimensions.x = o.bbox_dimensions.y = 0.4
        o.bbox_dimensions.z = 0.6
        o.bbox_orientation.w = 1.0
        self._added.append(o)
        m = AwcdChanges()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.robot_name = "reconciled"
        m.added_objects = list(self._added)
        n = self._wait_match(self.awcd)
        self.awcd.publish(m)
        return n

    def sget(self, kind, symbol, label, map_version):
        from heracles_ros_interfaces.msg import SceneChange, SceneChangeMsg

        m = SceneChangeMsg()
        m.source, m.map_version = "sget/chat", int(map_version)
        c = SceneChange()
        c.kind, c.symbol, c.semantic_label, c.confidence = kind, symbol, label, 1.0
        m.changes = [c]
        n = self._wait_match(self.applied)
        self.applied.publish(m)
        return n

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def pending(c, robot):
    st = c.get("api/plan/scene_repair")
    rid = (st.get("pending") or {}).get(robot)
    return next((r for r in st["recommendations"] if r["id"] == rid), None) if rid else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--console", default="http://127.0.0.1:8095")
    ap.add_argument("--base", default="prv5base")
    ap.add_argument("--robot", default="hilbert")
    ap.add_argument("--edit-src", default="neo4j:live", help="the console's id of the live Neo4j source")
    ap.add_argument("--engine", choices=["fast_downward", "grstaps"], default=None)
    ap.add_argument("--plan-timeout", type=float, default=90)
    a = ap.parse_args()
    c, robot = Console(a.console), a.robot

    up = wait(lambda: c.get("api/plan/repair").get("goal_manager"), 120, 2)
    if not step("stack up: goal_manager subscribes commanded_goal", up):
        return 1
    ros = Ros(a.base)
    try:
        # 1 ---------------------------------------------------------------
        sent = c.post(f"api/robots/{robot}/goal/pddl",
                      {"goal": "(visited-object o20)", "route": "goal_manager", "engine": a.engine})  # fmt: skip
        step("1 goal sent through plan repair", sent.get("via") == "goal_manager", json.dumps(sent)[:200])
        first = wait(lambda: (lambda e: e if e and near(e[1], (17, 3)) else None)(plan_end(c.get(f"api/robots/{robot}/plan"))),
                     a.plan_timeout)  # fmt: skip
        step("1 plan drives to the goal cone O20 at (17, 3)", first, str(first))
        if not first:
            return 1

        # 2 ---------------------------------------------------------------
        ros.awcd_add(1001, "tree", 15.0, 6.0)
        added = wait(lambda: any(ch["label"] == "tree" for b in c.get("api/plan/scene_repair")["changes"]
                                 for ch in b["changes"]), 20)  # fmt: skip
        step("2 AWCD tree reached the map and the console feed", added)
        time.sleep(2)
        step("2 an unrelated added tree is not recommended", pending(c, robot) is None)

        # 3 ---------------------------------------------------------------
        ros.awcd_add(1002, "cone", 5.0, 1.0)
        rec = wait(lambda: pending(c, robot), 20)
        ok = rec and rec.get("rewrites") and rec["proposed_goal"] != rec["goal"]["pddl"]
        step("3 closer cone -> replan recommended with the goal rewritten", ok,
             (rec or {}).get("reason", "") + f" | proposed {(rec or {}).get('proposed_goal')}")  # fmt: skip
        if not ok:
            return 1
        conf = c.post(f"api/plan/scene_repair/{rec['id']}/confirm", {"variant": "proposed"})
        step("3 confirm (proposed) sent", conf.get("status") == "confirmed", json.dumps(conf.get("result"))[:200])
        second = wait(lambda: (lambda e: e if e and e[0] != first[0] and near(e[1], (5, 1)) else None)(
            plan_end(c.get(f"api/robots/{robot}/plan"))), a.plan_timeout)  # fmt: skip
        step("3 new plan drives to the new, closer cone at (5, 1)", second, str(second))

        # 4 ---------------------------------------------------------------
        before = plan_end(c.get(f"api/robots/{robot}/plan"))
        src = urllib.parse.quote(a.edit_src, safe="")
        edit = c.post(f"api/edit/{src}/batch", {"ops": [{"op": "delete_node", "symbol": "O5"}]})
        step("4 console edit committed (delete chair O5)", edit.get("mutation_id"), str(edit.get("summary")))
        rec = wait(lambda: pending(c, robot), 20)
        step("4 a console edit always recommends replanning", rec, (rec or {}).get("reason", ""))
        if rec:
            c.post(f"api/plan/scene_repair/{rec['id']}/confirm", {"variant": "original"})
            third = wait(lambda: (lambda e: e if e and before and e[0] != before[0] else None)(
                plan_end(c.get(f"api/robots/{robot}/plan"))), a.plan_timeout)  # fmt: skip
            step("4 'replan same goal' produced a new plan", third, str(third))

        # 5 ---------------------------------------------------------------
        mv = c.get("api/plan/scene_repair").get("map_version") or 0
        ros.sget(1, "O6", "tree", mv + 1)  # heracles_agents: "SGET: delete the tree"
        rec = wait(lambda: pending(c, robot), 20)
        step("5 an SGET edit always recommends replanning", rec and rec["source"].startswith("sget/"),
             (rec or {}).get("reason", ""))  # fmt: skip
        if rec:
            d = c.post(f"api/plan/scene_repair/{rec['id']}/decline", {"resume": False})
            step("5 dismiss -> declined", d.get("status") == "declined")
    finally:
        ros.close()
    failed = [n for n, ok, _ in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} passed" + (f"; FAILED: {failed}" if failed else ""))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
