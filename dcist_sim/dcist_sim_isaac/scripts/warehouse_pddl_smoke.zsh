#!/usr/bin/env zsh
#
# PDDL-smoke acceptance gate for mapping-harness scenes (spec §3.6): brings up
# Isaac (warehouse_tour) + spot_isaac stack + omniplanner, warm-up drive (see
# docs/sim_runbook.md §11 for why each step exists), then runs e2e_smoke.py.
# Exit 0 iff Stage A/B/C all PASS. Verified 2026-07-18: A 3.32m goto, B pick
# cone_0, C carry 10.31m.
#
set -x
cd ~/dcist_ws/src/awesome_dcist_t4
source /opt/ros/jazzy/setup.zsh 2>/dev/null
source ~/dcist_ws/install/setup.zsh 2>/dev/null
export OMNI_KIT_ACCEPT_EULA=YES PRIVACY_CONSENT=Y ADT4_ROBOT_NAME=hilbert
export PYTHONPATH=~/dcist_ws/src/awesome_dcist_t4/dcist_sim/dcist_sim_isaac:$PYTHONPATH
export ADT4_WS=~/dcist_ws ADT4_ENV=~/environments/dcist
export ADT4_DLS_PKG=~/dcist_ws/src/awesome_dcist_t4/dcist_launch_system
OUT=~/adt4_output/warehouse_pddl_smoke; LOGS=~/adt4_output/warehouse_pddl_smoke_logs
rm -rf $OUT $LOGS; mkdir -p $OUT $LOGS

~/environments/dcist/isaac_sim/bin/python -m dcist_sim_isaac.sim_app \
  --scenario dcist_sim/scenarios/warehouse_tour.yaml --headless \
  --gt-out $LOGS/gt > $LOGS/isaac.log 2>&1 &
ISAAC_PID=$!
dcist_launch_system/bin/run-adt4 -n hilbert -c topaz -o $OUT -y -f \
  --tmuxp-args="-d -L t4pddl" spot_isaac-isaac_sim
export ADT4_OUTPUT_DIR=$LOGS
export config=isaac_sim
ros2 launch dcist_launch_system master.launch.yaml \
  conf_name:=isaac_sim sim_time:=false robot_name:=hilbert \
  launch_omniplanner:=true > $LOGS/omniplanner.log 2>&1 &
OMNI_PID=$!
for i in $(seq 1 80); do
  ros2 topic echo /sim/status --once --timeout 2 >/dev/null 2>&1 && break
  sleep 5
done
echo SIM_UP

# Warm-up: verify pub/sub MATCH (get_subscription_count) before trusting a
# publish -- rmw_zenoh sometimes never matches a publisher; recreate it then.
~/environments/dcist/spark_env/bin/python - <<'PYEOF'
import math, os, sys, time, threading, numpy as np, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from robot_executor_interface.action_descriptions import ActionSequence, Follow
from robot_executor_interface_ros.action_descriptions_ros import to_msg
from robot_executor_msgs.msg import ActionSequenceMsg
rclpy.init()
n = Node("warmup")
state = {"odom": None}
def ocb(m): state["odom"] = (m.pose.pose.position.x, m.pose.pose.position.y)
n.create_subscription(Odometry, "/hilbert/odom", ocb, 10)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
TOPIC = "/hilbert/omniplanner_node/compiled_plan_out"
pub = n.create_publisher(ActionSequenceMsg, TOPIC, 10)
t0 = time.time()
while state["odom"] is None and time.time() - t0 < 180:
    time.sleep(1)
if state["odom"] is None:
    print("NO_ODOM", flush=True); os._exit(4)
time.sleep(20)
seq = ActionSequence(plan_id="warm", robot_name="hilbert",
                     actions=[Follow(frame="hilbert/odom",
                                     path2d=np.array([[0.0, 0.0], [0.0, 6.0]]))])
t0 = time.time(); last_pub = 0
while time.time() - t0 < 300:
    o = state["odom"]
    if o and math.hypot(*o) > 4.0:
        print(f"MOVED at {time.time()-t0:.0f}s", flush=True)
        time.sleep(10)
        os._exit(0)
    subs = pub.get_subscription_count()
    if subs < 1:
        print(f"[warmup] 0 matched subs -- recreating publisher", flush=True)
        n.destroy_publisher(pub)
        time.sleep(2)
        pub = n.create_publisher(ActionSequenceMsg, TOPIC, 10)
        time.sleep(3)
        continue
    if time.time() - last_pub > 25:
        pub.publish(to_msg(seq)); last_pub = time.time()
        print(f"[warmup] sent (matched subs={subs}), odom={o}", flush=True)
    time.sleep(1)
print("WARMUP_FAIL", flush=True); os._exit(2)
PYEOF
WARM_EXIT=$?
echo "WARM_EXIT: $WARM_EXIT"
if [ $WARM_EXIT -eq 0 ]; then
  ~/environments/dcist/spark_env/bin/python - <<'PYEOF'
import os, sys, time, threading, rclpy, spark_dsg
from rclpy.node import Node
from hydra_ros import DsgSubscriber
rclpy.init()
n = Node("places_check")
state = {"dsg": None}
def cb(h, d): state["dsg"] = d
DsgSubscriber(n, "/hilbert/hydra/backend/dsg", cb)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
t0 = time.time()
while time.time() - t0 < 240:
    g = state["dsg"]
    if g is not None:
        ob = len(list(g.get_layer(spark_dsg.DsgLayers.OBJECTS).nodes))
        pl = len(list(g.get_layer(spark_dsg.DsgLayers.MESH_PLACES).nodes))
        print(f"[places] +{time.time()-t0:.0f}s objects={ob} places={pl}", flush=True)
        if ob >= 1 and pl >= 2:
            print("PREREQ_MET", flush=True); os._exit(0)
    time.sleep(15)
print("PLACES_FAIL", flush=True); os._exit(3)
PYEOF
  PLACES_EXIT=$?
  echo "PLACES_EXIT: $PLACES_EXIT"
  if [ $PLACES_EXIT -eq 0 ]; then
    ~/environments/dcist/spark_env/bin/python \
      dcist_sim/dcist_sim_isaac/scripts/e2e_smoke.py --robot hilbert
    E2E_EXIT=$?
  else
    E2E_EXIT=$PLACES_EXIT
  fi
else
  E2E_EXIT=$WARM_EXIT
fi
echo "E2E_EXIT: $E2E_EXIT"
echo "=== executor/omniplanner pane snapshots ==="
for p in $(tmux -L t4pddl list-panes -a -F '#{pane_id}'); do
  out=$(tmux -L t4pddl capture-pane -p -t $p 2>/dev/null | grep -iE "action sequence|follow|pick|place|grasp|error" | tail -4)
  [ -n "$out" ] && echo "-- $p --" && echo "$out"
done
tail -5 $LOGS/omniplanner.log 2>/dev/null
tmux -L t4pddl kill-server 2>/dev/null
kill -INT $OMNI_PID $ISAAC_PID 2>/dev/null
sleep 15; kill -9 $ISAAC_PID $OMNI_PID 2>/dev/null
exit $E2E_EXIT
