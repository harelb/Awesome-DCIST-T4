# Isaac Sim (dcist_sim) — Phase 2 follow-ups

Tracked from the final pre-merge whole-branch review of `feature/isaac_sim`.
None of these block the Phase 1 single-robot merge; all three matter once
multi-robot (Phase 2) work starts.

## (a) Sim service dispatch guard — validate `request.robot_name`

`RosBridge` registers grasp/place/teleport services per robot at
`/{name}/sim/grasp_object` etc. (`dcist_sim/dcist_sim_isaac/dcist_sim_isaac/ros_bridge.py:212-241`),
but the handlers (`_on_grasp`, `_on_place`, `_on_teleport`,
`ros_bridge.py:216-241`) forward `request.robot_name` straight to
`GraspBackend` without checking it matches the robot that owns the endpoint
the request arrived on. In Phase 1 (single robot) this is a no-op risk since
there is only one possible caller. In Phase 2, a caller that accidentally
targets `/robot_a/sim/grasp_object` with `robot_name: "robot_b"` in the
request body would silently operate on the wrong robot's object registry
state instead of failing loudly. Add a guard in each handler: if
`request.robot_name` is set and does not equal the endpoint's own robot
name, return `success=False` with a descriptive message instead of
dispatching to the backend. This should land before any multi-robot
scenario is exercised.

## (b) Upstream spot_tools bug: `set_classes(embeddings)` crash for late-registered classes

`spot_tools/spot_tools/src/spot_skills/detection_utils.py:67-82`
(`set_up_detector`) appends a newly-seen semantic class via the *low-level*
`self.yolo_model.model.set_classes(updated_classes)` call
(`detection_utils.py:81`) when `prompt_class` was not already registered at
init (`detection_utils.py:56-60`, the `["", "bag", "cone", "pipe"]` list).
That low-level API requires an `embeddings` positional argument the call
site doesn't supply, so it crashes with
`TypeError: set_classes() missing 1 required positional argument:
'embeddings'`. This affects real robots too, any time a novel object class
not present in `custom_classes` is queried. The sim workaround
(`docs/sim_runbook.md` §5/§8) sidesteps it entirely via the
`detector_class_synonyms` config mapping every scenario label to an
already-registered prompt (e.g. `box → "cement bag"`), so the append path is
never exercised — this is a config-level dodge, not a fix. The real fix is a
one-line change in `detection_utils.py` to call the high-level
`self.yolo_model.set_classes(updated_classes)` (matching the init call at
line 59) instead of reaching into `.model.set_classes(...)`.

## (c) Camera rate/resolution as per-robot scenario knobs

`dcist_sim/dcist_sim_isaac/dcist_sim_isaac/camera.py:140-159` defines the
full ZED contract (`IMAGE_WIDTH`, `IMAGE_HEIGHT`, `CAMERA_HZ`, encodings, K)
as module-level constants shared by every `SimZedCamera` instance. The
design spec (§8) promises these as per-robot scenario knobs, and the
measured cost (`dcist_sim_isaac/README.md`'s "VRAM cost per camera" section:
+184 MiB per camera at 640x360) makes resolution/rate the primary VRAM lever
for scaling to multiple simulated robots on one GPU (10 GB used by the full
single-robot stack out of a 24 GB card per `docs/sim_runbook.md` §10).
Phase 2 should thread resolution/Hz through `RobotSpec`
(`dcist_sim_isaac/scenario.py`) and `SimZedCamera`'s constructor so a
multi-robot scenario can trade camera fidelity for headroom on a
per-robot basis instead of every robot paying the same fixed cost.
