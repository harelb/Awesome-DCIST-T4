"""Experiment runs (API.md §18): per-trial run folders with an event log, pose
samples, robot + base-station bags, metrics, a bag sanity check and offload.

Layout under ``<output_root>/experiments/``::

    active.json                       {"experiment": <id> | null}
    <exp>/experiment.json
    <exp>/runs/<run_id>/run.json      manifest (status, robots, machines, outcome, ...)
    <exp>/runs/<run_id>/events.jsonl  every console + robot event (wall + ROS time)
    <exp>/runs/<run_id>/poses.jsonl   fleetlink poses, 2 Hz per robot
    <exp>/runs/<run_id>/metrics.json
    <exp>/runs/<run_id>/sanity.json

Robot bags are written on each robot under ``~/adt4_output/experiments/<exp>/
<run_id>/bags/<machine>`` and pulled by the offload job.
"""
