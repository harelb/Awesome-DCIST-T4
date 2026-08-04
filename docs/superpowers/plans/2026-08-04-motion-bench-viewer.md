# Motion-Bench Run Viewer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A generator script that writes a static forensics site (`viewer/`) into a benchmark run's out-dir: suite outcome matrix, per-trial trajectory/timeline/video pages, and detection-evidence keyframe views.

**Architecture:** One script (`scripts/benchmark_viewer.py`, repo convention: the benchmark trio are single scripts with pure importable functions) digests the evidence tree into per-trial JSON-able dicts, renders floor-plan PNGs once per env, and emits self-contained HTML pages that reference videos/keyframes RELATIVELY (nothing copied). The analyzer's `metrics.json` is the single source of outcomes — the viewer never re-scores.

**Tech Stack:** Python stdlib + PyYAML + numpy + PIL (all already in spark_env). No web framework, no CDN — inline CSS/JS in the page templates.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-08-04-motion-bench-viewer-design.md`.
- The viewer READS: `<trials-dir>/<id>/{trial.json,scenario_mission.yaml}`, `<out-dir>/{metrics.json,ledger.jsonl}`, `<out-dir>/trials/<id>/{events.jsonl,summary.json,mission_video/capture.mp4}`, `<out-dir>/mapping/<key12>/raw_robot/agents/agent_<ts>_rgb.jpg`, and `<env_usd>.floor.npz`. It WRITES only under `<out-dir>/viewer/`.
- `metrics.json` shape (verified live 2026-08-04): top keys `trials` (list) + `aggregates`; trial rows carry `id, outcome, condition, env, seed, nl, family, evidence_dir, exit_code, stop, reasons, target_class, expected_verdict, not_attempted, censored, undetectable_target, false_discovery, search_motion_m, total_motion_m, wall_s, coverage, discovery_source, discovery_score, archive_requery{...}, distances{...}`; aggregates carry `by_condition_env, motion_avoided_paired, failures, censored_trials, not_attempted_trials, undetectable_target_groups, mapping_coverage, absent_false_discoveries, llm_spend, weak_confusable_trials`. Outcome strings: `success | honest-negative | wrong-answer | infra-failure | censored | not-attempted | undetectable`.
- `scenario_mission.yaml`: `environment.usd` (contains `${ADT4_SIM_ASSETS}` — expand with `os.path.expandvars`), `objects: [{id,label,pose:{x,y,z,yaw},…}]`. Floor raster: `<expanded usd>.floor.npz`, npz keys `wall (bool HxW), wall_origin (2,), wall_cell (1,)`.
- Missing evidence NEVER breaks a page: absent video/keyframes/npz degrade to a labeled placeholder div (class `missing`), with the ledger/metrics reason shown.
- Outcome colors + one-letter glyphs (never color alone): success `#2e9e4f` "S", honest-negative `#5b6b7a` "H", wrong-answer `#d43b3b` "W", infra-failure `#e0871f` "I", censored `#8a5cc9` "C", not-attempted `#c9ced6` "·", undetectable `#1f8a99` "U".
- Tests live in `dcist_sim_isaac/test/test_benchmark_viewer.py`; run with `source $ADT4_ENV/spark_env/bin/activate && cd dcist_sim/dcist_sim_isaac && python -m pytest test/test_benchmark_viewer.py -q`. NEVER source ROS for pytest.
- Every event/summary field access uses `.get()` — real evidence trees predate schema additions.
- Commits in the `dcist_sim` submodule (`cd dcist_sim`), message prefix `feat(viewer):`, trailer `Claude-Session: https://claude.ai/code/session_01GB9DPnjriniEjAjsrJBXGs`.

## File Structure

- Create `dcist_sim/dcist_sim_isaac/scripts/benchmark_viewer.py` — everything: digest functions (pure), raster rendering, HTML rendering, CLI `main(argv)`.
- Create `dcist_sim/dcist_sim_isaac/test/test_benchmark_viewer.py` — all tests; synthetic evidence trees in `tmp_path` (pattern: `test_benchmark_analyze.py`).
- Modify `dcist_sim/docs/sim_runbook.md` — add §15.10 "Benchmark run viewer" (one short block, final task).

---

### Task 1: Event digests — trajectory, timeline, hits

**Files:**
- Create: `dcist_sim/dcist_sim_isaac/scripts/benchmark_viewer.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_benchmark_viewer.py`

**Interfaces:**
- Produces:
  - `read_events(path) -> list[dict]` (jsonl; missing file → `[]`; bad lines skipped)
  - `trajectory_from_events(events) -> list[dict]` — one point per `waypoint_sent`, `{"t": float, "x": float, "y": float, "purpose": str}` taken from the event's `frm` (robot pose) + `purpose`.
  - `timeline_from_events(events) -> list[dict]` — `{"t_off": float, "kind": str, "text": str}` ordered by time, t_off relative to the first event. Included kinds: `phase` (name), `waypoint_timeout` (wp + moved_m), `archive_requery_started/attempt/hit` (frames/score), `requery` events whose name starts with `requery_`, `objectnav_verified` (dist), `abort` (reason + detail keys). Excluded: `waypoint_sent`, `waypoint_reached`.
  - `hits_from_events(events) -> list[dict]` — for every event named `archive_requery_hit` or ending `_requery_hit` or named `requery_hit`: `{"t": .., "source": event-name, "score": .., "x": .., "y": .., "n_frames": .., "n_pixels": .., "frame_ts": [...]}` (missing keys → None/[]).

- [ ] **Step 1: Write the failing tests**

```python
# test/test_benchmark_viewer.py
import json
import os

from scripts.benchmark_viewer import (
    hits_from_events,
    read_events,
    timeline_from_events,
    trajectory_from_events,
)


def _write_events(tmp_path, events):
    p = tmp_path / "events.jsonl"
    p.write_text("\n".join(json.dumps(e) for e in events) + "\n")
    return str(p)


EVENTS = [
    {"t": 100.0, "event": "phase", "name": "archive_requery"},
    {"t": 101.0, "event": "archive_requery_hit", "prompt": "mop",
     "score": 0.73, "x": -18.9, "y": 44.9, "n_frames": 5, "n_pixels": 10078,
     "frame_ts": [111, 222]},
    {"t": 102.0, "event": "waypoint_sent", "wp": [1.0, 2.0, 0.1],
     "frm": [0.5, 1.5], "purpose": "objectnav"},
    {"t": 103.0, "event": "waypoint_reached", "wp": [1.0, 2.0]},
    {"t": 104.0, "event": "waypoint_sent", "wp": [2.0, 2.0, 0.0],
     "frm": [1.0, 2.0], "purpose": "objectnav"},
    {"t": 190.0, "event": "waypoint_timeout", "wp": [2.0, 2.0], "moved_m": 0.0},
    {"t": 200.0, "event": "abort", "reason": "objectnav_failed",
     "symbol": "archive/mop", "dist_to_object": 0.75},
]


def test_read_events_skips_garbage(tmp_path):
    p = tmp_path / "events.jsonl"
    p.write_text('{"t": 1, "event": "phase", "name": "x"}\nnot json\n')
    assert len(read_events(str(p))) == 1
    assert read_events(str(tmp_path / "absent.jsonl")) == []


def test_trajectory_uses_frm_and_purpose(tmp_path):
    pts = trajectory_from_events(EVENTS)
    assert pts == [
        {"t": 102.0, "x": 0.5, "y": 1.5, "purpose": "objectnav"},
        {"t": 104.0, "x": 1.0, "y": 2.0, "purpose": "objectnav"},
    ]


def test_timeline_keeps_salient_drops_chatter():
    tl = timeline_from_events(EVENTS)
    kinds = [e["kind"] for e in tl]
    assert "waypoint_sent" not in kinds and "waypoint_reached" not in kinds
    assert kinds == ["phase", "archive_requery_hit", "waypoint_timeout", "abort"]
    assert tl[0]["t_off"] == 0.0
    assert tl[-1]["t_off"] == 100.0
    assert "objectnav_failed" in tl[-1]["text"]


def test_hits_from_events():
    hits = hits_from_events(EVENTS)
    assert len(hits) == 1
    h = hits[0]
    assert (h["score"], h["x"], h["y"]) == (0.73, -18.9, 44.9)
    assert h["frame_ts"] == [111, 222]
    assert h["source"] == "archive_requery_hit"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `source $ADT4_ENV/spark_env/bin/activate && cd dcist_sim/dcist_sim_isaac && python -m pytest test/test_benchmark_viewer.py -q`
Expected: ImportError (module does not exist).

- [ ] **Step 3: Implement** `benchmark_viewer.py` with a module docstring (purpose + CLI example, style of `benchmark_analyze.py`), the four functions above, stdlib-only imports at this stage. `timeline_from_events` builds `text` by joining the event's non-housekeeping fields (`k=v` for keys not in `{"t","event"}`), truncated to 160 chars.

- [ ] **Step 4: Run tests — expect 4 passed.**

- [ ] **Step 5: Commit** `feat(viewer): event digests (trajectory, timeline, hits)`

---

### Task 2: Scenario + keyframe resolution

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/benchmark_viewer.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_benchmark_viewer.py`

**Interfaces:**
- Consumes: nothing from Task 1.
- Produces:
  - `gt_objects_from_scenario(path) -> list[dict]` — `{"id","label","x","y"}` per `objects[]` entry (pose.x/pose.y); missing file/keys → `[]`.
  - `env_npz_from_scenario(path) -> str | None` — `os.path.expandvars(environment.usd) + ".floor.npz"` if that file exists, else None.
  - `resolve_keyframes(frame_ts, agents_dir, limit=6) -> list[dict]` — for each ts (up to limit), `{"ts": ts, "rgb": "<agents_dir>/agent_<ts>_rgb.jpg"}` for files that exist (absolute paths; page rendering relativizes later).

- [ ] **Step 1: Write the failing tests**

```python
import yaml

from scripts.benchmark_viewer import (
    env_npz_from_scenario,
    gt_objects_from_scenario,
    resolve_keyframes,
)


def _write_scenario(tmp_path, usd):
    sc = {"environment": {"usd": usd},
          "objects": [{"id": "mop_0", "label": "mop",
                       "pose": {"x": -29.3, "y": 43.4, "z": 0, "yaw": 1.0}}]}
    p = tmp_path / "scenario_mission.yaml"
    p.write_text(yaml.safe_dump(sc))
    return str(p)


def test_gt_objects(tmp_path):
    p = _write_scenario(tmp_path, "/nonexistent/env.usd")
    assert gt_objects_from_scenario(p) == [
        {"id": "mop_0", "label": "mop", "x": -29.3, "y": 43.4}]
    assert gt_objects_from_scenario(str(tmp_path / "no.yaml")) == []


def test_env_npz_expansion(tmp_path, monkeypatch):
    monkeypatch.setenv("FAKE_ASSETS", str(tmp_path))
    usd = tmp_path / "env.usd"
    npz = tmp_path / "env.usd.floor.npz"
    npz.write_bytes(b"x")
    p = _write_scenario(tmp_path, "${FAKE_ASSETS}/env.usd")
    assert env_npz_from_scenario(p) == str(npz)
    p2 = _write_scenario(tmp_path, "${FAKE_ASSETS}/missing.usd")
    assert env_npz_from_scenario(p2) is None


def test_resolve_keyframes(tmp_path):
    (tmp_path / "agent_111_rgb.jpg").write_bytes(b"j")
    (tmp_path / "agent_333_rgb.jpg").write_bytes(b"j")
    out = resolve_keyframes([111, 222, 333], str(tmp_path))
    assert [k["ts"] for k in out] == [111, 333]
    assert out[0]["rgb"].endswith("agent_111_rgb.jpg")
    assert resolve_keyframes([1], str(tmp_path / "nope")) == []
```

- [ ] **Step 2: Run — expect ImportError on the new names.**
- [ ] **Step 3: Implement the three functions** (PyYAML import moves to module top; guard `yaml.safe_load` failures → `[]`/None).
- [ ] **Step 4: Run — all tests pass.**
- [ ] **Step 5: Commit** `feat(viewer): scenario GT + keyframe resolution`

---

### Task 3: Floor raster rendering

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/benchmark_viewer.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_benchmark_viewer.py`

**Interfaces:**
- Produces: `render_floor_png(npz_path, out_png) -> dict | None` — writes a grayscale PNG of the `wall` layer (walls dark `#555555` on white) and returns the world↔pixel transform `{"x0": float, "y0": float, "cell": float, "w": int, "h": int}` where pixel `(col,row)` maps to world `(x0 + col*cell, y0 + row*cell)` and the PNG is written with row 0 at the TOP (i.e. the array is vertically flipped for image convention; the transform describes the UNflipped grid and the SVG layer does `y_pix = h - 1 - (y_world - y0)/cell`). Returns None (no file written) on missing/corrupt npz.

- [ ] **Step 1: Write the failing test**

```python
import numpy as np

from scripts.benchmark_viewer import render_floor_png


def test_render_floor_png(tmp_path):
    wall = np.zeros((4, 6), dtype=bool)
    wall[1, 2] = True
    npz = tmp_path / "env.usd.floor.npz"
    np.savez(npz, wall=wall, wall_origin=np.array([-2.0, 3.0]),
             wall_cell=np.array([0.5]),
             floor=np.zeros((2, 3), dtype=bool),
             floor_origin=np.array([0.0, 0.0]), floor_cell=np.array([1.0]))
    out = tmp_path / "floor.png"
    tf = render_floor_png(str(npz), str(out))
    assert tf == {"x0": -2.0, "y0": 3.0, "cell": 0.5, "w": 6, "h": 4}
    from PIL import Image
    img = Image.open(out)
    assert img.size == (6, 4)
    # wall cell (row 1, col 2) is dark; image row = h-1-row = 2
    assert img.getpixel((2, 2))[0] < 100
    assert img.getpixel((0, 0))[0] > 200


def test_render_floor_png_missing(tmp_path):
    assert render_floor_png(str(tmp_path / "no.npz"), str(tmp_path / "o.png")) is None
```

- [ ] **Step 2: Run — ImportError.**
- [ ] **Step 3: Implement** with numpy + PIL (`Image.fromarray` on a uint8 RGB array, `np.flipud` for image convention).
- [ ] **Step 4: Run — pass.**
- [ ] **Step 5: Commit** `feat(viewer): per-env floor raster + transform`

---

### Task 4: Trial forensics page

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/benchmark_viewer.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_benchmark_viewer.py`

**Interfaces:**
- Consumes: Task 1 digests, Task 2 resolvers, Task 3 transform dict.
- Produces: `render_trial_page(row, digest, ctx) -> str` (full HTML document).
  - `row`: one metrics.json trial dict (`id, outcome, condition, env, seed, nl, exit_code, stop, reasons, target_class, expected_verdict, …`).
  - `digest`: `{"trajectory": [...], "timeline": [...], "hits": [{... , "keyframes": [{"ts","rgb"}], "gt_dist_m": float|None}], "gt_objects": [...], "video": str|None, "stop_video": str|None, "summary": dict}` — paths in `digest` are ABSOLUTE; rendering relativizes against `ctx["page_dir"]` via `os.path.relpath`.
  - `ctx`: `{"page_dir": str, "floor_png": str|None, "floor_tf": dict|None}`.
  - Page anatomy (all sections always present, `class="missing"` placeholders when data absent): header (id, outcome badge with color+glyph from `OUTCOME_STYLE`, exit/stop/reasons), map `<svg>` with `<image href=relative floor png>` + trajectory `<polyline>` (one per purpose: explore `#3b6fd4`, objectnav `#8a5cc9`), GT object markers (target = green circle + label), hit markers (red X + score), timeline `<ol>`, video `<video controls src=…>`, hits section with keyframe `<img>` strips + `gt_dist_m` colored green ≤ 1.0 else red, key-numbers `<table>` from `digest["summary"]` (coverage, distances, search_motion_m, archive_requery).
- Produces: `OUTCOME_STYLE: dict[str, tuple[str, str]]` mapping outcome → (hex, glyph) exactly as in Global Constraints.

- [ ] **Step 1: Write the failing tests**

```python
from scripts.benchmark_viewer import OUTCOME_STYLE, render_trial_page


def _min_row():
    return {"id": "trial_x", "outcome": "wrong-answer", "condition": "VOCAB-2a",
            "env": "floor3", "seed": 23, "nl": False, "exit_code": 4,
            "stop": None, "reasons": ["verdict_mismatch: expected success, exit=4"],
            "target_class": "mop", "expected_verdict": "success"}


def _min_digest(tmp_path):
    video = tmp_path / "trials" / "trial_x" / "mission_video" / "capture.mp4"
    video.parent.mkdir(parents=True)
    video.write_bytes(b"v")
    kf = tmp_path / "mapping" / "k" / "agent_111_rgb.jpg"
    kf.parent.mkdir(parents=True)
    kf.write_bytes(b"j")
    return {
        "trajectory": [{"t": 1.0, "x": 0.0, "y": 0.0, "purpose": "objectnav"},
                       {"t": 2.0, "x": 1.0, "y": 1.0, "purpose": "objectnav"}],
        "timeline": [{"t_off": 0.0, "kind": "abort", "text": "reason=objectnav_failed"}],
        "hits": [{"t": 1.0, "source": "archive_requery_hit", "score": 0.73,
                  "x": -18.9, "y": 44.9, "n_frames": 5, "n_pixels": 10078,
                  "frame_ts": [111], "keyframes": [{"ts": 111, "rgb": str(kf)}],
                  "gt_dist_m": 10.97}],
        "gt_objects": [{"id": "mop_0", "label": "mop", "x": -29.3, "y": 43.4}],
        "video": str(video), "stop_video": None,
        "summary": {"final_coverage": None, "search_motion_m": 0.0},
    }


def test_trial_page_content(tmp_path):
    page_dir = tmp_path / "viewer" / "trial"
    page_dir.mkdir(parents=True)
    html = render_trial_page(_min_row(), _min_digest(tmp_path),
                             {"page_dir": str(page_dir),
                              "floor_png": None, "floor_tf": None})
    assert "trial_x" in html and "wrong-answer" in html
    assert OUTCOME_STYLE["wrong-answer"][0] in html          # badge color
    assert "../../trials/trial_x/mission_video/capture.mp4" in html
    assert "../../mapping/k/agent_111_rgb.jpg" in html
    assert "polyline" in html and "10.97" in html
    assert "objectnav_failed" in html
    assert 'class="missing"' in html                          # no floor plan


def test_trial_page_survives_empty_digest(tmp_path):
    page_dir = tmp_path / "viewer" / "trial"
    page_dir.mkdir(parents=True)
    empty = {"trajectory": [], "timeline": [], "hits": [], "gt_objects": [],
             "video": None, "stop_video": None, "summary": {}}
    html = render_trial_page(_min_row(), empty,
                             {"page_dir": str(page_dir),
                              "floor_png": None, "floor_tf": None})
    assert "trial_x" in html and html.count('class="missing"') >= 2
```

- [ ] **Step 2: Run — ImportError.**
- [ ] **Step 3: Implement.** One `_html_escape` helper (`html.escape`), f-string template, SVG viewBox sized from trajectory+GT bounds (fallback 100×100 when floor_tf is None, else the raster extent), `os.path.relpath(path, page_dir)` for every media reference. Each trajectory vertex gets a small `<circle>` carrying `<title>t=+{t_off:.1f}s {purpose}</title>` — native SVG hover tooltip, no JS. Unknown outcome strings fall back to a gray style (`OUTCOME_STYLE.get(outcome, ("#c9ced6", "?"))`). No external assets.
- [ ] **Step 4: Run — pass.**
- [ ] **Step 5: Commit** `feat(viewer): per-trial forensics page`

---

### Task 5: Suite matrix index page

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/benchmark_viewer.py`
- Test: `dcist_sim/dcist_sim_isaac/test/test_benchmark_viewer.py`

**Interfaces:**
- Consumes: `OUTCOME_STYLE` (Task 4).
- Produces: `render_index(metrics) -> str` — `metrics` is the loaded metrics.json dict.
  - Matrix: one row per (condition, nl) family, one column per (env, seed) pair present in `trials`; each cell shows the glyph, is filled with the outcome color, links `trial/<id>.html` when `not_attempted` is False, `title` attr = id + outcome + reasons.
  - Sections beneath, straight from `aggregates`: false-discovery banner (`absent_false_discoveries`: green "0" or red count), `motion_avoided_paired` (overall + per-env + excluded pairs), `failures` table, `mapping_coverage` table (with its `warning` column), `censored_trials`, count of `not_attempted_trials`, `undetectable_target_groups`, `llm_spend`.

- [ ] **Step 1: Write the failing test**

```python
from scripts.benchmark_viewer import render_index


def test_index_matrix_and_aggregates():
    metrics = {
        "trials": [
            {"id": "a", "outcome": "success", "condition": "VOCAB-2a",
             "env": "floor3", "seed": 11, "nl": False, "not_attempted": False,
             "reasons": []},
            {"id": "b", "outcome": "not-attempted", "condition": "VOCAB-2a",
             "env": "floor2", "seed": 11, "nl": False, "not_attempted": True,
             "reasons": []},
        ],
        "aggregates": {
            "absent_false_discoveries": {"n": 0, "trials": []},
            "motion_avoided_paired": {"overall": {"n_pairs": 1,
                                                  "mean_m": 1375.1,
                                                  "median_m": 1375.1}},
            "failures": [], "mapping_coverage": [], "censored_trials": [],
            "not_attempted_trials": [{"id": "b"}],
            "undetectable_target_groups": [], "llm_spend": "$0.25",
            "weak_confusable_trials": [],
        },
    }
    html = render_index(metrics)
    assert 'href="trial/a.html"' in html
    assert 'href="trial/b.html"' not in html      # not attempted: no page
    assert "1375.1" in html and "VOCAB-2a" in html and "floor3" in html
```

Note for the implementer: `aggregates` sub-shapes vary — render dicts/lists generically (a small `_kv_table(obj)` that renders a dict as rows and a list-of-dicts as a table with union-of-keys columns) rather than hand-coding each aggregate's schema. The test above must pass against exactly this synthetic input; the real metrics.json is exercised in Task 6.

- [ ] **Step 2: Run — ImportError.**
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run — pass.**
- [ ] **Step 5: Commit** `feat(viewer): suite matrix index`

---

### Task 6: CLI, site assembly, runbook

**Files:**
- Modify: `dcist_sim/dcist_sim_isaac/scripts/benchmark_viewer.py`
- Modify: `dcist_sim/docs/sim_runbook.md` (append §15.10)
- Test: `dcist_sim/dcist_sim_isaac/test/test_benchmark_viewer.py`

**Interfaces:**
- Consumes: everything above.
- Produces:
  - `build_trial_digest(row, trials_dir, out_dir) -> dict` — assembles the Task-4 digest for one trial: events from `<out-dir>/trials/<id>/events.jsonl`, summary from `summary.json`, scenario GT + target distance for each hit (`gt_dist_m` = distance to nearest GT object whose label == row's `target_class`, None when no such object), keyframes resolved against the mapping archive: the agents dir is `<out-dir>/mapping/<cache_key12>/raw_robot/agents` where `cache_key12` comes from `trial.json`'s `sim_binding.mapping_pass.cache_key[:12]` (no mapping_pass → no keyframes), and video paths (`mission_video/capture.mp4`, `stop_video/capture.mp4`) that exist on disk or None.
  - `generate_site(trials_dir, out_dir) -> dict` — reads `<out-dir>/metrics.json` (missing → `SystemExit` with the exact analyzer command to run), renders floor PNG once per env (from the FIRST attempted trial of that env; cached in `viewer/assets/floor_<env>.png` + transform in `floor_<env>.json`), writes `viewer/index.html` + `viewer/trial/<id>.html` for every row with `not_attempted == False`, returns `{"pages": int, "envs": [str], "skipped": int}`.
  - `main(argv=None) -> int` — argparse `--trials-dir` (required), `--out-dir` (required); prints `benchmark_viewer.py: N page(s) -> <out-dir>/viewer (serve with: cd <out-dir> && python3 -m http.server)`.

- [ ] **Step 1: Write the failing integration test**

```python
from scripts.benchmark_viewer import generate_site, main


def _mini_tree(tmp_path):
    trials = tmp_path / "trials_v1"
    out = tmp_path / "v1"
    tdir = trials / "trial_x"
    tdir.mkdir(parents=True)
    tdir.joinpath("trial.json").write_text(json.dumps(
        {"id": "trial_x",
         "sim_binding": {"mapping_pass": {"cache_key": "a" * 64}}}))
    tdir.joinpath("scenario_mission.yaml").write_text(yaml.safe_dump(
        {"environment": {"usd": "/nonexistent/env.usd"},
         "objects": [{"id": "mop_0", "label": "mop",
                      "pose": {"x": 0.0, "y": 0.0, "z": 0, "yaw": 0}}]}))
    ev = out / "trials" / "trial_x"
    ev.mkdir(parents=True)
    ev.joinpath("events.jsonl").write_text(json.dumps(
        {"t": 1.0, "event": "phase", "name": "boot"}) + "\n")
    ev.joinpath("summary.json").write_text(json.dumps({"exit_code": 4}))
    out.joinpath("metrics.json").write_text(json.dumps({
        "trials": [{"id": "trial_x", "outcome": "honest-negative",
                    "condition": "ABSENT-invocab", "env": "floor3",
                    "seed": 11, "nl": False, "not_attempted": False,
                    "reasons": []}],
        "aggregates": {"absent_false_discoveries": {"n": 0},
                       "motion_avoided_paired": {}, "failures": [],
                       "mapping_coverage": [], "censored_trials": [],
                       "not_attempted_trials": [],
                       "undetectable_target_groups": [],
                       "llm_spend": "", "weak_confusable_trials": []},
    }))
    return str(trials), str(out)


def test_generate_site_end_to_end(tmp_path):
    trials, out = _mini_tree(tmp_path)
    stats = generate_site(trials, out)
    assert stats["pages"] == 1
    assert os.path.exists(os.path.join(out, "viewer", "index.html"))
    page = os.path.join(out, "viewer", "trial", "trial_x.html")
    assert os.path.exists(page)
    assert "honest-negative" in open(page).read()


def test_main_requires_metrics(tmp_path):
    trials, out = _mini_tree(tmp_path)
    os.remove(os.path.join(out, "metrics.json"))
    import pytest
    with pytest.raises(SystemExit):
        main(["--trials-dir", trials, "--out-dir", out])


def test_main_prints_serve_hint(tmp_path, capsys):
    trials, out = _mini_tree(tmp_path)
    assert main(["--trials-dir", trials, "--out-dir", out]) == 0
    assert "http.server" in capsys.readouterr().out
```

- [ ] **Step 2: Run — ImportError.**
- [ ] **Step 3: Implement** `build_trial_digest`, `generate_site`, `main`, plus `if __name__ == "__main__": raise SystemExit(main())`.
- [ ] **Step 4: Run the FULL viewer test file — all pass.**
- [ ] **Step 5: Smoke against the real run** (read-only inputs, writes only viewer/):

Run: `source $ADT4_ENV/spark_env/bin/activate && cd dcist_sim/dcist_sim_isaac && python scripts/benchmark_viewer.py --trials-dir ~/adt4_output/motion_bench/trials_v1 --out-dir ~/adt4_output/motion_bench/v1`
Expected: `40 page(s)` (stage-1+2 attempted trials), no traceback. Spot-open `viewer/trial/motion_tier_v1_floor3_vocab_2a_s23.html` and verify the FP hit renders with keyframes and gt_dist_m ≈ 10.97.

- [ ] **Step 6: Append runbook §15.10** to `dcist_sim/docs/sim_runbook.md`: the one-command generate + serve recipe and the note that metrics.json must exist (analyzer first).

- [ ] **Step 7: Commit** `feat(viewer): site assembly CLI + runbook §15.10`
