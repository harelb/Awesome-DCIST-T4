# Motion-tier benchmark run viewer — design

2026-08-04. Status: APPROVED (Harel, 2026-08-04 "go for it") and IMPLEMENTED — dcist_sim scripts/benchmark_viewer.py, commits 1fa4bb8..ec2388c.
Context: motion-tier benchmark (§6.5) produces 80-trial evidence trees under
`~/adt4_output/motion_bench/` (per-trial `events.jsonl`, `summary.json`,
`mission_video/capture.mp4`, mapping archives with keyframe RGB/depth, floor
rasters, analyzer `metrics.json`/`report.md`). Diagnosing the stage-1 failures
required hand-writing per-trial forensics (trajectory map, keyframe pulls);
the viewer makes that a one-command artifact. Primary user: Harel, analyzing
runs for the ICRA paper.

## Goals

- "What happened in this trial?" answered in under a minute without touching
  a terminal: trajectory on the floor plan, event timeline, video, verdict.
- "What happened to the suite?" at a glance: outcome matrix, paired deltas,
  the analyzer's exclusions (censored / not-attempted / undetectable).
- Detection forensics: for every archive/live requery hit, the contributing
  keyframes and the estimated-vs-ground-truth position error — the view that
  makes an s23-style false positive obvious instantly.

## Non-goals

- Not the paper-figure factory: publication figures stay matplotlib scripts
  in the repo. The viewer optimizes speed of understanding, not typography.
- No server/backend, no live tailing of an in-flight run (regenerate to
  refresh), no editing/annotation state.

## Form

A generator script — `dcist_sim_isaac/scripts/benchmark_viewer.py` — reads a
trials-dir + out-dir (same contract as `benchmark_analyze.py`) and writes a
static site into `<out-dir>/viewer/`:

    viewer/
      index.html          # suite matrix + analyzer summary
      trial/<id>.html     # one forensics page per attempted trial
      assets/…            # per-env floor-plan PNGs, digested JSON

Videos and keyframe images are NOT copied — pages reference them relatively
(`../../trials/<id>/mission_video/capture.mp4`, `../../mapping/<key>/…`), so
the site adds ~a few MB next to the evidence it describes. Open with
`python -m http.server` from the out-dir (file:// blocks mp4 range requests);
works over ssh port-forward.

Regeneration is idempotent and cheap (< 30 s): rerun after each
analyzer pass. One command in the runbook:

    python3 scripts/benchmark_viewer.py --trials-dir …/trials_v1 \
        --out-dir …/v1   # writes/refreshes …/v1/viewer/

## Views

### 1. Suite matrix (index.html)
- Grid: rows = condition family, columns = env × seed; each cell colored by
  analyzer outcome (success / honest-negative / wrong-answer / infra /
  censored / not-attempted / undetectable) and linking to the trial page.
  Outcomes come from `metrics.json` — the analyzer stays the single scorer;
  the viewer never re-derives verdicts.
- Motion-avoided table (pairs, deltas, excluded pairs with reasons), the
  false-discovery banner, LLM spend, mapping-tour table with coverage
  warnings — mirroring report.md's sections, linked.

### 2. Per-trial forensics (trial/<id>.html)
- **Trajectory map**: floor-plan raster (wall band from the env's
  `*.floor.npz`, rendered once per env into assets/) with an SVG overlay:
  mission path from `events.jsonl` (colored by purpose: explore vs objectnav),
  spawned GT objects from `scenario_mission.yaml` (target highlighted),
  requery/archive hit positions, and an abort marker at the last trajectory
  vertex (abort events carry no coordinates, so the marker is the last pose
  the robot was commanded FROM, not the abort's own location — the page's
  legend and the marker's tooltip both say so). Hover a path vertex → its
  timestamp/event.
- **Timeline**: phases and salient events (waypoint timeouts, requery
  attempts/hits/rejections, ground rounds, aborts) as a vertical list with
  t-offsets from mission start; verdict + exit code + stop reason on top,
  with the analyzer's category and reasons.
- **Video**: `capture.mp4` embedded; stop_video beside it when present.
- **Key numbers**: summary.json distances, coverage, scores, plus the
  trial.json expectations (expected verdict, target, calibration entry).

### 3. Detection evidence (section within the trial page)
- For each `archive_requery_hit` / live requery hit event: score, fused
  position, distance to the GT target spawn (green ≤ 1 m, red beyond),
  n_frames/n_pixels, and the contributing keyframe RGB thumbnails
  (`frame_ts` → `agent_<ts>_rgb.jpg` in the archive), each captioned with the
  camera's position/time. This is exactly the evidence chain that unmasked
  the s23 clutter FP; making it two clicks deep is the point.

## Implementation notes

- Python, stdlib + PyYAML + numpy (raster) — no web framework; one
  self-contained HTML template per page kind with inline CSS/JS (no CDN).
  Reuses `benchmark_run`'s evidence readers and `benchmark_analyze`'s loaded
  rows rather than re-parsing (same import discipline the analyzer follows).
- Floor rasters: `wall` layer of the env npz → PNG via PIL (already a
  dcist_sim dependency) with the world↔pixel transform stored in the digest
  JSON so the SVG overlay is exact.
- Missing evidence (no video, no archive, propless world) degrades to a
  labeled placeholder, never a broken page — the ledger's skip/censor reasons
  are shown instead.
- Tests, benchmark-trio conventions: digester unit tests (events → timeline
  JSON, path extraction, hit→keyframe resolution), matrix assembly from a
  synthetic metrics.json, and content assertions on generated pages (trial id,
  outcome class, relative video path present). No browser automation.

## Decisions taken by default (flag if wrong)

- Static site in out-dir over live Flask app and over a claude.ai Artifact
  (videos/keyframes exceed artifact limits; a served dir needs no upkeep).
- v1 ships views 1–3; cross-run comparison (same trial across arms/re-runs)
  is v2 — the per-trial JSON digest is designed so a compare page can be
  added without re-digesting.
- Analysis-only; no figure-export buttons.
