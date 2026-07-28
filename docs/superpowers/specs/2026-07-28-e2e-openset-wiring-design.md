# End-to-End Open-Set Wiring: NL Query → Explore → Query-Time Re-Query → Deliver

Design spec, 2026-07-28 (second overnight run). User-approved scope: NL + open-vocab
together; query-time re-query (tier 2) as the mechanism; realistic gate assets;
gpt-4.1-mini with ≤$5 budget; overnight autonomous execution.

Builds directly on the shipped exploration stack (superproject
`feature/isaac_sim_exploration`, dcist_sim `feature/exploration`, omniplanner
`feature/exploration`). Paper context: this implements tier 2 of the cost ladder in
`~/Documents/papers/icra_2026_openset_planning` spec §5.4–5.6, unified with the
already-shipped tier 3.

## 1. Goal

One command runs the full open-set story in Isaac Sim on floor3:

```
explore_mission.py --scenario mit_floor3_openset.yaml --robot hilbert \
    --nl "find the suitcase and bring it to the lobby" --output-dir ...
```

where `suitcase` is a class **absent from the labelspace** (GT semantics structurally
blind to it) spawned as a **realistic asset**. The robot: NL → PDDL goal carrying the
novel symbol `o_suitcase` → typed MissingSymbolError → explore; every ground round
additionally re-queries accumulated posed keyframes via SAM3 (warm server, separate
venv/process); on detection, the object is injected into the saved DSG (labelspace
append + parent edge) → ingest → plan → pick → deliver to the region. Absent class →
honest exit 4 after both explore-out AND re-query-empty.

## 2. Locked decisions

1. **Tier-2 identity: query-time re-query** over posed keyframes (mapping-time +
   accumulated during exploration). NOT a live detector in the loop.
2. **Realistic gate assets** via the proven 12-line wrapper-USD pattern
   (`scenarios/assets/objects/cone.usd` precedent): CDN
   `SM_FireExtinguisher_02/SM_Chair/SM_TrashCan_01/SM_Briefcase` (HEAD-verified 200)
   + Poly Haven CC0 `vintage_suitcase`/`baseball_01` downloads (SOURCES.md recipe).
   Box proxies remain as clutter. No changes to scenario.py/stage.py/converter.
3. **NL**: gpt-4.1-mini via `nlu_interface` (harelb fork, `feature/camp_mission_nl`),
   prompt `prompt_pnp_pddl_planner.yaml`; in-process call from the harness via
   `nl_grounding_check.request_llm_goal` (the node path swallows typed errors —
   deferred). Hard call cap + `max_retries=0` + token accounting; ≤$5.
4. **Novel-symbol convention**: `o_<underscored_class>` — flows through shipped
   omniplanner grounding unchanged (verified: survives token filters, kind_hint
   "object" from the leading `o`); zero omniplanner changes. Canonical internal class
   form = underscored lowercase (`gt_semantics.class_to_labelspace_id` normalizer);
   spaced form only at the SAM3 prompt boundary.
5. **Detector priming is OUT of the loop**: semantic_inference vocabulary is
   config-time (node reads config once; ultralytics runtime append is known-broken
   upstream). Between-missions re-prime = documented follow-up only.

## 3. Architecture (components, each independently testable)

### 3.1 NL escape hatch (nlu_interface + harness)
- Prompt: add unknown-object clause ("if no Object of the requested type exists,
  emit `o_<class>`, lowercased+underscored") + 1–2 few-shots + inject the labelspace
  vocabulary into the scene-graph block. The hardened camp no-op-guard paragraph must
  not regress (offline `nl_grounding_check --runs N` regression on BOTH sentence
  types is a gate).
- Harness: `--nl` flag on explore_mission (mutually exclusive with
  --target-class/--region); pure `parse_nl_goal(response_dict)` extracts either a
  grounded goal (known class → normal mission) or a novel `o_<class>` + region;
  STRICT validator — reject `O_Fire_Extinguisher`/`o-fire-ext`/bare `o99`
  (indistinguishable from a real churned id) with exit 2, never explore on a symbol
  we cannot name. Region symbol → scenario region label via room-labelspace inverse
  lookup (`nl_grounding_check.region_symbol` pattern).
- Cost control in `nlu_interface.interface.OpenAIWrapper`: `max_retries=0`,
  `max_output_tokens`, call counter + usage accumulation from `response.usage`,
  raise past cap. Config plumbed via `OpenAIConfig` (both construction sites mirror).

### 3.2 SAM3 re-query bridge (new module in dcist_sim_isaac + warm server)
- Server: existing `agentic_navigation.sam3_frontend.query_server` run under
  `~/environments/dcist/sam3/bin/python` (numpy 1.26 venv; process-level isolation —
  measured 4.2 GB VRAM peak beside Isaac, warm ~0.25 s/prompt-frame, 13 s cold load
  → spawn at mission start). The sam3 process NEVER touches DSG files (its spark_dsg
  1.1.4 cannot read Isaac saves) — JSON in/out only.
- Harness-side client `requery.py` (pure logic + thin HTTP): each ground round,
  glob `$ADT4_OUTPUT_DIR/agents/agent_*_meta.json` (format verified byte-identical
  to keyframe_store's contract; do NOT use `load_keyframes_from_graph` — returns 0
  on current saves; sub-keyframes have no pose on disk — unusable), send prompt
  (spaced form) + keyframe dir; receive detections
  `[{label, score, x, y, z, n_pixels, frame_ts}]`.
- Acceptance policy (pure, tested): score ≥ calibrated threshold (from Gate 0,
  expected 0.05–0.15 band on renders), ≥2 supporting frames OR single frame with
  high pixel count, fused centroid inside traversable-adjacent space. Rejected
  detections logged as events, never injected.

### 3.3 Injection with labelspace append (region_injector extension)
- New `ensure_labelspace_entry(G, label) -> id`: appends `label` to the saved
  graph's `_l2p0` metadata at `max_id+1` if absent. Operates on SAVED graphs only
  (dsg_augmented path). GATE-VERIFIED early: an appended id must round-trip
  Neo4j ingest → heracles publish → omniplanner grounding (this is the one genuinely
  new mechanism with unknown behavior; a dedicated smoke task proves it before the
  flagship gate).
- `ensure_object_of_class` then works unchanged (parent edge to nearest MESH_PLACES
  node — heracles drops orphans; KhronosObjectAttributes; semantic_label from the
  appended entry). Score dropped (no field) but logged in events.
- The discovered position from re-query (3D centroid) replaces the live-DSG
  discovery position in the objectnav/verify chain when the discovery came from
  tier 2.

### 3.4 Mission flow changes (explore_mission)
- Ground round becomes: (a) live-DSG `objects_of_class` (unchanged — catches
  labelspace classes via GT), (b) NEW tier-2 re-query for the novel class, (c) on
  either hit → discovery event (source tagged `dsg`|`requery`) → existing
  objectnav/save/inject/ingest/plan pipeline. Re-query failures/timeouts degrade to
  (a)-only with an event — the mission never dies because the server did.
- Negative gate semantics: exit 4 requires explored-out AND final re-query over the
  full archive returning nothing (stronger honest negative than today).
- events.jsonl additions: `requery_attempt` (n_frames, latency), `requery_hit`,
  `requery_rejected`, `nl_goal` (sentence, parsed class/region, novel?),
  `labelspace_appended`.

### 3.5 Scenario (new, floor3)
`mit_floor3_openset.yaml`: floor3_b env; existing catalog props as clutter (from
mit_floor3_explore); + realistic assets: suitcase (novel class, NOT in labelspace,
≥25 m from spawn, placement-checked), fire extinguisher + trash can (labelspace
classes, realistic geometry — used for Gate 1 and calibration); regions unchanged
(lobby/central_hall/west_wing). GT semantics ON (maps everything except the novel
class — exactly the "map committed before the task" story).

## 4. Gates (all on floor3; GPU-exclusive, after the in-flight routing fix closes)

- **Gate 0 (calibration, blocking, first GPU task):** spawn the realistic assets,
  drive a short scripted pass, run SAM3 over the keyframes; measure score
  distributions realistic vs box-proxy for the same classes; pick threshold;
  verify ≥1 accurate 3D lift against the known spawn pose (reprojection accuracy
  is measured here, not assumed). If realistic assets still score <0.1 → STOP,
  report (decision point, not silent fallback).
- **Gate A (labelspace-append smoke, no GPU):** synthetic append on a saved floor3
  graph → ingest → heracles → in-process ground of `(object-in-region o_new r)` —
  proves the round-trip before anything depends on it.
- **Gate 1 (NL regression):** `--nl "put a recycling bin in the lobby"` (known
  class) → escape hatch must NOT fire; mission completes as today. Plus offline
  `nl_grounding_check` runs: known-class ×5 and novel-class ×5 sentences, camp map +
  floor3 map — hardened rule intact, novel emission format correct.
- **Gate 2 (flagship):** `--nl "find the suitcase and bring it to the lobby"` →
  novel symbol → explore → tier-2 discovery (source=requery) → inject → deliver,
  exit 0, strict verify. Video (top-down replay) + events as evidence.
- **Gate 3 (negative):** `--nl` with a class absent from scene AND vocab-recognizable
  (e.g. "microwave") → explore-out + empty final re-query → exit 4, zero
  injections.

## 5. Error handling
- SAM3 server crash/timeout → event + continue GT-only; if the mission's target is
  novel-class, explored-out then exits 4 with `requery_unavailable` noted (infra
  distinguishable in events, exit code unchanged — evidence stays honest).
- LLM: malformed/over-cap → exit 2 with summary (existing contract). Unknown region
  → exit 2 (existing).
- Injection: labelspace append or parent-edge failure → exit 2 typed event (never a
  silent skip).

## 6. Testing
- TDD pure parts: NL validator/parser, class normalizer (3-convention table),
  acceptance policy, keyframe globbing/meta parsing, ensure_labelspace_entry on
  fixture graphs, requery-event summarization. Module-scope stdlib-only preserved.
- Mandated pytest invocation unchanged (spark_env + two --ignore flags), baseline
  656.
- GPU gates as §4; each gate's evidence dir + events.jsonl + replay video.

## 7. Out of scope (ledgered follow-ups)
Runtime/between-mission detector priming; node-level MissingSymbol feedback topic;
physics tier; building1/floor2 replication of the flagship; sam3.1 checkpoint (the
builder hardcodes sam3 — noted upstream quirk); embedding pre-filter tuning.

## 8. Model tiering (user directive)
Sonnet: assets/scenario/prompt-yaml mechanical tasks. Opus: requery bridge,
injection extension, NL harness integration, task reviews. Fable ONLY: Gate 2
flagship debug (iterative GPU authority) + final whole-branch review.

## 9. Known risks (from explorer evidence)
1. SAM3 domain gap on renders (top risk) — Gate 0 exists to kill or reroute it.
2. Labelspace append round-trip unproven — Gate A front-loads it.
3. Prompt regression on the delicate camp rule — offline regression in Gate 1.
4. gpt-4.1-mini novel-token compliance — strict validator + few-shots; reject, never
   guess.
5. GPU contention (Isaac + SAM3 + FD) — measured headroom 21 GB vs 4.2 GB peak; warm
   server amortizes load; budget latency per ground round ≤ ~10 s.
6. In-flight routing fix (round 2) must close before GPU gates; non-GPU tasks start
   immediately.
