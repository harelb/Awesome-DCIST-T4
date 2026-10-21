# Next development round: notes from the 2026-10-20 field session

State when written: `integrate/features-20261020` @ 72b4630 (harelb), deployed to willow + hamilton.

## Console chat (Cypher-first; was deployed 72b4630, robots since reverted to start-of-day)
- DONE locally on `feature/chat-sam3-followups` @4f06950 (not merged/deployed): no re-search after an
  [operator] note; sibling SAM3 cards re-checked after an approval (merge radius 0.75 m).
- **SAM3 approvals**: after approving a candidate the assistant's automatic follow-up re-ran the
  same search ("blue trash can" -> 4 new cards); approving #3 of the new set merged into the
  just-created O18. Fix: after an approval note the assistant only acknowledges (no new search
  unless asked); re-check the remaining cards of the same search and show "would merge into
  O18" for those within the dedup radius (0.75 m, same label). Ask Harel what exactly blocked
  adding a second object (greyed out / merged / error) and whether the 4 were distinct cans.
- Still untested live: goal send through plan repair goal_manager -> repair planner; Anthropic path.
- Pre-existing flaky test: `test_sam3_chat::test_openai_searches_images_when_map_has_nothing` in the
  full suite only.
- `/api/health` raised an unhandled exception once on willow (21:34, before a restart); investigate.

## Mapping vs relocalize (left as is for now, Harel 2026-10-20)
- The base planner only plans on Neo4j / the prior topic; nothing writes the robot's live Hydra
  graph into Neo4j. Mapping mode does not seed Neo4j, so planning, goto snapping and chat Cypher
  use the PREVIOUS run's map. Options: (1) mirror the live map into Neo4j in mapping mode,
  (2) point the planner at the live DSG, (3) wipe Neo4j at a mapping launch + show "planning map:
  none"; all should show which map the planner uses.

## Video
- Viewers of the same camera share one robot stream and its parameters (last change wins);
  consider "lowest requested quality wins" so one viewer cannot raise the radio load.
- ZED: `relocalize` (and classic/regions/relocalize_awcd) zed_overrides have no
  `grab_frame_rate`; only `default` grabs at 30.
- Fleetlink video samples the newest frame on its own timer: make it event-driven (encode on a new
  frame, decimate by stamp) and loop the encoder stdin write until the whole frame is written.

## Robots / ops
- oh-my-zsh "Would you like to update? [Y/n]" prompt blocks tmux panes (zsh -ic) on hamilton:
  it stopped the zenoh router of run reloc_20261020_2038 (no router -> no ZED). Disable the prompt
  for launch panes (DISABLE_AUTO_UPDATE) or in hamilton's ~/.zshrc.
- Silvus: IPv4 unicast over the new SC4200H pair failed for >= 20 min after the radios booted,
  then worked with no change (root cause unknown; ask Silvus). Hamilton's radio mgmt IP
  172.17.0.95 clashes with docker0's 172.17/16. Robot links now default to Silvus
  (zenoh.connect_network / fleetlink_networks [silvus]); Launch page has a per-launch picker.
- Remap leak (PR #345 group-scoped ZED remaps + #351 global /tf remap): another agent handles it.
- Phone drive: Android Chrome joystick root cause unconfirmed; Drive now auto-focuses the only
  streaming robot.

## Live mesh over the radio (measured 2026-10-20 on the Pennovation courtyard-alleyways run)
- Mesh 972,201 vertices / 1,617,780 faces over 200 x 180 m -> fleetlink 8 m tiles: 406 tiles,
  28.5 MB raw, 13.4 MB zstd (largest tile 391 kB). Fleetlink sends only tiles whose hash changed,
  but a loop closure moves (nearly) every tile: up to ~13.4 MB per LC = ~2.2 min at the default
  C3 budget (100 kB/s), up to ~45 min at the auto minimum (5 kB/s).
- Option A (small, fleetlink only): per-tile mesh decimation on the robot for viewing (5-10x
  fewer faces -> ~1.5-3 MB per full send). The planner does not use the console mesh.
- Option B (needs Hydra): deform on the base. Kimera-PGMO moves each vertex by blending the rigid
  transforms of its k=3 nearest control points (weights 1 - d/d_max, time window 15 s,
  control points every 2.5 m; kimera_pgmo mesh_deformation.cpp interpDeformation). Send the
  odometric (undeformed) mesh tiles once (they only grow) + control points (position, stamp) once
  + optimized control-point poses per LC: 4,837 control points here -> ~135 kB raw / ~60-100 kB
  packed per LC (~1 s at 100 kB/s), ~100-200x less than re-sending tiles. Needs Hydra to publish
  the odometric mesh, per-vertex stamps, control points and optimized values; base ports the
  ~40-line blend to numpy; per-tile checksum vs the robot's deformed mesh with tile fallback.
- Measure first: replay a bag with loop closures through fleetlink and log C3 bytes per LC.
- DECIDED (Harel, 2026-10-20): do BOTH A and B. Decimated tiles by default, plus a viewer toggle
  "Full mesh" that requests full-detail tiles (per robot / per area) when the link allows it
  (often it does); base-side deformation for loop closures.
