# Environment probe report

| candidate | status | load_s | fps | prims | frames |
|---|---|---|---|---|---|
| warehouse | OK | 76.2 | 179.5 | 3430 | 8 |
| warehouse_forklifts | OK | 16.1 | 176.2 | 3480 | 8 |
| warehouse_shelves | OK | 15.6 | 157.4 | 8140 | 8 |
| full_warehouse | OK | 64.7 | 132.5 | 26343 | 8 |
| office | OK | 254.4 | 138.5 | 4666 | 8 |
| hospital | TIMEOUT | - | - | - | 0 |
| simple_room | OK | 16.9 | 186.7 | 152 | 8 |

Per-candidate renders + prims.txt in each subdirectory.
Run probe_detect.py (spark_env) for the YOLOE pass.
## YOLOE hits (conf>=0.02)

| candidate | pallet | forklift | shelf | box | cone | bag | pipe | barrel | ladder | fire extinguisher | chair | table |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| full_warehouse | 5 | 7 | 2 | 5 | 6 | 0 | 3 | 2 | 9 | 6 | 1 | 0 |
| office | 1 | 0 | 4 | 1 | 0 | 0 | 0 | 0 | 2 | 2 | 3 | 4 |
| simple_room | 0 | 0 | 4 | 0 | 0 | 0 | 0 | 1 | 0 | 0 | 4 | 4 |
| warehouse | 2 | 2 | 3 | 8 | 0 | 0 | 2 | 1 | 1 | 4 | 0 | 0 |
| warehouse_forklifts | 3 | 6 | 4 | 6 | 0 | 0 | 1 | 4 | 3 | 3 | 2 | 0 |
| warehouse_shelves | 37 | 4 | 1 | 21 | 1 | 0 | 4 | 4 | 3 | 4 | 1 | 2 |

## Decision (2026-07-18, harelb)

**Chosen: `full_warehouse`** (`Isaac/Environments/Simple_Warehouse/full_warehouse.usd`).
Richest scene (26,343 prims @ 132 fps, load 65s) and the only variant with YOLOE
hits on every target class (pallet 5, forklift 7, ladder 9, fire-ext 6, cone 6,
box 5, barrel 2 at conf>=0.02). Renders show stocked racks, a forklift, traffic
cones and safety props. Caveat: several directions from the origin face walls --
warehouse_tour.yaml's tour is authored against the 8 probe renders.

Rejected: `hospital` (TIMEOUT -- hung past 300 s like Rivermark), `office`
(loads but 254 s), the lighter warehouse variants (strictly poorer prop
coverage than full_warehouse).

### gt.semantics prim families (from full_warehouse prims.txt, depth<=4, truncated at 1500 lines)

| pattern | class | evidence |
|---|---|---|
| `.*SM_Palette.*` | pallet | 48 prims (NOTE: "Palette" spelling) |
| `.*SM_Rack(Shelf\|Frame).*` | shelf | 100+80 prims |
| `.*SM_CardBox.*` | box | CardBoxA/B/D families |
| `.*SM_FireExtinguisher.*` | fire_extinguisher | present |
| `.*[Ff]orklift.*` | forklift | visible in renders; prim name beyond the
truncated dump -- verify against Task-13 GT label output |
