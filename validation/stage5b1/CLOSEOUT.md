# STAGE-5B.1 CLOSEOUT — anti-merge replay + single-binary provenance

Date: 2026-11-21. Branch: `main` @ `04d2fd4` (nothing committed, nothing pushed).

## 1. Final binary provenance

```
build/predictive_nav_tracking/lidar_obstacle_tracker_node
sha256 5f6a641727641794b8cb90e3b1547fc9d860d47c5e70fb6713935359cdc280f9
mtime  2026-11-20 23:23:28.515 +0100
```

`colcon build --packages-select predictive_nav_msgs predictive_nav_tracking` was
a no-op: the binary already postdated the last tracker source edit
(23:23:00), so it was ALREADY the final binary. The hash was re-verified after
the anti-merge replay, after each rerun, and after `colcon test`, and was
identical every time.

**Rebuilds during the campaign: 0.**

Source hashes pinned to this result:

```
e2a8d06d…  src/predictive_nav_tracking/src/lidar_obstacle_tracker_node.cpp
871ab80f…  include/predictive_nav_tracking/compound_observation.hpp
91739c14…  include/predictive_nav_tracking/obstacle_classification.hpp
615fa8c2…  src/predictive_nav_msgs/msg/TrackedObject.msg
```

### Why the earlier campaign was split, and what this fixes

The bike (23:16) and pedestrian (23:22) captures predate the 23:22:53 /
23:23:00 source edits that added the pre-screen; vehicle (23:25), compact
(23:29) and planter (23:33) postdate the 23:23:28 build. The rerun below puts
pedestrian and bicycle on the same binary as the other three, so the whole
dataset is now single-binary.

## 2. A defect was found in `antimerge_check.py` and fixed

The script drove every object from `obj['velocity']`. A scenario object may
instead declare `segments` of `[duration, vx, vy]` — the schema
`stage4g2/run.py` defines — and `near_crossing`, the scenario Stage-4G3
measured as the hardest separation, uses exactly that form. The script would
have raised `KeyError: 'velocity'` before recording a single frame, so the
replay had never actually run.

Fix: added `velocity_at()` / `drive()`, implementing Stage-4G2's **STEP**
semantics at a segment boundary (verified to step at t = 5.0 s), re-commanding
an object only when its segment-active velocity changes — so a
constant-velocity object is still commanded exactly once, as before. The
Stage-4G4 `accel` ramp is explicitly *rejected* rather than approximated; no
replayed scenario carries it.

The script was otherwise audited and found genuine: it reads start poses and
velocities from `stage4g2/scenarios.json` at run time rather than restating
them, launches the FROZEN `tracker_params.yaml`, and varies only
`compound_enabled`, live, through the node's own parameter service. All 12
compound bounds in the `CompoundParams` struct defaults were confirmed equal to
the validated `stage5b_classification_tracker.yaml` values, so running under
the frozen navigation profile still exercises the real thresholds.

## 3. Anti-merge replay — PASS

`python3 antimerge_check.py --out validation/stage5b1/antimerge.json`

Geometry replayed (surface separation, two 0.2 m-radius cylinders):

| scenario | motion | min surface gap | max |
|---|---|---|---|
| crossing | perpendicular, 0.5 / 0.5 m/s | 0.060 m @ t=3.64 s | 4.121 m |
| near_crossing | 0.3 m/s + segmented turn (−y then +x) | 0.547 m @ t=5.00 s | 2.469 m |
| parallel | side by side, common velocity 0.3 m/s | 1.600 m (constant) | 1.600 m |
| opposing | head-on ±0.3 m/s | 0.500 m @ t=4.99 s | 2.738 m |

8 arms (4 scenarios × compound off/on), 50 frames each, **400 frames**.

| scenario | arm | frames | compound | track distribution | distinct ids |
|---|---|---|---|---|---|
| crossing | off / on | 50 / 50 | 0 / 0 | {2:50} / {2:50} | 2 / 2 |
| near_crossing | off / on | 50 / 50 | 0 / 0 | {2:50} / {2:50} | 2 / 2 |
| parallel | off / on | 50 / 50 | 0 / 0 | {2:50} / {2:50} | 2 / 2 |
| opposing | off / on | 50 / 50 | 0 / 0 | {2:50} / {2:50} | 2 / 2 |

With exactly two clusters on every frame there is exactly one candidate pair
per frame, so **200 pair-presentations were evaluated in the compound-on arms
and 0 were accepted**.

All five gate checks PASS: `all_arms_produced_frames`,
`zero_false_compound_associations`, `both_arms_present`,
`two_objects_still_resolved_with_stage_on`, `resolution_not_degraded_by_stage`.
Two distinct objects remained two distinct tracks on 100 % of frames in both
arms; the stage never degraded resolution.

> The one post-replay edit to the script was a single docstring line (imperative
> mood, to keep lint debt unchanged). Verified by AST comparison that the
> executable logic is byte-for-byte identical to what the replay ran, so the
> result stands without a re-run.

## 4. Single-binary rerun

`run.py --actor actor_pedestrian` (02:38:48 → 02:43:00) and
`--actor actor_bike` (02:43:08 → 02:47:13), both exit 0, both on the binary
above. Combined with the three actors that already ran on it into
`validation/stage5b1/finalbinary/` (5 files, 2,549 rows) and analysed with the
existing `analyze.py`. No actor model, scenario, threshold, LiDAR, tracker or
cluster parameter was changed.

### Stable-track confusion — final binary (61 tracks)

```
true \ pred   PED  BIKE   VEH   UNK
PEDESTRIAN      8     0     0     4   (n=12)
BIKE            1     7     0     5   (n=13)
VEHICLE         6     0    11     7   (n=24)
UNKNOWN         0     0     1    11   (n=12)
```

### Previous AFTER vs final binary

| metric | previous AFTER | final binary | verdict |
|---|---|---|---|
| tracks | 57 | 61 | +4 ephemeral bike fragment tracks |
| PED precision | 0.500 | **0.533** | slightly better |
| PED recall | 0.667 | **0.667** | identical |
| PED F1 | 0.571 | **0.593** | slightly better |
| BIKE precision | 1.000 | **1.000** | identical |
| BIKE recall (per track) | 0.667 (6/9) | 0.538 (7/13) | more correct tracks, more ephemeral ones |
| BIKE F1 (per track) | 0.800 | 0.700 | counting artifact, see below |
| BIKE recall (per detection) | 0.904 | **0.900** | materially identical |
| BIKE precision (per detection) | 1.000 | **1.000** | identical |
| UNKNOWN rate (per track) | 0.404 | 0.443 | +0.039, from the extra UNKNOWN fragments |
| false BIKE predictions | 0 | **0** | identical |
| compound detections | 274 | 269 | materially identical |
| FALSE compound associations | 0 | **0** (of 2,226 non-bike detections) | identical |
| head-on non-detections | 6 | **6** (same scenarios) | identical |

**The per-track BIKE recall change is a track-genesis counting artifact, not a
classifier change.** Per-track scoring weights a 1-scan track the same as a
72-scan one. The 13 bike tracks split cleanly by lifetime:

- 7 substantive tracks (71, 12, 40, 29, 72, 41, 29 scans) — **all 7 labelled
  BIKE, 7/7 = 1.000**
- 6 ephemeral fragment tracks (1–8 scans) — 5 UNKNOWN, 1 PEDESTRIAN

So every bicycle that was tracked for a meaningful duration was classified
BIKE. The extra tracks are transient wheel fragments that had not yet reached
the compound support threshold; they abstain (UNKNOWN) rather than assert a
wrong class. The scan-weighted figure, which is immune to this artifact, is
unchanged: BIKE recall 0.900 vs 0.904, precision 1.000 vs 1.000.

Compound geometry on the final binary remains bicycle-shaped — length
1.433–1.634 m, thickness 0.016–0.078 m, fill 0.667–0.834, max gap
0.462–0.628 m — versus 0.51 m / solid for a single wheel fragment.

## 5. Functional regression — 10/10 PASS

| package | test | kind | result |
|---|---|---|---|
| tracking | stage4g2_lifecycle | functional | PASSED |
| tracking | stage4g3_deblend | functional | PASSED |
| tracking | stage4g4_reachability | functional | PASSED |
| tracking | stage5b_classification | functional | PASSED |
| tracking | stage5b_tracker_integration | functional | PASSED |
| tracking | stage5b1_compound | functional | PASSED |
| tracking | stage5b1_compound_integration | functional | PASSED |
| costmap | stage4g2_multi_layer | functional | PASSED |
| costmap | stage4g4_reach_layer | functional | PASSED |
| costmap | stage4g5_hybrid_layer | functional | PASSED |

Lint/style debt unchanged at **7 pre-existing failures**: tracking flake8,
tracking pep257, tracking uncrustify, costmap uncrustify, bringup flake8,
bringup pep257, msgs xmllint. No frozen source file was reformatted.
`antimerge_check.py` was confirmed to carry exactly the same 2 flake8 findings
before and after the fix (pre-existing I100 and D107).

## 6. Git audit

`git diff --check` clean. 3 modified files (703 insertions, 22 deletions):
`TrackedObject.msg`, `predictive_nav_tracking/CMakeLists.txt`,
`lidar_obstacle_tracker_node.cpp`. Untracked: the Stage-5B/5B.1 headers, tests,
config, scripts, models and validation outputs. **Not committed, not pushed.**

`cluster_distance` is exactly **0.35 m** in all three configs
(`tracker_params.yaml:48`, `stage5b_classification_tracker.yaml:82`,
`production_tracker_params.yaml:57`).

## 7. Verdict

| # | criterion | result |
|---|---|---|
| A | Stage-4G3 Gazebo anti-merge replay executes | PASS |
| B | zero legitimate close pairs falsely compounded | PASS (0 / 200 pair-presentations; 0 / 2,226 non-bike detections) |
| C | ped + bicycle rerun on one final binary | PASS |
| D | final-binary BIKE recall non-zero, supports prior result | PASS (0.900 per detection; 7/7 substantive tracks) |
| E | no systematic false BIKE class | PASS (precision 1.000) |
| F | pedestrian not materially degraded | PASS (F1 0.571 → 0.593) |
| G | cluster_distance exactly 0.35 m | PASS |
| H | relevant functional tests pass | PASS (10/10) |
| I | head-on bike non-detection documented as sensor limitation | PASS (6 of 60, explicitly reported) |

**STAGE-5B / STAGE-5B.1 PASS — SIMULATION OBSTACLE CLASSIFICATION QUALIFIED**

Hard blocker: none.

Open, explicitly reported, NOT hidden: six head-on bicycle scenarios
(`actor_bike` approach and recede, v0.25/0.50/0.75) produce no cluster at all.
At that aspect the two wheel chords are collinear with the line of sight, so
the sensor returns no separable geometry. This is a sensor-observability limit
of the frozen clustering envelope, not a classifier error, and those scenarios
are excluded from the matrices and reported only under detection coverage.
