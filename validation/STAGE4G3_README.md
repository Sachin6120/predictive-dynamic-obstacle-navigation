# Stage-4G3 — robustness through LiDAR cluster merges and short occlusions

**PASS.** The Stage-4G2 close-crossing failure was reproduced, its raw geometry
inspected, and its cause confirmed as a perception merge rather than an
association fault. A conservative track-aware deblending step now recovers the
identities when — and only when — the scan actually contains separable returns
from both objects. **Close-crossing ID switches go from 2 per trial to 0 in both
trials**, with every previously solved scenario bit-for-bit unaffected and zero
false tracks anywhere. Greedy association, the Kalman model, the Stage-4D
costmap mathematics, all frozen tracker parameters and the Nav2 configuration
are unchanged. Stage-4G4 has not started.

Checkpoint: clean `main` at `45bf1a0`. The Stage-4F reference still peels to
`68b2bf2ee1efb774ba563438a4c621b0052f61d5`. Work is confined to
`~/predictive_nav_ws`; `~/ur5e_ws` was neither read nor written.

## 1. What was measured before anything was changed

The Stage-4G2 recordings retain every raw scan, so the merge could be examined
directly rather than re-simulated. `scripts/stage4g3/frontend.py` reimplements
the node's range-aware static rejection and 0.35 m clustering offline, and
`scripts/stage4g3/analyze_merges.py` **verifies that reconstruction against the
centroids the runtime tracker actually published** before using it:

| Check | Result |
|---|---|
| Frames compared (22 runs) | 1,197 |
| Cluster **count** identical to runtime | 1,197 / 1,197 (100%) |
| Centroid agreement | worst error 2.1e-15 m |

Every number below therefore describes the real tracker's own clustering, not a
re-implementation that merely resembles it. Full output:
[merge_inspection.json](stage4g3/merge_inspection.json).

### The close crossing splits into two physically different phases

`crossing_01`, times relative to object release. "visible" is the number of
recorded returns attributable to each cylinder's surface (a **diagnostic**
attribution that never reaches the tracker):

| t (s) | clusters | cluster diameter | visible A / B | gap between the two point sets | true centre spacing | phase |
|---:|---:|---:|---:|---:|---:|---|
| 2.877 | 2 | — | 8 / 7 | — | .774 | resolved |
| 3.078 | **1** | .956 | 8 / 7 | .318 | .669 | **case A** |
| 3.279 | **1** | .914 | 8 / 7 | .268 | .577 | **case A** |
| 3.477 | **1** | .849 | 8 / 7 | .222 | .507 | **case A** |
| 3.678 | **1** | .798 | 8 / 7 | .228 | .466 | **case A** |
| 3.879 | **1** | .785 | 6 / 7 | .269 | .463 | **case A** |
| 4.077 | 2 | — | 5 / 7 | — | .499 | resolved |
| 4.278–4.878 | 1 | ~.33 | **0–2** / 8 | — | .567–.872 | **case B** |
| 5.079 onward | 2 | — | 5–6 / 7 | — | .991+ | resolved |

**Case A (4–5 scans): both objects are fully visible inside one cluster.** The
merged component carries 13–16 returns of which *every single one* lies on one
of the two cylinder surfaces, 6–8 per object. The information needed to separate
them is present in the scan.

**Case B (4 scans): one object physically occludes the other.** Object A drops
to 0–2 returns — below the 3-point cluster minimum — while B keeps 8. No amount
of cleverness can recover a measurement that was never sensed. Across all 22
runs: **71 case-A frames, 4 case-B frames.**

The Stage-4G2 failure is the case-A phase: the blended centroid could only feed
one track (greedy assignment is one-to-one), the other starved, and after five
misses it expired. **This is why association was not the limiting factor** — with
one measurement there is nothing for any assignment rule, greedy or global, to
distribute. The re-measured greedy-vs-global diagnostic still finds
**0 differing decisions over 806 comparable frames** (733 at Stage-4G2).

### The thresholds are measured, not assumed

| Quantity | n | min | p50 | p95 | max |
|---|---:|---:|---:|---:|---:|
| Diameter, cluster holding **one** object | 1,878 | .127 | .350 | .414 | **.468** |
| Diameter, cluster holding **two** objects | 71 | **.785** | .880 | 1.003 | 1.037 |
| Points, one object | 1,878 | 3 | 7 | 13 | 16 |
| Points, two objects | 71 | 12 | 14 | 15 | 16 |
| Largest gap between returns **on one object** | 71 | .072 | .114 | **.147** | **.161** |
| Smallest gap **between two merged objects** | 71 | **.123** | .235 | .327 | .350 |

Two conclusions drive the design. **Diameter separates the two populations
perfectly**, with a 0.317 m margin — any threshold from 0.50 to 0.75 m flags
100% of merges and 0% of single objects. **Point count does not separate them
at all** (single up to 16, merged from 12), so it is deliberately *not* used as
merge evidence. And in 65 of 71 merged frames the gap between the two objects
exceeds the worst gap within either object, which is what makes a split
defensible from the scan alone.

## 2. What changed in the tracker

One new stage sits between clustering and the unchanged association:

```
dynamic points → clustering (unchanged) → [merge detection + deblending] → greedy association (unchanged) → KF / lifecycle (unchanged)
```

`build_clusters()` is the checkpoint's `cluster_points()` with member points
retained; its filters and centroid arithmetic are untouched. Everything else —
static rejection, association, Kalman predict/update, pruning, prediction,
message construction — is byte-identical to `45bf1a0`.

### Merge detection

A cluster is *suspicious* only if **both** hold:

1. its diameter exceeds `merge_min_cluster_diameter` (0.55 m — above every
   single-object cluster ever recorded, below every merged one), and
2. at least two **confirmed** tracks (`observations >= 3`) have a predicted
   position within `merge_track_radius` (0.35 m) of one of its **points**.

Distance is measured to the nearest member point, not to the centroid:
during a merge the blended centroid sits between the objects and would look
equally near to both, which is precisely the ambiguity being diagnosed.
Tentative tracks are excluded — an unconfirmed track is not yet evidence that
an object exists, so it must not be able to carve up a cluster.

### Deblending

Points are assigned to whichever of the two furthest-apart claiming seeds is
nearer in Mahalanobis distance, using each track's predicted position covariance
`(F P Fᵀ + Q)[0:2,0:2] + R`. The seeds are the tracker's own Kalman predictions:
**runtime state, never ground truth.** The partition is then accepted only if
*all* of the following hold, and otherwise the cluster is left completely intact:

| Guard | Value | Why |
|---|---:|---|
| points per child | ≥ 3 | a child must be an acceptable measurement on its own (== `cluster_min_points`) |
| child diameter | ≤ .50 m | a child must look like one object (max observed: .468 m) |
| child centroid separation | ≥ .30 m | two centroids on top of each other are not two objects |
| finite centroids | required | numerical safety |
| **empty gap between the two point sets** | **≥ .16 m** | **above the .161 m worst within-object spacing** |

The gap test is the decisive anti-fabrication guard. Nearest-seed partitioning
will happily slice any blob in two; requiring the returns themselves to show
empty space at the split surface is what prevents a solid structure from being
invented into two objects. Case B fails it automatically, because an occluded
object contributes no second point set at all.

### The ambiguous case, and why one centroid never updates two filters

When a cluster is suspicious and claimed but the split is refused, it keeps its
single blended centroid and is counted as an `ambiguous` merge. Greedy
association is one-to-one, so that centroid updates **exactly one** track — the
best-supported visible one — and the other coasts on prediction. This is
Stage-4G3's required behaviour, and it needed no code change; a regression test
now pins it, asserting that one centroid produces exactly one observation
increment, exactly one miss, and leaves the coasting track's stored state
bit-identical. No "coast both" rule was added, because no measured frame in any
of the 22 runs reached that case with two claimants (the only refusals were
`insufficient_tracks`, below).

### Deblending cannot invent an identity

Two confirmed tracks are required, so objects that were merged from the moment
they appeared have only one track and are never split. This is visible in the
sweep: at 0.55 m and 0.65 m spacing the tracker never resolved two objects, the
clusters were flagged suspicious on all 30 frames, and the split was **refused
every time** for lack of a second track. Deblending *maintains* identities that
the sensor already established; it does not manufacture them.

## 3. Before / after — 24 fresh Gazebo runs

Identical arena, robot, 5 Hz LiDAR, scenario definitions, Nav2 configuration and
0.45 m evaluation gate as Stage-4G2. The Stage-4G2 baseline evidence is
untouched in `validation/stage4g2/`.

| Trial | ID switches | Obs. fragments | Identity fragments | Purity | Matched fraction | Velocity err m/s | First → last IDs |
|---|---:|---:|---:|---:|---:|---:|---|
| **crossing_01** G2 | **2** | 2 | 2 | .636 | .890 | .089 | [1,2] → [3,1] |
| **crossing_01** G3 | **0** | 1 | 0 | **1.000** | **.960** | **.048** | [1,2] → **[1,2]** |
| **crossing_02** G2 | **2** | 3 | 1 | .969 | .920 | .079 | [1,2] → [1,2] |
| **crossing_02** G3 | **0** | 1 | 0 | **1.000** | **.960** | **.049** | [1,2] → [1,2] |
| resolution_75 G2 | 0 | 1 | 0 | 1.000 | .967 | .059 | [1,2] → [1,2] |
| resolution_75 G3 | 0 | **0** | 0 | 1.000 | **1.000** | **.028** | [1,2] → [1,2] |

Every other scenario is unchanged within run-to-run noise — parallel (×2),
opposing (×2), near crossing (×2), wider crossing (×2), short occlusion (×2),
single (×2), three objects, navigation, static navigation, long occlusion:
**0 ID switches, purity 1.000, matched fraction 1.000** wherever they were
before. Per-trial: [trials.csv](stage4g3/trials.csv),
[results.json](stage4g3/results.json).

**Zero false confirmed tracks, zero false persistent tracks and zero duplicate
frames across all 24 runs.**

### What happens to the crossing now

Both merged and occluded phases are handled by the mechanism appropriate to
each. In `crossing_01`: the five case-A scans are split, so ID 2 keeps receiving
its own measurement and never starves; the four case-B scans are **not** split,
so track 1 coasts with `missed_count` 1→4 and its observation count frozen at
37; at t+5.058 it reacquires **with the same ID** after 1.002 s and 4 missed
scans. Stage-4G2 recorded a disappearance here instead, and a new ID 3 after it.

| | Stage-4G2 | Stage-4G3 |
|---|---|---|
| crossing_01 | ID 2 expired at 1.002 s; A reappeared as ID 3 | no disappearance; ID 1 reacquired, same ID, 4 misses |
| crossing_02 | two reacquisitions | one reacquisition, same ID, 4 misses |

Note the existing boundary this depends on and which was **not** changed:
association precedes pruning, so a detection arriving at 1.002 s can still
revive a track whose observation age has just passed the 1.0 s timeout.
`track_timeout` and `max_missed_scans` are untouched.

## 4. Separation sweep

Two cylinders in parallel at 3–3.9 m range, 6 s each, extended beyond the
Stage-4G2 range at both ends:

| Centre spacing | Surface gap | Merged frames | Split frames | ID switches | Purity | Matched fraction | Outcome |
|---:|---:|---:|---:|---:|---:|---:|---|
| .55 m | .15 m | 30 / 30 | 0 | 0 | 1.000 | .500 | never resolved; split correctly **refused** |
| .65 m | .25 m | 30 / 30 | 0 | 0 | .500 | .500 | never resolved; split correctly **refused** |
| .75 m | .35 m | 2 / 30 | 2 | 0 | 1.000 | **1.000** | deblended (G2: .967) |
| .85 m | .45 m | 0 / 30 | 0 | 0 | 1.000 | 1.000 | raw-resolved |
| .95 m | .55 m | 0 / 30 | 0 | 0 | 1.000 | 1.000 | raw-resolved |
| 1.05 m | .65 m | 0 / 30 | 0 | 0 | 1.000 | 1.000 | raw-resolved |

- **Raw-clustering resolution limit: ~0.85 m** centre spacing in this geometry
  (0.75 m is marginal — 2 of 30 frames merge). Unchanged from Stage-4G2; nothing
  in Stage-4G3 improves raw clustering.
- **Deblending resolution limit: 0.75 m** in the parallel sweep, and **0.46 m
  in the crossing**, where the objects were already separately tracked before
  they merged and the viewing angle kept both surfaces exposed
  (`min_resolved_center_separation_m` 0.494 / 0.504; merged frames up to 0.679).

The two limits differ because deblending needs a *prior*: it extends identity
through a merge that begins after both objects are resolved, and does nothing
for objects that arrive already merged. **Below ~0.65 m in this geometry, two
never-resolved objects remain one object, and that is reported as such rather
than fabricated.** Separation alone does not predict resolvability — angular
occlusion and viewing geometry matter more.

## 5. Occlusion, static world, prediction, costmap, navigation

**Short occlusion** (both trials): B loses 2–3 scans, **same ID returns** after
0.60–0.80 s. Unchanged from Stage-4G2.

**Long occlusion** (expiry control): B has <3 returns for 11 scans, ID 2 expires
at 1.2 s since its last observation and becomes ID 3 on reappearance — **exactly
as before**. One switch, one fragment, purity .766. Deblending correctly does
nothing here: there is no second point set to split. Track semantics were kept
honest; no timeout was extended to make a test pass.

**Static world — the regression that matters most for a splitting rule.** The
Stage-4G1 large-arena benchmark, 6 runs, predictive layer enabled, deblending on:

| | Stage-4G1 | Stage-4G3 |
|---|---:|---:|
| Runs | 6 | 6 |
| Persistent false tracks (max) | **0** | **0** |
| False predictive cells (max) | **0** | **0** |
| Frames with any track | **0** | **0** |
| Navigation | 6/6 SUCCEEDED | 6/6 SUCCEEDED |

No static residual was split into a fake dynamic object; the gap guard is
specifically tested against a gapless 0.87 m structure with two seeds placed on
its ends, which must not split. The focused moving-robot static navigation run
also holds **zero active tracks of any kind and zero predictive cells** over 97
grids. No birth heuristic was added, because no evidence calls for one.
[static_large.json](stage4g3/static_large.json).

**Multi-track prediction: 12,132 published samples** (11,370 at G2) passed
per-parent checks of predicted position, absolute timestamp and `F P Fᵀ + Q`
position covariance — **0 failures**. Through the occlusion, the coasting track's
predictions stay constant while the observed track's vary independently every
scan, confirming no shared state. Covariance grows with horizon as expected
(0.013 → 5.24 m² over 0.5 → 3.0 s).

*Honest detail:* a coasting track's **stored** covariance does not grow, and its
published prediction set stays anchored to its last observation stamp rather than
being re-propagated to the current scan. That is the pre-existing Stage-4C/4D
semantic documented at Stage-4G2 ("stored state/covariance and object stamp
remain at the last observation"), and downstream freshness rules act on the
ageing object stamp. Stage-4G3 did not change it; doing so would be a Stage-4D
mathematics change, which was out of scope.

**Costmap.** Through the crossing's merge, occlusion and reacquisition, every
eligible track is represented in every sampled grid (16/16 during the crossing
window; 112/112 over the navigation run), with **no duplicate corridor** and no
ghost region on reacquisition. The isolated Stage-4D layer regression still
passes cell-for-cell: 432 overlapping cells, 327 A-only, 336 B-only,
`both == max(A,B)` exact, independent expiry and removal exact, and **0 ghost
cells** from an empty array. Stage-4D mathematics and parameters are unchanged.

**Navigation.** Two-object focused run: goal **SUCCEEDED**, controller lifecycle
active, displacement 5.903 m, 0 identity events, both tracks represented in all
112 sampled grids. Static navigation: **SUCCEEDED**, 5.871 m. No controller
retuning; the Nav2 configuration file is byte-identical.

## 6. Runtime

Same build and machine, all markers enabled. Deblending is the new row.

| Objects | Frames | Clustering ms | **Deblending ms** | Assoc + KF ms | Callback mean / max ms |
|---:|---:|---:|---:|---:|---:|
| 1 (G2 → G3) | 100 | .018 → .022 | — → **.077** | .144 → .109 | 14.176 → **13.900** / 32.327 → 32.179 |
| 2 (G2 → G3) | 100 | .030 → .038 | — → **.107** | .202 → .175 | 14.775 → **14.367** / 32.922 → 33.236 |
| 3 (G2 → G3) | 50 | .060 → .061 | — → **.123** | .274 → .217 | 15.075 → **15.243** / 34.287 → 34.087 |

On the frames where a split actually happens (close crossing), deblending costs
**0.088–0.184 ms**, mean 0.14 ms. Maximum deblending time observed anywhere is
0.257 ms. The total callback is unchanged within noise and remains dominated by
the TF lookup/wait (~13 ms), not by tracking: **max 34.087 ms = 17.0% of the
200 ms scan period.** [performance.json](stage4g3/performance.json).

## 7. Regression tests

`stage4g3_deblend` runs against the production translation unit using the
**actual recorded map-frame returns** of the crossing frames above, with the
tracker's own predicted positions as seeds:

| Check | Result |
|---|---|
| Case-A merged frames split, each child nearer a *different* object, inside the .45 m gate | 3 / 3 |
| Each child is a better measurement than the blended centroid it replaced | worst child error .191 m vs .312 m for the merged centroid |
| Case-B occlusion frames refused | 2 / 2 |
| Single-object cluster not split, not flagged ambiguous | pass |
| Gapless 0.87 m structure with two seeds not split, flagged ambiguous | pass |
| One confirmed track cannot produce a split | pass |
| Tentative tracks cannot seed a split | pass |
| One ambiguous centroid updates exactly one track; the other coasts untouched | pass |

The pre-existing `stage4g2_lifecycle` and `stage4g2_multi_layer` regressions and
the four evaluator unit tests all still pass unchanged.

## 8. Failure classification of what remains

| Class | Status after Stage-4G3 |
|---|---|
| **A** — two distinguishable surfaces, deblending failed | **None observed.** All 71 case-A frames with two confirmed tracks were split. |
| **B** — effectively one observable object (physical occlusion) | **Remains, and is correct.** 4 frames in the crossing, 11 in the long-occlusion control. Handled by coasting; no measurement is fabricated. |
| **C** — clustering joined distinguishable objects | **Remains upstream, now compensated.** Raw clustering still merges at ≤.75 m; deblending recovers identity when a prior exists, and the raw limit (~.85 m) is unchanged. |
| **D** — association failed after valid separate detections | **None.** 0 greedy-vs-global differences over 806 comparable frames. |
| **E** — timeout / reacquisition failure | **Only the intended one:** the long-occlusion control expires at 1.2 s by design. The crossing no longer expires anything. |
| **F** — static rejection | **None.** 0 false tracks, 0 predictive cells over 6 large-world runs plus the moving-robot static run. |

**The remaining physical limit**, stated plainly: two objects that are never
resolved as separate clusters cannot be separated. At ≤0.65 m centre spacing
(≤0.25 m surface gap) in this geometry the tracker reports one object, and
matched fraction is 0.5 because one GT object genuinely has no detection. That
is an observability boundary of a single 5 Hz planar LiDAR at 3–4 m, not a
tracker defect, and it is reported rather than papered over.

## 9. Acceptance criteria

| # | Criterion | Verdict |
|---|---|---|
| 1 | G2 failure reproduced, raw geometry inspected | **PASS** — reproduced; front-end verified bit-exact on 1,197 frames |
| 2 | Observable merge vs true occlusion distinguished | **PASS** — 71 case A / 4 case B, separate handling |
| 3 | Deblending uses only LiDAR + track state | **PASS** — seeds are the KF's own predictions; GT only in the evaluator |
| 4 | Split only when scan evidence supports it | **PASS** — 5 guards incl. the .16 m gap test; refused 30/30 at .55 m and .65 m |
| 5 | No ambiguous centroid updates two tracks | **PASS** — one-to-one greedy, pinned by regression test |
| 6 | Close-crossing identity improves materially | **PASS** — 2 → 0 switches, purity .636 → 1.000, vel err .089 → .048 |
| 7 | No regression in previously solved scenarios | **PASS** — all unchanged; 0 false tracks |
| 8 | Short occlusion still reacquires same ID | **PASS** — .60–.80 s, same ID, both trials |
| 9 | Long occlusion still expires | **PASS** — expires at 1.2 s, unchanged |
| 10 | Static false persistent tracks remain zero | **PASS** — 0 over 6 large-world runs + static nav |
| 11 | Multi-track predictions independent | **PASS** — 12,132 samples, 0 failures |
| 12 | Costmap stable through merge/occlusion/reacquisition | **PASS** — 16/16 and 112/112 represented, no duplicate corridor |
| 13 | No ghost predictive costs after expiry | **PASS** — isolated layer regression, 0 ghost cells |
| 14 | Three-object scaling functional | **PASS** — IDs 1/2/3 throughout, purity 1.000 |
| 15 | Runtime comfortably below 200 ms | **PASS** — max callback 34.087 ms (17.0%); deblending ≤ .257 ms |
| 16 | Remaining ambiguity measured and documented | **PASS** — section 8 and the sweep |
| 17 | Stage-4G1 behaviour preserved | **PASS** — static rejection parameters byte-identical; benchmark reproduced |
| 18 | `stage4f-validated` unchanged | **PASS** — still `68b2bf2ee1efb774ba563438a4c621b0052f61d5` |
| 19 | `~/ur5e_ws` untouched | **PASS** — not read or written (mtime 2026-09-09, predates this work) |

## 10. Limits of these claims

Two repetitions per scenario exercise timing and noise variation; they are not
independent population samples. One sensor viewpoint, identical 0.4 m cylinders,
constant-velocity motion, and a 0.45 m evaluation gate with approximate surface
labelling bound every number here. The diameter and gap thresholds are measured
for objects of this size and must be rescaled for substantially different
obstacles — they are parameters, and the rule they encode is "clearly larger
than one tracked object", not a hard-coded cylinder. Deblending performs only
two-way splits: in a three-way merge it separates along the two furthest-apart
predicted tracks and the third receives no measurement that scan and coasts.
No such frame occurred in the 24 runs, so that path is reasoned, not measured. No Hungarian, JPDA, MHT, appearance model or learned segmentation
was implemented, and none is required by the measured evidence.

## 11. Exact reproduction

Trial directories are exclusive-created, so use a fresh output directory.

```bash
cd ~/predictive_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Pre-change inspection: verifies the offline front-end against the runtime
# tracker, then derives every threshold in section 1 from the recordings.
python3 src/predictive_nav_bringup/scripts/stage4g3/analyze_merges.py \
  --logs validation_logs/stage4g2 \
  --map src/predictive_nav_bringup/maps/stage4f_benchmark.yaml \
  --out validation/stage4g3/merge_inspection.json

# Unit / white-box regressions
python3 -m unittest discover \
  -s src/predictive_nav_bringup/scripts/stage4g2 -p 'test_*.py'
ROS_DOMAIN_ID=95 build/predictive_nav_tracking/stage4g3_deblend
ROS_DOMAIN_ID=94 build/predictive_nav_tracking/stage4g2_lifecycle
ROS_DOMAIN_ID=93 build/predictive_nav_costmap/stage4g2_multi_layer

# Gazebo matrix (24 runs)
python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
  --out validation_logs/stage4g3_repro/baseline \
  --scenarios single,parallel,opposing,crossing,crossing_resolved,near_crossing,occlusion \
  --trials 2 --domain 140
python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
  --out validation_logs/stage4g3_repro/regressions \
  --scenarios triple,navigation,static_navigation,occlusion_long,resolution_55,resolution_65,resolution_75,resolution_85,resolution_95,resolution_105 \
  --trials 1 --domain 160
python3 src/predictive_nav_bringup/scripts/stage4g2/summarize.py \
  --baseline validation_logs/stage4g3_repro/baseline \
  --regressions validation_logs/stage4g3_repro/regressions \
  --out validation_logs/stage4g3_repro/curated

# Stage-4G1 large-world static regression
python3 install/predictive_nav_bringup/lib/predictive_nav_bringup/run_stage4f_bench.py \
  --out validation_logs/stage4g3_repro/static_large \
  --conditions nominal --modes predictive --trials 6

# A/B against the checkpoint's behaviour, no rebuild needed:
#   set deblend_enabled: false in tracker_params.yaml
```

## 12. Handoff

Deblending is a *maintenance* mechanism for identities the sensor has already
established. The measured next constraint is not association or track
management — both are clean — but the raw clustering limit of ~0.85 m and the
observability gap when one object hides another. **Recommended Stage-4G4 first
step: decide whether the remaining case-B gap should be addressed by
prediction-side work (reachability / intent, which is what a coasting track
needs to stay useful over a longer gap) or by a sensing change, and review that
scope before implementing either.** Nothing here justifies JPDA, MHT, camera
fusion or learned instance segmentation. No Stage-4G4 work was started.
