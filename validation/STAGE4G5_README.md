# Stage-4G5 — hybrid CV / reachability prediction policy

**PASS.** All 22 acceptance criteria are met. The hybrid policy does exactly
what the Stage-4G4 evidence said it should: it is **bit-for-bit the CV
representation while a track is fresh** and **bit-for-bit the reachable set
while it is coasting**, selected per track from that track's own observation
age. It recovers the whole of reachability's coasting benefit —
**coverage .524 → 1.000** at 0.5 s — while spending reachability on only
**2.43% of samples**, cutting total predictive region area **49.9%** against
always-reachability and landing within **7.2%** of always-CV.

The honest qualifier: as at Stage-4G4, **no navigation benefit is separable at
n=3.** Hybrid's minimum clearance is nominally best in 3 of 4 scenarios, but the
per-trial spread is wider than the difference between arms. What Stage-4G5
demonstrably buys is *simplification* — reachability can be kept for the 2.4%
of frames where it was measured to help, and dropped everywhere else — not a
measured navigation win.

Stage-4G6 has not started.

Checkpoint: Stage-4G4 at `eeb531b` and Stage-4G3 at `7adb5e0` are preserved and
re-verified. `stage4f-validated` still peels to
`68b2bf2ee1efb774ba563438a4c621b0052f61d5`. Work is confined to
`~/predictive_nav_ws`; `~/ur5e_ws` was neither read nor written.

## 1. Where the policy lives, and why

**Stage-4G5 changes only the consumer.** Every tracker file, every message
definition and every tracker parameter is byte-identical to Stage-4G4, compared
against the hashes Stage-4G4 itself recorded rather than retyped constants
([provenance.json](stage4g5/provenance.json), `all_unchanged: true`). The only
production change is `predicted_obstacle_layer.cpp` gaining a third
`prediction_mode` and one threshold.

That placement follows from the brief's own requirement that *exactly one
policy representation drive the production costmap for a given track at a given
time*. The tracker keeps publishing **both** representations — they remain
individually selectable for research, which is what makes the three-way
comparison in §4 possible with zero run-to-run variance — and the costmap picks
one per track.

Because the file that did change is a file both earlier modes depend on, the
claim "the existing modes are unchanged" is verified **behaviourally rather than
by hash**: after refactoring the two rasterizers into per-track helpers,
`stage4g2_multi_layer` and `stage4g4_reach_layer` both reproduce their committed
Stage-4D / Stage-4G4 outputs **exactly**
([cv_layer_unchanged.json](stage4g5/cv_layer_unchanged.json),
[reach_layer_unchanged.json](stage4g5/reach_layer_unchanged.json)).

## 2. Selection logic and the fresh threshold

```
observation_age = TrackedObjectArray.header.stamp − TrackedObject.stamp

for each eligible track, independently:
    if observation_age <= fresh_threshold:   rasterizeTrackCv(track)
    else:                                    rasterizeTrackReachability(track)
```

Eligibility (`kalman_initialized`, `track_age <= track_timeout`) is shared by
all three modes, so the mode can only change **how** an eligible track is
painted, never **which** tracks are painted. The age used is exactly the
quantity the tracker itself used to build the reachable sets, so producer and
consumer cannot disagree about how stale a track is. No hidden layer state
participates and one track's age never influences another's choice.

**`fresh_threshold = 0.1 s`, derived rather than guessed.** Measured over 1,532
recorded samples:

| quantity | measurement |
|---|---|
| scan interval | mean **0.2000 s**, range 0.198–0.201 (n=961) |
| `observation_age` when associated this scan | **exactly 0.0 s**, 1496 samples (97.7%) |
| `observation_age` after ≥1 missed scan | **≥ 0.198 s**, 36 samples |
| samples in `0 < age < 0.198` | **zero** |

The distribution is strictly bimodal with an empty gap. Half a scan interval
sits in the middle of that gap with ~0.1 s of margin either side, so the
classification is insensitive to timestamp jitter. Any value in (0, 0.198)
produces identical behaviour on this evidence.

## 3. No discontinuous ghost behaviour

The layer calls `resetMaps()` and re-`touch()`es its previous written bounds
every cycle, so it rebuilds its own grid from scratch and expands the update
window to cover whatever it painted last time. A track that switches
representation therefore has **no stale-region path to clear, because there is
no stale region**. The hybrid branch is a single `if`/`else` over one track, so
a track cannot contribute a CV corridor and a reachable set in the same update.

[hybrid_layer.json](stage4g5/hybrid_layer.json) pins all of this cell for cell:

| check | result |
|---|---|
| all tracks fresh → hybrid grid **identical** to `cv_covariance` | pass (13362 cells both) |
| all tracks coasting → hybrid grid **identical** to `reachability` | pass (12032 cells both) |
| mixed → **identical** to max-composition of CV(fresh) ⊕ reachability(coasting) | pass |
| negative control: mixed ≠ all-CV and ≠ all-reachability | pass |
| each transition step equals its single-representation render | pass (5 steps) |
| reacquisition returns **exactly** to the CV grid | pass (13362 → 13362) |
| prediction never disappears mid-transition | pass |
| expired track paints 0 cells; empty array leaves 0 ghost cells | pass |
| one track expires while another stays fresh → survivor unchanged | pass |

## 4. Transition analysis

29 coasting episodes across the hybrid recordings (9 reacquired, 20 expired),
**124 frames with a coasting and a fresh track simultaneously**. Regions are
characterised at the 0.5 s horizon.

Representative episode, `nav_occlusion_maneuver_01` track 2 — a full
CV → reachability → CV cycle with a second track observed throughout:

| step | t (s) | age (s) | policy | area m² | max extent m | costmap cells | live tracks |
|---|---:|---:|---|---:|---:|---:|---:|
| A. before switch | 4.76 | 0.000 | **cv** | 0.164 | 0.229 | 1112 | 2 |
| B1. first missed | 4.96 | 0.198 | **reachability** | 0.681 | 0.466 | 1161 | 2 |
| B2. second missed | 5.16 | 0.399 | reachability | 1.550 | 0.702 | 1138 | 2 |
| B3. third missed | 5.36 | 0.600 | reachability | 3.144 | 1.001 | 1183 | 2 |
| B4. fourth missed | 5.56 | 0.798 | reachability | 5.759 | 1.354 | 1217 | 2 |
| B5. fifth missed | 5.76 | 0.999 | reachability | 9.880 | 1.773 | 1212 | 2 |
| C. reacquisition | 5.96 | 0.000 | **cv** | 0.233 | 0.273 | 812 | 2 |

Pooled over all 29 episodes:

| switch | centroid jump m | area ratio | extent ratio | costmap cell delta |
|---|---:|---:|---:|---:|
| CV → reachability (n=29) | **0.028** (0.005–0.098) | **3.92×** (3.09–4.20) | 1.98× (1.76–2.05) | +79 (−80…+323) |
| reachability → CV (n=9) | 0.425 (0.237–0.639) | **0.045×** (0.024–0.077) | 0.21× (0.15–0.28) | +34 (−400…+563) |

Read carefully:

- **The CV→reachability switch is a bound change, not a position change.** The
  centroid moves 0.028 m on average — the nominal centre is the same
  constant-velocity extrapolation in both representations — while the area
  jumps ~3.9×. The system becomes more conservative without jumping somewhere
  else.
- **The reachability→CV jump of 0.425 m is the correction, not a
  discontinuity.** The object genuinely moved while it was unobserved; on
  reacquisition the filter is corrected by a real measurement. A representation
  that did *not* jump there would be hiding the error.
- Expiry cases (20 of 29) end with the track removed, not with a region left
  behind — see §8.
- No smoothing was added. There is no evidence any of these discontinuities
  harm navigation (§7), so smoothing them would be aesthetics.

## 5. Coverage: fresh and coasting kept strictly apart

27 trials, 6,744 scored samples. Hybrid is evaluated on **exactly the same
recorded samples** as the two representations it selects between — the tracker
publishes both on every frame and Stage-4G5 changes only the consumer — so this
three-way comparison carries **zero run-to-run variance**.

### FRESH samples

| h (s) | representation | n | coverage | area m² | coverage/m² | missed GT |
|---:|---|---:|---:|---:|---:|---:|
| 0.5 | CV | 1896 | .952 | 0.166 | **5.749** | 91 |
| 0.5 | reachability | 1896 | .992 | 0.268 | 3.700 | 15 |
| 0.5 | **hybrid** | 1896 | **.952** | **0.166** | **5.749** | 91 |
| 1 | CV | 1778 | .999 | 1.114 | 0.897 | 2 |
| 1 | reachability | 1778 | 1.000 | 2.246 | 0.445 | 0 |
| 1 | **hybrid** | 1778 | **.999** | **1.114** | **0.897** | 2 |
| 2 | CV / reachability / **hybrid** | 1561 | 1.000 / 1.000 / **1.000** | 13.664 / 29.808 / **13.664** | | 0 |
| 3 | CV / reachability / **hybrid** | 1346 | 1.000 / 1.000 / **1.000** | 65.946 / 137.196 / **65.946** | | 0 |

**Hybrid ≡ CV on fresh samples**, to every reported digit, by construction.

### COASTING samples

CV is scored **as consumed** — the same published region read as though it
described the present, which is what a downstream consumer actually gets from a
message with no staleness semantics.

| h (s) | representation | n | coverage | area m² | missed GT |
|---:|---|---:|---:|---:|---:|
| 0.5 | CV as consumed | 42 | **.524** | 0.164 | **20** |
| 0.5 | reachability | 42 | 1.000 | 2.432 | 0 |
| 0.5 | **hybrid** | 42 | **1.000** | **2.432** | **0** |
| 1 | CV as consumed | 42 | .976 | 1.109 | 1 |
| 1 | reachability / **hybrid** | 42 | 1.000 / **1.000** | 9.787 / **9.787** | 0 |
| 2 | CV / reachability / **hybrid** | 42 | 1.000 / 1.000 / **1.000** | 13.646 / 65.473 / **65.473** | 0 |
| 3 | CV / reachability / **hybrid** | 38 | 1.000 / 1.000 / **1.000** | 65.902 / 223.665 / **223.665** | 0 |

**Hybrid ≡ reachability on coasting samples**, again exactly. Twenty
ground-truth positions that a CV-only consumer would have missed at 0.5 s are
covered.

**§8's hypothesis is therefore confirmed in both halves**: fresh coverage and
area match CV exactly; coasting coverage matches reachability exactly.

## 6. Simplification value

| | samples | share |
|---|---:|---:|
| frames served by **CV** | 6580 | **97.57%** |
| frames served by **reachability** | 164 | **2.43%** |

| total predictive region area (summed over every scored sample) | m² | vs hybrid |
|---|---:|---:|
| always-reachability | 247,460 | 2.00× |
| **hybrid** | **124,084** | 1.00× |
| always-CV | 115,782 | 0.93× |

**Hybrid uses 49.9% less predictive area than always-reachability while
retaining 100% of its coasting coverage, and sits only 7.2% above always-CV.**
Per horizon the reduction is 31.8% / 45.7% / 51.1% / 49.7% at 0.5 / 1 / 2 / 3 s
([policy.json](stage4g5/policy.json)).

This is the Stage-4G5 result that matters: **reachability can be kept only
where it was measured to have value.**

The live costmap confirms the same split independently — the layer's own
counter reported `cv_tracks=320 reach_tracks=8` (2.4%) on a navigation trial.

## 7. Focused navigation comparison

Four arms, one Gazebo launch per trial, differing **only** in the live
`predicted_obstacle_layer` parameters. 4 scenarios × 4 arms × 3 trials.

| scenario | arm | success | collisions | min clearance m | goal s | path m | stopped % |
|---|---|---:|---:|---:|---:|---:|---:|
| **constant velocity** | reactive | 3/3 | 0 | 0.082 | 23.33 | 6.68 | 46.0 |
| | cv | 3/3 | 0 | 0.803 | 23.54 | 6.08 | 44.7 |
| | reachability | 3/3 | 0 | 0.788 | 23.80 | 6.04 | 46.4 |
| | **hybrid** | 3/3 | 0 | **0.819** | **23.33** | **6.04** | 46.2 |
| **reversal** | reactive | **0/3** | **3** | **−0.185** | — | 3.76 | 72.5 |
| | cv | 3/3 | 0 | 0.740 | 23.40 | 6.13 | 46.0 |
| | reachability | 3/3 | 0 | 0.712 | 23.73 | 6.14 | 45.2 |
| | **hybrid** | 3/3 | 0 | **0.757** | 23.40 | 6.06 | 46.8 |
| **short occlusion** | reactive | 3/3 | 0 | 0.412 | 26.40 | 7.62 | 52.4 |
| | cv | 3/3 | 0 | 0.533 | 31.14 | 7.18 | 46.6 |
| | reachability | 3/3 | 0 | 0.428 | 28.73 | 7.95 | 44.0 |
| | **hybrid** | 3/3 | 0 | **0.681** | 31.27 | **6.80** | 48.8 |
| **occlusion + manoeuvre** | reactive | 3/3 | 0 | 0.659 | 25.94 | 6.41 | 41.6 |
| | **cv** | 3/3 | 0 | **0.707** | 25.27 | 6.26 | 42.2 |
| | reachability | 3/3 | 0 | 0.683 | 25.33 | 6.38 | 38.7 |
| | hybrid | 3/3 | 0 | 0.679 | 25.47 | 6.29 | 40.2 |

**What is real:** prediction versus no prediction. Reactive collides in 3/3
reversal trials with the bodies actually intersecting (−0.185 m); all three
predictive arms are collision-free at ~0.71–0.76 m.

**What is not separable at n=3:** the differences *between* predictive arms.
Hybrid's minimum clearance is nominally best in three of four scenarios, but the
per-trial spread swamps it — on short occlusion, CV recorded
[0.497, 0.345, 0.757] and hybrid [0.820, 0.516, 0.705]; on constant velocity,
CV [0.806, 0.789, 0.813] and hybrid [0.875, 0.824, 0.758]. Overlapping ranges.
The correct statement is **"indistinguishable"**, not "hybrid wins".

*A methodological note, recorded because it nearly produced a false result.* At
the original 32 s trial window the hybrid arm showed 2/3 unresolved goals on the
short-occlusion scenario. Inspection showed the CV arm resolving its goals at
29.0 s and 30.3 s — within 2 s of the deadline — i.e. goal resolution was being
**truncated rather than measured** for every predictive arm. The window was
extended to 45 s and all four arms were re-run; all are now 3/3. The earlier
figure was a deadline artefact, and reporting it as a hybrid failure would have
been wrong.

## 8. Constant velocity, reversal, occlusion, expiry

- **Constant velocity (§10).** With `observation_age = 0` on every frame the
  hybrid policy never invokes reachability: 0 reachability track-frames across
  all three `nav_cv_control` trials, and 0 coasting episodes. Prediction
  footprint, costmap cells (599.6 vs CV's 597.9 mean), navigation time, path
  length and clearance are all CV's, because the code path *is* CV's. Hybrid
  does not differ materially while observations are fresh, and the reason is
  structural rather than empirical.
- **Reversal.** Continuously observed, so again 0 coasting episodes and pure CV.
  Hybrid matches CV; both avoid the collisions the reactive arm suffers.
- **Short occlusion.** 4–5 consecutive missed scans at t ≈ 5.4–6.3 s in every
  trial; hybrid switches for exactly those frames and returns to CV.
- **Occlusion + manoeuvre (§11).** The most important scenario. 4–5 missed scans
  at t ≈ 5.0–5.8 s. The CV baseline's stale-anchor weakness remains visible and
  unmodified (.524 coverage as consumed at 0.5 s). Reachability is conservative
  but large (2.432 m² against 0.164). Hybrid switches **only** for the coasting
  frames, and the switch occurs on the first missed scan — `observation_age`
  0.000 → 0.198 s — with reacquisition restoring CV on the first re-associated
  scan.
- **Expiry (§14).** 20 of the 29 episodes end in expiry rather than
  reacquisition. In the unit test the full chain is pinned: fresh CV →
  coasting reachability → expired → **zero cost**, and fresh CV → coasting
  reachability → reacquisition → fresh CV with the grid **exactly** equal to the
  original CV grid, i.e. the reachable region is completely cleared. Zero ghost
  cells in every case.

## 9. Multi-object

**124 frames** across the hybrid recordings carried a coasting track and a
fresh track at the same instant, each on its own policy — e.g.
`nav_occlusion_maneuver_01` at t = 4.96–5.76 s, where id 2 used reachability at
ages 0.198→0.999 s while the other track stayed on CV throughout. Two observed
tracks (`occlusion_short`), three observed tracks (`triple`, 153 CV
track-frames, 0 reachability) and one-reacquired-while-another-observed are all
covered. The unit test's negative control additionally proves the absence of
global leakage: the mixed render differs from both all-CV and all-reachability.

## 10. Stage-4G3 regression and static world

Re-run with the costmap in **hybrid** mode and scored by the unchanged
Stage-4G2 scorer ([regression.json](stage4g5/regression.json),
`all_identical: true`):

| trial | vs committed Stage-4G3 |
|---|---|
| `crossing_01` (deblending, 5 merge frames + 4 occlusion frames) | **identical** field for field |
| `triple_01` | **identical** field for field |
| `static_navigation_01` | **identical** field for field |

Static world in hybrid mode: **0 confirmed tracks, 0 false confirmed, 0 false
persistent, 0 of 96 grids with predictive cost**, navigation SUCCEEDED. Static
rejection, deblending and track lifecycle cannot be affected by a consumer-side
policy — the tracker binary is byte-identical — and this confirms it rather than
asserting it. `stage4g3_deblend`, `stage4g2_lifecycle`,
`stage4g4_reachability`, `stage4g2_multi_layer`, `stage4g4_reach_layer` and the
four evaluator unit tests all pass unchanged.

## 11. Runtime

Same scenarios under each mode, bucketed by live track count. The tracker is
byte-identical across modes, so its figures are a control and the layer's
`updateCosts` is where any mode cost would appear.

| mode | tracks | tracker callback mean / max ms | layer updateCosts mean / max µs |
|---|---:|---:|---:|
| cv_covariance | 1 / 2 / 3 | 14.98 / 16.14 / 16.54 — max 35.11 | 15.4 / 14.4 / 14.8 — max 46.0 |
| reachability | 1 / 2 / 3 | 14.43 / 16.52 / 16.87 — max 34.66 | 23.8 / 23.7 / 21.5 — max 89.0 |
| **hybrid** | 1 / 2 / 3 | 15.29 / 16.35 / **16.36** — max **35.49** | 16.6 / 15.2 / **13.5** — max **40.4** |

Hybrid's layer cost sits at or below CV's and well below reachability's, which
is the expected consequence of serving reachability on 2.4% of track-updates.
**Max tracker callback 35.49 ms = 17.7% of the 200 ms scan period**; the costmap
layer's contribution is **tens of microseconds**, four orders of magnitude below
any Nav2 costmap budget.

## 12. Acceptance criteria

| # | Criterion | Verdict |
|---|---|---|
| 1 | CV mode unchanged and reproducible | **PASS** — `stage4g2_multi_layer` reproduces Stage-4D output exactly |
| 2 | Reachability mode unchanged and reproducible | **PASS** — `stage4g4_reach_layer` reproduces Stage-4G4 output exactly |
| 3 | Hybrid selects CV for fresh tracks | **PASS** — grid identical to cv_covariance |
| 4 | Hybrid selects reachability for coasting tracks | **PASS** — grid identical to reachability |
| 5 | Selection is per-track, not global | **PASS** — 124 mixed-state frames; negative control passes |
| 6 | Observation-age semantics explicit | **PASS** — `header.stamp − track.stamp`, threshold derived from measured timing |
| 7 | One representation per track at a time | **PASS** — single if/else; mixed render equals the exact composition |
| 8 | CV→reachability switch has no duplicate/ghost region | **PASS** — step-wise grid equality |
| 9 | Reacquisition clears old reachable occupancy | **PASS** — returns exactly to the CV grid |
| 10 | Expiry clears all predictive occupancy | **PASS** — 0 cells, 0 ghost cells |
| 11 | Fresh hybrid coverage/area matches CV | **PASS** — identical to every digit |
| 12 | Coasting hybrid coverage matches reachability | **PASS** — 1.000 at every horizon |
| 13 | Hybrid uses materially less area than always-reachability | **PASS** — **49.9%** less |
| 14 | Occlusion + manoeuvre evaluated | **PASS** — §8 |
| 15 | Constant-velocity behaviour remains CV-like | **PASS** — 0 reachability frames; identical code path |
| 16 | Multi-object mixed fresh/coasting works | **PASS** — §9 |
| 17 | Stage-4G3 deblending does not regress | **PASS** — identical field for field |
| 18 | Static-world false predictive occupancy zero | **PASS** — 0 of 96 grids |
| 19 | Focused navigation comparison completed | **PASS** — §7, 48 trials |
| 20 | Runtime within budget | **PASS** — 17.7% of scan period; layer in µs |
| 21 | `stage4f-validated` unchanged | **PASS** — `68b2bf2…` |
| 22 | `~/ur5e_ws` untouched | **PASS** — not read or written |

## 13. Limitations

1. **No navigation benefit is demonstrated.** At n=3 the between-arm
   differences are smaller than the per-trial spread. Hybrid's value here is
   simplification and coasting coverage, not a measured navigation win.
2. **The coasting statistics rest on 42 samples** from 27 trials. The effect is
   large (.524 → 1.000) but the sample is small, and all of it comes from
   4–5-scan occlusions in one arena with one sensor geometry.
3. **The CV 2σ ceiling persists.** From h ≥ 1 s every representation covers
   1.000 on fresh samples, so this evidence cannot discriminate them at longer
   horizons; only the 0.5 s column carries information there.
4. **Costmap cell counts are not a clean area proxy.** CV mode's
   `max_influence_radius` clamp bounds the ellipse's *bounding box*, so a
   high-variance CV sample paints a filled square while a reachable set paints
   an inscribed ellipse. That pre-existing Stage-4D behaviour is why the
   49.9% area saving is reported from published region geometry, and why the
   costmap cell deltas at a switch are small and occasionally negative.
5. **`fresh_threshold` is validated only for a 5 Hz scan.** It is derived from
   that interval and would need re-deriving for a different sensor rate. It is
   a parameter, not a constant.
6. **Two representations are still maintained.** Hybrid reduces where
   reachability *runs*, not the amount of code that must be kept correct.

## 14. Exact reproduction

Trial directories are exclusive-created, so use a fresh output directory. Keep
`--domain` below 226: the runner adds a per-trial offset and DDS rejects domain
ids above 232.

```bash
cd ~/predictive_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Unit / white-box regressions (hybrid, plus the two modes it must not disturb)
ROS_DOMAIN_ID=98 build/predictive_nav_costmap/stage4g5_hybrid_layer
ROS_DOMAIN_ID=97 build/predictive_nav_costmap/stage4g4_reach_layer
ROS_DOMAIN_ID=93 build/predictive_nav_costmap/stage4g2_multi_layer
ROS_DOMAIN_ID=96 build/predictive_nav_tracking/stage4g4_reachability
ROS_DOMAIN_ID=95 build/predictive_nav_tracking/stage4g3_deblend
ROS_DOMAIN_ID=94 build/predictive_nav_tracking/stage4g2_lifecycle
python3 -m unittest discover -s src/predictive_nav_bringup/scripts/stage4g2 -p 'test_*.py'

# Coverage: hybrid is derived from the SAME recordings as the other two modes,
# so the Stage-4G4 prediction matrix is re-scored rather than re-run. To
# regenerate those recordings, use the Stage-4G4 commands in STAGE4G4_README.md.
python3 src/predictive_nav_bringup/scripts/stage4g4/evaluate.py \
  --logs validation_logs/stage4g4/prediction validation_logs/stage4g4/regress \
         validation_logs/stage4g4/occlusion \
  --fresh-threshold 0.1 \
  --out validation_logs/stage4g5_repro/prediction_eval.json

# Focused navigation: four arms, one launch configuration
for arm in reactive:100 cv_covariance:120 reachability:140 hybrid:160; do
  python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
    --out validation_logs/stage4g5_repro/nav_${arm%%:*} --layer-mode ${arm%%:*} \
    --scenarios nav_cv_control,nav_reversal,nav_occlusion_short,nav_occlusion_maneuver \
    --trials 3 --domain ${arm##*:}
done

# Runtime, one batch per mode
for arm in cv_covariance:100 reachability:120 hybrid:140; do
  python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
    --out validation_logs/stage4g5_repro/perf_${arm%%:*} --layer-mode ${arm%%:*} \
    --scenarios single,occlusion_short,triple --trials 1 --domain ${arm##*:}
done

# Stage-4G3 regression and static world, costmap in hybrid mode
python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
  --out validation_logs/stage4g5_repro/regress_hybrid --layer-mode hybrid \
  --scenarios crossing,static_navigation,occlusion_maneuver --trials 1 --domain 160

# Transition analysis and curation
python3 src/predictive_nav_bringup/scripts/stage4g5/transitions.py \
  --logs validation_logs/stage4g5_repro/regress_hybrid validation_logs/stage4g5_repro/perf_hybrid \
         validation_logs/stage4g5_repro/nav_hybrid \
  --fresh-threshold 0.1 --out validation_logs/stage4g5_repro/transitions.json
for arm in reactive cv_covariance reachability hybrid; do
  python3 src/predictive_nav_bringup/scripts/stage4g4/evaluate.py \
    --logs validation_logs/stage4g5_repro/nav_$arm \
    --out validation_logs/stage4g5_repro/nav_${arm}_eval.json
done
python3 src/predictive_nav_bringup/scripts/stage4g5/summarize.py \
  --prediction-eval validation_logs/stage4g5_repro/prediction_eval.json \
  --nav-eval validation_logs/stage4g5_repro/nav_*_eval.json \
  --transitions validation_logs/stage4g5_repro/transitions.json \
  --perf-logs validation_logs/stage4g5_repro/perf_cv_covariance \
              validation_logs/stage4g5_repro/perf_reachability \
              validation_logs/stage4g5_repro/perf_hybrid \
  --out-dir validation_logs/stage4g5_repro/curated

# A/B against either pure mode, no rebuild needed:
#   predicted_obstacle_layer.prediction_mode: cv_covariance | reachability | hybrid
```

## 15. Should reachability stay in the production system?

**Yes — as the hybrid coasting fallback, and only there.**

The case for keeping it is now narrow, measured and specific: during observation
loss a CV prediction read as current covers **52.4%** of ground truth at 0.5 s,
and 20 ground-truth positions across these trials fall outside it. Reachability
covers all of them. No choice of CV confidence level fixes this cleanly — the
failure is a centre bias, not insufficient spread, and Stage-4G4 measured even a
3σ CV ellipse still missing 9.5%.

The case against keeping it *as the default* is equally measured and was already
made at Stage-4G4: it doubles predictive area for no coverage gain whenever
observations are fresh, which is 97.6% of the time.

Hybrid resolves that tension at a cost of one threshold and one `if`. It is what
should ship. Two caveats stated plainly: **no navigation benefit has been
demonstrated for any predictive variant over another**, so if the project's bar
is "measurable navigation improvement", reachability has not cleared it and
removing it entirely would be defensible; and maintaining two representations
still costs code, even though only one now runs on any given frame.

## 16. Handoff

**Recommended Stage-4G6 first step: stop adding representations and test the
one decision that is still unmeasured — whether short-horizon coasting coverage
changes navigation outcomes at all.** Every prediction-quality question this
arena can answer has now been answered; the arms differ by less than the n=3
noise floor in all four navigation scenarios. The options, in order of evidence
support:

1. Raise statistical power on the existing scenarios (n ≥ 20 per arm, the
   Stage-4F protocol) so a 0.05–0.15 m clearance difference becomes decidable.
   This is the cheapest way to settle whether any of Stage-4G4/4G5 matters to
   navigation.
2. Build scenarios with **longer** occlusions, where coasting is the dominant
   regime rather than 2.4% of frames. The current gaps are 4–5 scans; at that
   duration CV survives on ellipse size alone beyond 0.5 s.
3. If neither shows a navigation effect, remove reachability from the
   production path and keep it as a research mode — the honest outcome that
   §18 of the brief explicitly allows for.

Nothing measured here justifies intent prediction, neural trajectory
prediction, camera classification, JPDA or MHT. No Stage-4G6 work was started.
