# Stage-4G4 — conservative reachability-based obstacle prediction

**PASS, with a genuinely mixed result that is reported as measured rather than
as hoped.** Bounded-motion reachable sets are implemented as a second,
independent future-motion representation alongside the untouched Stage-4C
constant-velocity Gaussian. They are strictly correct as bounds, they are the
only representation that achieved **100% coverage on coasting tracks at every
horizon**, and they fix a real defect: a CV prediction read by a consumer that
assumes it describes *now* covers only **52.4%** of coasting ground truth at
0.5 s, because the CV message carries no notion of its own staleness.

**But reachability is not more area-efficient than simply raising the CV
confidence level, and it produced no measurable navigation benefit.** Across
three focused navigation scenarios, CV-predictive and reachability-predictive
were indistinguishable (and reachability was marginally *worse* on minimum
clearance in all three). Both beat reactive decisively on the reversal
scenario, where reactive collided in 3/3 trials. That is the honest headline:
**the win at Stage-4G4 belongs to prediction, not to reachability.**

Stage-4G5 has not started.

Checkpoint: the Stage-4G3 result at `7adb5e0` is preserved and re-verified
bit-for-bit. `stage4f-validated` still peels to
`68b2bf2ee1efb774ba563438a4c621b0052f61d5`. Work is confined to
`~/predictive_nav_ws`; `~/ur5e_ws` was neither read nor written.

## 1. What was inspected before anything was written

| Item | Finding |
|---|---|
| Stage-4C predictor | `predict_future()` peeks `F(dt)·x`, `F P Fᵀ + Q_CWNA(dt, 0.5)` at `dt = i·0.5 s`, live state never mutated |
| Messages | `TrackedObject` (+`float64[16] covariance`), `TrackedObjectPrediction` (+`float64[4] position_covariance`) |
| Stage-4D layer | skips `track_age > track_timeout`; rasterizes the `sigma_level=2.0` ellipse with cost `= min_cost + exp(−½d²)·exp(−0.35t)·span`, i.e. a **probability-density ratio**, combined `updateWithMax` |
| Stage-4G3 coasting | `update_track()` runs only on association, so a coasting track's `kf_x`, `kf_P` and `last_update` are frozen and predictions stay anchored at the last observation |

### The measurement that shaped the whole design

The obstacle is driven by `gz-sim-velocity-control-system`, which sets planar
velocity **directly**. Measured from the retained Stage-4G3 ground-truth
odometry (20 Hz):

| Trial | max speed | ‖a‖ p50 | ‖a‖ p95 | ‖a‖ max | where |
|---|---:|---:|---:|---:|---|
| single_01 | .285 | .020 | .128 | **5.67** | t=0 release, t=10 stop |
| crossing_01 obj1 | .486 | .020 | .119 | **9.51** | t=0, t=10 |
| near_crossing_01 obj1 | .286 | .020 | .117 | **7.87** | **t=4.99, the `segments` boundary** |

So the existing scenario mechanism produces a **step** velocity change —
effectively unbounded acceleration — and between segments ‖a‖ ≈ 0.12 m/s² is
odometry noise. `a_max` therefore **cannot** be read off the simulator as it
stood, and no finite bound covers a step Δv at short horizon (a `½a t²`
envelope only catches a step Δv at `t ≥ 2Δv/a_max`).

The scenario driver was therefore extended with an optional `accel` field that
**ramps** the commanded velocity at a declared bound, so the obstacle's
dynamics become a known physical quantity. Both are evaluated: ramped
manoeuvres (in model) and step manoeuvres (deliberately out of model, reported
as such).

`a_max = 0.5 m/s²` was frozen on that evidence, and verified after the fact:
under a 0.5 m/s² command the measured obstacle reaches ‖a‖ p95 = 0.503,
max 0.545 at the 0.2 s scan timescale, and its **actual deviation from a CV
extrapolation stayed inside the resulting envelope at every horizon**
(0.046 < 0.062 m at 0.5 s, 0.207 < 0.250 at 1 s, 0.448 < 0.562 at 1.5 s,
0.737 < 1.000 at 2 s, 1.314 < 2.250 at 3 s).

## 2. The reachability model

`src/predictive_nav_tracking/include/predictive_nav_tracking/reachability.hpp`.

Let the last real observation give filtered position `p0`, velocity `v0`,
`s0 = |v0|`, and let `t` be the **total** elapsed time since that observation
(observation age + horizon). For any admissible trajectory,

```
d(t) := p(t) − (p0 + v0 t) = ∫₀ᵗ ( v(s) − v0 ) ds
```

Two bounds constrain the integrand simultaneously:

```
|v(s) − v0| ≤ a_max · s                (bounded acceleration)
|v(s) − v0| ≤ |v(s)| + |v0| ≤ v_max + s0   (bounded speed; the obstacle may
                                            reverse, hence the SUM)
```

so with crossover `t_c = (v_max + s0) / a_max`,

```
              ⎧ ½ · a_max · t²                                  t ≤ t_c
|d(t)| ≤ r_det = ⎨
              ⎩ ½ · a_max · t_c² + (v_max + s0) · (t − t_c)     t > t_c
```

This is **isotropic** — it bounds the magnitude of the deviation and ignores
the coupling between direction and achievable speed — so the disc of radius
`r_det` about the constant-velocity nominal is a strict **superset** of the
true reachable set. The speed cap can only ever shrink it, so a region
violating `max_speed` cannot be produced.

**Which uncertainty is which, stated explicitly.**

| Term | Kind | Source |
|---|---|---|
| `reach_radius` = `r_det(t)` | **deterministic** kinematic bound | `a_max`, `v_max`, `t`, `s0` |
| `sigma_semi_major/minor/yaw` | **statistical** | `k·√λ` of the Stage-4C propagated `F P Fᵀ + Q` position block |
| `safety_margin` | **deterministic** additive body allowance | parameter, **0.0 by default** |
| `semi_major/minor` | published region | sum of the three above |

The final region is the Minkowski **outer** bound of the statistical ellipse
and the deterministic disc: `semi_axes = k√λ + r_det + margin`, oriented at
`sigma_yaw`. (The true Minkowski sum of an ellipse and a disc is an offset
curve, not an ellipse; inflating both semi-axes contains it, so this too errs
conservatively.) **No deterministic bound is ever written into a covariance
field, and no covariance is ever relabelled as a reachable set.**

## 3. Interface

`TrackedObject` gains one field. A separate topic was rejected: the costmap
layer would have to correlate two streams by stamp and id and duplicate
identity and freshness state.

```
ReachabilityPrediction[] reachability_predictions   # empty when disabled
```

`TrackedObjectPrediction` and `position_covariance` are **untouched**, and the
new field is empty unless `reachability_enabled`, so every pre-Stage-4G4
consumer is unaffected. Each sample publishes `time_from_now`, `stamp`,
`observation_age`, `total_time`, `position`, `velocity`, `reach_radius`,
`speed_capped`, `sigma_semi_major`, `sigma_semi_minor`, `sigma_yaw`,
`covariance_sigma_level`, `safety_margin`, `semi_major`, `semi_minor`,
`valid` — enough to reconstruct the region exactly, and to recover `p0` and
`v0` for offline re-derivation.

**The time anchors deliberately differ, and this is the point.**

| | anchor of `time_from_now` | knows how stale it is |
|---|---|---|
| `predictions` (Stage-4C, unchanged) | `TrackedObject.stamp`, the last real observation | **no** |
| `reachability_predictions` (new) | `TrackedObjectArray.header.stamp`, this scan | **yes** |

## 4. Observation-age handling

For a coasting track the region uses `total_time = observation_age + horizon`
for **both** the constant-velocity nominal centre and the bound around it, so
the track pays for the time it has not been seen. Past `max_observation_age`
the region is published with `valid = false` rather than grown without limit.
The legacy CV message is **not** re-anchored — that would be a silent change to
Stage-4C semantics.

Measured on a real coasting track in the unit test: identical stored state,
0.8 s of unobserved time, `reach_radius` 0.0625 → **0.4225 m**; and the live
filter state is bit-identical before and after prediction.

## 5. Parameters

| Parameter | Value | Physical justification |
|---|---:|---|
| `reachability_enabled` | true | set false → byte-identical Stage-4G3 payload |
| `max_acceleration` | 0.5 m/s² | the bound the scenario driver commands; verified to contain the measured deviation at every horizon |
| `max_speed` | 0.8 m/s | scenario obstacles run 0.15–0.50 m/s; headroom without licensing motion the models cannot perform |
| `reachability_horizon` / `_time_step` | 3.0 / 0.5 s | mirror the Stage-4C grid so both are scored at identical horizons |
| `base_safety_margin` | **0.0 m** | zero on purpose: reachability must not get additive inflation that CV does not |
| `max_observation_age` | 1.0 s | equals `track_timeout`, so a region never outlives its track |
| `covariance_sigma_level` | 2.0 | equals the Stage-4D layer's `sigma_level`; both representations use the same k on the same covariance |

All 20 frozen Stage-4B/4C/4G3 tracker parameters are unchanged
([provenance.json](stage4g4/provenance.json)).

## 6. Coverage vs region size — 27 Gazebo trials

Both representations scored at the same horizons, each against ground truth at
the absolute time **it** claims. `cv_as_consumed` scores the *same* CV region
against ground truth at `scan + h`, i.e. what a consumer reading the current
message actually gets.

| h (s) | set | n | coverage | area m² |
|---:|---|---:|---:|---:|
| 0.5 | CV, fresh | 1896 | .952 | .166 |
| 0.5 | **Reach, fresh** | 1896 | **.992** | .268 |
| 0.5 | CV, coasting (own anchor) | 42 | .952 | .164 |
| 0.5 | **CV, coasting AS CONSUMED** | 42 | **.524** | .164 |
| 0.5 | **Reach, coasting** | 42 | **1.000** | 2.432 |
| 1 | CV, fresh | 1778 | .999 | 1.114 |
| 1 | Reach, fresh | 1778 | 1.000 | 2.246 |
| 1 | CV, coasting as consumed | 42 | .976 | 1.109 |
| 1 | Reach, coasting | 42 | 1.000 | 9.787 |
| 2 | CV / Reach, fresh | 1561 | 1.000 / 1.000 | 13.664 / 29.808 |
| 3 | CV / Reach, fresh | 1346 | 1.000 / 1.000 | 65.946 / 137.196 |

**The CV 2σ ellipse saturates at coverage 1.000 from h ≥ 1 s in every scenario
— including reversal and occlusion+manoeuvre.** That is not an accident: the
Stage-4C process noise is a CWNA model with `accel_noise = 0.5 m/s²`, i.e. the
*same physical acceleration assumption*, expressed probabilistically. Beyond
0.5 s the reachability envelope is therefore adding area inside a region CV
already covers, and every "reachability wins" at h ≥ 1 s would be an artefact
of a ceiling, not a finding.

Per scenario at h = 0.5 s ([coverage.json](stage4g4/coverage.json)):

| scenario | CV | CV as consumed | Reach |
|---|---:|---:|---:|
| cv_control | .949 | .949 | **.990** |
| accelerate | .898 | .898 | **.980** |
| accelerate_step *(out of model)* | .898 | .898 | .959 |
| decelerate_stop | .898 | .898 | **.959** |
| reversal | .898 | .898 | **1.000** |
| reversal_step *(out of model)* | .939 | .939 | .980 |
| occlusion_short | .950 | **.935** | **.994** |
| occlusion_maneuver | .962 | **.944** | **1.000** |
| occlusion_long | 1.000 | 1.000 | 1.000 |
| crossing | .958 | **.938** | .969 |
| triple | .952 | .952 | .993 |
| reach_triple | .951 | **.923** | **1.000** |

## 7. Efficiency — the result that does not favour reachability

Pooled over all samples, varying **only** the confidence level applied to the
unchanged published covariance, against varying **only** `a_max`:

| representation | h=0.5 coverage | area m² | h=1 coverage | area m² |
|---|---:|---:|---:|---:|
| CV k=1 | .023 | .04 | .939 | .28 |
| CV k=1.5 | .486 | .09 | .988 | .63 |
| CV k=2 *(frozen)* | .952 | .17 | .999 | 1.11 |
| CV k=3 | .999 | .37 | 1.000 | 2.51 |
| Reach a=0.25 | .980 | .25 | 1.000 | 1.75 |
| Reach a=0.5 *(frozen)* | .992 | .31 | 1.000 | 2.42 |
| Reach a=1.0 | .999 | .47 | 1.000 | 4.00 |

**At matched coverage, CV is the cheaper representation.** CV k=3 reaches .999
at 0.37 m²; reachability needs `a_max = 1.0` and 0.47 m² for the same. This
holds at every horizon. Reported because it is true, not because it was
expected: *if all you want is coverage per unit area, raising the CV sigma is
the better instrument.*

The one place that argument fails is the coasting case, where the failure is a
centre **bias**, not insufficient spread. On coasting frames, scored as
consumed:

| representation | h=0.5 coverage | area m² |
|---|---:|---:|
| CV k=1 | **.000** | .04 |
| CV k=1.5 | **.000** | .09 |
| CV k=2 | .524 | .16 |
| CV k=3 | .905 | .37 |
| **Reach (a=0.5)** | **1.000** | 2.432 |

Even a 3σ CV ellipse still misses 9.5% of coasting ground truth at 0.5 s.
Reachability is the only representation that reached 1.000 — at 6.6× the area.

`base_safety_margin` sensitivity (h=0.5): 0.0 m → .992 / 0.31 m²;
0.1 m → .999 / 0.54 m²; 0.2 m → 1.000 / 0.82 m². Left at **0.0** so no free
inflation enters the comparison.

Offline reproduction of the runtime bound was **exact on every sample**
(max error 0.0 m), so the sweeps describe the shipped mathematics, not a
re-implementation of it.

## 8. Scenario results

- **Constant velocity** (`cv_control`, §17 control) — CV does what it assumes:
  .949 coverage at 0.164 m². Reachability buys +.041 for 1.62× the area. In
  navigation, CV-predictive reached the goal 1.3 s sooner with 0.035 m more
  minimum clearance. **CV is the better choice under genuine CV motion, and is
  reported as such.**
- **Acceleration** (ramped) — .898 → .980 at h=0.5.
- **Deceleration / stop** — .898 → .959 at h=0.5.
- **Direction reversal** (ramped) — .898 → **1.000** at h=0.5, the largest
  in-model gain of any scenario.
- **Step controls** (`reversal_step`, `accelerate_step`) — deliberately out of
  model. Reachability still improves (.939→.980, .898→.959) but does **not**
  reach 1.000, which is the expected and correct behaviour: a step Δv is not
  coverable by any finite acceleration bound at short horizon.
- **Short occlusion** — 2–3 missed scans; same ID returns. CV as consumed
  .935, reachability .994.
- **Occlusion + manoeuvre** (the stress test) — target changes velocity while
  hidden. CV as consumed .944, reachability **1.000**.

## 9. Multi-object independence

`reach_triple` (3 tracks, one manoeuvring) and `triple`: 0 ID switches, purity
1.000, matched 1.000, IDs 1/2/3 throughout. The unit test additionally builds
three tracks through the production association path, feeds only two on the
next scan, and asserts that the coasting track — **and only it** — pays an
observation age, that no two tracks share a reachable centre, and that
predicting one track never mutates another's state. `crossing` and `triple`
re-run on the Stage-4G4 build reproduce their **committed Stage-4G3 results
field for field** ([regression.json](stage4g4/regression.json),
`all_identical: true`).

## 10. Costmap

`prediction_mode: cv_covariance` (default) | `reachability`, switchable live.

**Cost semantics, which differ on purpose.** CV mode:
`exp(−½d²_maha)·exp(−decay·t)` — a probability density ratio, peaked at the
mean. Reachability mode: **uniform** inside the region, scaled only by the same
temporal decay — set membership. A bounded-motion reachable set has no internal
density, so painting a peak would invent information the model does not
contain. Both respect `max_cost < INSCRIBED_INFLATED_OBSTACLE` and compose with
`updateWithMax`.

[reach_layer.json](stage4g4/reach_layer.json): CV output is **byte-identical**
whether or not tracks carry reachability data; the reachable set contains every
cell the CV ellipse claims; painted area grows with horizon; a coasting track's
footprint grows with observation age (12024 → 12364 cells); a region past
`max_observation_age` paints **0** cells; multi-object composition is exactly
`max(A,B)` (10592 overlap, 1432/1432 unique); expiry and removal leave **0**
ghost cells; and CV mode is restored exactly after switching back. The
Stage-4D/4G2 layer regression still passes unchanged (432 / 327 / 336 cells,
0 ghost cells).

*One measured asymmetry, reported rather than smoothed over:* in the navigation
runs 14 of 1456 sampled grids (1.0%) carried predictive cost in CV mode but not
in reachability mode, all within the first ~1 s of a trial while the obstacle
was outside the 6×6 m local window. Cause: CV mode's `max_influence_radius`
clamp bounds the ellipse's **bounding box**, so a high-variance long-horizon
sample degenerates into a filled square whose corner clips the window (1–29
cells at the grid corner), while a reachable set stays an inscribed ellipse.
That is pre-existing Stage-4D behaviour which Stage-4G4 must not change.

## 11. Static world

| mode | runs | persistent false tracks | grids with predictive cost | navigation |
|---|---:|---:|---:|---|
| `cv_covariance` | 1 | **0** | **0 / 97** | SUCCEEDED |
| `reachability` | 2 | **0** | **0 / 96, 0 / 97** | SUCCEEDED |

Zero confirmed tracks, zero false predictive occupancy, zero persistent false
tracks in both modes. Inflating a region cannot create an object where the
tracker reports none ([static_world.json](stage4g4/static_world.json)).

## 12. Focused navigation comparison

One Gazebo launch per trial; the three arms differ **only** in the live
`predicted_obstacle_layer` parameters (`enabled`, `prediction_mode`). No
controller weight, planner, costmap geometry or tracker parameter differs
between arms. 3 scenarios × 3 arms × 3 trials.

| scenario | arm | success | collisions | min clearance | mean clearance | goal time s | path m | stopped % |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| nav_cv_control | reactive | 3/3 | 0 | .117 | 2.328 | 23.33 | 6.76 | 46.0 |
| nav_cv_control | **cv** | 3/3 | 0 | **.803** | 2.372 | **23.60** | 6.06 | 45.3 |
| nav_cv_control | reach | 3/3 | 0 | .768 | 2.351 | 24.87 | 6.04 | 47.0 |
| nav_reversal | reactive | **0/3** | **3** | **−.173** | 1.333 | — | 3.74 | 72.5 |
| nav_reversal | **cv** | 3/3 | 0 | **.731** | 1.964 | 24.27 | 6.11 | 44.5 |
| nav_reversal | reach | 3/3 | 0 | .717 | 1.947 | 24.20 | 6.20 | 42.4 |
| nav_occlusion_maneuver | reactive | 3/3 | 0 | .671 | 1.300 | 25.80 | 6.39 | 42.0 |
| nav_occlusion_maneuver | **cv** | 3/3 | 0 | **.700** | 1.334 | 26.07 | 6.37 | 41.6 |
| nav_occlusion_maneuver | reach | 3/3 | 0 | .679 | 1.320 | 25.20 | 6.29 | 41.0 |

The intended events did occur: the reversal obstacle crossed the robot's path
under a bounded ramp, and the occlusion produced 4–5 consecutive missed scans
at t ≈ 5.1–5.8 s in all three trials.

**Prediction matters enormously; which prediction does not.** Reactive
collides in 3/3 reversal trials with the bodies actually intersecting
(−0.173 m); both predictive arms are collision-free with ~0.72 m clearance.
Between the predictive arms the differences (0.014–0.035 m clearance,
0.07–0.87 s) are within run-to-run variation at n=3, and **reachability is
marginally worse on minimum clearance in all three scenarios**. No navigation
benefit from reachability was demonstrated.

## 13. Runtime

A/B on **identical scenarios**, identical build and machine; the only
difference is `reachability_enabled` ([runtime_ab.json](stage4g4/runtime_ab.json)).

| tracks | cluster µs | deblend µs | assoc µs | **reach µs** | callback mean ms | callback max ms |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 23.7 → 21.9 | 66.9 → 65.1 | 102.8 → 97.5 | **354.0** | 14.10 → **15.90** | 32.72 → 32.96 |
| 2 | 46.6 → 42.2 | 102.5 → 92.2 | 165.0 → 148.6 | **666.1** | 16.66 → **18.65** | 32.96 → 33.77 |
| 3 | 60.8 → 67.5 | 117.9 → 124.2 | 205.2 → 212.0 | **980.1** | 15.90 → **17.42** | 34.37 → 35.63 |

The unchanged stages do not move. Reachability itself costs 0.35–0.98 ms
(max observed 2.00 ms); the remaining ~1 ms of callback growth is the larger
published message. **Max callback 35.63 ms = 17.8% of the 200 ms scan period**
(Stage-4G3: 34.09 ms, 17.0%). Costmap rasterization is bounded by the
pre-existing `max_influence_radius` policy in both modes.

## 14. Remaining limitations

1. **Step velocity changes are not coverable.** Gazebo's VelocityControl can
   apply |a| of 5–9.5 m/s²; no finite `a_max` bounds that at 0.5 s. The step
   scenarios are reported separately and reachability does not reach 1.000 on
   them.
2. **The comparison has a ceiling.** CV at 2σ already covers 100% from h ≥ 1 s,
   so this evidence cannot discriminate the two representations beyond 0.5 s.
   A weaker process-noise model, or a genuinely harder obstacle, would be
   needed to probe further.
3. **Two-object, n=3 navigation trials** exercise timing variation, not
   population statistics. The reachability-vs-CV navigation differences are
   below that resolution and are reported as "no measurable difference", not
   as a tie.
4. **The bound is isotropic**, so it over-covers laterally for a fast-moving
   obstacle. A longitudinal/lateral split would tighten it; the measured
   evidence did not justify the extra complexity.
5. **The ~0.17 m LiDAR surface-vs-centre bias** consumes most of the
   short-horizon error budget for *both* representations and is why neither
   reaches 1.000 on fresh tracks at 0.5 s.
6. **Coasting statistics rest on 42 samples** from 27 trials; the effect is
   large (.524 vs 1.000) but the sample is small.

## 15. Acceptance criteria

| # | Criterion | Verdict |
|---|---|---|
| 1 | Existing CV prediction reproducible and unchanged | **PASS** — `kalman_filter.hpp` hash unchanged; unit test asserts CV predictions bit-identical with reachability on; Stage-4D layer regression unchanged |
| 2 | Reachability a separate, interpretable model | **PASS** — own header, own message, own costmap mode |
| 3 | Deterministic bounds not mislabelled as covariance | **PASS** — separate named fields; `a_max=0` collapses the region exactly onto the CV ellipse |
| 4 | Uses only runtime track state, never GT | **PASS** — built from `kf_x`/`kf_P`; GT appears only in the offline evaluator |
| 5 | Observation age explicitly affects coasting | **PASS** — `total_time = age + h`; 0.0625 → 0.4225 m at 0.8 s age |
| 6 | Constant-velocity scenario evaluated | **PASS** — §8, and CV reported as the better choice there |
| 7 | Acceleration evaluated | **PASS** — .898 → .980 |
| 8 | Deceleration/stop evaluated | **PASS** — .898 → .959 |
| 9 | Direction reversal evaluated | **PASS** — .898 → 1.000 |
| 10 | Short occlusion evaluated | **PASS** — .935 → .994 as consumed |
| 11 | Occlusion + manoeuvre evaluated | **PASS** — .944 → 1.000 as consumed |
| 12 | Coverage-vs-size tradeoff quantified | **PASS** — §6, §7, including the result unfavourable to reachability |
| 13 | Sensitivity to acceleration bound quantified | **PASS** — a_max 0.25/0.5/1.0, exact offline reproduction |
| 14 | Multi-object independence valid | **PASS** — 3-track scenarios + unit test |
| 15 | Stage-4G3 association/deblending does not regress | **PASS** — `all_identical: true`; all three prior regressions pass |
| 16 | Optional reachability costmap mode works, CV unbroken | **PASS** — §10 |
| 17 | Expiry and clearing correct | **PASS** — 0 ghost cells, stale regions paint 0 |
| 18 | Static-world false predictive occupancy zero | **PASS** — 0 in both modes, 3 runs |
| 19 | Focused navigation comparison performed | **PASS** — §12 |
| 20 | Runtime within budget | **PASS** — max 35.63 ms = 17.8% of 200 ms |
| 21 | `stage4f-validated` unchanged | **PASS** — `68b2bf2…` |
| 22 | `~/ur5e_ws` untouched | **PASS** — not read or written |

## 16. Exact reproduction

Trial directories are exclusive-created, so use a fresh output directory.
Keep `--domain` below 226: the runner adds a per-trial offset and DDS rejects
domain ids above 232.

```bash
cd ~/predictive_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Unit / white-box regressions
ROS_DOMAIN_ID=96 build/predictive_nav_tracking/stage4g4_reachability
ROS_DOMAIN_ID=97 build/predictive_nav_costmap/stage4g4_reach_layer
ROS_DOMAIN_ID=95 build/predictive_nav_tracking/stage4g3_deblend
ROS_DOMAIN_ID=94 build/predictive_nav_tracking/stage4g2_lifecycle
ROS_DOMAIN_ID=93 build/predictive_nav_costmap/stage4g2_multi_layer
python3 -m unittest discover -s src/predictive_nav_bringup/scripts/stage4g2 -p 'test_*.py'

# Prediction-quality matrix (12 trials) + regressions (6) + occlusion (9)
python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
  --out validation_logs/stage4g4_repro/prediction \
  --scenarios cv_control,accelerate,decelerate_stop,reversal,occlusion_short,occlusion_maneuver \
  --trials 2 --domain 200
python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
  --out validation_logs/stage4g4_repro/regress \
  --scenarios reach_triple,reversal_step,accelerate_step,triple,crossing,static_navigation \
  --trials 1 --domain 100
python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
  --out validation_logs/stage4g4_repro/occlusion \
  --scenarios occlusion_short,occlusion_maneuver,occlusion_long \
  --trials 3 --domain 120

# Focused navigation: three arms, one launch configuration
for arm in reactive:140 cv_covariance:150 reachability:160; do
  python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
    --out validation_logs/stage4g4_repro/nav_${arm%%:*} --layer-mode ${arm%%:*} \
    --scenarios nav_cv_control,nav_reversal,nav_occlusion_maneuver \
    --trials 3 --domain ${arm##*:}
done

# Static world under the reachability costmap mode
python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
  --out validation_logs/stage4g4_repro/static_reach --layer-mode reachability \
  --scenarios static_navigation --trials 2 --domain 190

# Runtime A/B: same scenarios with reachability_enabled: false in
# src/predictive_nav_tracking/config/tracker_params.yaml (no rebuild needed
# under --symlink-install), then restore it to true.
python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
  --out validation_logs/stage4g4_repro/perf_off \
  --scenarios cv_control,occlusion_short,triple --trials 1 --domain 180

# Scoring and curation
python3 src/predictive_nav_bringup/scripts/stage4g4/evaluate.py \
  --logs validation_logs/stage4g4_repro/prediction validation_logs/stage4g4_repro/regress \
         validation_logs/stage4g4_repro/occlusion \
  --out validation_logs/stage4g4_repro/prediction_eval.json
for arm in reactive cv_covariance reachability; do
  python3 src/predictive_nav_bringup/scripts/stage4g4/evaluate.py \
    --logs validation_logs/stage4g4_repro/nav_$arm \
    --out validation_logs/stage4g4_repro/nav_${arm}_eval.json
done
python3 src/predictive_nav_bringup/scripts/stage4g4/summarize.py \
  --prediction-eval validation_logs/stage4g4_repro/prediction_eval.json \
  --nav-eval validation_logs/stage4g4_repro/nav_*_eval.json \
  --perf-logs validation_logs/stage4g4_repro/prediction \
  --perf-off-logs validation_logs/stage4g4_repro/perf_off \
  --perf-on-logs validation_logs/stage4g4_repro/prediction/cv_control_01 \
                 validation_logs/stage4g4_repro/prediction/occlusion_short_01 \
                 validation_logs/stage4g4_repro/regress/triple_01 \
  --regression-trials validation_logs/stage4g4_repro/regress/crossing_01 \
                      validation_logs/stage4g4_repro/regress/triple_01 \
  --out-dir validation_logs/stage4g4_repro/curated

# A/B against the Stage-4G3 payload, no rebuild needed:
#   set reachability_enabled: false in tracker_params.yaml
```

## 17. Handoff

Reachability is correct, cheap and the only representation that fully closes
the coasting gap — but on this evidence it does not pay for itself against a CV
Gaussian whose process noise already encodes the same acceleration bound, and
it changed no navigation outcome. Its defensible use is narrow and specific:
**short-horizon decisions about tracks that are not currently observed**, where
a hard bound and explicit age accounting matter more than area efficiency.

**Recommended Stage-4G5 first step: decide whether to keep reachability at
all.** The measured options, in order of evidence support:

1. Use reachability **only** for coasting tracks (`observation_age > 0`) and CV
   elsewhere — this targets the one measured failure (0.524 → 1.000) without
   paying 2× area on the ~97% of frames where CV already covers.
2. Re-examine the Stage-4C process noise instead. If `accel_noise` is doing the
   work of `a_max`, the honest comparison is between *tunings of one model*,
   not two models — and that is a Stage-4C/4D mathematics change needing its
   own scope review.
3. Build a scenario set that actually discriminates beyond 0.5 s. Everything
   here saturates, so no amount of further analysis of this data will separate
   the representations at longer horizons.

Nothing measured here justifies JPDA, MHT, camera fusion, learned trajectory
prediction or intent recognition. No Stage-4G5 work was started.
