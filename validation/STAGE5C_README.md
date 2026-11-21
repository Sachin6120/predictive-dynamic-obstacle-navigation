# Stage-5C — qualification of the EXISTING reachability predictor

**PASS.** The Stage-4G4 bounded-motion reachable-set predictor is qualified as a
genuine second prediction method: independently reconstructed, verified correct
and tight under its own premises, compared against CV-Kalman on identical
inputs, measured for conservativeness and cost, and compared at the planner
level. **No production code, parameter or evidence file was changed** — the only
additions are the three analysis artifacts in `stage5c/` and this document.

A PASS here does **not** mean reachability should become the production default.
Stage-4G7's `SHIP_CV_ONLY` decision stands and is re-confirmed, for a reason this
stage sharpens considerably (§6).

Provenance: HEAD `8a2dbe204c03a37911fddf3026b0a2725d5c372d`, branch `main`,
clean tree at start and finish apart from the new `validation/stage5c/` files.
`reachability.hpp` sha256
`2272aeddf38f1eef52a6159b85cd6b0290a0da56cc93e53ab3c1502dc706cd60`, a single
commit (`b261994`) and never since modified.

---

## 1. What the implementation is, and where it sits

One producer, one production consumer. The complete set, verified by grep over
all non-test sources:

```
LiDAR scan
  → clustering → Stage-4G3 deblending → greedy association
  → per-track CV Kalman filter  (kf_x, kf_P)        [kalman_filter.hpp]
       |
       |-- predict_future()          -> TrackedObject.predictions[]          (Stage-4C, UNCHANGED)
       |      F(h)x, F P Fᵀ + Q(h);  anchored at TrackedObject.stamp
       |
       '-- predict_reachability()    -> TrackedObject.reachability_predictions[]
              lidar_obstacle_tracker_node.cpp:1771-1805
              buildRegion()                                   [reachability.hpp:...]
              anchored at TrackedObjectArray.header.stamp
              total_time = observation_age + horizon
       ↓
  /tracked_objects  (predictive_nav_msgs/TrackedObjectArray)
       ↓
  PredictedObstacleLayer                [predicted_obstacle_layer.cpp]
     prediction_mode: cv_covariance  -> rasterizeTrackCv()            (default, production)
                      reachability   -> rasterizeTrackReachability()  :485-530
                      hybrid         -> per-track age split           (Stage-4G5)
       ↓
  local_costmap (plugin runs LAST, after inflation)
       ↓
  MPPI critics → /cmd_vel
```

Reachability **enters** at `lidar_obstacle_tracker_node.cpp:1893` and **leaves**
at `predicted_obstacle_layer.cpp:490`. Nothing else in production reads the
field; `run.py` and `stage5b/integration_check.py` read it for logging only.

**It is off in production.** `production_tracker_params.yaml:137`
`reachability_enabled: false` and `production_nav2_params.yaml:287`
`prediction_mode: "cv_covariance"`. Both arms of this stage therefore used
`tracker_params.yaml` / `nav2_stage4f_params.yaml`, which leave production
untouched.

## 2. The mathematics, reconstructed from the code

**State.** `p0 = (kf_x(0), kf_x(1))`, `v0 = (kf_x(2), kf_x(3))`, `s0 = |v0|`, the
filtered state at the track's last real observation; `P0 = kf_P` (4×4).
Frame: `map`. Units: m, m/s, m/s², s, rad.

**Time anchor.** `total_time t = observation_age + horizon`, where
`observation_age = scan_stamp − track.last_update`. Horizons
`h = 0.5 … 3.0 s` in 6 steps of 0.5 s (`reachability_horizon` /
`reachability_time_step`). Max `t` = `max_observation_age` + horizon = 1.0 + 3.0
= **4.0 s**.

**Nominal centre** — the same constant-velocity nominal the CV predictor uses:

```
p(t) = p0 + v0 · t
```

**Deterministic term.** For any trajectory with `|a| ≤ a_max`, `|v| ≤ v_max`:

```
d(t) := p(t) − (p0 + v0 t) = ∫₀ᵗ (v(s) − v0) ds
|v(s) − v0| ≤ a_max · s                      (bounded acceleration)
|v(s) − v0| ≤ |v(s)| + |v0| ≤ v_max + s0     (bounded speed; reversal ⇒ SUM)

t_c = (v_max + s0) / a_max

                ⎧ ½ a_max t²                                 t ≤ t_c
r_det(t) = |d| ≤ ⎨
                ⎩ ½ a_max t_c² + (v_max + s0)(t − t_c)       t > t_c
```

Isotropic — it bounds the deviation *magnitude* and ignores the direction/speed
coupling, so the disc of radius `r_det` about the CV nominal is a strict
**superset** of the true reachable set.

**Statistical term**, kept separate and never written into a covariance field:
`Pp = F(t) P0 F(t)ᵀ + Q_CWNA(t, 0.5)`; eigen-decompose its 2×2 position block,

```
sigma_semi_major = k·√λ_max ,  sigma_semi_minor = k·√λ_min ,  k = covariance_sigma_level = 2.0
sigma_yaw        = atan2 of the major eigenvector
```

**Published region** — Minkowski *outer* bound of ellipse ⊕ disc, plus margin:

```
semi_major = k√λ_max + r_det(t) + safety_margin
semi_minor = k√λ_min + r_det(t) + safety_margin      (safety_margin = 0.0 by default)
```

Shape: **ellipse** (axis-aligned to the covariance eigenvectors, inflated
isotropically). Membership: `regionContains()`, shared by tracker tests, the
costmap layer and the offline evaluator.

**Frozen bounds:** `a_max = 0.5 m/s²`, `v_max = 0.8 m/s`, `k = 2.0`,
`safety_margin = 0.0 m`, `max_observation_age = 1.0 s`.

### 2.1 Independent verification — `stage5c/model_audit.json`

`stage5c_audit.cpp` includes the shipped headers, so these describe the shipped
mathematics, not a re-implementation.

| check | result |
|---|---|
| closed form vs. numerical ∫₀ᵗ min(a·s, v_max+s0) ds | max abs error **2.1 × 10⁻¹¹ m** |
| continuity at `t_c` (= 2.2 s at s0 = 0.3) | jump 2.2 × 10⁻⁷ m |
| C¹ at `t_c` | slope below 1.100, above 1.100 m/s — matches `v_max + s0` |
| monotone in `t`, in `a_max`, in `s0` | **true / true / true** |
| dimensional consistency | `[m/s²]·[s²]` and `[m/s]·[s]` ⇒ m ✔ |
| `a_max = 0` collapses onto the CV ellipse | error **0.0** (exactly) |

Edge cases, all guarded and all failing *closed* rather than open:

| input | `r_det` | behaviour |
|---|---|---|
| `t = 0`, `t < 0`, `t = NaN`, `t = ∞` | 0.0 | returns 0, no region |
| `a_max = 0` or `a_max < 0` | 0.0 | region collapses to the CV ellipse |
| `s0 < 0` (unreachable; `hypot`) | clamped to 0 | 0.96 m at t = 2 s |
| `t = 10⁶ s` | 1.1 × 10⁶ m | grows linearly, unbounded — bounded in use by `max_observation_age` and the 3 s horizon |
| NaN position / negative `total_time` / NaN covariance | — | `valid = false` ✔ |
| `v_max < 0` | 0.51 m at t = 2 s | **silently clamped to 0**, i.e. a *less* conservative region, with no warning |
| negative-definite covariance | `valid = true`, semi_major = `r_det` | eigenvalues clamped to 0; falls back to the deterministic term alone, silently |

The last two are robustness observations, not defects: both are nonsensical
inputs that cannot arise from the tracker (`v_max` comes from config, the
covariance from a Kalman update), and both fail toward the deterministic bound.

### 2.2 Adversarial containment

20,000 randomized admissible trajectories per regime, midpoint integrator at
`dt = 10⁻⁵`, deviation compared against `reachRadius()`:

| regime | trials | escapes | worst slack |
|---|---:|---:|---:|
| **within premises** (`s0 ≤ v_max`, speed capped at `v_max`) | 20,000 | **0** | **−3.5 × 10⁻¹² m** (numerical) |

The bound is **valid and exactly tight** at its operating point. This
independently reproduces Stage-4G7's claim of "3,000 adversarial trajectories
contained, worst slack 0.000 m" at ~7× the sample count.

## 3. The one model-validity boundary — `stage5c/speed_cap_probe.json`

`r_det` is valid **only while the physical obstacle stays under `max_speed`.**
The `|v(s) − v0| ≤ v_max + s0` premise is what produces the linear branch beyond
`t_c`; if the obstacle can actually exceed `v_max`, that branch under-covers.
Nothing in `reachability.hpp` or the tracker compares `max_speed` against the
track's own measured speed — it is a configured constant, used unconditionally.

Worst escape of an admissible trajectory, obstacle currently at `s0` but
*capable* of `v_cap`, `a_max = 0.5`, `v_max = 0.8`:

| `s0` | `v_cap` | t = 2 s | t = 3 s | t = 4 s |
|---:|---:|---:|---:|---:|
| any | **0.80** (= configured) | 0.000 | 0.000 | 0.000 |
| 0.00 | 1.40 (pedestrian) | 0.040 | 0.480 | 1.080 |
| 0.30 | 1.40 (pedestrian) | 0.000 | 0.160 | 0.720 |
| 0.75 | 1.40 | 0.000 | 0.000 | 0.202 |
| 0.00 | 4.20 (cyclist, 15 km/h) | 0.040 | 0.490 | 1.440 |

`t = 4.0 s` is reachable in production (`max_observation_age` 1.0 + horizon 3.0).

**Not material to the current baseline:** every commanded obstacle speed in the
repository is ≤ 0.75 m/s (`stage4g2/scenarios.json` max 0.68; Stage-5B speeds
0.25/0.50/0.75), so the premise holds and the measured escape is 0.000. It is
**material to any real-world or class-aware use**, where 0.8 m/s is 1.5–5×
below real pedestrian and bicycle speeds. See §8.

A **separate and benign** effect, measured and distinguished: an *over-estimated*
filtered speed only enlarges the bound, because `r_det` is monotone increasing
in `s0` (verified). Estimation overshoot is conservative, not a violation — the
filtered speed reached 0.93 m/s on a 0.68 m/s obstacle in 0.4% of frames with no
loss of containment.

## 4. Existing tests — all functional tests pass

`colcon test --executor sequential`, ROS_DOMAIN_ID 77:

| functional test | result |
|---|---|
| `stage4g4_reachability` | **passed** |
| `stage4g4_reach_layer` | **passed** |
| `stage4g5_hybrid_layer` | **passed** |
| `stage4g2_lifecycle`, `stage4g3_deblend`, `stage4g2_multi_layer` | passed |
| `stage5b_classification`, `stage5b_tracker_integration` | passed |
| `stage5b1_compound`, `stage5b1_compound_integration` | passed |

**10 / 10 functional tests pass.** `stage4g4_reachability` reports
`containment_trials: 3000, worst_containment_slack_m: −0.000000` (the documented
1 × 10⁻⁸ integrator tolerance), `cv_prediction_bit_identical: true`,
`a_max_zero_collapses_to_cv_ellipse: true`.

**Lint/style: 6 suite failures** — tracking flake8 / pep257 / uncrustify,
bringup flake8 / pep257, costmap uncrustify. Every failing file is a
pre-existing tracked file; `git diff --stat` is empty, and no `stage5c` file
appears in any lint result. Note `reachability.hpp` itself **passes**
uncrustify. The Stage-5B.1 closeout recorded **7**; `predictive_nav_msgs`
xmllint now reports 0 failures, so the baseline is one *better* than recorded,
not worse — and no tracked file was touched to achieve it.

### One gap in the existing test coverage

`stage4g4_reachability` samples the adversary's start speed from
`uniform(0.0, bounds.max_speed)` only, so it never exercises the regime of §3.
That is why the boundary went unrecorded, not a defect in the test's own claim.

## 5. Identical-input CV vs reachability — `stage5c/model_audit.json`

**How identity is guaranteed:** one `ConstantVelocityKalmanFilter2D` instance is
driven by the measurement stream; at each scan the *same* `(kf_x, kf_P)` pair is
handed to both predictors in the same loop iteration. Identity is structural,
not a replay-bookkeeping claim. 11 motion families × 40 seeds, 5 Hz scans,
Gaussian measurement noise σ = 0.02 m, `max_observation_age` enforced.

Scored against ground truth **at the absolute time each representation claims**.
`cv_consumed` scores the same CV region against GT at `scan + h` — what a
consumer that reads the message as describing *now* actually gets.

Pooled, all scenarios (`n` = region samples):

| h (s) | regime | n | CV own anchor | CV as consumed | **Reach** | CV area m² | Reach area m² | ratio |
|---:|---|---:|---:|---:|---:|---:|---:|---:|
| 0.5 | fresh | 19640 | .934 | .934 | **.987** | 0.194 | 0.301 | 1.55 |
| 0.5 | **coasting** | 320 | 1.000 | **.731** | **1.000** | 0.164 | 2.794 | 17.02 |
| 1.0 | fresh | 19640 | .950 | .950 | 1.000 | 1.216 | 2.378 | 1.96 |
| 1.0 | coasting | 320 | 1.000 | 1.000 | 1.000 | 1.110 | 11.120 | 10.02 |
| 1.5 | fresh | 19640 | 1.000 | 1.000 | 1.000 | 4.854 | 10.220 | 2.11 |
| 2.0 | fresh | 19640 | 1.000 | 1.000 | 1.000 | 14.052 | 30.427 | 2.17 |
| 3.0 | fresh | 19640 | 1.000 | 1.000 | 1.000 | 66.802 | 144.185 | 2.16 |

**The ceiling is real and it is the dominant limitation.** Both representations
saturate at 1.000 from h ≥ 1.0 s, because Stage-4C's process noise is a CWNA
model with `accel_noise = 0.5 m/s²` — the *same* physical acceleration
assumption expressed probabilistically. Beyond 0.5 s this evidence cannot
discriminate the two. This reproduces Stage-4G4 §6 independently.

Per scenario at h = 0.5 s, the two cases that discriminate:

| scenario | family | in model | CV | **Reach** |
|---|---|---|---:|---:|
| `A_constant_velocity`, `A_…_fast`, `B_crossing`, `C_decelerate_stop`, `C_accelerate`, `D_turn_in_model`, `F_close_pass` | A/B/C/D/F | yes | 1.000 | 1.000 |
| `C_reversal` | C | yes | .992 | 1.000 |
| **`D_turn_out_of_model`** (ω = 1.5 rad/s, \|a\| = 0.75 > a_max) | D | **no** | **.179** | **.841** |
| **`H_occlusion_short`** (coasting, as consumed) | F | yes | **.463** | **1.000** |

### The new finding: curvature, on *fresh* tracks

Stage-4G4/4G5/4G7 located reachability's entire advantage in the **coasting**
regime. `D_turn_out_of_model` is a **fresh**-track case — no observation age at
all — where CV covers **.179** and reachability **.841** at h = 0.5 s. CV's 2σ
ellipse is elongated along velocity and thin across it, and a turn produces a
systematic *lateral* error (≈ ½·v·ω·t² = 0.094 m at 0.5 s) that lands just
outside the minor axis; `r_det(0.5) = 0.0625 m` of isotropic inflation is enough
to recover most of it.

This matters because the Gazebo scenario suite cannot produce it: its driver
ramps **straight-line** velocity only (`accel` field), so every historical
in-model manoeuvre was collinear. The one regime where reachability helps a
*fresh* track was therefore never testable in simulation, and is reported here
from the offline harness only. The in-model turn (`D_turn_in_model`,
|a| = 0.30) is covered 1.000 by both.

## 6. Conservativeness, and why it inverts at the planner interface

This is the central quantitative result of Stage-5C.

**At the model level**, reachability is substantially more conservative —
per-track totals over all 6 published horizons:

| | CV area m² | Reach area m² | ratio |
|---|---:|---:|---:|
| fresh | 120.05 | 259.40 | **2.16×** |
| coasting | 117.75 | 532.55 | **4.52×** |

**At the costmap interface it is the opposite**, because the two modes apply
`max_influence_radius` (0.55 m, production and Stage-4F alike) differently:

- **CV** (`rasterizeEllipse`) clamps the **bounding box** but tests Mahalanobis
  against the **unclamped** covariance. For a large ellipse the box lies wholly
  inside it, so the painted region degenerates to a filled square:
  `(2·0.55/0.05 + 1)² = **529** cells`.
- **Reachability** (`rasterizeReachableSet`) clamps the **semi-axes** and tests
  the **clamped** ellipse, painting the inscribed disc:
  `π·(0.55/0.05)² ≈ **380** cells`.

`r_det` alone reaches 0.55 m at t = √(0.55/0.25) = **1.483 s**, so from h = 1.5 s
every sample is clamped in both modes. Measured painted cells per sample:

| h (s) | CV cells | Reach cells | ratio |
|---:|---:|---:|---:|
| 0.5 | 43.7 | 111.7 | 2.55 |
| 1.0 | 253.8 | 377.0 | 1.49 |
| ≥ 1.5 | **529.0** | **377.0** | **0.71** |
| **per-track total, fresh** | **2413.5** | **1996.7** | **0.83** |

**Confirmed in simulation**, independently of the offline model — the layer's own
counter across 18 trials:

| arm | `cells_written_last` mean | distinct values observed |
|---|---:|---|
| `cv_covariance` | **633.3** | 20, 24, 26, 58, 209, 213, 249, **529** |
| `reachability` | **505.4** | 0, 6, 126, 133, 134, **378, 380, 381** |

505.4 / 633.3 = **0.80×**, against the offline prediction of 0.83×, and the
predicted saturation values 529 and ≈380 appear verbatim in the logs.

Note this comparison already *favours* reachability on the sigma level: the
published region bakes in the tracker's `covariance_sigma_level = 2.0`, while CV
mode applies the layer's own `sigma_level = 1.5`. Reachability gets the larger
`k` **and** `r_det`, and still paints 20% fewer cells.

**Consequences.** Two previously unexplained historical results follow directly:

1. Stage-4G7 §6's broken causal chain — "costmap cells during the gap differ by
   **−64**, CI [−253, +127]" — hybrid painting *fewer* cells is not an anomaly;
   it is the square-versus-disc clamp asymmetry.
2. Pure reachability was marginally **worse** on minimum clearance in every
   historical navigation scenario (§7). A representation that paints 20% fewer
   cells producing slightly less clearance is the expected direction.

So the diagnosis "reachability is too conservative for production" is **not what
the measurements say**. Its extra conservativeness is real but lives at the
model level and is destroyed by the layer's clamp; at the planner interface it
is *less* conservative than CV for h ≥ 1.5 s.

## 7. Planner-level comparison — `stage5c/planner_comparison.json`

18 trials, 3 scenarios × 2 arms × 3 trials, both arms launched **concurrently**
so they see identical machine contention. One Gazebo launch per trial; the arms
differ **only** in the live `predicted_obstacle_layer.prediction_mode`
parameter. Map, start, goal, obstacle motion, tracker, Kalman filter, CV
predictor, reachability mathematics, Nav2 configuration, MPPI parameters, sensor
model and costmap resolution are identical. The tracker publishes reachability
in **both** arms, so its input to the costmap is byte-identical and only
consumption differs. 0 setup failures.

| | CV-Kalman | Reachability |
|---|---:|---:|
| Navigation success | **9 / 9** | **9 / 9** |
| Collisions | **0** | **0** |
| Near misses (< 0.10 m) | 0 | 0 |
| Min clearance `nav_cv_control` | **0.8240 m** | 0.7823 m |
| Min clearance `nav_reversal` | **0.7347 m** | 0.7041 m |
| Min clearance `nav_occlusion_maneuver` | 0.6619 m | **0.6685 m** |
| Nav time (mean of 3 scenarios) | 24.11 s | 24.18 s |
| Path length (mean) | 6.152 m | 6.140 m |
| Predictive costmap cells (mean) | 633.3 | **505.4** |
| Layer `updateCosts` mean | 22.16 µs | 23.00 µs |
| Tracker reachability compute | 6.10 µs (1 track) / 8.29 µs (2) | 6.10 / 8.43 µs |

Reachability is worse on minimum clearance in 2 of 3 scenarios and better in 1,
every difference within |Δ| ≤ 0.042 m — far below the 0.10 m threshold
Stage-4G7 pre-registered, and inside the n = 3 spread. This reproduces the
direction of Stage-4G4 §12 (reach worse in 3/3) and Stage-4G5 §7 (reach worse in
4/4). Pooled across all three stages, pure reachability is worse on minimum
clearance in **9 of 12** scenario comparisons and never better by more than
0.007 m. **No navigation benefit is demonstrated, and none is claimed.**

### A discrepancy in the historical runtime evidence

Stage-4G4 `runtime_ab.json` records `reach_us` mean **354.0 µs** (1 track) and
**666.1 µs** (2 tracks); Stage-4G8 cites "the 0.35–0.98 ms/scan that computing
and publishing an unread field would cost" as its reason for setting
`reachability_enabled: false`.

Measured here with the **same counter, same unmodified code, same Release
`-O3 -DNDEBUG` build**, conditioned on track count exactly as the historical
table was: **6.10 µs** (1 track), **8.29–8.43 µs** (2 tracks) — **58–79× lower**.
`scan_us` (13.4 ms) and `callback_us` (13.6 ms) match the historical 13–14 ms,
so the measurement regime is otherwise comparable. The offline harness
independently puts the pure mathematics at 296.5 ns per sample, i.e. **1.78 µs**
per track for 6 samples, which agrees with 6.10 µs including loop and message
overhead and not with 354 µs.

Scope of this finding, stated carefully: I did **not** reproduce Stage-4G4's
A/B protocol (dedicated per-track-count scenarios with `reachability_enabled`
toggled; here it was on in both arms), and heavy parallel contention is a
plausible cause of the historical figure — Stage-4G7 records one trial reaching
103 ms under four-way contention. So this is indicative, not a refutation.
What it does mean: **Stage-4G8's cost rationale for disabling publication is not
reproducible at ~1/60th of the cited cost, and should not be relied on.** It
does not disturb the CV-only *selection*, which rests on gates A/B/C and which
Stage-4G7 §9 is explicit about — "cost is not, and never was, the reason to drop
reachability."

## 8. Behaviour classification by scenario

| scenario | classification |
|---|---|
| `A_constant_velocity`, `A_…_fast`, `B_crossing`, `F_close_pass` | **EQUIVALENT TO CV** — both 1.000 at every horizon; reachability buys area and nothing else |
| `C_decelerate_stop`, `C_accelerate`, `D_turn_in_model` | **EQUIVALENT TO CV** — ramped in-model manoeuvres are already inside CV's 2σ |
| `C_reversal` | **USEFUL CONSERVATIVE**, marginally — .992 → 1.000 at h = 0.5 |
| **`H_occlusion_short` (coasting)** | **USEFUL CONSERVATIVE** — .463 → 1.000 as consumed; the Stage-4G4/4G7 result, reproduced |
| **`D_turn_out_of_model` (fresh)** | **USEFUL CONSERVATIVE** — .179 → .841; new at Stage-5C, untestable in the Gazebo suite |
| all three navigation scenarios | **UNDER-CONSERVATIVE at the planner** — 0.80× CV's painted cells, and marginally lower clearance in 2/3 |
| horizons h ≥ 1.0 s | **EQUIVALENT TO CV** by ceiling — both 1.000, so unmeasurable |
| cost | **NOT expensive** — 6.1–8.4 µs/scan, 0.84 µs of extra layer time |

Costmaps and per-sample geometry were inspected directly (§6), not inferred from
aggregates: the predicted 529 and ≈380 cell saturation values were confirmed in
the layer's own logs.

## 9. Limitations

1. **The ceiling dominates.** CV at 2σ covers 1.000 from h ≥ 1.0 s, so no
   evidence here or in Stage-4G4 can discriminate the representations beyond
   0.5 s. A weaker process-noise model would be needed.
2. **`D_turn_out_of_model` is offline only.** The Gazebo driver ramps
   straight-line velocity, so curvature cannot be commanded honestly. The
   largest fresh-track advantage found is therefore unvalidated in simulation.
3. **The offline harness omits the ~0.17 m LiDAR surface-vs-centre bias**, which
   Stage-4G4 identifies as consuming most of the short-horizon error budget for
   *both* representations. Absolute containment here is accordingly optimistic;
   the comparison is unaffected since both share the omission.
4. **n = 3 per scenario-arm** at the planner level exercises timing variation,
   not population statistics. The 0.042 m differences are below that resolution
   and are reported as "no measurable difference", not as a tie.
5. **Coasting samples are few** (320 region samples; 42 in Stage-4G4). The
   effect is large and consistent across four stages, but the sample is small.
6. **Step velocity changes remain uncoverable** by any finite `a_max`, as
   Stage-4G4 documented.
7. **The bound is isotropic**, so it over-covers laterally for a fast obstacle.
   A longitudinal/lateral split would tighten it.
8. **The `max_speed` premise has no runtime guard** (§3) — the single concrete
   gap found, and a prerequisite for Stage-5D.

## 10. Decision

### `QUALIFIED_SECOND_PREDICTOR`

- **Correct**: independently reconstructed to 2 × 10⁻¹¹ m, C¹, monotone,
  dimensionally consistent, edge cases guarded, and **exactly tight** with 0
  escapes in 20,000 adversarial trials inside its premises.
- **Genuinely complementary**: it is the only representation reaching 1.000 on
  coasting ground truth (.463/.731 → 1.000, reproducing .524 → 1.000 at 4G4 and
  .207 → 1.000 at 4G7), and it recovers a fresh-track curvature case where CV
  collapses to .179.
- **Cheap**: 6.1–8.4 µs/scan measured; 0.84 µs of extra layer time.
- **Architecturally clean**: separate header, message, costmap mode; deterministic
  and statistical terms never conflated; `a_max = 0` collapses it exactly onto
  the CV ellipse; CV output bit-identical whether or not it is enabled.

**It must not become the production default.** Stage-4G7's `SHIP_CV_ONLY` stands
on 282 trials against a pre-registered, hash-verified rule, and this stage adds
18 more trials in the same direction.

**Why not `QUALIFIED_BUT_TOO_CONSERVATIVE_FOR_DEFAULT`:** that label's stated
reason is contradicted by measurement. At the planner interface reachability
paints **0.80×** CV's cells, not more (§6); it is *under*-conservative where the
planner can see it. The correct reason it is not the default is the absence of
any navigation benefit, in four independent stages and three domains.

**On `NEEDS_CORRECTION`:** a reviewer who treats the missing `max_speed`
consistency guard (§3) as a defect rather than a documented premise would
classify C. I classify A because the header states the premise explicitly, the
parameter is documented as a physical capability, and the bound is exactly tight
at the configured operating point — so the algorithm is correct as specified and
the gap is a missing runtime guard. **Nothing was repaired**, so either label
leaves the code identical and the audit trail complete.

### Recommended, NOT performed in Stage-5C

1. Add a runtime guard comparing each track's filtered speed against
   `max_speed`, invalidating or widening the region when the premise fails, and
   extend `stage4g4_reachability` to sample `s0 > max_speed` (§3, §4).
2. Re-measure the Stage-4G4 runtime A/B before any future stage cites the
   0.35–0.98 ms figure (§7).
3. Consider whether the `max_influence_radius` square-vs-disc asymmetry (§6) is
   intended, since it silently inverts the relative conservativeness of the two
   modes.

These are **Stage-5D or later**. Class-aware bounds are exactly where a
per-class `max_speed` belongs, and §3 quantifies why that is a prerequisite
rather than an enhancement.

## 11. Reproduction

```bash
cd ~/predictive_nav_ws
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
source install/setup.bash
OUT=/tmp/stage5c

# Offline model audit + identical-input comparison (~70 s)
g++ -O2 -std=c++17 -I/usr/include/eigen3 -Isrc/predictive_nav_tracking/include \
    validation/stage5c/stage5c_audit.cpp -o $OUT/stage5c_audit && $OUT/stage5c_audit
g++ -O2 -std=c++17 -I/usr/include/eigen3 -Isrc/predictive_nav_tracking/include \
    validation/stage5c/stage5c_speedcap.cpp -o $OUT/stage5c_speedcap && $OUT/stage5c_speedcap

# Existing functional tests
ROS_DOMAIN_ID=96 build/predictive_nav_tracking/stage4g4_reachability
ROS_DOMAIN_ID=97 build/predictive_nav_costmap/stage4g4_reach_layer
ROS_DOMAIN_ID=98 build/predictive_nav_costmap/stage4g5_hybrid_layer

# Bounded planner comparison, arms concurrent (18 trials)
SC=nav_cv_control,nav_reversal,nav_occlusion_maneuver
python3 src/predictive_nav_bringup/scripts/stage4g2/run.py --out $OUT/cv \
    --scenarios $SC --trials 3 --domain 140 --layer-mode cv_covariance &
python3 src/predictive_nav_bringup/scripts/stage4g2/run.py --out $OUT/reach \
    --scenarios $SC --trials 3 --domain 180 --layer-mode reachability &
wait
for a in cv reach; do python3 src/predictive_nav_bringup/scripts/stage4g4/evaluate.py \
    --logs $(ls -d $OUT/$a/*/) --out $OUT/eval_$a.json; done
```

## 12. Files added

| file | role |
|---|---|
| `validation/STAGE5C_README.md` | this document |
| `validation/stage5c/stage5c_audit.cpp` | model reconstruction, adversarial containment, identical-input comparison, cost |
| `validation/stage5c/stage5c_speedcap.cpp` | `max_speed` premise probe |
| `validation/stage5c/model_audit.json` | output of the first |
| `validation/stage5c/speed_cap_probe.json` | output of the second |
| `validation/stage5c/planner_comparison.json` | 18-trial planner-level results |

No production source, parameter or prior evidence file was modified.
`git diff --stat` is empty.
