# Stage-5C.1 — Reachability contract & costmap fidelity

Closes the two prerequisites Stage-5C identified before class-aware
reachability can be attempted. **No class-specific bound is introduced here**,
and the production default remains CV-Kalman.

Baseline: `ff23d9febc94223ed2b9f8e3559cdac584a1bae9` (`main`, clean, in sync
with `origin/main`).

The pre-change audit required by sections 1, 6 and 7 of the brief is in
[`CONTRACT_AND_RASTER_AUDIT.md`](CONTRACT_AND_RASTER_AUDIT.md) and was written
before any file in `src/` was touched.

---

## A. The reachability validity contract

### A.1 What `max_speed` is

`max_speed` is `v_max` in the `reachability.hpp` derivation. It enters at one
place only, `delta_v_cap = v_max + s0`, where it bounds the deviation
integrand via `|v(s) - v0| <= |v(s)| + |v0| <= v_max + s0`. The quantity
actually assumed is therefore

> **(P2)  |v(s)| <= max_speed  for every s in [0, total_time]**

— a statement about the obstacle's **future speed capability**, not about its
observed velocity, and not a tuning weight. Configured value **0.8 m/s**;
maximum total_time **4.0 s** (`max_observation_age` 1.0 + `reachability_horizon`
3.0).

### A.2 The observable invalidity condition

(P2) quantifies over the future, so **no runtime check can verify it**, and
none is claimed. Exactly one direction is observable:

```
s0 > max_speed  =>  the obstacle is demonstrably moving at s0
                =>  it is demonstrably CAPABLE of s0
                =>  v_phys >= s0 > max_speed
                =>  (P2) is FALSE
```

An observed filtered speed above the cap is a **proof** that the premise is
broken. That is the condition the runtime enforces. The converse —
`v_phys > max_speed` while `s0 <= max_speed`, Stage-5C's `s0 = 0`,
`v_phys = 1.4 m/s` case — is **not observable** and remains a declared
configuration assumption, stated in the message, in the node and in all three
tracker YAMLs.

### A.3 The runtime response: weakest observation-consistent premise

A disproved premise is **neither ignored nor used to drop the sample**.
Dropping would *remove* coverage, which is the wrong direction for a safety
envelope. Instead the disproved premise is replaced by the weakest one
observation does not contradict:

```
v_eff = max(max_speed, s0)
```

restoring a valid bound under `(P2') |v(s)| <= v_eff`. Because
`v_eff >= max_speed` always, the response is **monotone-safe**: it can only
enlarge the region.

That property is what lets the dead-band stay at numeric scale. The Stage-5C
recordings show the Kalman filtered speed overshooting ground truth by up to
0.25 m/s and exceeding the 0.8 m/s cap by up to **0.131 m/s on tracks that are
genuinely in domain** (`A_constant_velocity_fast`: GT 0.680, filtered peak
0.931). A rejection would fire on those frames; a dead-band wide enough to
absorb them (>= 0.15 m/s) would mask the `s0 in [0.80, 0.95]` band where
Stage-5C's adversary already measured escapes (3334/20000 trials, worst
0.155 m). Enlarging the region instead costs nothing and removes nothing.

### A.4 One dead-band, both paths

`max_speed_tolerance` (default **1e-6 m/s**) gates the reported status **and**
the effective cap together, so the two can never disagree. This makes an exact
theorem rather than an approximation:

```
contract_status == CONTRACT_SATISFIED
  <=> effective_max_speed is bit-identical to configured_max_speed
  <=> every published number is bit-identical to Stage-5C
```

The concession is explicit and bounded: an excursion inside the dead-band is
treated as numerically equal to the cap, i.e. at most
`1e-6 m/s * 4.0 s = 4e-6 m` of under-coverage across the whole window.

### A.5 Interface

`ReachabilityPrediction.msg` gains four additive fields:

| field | meaning |
|---|---|
| `observed_speed` | `\|v0\|` at the last real observation — what the contract is evaluated against |
| `configured_max_speed` | the configured premise, verbatim |
| `effective_max_speed` | the cap `reach_radius` was actually computed with |
| `contract_status` | `CONTRACT_SATISFIED` / `CONTRACT_OBSERVED_SPEED_EXCEEDS_CAP` / `CONTRACT_NOT_EVALUATED` |

`contract_status` is the **single** encoding of "is this a qualified
conservative bound". A duplicate `model_assumption_satisfied` boolean was
deliberately not added: two encodings of one fact can disagree.

`valid` keeps its existing meaning. A contract breach does **not** set
`valid = false`, because consumers drop invalid samples and that would remove
coverage. `speed_capped` now refers to the effective cap; its comment was
corrected accordingly.

Both Python consumers (`stage4g2/run.py`, `stage5b/integration_check.py`) read
only pre-existing fields and never enumerate fields, so the additions are
source-compatible.

---

## B. Costmap representation fidelity

### B.1 Root cause

`predicted_obstacle_layer.cpp:538-541` (pre-fix) clamped the **geometry**:

```cpp
if (max_influence_radius_ > 0.0) {
  semi_major = std::min(semi_major, max_influence_radius_);
  semi_minor = std::min(semi_minor, max_influence_radius_);
}
```

`semi_major`/`semi_minor` are by-value parameters and were **reassigned**, so
the membership test at `:574-575`, which divides by those same variables, tested
against a **shrunken ellipse**. CV applies the identical policy bound to the
bounding **box** only (`:635-638`) and tests membership against the unclamped
covariance (`:657-659`), so the two modes treated the same bound at different
levels.

### B.2 The fidelity invariant

With cell `(i,j)` centred at `origin + (index + 0.5) * resolution`,
`M` the representable extent, `E` the ellipse **as published**, and `W(E)` the
policy window (bounding half-extents each clamped to `max_influence_radius`):

> **Invariant F.** Every cell in `M` whose centre lies in `E ∩ W(E)` must be
> written. No cell may be excluded *because the semi-axes of `E` were reduced*.
>
> **Invariant S.** Only cells in `M` are written, and no cell whose centre lies
> outside `E`.

| clipping | status |
|---|---|
| iteration window clipped to the map `M` | legitimate — kept |
| iteration window clipped to the policy box | legitimate — kept, and now applied at the same level in both modes |
| reducing the semi-axes and testing against the reduced values | **the defect — removed** |

### B.3 The fix

Clamp `half_x`/`half_y` (the window) instead of the axes, and keep testing
membership against the published semi-axes. Where the bound does not bind this
is a no-op. Work stays bounded by the same `(2R/res)^2` cell budget as before.
`rasterizeEllipse` (the CV path) is **untouched**.

### B.4 Anomaly reproduced, then resolved

At production geometry (`max_influence_radius` 0.55 m, `resolution` 0.05 m) with
semi-axes far above the bound — every horizon from 1.5 s on:

| quantity | value |
|---|---|
| policy window side | 23 cells |
| policy window | **529** cells |
| inscribed disc (pre-fix behaviour) | **377** cells |
| ratio | **0.7127** |
| Stage-5C recorded `cv_cells` | **529.0** |
| Stage-5C recorded `reach_cells` | **377.0** |
| Stage-5C recorded `cell_ratio` | **0.713** |
| **after the fix** | **529** cells |

The independently derived figures match `validation/stage5c/model_audit.json`
**exactly**, confirming the mechanism: reachability was being rasterized as the
disc *inscribed in* the window CV fills completely, and `0.713 ~ pi/4` is the
area ratio of a disc to its circumscribed square. The strict count is 373; the
4-cell difference is the axis-aligned cells at distance exactly 0.55 m, a
floating-point boundary tie that the test reports rather than asserts.

---

## C. Files changed

| file | change |
|---|---|
| `src/predictive_nav_tracking/include/predictive_nav_tracking/reachability.hpp` | contract derivation, `ContractStatus`, `capabilityPremiseContradicted()`, `effectiveMaxSpeed()`, `contractStatus()`, bounds + region metadata, `reachRadius` uses the effective cap |
| `src/predictive_nav_msgs/msg/ReachabilityPrediction.msg` | 4 additive fields + 3 constants; `speed_capped` comment corrected |
| `src/predictive_nav_tracking/src/lidar_obstacle_tracker_node.cpp` | 2 parameters, contract metadata published, throttled breach warning, cumulative breach counters in the profile line |
| `src/predictive_nav_costmap/src/predicted_obstacle_layer.cpp` | **the fidelity fix**: policy bound moved from the semi-axes to the iteration window |
| `src/predictive_nav_tracking/config/tracker_params.yaml` | premise documented; 2 params at their code defaults |
| `src/predictive_nav_tracking/config/production_tracker_params.yaml` | same (`reachability_enabled` stays `false`) |
| `src/predictive_nav_tracking/config/stage5b_classification_tracker.yaml` | same |
| `src/predictive_nav_tracking/test/stage5c1_contract.cpp` | **new** |
| `src/predictive_nav_costmap/test/stage5c1_raster_fidelity.cpp` | **new** |
| `src/predictive_nav_tracking/CMakeLists.txt` | register `stage5c1_contract` |
| `src/predictive_nav_costmap/CMakeLists.txt` | register `stage5c1_raster_fidelity` |

The YAML additions set the two new parameters to the values the node already
defaults to, so they are behaviourally inert and exist to make the premise and
the response explicit in configuration.

---

## D. Bounded planner regression

18 Gazebo trials: 3 conditions x 3 trials x 2 arms, one launch per trial. The
arms differ **only** in `predicted_obstacle_layer.prediction_mode`; the two
params files are otherwise byte-identical (verified by diff). Map, start, goal,
obstacle trajectories, tracker, Kalman filter, CV predictor, reachability
mathematics, MPPI, Nav2 configuration, sensor model, classifier, compound
clustering and costmap resolution are shared, and the tracker publishes
reachability in **both** arms (`tracker_params.yaml` `reachability_enabled:
true`), so the costmap's input is identical and only consumption differs.

Full data: [`planner_regression.json`](planner_regression.json).

### D.1 Aggregate

| metric | CV (production default) | reachability (corrected) |
|---|---|---|
| layer mode observed | `cv_covariance` | `reachability` |
| trials | 9 | 9 |
| **succeeded** | **9 / 9** | **9 / 9** |
| **collisions** | **0** | **0** |
| collision-monitor interventions | 0 | 0 |
| min clearance, mean (m) | 0.6688 | 0.6872 |
| min clearance, worst single trial (m) | 0.0169 | 0.0864 |
| nav time, mean (s) | 16.574 | 16.053 |
| path length, mean (m) | 5.8745 | 5.8431 |
| **planner-visible predictive cells, mean peak** | **934.9** | **1037.9** |
| layer `updateCosts` mean (us) | 23.23 | 25.67 |
| layer `updateCosts` max (us) | 113.0 | 93.0 |
| reachability contract warnings | 0 | 0 |

### D.2 Per condition

| condition | arm | ok | collisions | clr min | clr mean | nav mean | path mean | cells mean |
|---|---|---|---|---|---|---|---|---|
| perp_050 | CV | 3 | 0 | 0.7832 | 0.7887 | 18.451 | 5.8078 | 928.0 |
| perp_050 | reach | 3 | 0 | 0.7916 | 0.7972 | 18.618 | 5.8210 | 1016.3 |
| headon | CV | 3 | 0 | **0.0169** | 0.1076 | 17.953 | 6.1514 | 943.7 |
| headon | reach | 3 | 0 | **0.0864** | 0.1413 | 16.168 | 6.0216 | 1051.7 |
| noconflict | CV | 3 | 0 | 1.1007 | 1.1099 | 13.318 | 5.6641 | 933.0 |
| noconflict | reach | 3 | 0 | 1.0955 | 1.1231 | 13.375 | 5.6866 | 1045.7 |

**On the low clearances.** They come entirely from `headon`, which is tight by
construction: the obstacle drives down a lane offset only 0.25 m from the route,
so it passes through the robot's corridor and roughly a 0.45 m lateral shift is
what clears it. This is a pre-existing property of that condition, not a
Stage-5C.1 effect — and the **CV** arm is the tighter of the two (0.0169 m
versus 0.0864 m). Neither arm collided and neither needed a collision-monitor
intervention, so nothing here is attributable to the fidelity fix.

**Scope caveat.** These are the Stage-4F bench conditions
(`perp_050`, `headon`, `noconflict`), not the `nav_cv_control` /
`nav_occlusion_maneuver` / `nav_reversal` scenarios behind
`validation/stage5c/planner_comparison.json`. The absolute numbers are
therefore **not** comparable trial-for-trial with the Stage-5C table; the
CV arm here is the control.

### D.3 The anomaly, resolved end to end

The point of the regression is the cell count, and it inverts:

| | planner-visible cells, reachability / CV |
|---|---|
| Stage-5C (`planner_comparison.json`, 505.4 / 633.3) | **0.80x** |
| Stage-5C per-sample saturation (`model_audit.json`, 377 / 529) | **0.713x** |
| Stage-5C.1 corrected (1037.9 / 934.9) | **1.11x** |

Reachability now presents the planner with *more* cells than CV, which is the
direction its mathematically larger region always implied. Fidelity to its own
region — not the comparison by size — is the gate, and that is what the
geometric golden set proves; this is the end-to-end confirmation.

---

## E. Performance

| measurement | Stage-5C baseline | Stage-5C.1 |
|---|---|---|
| layer `updateCosts` mean, CV arm | 22.16 us | 23.23 us |
| layer `updateCosts` mean, reachability arm | 23.0 us | **25.67 us** |
| layer `updateCosts` max, reachability arm | 79.8 us | 93.0 us |
| tracker reachability compute | ~6-8 us/scan | unchanged by construction |
| full master rebuild, 6 samples at production geometry (microbenchmark) | — | 29-49 us |

The reachability layer costs **+2.7 us** (about +12%) while painting **1.4x
more cells** — sub-linear, because the extra cells are added to a window that
was already being iterated. Nothing was optimized, and nothing needed to be.
The tracker-side contract adds one `hypot` comparison per sample on a path that
already computes `hypot(v0x, v0y)`, so the tracker figure is unchanged by
construction.

---

## F. Test accounting

**New Stage-5C.1 tests** (both PASS, 0 failures):

| test | package | covers |
|---|---|---|
| `stage5c1_contract` | tracking | 12 boundary cases, degenerate inputs, 1104-sample bit-identical in-domain equivalence, 1280 out-of-contract samples, adversarial proof the repair is real, tolerance rationale, the unobservable residual, end-to-end through the production node |
| `stage5c1_raster_fidelity` | costmap | 11 geometric golden cases + 6 degenerate, the discretization rule asserted against production, window-vs-map clipping, no out-of-bounds indices, the anomaly before/after, microbenchmark |

**Pre-existing functional tests — all 12 still pass:**
`stage4g2_lifecycle`, `stage4g3_deblend`, `stage4g4_reachability`,
`stage5b_classification`, `stage5b_tracker_integration`, `stage5b1_compound`,
`stage5b1_compound_integration` (tracking); `stage4g2_multi_layer`,
`stage4g4_reach_layer`, `stage4g5_hybrid_layer` (costmap).

`stage4g4_reach_layer` still reports `cv_mode_unaffected_by_reachability_field`,
`max_composition_exact`, `independent_expiry_exact`, `independent_removal_exact`
and `cv_mode_restored_exactly`, which is the direct evidence that the CV path
is untouched. Its reachability cell counts rose (the fix paints more), which the
test permits because it asserts monotonicity and containment, not absolute
counts.

**Pre-existing lint/style debt — untouched, and none of it caused by this
stage:**

| linter | failures | note |
|---|---|---|
| flake8 | 996 (bringup) + 14 (tracking) | pre-existing |
| pep257 | 193 (bringup) + 13 (tracking) | pre-existing |
| uncrustify | 5 (costmap) + 4 (tracking) | pre-existing; `predicted_obstacle_layer.cpp` has the **identical** single divergence at `ff23d9f`, verified by running the linter on the file as checked out at HEAD |
| xmllint | 1 (msgs `package.xml`) | pre-existing; `package.xml` was not modified |

Frozen files were **not** broad-formatted. The only file reformatted is the new
`stage5c1_raster_fidelity.cpp`; `stage5c1_contract.cpp`, `reachability.hpp` and
`lidar_obstacle_tracker_node.cpp` are uncrustify-clean as written.

---

## G. What Stage-5C.1 does NOT do

* No pedestrian / bike / vehicle-specific `max_speed`, acceleration or margin.
  `predict_reachability()` reads one global `reachability_bounds_` and nothing
  class-specific; `reachability.hpp` and `predicted_obstacle_layer.cpp` contain
  no reference to classification at all.
* No IMM, camera perception or learned prediction.
* **No production default change.** `production_nav2_params.yaml` keeps
  `prediction_mode: "cv_covariance"`; `production_tracker_params.yaml` keeps
  `reachability_enabled: false`.
* **It does not make the capability premise verifiable.** The unobservable half
  — true capability above the cap while the filtered speed stays below it —
  is measured, reported and left open. It is a declared configuration
  assumption, and closing it needs information this stage does not have.
