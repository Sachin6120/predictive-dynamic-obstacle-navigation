# Stage-5C.1 pre-change audit

Written BEFORE any code change, as required by sections 1, 6 and 7 of the
Stage-5C.1 brief. Everything below is read out of the tree at
`ff23d9febc94223ed2b9f8e3559cdac584a1bae9` or out of the committed Stage-5C
evidence in `validation/stage5c/`. No file in `src/` has been modified at the
time of writing.

---

## 1. The reachability validity contract, as it exists today

### 1.1 Where the model lives

| Thing | Location |
|---|---|
| Mathematics | `src/predictive_nav_tracking/include/predictive_nav_tracking/reachability.hpp` |
| Bounds struct | `ReachabilityBounds` (`reachability.hpp:80-86`) |
| Deterministic radius | `reachRadius()` (`reachability.hpp:115-140`) |
| Region assembly | `buildRegion()` (`reachability.hpp:150-191`) |
| Membership predicate | `regionContains()` (`reachability.hpp:196-208`) |
| Parameter declaration | `lidar_obstacle_tracker_node.cpp:390-397` |
| Parameter read | `lidar_obstacle_tracker_node.cpp:598-608` |
| Per-track production | `predict_reachability()` (`lidar_obstacle_tracker_node.cpp:1771-1805`) |
| Message | `src/predictive_nav_msgs/msg/ReachabilityPrediction.msg` |
| Consumer | `rasterizeTrackReachability()` / `rasterizeReachableSet()` (`predicted_obstacle_layer.cpp:485-595`) |

### 1.2 What `max_speed` means mathematically

`max_speed` is `v_max` in the derivation at the head of `reachability.hpp`. It
enters the bound at exactly one place, `reachability.hpp:131`:

```
delta_v_cap = v_max + s0
t_c         = delta_v_cap / a_max
r_det(t)    = 0.5*a_max*t^2                                  t <= t_c
            = 0.5*a_max*t_c^2 + (v_max + s0)*(t - t_c)       t >  t_c
```

It is used to bound the integrand of the deviation integral:

```
|v(s) - v0| <= |v(s)| + |v0| <= v_max + s0
```

So the quantity the model actually assumes is

> **(P)  |v(s)| <= v_max for every s in [0, t].**

That is a statement about the obstacle's **future speed over the whole
prediction window**, i.e. a **future physical capability bound**. It is NOT the
observed velocity, and it is NOT a tuning weight: the guarantee
`|p(t) - (p0 + v0 t)| <= r_det(t)` is only true while (P) and
`|a(u)| <= a_max` both hold.

### 1.3 How `s0` enters

`s0 = |v0| = hypot(v0x, v0y)` of the filtered state at the track's last real
observation (`reachability.hpp:171`). It enters twice:

1. the constant-velocity nominal centre `p0 + v0*t` (`reachability.hpp:167-168`);
2. additively inside `delta_v_cap = v_max + s0` — the obstacle is allowed to
   reverse, so the sum is correct, not the difference.

`s0` is floored at 0 (`reachability.hpp:126`) and is an **observed** quantity.
`v_max` is an **assumed** quantity. The existing code mixes them in one
expression and never compares them.

### 1.4 Configuration and current values

| Parameter | Default (node) | `tracker_params.yaml` | `production_tracker_params.yaml` | `stage5b_classification_tracker.yaml` |
|---|---|---|---|---|
| `reachability_enabled` | `true` | `true` | **`false`** | `true` |
| `max_acceleration` | 0.5 | 0.5 | 0.5 | 0.5 |
| `max_speed` | **0.8** | **0.8** | **0.8** | **0.8** |
| `reachability_horizon` | 3.0 | 3.0 | 3.0 | 3.0 |
| `reachability_time_step` | 0.5 | 0.5 | 0.5 | 0.5 |
| `base_safety_margin` | 0.0 | 0.0 | 0.0 | 0.0 |
| `max_observation_age` | 1.0 | 1.0 | 1.0 | 1.0 |
| `covariance_sigma_level` | 2.0 | 2.0 | 2.0 | 2.0 |

Production publication of reachability is **off**
(`production_tracker_params.yaml:137`); production costmap consumption is
`prediction_mode: "cv_covariance"` (`production_nav2_params.yaml:287`).
Reachability is a qualified *second* predictor, not the production default.

### 1.5 Observation age and total time

`predict_reachability()` (`lidar_obstacle_tracker_node.cpp:1778-1783`):

```
observation_age = max(scan_stamp - track.last_update, 0)
total_time      = observation_age + i*reachability_time_step,  i = 1..6
```

Every bound uses `total_time`, so a coasting track pays for its age. A region
is marked `valid = false` when `observation_age > max_observation_age`
(`lidar_obstacle_tracker_node.cpp:1799-1801`) — note this keys on
**observation_age**, not on total_time.

**Maximum valid total_time** =
`max_observation_age + reachability_horizon` = `1.0 + 3.0` = **4.0 s**.
This is the production total-time window inside which Stage-5C measured
under-coverage.

### 1.6 Assumptions the guarantee requires

1. `|a(u)| <= max_acceleration` for all `u in [0, t]`.
2. **(P)** `|v(s)| <= max_speed` for all `s in [0, t]`.
3. `p0, v0` are the filtered state at `track.last_update`, and the propagated
   covariance is taken at the same `total_time`.
4. Isotropy: the bound ignores direction/speed coupling, so the disc is a
   strict superset of the true reachable set *given 1 and 2*.

Assumption 2 is the one Stage-5C proved to be a real validity premise. Stage-5C
also recorded a measured limitation of assumption 1 that Stage-5C.1 does not
address: Gazebo's `VelocityControl` can apply step velocity changes with
|a| of 5-9.5 m/s^2, which no finite `a_max` covers at short horizon
(`lidar_obstacle_tracker_node.cpp:373-376`).

### 1.7 There is no runtime enforcement today

Searched `reachability.hpp` and `lidar_obstacle_tracker_node.cpp`: no
comparison of `s0` against `bounds.max_speed` exists anywhere. `reachRadius()`
absorbs any `s0` silently through `v_max + s0`, which keeps growing with `s0`,
so an out-of-contract sample produces a perfectly well-formed, finite,
`valid = true` region that *looks* like a qualified conservative bound and is
not one. `ReachabilityPrediction.msg` publishes `speed_capped` (whether the cap
*bound the radius*) but nothing about whether the cap was *credible*.

### 1.8 The observability limit, stated plainly

The premise is about **future** capability. Runtime cannot observe an object's
unknown future capability, so the premise cannot be *verified* at runtime.

What runtime **can** do is detect a **contradiction**: if the filtered speed is
`s0 > v_max`, then the object is demonstrably moving at `s0`, hence
demonstrably *capable* of `s0`, hence `v_phys >= s0 > v_max`, so premise (P) is
**disproved by observation**. That implication is sound and is the only part of
the premise that is observable. The residual — `v_phys > v_max` while
`s0 <= v_max` (Stage-5C's `s0 = 0`, `v_phys = 1.4` case) — is **not
observable** and must stay a declared configuration assumption.

### 1.9 Measured estimator behaviour that any guard must survive

From `validation/stage5c/model_audit.json`, per scenario, filtered speed vs the
0.8 m/s cap. Every scenario below is **in domain** (ground-truth speed <= 0.8):

| scenario | in model | max GT speed | max **filtered** speed | frames with s0 > 0.8 | filtered frames |
|---|---|---|---|---|---|
| A_constant_velocity | yes | 0.300 | 0.466 | 0 | 1960 |
| A_constant_velocity_fast | yes | 0.680 | **0.931** | 6 | 1560 |
| B_crossing | yes | 0.500 | 0.838 | 1 | 1960 |
| C_decelerate_stop | yes | 0.500 | 0.851 | 2 | 1960 |
| C_reversal | yes | 0.500 | 0.710 | 0 | 1960 |
| C_accelerate | yes | 0.700 | 0.816 | 1 | 1960 |
| D_turn_in_model | yes | 0.500 | 0.762 | 0 | 1560 |
| D_turn_out_of_model | no | 0.500 | 0.696 | 0 | 1560 |
| F_close_pass | yes | 0.400 | 0.602 | 0 | 1560 |
| H_occlusion_short | yes | 0.500 | 0.756 | 0 | 1960 |
| H_occlusion_maneuver | yes | 0.500 | 0.831 | 1 | 1960 |

**Consequence for the design.** The Kalman filtered speed transiently
overshoots true speed by up to 0.25 m/s, and exceeds the cap on in-domain
tracks by up to **0.131 m/s** (0.931 vs 0.800). Therefore:

* a bare `s0 > max_speed` rejection would fire on 11 of ~19,640 in-domain
  frames across 6 scenarios — it must not be a rejection;
* a dead-band wide enough to absorb that overshoot (>= 0.15 m/s) would mask
  exactly the `s0 in [0.80, 0.95]` band in which Stage-5C's adversary already
  measured escapes (`model_audit.json` `s0_just_above_cap`: 3334/20000 escapes,
  worst 0.155 m). A wide tolerance is therefore the wrong instrument.

The runtime response must consequently be **monotone-safe** — it may only ever
make the region larger — so that reacting to a transient overshoot costs
nothing and never removes coverage. That requirement drives the design in
section 2 of the implementation.

---

## 6. Costmap rasterization audit

### 6.1 The two paths, traced

Both modes share eligibility (`eligible()`, `predicted_obstacle_layer.cpp:323-330`)
so the mode can only change *how* a track is painted, never *which*.

**CV path** — `rasterizeTrackCv()` -> `rasterizeEllipse()`:

1. per prediction: horizon and finiteness guards (`:413-423`);
2. symmetrise covariance (`:427-429`), reject non-positive diagonal (`:431-433`);
3. transform mean and covariance into the costmap frame (`:435-438`);
4. eigen-decompose, **floor eigenvalues at `min_position_variance_`** (`:606-614`);
5. rebuild `sigma` and invert it -> `sigma_inverse` (`:616-621`);
6. bounding half-extents `half_x = sigma_level_*sqrt(sigma(0,0))`,
   `half_y = sigma_level_*sqrt(sigma(1,1))` (`:626-627`);
7. **clamp `half_x`, `half_y` to `max_influence_radius_`** (`:635-638`);
8. iteration window via `worldToMapEnforceBounds` on the two clamped corners
   (`:648-649`);
9. per cell: `mapToWorld` to the cell centre, then membership test
   **`delta^T sigma_inverse delta <= sigma_level_^2`** (`:657-659`).

> The membership test at step 9 uses `sigma_inverse`, which was built at step 5
> from the **unclamped** covariance. The clamp at step 7 touched only the
> bounding box. **The CV ellipse geometry is never shrunk.**

**Reachability path** — `rasterizeTrackReachability()` -> `rasterizeReachableSet()`:

1. per region: `valid`, horizon, `max_observation_age_`, finiteness and
   positive-axis guards (`:498-515`);
2. transform centre; rotate `sigma_yaw` by the frame yaw (`:517-523`);
3. **`semi_major = min(semi_major, max_influence_radius_)`**,
   **`semi_minor = min(semi_minor, max_influence_radius_)`** (`:538-541`);
4. bounding half-extents from the **already-clamped** axes (`:546-547`);
5. iteration window via `worldToMapEnforceBounds` (`:563-564`);
6. per cell: `mapToWorld` to the cell centre, then membership test
   `u = (dx*c + dy*sn)/semi_major`, `v = (-dx*sn + dy*c)/semi_minor`,
   `u^2 + v^2 <= 1` (`:574-576`) — **against the clamped axes**.

### 6.2 EXACTLY where the reachability semi-axes are reduced

`src/predictive_nav_costmap/src/predicted_obstacle_layer.cpp:538-541`

```cpp
if (max_influence_radius_ > 0.0) {
  semi_major = std::min(semi_major, max_influence_radius_);
  semi_minor = std::min(semi_minor, max_influence_radius_);
}
```

`semi_major` / `semi_minor` are by-value parameters of
`rasterizeReachableSet()`, and they are **reassigned** here. Every later use —
the bounding box at `:546-547` **and the membership test at `:574-575`** — sees
the reduced values. This is the single defect: the clamp is applied to the
**geometry**, not to the **window**.

### 6.3 Side-by-side

| | CV (`rasterizeEllipse`) | Reachability (`rasterizeReachableSet`) |
|---|---|---|
| what `max_influence_radius_` clamps | `half_x`, `half_y` — the **bounding box** | `semi_major`, `semi_minor` — the **ellipse itself** |
| membership tested against | **original** covariance (`sigma_inverse`) | **clamped** semi-axes |
| geometry shrunk? | **no** | **yes** |
| result when the clamp binds | ellipse ∩ box -> a **filled box** | a **disc of radius `max_influence_radius_`** |

### 6.4 Why the clamp exists

Documented at `predicted_obstacle_layer.cpp:629-634` (CV) and `:536-537`
(reachability): it is a **costmap-policy / performance bound** on how far the
layer will paint per sample, so that a long-horizon, high-variance prediction
cannot cost an unbounded number of cells. It is explicitly *not* a statement
about the model, and it is **not** a map-bounds guard — map bounds are enforced
separately and correctly by `worldToMapEnforceBounds()` at `:563-564` / `:648-649`.

So the clamp protects **computation**, not memory safety. Equivalent clamping
*is* present in CV, but applied to the bounding box only. The reachability path
applies it one level too deep.

### 6.5 Quantitative reproduction of the Stage-5C anomaly, from first principles

Production geometry: `max_influence_radius = 0.55 m`, `resolution = 0.05 m`
(`validation/stage5c/planner_comparison.json:config`).

When the clamp binds (`semi_major, semi_minor >> 0.55`, i.e. every horizon from
1.5 s on):

* **CV**: window is `±0.55` in x and y. Cell centres lie on a 0.05 grid, so the
  window holds `(2*floor(0.55/0.05)+1)^2 = 23^2 = **529**` cells. The true
  ellipse is far larger than the window, so **all 529** pass the membership
  test.
* **Reachability**: axes are reduced to 0.55, so the region *becomes* a disc of
  radius 0.55 inscribed in the same `±0.55` window. Cells whose centre
  satisfies `x^2 + y^2 <= 0.55^2` number **377**.
* Ratio `377 / 529 = **0.713**`.

`validation/stage5c/model_audit.json` reports, for every horizon from h = 1.5 s
onward, in both fresh and coasting regimes:

```
"cv_cells": 529.0, "reach_cells": 377.0, "cell_ratio": 0.713
```

The predicted values and the recorded values agree **exactly**. The anomaly is
therefore fully explained: the reachability region is being rasterized as the
disc **inscribed in** the window that CV fills completely, and `0.713 ~ pi/4`
is the area ratio of a disc to its circumscribed square.

The aggregate planner-visible figures in `planner_comparison.json`
(CV mean 633.3 cells vs reachability mean 505.4 cells, ratio 0.798) are the
same effect pooled over overlapping samples and track counts; the per-sample
saturation values 529 and 377 appear verbatim in both arms'
`distinct_values_sorted` lists.

This is also why the model-level ratios point the other way
(`reachability ~= 2.16x` CV fresh area, `~= 4.52x` CV coasting at h = 3.0):
the mathematical region really is larger, and the rasterizer discards the
difference *and then some*.

### 6.6 Pre-existing acknowledgement

`src/predictive_nav_costmap/test/stage4g4_reach_layer.cpp:170-176` already
documents the asymmetry:

> "CV mode's `max_influence_radius` clamp bounds the ellipse's BOUNDING BOX, so
> a long-horizon, high-variance CV sample degenerates into a filled square,
> while a reachable set stays an inscribed ellipse. That is pre-existing
> Stage-4D behaviour which Stage-4G4 must not change"

So the behaviour was **known and deliberately sidestepped** at Stage-4G4 (the
containment check there is run only at the one horizon where the clamp does not
bind). Stage-5C measured its consequence. Stage-5C.1 fixes it.

---

## 7. The required fidelity invariant

Coordinate semantics, as used by the code:

* `C` = this layer's `Costmap2D`, `size_x × size_y` cells of side `res`, origin
  `(ox, oy)`.
* Cell `(i, j)` has centre `c(i,j) = (ox + (i+0.5)*res, oy + (j+0.5)*res)`
  (`mapToWorld`). **Membership is decided at the cell centre** — that is the
  discretization rule, in both modes, unchanged by Stage-5C.1.
* `M = { (i,j) : 0 <= i < size_x, 0 <= j < size_y }` — the **representable
  extent**.
* `E` = the published continuous region of one sample: the closed ellipse
  centred at the transformed `position`, semi-axes `semi_major`/`semi_minor`
  **as published**, rotated by the transformed `sigma_yaw`.
* `W(E)` = the **policy window**: the axis-aligned box of half-extents
  `min(half_x, R)`, `min(half_y, R)` about the centre, where `half_x`, `half_y`
  are the bounding half-extents of `E` and `R = max_influence_radius_`
  (`R <= 0` disables it).

**Invariant F (fidelity).** For every cell `(i,j)` in `M` whose centre `c(i,j)`
lies in `E ∩ W(E)`, the rasterizer must write that cell. Equivalently: no cell
may be excluded *because the semi-axes of `E` were reduced*.

**Invariant S (safety).** The rasterizer writes only cells in `M`, and no cell
whose centre lies outside `E`.

Distinguishing the two kinds of clipping:

| | status |
|---|---|
| (A) clipping the iteration window to `M` — the map cannot represent occupancy outside itself | **legitimate**, kept |
| (A') clipping the iteration window to the policy box `R` — a bounded-work policy, applied identically in CV today | **legitimate**, kept, and now applied at the same level in both modes |
| (B) reducing `semi_major`/`semi_minor` and then testing membership against the reduced values | **the defect**, removed |

Invariant F is scoped to `E ∩ W(E)` rather than to `E` alone precisely because
(A) and (A') are legitimate: the layer is not required to paint outside the map,
nor outside its own declared policy window. It *is* required not to shrink the
geometry inside that window. After the fix the two modes treat `R` identically —
as a window bound — which is what makes them comparable at all.

**Expected effect.** Where the clamp binds, reachability's painted set becomes
`W(E)` filled (529 cells at production geometry) instead of the inscribed disc
(377), i.e. parity with CV's treatment of the same policy bound. Where the
clamp does not bind, `W(E)` is the true bounding box and the painted set is
unchanged — the fix is a no-op there.
