# Stage-4G1 validation evidence

Fix for the range-dependent static-map rejection weakness found in Stage-4F.

Scope is deliberately narrow: **only the static-rejection stage changed.**
Clustering, association, track lifecycle, the Kalman filter, the prediction
model, the predictive costmap layer, the MPPI settings and the Stage-4D/4E/4F
reference configurations are untouched. The pipeline is unchanged:

```
/scan -> transform to map -> STATIC REJECTION -> clustering -> association
      -> Kalman tracking -> predictions -> predictive costmap -> Nav2
```

`stage4f-validated` (68b2bf2) is preserved and unmoved.

**Result: Stage-4G1 PASSES** — all 16 acceptance criteria.

---

## 1. Root cause — measured, and NOT what Stage-4F assumed

Stage-4F recorded the hypothesis that the residuals came from *LiDAR angular
sampling*: 1° beams are 0.105 m apart at 6 m, so wall returns scatter past the
fixed 0.15 m rejection radius. **That hypothesis is wrong**, and the fix would
have been mis-derived if it had been taken on trust.

Angular sampling does not move a return *off* a wall. A beam that hits a wall
lands on the wall, wherever along it the sample happens to fall. Wider spacing
changes *which* point on the wall is sampled, not its distance to the wall.

`scripts/stage4g1_static_probe.py` measures the real quantity: for every return
in an obstacle-free world (so every return is static by construction), the true
Euclidean distance from the transformed map-frame point to the nearest occupied
map cell, binned by beam range. Two runs in the Stage-4F arena, same robot, same
map, same tracker:

### Robot STATIONARY — `probe_large_before.json`

| range [m] | points | mean d | p99 d | max d | retained by the 0.15 m rule |
|---|---|---|---|---|---|
| 1-2 | 2993 | 0.000 | 0.000 | 0.050 | **0 (0.00%)** |
| 2-3 | 4623 | 0.000 | 0.000 | 0.050 | **0 (0.00%)** |
| 3-4 | 4651 | 0.003 | 0.050 | 0.050 | **0 (0.00%)** |
| 4-5 | 5661 | 0.001 | 0.050 | 0.050 | **0 (0.00%)** |
| 5-6 | 1870 | 0.004 | 0.050 | 0.050 | **0 (0.00%)** |
| 6-8 | 1819 | 0.004 | 0.050 | 0.050 | **0 (0.00%)** |
| 8+ | 3583 | 0.003 | 0.050 | 0.050 | **0 (0.00%)** |

Zero residuals at **every** range, including 8+ m. Localisation error:
translation 0.008 m, yaw 0.00112 rad (0.064°).

If angular sampling were the mechanism, this table would already show residuals
at long range. It does not.

### Robot DRIVING — `probe_large_moving_before.json`

| range [m] | points | mean d | p99 d | max d | retained by the 0.15 m rule |
|---|---|---|---|---|---|
| 1-2 | 1595 | 0.034 | 0.050 | 0.100 | 0 (0.00%) |
| 2-3 | 4626 | 0.033 | 0.100 | 0.200 | 13 (0.28%) |
| 3-4 | 5835 | 0.043 | 0.100 | 0.200 | 45 (0.77%) |
| 4-5 | 7526 | 0.018 | 0.150 | 0.224 | 106 (1.41%) |
| 5-6 | 3306 | 0.015 | 0.150 | 0.250 | **82 (2.48%)** |
| 6-8 | 3289 | 0.013 | 0.150 | 0.250 | 51 (1.55%) |
| 8+ | 2263 | 0.010 | 0.100 | 0.100 | 0 (0.00%) |
| **total** | **28440** | | | | **297 (1.04%)** |

Localisation error while driving: translation mean 0.082 / p95 0.118 / max
0.137 m; **yaw mean 0.0171 / p95 0.0262 / max 0.0608 rad (3.48°)**.

### The actual mechanism

A pose **yaw** error of `dtheta` rotates the whole scan about the sensor, so a
beam endpoint at range `r` is displaced **laterally by `r · dtheta`**. Combined
with the translation error, the endpoint moves off the mapped wall by an amount
that **grows with range**, while the rejection radius did not.

That is why:
* the effect is **zero when stationary** (AMCL is seeded at the true pose and
  yaw error is 0.001 rad) and appears only under motion (yaw error rises ~15x);
* the residual rate **grows with range**, peaking at 2.48% in the 5-6 m bin;
* it never showed up in `tb3_sandbox`, where no wall is more than ~2.5 m away:
  at 2.5 m even a 0.06 rad yaw error displaces an endpoint only 0.15 m, right at
  the old threshold, whereas at 6 m the same error gives 0.36 m.

So it is **range-dependent**, as Stage-4F said, but the range-dependent
coefficient is the **pose angular uncertainty**, not the beam angular
resolution. Both are "an angle multiplied by range"; only one is ~15x larger
than the other while driving (0.0262 rad p95 versus 0.0087 rad half-spacing).

The residual *distance* is much smaller than the full lateral displacement
(max 0.25 m observed versus 0.28 m predicted at p95, 0.50 m at max) because a
lateral shift along a wall mostly slides the point **along** the surface. Only
at oblique incidence does it move the point **off** the surface — which is why
the residuals concentrate on the long walls seen at shallow angles.

---

## 2. Chosen method, and why

Both preferred approaches are used, because they solve different halves of the
problem and compose cleanly:

**(B) Distance field** — the *measurement*. An exact Euclidean distance
transform of the static map is built once per map message, giving metres from
every cell to the nearest static cell. The per-point query becomes a single
array lookup.

This is not merely an optimisation. The old code scanned a square
neighbourhood per point: at 0.15 m / 0.05 m that is 7x7 = 49 cells; a
range-aware radius reaching 0.40 m would need 17x17 = **289 cells per point**,
~6x the work, on every one of 360 points every scan. A distance field makes a
range-*dependent* radius cost exactly the same as a fixed one. It is also more
accurate: the old test compared **cell-index** offsets, quantising the distance
to multiples of the cell size and implicitly treating the query point as being
at its own cell's centre.

Implemented as Felzenszwalb & Huttenlocher's O(n) two-pass squared-distance
transform, ~50 lines in the node. **No new dependency** — the brief's caution
about pulling in a large library for a distance transform is respected.

**(A) Range-aware tolerance** — the *threshold*:

```
r_reject(range) = clamp( base_static_margin
                       + localization_margin
                       + angular_sampling_scale * range,
                         static_reject_radius,          // minimum
                         max_static_reject_radius )     // ceiling
```

A point is static when `distance_to_nearest_static_cell <= r_reject(range)`.

### Parameters and their physical meaning

| Parameter | Default | Unit | Basis |
|---|---|---|---|
| `range_aware_static_rejection` | `true` | - | `false` reproduces Stage-4B exactly, for A/B |
| `static_reject_radius` | 0.15 | m | **Minimum** radius (and the *only* radius when range-aware is off). Unchanged Stage-4B value and name |
| `base_static_margin` | 0.06 | m | One 0.05 m map cell of discretisation + ~3σ of the 0.01 m simulated range noise |
| `localization_margin` | 0.06 | m | Translation component of the map→sensor pose error (measured p95 0.118 m; only the part normal to a surface displaces a point off it) |
| `angular_sampling_scale` | 0.030 | rad | Beam half-spacing 0.0087 + measured pose yaw p95 0.0262 |
| `max_static_reject_radius` | 0.40 | m | Ceiling, so an anomalous long return can never blank a region |

Resulting radius: 0.15 m out to 1 m (the floor binds), then 0.18 m at 2 m,
0.24 m at 4 m, 0.30 m at 6 m, 0.36 m at 8 m, 0.40 m beyond 9.3 m.

**Close-range behaviour is bit-for-bit Stage-4B** because the floor is the old
0.15 m — the fix only ever *adds* tolerance where the geometry earns it. That
directly answers the brief's warning against globally raising the fixed radius:
at the ranges where a dynamic obstacle is most likely to matter, nothing
changed at all.

### Backward compatibility

`static_reject_radius` keeps its name and its 0.15 default, so **existing YAML
does not break**. Its meaning is now explicitly documented as *the minimum*.
Setting `range_aware_static_rejection: false` restores exact legacy behaviour.

---

## 3. Range-bin results, before and after

Both columns below are computed on **the same points from the same run**, so
this is a like-for-like comparison, not two runs compared across conditions.
Run: `probe_large_moving_after.json`, robot driving, 73 scans, 26280 points.
This run happened to have a *harder* localisation excursion than the "before"
run (yaw max 0.0693 rad vs 0.0608), which makes the comparison conservative.

| range [m] | points | max d | **old fixed 0.15 m** | **new range-aware** | r_reject at bin |
|---|---|---|---|---|---|
| 2-3 | 2697 | 0.112 | 0 (0.00%) | **0 (0.00%)** | 0.19 m |
| 3-4 | 5781 | 0.354 | 94 (1.63%) | **7 (0.12%)** | 0.23 m |
| 4-5 | 9059 | 0.291 | 135 (1.49%) | **3 (0.03%)** | 0.26 m |
| 5-6 | 4337 | 0.300 | 217 (5.00%) | **2 (0.05%)** | 0.29 m |
| 6-8 | 3854 | 0.350 | 463 (12.01%) | **4 (0.10%)** | 0.33 m |
| 8+ | 552 | 0.250 | 66 (11.96%) | **0 (0.00%)** | 0.40 m |
| **total** | **26280** | | **975 (3.71%)** | **16 (0.061%)** | |

**61x fewer residual points**, and — the point of the exercise — the residual
rate is now **flat with range** (0.00 / 0.12 / 0.03 / 0.05 / 0.10 / 0.00 %)
instead of climbing from 0% to 12%. The range-dependent weakness is gone.

Per the brief, residual raw points are **not hidden**: 16 points survived, about
0.22 per scan. They do not become tracks, because `cluster_min_points: 3`
requires three mutually-near points and `min_observations_to_publish: 3`
requires three consecutive associations. The distinction the brief asks for:

| level | before (per run) | after (per run) |
|---|---|---|
| raw retained points | 975 (3.71%) | **16 (0.061%)** |
| clusters formed | yes | none observed |
| confirmed persistent tracks | up to 3 | **0** |
| predictive occupancy | up to 441 cells | **0** |

---

## 4. Static-world regression — the main Stage-4F regression

`--conditions nominal --modes predictive`, i.e. the large arena, no dynamic
obstacle, predictive layer enabled, robot driving its full 6 m route.

| trial | nav time [s] | max tracks | frames tracked | frames with predictive cost | predictive cells |
|---|---|---|---|---|---|
| 01 | 13.30 | **0** | **0** | **0** | **0** |
| 02 | 13.30 | **0** | **0** | **0** | **0** |
| 03 | 13.35 | **0** | **0** | **0** | **0** |
| 04 | 13.30 | **0** | **0** | **0** | **0** |
| 05 | 13.41 | **0** | **0** | **0** | **0** |
| 06 | 13.30 | **0** | **0** | **0** | **0** |

Stage-4F, same test, before the fix: **6 of 10 runs** produced spurious
predictive occupancy (1, 29, 152, 180, 377, 441 cells), with up to 3
simultaneous false tracks and up to 100 tracked frames.

**Persistent false dynamic tracks: 0. False predictive occupancy: 0.**
Criteria 4 and 5 met.

---

## 5. Original Stage-4B world — no regression

`stage4d_bringup.launch.py spawn_obstacle:=False` (the `tb3_sandbox` world) with
the original Stage-4B evaluator:

```json
{"mode": "static", "frames_observed": 225,
 "distinct_track_ids_seen": 0, "false_persistent_tracks": 0}
```

Identical to the Stage-4B/4D validated result (225 frames,
`false_persistent_tracks: 0`). In this small world the new rule rejects
**100.00%** of returns, exactly as the old one did.

---

## 6. Dynamic-obstacle regression

Same world, same obstacle, same Stage-4B evaluator, 60 s:

| metric | Stage-4B/4C baseline | **Stage-4G1** |
|---|---|---|
| frames observed | 300 | 300 |
| matched frames | 300 | 300 |
| distinct track IDs | 1 | **1** |
| **ID switches** | 0 | **0** |
| dominant track ID fraction | 1.0 | **1.0** |
| mean position error [m] | 0.1494 | **0.1373** |
| mean velocity error [m/s] | 0.0293 | **0.0298** |

Stable ID preserved, no accuracy degradation (position error is in fact
marginally lower; velocity error is unchanged to 0.0005 m/s). The residual
~0.15 m position error is the known LiDAR-surface-versus-centre bias, not a
tracking error.

---

## 7. Near-wall dynamic obstacle — the risk this fix creates

A wider radius at long range could erase a genuine obstacle passing close to
mapped geometry. `scripts/stage4g1_nearwall_probe.py` measures that in **one**
run: the obstacle is driven diagonally so its surface-to-wall clearance sweeps
continuously from >1 m down to contact, and detection is binned by the
instantaneous clearance.

| surface clearance to wall [m] | frames | detected | rate | mean pos err [m] | track IDs |
|---|---|---|---|---|---|
| 0.0-0.1 | 50 | 50 | **1.00** | 0.184 | 1 |
| 0.1-0.2 | 4 | 4 | **1.00** | 0.165 | 1 |
| 0.2-0.3 | 5 | 5 | **1.00** | 0.169 | 1 |
| 0.3-0.4 | 5 | 5 | **1.00** | 0.167 | 1 |
| 0.4-0.6 | 9 | 9 | **1.00** | 0.173 | 1 |
| 0.6-1.0 | 19 | 19 | **1.00** | 0.173 | 1 |
| >1.0 | 105 | 105 | **1.00** | 0.166 | 1 |
| **overall** | **197** | **197** | **1.00** | — | **1** |

100% detection at every clearance down to touching the wall, one stable track
ID throughout, no accuracy loss.

**Why it holds up, stated honestly.** This is a favourable viewing geometry: the
robot sees the obstacle from the side *away* from the wall, so the
LiDAR-visible surface is the obstacle's far face, which stays well outside the
rejection radius even when the obstacle's *near* face is inside it. The result
should therefore be read as "the common case is safe", not as proof of general
discrimination. The genuinely hard case — an obstacle viewed nearly along a
wall, or one whose visible surface is inside the rejection radius — is not
solved by any map-differencing method, including this one, and is listed in
section 11.

---

## 8. Focused Stage-4F navigation regression

Three conditions x two modes x three trials on the frozen Stage-4F benchmark
(`stage4g1_navreg.json`). The purpose is regression safety, not a new
benchmark, so the Stage-4F 10+10 figures are shown beside them.

| condition | mode | succ | coll | min clr [m] | nav t [s] | react [s] | lead [s] | Stage-4F clr | Stage-4F lead |
|---|---|---|---|---|---|---|---|---|---|
| perp_050 | reactive | 3/3 | 0 | 0.151 | 18.29 | 6.02 | -- | 0.133 | -- |
| perp_050 | predictive | 3/3 | 0 | **0.794** | 18.55 | 3.97 | **2.05** | 0.795 | 1.90 |
| headon | reactive | 3/3 | 2 | 0.032 | 16.22 | 5.23 | -- | -0.010 | -- |
| headon | predictive | 3/3 | **0** | **0.111** | 16.54 | 3.97 | **1.26** | 0.141 | 1.28 |
| noconflict | reactive | 3/3 | 0 | 1.147 | 13.40 | *none* | -- | 1.136 | -- |
| noconflict | predictive | 3/3 | 0 | 1.129 | 13.32 | *none* | *n/a* | 1.145 | *n/a* |

Every figure is within normal run-to-run spread of the Stage-4F result:
reactive navigation still works, predictive navigation still works, predictive
still holds ~6x the clearance on the crossing and still eliminates the head-on
collisions, and the reaction lead is unchanged (2.05 vs 1.90 s; 1.26 vs 1.28 s).

On the no-conflict condition the predictive arm still paints ~934 cells — that
is the **real** obstacle in the parallel lane, correctly predicted — while
firing no reaction event and matching the reactive arm's navigation time to
0.08 s. Combined with section 4 (zero predictive cells with no obstacle at
all), this confirms the brief's specific requirement: **no false predictive
occupancy in empty or no-conflict regions caused by distant walls.**

---

## 9. Runtime performance

Measured in the node itself over the whole static-rejection stage (per-point
TF transform + distance-field lookup), reported on a throttled log line:

| world | scans | mean per scan | max per scan | points in | rejected |
|---|---|---|---|---|---|
| Stage-4F large | 562 | **249.4 µs** | **500.6 µs** | 360 / scan | 99.79% |
| Stage-4B small | 256 | **273.4 µs** | **497.3 µs** | 360 / scan | 100.00% |

The scan period is 200 ms (5 Hz), so the stage uses **0.12% of the budget**
(worst case 0.25%). The one-off distance-transform build is O(cells) and runs
once per map message on a 260x220 grid.

For reference, the old fixed-radius search scanned 49 cells per point; the new
lookup is O(1) per point, so the range-aware radius costs **nothing extra** at
long range where the old code would have had to scan 289 cells.

---

## 10. Localisation sensitivity

Not injected artificially — the two probe runs happened to experience different
AMCL excursions, which brackets normal variation:

| run | yaw p95 [rad] | yaw max [rad] | trans max [m] | old-rule residuals | new-rule residuals |
|---|---|---|---|---|---|
| before | 0.0262 | 0.0608 (3.48°) | 0.137 | 1.04% | — |
| after | — | **0.0693 (3.97°)** | — | 3.71% | **0.061%** |

The model's angular term is 0.030 rad, yet it absorbed a transient yaw error of
0.069 rad — **2.3x** the modelled value — without producing a single track. The
margin exists because the residual *distance* to a wall is much smaller than the
full lateral displacement: a lateral shift along a wall mostly slides the point
along the surface rather than off it.

**The trade-off, explicitly.** Raising the tolerance suppresses static clutter
better but increases the risk of suppressing a genuine dynamic object near
mapped geometry. This design places the whole increase at long range and leaves
the short-range radius at the original 0.15 m, so the risk is concentrated where
an obstacle is far away (and therefore not yet navigationally urgent) and absent
where it is close. If localisation were materially worse than measured here, the
correct response is to raise `localization_margin` and `angular_sampling_scale`
to match the measured error — and to accept the near-wall cost knowingly — not
to raise the floor.

---

## 11. Acceptance criteria

| # | Criterion | Verdict |
|---|---|---|
| 1 | Root cause reproduced/measured, not assumed | **PASS** — and the Stage-4F hypothesis was disproved in the process (section 1) |
| 2 | No longer a single global 0.15 m tolerance | **PASS** — range-aware, 0.15→0.40 m |
| 3 | Physically/geometrically defensible basis | **PASS** — every term derived from measured sensor/pose geometry (section 2) |
| 4 | Large-world static: 0 persistent false tracks | **PASS** — 0 in 6/6 runs |
| 5 | False predictive occupancy eliminated | **PASS** — 0 cells in 6/6 runs (was up to 441) |
| 6 | Range-bin analysis shows the distant problem improved | **PASS** — 3.71% → 0.061%, rate now flat with range |
| 7 | Genuine moving obstacle still reliable | **PASS** — 300/300 frames, 0 ID switches |
| 8 | Dynamic obstacle near static geometry tested | **PASS** — swept to contact, 100% detection |
| 9 | Stable IDs preserved | **PASS** — 1 ID, 0 switches, everywhere |
| 10 | Kalman prediction still functional | **PASS** — predictions published, velocity error 0.0298 m/s |
| 11 | Predictive costmap still functional | **PASS** — section 8 |
| 12 | Stage-4B world does not regress | **PASS** — identical result, 100% rejection |
| 13 | Focused Stage-4F navigation regression | **PASS** — section 8 |
| 14 | Runtime quantified and within budget | **PASS** — 249 µs mean / 501 µs max vs a 200 ms period |
| 15 | No unnecessary Stage-4C/D/E/F changes | **PASS** — only the static-rejection stage, its parameters, and new evaluation tooling |
| 16 | `~/ur5e_ws` untouched | **PASS** |

---

## 12. Reproduction

```bash
cd ~/predictive_nav_ws && colcon build --symlink-install && source install/setup.bash

# --- root-cause / range-bin measurement (large world, no obstacle) ---
ros2 launch predictive_nav_bringup stage4f_bringup.launch.py \
    headless:=True use_rviz:=False spawn_obstacle:=False run_trial:=False &
# stationary:
ros2 run predictive_nav_tracking stage4g1_static_probe.py --ros-args \
    -p out_path:=validation_logs/stage4g1/probe_stationary.json -p duration:=40.0
# driving (send goals to -3/+3 while the probe runs):
ros2 run predictive_nav_tracking stage4g1_static_probe.py --ros-args \
    -p out_path:=validation_logs/stage4g1/probe_moving.json -p duration:=45.0
# Both the old fixed rule and the new range-aware rule are scored on the same
# points, so one run yields the before/after table.

# --- large-world static regression (the main one) ---
python3 install/predictive_nav_bringup/lib/predictive_nav_bringup/run_stage4f_bench.py \
    --out validation_logs/stage4g1/static_large --conditions nominal \
    --modes predictive --trials 6

# --- original Stage-4B world, static then dynamic ---
ros2 launch predictive_nav_bringup stage4d_bringup.launch.py \
    headless:=True use_rviz:=False spawn_obstacle:=False &
ros2 run predictive_nav_bringup evaluate_tracking.py --mode static --duration 45
ros2 launch predictive_nav_bringup stage4d_bringup.launch.py \
    headless:=True use_rviz:=False spawn_obstacle:=True &
ros2 run predictive_nav_bringup evaluate_tracking.py --mode dynamic --duration 60

# --- near-wall dynamic obstacle (one run, clearance swept continuously) ---
ros2 launch predictive_nav_bringup stage4f_bringup.launch.py headless:=True \
    use_rviz:=False run_trial:=False spawn_obstacle:=True \
    obstacle_start_x:=-2.5 obstacle_start_y:=0.5 &
ros2 run predictive_nav_tracking stage4g1_nearwall_probe.py --ros-args \
    -p out_path:=validation_logs/stage4g1/nearwall.json -p duration:=40.0

# --- focused Stage-4F navigation regression ---
python3 install/predictive_nav_bringup/lib/predictive_nav_bringup/run_stage4f_bench.py \
    --out validation_logs/stage4g1/navreg \
    --conditions perp_050,headon,noconflict --trials 3

# --- exact legacy behaviour, for A/B ---
#   set range_aware_static_rejection: false in tracker_params.yaml
```

---

## 13. Known limitations and trade-offs

1. **Map differencing cannot separate an obstacle that overlaps mapped
   geometry.** When a dynamic object's visible surface lies inside
   `r_reject(range)` of a mapped wall, its returns are indistinguishable from
   wall returns by construction. Section 7 shows the common viewing geometry is
   safe, but no claim of general discrimination is made.
2. **The near-wall result is geometry-favourable.** The robot viewed the
   obstacle from the side away from the wall. An obstacle seen nearly *along* a
   wall would present its near face and is the harder case; it was not tested.
3. **`angular_sampling_scale` bundles two different angles** — beam half-spacing
   (0.0087 rad, sensor-intrinsic) and pose yaw uncertainty (0.0262 rad p95,
   localisation-dependent). They are summed because both scale with range and
   both are uncertainties on the same endpoint, but only the second varies with
   localisation quality. On a platform with materially different localisation,
   this value must be re-measured, not carried over.
4. **The defaults are calibrated to this simulator's AMCL behaviour.** The
   probe is committed precisely so the numbers can be re-measured rather than
   assumed on new hardware or in a new world.
5. **16 residual points per run remain** (0.061%). They are reported rather than
   suppressed. They never clustered into a track in any test here, but that is
   an empirical result at the current cluster thresholds, not a guarantee.
6. **The distance field is rebuilt on every map message.** For a static map
   published transient-local once, that is a single build. A continuously
   updating map (SLAM) would rebuild each time; at 260x220 that is cheap, but it
   was not profiled for large maps or high map rates.
