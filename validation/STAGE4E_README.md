# Stage-4E validation evidence

Controlled reactive-vs-predictive comparison on a crossing-conflict scenario.
Extends, and does not modify, the Stage-4B/4C/4D evidence in `README.md`,
`STAGE4C_README.md` and `STAGE4D_README.md`. The Stage-4D commit
(`7fff9f6`) and its parameter file `config/nav2_predictive_params.yaml` are
untouched, so Stage-4D stays reproducible; Stage-4E owns a separate
`config/nav2_stage4e_params.yaml`. Nothing under `/opt/ros` was modified and
no custom MPPI critic was written.

**Result: Stage-4E PASSES** (15/15 acceptance criteria), with criterion 9 qualified — see section 9.

---

## 1. Inspection of the pre-existing stack (before any change)

| Item | Finding |
|---|---|
| HEAD | `7fff9f6`, clean tree |
| Controller | **`nav2_mppi_controller::MPPIController` was ALREADY in use.** No controller swap was needed or made. |
| MPPI critics | ConstraintCritic, **CostCritic**, GoalCritic, GoalAngleCritic, PathAlignCritic, PathFollowCritic, PathAngleCritic, PreferForwardCritic |
| MPPI horizon | `time_steps: 56` x `model_dt: 0.05` = **2.8 s**; `batch_size` 2000, `iteration_count` 1 |
| Rates | controller 20 Hz, local costmap update 5 Hz / publish 2 Hz, `expected_planner_frequency` 20 Hz, global costmap 1 Hz |
| Local costmap | 3 x 3 m, 0.05 m, rolling, `odom` frame, `robot_radius` 0.22, plugins `[voxel_layer, inflation_layer, predicted_obstacle_layer]`, `inflation_radius` 0.70 |
| Planner | `nav2_navfn_planner::NavfnPlanner` |
| Speed / accel | MPPI `vx_max` 0.5, `vx_min` -0.35, `wz_max` 1.9, `ax_max/min` ±3.0; velocity smoother `max_accel` 2.5, `max_decel` -2.5 m/s² |
| Moving obstacle | `models/dynamic_obstacle.sdf`, r = 0.20 m cylinder, driven by `scripts/obstacle_mover.py` in a **back-and-forth** sweep |

Because MPPI was already configured, Stage-4E followed the brief's first
branch: use its existing costmap-aware CostCritic. Nav2's
`CostCritic::inCollision` treats `LETHAL_OBSTACLE` (254) and, with
`consider_footprint: false`, `INSCRIBED_INFLATED_OBSTACLE` (253) as
collisions and rejects the trajectory outright; costs in 1..252 are instead
summed into the trajectory score. The predictive layer is capped at 252 by
design, so it can only ever act through that soft-cost path.

---

## 2. Scenario (deterministic, identical in both arms)

Map `tb3_sandbox`: a circular arena with a 3 x 3 grid of pillars leaving
corridors whose free half-width is 0.35-0.57 m. The robot's footprint is a
0.265 m box, so the lateral margin is 0.13-0.35 m per side. Geometry was
chosen by computing the clearance field of the map, not by eye.

| Quantity | Value |
|---|---|
| Robot start | **(1.75, -0.55)**, yaw pi (facing -x) |
| Robot goal | **(-1.75, -0.55)**, yaw pi |
| Route length | 3.50 m, straight corridor |
| Crossing point | **(-0.55, -0.55)** -- the most open junction on the route (0.57 m clearance) |
| Obstacle park pose | **(-0.55, -2.05)**, stationary, held at zero velocity |
| Obstacle motion | **+y at 0.25 m/s**, one-way, no reversal |
| Obstacle release | **t = 1.15 s** after goal acceptance |
| Obstacle halt | y = **+0.90** (1.45 m clear of the corridor, outside its own 0.90 m inflation reach) |
| Obstacle travel to corridor | 1.50 m / 0.25 = **6.00 s**, arriving at t = **7.15 s** |
| Robot arrival at crossing (obstacle-free) | **7.13 s** (7.0 / 7.0 / 7.3 over 3 nominal runs) |

The two arrival times are matched to within 0.02 s, so **without prediction
this is a genuine simultaneous-arrival conflict**, not a stationary blockage.

At t = 0 the obstacle is 1.50 m from the crossing and completely outside the
robot's 3x3 m rolling local costmap; the route is fully traversable and the
global plan is a straight line. Only the obstacle's *velocity*, once
estimated, implies the future conflict. Obstacle speed 0.25 m/s is the same
regime validated in Stage-4B/4C.

The Stage-4D back-and-forth `obstacle_mover` is deliberately **not** used:
Stage-4E's runner drives the obstacle one-way, so the Kalman filter's
post-reversal velocity re-convergence (documented in `STAGE4C_README.md`)
cannot contaminate the measurement, and obstacle release is keyed to goal
acceptance so `t = 0` means the same thing in both arms.

### Route direction is deliberate

The corridor's only severe pinch (0.13 m lateral margin, x = 0.90..1.30) sits
in the **first 0.75 m** of the route, traversed undisturbed and at speed. An
earlier 10+10 batch ran the route in the opposite direction, putting that
pinch *after* the crossing; **every timeout in that batch, in both arms
alike, was the robot stalling at x = 0.58 on the pinch entrance with the
conflict already resolved.** That is a route artifact unrelated to
prediction, and it was corrupting the success-rate metric.

That superseded batch is **reported, not discarded**, in
`stage4e_superseded_plusx_ab.json`. Its headline numbers, for comparison:
reaction lead **1.400 s** (reactive 6.936 ± 0.118, predictive 5.536 ± 0.095),
advance warning **2.775 ± 0.179 s**, min clearance **0.062 ± 0.047 -> 0.138
± 0.030 m**, collisions 1 -> 0 -- i.e. every prediction-related finding
reproduces on the opposite route direction. Only the success rate differed
(4/10 vs 5/10), because in that direction the pinch defeated both arms
equally.

### Mode switch

Both arms load the **identical** `nav2_stage4e_params.yaml`, the identical
plugin set, the identical tracker, and the identical obstacle schedule. The
trial runner sets exactly one parameter differently before sending the goal:

```
/local_costmap/local_costmap  predicted_obstacle_layer.enabled  =  False | True
```

The Stage-4E predictive policy values are applied in **both** arms (inert
when the layer is disabled), so the layer's boolean is the single independent
variable. Stage-4D test G already validated that disabling the layer clears
its contribution completely.

---

## 3. Deriving the Stage-4E predictive policy

Stage-4D's defaults (horizon 3.0 s, sigma 2.0, decay 0.7, cost 0-200) are
generic and blanket the local window. The final Stage-4E policy is:

| Parameter | Stage-4D | **Stage-4E** | Why |
|---|---|---|---|
| `max_prediction_horizon` | 3.0 | **3.0** | Lead offered = `v_obs * H + max_influence_radius` = 0.25*3.0 + 0.55 = **1.30 m** ahead of the obstacle, vs **0.90 m** for reactive sensing (obstacle radius 0.20 + `inflation_radius` 0.70). That 0.40 m is the entire structural basis for reacting earlier, so H is set as long as the inputs allow: Stage-4C publishes to exactly 3.0 s and MPPI integrates 2.8 s. |
| `sigma_level` | 2.0 | **1.5** | ~67% of 2D mass. Stage-4C's 0.5 m/s² accel noise gives sigma_pos ~ 0.25t², so beyond ~1.5 s it is `max_influence_radius`, not sigma, that bounds the region. Stage-4C's covariance is **not** retuned. |
| `temporal_decay` | 0.7 | **0.35** | Monotonic 0.84 (0.5 s) -> 0.35 (3.0 s). At 0.7 the far horizon collapses to 0.12 -- and the far horizon is precisely the part that reaches the robot's path first, so a steep decay makes the layer inert exactly where it needs authority. |
| `max_cost` | 200 | **250** | Below `MAX_NON_OBSTACLE` (252) and below 253, which CostCritic treats as a collision. Prediction stays a strong **soft** cost, never a sensed lethal obstacle. |
| `min_cost` | 0 | **200** | See the measurement below. |
| `max_influence_radius` | 3.0 | **0.55** | robot_radius 0.22 + obstacle radius 0.20 + 0.13 margin: the radius within which the two bodies would actually conflict. This is the compactness lever; measured window coverage **20.9%** vs 55% at Stage-4D defaults. Still a costmap-policy bound, never a truncation of the published covariance. |

### The measurement that set the cost band

The layer combines with `updateWithMax`, so it can only influence the
controller **where it exceeds what the other layers already wrote there.**
The trial runner samples `/local_costmap/costmap_raw` in the strip the MPPI
controller can actually reach (0.2-1.4 m ahead, ±0.15 m lateral -- the strip a
forward trajectory's sampled centre points occupy):

* Ambient cost on the robot's own centreline, from pillar inflation alone:
  **88-170, peaks 234-253**.
* Stage-4D's 0-200 band puts only **~50-60** on the cells that actually reach
  the corridor (they sit near the ellipse edge). It raised essentially no
  master cells and changed commanded speed by **~4%**.
* The 200-250 band raises the measured mean ahead from **186.0 -> 201.7**
  and cells >=180 from 128.6 -> 143.5.

`inflation_radius: 0.70` exceeds the corridor's 0.35 m free half-width, so
ambient cost saturates every drivable cell. This is the central difficulty of
Stage-4E and it is a property of the world, not of the layer.

### Corrections that were tried and rejected (with evidence)

| Change | Result | Verdict |
|---|---|---|
| Compact policy only (H 1.5, sigma 1.0, r 0.75) | Window coverage 55% -> 13%, but reaction time unchanged (5.15 s vs 5.05 s reactive) | insufficient alone |
| `min_cost` 200 at stock critic weight | Mean cost ahead 145 -> 212, commanded speed changed by only 0.02 m/s | insufficient alone |
| `temporal_decay` 0.15 and 0.08 | No change in reaction time | insufficient alone |
| `inflation_radius` 0.70 -> 0.30 (restore dynamic range) | **Nominal navigation ABORTS.** With only 0.13 m margin the inflation gradient is what keeps the robot centred; removing it makes MPPI clip the pillars. | **rejected** |
| `CostCritic.cost_weight` 30.0 | Robot cannot traverse the corridor at all | **rejected** |
| `CostCritic.cost_weight` 6.0 / 8.0 | Too weak to change behaviour | rejected |

---

## 4. Controller: why one parameter changed

Per the brief, a project-owned MPPI configuration was introduced **only after**
the stock configuration was shown to be unable to exploit the cost. The
evidence: with the predictive layer raising the mean cost on the robot's own
centreline from 145 to 212, commanded speed changed by **~4% (0.02 m/s)**, and
reaction time was identical in both arms (5.05-5.25 s) across every policy
tried. At `cost_weight: 12.0` the same cost rise produces a ~24% early
slowdown.

The **only** controller difference from Stage-4D:

```yaml
CostCritic:
  cost_weight: 3.81   ->   12.0     # stock nav2_mppi_controller critic
```

This is the stock built-in critic with a reweighted parameter. **No custom
critic was written and no Nav2 source was modified.** It is applied
identically in both arms, so it cannot bias the A/B result -- the reactive arm
benefits from it too, since its response to the obstacle's own inflation also
strengthens.

Separating the controller change from the prediction effect:

Measured on the **same** route (3.65 m, +x direction) so the two weights are
directly comparable:

| Configuration | Obstacle-free nav time, 3 runs | Predictive reaction lead |
|---|---|---|
| Stock `cost_weight` 3.81 | **8.45 s** (8.40 / 8.45 / 8.50) | **none measurable** |
| Stage-4E `cost_weight` 12.0 | **10.15 s** (9.90 / 10.10 / 10.45) | **1.30-1.40 s** |

The controller change costs **~20%** in obstacle-free traversal time and buys
the ability to act on costmap cost at all. Both numbers are reported rather
than only the favourable one. (On the final -x route the Stage-4E controller
covers its 3.50 m in 9.58 s: 9.50 / 9.50 / 9.75.)

---

## 5. Reaction definitions (frozen before the final trials)

Thresholds were derived from the obstacle-free nominal envelope measured at
the Stage-4E controller configuration (3 runs, `validation/stage4e_nominal.json`):
commanded cruise **min 0.445 / median 0.490 m/s**, max `|w|` **0.095 rad/s**,
max lateral deviation from the corridor **0.036 m**. Every threshold sits
outside that envelope, so ordinary corridor following cannot trip one.

| Event | Definition | Nominal envelope |
|---|---|---|
| **R1 primary** -- speed reduction | commanded `v_x` < 0.288 m/s (0.60 x 0.48) sustained 0.25 s | never below 0.445 |
| R2 -- angular | `abs(w_z)` > 0.30 rad/s sustained 0.25 s | max 0.095 |
| R3 -- lateral deviation | `abs(y + 0.55)` > 0.10 m sustained 0.25 s | max 0.036 |
| R4 -- stop | commanded `v_x` < 0.05 m/s sustained 0.25 s | never |

Two further rules keep the metric honest:

* Scoring opens only once the robot has reached cruise (first sample at
  >= 0.8 x nominal). The first ~1.3 s is the startup acceleration ramp, during
  which `v_x` is legitimately below threshold in **every** run including
  obstacle-free ones; counting it would report t~0 for every trial in both arms.
* Events are only counted while the robot is still **approaching** the
  crossing. A slowdown after passing it is goal deceleration, not avoidance.

Reaction events are scored on `/cmd_vel_nav`, the **controller's own output**,
upstream of the velocity smoother and collision monitor, so "Nav2 reacted"
means the controller reacted and not a downstream safety filter.

---

## 6. Ground truth

Ground truth is used for **evaluation only**, and for commanding the scripted
obstacle. It never enters the tracker, the Kalman filter, the costmap layer,
the planner or the controller -- those consume only `/scan` and the normal ROS
graph. The obstacle is detected from real simulated LiDAR throughout; the
tracker's Kalman filter reached `kalman_initialized` in every trial.

* Robot pose: the simulator's exact odometry, rotated by the known spawn pose.
* Obstacle pose: `/model/dynamic_obstacle/odometry` (Gazebo model odometry).
* **Clearance is measured against the real collision geometry**, not Nav2's
  planning radius: `gz_waffle.sdf.xacro` gives `base_collision` as a
  0.265 x 0.265 m box at (-0.064, 0) in `base_link`, widened to ±0.153 m in y
  to enclose the wheels. The waffle is markedly asymmetric -- 0.197 m of body
  behind `base_link` but only 0.069 m in front -- so a circumscribed circle
  (0.237 m) overstates frontal overlap by ~0.09 m and reports phantom
  collisions for an obstacle passing in front of a stopped robot. `collision`
  means the bodies actually intersect.

---

## 7. Primary result -- 10 reactive vs 10 predictive

Machine-generated by `scripts/summarize_stage4e.py`; full tables in
`stage4e_crossing_ab.json`, per-trial rows in `stage4e_crossing_trials.csv`.
Trials were interleaved (R,P,R,P,...) so machine drift cannot bias one arm.

|  | Reactive | Predictive |
|---|---|---|
| Trials | 10 | 10 |
| **Navigation success** | **2 / 10 (0.20)** | **9 / 10 (0.90)** |
| **Collisions (body contact)** | **1** | **0** |
| **Mean min clearance [m]** | **0.066 ± 0.036** | **0.506 ± 0.269** |
| **Worst min clearance [m]** | **-0.013** (contact) | **+0.091** |
| Mean min centre separation [m] | 0.382 ± 0.051 | 0.803 ± 0.251 |
| Mean navigation time [s] | 51.02 ± 17.96 | 22.43 ± 12.72 |
| Mean path length [m] | 2.658 ± 0.476 | 3.370 ± 0.530 |
| **Mean reaction time R1 [s]** | **6.451 ± 0.148** | **5.151 ± 0.409** |
| Stops [count] | 2.4 ± 0.7 | 3.1 ± 0.9 |
| Mean stop duration [s] | 41.53 ± 18.52 | 11.59 ± 13.74 |
| Min commanded v, approach [m/s] | -0.229 ± 0.068 | -0.193 ± 0.097 |
| Max lateral deviation [m] | 0.058 ± 0.026 | 0.050 ± 0.016 |
| Collision-monitor interventions | 1.7 ± 1.0 | 0.3 ± 0.9 |
| Global route changes | 2.2 ± 0.9 | 2.8 ± 0.9 |
| Predictive cells (max) | 0 | 754 ± 35 |
| Predictive window coverage | 0.000 | 0.209 ± 0.010 |
| Mean master cost ahead | 186.0 ± 9.1 | 201.7 ± 5.8 |

### Predictive reaction lead time

**1.300 s** (reactive 6.451 s, predictive 5.151 s). Welch t = **-8.97**,
df 11.3; the arms do not overlap.

At its reaction the predictive robot is still **1.12 m** from the crossing
with the obstacle **0.62 m** away from it; the reactive robot reacts at
0.72 m with the obstacle 0.31 m away.

### Advance warning (acceptance criterion 4)

| Quantity | Value |
|---|---|
| First predictive cost anywhere | t = 2.4 s (typical) |
| First predictive cost **within 0.35 m of the crossing point** | t = 3.35-3.50 s |
| Obstacle **physically** within 0.35 m of the crossing point | t = 6.15 s |
| **Lead** | **2.690 ± 0.156 s** |

Predictive cost occupies the future crossing point **2.7 s before the
obstacle physically gets there**, in 10/10 predictive trials.

### The causal chain, one matched pair

Trial 02. `pcells` is the predictive layer's own debug grid.

| t [s] | REACTIVE x, v_cmd, clearance | PREDICTIVE x, v_cmd, clearance, pcells |
|---|---|---|
| 2.4 | 1.56, 0.235, 2.10 | 1.59, 0.164, 2.13, **334** |
| 4.8 | 0.58, 0.459, 0.96 | 0.69, **0.426**, 1.06, 343 |
| 5.1 | 0.45, 0.458, 0.82 | 0.57, **0.326**, 0.93, 678 |
| 5.4 | 0.30, 0.448, 0.65 | 0.49, **0.165**, 0.83, 736 |
| 5.7 | 0.17, 0.481, 0.51 | 0.45, **0.077**, 0.76, 642 |
| 6.3 | -0.10, **0.379**, 0.20 | 0.35, 0.306, 0.63, 709 |
| 6.6 | -0.20, **0.049**, **0.086** | 0.25, 0.325, 0.52, 688 |
| 7.2 | -0.22, -0.108, **0.044** | 0.09, 0.205, 0.34, 688 |
| 12.9 | -0.58, 0.031, 0.89 | 0.57, -0.042, 1.28, **0** |
| 18.3 | -0.59, -0.006, 1.14 (stuck) | 0.22, **0.405**, 1.27, 0 |
| 21.9 | -0.59, -0.005, 1.14 (stuck) | -1.38, 0.366, 1.23, 288 |

LiDAR observes the obstacle -> the tracker estimates its motion -> the Kalman
filter predicts the future crossing -> predictive cost appears in the costmap
from t = 2.4 s -> MPPI begins a **graded** slowdown at t = 4.8 s while the
obstacle is still 0.66 m from the corridor -> the robot holds ~0.71 m
clearance, reverses slightly, waits -> predictive cost clears at t ~ 12.9 s ->
the robot resumes at full 0.5 m/s and completes. The reactive robot cruises
at 0.48 m/s until t = 6.3 s, hard-stops at 0.086 m clearance, and does not
recover.

Note the qualitative difference: predictive produces a **progressive**
deceleration (0.497 -> 0.426 -> 0.326 -> 0.165 -> 0.077 over 0.9 s), reactive
produces a **hard stop** (0.486 -> 0.049 over 0.3 s).

---

## 8. Control experiment -- no future conflict

`--scenario noconflict`: the same obstacle, same column, same release
schedule, but driven **away** from the robot's corridor (starting at
(-0.55, -1.30), moving -y to -2.20). It is visible, tracked and predicted
throughout, but its predicted trajectory never intersects the route.

Machine-generated into `stage4e_noconflict_ab.json`.

|  | Reactive | Predictive |
|---|---|---|
| Trials | 5 | 5 |
| Navigation success | 5/5 | **5/5** |
| Collisions | 0 | 0 |
| Navigation time [s] | 9.871 ± 0.133 | **9.721 ± 0.160** |
| Path length [m] | 2.974 ± 0.011 | **2.976 ± 0.009** |
| Min clearance [m] | 1.288 ± 0.009 | 1.282 ± 0.008 |
| Min commanded v, approach [m/s] | 0.389 ± 0.002 | **0.393 ± 0.004** |
| Mean commanded v, approach [m/s] | 0.483 ± 0.001 | **0.481 ± 0.004** |
| **R1 speed-reduction events** | **none** | **none** |
| **R4 stop events** | **0** | **0** |
| Stops (goal stop only) | 1.0 | 1.0 |
| Global route changes | 0 | 0 |
| Collision-monitor interventions | 0 | 0 |
| **Predictive cells (max)** | 0 | **676 ± 60** |

The layer is demonstrably active -- it paints 676 cells, 18.8% of the window,
and the tracker is estimating the obstacle's motion throughout -- yet **not one
reaction event fires in either arm**, navigation time differs by 0.15 s (the
predictive arm is marginally *faster*, i.e. the difference is noise), and the
path is identical to 2 mm. No unnecessary stop, no detour, no route change.

This is the guard the brief asks for: the predictive layer is behaving as a
*prediction of a specific future conflict*, not as a generic penalty field
that slows the robot near any tracked object.

---

## 9. Acceptance criteria

| # | Criterion | Verdict |
|---|---|---|
| 1 | Identical reproducible reactive/predictive crossing scenarios | **PASS** -- one scenario definition, one launch file, mode parameter |
| 2 | Reactive uses live sensing, layer disabled | **PASS** -- voxel layer + LiDAR unchanged, `enabled: False` |
| 3 | Predictive differs primarily by enabling the layer | **PASS** -- one boolean; identical params file, plugins, tracker, obstacle |
| 4 | Predictive cost precedes physical occupancy | **PASS** -- 2.690 ± 0.156 s lead, 10/10 |
| 5 | Nav2 demonstrably reacts to predictive cost | **PASS** -- graded slowdown at t = 4.8 s, 1.30 s before the reactive arm, measured on the controller's own output |
| 6 | Reaction timing measured objectively | **PASS** -- thresholds frozen from the obstacle-free envelope before the final trials |
| 7 | Predictive reacts measurably earlier / safer | **PASS** -- lead 1.300 s, Welch t = -8.97; clearance 7.7x better, t = 4.85 |
| 8 | Clearance equal or improved without unacceptable degradation | **PASS** -- 0.066 -> 0.506 m; navigation time also *improves* (51.0 -> 22.4 s). Path length rises 2.66 -> 3.37 m, which is the yield-and-resume manoeuvre. |
| 9 | Navigation success remains high | **QUALIFIED PASS** -- predictive 9/10. Reactive is 2/10, but that is the arm under test failing, not a regression: see below. |
| 10 | No-conflict control does not cause unreasonable avoidance | **PASS** -- 5/5 success both arms, zero reaction events, nav time 9.87 vs 9.72 s, path length identical to 2 mm, while the layer still paints 676 cells |
| 11 | >= 10 trials per arm | **PASS** -- 10 + 10, interleaved |
| 12 | Quantitative results machine-generated | **PASS** -- `summarize_stage4e.py`; no hand transcription |
| 13 | No predictive-cost ghosting/regression | **PASS** -- section 10 |
| 14 | Build and lifecycle clean | **PASS** -- section 10 |
| 15 | `~/ur5e_ws` untouched | **PASS** -- never referenced by any Stage-4E file |

### Criterion 9, stated honestly

The predictive arm reaches the goal in 9/10 trials. The reactive arm reaches
it in 2/10: in the other 8 it grazes the obstacle (min clearance 0.03-0.11 m,
once -0.013 m), backs away, and after the obstacle has cleared it remains at
x ~ -0.58 with the controller commanding ~0 m/s until the 60 s budget expires.

This is a real navigation failure and it is downstream of the late reaction --
but it is **amplified** by the Stage-4E CostCritic weight, which makes MPPI
conservative about committing to the 0.18 m-margin corridor from a disturbed
pose. At the stock weight the reactive arm completes the same scenario
(slowly, at 0.08-0.17 m clearance) but then no arm reacts to prediction at
all. The success-rate gap should therefore be read as *directionally* correct
but *magnitude-inflated*; the reaction-lead and clearance figures do not
depend on it.

---

## 10. Regression

| Check | Result |
|---|---|
| `colcon build` | clean, 4 packages |
| Layer source unchanged since Stage-4D | `git diff 7fff9f6 -- src/predictive_nav_costmap` is empty -- Stage-4E changed **configuration only** |
| Stage-4D params file unchanged | `git diff 7fff9f6 -- .../nav2_predictive_params.yaml` empty |
| Predictive layer loads via pluginlib | yes, every trial (754 ± 35 cells painted) |
| Static world -> zero predictive cost | **yes** -- layer enabled, no obstacle: `max_pred_cells 0`, `frames_with_pred_cost 0`, `max_tracks 0` in 2/2 runs |
| `NavigateToPose` with layer enabled, static world | 2/2 SUCCEEDED, 9.80 / 9.80 s |
| `NavigateToPose` on a static world | 3/3 SUCCEEDED, 9.50 / 9.50 / 9.75 s |
| Lifecycle | all nodes reach `active`; no crashes or deadlocks in 60+ trials |
| Ghosting / expiry / reversal | Stage-4D tests E and F remain valid: the plugin is byte-identical, and the obstacle halting at y = +0.90 drives predictive cells to exactly **0** mid-run in every predictive trial (visible at t ~ 12.9 s above), which re-demonstrates clean expiry |
| `~/ur5e_ws` | untouched |

One infrastructure fault was found and fixed while building the harness:
surviving `component_container` processes from a previous trial kept their
Nav2 lifecycle nodes on the ROS graph, so the next trial reported *"Failed to
bring up all requested nodes"* and rejected every goal, or two controllers
published to `/cmd_vel` at once and the robot deadlocked mid-corridor. The
batch driver now kills and **verifies** teardown, and gives each trial its own
`ROS_DOMAIN_ID`. Trials run before that fix were re-run, not reported.

---

## 11. Reproduction

```bash
cd ~/predictive_nav_ws && colcon build --symlink-install && source install/setup.bash

# Primary A/B: 10 reactive + 10 predictive, interleaved  (~25 min)
python3 install/predictive_nav_bringup/lib/predictive_nav_bringup/run_stage4e_ab.py \
    --out validation_logs/stage4e/primary2 --trials 10 \
    --scenario crossing --modes reactive,predictive --interleave

# No-conflict control (5 + 5)
python3 install/predictive_nav_bringup/lib/predictive_nav_bringup/run_stage4e_ab.py \
    --out validation_logs/stage4e/control --trials 5 \
    --scenario noconflict --modes reactive,predictive --interleave

# Obstacle-free baseline / reaction-threshold calibration
python3 install/predictive_nav_bringup/lib/predictive_nav_bringup/run_stage4e_ab.py \
    --out validation_logs/stage4e/nom2 --trials 3 --scenario nominal --modes reactive

# Statistics (machine-generated; nothing is transcribed by hand)
python3 src/predictive_nav_bringup/scripts/summarize_stage4e.py \
    validation_logs/stage4e/primary2 \
    --out validation/stage4e_crossing_ab.json \
    --csv validation/stage4e_crossing_trials.csv

# Watch one trial live (RViz: robot, global path, LiDAR, tracked object,
# filtered velocity, predicted trajectory, uncertainty ellipses, predictive
# costmap, MPPI optimal + candidate trajectories)
ros2 launch predictive_nav_bringup stage4e_bringup.launch.py \
    headless:=False use_rviz:=True mode:=predictive     # or mode:=reactive
```

Stage-4D remains reproducible exactly as before:

```bash
ros2 launch predictive_nav_bringup stage4d_bringup.launch.py \
    headless:=True use_rviz:=False spawn_obstacle:=True
```

---

## 12. Known limitations

1. **The corridor is narrow (0.13-0.35 m lateral margin).** Prediction can
   therefore only express itself as speed modulation and small lateral shifts,
   never as a route detour. Max lateral deviation is 0.05-0.06 m in both arms.
   A wider environment would allow -- and should be used to test -- genuine
   predictive re-routing.
2. **`inflation_radius: 0.70` exceeds the corridor's free half-width**, so the
   costmap carries almost no dynamic range above ambient. This forced both the
   high predictive cost band (200-250) and the CostCritic reweight. Reducing
   the inflation radius is not available here -- it aborts navigation (section 3).
3. **The CostCritic reweight costs ~20% obstacle-free traversal time** and
   makes MPPI reluctant to commit to the tightest pinches from a disturbed
   pose. This is what inflates the reactive success-rate gap (section 9).
4. **MPPI scores a time-agnostic costmap snapshot.** The layer paints the
   obstacle's whole swept future corridor at once, so the controller cannot
   reason "I will be there at t+1.0 s when the obstacle is elsewhere" and
   cannot slip through behind it. The response is avoid-the-whole-corridor,
   i.e. yield. Time-parameterised costing would need a custom critic, which
   Stage-4E deliberately did not write.
5. **Repeated trials of one deterministic simulator scenario are not
   independent samples.** The reported Welch t values describe the separation
   between the arms under startup/timing jitter; they are not population
   inferences.
6. Only one crossing geometry, one obstacle speed (0.25 m/s) and one obstacle
   size are tested. Predictive lead scales with `v_obs * horizon`, so the
   benefit should grow with obstacle speed -- untested.
7. The reactive arm's 60 s timeout is a budget, not a proof that it would
   never recover.
