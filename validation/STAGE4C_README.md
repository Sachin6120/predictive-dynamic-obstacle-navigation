# Stage-4C validation evidence

Extends (does not replace) `README.md` (Stage-4B). Both runs used
`predictive_nav_bringup stage4b_bringup.launch.py` (headless gz sim, Stage-4A
Nav2 TB3 simulation unmodified, project-owned automatic `/initialpose`
publishing). No parameters were changed between the Stage-4B baseline run
and these Stage-4C runs other than adding the new `kf_*`/`prediction_*`
keys to `tracker_params.yaml` (Stage-4B's own parameters are untouched).

## stage4c_static_result.json

`spawn_obstacle:=False`, 60 s / 301 scan frames. Regression check that
Stage-4C's Kalman/prediction additions did not reintroduce false persistent
dynamic tracks from static geometry.

Result: `false_persistent_tracks: 0` (Stage-4B: 0/300; Stage-4C: 0/301 --
statistically equivalent, frame-count difference is scan-timing jitter).

## stage4c_dynamic_result.json

`spawn_obstacle:=True`, obstacle bouncing along x between x=-1.5 and
x=0.5 at y=-0.5, speed 0.25 m/s (`obstacle_mover.py`, unchanged from
Stage-4B). 90 s / 450 scan frames, captured via
`scripts/evaluate_stage4c.py --duration 90 --motion-axis x`. Ground truth:
`/model/dynamic_obstacle/odometry` (simulator-only, never fed to the
tracker). 10 direction reversals occurred during the run, giving both a
"constant-velocity segment" evaluation (Experiment B, the `steady_state`
breakdown) and a "full bounce trajectory" evaluation (Experiment C, the
unsplit / `all` and `near_reversal` breakdowns) from a single recorded run,
consistent with §7's request to evaluate both without introducing separate
artificial-noise runs.

### Current-state tracking: RAW vs KALMAN

| | RAW position MAE | KALMAN position MAE | RAW velocity MAE | KALMAN velocity MAE |
|---|---|---|---|---|
| all (450 frames) | 0.1474 m | 0.1475 m | 0.0321 m/s | 0.0309 m/s |
| steady_state (400) | 0.1471 m | 0.1471 m | 0.0156 m/s | 0.0150 m/s |
| near_reversal (50) | 0.1502 m | 0.1512 m | 0.1642 m/s | 0.1584 m/s |

- ID switches: 0. Dominant-track fraction: 1.0 (single stable ID for the
  whole run, matching Stage-4B).
- Position MAE is essentially identical RAW vs KALMAN (~0.147 m either
  way): this is the expected, previously-documented LiDAR-surface-vs-model-
  center geometric bias, and a Kalman filter on noisy-but-unbiased
  measurements cannot and should not remove a *systematic* bias -- per the
  task instructions this was **not** compensated in either baseline.
- Velocity MAE improves modestly with the Kalman filter in both regimes
  (~4% steady-state, ~4% near-reversal), consistent with a filter that
  smooths per-scan measurement noise but cannot outrun a genuine,
  instantaneous direction reversal any faster than the raw 2-point
  finite-difference does -- neither estimator has a way to "know" about a
  reversal before it happens.

### Future trajectory prediction: ADE / FDE by horizon

Position error at each fixed horizon `h`, pooled across all prediction
events made at that horizon distance from their (later-arriving) ground
truth:

| horizon | all: MAE / RMSE (n) | near_reversal: MAE / RMSE (n) |
|---|---|---|
| 0.5 s | 0.152 / 0.159 m (448) | 0.171 / 0.197 m (50) |
| 1.0 s | 0.172 / 0.195 m (446) | 0.234 / 0.275 m (50) |
| 2.0 s | 0.256 / 0.343 m (441) | 0.379 / 0.464 m (50) |
| 3.0 s | 0.394 / 0.557 m (436) | 0.530 / 0.666 m (50) |

Classic ADE/FDE per full predicted 6-point trajectory (0.5 - 3.0 s):

| | n trajectories | ADE mean | FDE mean |
|---|---|---|---|
| all | 448 | 0.248 m | 0.388 m |
| steady_state | 398 | 0.236 m | 0.370 m |
| near_reversal | 50 | 0.346 m | 0.530 m |

Error grows monotonically with horizon (expected for a constant-velocity
model with propagated process noise), and is consistently worse for
trajectories predicted while a direction reversal happened within the
previous `--reversal-window` (default 1.0 s) than during steady motion --
this is the expected, and intentionally *not hidden*, failure mode of a
constant-velocity model across an abrupt bounce. `near_reversal` accounts
for 50/448 (~11%) of scored trajectories, consistent with 10 reversal
events x a ~1 s window x ~5 Hz scan rate.

### Uncertainty growth (spot check, not in the JSON files)

Sampled directly from one track's published `covariance`/`predictions` at
the default `kf_process_accel_noise=0.5`: position covariance `cov_xx`
grows 0.013 -> 0.088 -> 0.368 -> 1.086 -> 2.570 -> 5.244 m^2 at
t+0.5..3.0 s, i.e. monotonically and consistent with the closed-form CWNA
Q(dt) (dominated by the `q*dt^4/4` term at long horizons). The
corresponding 2-sigma uncertainty-ellipse marker grows from a 0.46 m to a
9.2 m diameter over the 3 s horizon. See "Known limitations" in the
handoff: this is a mathematically correct consequence of the chosen
(un-tuned, generic) acceleration-noise default, not a bug, but it is a
large region relative to this test arena's ~2 m obstacle travel range.

### Navigation regression (Experiment D)

A `navigate_to_pose` goal to (1.0, 0.5) was sent while the Kalman
tracker/predictor and the bouncing obstacle were both active. Result:
`SUCCEEDED`. All ten Nav2 lifecycle nodes (`map_server`, `amcl`,
`controller_server`, `planner_server`, `behavior_server`, `bt_navigator`,
`waypoint_follower`, `velocity_smoother`, `collision_monitor`,
`docking_server`) remained `active` throughout and after the goal.
