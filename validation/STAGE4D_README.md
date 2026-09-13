# Stage-4D validation evidence

Predictive Nav2 costmap layer (`predictive_nav_costmap::PredictedObstacleLayer`).
Extends, and does not modify, the Stage-4B/4C evidence in `README.md` and
`STAGE4C_README.md`. Nothing under `/opt/ros` was changed: Nav2 is
reconfigured only through the project-owned `params_file` launch argument
that `nav2_bringup/tb3_simulation_launch.py` already exposes.

All runs: `ros2 launch predictive_nav_bringup stage4d_bringup.launch.py
headless:=True use_rviz:=False spawn_obstacle:=<True|False>`.

## Layer design summary (see the plugin source for detail)

| Aspect | Choice |
|---|---|
| Base class | `nav2_costmap_2d::CostmapLayer` (owns its own `Costmap2D`, provides `updateWithMax`/`touch`/`matchSize`) |
| Combination | `updateWithMax` -- can only raise master cost, never erase static/voxel/obstacle/inflation |
| Empty value | `NO_INFORMATION`, which under max-combination is a true no-op |
| Spatial cost | `exp(-d²/2)` with `d²` the Mahalanobis distance to the predicted mean, cells kept while `d <= sigma_level` |
| Temporal cost | `exp(-temporal_decay * time_from_now)` |
| Cell cost | `min_cost + spatial * temporal * (max_cost - min_cost)`, clamped to `MAX_NON_OBSTACLE` (252) |
| Plugin order | last, after inflation (see `nav2_predictive_params.yaml` for the rationale) |

## A. Plugin loading

`local_costmap.local_costmap: Using plugin "predicted_obstacle_layer"` ->
`Initialized plugin "predicted_obstacle_layer"`, with the layer reporting
its resolved configuration (`frame=odom rolling=1 sigma=2.00 horizon=3.00s
decay=0.70 cost=[0,200] influence_radius=3.00m`). All ten Nav2 lifecycle
nodes reached `active`.

## B. Empty/static world -- `stage4d_B_static_layer.json`, `stage4d_B_static_tracks.json`

`spawn_obstacle:=False`, 45 s. The layer published 220 debug grids and wrote
**0 predictive cells in every one**. The Stage-4B static regression run
concurrently still reported `false_persistent_tracks: 0` (225 frames). The
master local costmap remained normal (3289/3600 cells > 0, 161 lethal).

## C. Effectively stationary tracked object

Obstacle halted in place by commanding zero velocity (ground-truth speed
0.0000 m/s). Exactly one track, 0.15 m from the model centre (the known
Stage-4B/4C LiDAR-surface-vs-centre bias). All six predictions stayed within
**0.001-0.018 m** of the current filtered position, while covariance still
grew normally (`cov_xx` 0.010 -> 5.21 m² over 0.5-3.0 s). No degenerate or
invalid rasterization: the covariance eigenvalue floor
(`min_position_variance`) keeps a near-deterministic prediction invertible.

## D. Constant-velocity obstacle -- `stage4d_D_dynamic.json`

45 s, obstacle sweeping along x at 0.25 m/s. Predictive cost present in
**224/224 frames**. Peak observed cost 56 on the debug grid's 0-100 scale
= 141 of 252, which is exactly the designed value for the nearest (0.5 s)
prediction at its mean: `0 + 1.0 * exp(-0.7*0.5) * 200 = 141`.

Coordinate correctness (the check §11 asks for):

| metric | mean | max | n |
|---|---|---|---|
| peak predictive cell vs nearest-horizon predicted mean, **synchronized samples with the mean inside the rolling window** | **0.057 m** | 0.123 m | 98 |
| same, all samples | 0.238 m | 1.157 m | 224 |
| region centroid vs mean of all predicted means | 0.911 m | 1.928 m | 224 |

The strict figure (0.057 m ≈ one 0.05 m cell) is the meaningful one. The
looser figures are confounded, and the confound was verified rather than
assumed: every large-offset sample has its predicted mean **outside** the
3 m rolling window, with the brightest visible cell pinned to the window
edge (x = 1.53) and peak cost collapsing to 12-22 because only the weak
far-horizon tail is inside. The region centroid is likewise pulled by the
window clipping the far-horizon ellipses asymmetrically.

## E. Direction reversal -- `stage4d_E_reversal.json`

Captured with the layer's *policy* parameters tightened at runtime
(`max_prediction_horizon:=1.5`, `sigma_level:=1.0`) purely so the corridor
is spatially distinguishable inside the 3 m window; the Stage-4C tracker and
its process noise were **not** touched, and the committed defaults are
unchanged.

Predictive-region x-extent through one reversal (0.2 s per row):

| t (s) | cells | bbox x [min, max] | corridor |
|---|---|---|---|
| 12.86 | 464 | [-0.47, 0.68] | -x |
| 13.26 | 462 | [-0.57, 0.58] | -x |
| 13.46 | 460 | [-0.62, 0.53] | -x |
| 13.66 | 461 | [-0.47, 0.68] | reversing |
| 13.86 | 461 | [-0.27, 0.93] | reversing |
| 14.06 | 463 | [-0.02, 1.13] | +x |
| 14.46 | 460 | [ 0.28, 1.43] | +x |
| 14.66 | 461 | [ 0.38, 1.53] | +x |

The stale trailing edge retracts monotonically (`min_x` -0.62 -> -0.47 ->
-0.27 -> -0.02 -> +0.18 -> +0.38) and is fully gone within ~4 cycles
(0.8 s). Cell count stays flat at ~460 instead of growing, which is the
direct signature that old cells are cleared as new ones are written: a
ghost corridor would both pin `min_x` and inflate the count. The ~0.8 s lag
is the Kalman filter's own post-bounce velocity re-convergence (documented
in `STAGE4C_README.md`), not costmap staleness -- the predictions
themselves still point backwards briefly.

## F. Track expiry -- `stage4d_F_expiry.json`

Obstacle entity removed from Gazebo mid-run
(`gz service -s /world/default/remove`).

- Last frame with predictive cost: 3600 cells.
- **Next costmap cycle: 0 cells** (`clear_latency_after_last_prediction_s:
  0.199` = one 5 Hz cycle).
- End to end, entity removal -> zero predictive occupancy: **~1.4 s**,
  bounded by the configured `track_timeout` (1.0 s) plus one costmap cycle.
- The layer's own freshness gate fired **two cycles before** the tracker
  pruned the track (the tracker was still publishing a track with six
  predictions, but with a frozen observation stamp), i.e. the layer is
  deliberately slightly more conservative than its input.
- `trailing_ghost_cells: 0`; 182 consecutive subsequent frames (~36 s) at
  exactly 0 predictive cells.
- Zero layer warnings were emitted, confirming this was the per-track
  freshness path and not a TF/stale-array fallback.

## G. Plugin disabled at runtime

`ros2 param set /local_costmap/local_costmap predicted_obstacle_layer.enabled
false`, measured in the stable stationary-obstacle scene:

| state | predictive cells | master cells > 0 | master lethal |
|---|---|---|---|
| enabled | 2280 | 3367 | 1108 |
| disabled +3 s | **0** | 3292 | 1112 |
| disabled +7 s | **0** | 3289 | 1110 |
| re-enabled +3 s | 2280 | 3367 | 1100 |

The master returns to exactly its pre-layer cell count and back again:
fully reversible, with lethal cells untouched throughout (criterion 8).
This run also proves `updateBounds`/`updateCosts` *are* invoked on a
disabled layer -- the layer keeps publishing an empty debug grid -- which is
what lets the one-shot clear of the previously-written region run.

Note the layer writes 2280 cells but only raises 78 master cells above
zero: most of the window already carries higher inflation cost. That is
max-combination behaving exactly as intended.

## H. Navigation regression

`NavigateToPose` to (1.0, 0.5) with the layer loaded and actively updating:
**SUCCEEDED**, all ten lifecycle nodes still `active` afterwards, layer
updating at 5.00 Hz throughout. No claim of improved avoidance is made
here -- that is Stage-4E.

## Runtime performance (§11)

| costmap | rate | `updateCosts` mean | max | cells/cycle (max) | max extent |
|---|---|---|---|---|---|
| local (3x3 m, 0.05 m, rolling, odom) | 5.00 Hz | 6.2 us | 26.0 us | 3600 | 4.24 m |
| global (demonstration, map frame, non-rolling) | 1.00 Hz | 18.1 us | 69.0 us | 22730 | 4.28 m |

`updateCosts` is only the max-combination write; the ellipse rasterization
happens in `updateBounds`. Both are far below the 200 ms / 1000 ms cycle
budgets.

## Global-costmap reusability demonstration -- `stage4d_global_layer.json`

Run with `params_file:=.../nav2_predictive_params_global.yaml`, which adds
the same plugin to the global costmap as well. It initialized on the
genuinely different code path (`frame=map rolling=0`: no rolling-origin
update, and no TF because the tracker already publishes in the map frame)
and produced predictive cost in 30/30 frames, ~14757 cells, peak cost 56
(identical to the local instance), max extent 4.30 m unclipped. Strict
coordinate check: **0.046 m mean / 0.078 m max**. `NavigateToPose`
succeeded with both instances loaded. No planner or controller settings
were changed; this is a map-representation demonstration only.

## Known limitations

1. **The default configuration blankets a 3 m local window.** Stage-4C's
   deliberately conservative acceleration noise gives a 2-sigma radius of
   ~4.6 m at t+3 s, which exceeds the 3x3 m local costmap, so at defaults
   the layer writes most of the window (mean 3289 of 3600 cells). The cost
   there is low (temporal weight 0.12 at 3 s) and mostly below existing
   inflation, so only ~78 master cells actually rise. Per the Stage-4D
   brief the Stage-4C process noise was **not** reduced to make this look
   smaller; the policy knobs (`temporal_decay`, `max_prediction_horizon`,
   `sigma_level`, `max_influence_radius`) are the intended levers, and
   Test E shows a tightened policy produces a compact corridor.
2. `max_influence_radius` is a costmap-policy/performance bound, not a
   statistical statement. The published Stage-4C covariance is never
   modified or truncated.
3. Writing predictive cost into `NO_INFORMATION` master cells converts
   unknown space to known-costly (`updateWithMax` semantics). This only
   matters for a costmap with `track_unknown_space: true`, i.e. the global
   demonstration, and is conservative.
4. Static rejection remains sensitive to localization error, a Stage-4B
   property inherited here: during an early exploratory run the obstacle
   was accidentally driven into the robot, displacing it, after which
   distant wall returns survived static rejection and produced several
   stationary false tracks. That run was discarded and the world restarted;
   all results above come from clean runs.
5. Only one costmap integration is validated in depth (local). The global
   configuration is demonstrated, not validated to the same depth.
