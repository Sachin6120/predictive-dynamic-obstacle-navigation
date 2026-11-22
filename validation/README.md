# Validation evidence

One directory or report per stage. Each is self-contained: a report, the
machine-readable data behind every table, and a reproduction recipe. Nothing here
is transcribed by hand — the tables are generated from the raw trial data by the
analysis scripts committed alongside them.

Read top-down for the final architecture; the Stage-4 reports are the history
that produced it and are unmodified.

## Stage 5 — classification, reachability, and the final architecture decision

| stage | report | what it establishes |
|---|---|---|
| 5B | [`STAGE5B_README.md`](STAGE5B_README.md) | Obstacle classification from 2D LiDAR cluster geometry: confusion matrices, per-class precision/recall/F1, UNKNOWN rate, and the measured geometry behind each confusion. Data: [`stage5b/`](stage5b) |
| 5B.1 | [`stage5b1/CLOSEOUT.md`](stage5b1/CLOSEOUT.md) | Anti-merge replay and single-binary provenance for the classification dataset |
| 5C | [`STAGE5C_README.md`](STAGE5C_README.md) | Mathematical qualification of the reachability predictor, adversarial containment, and the one model-validity boundary. Data: [`stage5c/`](stage5c) |
| 5C.1 | [`stage5c1/STAGE5C1_README.md`](stage5c1/STAGE5C1_README.md) | Capability-contract and raster-fidelity audit of the costmap layer. See also [`stage5c1/CONTRACT_AND_RASTER_AUDIT.md`](stage5c1/CONTRACT_AND_RASTER_AUDIT.md) |
| 5D | [`stage5d/STAGE5D_README.md`](stage5d/STAGE5D_README.md) | Class-aware reachability qualification, plus the [boundary audit](stage5d/CLASS_REACHABILITY_BOUNDARY_AUDIT.md) |
| 5D.1 | [`stage5d1/STAGE5D1_README.md`](stage5d1/STAGE5D1_README.md) | Semantic corridor discriminability audit → diagnosis `PROFILE_LIMITED` |
| 5E | [`stage5e/STAGE5E_README.md`](stage5e/STAGE5E_README.md) | Final three-arm comparative campaign (132 scored trials) and the architecture decision. [`TABLES.md`](stage5e/TABLES.md) · [`DECISION.json`](stage5e/DECISION.json) · [`DECISION_RULE.md`](stage5e/DECISION_RULE.md) |

The Stage-5E raw simulator output (406 files: 135 trial JSONs, 135 series JSONs,
135 per-trial logs) is deliberately **not** tracked in Git. Everything the
analysis depends on is derived into the committed JSON files under
[`stage5e/`](stage5e).

## Stage 4 — tracking, the predictive layer, and the CV-only prediction policy

| stage | report | what it establishes |
|---|---|---|
| 4B | this file, below | First tracking evidence: static rejection, dynamic tracking, track expiry, Nav2 non-interference |
| 4C | [`STAGE4C_README.md`](STAGE4C_README.md) | Per-track constant-velocity Kalman filter, RAW vs KALMAN comparison |
| 4D | [`STAGE4D_README.md`](STAGE4D_README.md) | The uncertainty-aware predictive costmap layer |
| 4E | [`STAGE4E_README.md`](STAGE4E_README.md) | A/B crossing trials; where the 0.55 m influence window comes from |
| 4F | [`STAGE4F_README.md`](STAGE4F_README.md) | Wide-arena reactive-vs-predictive benchmark, 120 trials |
| 4G1 | [`STAGE4G1_README.md`](STAGE4G1_README.md) | Range-aware static-map rejection |
| 4G2 | [`STAGE4G2_README.md`](STAGE4G2_README.md) · [inspection](STAGE4G2_INSPECTION.md) | Multi-object scenes |
| 4G3 | [`STAGE4G3_README.md`](STAGE4G3_README.md) | Track-aware cluster deblending |
| 4G4 | [`STAGE4G4_README.md`](STAGE4G4_README.md) | Bounded-motion reachability model, adversarial containment |
| 4G5 | [`STAGE4G5_README.md`](STAGE4G5_README.md) | Hybrid CV/reachability coasting policy |
| 4G6 | [`STAGE4G6_README.md`](STAGE4G6_README.md) | First KEEP/REMOVE decision benchmark, 365 trials |
| 4G7 | [`STAGE4G7_README.md`](STAGE4G7_README.md) | Blind-corner occlusion decision benchmark, 282 trials |
| 4G8 | [`STAGE4G8_README.md`](STAGE4G8_README.md) | Robustness pass, 448 trials across 46 condition groups |

Plots from the Stage-4F benchmark: [`plots/`](plots).

---

# Stage-4B validation evidence

Both runs used `predictive_nav_bringup stage4b_bringup.launch.py` (headless
gz sim, Stage-4A Nav2 TB3 simulation unmodified, project-owned automatic
`/initialpose` publishing).

## static_test_result.json
`spawn_obstacle:=False` (no moving obstacle at all — pure Stage-4A world),
30 s / 300 scan frames. Verifies the static-rejection filter does not
manufacture persistent dynamic tracks out of walls or other mapped geometry.

Result: `false_persistent_tracks: 0`.

## dynamic_test_result.json
`spawn_obstacle:=True`, obstacle swept along x at y=-0.5 (the robot's own
row, confirmed by map inspection to have unobstructed line-of-sight from the
spawn pose), 60 s / 300 scan frames. Ground truth taken from the obstacle's
own `gz-sim-odometry-publisher-system` plugin
(`/model/dynamic_obstacle/odometry`), matched to the nearest published track
each frame (gate: 1.0 m) via `scripts/evaluate_tracking.py`.

Result: single track ID for the entire run (`id_switches: 0`,
`dominant_track_id_fraction: 1.0`), `mean_position_error_m: 0.149`
(consistent with the ~0.2 m cylinder radius — the tracker centroids the
LiDAR-visible surface, not the model's geometric center, so a bias close to
the radius is expected, not an association error), `mean_velocity_error_mps:
0.029` against a commanded 0.25 m/s.

## Not captured by the JSON files (see handoff for detail)
- **Track expiry**: the obstacle entity was removed mid-run via
  `gz service -s /world/default/remove`; the track's `missed_count` climbed
  and it was pruned ~1.3-1.5 s later, consistent with `track_timeout: 1.0`
  plus one scan period.
- **Nav2 non-interference**: a `navigate_to_pose` goal to (1.0, 0.5) was sent
  while the tracker and moving obstacle were both active; result
  `SUCCEEDED`, and all Nav2 lifecycle nodes remained `active` afterward.
