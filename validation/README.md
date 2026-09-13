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
