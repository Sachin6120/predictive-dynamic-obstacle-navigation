# Project summary

A reusable short-form description of this project, plus CV bullets. Every number
is taken from committed evidence under `validation/`.

---

## Summary (≈160 words)

Predictive Dynamic-Obstacle Navigation is a ROS 2 / C++ autonomous navigation
system that anticipates moving obstacles instead of reacting to them. From a
single 2D LiDAR it performs static-map rejection, clustering with
compound-observation recovery, and multi-object tracking with a per-track Kalman
filter, then classifies each obstacle as pedestrian, bicycle, vehicle or unknown
from cluster geometry. Three prediction methods were implemented and compared:
Bayesian constant-velocity Kalman prediction with covariance, global reachability
analysis, and class-aware reachability using per-class capability profiles. Each
publishes an uncertainty-aware movement corridor that is injected into Nav2
through a native C++ `nav2_costmap_2d` plugin, where an MPPI controller plans
against predicted future occupancy rather than last-seen position. A systematic
132-trial comparative campaign across nine scenarios selected the architecture on
evidence: constant-velocity Kalman prediction ships as the production predictor,
global reachability is retained as a secondary representation, and class-aware
reachability — mathematically sound but planner-uninformative at the production
influence window — is documented as a research result. Simulation validated.

---

## CV bullets

- **Developed** a predictive dynamic-obstacle navigation system in C++ for ROS 2 /
  Nav2 that tracks obstacles from 2D LiDAR with per-object Kalman filters,
  classifies them as pedestrian / bicycle / vehicle, and injects uncertainty-aware
  predicted occupancy into the planner through a native `nav2_costmap_2d` plugin
  consumed by an MPPI controller — raising mean obstacle clearance from 0.13 m to
  0.80 m on perpendicular-crossing trials and eliminating 6 of 6 head-on
  collisions, at 25 µs mean layer compute.
- **Implemented and mathematically qualified** three prediction models — Kalman
  constant-velocity with covariance, global reachability, and class-aware
  reachability with per-class capability profiles and a safe fallback policy —
  achieving 1.0000 declared-capability containment for the class-aware predictor
  with zero runtime contract breaches.
- **Validated** the architecture through a 132-trial, three-arm, nine-scenario
  Gazebo campaign judged against a decision rule fixed in writing before the
  results, selecting constant-velocity Kalman prediction for production on
  measured planner-visible structure and cost rather than on an unresolved safety
  difference, and documenting the class-aware negative result (`PROFILE_LIMITED`)
  with its causal mechanism.

---

## Positioning notes

Technical themes this project evidences, for matching against a role description:
autonomous navigation · dynamic obstacle detection · 2D LiDAR perception ·
multi-object tracking · pedestrian / bicycle / vehicle classification ·
Bayesian and Kalman state estimation and prediction · reachability analysis ·
uncertainty-aware movement corridors · ROS 2 · modern C++ · Nav2 costmap plugin
development · MPPI model-predictive control · systematic experimental comparison ·
evidence-based architecture selection.

Claims to avoid making: real-world validation, statistically resolved safety
superiority of any predictor, or zero collisions for the production predictor
(there was one, and it is counted).
