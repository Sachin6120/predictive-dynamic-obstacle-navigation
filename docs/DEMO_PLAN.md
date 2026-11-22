# Visual demo plan

Target: one **30–60 s** clip for the repository README and portfolio use.

## Status

**Not yet recorded.** This machine has no screen-capture tooling installed
(`ffmpeg`, `Xvfb`, `recordmydesktop`, `byzanz`, ImageMagick `import` are all
absent), so the capture is specified here rather than performed. Nothing in this
plan requires a production source change.

What already exists and is reusable:

| asset | what it shows | reusable? |
|---|---|---|
| `docs/media/predictive_navigation_demo.gif` | perpendicular-crossing trial, production CV configuration, predicted corridor and MPPI avoidance | **yes** — the production predictor is unchanged, so this footage is still architecturally accurate |
| `docs/images/predictive_navigation_hero.png` | RViz: tracked obstacle ID + Kalman velocity, uncertainty corridor, avoidance trajectory | yes |
| `docs/images/gazebo_simulation_environment.png` | the Gazebo arena | yes |

What is missing: any footage of the Stage-5 work — classification, reachability
corridors, and a CV vs reachability side-by-side.

## One constraint worth stating

The RViz track label is built in the tracker as `"ID %u %.2f m/s (KF)"` — it does
**not** carry the class label. Putting PED / BIKE / VEHICLE into the marker text
would mean editing `lidar_obstacle_tracker_node.cpp`, which is frozen production
code. So the class is shown one of two ways, both source-free:

1. a terminal pane beside RViz running the echo command in §2 below, or
2. a text overlay added in post-production.

## Shot list (≈55 s)

| t | shot | source |
|---|---|---|
| 0–8 s | Gazebo: robot and moving obstacle, wide view | §1 |
| 8–18 s | RViz: tracked obstacle, ID and Kalman velocity, class from the echo pane | §1 + §2 |
| 18–30 s | RViz: predicted future occupancy — the CV-Kalman corridor, `Predictive Layer Only (debug)` display | §1 |
| 30–40 s | Nav2 predictive costmap + MPPI candidate and optimal trajectories bending around the prediction | §1 |
| 40–50 s | side-by-side: CV corridor vs global reachability corridor, same condition | §3 |
| 50–55 s | static end card: architecture figure + headline result table | `docs/images/predictive_nav_architecture.svg`, README table |

## Commands

Prerequisite (one install, no repository change):

```bash
sudo apt-get install -y ffmpeg
```

Then, in every shell:

```bash
cd ~/predictive_nav_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### §1 — production CV run, visible, with RViz

```bash
ros2 launch predictive_nav_bringup stage4f_bringup.launch.py \
  headless:=False use_rviz:=True \
  params_file:=$(ros2 pkg prefix predictive_nav_bringup)/share/predictive_nav_bringup/config/production_nav2_params.yaml \
  tracker_params:=$(ros2 pkg prefix predictive_nav_tracking)/share/predictive_nav_tracking/config/production_tracker_params.yaml
```

The shipped `stage4f.rviz` config already carries every display the shot list
needs — `Tracked Dynamic Obstacles`, `Local Costmap (combined)`, `Predictive Layer
Only (debug)`, `MPPI Optimal Trajectory`, `MPPI Candidate Trajectories` — so no
RViz work is required beyond framing the view.

### §2 — classification pane (second terminal)

```bash
ros2 topic echo /tracked_objects --field objects[0].classification
# 0 = UNKNOWN, 1 = PEDESTRIAN, 2 = BIKE, 3 = VEHICLE
```

Confidence, if wanted in the same pane:

```bash
ros2 topic echo /tracked_objects --field objects[0].classification_confidence
```

### §3 — CV vs reachability, same condition

Run the two arms one after the other from the Stage-5E runner, which sets the
tracker config and layer mode for each arm itself:

```bash
python3 validation/stage5e/run_nav5.py \
  --conditions bike_cross --arms cv --trials 1 --out ./demo_cv

python3 validation/stage5e/run_nav5.py \
  --conditions bike_cross --arms global --trials 1 --out ./demo_global
```

The runner launches headless. For a *visible* comparison, launch each arm
directly with its own tracker config and layer mode instead — these are the exact
two values the runner varies, and nothing else changes:

```bash
# CV arm (production predictor)
ros2 launch predictive_nav_bringup stage4f_bringup.launch.py \
  headless:=False use_rviz:=True \
  layer_prediction_mode:=cv_covariance \
  tracker_params:=$PWD/validation/stage5e/stage5e_cv_tracker.yaml

# global reachability arm
ros2 launch predictive_nav_bringup stage4f_bringup.launch.py \
  headless:=False use_rviz:=True \
  layer_prediction_mode:=reachability \
  tracker_params:=$(ros2 pkg prefix predictive_nav_tracking)/share/predictive_nav_tracking/config/stage5d_global_tracker.yaml
```

### §4 — screen capture

```bash
# record a window region at 25 fps (adjust WxH+X+Y to the RViz/Gazebo window)
ffmpeg -y -video_size 1280x720 -framerate 25 -f x11grab -i :1.0+0,0 \
  -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p /tmp/demo_raw.mp4

# trim and assemble the shot list
ffmpeg -y -i /tmp/demo_raw.mp4 -ss 00:00:05 -t 00:00:55 \
  -c:v libx264 -crf 20 -pix_fmt yuv420p docs/media/predictive_navigation_demo_v2.mp4

# GIF for inline README display (keep under ~8 MB)
ffmpeg -y -i docs/media/predictive_navigation_demo_v2.mp4 \
  -vf "fps=12,scale=900:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" \
  docs/media/predictive_navigation_demo_v2.gif
```

Text overlays, if the class labels are burned in rather than shown in a pane:

```bash
ffmpeg -y -i docs/media/predictive_navigation_demo_v2.mp4 \
  -vf "drawtext=text='CV-Kalman predicted corridor':x=24:y=24:fontsize=28:fontcolor=white:box=1:boxcolor=black@0.45" \
  docs/media/predictive_navigation_demo_v2_labelled.mp4
```

## Acceptance checks for the captured clip

- 30–60 s total; readable at 900 px wide.
- The predictive corridor is visibly ahead of the obstacle, not on top of it.
- MPPI trajectories visibly bend before the conflict, not at it.
- No absolute local paths visible in any terminal pane on screen.
- No claim of real-world operation in any overlay text.
- GIF under ~8 MB so GitHub renders it inline.
