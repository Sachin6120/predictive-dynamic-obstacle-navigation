# Stage-5B — Dynamic Obstacle Classification

Geometry/motion classification of tracked dynamic obstacles into
`PEDESTRIAN` / `BIKE` / `VEHICLE` / `UNKNOWN`, from a single-plane 2D LiDAR.

**Observation only.** The label is published in new, additive
`TrackedObject.classification*` fields and is read by nothing in the
prediction, reachability or costmap path. Class-aware prediction is a later
stage. That the pre-Stage-5B outputs are *bit-identical* with classification
on and off is proved exactly by
`src/predictive_nav_tracking/test/stage5b_tracker_integration.cpp`.

## What the sensor can actually see

`sensor_msgs/LaserScan` from the TurtleBot3 waffle LDS: **one horizontal
plane ~0.16 m above the floor**, 360 beams over 6.28 rad (0.01749 rad
spacing), 0.01 m range noise, 5 Hz. There is **no height information**, so no
feature in this stage uses any. What the plane cuts is:

| object | what the 0.16 m plane actually intersects |
|---|---|
| pedestrian | shins/ankles — two small blobs, never a torso |
| bicycle | the two wheel *chords*, with the frame passing above the plane |
| vehicle | the lower body panel and tyres |

## Feature vector

All computed per cluster, per scan, relative to that scan's sensor origin.
Definitions and units are in
`include/predictive_nav_tracking/obstacle_classification.hpp`.

| feature | unit | role in the decision |
|---|---|---|
| `length` | m | PCA major-axis extent — absolute size |
| `thickness` | m | PCA minor-axis extent — depth; defines the vehicle corner aspect |
| `fill_ratio` | — | returns / beams crossing the angular span — solidity |
| `max_gap` | m | largest gap between bearing-adjacent returns — internal structure |
| `point_count` | count | evidence gate |
| `range` | m | evidence gate |
| `angular_span` | rad | normalises `fill_ratio` |
| `elongation` | — | **published, diagnostic only, not used in the decision** |
| track speed | m/s | **veto only**, never positive evidence |

`elongation` is excluded from the decision because the minor axis of any flat
or broadside view is zero to within the 0.01 m range noise, so the ratio
degenerates to 20x`length` and adds nothing. Speed is veto-only because the
scenario driver commands every class at 0.15–0.75 m/s, so the measured speed
distributions overlap almost completely — geometry carries the decision.

## Classifier

Deterministic and explainable; no learning of any kind. Each class owns one or
more **aspects** (view-dependent boxes of trapezoidal feature intervals). An
aspect scores the **minimum** over its features (a conjunction: "long AND thin
AND gapped"); a class scores the **maximum** over its aspects (a disjunction:
a car seen along its flank and at its corner are two different measurements).
A label is accepted only when there is enough evidence, the winner is decisive
(`min_accept_score`), and it beats its rival clearly (`min_margin`) — otherwise
`UNKNOWN`. Every threshold is an interval on a physical dimension with its
derivation recorded at the point of definition.

Temporal stability is an exponentially-weighted score per class with
hysteresis. `switch_margin` is **derived, not tuned**: under perfectly
alternating evidence the two scores settle `c*alpha/(2-alpha)` apart, which is
0.25 at `alpha = 0.4` and `c = 1`, so a required lead of 0.30 makes
oscillation provably impossible at any confidence.

## Ground truth

Comes from **model identity only**. `scripts/stage5b/actors.py` is the single
source of truth: it emits both the Gazebo SDF the simulator spawns and the
ground-truth class the evaluation scores against, so geometry and label cannot
drift apart. Classifier output is never relabelled as ground truth.

| actor | ground truth | footprint diagonal |
|---|---|---|
| `actor_pedestrian` | PEDESTRIAN | 0.488 m |
| `actor_bike` | BIKE | 1.620 m |
| `actor_vehicle` | VEHICLE | 4.163 m |
| `actor_vehicle_compact` | VEHICLE | 1.118 m |
| `actor_planter` | UNKNOWN | 1.273 m |

`actor_planter` is a non-target obstacle, included so the `UNKNOWN` rate is
*measured* against something rather than asserted.

## Reproducing

The classification set runs on a **separate tracker profile**,
`config/stage5b_classification_tracker.yaml`, which differs from the frozen
`tracker_params.yaml` in **exactly one value**: `cluster_max_diameter`
1.2 → 5.0 m. The frozen value rejects any cluster whose bounding-box diagonal
exceeds 1.2 m, which is smaller than the scan-plane signature of a bicycle or
a full-size vehicle — under it those classes are not misclassified, they are
*invisible*. The navigation integration check runs on the **frozen** profile.

```bash
cd ~/predictive_nav_ws && source install/setup.bash

# 1. regenerate the actor models from the single source of truth
python3 src/predictive_nav_bringup/scripts/stage5b/actors.py \
        --out src/predictive_nav_bringup/models/stage5b

# 2. list the validation set (5 actors x 4 patterns x 3 speeds)
python3 src/predictive_nav_bringup/scripts/stage5b/scenarios.py

# 3. record it, one launch per actor (~6 min each)
for a in actor_pedestrian actor_bike actor_vehicle \
         actor_vehicle_compact actor_planter; do
  python3 src/predictive_nav_bringup/scripts/stage5b/run.py \
          --actor $a --out validation/stage5b/raw/$a.jsonl
done

# 4. metrics: confusion matrices, per-class P/R/F1, UNKNOWN rate, failures
#    (reads .jsonl and .jsonl.gz alike; the committed dataset is gzipped,
#     which reproduces metrics.txt byte-for-byte)
python3 src/predictive_nav_bringup/scripts/stage5b/analyze.py \
        --raw validation/stage5b/raw --out validation/stage5b/metrics.txt

# 5. one navigation scenario on the FROZEN tracker profile
python3 src/predictive_nav_bringup/scripts/stage5b/integration_check.py \
        --out validation/stage5b/integration
```

The robot is **stationary** for the classification set. A driving robot adds
AMCL pose error (Stage-4G1 measured p95 0.118 m while moving) which leaks into
static rejection as false dynamic returns; holding it still makes a
misclassification attributable to the classifier and the sensor rather than to
localisation drift. The moving case is exercised by step 5.

`stage5b/fillratio_defect_evidence.txt` is the retained evidence from the
first recording, made before the `fill_ratio` clamp described at
`ClusterFeatures::fill_ratio` was applied: 977 of 2816 detections reported a
solidity above 1.05, which is impossible by definition, and the measured
angular-span deficit of 0.80 increments matches the near-duplicate beam pair
exactly. The reported metrics come from a clean re-recording made **after**
that fix; the superseded raw JSONL was not kept.

## Results

Full numbers in `stage5b/metrics.txt`; the navigation integration check in
`stage5b/integration/integration_check.json`.

**Headline (per stable track, 60 tracks over 54 detected scenarios):**

| class | precision | recall | F1 | tracks |
|---|---|---|---|---|
| PEDESTRIAN | 0.280 | 0.583 | 0.378 | 12 |
| BIKE | n/a | **0.000** | n/a | 12 |
| VEHICLE | 1.000 | 0.458 | 0.629 | 24 |
| UNKNOWN | 0.500 | 1.000 | 0.667 | 12 |

UNKNOWN rate 0.400 per track. Navigation integration check: **PASS** on all
nine requirements (goal SUCCEEDED, no collision, min clearance 0.794 m).

### The three limitations that dominate these numbers

1. **BIKE is unreachable, not misclassified.** `cluster_distance` is 0.35 m,
   but the empty span between a bicycle's two wheel chords at the 0.16 m scan
   plane is ~0.48 m. Single-link clustering therefore **splits every bicycle
   into two separate tracks** (measured: median 2 tracks per bike scenario) of
   ~0.51 m each, which are geometrically indistinguishable from a pedestrian's
   legs — hence 576/582 bike detections labelled PEDESTRIAN. Head-on, the two
   wheels self-occlude into one 0.04 m-wide target and the bicycle is not
   detected at all (6 of 60 scenarios). Raising `cluster_distance` above
   0.48 m would re-break the Stage-4G3 deblending case, where two objects
   0.46–0.67 m apart already merge into one component, so it was **not**
   changed.
2. **An object's 2D slice, not its identity, is what is observable.** A
   0.5 m-wide compact vehicle seen end-on (`len` 0.45 m) is the same slice as
   a pedestrian, and is labelled PEDESTRIAN on 174/174 such detections. A
   0.9 m round planter at a corner aspect (`thick` 0.25–0.37 m) satisfies the
   vehicle corner box on 174/670 detections. Neither is a threshold error;
   both are genuinely ambiguous to a single horizontal scan line.
3. **Range bounds pedestrian classification.** The UNKNOWN rate for true
   pedestrians rises 0.00 (0–2 m) → 0.41 (3–4 m) → 0.81 (4–5 m) → 1.00 (5 m+),
   because two 0.12 m shins subtend fewer than `min_points` beams at 1 deg
   angular resolution. UNKNOWN is the correct answer there, and the classifier
   returns it rather than guessing.

**Speed is not a usable discriminator in this dataset** and is reported as
such: the measured per-class medians are 0.25 / 0.24 / 0.22 / 0.23 m/s —
effectively identical, because the scenario driver commands every class over
the same 0.25–0.75 m/s band. Geometry carries every decision.

A bound worth revisiting next stage, identified but deliberately **not**
retuned post-hoc on this data: the vehicle flank aspect requires
`length >= 1.80 m`, while a 1.7 m-wide car seen end-on measures 1.66 m, so
156 of 194 such detections go UNKNOWN. 1.80 m was chosen to sit above a
bicycle's apparent length, but on ray-cast reference geometry a whole
bicycle measures 1.55-1.60 m, so the margin is larger than it needs to be.
The separator that does not depend on absolute length is `max_gap`: a car
end-on measures 0.06-0.07 m here, and a whole bicycle 0.52 m on reference
geometry. Note this is a prediction, not a measurement from this dataset —
no bicycle was ever clustered whole in it (see limitation 1), so its
*fragments* measure `max_gap` 0.07 m, the same as a car end-on.
