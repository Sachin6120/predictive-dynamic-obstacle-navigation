# Stage-5D section 1 — the classification → reachability boundary

Written **before** any file under `src/` was modified, as section 1 of the
brief requires. Baseline `35f293ac13531b2f975a5ecf91fcb6fd7da569c3`
(`main`, clean, in sync with `origin/main`).

Everything below is read off the shipped code at that commit, or measured from
the retained Stage-5B recordings
(`validation/stage5b/raw/*.jsonl.gz`, 2,547 rows, 57 tracks, 54 scenarios that
produced a track).

---

## 1.1 The runtime path, as actually ordered

One `scan_callback` executes the following, in this order
(`lidar_obstacle_tracker_node.cpp`):

```
/scan (LaserScan, 5 Hz)
  -> transform to map frame, record sensor_origin_
  -> range-aware static-map rejection                        (Stage-4G1)
  -> single-link clustering -> Cluster{points, centroid, diameter}
  -> track-aware deblending                                  (Stage-4G3)
  -> compound-observation stage -> Measurement{centroid, features, compound}
                                                             (Stage-5B.1)
  -> gated nearest-neighbour association
       associated track  : update_track()
                             raw finite-difference baseline
                             Kalman predict + update         (Stage-4C)
                             observations += 1
                             kalman_initialized |= obs >= 2
                             classify_track()                (Stage-5B)
                               classifyInstant()  -> class_instant
                               class_state.update() -> stable label
       unassociated track: missed_count += 1, age += 1,
                             class_features / class_instant cleared,
                             class_state.update(UNKNOWN)  <-- scores DECAY
       unassociated det  : make_new_track()  (class_state = prototype, UNKNOWN)
  -> prune_stale_tracks()
  -> publish_tracks()
       for each track with observations >= min_observations_to_publish (3):
         predict_future()        -> TrackedObjectPrediction[]   (CV, Stage-4C)
         predict_reachability()  -> ReachabilityPrediction[]     (Stage-4G4)
  -> TrackedObjectArray
  -> PredictedObstacleLayer (costmap plugin, prediction_mode)
  -> nav2_costmap_2d local costmap -> MPPI
```

**The decisive ordering fact for Stage-5D:** within one scan,
`classify_track()` runs inside `update_track()`, which runs **before**
`publish_tracks()` calls `predict_reachability()`. So a per-track class label
is already available at the point reachability bounds would be selected, on the
same scan, with no extra plumbing and no one-scan lag. A coasting track's
`class_state.update(UNKNOWN)` also runs before publishing, so its decayed
label is likewise current.

The costmap layer reads only `semi_major`, `semi_minor`, `position`,
`sigma_yaw` and `valid` from `ReachabilityPrediction`
(`predicted_obstacle_layer.cpp:490-530`). It never reads `max_speed`,
`max_acceleration` or `contract_status`. Class-aware bounds are therefore a
**tracker-side** change only; no costmap change is required for the region to
reach the planner.

## 1.2 Exact class enum representation

`ObstacleClass : uint8_t` in `obstacle_classification.hpp:93`:

| name | value | wire constant in `TrackedObject.msg` |
|---|---|---|
| `UNKNOWN` | 0 | `CLASS_UNKNOWN=0` |
| `PEDESTRIAN` | 1 | `CLASS_PEDESTRIAN=1` |
| `BIKE` | 2 | `CLASS_BIKE=2` |
| `VEHICLE` | 3 | `CLASS_VEHICLE=3` |

`kNumSemanticClasses = 3`. The values are declared wire-stable and must not be
renumbered. UNKNOWN is value 0 **and** the default-constructed value, so any
uninitialised or disabled path yields UNKNOWN rather than a semantic class.
A class-aware profile table indexed by this enum therefore has 4 entries and
its index 0 is the fallback — not a sentinel.

## 1.3 Class confidence representation

Two distinct quantities are published:

| field | source | meaning |
|---|---|---|
| `classification_instant_confidence` | `ClassificationResult::confidence` | this scan only: `clamp(best * (1 - second/best), 0, 1)` — the winning box score discounted by the runner-up |
| `classification_confidence` | `ClassLabelStabilizer::confidence()` | the **smoothed** EWMA score of the currently held label, `clamp(scores_[label], 0, 1)`; exactly `0.0` when the held label is UNKNOWN |

The stable pair (`classification`, `classification_confidence`) is the only
pair with per-track hysteresis, so it is the only pair a capability profile
should key on. The instant pair flickers by construction.

Note the structural consequence: **UNKNOWN carries confidence exactly 0.0**,
never a positive "confidence that it is unknown". A policy cannot distinguish
"UNKNOWN because out of range" from "UNKNOWN because ambiguous" from the
confidence number alone; `ClassificationResult::reject_reason` holds that, and
is not published.

## 1.4 When a stable label becomes available; what a new track reports

Gates, in the order they bind:

| gate | value | source |
|---|---|---|
| `min_observations_to_publish` | 3 | track is not published at all below this |
| `min_observations_for_kalman_velocity` | 2 | `kalman_initialized`, and `predict_reachability()` returns empty below it |
| `ClassificationParams::min_observations` | 3 | `classifyInstant` returns UNKNOWN (`track_too_young`) below it |
| `ClassificationParams::min_points` | 5 | UNKNOWN (`too_few_points`) |
| `ClassificationParams::max_range` | 6.0 m | UNKNOWN (`out_of_range`) |
| `min_accept_score` | 0.50 | UNKNOWN (`below_accept_score`) |
| `min_margin` | 0.15 | UNKNOWN (`ambiguous`) |
| `ClassLabelStabilizer::enter_threshold` | 0.45 | stable label stays UNKNOWN below it |

A brand-new track (`make_new_track`) starts with `class_state =
stabilizer_prototype_`, i.e. all four EWMA scores 0.0 and `label_ = UNKNOWN`,
and inherits **nothing** from any previous track. With `smoothing = 0.40`, two
consecutive instant decisions at confidence 1.0 are needed to cross
`enter_threshold` (0.400 then 0.640), and instant decisions only begin at
`observations >= 3`, so the **earliest possible** stable semantic label is at
`observations = 4`.

Measured over the 51 Stage-5B tracks that ever received a semantic label:

| observations at first semantic label | value |
|---|---|
| min | 4 |
| median | **16** |
| p90 | 39 |
| max | 45 |

**So prediction does execute before classification stabilises, routinely.**
Reachability is published from `observations = 3`; the median track is
UNKNOWN for its first 16 observations, i.e. about **3.2 s at 5 Hz**. Any
class-aware design must therefore treat "UNKNOWN because not yet decided" as
the normal early-life state of every track, not an exception.

## 1.5 What UNKNOWN means

UNKNOWN is overloaded, and all three senses arrive on the same wire value:

1. **not yet decided** — track younger than 3 observations, or the stabiliser
   has not reached `enter_threshold` (the dominant case, see 1.4);
2. **cannot be decided** — fewer than 5 beam returns, centroid beyond 6.0 m,
   winning score below 0.50, or winner/runner-up margin below 0.15;
3. **decided not to be any of the three** — e.g. the Stage-5B planter, a real
   obstacle that is genuinely none of PEDESTRIAN/BIKE/VEHICLE (per-track
   UNKNOWN recall 0.917 on it).

Sense 3 is the only one where "unknown" describes the object. Senses 1 and 2
describe the *sensor and the classifier*, and in both the object may well be a
vehicle. Measured: of 1,137 UNKNOWN detections, **382 were ground-truth
VEHICLE** and 197 ground-truth PEDESTRIAN; only 550 were the planter. An
UNKNOWN capability profile that assumed "small slow thing" would therefore be
wrong on a third of its own population.

## 1.6 Whether a class can change during one track, and the hysteresis rules

Yes, and often. Hysteresis (`ClassLabelStabilizer::update`):

* score update `S_c <- (1-a) S_c + a * (c == instant ? conf : 0)`, `a = 0.40`;
* UNKNOWN -> c when `S_c >= enter_threshold` (0.45);
* held c -> UNKNOWN when `S_c < exit_threshold` (0.25);
* held c -> rival d only when `S_d >= 0.45` **and** `S_d >= S_c + switch_margin`
  (0.30). 0.30 strictly exceeds the derived anti-phase bound
  `c*a/(2-a) = 0.25`, so perfectly alternating evidence provably cannot flip
  the label.
* a coasting scan contributes no positive evidence but still decays every
  score, so a label fades rather than persisting on stale evidence.

Measured over the 57 Stage-5B tracks: **115 stable-label transitions in 36 of
57 tracks (63%)**:

| transition | count |
|---|---|
| UNKNOWN -> VEHICLE | 37 |
| VEHICLE -> UNKNOWN | 37 |
| UNKNOWN -> PEDESTRIAN | 22 |
| PEDESTRIAN -> UNKNOWN | 16 |
| BIKE -> PEDESTRIAN | 2 |
| PEDESTRIAN -> BIKE | 1 |

Hysteresis prevents *oscillation between two semantic classes on alternating
scans*; it does not prevent repeated semantic<->UNKNOWN cycling as an object's
aspect and range change, which is what the 74 UNKNOWN<->VEHICLE transitions
are. Section 6 of the brief is therefore addressing a real, frequent event,
not a hypothetical.

## 1.7 How long ephemeral tracks live

Track length in associated-or-coasting scans, over the same 57 tracks:

| quantile | scans | approx seconds at 5 Hz |
|---|---|---|
| min | 2 | 0.4 |
| p10 | 18 | 3.6 |
| median | 41 | 8.2 |
| max | 96 | 19.2 |

Only **1 of 57** tracks lived 5 scans or fewer. Lifecycle bounds:
`max_missed_scans = 5`, `track_timeout = 1.0 s`, `max_observation_age = 1.0 s`
(so a region never outlives its track). Fragmentation is low — median 1 track
per actor, max 3, and only for `actor_bike`.

The practical reading: ephemeral tracks are **not** the dominant
class-aware risk in this dataset. The dominant risk is a long-lived track
holding a *wrong* label (1.8, next).

## 1.8 The classifier error structure a profile policy must survive

Predicted-label purity, per detection, from the Stage-5B recordings:

| predicted | n | ground-truth composition |
|---|---|---|
| PEDESTRIAN | 715 | PEDESTRIAN 381, **VEHICLE 306**, BIKE 28 |
| BIKE | 277 | **BIKE 277 — pure** |
| VEHICLE | 418 | VEHICLE 301, UNKNOWN(planter) 117 |
| UNKNOWN | 1137 | UNKNOWN 550, VEHICLE 382, PEDESTRIAN 197, BIKE 8 |

And the confidence attached to each confusion direction — the number that
decides whether a confidence gate can help at all:

| gt -> predicted | n | min conf | median | max |
|---|---|---|---|---|
| PEDESTRIAN -> PEDESTRIAN | 381 | 0.253 | 0.856 | 1.000 |
| **VEHICLE -> PEDESTRIAN** | **306** | **0.640** | **1.000** | **1.000** |
| VEHICLE -> VEHICLE | 301 | 0.252 | 0.999 | 1.000 |
| BIKE -> BIKE | 277 | 0.348 | 0.960 | 1.000 |
| UNKNOWN -> VEHICLE | 117 | 0.268 | 0.536 | 0.999 |
| BIKE -> PEDESTRIAN | 28 | 0.280 | 0.674 | 0.926 |
| * -> UNKNOWN | 1137 | 0.000 | 0.000 | 0.000 |

**This is the central Stage-5D constraint, and it is measured, not assumed.**
`VEHICLE -> PEDESTRIAN` is *more* confident than `PEDESTRIAN -> PEDESTRIAN`
(median 1.000 vs 0.856; the misclassification's **minimum** confidence, 0.640,
is above the correct classification's median). No threshold on
`classification_confidence` can admit true pedestrians while rejecting
these vehicles. The mechanism is understood and physical: `actor_vehicle_compact`
viewed end-on presents a 0.42-0.45 m long, 0.02-0.03 m thick, fully-filled
slice, which is a pedestrian-sized return in a single horizontal plane
(`metrics.txt` section 5: `actor_vehicle_compact recede -> PEDESTRIAN:175`,
`approach -> PEDESTRIAN:134`). It is the same family of 2D-LiDAR observability
limit already documented for the six head-on bicycle scenarios.

Consequences carried into the design:

* a confidence gate is **necessary but not sufficient**; the PEDESTRIAN
  profile cannot be adopted on label + confidence alone;
* `BIKE` is the one semantic label whose measured precision is 1.000, per
  detection **and** per track, with 0 false BIKE predictions — so it is the
  one label that can carry a profile weaker than the fallback on evidence;
* `-> VEHICLE` admits only VEHICLE and the planter, and `-> UNKNOWN` admits
  vehicles, so both must map to a vehicle-grade envelope;
* 36 of 57 tracks change label, so a transition rule is mandatory.

## 1.9 Current single global configuration, for the record

`ReachabilityBounds` defaults and the shipped YAML agree:
`max_acceleration = 0.5 m/s^2`, `max_speed = 0.8 m/s`,
`covariance_sigma_level = 2.0`, `safety_margin = 0.0 m`,
`max_speed_tolerance = 1e-6 m/s`, `enforce_observed_speed = true`;
`reachability_horizon = 3.0 s`, `reachability_time_step = 0.5 s`,
`max_observation_age = 1.0 s`, so the worst-case total-time window is 4.0 s.

`max_speed` enters the mathematics at exactly one place,
`delta_v_cap = v_max + s0` in `reachRadius()`. `max_acceleration` enters at
`a_max`, in `0.5*a_max*t^2`, in `t_c = delta_v_cap/a_max`, and in
`0.5*a_max*t_c^2`. Both are therefore already single-point parameters of one
pure function — which is why a per-class profile can be introduced without
touching the reachability equation itself.

Production defaults that Stage-5D must not change:
`prediction_mode = cv_covariance` (costmap), `reachability_enabled = false`
(`production_tracker_params.yaml`).
