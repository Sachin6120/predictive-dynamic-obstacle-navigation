# STAGE-5D.1 — SEMANTIC CORRIDOR DISCRIMINABILITY AUDIT

**Repository** `~/predictive_nav_ws`
**Branch** `main`
**HEAD** `755d7081b5475b9a9e4340f14af9ab4c1bada171` (== `origin/main`)
**Working tree at start** clean

**Scope.** Diagnosis only. No production parameter, policy, default or source
file is changed. Nothing is committed or pushed. Stage-5E is not started.

---

## 0. The question Stage-5D left open

Stage-5D qualified class-aware reachability mathematically
(`QUALIFIED_CLASS_AWARE_NO_PLANNER_BENEFIT`) but could not show that *semantic
class discrimination itself* changes planner behaviour. Two candidate causes
were named, and this stage separates them:

* **the 0.55 m `max_influence_radius` costmap policy window**, which every
  class profile saturates within `total_time` 0.34–0.67 s;
* **classifier ambiguity / conservative fallback policy**, under which 381
  correctly-labelled PED→PED tracks received a 40.0 m/s envelope rather than
  the 2.5 m/s pedestrian profile.

The instrument is a controlled four-arm comparison — class source crossed with
policy window — run offline over one fixed set of recorded tracks and then in
a bounded Gazebo campaign.

---

## 1. `max_influence_radius`: what it is, and where 0.55 m came from

### 1.1 What it clamps

Post-Stage-5C.1 the parameter clamps the **rasterization iteration window**,
not the geometry (`predicted_obstacle_layer.cpp:563-566` for reachability,
`:657-660` for CV):

```cpp
if (max_influence_radius_ > 0.0) {
  half_x = std::min(half_x, max_influence_radius_);
  half_y = std::min(half_y, max_influence_radius_);
}
```

Membership is then tested against the semi-axes **as published**
(`:596-600`), so the planner-visible region is exactly
`ellipse ∩ box(±max_influence_radius)`. It is therefore a bound on

* **raster iteration window** — yes, directly: it sets `min_i/max_i`,
  `min_j/max_j`;
* **planner-visible extent** — yes, as a consequence: cells outside the box
  are never written, whatever the published region says;
* **CPU cost** — yes, as a consequence: the loop is `O(window cells)`;
* **prediction geometry** — **no.** The published `semi_major`/`semi_minor`
  on `/tracked_objects` are untouched, and the membership test uses them
  unclamped. This was the Stage-5C.1 fidelity fix: before it, the clamp was
  applied to `semi_major`/`semi_minor` themselves, which made the region a
  *smaller ellipse* rather than the published ellipse clipped to the window.

### 1.2 Historical basis — **PROVEN**, not assumed

The parameter was introduced at **Stage-4D** (`fa2bcf9`,
`predicted_obstacle_layer.cpp`) with

```cpp
declareParameter("max_influence_radius", rclcpp::ParameterValue(3.0));
```

i.e. a **declared default of 3.0 m**, documented there as a
"costmap-policy/performance bound … NOT a statistical statement about the
prediction" (`:649-655`).

It was set to **0.55 m at Stage-4E** (`d9a93ef`). The derivation is recorded
in the repository in three independent places and is arithmetic, not a tuned
constant:

| source | statement |
|---|---|
| `nav2_stage4f_params.yaml:340-346` | "0.55 m: robot_radius (0.22) + obstacle radius (0.20) + 0.13 m margin, i.e. the radius within which the two bodies would actually conflict. This is the compactness lever … (measured: ~18% of the local window, versus 55% at Stage-4D defaults)." |
| `validation/STAGE4E_README.md:122` | "robot_radius 0.22 + obstacle radius 0.20 + 0.13 margin … This is the compactness lever; measured window coverage **20.9%** vs 55% at Stage-4D defaults. Still a costmap-policy bound, never a truncation of the published covariance." |
| `validation/STAGE4F_README.md:119` | "`max_influence_radius` \| **0.55 m** \| robot radius 0.22 + obstacle radius 0.20 + 0.13 margin" |

`0.22 + 0.20 + 0.13 = 0.55`. The historical reason is therefore a **conflict
geometry argument plus a measured compactness target**, and secondarily a
performance bound. It is *not* a safety policy (the published region is
unchanged), and it is *not* an arbitrary tuning constant.

Also recorded at Stage-4D, and relevant to the diagnosis below:
`validation/STAGE4D_README.md:185` — "`max_influence_radius` is a
costmap-policy/performance bound, **not** a [statistical statement]".

### 1.3 Semantic effect

None of the three recorded justifications considered semantic class, because
no classifier existed before Stage-5B. The clamp was sized for **one** object
geometry (0.20 m radius) and **one** global capability premise
(`max_speed 0.8`, `max_acceleration 0.5`), under which it binds at 1.483 s and
saturates at 1.764 s — i.e. late in a 3.0 s horizon. Section 3 shows that
under the Stage-5D class profiles the same 0.55 m binds at 0.33–0.66 s, so the
window that was late for the global premise is early for every class premise.

---

## 2. The four arms

|  | current window 0.55 m | expanded window 3.00 m |
|---|---|---|
| **runtime classifier** | **A** (= Stage-5D behaviour) | **C** |
| **ground-truth class** | **B** *(evaluation only)* | **D** *(evaluation only)* |

Two supplementary arms, **B2** and **D2**, push the ground-truth label through
the **unmodified shipped selector** (so it still receives the closure
envelope — this is exactly Stage-5D's arm C). Reporting them alongside B and D
separates the effect of the *ambiguity policy* from the effect of *label
error* without weakening the policy anywhere.

**Oracle definitions.** B/D use the **oracle-ideal** profile: the class's own
capability entry with a singleton ambiguity set, because a perfect classifier
has nothing to hedge against. True `UNKNOWN` maps to the derived fallback
envelope, since an UNKNOWN object's true capability is by definition not
known — the same `truthProfile` rule Stage-5D used.

**Oracle discipline.** Offline, `Row::gt_label` is read only by arms B, D, B2,
D2 and the penalty analysis. In the navigation campaign the oracle arms load
an `oracle_*_tracker.yaml` that is `stage5d_global_tracker.yaml` with **only
`max_speed` and `max_acceleration` changed**; `reachability_class_aware` stays
**false**, so the node takes the bit-identical Stage-5C.1 global path and has
**no class input at all**. Which file to load is chosen *outside the robot* by
the campaign runner, from the scenario it is itself spawning. No label, no
classifier output and no ground-truth field crosses into the tracker, the
costmap layer, the planner or the controller.

The oracle arms are an **upper-bound measurement, not a deployable
configuration**: a real robot does not know the class of an obstacle it has
not classified.

---

## 3. Expanded window design

### 3.1 Two thresholds, both geometric

For a window of half-extent `R` and a region of radius `r` (the reachability
region is a **disc** — the Stage-4C covariance is isotropic in position, so
`semi_major == semi_minor` at every horizon, see T3):

* **binds** at `r ≥ R` — the clamp starts truncating the iteration window;
* **saturates** at `r ≥ R·√2` — the whole clamped box lies inside the region,
  so **every** window cell is painted and two different classes produce
  **byte-identical rasters**. This is the precise condition under which the
  planner cannot tell two classes apart.

### 3.2 The sweep
### T1 radius sweep


| radius m | window cells | frac of local costmap | PED binds/sat s | BIKE binds/sat s | VEH binds/sat s | UNKNOWN binds/sat s |
|---|---|---|---|---|---|---|
| **0.55** | 23x23 = 529 | 0.0367 | 0.663 / 0.789 | 0.396 / 0.471 | 0.332 / 0.394 | 0.332 / 0.394 |
| **1.00** | 41x41 = 1681 | 0.1167 | 0.894 / 1.064 | 0.534 / 0.636 | 0.447 / 0.532 | 0.447 / 0.532 |
| **1.50** | 61x61 = 3721 | 0.2584 | 1.095 / 1.307 | 0.655 / 0.778 | 0.548 / 0.651 | 0.548 / 0.651 |
| **2.00** | 81x81 = 6561 | 0.4556 | 1.267 / 1.543 | 0.756 / 0.899 | 0.632 / 0.752 | 0.632 / 0.752 |
| **3.00** | 121x121 = 14641 | 1.0167 | 1.600 / 2.014 | 0.926 / 1.101 | 0.775 / 0.921 | 0.775 / 0.921 |
| **5.00** | 201x201 = 40401 | 2.8056 | 2.267 / 2.957 | 1.195 / 1.421 | 1.000 / 1.189 | 1.000 / 1.189 |

### T2 r_det by horizon, s0 = 0.50 m/s


| horizon s | PEDESTRIAN | BIKE | VEHICLE | UNKNOWN |
|---|---|---|---|---|
| 0.5 | 0.312 | 0.875 | 1.250 | 1.250 |
| 1.0 | 1.250 | 3.500 | 5.000 | 5.000 |
| 1.5 | 2.700 | 7.875 | 11.250 | 11.250 |
| 2.0 | 4.200 | 13.839 | 20.000 | 20.000 |
| 2.5 | 5.700 | 20.089 | 31.250 | 31.250 |
| 3.0 | 7.200 | 26.339 | 45.000 | 45.000 |

### 3.3 The chosen value: 3.00 m, and why it is not a tuned number

`max_influence_radius = 3.00 m` for the analysis arms. Three independent
reasons, none of which is a navigation metric:

1. **It is the layer's own declared default** —
   `declareParameter("max_influence_radius", rclcpp::ParameterValue(3.0))`,
   `predicted_obstacle_layer.cpp:44`, and the value still in
   `nav2_predictive_params.yaml:243`. No new number is introduced.
2. **It is the geometric ceiling of this costmap.** The local costmap is
   6 × 6 m at 0.05 m (`nav2_stage4f_params.yaml:256-258`), so its half-extent
   is exactly 3.0 m. A 3.0 m window is 121 × 121 = 14 641 cells against a
   14 400-cell costmap: **101.7%**. Anything larger cannot be represented at
   all — `worldToMapEnforceBounds` clips it. 5.0 m would be 281% of the
   costmap and is listed in T1 only to show that it buys nothing.
3. **It keeps the discrimination that exists.** At 3.0 m, PEDESTRIAN stays
   non-saturating to 2.01 s, BIKE to 1.10 s and VEHICLE/UNKNOWN to 0.92 s, so
   class geometry survives over the first half of the 3.0 s horizon (T8).

Computational tractability is the stated bound and it is already strained:
section 9 measures the cost.

### 3.4 **No single radius separates all four classes across the horizon** — documented, as the brief requires

This is a hard geometric fact, not a tuning failure. At 3.0 s the corridors
are 7.20 / 26.34 / 45.00 / 45.00 m (T2). For PEDESTRIAN to remain
non-saturating at 3.0 s one needs `R ≤ 7.20/√2 = 5.09 m`; at that radius BIKE
(26.3 m) and VEHICLE (45.0 m) are both saturated many times over, and the
window is 281% of the entire local costmap. Conversely any radius small
enough to be affordable saturates PEDESTRIAN early.

The reason is that the class profiles span **16×** in `max_speed`
(2.5 → 40.0) and **4×** in `max_acceleration` (2.5 → 10.0), while the costmap
spans 6 m. Over a 3 s horizon those premises generate regions **one to two
orders of magnitude larger than the map they must be drawn on**. Reporting
that is part of the result.

### 3.5 A structural limit no radius can repair

| predicted label | ambiguity closure | envelope `max_speed` / `max_accel` | equals fallback? |
|---|---|---|---|
| UNKNOWN | {unknown, ped, bike, vehicle} | 40.0 / 10.0 | yes |
| PEDESTRIAN | {ped, bike, vehicle} | 40.0 / 10.0 | **yes** |
| BIKE | {bike} | 12.0 / 7.0 | no |
| VEHICLE | {vehicle, unknown} | 40.0 / 10.0 | **yes** |

Under the **shipped** ambiguity policy the runtime system can express exactly
**two** distinct corridors — BIKE and the fallback — and `VEHICLE` vs
`UNKNOWN` is **identical by construction at every radius**. The oracle-ideal
policy can express **three** (PED, BIKE, VEH≡UNKNOWN-fallback).

This ceiling is independent of `max_influence_radius`, and it is why BIKE is
the only class Stage-5D saw receive corridor reduction.

---

## 4. Profile-separation analysis

Method: every class profile is evaluated on the **same recorded track state,
the same Kalman covariance and the same horizon**, so the only difference
between classes is the capability profile. Cell counts replicate
post-Stage-5C.1 `rasterizeReachableSet` exactly. 2 547 rows, 57 tracks,
15 282 (row × horizon) samples per arm.
### T3 published semi-axis m (region is a disc: major == minor)


| horizon s | PEDESTRIAN | BIKE | VEHICLE | UNKNOWN |
|---|---|---|---|---|
| 0.5 | 0.5935 | 1.1560 | 1.5310 | 1.5310 |
| 1.0 | 1.9410 | 4.1910 | 5.6910 | 5.6910 |
| 1.5 | 3.9745 | 9.2211 | 12.5961 | 12.5961 |
| 2.0 | 6.2836 | 16.0418 | 22.2480 | 22.2480 |
| 2.5 | 8.8396 | 23.3478 | 34.6467 | 34.6467 |
| 3.0 | 11.6427 | 30.9009 | 49.7926 | 49.7926 |

### T4 continuous region area m^2


| horizon s | PEDESTRIAN | BIKE | VEHICLE | UNKNOWN |
|---|---|---|---|---|
| 0.5 | 1.33 | 4.42 | 7.59 | 7.59 |
| 1.0 | 12.67 | 56.02 | 102.58 | 102.58 |
| 1.5 | 51.22 | 268.80 | 500.13 | 500.13 |
| 2.0 | 126.50 | 811.03 | 1557.65 | 1557.65 |
| 2.5 | 248.89 | 1716.02 | 3774.83 | 3774.83 |
| 3.0 | 430.25 | 3004.20 | 7793.65 | 7793.65 |

### T5 rasterized cells at the 0.55 m PRODUCTION window


| horizon s | PEDESTRIAN | BIKE | VEHICLE | UNKNOWN |
|---|---|---|---|---|
| 0.5 | 384.03 | 529.00 | 529.00 | 529.00 |
| 1.0 | 529.00 | 529.00 | 529.00 | 529.00 |
| 1.5 | 529.00 | 529.00 | 529.00 | 529.00 |
| 2.0 | 529.00 | 529.00 | 529.00 | 529.00 |
| 2.5 | 529.00 | 529.00 | 529.00 | 529.00 |
| 3.0 | 529.00 | 529.00 | 529.00 | 529.00 |

### T6 rasterized cells at the 3.00 m ANALYSIS window


| horizon s | PEDESTRIAN | BIKE | VEHICLE | UNKNOWN |
|---|---|---|---|---|
| 0.5 | 537.41 | 1771.78 | 3023.08 | 3023.08 |
| 1.0 | 4608.99 | 14586.24 | 14641.00 | 14641.00 |
| 1.5 | 14346.21 | 14641.00 | 14641.00 | 14641.00 |
| 2.0 | 14641.00 | 14641.00 | 14641.00 | 14641.00 |
| 2.5 | 14641.00 | 14641.00 | 14641.00 | 14641.00 |
| 3.0 | 14641.00 | 14641.00 | 14641.00 | 14641.00 |

### T7 saturation rate at 0.55 m


| horizon s | PEDESTRIAN | BIKE | VEHICLE | UNKNOWN |
|---|---|---|---|---|
| 0.5 | 0.0448 | 1.0000 | 1.0000 | 1.0000 |
| 1.0 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| 1.5 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| 2.0 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| 2.5 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| 3.0 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |

### T8 saturation rate at 3.00 m


| horizon s | PEDESTRIAN | BIKE | VEHICLE | UNKNOWN |
|---|---|---|---|---|
| 0.5 | 0.0000 | 0.0000 | 0.0000 | 0.0000 |
| 1.0 | 0.0224 | 0.0667 | 1.0000 | 1.0000 |
| 1.5 | 0.0448 | 1.0000 | 1.0000 | 1.0000 |
| 2.0 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| 2.5 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| 3.0 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |

### 4.1 Reading T5 and T6 — the single most important result of this stage

At the **production 0.55 m window** (T5), from horizon **1.0 s onward every
class paints exactly 529 cells**. Not approximately: identically, on every one
of the 15 282 samples. The only semantic separation anywhere in the whole
6-horizon window is PEDESTRIAN at 0.5 s (384.03 cells vs 529), present on
95.5% of samples.

The continuous geometry those rasters come from differs by **7.3× to 18.1× in
area** (T4). All of it is erased by the clamp.

At the **3.00 m analysis window** (T6) separation reappears at 0.5 s
(537 / 1 772 / 3 023 / 3 023), at 1.0 s (4 609 / 14 586 / 14 641 / 14 641) and
marginally at 1.5 s (14 346 / 14 641 / …), and is gone again from 2.0 s on.

Note what the 3.00 m numbers mean physically: **14 641 cells is 101.7% of the
local costmap**. From 1.0 s onward the VEHICLE/fallback corridor is not a
corridor, it is a blanket over the entire local window.

### 4.2 Pairwise distinguishability
### T9 pairwise raster separation: rate the two classes paint a DIFFERENT cell count


| pair | 0.5 s | 1.0 s | 1.5 s | 2.0 s | 2.5 s | 3.0 s |
|---|---|---|---|---|---|---|
| PED_vs_BIKE | 0.955 -> 1.000 | 0.000 -> 0.978 | 0.000 -> 0.955 | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 |
| PED_vs_VEHICLE | 0.955 -> 1.000 | 0.000 -> 0.978 | 0.000 -> 0.955 | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 |
| BIKE_vs_VEHICLE | 0.000 -> 1.000 | 0.000 -> 0.933 | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 |
| VEHICLE_vs_UNKNOWN | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 |
| PED_vs_UNKNOWN | 0.955 -> 1.000 | 0.000 -> 0.978 | 0.000 -> 0.955 | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 |
| BIKE_vs_UNKNOWN | 0.000 -> 1.000 | 0.000 -> 0.933 | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 | 0.000 -> 0.000 |

(each cell reads `rate at 0.55 m -> rate at 3.00 m`)

### T10 pairwise mean |cell difference|, and continuous area ratio


| pair | 0.5 s | 1.0 s | 1.5 s | 2.0 s | 2.5 s | 3.0 s | area ratio 0.5s | area ratio 3.0s |
|---|---|---|---|---|---|---|---|---|
| PED_vs_BIKE | 145 -> 1234 | 0 -> 9977 | 0 -> 295 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0.2545 | 0.1418 |
| PED_vs_VEHICLE | 145 -> 2486 | 0 -> 10032 | 0 -> 295 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0.1460 | 0.0548 |
| BIKE_vs_VEHICLE | 0 -> 1251 | 0 -> 55 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0.5649 | 0.3850 |
| VEHICLE_vs_UNKNOWN | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 1.0000 | 1.0000 |
| PED_vs_UNKNOWN | 145 -> 2486 | 0 -> 10032 | 0 -> 295 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0.1460 | 0.0548 |
| BIKE_vs_UNKNOWN | 0 -> 1251 | 0 -> 55 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0 -> 0 | 0.5649 | 0.3850 |

`VEHICLE_vs_UNKNOWN` is **0.000 at every horizon and every radius**, with a
continuous area ratio of exactly 1.0000 — the two are the same profile by
construction (section 3.5), so no window can separate them.

---

## 5. Classifier penalty analysis

Per recorded sample, the runtime-selected profile is compared with the
oracle-ideal profile:

* **EXACT** — runtime profile equals the oracle-ideal profile;
* **OVER-CONSERVATIVE** — it strictly dominates it;
* **UNDER-CONSERVATIVE** — it does **not** dominate it. Required: **0**.
### T11 classifier penalty, runtime selection vs oracle-ideal, per TRUE class


| true class | samples | exact | over-conservative | under-conservative | fallback rate | mean area inflation |
|---|---|---|---|---|---|---|
| PEDESTRIAN | 578 | 0 (0.000) | 578 (1.000) | **0** | 0.5225 | 12.193x |
| BIKE | 313 | 267 (0.853) | 46 (0.147) | **0** | 0.0927 | 1.149x |
| VEHICLE | 989 | 989 (1.000) | 0 (0.000) | **0** | 0.4267 | 1.000x |
| UNKNOWN | 667 | 667 (1.000) | 0 (0.000) | **0** | 0.9430 | 1.000x |
| **pooled** | 2547 | 1923 (0.755) | 624 (0.245) | **0** | 0.5426 | 3.558x |

### T12 confusion detail: true class -> runtime label


| true -> runtime label | samples | exact | over-conservative | under-conservative | fallback |
|---|---|---|---|---|---|
| UNKNOWN->UNKNOWN | 550 | 550 | 0 | **0** | 550 |
| UNKNOWN->VEHICLE | 117 | 117 | 0 | **0** | 79 |
| PEDESTRIAN->UNKNOWN | 197 | 0 | 197 | **0** | 197 |
| PEDESTRIAN->PEDESTRIAN | 381 | 0 | 381 | **0** | 105 |
| BIKE->UNKNOWN | 8 | 0 | 8 | **0** | 8 |
| BIKE->PEDESTRIAN | 28 | 0 | 28 | **0** | 14 |
| BIKE->BIKE | 277 | 267 | 10 | **0** | 7 |
| VEHICLE->UNKNOWN | 382 | 382 | 0 | **0** | 382 |
| VEHICLE->PEDESTRIAN | 306 | 306 | 0 | **0** | 0 |
| VEHICLE->VEHICLE | 301 | 301 | 0 | **0** | 40 |

### 5.1 Selection reasons over the 2 547 samples

| reason | samples | rate |
|---|---|---|
| `class_closure` | 1 162 | 0.4562 |
| `fallback_unknown_label` | 1 137 | 0.4464 |
| `fallback_low_confidence` | 243 | 0.0954 |
| `transition_hold` | 3 | 0.0012 |
| `fallback_immature_track` | 2 | 0.0008 |
| `fallback_invalid_config` | 0 | 0 |
| **fallback total** | **1 382** | **0.5426** |

### 5.2 What the penalty actually is

* **UNDER-CONSERVATIVE = 0.** Gate F passes. The Stage-5D ambiguity handling
  does what it was designed to do.
* **Exact 75.50%, over-conservative 24.50%.**
* The over-conservatism is **entirely concentrated in true PEDESTRIAN**
  (578/578 samples, 12.193× mean area inflation) **and a tail of true BIKE**
  (46/313, 1.149×).
* **True VEHICLE and true UNKNOWN are 100% exact** — not because the
  classifier is right about them (it labels 382/989 true-VEHICLE samples
  `UNKNOWN` and 306/989 `PEDESTRIAN`), but because their oracle-ideal profile
  **is** the fallback envelope. There is nothing for a better classifier to
  win there.
* The 381 correct `PED→PED` samples Stage-5D flagged are all
  over-conservative, and so are the 197 `PED→UNKNOWN` ones. Correct
  classification does not help, because `A(PEDESTRIAN) = {ped, bike, vehicle}`
  envelopes to the fallback (section 3.5).

So the classifier penalty is real, it is **12.2× in continuous area for
pedestrians**, and it is caused by the **ambiguity policy**, not by label
error. Per the brief, it is reported, not fixed.

---

## 6. Classifier untouched

No confidence threshold, hysteresis, ambiguity set, compound-clustering rule
or class threshold was changed. `git status --short` reports exactly one
untracked entry, `validation/stage5d1/`, and `git diff --stat` against
`755d7081` is empty: **no tracked file was modified by this stage**.

---
## 7. Offline four-arm comparison

Identical inputs across all six arms: the same 2 547-row replay table, the
same production Kalman filter re-run along each track in order, the same
horizons, the same `buildRegion`, the same rasterization model. 15 282
samples per arm.
### T13 offline four-arm comparison, pooled over all horizons


| metric | A runtime / 0.55 | B oracle / 0.55 | C runtime / 3.00 | D oracle / 3.00 | B2 oracle-closure / 0.55 | D2 oracle-closure / 3.00 |
|---|---|---|---|---|---|---|
| samples | 15282 | 15282 | 15282 | 15282 | 15282 | 15282 |
| capability containment | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| observed containment | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 | 1.0000 |
| mean region area m^2 | 2151.7 | 1641.7 | 2151.7 | 1641.7 | 2127.8 | 2127.8 |
| mean raster cells | 529.0 | 523.5 | 12681.9 | 12192.9 | 529.0 | 12677.8 |
| mean clamped-window cells | 529.0 | 526.0 | 12808.3 | 12345.1 | 529.0 | 12803.7 |
| saturation rate | 1.0000 | 0.9637 | 0.8170 | 0.7413 | 1.0000 | 0.8146 |
| visible distinct class corridors (of 3 possible) | 1.1592 | 1.1592 | 1.8110 | 1.8110 | 1.1592 | 1.8110 |
| fallback rate | 0.5426 | 0.0000 | 0.5426 | 0.0000 | 0.2619 | 0.2619 |
| exact vs oracle | 11538 | 15282 | 11538 | 15282 | 11814 | 11814 |
| over-conservative | 3744 | 0 | 3744 | 0 | 3468 | 3468 |
| UNDER-conservative | 0 | 0 | 0 | 0 | 0 | 0 |
| mean area inflation vs oracle | 3.5585 | 1.0000 | 3.5585 | 1.0000 | 3.5401 | 3.5401 |
| contract breaches | 0 | 0 | 0 | 0 | 0 | 0 |
| mean raster compute us | 1.2458 | 1.2376 | 28.4579 | 27.4807 | 1.2436 | 28.4486 |

### T14 four-arm raster cells by horizon


| arm | 0.5 s | 1.0 s | 1.5 s | 2.0 s | 2.5 s | 3.0 s |
|---|---|---|---|---|---|---|
| A runtime / 0.55 | 529 | 529 | 529 | 529 | 529 | 529 |
| B oracle / 0.55 | 496 | 529 | 529 | 529 | 529 | 529 |
| C runtime / 3.00 | 2892 | 14635 | 14641 | 14641 | 14641 | 14641 |
| D oracle / 3.00 | 2306 | 12352 | 14576 | 14641 | 14641 | 14641 |
| B2 oracle-closure / 0.55 | 529 | 529 | 529 | 529 | 529 | 529 |
| D2 oracle-closure / 3.00 | 2868 | 14634 | 14641 | 14641 | 14641 | 14641 |

### T15 visible distinct class corridors by horizon (window effect only)


| window | 0.5 s | 1.0 s | 1.5 s | 2.0 s | 2.5 s | 3.0 s |
|---|---|---|---|---|---|---|
| 0.55 m | 1.955 | 1.000 | 1.000 | 1.000 | 1.000 | 1.000 |
| 3.00 m | 3.000 | 2.911 | 1.955 | 1.000 | 1.000 | 1.000 |

### 7.1 What the arm table shows

* **Containment is 1.0000 in every arm**, both capability containment against
  the true class's own disc and observed containment against the track's real
  later position. Expanding the window and swapping the class source change
  nothing about the Stage-5C.1 contract: **0 breaches everywhere**.
* **UNDER-CONSERVATIVE = 0 in every arm.** Gate F.
* **A and C have identical selections** (they differ only in the window), as
  do B and D. That is by construction and is the point: it isolates the
  window.
* `mean_visible_distinct_class_corridors` is **1.1592 at 0.55 m and 1.8110 at
  3.00 m**, out of 3 achievable. The window, and only the window, moves this
  number — it is identical for runtime and oracle arms, because it measures
  what the *raster* can represent, not what the classifier chose.

### 7.2 The four causal comparisons, offline

| comparison | question | result |
|---|---|---|
| **A vs B** | does perfect classification help at 0.55 m? | raster cells 529.00 → 523.49 (**−1.04%**); distinct corridors 1.1592 → 1.1592 (**0%**); saturation 1.0000 → 0.9637. Continuous area falls 23.7% and **none of it reaches the planner**. → **NO** |
| **B vs D** | does expanding help with perfect labels? | distinct corridors 1.1592 → 1.8110 (**+56.2%**); saturation 0.9637 → 0.7413; pairwise separation appears at 1.0 s and 1.5 s where it was exactly 0. → **YES** |
| **C vs D** | cost of the runtime classifier once geometry is visible | raster cells 12 681.9 → 12 192.9 (**−3.9%** pooled); for true PEDESTRIAN 12 701.4 → 10 564.8 (**−16.8%**); area inflation 3.56× → 1.00×. Runtime does **not** collapse — it is within 4% of oracle. |
| **A vs C** | does exposing geometry help the **current runtime** system? | distinct corridors 1.1592 → 1.8110, but raster cells 529 → 12 682 (**24.0×**) and raster compute 1.246 → 28.458 µs (**22.8×**). Geometry becomes visible; the corridor becomes a blanket over 88% of the local costmap. |
---

## 8. Navigation campaign

**72 trials: 6 conditions × 4 arms × 3 repetitions**, 4 266 s wall time, arms
interleaved within each trial index so machine drift cannot bias one arm.

Held fixed by construction (one launch invocation, the same values, the
**same unmodified `nav2_stage4f_params.yaml`** in all four arms): map, world,
start, goal, route, obstacle body, obstacle trajectory, release time, sensor
model, AMCL, Kalman filter, classifier, compound clustering, reachability
mathematics, Stage-5C.1 contract, prediction horizon, MPPI and every critic
weight, Nav2, costmap geometry, costmap resolution, the capability profile
table and the ambiguity policy. **MPPI is not retuned per arm.**

Varied: the two factors only. Verified per trial, not assumed — T16 reads the
window back out of each trial's own recorded config (`window mismatches = 0`
in every arm) and reads the class-aware banner out of each launch log
(`[True]` for the runtime arms, `[False]` for the oracle arms, confirming the
oracle arms ran the bit-identical global path with no class input).

Conditions: `ped_cross`, `ped_longitudinal`, `bike_cross`, `veh_cross`,
`veh_longitudinal`, `unknown_cross` — pedestrian crossing and longitudinal,
bicycle crossing, vehicle crossing and longitudinal, and an UNKNOWN obstacle,
i.e. all six the brief asks for.

**One infrastructure fault** (`veh_longitudinal_a_02`: simulator never
produced a navigation attempt, `nav_status = null`). Separated from navigation
failures per the Stage-4F convention rather than charged to the arm, which
leaves arm A with 17 scored trials against 18 for the others. It does not
affect the direction of any conclusion below, since A is one of the two arms
with zero collisions.
### T16 four-arm navigation comparison


| metric | A runtime / 0.55 | B oracle / 0.55 | C runtime / 3.00 | D oracle / 3.00 |
|---|---|---|---|---|
| trials launched | 18 | 18 | 18 | 18 |
| infrastructure faults (excluded) | 1 | 0 | 0 | 0 |
| trials scored | 17 | 18 | 18 | 18 |
| **success** | **17/17** | **18/18** | **15/18** | **14/18** |
| **collisions** | **0** | **0** | **5** | **3** |
| near misses < 0.20 m | 3 | 3 | 1 | 3 |
| planner/controller failures | 0 | 0 | 3 | 4 |
| collision-monitor interventions | 0 | 0 | 17 | 18 |
| **worst clearance m** | **0.1158** | **0.1303** | **-0.128** | **-0.1351** |
| mean clearance m (sd) | 0.6302 (0.3132) | 0.6278 (0.3031) | 1.0204 (1.2794) | 1.0724 (1.3191) |
| **nav time s** (sd) | **19.6372** (2.6306) | **19.5655** (2.8327) | **29.3268** (14.7056) | **31.8596** (15.448) |
| path length m (sd) | 5.9741 (0.1391) | 5.986 (0.1489) | 5.8682 (2.2297) | 5.8549 (2.4655) |
| **peak predictive cells** (sd) | **1272** (208) | **1238** (200) | **12225** (2913) | **11805** (3282) |
| mean predictive frac of window | 0.0499 (0.0093) | 0.0478 (0.0098) | 0.5064 (0.1364) | 0.51 (0.1823) |
| layer updateCosts mean us | 25.6941 (2.6152) | 24.5667 (2.2342) | 29.0333 (2.6938) | 30.0444 (2.567) |
| layer updateCosts max us (worst trial) | 109.0 | 160.6 | 608.3 | 102.8 |
| layer p95 / p99 | not instrumented | not instrumented | not instrumented | not instrumented |
| layer update rate Hz (target 5.0) | 5.0218 | 5.0217 | 5.0189 | 5.0189 |
| missed update deadlines | 0 | 0 | 0 | 0 |
| tracker reach us | 7.4531 | 7.1022 | 7.8283 | 7.3937 |
| fallback rate | 0.4888 | 0.0 | 0.5154 | 0.0 |
| class-closure rate | 0.5112 | 0.0 | 0.4846 | 0.0 |
| global-profile rate | 0.0 | 1.0 | 0.0 | 1.0 |
| **contract violations** | **0** | **0** | **0** | **0** |
| **unsafe profile events** | **0** | **0** | **0** | **0** |
| class-aware banner seen | [True] | [False] | [True] | [False] |
| window recorded in trial config | [0.55] | [0.55] | [3.0] | [3.0] |
| window mismatches | 0 | 0 | 0 | 0 |
| failure causes | - | - | collision_monitor_intervention, prediction_error, unclassified | collision_monitor_intervention, prediction_error, unclassified |

### T17 per condition: success / scored, collisions, mean nav time s


| condition | true class | A runtime / 0.55 | B oracle / 0.55 | C runtime / 3.00 | D oracle / 3.00 |
|---|---|---|---|---|---|
| bike_cross | BIKE | 3/3, c=0, 22.8s | 3/3, c=0, 22.1s | 1/3, c=0, 48.0s | 0/3, c=0, 60.0s |
| ped_cross | PEDESTRIAN | 3/3, c=0, 19.1s | 3/3, c=0, 19.1s | 3/3, c=0, 26.9s | 3/3, c=0, 26.4s |
| ped_longitudinal | PEDESTRIAN | 3/3, c=0, 16.5s | 3/3, c=0, 16.2s | 3/3, c=2, 16.8s | 3/3, c=1, 15.8s |
| unknown_cross | UNKNOWN | 3/3, c=0, 19.9s | 3/3, c=0, 20.5s | 3/3, c=0, 25.1s | 3/3, c=0, 27.4s |
| veh_cross | VEHICLE | 3/3, c=0, 22.1s | 3/3, c=0, 23.3s | 3/3, c=0, 25.4s | 3/3, c=0, 27.6s |
| veh_longitudinal | VEHICLE | 2/2, c=0, 16.6s | 3/3, c=0, 16.2s | 2/3, c=3, 33.8s | 2/3, c=2, 33.9s |

### T18 per true class: peak predictive cells / mean frac of local window


| true class | A runtime / 0.55 | B oracle / 0.55 | C runtime / 3.00 | D oracle / 3.00 |
|---|---|---|---|---|
| BIKE | 1683 / 0.065 | 1663 / 0.064 | 7801 / 0.344 | 7157 / 0.319 |
| PEDESTRIAN | 1240 / 0.044 | 1163 / 0.041 | 11921 / 0.448 | 10872 / 0.420 |
| UNKNOWN | 1136 / 0.052 | 1121 / 0.052 | 13914 / 0.558 | 13910 / 0.540 |
| VEHICLE | 1145 / 0.046 | 1160 / 0.044 | 13897 / 0.619 | 14010 / 0.680 |

### T19 every non-clean trial, all four arms


| trial | arm | class source | window m | nav status | collision | clearance m | peak cells | frac window | cause |
|---|---|---|---|---|---|---|---|---|---|
| bike_cross_c_01 | C | runtime | 3.0 | TIMEOUT | no | 2.8677 | 7184 | 0.3202 | unclassified |
| bike_cross_c_02 | C | runtime | 3.0 | TIMEOUT | no | 2.8691 | 7184 | 0.3168 | unclassified |
| bike_cross_d_01 | D | oracle | 3.0 | TIMEOUT | no | 2.8616 | 7080 | 0.3224 | unclassified |
| bike_cross_d_02 | D | oracle | 3.0 | TIMEOUT | no | 2.8704 | 7199 | 0.3145 | unclassified |
| bike_cross_d_03 | D | oracle | 3.0 | TIMEOUT | no | 2.8605 | 7192 | 0.3194 | unclassified |
| ped_longitudinal_c_01 | C | runtime | 3.0 | SUCCEEDED | **YES** | -0.0107 | 14391 | 0.5062 | collision_monitor_intervention |
| ped_longitudinal_c_02 | C | runtime | 3.0 | SUCCEEDED | **YES** | -0.0731 | 14273 | 0.5157 | collision_monitor_intervention |
| ped_longitudinal_d_03 | D | oracle | 3.0 | SUCCEEDED | **YES** | -0.0042 | 14337 | 0.4695 | collision_monitor_intervention |
| veh_longitudinal_a_02 | A | runtime | 0.55 | None | no | None | None | None | INFRASTRUCTURE FAULT |
| veh_longitudinal_c_01 | C | runtime | 3.0 | SUCCEEDED | **YES** | -0.0071 | 14400 | 0.5936 | collision_monitor_intervention |
| veh_longitudinal_c_02 | C | runtime | 3.0 | TIMEOUT | **YES** | -0.128 | 14400 | 0.9098 | prediction_error |
| veh_longitudinal_c_03 | C | runtime | 3.0 | SUCCEEDED | **YES** | -0.0114 | 14400 | 0.5711 | collision_monitor_intervention |
| veh_longitudinal_d_02 | D | oracle | 3.0 | SUCCEEDED | **YES** | -0.0891 | 14400 | 0.9132 | prediction_error |
| veh_longitudinal_d_03 | D | oracle | 3.0 | ABORTED | **YES** | -0.1351 | 14400 | 0.9707 | prediction_error |

---

## 9. Navigation metrics

### 9.1 What the navigation campaign shows

**The two production-window arms are clean. The two expanded-window arms are
not.**

* **0 collisions in 35 scored trials at 0.55 m** (A and B combined), worst
  clearance +0.116 m.
* **8 collisions in 36 scored trials at 3.00 m** (C: 5, D: 3), worst clearance
  **−0.135 m**. Collision-monitor interventions go from **0 and 0** to **17
  and 18**.
* Navigation time rises from 19.6 s / 19.6 s to 29.3 s / 31.9 s — the only
  nav metric besides cells and layer cost that is resolved beyond two sigma.
* 5 of the 6 `bike_cross` trials in arms C and D **time out** (60 s) stalled
  ~2.86 m short, and `bike_cross` success falls from **3/3 and 3/3** to
  **1/3 and 0/3** (T17).

The mechanism is visible in the cell counts. Mean predictive coverage of the
local window goes from **0.050 / 0.048** to **0.506 / 0.510**, and every
`veh_longitudinal` collision trial records `max_pred_cells = 14400` — **the
entire 120 × 120 local costmap** — with mean coverage up to **0.971** (T19).
At that point the predictive layer is not a corridor the planner can steer
around; it is a near-uniform plateau over the whole local window, MPPI has no
cost gradient left to express an avoidance manoeuvre, and the robot either
stalls until timeout or is carried into the obstacle with the collision
monitor firing.

**This is not a CPU limit.** Layer `updateCosts` rises only 25.7 → 29.0 µs
mean (+13%), the worst single update across 72 trials is 608 µs, the layer
holds 5.019 Hz against a 5.0 Hz target, and **missed update deadlines = 0 in
every arm**. Expanded semantic visibility is computationally affordable and
navigationally destructive — the opposite of the failure mode the brief
anticipated, and worth stating plainly.

**Oracle labels do not rescue it.** Arm D has perfect class knowledge and is
the *worst* arm on success (14/18) and navigation time (31.9 s).

### 9.2 Uncertainty

Three repetitions per condition resolves a large effect and not a small one,
so T20 prints every difference beside two sigma of the difference and marks
which are resolved. Resolved: navigation time, peak predictive cells, mean
predictive coverage, layer compute. **Not resolved, and therefore not
claimed:** minimum clearance, mean clearance and path length in every
comparison. The collision and success counts are categorical rather than
mean-difference claims: 0 and 0 against 5 and 3, with the mechanism
independently visible in the cell counts.
### T20 navigation causal comparisons, difference +- two sigma of the difference


**A_vs_B** — Does PERFECT CLASSIFICATION help while the 0.55 m window remains? NO => the classifier is not the primary bottleneck under current policy.

| metric | left mean | right mean | difference | 2-sigma band | resolved? |
|---|---|---|---|---|---|
| min_clearance_m | 0.6302 | 0.6278 | -0.0024 | ±0.2086 | within noise |
| nav_time_s | 19.6372 | 19.5655 | -0.0717 | ±1.847 | within noise |
| path_length_m | 5.9741 | 5.986 | +0.0119 | ±0.0973 | within noise |
| max_pred_cells | 1271.6471 | 1238.2778 | -33.3693 | ±137.9547 | within noise |
| mean_pred_frac_window | 0.0499 | 0.0478 | -0.0021 | ±0.0064 | within noise |
| layer_update_us_mean | 25.6941 | 24.5667 | -1.1274 | ±1.6488 | within noise |

**B_vs_D** — Does EXPANDING THE WINDOW help even with perfect labels? YES => the 0.55 m policy window is suppressing semantic information.

| metric | left mean | right mean | difference | 2-sigma band | resolved? |
|---|---|---|---|---|---|
| min_clearance_m | 0.6278 | 1.0724 | +0.4446 | ±0.638 | within noise |
| nav_time_s | 19.5655 | 31.8596 | +12.2941 | ±7.4036 | **RESOLVED** |
| path_length_m | 5.986 | 5.8549 | -0.1311 | ±1.1643 | within noise |
| max_pred_cells | 1238.2778 | 11805.3333 | +10567.0555 | ±1549.841 | **RESOLVED** |
| mean_pred_frac_window | 0.0478 | 0.51 | +0.4622 | ±0.0861 | **RESOLVED** |
| layer_update_us_mean | 24.5667 | 30.0444 | +5.4777 | ±1.6043 | **RESOLVED** |

**C_vs_D** — With semantic geometry visible, how much is lost to the RUNTIME classifier / ambiguity policy?

| metric | left mean | right mean | difference | 2-sigma band | resolved? |
|---|---|---|---|---|---|
| min_clearance_m | 1.0204 | 1.0724 | +0.052 | ±0.8663 | within noise |
| nav_time_s | 29.3268 | 31.8596 | +2.5328 | ±10.0541 | within noise |
| path_length_m | 5.8682 | 5.8549 | -0.0133 | ±1.5671 | within noise |
| max_pred_cells | 12225.2222 | 11805.3333 | -419.8889 | ±2068.4669 | within noise |
| mean_pred_frac_window | 0.5064 | 0.51 | +0.0036 | ±0.1073 | within noise |
| layer_update_us_mean | 29.0333 | 30.0444 | +1.0111 | ±1.7541 | within noise |

**A_vs_C** — Does exposing more geometry help the CURRENT RUNTIME system? This is the practically relevant deployment comparison.

| metric | left mean | right mean | difference | 2-sigma band | resolved? |
|---|---|---|---|---|---|
| min_clearance_m | 0.6302 | 1.0204 | +0.3902 | ±0.6221 | within noise |
| nav_time_s | 19.6372 | 29.3268 | +9.6896 | ±7.0487 | **RESOLVED** |
| path_length_m | 5.9741 | 5.8682 | -0.1059 | ±1.0534 | within noise |
| max_pred_cells | 1271.6471 | 12225.2222 | +10953.5751 | ±1376.8005 | **RESOLVED** |
| mean_pred_frac_window | 0.0499 | 0.5064 | +0.4565 | ±0.0644 | **RESOLVED** |
| layer_update_us_mean | 25.6941 | 29.0333 | +3.3392 | ±1.7949 | **RESOLVED** |
---

## 10. Causal interpretation

### A vs B — does perfect classification help while 0.55 m remains?

**NO.**

| | offline | navigation |
|---|---|---|
| raster cells | 529.00 → 523.49 (**−1.04%**) | 1 272 → 1 238, within noise (±138) |
| visible distinct class corridors | 1.1592 → 1.1592 (**0%**) | — |
| continuous region area | 2 151.7 → 1 641.7 m² (−23.7%) | — |
| success / collisions | — | 17/17, 0 → 18/18, 0 |
| nav time | — | 19.64 → 19.57 s, within noise |

Every navigation metric is within two sigma. A 23.7% reduction in true region
area reaches the planner as **1.04% of cells**, and changes the number of
distinguishable class corridors by nothing at all.

**=> The classifier is NOT the primary bottleneck under current policy.**

### B vs D — does expanding the window help even with perfect labels?

**It exposes the information, and the information is not useful.**

Expansion genuinely un-suppresses semantic geometry — this is measured, not
asserted: visible distinct class corridors **1.1592 → 1.8110** (+56.2%),
saturation **0.9637 → 0.7413**, and pairwise raster separation that was
**exactly 0.000** at every horizon ≥ 1.0 s becomes 0.933–0.978 at 1.0 s and
0.955 at 1.5 s (T9).

So the literal claim "the 0.55 m policy window is suppressing semantic
information" is **TRUE**.

But the navigation consequence of removing that suppression, *with perfect
labels*, is: success **18/18 → 14/18**, collisions **0 → 3**, worst clearance
**+0.130 → −0.135 m**, navigation time **19.57 → 31.86 s** (resolved,
±7.40). The suppressed information, once exposed, makes navigation worse.

### C vs D — how much is lost to the runtime classifier once geometry is visible?

**Very little.**

Offline: raster cells 12 681.9 → 12 192.9 (**−3.9%** pooled; −16.8% on true
PEDESTRIAN alone), area inflation 3.56× → 1.00×. Navigation: **every metric
within noise**, and the oracle arm is the worse of the two on success (15/18
vs 14/18) and navigation time.

The runtime arm does not collapse relative to oracle — it tracks it to within
4%. **The ambiguity/fallback policy is not what limits the expanded arms.**

### A vs C — does exposing more geometry help the CURRENT runtime system?

**No: it harms it.** This is the deployment-relevant comparison.

Visible distinct corridors 1.1592 → 1.8110, bought at 24.0× raster cells,
22.8× offline raster compute, mean local-window coverage 0.050 → 0.506
(resolved), navigation time 19.64 → 29.33 s (resolved, ±7.05), success 17/17
→ 15/18, and **collisions 0 → 5**.

---

## 11. Diagnosis

### **D. PROFILE_LIMITED**

> Even Oracle/expanded provides little useful discrimination / navigation
> value.

**Why not the others:**

* **B. CLASSIFIER_LIMITED** — refuted. `Runtime/expanded` does **not**
  collapse relative to `Oracle/expanded`: −3.9% cells offline and every
  navigation metric within noise (C vs D).
* **C. WINDOW_AND_CLASSIFIER_LIMITED** — refuted on the classifier half, by
  both A vs B (−1.04%, 0% corridor change) and C vs D (within noise).
* **A. WINDOW_LIMITED** — its first clause holds exactly
  (`Oracle/current ≈ Runtime/current`), and the window **is** demonstrably
  suppressing semantic information. Its second clause requires the expanded
  arms to show *meaningful* semantic differentiation. They show
  differentiation that is real, measurable, confined to horizons ≤ 1.5 s, and
  embedded in corridors covering 85–100% of the local costmap — and that
  produces 8 collisions, 5 timeouts and +10 to +12 s of navigation time
  against zero collisions in the production window. Calling that "meaningful"
  would be forcing a favourable result, which the brief explicitly forbids.
* **E. NO_MEANINGFUL_SEMANTIC_BENEFIT** — consistent with the evidence, but
  weaker: it records the outcome without naming the cause. D names it.

**The cause, stated precisely.** The Stage-5D capability profiles span **16×
in `max_speed`** (2.5 → 40.0 m/s) and **4× in `max_acceleration`**
(2.5 → 10.0 m/s²). Integrated over the production 3.0 s horizon they generate
regions of semi-axis **11.64 / 30.90 / 49.79 / 49.79 m** (T3) on a **6 × 6 m**
local costmap. Any window small enough to keep the corridor inside the map
saturates every class early; any window large enough to show the class
difference also blankets the map. The two requirements are mutually exclusive
*for these profiles at this horizon on this costmap* — and that is a property
of the profile design, not of the clamp.

Two further structural findings reinforce it, and neither depends on any
radius:

1. `A(PEDESTRIAN)` and `A(VEHICLE)` both envelope to the fallback, so the
   runtime system can express only **two** corridors, and `VEHICLE` vs
   `UNKNOWN` is identical **by construction** (area ratio exactly 1.0000 at
   every horizon).
2. The reachability region is a **disc** at every horizon — the Stage-4C
   covariance is isotropic in position, so `semi_major == semi_minor` always.
   Class membership therefore modulates a single scalar radius and carries no
   directional information for a planner to exploit.

And the sharpest single datum: **BIKE**, the only class the runtime system can
give a distinct corridor, is the class whose navigation collapses hardest
when that corridor is made visible — `bike_cross` success 3/3 and 3/3 at
0.55 m against 1/3 and 0/3 at 3.00 m.
---

## 12. Safety check

| check | result |
|---|---|
| zero unsafe profile selections in the runtime path | **0** — offline `under_conservative = 0` across all 15 282 samples in all six arms; live `unsafe_profile_events = 0` and `prof_fb_badconfig = 0` in all four nav arms |
| containment preserved inside declared profile assumptions | capability containment **1.0000** and observed containment **1.0000** in all six offline arms |
| Stage-5C.1 contract | **0 breaches** offline, **0 violations** live in all four arms, worst observed speed 0.000 |
| no ground-truth label enters production nodes | **verified** — no file under `src/predictive_nav_tracking/{src,include}` or `src/predictive_nav_costmap/{src,include}` contains `gt_class`, `gt_label`, `ground_truth` or `oracle`; the oracle nav arms run `reachability_class_aware: false` (banner `[False]`) and have no class input at all |
| oracle evaluation isolated from production configuration | `validation/stage5d1/oracle_*_tracker.yaml` are not installed by any package, not referenced by any `CMakeLists.txt`, launch default or production params file; `grep -rn "stage5d1" src/` returns nothing |
| CV production default unchanged | `production_nav2_params.yaml:287` `prediction_mode: "cv_covariance"`, `:365` `max_influence_radius: 0.55`; `production_tracker_params.yaml:160` `reachability_enabled: false`, `:280` `reachability_class_aware: false` — all byte-identical to HEAD |
| production safety policy weakened? | **no** — confidence gate 0.60, `min_observations` 5, `transition_hold_scans` 3, every ambiguity set and every profile value untouched |

The 3.00 m window was applied **only** through the layer's own documented
runtime parameter (`policy_max_influence_radius` launch argument →
`SetParameters` on `max_influence_radius`), in the analysis arms, for the
duration of those trials. No YAML was written.

---

## 13. Performance check

| | A 0.55 | B 0.55 | C 3.00 | D 3.00 |
|---|---|---|---|---|
| peak predictive cells (mean over trials) | 1 272 | 1 238 | **12 225** | **11 805** |
| mean coverage of local window | 0.0499 | 0.0478 | **0.5064** | **0.5100** |
| layer `updateCosts` mean µs | 25.69 | 24.57 | 29.03 | 30.04 |
| layer `updateCosts` max µs (worst of 72 trials) | 109.0 | 160.6 | **608.3** | 102.8 |
| layer p95 / p99 | **not instrumented** | | | |
| layer update rate Hz (target 5.0) | 5.0218 | 5.0217 | 5.0189 | 5.0189 |
| **missed update deadlines** | **0** | **0** | **0** | **0** |
| tracker reachability µs | 7.45 | 7.10 | 7.83 | 7.39 |
| offline raster model µs/region | 1.246 | 1.238 | 28.458 | 27.481 |

`p95`/`p99` are **not available**: the layer's stats line instruments mean and
max only, so they are reported as absent rather than invented.

**The result is the opposite of the one anticipated.** Expanded semantic
visibility is **computationally practical** — +13% on the layer's mean update,
no missed Nav2 costmap deadlines in any of 72 trials — and
**navigationally destructive**. The binding cost is not CPU; it is the
destruction of the cost-field structure MPPI needs, measured as coverage
rising from 5% to 51% of the local window and reaching 100% in every
`veh_longitudinal` collision trial. Per the brief, nothing is optimized here.

---

## 14. Scope discipline

Not done, and deliberately: production `max_influence_radius` unchanged (0.55 m);
classifier ambiguity handling unchanged; no new class profiles; MPPI untuned;
planner and controller untuned; CV remains the production default; class-aware
reachability remains non-default; no IMM, no camera perception, no learned
models. Stage-5D.1 is diagnosis only.

---

## 15. Acceptance gates

| gate | requirement | result |
|---|---|---|
| A | historical/current role of `max_influence_radius` documented | **PASS** — §1, basis proven from three independent repository sources plus git provenance |
| B | four analysis arms implemented without ground-truth leakage | **PASS** — §2, §12 |
| C | same inputs used across arms | **PASS** — one replay table offline; one launch invocation and one unmodified params file in nav, window read back per trial (0 mismatches) |
| D | current vs expanded raster class separation quantified | **PASS** — T5–T10 |
| E | runtime vs oracle profile-selection penalty quantified | **PASS** — T11, T12 |
| F | zero runtime under-conservative selections | **PASS** — 0/15 282 offline, 0 live |
| G | bounded four-arm navigation comparison completed | **PASS** — 72 trials |
| H | planner metrics reported by arm | **PASS** — T16–T20 |
| I | performance cost of expanded visibility measured | **PASS** — §13 |
| J | causal diagnosis assigned from the decision matrix | **PASS** — PROFILE_LIMITED |
| K | CV production default unchanged | **PASS** |
| L | no production safety policy weakened | **PASS** |

**A PASS does not require semantic navigation benefit, and none was found.**

---

## 16. Reproduction

```bash
# offline four-arm audit
validation/stage5d/extract_labels.py --out <dir>/replay.txt
g++ -O2 -std=c++17 -I/usr/include/eigen3 \
    -Isrc/predictive_nav_tracking/include \
    validation/stage5d1/stage5d1_audit.cpp -o <dir>/stage5d1_audit
<dir>/stage5d1_audit <dir>/replay.txt > validation/stage5d1/discriminability_audit.json

# four-arm navigation campaign (72 trials, ~71 min)
validation/stage5d1/run_nav4.py --out <dir>/nav4 --trials 3 --arms a,b,c,d
validation/stage5d1/analyze_nav4.py --trials <dir>/nav4 \
    --out validation/stage5d1/nav_campaign4.json
```

---

## 17. Files

New, all under `validation/stage5d1/` — nothing tracked outside it changed:

| file | role |
|---|---|
| `stage5d1_audit.cpp` | offline six-arm audit, built against the shipped production headers |
| `discriminability_audit.json` | its output (T1–T15) |
| `run_nav4.py` | four-arm Gazebo campaign runner; imports Stage-5D's conditions rather than restating them |
| `analyze_nav4.py` | campaign summary; imports Stage-5D's log parser rather than restating it |
| `nav_campaign4.json` | its output (T16–T20) |
| `oracle_ped_tracker.yaml` | evaluation-only: global path, `max_speed`/`max_acceleration` = 2.5 / 2.5 |
| `oracle_bike_tracker.yaml` | evaluation-only: 12.0 / 7.0 |
| `oracle_fallback_tracker.yaml` | evaluation-only: 40.0 / 10.0 (VEHICLE and UNKNOWN) |
| `STAGE5D1_README.md` | this document |

## 18. Tests

* **New:** none. This stage adds no production code, so it adds no unit test.
  The audit is itself an executable check and reports its own gates.
* **Existing:** all **13 functional gtests pass** —
  `stage4g2_lifecycle`, `stage4g3_deblend`, `stage4g4_reachability`,
  `stage5b1_compound`, `stage5b1_compound_integration`,
  `stage5b_classification`, `stage5b_tracker_integration`,
  `stage5c1_contract`, `stage5d_profiles`, `stage4g2_multi_layer`,
  `stage4g4_reach_layer`, `stage4g5_hybrid_layer`,
  `stage5c1_raster_fidelity`.
* **Pre-existing lint debt (unchanged by this stage):** 1 237 `colcon test`
  failures, all of them `flake8` (1 011), `pep257` (210), `uncrustify` (8) and
  `xmllint` (1) on files tracked at HEAD. Zero functional failures. No lint
  result references `validation/stage5d1`, which sits outside `src/` and is
  linted by no package.
