# Stage-4G8 — final production robustness validation

# FINAL SYSTEM VALIDATED — PASS WITH LIMITATION

448 trials across 46 condition groups exercise the frozen CV-only production
architecture under obstacle-speed, encounter-timing, sensor-noise,
localisation, geometry, multi-object, occlusion and randomised perturbation.
The system reproduces its validated behaviour, degrades gracefully, and its
breakdown points are physical rather than algorithmic. Every limitation found
is an understood sensing or reaction-time limit, not a defect — hence PASS
WITH LIMITATION rather than an unqualified PASS.

**Prediction still earns its place well outside the nominal benchmark.**
Against a reactive baseline under perturbation:

| scenario | reactive | CV-predictive |
|---|---:|---:|
| perpendicular crossing, mean clearance | 0.112 m | **0.812 m** |
| reversal, collisions | **10 / 10** | **0 / 9** |
| blind-corner world, mean clearance | 0.412 m | **0.945 m** |
| reaction lead | ~0.0 s | **2.1 – 2.5 s** |

No later stage has been started.

## 1. Final production configuration

Two files, both explicit, both committed. Neither is an experimental overlay.

| file | derived from | the only differences |
|---|---|---|
| `config/production_nav2_params.yaml` | `nav2_stage4f_params.yaml` (byte-unchanged) | `predicted_obstacle_layer.prediction_mode: "cv_covariance"` stated explicitly instead of inherited from the plugin's C++ default |
| `predictive_nav_tracking/config/production_tracker_params.yaml` | `tracker_params.yaml` (byte-unchanged) | `reachability_enabled: true → false` |

Everything else is identical: Stage-4G1 range-aware static rejection,
Stage-4B clustering, Stage-4G3 track-aware deblending, greedy association,
Stage-4C constant-velocity Kalman prediction, the Stage-4D predictive costmap
and the frozen MPPI configuration.

Turning reachability publication off cannot change the CV output —
`stage4g4_reachability` asserts `cv_prediction_bit_identical` with the flag in
either state — and it saves the 0.35–0.98 ms/scan that computing and publishing
an unread field would cost. Verified in the runs: production trials publish
**6 CV predictions and 0 reachability entries** per track, and the layer logs
`mode=cv_covariance`.

Run it with:

```bash
ros2 launch predictive_nav_bringup stage4f_bringup.launch.py \
  params_file:=<pkg>/config/production_nav2_params.yaml \
  tracker_params:=<pkg_tracking>/config/production_tracker_params.yaml
```

## 2. Provenance

| item | value |
|---|---|
| HEAD at start | `3b52d5a92530cb054facdd70b09bac68d0bf1b5b`, clean tree |
| `stage4f-validated` | `68b2bf2ee1efb774ba563438a4c621b0052f61d5` — unchanged |
| `stage4g5-hybrid` | `76ee6b0d…` — unchanged |
| `stage4g6-decision` | `5281f234…` — unchanged |
| `stage4g7-cv-final` | `3b52d5a9…` — unchanged, equals the starting HEAD |
| algorithm files | **all byte-identical to Stage-4G5/4G7** (`provenance.json`, `all_algorithm_files_unchanged: true`) |
| `~/ur5e_ws` | untouched (mtime 2026-09-09, predates all Stage-4G work) |

No prediction mathematics, association, deblending or static rejection was
modified, and nothing was retuned per scenario. The only non-test changes are
the two production config files. Three launch hooks and one relay node were
added **for testing only**, all defaulting to the previously validated
behaviour: a `tracker_params` file argument, a `tracker_scan_topic` override,
initial-pose offsets, and a seeded `scan_noise.py` relay.

Two harness bugs were found and fixed, both in the test runner, neither in the
system: a fixed 120 s wall-clock guard that aborted any scenario longer than
that (it killed the 150 s endurance run), and the scenario-duration window that
truncated goal completion.

## 3. Nominal regression against recorded references (§4)

Re-run under the **production** configuration and compared with the values the
earlier stages actually recorded:

| scenario | Stage-4G8 (n) | recorded reference | match |
|---|---|---|---|
| `nav_cv_control` perpendicular | **0.812 m**, 10/10, 0 coll (10) | Stage-4G6 CV 0.803 m, 3/3 | ✔ +0.009 |
| `nav_reversal` | **0.725 m**, 9/9, 0 coll (9) | Stage-4G6 CV 0.740 m, 3/3 | ✔ −0.015 |
| `g6_m08_cv` | **0.676 m**, 10/10, 0 coll (10) | Stage-4G6 CV 0.689 m, 20/20 | ✔ −0.013 |
| `nav_occlusion_short` | **0.484 m**, 10/10, 0 coll (10) | Stage-4G6 CV 0.533 m, 3/3 | ✔ −0.049 |
| `navigation` (2 objects) | 0 ID switches, purity 1.000, matched 1.000, 10/10 | Stage-4G3 identical | ✔ |
| `triple` (3 objects) | 0 ID switches, purity 1.000, matched 1.000 | Stage-4G3 identical | ✔ |
| `resolution_75` / `_85` | 0 ID switches, purity 1.000, matched 1.000 | Stage-4G3 identical | ✔ |
| `static_navigation` | 0 tracks, 0 predictive cells | Stage-4G3 identical | ✔ |

Nominal behaviour is reproduced by the explicit production configuration.

## 4. Obstacle-speed sweep (§5)

Perpendicular crossing, production config, 10 trials per speed.

| speed | n | success | coll | mean clr | worst clr | reaction lead | vel err | pred ADE |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.15 m/s | 10 | 10/10 | 0 | 0.883 | 0.855 | −3.74 s* | 0.034 | 0.173 |
| 0.25 | 9 | 9/9 | 0 | 0.649 | 0.586 | 3.47 s | 0.045 | 0.238 |
| 0.50 | 10 | 10/10 | 0 | 0.803 | 0.759 | 2.36 s | 0.065 | 0.300 |
| 0.75 | 10 | 10/10 | 0 | 0.690 | 0.660 | 1.68 s | 0.085 | 0.374 |
| **1.00** | 9 | 9/9 | 0 | **0.184** | **0.028** | **0.76 s** | 0.113 | 0.471 |

\* at 0.15 m/s the obstacle is so slow that the closest approach precedes the
robot's reaction, making the lead negative — an artefact of the metric, not a
failure.

**Operating region: obstacle speeds ≤ 0.75 m/s.** Up to 0.75 m/s clearance
stays ≥ 0.66 m with ≥ 1.68 s of reaction lead. At 1.00 m/s the system is still
collision-free but the margin is effectively gone (worst 0.028 m) and reaction
lead has collapsed to 0.76 s. Tracking itself does not break — velocity error
rises only 0.034 → 0.113 m/s and prediction ADE 0.173 → 0.471 m, both smooth.
The limit is reaction time, not perception.

## 5. Encounter-timing sweep (§6)

Release offset shifts the conflict away from the tuned synchronisation point.

| offset | n | success | coll | mean clr | worst clr | lead |
|---:|---:|---:|---:|---:|---:|---:|
| −1.5 s | 8 | 8/8 | 0 | 0.935 | 0.887 | 1.42 s |
| −1.0 s | 16 | 16/16 | 0 | 0.880 | 0.838 | 2.05 s |
| −0.5 s | 8 | 8/8 | 0 | 0.852 | 0.808 | 2.20 s |
| 0.0 s | 15 | 15/15 | 0 | 0.782 | 0.736 | 2.39 s |
| +0.5 s | 8 | 8/8 | 0 | 0.731 | 0.694 | 2.73 s |
| +1.0 s | 16 | 16/16 | 0 | 0.662 | 0.587 | 2.66 s |
| **+1.5 s** | 8 | 7/8 | **2** | **0.468** | **−0.115** | 2.89 s |

**Performance does not depend on one carefully synchronised encounter**:
across a 2.5 s window (−1.5 to +1.0 s) every trial succeeded with ≥ 0.587 m
clearance. Only at +1.5 s — where the crossing arrives inside the braking
distance — do collisions appear.

## 6. Sensor-noise robustness (§7)

Extra Gaussian range noise injected on the **tracker's** input only (a seeded
relay; AMCL and the obstacle costmap keep the clean scan, so this isolates
tracker tolerance). Tracker tuning was **not** changed per level.

| added σ | n | succ | coll | false confirmed | false persistent | ID sw | purity | pos err | vel err | ADE |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.00 | 20 | 20/20 | 0 | 1 | 0 | 0 | 1.000 | 0.214 | 0.063 | 0.274 |
| 0.03 | 9 | 9/9 | 0 | **0** | **0** | 0 | 1.000 | 0.210 | 0.063 | 0.270 |
| 0.06 | 10 | 10/10 | 0 | **0** | **0** | 0 | 1.000 | 0.208 | 0.070 | 0.276 |
| **0.10** | 9 | 9/9 | 0 | **80** | **35** | 0 | 1.000 | — | — | — |

**The tracker is essentially insensitive to added range noise up to 0.06 m** —
position error, velocity error and prediction ADE do not move. At 0.10 m the
noise exceeds the 0.15 m static-rejection radius and walls begin generating
false tracks (failure class **B**), though navigation stayed collision-free
because the spurious cost is conservative.

## 7. Localisation robustness (§8)

Controlled and repeatable: the robot is **spawned** at the true pose while AMCL
is **initialised** at true + (dx, dy, dyaw), so the injected error is known
exactly rather than random.

| offset | n | succ | coll | mean clr | false confirmed | false persistent | matched |
|---|---:|---:|---:|---:|---:|---:|---:|
| none | 20 | 20/20 | 0 | 1.116 | 1 | 0 | 0.463 |
| dx 0.15, dyaw 0.10 | 10 | 10/10 | 0 | 1.182 | 24 | 1 | 0.460 |
| dx 0.30, dyaw 0.20 | 10 | 10/10 | 0 | 1.221 | **126** | **73** | 0.368 |
| dx 0.50, dyaw 0.35 | 10 | 10/10 | 0 | 1.359 | **155** | **69** | — |

This **re-confirms the Stage-4G1 finding quantitatively**: pose error is what
breaks static-map rejection, because scan points stop landing on their map
cells. Moving-object tracking stays functional and navigation stays
collision-free in all 30 perturbed trials — the false tracks sit near walls,
away from the route, and only add conservative cost. Failure class **B**, and
the most important operational caveat in the system.

## 8. Geometry-rich world (§9)

Stage-4G7 blind-corner world (`stage4g7_c30`), production config, 10 trials:
**10/10 success, 0 collisions, mean clearance 0.945 m, 0 ID switches, 0 false
confirmed and 0 false persistent tracks.** The tracker does not depend on the
open-room geometry; adding static structure near the route produced no static
residuals and no costmap ghosts.

## 9. Multi-object (§10)

| scenario | objects | n | succ | coll | ID switches | purity | false persistent | costmap multi-representation |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| `navigation` | 2 | 10 | 10/10 | 0 | 0 | 1.000 | 0 | **1140 / 1140 (100%)** |
| `triple` | 3 | 10 | — † | 0 | 0 | 1.000 | 0 | **239 / 239 (100%)** |
| `g6_mixed` | 3, one occluded | 10 | 10/10 | 0 | 34 | 0.609 | 0 | 1821 / 1823 (99.89%) |

† `triple` is a non-navigating tracking scenario, so it has no goal status.

IDs are stable wherever observations are resolvable; multiple prediction
corridors coexist; max-composition works; no ghost cells. `g6_mixed` shows the
expected identity churn when one of three objects is occluded — the Stage-4G3
behaviour, not a regression. Deblending resolution is unchanged
(`resolution_75`/`_85`: 0 ID switches, purity 1.000, matched 1.000).

## 10. Occlusion (§11) — a documented CV-only limitation

Brief occlusion, production CV-only, 10 trials: **0 collisions, 0 false
persistent tracks, same-ID reacquisition after 2–5 missed scans, 0
disappearances**, mean clearance 0.381 m, worst **0.110 m**. The robot reached
the goal position (final x 3.10–3.74) in every trial, though the
`NavigateToPose` result had not returned inside the recording window.

Track survival, lifecycle and reacquisition are exactly the validated
Stage-4G3 behaviour. The known limitation stands, unchanged and not reopened:
**CV coasting coverage is poor during an observation gap** (Stage-4G7 measured
0.207 at 0.5 s under static occlusion). Stage-4G4–4G7 established that
reachability fixes that coverage hole and that the fix does not propagate into
a navigation outcome; Stage-4G8 confirms the consequence is a *reduced margin*
(worst 0.110 m here) rather than a failure. Failure class **A**.

## 11. Randomised robustness (§13)

Seeded per-trial perturbation of obstacle start position (±0.15 m), speed
(±12%) and release time (±0.8 s); every seed logged in the trial's
`provenance.json` and the exact jittered definition written to
`definition.json`, so any trial replays exactly.

**12 trials, 12/12 success, 0 collisions, mean clearance 1.149 ± 0.130 m,
worst 0.906 m, 0 false persistent tracks.** Behaviour does not depend on
deterministic exact timing.

## 12. Reactive vs final CV-predictive (§14)

Production configuration throughout; the arms differ only in whether the
predictive layer is enabled.

| scenario | arm | n | succ | coll | mean clr | worst | lead | nav time | path | stop % |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| perpendicular | reactive | 10 | 10/10 | 0 | 0.112 | 0.050 | 0.00 s | 19.00 s | 6.57 | 45.7 |
| perpendicular | **CV** | 10 | 10/10 | 0 | **0.812** | **0.756** | **2.30 s** | 19.47 s | **6.02** | 46.7 |
| reversal | reactive | 10 | **0/10** | **10** | **−0.176** | −0.200 | 0.24 s | — | 3.75 | 72.5 |
| reversal | **CV** | 9 | **9/9** | **0** | **0.725** | **0.694** | **2.49 s** | 19.10 s | 6.12 | 44.9 |
| blind corner | reactive | 10 | 10/10 | 0 | 0.412 | 0.371 | −0.02 s | 19.02 s | 6.24 | 47.6 |
| blind corner | **CV** | 10 | 10/10 | 0 | **0.945** | **0.885** | **2.14 s** | 18.89 s | **5.93** | 46.5 |

Prediction gives **7.3× the clearance** in the crossing, **eliminates 10 of 10
collisions** in the reversal, **2.3×** in the corner world, and buys 2.1–2.5 s
of reaction lead — for comparable navigation time and a *shorter* path. This is
the clearest evidence in the project that the predictive layer is worth
shipping, and it holds outside the nominal benchmark.

## 13. Failure envelope (§15)

Full table: [failure_envelope.csv](stage4g8/failure_envelope.csv).

| condition | trials | coll | worst clr | class | mechanism |
|---|---:|---:|---:|:---:|---|
| obstacle speed 1.00 m/s | 9 | 0 | 0.028 | **H** | reaction lead 0.76 s — insufficient time, not a perception fault |
| encounter timing +1.5 s | 8 | 2 | −0.115 | **H** | conflict arrives inside braking distance |
| head-on (stress) | 10 | 2 | −0.200 | **H** | closing speed leaves minimal braking distance |
| 2 objects converging (stress) | 10 | 6 | −0.200 | **H** | obstacles converge from both sides; no escape corridor |
| 3 objects converging (stress) | 10 | 7 | −0.200 | **H** | as above |
| sensor noise +0.10 m | 9 | 0 | 1.124 | **B** | noise exceeds the 0.15 m static-rejection radius |
| localisation dx ≥ 0.30 m | 20 | 0 | 1.126 | **B** | pose error moves scan points off their map cells (Stage-4G1) |
| brief occlusion | 10 | 0 | 0.110 | **A** | CV coasting coverage is poor; margin reduced, no failure |

**No failure was traced to clustering resolution (C), association (D), the
Kalman/CV model (E), or the costmap (F).** Every collision observed is class
**H** — physical insufficiency of reaction time — and every tracking
degradation is class **A** or **B** — sensing or static-rejection limits.

The three "stress" scenarios are explicitly **harsher than anything previously
validated**: they were invented for Stage-4G8 and place obstacles that converge
on the robot's position regardless of what it does. They are reported to map
the envelope, not as regressions.

## 14. Tracking and prediction accuracy (§17)

Consolidated per-condition table: [results.csv](stage4g8/results.csv) (46 rows).
Representative nominal values: position error **0.16–0.22 m** (dominated by the
known LiDAR surface-vs-centre bias), velocity error **0.03–0.11 m/s**,
prediction **ADE 0.17–0.47 m** and **FDE at 3 s 0.22–0.47 m**, both degrading
smoothly with obstacle speed. Purity 1.000 and matched fraction 1.000 in every
validated multi-object scenario.

## 15. Runtime (§18) and long-duration stability (§19)

Measured with the production configuration on an otherwise idle machine:

| tracks | frames | callback mean | p95 | max | % of 200 ms | layer updateCosts mean / max |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 66 | 14.83 ms | 31.87 | 32.64 | **16.3%** | 16.6 / 42.8 µs |
| 2 | 58 | 18.06 ms | 32.53 | 32.94 | **16.5%** | 26.1 / 157.1 µs |
| 3 | 73 | 16.11 ms | 33.01 | 33.52 | **16.8%** | 15.5 / 41.7 µs |

Large margins against both the 200 ms LiDAR period and any Nav2 costmap budget;
the callback remains dominated by the TF wait, not by tracking. No optimisation
was needed and none was done.

**Endurance — 150 s simulation, 3 obstacles, 752 tracker frames:** goal
SUCCEEDED, **0 false confirmed tracks, 0 false persistent tracks, 0 duplicate
frames**, **9,642 predictions checked with 0 mathematical failures**, costmap
written on 752/752 cycles with multi-object representation 113/113 and 108/109,
2 ID switches, no collisions, no lifecycle or node failures, no deadlock. No
accumulation of stale tracks or costmap cells over 2.5 minutes.

## 16. Automated regression suite (§20)

| test | covers | result |
|---|---|---|
| `stage4g2_lifecycle` | track lifecycle, expiry, reacquisition | **pass** |
| `stage4g3_deblend` | track-aware deblending, static rejection geometry | **pass** |
| `stage4g2_multi_layer` | multi-object costmap, max composition, ghosts | **pass** |
| `stage4g4_reachability` | reachability mathematics (research mode) | **pass** |
| `stage4g4_reach_layer` | reachability costmap mode (research mode) | **pass** |
| `stage4g5_hybrid_layer` | hybrid policy (research mode) | **pass** |
| 4 evaluator unit tests | scorer correctness | **pass** |

**The research modes remain fully functional** under the production build even
though production never selects them.

## 17. Acceptance criteria

| # | Criterion | Verdict |
|---|---|---|
| 1 | Explicit, reproducible CV-only production config | **PASS** — §1 |
| 2 | Nominal Stage-4F/4G behaviour reproduced | **PASS** — §3 |
| 3 | Obstacle-speed robustness quantified | **PASS** — §4 |
| 4 | Encounter-timing sensitivity quantified | **PASS** — §5 |
| 5 | Sensor-noise robustness evaluated | **PASS** — §6 |
| 6 | Localisation robustness evaluated | **PASS** — §7 |
| 7 | Open-room and geometry-rich worlds | **PASS** — §8 |
| 8 | Two-object tracking stable | **PASS** — §9 |
| 9 | Three-object tracking stable | **PASS** — §9 |
| 10 | Deblending remains functional | **PASS** — §9 |
| 11 | Brief-occlusion behaviour documented | **PASS (limitation)** — §10 |
| 12 | Static false persistent tracks controlled | **PASS with bound** — 0 under nominal, noise ≤ 0.06 m, jitter, geometry-rich and endurance; quantified degradation above those bounds (§6, §7) |
| 13 | Randomised trials reproducible | **PASS** — seeds + definitions logged (§11) |
| 14 | Compared against reactive | **PASS** — §12 |
| 15 | Failure envelope documented | **PASS** — §13 |
| 16 | Runtime inside budgets | **PASS** — 16.8% of period (§15) |
| 17 | Long-duration stability | **PASS** — §15 |
| 18 | Regression suite passes | **PASS** — §16 |
| 19 | Research modes available, not default | **PASS** — §1, §16 |
| 20 | No retuning per scenario | **PASS** — §2 |
| 21 | `stage4f-validated` unchanged | **PASS** |
| 22 | `stage4g7-cv-final` unchanged | **PASS** |
| 23 | `~/ur5e_ws` untouched | **PASS** |

## 18. Known limitations

1. **Localisation error degrades static rejection** (class B). Above ~0.30 m /
   0.20 rad of pose error, walls generate false tracks. Navigation remained
   collision-free in all 30 perturbed trials, but this is the operational
   caveat to carry forward: the system assumes reasonable localisation.
2. **Added range noise above ~0.06 m** exceeds the static-rejection radius and
   produces the same effect (class B).
3. **Obstacle speeds above ~0.75 m/s** exhaust the reaction margin (class H).
4. **Brief occlusion reduces margin** because CV coasting coverage is poor
   (class A) — the known, deliberately-accepted CV-only limitation.
5. **Head-on and converging multi-object encounters** can be physically
   unavoidable (class H); these stress scenarios are harsher than any validated
   condition.
6. Eight setup failures in 448 trials (1.8%), all Nav2/tracker readiness
   timeouts under heavy parallel startup; retained and listed in
   `results.json`.
7. Collision-monitor interventions are not recorded by this harness; stop count
   and stopped fraction are the braking proxy.
8. One arena family, one robot, one 5 Hz planar LiDAR, simulation only.

## 19. Exact reproduction

```bash
cd ~/predictive_nav_ws
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
source install/setup.bash

PROD="--production --layer-mode cv_covariance"
REACT="--production --layer-mode reactive"
R=src/predictive_nav_bringup/scripts/stage4g2/run.py
L=validation_logs/stage4g8_repro

# Nominal reproduction against recorded references
python3 $R --out $L/val_a  --scenarios nav_cv_control        --trials 10 --domain 100 $PROD
python3 $R --out $L/val_b  --scenarios nav_reversal          --trials 10 --domain 130 $PROD
python3 $R --out $L/val_c  --scenarios nav_occlusion_short   --trials 10 --domain 160 $PROD
python3 $R --out $L/val_d  --scenarios navigation            --trials 10 --domain 190 $PROD
python3 $R --out $L/val_e  --scenarios triple                --trials 10 --domain 100 $PROD
python3 $R --out $L/val_h  --scenarios resolution_75,resolution_85 --trials 3 --domain 130 $PROD
python3 $R --out $L/val_g  --scenarios static_navigation     --trials 6  --domain 160 $PROD

# Speed and timing sweeps
python3 $R --out $L/speed --trials 10 --domain 100 $PROD \
  --scenarios g8_speed_015,g8_speed_025,g8_speed_05,g8_speed_075,g8_speed_10
python3 $R --out $L/time  --trials 8  --domain 130 $PROD \
  --scenarios g8_time_m15,g8_time_m10,g8_time_m05,g8_time_p00,g8_time_p05,g8_time_p10,g8_time_p15

# Sensor noise, localisation, jitter (all seeded)
for s in 0.03 0.06 0.10; do
  python3 $R --out $L/noise_$s --scenarios g8_perp --trials 10 --domain 100 $PROD --scan-noise $s
done
python3 $R --out $L/loc_a --scenarios g8_perp --trials 10 --domain 130 $PROD --pose-dx 0.15 --pose-dyaw 0.10
python3 $R --out $L/loc_b --scenarios g8_perp --trials 10 --domain 160 $PROD --pose-dx 0.30 --pose-dyaw 0.20
python3 $R --out $L/loc_c --scenarios g8_perp --trials 10 --domain 190 $PROD --pose-dx 0.50 --pose-dyaw 0.35
python3 $R --out $L/jitter --scenarios g8_perp --trials 12 --domain 100 $PROD --jitter 1.0 --seed 4008

# Geometry, occlusion, multi-object, endurance
python3 $R --out $L/geom  --scenarios g8_corner,g8_occlusion --trials 10 --domain 130 $PROD
python3 $R --out $L/multi --scenarios g6_mixed              --trials 10 --domain 160 $PROD
python3 $R --out $L/endur --scenarios g8_endurance          --trials 2  --domain 190 $PROD

# Reactive comparison
python3 $R --out $L/react_a --scenarios nav_cv_control --trials 10 --domain 100 $REACT
python3 $R --out $L/react_b --scenarios nav_reversal   --trials 10 --domain 130 $REACT
python3 $R --out $L/react_c --scenarios g8_corner      --trials 10 --domain 160 $REACT

# Analysis
python3 src/predictive_nav_bringup/scripts/stage4g8/analyze.py \
  --logs $(ls -d $L/*/) --out $L/all_trials.json
python3 src/predictive_nav_bringup/scripts/stage4g8/summarize.py \
  --trials $L/all_trials.json --out-dir $L/curated

# Regression suite
ROS_DOMAIN_ID=94 build/predictive_nav_tracking/stage4g2_lifecycle
ROS_DOMAIN_ID=95 build/predictive_nav_tracking/stage4g3_deblend
ROS_DOMAIN_ID=96 build/predictive_nav_tracking/stage4g4_reachability
ROS_DOMAIN_ID=93 build/predictive_nav_costmap/stage4g2_multi_layer
ROS_DOMAIN_ID=97 build/predictive_nav_costmap/stage4g4_reach_layer
ROS_DOMAIN_ID=98 build/predictive_nav_costmap/stage4g5_hybrid_layer
python3 -m unittest discover -s src/predictive_nav_bringup/scripts/stage4g2 -p 'test_*.py'
```

Keep `--domain` below 226: the runner adds a per-trial offset and DDS rejects
ids above 232. Do not run more than about four trials concurrently — heavier
oversubscription produces readiness timeouts.

## 20. Recommendation

**The project is ready for FINAL FREEZE.**

The architecture is frozen and explicit, its nominal behaviour reproduces the
recorded references, it degrades gracefully across every perturbation tested,
its breakdown points are physical rather than algorithmic, the regression suite
passes, runtime has a 6× margin, and a 2.5-minute run shows no accumulation of
any kind. Prediction demonstrably earns its place against a reactive baseline
well outside the nominal benchmark.

Freeze with the operating envelope stated plainly: **obstacle speeds ≤ 0.75 m/s,
localisation error ≲ 0.30 m / 0.20 rad, added range noise ≲ 0.06 m**, and the
accepted CV-only limitation that a brief observation gap reduces margin without
causing failure. No further algorithm stage is warranted by anything measured
here.
