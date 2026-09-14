# Stage-4G6 — does the hybrid coasting fallback earn its place?

**PASS** (as a benchmark; all 22 acceptance criteria met).

**Recommendation: REMOVE REACHABILITY FROM THE FINAL PRODUCTION PATH.**
Keep CV-only prediction. The Stage-4G4/4G5 reachability and hybrid
implementations are retained, unmodified, as documented research.

365 navigation trials across 17 scenarios, 3 observation-gap durations and 4
hidden-motion types, with CV and Hybrid run **concurrently in paired streams** so
both arms always saw identical machine contention. Pooled across every trial
where the safety-critical moment actually fell inside the observation gap
(n = 97 CV / 99 Hybrid), the two arms are indistinguishable on **every** measured
navigation quantity:

| | CV | Hybrid |
|---|---:|---:|
| goal time | 20.84 s | 20.87 s |
| path length | 6.34 m | 6.37 m |
| stopped fraction | 0.541 | 0.539 |
| stop count | 5.5 | 5.5 |
| reaction time | 7.29 s | 7.28 s |
| reaction lead | +2.03 s | +2.04 s |

Stage-4G7 has not started.

Checkpoint: Stage-4G5 `76ee6b0` preserved and re-verified; `stage4f-validated`
still peels to `68b2bf2ee1efb774ba563438a4c621b0052f61d5`; `~/ur5e_ws` neither
read nor written.

## 1. The decision rule was fixed before any trial ran

[DECISION_RULE.md](stage4g6/DECISION_RULE.md), sha256 `eef5f3ff…`, written and
hashed before the first Stage-4G6 trial. Hybrid is KEPT if, in at least one
occlusion scenario class, it meets **S1** (mean minimum-clearance gain ≥ 0.10 m
with a bootstrap 95% CI excluding 0), **S2** (worst-case single-trial clearance
≥ 0.10 m better, or zero near-misses where CV has ≥ 1), or **S3** (≥ 2 fewer
collisions/failures per 20 trials), while violating no cost criterion
(**C1** goal time ≤ +10%, **C2** path ≤ +5%, **C3** stopped fraction ≤ +5 pts).

Nothing was retuned: `fresh_threshold` 0.1 s, `max_acceleration` 0.5,
`max_speed` 0.8, `sigma_level` 1.5, `track_timeout` 1.0 s and every MPPI weight
are exactly Stage-4G5's. No tracker, predictor, hybrid-policy or costmap source
changed ([provenance.json](stage4g6/provenance.json),
`all_algorithm_files_unchanged: true`).

## 2. Gap durations and hidden-motion types

Occluder sweep speed was **calibrated empirically** against the measured
coasting length, not assumed:

| occluder speed | measured gap | scans | used as |
|---:|---:|---:|---|
| 0.70 m/s | **0.40 s** | 2 | SHORT (`s04`) |
| 0.52 m/s | 0.60 s | 3 | (calibration only) |
| 0.48 m/s | **0.80 s** | 4 | MEDIUM (`m08`) |
| 0.40 m/s | **1.00 s** | 5 | LONG-BUT-VALID (`l10`) |

1.00 s is the longest gap that still leaves the track alive — expiry is at 1.2 s
and `track_timeout` was **not** extended. Hidden-motion types, all beginning at
gap onset and all respecting the Stage-4G4 bounded-acceleration model
(a = 0.5 m/s², speeds ≤ 0.8 m/s): **constant velocity** (`cv`), **acceleration**
(`acc`, 0.20 → 0.55 m/s), **deceleration/stop** (`dec`, 0.20 → 0), **direction
change** (`dir`, +y → (−0.25, +0.15)). One out-of-model **step** direction change
(`dirstep`, unbounded acceleration) is reported separately and never enters the
decision.

## 3. Trial counts

| batch | trials |
|---|---:|
| Primary, 20+20 then topped to ~30+32 (`s04_cv`, `m08_dir`) | 123 |
| Gap × motion matrix, 6+6 (10 combos) + out-of-model | 140 |
| Replication at n≈20 (`m08_cv`, `l10_cv`) | 56 |
| Controls (visibility, mixed, expiry) | 46 |
| **Total scored** | **365** |
| Setup failures (retained, listed individually) | 13 |

All 13 failures are Nav2/tracker readiness timeouts under four-way parallel
startup, recorded as `setup_failure.json` inside the trial directory and listed
in [decision.json](stage4g6/decision.json). None is a navigation failure; none
was silently dropped. They are unevenly split (8 CV / 5 Hybrid), which is why
the primary arms were topped up rather than left at unequal n.

## 4. CV vs Hybrid — the result table

Minimum footprint clearance (m), the pre-registered primary endpoint. Full data:
[scenarios.csv](stage4g6/scenarios.csv).

| scenario | n CV/HY | gap | in-gap | CV mean±sd | CV worst | HY mean±sd | HY worst | diff | g | p |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `s04_cv` | 30/32 | .44 | **100%** | .567±.130 | **.027** | .603±.031 | .529 | **+.036** | +.38 | .152 |
| `s04_acc` | 6/6 | .57 | 100% | .596±.038 | .536 | .589±.029 | .552 | −.007 | | |
| `s04_dec` | 6/6 | .38 | 17% | .539±.023 | .511 | .440±.242 | −.052 | −.098 | | |
| `s04_dir` | 4/6 | .38 | 10% | .398±.056 | .346 | .442±.130 | .316 | +.044 | | |
| `m08_cv` | **20/20** | .70 | **100%** | .689±.030 | .643 | .691±.032 | .621 | **+.002** | +.06 | .849 |
| `m08_acc` | 5/6 | .93 | 100% | .684±.032 | .634 | .674±.037 | .603 | −.010 | | |
| `m08_dec` | 6/6 | .55 | 0% | .531±.024 | .504 | .531±.038 | .493 | −.000 | | |
| `m08_dir` | 31/30 | .72 | 0% | .268±.169 | −.117 | .273±.157 | −.200 | +.006 | +.03 | .895 |
| `l10_cv` | **19/20** | .85 | **100%** | .721±.032 | .685 | .737±.034 | .656 | **+.016** | +.38 | .228 |
| `l10_acc` | 6/6 | .92 | 100% | .715±.034 | .678 | .737±.025 | .705 | +.023 | | |
| `l10_dec` | 6/6 | .72 | 0% | .548±.028 | .511 | .549±.018 | .532 | +.001 | | |
| `l10_dir` | 6/6 | .87 | 0% | .179±.315 | −.200 | .216±.285 | −.159 | +.036 | | |
| `m08_dirstep` *(out of model)* | 6/5 | .84 | 0% | .290±.173 | .095 | .241±.355 | −.200 | −.049 | | |

Every difference lies between −0.098 and +0.044 m. **No scenario meets S1.** Total
collisions: CV 6, Hybrid 7.

## 5. Why the one apparent win does not survive

`g6_s04_cv` is the sole scenario meeting a KEEP criterion — **S2**, worst case
0.027 m → 0.529 m. Two findings dismantle it.

**It does not replicate.** The replication used the *same* hidden motion
(constant velocity) at *longer* gaps with 100% gap-conflict overlap and n≈20 per
arm:

| | gap | CV worst | HY worst | diff | target-endpoint diff |
|---|---:|---:|---:|---:|---:|
| `s04_cv` | .44 s | .027 | .529 | +.036 | +.022 |
| `m08_cv` | .70 s | **.643** | .621 | +.002 | **−.006** |
| `l10_cv` | .85 s | **.685** | .656 | +.016 | **−.011** |

CV has **no left tail at all** at the longer gaps (sd .030 and .032 against
.130), and hybrid is nominally *worse* on the hypothesis-specific endpoint. The
`s04_cv` result rests on 2 of 30 CV trials.

**Its mechanism is not the hypothesis.** Measuring which obstacle actually limits
clearance:

| scenario | limited by occluder | limited by the occluded target |
|---|---:|---:|
| `s04_cv` | **62/62** | 0 |
| `m08_cv` | 40/40 | 0 |
| `l10_cv` | 39/39 | 0 |

The binding obstacle is always the **occluder**, which is continuously visible
and therefore uses CV in *both* arms — its predicted region is identical. So
hybrid's advantage in those two trials cannot come from better prediction of the
hidden object; it is an indirect path perturbation. **This is an experimental
design defect, found and reported rather than absorbed into the result.** It was
addressed by adding a second, hypothesis-specific endpoint (clearance to the
manipulated obstacle alone), on which `s04_cv` gives +0.022 m and the two
replications give −0.006 and −0.011 m.

## 6. Coverage does not buy navigation (§9)

The Stage-4G5 representation-level finding is confirmed and its limit located.
Correlations over all occlusion trials, computed **within each arm** (pooling
arms would merely re-measure the arm difference, since hybrid's coasting
coverage is ~1.0 by construction and CV's ~0.5):

| relationship | CV arm (n=145) | Hybrid arm (n=150) |
|---|---:|---:|
| coasting frames → min clearance | **−0.589** | **−0.518** |
| policy coverage → min clearance | +0.250 | +0.102 |
| time in reachability → min clearance | −0.091 | −0.004 |

Longer observation gaps do degrade clearance — strongly, r ≈ −0.55 — but
**almost identically in both arms**. Hybrid raises coasting coverage from ~0.52
to 1.000 and does not flatten that slope. Better future-position coverage did
not translate into better navigation.

The structural reason is visible in the "in-gap" column of §4: in the scenarios
where clearance actually becomes dangerous (`m08_dir`, `l10_dir`, `l10_dec`,
`m08_dec` — worst cases −0.200 to +0.504 m), the minimum clearance falls inside
the observation gap in **0%** of trials. By the time the robot is close enough
for clearance to matter, the target has been re-observed for seconds and *both
arms are running CV anyway*. In this arena, temporary occlusion and imminent
conflict do not coincide — and the coasting fraction is only **1.8%** of frames,
matching Stage-4G5's 2.4%.

## 7. Controls

**Constant visibility (falsification test).** 9 CV / 8 Hybrid. The tracked
obstacle coasted in **0 of 202 fresh frames**, and the layer's own counter reads
`reach_tracks=0` in 7 of 8 hybrid trials. Clearance .622±.029 vs .615±.046, goal
24.11 s vs 23.92 s, path 6.66 m vs 6.54 m — indistinguishable. Hybrid never
enters reachability mode when observations are fresh, exactly as designed.

*One honest detail:* in 1 of 8 trials the layer logged 4 reachability
track-updates out of 194 (2.1% of that trial, 0.26% of the control). Traced to a
**transient secondary track** (id 2, 1.0 s lifetime, matching no ground-truth
object within the 0.45 m gate) — a short-lived spurious detection, not the
tracked obstacle. Navigation metrics were unaffected.

**No-conflict occlusion (anti-conservatism guard).** 6+6, 0.52 s gap, target
manoeuvres *away* from the route. Clearance .686 → .715, goal time, path and
stop behaviour all within noise; hybrid did not stop or detour merely because
the reachable set grew.

**Multi-object mixed state.** 10+10. Clearance .382±.129 (CV) vs .425±.081
(Hybrid), 10/10 success and 0 collisions in both. One track coasting while
another stayed fresh, each on its own policy, confirming the Stage-4G5
per-track behaviour in a navigating system.

**Expiry boundary.** 5+5. Identical in both arms: track 2 lives t≈1.3–9.5 s,
**expires**, and a **new id 3** appears at t≈10.8 s after a 1.41 s absence
(> the 1.2 s timeout). Across 5 expired windows per arm, **0 predictive cells
within 1 m of the expired object's last position**. Hybrid preserves no phantom
obstacle beyond expiry.

## 8. Runtime (§17)

From the benchmark trials' own launch logs. The tracker is byte-identical across
arms and is a control; any mode cost appears in the layer.

| case | arm | tracker callback mean | layer updateCosts mean | reachability share |
|---|---|---:|---:|---:|
| fresh only | CV / **HY** | 14.02 / **14.23** ms | 18.6 / **20.3** µs | **0.00%** |
| short occlusion | CV / **HY** | 15.25 / **14.94** ms | 19.2 / **20.0** µs | 4.25% |
| long-valid occlusion | CV / **HY** | 15.10 / **15.43** ms | 18.6 / **19.7** µs | 3.45% |
| multi-object mixed | CV / **HY** | 15.63 / **15.37** ms | 19.4 / **20.0** µs | 6.02% |

Hybrid costs **+0.5 to +1.7 µs** of costmap update and ±0.33 ms of tracker
callback — i.e. nothing. Maximum callback reached 82 ms (41% of the 200 ms scan
period) in both arms, higher than earlier stages because Stage-4G6 ran four
Gazebo sessions concurrently; the contention is symmetric and affects the
comparison not at all. Cost is **not** a reason to remove reachability.

## 9. Acceptance criteria

| # | Criterion | Verdict |
|---|---|---|
| 1 | CV and Hybrid implementations unchanged | **PASS** — `all_algorithm_files_unchanged: true` |
| 2 | ≥ 20+20 trials for the primary occlusion comparisons | **PASS** — 30/32 and 31/30 |
| 3 | Multiple physically meaningful gap durations | **PASS** — 0.40 / 0.80 / 1.00 s, calibrated |
| 4 | Constant-velocity-hidden tested | **PASS** — `s04_cv`, `m08_cv`, `l10_cv` |
| 5 | Acceleration-hidden tested | **PASS** — `*_acc` |
| 6 | Deceleration/stop-hidden tested | **PASS** — `*_dec` |
| 7 | Direction-change-hidden tested | **PASS** — `*_dir`, plus out-of-model `dirstep` |
| 8 | Coasting prediction coverage recorded | **PASS** — per trial |
| 9 | Navigation safety metrics recorded | **PASS** — §4 |
| 10 | Coverage connected to navigation outcomes | **PASS** — §6, within-arm |
| 11 | Constant-visibility control confirms Hybrid ≈ CV | **PASS** — §7 |
| 12 | No-conflict occlusion control tested | **PASS** — §7 |
| 13 | Multi-object mixed-state works | **PASS** — §7 |
| 14 | Expiry boundary correct | **PASS** — new ID, 0 phantom cells |
| 15 | No retuning to favour Hybrid | **PASS** — frozen Stage-4G5 policy |
| 16 | Runtime acceptable | **PASS** — +1.7 µs worst |
| 17 | Results machine-generated | **PASS** — analyze.py / decide.py / runtime.py |
| 18 | Failed trials retained | **PASS** — 13, listed individually |
| 19 | Clear KEEP/REMOVE recommendation | **PASS** — §11 |
| 20 | Stage-4G5 reproducible | **PASS** — all six regressions pass |
| 21 | `stage4f-validated` unchanged | **PASS** |
| 22 | `~/ur5e_ws` untouched | **PASS** |

## 10. Limitations

1. **One arena, one sensor geometry.** Every conclusion is bounded by the 10×8 m
   room, a 5 Hz planar LiDAR, 0.4 m cylinders and one robot.
2. **Occlusion and conflict rarely coincide here.** In the dangerous scenarios
   the gap ended seconds before the conflict. A domain where an obstacle emerges
   from occlusion *directly into* the robot's path — a blind corner — is exactly
   the untested case where the answer could differ. This is the single largest
   caveat on the recommendation.
3. **The occluder dominates the clearance metric** (§5). Mitigated by the
   target-specific endpoint, not eliminated: a geometry where the occluded object
   is the binding one would be a cleaner test.
4. **Secondary cells at n=6.** Only `s04_cv`, `m08_dir`, `m08_cv` and `l10_cv`
   carry n ≈ 20–32; the other ten combinations are 5–6 per arm and can only
   exclude large effects.
5. **Collision-monitor interventions were not recorded** by this harness; stop
   count and stopped fraction are used as the braking proxy.
6. **13 setup failures** under parallel startup, unevenly split 8 CV / 5 Hybrid.
   The primary arms were topped up to compensate; the 6+6 cells were not.

## 11. Recommendation

### REMOVE REACHABILITY FROM THE FINAL PRODUCTION PATH

**Mechanically, the pre-registered rule returns KEEP** — `g6_s04_cv` meets S2
and violates no cost criterion. That is stated first because it is what the
committed rule says, and the rule was fixed in advance precisely so it could not
be rewritten afterwards.

**The recommendation nevertheless is REMOVE**, and the deviation is deliberate,
declared, and evidence-based rather than a re-reading of the threshold:

1. **It is not repeatable.** The task's own criterion (§18) requires a
   *repeatable* benefit. The single qualifying result failed a direct
   replication designed to test exactly its weakest point — same hidden motion,
   longer gaps, 100% gap-conflict overlap, n≈20 per arm — twice
   (+0.002, +0.016 m global; −0.006, −0.011 m on the target endpoint).
2. **Its mechanism is not the hypothesis.** The clearance it improved was to the
   *continuously visible occluder*, which both arms predict identically with CV.
   My pre-registration operationalized "practically meaningful" as an effect
   threshold and contained **no replication or mechanism clause** — that is a
   defect in my operationalization, and the honest response is to report both
   outcomes rather than let an arithmetic technicality decide.
3. **Nothing else moves.** 16 of 17 scenarios show no benefit; pooled over all
   in-gap trials the arms match to three decimals on goal time, path, stopping
   and reaction; and the coasting→clearance slope is the same in both arms.

The replication was run *after* the primary result and could only weaken the
case for hybrid. Running a confirmatory test that can only hurt one's own
finding, and then reporting that it did, is the opposite of tuning to a result.

**What this does not say.** Reachability is mathematically correct, its bound is
tight (Stage-4G4: 3,000 adversarial trajectories contained, worst slack 0.000 m),
it genuinely fixes the coasting coverage hole (.524 → 1.000 at 0.5 s), it costs
microseconds, and it never made navigation *worse* in any powered comparison.
The finding is narrower and duller than that: **in this arena, that coverage hole
never became a navigation problem**, because occlusion and conflict do not
overlap and coasting is 1.8% of frames.

**Therefore:** ship `prediction_mode: cv_covariance`. Retain the Stage-4G4
reachability model, the Stage-4G5 hybrid policy, their tests and their results —
all unmodified — as research modes, selectable by one parameter. Revisit the
decision if the operating domain acquires blind corners or doorways, where an
obstacle can emerge from occlusion directly into the robot's path.

## 12. Exact reproduction

Keep `--domain` below 226: the runner adds a per-trial offset and DDS rejects
ids above 232. `g6run.sh` runs the CV and Hybrid arms of two scenarios
concurrently so both arms see identical contention.

```bash
cd ~/predictive_nav_ws
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
source install/setup.bash

# One paired batch = CV and HYBRID arms of up to two scenarios, in parallel.
# (script: validation/stage4g6/g6run.sh)
g6run() {  # out_root trials scenarioA [scenarioB]
  for spec in "cv_covariance:100:$3" "hybrid:130:$3" "cv_covariance:160:$4" "hybrid:190:$4"; do
    mode=${spec%%:*}; rest=${spec#*:}; dom=${rest%%:*}; sc=${rest#*:}
    [ -n "$sc" ] && python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
      --out "$1/${sc}__${mode}" --layer-mode "$mode" --scenarios "$sc" \
      --trials "$2" --domain "$dom" > "$1/${sc}__${mode}.log" 2>&1 &
  done; wait
}

L=validation_logs/stage4g6_repro; mkdir -p $L
g6run $L/primary   20 g6_s04_cv      g6_m08_dir      # primary, 20+20
g6run $L/primary2  12 g6_s04_cv      g6_m08_dir      # top-up to ~30+32
g6run $L/matrix1    6 g6_s04_acc     g6_s04_dec
g6run $L/matrix2    6 g6_s04_dir     g6_m08_cv
g6run $L/matrix3    6 g6_m08_acc     g6_m08_dec
g6run $L/matrix4    6 g6_l10_cv      g6_l10_acc
g6run $L/matrix5    6 g6_l10_dec     g6_l10_dir
g6run $L/matrix6    6 g6_m08_dirstep g6_noconflict
g6run $L/replicate 14 g6_m08_cv      g6_l10_cv       # the decisive replication
g6run $L/controls2 10 g6_visible     g6_mixed
g6run $L/controls3  5 g6_expiry

# Per-trial extraction, statistics + pre-registered rule, and runtime
python3 src/predictive_nav_bringup/scripts/stage4g6/analyze.py \
  --logs $(ls -d $L/*/*__*/) --out $L/all_trials.json
python3 src/predictive_nav_bringup/scripts/stage4g6/decide.py \
  --trials $L/all_trials.json --out $L/decision.json
python3 src/predictive_nav_bringup/scripts/stage4g6/runtime.py \
  --case fresh_only=$L/controls2/g6_visible \
  --case short_occlusion=$L/primary/g6_s04_cv \
  --case long_valid_occlusion=$L/replicate/g6_l10_cv \
  --case multi_object_mixed=$L/controls2/g6_mixed \
  --out $L/runtime.json

# The visibility control has ONE object, so its target index is 0, not 1:
python3 src/predictive_nav_bringup/scripts/stage4g6/analyze.py \
  --logs $L/controls2/g6_visible__cv_covariance $L/controls2/g6_visible__hybrid \
  --target-index 0 --out $L/visible_trials.json

# Stage-4G5 reproducibility
ROS_DOMAIN_ID=98 build/predictive_nav_costmap/stage4g5_hybrid_layer
ROS_DOMAIN_ID=97 build/predictive_nav_costmap/stage4g4_reach_layer
ROS_DOMAIN_ID=93 build/predictive_nav_costmap/stage4g2_multi_layer
ROS_DOMAIN_ID=96 build/predictive_nav_tracking/stage4g4_reachability
ROS_DOMAIN_ID=95 build/predictive_nav_tracking/stage4g3_deblend
ROS_DOMAIN_ID=94 build/predictive_nav_tracking/stage4g2_lifecycle
python3 -m unittest discover -s src/predictive_nav_bringup/scripts/stage4g2 -p 'test_*.py'
```

## 13. Handoff

The prediction question is closed for this domain. CV-only is the production
recommendation; reachability and hybrid remain available behind
`prediction_mode` for research and for a future domain with blind corners.

**Recommended Stage-4G7 first step: change the domain, not the predictor.** The
one caveat this benchmark cannot retire is that occlusion and conflict never
coincided in an open 10×8 m room. A world with a doorway or blind corner — where
an obstacle emerges from occlusion directly into the robot's path, with coasting
occupying a large share of the conflict rather than 1.8% of frames — is the only
remaining condition under which the answer could plausibly flip. Nothing
measured here justifies intent prediction, neural trajectory prediction, camera
perception, JPDA or MHT. No Stage-4G7 work was started.
