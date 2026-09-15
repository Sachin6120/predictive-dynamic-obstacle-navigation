# Stage-4G7 — prediction through static (blind-corner) occlusion: the final decision

**PASS** (as a decision benchmark; all 26 acceptance criteria addressed).

# FINAL PRODUCTION RECOMMENDATION: SHIP CV-ONLY

Ship `prediction_mode: cv_covariance`. The Stage-4G4 reachability model and the
Stage-4G5 hybrid policy are retained **unchanged** — code, tests, results and
documentation — as research modes selectable by one parameter. Nothing is
deleted.

282 trials in purpose-built blind-corner worlds where **static map geometry**
hides a **previously-tracked** dynamic obstacle. This removed the Stage-4G6
confound completely: the occluded target is the clearance-limiting object in
**100%** of primary trials (Stage-4G6: 0%). Hybrid's coasting coverage advantage
here is the largest ever measured in this project — **0.207 → 1.000** at 0.5 s —
and it still produced **no measurable safety benefit**:

| primary scenario `g7_gap08`, n = 30 + 30 | CV | Hybrid |
|---|---:|---:|
| target min clearance, mean ± sd | 0.939 ± 0.028 m | 0.944 ± 0.031 m |
| 10th percentile | 0.906 m | 0.897 m |
| worst single trial | 0.889 m | 0.893 m |
| target collisions | 0 | 0 |
| goal reached | 30/30 | 30/30 |

Difference **+0.005 m**, 95% CI **[−0.010, +0.019]**, Hedges' g = +0.17, p = 0.51.

Stage-4G8 and every later stage have not been started.

Checkpoint: Stage-4G6 `5281f23` preserved and re-verified; `stage4f-validated`
still peels to `68b2bf2ee1efb774ba563438a4c621b0052f61d5`; `~/ur5e_ws` neither
read nor written.

## 1. The decision rule was fixed first, and it is stricter than Stage-4G6's

[DECISION_RULE.md](stage4g7/DECISION_RULE.md), sha256 `446f53e0…`, written and
hashed before any decision trial. It repairs the two defects that let
Stage-4G6's rule return a KEEP its own follow-up evidence contradicted: it adds
a mandatory **replication** gate and a mandatory **mechanism** gate. All four
gates — A practical effect, B replication, C mechanism, D cost — are required.

No parameter was retuned (`provenance.json`, `all_algorithm_files_unchanged:
true`): the tracker, Kalman filter, CV predictor, reachability mathematics,
hybrid policy, `fresh_threshold`, `track_timeout`, deblending, association,
predictive-costmap mathematics and MPPI configuration are all byte-identical to
Stage-4G5.

## 2. World geometry

Two worlds, each the validated Stage-4F 10 × 8 m room and its four localisation
pillars **plus** static occluders, emitted world-and-map together by
`scripts/stage4g7/make_worlds.py` so the SDF and the PGM cannot drift apart —
which matters more here than ever, since a wall present in one but not the other
would become a permanent false dynamic obstacle.

| world | occluders | used by |
|---|---|---|
| `stage4g7_c30` | one 0.30 × 0.15 m stub at (−0.80, 0.85) | primary, turn, controls |
| `stage4g7_c45` | one 0.45 × 0.15 m stub at (−0.80, 0.85) | longer gap |
| `stage4g7_door` | walls at y = 0.85 leaving a 0.70 m doorway at x ∈ [−0.45, 0.25] | doorway |

Every occluder's south face is at y = 0.775, i.e. at or beyond
`inflation_radius` (0.70 m) from the route centre, so the robot's route keeps
the ~zero static inflation Stage-4F was built to guarantee.

The layouts were not guessed. `scripts/stage4g7/design.py` solves the
line-of-sight problem offline — when the LiDAR loses the target behind static
geometry, when closest approach occurs, and how much of the pre-conflict
interval is unobserved — and only layouts that pass were simulated. It also
checks that the **target's own path clears the static geometry**, a check added
after a pilot showed the target driving into the occluder and stopping 1.1 m
short of the route.

## 3. The structural finding, and it is the important one

**Across more than 40,000 candidate layouts in ten systematic sweeps, no static
occlusion geometry in this arena produces all three of: a track established
before occlusion, a gap inside the 1.2 s track lifetime, and the closest
approach occurring inside that gap.** The constraints are mutually exclusive,
for reasons that are geometric rather than incidental:

1. **A static occluder hides an approaching robot's view for seconds, not
   fractions of one.** Only 1 layout in the first 3,600-layout sweep gave a gap
   ≤ 1.05 s. Short gaps require a *narrow* occluder — a post or a door jamb —
   because it is the robot's own motion that sweeps the shadow past the target.
2. **A narrow occluder's shadow reaches only a short way past it**, so the target
   re-emerges while still far from the route. Measured emergence-to-closest-
   approach lead over 180 valid layouts: 3.6 s minimum, 5.4 s median.
3. **Conflict timing forces a slow target** (~0.27 m/s to arrive when the robot
   does), and a slow target sits in any given shadow longer, pushing the gap
   back over the lifetime limit.
4. A **doorway** with walls long enough to be a doorway casts a 2–5.6 s shadow.
   That is an *expiry* case, not a coasting case.

Achieved in the final design: gaps of 0.40 / 0.47 / 0.60 s, 15 tracked
observations before occlusion, conflict overlap 0.14–0.15, and the closest
approach falling ~1.0 s **before** the gap ends rather than inside it. This is a
large improvement on Stage-4G6 but still short of the ideal, and it is reported
as a bound on what this arena can test, not as a success.

## 4. Results — CV vs Hybrid

Target-specific minimum clearance (m). Full data: [scenarios.csv](stage4g7/scenarios.csv).

| scenario | world | n CV/HY | gap | target limiting | CV mean±sd | CV worst | HY mean±sd | HY worst | diff | g | p |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **`g7_gap08`** *(primary)* | c30 | **30/30** | .47 | **100%** | .939±.028 | .889 | .944±.031 | .893 | **+.005** | +.17 | .51 |
| `g7_gap06` | c30 | 20/19 | .40 | 100% | 1.026±.033 | .968 | 1.007±.025 | .958 | −.019 | | |
| `g7_gap10` | c45 | 20/18 | .60 | 100% | .956±.034 | .902 | .944±.039 | .887 | −.012 | | |
| `g7_turn` | c30 | 18/19 | .55 | 100% | −.133±.052 | −.200 | −.130±.054 | −.200 | +.003 | | |
| `g7_door` | door | 8/8 | .00 | 100% | .834±.038 | .780 | .850±.027 | .801 | +.015 | | |
| `g7_noconflict` | c30 | 10/9 | .50 | 100% | .676±.027 | .647 | .701±.042 | .650 | +.024 | | |
| `g7_multi` | c30 | 8/6 | .53 | 0% | .936±.018 | .911 | .925±.013 | .907 | −.012 | | |
| `g7_visible` | c30 | 10/10 | .28 | 100% | .910±.016 | .882 | .885±.023 | .850 | −.025 | | |
| `g7_turnstep` *(out of model)* | c30 | 19/20 | .94 | 100% | .302±.047 | .211 | .318±.032 | .269 | +.016 | | |

Every difference lies between **−0.025 and +0.024 m**, against a 0.10 m
threshold. Target collisions: **0 vs 0** everywhere except `g7_turn`, where
**both arms collide in 18 of 18 trials**.

## 5. The four gates

| gate | verdict | evidence |
|---|---|---|
| **A. Practical effect** | **FAIL** | A1 mean gain +0.005 m, CI [−0.010, +0.019] (needs ≥0.10 and CI excluding 0). A2 worst-case +0.004 m. A3 10th-percentile **−0.009 m**. A4 target collisions 0 vs 0. |
| **B. Replication** | **n/a → FAIL** | No qualifying effect exists to replicate. The three candidate variants give −0.019, −0.012, +0.003 m. |
| **C. Mechanism** | **FAIL on C3** | C1 ✔ target is clearance-limiting in 100%/100%. C2 ✔ reachability genuinely in use (4.2 coasting frames; the layer logs 13–15% reachability track-updates). **C3 ✘** — costmap cells during the gap differ by −64, CI [−253, +127]; robot speed during the gap by −0.023 m/s, CI [−0.074, +0.028]. Both indistinguishable from zero. |
| **D. Cost** | **PASS** (no violation) | goal time +1.1%, path +0.13%, stopped fraction +0.0 pts. |

**FINAL: SHIP CV-ONLY.** Unlike Stage-4G6, this is not a deviation from the
pre-registered rule — the rule and the evidence agree.

## 6. The causal chain — where it breaks (§22)

| step | measured | works? |
|---|---|---|
| 1. Hybrid → better target coverage while coasting | CV **0.207** → Hybrid **1.000** at 0.5 s | **YES**, decisively |
| 2. → different predictive costmap **before** reacquisition | −64 cells, CI [−253, +127] | **NO** |
| 3. → different robot command **before** reacquisition | −0.023 m/s, CI [−0.074, +0.028] | **NO** |
| 4. → better target-specific safety | +0.005 m, CI [−0.010, +0.019] | **NO** |

**The chain breaks at step 2.** A fivefold coverage improvement does not produce
a measurably different costmap before the target is re-observed. Two reasons,
both measured: the coasting window is only ~4 scans long, and the layer's
pre-existing `max_influence_radius` policy caps how far any single prediction
may paint — so the extra reachable area is bounded and is diluted among the many
cells the CV predictions already paint. Hybrid nominally paints *fewer* cells
during the gap, a consequence of the documented Stage-4G4 ellipse-versus-
bounding-box artefact in CV mode.

This is a stronger negative result than a correlation: the mechanism was
instrumented directly and it is absent.

## 7. Controls — including the two that did not work

**Occluded no-conflict** ✔ — same occlusion, target turns away. Clearance
0.676 → 0.701 m, stopped fraction 0.516 vs 0.514, goal time 15.2 s both.
Hybrid does **not** stop or detour merely because its coasting envelope grows.

**Multi-object mixed state** ✔ — hidden target plus a continuously visible
southern obstacle. 8/8 and 6/6 goals reached, zero collisions, and the *other*
obstacle is clearance-limiting in 100% of trials, as expected. Per-track policy
selection behaved as Stage-4G5 pinned it.

**Constant visibility — PARTIALLY FAILED, reported rather than hidden.** The
control placed the target south of the route, where a wall at y = +0.85 cannot
occlude it. It is nevertheless briefly occluded (1–2 scans) *after* it crosses,
at t ≈ 7.4 s — about 2 s **after** the closest approach at t ≈ 5.4 s — so the
layer logged 4–24 reachability track-updates of ~155 (2.6–15%) instead of the
required zero. The safety comparison in this control is unaffected (0.910 vs
0.885 m) and the criterion-16 requirement of **exactly zero** reachability
frames is satisfied instead by the Stage-4G6 constant-visibility control, which
ran in a world with no occluder at all and logged `reach_tracks=0` in 7 of 8
trials with the byte-identical hybrid implementation.

**Never-observed and expiry controls — NOT ACHIEVED AS DESIGNED.** The doorway
scenario was intended to hide the target from the start and force expiry.
Because the target descends *through* the door opening it remained visible, and
`g7_door` recorded 0 coasting frames and 0 expiries. Expiry was nevertheless
observed incidentally and behaved correctly in both arms — `g7_visible` 19/20,
`g7_turn` hybrid 4/19, `g7_gap06` and `g7_gap10` 2 each — always producing a new
track ID with no phantom occupancy, matching the Stage-4G5 and Stage-4G6 expiry
regressions, which still pass unchanged. **The never-observed limitation is
therefore documented from first principles rather than demonstrated here:**
neither CV nor reachability can predict an obstacle that has never been
observed, because both are propagations of a filter state that does not yet
exist. This is a perception limit, not a prediction limit, and no prediction
policy can compensate for it.

## 8. The one genuinely dangerous scenario

`g7_turn` — the target changes direction *while hidden* — produces a collision
in **18 of 18 trials in both arms** (clearance −0.133 vs −0.130 m). This is the
sharpest result in the stage: in the only Stage-4G7 condition that is actually
dangerous, **the hybrid policy does not help at all**. Hybrid also took 28.2 s
to the goal against CV's 20.6 s there. The out-of-model step variant
(`g7_turnstep`) produced no collisions in either arm, so the in-model ramped
turn is the harder case — the target accelerates *toward* the robot through the
whole gap.

## 9. Runtime (§23)

| case | arm | tracker callback mean / max | layer updateCosts mean | reachability share |
|---|---|---:|---:|---:|
| primary occlusion | CV / **HY** | 13.83 / **13.83** ms, max 34.2 | 18.6 / **17.7** µs | **14.7%** |
| constant visibility | CV / **HY** | 12.08 / **12.04** ms | 22.0 / **23.4** µs | 13.0% |
| long gap | CV / **HY** | 12.73 / **13.01** ms | 20.5 / **21.5** µs | 14.8% |
| multi-object | CV / **HY** | 13.42 / **13.68** ms | 21.7 / **15.9** µs | 2.9% |

Hybrid costs **−5.8 to +1.4 µs** of costmap update and −0.00 to +0.27 ms of
tracker callback. Max callback 34–43 ms (17–21% of the 200 ms scan period);
one multi-object hybrid trial reached 103 ms under four-way parallel contention,
present in both arms. **Cost is not, and never was, the reason to drop
reachability.** The reachability share rises to 13–15% here from 2.4% in the
open room, confirming the policy is far more active in this domain — and still
without effect.

## 10. Acceptance criteria

| # | Criterion | Verdict |
|---|---|---|
| 1 | Static geometry creates the main occlusion | **PASS** |
| 2 | Target track established before occlusion | **PASS** — 15 observations, single ID |
| 3 | Conflict develops materially during observation loss | **PARTIAL** — overlap 0.14–0.15; closest approach ~1 s before gap end. §3 bounds why. |
| 4 | Multiple valid gap durations | **PASS** — 0.40 / 0.47 / 0.60 s |
| 5 | Blind-corner scenario | **PASS** |
| 6 | Doorway or equivalent second geometry | **PASS** — tested; target stayed visible through the door, reported |
| 7 | Hidden manoeuvre scenario | **PASS** — `g7_turn` (+ out-of-model variant) |
| 8 | ≥ 20+20 primary trials | **PASS** — 30+30 |
| 9 | Target-specific clearance measured separately | **PASS** |
| 10 | Clearance-limiting entity identified | **PASS** — 100% target in primary |
| 11 | Decision rule pre-registered | **PASS** — sha256 `446f53e0…` |
| 12 | Replication clause applied | **PASS** — gate B |
| 13 | Mechanism clause applied | **PASS** — gate C, failed on C3 |
| 14 | CV and Hybrid algorithms unchanged | **PASS** |
| 15 | No policy/controller retuning | **PASS** |
| 16 | Constant-visibility control | **PARTIAL** — see §7; strict zero met by the Stage-4G6 control |
| 17 | Occluded no-conflict control | **PASS** |
| 18 | Never-observed limitation demonstrated separately | **PARTIAL** — not achieved in simulation; documented from first principles |
| 19 | Multi-object mixed visibility works | **PASS** |
| 20 | Expiry remains correct | **PASS** — observed incidentally, correct in both arms; unit regressions pass |
| 21 | Coverage→costmap→command→safety chain analysed | **PASS** — §6, breaks at step 2 |
| 22 | Runtime acceptable | **PASS** |
| 23 | Stage-4G6 reproducible | **PASS** — all six regressions + 4 unit tests pass |
| 24 | `stage4f-validated` unchanged | **PASS** |
| 25 | `~/ur5e_ws` untouched | **PASS** |
| 26 | Final KEEP/SHIP recommendation | **PASS** — §12 |

## 11. Limitations

1. **The ideal test was not constructible in this arena.** §3 quantifies why over
   40,000 layouts. The closest approach still falls ~1 s before the gap ends.
   A domain where an obstacle emerges from occlusion *directly* into a robot
   already committed at speed — a narrow corridor, a faster robot, a doorway the
   robot drives through — remains untested.
2. **The robot is the confound.** With a 5 Hz LiDAR, a 0.46 m/s nominal speed and
   an MPPI controller that reacts on sight, the robot has already slowed to
   ~0.25 m/s and is ~0.9 m away by the time the target vanishes. A 0.4–0.6 s
   blackout adds nothing it has not already handled.
3. **Two controls failed their design intent** (§7) and are reported as such.
4. **`g7_turn` collides in both arms 18/18** — the conflict there is unavoidable
   by prediction, so it tests neither policy's advantage, only that neither helps.
5. Nine setup failures (readiness timeouts under four-way parallel startup) are
   retained and listed; arms were topped where they mattered.
6. **Collision-monitor interventions were not recorded** by this harness; stop
   count and stopped fraction serve as the braking proxy.

## 12. Final recommendation

### SHIP CV-ONLY

All four pre-registered gates were evaluated and three failed outright. The
evidence is now consistent across three independent stages and three domains:

| stage | domain | coasting share | CV coasting coverage | measured navigation benefit |
|---|---|---:|---:|---|
| 4G5 | open room, moving occluder | 2.4% | .524 | none (n=3) |
| 4G6 | open room, high power | 1.8% | ~.5 | none (365 trials) |
| **4G7** | **blind corner, static occluder** | **13–15%** | **.207** | **none (282 trials)** |

Stage-4G7 was designed specifically to give reachability its best chance: static
occlusion, the target as the clearance-limiting object in 100% of trials, the
largest coverage deficit ever measured for CV, and the policy active on 13–15%
of track-updates rather than 2.4%. It still produced **+0.005 m** of clearance,
a CI spanning zero, a worse 10th percentile, and **no detectable difference in
the costmap or the robot's command before reacquisition**.

**What this does not say.** Reachability is mathematically correct and its bound
is tight (3,000 adversarial trajectories contained, worst slack 0.000 m). It
genuinely fixes a real coverage hole — one that is *worse* under static
occlusion than anywhere previously measured. It costs microseconds. It never
made navigation worse in any powered comparison. The finding is narrower: **in
every domain this project can build, that coverage hole does not propagate into
a navigation outcome**, because the robot's reaction to a target it *can* see
already dominates, and a sub-second blackout does not change what it has
committed to.

**Therefore:** production ships `prediction_mode: cv_covariance`. Keep the
Stage-4G4 reachability model, the Stage-4G5 hybrid policy, their eight
regression tests and all results — unchanged, documented, one parameter away.
Revisit only with evidence from a domain this benchmark could not construct:
faster robot, longer commitment distance, or an obstacle emerging from occlusion
inside the robot's braking distance.

## 13. Exact reproduction

Keep `--domain` below 226 (the runner adds a per-trial offset; DDS rejects ids
above 232). `g7run.sh` runs the CV and Hybrid arms concurrently so both arms see
identical machine contention.

```bash
cd ~/predictive_nav_ws
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
source install/setup.bash

# Regenerate the two occlusion worlds and their maps (world+map from one source)
python3 src/predictive_nav_bringup/scripts/stage4g7/make_worlds.py

# Re-derive the layouts (line of sight AND target-path clearance)
python3 src/predictive_nav_bringup/scripts/stage4g7/design.py

L=validation_logs/stage4g7_repro
validation/stage4g7/g7run.sh $L/primary   30 g7_gap08
validation/stage4g7/g7run.sh $L/gaps      20 g7_gap06 g7_gap10
validation/stage4g7/g7run.sh $L/turn      20 g7_turn  g7_turnstep
validation/stage4g7/g7run.sh $L/controls  10 g7_visible g7_noconflict
validation/stage4g7/g7run.sh $L/controls2  8 g7_multi g7_door

python3 src/predictive_nav_bringup/scripts/stage4g7/analyze.py \
  --logs $(ls -d $L/*/*__*/) --out $L/all_trials.json
python3 src/predictive_nav_bringup/scripts/stage4g7/decide.py \
  --trials $L/all_trials.json --out $L/decision.json
python3 src/predictive_nav_bringup/scripts/stage4g6/runtime.py \
  --case primary_occlusion=$L/primary/g7_gap08 \
  --case constant_visibility=$L/controls/g7_visible \
  --case long_gap=$L/gaps/g7_gap10 \
  --case multi_object=$L/controls2/g7_multi \
  --out $L/runtime.json

# Stage-4G6 reproducibility
ROS_DOMAIN_ID=98 build/predictive_nav_costmap/stage4g5_hybrid_layer
ROS_DOMAIN_ID=97 build/predictive_nav_costmap/stage4g4_reach_layer
ROS_DOMAIN_ID=93 build/predictive_nav_costmap/stage4g2_multi_layer
ROS_DOMAIN_ID=96 build/predictive_nav_tracking/stage4g4_reachability
ROS_DOMAIN_ID=95 build/predictive_nav_tracking/stage4g3_deblend
ROS_DOMAIN_ID=94 build/predictive_nav_tracking/stage4g2_lifecycle
python3 -m unittest discover -s src/predictive_nav_bringup/scripts/stage4g2 -p 'test_*.py'
```

## 14. Handoff

The prediction-policy question is closed. Production configuration is CV-only;
reachability and hybrid remain in the tree as research, fully tested and one
parameter away from being re-enabled if a future domain justifies it.

No Stage-4G8 or any later stage has been started.
