# STAGE-5E — FINAL PREDICTOR ARCHITECTURE DECISION & COMPARATIVE VALIDATION

**Repository:** `~/predictive_nav_ws`
**Branch:** `main`
**HEAD:** `09e96a54d445adf16b7ad00a8f8ae51b01959475` (== `origin/main`)
**Working tree at capture:** `?? validation/stage5e/` — nothing tracked modified.

Everything below is **simulation only** (Gazebo, headless, TurtleBot3 planar 5 Hz LDS,
Nav2 + MPPI). No real-world validation is claimed anywhere in this document.

---

## 0. What this stage did, and what it deliberately did not do

Stage-5E is a **comparison**, not a development stage. No algorithm, parameter,
policy, message, launch file, trial runner or production configuration was
modified. `validation/stage5e/provenance.json` hashes 19 algorithm files and
compares each against its blob at HEAD: `all_algorithm_files_unchanged: true`.

The production default was **not changed during qualification** and is **not
changed by this document**. Stage-5E issues a *recommendation* (section 7).

### The pre-registered decision rule

`validation/stage5e/DECISION_RULE.md`, sha256
`cfff14a96218264f0d2ca2929db173505c791a522228ecd13a12948fd8c483d6`, was written
and hashed **at trial 4 of 135**, before any aggregate, per-condition table or
pairwise comparison existed. The contamination state at that moment is disclosed
inside the file itself. Its two exclusions exist precisely so that the offline
containment numbers — which *were* known when it was written — could not by
themselves decide the architecture:

- **E1** — a larger conservative set does not win.
- **E2** — higher mathematical containment does not win.

---

## 1. The three arms

| | A | B | C |
|---|---|---|---|
| name | CV-Kalman | global reachability | runtime class-aware reachability |
| `prediction_mode` | `cv_covariance` | `reachability` | `reachability` |
| `reachability_class_aware` | — | `false` | `true` |
| tracker config | `validation/stage5e/stage5e_cv_tracker.yaml` | `stage5d_global_tracker.yaml` | `stage5d_classaware_tracker.yaml` |
| classifier | runs (unconsumed) | runs (unconsumed) | **consumed**, with the safe ambiguity/fallback policy |
| `max_influence_radius` | 0.55 m | 0.55 m | 0.55 m |

There is **no oracle arm**. The Stage-5D/5D.1 oracle is an evaluation-only upper
bound and has no place in an architecture decision.

**Arm separation is proved, not asserted.** The comment-stripped diff between the
CV and global tracker configs is exactly one semantic line
(`reachability_enabled: false` vs `true`); between global and class-aware,
exactly one (`reachability_class_aware`). All three arms load the same,
unmodified Stage-4F Nav2 params file. Every scored trial reads back its own layer
mode, a 0.55 m window and the expected class-aware banner — **zero mismatches
across all 132 scored trials** (Table 8).

---

## 2. Campaign

- **Scenarios:** 9 conditions — `ped_cross`, `ped_longitudinal`, `bike_cross`,
  `veh_cross`, `veh_longitudinal`, `unknown_cross`, `ped_oblique`,
  `ped_close_pass`, `ped_corner`.
  Six are imported verbatim from the Stage-5D runner. The other three are built
  from already-validated definitions read at run time: `ped_oblique` and
  `ped_close_pass` from the Stage-4F `oblique` / `noconflict` conditions,
  `ped_corner` from `stage4g2/scenarios.json → g8_corner`. No new benchmark was
  invented.
- **Repetitions:** 5 per (condition, arm). Arms interleaved within each trial
  index, one full simulator restart per trial, isolated `ROS_DOMAIN_ID`.
- **Total trials:** 135 (9 × 3 × 5). Wall time 7187 s.
- **Excluded infrastructure trials:** 3 (2.2%) — CV 0, global 2, class-aware 1.
- **Scored trials:** 132.

Only the predictor mode differed. Same map, start/goal, actors, sensor, tracker
binary, trajectories, classifier execution, Nav2, MPPI, local costmap, resolution,
seeds and trial harness.

### Conditions NOT run, and why

| brief item | status |
|---|---|
| reversal / acceleration change | **not run.** The shared scorer is written for scripted one-way actors (`stage4f_trial.py:659`, "scripted obstacle (one-way, no reversal)"). Modifying it would have broken comparability with every earlier stage and violated the freeze. Existing evidence: `validation/stage5c/planner_comparison.json` `nav_reversal` — CV 0.7347/0.6971 vs reachability 0.7041/0.6605; `validation/stage4g8/results.csv` `g8_reversal` 10/10 success, 0 collisions. |
| multi-obstacle | **not run.** The bringup launch spawns exactly one obstacle; Stage-5D declined this for the same reason. Existing evidence: `validation/stage4g8/results.csv` `g8_multi2` 4/10 with 6 collisions, `g8_multi3` 4/10 with 7 collisions, which Stage-4G8 §18 classed as physically unavoidable (class H). |

**Scope of the historical evidence cited above.** Those `nav_reversal`,
`g8_reversal`, `g8_multi2` and `g8_multi3` results are from earlier stages. They
are **not part of the Stage-5E three-arm campaign**, were not produced under the
three-arm configuration, and are referenced as **supporting evidence only**.
Nothing in this document counts them among the 135 attempted or 132 scored trials,
and per DECISION_RULE §6 **neither gap may be used to support a positive claim
about any arm.**

Stated plainly: **the final three-arm Stage-5E campaign did not include equivalent
reversal or multi-obstacle trials.** The reason is structural, not a matter of
choice — the shared, validated Stage-4F runner drives one obstacle at a constant
one-way velocity, and the shared launch spawns exactly one obstacle. Changing
either would have broken the freeze and the comparability with every earlier
stage. This limitation does **not** invalidate the architecture decision, which
rests on the nine conditions that were run; it bounds the decision's reach, and it
remains stated here, in section 8 and in `DECISION.json`.

---

## 3. Final comparison

Full tables: `validation/stage5e/TABLES.md` (9 tables).
Raw aggregates: `validation/stage5e/nav_campaign5.json`.
Offline prediction audit: `validation/stage5e/prediction_audit.json`.

| | CV | Global Reach | Class-Aware Reach |
|---|---:|---:|---:|
| Trials launched / excluded / scored | 45 / 0 / 45 | 45 / 2 / 43 | 45 / 1 / 44 |
| Success | 45/45 (1.000) | 43/43 (1.000) | 44/44 (1.000) |
| Collisions | **1** | 0 | 0 |
| Near misses (< 0.20 m) | 5 | 5 | 7 |
| Worst clearance, m | **−0.128** | 0.123 | 0.069 |
| Mean clearance, m | 0.658 ± 0.315 | 0.668 ± 0.303 | 0.665 ± 0.325 |
| Median clearance, m | 0.691 | 0.703 | 0.695 |
| Clearance p10 / p25 / p75 (conflict only) | 0.133 / 0.507 / 0.747 | 0.149 / 0.440 / 0.728 | 0.122 / 0.478 / 0.792 |
| Nav time, s | 19.013 ± 3.281 | 18.708 ± 3.195 | 19.002 ± 3.135 |
| Path length, m | 6.019 ± 0.316 | 6.028 ± 0.309 | 6.013 ± 0.317 |
| Timeouts | 0 | 0 | 0 |
| Planner / controller failures | 0 | 0 | 0 |
| Trials with no global plan | 0 | 0 | 0 |
| Stops | 2.07 ± 1.03 | 1.84 ± 0.78 | 1.82 ± 0.76 |
| Global path changes | 4.62 ± 2.08 | 4.42 ± 2.00 | 4.80 ± 2.08 |
| Predictive cells (max/trial) | 1122.6 ± 289.8 | 1187.8 ± 216.7 | **1325.5 ± 252.3** |
| Layer compute, µs (mean / max) | 25.07 / 62.7 | 24.93 / 57.7 | 25.36 / 63.5 |
| Layer rate, Hz / shortfalls | 5.023 / 0 | 5.023 / 0 | 5.023 / 0 |
| Costmap update failures | 0 | 0 | 0 |
| Tracker reachability stage, µs/scan | 0.090 | 6.430 | 6.895 |
| Capability containment (offline, pooled) | 0.0098 | 0.0171 | **1.0000** |
| Observed containment (offline, pooled) | 0.9888 (k=2.0) / 0.9783 (k=1.5) | 0.9956 | 1.0000 |
| Predicted region area, m² (offline, pooled) | 24.25 | 47.31 | **2151.73** |
| Contract breaches (live) | 0 | 66 (all repaired) | 0 |
| Unsafe profile events (`prof_fb_badconfig`) | 0 | 0 | 0 |

**p95 / p99 of the layer update time are NOT reported: that instrumentation does
not exist.** Mean and max only. No percentile was invented.

The two containment rows measure different things, are **not the same kind of
guarantee**, and are deliberately kept apart:

- **capability containment** asks whether the region covers everything the
  obstacle's true class could physically do — a *worst-case set* property, and the
  safety question;
- **observed containment** asks whether the track's later measured position fell
  inside the region — a *coverage/calibration* property of a probabilistic model.

CV's 0.9783 is probabilistic coverage of realised positions at k = 1.5 sigma; it
is **not** a claim that the region bounds what the obstacle could have done, and
it must not be read against class-aware's 1.0000 capability containment as though
the two numbers were commensurable. They are not combined into a single score, no
single "containment" figure of merit is computed, and CV is reported at both its
published k = 2.0 and the k = 1.5 the layer actually consumes.

### Does any difference survive the noise?

Two-sigma-of-the-difference band, `2·sqrt(sem_a² + sem_b²)` — a crude noise bound,
explicitly **not** a hypothesis test.

| pair | metric | diff | 2σ band | outside? |
|---|---:|---:|---:|---|
| CV → Global | min clearance | +0.010 m | 0.132 | no |
| CV → Global | nav time | −0.305 s | 1.381 | no |
| CV → Global | stops | −0.230 | 0.390 | no |
| CV → Class-aware | min clearance | +0.007 m | 0.136 | no |
| CV → Class-aware | **predictive cells** | **+203.0** | 115.1 | **YES** |
| Global → Class-aware | min clearance | −0.003 m | 0.135 | no |
| Global → Class-aware | **predictive cells** | **+137.7** | 100.8 | **YES** |

**Every navigation metric, for every pair, is inside the band. The only
differences that resolve are how much costmap each arm paints** — which E1
classes as a cost until it buys an outcome, and it bought none.

Per condition, the largest clearance gain over CV anywhere in the campaign is
**+0.0971 m** (global, `veh_longitudinal`) — below the 0.10 m practical threshold
carried forward from Stage-4G6/4G7, inside its own 0.1896 m band, and produced
entirely by CV's single collision trial. Removing that one trial, the same
comparison is **+0.0125 m**.

---

## 4. Failure analysis (section 9 classification)

Full record: `validation/stage5e/failure_analysis.json`.

### Exclusion convention, stated explicitly

A trial whose `nav_status` is null never produced a navigation result; it is
marked an infrastructure fault, excluded from scoring, and **counted against no
arm**. This is the convention in force since Stage-4F; Stage-5E does not change
it. 3 of 135 trials (2.2%) were excluded, all pre-navigation bringup faults:

| trial | arm | cause |
|---|---|---|
| `veh_cross_global_01` | global | `ros_gz_sim create` "Timed out when getting world names", died exit 255; obstacle never spawned; `local_costmap` never got `base_link→odom` |
| `ped_corner_global_03` | global | identical Gazebo spawn failure |
| `ped_corner_classaware_05` | class-aware | Nav2 map_server ImageMagick **SIGSEGV** during bringup, container exit −6 |

### CV — one collision

`veh_longitudinal_cv_03`, min clearance −0.1284 m. **Classified
PLANNER_FIELD_COLLAPSE. Counted against CV.**

Head-on vehicle at 0.471 m/s with a 0.25 m lateral offset. The robot reached
x = −0.910 having achieved only **0.4335 m of lateral offset — the smallest of all
15 trials in this condition** — and then stopped. From t = 7.2 s its pose is
frozen while the controller keeps requesting 0.18–0.20 m/s (`v_nav` > 0,
`v_out` = 0 for **3.60 s**): the Nav2 collision monitor was zeroing the command.
The scripted actor, which has no avoidance and no collision response, continued at
a constant 0.471 m/s and traversed the stationary robot — separation fell
0.455 → 0.321 m *with the robot motionless*, clearance crossed zero at t = 7.95 s
and bottomed at −0.128 m at t = 8.35 s.

**Ruled out:**
- *TRACKING_FAILURE.* The trial does show 13 simultaneous tracks, unique in the
  campaign (every other `veh_longitudinal` trial holds exactly 1). But the
  proliferation begins at t = 8.50 s, **after** contact at t = 8.35 s, and
  `n_tracks` = 1 throughout the approach. The fragmentation is a *consequence* of
  a vehicle body at point-blank range, as is the elevated per-trial tracking
  error (0.651 m vs 0.44 m in siblings).
- *PREDICTION_FAILURE.* CV displacement error at the decisive horizons is ordinary
  for the condition (0.66 m at 0.5 s, within the spread of the other four CV
  trials), and the predicted field was not what stopped the robot — the monitor was.
- *SIM_INFRASTRUCTURE.* The simulator ran correctly and the trial produced a
  complete, scored result.

**It is not a CV-specific failure class.** The same collision-monitor hold occurs
in **3 of 5 class-aware trials** in the same condition and **0 of 5 global**.
Across all 15 trials of the condition, hold duration and minimum clearance
correlate at **r = −0.784**, and every trial with a nonzero hold sits at the
bottom of the clearance ranking:

| trial | min clearance, m | monitor hold, s |
|---|---:|---:|
| `veh_longitudinal_cv_03` | **−0.1284** | 3.60 |
| `veh_longitudinal_classaware_01` | 0.0687 | 0.30 |
| `veh_longitudinal_classaware_03` | 0.1306 | 0.20 |
| `veh_longitudinal_classaware_05` | 0.2048 | 0.10 |
| the other 11 trials | 0.2297 – 0.4012 | 0.00 |

The mechanism is a **planner/monitor interaction in a tight head-on corridor**,
sampled on the wrong side of the line once by CV. 0 of 5 for global at n = 5 is
suggestive and supports no claim (Fisher 1/5 vs 0/5 → p = 1.0).

*Contributing factor, reported but not used as an excuse:* the obstacle actors are
scripted one-way bodies with no avoidance and no collision response, so once the
robot is held stationary in the actor path, contact is geometric. This property is
shared by all three arms and by every stage since 4F.

### Global Reach — no navigation failure; 66 contract breaches, all repaired

**Classified SENSOR_OBSERVABILITY. Not counted against the arm** (DECISION_RULE
G5: a breach the Stage-5C.1 observable response repairs is reported, not counted).

All 66 are in `ped_corner` and nowhere else; the other eight conditions record
zero. **Ground-truth actor speed never exceeds 0.655 m/s anywhere in the
campaign**, so no obstacle ever approached the 0.8 m/s global cap. At corner
emergence the visible extent of the body changes rapidly, the cluster centroid
jumps, and the filtered speed overshoots to 1.098–1.240 m/s. The same overshoot
appears in the CV and class-aware arms — they simply have no cap to breach (CV has
no capability premise at all; the class-aware PEDESTRIAN cap is 2.5 m/s).

The repair is observable and was confirmed live: `effectiveMaxSpeed()` raises the
cap to `max(max_speed, observed_speed)` on a contradicted premise — the region is
**enlarged, never dropped** — publishes `contract_status = kObservedSpeedExceedsCap`,
and logs a throttled WARN naming the track, the observed speed, the configured cap
and the effective cap.

### Class-Aware Reach — no navigation failure; 2 condition-specific near misses

`veh_longitudinal_classaware_01` (0.0687 m) and `_03` (0.1306 m), same
collision-monitor mechanism as CV's collision, short of contact. **Counted against
the arm.** 0.0687 m is the worst clearance of any non-colliding trial in the
campaign — produced by the arm with the largest predicted region.

### All three arms — 15/15 near misses in `ped_longitudinal`

**Classified OTHER (condition-inherent geometry). Counted against no arm.**
Head-on pedestrian at 0.46 m/s with a 0.25 m lateral offset. In every trial the
robot keeps moving at 0.46–0.50 m/s, detours to 0.33–0.39 m and passes alongside
at 0.099–0.190 m, with zero monitor interventions, zero planner failures and
SUCCEEDED in all 15. Means: CV 0.137 m, global 0.136 m, class-aware 0.121 m — a
spread of 0.016 m, inside the band. It is identical in all three arms and
therefore discriminates nothing.

*Contestable reading, recorded rather than suppressed:* a reader could class this
COSTMAP_UNDERREPRESENTED, since the predicted region carried little cost in the
pass corridor (`pred_peak_ahead` 0 for CV, 70 for global at the pass). It changes
no arm comparison, because the three arms are inside the noise band here.

---

## 5. Decision

### `SHIP_CV`

#### What `SHIP_CV` means, and what it does not claim

`SHIP_CV` means: **CV-Kalman is recommended as the production/default predictor
for the validated simulation + Nav2 architecture.** That is the whole of the
claim.

It explicitly does **not** claim any of the following, each of which the evidence
contradicts or fails to support:

| not claimed | what the evidence actually says |
|---|---|
| CV is universally safest | It is the only arm that recorded a collision. |
| CV had zero collisions | CV had **1** collision (`veh_longitudinal_cv_03`, −0.1284 m), counted against it. |
| CV statistically outperformed every reachability arm | **No** pairwise navigation metric, for any pair of arms, fell outside its two-sigma band. Nothing was statistically resolved in CV's favour. |
| CV has better capability containment | CV's capability containment is **0.0098** — the lowest of the three. Class-aware is 1.0000. |

The three arms' reliability records, kept side by side:

| | success | collisions |
|---|---|---|
| CV | 45/45 | **1** |
| Global Reach | 43/43 | 0 |
| Class-Aware Reach | 44/44 | 0 |

The recommendation therefore rests on the **complete architecture trade**, not on
a safety-superiority claim:

- reliable navigation (all three arms: 100% success, 0 timeouts, 0 planner or
  controller failures);
- **graded early planner-visible structure** — CV is the only arm whose field is
  not saturated through the first second, the horizon a 0.48 m/s robot acts on;
- **fewer predictive cells** — 1122.6 vs 1187.8 and 1325.5, the only axis on which
  the arms separate outside the noise band, and E1 classes extra paint as a cost
  until it buys an outcome;
- stable planner interaction (stops, churn and reaction time inside the band in
  every pair; a global plan in every scored trial);
- bounded computation (25.07 µs mean against a 200 000 µs budget, 0 rate
  shortfalls, 0 costmap update failures, and no reachability stage on the tracker
  hot path);
- adequate **observed** prediction performance (0.9783 at the k = 1.5 the layer
  actually consumes), which is the calibration standard a CV model is accountable
  to.

Had any arm resolved a navigation advantage outside the band, that would have
outranked every item on this list. None did.

#### The gates

Applying DECISION_RULE §3–§4 to the evidence above:

| gate | CV | Global | Class-aware |
|---|---|---|---|
| **G1** reliability | 45/45 success, 0 timeouts, 0 planner/controller failures, 0 trials without a global plan; 1 collision (above) | 43/43, 0 collisions, 0 timeouts, 0 planner failures — **pass** | 44/44, 0 collisions, 0 timeouts, 0 planner failures — **pass** |
| **G2** safety behaviour (required to displace CV) | incumbent | **FAIL** | **FAIL** |
| **G3** planner stability | stops/churn inside band, plan in every trial, no stall — **pass** | pass | pass |
| **G4** cost | 0 rate shortfalls, 0 costmap update failures, 25.07 µs mean against a 200 000 µs budget — **pass** | pass (6.43 µs/scan tracker overhead) | pass (6.90 µs/scan) |
| **G5** soundness | 0 `prof_fb_badconfig`, 0 breaches — **pass** | pass (66 breaches, all repaired) | pass (0 breaches, capability containment 1.0) |

**Why G2 fails for both reachability arms:**

*Route 1 — clearance.* G2 requires a mean minimum-clearance gain ≥ 0.10 m **and**
outside the two-sigma band. Pooled over conflict conditions the gain is
**+0.0073 m** (global) and **+0.0030 m** (class-aware). The largest gain in any
single condition is +0.0971 m (global, `veh_longitudinal`) — below threshold,
inside its band, and an artefact of the one CV collision trial. Two per-condition
differences do fall outside a very tight band (global +0.0134 m on `ped_cross`,
class-aware +0.0097 m on `unknown_cross`), but both are ~1 cm — well under one
costmap cell (0.05 m) and far under the 0.10 m practical threshold. Statistically
resolvable, practically negligible.

*Route 2 — removing a collision or near-miss class.* Global has 0 collisions where
CV has 1. One event in 45 trials against 0 in 43 is not a *class*: the mechanism
that produced it (the collision-monitor hold) is present in 1/5 CV and 3/5
class-aware trials of that condition and is not predictor-specific, and 0/5 for
global at n = 5 is not separable from chance. Near misses are 5 / 5 / 7 — global
removes nothing and class-aware adds two.

DECISION_RULE §4 is explicit: *"This is the decision whenever the reachability
arms are merely not worse: the incumbent is not displaced by a tie."* They are not
worse. They are also not better. **CV ships.**

This is **not** the §12 historical expectation being honoured — DECISION_RULE §6
forbids that as a ground. It is what the gates return on the measured evidence.
Had global reproducibly removed the `veh_longitudinal` contact class, G2 route 2
would have carried it; the pre-registered route was open and the evidence did not
walk through it.

### Global reachability role: `DEPLOYED_SECONDARY`

It passes G1, G4 and G5, and it supplies **measured properties the shipped
predictor demonstrably does not**, readable by a consumer off the navigation path:

1. **A region with a falsifiable validity claim.** Global publishes a
   time-parameterised bound with an explicit capability premise and a runtime
   contract that fires when that premise is observably contradicted — 66 live
   firings, each naming the track, the observed speed, the configured cap and the
   effective cap, with the region enlarged in response. CV's covariance ellipse
   has no premise and therefore cannot report that the region it just published is
   not a qualified bound. A safety monitor or speed limiter can read that signal.
2. **Better observed containment at bounded area cost.** 0.9956 vs CV's 0.9888
   (k = 2.0) / 0.9783 (k = 1.5), for 47.3 m² against 24.2 m² — roughly 2×. It is
   the only arm that improves calibration over CV at an affordable area; the
   class-aware route to containment 1.0 costs 2151.7 m², 45× CV.

**Stated honestly:** all 66 firings were estimator transients at corner emergence,
not real capability violations — no obstacle in this environment ever exceeded
0.655 m/s. The mechanism is confirmed to work on its own terms (it is defined on
*observed* speed, and the observed speed did exceed the cap), but its value as a
detector of a *true* capability violation is **untested here**, and a consumer
must tolerate a 0.16% transient firing rate (66 / 41 712 samples).

### Class-aware reachability role: `RESEARCH_ONLY`

It is sound as mathematics: capability containment **1.0000**, zero contract
breaches, zero unsafe profile events, zero `prof_fb_badconfig`. It is the only arm
whose region is a genuine bound on what the obstacle's true class could do.

But at the production 0.55 m window its field is **saturated at every horizon,
including 0.5 s** — 529 of 529 cells, where CV still carries graded structure
(54 cells at 0.5 s, 260 at 1.0 s) through the first second, which for a 0.48 m/s
robot is the horizon that matters. A saturated field is indistinguishable from any
other saturated field: the planner cannot act on a distinction it cannot see. That
is DECISION_RULE E1 and the RESEARCH_ONLY definition verbatim: *"sound as
mathematics but the measured planner behaviour at the production window is
uninformative (e.g. a saturated field), so no deployable consumer can be specified
for it as configured."*

The measurements agree: no clearance gain (+0.0030 m pooled), the only metric that
resolves is that it paints 203 more cells than CV, the worst non-colliding
clearance in the campaign (0.0687 m) is its own, and it adds two near misses.

A second limitation, measured live: **fallback rate 0.4193** — 16 194 of 43 182
samples were `prof_fb_unknown`, 1 794 `prof_fb_lowconf`, 120 `prof_fb_immature`.
The classifier closes the class on only 58% of samples at this sensor fidelity, so
the class-aware premise is not even exercised as designed on the remainder. This
is the **PROFILE_LIMITED** mechanism, now quantified.

`RETIRED` is wrong for both: global has a specified consumer, and class-aware is
the sound upper reference against which the other two are judged unsound-but-useful.

---

## 6. Final research interpretation (section 16)

**1. Why does the selected production predictor win?**

Not because it is the best model of obstacle motion — it is not, and not because
it was measured safer — it was not. CV is selected because it is the only arm
whose predicted field still **carries information at the horizons the planner
acts on**. At 0.5 s CV paints 54 of 529 cells and at 1.0 s
260; global saturates from 1.0 s; class-aware is saturated from 0.5 s. MPPI steers
by cost *gradients*. A graded field gives it a direction; a saturated one gives it
a wall. Everything else — reliability, clearance, nav time, path length, stops,
churn, compute — is a tie inside the noise band, so the tie-break falls to the
representation the planner can actually use, and to the incumbent, which is not
displaced by a tie.

**2. What does global reachability add that CV does not?**

A *premise*. CV's ellipse is a statement about estimator uncertainty; it will
track whatever the obstacle does and can never be wrong in a way it can announce.
Global's region is a statement about what the obstacle *can* do given a speed and
acceleration cap, and it comes with a contract that detects and announces its own
violation — measured 66 times, with the region enlarged, never dropped. It also
buys genuinely better observed containment (0.9956 vs 0.9783 as consumed) at about
double the area. What it does **not** add is any measurable planner benefit: every
navigation metric sits inside the band.

**3. Why did class-aware reachability fail at planner level?**

Two measured reasons, in this order:

- **Saturation (E1).** PEDESTRIAN 2.5 m/s, BIKE 12.0 m/s, VEHICLE 40.0 m/s over a
  3 s horizon describe regions of 7 m² to 7 291 m². The production influence
  window is 0.55 m — 529 cells. The region is clipped to the window at every
  horizon, so the layer paints a constant saturated disc, and the planner receives
  the same field regardless of class, speed or direction. Semantic class is
  computed correctly and then destroyed by the window.
- **Fallback (PROFILE_LIMITED).** Even where the window did not clip, 41.9% of
  samples never closed a class at all. A planar 5 Hz LDS at these ranges does not
  support confident class closure often enough for the class-aware premise to be
  exercised as designed.

It **succeeded** at exactly one thing, and it is worth stating plainly: it is the
only arm with capability containment 1.0 and zero contract breaches. It is sound;
it is just not usable at this window on this planner. **Perfect mathematical
containment did not translate into any planner-level benefit** — no clearance
gain, no reliability gain, the worst non-colliding clearance in the campaign, two
extra near misses, and the largest painted area of the three. Containment
qualifies a predictor as sound; it does not qualify it as deployable.

**4. What is the most scientifically justified future improvement?**

**Directional / anisotropic reachable sets.** The evidence points there more
directly than at any other option. The failure mode of both reachability arms is
not that the set is wrong — class-aware's is provably right — but that an
*isotropic* set spends its whole area budget on directions the obstacle is not
going, and then saturates a small influence window. The tracker already publishes
a velocity estimate with a covariance; conditioning the reachable set on heading
would shrink the painted area by a large factor at identical soundness, which is
precisely what is needed to get a class-aware bound under a 0.55 m window without
saturating it. The second-order candidate is a **larger / global planning
representation**: the 0.55 m window is itself a binding constraint, and some of
what reachability offers may simply be invisible at that scale. CTRV/IMM/maneuver
models and intent-conditioned or learned multimodal prediction are legitimate but
address prediction *accuracy*, which this campaign did not identify as the
bottleneck — ADE is 0.32–0.33 m in all three arms, and the arms are tied on every
navigation metric. **None of these is implemented here.**

A smaller, cheap follow-up the data also justifies: the `veh_longitudinal`
collision-monitor hold (r = −0.784 against clearance) is a planner/monitor
interaction worth characterising on its own, independently of the predictor
question.

---

## 7. Production configuration

**Unchanged.** `production_nav2_params.yaml` and `production_tracker_params.yaml`
are byte-identical to HEAD. `prediction_mode` remains `cv_covariance`,
`reachability_enabled` remains `false`.

**Recommendation only:** keep CV as the production default. Retain global
reachability as a deployed secondary representation for a consumer off the
navigation path. Keep class-aware reachability as a research configuration and as
the soundness reference.

---

## 8. Limitations, stated honestly

1. **Simulation only.** Gazebo, headless, one map, one robot, one sensor model.
   Nothing here is real-world validation.
2. **n = 5 per cell.** Enough to show that the arms are tied inside a crude noise
   band; not enough to resolve a rare-event difference. The single CV collision
   and global's 0/5 on the monitor hold both sit well inside what n = 5 cannot
   separate.
3. **Two brief conditions were not run in the three-arm campaign** —
   reversal/acceleration-change and multi-obstacle — because the shared Stage-4F
   runner drives one obstacle at constant one-way velocity and the shared launch
   spawns one obstacle; modifying either would have broken the freeze and
   comparability. The prior-stage results cited in section 2 are **supporting
   evidence only, not part of this campaign**, and are not used to support any
   claim about any arm. The decision's reach is bounded accordingly.
4. **p95/p99 layer compute do not exist.** Mean and max only; nothing was invented.
5. **Scripted, non-reactive actors.** Obstacles never avoid the robot and have no
   collision response. This bounds how much any predictor can help and affects all
   three arms equally.
6. **Tracker per-scan timings are bimodal** (~150 µs vs ~10 ms, driven by a TF
   wait under Gazebo/Nav2/MPPI contention), so Table 9 reports the **median per
   trial**. The mean of that distribution is not a meaningful figure and is not used.
7. **The offline containment audit runs on one replay set** (2547 rows, 57 tracks)
   from the Stage-5D captures, not on the Stage-5E navigation trials. Arms B and C
   reproduce Stage-5D's numbers exactly, which validates the audit harness but does
   not make the containment figures contemporaneous with this campaign.
8. **Global's contract signal has no true positive in this environment.** All 66
   firings were estimator transients; no obstacle exceeded 0.655 m/s.
9. **The decision rule was fixed at trial 4, not trial 0.** The offline containment
   numbers were known when it was written. This is disclosed in the rule file
   itself, and E1/E2 exist specifically to neutralise it.

---

## 9. What this project demonstrates

*(Application-ready project summary. Simulation only — no real-world validation is
claimed.)*

This project builds and evaluates a complete predictive navigation stack for a
differential-drive robot in ROS 2 Jazzy, in simulation:

- **Dynamic obstacle detection and tracking** from a planar 5 Hz LiDAR —
  clustering with deblending and compound-cluster recovery for fragmented extended
  bodies, nearest-neighbour association with gating, and per-track lifecycle
  management with coasting and expiry.
- **Semantic classification** of tracked obstacles into pedestrian / bicycle /
  vehicle / unknown from observable geometric and kinematic features, with a
  calibrated confidence and an explicit unknown class.
- **Bayesian prediction** — a constant-velocity Kalman filter producing a position
  estimate *and* a propagated covariance, published as an uncertainty ellipse over
  a 3 s horizon at 0.5 s steps.
- **Reachability-based prediction** — forward reachable sets under explicit speed
  and acceleration capability premises, in both a class-agnostic global form and a
  class-conditioned form, each carrying a runtime contract that detects and
  announces observable violation of its own premise and enlarges the region in
  response.
- **Uncertainty-aware movement corridors** rendered as time-decayed predictive
  cost over a bounded influence window, with the two prediction families kept
  distinguishable and independently auditable.
- **Native C++ Nav2 costmap integration** — a custom `costmap_2d` layer plugin
  consuming predicted regions directly, holding a mean update cost of ~25 µs
  against a 5 Hz budget with zero rate shortfalls and zero update failures across
  132 trials.
- **MPPI planning** under Nav2, unmodified, identical in every arm.
- **Systematic comparison of prediction strategies** — a balanced 135-trial
  campaign over 9 scenario conditions × 3 predictor arms × 5 repetitions, one
  simulator restart per trial, arms interleaved to defeat machine drift, with
  per-trial read-back proving arm separation and infrastructure faults separated
  from navigation outcomes.
- **Evidence-based architecture decision** — a decision rule written and hashed
  before the aggregate was read, explicitly excluding "paints more" and "contains
  more" as grounds; gates applied to the measured evidence; the resulting decision
  contradicts neither the data nor the prior hypothesis by construction, because
  the hypothesis was barred from being a ground.

The headline engineering result is a negative one, and it is the useful kind: on
this platform, at this influence window, a mathematically sound class-conditioned
reachable set is **not** usable by the planner, because the window saturates and a
saturated field carries no information. Soundness and deployability are different
properties, and this project measured both separately rather than assuming one
implies the other.

---

## 10. Acceptance gates (section 18)

| gate | status | evidence |
|---|---|---|
| A. three qualified predictor arms compared | **PASS** | CV / global / class-aware, no oracle arm; Table 1 |
| B. identical inputs/config except predictor choice | **PASS** | one-line config diffs; shared Nav2 params; per-trial read-back, 0 mismatches in 132 scored trials (Table 8) |
| C. balanced final campaign completed | **PASS** | 9 × 3 × 5 = 135 launched, 132 scored, balanced by construction |
| D. navigation metrics reported | **PASS** | Tables 1–4; `nav_campaign5.json` |
| E. prediction metrics reported | **PASS** | Tables 5–6; `prediction_audit.json`; both containment definitions kept apart |
| F. computation metrics reported | **PASS** | Tables 1, 9; p95/p99 declared NOT INSTRUMENTED |
| G. failures causally classified | **PASS** | section 4; `failure_analysis.json`; exclusions documented explicitly |
| H. relevant functional tests pass | **PASS** | 13/13 functional suites, 0 failures; lint debt separated (section 11) |
| I. production configuration unchanged during qualification | **PASS** | `provenance.json`, `all_algorithm_files_unchanged: true` |
| J. explicit final architecture recommendation issued | **PASS** | `SHIP_CV` |
| K. global reachability role classified | **PASS** | `DEPLOYED_SECONDARY` |
| L. class-aware reachability role classified | **PASS** | `RESEARCH_ONLY` |
| M. limitations/future work stated honestly | **PASS** | sections 6.4 and 8 |
| N. no new algorithm introduced | **PASS** | only evidence/analysis files under `validation/stage5e/`; nothing installed or launch-reachable |

---

## 11. Functional tests (section 13)

All 13 functional suites run post-campaign, **0 failures**:

`stage4g2_lifecycle`, `stage4g3_deblend`, `stage4g4_reachability`,
`stage5b1_compound` (128 checks), `stage5b1_compound_integration`,
`stage5b_classification` (95 checks), `stage5b_tracker_integration`,
`stage5c1_contract`, `stage5d_profiles`, `stage4g2_multi_layer`,
`stage4g4_reach_layer`, `stage4g5_hybrid_layer`, `stage5c1_raster_fidelity`.

**Before/after:** the algorithm tree is byte-identical to `origin/main` (19/19
files verified by sha256 against their HEAD blobs), so the pre-campaign and
post-campaign trees are the same tree and one run characterises both. The hashes
are the evidence.

**Lint debt, separated from functional results:** 14 failing flake8 cases,
13 pep257, 8 uncrustify. Every failing case is in pre-existing sources —
`launch/`, `scripts/` and `scripts/stage4g2…stage4g8`, `scripts/stage5d`, plus
four uncrustify cases each in the tracking and costmap packages — predominantly
C408 dict-literal style and D-code docstring conventions. **No Stage-5E file is scanned by ament lint**, because `validation/`
is not inside a ROS package. This is pre-existing style debt, not a functional
failure, and Stage-5E neither added to it nor fixed it.

---

## 12. Files

New, all under `validation/stage5e/`. Nothing tracked was modified.

| file | what it is |
|---|---|
| `DECISION_RULE.md` | the pre-registered rule, sha256 `cfff14a9…`, fixed at trial 4/135 |
| `run_nav5.py` | three-arm, nine-condition campaign runner |
| `stage5e_cv_tracker.yaml` | arm A tracker config (kept out of `src/` so production config stays byte-unchanged) |
| `stage5e_audit.cpp` | three-arm offline prediction audit over one replay set |
| `analyze_nav5.py` | campaign analyser |
| `report_tables.py` | renders the nine tables |
| `nav_campaign5.json` | campaign aggregates, per-condition, per-trial |
| `prediction_audit.json` | offline containment / area / cells / cost |
| `failure_analysis.json` | section 9 causal classification |
| `provenance.json` | HEAD, algorithm-file hashes, campaign and test provenance |
| `TABLES.md` | the nine rendered tables |
| `STAGE5E_README.md` | this document |

### Raw campaign evidence (deliberately NOT in Git)

The 135 raw trial JSONs, 135 per-trial series JSONs, 135 per-trial Nav2/Gazebo
logs and the campaign console log are preserved outside the repository:

```
~/predictive_nav_evidence/stage5e_raw_09e96a54.tar.gz          5 890 822 bytes
~/predictive_nav_evidence/stage5e_raw_09e96a54.tar.gz.sha256
sha256 45c695f9754c036005b19ed9ee6ad8770df3e9c00714abaafc50a4c1ffe630cd
```

406 files, verified by `sha256sum -c` and by a byte-for-byte round-trip extraction
against the source. It is **not** added to Git: ~55 MB uncompressed of raw
simulator output does not belong in the repository. Everything the analysis
depends on is derived into the JSON files above, which are committed.

---

## 13. Verdict

**PASS.** All fourteen acceptance gates met. No hard blocker.

- **Decision:** `SHIP_CV`
- **Global reachability role:** `DEPLOYED_SECONDARY`
- **Class-aware reachability role:** `RESEARCH_ONLY`
- **Production config changed?** **NO** — unchanged throughout qualification, and
  unchanged by this document.

The one result a reader should not skip: **CV recorded a collision and the two
reachability arms did not.** It is reported here as a collision counted against
CV, and the gates were still applied as pre-registered, because the mechanism
behind it is a planner/monitor interaction shared by all three arms and n = 5
cannot separate 1/45 from 0/43. If a larger campaign showed that contact class to
be reproducible and reachability-removable, DECISION_RULE G2 route 2 would carry
the decision the other way. That route was open. The evidence did not take it.
