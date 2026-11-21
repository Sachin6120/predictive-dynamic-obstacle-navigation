# Stage-5D — Class-aware reachability bounds

Replaces the one global reachability capability profile with configurable,
class-aware motion-capability profiles for PEDESTRIAN / BIKE / VEHICLE /
UNKNOWN, and measures whether semantic classification yields a useful,
defensible second prediction mode.

Baseline `35f293ac13531b2f975a5ecf91fcb6fd7da569c3` (`main`, clean, in sync
with `origin/main`).

**Production behaviour is unchanged.** `prediction_mode` stays
`cv_covariance`, `reachability_enabled` stays `false`, and the new
`reachability_class_aware` defaults to **`false`** — which returns the single
Stage-5C.1 global configuration bit for bit.

The pre-implementation audit required by section 1 of the brief is in
[`CLASS_REACHABILITY_BOUNDARY_AUDIT.md`](CLASS_REACHABILITY_BOUNDARY_AUDIT.md)
and was written before any file under `src/` was touched.

---

## 0. The result in four sentences

The class-aware mathematics qualify cleanly: **0 in-domain adversarial escapes
over 6,912 trials across all four profiles**, and the Stage-5C.1 contract is
preserved per profile. Over the 2,547 recorded real classifier decisions, the
runtime class-aware arm produced **0 unsafe profile selections in every one of
the 10 measured confusion directions** — including the 306-sample
`VEHICLE -> PEDESTRIAN` confusion that runs at median confidence 1.000 — and
cost only **1.13% more region area** than the ground-truth oracle arm. But at
the production costmap geometry every class profile saturates the layer's
0.55 m `max_influence_radius` window by `total_time` 0.34–0.67 s, so from the
first published horizon onward **the planner cannot distinguish one class
profile from another**: runtime and oracle class-aware paint an identical
529 cells at every horizon. And classifier ambiguity independently limits
usefulness: the only label whose measured precision earns a profile weaker
than the fallback is **BIKE**, and 381 *correctly* classified
`PED -> PED` tracks are served 40.0 m/s rather than the nominal 2.5 m/s,
because the high-confidence `VEHICLE -> PEDESTRIAN` confusion forces the
pedestrian envelope up to vehicle grade. That is a calibration and usefulness
limitation, not a safety defect.

Decision: **`QUALIFIED_CLASS_AWARE_NO_PLANNER_BENEFIT`**. Stage-5D verdict:
**PASS**.

---

## 1. What changed, and the one coupling point

Stage-5D introduces exactly one new dependency of prediction on
classification. `reachability.hpp`'s equation, its validity contract and its
published region are **not modified**; they become functions of a *selected*
profile instead of a single global one.

```
LiDAR -> clustering -> compound -> tracker
                                     |
                      Stage-5B stable label + confidence
                                     |
                                     v
                     selectCapabilityProfile()      <-- NEW (Stage-5D)
                                     |
                              MotionCapabilityProfile
                                     |
                            boundsForProfile()       <-- NEW, 3 assignments
                                     |
                              ReachabilityBounds
                                     |
                         buildRegion()  (UNCHANGED, Stage-4G4/5C.1)
                                     |
                         ReachabilityPrediction -> costmap -> MPPI
```

The decisive ordering fact (audit 1.1): `classify_track()` already runs inside
`update_track()`, which runs **before** `publish_tracks()` calls
`predict_reachability()`. The stabilised label is therefore available on the
same scan with no lag and no new plumbing. The selection is made **once per
track per scan** and shared by all six horizon samples, so a track's six
samples can never describe two profiles.

The costmap layer reads only `position`, `semi_major`, `semi_minor`,
`sigma_yaw` and `valid`, so **no costmap change was required**.

---

## 2. The profiles

| | `max_speed` | `max_accel` | `margin` | evidence basis |
|---|---|---|---|---|
| **PEDESTRIAN** | 2.5 m/s | 2.5 m/s² | 0.0 m | speed: **configured conservative capability prior**, aligned with the shipped Stage-5B `pedestrian.speed_max` gate. accel: measured crossing accelerations, mean 0.68 / **max 1.44 m/s²** [1], ×1.7, ≈0.25 g of foot traction |
| **BIKE** | 12.0 m/s | 7.0 m/s² | 0.0 m | speed: **configured conservative capability prior**, aligned with the shipped `bike.speed_max` gate. accel: pitch-over-limited braking, calculated **5.4–7.0 m/s²**; measured emergency front-or-both-brake 0.40–0.71 g = **3.9–7.0 m/s²** [2] |
| **VEHICLE** | 40.0 m/s | 10.0 m/s² | 0.0 m | speed: **configured conservative capability prior**, aligned with the shipped `vehicle.speed_max` gate. accel: measured ABS decelerations, 20 late-model vehicles, dry asphalt and concrete, **0.871–1.081 g = 8.5–10.6 m/s²** [3] |
| **UNKNOWN** | 40.0 m/s | 10.0 m/s² | 0.0 m | **not configured — DERIVED** as the envelope over its ambiguity set (§4) |

**What the `max_speed` column is, stated exactly.** 2.5 / 12.0 / 40.0 m/s are
**conservative configured capability priors aligned with the classifier's
per-class speed gates**. They are **not measured values and not
literature-proven physical maxima**: no evidence inside or outside this
repository establishes them as the fastest a member of each class can travel,
and Stage-5D does not claim it. They are declared premises, retained because
they are conservative at this system's operating geometry and because aligning
them with the gate the classifier already enforces is self-consistent (§2.1).
The `max_accel` column is different in kind: it rests on independently sourced
external measurement, cited below. §3 separates the three categories of number
in full, and §8.2 measures what an out-of-profile object costs.

Sources for the acceleration figures, which are the only externally sourced
numbers in Stage-5D:

* **[1]** *Microscopic Pedestrian Flow Characteristics: Development of an Image
  Processing Data Collection and Simulation Model*, arXiv:1610.00029 — measured
  crossing-pedestrian acceleration distribution, mean 0.68 m/s², maximum
  1.44 m/s².
* **[2]** SAE 2020-01-0876, *bicycle braking deceleration measured with a Slam
  Stick X*: 0.40–0.71 g front or front+rear, 0.25–0.37 g rear only; and the
  model-derived pitch-over limit of 5.4–7.0 m/s².
* **[3]** SAE 2023-01-0616, *Decelerations for Vehicles with Anti-lock Brake
  Systems (ABS) on Dry Asphalt and Concrete Road Surfaces*: average
  decelerations 0.871–1.081 g across 20 late-model vehicles.

### 2.1 Why `max_speed` is aligned with the classifier's own gate

This is self-consistency, not independent evidence and not a new constant.
It is the *reason the prior was set where it was*, not a proof that the
value is a physical maximum. `ClassificationParams` already
ships a per-class `speed_max` that **refuses** a label to an object observed
above it: 2.5 / 12.0 / 40.0 m/s. A held PEDESTRIAN label is therefore already
an assertion that `|v| <= 2.5`; reachability premise (P2) with
`max_speed = 2.5` asserts the same thing over the prediction window. Two
numbers that could disagree become one. It also means the classifier's veto
and `capabilityPremiseContradicted()` fire at the same speed.

### 2.2 Why `margin` is 0.0 for every class

Inherited deliberately from Stage-4G4's `base_safety_margin: 0.0`, so the
published region measures the motion model alone and the
global-vs-class-aware comparison is not confounded by a free additive term.
The field exists, is per-class and is configurable. For a deployment, the
value to set is the body's scan-plane half-extent; from the Stage-5B actor
footprints those are:

| actor | scan-plane footprint | bounding half-diagonal |
|---|---|---|
| `actor_pedestrian` | 0.40 × 0.28 m | 0.244 m |
| `actor_bike` | 1.61 × 0.04 m | 0.805 m |
| `actor_vehicle_compact` | 1.00 × 0.50 m | 0.559 m |
| `actor_vehicle` | 3.80 × 1.70 m | 2.082 m |
| `actor_planter` | 0.90 × 0.90 m | 0.636 m |

---

## 3. Three categories of number, kept apart

Section 3 of the brief requires the simulation domain and the capability
premises to be separated and never conflated. They are — and the separation
turns out to be the central honest finding about the dataset. Stage-5D uses
exactly three categories, and every number in this document belongs to one of
them:

| category | what it is | what it is not | where |
|---|---|---|---|
| **1. Simulation-qualified operating envelope** | what the Gazebo actors were commanded and observed to do — filtered speed never above 0.967 m/s, ramped 0.5 m/s² | not a capability claim, and **never used as a profile default** | §3.A; the domain §12's campaign is qualified in |
| **2. Configured conservative capability prior** | a declared premise chosen to be hard to exceed, asserted rather than proved. **The `max_speed` column (2.5 / 12.0 / 40.0 m/s)**, aligned with the classifier's per-class speed gates | **not measured, not literature-proven, not a universal physical maximum** | §2, §2.1, §3.B |
| **3. Independently sourced physical evidence** | an external measurement this repository did not produce. **The `max_accel` column (2.5 / 7.0 / 10.0 m/s²)**, with three citations | not evidence about *speed*, and not evidence this repository generated | §2 [1][2][3] |

Categories 2 and 3 are both premises the reachability bound rests on; they
differ only in what backs them, and that difference is stated wherever either
is used.

### 3.A Simulation-qualified capability (the Gazebo validation domain)

Measured over the retained Stage-5B recordings (2,547 rows), filtered speed
per ground-truth class:

| gt class | n | commanded | median | p95 | p99 | **max** |
|---|---|---|---|---|---|---|
| PEDESTRIAN | 578 | 0.25 / 0.50 / 0.75 | 0.250 | 0.723 | 0.759 | **0.776** |
| BIKE | 313 | 0.25 / 0.50 / 0.75 | 0.240 | 0.726 | 0.838 | **0.967** |
| VEHICLE | 989 | 0.25 / 0.50 / 0.75 | 0.223 | 0.721 | 0.743 | **0.759** |
| UNKNOWN | 667 | 0.25 / 0.50 / 0.75 | 0.233 | 0.720 | 0.752 | **0.771** |

Commanded acceleration: the Stage-4G4 scenario driver's ramped bound is
0.5 m/s²; the same Gazebo `VelocityControl` plugin can also apply a **step**
velocity change, and 5–9.5 m/s² was measured in the retained Stage-4G3
ground-truth odometry at segment boundaries — a residual that **no finite
acceleration bound covers at short horizon**, already documented and retained
as an out-of-model stress test.

**The finding: every actor was commanded at the same three speeds whatever its
class, so a simulation-fitted profile table is CLASS-INDEPENDENT.** It would be
about 1.0 m/s and 0.5 m/s² for all four classes, i.e. a single global profile
numerically within 25% of the Stage-5C configuration, and it would carry **no
semantic information at all**. Stage-5B said as much about its own dataset
("speed carries nearly no discriminative information… Real pedestrians,
cyclists and cars separate on speed; simulated ones here do not"); Stage-5D
confirms it quantitatively.

### 3.B The configured conservative capability prior — and why it is the default

`max_speed` and `max_acceleration` are premises about an obstacle's **future
capability** (`reachability.hpp`), so they must cover what the class *can* do,
not what the actors were *commanded* to do. Defaulting the class-aware mode to
the simulation envelope would under-bound a real bicycle by a factor of 15 and
a real car by 50 — exactly the error section 4 of the brief forbids for
UNKNOWN. So the table in §2 is the configured conservative prior (category 2,
with category-3 evidence behind its acceleration column only), and the
simulation envelope above is reported as the domain in which the Gazebo
results were obtained, not as a source of defaults.

**Which domain each Stage-5D result belongs to is stated with the result.**
§6–§8 are mathematical and hold for any profile in their declared domain.
§9–§11 use the real Stage-5B classifier output and the real-world-prior
profiles. §12 is a Gazebo campaign and is qualified only for the simulation
envelope above.

**Not claimed:** that the §2 numbers are universal real-world maxima. The
`max_speed` values in particular are configured conservative priors aligned
with the classifier's speed gates — **not measured and not
literature-proven** — and only the `max_accel` column carries independently
sourced evidence. Both are declared premises, their basis is stated per
column, and §8.2 measures what an out-of-profile object costs. Future physical
capability therefore remains a **declared model assumption**, not a verified
fact.

---

## 4. UNKNOWN is the top of the order, not a fourth class

UNKNOWN has **no profile parameters**. It is derived:

```
fallback = envelope over { profile(c) : c in A(UNKNOWN) }
```

so `fallback >= profile(c)` for every class `c` **by identity**, not by a
chosen pair of numbers that could drift. Asserted over 20,000 randomised
profile tables (140,000 checks, 0 violations) by `stage5d_profiles`, not only
at the shipped defaults.

Justification for the three values, as the brief requires:

* **`max_speed` 40.0 m/s, `max_accel` 10.0 m/s²** — the componentwise maximum
  over PEDESTRIAN/BIKE/VEHICLE, which happens to coincide with VEHICLE. The
  evidence that this is *necessary* rather than merely cautious is measured:
  of the 1,137 UNKNOWN detections in the Stage-5B recordings, **382 were
  ground-truth VEHICLE** and 197 ground-truth PEDESTRIAN; only 550 were the
  planter. An UNKNOWN object in that dataset demonstrably *is* a vehicle about
  a third of the time.
* **`margin` 0.0 m** — the maximum over the classes, all of which are 0.0 for
  the comparability reason in §2.2.

**The old global 0.8 m/s is explicitly NOT reused for UNKNOWN.** It would
under-bound a bicycle 15× and a car 50×. The audit (1.5) also records that
UNKNOWN is overloaded three ways — *not yet decided* (the dominant case: the
median track is UNKNOWN for its first 16 observations ≈ 3.2 s), *cannot be
decided*, and *decided to be none of the three* — and that only the third
describes the object at all. The first two describe the sensor and the
classifier, and in both the object may well be a vehicle.

---

## 5. Confidence policy, and why a confidence gate is not enough

**This is the central Stage-5D constraint, and it is measured, not assumed.**
Confidence attached to each confusion direction over the 2,547 recordings:

| gt → predicted | n | min conf | median | max |
|---|---|---|---|---|
| PEDESTRIAN → PEDESTRIAN | 381 | 0.253 | 0.856 | 1.000 |
| **VEHICLE → PEDESTRIAN** | **306** | **0.640** | **1.000** | **1.000** |
| VEHICLE → VEHICLE | 301 | 0.252 | 0.999 | 1.000 |
| BIKE → BIKE | 277 | 0.348 | 0.960 | 1.000 |
| UNKNOWN → VEHICLE | 117 | 0.268 | 0.536 | 0.999 |
| BIKE → PEDESTRIAN | 28 | 0.280 | 0.674 | 0.926 |
| * → UNKNOWN | 1137 | 0.000 | 0.000 | 0.000 |

A vehicle mislabelled PEDESTRIAN is reported **more** confidently than a
correctly labelled pedestrian (median 1.000 vs 0.856), and the
misclassification's *minimum* confidence, 0.640, sits above the correct
classification's *median*. **No threshold on `classification_confidence` can
admit true pedestrians while excluding those vehicles.** The mechanism is
physical and understood: a 1.0 × 0.5 m compact body viewed end-on returns a
0.42–0.45 m long, 0.02–0.03 m thick, fully filled slice in one horizontal
plane — a pedestrian-sized return. It is the same family of 2D-LiDAR
observability limit already documented for the six head-on bicycle scenarios.

So the policy has two independent parts.

**Part 1 — confidence and maturity gates (necessary, not sufficient).**

| gate | value | why that value |
|---|---|---|
| `reachability_profile_min_confidence` | **0.60** | the measured *floor* of the VEHICLE→PEDESTRIAN confusion. Below it no label is trusted; above it the confusion extends to 1.000, so confidence is asked only to reject weakly-held labels — the job it can do |
| `reachability_profile_min_observations` | **5** | the earliest observation at which the stabiliser can hold a label built from two consecutive confident scans (instant labels start at observation 3; two scans at confidence 1.0 cross `enter_threshold` 0.45 at 0.400 then 0.640), plus one. Measured first-label observation: min 4, median 16 |

A NaN confidence takes the conservative branch: the test is written `!(conf >=
min_confidence)`.

**Part 2 — ambiguity sets (the sufficient condition).** The applied profile is
the envelope over every class a label-carrying object might really *be*:

| predicted label | ground-truth composition (n) | `A(label)` | applied envelope |
|---|---|---|---|
| PEDESTRIAN | PED 381, **VEH 306**, BIKE 28 (715) | {PED, BIKE, VEH} | **vehicle grade** |
| BIKE | BIKE 277 (277) — **precision 1.000** | {BIKE} | **BIKE** |
| VEHICLE | VEH 301, UNKNOWN 117 (418) | {VEH, UNKNOWN} | vehicle grade |
| UNKNOWN | UNK 550, **VEH 382**, PED 197, BIKE 8 (1137) | all four | vehicle grade |

`CapabilityProfileSet` answers "what can a BIKE do" — physics, the same
whatever classifier produced the word. `AmbiguitySets` answers the different
question "what might an object *this* classifier calls BIKE actually be" — a
property of the classifier and its sensor. Conflating them in one number is
what makes a misclassification *invalidate* the bound instead of being
survived by it. The label is forced into its own closure in code, so a
configuration that omits it cannot weaken the guarantee.

**Consequence, stated plainly rather than engineered around: a PEDESTRIAN
label buys no reduction at all.** BIKE is the only label whose measured
precision — 1.000 per detection *and* per track, 0 false BIKE predictions —
earns a profile weaker than the fallback. That is a measurement of *this*
classifier at *this* operating point; a better classifier earns narrower sets
and must then re-measure them to claim them.

### 5.1 The required invariant

```
uncertain  =>  selected >= profile(c) for EVERY class c          (I3)
```

Whenever the selection is uncertain — UNKNOWN label, confidence below the
gate, or a track younger than `min_observations` — the derived fallback is
substituted outright, and by §4 the fallback dominates every class. So
uncertainty can never produce a smaller reachable set than certainty about
anything. Verified for 7 uncertainty cases × 8 horizons × 4 initial speeds:
**0 region shrinks**.

---

## 6. The safety order is a theorem, and it is measured

Define `A >= B` componentwise on (`max_speed`, `max_acceleration`, `margin`).

**Claim.** `A >= B` implies `region(A, t)` contains `region(B, t)` at every
`t`, for every track state and every observed speed.

**Proof.** `r_det(t) = ∫₀ᵗ min(a_max·s, v_eff + s₀) ds` with
`v_eff = max(v_max, s₀) >= v_max`. The integrand is pointwise nondecreasing in
`a_max` and in `v_max`, so the integral is nondecreasing in both. The semi-axes
are `sigma_term + r_det + margin` with a `sigma_term` that does not depend on
the profile at all, so both semi-axes are nondecreasing in each of the three
components, and the ellipses are concentric and co-oriented. ∎

`profileEnvelope` is the componentwise maximum, i.e. the **least upper bound**
of this order, which reduces every fallback and transition argument below to
one line. Measured rather than only argued:

| check | trials | violations |
|---|---|---|
| `r_det` monotone in each component | 9,216 | **0** (worst decrease 0.0) |
| full published region monotone, margin included | 320 | **0** |

---

## 7. Class transitions

Label changes are frequent, not hypothetical: **115 stable-label transitions
over 36 of 57 Stage-5B tracks (63%)**:

| transition | count | effect on the applied profile under §5's sets |
|---|---|---|
| UNKNOWN ↔ VEHICLE | 37 + 37 | none — both envelopes are vehicle grade |
| UNKNOWN ↔ PEDESTRIAN | 22 + 16 | none — both vehicle grade |
| BIKE → PEDESTRIAN | 2 | **enlarges** (BIKE → vehicle grade) |
| PEDESTRIAN → BIKE | 1 | **SHRINKS** — the one direction needing protection |

The rule is the minimum that makes a shrink impossible without warning. With
`H_k` the held profile after scan `k` and `C_k` that scan's selection:

```
H_k = C_k                      if the track is new
    = envelope(H_{k-1}, C_k)   if the label changed, or a hold is live
    = C_k                      otherwise
```

`reachability_profile_transition_hold_scans` = **3**, i.e. 0.6 s at the 5 Hz
LDS — one full `classification_smoothing` EWMA window (1/0.40 = 2.5 scans) — so
a shrink cannot be published until the new label has outlived the evidence
window that produced the old one, on top of the Stage-5B hysteresis
(enter 0.45 / exit 0.25 / switch margin 0.30) the label already had to survive.

Three invariants follow directly from `envelope` being the least upper bound:

* **(T1)** `H_k >= C_k` always — never weaker than the current selection;
* **(T2)** during a hold `H_k >= H_{k-1}` — occupancy cannot shrink at all;
* **(T3)** `H_k <= fallback` always — the envelope cannot grow without bound
  however often the label flaps.

Measured over 2,000 random label sequences × 40 scans = **80,000 steps**, with
randomised confidence and track maturity: **0 illegal shrinks, 0 exceedances of
the fallback**. And the hold is a *delay, not a ratchet*: one scan past the
window with a stable label, the profile collapses exactly to the new class's
closure (asserted bit-identically).

---

## 8. Mathematical qualification

### 8.1 In domain — the gate

Adversary: initial speed swept 0 → the profile's own cap in 9 steps, 16
acceleration directions at the extremal magnitude `|a| = a_max`, 6 published
horizons × {fresh, 1.0 s coasting}. `total_time` spans 0.5 → **4.0 s**, the
full production window (`max_observation_age` 1.0 + `reachability_horizon`
3.0).

| profile | `max_speed` | `max_accel` | trials | **escapes** | raw worst slack (m) | area @0.5 s | area @4.0 s | growth |
|---|---|---|---|---|---|---|---|---|
| PEDESTRIAN | 2.5 | 2.5 | 1,728 | **0** | −5.567e−12 | 0.476 m² | 331.8 m² | 697× |
| BIKE | 12.0 | 7.0 | 1,728 | **0** | −4.662e−11 | 2.846 m² | 4,757.8 m² | 1,672× |
| VEHICLE | 40.0 | 10.0 | 1,728 | **0** | −8.397e−10 | 5.530 m² | 20,144.8 m² | 3,643× |
| UNKNOWN (derived) | 40.0 | 10.0 | 1,728 | **0** | −8.397e−10 | 5.530 m² | 20,144.8 m² | 3,643× |
| **total** | | | **6,912** | **0** | | | | |

The raw worst slack is reported at full precision because the bound is
**tight**: for extremal straight-line motion the simulated trajectory lands
exactly on `r_det(t)`, so the sign at the optimum is decided by the midpoint
rule's `O(a·dt²)` residual, measured here at 1e−10 to 1e−12 m. Escapes are
therefore counted beyond a **1e−6 m** numerical allowance — the *same*
allowance Stage-5C's adversary used (`stage5c_audit.cpp:462`), kept identical
so the two qualifications are comparable. It hides nothing: a real escape is
six to seven orders of magnitude larger (§8.2).

Verified separately by `stage5d_profiles`: monotonicity (§6), the zero-profile
edge case (`a_max = 0` ⇒ `r_det = 0`, and an all-zero profile is *well formed*
— zero is a legitimate maximally restrictive premise, a provably immobile
object, not an error), very high acceleration (20 m/s² in the monotonicity
grid), observed speed at and above the configured cap (§8.3), and dimensional
consistency (`max_speed` enters only `delta_v_cap = v_max + s0` in m/s;
`max_acceleration` only `a_max` in m/s²; the equation itself is unmodified).

### 8.2 Out of domain — the limitation, demonstrated

A profile is a **declared premise about an unobservable future**, not a proven
bound. Where the true capability exceeds the declared one and the observed
speed does **not** reveal it, the region is escaped, and no runtime check can
prevent that. Observed speed held strictly below the configured cap throughout,
so `contract_status` reads `CONTRACT_SATISFIED` on every one of these samples —
which is the honest limit, not a defect:

| declared profile | true capability | trials | escapes | rate | worst escape | escape @4.0 s |
|---|---|---|---|---|---|---|
| PEDESTRIAN (2.5, 2.5) | a cyclist mislabelled PEDESTRIAN (12, 7) | 288 | 288 | 1.000 | **28.96 m** | 28.96 m |
| PEDESTRIAN (2.5, 2.5) | an athletic side-step at 0.5 g (2.5, 5) | 288 | 199 | 0.691 | **1.41 m** | 1.41 m |
| BIKE (12, 7) | a car mislabelled BIKE (40, 10) | 288 | 288 | 1.000 | **42.29 m** | 42.29 m |
| VEHICLE (40, 10) | 1.5× the declared capability (60, 15) | 288 | 288 | 1.000 | **40.00 m** | 40.00 m |

The second row is the **declared residual** of the PEDESTRIAN acceleration
figure, stated in §2 and measured here: an athletic standing start or side-step
is traction-limited nearer 0.5 g and escapes a 2.5 m/s² profile by up to
1.41 m inside the 4.0 s window. It is reported in the same spirit as the
existing 5–9.5 m/s² Gazebo step-velocity residual, not hidden.

This is precisely why §5's ambiguity closure exists: it is what stops rows 1
and 3 from being reachable at runtime, by never letting a PEDESTRIAN or BIKE
label select a profile weaker than what the label might really be covering.

### 8.3 The observable breach — Stage-5C.1 preserved per profile

For the selected class profile, `configured_max_speed = class_profile.max_speed`
and the Stage-5C.1 response is untouched:
`effective_max_speed = max(configured_max_speed, observed_speed)` when the
breach predicate holds.

| profile | configured | observed | trials | escapes *without* the 5C.1 response | escapes *with* it | worst slack with response | every sample flagged |
|---|---|---|---|---|---|---|---|
| PEDESTRIAN | 2.5 | 3.125 | 96 | 6 | **0** | +5.5e−14 | yes |
| BIKE | 12.0 | 15.0 | 96 | 1 | **0** | +2.4e−12 | yes |
| VEHICLE | 40.0 | 50.0 | 96 | 0 | **0** | −2.0e−09 | yes |

The VEHICLE row's 0 is reported as measured and explained rather than smoothed:
at 40 m/s with `a_max` 10 m/s², the crossover time `t_c = (v+s₀)/a` is 9.0 s,
well beyond the 4.0 s window, so the *acceleration* branch binds throughout and
raising the speed cap changes nothing. The premise is still disproved and
`CONTRACT_OBSERVED_SPEED_EXCEEDS_CAP` is still published on every sample.

A measured case where class-awareness makes a real contract difference, not
just a size difference: the Stage-5B recordings contain a filtered bike speed
of **0.967 m/s**. That **breaches** the global 0.8 m/s cap (17 breached samples
× 6 horizons = 102 in the replay) and is **far inside** every class profile
(0 breaches in the class-aware arms).

**What is still not claimed:** the premise quantifies over the future, so
nothing verifies it. The unobservable residual — true capability above the cap
while the filtered speed stays below it — is inherited verbatim from
Stage-5C.1, now *per class*, and §8.2 measures what it costs.

---

## 9. Three arms over the real recorded classifier output

2,547 recorded Stage-5B rows, 57 tracks, 54 scenarios. The replay re-runs the
**production** Kalman filter along each track's measured centroids, so the
state each arm predicts from is the real filter's, driven by real measurements.
The arms differ in **nothing but which profile the selector returns**.

* **A GLOBAL** — the single Stage-5C/5C.1 configuration (0.8, 0.5, 0.0).
* **B RUNTIME CLASS-AWARE** — the recorded Stage-5B label and confidence.
* **C ORACLE CLASS-AWARE** — the ground-truth class.
  **ANALYTICAL / EVALUATION ONLY.** It exists only to separate class-aware
  *model* quality from *classifier error*.

### 9.1 Oracle discipline

Ground truth is read in exactly one field (`Row::gt_label`) of one
analysis-only file (`stage5d_audit.cpp`), and reaches only arm C and §10. The
production node is not built against that file and has **no parameter that
could read it**. `extract_labels.py` emits the label as a column of a replay
table that only the offline audit consumes.

### 9.2 Two containment definitions, kept apart

Conflating them is how a safety claim gets overstated, so both are reported:

* **CAPABILITY containment** — does the arm's region cover everything an object
  of the **true** class could do? Exactly: is the region's *minor* semi-axis at
  least `r_det(t)` under the true class's profile. This is the safety question,
  and a misclassification can fail it.
* **OBSERVED containment** — does the track's own **later measured position**
  fall inside the region predicted for that time? This is the calibration
  question, answered from recorded data with no ground truth of any kind.

### 9.3 Pooled over all six horizons (15,282 samples per arm)

| | **A global** | **B runtime class-aware** | **C oracle — EVAL ONLY** |
|---|---|---|---|
| **Capability containment** | **0.0171** | **1.0000** | **1.0000** |
| Observed containment | 0.9956 | 1.0000 | 1.0000 |
| Mean region area | 47.31 m² | 2,151.73 m² | 2,127.76 m² |
| **Planner-visible cells** | **461.4** | **529.0** | **529.0** |
| Contract breaches | **102** | **0** | 0 |
| Fallback usage | 0.000 (n/a) | **0.543** | 0.262 |
| UNKNOWN-profile usage | 0.000 (n/a) | 0.895 | 0.877 |
| Transition-hold rate | 0.000 | 0.0012 | 0.0000 |
| **Unsafe profile selections** | **15,282 / 15,282** | **0** | **0** |
| Label transitions | — | 115 | 0 (ground truth is constant per track) |

Two results dominate this table.

**The qualified global configuration does not bound any real class.** Arm A's
capability containment is 1.7%, and **every one of its 15,282 samples** selects
a profile that fails to dominate the true class's capability. Its *observed*
containment is 99.6% only because the Gazebo actors are slow. This is the
quantitative form of the warning Stage-5C.1 already wrote into the YAML, and it
is why §4 refuses to reuse 0.8 m/s for UNKNOWN.

**Classifier error costs almost nothing under this policy.** Arm B's mean
region area is **1.13% larger** than the oracle's, and its capability
containment is identical at 1.0000. What classifier error actually costs is
*fallback rate* — 0.543 vs 0.262 — i.e. the runtime arm spends twice as many
samples on the conservative envelope, which is the designed response to
uncertainty rather than a failure.

### 9.4 Per horizon — and where the planner stops seeing a difference

| horizon | arm | capability cont. | mean area (m²) | mean `r_det` (m) | **cells** |
|---|---|---|---|---|---|
| 0.5 s | A global | 0.0365 | 0.60 | 0.062 | **123.3** |
| 0.5 s | B runtime | 1.0000 | 7.26 | 1.211 | **529.0** |
| 0.5 s | C oracle | 1.0000 | 7.20 | 1.204 | **529.0** |
| 1.0 s | A global | 0.0130 | 3.62 | 0.250 | 529.0 |
| 1.0 s | B runtime | 1.0000 | 97.70 | 4.843 | 529.0 |
| 1.0 s | C oracle | 1.0000 | 96.83 | 4.816 | 529.0 |
| 3.0 s | A global | 0.0271 | 151.74 | 2.056 | 529.0 |
| 3.0 s | B runtime | 1.0000 | 7,291.32 | 43.018 | 529.0 |
| 3.0 s | C oracle | 1.0000 | 7,204.01 | 42.677 | 529.0 |

**The planner-visible difference between runtime and oracle class-aware is
exactly zero at every horizon.** The difference between global and class-aware
exists at exactly one sample, 0.5 s: 123.3 vs 529.0 cells.

### 9.5 Per true class, pooled

| true class | arm | n | capability cont. | mean area (m²) | cells | fallback rate | unsafe |
|---|---|---|---|---|---|---|---|
| PEDESTRIAN | A global | 3,468 | 0.0597 | 47.05 | 461.1 | — | 3,468 |
| | B runtime | 3,468 | 1.0000 | 2,287.50 | 529.0 | 0.522 | **0** |
| | C oracle | 3,468 | 1.0000 | 2,287.50 | 529.0 | 0.000 | 0 |
| **BIKE** | A global | 1,878 | 0.0096 | 48.79 | 462.4 | — | 1,878 |
| | **B runtime** | 1,878 | **1.0000** | **1,176.54** | 529.0 | **0.093** | **0** |
| | C oracle | 1,878 | 1.0000 | **981.45** | 529.0 | 0.000 | 0 |
| VEHICLE | A global | 5,934 | 0.0040 | 47.68 | 461.7 | — | 5,934 |
| | B runtime | 5,934 | 1.0000 | 2,291.65 | 529.0 | 0.427 | **0** |
| | C oracle | 5,934 | 1.0000 | 2,291.65 | 529.0 | 0.000 | 0 |
| UNKNOWN | A global | 4,002 | 0.0030 | 46.28 | 460.7 | — | 4,002 |
| | B runtime | 4,002 | 1.0000 | 2,284.25 | 529.0 | 0.943 | **0** |
| | C oracle | 4,002 | 1.0000 | 2,284.25 | 529.0 | 1.000 | 0 |

**BIKE is where class-awareness pays, and only BIKE.** Its mean region area is
1,176.54 m² against ~2,290 m² for every other class — a **48.6% reduction**
against the fallback — and that is earned by measured precision 1.000, not
assumed. PEDESTRIAN's 2,287.50 m² is *identical* to the oracle's, because the
measured VEHICLE→PEDESTRIAN confusion forces its envelope to vehicle grade in
both arms: a correctly classified pedestrian gains nothing.

The runtime BIKE arm is 19.9% larger than the oracle BIKE arm
(1,176.54 vs 981.45 m²), entirely from its 9.3% fallback rate — the scans where
the bike track was too young or too weakly labelled to be trusted.

### 9.6 Cost

| measurement | value |
|---|---|
| `buildRegion` mean, per sample | **163.8 ns** |
| per track, 6 samples | **0.98 µs** |
| CV state propagation, per sample | 14.6 ns |
| samples timed | 45,846 |

The profile selection is a handful of comparisons and three `std::max` calls on
a path that already computes `hypot(v0x, v0y)`, so the tracker-side cost is
unchanged by construction; the live figure is in §12.

---

## 10. Classification-error analysis — every direction, not an aggregate

For every confusion direction that actually occurred, the one question that
matters: **does the profile the runtime arm selected still dominate what the
object really is?**

| ground truth → runtime label | n | **unsafe selections** | fallback samples | class-closure samples | confidence range | mean applied `max_speed` | true class `max_speed` |
|---|---|---|---|---|---|---|---|
| UNKNOWN → UNKNOWN | 550 | **0** | 550 | 0 | [0.000, 0.000] | 40.00 | 40.00 |
| VEHICLE → UNKNOWN | 382 | **0** | 382 | 0 | [0.000, 0.000] | 40.00 | 40.00 |
| PEDESTRIAN → PEDESTRIAN | 381 | **0** | 105 | 276 | [0.253, 1.000] | 40.00 | 2.50 |
| **VEHICLE → PEDESTRIAN** | **306** | **0** | 0 | 306 | **[0.640, 1.000]** | **40.00** | **40.00** |
| VEHICLE → VEHICLE | 301 | **0** | 40 | 261 | [0.252, 1.000] | 40.00 | 40.00 |
| **BIKE → BIKE** | 277 | **0** | 7 | 267 | [0.348, 1.000] | **13.01** | 12.00 |
| PEDESTRIAN → UNKNOWN | 197 | **0** | 197 | 0 | [0.000, 0.000] | 40.00 | 2.50 |
| UNKNOWN → VEHICLE | 117 | **0** | 79 | 38 | [0.268, 0.999] | 40.00 | 40.00 |
| BIKE → PEDESTRIAN | 28 | **0** | 14 | 14 | [0.280, 0.926] | 40.00 | 12.00 |
| BIKE → UNKNOWN | 8 | **0** | 8 | 0 | [0.000, 0.000] | 40.00 | 12.00 |
| **total** | **2,547** | **0** | | | | | |

Direction by direction, as the brief requires:

* **VEHICLE → PEDESTRIAN (306 samples, the dangerous one).** Confidence
  [0.640, 1.000], so the gate does not and cannot catch it. The closure does:
  because `A(PEDESTRIAN)` contains VEHICLE, the applied profile is 40.00 m/s,
  exactly covering the true 40.00. **0 unsafe selections.** This is the single
  measured justification for the whole ambiguity-set design.
* **BIKE → PEDESTRIAN (28).** Applied 40.00 against a true 12.00 —
  over-conservative by 3.3×, which is safe. Half the samples were caught by the
  gates anyway.
* **BIKE → UNKNOWN (8), PEDESTRIAN → UNKNOWN (197), VEHICLE → UNKNOWN (382).**
  All take the fallback at 40.00. Safe by §4, and the VEHICLE→UNKNOWN row is
  the measured reason §4 refuses an optimistic UNKNOWN.
* **UNKNOWN(planter) → VEHICLE (117).** Applied 40.00 against a planter —
  maximally over-conservative, safe, and the direction a tighter UNKNOWN
  profile would have had to justify.
* **The cost, stated plainly: PEDESTRIAN → PEDESTRIAN (381 *correct*
  classifications) is served 40.00 m/s against a true 2.50 — a 16× over-bound.**
  A correctly classified pedestrian gains nothing from class-awareness. That is
  the price of the measured confusion, paid by the class that would have
  benefited most.
* **BIKE → BIKE (277).** Mean applied 13.01 m/s against the profile's 12.00;
  the excess is the 7 fallback scans and the transition holds. This is the one
  direction where a semantic label buys a genuinely tighter corridor.
* **Ephemeral tracks / fragments.** Only 1 of 57 tracks lived ≤5 scans
  (audit 1.7), and `actor_bike` was the only actor that fragmented (median 1,
  max 3 tracks). Fragments arrive as new track ids, which start UNKNOWN and
  immature and therefore on the fallback — the conservative branch by
  construction.

---

## 11. Planner visibility — why the difference stops at the costmap

Region area is the model's own calibration measure; planner-visible **cells**
are what MPPI consumes, and they are bounded by the layer's
`max_influence_radius` policy (0.55 m at production geometry, 0.05 m
resolution, saturating at **529 cells**).

| profile | `r_det` @0.5 s | @1.0 s | @4.0 s | **saturates the window at `total_time`** | cells @0.5 s |
|---|---|---|---|---|---|
| A global (0.8, 0.5) | 0.0625 m | 0.250 m | 2.56 m | **1.49 s** | 5 |
| PEDESTRIAN (2.5, 2.5) | 0.3125 m | 1.250 m | 8.75 m | **0.67 s** | 121 |
| BIKE (12, 7) | 0.8750 m | 3.500 m | 37.71 m | **0.40 s** | 529 |
| VEHICLE (40, 10) | 1.2500 m | 5.000 m | 80.00 m | **0.34 s** | 529 |
| UNKNOWN (derived) | 1.2500 m | 5.000 m | 80.00 m | **0.34 s** | 529 |

**Every class profile saturates the representable window before or just after
the first published horizon sample.** Beyond that the planner receives the same
529 cells whatever the class, so the 48.6% region-area reduction BIKE earns in
§9.5 is **invisible to MPPI at production geometry**. This is a property of the
costmap policy bound, not of the class-aware model, and it is the mechanical
reason the decision below is *no planner benefit* rather than *not useful*: the
model is better calibrated and demonstrably safer against misclassification,
and the representation layer cannot currently express the difference.

---

## 12. Bounded navigation campaign

36 Gazebo trials: 6 conditions × 2 arms × 3 trials, one full simulator restart
per trial on its own `ROS_DOMAIN_ID`, arms **interleaved** within each trial
index so machine drift over a 33-minute campaign cannot bias one of them.
Full data: [`nav_campaign.json`](nav_campaign.json).

### 12.1 What differs between the arms, and it is one line

| | arm GLOBAL | arm CLASS-AWARE |
|---|---|---|
| tracker params | `stage5d_global_tracker.yaml` | `stage5d_classaware_tracker.yaml` |
| the only difference between those two files | `reachability_class_aware: false` | `reachability_class_aware: true` |
| nav2 params file | `nav2_stage4f_params.yaml` — **the same unmodified file** | the same unmodified file |
| costmap `prediction_mode` | `reachability`, applied at runtime | `reachability`, applied at runtime |

```
$ diff <(grep -v '^\s*#' stage5d_global_tracker.yaml    | grep -v '^\s*$') \
       <(grep -v '^\s*#' stage5d_classaware_tracker.yaml | grep -v '^\s*$')
36c36
<     reachability_class_aware: false
---
>     reachability_class_aware: true
```

Because **one** nav2 params file serves both arms, MPPI, every critic weight,
the planner, the robot limits, the velocity smoother, the collision monitor,
the costmap geometry and the resolution **cannot** differ between arms even by
accident — MPPI was not retuned, and there is no second file that could drift.
Map, world, start, goal, obstacle body, obstacle trajectory, release time,
sensor model, AMCL, the Kalman filter, the CV predictor, the Stage-5B
classifier, compound clustering, the reachability mathematics and the
Stage-5C.1 contract are shared through the same launch invocation.

Three independent checks that the arms really were what they claim:

| check | GLOBAL | CLASS-AWARE |
|---|---|---|
| layer `mode=` observed in the costmap's own stats line | `reachability` | `reachability` |
| tracker's class-aware start-up banner | absent | present |
| `PROFILE_GLOBAL` share of published samples | **20,364 / 20,364 (100%)** | **0 / 20,256 (0%)** |

### 12.2 Conditions

Pedestrian crossing, pedestrian longitudinal, bicycle crossing, vehicle
crossing, vehicle longitudinal and an UNKNOWN (planter) crossing. The bodies
are the Stage-5B actors, generated from the single `ACTORS` table that also
supplies their ground-truth class, each yawed along its travel — a 2D LiDAR
measures one horizontal slice, so a bicycle not yawed along travel presents
its end rather than its two wheel chords and is not classifiable at all.

`actor_vehicle_compact` (1.0 × 0.5 m) is the vehicle body, **not**
`actor_vehicle` (3.8 × 1.7 m): the frozen tracker rejects any cluster whose
bounding-box diagonal exceeds `cluster_max_diameter` 1.2 m, so the compact
body is the only VEHICLE-class object the unmodified clustering can represent.
That is a Stage-5B finding carried forward, not a Stage-5D choice.

**A multi-obstacle condition is NOT included, and its absence is reported
rather than worked around:** the shared launch spawns exactly one scripted
obstacle, and adding a second would change the launch for every earlier stage.

### 12.3 Aggregate

| metric | **GLOBAL reachability** | **RUNTIME class-aware** |
|---|---|---|
| trials launched | 18 | 18 |
| infrastructure faults (re-run) | 1 → re-run, then 0 | 0 |
| **trials scored** | **18** | **18** |
| **succeeded** | **18 / 18** | **18 / 18** |
| **collisions** | **0** | **0** |
| near misses (< 0.20 m clearance) | 4 | **3** |
| planner / controller failures | 0 | 0 |
| collision-monitor interventions | 1 | 1 |
| min clearance, mean (m) | 0.5948 | **0.6101** |
| **min clearance, worst single trial (m)** | **0.0578** | **0.1254** |
| navigation time, mean (s) | 20.100 | 19.792 |
| path length, mean (m) | 5.9997 | 5.9896 |
| primary reaction time, mean (s) | 4.018 | 3.973 |
| **peak planner-visible predictive cells, mean** | **1140.6** | **1261.1** |
| layer `updateCosts` mean (µs) | 25.69 | 26.06 |
| layer `updateCosts` max (µs) | 85.1 | 675.8 (one spike, §12.6) |
| tracker reachability compute, mean (µs/scan) | 7.10 | 7.49 |
| **reachability contract breaches** | **0** | **0** |
| published profile samples | 20,364 | 20,256 |
| fallback rate | — (n/a) | **0.4991** |
| class-closure rate | — | 0.5009 |
| transition-hold rate | — | 0.0000 |
| **unsafe profile-selection events** (`PROFILE_FALLBACK_INVALID_CONFIG`) | **0** | **0** |

### 12.4 Per condition

| condition | arm | ok | col | near miss | clr worst | clr mean | nav mean | path mean | cells mean |
|---|---|---|---|---|---|---|---|---|---|
| ped_cross | global | 3 | 0 | 0 | 0.7068 | 0.7158 | 18.88 | 5.805 | 1036.0 |
| ped_cross | class-aware | 3 | 0 | 0 | 0.6923 | 0.7034 | 18.24 | 5.752 | 1172.0 |
| **ped_longitudinal** | global | 3 | 0 | 3 | **0.0578** | 0.0861 | 16.50 | 6.002 | 1153.3 |
| **ped_longitudinal** | class-aware | 3 | 0 | 3 | **0.1254** | 0.1449 | 16.65 | 6.062 | 1308.0 |
| bike_cross | global | 3 | 0 | 0 | 0.6657 | **0.6913** | 23.63 | 6.179 | 1536.3 |
| bike_cross | class-aware | 3 | 0 | 0 | 0.6209 | **0.6349** | 22.98 | 6.126 | 1669.7 |
| veh_cross | global | 3 | 0 | 0 | 0.8634 | 0.8707 | 23.54 | 5.965 | 1022.0 |
| veh_cross | class-aware | 3 | 0 | 0 | 0.8660 | 0.8732 | 23.47 | 5.893 | 1129.7 |
| **veh_longitudinal** | global | 3 | 0 | **1** | 0.1601 | 0.2455 | 16.55 | 6.141 | 1049.7 |
| **veh_longitudinal** | class-aware | 3 | 0 | **0** | 0.2044 | 0.3198 | 16.49 | 6.178 | 1160.0 |
| unknown_cross | global | 3 | 0 | 0 | 0.9469 | 0.9593 | 21.48 | 5.906 | 1046.3 |
| unknown_cross | class-aware | 3 | 0 | 0 | 0.9780 | 0.9841 | 20.92 | 5.926 | 1127.0 |

### 12.5 Per obstacle class

| gt class | arm | trials | clr mean | clr worst | cells mean | fallback rate | class-closure rate |
|---|---|---|---|---|---|---|---|
| PEDESTRIAN | global | 6 | 0.4009 | 0.0578 | 1094.7 | — | — |
| PEDESTRIAN | class-aware | 6 | **0.4241** | **0.1254** | 1240.0 | 0.697 | 0.303 |
| BIKE | global | 3 | **0.6913** | 0.6657 | 1536.3 | — | — |
| BIKE | class-aware | 3 | **0.6349** | 0.6209 | 1669.7 | 0.397 | **0.603** |
| VEHICLE | global | 6 | 0.5581 | 0.1601 | 1035.8 | — | — |
| VEHICLE | class-aware | 6 | **0.5965** | **0.2044** | 1144.8 | 0.135 | **0.865** |
| UNKNOWN | global | 3 | 0.9593 | 0.9469 | 1046.3 | — | — |
| UNKNOWN | class-aware | 3 | **0.9841** | **0.9780** | 1127.0 | **0.998** | 0.002 |

The fallback rates are the live confirmation of §5 and §9: the UNKNOWN
(planter) condition sits on the fallback 99.8% of the time, VEHICLE takes its
closure 86.5% of the time, and BIKE — the one label that earns a weaker
profile — takes its own closure 60.3% of the time.

### 12.6 Where class-aware helped, where it hurt

**Helped — the two tight longitudinal conditions, which are the only ones in
the set where clearance is genuinely at stake.** Both are head-on passes down
a lane offset only 0.25 m from the route, so the obstacle crosses the robot's
corridor and roughly a 0.45 m lateral shift is what clears it.

* `ped_longitudinal`: worst-trial clearance **0.0578 → 0.1254 m**, a 2.2×
  improvement, and the mean 0.0861 → 0.1449 m. Still 3 near misses in both
  arms — the condition is tight by construction — but every one of them is
  further from contact.
* `veh_longitudinal`: worst-trial clearance 0.1601 → **0.2044 m**, mean
  0.2455 → 0.3198 m, and the single near miss in the global arm becomes
  **zero**.

The mechanism is the one §9 predicts: both classes' labels resolve to a
vehicle-grade envelope, which is far larger than the global 0.8 m/s profile,
so the planner sees the conflict corridor earlier and shifts further.

**This is NOT evidence that semantic discrimination helped the planner, and is
not recorded as such.** Stated as five separate facts, none of which supports
that conclusion:

1. The runtime class-aware arm improved **some aggregate navigation metrics**
   relative to the old global 0.8 m/s reachability profile — worst-trial
   clearance on the two tight longitudinal conditions, one near miss removed,
   marginally lower mean navigation time.
2. The global profile is **known to under-bound** the broader real-world
   capability assumptions these profiles encode: it fails to dominate the true
   class capability on **100% of 15,282 replay samples** (§9.3), and a
   measured 0.967 m/s bike speed already disproves its premise (§8.3).
   Beating it is therefore beating a profile that was too small, not evidence
   that class labels are informative.
3. **Every** class profile saturates the 0.55 m `max_influence_radius` policy
   window early — by `total_time` 0.34–0.67 s, at or just past the first
   published horizon sample (§11).
4. Therefore the planner **cannot meaningfully distinguish most class-specific
   regions**: beyond that point it receives the same 529 cells per sample
   whatever the class, and runtime and oracle class-aware paint an
   **identical** cell count at every horizon (§9.4).
5. Therefore the observed navigation improvement **cannot be attributed to
   semantic class discrimination.** It is attributable to class-aware being
   uniformly more conservative than an under-bounding global profile.

**Hurt — `bike_cross`.** Mean clearance **0.6913 → 0.6349 m** (−0.056 m) and
worst 0.6657 → 0.6209 m. This is the expected, and in fact the *designed*,
consequence of the only place class-awareness tightens anything: BIKE takes
its own closure on 60.3% of samples, and the BIKE profile is the only one
weaker than the fallback. Neither arm came close to contact in this condition
(both ≈ 0.62–0.69 m, against a 0.42 m contact threshold), so this is a
calibration difference rather than a safety one — but it is the direction a
tighter corridor must move clearance, and it is reported as such rather than
averaged away.

**Neutral.** `ped_cross`, `veh_cross` and `unknown_cross` differ by
0.002–0.025 m in mean clearance, inside trial-to-trial spread.

**Cost.** The tracker pays 7.10 → 7.49 µs/scan (+5.5%) for the profile
selection, and the layer 25.69 → 26.06 µs (+1.4%) for 10.6% more painted
cells — sub-linear, because the extra cells land in a window already being
iterated. The 675.8 µs layer maximum is **one spike in one trial**
(`veh_longitudinal_classaware_01`, between update 76 and 102); that trial's
running mean stayed at 31.0 µs and the next-worst maximum across all 18
class-aware trials is 120.0 µs. It is reported rather than trimmed, and
nothing was optimized.

### 12.7 What the campaign does NOT show

Planner-visible cells rose 1140.6 → 1261.1 (+10.6%), which is a real but
*undifferentiated* increase: §11 shows every class profile saturates the
0.55 m `max_influence_radius` window by `total_time` 0.34–0.67 s, so beyond
the first horizon sample the planner receives the same 529 cells per sample
whatever the class. The offline replay makes this explicit — **runtime and
oracle class-aware paint an identical cell count at every horizon** (§9.4).
So the navigation differences above come from class-aware being *more*
conservative than the under-bounding global profile, **not** from the planner
distinguishing a pedestrian from a bicycle.

A 36-trial campaign with 3 trials per cell also cannot resolve differences of
a few centimetres in mean clearance. The claims drawn from it are limited to
the ones the margins support: no collisions and no failures in either arm,
and a worst-case clearance improvement on the two tight conditions that is
larger than the spread within those cells.

---

## 13. Production baseline preserved

| | value | checked by |
|---|---|---|
| costmap `prediction_mode` (production) | **`cv_covariance`** | `production_nav2_params.yaml` unmodified by Stage-5D |
| tracker `reachability_enabled` (production) | **`false`** | `production_tracker_params.yaml`, unchanged |
| tracker `reachability_class_aware` (all three shipped YAMLs) | **`false`** | `stage5d_profiles` asserts the node's own default is false |
| class-aware-off path vs Stage-5C.1 | **0 bit differences** | `stage5d_profiles`, through the real node |
| CV prediction vs class label | **0 bit differences** | `stage5d_profiles`: two tracks, identical state, different labels, `predict_future()` compared bitwise on x/y/vx and all four covariance entries |
| MPPI / controller / planner tuning | **unchanged** | one `params_file` shared by both Stage-5D arms; `production_nav2_params.yaml` untouched |
| ground-truth label in runtime | **none** | §9.1 |

`reachability_class_aware: false` returns `reachability_bounds_` **verbatim**
(not an equivalent reconstruction) and deliberately does not touch the
per-track transition state, so switching the mode on at runtime starts every
track from a clean, conservative first scan.

---

## 14. Tests

### New

| test | package | covers |
|---|---|---|
| `stage5d_profiles` | tracking | 8 groups: monotonicity of the order (9,536 checks); per-label selection at the defaults; fallback dominance over 20,000 random tables (140,000 checks); the uncertainty invariant (7 cases, incl. NaN confidence, new track, gate boundaries); transitions (80,000 steps over 2,000 random label sequences, plus the shrink/enlarge/expiry cases); the observable breach per profile; malformed configuration (9 cases + all-NaN table + empty/narrowed ambiguity sets + every valid enum value); and end to end through the production node incl. the class-aware-off bit-identity and CV-unchanged checks |

Result: **0 failures**, machine output in
[`profile_tests.json`](profile_tests.json).

### Configuration-robustness cases (brief section 17)

| case | handling | why |
|---|---|---|
| negative `max_speed` / `max_acceleration` / `margin` | **rejected**, replaced by a dominating substitute, `PROFILE_FALLBACK_INVALID_CONFIG` published on every sample | a negative bound is not a bound |
| NaN in any field | **rejected** as above | NaN propagates into the published semi-axes |
| Inf in any field | **rejected** as above | not a usable bound |
| all four entries malformed | **rejected**, replaced by the built-in conservative default (not zero) | there is nothing well formed to envelope |
| zero in every field | **accepted** — it is well formed | zero is a legitimate maximally restrictive premise (a provably immobile object); and because the fallback is a *maximum*, zeroing one class cannot pull it down |
| missing UNKNOWN profile | **by design** — UNKNOWN has no parameters and is derived (§4) | it cannot be configured into being less conservative than a class it must cover |
| unknown `ObstacleClass` enum value | the four wire values are asserted total, and every one yields a well-formed profile bounded by the fallback | the stabiliser only ever returns one of the four |
| unrecognised class name in an ambiguity set | the whole set falls back to **all four classes**, with an `RCLCPP_ERROR` | narrowing a set weakens the profile, so a configuration the node cannot read must never buy a smaller region |
| empty ambiguity set | the label is forced into its own closure in code | an empty set would otherwise hand the label a zero profile |
| narrowed `A(UNKNOWN)` | honoured, and the resulting weaker fallback is **not** silently overridden | it is a legitimate operator choice and must be visible, not hidden |

In every rejection case the substituted profile is asserted to **dominate
VEHICLE**, so a typo can never buy a smaller region than the correct table
would have.

### Pre-existing functional tests — all 12 still pass

`stage4g2_lifecycle`, `stage4g3_deblend`, `stage4g4_reachability`,
`stage5b_classification`, `stage5b_tracker_integration`, `stage5b1_compound`,
`stage5b1_compound_integration`, `stage5c1_contract` (8, tracking);
`stage4g2_multi_layer`, `stage4g4_reach_layer`, `stage4g5_hybrid_layer`,
`stage5c1_raster_fidelity` (4, costmap).

With `stage5d_profiles` that is **13 / 13 functional tests passing**, 9 in
tracking and 4 in costmap.

`stage4g4_reachability`, `stage5b_tracker_integration`,
`stage5b1_compound_integration` and `stage5c1_contract` all call
`predict_reachability(track, stamp)` with the pre-Stage-5D two-argument shape;
the third parameter defaults to `nullptr`, meaning "use the single global
configuration", so those four tests exercise the legacy path **exactly**
rather than an equivalent of it.

**One reproducibility note for anyone extending the message.**
`ReachabilityPrediction.msg` gained four fields, so `predictive_nav_costmap`
**must** be rebuilt along with `predictive_nav_msgs`: a costmap built against
the old layout reads the new message at the old field offsets, which yields
garbage semi-axes and aborts `nav2_container` as soon as the layer is in
`reachability` mode and a track exists. `colcon build` with no
`--packages-select` is the safe invocation.

### Pre-existing lint debt (not introduced by Stage-5D, not fixed by it)

Linters fail at the baseline commit, in files Stage-5D does not touch. The
exact set, after Stage-5D:

| package | linter | failures | files |
|---|---|---|---|
| tracking | `uncrustify` | 4 | `include/.../kalman_filter.hpp`, `test/stage4g2_lifecycle.cpp`, `test/stage4g3_deblend.cpp`, `test/stage4g4_reachability.cpp` |
| tracking | `flake8` | 14 | `scripts/stage4g1_nearwall_probe.py`, `scripts/stage4g1_static_probe.py` |
| tracking | `pep257` | 13 | the same two scripts |
| costmap | `uncrustify` | 4 | `src/predicted_obstacle_layer.cpp`, `test/stage4g2_multi_layer.cpp`, `test/stage4g4_reach_layer.cpp`, `test/stage4g5_hybrid_layer.cpp` |
| bringup | `flake8` | 996 | 35 pre-existing `scripts/` files, incl. `scripts/stage4f_trial.py` |
| bringup | `pep257` | 193 | 34 pre-existing `scripts/` files, incl. `scripts/stage4f_trial.py` |

`cppcheck`, `lint_cmake` and `xmllint` pass in tracking and costmap;
`lint_cmake` and `xmllint` pass in bringup.

**Every file Stage-5D ADDED passes `uncrustify`, `flake8` and `pep257`** —
`capability_profiles.hpp`, `test/stage5d_profiles.cpp`,
`scripts/stage5d/run_nav.py` (0 flake8, 0 pep257),
`validation/stage5d/extract_labels.py` and
`validation/stage5d/analyze_nav.py`.

**Stage-5D added no new lint failure to any file it MODIFIED**, which for
`scripts/stage4f_trial.py` is stated as measured rather than as a pass: that
file carries pre-existing bringup debt and its failure counts are **identical
before and after Stage-5D** (flake8 34 → 34, pep257 46 → 46, measured against
the baseline commit's copy of the file). `lidar_obstacle_tracker_node.cpp`
passes `uncrustify` and is not among the four tracking `uncrustify` failures
above. Stage-5D neither added to the debt nor fixed it.

---

## 15. Scope exclusions honoured

No IMM filter, no neural trajectory prediction, no YOLO, no camera fusion, no
intent prediction, no learned classifier, no real robot, no 448-trial
regression, no MPPI retuning, no unrelated refactor. The reachability equation
was not modified; no defect in it was found.

The three non-Stage-5D files that were touched are enabling changes for
section 13's own requirement and all default to the pre-Stage-5D behaviour:

* `stage4f_bringup.launch.py` — `obstacle_model`, `obstacle_yaw`,
  `obstacle_spawn_z`, defaulting to `dynamic_obstacle.sdf`, yaw 0, z 0.3.
* `stage4f_trial.py` — `obstacle_yaw` (default 0.0) and one generalised
  expression, `cos/sin(heading - yaw)`, which at yaw 0 is *literally* the
  previous expression.
* `stage4f_trial.py` / `stage4f_bringup.launch.py` — `layer_prediction_mode`
  (default `''`, which sends nothing), applied through `SetParameters`, which
  is the mechanism `predicted_obstacle_layer.cpp` already documents for that
  parameter. **No new nav2 params file was needed**: both Stage-5D arms launch
  with the unmodified `nav2_stage4f_params.yaml`, so MPPI and every critic
  weight cannot differ between arms even by accident.

---

## 16. Decision gates

| gate | status | evidence |
|---|---|---|
| A. class capability profiles explicitly defined | **PASS** | §2, one `MotionCapabilityProfile` per class in one parameter contract |
| B. values backed by stated evidence / operating assumptions | **PASS** | §2 (3 cited external sources + the shipped speed vetoes), §3 (sim vs real separated) |
| C. UNKNOWN fallback conservatively defined | **PASS** | §4, derived as an envelope; 140,000 randomised dominance checks, 0 violations |
| D. low-confidence handling safe | **PASS** | §5, §5.1; 0 region shrinks over 7 uncertainty cases |
| E. class transitions cannot silently shrink occupancy unsafely | **PASS** | §7; 80,000 steps, 0 illegal shrinks, 0 fallback exceedances |
| F. Stage-5C.1 observable breach handling preserved | **PASS** | §8.3, per profile, every sample flagged |
| G. each profile mathematically qualified in its declared domain | **PASS** | §8.1 |
| H. 0 in-domain adversarial escapes | **PASS** | **0 / 6,912** |
| I. out-of-domain limitation explicitly demonstrated | **PASS** | §8.2, escapes 1.41–42.29 m measured and reported |
| J. runtime classifier arm compared against global reachability | **PASS** | §9, §12 |
| K. classifier errors analyzed explicitly | **PASS** | §10, all 10 directions, per direction |
| L. bounded planner comparison completed | **PASS** | §12 |
| M. CV production behaviour unchanged | **PASS** | §13, 0 bit differences |
| N. no ground-truth label leakage into production runtime | **PASS** | §9.1 |
| O. relevant functional tests pass | **PASS** | §14 |

---

## 17. Files

### New

```
src/predictive_nav_tracking/include/predictive_nav_tracking/capability_profiles.hpp
src/predictive_nav_tracking/test/stage5d_profiles.cpp
src/predictive_nav_tracking/config/stage5d_global_tracker.yaml
src/predictive_nav_tracking/config/stage5d_classaware_tracker.yaml
src/predictive_nav_bringup/scripts/stage5d/run_nav.py
validation/stage5d/CLASS_REACHABILITY_BOUNDARY_AUDIT.md
validation/stage5d/STAGE5D_README.md
validation/stage5d/stage5d_audit.cpp
validation/stage5d/extract_labels.py
validation/stage5d/analyze_nav.py
validation/stage5d/profile_tests.json
validation/stage5d/profile_audit.json
validation/stage5d/nav_campaign.json
```

### Modified

```
src/predictive_nav_msgs/msg/ReachabilityPrediction.msg          4 additive fields + 7 constants
src/predictive_nav_tracking/src/lidar_obstacle_tracker_node.cpp selection + parameters + metadata + counters
src/predictive_nav_tracking/CMakeLists.txt                      register stage5d_profiles
src/predictive_nav_tracking/config/tracker_params.yaml          Stage-5D block at code defaults
src/predictive_nav_tracking/config/production_tracker_params.yaml   same (class_aware stays false)
src/predictive_nav_tracking/config/stage5b_classification_tracker.yaml   same
src/predictive_nav_bringup/launch/stage4f_bringup.launch.py     4 additive launch args, pre-5D defaults
src/predictive_nav_bringup/scripts/stage4f_trial.py             obstacle_yaw + layer_prediction_mode (both inert by default)
src/predictive_nav_bringup/CMakeLists.txt                       install run_nav.py
```

The YAML additions set every new parameter to the value the node already
defaults to, so they are behaviourally inert and exist to make the profiles,
the ambiguity sets and the gates explicit in configuration.

## 18. Decision

**`QUALIFIED_CLASS_AWARE_NO_PLANNER_BENEFIT`**

The profiles are correct and better calibrated than the single global
configuration, and they are demonstrably safe against the classifier's
measured errors — but a navigation benefit attributable to *semantic
discrimination* is not demonstrated.

Why this option and not the others:

* **not `QUALIFIED_CLASS_AWARE_REACHABILITY`.** That would claim useful
  calibrated prediction reaching the planner. At the production costmap
  geometry it does not: every class profile saturates the 0.55 m policy window
  by `total_time` 0.34–0.67 s (§11), and runtime and oracle class-aware paint
  an **identical** 529 cells at every horizon (§9.4). The navigation
  improvements in §12.6 come from class-aware being *more* conservative than
  an under-bounding global profile, not from telling a pedestrian from a
  bicycle. And only BIKE earns a profile weaker than the fallback — a
  correctly classified pedestrian gains nothing (§10).
* **not `PARTIAL_CLASSIFICATION_LIMITED`.** Classifier uncertainty does *not*
  prevent safe deployment. The measured result is the opposite: **0 unsafe
  profile selections in all 10 confusion directions over 2,547 real
  classifier decisions** (§10), including the 306-sample `VEHICLE →
  PEDESTRIAN` confusion at median confidence 1.000, and 1.13% more region area
  than the ground-truth oracle (§9.3). The ambiguity-closure policy converts
  classifier uncertainty into *conservatism*, which is the correct direction,
  rather than into risk. What classifier error costs is a fallback rate of
  0.499 live (0.543 in replay) against the oracle's 0.262 — a usefulness cost,
  not a safety one.
* **not `NEEDS_CORRECTION`.** No implementation or model defect was found. The
  reachability equation was not modified and needed no modification; 0
  in-domain escapes over 6,912 adversarial trials; 0 bit differences on the
  class-aware-off path and on CV output; 36/36 navigation trials succeeded with
  0 collisions.
* **not `NOT_USEFUL`.** Class-aware profiles provide two things the qualified
  global method provably does not. First, **capability containment rises from
  0.0171 to 1.0000** (§9.3): the global 0.8 m/s / 0.5 m/s² configuration fails
  to dominate the true class capability on 100% of samples, and the class-aware
  arms dominate it on 100%. Second, **contract breaches fall from 102 to 0**
  (§8.3, §9.3): a 0.967 m/s filtered bike speed that disproves the global
  premise is comfortably inside every class profile. Both are real, measured
  improvements in the correctness of the second prediction method, independent
  of whether the planner can see them.

### Stage-5D verdict: **PASS**

All 15 acceptance gates pass (§16). A PASS does not require navigation
superiority, and none is claimed.

### The limitations on turning this on — there are two, and they are independent

**Limitation 1 — `max_influence_radius` = 0.55 m saturates the
planner-visible class corridors.** Until the costmap can represent a region
larger than its policy window without saturating, no class-aware profile is
distinguishable from any other at the planner: every profile saturates by
`total_time` 0.34–0.67 s (§11) and runtime and oracle class-aware paint an
identical 529 cells at every horizon (§9.4). Raising that bound is a
costmap-policy and MPPI-interaction question — it changes what every mode
paints, including the production CV mode — and is therefore outside Stage-5D.

**Limitation 2 — classifier ambiguity forces conservative envelopes, so most
labels buy no corridor reduction even if limitation 1 were removed.** This is
a *separate* cause with the same effect on usefulness, and it is measured:

* The `VEHICLE -> PEDESTRIAN` confusion exists **at high confidence** — 306
  samples at min 0.640, **median 1.000**, above correct
  `PEDESTRIAN -> PEDESTRIAN`'s median of 0.856 — so no confidence threshold
  can separate them (§5).
* Consequently the ambiguity-envelope policy gives many PEDESTRIAN-labelled
  tracks the 40 m/s fallback / vehicle-grade envelope, because
  `A(PEDESTRIAN)` must contain VEHICLE for the bound to survive the confusion.
* **381 correct `PED -> PED` tracks were served 40.0 m/s instead of the
  nominal 2.5 m/s pedestrian profile** — a 16× over-bound paid by the class
  that would have benefited most (§10).
* **BIKE is currently the only class with a substantial corridor reduction**:
  48.6% less region area than the fallback (§9.5), earned by measured
  precision 1.000 per detection and per track.

**This is not a safety defect.** It is a calibration / usefulness limitation.
The envelope policy is working exactly as designed and in the correct
direction: 0 unsafe profile selections in all 10 confusion directions over
2,547 real classifier decisions (§10), 0 collisions and 0 failures across
36/36 navigation trials (§12.3). The policy is **not weakened** on the
strength of this finding; narrowing `A(PEDESTRIAN)` requires a sensor that can
separate a compact car's end-on slice from a pair of legs, and until then the
conservative envelope is the correct response.

**Limitation 3 — the profile `max_speed` values are configured conservative
capability priors, not universal experimentally proven physical maxima** (§2,
§3). Future physical capability remains a **declared model assumption**; §8.2
measures what it costs when the assumption is wrong.

Two narrower follow-ups that do not need limitation 1 lifted:

1. **The PEDESTRIAN profile is only worth having if `A(PEDESTRIAN)` can be
   narrowed**, which needs a sensor that can separate a compact car's end-on
   slice from a pair of legs. That is a perception problem, not a reachability
   one.
2. **`max_acceleration` carries most of the weight, and for VEHICLE/UNKNOWN
   `max_speed` is effectively inert.** The crossover time is
   `t_c = (max_speed + s0) / max_acceleration`, so at `s0 = 0` it is 1.00 s for
   PEDESTRIAN, 1.71 s for BIKE, 1.60 s for the global profile, and **4.00 s for
   VEHICLE** — exactly the edge of the production window, and later still for
   any `s0 > 0`. So for VEHICLE and the derived UNKNOWN fallback the
   acceleration branch `0.5·a·t²` governs the whole window and the speed cap
   never binds; §8.3's VEHICLE row (0 escapes even *without* the Stage-5C.1
   response) is the measured consequence. That matters because
   `max_acceleration` is also where the evidence is weakest: it is the one
   quantity with no per-class measurement anywhere in this repository, and all
   three values are external citations (§2).
