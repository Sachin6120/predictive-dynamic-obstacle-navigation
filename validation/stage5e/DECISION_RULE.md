# Stage-5E decision rule — fixed and hashed BEFORE the aggregate was read

Written and hashed while the campaign was at trial 4 of 135. At that point one
`ped_cross` trial per arm for two of the three arms existed and had been used
only to smoke-test the analyser; no aggregate, no per-condition table and no
pairwise comparison had been computed, and the class-aware arm had produced no
scored navigation trial at all. The offline prediction audit
(`prediction_audit.json`) had been run, so its containment numbers were known —
and the rule below is written precisely so that containment **cannot by itself
decide the architecture**.

This file follows the Stage-4G6 / Stage-4G7 convention: the rule is committed
to before the evidence, so the evidence cannot be read backwards into the rule.

---

## 1. What the production/default predictor must maximise

In priority order, as Stage-5E section 10 states:

1. **navigation reliability** — goal reached, no collision, no timeout, no
   planner or controller failure;
2. **useful clearance / safety behaviour** — clearance that is actually
   larger where a conflict exists, and a *response* that resolves the conflict
   (yield or detour) rather than one that stalls;
3. **stable planner interaction** — a bounded number of stops, bounded global
   path churn, no field collapse, no trial in which the robot cannot find a
   way through;
4. **bounded computation** — layer update time and tracker time within the
   5 Hz costmap budget, with no missed update deadlines;

while retaining **adequate prediction quality** — the predictor must still
describe where the obstacle will be, to the standard its own class of model is
accountable to.

## 2. Two things that do NOT win an architecture decision

These are stated as exclusions because both are true of at least one arm and
neither is evidence of fitness:

- **E1 — a larger conservative set does not win.** Painting more cells, or a
  larger region area, is a cost until it is shown to buy a navigation outcome.
  An arm whose predictive region saturates the influence window carries *less*
  information than one that does not, because a saturated field is
  indistinguishable from any other saturated field: the planner cannot act on a
  distinction it cannot see.
- **E2 — higher mathematical containment does not win.** Capability
  containment is a property of a premise, not of a trajectory. An arm may
  contain everything a class could conceivably do and still give the planner
  nothing it can use. Containment qualifies a predictor as *sound*; it does not
  qualify it as *deployable*.

## 3. The gates

Let **P** be the candidate production arm and **B** the incumbent (CV, the
Stage-4G7/4G8 production default).

**G1 — RELIABILITY (mandatory).** P must be no worse than B on scored success
rate, collisions, timeouts and planner/controller failures. A single
collision or timeout that B does not have, in any condition, fails G1.

**G2 — SAFETY BEHAVIOUR (mandatory for a CHANGE of default).** To displace B,
P must show a clearance advantage in the conflict conditions that is
**practical** (mean minimum-clearance gain ≥ 0.10 m, the Stage-4G6/4G7
threshold, carried forward unchanged) **and outside the two-sigma-of-the-
difference band**, or must remove a collision/near-miss class that B has.

**G3 — PLANNER STABILITY (mandatory).** P must not increase stops, stop
duration or global path churn beyond run-to-run noise, must produce a global
plan in every scored trial, and must not convert a yield/detour response into
a stall.

**G4 — COST (mandatory).** P must keep the layer within the 5 Hz budget with
zero rate shortfalls and zero costmap update failures.

**G5 — SOUNDNESS (mandatory).** P must have zero unsafe profile events
(`prof_fb_badconfig`) and must not introduce an uncontained contract breach
that B does not have. A contract breach that the Stage-5C.1 observable
response already repairs is reported, not counted against P.

## 4. The decision

- **SHIP_CV** if CV passes G1, G3, G4, G5 and no other arm passes G2 against
  it. This is the decision whenever the reachability arms are merely *not
  worse*: the incumbent is not displaced by a tie.
- **SHIP_GLOBAL_REACHABILITY** if the global arm passes G1–G5 against CV.
- **SHIP_CLASS_AWARE_REACHABILITY** if the class-aware arm passes G1–G5
  against CV.
- **NO_SINGLE_WINNER** if two or more arms pass every gate and are separated by
  nothing outside noise on any priority-1 or priority-2 metric, **or** if
  different conditions select different arms under G1/G2 with no arm dominating.

## 5. Role classification, decided by the same evidence

For each non-shipped reachability arm:

- **DEPLOYED_SECONDARY** — it passes G1, G4 and G5, and it supplies a
  representation the shipped predictor demonstrably does not, which a
  second consumer (a safety monitor, a speed limiter, an analysis view) could
  read *without* being on the navigation path. Being runnable is not enough;
  there must be a measured property it uniquely provides.
- **ANALYSIS_ONLY** — it is sound (G5) and reproducible, and it answers a
  question about the system, but it does not give a deployable consumer
  anything beyond what the shipped predictor already gives.
- **RESEARCH_ONLY** — it is sound as mathematics but the measured planner
  behaviour at the production window is uninformative (e.g. a saturated
  field), so no deployable consumer can be specified for it as configured.
- **RETIRED** — it is unsound, or it is strictly dominated with no path to a
  consumer, so keeping it costs more than it is worth.

## 6. What is explicitly NOT a ground for any of the above

- the Stage-5D/5D.1 **oracle** arms. They are an evaluation-only upper bound
  and are not in this campaign;
- the historical expectation recorded in Stage-5E section 12. It is a
  hypothesis. If the evidence contradicts it, the evidence wins;
- any result from the two conditions this campaign could not run
  (reversal/acceleration-change, multi-obstacle). Those are reported as
  limitations and may not be used to support a positive claim about any arm.

## 7. Production configuration during qualification

Unchanged, in all cases. Stage-5E issues a **recommendation**; it does not
edit `production_nav2_params.yaml` or `production_tracker_params.yaml`.
