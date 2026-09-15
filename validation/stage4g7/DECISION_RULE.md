# Stage-4G7 pre-registered decision rule (FINAL production decision)

**Written and hashed before any Stage-4G7 decision trial was run.** Only the
geometry-calibration pilots (which measure line-of-sight, not arm performance)
preceded it. Nothing here was revised after aggregate results were read.

This rule supersedes Stage-4G6's for the final decision. Stage-4G6's version had
two defects, both found only after it returned a KEEP that its own follow-up
evidence contradicted: it contained **no replication clause** and **no mechanism
clause**, so a single non-reproducing result driven by the wrong obstacle
satisfied it. Both are now mandatory gates rather than commentary.

## Question

Does the hybrid coasting fallback improve robot safety when a previously
established dynamic track is hidden by STATIC geometry, keeps moving while
hidden, and the conflict develops during or immediately after the occlusion?

## Primary endpoint

**Minimum footprint clearance to the OCCLUDED TARGET** (ground-truth object 0) —
not the global minimum over all obstacles. Stage-4G6 demonstrated why: there the
global minimum was set by the moving occluder in 100% of trials, an obstacle
both arms predict identically, so the global metric could register a
"benefit" that prediction had nothing to do with. Global minimum clearance and
the identity of the clearance-limiting entity are still reported, always.

## KEEP HYBRID requires ALL FOUR gates

### A. PRACTICAL EFFECT — at least one of

| id | criterion | threshold |
|---|---|---|
| **A1** | mean target-clearance gain | **≥ 0.10 m** with a bootstrap 95% CI excluding 0 |
| **A2** | worst-case single-trial target clearance | **≥ 0.10 m** better than CV's worst |
| **A3** | lower-tail target clearance (10th percentile) | **≥ 0.10 m** better than CV's |
| **A4** | collisions with the target | **≥ 2 fewer per 20 trials** (rate-scaled to actual n) |

0.10 m is half the obstacle radius and ~20% of typical clearances; Stage-4G5/4G6
measured per-trial spreads of ±0.13–0.20 m, so smaller differences are inside
already-characterised noise.

### B. REPLICATION — required

The qualifying effect must reproduce in **at least two** of:
* the primary scenario run as two independent batches, or
* a second gap duration (0.40 / 0.60 / 0.80 s variants), or
* the hidden-manoeuvre scenario.

"Reproduce" means same sign and at least half the magnitude of the qualifying
effect. A single qualifying scenario is **not** sufficient.

### C. MECHANISM — required, all three

1. **The occluded target is the clearance-limiting object** in the majority of
   trials of the qualifying scenario.
2. **The arms differ while Hybrid is actually using reachability** — the layer
   reports reachability track-updates > 0 during the gap.
3. **The predictive costmap and the robot command differ BEFORE reacquisition.**
   If both arms behave identically until the target reappears, reachability had
   no causal role and the effect is attributed elsewhere, regardless of A.

### D. COST — none may be violated

| id | criterion | threshold |
|---|---|---|
| **D1** | goal time increase vs CV | ≤ 15% of CV's mean |
| **D2** | path length increase vs CV | ≤ 8% of CV's mean |
| **D3** | stopped-fraction increase | ≤ 8 percentage points |

## Decision

* **KEEP HYBRID** — A and B and C and D all satisfied.
* **SHIP CV-ONLY** — otherwise. The Stage-4G4/4G5 reachability and hybrid
  implementations, tests and results are retained unchanged as research; only
  the production `prediction_mode` recommendation is affected.

## Fixed in advance

* Statistics: mean, SD, median, min, 10th percentile, bootstrap 95% CI
  (10,000 resamples, no normality assumption), Hedges' g, Welch's t and Fisher's
  exact reported for context. **The decision is made by the thresholds above,
  never by a p-value.**
* Out-of-model (unbounded-acceleration) manoeuvres are reported separately and
  never enter the decision.
* The never-observed case is a documented limitation, never a comparison.
* Failed trials are retained. Exclusion only for verified simulation/setup
  failures, each listed individually with its cause, and both arms topped up
  symmetrically afterwards.
* The constant-visibility control is a falsification test: Hybrid must record
  **zero** reachability track-frames there and match CV. If it does not, the
  implementation is wrong and the primary comparison is void.
* No parameter is retuned: `fresh_threshold` 0.1 s, `max_acceleration` 0.5,
  `max_speed` 0.8, `sigma_level` 1.5, `temporal_decay` 0.35, `track_timeout`
  1.0 s and every MPPI weight are exactly as validated at Stage-4G5.
