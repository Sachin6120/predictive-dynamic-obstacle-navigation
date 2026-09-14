# Stage-4G6 pre-registered decision rule

**Written and committed to before any Stage-4G6 trial was run.** Nothing in this
file was revised after aggregate results were read. Its purpose is to stop the
decision being made by whichever comparison happened to look best.

Question: does the hybrid coasting fallback provide a practically meaningful
NAVIGATION benefit over CV-only prediction, sufficient to justify maintaining
two prediction representations in the production path?

## Primary endpoint

**Minimum footprint-to-obstacle clearance** in occlusion scenarios where the
observation gap overlaps the robot's approach to the conflict. Chosen because it
is the safety quantity the predictive layer is supposed to influence, it is
continuous (so it carries more information per trial than success/failure), and
it was the metric that showed the largest nominal — but non-separable —
difference at Stage-4G5.

## What counts as a meaningful Hybrid benefit

Hybrid is KEPT if, in at least one occlusion scenario class, it satisfies at
least one SAFETY criterion **and** violates no COST criterion.

### Safety (at least one required)

| id | criterion | threshold | why this number |
|---|---|---|---|
| **S1** | mean minimum clearance improvement | **≥ 0.10 m** *and* the 95% CI of the paired-arm difference excludes 0 | 0.10 m is half the obstacle radius and ~20% of the 0.4–0.8 m clearances observed at Stage-4G5. Stage-4G5 measured a per-trial spread of ±0.2 m, so anything below 0.10 m is inside the noise we have already characterised and cannot be called a safety improvement. |
| **S2** | worst-case single-trial minimum clearance | **≥ 0.10 m** better than CV's worst, **or** hybrid has zero trials below a 0.10 m near-miss while CV has ≥ 1 | A prediction safety net earns its place by improving the tail, not the mean. |
| **S3** | collisions or navigation failures | **≥ 2 fewer events out of 20** (≥ 10 percentage points) | Below 2/20 a difference is one unlucky trial. |

### Cost (none may be violated)

| id | criterion | threshold |
|---|---|---|
| **C1** | goal time increase vs CV | ≤ 10% of CV's mean |
| **C2** | path length increase vs CV | ≤ 5% of CV's mean |
| **C3** | stopped-fraction increase vs CV | ≤ 5 percentage points |

### Decision

* **KEEP HYBRID** — at least one of S1/S2/S3 met in at least one occlusion
  scenario class, and no cost criterion violated there.
* **REMOVE REACHABILITY FROM THE PRODUCTION PATH** — otherwise. The Stage-4G4
  and Stage-4G5 implementations are retained as documented research; only the
  recommendation for the final architecture changes.

## Reporting rules fixed in advance

* Mean, SD, median, n, and a 95% CI for the arm difference (bootstrap, 10,000
  resamples — no normality assumption, since clearance distributions are
  bounded below and visibly skewed).
* **Hedges' g** effect size (Cohen's d with the small-sample correction), since
  per-arm n is 20–30.
* p-values reported where a test is appropriate (Welch's t on clearance,
  Fisher's exact on binary outcomes) but explicitly **not** used as the decision
  rule; the thresholds above are.
* Out-of-model (abrupt, unbounded-acceleration) manoeuvres are analysed and
  reported **separately** and never enter the primary decision.
* Failed trials are retained and reported. A trial is excluded **only** if it is
  a verified simulation/setup failure (launch crash, recording error), and every
  exclusion is listed individually with its cause.
* The constant-visibility control is a **falsification test**: if Hybrid differs
  materially from CV there, the hybrid implementation is wrong and the primary
  comparison is invalid regardless of its outcome.

## Frozen configuration

No parameter is retuned for Stage-4G6. `fresh_threshold` 0.1 s,
`max_acceleration` 0.5 m/s², `max_speed` 0.8 m/s, `sigma_level` 1.5,
`temporal_decay` 0.35, `max_cost` 250, `track_timeout` 1.0 s, and every MPPI
weight are exactly as validated at Stage-4G5.
