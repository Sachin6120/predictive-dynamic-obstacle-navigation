# Stage-4G2 — multi-object tracking validation

**PASS within the explicitly measured LiDAR/clustering resolvability envelope.**
The close, merged-cluster crossing is **not** an identity-preservation success.
It is retained as an upstream limitation, as permitted by acceptance criterion
12. Greedy association is retained. No association upgrade, birth heuristic,
clustering redesign, timeout extension, costmap change or controller tuning.
Stage-4G3 has not started.

Checkpoint: clean `main` at `eab2218`; Stage-4F annotated reference still peels
to `68b2bf2ee1efb774ba563438a4c621b0052f61d5`. Work is confined to
`~/predictive_nav_ws`; `~/ur5e_ws` was neither accessed nor modified.

## What changed

- Repository-local simulation/evaluation tooling in
  `src/predictive_nav_bringup/scripts/stage4g2/` and explicit Python/ROS dependencies.
- Optional `profile_scans` timing logs, false by default. This is the **only
  production tracker source change**. The functions implementing static
  rejection, clustering, association, lifecycle, KF updates and predictions
  remain identical to the checkpoint.
- Two C++ regression targets exercising the actual tracker and actual costmap
  plugin; three evaluator correctness checks and a process-group cleanup regression.
- Compact measured JSON/CSV evidence. Raw logs remain ignored under
  `validation_logs/stage4g2/`; no raw LiDAR logs are committed.

Pre-change inspection: [STAGE4G2_INSPECTION.md](STAGE4G2_INSPECTION.md).
Per-trial metrics: [trials.csv](stage4g2/trials.csv),
[results.json](stage4g2/results.json). Failure/reacquisition windows:
[crossing_timelines.csv](stage4g2/crossing_timelines.csv).

## Frozen association and message semantics

For every prior track i and centroid j:

```
dt_i = clamp(scan_stamp - last_observation_stamp_i, 0.001, 1.0)
x_pred_i = F(dt_i) x_i             # state [px, py, vx, vy]
d_ij = ||z_j - H x_pred_i||_2
eligible iff d_ij <= 0.6 metres
```

Sort **all eligible pairs together** by distance and accept each pair whose
track and detection are both unused. This is greedy one-to-one edge selection;
equal-distance tie order is unspecified. It is not a per-track loop independently
choosing the nearest detection, and is not minimum-total-cost assignment.
Velocity already enters through F. Covariance is **not** an association cost.

Matched tracks perform the existing KF prediction/correction with
`R = 0.05² I`, acceleration parameter `0.5`, and per-axis
`Q = 0.5² [[dt⁴/4, dt³/2], [dt³/2, dt²]]`.
The implemented Q is discrete random acceleration despite the older header's
CWNA label; this documentation observation does not change the model.
Unmatched centroids create tentative tracks. Publication requires three **total**
observations, not necessarily consecutive. Velocity is initialized after two.

Tracks are pruned **after association**, when `missed_count > 5` or elapsed
observation time `> 1.0 s`. Missing scans increment age/missed count, not
observations. Stored state/covariance and object stamp remain at the last
observation; published array stamp is the current scan. Future sample stamps
are the object observation stamp + horizon. Retained state is not a detection.

All track containers and messages are variable-length. Each Track owns x, P,
history, counters and timestamps. Each owns six predictions at 0.5…3.0 s,
independently calculated from its parent x/P. No shared mutable filter exists.

## Simulation and trial definitions

The unchanged Stage-4F 10 × 8 m Gazebo arena, localization map, robot, 5 Hz
LiDAR, tracker YAML and Nav2 YAML are used. Robot starts at (-3,0), yaw 0.
Every obstacle is the **same 0.20 m radius, 0.60 m tall cylinder**, with separate
Gazebo model name, velocity command topic, and simulator odometry topic.
The mover uses simulation elapsed time, not GT identity feedback. Start
positions and Cartesian velocity/segments are independently configurable.
All objects are spawned and held until discovery and initial confirmation
settle, then receive deterministic commands. Actual simulator odometry, rather
than assumed commanded motion, supplies scoring truth.

`scenarios.json` is the exact definition. Coordinates are metres and velocities
m/s. Each object's entry is `start → velocity`:

| Scenario | A | B | Duration / repetitions |
|---|---|---|---|
| Single timing control | (-1,-1) → (0.3,0) | — | 10 s × 2 |
| Separated parallel | (-1,-1) → (0.3,0) | (-1,1) → (0.3,0) | 10 s × 2 |
| Opposing pass | (-1.5,-0.45) → (0.3,0) | (1.5,0.45) → (-0.3,0) | 10 s × 2 |
| Close perpendicular crossing | (-1.5,0) → (0.5,0) | (0,2.15) → (0,-0.5) | 10 s × 2 |
| Wider-offset perpendicular crossing | (-1.5,0) → (0.5,0) | (0,2.7) → (0,-0.5) | 10 s × 2 |
| Near crossing, paths do not intersect | (-1.5,-0.45) → (0.3,0) | (0,2) → (0,-0.3) for 5 s, then (0.3,0) | 10 s × 2 |
| Short physical occlusion | (-1,-2) → (0,0.6) | (0,0) → (0.15,0) | 7 s × 2 |
| Three-object scaling | (-1,-1.5) → (0.25,0) | (-1,0) → (0.2,0); C: (-1,1.5) → (0.3,0) | 10 s × 1 |
| Focused navigation | (-1,-1.5) → (0.15,0) | (-1,1.5) → (0.15,0) | 22 s × 1 |
| Static navigation regression | no objects | robot goal (3,0) | 19 s × 1 |
| Long physical occlusion, expiry control | (-1,-0.8) → (0,0.15) | (0,0) → (0.05,0) | 12 s × 1 |
| Resolution sweep | (0,-d/2) → (0.15,0) | (0,d/2) → (0.15,0), d = .55/.65/.75/.85 | 6 s × 1 each |

The close crossing has a commanded arrival offset of 1.3 s and measured
minimum centre spacing about 0.46 m, so cylinders do not physically intersect.
The wider crossing has offset 2.4 s and minimum spacing about 0.85 m. Identical
objects occupying the exact intersection simultaneously would force a physical
collision; that would be a poor identity-preservation test.

**22 final scored Gazebo runs, 1,105 scan frames.** The 14-run baseline matrix
was completed before the 8 regression/limit runs. Two initial successful pilot
runs remain separate (`baseline/parallel_01`, `baseline_v1/single_01`). One
unscored startup failure (`baseline_v1/single_02`) timed out waiting for Nav2
component discovery/static map. It is not counted as a tracking trial. The
runner now uses a new DDS domain and Gazebo partition per trial, and terminates
only process groups it created. The final cleanup audit found some Nav2
containers had outlived their launch leader during shutdown. Cleanup now
verifies every non-zombie member of the owned group and escalates bounded
SIGINT → SIGTERM → SIGKILL if needed; the exited-leader/live-child case has a
regression test. All verified task-owned leftovers were removed. This shutdown
issue did not prevent the scored navigation goals succeeding and is distinct
from navigation-time deadlock. Repeats exercise timing/noise variation; they
are not independent population samples. No runtime parameters were optimized
against any scenario or ID-switch count.

## Scoring

GT odometry is interpolated at the scan timestamp, requiring bracketing samples.
GT exists only in the simulation/evaluator process. Runtime tracker subscriptions
remain `/scan` and `/map` plus TF; no GT is sent to association or prediction.

Evaluation uses maximum-cardinality, minimum-Euclidean-distance one-to-one
assignment with a **0.45 m gate**: 0.20 m cylinder surface bias + 0.15 m
(three measurement-noise sigma) + 0.10 m localization allowance. This gate was
specified before the baseline matrix. The evaluator's linear assignment is
**not a runtime association upgrade**. IDs do not enter its cost.

- A detection/match requires a fresh track (`missed_count == 0`). Retained
  tracks are scored separately after explicitly propagating their state to
  scan time; their existence never increments detection rate.
- ID switch: assigned ID differs from the previous matched ID for that GT,
  including across observation gaps.
- Observation fragment: matched → unmatched → matched interruption, even if
  the same ID returns. Identity fragment: number of distinct assigned IDs − 1.
- Purity per GT: dominant-ID matched frames / all its matched frames.
- Position/velocity errors use fresh matched KF outputs versus simulator centres
  and velocities. Initial velocity convergence is included. Surface-centre bias
  is not compensated.
- False confirmed track: a fresh track outside every GT gate. Extra tracks
  inside a matched GT gate are counted as duplicates. Persistence requires
  >=1 s between false observations of that ID. Tentative tracks are counted in
  timing telemetry but are not individually identity-scored; zero false
  **confirmed** births does not prove an absence of every transient tentative
  birth in dynamic scenes. Static navigation has zero active tracks of any kind.

For upstream diagnosis, the evaluator attributes **actual LiDAR endpoints** to
GT cylinder surfaces within 0.10 m, then checks the real clusterer's 0.35 m
connectivity. Merge evidence requires at least two returns from each object
in one connected component. Actual runtime candidate centroids are also saved.
Fewer than three supported returns is reported separately as observation loss.
This diagnostic reconstruction never supplies runtime measurements. Its surface
labels are approximate under localization error; it is not an oracle clusterer.

Metrics in a merged frame cannot identify which physical object a blended
centroid represents. Raw geometric switch/purity scores are nevertheless kept,
with before/after IDs and merge flags, rather than silently discarding failures.
The separation sweep demonstrates why zero switches alone is insufficient:
a single merged track can leave an entire GT object unmatched.

## Baseline = final results (greedy unchanged)

Values separated by `/` are the two trials. Purity is the mean of per-GT
purities; matched fraction pools object-frames. Errors are matched-frame means.

| Scenario | ID switches | Observation fragments | Mean purity | Fresh matched fraction | Position error m | Velocity error m/s |
|---|---:|---:|---:|---:|---:|---:|
| Parallel | 0 / 0 | 0 / 0 | 1 / 1 | 1 / 1 | .172 / .177 | .035 / .037 |
| Opposing | 0 / 0 | 0 / 0 | 1 / 1 | 1 / 1 | .161 / .163 | .033 / .033 |
| Close crossing, merged | **2 / 2** | **2 / 3** | **.636 / .969** | .890 / .920 | .177 / .172 | .089 / .079 |
| Wider crossing | 0 / 0 | 1 / 1 | 1 / 1 | .960 / .950 | .166 / .167 | .047 / .049 |
| Near crossing | 0 / 0 | 0 / 0 | 1 / 1 | 1 / 1 | .173 / .172 | .043 / .037 |
| Short occlusion | 0 / 0 | 1 / 1 | 1 / 1 | .957 / .957 | .163 / .174 | .053 / .054 |
| Three objects, one trial | 0 | 0 | 1 | 1 | .163 | .030 |
| Navigation, one trial | 0 | 0 | 1 | 1 | .159 | .030 |

Identity fragments are zero for the resolvable primary scenarios. The close
crossing records 2 / 1 geometric identity fragments; the second trial's one
is a temporary merged-frame assignment to the other GT, with final IDs restored.
The longer-than-timeout occlusion records one switch/fragment, as expected.
**Zero false confirmed tracks, zero false persistent tracks, zero duplicate
frames across all 22 runs.** No evidence justifies a range-dependent birth rule.

## Why association was not changed

The first close-crossing trial is the decisive failure timeline:

| Time after release | Evidence | Result |
|---|---|---|
| 2.877 s | two centroids, centre spacing .77 m | A=1, B=2 |
| 3.078 s | both objects visible; one connected cluster, spacing .67 m | merged centroid updates ID 1; ID 2 misses |
| 3.477 s | still one cluster, spacing .51 m | ID 1 moves between objects; geometric evaluator assigns it to B |
| 3.879 s | last ID 2 observation now 1.002 s old | ID 2 expires under the existing timeout |
| 4.077 s | two centroids briefly return | ID 1 follows B; A creates a tentative replacement |
| 4.278–4.879 s | A has fewer than three LiDAR returns | physical occlusion prevents consistent correction/confirmation |
| 5.278 s onward | two resolved fresh tracks | A=3, B=1: identity loss persists |

In the second trial, merging lasts four rather than five scans. ID 2 is
reacquired at 4.023 s after 0.999 s, and both original IDs are restored at the
end. Its two geometric switches occur when the single blended estimate is
scored as B and then returns to A; they are not a lasting two-track swap.

Classification: **A, perception/clustering merge**, followed by **D, occlusion**
and the configured **C, track expiry/reconfirmation** consequence. The two
resolved-cluster frames without a confirmed fresh A in the first trial occur
while its replacement is tentative. They are not evidence of greedy choosing
the wrong pair between two established, separately observed tracks.

An evaluation-only diagnostic compared greedy and global Euclidean assignment
on the **same recorded pre-decision states**, only where both existing tracks
were confirmed and two centroids existed: **523 baseline frames**, **733 frames
including regression/limit runs; zero differing decisions**. This is not a
counterfactual full tracker replay and does not establish general equivalence.
It does show no measured global-assignment advantage in these resolved cases.
For one merged centroid, global one-to-one assignment still has only one
measurement to distribute. No Hungarian/Mahalanobis/JPDA/MHT was implemented.

## Resolution and gap limits

At approximately 3–3.9 m viewing range, two cylinders moving in parallel:

| Centre separation | Surface separation | Merge frames / 30 | Two objects freshly matched |
|---|---:|---:|---:|
| .55 m | .15 m | 30 | 0 frames |
| .65 m | .25 m | 30 | 0 frames |
| .75 m | .35 m | 2 | 28 frames |
| .85 m | .45 m | 0 | 30 frames |

Thus **.85 m is the smallest tested reliably resolved spacing in this geometry**;
.75 m is marginal. This is not a universal hard distance threshold. During the
crossing, view-dependent exposed surfaces briefly resolved two centroids at
.486–.499 m while other frames merged up to .669 m. Separation alone cannot
predict angular occlusion or the clusterer's connectivity.

Short physical occlusion: B lost 3 scans, same ID returned after .801 s in both
trials. Wider crossing: A survived 4 / 5 missed scans; same ID returned after
.999 / 1.200 s. The latter exposes an existing boundary: **association precedes
pruning**, so a detection on the next scan may revive a still-stored track even
when its observation age has just exceeded 1 s. No expiry ordering was changed.

Long physical occlusion: B has <3 returns for 11 scans; ID 2 expires at 1.2 s
since its last observation and becomes ID 3 after reappearance and confirmation.
B's fresh matched fraction is .783; one expected identity fragment. The isolated
lifecycle unit test separately verifies five misses survive at exact 0.2 s
spacing, sixth miss expires at 1.2 s, four-miss reacquisition preserves the ID,
new observation after expiry gets a new ID, and misses preserve observation
count/stamp. These synthetic-centroid unit tests are explicitly **not** physical
occlusion/detection evidence. All primary gaps came from real Gazebo sensing.

## Prediction, costmap, navigation, static regression

**11,370 published prediction samples** passed per-parent checks of predicted
position, absolute timestamp and `F P Fᵀ + Q` position covariance, with zero
failures. The unit test additionally changes A's x/P and confirms B's future
state/covariance is unchanged. Equal covariances on similarly observed objects
are expected, not evidence of shared state.

The real Stage-4D plugin regression uses its ROS subscription and the actual
LayeredCostmap rebuild cycle, with two controlled valid prediction arrays and
Stage-4F policy parameters. It verifies cell-for-cell `both == max(A,B)`:
432 overlapping cells, 327 A-only cells, 336 B-only cells. Expiring A or removing
A produces exactly the B-only master grid; an empty array produces zero ghost
cells. This test injects messages only into its isolated layer-test topic; it
never replaces runtime LiDAR observations.

Focused navigation uses the frozen MPPI/Nav2 configuration, two observable
objects in side lanes, and a single goal (-3,0) → (3,0). Goal status **SUCCEEDED**,
controller lifecycle remains active, robot displacement 5.872 m; 110/110 fresh
matches per object, zero identity events, and both track predictions represented
in **111/111** sampled layer grids. Coverage checks transform predictions into
the grid frame and allow a .15 m neighbourhood for scan/layer scheduling offsets;
the isolated regression above is the exact composition/clearing check. This is
a continuity regression, not a difficult multi-obstacle avoidance benchmark.

Three-object scaling: 50/50 matches per object, IDs 1/2/3 throughout, no merging,
false tracks or duplicates. All three have independent prediction arrays.

Static navigation: **SUCCEEDED**, active controller, displacement 5.874 m,
95 scored scans, **zero active tracks (including tentative)** and **zero
predictive cells**. Stage-4G1 static rejection and parameters are byte-identical;
its full historical 300/300 dynamic and six-run static evidence remains valid.
G2 adds this focused moving-robot static check; it does not pretend to rerun the
entire G1 benchmark. The Stage-4F tag and source hashes are in
[provenance.json](stage4g2/provenance.json).

## Runtime

Measured by steady-clock timestamps in the actual callback, same build/machine,
with all marker publication enabled. `scan_us` includes TF lookup/wait and static
rejection. `association_us` includes candidate construction/sort, assignment,
KF updates, misses and births. Total callback includes pruning and message/marker
publication, but excludes emitting the final profiling log line itself.

| Objects | Scans | Scan/TF mean ms | Clustering mean ms | Association + KF mean ms | Callback mean / p95 / max ms |
|---|---:|---:|---:|---:|---:|
| 1 | 100 | 13.147 | .018 | .144 | 14.176 / 31.828 / 32.327 |
| 2 | 100 | 13.227 | .030 | .202 | 14.775 / 32.262 / 32.922 |
| 3 | 50 | 12.829 | .060 | .274 | 15.075 / 32.654 / 34.287 |

Clusters and active tracks are exactly 1/2/3 respectively in these timing
controls. Maximum association+KF time for three is .419 ms. The 34.287 ms
maximum callback is 17.1% of the 200 ms sensor period; TF wait dominates, not
N-target assignment. G1's ~0.25 ms static-rejection-only measurement is a
different timing scope and should not be compared directly with this total.
Full statistics: [performance.json](stage4g2/performance.json).

## Acceptance (user criteria 1–20)

| Criteria | Verdict / evidence |
|---|---|
| 1–2: independent obstacles, baseline first | PASS: independent Gazebo models; 14-run unchanged-greedy baseline preserved |
| 3–7: all required scenarios and physical gaps | PASS: table above; close crossing explicitly fails identity continuity under merge |
| 8–10: identity metrics, GT isolation, merge distinction | PASS: fresh/retained metrics, actual scan diagnostics, separate failure timelines |
| 11–12: evidence-driven change, stable resolvable IDs or documented ambiguity | PASS within measured envelope: no change justified; cluster/occlusion limit quantified |
| 13–14: independent predictions, safe multi-track costmap | PASS: 11,370 samples + lifecycle independence + exact real-layer regression |
| 15–16: three objects, runtime | PASS: three stable IDs, max callback 34.287 ms / 200 ms |
| 17–18: Nav2 and G1 static regression | PASS: both goals succeed, active lifecycle, static tracks/cells zero |
| 19–20: Stage-4F reference, UR5 workspace | PASS: tag unchanged; no accesses or writes to `~/ur5e_ws` |

## Exact reproduction

Use a fresh output directory: trial directories and CSV outputs are
exclusive-created to protect baseline evidence. Scripts are intentionally
repository-local; no installed Nav2 files are edited.

```bash
cd ~/predictive_nav_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

python3 -m unittest discover \
  -s src/predictive_nav_bringup/scripts/stage4g2 -p 'test_*.py'
colcon test --packages-select predictive_nav_tracking predictive_nav_costmap \
  --ctest-args -R '^stage4g2_' --event-handlers console_direct+

python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
  --out validation_logs/stage4g2_repro/baseline \
  --scenarios single,parallel,opposing,crossing,crossing_resolved,near_crossing,occlusion \
  --trials 2 --domain 100

python3 src/predictive_nav_bringup/scripts/stage4g2/run.py \
  --out validation_logs/stage4g2_repro/regressions \
  --scenarios triple,navigation,static_navigation,occlusion_long,resolution_55,resolution_65,resolution_75,resolution_85 \
  --trials 1 --domain 120

ROS_DOMAIN_ID=93 build/predictive_nav_costmap/stage4g2_multi_layer \
  > validation_logs/stage4g2_repro/costmap_regression.log 2>&1
ROS_DOMAIN_ID=94 build/predictive_nav_tracking/stage4g2_lifecycle \
  > validation_logs/stage4g2_repro/lifecycle_regression.log 2>&1
python3 src/predictive_nav_bringup/scripts/stage4g2/summarize.py \
  --baseline validation_logs/stage4g2_repro/baseline \
  --regressions validation_logs/stage4g2_repro/regressions \
  --out validation_logs/stage4g2_repro/curated
```

## Handoff / next-stage recommendation

Retain greedy association and all frozen tracker/Nav2 parameters. This test
supports N-target tracking in resolvable LiDAR geometry, not general identity
recovery from long merged clusters or long occlusions. Velocity estimates are
useful on resolved paths and degrade when blended centroids are treated as
individual observations; useful prediction cannot remove that upstream ambiguity.
The 0.45 m evaluation gate, approximate surface labels, one sensor viewpoint,
two repetitions, and identical cylinder/constant-velocity geometry bound these
claims. No range-dependent birth rule is warranted by the measured false-track
results.

Recommended Stage-4G3 first step: review the recorded merge/occlusion limitation
and decide the intended perception/track-management scope before adding downstream
prediction sophistication. Any major clustering or multi-hypothesis work requires
that separate review. No reachability, intent, neural, camera/YOLO, controller,
costmap redesign or Stage-4G3 implementation was started.
