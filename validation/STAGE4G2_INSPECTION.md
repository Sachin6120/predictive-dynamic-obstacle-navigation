# Stage-4G2 pre-change inspection

Initial tree clean; branch `main`, HEAD `eab2218765a19474d15daa26ab7cf7ac64631ba4`.
Annotated `stage4f-validated` peels to `68b2bf2ee1efb774ba563438a4c621b0052f61d5`.
No repository AGENTS.md is present. No access to `~/ur5e_ws` is needed.

## Frozen baseline

`associate_and_update` predicts each track using `F(clamp(scan_stamp -
last_update, .001, 1.0)) * kf_x`. It enumerates ALL track/centroid pairs,
retains Euclidean distances <= 0.6 m, sorts ascending, and accepts a pair
only if both endpoints remain unassigned. Equal-distance ordering is not
specified by the comparator. It is globally sorted greedy edge selection,
not independent nearest-neighbour lookup per track and not global minimum
total cost. Covariance is not used in matching. Velocity enters through F.

Each accepted match predicts and corrects that track's KF; increments age
and total observations; resets missed_count; updates last_update and its
five-measurement raw-velocity history. Each unmatched track increments age
and consecutive missed_count without mutating state/covariance/last_update.
Every unmatched centroid creates a tentative zero-velocity track with a new
monotonic ID. Pruning runs AFTER association: delete when missed_count > 5
OR observation age > 1.0 s (strict inequalities). Thus a returning detection
can match before expiry pruning; this boundary must be reported honestly.

Publication requires >= 3 TOTAL observations, not consecutive observations.
KF velocity is enabled at >= 2 observations. R = .05^2 I; initial position
variance .01, velocity variance 4; process acceleration parameter .5.
The implemented per-axis Q is .5^2 [[dt^4/4, dt^3/2],[dt^3/2,dt^2]].
Despite the header's CWNA label, this is the discrete random-acceleration Q.
No model change is planned in G2.

## Structure and time semantics

Tracks are a vector; candidate/assignment arrays depend on N tracks and M
centroids. No two-object limit exists. Every Track owns its x, P, history,
counters and timestamp. TrackedObjectArray carries a variable-length array;
each object owns six future samples (.5 to 3 s) with covariance. Predictions
independently apply F and Q to the same parent state; no shared mutable KF.

Array header is the scan stamp. Object stamp is LAST OBSERVATION, and its
position/raw_position/covariance remain at that stamp through misses.
Prediction stamps are object stamp + horizon, not array stamp + horizon.
Evaluation must separate fresh observations (missed_count == 0) from retained
state. It may propagate retained state to scoring time explicitly, but may
not label this a detection.

The costmap loops over all objects, checks array and object expiry, transforms
each mean/covariance, and rebuilds its layer each cycle with previous bounds
included for clearing. Contributions combine with maximum cost. Multi-object
runtime regressions are still required; inspection alone is not a PASS.

No association, birth, clustering, lifetime, prediction, costmap or controller
change is justified by inspection. First measure the existing pipeline.
