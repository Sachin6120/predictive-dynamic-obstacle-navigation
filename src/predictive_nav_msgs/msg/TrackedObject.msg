# A single persistent dynamic-obstacle track, expressed in the map frame.

uint32 id                          # stable track identifier, unique for the lifetime of the tracker

geometry_msgs/Point position       # published position: Kalman-filtered once kalman_initialized,
                                    # otherwise identical to raw_position (x, y; z unused, set to 0)
geometry_msgs/Vector3 velocity     # published velocity: Kalman-filtered once kalman_initialized,
                                    # otherwise the Stage-4B raw finite-difference velocity (vz unused)
float64 speed                      # hypot(vx, vy) of the published velocity, convenience field

geometry_msgs/Point raw_position   # Stage-4B measurement: this scan's associated cluster centroid,
                                    # unfiltered (x, y; z unused, set to 0)
geometry_msgs/Vector3 raw_velocity # Stage-4B baseline: raw finite-difference velocity from the
                                    # short position history (vz unused). Always populated so
                                    # RAW vs KALMAN comparison remains possible on every track.

bool kalman_initialized            # true once the per-track constant-velocity Kalman filter has
                                    # been corrected by >= 2 observations (a real velocity estimate
                                    # exists); false for a just-created track (position-only, v=0)

# Row-major 4x4 Kalman state covariance P for state [px, py, vx, vy]^T.
# Meaningless (identity-scaled prior) until kalman_initialized is true.
float64[16] covariance

uint32 age                         # number of scan cycles since track creation
uint32 observations                # number of scans in which this track was successfully associated
uint32 missed_count                # consecutive scan cycles since the last successful association

builtin_interfaces/Time stamp      # timestamp of the most recent observation used to update this track

# Future trajectory prediction (Stage-4C): state propagated forward from the
# current filtered state without modifying it. Empty until kalman_initialized.
# UNCHANGED at Stage-4G4: semantics, anchoring and covariance are exactly as
# validated at Stage-4C/4D/4F, and this remains the sole statistical output.
TrackedObjectPrediction[] predictions

# Conservative bounded-motion reachable regions (Stage-4G4). ADDITIVE and
# OPTIONAL: empty when reachability_enabled is false, and empty until
# kalman_initialized, so every pre-Stage-4G4 consumer is unaffected. These are
# deterministic set bounds, not covariances -- see ReachabilityPrediction.msg.
# Note the different time anchor: these samples are relative to the CURRENT
# SCAN (TrackedObjectArray.header.stamp), whereas `predictions` above are
# relative to this track's `stamp` (its last real observation).
ReachabilityPrediction[] reachability_predictions

# --- Stage-5B: dynamic obstacle classification -----------------------------
# ADDITIVE and OBSERVATION-ONLY. These fields carry a semantic label for this
# track; nothing in the prediction, reachability or costmap path reads them,
# so every pre-Stage-5B consumer is bit-for-bit unaffected. Class-aware
# prediction is deliberately NOT part of this stage.
#
# The label is the TEMPORALLY STABILISED one -- the output of the per-track
# hysteresis described in obstacle_classification.hpp -- not this scan's raw
# guess, so it is safe to display or log directly. The raw per-scan guess is
# published alongside it for evaluation purposes only.
#
# classification uses the CLASS_* constants below. UNKNOWN is a real,
# expected answer: it is returned whenever the 2D scan geometry does not
# support a confident label (too few returns, too far away, track too young,
# or two class hypotheses fitting equally well). Consumers must handle it.
uint8 CLASS_UNKNOWN=0
uint8 CLASS_PEDESTRIAN=1
uint8 CLASS_BIKE=2
uint8 CLASS_VEHICLE=3

uint8 classification              # stable label, one of CLASS_* above
float64 classification_confidence # [0,1] smoothed evidence for `classification`;
                                  # exactly 0.0 when classification is CLASS_UNKNOWN

uint8 classification_instant      # this scan's unsmoothed label, CLASS_* (evaluation aid)
float64 classification_instant_confidence  # [0,1] for classification_instant

# Scan-plane geometry the label was derived from, for THIS scan's associated
# cluster (all zero, and classification_features_valid false, on a scan where
# the track was not associated and therefore coasted).
# See ClusterFeatures in obstacle_classification.hpp for exact definitions.
bool classification_features_valid
float64 cluster_length            # m, extent along the cluster's principal axis
float64 cluster_thickness         # m, extent along the perpendicular axis
float64 cluster_elongation        # dimensionless, length / max(thickness, floor)
uint32 cluster_point_count        # count of beam returns in the cluster
float64 cluster_range             # m, sensor origin to cluster centroid
float64 cluster_angular_span      # rad, angular extent subtended at the sensor
float64 cluster_fill_ratio        # dimensionless, returns / beams crossing the span
float64 cluster_max_gap           # m, largest gap between bearing-adjacent returns

# --- Stage-5B.1: compound-observation provenance ---------------------------
# ADDITIVE and PROVENANCE-ONLY. True when the cluster geometry above is a
# COMPOUND OBSERVATION: the pooled returns of two primary clusters that the
# Stage-5B.1 gap-aware stage judged to be two slices of ONE rigid object (the
# measured case is a bicycle, whose two wheel chords are split by the frozen
# 0.35 m clustering rule). False for an ordinary single-cluster observation
# and on any scan where the track coasted.
#
# Nothing reads this but logging and validation. It exists so that a recovered
# object is AUDITABLE: the validation set reports true and false compound
# associations from this flag directly, instead of inferring them from the
# resulting geometry.
bool classification_from_compound
