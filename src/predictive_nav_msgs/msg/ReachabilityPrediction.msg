# Stage-4G4: a conservative BOUNDED-MOTION reachable region for one
# TrackedObject at one future horizon.
#
# This message is NOT a probability distribution and NOT a covariance. It is
# an outer bound on the set of positions the obstacle can physically occupy,
# given its last filtered state and configured bounds on acceleration and
# speed. The statistical (covariance-derived) and deterministic (reachability)
# contributions are published SEPARATELY and are individually labelled below,
# so a consumer can never confuse the two. The legacy Stage-4C
# TrackedObjectPrediction message and its position_covariance field are
# unchanged and remain the sole statistical prediction output.
#
# TIME ANCHOR -- DELIBERATELY DIFFERENT FROM TrackedObjectPrediction.
#   TrackedObjectPrediction.time_from_now is relative to the parent
#   TrackedObject.stamp, i.e. to the last real observation (Stage-4C
#   semantics, preserved).
#   ReachabilityPrediction.time_from_now is relative to the CURRENT SCAN,
#   i.e. to TrackedObjectArray.header.stamp. A coasting track's region is
#   therefore propagated through the elapsed observation age as well, which
#   is the whole point: a stale track must not be presented as a fresh one.
#   observation_age and total_time below make the anchoring explicit, and the
#   absolute `stamp` field resolves it unambiguously either way.

float64 time_from_now              # s after TrackedObjectArray.header.stamp (the scan), >= 0
builtin_interfaces/Time stamp      # absolute predicted time = scan stamp + time_from_now

float64 observation_age            # s between the parent track's last real observation
                                   # (TrackedObject.stamp) and the scan stamp. 0.0 when the
                                   # track was associated on this scan; grows while coasting.
float64 total_time                 # observation_age + time_from_now: total elapsed time since
                                   # the last real measurement. ALL bounds below use this.

# Nominal centre: p0 + v0 * total_time, where (p0, v0) is the filtered state at
# the last real observation. Same constant-velocity nominal as the Stage-4C
# predictor -- reachability adds a bound around it, it does not add a new
# motion model.
geometry_msgs/Point position       # nominal centre in the map frame (z unused, set to 0)
geometry_msgs/Vector3 velocity     # v0 actually used for this sample (vz unused)

# --- DETERMINISTIC bounded-motion term -------------------------------------
# r_det(t) is an outer bound on |p(t) - (p0 + v0 t)| for any trajectory with
# |a| <= max_acceleration and |v| <= max_speed. Purely kinematic: it carries no
# probability, no sigma level, and no statistical interpretation.
float64 reach_radius               # m, r_det(total_time)
bool speed_capped                  # true when the speed cap bound r_det for this sample, i.e.
                                   # total_time exceeded (effective_max_speed + |v0|)/
                                   # max_acceleration. Stage-5C.1: the cap referred to is
                                   # effective_max_speed below, which equals configured_max_speed
                                   # in domain. On an out-of-contract sample the effective cap is
                                   # larger, so the crossover time is later and this flag may read
                                   # false where the configured cap alone would have bound it.

# --- STATISTICAL term ------------------------------------------------------
# The covariance_sigma_level ellipse of the Stage-4C propagated Kalman position
# covariance F P F^T + Q at total_time. This IS a covariance-derived quantity.
float64 sigma_semi_major           # m, sigma_level * sqrt(largest eigenvalue)
float64 sigma_semi_minor           # m, sigma_level * sqrt(smallest eigenvalue)
float64 sigma_yaw                  # rad, map-frame orientation of the major axis
float64 covariance_sigma_level     # the k that produced the two axes above

# --- DETERMINISTIC additive margin -----------------------------------------
# Fixed physical allowance (e.g. obstacle radius / sensor surface-vs-centre
# bias). Zero by default so the published region measures the motion model
# alone; a consumer that needs a body allowance sets it explicitly.
float64 safety_margin              # m

# --- Final published region ------------------------------------------------
# Minkowski OUTER bound of the statistical ellipse and the deterministic disc:
#   semi_major = sigma_semi_major + reach_radius + safety_margin
#   semi_minor = sigma_semi_minor + reach_radius + safety_margin
# oriented at sigma_yaw. Reconstructible exactly from the fields above.
float64 semi_major                 # m
float64 semi_minor                 # m

bool valid                         # false when total_time exceeded max_observation_age or the
                                   # state was not finite; consumers must ignore invalid samples

# --- Stage-5C.1: VALIDITY CONTRACT -----------------------------------------
# The deterministic bound above is only a bound while its premises hold:
#   (P1) |a(u)| <= max_acceleration   for all u in [0, total_time]
#   (P2) |v(s)| <= max_speed          for all s in [0, total_time]
# (P2) is a FUTURE CAPABILITY premise, not a statement about observed velocity.
# Stage-5C measured the cost of it being false: with observed_speed 0, a
# configured cap of 0.8 m/s and a true capability of 1.4 m/s, the model reaches
# 2.56 m at total_time 4.0 s while the obstacle reaches 3.64 m -- a 1.08 m
# escape inside the production 4.0 s window.
#
# (P2) quantifies over the future, so NOTHING here verifies it; an object's
# unknown future capability is not observable and this interface does not claim
# otherwise. One direction IS observable: observed_speed > configured_max_speed
# proves the obstacle is capable of observed_speed, hence DISPROVES (P2). That
# is what contract_status reports.
#
# When the premise is disproved the region is neither dropped (that would
# remove coverage) nor left silently claiming a guarantee it lost. The
# disproved premise is replaced by the weakest premise observation does not
# contradict, effective_max_speed = max(configured_max_speed, observed_speed),
# which can only ENLARGE the region. In domain the two caps are equal and every
# field above is bit-identical to Stage-5C.

float64 observed_speed             # m/s, |v0| of the track at its last real observation:
                                   # the quantity the contract is evaluated against
float64 configured_max_speed       # m/s, the configured capability premise, verbatim
float64 effective_max_speed        # m/s, the cap reach_radius was actually computed with.
                                   # == configured_max_speed in domain; raised to
                                   # observed_speed when observation disproved the premise

uint8 CONTRACT_SATISFIED=0                  # nothing observed contradicts the premise.
                                            # NOT a proof that it holds: future capability
                                            # is unobservable. The residual risk --
                                            # true capability above the cap while
                                            # observed_speed stays below it -- remains a
                                            # declared configuration assumption.
uint8 CONTRACT_OBSERVED_SPEED_EXCEEDS_CAP=1 # observed_speed proves (P2) false;
                                            # effective_max_speed was raised accordingly
uint8 CONTRACT_NOT_EVALUATED=2              # not a usable region (non-finite state, or the
                                            # covariance decomposition failed); `valid` is
                                            # false and the sample must be ignored

# A consumer must NOT present a sample as a qualified conservative reachability
# bound unless contract_status == CONTRACT_SATISFIED. Deliberately the single
# encoding of that fact: no duplicate boolean is carried, because two encodings
# of one fact can disagree.
uint8 contract_status
