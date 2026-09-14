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
bool speed_capped                  # true when the max_speed cap bound r_det for this sample,
                                   # i.e. total_time exceeded (max_speed + |v0|)/max_acceleration

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
