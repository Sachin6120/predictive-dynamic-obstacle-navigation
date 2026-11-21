// Stage-4G4: conservative bounded-motion reachable sets for 2D obstacle tracks.
//
// WHAT THIS IS
// ------------
// A deterministic OUTER BOUND on where a tracked obstacle can physically be at
// a future time, given (a) its last filtered constant-velocity state and
// (b) configured kinematic bounds on acceleration and speed. It is an
// alternative future-motion REPRESENTATION that sits alongside -- never
// replaces -- the validated Stage-4C constant-velocity Gaussian prediction.
//
// WHAT THIS IS NOT
// ----------------
// It is not a probability distribution, not a covariance, and not a sigma
// level. `reachRadius()` below carries no statistical interpretation
// whatsoever. Where a statistical quantity IS combined in (the Stage-4C
// propagated position covariance, at a caller-chosen sigma level), it is kept
// as a separately named, separately published term -- see ReachabilityRegion.
//
// MATHEMATICS
// -----------
// Let the last real observation give filtered position p0 and velocity v0,
// with s0 = |v0|. Let t be the total elapsed time since that observation
// (observation age + requested horizon -- a coasting track pays for its age).
// Any physically admissible trajectory p(.) with p(0) = p0, p'(0) = v0
// satisfies, for acceleration a(.) and velocity v(.),
//
//   d(t) := p(t) - (p0 + v0 t) = int_0^t int_0^s a(u) du ds
//                              = int_0^t ( v(s) - v0 ) ds
//
// Two bounds apply simultaneously to the integrand:
//   |v(s) - v0| <= a_max * s            (bounded acceleration)
//   |v(s) - v0| <= |v(s)| + |v0|
//                <= v_max + s0          (bounded speed; the obstacle may
//                                        reverse, hence the SUM, not the
//                                        difference)
// so with the crossover time t_c = (v_max + s0) / a_max,
//
//   |d(t)| <= int_0^t min(a_max * s, v_max + s0) ds
//           = { 0.5 * a_max * t^2                                  t <= t_c
//             { 0.5 * a_max * t_c^2 + (v_max + s0) * (t - t_c)     t >  t_c
//
// This is exact as a bound on the MAGNITUDE of the deviation and is isotropic:
// it ignores the coupling between direction and achievable speed, so the disc
// of radius r_det(t) about the constant-velocity nominal is a strict superset
// of the true reachable set, never a subset. Being an over-bound is the
// intended direction of error for a safety envelope; the price is measured
// (region size) rather than assumed.
//
// The speed cap can only ever SHRINK the bound relative to pure bounded
// acceleration, so a region that violates max_speed cannot be produced.
//
// COMBINATION WITH THE STATISTICAL TERM
// -------------------------------------
// The caller supplies the Stage-4C propagated 2x2 position covariance at the
// same total time. Its sigma-level ellipse has semi-axes k*sqrt(lambda_1,2)
// oriented along the eigenvectors. The published region is the Minkowski
// OUTER bound of that ellipse and the deterministic disc (plus an optional
// fixed margin):
//
//   semi_major = k*sqrt(lambda_max) + r_det + margin
//   semi_minor = k*sqrt(lambda_min) + r_det + margin
//
// The true Minkowski sum of an ellipse and a disc is not an ellipse (it is an
// offset curve); inflating both semi-axes by the disc radius contains it, so
// this too errs conservatively. Each term stays individually available.
//
// VALIDITY CONTRACT (Stage-5C.1)
// ------------------------------
// The bound above is only a bound while its premises hold. Written out:
//
//   (P1)  |a(u)| <= max_acceleration   for all u in [0, t]
//   (P2)  |v(s)| <= max_speed          for all s in [0, t]
//
// (P2) is a statement about the obstacle's FUTURE SPEED CAPABILITY over the
// whole prediction window -- not about its observed velocity, and not a tuning
// weight. Stage-5C measured what happens when it is false: with s0 = 0,
// configured max_speed = 0.8 m/s and a true capability of 1.4 m/s, the model
// reaches 2.56 m at t = 4.0 s while the obstacle reaches 3.64 m, a 1.08 m
// escape INSIDE the production 4.0 s total-time window (worst tested escape
// 1.44 m). max_speed is therefore a validity premise, not a knob.
//
// WHAT RUNTIME CAN AND CANNOT CHECK. (P2) quantifies over the future, so no
// runtime check can VERIFY it -- an object's unknown future capability is not
// observable, and this code does not pretend otherwise. One direction is
// observable, though: if the filtered speed is s0 > max_speed, the obstacle is
// demonstrably moving at s0 and is therefore demonstrably CAPABLE of s0, so
//
//     s0 > max_speed  =>  v_phys >= s0 > max_speed  =>  (P2) is FALSE.
//
// An observed s0 above the cap is thus a PROOF that the premise is broken,
// and that is exactly what `contractStatus()` below reports. The converse
// case -- v_phys > max_speed while s0 <= max_speed -- remains unobservable and
// stays a declared configuration assumption; see the max_speed comment in
// lidar_obstacle_tracker_node.cpp and in the tracker YAML.
//
// RESPONSE: WEAKEST OBSERVATION-CONSISTENT PREMISE. When the premise is
// disproved, the region must not silently keep claiming a guarantee it no
// longer has, and it must not be dropped either -- dropping would REMOVE
// coverage, which is the wrong direction for a safety envelope. Instead the
// disproved premise is replaced by the weakest premise observation does not
// contradict,
//
//     v_eff = max(max_speed, s0),
//
// which restores a valid bound under (P2') |v(s)| <= v_eff. Because
// v_eff >= max_speed always, this is MONOTONE-SAFE: it can only ever enlarge
// the region, never shrink it. That matters because the Stage-5C recordings
// show the Kalman filtered speed transiently overshooting true speed by up to
// 0.25 m/s, exceeding the 0.8 m/s cap by up to 0.131 m/s on tracks that are
// genuinely IN domain (A_constant_velocity_fast: ground truth 0.680 m/s,
// filtered peak 0.931 m/s). A rejection would fire on those frames; a
// dead-band wide enough to absorb them (>= 0.15 m/s) would mask the
// s0 in [0.80, 0.95] band where Stage-5C's adversary already measured escapes.
// Enlarging the region costs nothing and removes nothing, so the dead-band can
// stay at NUMERIC scale (1e-6 m/s) rather than being widened to swallow
// estimator transients.
//
// ONE DEAD-BAND, BOTH PATHS. `max_speed_tolerance` gates the reported status
// and the effective cap together, so the two can never disagree. That makes
// the following an exact theorem rather than an approximation:
//
//     contract_status == kSatisfied  <=>  v_eff is bit-identical to max_speed
//                                   <=>  every number produced here is
//                                        bit-identical to Stage-5C.
//
// The concession is explicit and bounded: an excursion inside the dead-band is
// treated as numerically equal to the cap, i.e. at most 1e-6 m/s * 4.0 s =
// 4e-6 m of under-coverage across the whole production window.
#ifndef PREDICTIVE_NAV_TRACKING__REACHABILITY_HPP_
#define PREDICTIVE_NAV_TRACKING__REACHABILITY_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Eigen/Dense>

namespace predictive_nav_tracking
{

/// Kinematic bounds describing what the obstacle class is permitted to do.
/// Both are physical quantities, chosen from the obstacle's motion capability,
/// never from a desired navigation outcome.
struct ReachabilityBounds
{
  double max_acceleration{0.5};   // m/s^2, isotropic |a| bound
  double max_speed{0.8};          // m/s, |v| bound -- a FUTURE CAPABILITY
                                  // premise, see the contract note above
  double covariance_sigma_level{2.0};
  double safety_margin{0.0};      // m, deterministic additive body allowance

  // --- Stage-5C.1 validity contract --------------------------------------
  /// Numeric dead-band on the observable contradiction s0 > max_speed: an
  /// excursion no larger than this is treated as numerically equal to the cap,
  /// so float-level jitter at exactly the cap is neither announced as a
  /// contract breach nor allowed to perturb the region. Deliberately
  /// NUMERIC-scale, not wide enough to absorb estimator transients -- the
  /// Stage-5C recordings show the filtered speed exceeding the 0.8 m/s cap by
  /// up to 0.131 m/s on tracks that are genuinely in domain, while Stage-5C's
  /// adversary measured real escapes for s0 in [0.80, 0.95]; a dead-band wide
  /// enough for the former would mask the latter. Transients are instead made
  /// harmless by the response being monotone-safe. At the default, the
  /// under-coverage conceded is at most 1e-6 m/s * 4.0 s = 4e-6 m. Set to 0 to
  /// act on every strict excursion.
  double max_speed_tolerance{1e-6};   // m/s
  /// When true (default), a disproved capability premise is replaced by the
  /// weakest observation-consistent one, v_eff = max(max_speed, s0). Setting
  /// it false restores the pre-Stage-5C.1 arithmetic exactly, which is how the
  /// regression test reproduces the old behaviour; it is not a production
  /// setting, because it lets a disproved bound keep being published as if it
  /// held.
  bool enforce_observed_speed{true};
};

/// Outcome of the Stage-5C.1 validity contract for one sample. Values match
/// the CONTRACT_* constants in ReachabilityPrediction.msg.
enum class ContractStatus : uint8_t
{
  /// Nothing observed contradicts the capability premise. This is NOT a proof
  /// that the premise holds -- future capability is unobservable.
  kSatisfied = 0,
  /// The filtered speed exceeds the configured cap, which DISPROVES the
  /// premise. `effective_max_speed` was raised to the observed speed.
  kObservedSpeedExceedsCap = 1,
  /// The contract was not evaluated: the sample is not a usable region
  /// (non-finite state, or the covariance eigen-decomposition failed).
  kNotEvaluated = 2,
};

/// True when the observed speed PROVES the capability premise false, outside
/// the numeric dead-band. See the contract note at the head of this file for
/// why this is the only observable part of the premise.
///
/// A NON-FINITE speed is not evidence about the obstacle, so it is NOT a
/// contradiction: such a sample is rejected by buildRegion() as
/// kNotEvaluated instead, which is the honest answer -- "unusable", not
/// "breached".
inline bool capabilityPremiseContradicted(
  double observed_speed, const ReachabilityBounds & bounds)
{
  if (!std::isfinite(observed_speed)) {
    return false;
  }
  const double tol = std::max(bounds.max_speed_tolerance, 0.0);
  return observed_speed > std::max(bounds.max_speed, 0.0) + tol;
}

/// The weakest speed cap that observation does not contradict: the configured
/// cap unless the premise is actually disproved, and the observed speed when it
/// is. Always >= bounds.max_speed, so substituting it can only enlarge the
/// resulting region.
///
/// It is gated on capabilityPremiseContradicted() -- not on a bare
/// `observed_speed > max_speed` -- so that the safety path and the reported
/// status can never disagree. That makes one exact invariant hold:
///
///     contract_status == kSatisfied  <=>  effective cap IS bit-identical to
///                                         the configured cap
///                                    <=>  every published number is
///                                         bit-identical to Stage-5C.
///
/// The price is explicit and bounded: an excursion inside the dead-band is
/// treated as numerically equal to the cap, which at the default 1e-6 m/s over
/// the 4.0 s production window is at most 4e-6 m of under-coverage.
inline double effectiveMaxSpeed(double observed_speed, const ReachabilityBounds & bounds)
{
  const double v_max = std::max(bounds.max_speed, 0.0);
  if (!bounds.enforce_observed_speed) {
    return v_max;
  }
  if (!capabilityPremiseContradicted(observed_speed, bounds)) {
    return v_max;
  }
  return std::max(v_max, observed_speed);
}

/// Reported contract status for an observed speed.
inline ContractStatus contractStatus(double observed_speed, const ReachabilityBounds & bounds)
{
  return capabilityPremiseContradicted(observed_speed, bounds) ?
         ContractStatus::kObservedSpeedExceedsCap : ContractStatus::kSatisfied;
}

/// One published reachable region. The three radius contributions are kept
/// apart on purpose so that a deterministic bound is never reported as, or
/// mistaken for, a covariance.
struct ReachabilityRegion
{
  double total_time{0.0};
  double center_x{0.0};
  double center_y{0.0};
  double vx{0.0};
  double vy{0.0};

  double reach_radius{0.0};        // DETERMINISTIC bounded-motion term
  bool speed_capped{false};

  double sigma_semi_major{0.0};    // STATISTICAL covariance term
  double sigma_semi_minor{0.0};
  double sigma_yaw{0.0};

  double safety_margin{0.0};       // DETERMINISTIC additive margin

  double semi_major{0.0};          // final region = sum of the three above
  double semi_minor{0.0};
  bool valid{false};

  // --- Stage-5C.1 validity contract metadata -----------------------------
  /// |v0| of the track at its last real observation: the OBSERVED speed the
  /// contract is evaluated against.
  double observed_speed{0.0};
  /// The configured capability premise, verbatim from the bounds.
  double configured_max_speed{0.0};
  /// The cap the radius was actually computed with. Equals
  /// configured_max_speed in domain; raised to observed_speed when observation
  /// disproved the premise.
  double effective_max_speed{0.0};
  /// Single source of truth for "is this a qualified conservative bound".
  /// A separate duplicate boolean is deliberately NOT carried: two encodings
  /// of one fact can disagree. Test `contract_status == kSatisfied`.
  ContractStatus contract_status{ContractStatus::kNotEvaluated};
};

/// Deterministic bounded-motion radius r_det(t) derived above.
/// `initial_speed` is |v0| of the track at its last real observation.
inline double reachRadius(
  double t, double initial_speed, const ReachabilityBounds & bounds, bool * speed_capped = nullptr)
{
  if (speed_capped != nullptr) {
    *speed_capped = false;
  }
  if (!(t > 0.0) || !std::isfinite(t)) {
    return 0.0;
  }
  const double a_max = std::max(bounds.max_acceleration, 0.0);
  // Stage-5C.1: the cap used here is the weakest one observation does not
  // contradict. It equals bounds.max_speed whenever s0 <= max_speed, so every
  // in-domain radius is bit-identical to Stage-5C.
  const double v_max = effectiveMaxSpeed(initial_speed, bounds);
  const double s0 = std::max(initial_speed, 0.0);
  if (a_max <= 0.0) {
    return 0.0;
  }

  const double delta_v_cap = v_max + s0;
  const double t_c = delta_v_cap / a_max;
  if (t <= t_c) {
    return 0.5 * a_max * t * t;
  }
  if (speed_capped != nullptr) {
    *speed_capped = true;
  }
  return 0.5 * a_max * t_c * t_c + delta_v_cap * (t - t_c);
}

/// Builds the region for one horizon sample.
///
/// p0/v0 are the filtered state at the track's LAST REAL OBSERVATION, and
/// `total_time` is observation age + horizon, so a coasting track is expanded
/// for the time it has not been seen rather than being treated as fresh.
/// `position_covariance` is the Stage-4C propagated 2x2 position block at the
/// same total_time; it is used only for the separately reported statistical
/// term.
inline ReachabilityRegion buildRegion(
  double p0x, double p0y, double v0x, double v0y,
  const Eigen::Matrix2d & position_covariance,
  double total_time, const ReachabilityBounds & bounds)
{
  ReachabilityRegion region;
  region.total_time = total_time;
  region.vx = v0x;
  region.vy = v0y;
  region.safety_margin = std::max(bounds.safety_margin, 0.0);
  region.configured_max_speed = std::max(bounds.max_speed, 0.0);

  if (!std::isfinite(p0x) || !std::isfinite(p0y) || !std::isfinite(v0x) ||
    !std::isfinite(v0y) || !std::isfinite(total_time) || total_time < 0.0)
  {
    // Contract stays kNotEvaluated: there is no usable region to qualify.
    return region;
  }

  region.center_x = p0x + v0x * total_time;
  region.center_y = p0y + v0y * total_time;

  // Stage-5C.1: evaluate the validity contract against the OBSERVED speed
  // before the radius is built, and record which cap the radius then used.
  const double s0 = std::hypot(v0x, v0y);
  region.observed_speed = s0;
  region.effective_max_speed = effectiveMaxSpeed(s0, bounds);
  region.contract_status = contractStatus(s0, bounds);

  bool capped = false;
  region.reach_radius = reachRadius(total_time, s0, bounds, &capped);
  region.speed_capped = capped;

  // Statistical term: sigma-level ellipse of the propagated Kalman covariance.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(position_covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
    region.contract_status = ContractStatus::kNotEvaluated;
    return region;
  }
  const Eigen::Vector2d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
  const Eigen::Matrix2d eigenvectors = solver.eigenvectors();
  const double k = std::max(bounds.covariance_sigma_level, 0.0);
  region.sigma_semi_major = k * std::sqrt(eigenvalues(1));
  region.sigma_semi_minor = k * std::sqrt(eigenvalues(0));
  region.sigma_yaw = std::atan2(eigenvectors(1, 1), eigenvectors(0, 1));

  region.semi_major = region.sigma_semi_major + region.reach_radius + region.safety_margin;
  region.semi_minor = region.sigma_semi_minor + region.reach_radius + region.safety_margin;
  region.valid = std::isfinite(region.semi_major) && std::isfinite(region.semi_minor) &&
    std::isfinite(region.center_x) && std::isfinite(region.center_y);
  if (!region.valid) {
    region.contract_status = ContractStatus::kNotEvaluated;
  }
  return region;
}

/// True when `point` lies inside the region's ellipse. Shared by the tracker
/// tests, the costmap layer and the offline evaluator so that "inside" means
/// exactly one thing everywhere.
inline bool regionContains(const ReachabilityRegion & region, double x, double y)
{
  if (!region.valid || region.semi_major <= 0.0 || region.semi_minor <= 0.0) {
    return false;
  }
  const double dx = x - region.center_x;
  const double dy = y - region.center_y;
  const double c = std::cos(region.sigma_yaw);
  const double s = std::sin(region.sigma_yaw);
  const double u = (dx * c + dy * s) / region.semi_major;
  const double v = (-dx * s + dy * c) / region.semi_minor;
  return u * u + v * v <= 1.0;
}

}  // namespace predictive_nav_tracking

#endif  // PREDICTIVE_NAV_TRACKING__REACHABILITY_HPP_
