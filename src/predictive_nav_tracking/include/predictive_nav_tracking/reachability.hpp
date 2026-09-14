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
#ifndef PREDICTIVE_NAV_TRACKING__REACHABILITY_HPP_
#define PREDICTIVE_NAV_TRACKING__REACHABILITY_HPP_

#include <algorithm>
#include <cmath>

#include <Eigen/Dense>

namespace predictive_nav_tracking
{

/// Kinematic bounds describing what the obstacle class is permitted to do.
/// Both are physical quantities, chosen from the obstacle's motion capability,
/// never from a desired navigation outcome.
struct ReachabilityBounds
{
  double max_acceleration{0.5};   // m/s^2, isotropic |a| bound
  double max_speed{0.8};          // m/s, |v| bound
  double covariance_sigma_level{2.0};
  double safety_margin{0.0};      // m, deterministic additive body allowance
};

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
  const double v_max = std::max(bounds.max_speed, 0.0);
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

  if (!std::isfinite(p0x) || !std::isfinite(p0y) || !std::isfinite(v0x) ||
    !std::isfinite(v0y) || !std::isfinite(total_time) || total_time < 0.0)
  {
    return region;
  }

  region.center_x = p0x + v0x * total_time;
  region.center_y = p0y + v0y * total_time;

  bool capped = false;
  region.reach_radius = reachRadius(total_time, std::hypot(v0x, v0y), bounds, &capped);
  region.speed_capped = capped;

  // Statistical term: sigma-level ellipse of the propagated Kalman covariance.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(position_covariance);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) {
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
