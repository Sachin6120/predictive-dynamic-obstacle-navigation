// Stage-5C -- qualification audit of the EXISTING Stage-4G4 reachability
// predictor. ANALYSIS ONLY: this file includes the shipped production headers
// and does not modify them, so every number below describes the shipped
// mathematics rather than a re-implementation of it.
//
// Build (no CMake/production build file is touched):
//   g++ -O2 -std=c++17 -I/usr/include/eigen3 \
//       -Isrc/predictive_nav_tracking/include \
//       validation/stage5c/stage5c_audit.cpp -o <out>/stage5c_audit
//
// Four parts:
//   1. independent reconstruction + edge-case audit of r_det(t)
//   2. adversarial containment, INCLUDING the s0 > max_speed regime that the
//      existing stage4g4_reachability test never samples
//   3. identical-input CV-vs-reachability comparison: one Kalman filter state
//      feeds both predictors, so the inputs are identical by construction
//   4. painted-area comparison replicating BOTH layer clamp policies exactly,
//      plus per-call cost
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "predictive_nav_tracking/kalman_filter.hpp"
#include "predictive_nav_tracking/reachability.hpp"

using predictive_nav_tracking::ConstantVelocityKalmanFilter2D;
using predictive_nav_tracking::ReachabilityBounds;
using predictive_nav_tracking::ReachabilityRegion;
using predictive_nav_tracking::buildRegion;
using predictive_nav_tracking::reachRadius;
using predictive_nav_tracking::regionContains;

namespace
{

// ---- frozen production parameters (read from production_tracker_params.yaml
//      and production_nav2_params.yaml; NOT retuned here) -------------------
constexpr double kScanPeriod = 0.2;            // s, 5 Hz LiDAR
constexpr double kAccelNoise = 0.5;            // kf_process_accel_noise
constexpr double kMeasNoise = 0.05;            // kf_measurement_noise
constexpr double kInitPosVar = 0.01;           // kf_initial_position_variance
constexpr double kInitVelVar = 4.0;            // kf_initial_velocity_variance
constexpr double kTimeStep = 0.5;              // reachability/prediction_time_step
constexpr int kNumHorizons = 6;                // horizon 3.0 s / 0.5 s
constexpr double kTrackerSigma = 2.0;          // uncertainty_ellipse_sigma (message)
constexpr double kLayerSigmaProd = 1.5;        // layer sigma_level (production)
constexpr double kMaxInfluenceProd = 0.55;     // layer max_influence_radius (production)
constexpr double kCostmapRes = 0.05;           // costmap resolution
constexpr double kMinPosVariance = 1e-6;       // layer min_position_variance guard

ReachabilityBounds productionBounds()
{
  ReachabilityBounds b;
  b.max_acceleration = 0.5;
  b.max_speed = 0.8;
  b.covariance_sigma_level = kTrackerSigma;
  b.safety_margin = 0.0;
  return b;
}

// =====================================================================
// PART 1 helpers -- independent reconstruction of r_det
// =====================================================================

// Numerically integrates int_0^t min(a_max*s, v_max+s0) ds with no knowledge
// of the closed form, so agreement is genuine corroboration of the algebra.
double reachRadiusNumeric(double t, double s0, const ReachabilityBounds & b, int steps = 2000000)
{
  if (!(t > 0.0)) {
    return 0.0;
  }
  const double h = t / steps;
  double acc = 0.0;
  for (int i = 0; i < steps; ++i) {
    const double s = (i + 0.5) * h;                       // midpoint rule
    acc += std::min(b.max_acceleration * s, b.max_speed + s0) * h;
  }
  return acc;
}

// =====================================================================
// PART 2 helpers -- the adversary
// =====================================================================

// Forward-simulates an admissible trajectory and returns |deviation| from the
// constant-velocity nominal at time t. `enforce_cap` lets us run the adversary
// both WITH the model's speed cap (legal) and WITHOUT it (the case of a track
// whose filtered speed already exceeds max_speed).
double simulatedDeviation(
  double v0x, double v0y, double ax, double ay, double t,
  const ReachabilityBounds & b, bool enforce_cap, double dt = 1e-5)
{
  double px = 0.0, py = 0.0, vx = v0x, vy = v0y;
  const double a_norm = std::hypot(ax, ay);
  if (a_norm > b.max_acceleration) {
    ax *= b.max_acceleration / a_norm;
    ay *= b.max_acceleration / a_norm;
  }
  for (double s = 0.0; s < t; s += dt) {
    const double step = std::min(dt, t - s);
    const double vxb = vx, vyb = vy;
    vx += ax * step;
    vy += ay * step;
    if (enforce_cap) {
      const double sp = std::hypot(vx, vy);
      if (sp > b.max_speed) {
        vx *= b.max_speed / sp;
        vy *= b.max_speed / sp;
      }
    }
    px += 0.5 * (vxb + vx) * step;                        // midpoint update
    py += 0.5 * (vyb + vy) * step;
  }
  return std::hypot(px - v0x * t, py - v0y * t);
}

// =====================================================================
// PART 3 -- scenario ground truth
// =====================================================================

struct GroundTruth
{
  double x, y, vx, vy;
};

struct Scenario
{
  std::string id;
  std::string family;                 // A..G
  bool in_model;                      // obstacle respects |a|<=a_max, |v|<=v_max
  double duration;
  double dropout_start{-1.0};         // withhold measurements in [start, end)
  double dropout_end{-1.0};
  std::function<GroundTruth(double)> gt;
};

// piecewise-constant-acceleration straight motion (ramped == in model)
std::function<GroundTruth(double)> rampMotion(
  double x0, double y0, double vx0, double vy0, double ax, double ay,
  double ramp_start, double ramp_end)
{
  return [=](double t) {
    double x = x0, y = y0, vx = vx0, vy = vy0;
    const double t1 = std::min(t, ramp_start);
    x += vx * t1;
    y += vy * t1;
    if (t > ramp_start) {
      const double d = std::min(t, ramp_end) - ramp_start;
      x += vx * d + 0.5 * ax * d * d;
      y += vy * d + 0.5 * ay * d * d;
      vx += ax * d;
      vy += ay * d;
      if (t > ramp_end) {
        const double d2 = t - ramp_end;
        x += vx * d2;
        y += vy * d2;
      }
    }
    return GroundTruth{x, y, vx, vy};
  };
}

// constant-speed turn at yaw rate omega; lateral accel = speed*omega
std::function<GroundTruth(double)> turnMotion(
  double x0, double y0, double speed, double heading0, double omega)
{
  return [=](double t) {
    // integrate exactly: circular arc
    const double h = heading0 + omega * t;
    double x, y;
    if (std::fabs(omega) < 1e-9) {
      x = x0 + speed * std::cos(heading0) * t;
      y = y0 + speed * std::sin(heading0) * t;
    } else {
      const double r = speed / omega;
      x = x0 + r * (std::sin(h) - std::sin(heading0));
      y = y0 - r * (std::cos(h) - std::cos(heading0));
    }
    return GroundTruth{x, y, speed * std::cos(h), speed * std::sin(h)};
  };
}

std::vector<Scenario> buildScenarios()
{
  std::vector<Scenario> s;
  // --- A. constant straight-line motion -------------------------------
  s.push_back({"A_constant_velocity", "A", true, 10.0, -1, -1,
    rampMotion(-1.5, -1.0, 0.30, 0.0, 0, 0, 1e9, 1e9)});
  s.push_back({"A_constant_velocity_fast", "A", true, 8.0, -1, -1,
    rampMotion(-2.0, -1.0, 0.68, 0.0, 0, 0, 1e9, 1e9)});

  // --- B. crossing motion ---------------------------------------------
  s.push_back({"B_crossing", "B", true, 10.0, -1, -1,
    rampMotion(0.0, -1.5, 0.0, 0.50, 0, 0, 1e9, 1e9)});

  // --- C. stopping / decelerating (ramped, in model) ------------------
  s.push_back({"C_decelerate_stop", "C", true, 10.0, -1, -1,
    rampMotion(-1.5, -1.0, 0.50, 0.0, -0.50, 0.0, 4.0, 5.0)});
  // --- C'. direction reversal (ramped, in model) ----------------------
  s.push_back({"C_reversal", "C", true, 10.0, -1, -1,
    rampMotion(-1.5, -1.0, 0.50, 0.0, -0.50, 0.0, 3.0, 5.0)});
  // --- C''. acceleration (ramped, in model) ---------------------------
  s.push_back({"C_accelerate", "C", true, 10.0, -1, -1,
    rampMotion(-1.5, -1.0, 0.20, 0.0, 0.50, 0.0, 3.0, 4.0)});

  // --- D. turning / change of direction -------------------------------
  // in model: speed 0.5, omega 0.6 -> |a| = 0.30 m/s^2 <= 0.5
  s.push_back({"D_turn_in_model", "D", true, 8.0, -1, -1,
    turnMotion(-1.5, -1.0, 0.50, 0.0, 0.60)});
  // OUT of model: speed 0.5, omega 1.5 -> |a| = 0.75 m/s^2 > 0.5
  s.push_back({"D_turn_out_of_model", "D", false, 8.0, -1, -1,
    turnMotion(-1.5, -1.0, 0.50, 0.0, 1.50)});

  // --- F. close pass / near-intersection ------------------------------
  s.push_back({"F_close_pass", "F", true, 8.0, -1, -1,
    rampMotion(-1.2, -0.15, 0.40, 0.0, 0, 0, 1e9, 1e9)});

  // --- coasting variants (the regime Stage-4G4/4G7 identified) --------
  Scenario occ = {"H_occlusion_short", "F", true, 10.0, 5.0, 5.8,
    rampMotion(-1.5, -1.0, 0.50, 0.0, 0, 0, 1e9, 1e9)};
  s.push_back(occ);
  Scenario occm = {"H_occlusion_maneuver", "F", true, 10.0, 5.0, 5.8,
    rampMotion(-1.5, -1.0, 0.50, 0.0, -0.50, 0.0, 5.0, 7.0)};
  s.push_back(occm);
  return s;
}

// --- metric accumulator ---------------------------------------------
struct Acc
{
  long n{0};
  long cv_own_in{0};
  long cv_consumed_in{0};
  long reach_in{0};
  double cv_area{0.0};
  double reach_area{0.0};
  double cv_cells{0.0};
  double reach_cells{0.0};
  double reach_radius{0.0};
  void add(
    bool cvo, bool cvc, bool r, double ca, double ra, double cc, double rc, double rr)
  {
    ++n;
    cv_own_in += cvo;
    cv_consumed_in += cvc;
    reach_in += r;
    cv_area += ca;
    reach_area += ra;
    cv_cells += cc;
    reach_cells += rc;
    reach_radius += rr;
  }
};

// Area of the k-sigma ellipse of a 2x2 covariance.
double ellipseArea(const Eigen::Matrix2d & cov, double k)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
  const Eigen::Vector2d ev = es.eigenvalues().cwiseMax(0.0);
  return M_PI * k * k * std::sqrt(ev(0) * ev(1));
}

bool insideCvEllipse(const Eigen::Vector2d & mu, const Eigen::Matrix2d & cov,
  double k, double gx, double gy)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
  if (es.info() != Eigen::Success) {
    return false;
  }
  Eigen::Vector2d ev = es.eigenvalues().cwiseMax(kMinPosVariance);
  const Eigen::Matrix2d V = es.eigenvectors();
  const Eigen::Matrix2d sig = V * ev.asDiagonal() * V.transpose();
  const Eigen::Vector2d d(gx - mu.x(), gy - mu.y());
  const double m2 = d.transpose() * sig.inverse() * d;
  return std::isfinite(m2) && m2 <= k * k;
}

// Replicates rasterizeEllipse's painted-cell count EXACTLY: the bounding box
// is clamped to max_influence_radius but the Mahalanobis test uses the
// UNCLAMPED covariance -> painted region = ellipse INTERSECT clamped box.
long cvPaintedCells(const Eigen::Matrix2d & cov, double k, double max_influence)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
  if (es.info() != Eigen::Success) {
    return 0;
  }
  Eigen::Vector2d ev = es.eigenvalues().cwiseMax(kMinPosVariance);
  const Eigen::Matrix2d V = es.eigenvectors();
  const Eigen::Matrix2d sig = V * ev.asDiagonal() * V.transpose();
  const Eigen::Matrix2d inv = sig.inverse();
  double half_x = k * std::sqrt(sig(0, 0));
  double half_y = k * std::sqrt(sig(1, 1));
  if (max_influence > 0.0) {
    half_x = std::min(half_x, max_influence);
    half_y = std::min(half_y, max_influence);
  }
  long cells = 0;
  const int ni = static_cast<int>(std::floor(half_x / kCostmapRes));
  const int nj = static_cast<int>(std::floor(half_y / kCostmapRes));
  for (int j = -nj; j <= nj; ++j) {
    for (int i = -ni; i <= ni; ++i) {
      const Eigen::Vector2d d(i * kCostmapRes, j * kCostmapRes);
      const double m2 = d.transpose() * inv * d;
      if (std::isfinite(m2) && m2 <= k * k) {
        ++cells;
      }
    }
  }
  return cells;
}

// Replicates rasterizeReachableSet's painted-cell count EXACTLY: the SEMI-AXES
// themselves are clamped, and membership is tested against the CLAMPED ellipse
// -> painted region = the whole clamped ellipse.
long reachPaintedCells(double semi_major, double semi_minor, double max_influence)
{
  if (max_influence > 0.0) {
    semi_major = std::min(semi_major, max_influence);
    semi_minor = std::min(semi_minor, max_influence);
  }
  if (semi_major <= 0.0 || semi_minor <= 0.0) {
    return 0;
  }
  long cells = 0;
  const int ni = static_cast<int>(std::floor(semi_major / kCostmapRes));
  const int nj = static_cast<int>(std::floor(semi_minor / kCostmapRes));
  for (int j = -nj; j <= nj; ++j) {
    for (int i = -ni; i <= ni; ++i) {
      const double u = (i * kCostmapRes) / semi_major;
      const double v = (j * kCostmapRes) / semi_minor;
      if (u * u + v * v <= 1.0) {
        ++cells;
      }
    }
  }
  return cells;
}

}  // namespace

int main()
{
  const ReachabilityBounds B = productionBounds();
  std::printf("{\n");

  // =================================================================
  // PART 1 -- independent reconstruction and edge cases
  // =================================================================
  double worst_closed_form_err = 0.0;
  for (double s0 : {0.0, 0.15, 0.3, 0.5, 0.68, 0.8}) {
    for (double t : {0.1, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0}) {
      const double a = reachRadius(t, s0, B);
      const double b = reachRadiusNumeric(t, s0, B, 400000);
      worst_closed_form_err = std::max(worst_closed_form_err, std::fabs(a - b));
    }
  }
  // continuity and C1 at the crossover
  const double tc = (B.max_speed + 0.3) / B.max_acceleration;
  const double eps = 1e-7;
  const double jump = std::fabs(reachRadius(tc + eps, 0.3, B) - reachRadius(tc - eps, 0.3, B));
  const double slope_lo = (reachRadius(tc - eps, 0.3, B) - reachRadius(tc - 2 * eps, 0.3, B)) / eps;
  const double slope_hi = (reachRadius(tc + 2 * eps, 0.3, B) - reachRadius(tc + eps, 0.3, B)) / eps;
  // monotonicity in t and in a_max
  bool mono_t = true, mono_a = true;
  for (double t = 0.05; t < 6.0; t += 0.05) {
    if (reachRadius(t + 0.05, 0.3, B) < reachRadius(t, 0.3, B)) {mono_t = false;}
  }
  for (double a = 0.05; a < 3.0; a += 0.05) {
    ReachabilityBounds b1 = B, b2 = B;
    b1.max_acceleration = a;
    b2.max_acceleration = a + 0.05;
    if (reachRadius(1.5, 0.3, b2) < reachRadius(1.5, 0.3, b1)) {mono_a = false;}
  }
  // edge cases
  ReachabilityBounds zero_a = B; zero_a.max_acceleration = 0.0;
  ReachabilityBounds neg_a = B; neg_a.max_acceleration = -1.0;
  ReachabilityBounds neg_v = B; neg_v.max_speed = -1.0;
  const double r_t0 = reachRadius(0.0, 0.3, B);
  const double r_tneg = reachRadius(-1.0, 0.3, B);
  const double r_tnan = reachRadius(std::nan(""), 0.3, B);
  const double r_tinf = reachRadius(std::numeric_limits<double>::infinity(), 0.3, B);
  const double r_zero_a = reachRadius(2.0, 0.3, zero_a);
  const double r_neg_a = reachRadius(2.0, 0.3, neg_a);
  const double r_neg_v = reachRadius(2.0, 0.3, neg_v);
  const double r_neg_s0 = reachRadius(2.0, -5.0, B);
  const double r_huge_t = reachRadius(1e6, 0.3, B);
  // a_max = 0 must collapse the region exactly onto the CV sigma ellipse
  Eigen::Matrix2d cov_probe = Eigen::Matrix2d::Identity() * 0.04;
  const ReachabilityRegion collapsed =
    buildRegion(1.0, 2.0, 0.3, 0.0, cov_probe, 1.0, zero_a);
  const double collapse_err =
    std::fabs(collapsed.semi_major - kTrackerSigma * 0.2) +
    std::fabs(collapsed.semi_minor - kTrackerSigma * 0.2);
  // non-finite / invalid input to buildRegion
  const ReachabilityRegion nan_state =
    buildRegion(std::nan(""), 0.0, 0.0, 0.0, cov_probe, 1.0, B);
  const ReachabilityRegion neg_time =
    buildRegion(0.0, 0.0, 0.0, 0.0, cov_probe, -1.0, B);
  Eigen::Matrix2d cov_nan = Eigen::Matrix2d::Constant(std::nan(""));
  const ReachabilityRegion nan_cov = buildRegion(0, 0, 0, 0, cov_nan, 1.0, B);
  Eigen::Matrix2d cov_negdef;
  cov_negdef << -1.0, 0.0, 0.0, -1.0;
  const ReachabilityRegion negdef_cov = buildRegion(0, 0, 0, 0, cov_negdef, 1.0, B);

  std::printf("  \"part1_model_reconstruction\": {\n");
  std::printf("    \"closed_form_vs_numeric_max_abs_err_m\": %.3e,\n", worst_closed_form_err);
  std::printf("    \"crossover_t_c_s\": %.6f,\n", tc);
  std::printf("    \"crossover_value_jump_m\": %.3e,\n", jump);
  std::printf("    \"crossover_slope_below_mps\": %.6f,\n", slope_lo);
  std::printf("    \"crossover_slope_above_mps\": %.6f,\n", slope_hi);
  std::printf("    \"monotone_in_t\": %s,\n", mono_t ? "true" : "false");
  std::printf("    \"monotone_in_a_max\": %s,\n", mono_a ? "true" : "false");
  std::printf("    \"edge_t_zero\": %.6f,\n", r_t0);
  std::printf("    \"edge_t_negative\": %.6f,\n", r_tneg);
  std::printf("    \"edge_t_nan\": %.6f,\n", r_tnan);
  std::printf("    \"edge_t_inf\": %.6f,\n", r_tinf);
  std::printf("    \"edge_a_max_zero\": %.6f,\n", r_zero_a);
  std::printf("    \"edge_a_max_negative\": %.6f,\n", r_neg_a);
  std::printf("    \"edge_v_max_negative\": %.6f,\n", r_neg_v);
  std::printf("    \"edge_s0_negative\": %.6f,\n", r_neg_s0);
  std::printf("    \"edge_t_1e6\": %.6f,\n", r_huge_t);
  std::printf("    \"a_max_zero_collapse_err_m\": %.3e,\n", collapse_err);
  std::printf("    \"nan_position_valid\": %s,\n", nan_state.valid ? "true" : "false");
  std::printf("    \"negative_time_valid\": %s,\n", neg_time.valid ? "true" : "false");
  std::printf("    \"nan_covariance_valid\": %s,\n", nan_cov.valid ? "true" : "false");
  std::printf("    \"negdef_covariance_valid\": %s,\n", negdef_cov.valid ? "true" : "false");
  std::printf("    \"negdef_covariance_semi_major_m\": %.6f\n", negdef_cov.semi_major);
  std::printf("  },\n");

  // =================================================================
  // PART 2 -- adversarial containment, two regimes
  // =================================================================
  auto runAdversary = [&](double s0_lo, double s0_hi, bool enforce_cap, int trials) {
    std::mt19937 rng(20261121);
    std::uniform_real_distribution<double> speed(s0_lo, s0_hi);
    std::uniform_real_distribution<double> angle(-M_PI, M_PI);
    std::uniform_real_distribution<double> horizon(0.1, 4.0);
    double worst_slack = std::numeric_limits<double>::infinity();
    long escapes = 0;
    double worst_escape = 0.0;
    double worst_escape_t = 0.0, worst_escape_s0 = 0.0;
    for (int i = 0; i < trials; ++i) {
      const double s0 = speed(rng);
      const double hd = angle(rng);
      const double ad = angle(rng);
      const double t = horizon(rng);
      const double dev = simulatedDeviation(
        s0 * std::cos(hd), s0 * std::sin(hd),
        B.max_acceleration * std::cos(ad), B.max_acceleration * std::sin(ad),
        t, B, enforce_cap);
      const double bound = reachRadius(t, s0, B);
      const double slack = bound - dev;
      worst_slack = std::min(worst_slack, slack);
      if (slack < -1e-6) {
        ++escapes;
        if (-slack > worst_escape) {
          worst_escape = -slack;
          worst_escape_t = t;
          worst_escape_s0 = s0;
        }
      }
    }
    return std::vector<double>{
      static_cast<double>(trials), worst_slack, static_cast<double>(escapes),
      worst_escape, worst_escape_t, worst_escape_s0};
  };

  // Regime 1: the model's own assumptions hold (|v0| <= v_max). This is what
  // the existing stage4g4_reachability test samples.
  const auto legal = runAdversary(0.0, B.max_speed, true, 20000);
  // Regime 2: the track's filtered speed EXCEEDS max_speed, and the obstacle
  // is correspondingly not speed-capped at v_max. Never sampled by the
  // existing test.
  const auto over = runAdversary(B.max_speed * 1.001, B.max_speed * 3.0, false, 20000);
  // Regime 2b: filtered speed just barely over the cap, the realistic case for
  // a 0.75 m/s Stage-5B obstacle whose estimate overshoots 0.8 m/s.
  const auto over_small = runAdversary(0.80, 0.95, false, 20000);

  std::printf("  \"part2_adversarial_containment\": {\n");
  std::printf("    \"legal_regime\": {\"s0_range\":[0.0,0.8],\"speed_cap_enforced\":true,"
    "\"trials\":%.0f,\"worst_slack_m\":%.3e,\"escapes\":%.0f},\n",
    legal[0], legal[1], legal[2]);
  std::printf("    \"s0_above_cap\": {\"s0_range\":[0.8008,2.4],\"speed_cap_enforced\":false,"
    "\"trials\":%.0f,\"worst_slack_m\":%.6f,\"escapes\":%.0f,\"escape_rate\":%.4f,"
    "\"worst_escape_m\":%.6f,\"worst_escape_t_s\":%.3f,\"worst_escape_s0_mps\":%.3f},\n",
    over[0], over[1], over[2], over[2] / over[0], over[3], over[4], over[5]);
  std::printf("    \"s0_just_above_cap\": {\"s0_range\":[0.80,0.95],"
    "\"speed_cap_enforced\":false,\"trials\":%.0f,\"worst_slack_m\":%.6f,\"escapes\":%.0f,"
    "\"escape_rate\":%.4f,\"worst_escape_m\":%.6f}\n",
    over_small[0], over_small[1], over_small[2], over_small[2] / over_small[0], over_small[3]);
  std::printf("  },\n");

  // =================================================================
  // PART 3 + 4 -- identical-input comparison
  // =================================================================
  const auto scenarios = buildScenarios();
  std::printf("  \"part3_identical_input_comparison\": {\n");
  std::printf("    \"scan_period_s\": %.2f,\n", kScanPeriod);
  std::printf("    \"seeds_per_scenario\": 40,\n");
  std::printf("    \"tracker_sigma_k\": %.2f,\n", kTrackerSigma);
  std::printf("    \"layer_sigma_k_production\": %.2f,\n", kLayerSigmaProd);
  std::printf("    \"max_influence_radius_production_m\": %.2f,\n", kMaxInfluenceProd);
  std::printf("    \"scenarios\": [\n");

  // pooled accumulators, indexed [horizon][fresh?0:1]
  Acc pooled[kNumHorizons][2];
  Acc pooled_inmodel[kNumHorizons][2];
  double reach_build_ns = 0.0, cv_build_ns = 0.0;
  long reach_calls = 0, cv_calls = 0;

  for (size_t si = 0; si < scenarios.size(); ++si) {
    const Scenario & sc = scenarios[si];
    Acc acc[kNumHorizons][2];
    long coasting_frames = 0, fresh_frames = 0;
    double max_gt_speed = 0.0, max_filt_speed = 0.0;
    long filt_speed_over_cap = 0, filt_frames = 0;

    for (int seed = 0; seed < 40; ++seed) {
      std::mt19937 rng(1000 * static_cast<unsigned>(si) + seed + 7);
      const double noise_std = (sc.id == "E_noisy") ? 0.10 : 0.02;
      std::normal_distribution<double> noise(0.0, noise_std);

      ConstantVelocityKalmanFilter2D kf(kAccelNoise, kMeasNoise);
      bool initialized = false;
      int observations = 0;
      double last_obs_t = 0.0;

      const int nscans = static_cast<int>(sc.duration / kScanPeriod);
      for (int k = 1; k <= nscans; ++k) {
        const double t_scan = k * kScanPeriod;
        const bool dropped = (sc.dropout_start >= 0.0 &&
          t_scan >= sc.dropout_start && t_scan < sc.dropout_end);

        const GroundTruth g = sc.gt(t_scan);
        max_gt_speed = std::max(max_gt_speed, std::hypot(g.vx, g.vy));

        if (!dropped) {
          const double zx = g.x + noise(rng);
          const double zy = g.y + noise(rng);
          if (!initialized) {
            kf.initialize(zx, zy, kInitPosVar, kInitVelVar);
            initialized = true;
            observations = 1;
            last_obs_t = t_scan;
            continue;
          }
          kf.predict(t_scan - last_obs_t);
          kf.update(zx, zy);
          ++observations;
          last_obs_t = t_scan;
        }
        if (!initialized || observations < 2) {
          continue;
        }

        const double age = t_scan - last_obs_t;
        if (age > 1.0) {           // max_observation_age / track_timeout
          continue;
        }
        const int regime = (age <= 1e-9) ? 0 : 1;
        if (regime == 0) {++fresh_frames;} else {++coasting_frames;}

        // ONE filter state feeds BOTH predictors -- identical input by
        // construction, not by replay bookkeeping.
        const Eigen::Vector4d x0 = kf.state();
        const Eigen::Matrix4d P0 = kf.covariance();
        const double s0 = std::hypot(x0(2), x0(3));
        max_filt_speed = std::max(max_filt_speed, s0);
        ++filt_frames;
        if (s0 > B.max_speed) {++filt_speed_over_cap;}

        for (int hi = 0; hi < kNumHorizons; ++hi) {
          const double h = (hi + 1) * kTimeStep;

          // ---- CV predictor (Stage-4C semantics: anchored at last obs) ----
          auto t_cv0 = std::chrono::steady_clock::now();
          const Eigen::Matrix4d Fcv = ConstantVelocityKalmanFilter2D::stateTransition(h);
          const Eigen::Vector4d xcv = Fcv * x0;
          const Eigen::Matrix4d Pcv = Fcv * P0 * Fcv.transpose() +
            ConstantVelocityKalmanFilter2D::processNoise(h, kAccelNoise);
          auto t_cv1 = std::chrono::steady_clock::now();
          cv_build_ns += std::chrono::duration<double, std::nano>(t_cv1 - t_cv0).count();
          ++cv_calls;
          const Eigen::Matrix2d Pcv_pos = Pcv.topLeftCorner<2, 2>();
          const Eigen::Vector2d mu_cv(xcv(0), xcv(1));

          // ---- Reachability predictor (total_time = age + h) --------------
          const double total_time = age + h;
          auto t_r0 = std::chrono::steady_clock::now();
          const Eigen::Matrix4d Fr =
            ConstantVelocityKalmanFilter2D::stateTransition(total_time);
          const Eigen::Matrix4d Pr = Fr * P0 * Fr.transpose() +
            ConstantVelocityKalmanFilter2D::processNoise(total_time, kAccelNoise);
          const ReachabilityRegion reg = buildRegion(
            x0(0), x0(1), x0(2), x0(3), Pr.topLeftCorner<2, 2>(), total_time, B);
          auto t_r1 = std::chrono::steady_clock::now();
          reach_build_ns += std::chrono::duration<double, std::nano>(t_r1 - t_r0).count();
          ++reach_calls;

          // ---- ground truth at each representation's claimed time --------
          const GroundTruth g_own = sc.gt(last_obs_t + h);     // CV's own anchor
          const GroundTruth g_now = sc.gt(t_scan + h);         // scan-anchored

          const bool cv_own = insideCvEllipse(mu_cv, Pcv_pos, kTrackerSigma, g_own.x, g_own.y);
          const bool cv_consumed =
            insideCvEllipse(mu_cv, Pcv_pos, kTrackerSigma, g_now.x, g_now.y);
          const bool reach_in = regionContains(reg, g_now.x, g_now.y);

          const double cv_area = ellipseArea(Pcv_pos, kTrackerSigma);
          const double reach_area = M_PI * reg.semi_major * reg.semi_minor;
          const double cv_cells = static_cast<double>(
            cvPaintedCells(Pcv_pos, kLayerSigmaProd, kMaxInfluenceProd));
          const double reach_cells = static_cast<double>(
            reachPaintedCells(reg.semi_major, reg.semi_minor, kMaxInfluenceProd));

          acc[hi][regime].add(cv_own, cv_consumed, reach_in, cv_area, reach_area,
            cv_cells, reach_cells, reg.reach_radius);
          pooled[hi][regime].add(cv_own, cv_consumed, reach_in, cv_area, reach_area,
            cv_cells, reach_cells, reg.reach_radius);
          if (sc.in_model) {
            pooled_inmodel[hi][regime].add(cv_own, cv_consumed, reach_in, cv_area,
              reach_area, cv_cells, reach_cells, reg.reach_radius);
          }
        }
      }
    }

    std::printf("      {\"id\":\"%s\",\"family\":\"%s\",\"in_model\":%s,"
      "\"fresh_frames\":%ld,\"coasting_frames\":%ld,\"max_gt_speed_mps\":%.3f,"
      "\"max_filtered_speed_mps\":%.3f,\"filtered_frames\":%ld,"
      "\"filtered_speed_over_cap_frames\":%ld,\"horizons\":[",
      sc.id.c_str(), sc.family.c_str(), sc.in_model ? "true" : "false",
      fresh_frames, coasting_frames, max_gt_speed, max_filt_speed,
      filt_frames, filt_speed_over_cap);
    for (int hi = 0; hi < kNumHorizons; ++hi) {
      for (int rg = 0; rg < 2; ++rg) {
        const Acc & a = acc[hi][rg];
        if (a.n == 0) {continue;}
        std::printf("%s{\"h\":%.1f,\"regime\":\"%s\",\"n\":%ld,"
          "\"cv_own\":%.4f,\"cv_consumed\":%.4f,\"reach\":%.4f,"
          "\"cv_area_m2\":%.4f,\"reach_area_m2\":%.4f,"
          "\"cv_cells\":%.1f,\"reach_cells\":%.1f,\"reach_radius_m\":%.4f}",
          (hi == 0 && rg == 0) ? "" : ",",
          (hi + 1) * kTimeStep, rg == 0 ? "fresh" : "coasting", a.n,
          static_cast<double>(a.cv_own_in) / a.n,
          static_cast<double>(a.cv_consumed_in) / a.n,
          static_cast<double>(a.reach_in) / a.n,
          a.cv_area / a.n, a.reach_area / a.n,
          a.cv_cells / a.n, a.reach_cells / a.n, a.reach_radius / a.n);
      }
    }
    std::printf("]}%s\n", (si + 1 == scenarios.size()) ? "" : ",");
  }
  std::printf("    ],\n");

  // pooled summary
  auto printPooled = [&](const char * label, Acc p[kNumHorizons][2], bool last) {
    std::printf("    \"%s\": [", label);
    bool first = true;
    for (int hi = 0; hi < kNumHorizons; ++hi) {
      for (int rg = 0; rg < 2; ++rg) {
        const Acc & a = p[hi][rg];
        if (a.n == 0) {continue;}
        const double ra = a.reach_area / a.n;
        const double ca = a.cv_area / a.n;
        std::printf("%s{\"h\":%.1f,\"regime\":\"%s\",\"n\":%ld,"
          "\"cv_own\":%.4f,\"cv_consumed\":%.4f,\"reach\":%.4f,"
          "\"cv_area_m2\":%.4f,\"reach_area_m2\":%.4f,\"area_ratio\":%.3f,"
          "\"cv_cells\":%.1f,\"reach_cells\":%.1f,\"cell_ratio\":%.3f,"
          "\"reach_containment_per_m2\":%.4f,\"cv_containment_per_m2\":%.4f}",
          first ? "" : ",", (hi + 1) * kTimeStep, rg == 0 ? "fresh" : "coasting", a.n,
          static_cast<double>(a.cv_own_in) / a.n,
          static_cast<double>(a.cv_consumed_in) / a.n,
          static_cast<double>(a.reach_in) / a.n,
          ca, ra, (ca > 0 ? ra / ca : 0.0),
          a.cv_cells / a.n, a.reach_cells / a.n,
          (a.cv_cells > 0 ? a.reach_cells / a.cv_cells : 0.0),
          (ra > 0 ? (static_cast<double>(a.reach_in) / a.n) / ra : 0.0),
          (ca > 0 ? (static_cast<double>(a.cv_consumed_in) / a.n) / ca : 0.0));
        first = false;
      }
    }
    std::printf("]%s\n", last ? "" : ",");
  };
  printPooled("pooled_all_scenarios", pooled, false);
  printPooled("pooled_in_model_only", pooled_inmodel, true);
  std::printf("  },\n");

  // =================================================================
  // PART 4 -- cost
  // =================================================================
  std::printf("  \"part4_cost\": {\n");
  std::printf("    \"cv_sample_mean_ns\": %.1f,\n", cv_build_ns / cv_calls);
  std::printf("    \"reach_sample_mean_ns\": %.1f,\n", reach_build_ns / reach_calls);
  std::printf("    \"reach_over_cv_ratio\": %.2f,\n",
    (reach_build_ns / reach_calls) / (cv_build_ns / cv_calls));
  std::printf("    \"samples_timed\": %ld,\n", reach_calls);
  std::printf("    \"per_track_6_sample_reach_us\": %.3f\n",
    6.0 * (reach_build_ns / reach_calls) / 1000.0);
  std::printf("  }\n}\n");
  return 0;
}
