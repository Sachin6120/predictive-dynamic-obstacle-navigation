// Stage-5C -- CORRECTED probe of the configured-max_speed validity premise of
// the Stage-4G4 reachability bound. ANALYSIS ONLY: this file includes the
// shipped production header and changes nothing in it.
//
// Build (no CMake/production build file is touched):
//   g++ -O2 -std=c++17 -I/usr/include/eigen3 \
//       -Isrc/predictive_nav_tracking/include \
//       validation/stage5c/stage5c_speedcap.cpp -o <out>/stage5c_speedcap
//
// WHY THIS FILE WAS REWRITTEN
// ---------------------------
// The first version of this probe (committed in e1c1012 alongside
// validation/stage5c/speed_cap_probe.json) conflated two quantities the
// mathematics keeps strictly apart:
//
//   s0      the OBSERVED current speed of the track. A model INPUT: it sets
//           the constant-velocity nominal AND enters the deviation cap
//           |v(s) - v0| <= max_speed + s0.
//   v_phys  the obstacle's TRUE future speed capability. A property of the
//           physical world that the model never sees.
//
// It drew a single `v_true` and passed it as BOTH -- it evaluated
//     reachRadius(t, v_true, bounds)   against an adversary ceilinged at v_true
// -- so raising the adversary's capability also raised the model's own bound by
// the same amount. Since r_det is monotone increasing in s0, that is a
// tautology, and the probe duly reported escape = 0 at every speed, 1.40 and
// 4.20 m/s included. It never tested the premise.
//
// Two further defects followed from the same design:
//   * it swept only t = 0.5..3.0 s, while production admits
//     total_time = observation_age + horizon <= 1.0 + 3.0 = 4.0 s;
//   * `min_max_speed_for_validity` reported `required = v_true` by
//     construction and was degenerate (0.2500 m at every row).
//
// THE QUESTION THIS FILE ACTUALLY ASKS
// ------------------------------------
// "Given an observed state of speed s0, what happens if the physical obstacle
//  can subsequently move faster than the configured max_speed the model
//  assumes?"
//
// The model under test keeps the frozen production bounds
// (a_max = 0.5 m/s^2, max_speed = 0.8 m/s). The adversary's capability v_phys
// is an independent third quantity. Part E is the only place max_speed is
// varied, and it is varied as a deliberate sensitivity sweep.
//
// WHAT `escape_m` IS MEASURED AGAINST
// -----------------------------------
// Against the DETERMINISTIC term r_det alone, exactly as README section 3
// frames it. The published region additionally carries the statistical term
// k*sqrt(lambda), which may absorb part of an escape in practice. Reporting
// against r_det alone is therefore the stricter reading of the premise and is
// the one relevant to the bound's own validity claim.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "predictive_nav_tracking/reachability.hpp"

using predictive_nav_tracking::ReachabilityBounds;
using predictive_nav_tracking::reachRadius;

namespace
{

// ---- frozen production values (production_tracker_params.yaml:138-143) ----
constexpr double kAmax = 0.5;                  // m/s^2, max_acceleration
constexpr double kConfiguredMaxSpeed = 0.8;    // m/s, max_speed  (THE PREMISE)
constexpr double kMaxObservationAge = 1.0;     // s
constexpr double kHorizonMax = 3.0;            // s
constexpr double kTimeStep = 0.5;              // s
constexpr double kMaxTotalTime = kMaxObservationAge + kHorizonMax;   // 4.0 s

// Integrator steps. The velocity profile of an admissible trajectory is
// piecewise linear (constant acceleration, then clamped at the speed ceiling),
// and the trapezoidal update below is exact on linear segments, so the only
// error is the single kink inside one step: O(a_max*dt^2) ~ 5e-9 m at 1e-4.
constexpr double kDtSweep = 1e-4;
constexpr double kDtFine = 1e-6;

ReachabilityBounds configuredBounds()
{
  ReachabilityBounds b;
  b.max_acceleration = kAmax;
  b.max_speed = kConfiguredMaxSpeed;
  b.covariance_sigma_level = 2.0;
  b.safety_margin = 0.0;
  return b;
}

// Supremum of |d(t)| = |int_0^t (v(s) - v0) ds| over trajectories with
// |a| <= a_max, |v| <= v_ceiling and |v(0)| = s0:
//
//   R(t) = int_0^t min(a_max * s, v_ceiling + s0) ds
//
// This is reachRadius()'s own closed form with the CEILING left free.
//   v_ceiling = configured max_speed -> the MODEL BOUND (what the robot uses)
//   v_ceiling = v_phys               -> the TRUE reachable deviation
// Part A verifies by forward simulation that the true value is attained.
double reachClosedForm(double t, double s0, double v_ceiling, double a_max)
{
  if (!(t > 0.0) || !(a_max > 0.0)) {
    return 0.0;
  }
  const double cap = std::max(v_ceiling, 0.0) + std::max(s0, 0.0);
  const double t_c = cap / a_max;
  if (t <= t_c) {
    return 0.5 * a_max * t * t;
  }
  return 0.5 * a_max * t_c * t_c + cap * (t - t_c);
}

// Forward-integrates ONE physically admissible trajectory and returns the
// magnitude of its deviation from the constant-velocity nominal p0 + v0*t.
//
// Physical admissibility is enforced here and ONLY here: |a| <= a_max and
// |v| <= v_phys. `v_phys` is the PHYSICAL capability and is deliberately NOT
// the model's configured max_speed. `switch_time` flips the acceleration sign
// once, so bang-bang profiles are reachable by the adversary too.
double simulateDeviation(
  double v0x, double v0y, double ax, double ay, double switch_time,
  double t, double a_max, double v_phys, double dt)
{
  double px = 0.0, py = 0.0, vx = v0x, vy = v0y;
  const double an = std::hypot(ax, ay);
  if (an > a_max && an > 0.0) {
    ax *= a_max / an;
    ay *= a_max / an;
  }
  for (double s = 0.0; s < t; s += dt) {
    const double step = std::min(dt, t - s);
    const double sgn = (s >= switch_time) ? -1.0 : 1.0;
    const double vxb = vx, vyb = vy;
    vx += sgn * ax * step;
    vy += sgn * ay * step;
    const double sp = std::hypot(vx, vy);
    if (sp > v_phys && sp > 0.0) {            // PHYSICAL ceiling, not the model's
      vx *= v_phys / sp;
      vy *= v_phys / sp;
    }
    px += 0.5 * (vxb + vx) * step;
    py += 0.5 * (vyb + vy) * step;
  }
  return std::hypot(px - v0x * t, py - v0y * t);
}

// The worst-case adversary: 1-D full reversal. Start at +s0 along x, apply
// -a_max until the physical ceiling -v_phys is reached, then hold. This makes
// |v(s) - v0| = min(a_max*s, v_phys + s0) at every instant, which is exactly
// the integrand of R(t), so it ATTAINS the supremum.
double simulateWorstCase(double t, double s0, double v_phys, double a_max, double dt)
{
  return simulateDeviation(
    s0, 0.0, -a_max, 0.0, std::numeric_limits<double>::infinity(),
    t, a_max, v_phys, dt);
}

struct Capability
{
  double v_phys;
  const char * label;
};

const std::vector<Capability> & capabilities()
{
  static const std::vector<Capability> c = {
    {0.25, "stage5b slow class, in domain"},
    {0.50, "stage5b medium class, in domain"},
    {0.68, "stage4g2 sim maximum, in domain"},
    {0.75, "stage5b fastest commanded speed, in domain"},
    {0.80, "exactly the configured premise"},
    {0.90, "12 percent over the premise"},
    {1.10, "slow walk, real world"},
    {1.40, "typical pedestrian, real world"},
    {2.00, "fast walk / jog"},
    {3.00, "slow cyclist"},
    {4.20, "cyclist 15 km/h"},
  };
  return c;
}

// Observed speeds. 0.00 / 0.30 / 0.75 are the rows README section 3 discusses.
const std::vector<double> & observedSpeeds()
{
  static const std::vector<double> s = {0.00, 0.30, 0.50, 0.75, 0.80};
  return s;
}

// total_time grid over the FULL production window. Each t is realisable as
// observation_age + horizon with horizon in {0.5..3.0} and age in [0, 1.0].
const std::vector<double> & totalTimes()
{
  static const std::vector<double> t = {0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
  return t;
}

}  // namespace

int main()
{
  const ReachabilityBounds B = configuredBounds();
  std::printf("{\n");
  std::printf("  \"experiment\": \"stage5c_speed_cap_validity_corrected\",\n");
  std::printf("  \"corrects\": \"validation/stage5c/speed_cap_probe.json as committed in "
    "e1c101287cba94d22cc72882ec306c2a26d5005d\",\n");
  std::printf("  \"defect_corrected\": \"the superseded probe passed the adversary's "
    "physical speed capability into the model as its observed s0; because r_det is "
    "monotone increasing in s0 this raised the model bound in step with the adversary "
    "and reported escape=0 at every speed. It also swept only t<=3.0 s and derived a "
    "degenerate min_max_speed_for_validity field.\",\n");

  // ------------------------------------------------------------------
  // Semantics, stated so the JSON is self-describing
  // ------------------------------------------------------------------
  std::printf("  \"semantics\": {\n");
  std::printf("    \"s0_observed_speed_mps\": \"|v0| of the Kalman state at the track's "
    "last real observation; a MODEL INPUT, entering both the CV nominal p0+v0*t and the "
    "deviation cap |v(s)-v0| <= max_speed + s0\",\n");
  std::printf("    \"configured_max_speed_mps\": \"bounds.max_speed, a configuration "
    "constant (production 0.8); a MODEL ASSUMPTION, never compared against the track's "
    "own measured speed anywhere in reachability.hpp or the tracker\",\n");
  std::printf("    \"v_phys_capability_mps\": \"the obstacle's true future speed ceiling; "
    "a property of the PHYSICAL WORLD, independent of s0 and of the configured value, and "
    "not represented anywhere in the shipped code\",\n");
  std::printf("    \"total_time_s\": \"observation_age + horizon, the t of r_det(t); "
    "production admits up to %.1f s (max_observation_age %.1f + horizon %.1f)\",\n",
    kMaxTotalTime, kMaxObservationAge, kHorizonMax);
  std::printf("    \"model_bound_m\": \"r_det(t) = reachRadius(t, s0, configured bounds) "
    "-- the DETERMINISTIC term the robot actually uses\",\n");
  std::printf("    \"true_reach_m\": \"sup |d(t)| over trajectories with |a|<=a_max and "
    "|v|<=v_phys, i.e. the same closed form with v_phys in place of the configured "
    "max_speed; Part A verifies it is attained\",\n");
  std::printf("    \"escape_m\": \"max(0, true_reach_m - model_bound_m), measured against "
    "the deterministic term ALONE; the published region additionally carries the "
    "statistical term k*sqrt(lambda), so this is the stricter reading\",\n");
  std::printf("    \"guarantee_condition\": \"r_det is an outer bound iff BOTH |a|<=a_max "
    "AND |v|<=configured max_speed hold; the second premise is what makes "
    "|v(s)-v0| <= max_speed+s0 valid\"\n");
  std::printf("  },\n");
  std::printf("  \"model_configuration\": {\"max_acceleration_mps2\": %.2f, "
    "\"configured_max_speed_mps\": %.2f, \"safety_margin_m\": %.2f, "
    "\"max_total_time_s\": %.1f, \"horizon_step_s\": %.1f},\n",
    kAmax, kConfiguredMaxSpeed, B.safety_margin, kMaxTotalTime, kTimeStep);
  std::printf("  \"integrator\": {\"sweep_dt_s\": %.0e, \"fine_dt_s\": %.0e, "
    "\"scheme\": \"trapezoidal on velocity, exact on linear segments; error is the "
    "single clamp kink inside one step\"},\n", kDtSweep, kDtFine);

  // ==================================================================
  // PART A -- the true reachable deviation, and that it is attained
  // ==================================================================
  // A1: the 1-D full reversal matches the closed form with v_ceiling = v_phys.
  double worst_closed_vs_sim = 0.0;
  double worst_closed_vs_sim_at_t = 0.0;
  for (double s0 : observedSpeeds()) {
    for (const auto & c : capabilities()) {
      if (c.v_phys < s0) {
        continue;                                 // s0 > capability is unphysical
      }
      for (double t : totalTimes()) {
        const double sim = simulateWorstCase(t, s0, c.v_phys, kAmax, kDtFine);
        const double closed = reachClosedForm(t, s0, c.v_phys, kAmax);
        const double err = std::fabs(sim - closed);
        if (err > worst_closed_vs_sim) {
          worst_closed_vs_sim = err;
          worst_closed_vs_sim_at_t = t;
        }
      }
    }
  }

  // A2: no randomized admissible trajectory beats the closed form. Random
  // heading, random acceleration direction, random single bang-bang switch.
  std::mt19937 rng(20261121);
  std::uniform_real_distribution<double> angle(-M_PI, M_PI);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  double worst_overshoot = -std::numeric_limits<double>::infinity();
  long overshoots = 0;
  const int kAdvTrials = 4000;
  for (int i = 0; i < kAdvTrials; ++i) {
    const double s0 = observedSpeeds()[rng() % observedSpeeds().size()];
    const auto & c = capabilities()[rng() % capabilities().size()];
    if (c.v_phys < s0) {
      continue;
    }
    const double t = 0.1 + unit(rng) * (kMaxTotalTime - 0.1);
    const double hd = angle(rng);
    const double ad = angle(rng);
    const double sw = (unit(rng) < 0.5) ?
      std::numeric_limits<double>::infinity() : unit(rng) * t;
    const double dev = simulateDeviation(
      s0 * std::cos(hd), s0 * std::sin(hd),
      kAmax * std::cos(ad), kAmax * std::sin(ad), sw, t, kAmax, c.v_phys, kDtSweep);
    const double over = dev - reachClosedForm(t, s0, c.v_phys, kAmax);
    if (over > worst_overshoot) {
      worst_overshoot = over;
    }
    if (over > 1e-6) {
      ++overshoots;
    }
  }
  std::printf("  \"partA_true_reach_is_the_closed_form\": {\n");
  std::printf("    \"claim\": \"sup|d(t)| under (|a|<=a_max, |v|<=v_phys) equals "
    "int_0^t min(a_max*s, v_phys+s0) ds, attained by the 1-D full-reversal maneuver\",\n");
  std::printf("    \"worst_case_sim_vs_closed_form_max_abs_err_m\": %.3e,\n",
    worst_closed_vs_sim);
  std::printf("    \"worst_err_at_total_time_s\": %.1f,\n", worst_closed_vs_sim_at_t);
  std::printf("    \"randomized_admissible_trials\": %d,\n", kAdvTrials);
  std::printf("    \"trials_exceeding_closed_form\": %ld,\n", overshoots);
  std::printf("    \"worst_overshoot_above_closed_form_m\": %.3e\n", worst_overshoot);
  std::printf("  },\n");

  // ==================================================================
  // PART B -- IN DOMAIN: v_phys <= configured max_speed
  // ==================================================================
  // The premise holds. r_det must contain every admissible trajectory.
  double in_worst_slack = std::numeric_limits<double>::infinity();
  long in_trials = 0, in_escapes = 0;
  double in_worst_escape = 0.0;
  std::uniform_real_distribution<double> v_in(0.0, kConfiguredMaxSpeed);
  for (int i = 0; i < 20000; ++i) {
    const double v_phys = v_in(rng);
    std::uniform_real_distribution<double> s0_dist(0.0, v_phys);
    const double s0 = s0_dist(rng);               // observed <= physical capability
    const double t = 0.1 + unit(rng) * (kMaxTotalTime - 0.1);
    const double hd = angle(rng);
    const double ad = angle(rng);
    const double sw = (unit(rng) < 0.5) ?
      std::numeric_limits<double>::infinity() : unit(rng) * t;
    const double dev = simulateDeviation(
      s0 * std::cos(hd), s0 * std::sin(hd),
      kAmax * std::cos(ad), kAmax * std::sin(ad), sw, t, kAmax, v_phys, kDtSweep);
    const double bound = reachRadius(t, s0, B);   // THE SHIPPED FUNCTION
    const double slack = bound - dev;
    ++in_trials;
    in_worst_slack = std::min(in_worst_slack, slack);
    if (slack < -1e-6) {
      ++in_escapes;
      in_worst_escape = std::max(in_worst_escape, -slack);
    }
  }
  // Worst-case (non-random) in-domain adversary at every grid point.
  double in_worst_slack_wc = std::numeric_limits<double>::infinity();
  for (double s0 : observedSpeeds()) {
    for (double t : totalTimes()) {
      const double dev = simulateWorstCase(t, s0, kConfiguredMaxSpeed, kAmax, kDtFine);
      in_worst_slack_wc = std::min(in_worst_slack_wc, reachRadius(t, s0, B) - dev);
    }
  }
  std::printf("  \"partB_in_domain\": {\n");
  std::printf("    \"condition\": \"v_phys <= configured max_speed %.2f, s0 <= v_phys\",\n",
    kConfiguredMaxSpeed);
  std::printf("    \"randomized_trials\": %ld,\n", in_trials);
  std::printf("    \"escapes\": %ld,\n", in_escapes);
  std::printf("    \"worst_slack_m\": %.3e,\n", in_worst_slack);
  std::printf("    \"worst_escape_m\": %.3e,\n", in_worst_escape);
  std::printf("    \"worst_case_adversary_worst_slack_m\": %.3e,\n", in_worst_slack_wc);
  std::printf("    \"verdict\": \"%s\"\n",
    (in_escapes == 0) ? "bound VALID and exactly tight in domain" : "ESCAPES IN DOMAIN");
  std::printf("  },\n");

  // ==================================================================
  // PART C -- OUT OF DOMAIN grid: s0 x v_phys x total_time
  // ==================================================================
  std::printf("  \"partC_grid\": [\n");
  bool first_row = true;
  double grid_max_escape = 0.0;
  double grid_max_escape_t = 0.0, grid_max_escape_s0 = 0.0, grid_max_escape_v = 0.0;
  for (double s0 : observedSpeeds()) {
    for (const auto & c : capabilities()) {
      if (c.v_phys < s0) {
        continue;
      }
      for (double t : totalTimes()) {
        const double bound = reachRadius(t, s0, B);
        const double truth = reachClosedForm(t, s0, c.v_phys, kAmax);
        const double truth_sim = simulateWorstCase(t, s0, c.v_phys, kAmax, kDtSweep);
        const double escape = std::max(0.0, truth - bound);
        const double escape_sim = std::max(0.0, truth_sim - bound);
        if (escape > grid_max_escape) {
          grid_max_escape = escape;
          grid_max_escape_t = t;
          grid_max_escape_s0 = s0;
          grid_max_escape_v = c.v_phys;
        }
        std::printf("%s    {\"s0_mps\":%.2f,\"configured_max_speed_mps\":%.2f,"
          "\"v_phys_mps\":%.2f,\"capability_label\":\"%s\","
          "\"in_domain\":%s,\"total_time_s\":%.1f,"
          "\"model_t_c_s\":%.4f,\"phys_t_c_s\":%.4f,"
          "\"model_bound_m\":%.6f,\"true_reach_m\":%.6f,\"true_reach_simulated_m\":%.6f,"
          "\"escape_m\":%.6f,\"escape_simulated_m\":%.6f,"
          "\"relative_under_coverage\":%.4f}",
          first_row ? "" : ",\n", s0, kConfiguredMaxSpeed, c.v_phys, c.label,
          (c.v_phys <= kConfiguredMaxSpeed) ? "true" : "false", t,
          (kConfiguredMaxSpeed + s0) / kAmax, (c.v_phys + s0) / kAmax,
          bound, truth, truth_sim, escape, escape_sim,
          (bound > 0.0 ? escape / bound : 0.0));
        first_row = false;
      }
    }
  }
  std::printf("\n  ],\n");

  // ==================================================================
  // PART D -- where under-coverage begins
  // ==================================================================
  // Analytically: both model and truth sit in the same quadratic branch
  // 0.5*a*t^2 while t <= t_c_model = (configured_max_speed + s0)/a_max, so
  // escape is identically zero there and strictly positive after, whenever
  // v_phys > configured_max_speed. Confirmed by a 1 ms scan.
  std::printf("  \"partD_escape_onset\": {\n");
  std::printf("    \"analytic_onset_total_time_s\": "
    "\"(configured_max_speed + s0) / a_max, independent of v_phys\",\n");
  std::printf("    \"rows\": [\n");
  bool first_onset = true;
  for (double s0 : observedSpeeds()) {
    const double analytic = (kConfiguredMaxSpeed + s0) / kAmax;
    for (const auto & c : capabilities()) {
      if (c.v_phys <= kConfiguredMaxSpeed || c.v_phys < s0) {
        continue;
      }
      double numeric = -1.0;
      for (double t = 1e-3; t <= kMaxTotalTime + 1e-12; t += 1e-3) {
        if (reachClosedForm(t, s0, c.v_phys, kAmax) - reachRadius(t, s0, B) > 1e-9) {
          numeric = t;
          break;
        }
      }
      std::printf("%s      {\"s0_mps\":%.2f,\"v_phys_mps\":%.2f,"
        "\"analytic_onset_s\":%.4f,\"numeric_onset_s\":%.4f,"
        "\"within_production_window\":%s,\"escape_at_4s_m\":%.6f}",
        first_onset ? "" : ",\n", s0, c.v_phys, analytic, numeric,
        (analytic <= kMaxTotalTime) ? "true" : "false",
        std::max(0.0, reachClosedForm(kMaxTotalTime, s0, c.v_phys, kAmax) -
        reachRadius(kMaxTotalTime, s0, B)));
      first_onset = false;
    }
  }
  std::printf("\n    ]\n  },\n");

  // ==================================================================
  // PART E -- replacement for the degenerate min_max_speed_for_validity
  // ==================================================================
  // DEFINITION. min_configured_max_speed_for_containment(T; s0, v_phys) is the
  // smallest value of bounds.max_speed such that the model bound contains the
  // true reachable deviation at EVERY total_time in (0, T], for an obstacle of
  // observed speed s0 and physical capability v_phys.
  //
  // DERIVATION. Containment at t needs
  //     R(t; s0, v_cfg) >= R(t; s0, v_phys).
  // Both equal 0.5*a_max*t^2 while t <= (cap + s0)/a_max, so containment holds
  // iff the model is still in its quadratic branch at T, or its cap is not
  // optimistic at all:
  //     T <= (v_cfg + s0)/a_max      OR      v_cfg >= v_phys.
  // Escape is zero before the onset and increasing after, so t = T is the
  // binding case. Hence
  //     v_cfg_min(T) = min(v_phys, max(0, a_max*T - s0)).
  // Verified below against a 1e-4 m/s bisection on the shipped reachRadius().
  std::printf("  \"partE_min_configured_max_speed_for_containment\": {\n");
  std::printf("    \"definition\": \"smallest bounds.max_speed for which the model bound "
    "contains the true reachable deviation at every total_time in (0,T], given observed "
    "speed s0 and physical capability v_phys\",\n");
  std::printf("    \"closed_form\": \"min(v_phys, max(0, a_max*T - s0))\",\n");
  std::printf("    \"replaces\": \"min_max_speed_for_validity, which was degenerate "
    "(required = v_true by construction, 0.2500 m at every row) and is not retained\",\n");
  std::printf("    \"rows\": [\n");
  bool first_e = true;
  double worst_bisect_err = 0.0;
  for (double T : {2.0, 3.0, kMaxTotalTime}) {
    for (double s0 : observedSpeeds()) {
      for (const auto & c : capabilities()) {
        if (c.v_phys < s0) {
          continue;
        }
        const double analytic = std::min(c.v_phys, std::max(0.0, kAmax * T - s0));
        // bisection on the shipped function over the whole window
        double lo = 0.0, hi = c.v_phys;
        auto contains = [&](double v_cfg) {
          ReachabilityBounds t_b = B;
          t_b.max_speed = v_cfg;
          for (double t = 0.01; t <= T + 1e-12; t += 0.01) {
            if (reachClosedForm(t, s0, c.v_phys, kAmax) - reachRadius(t, s0, t_b) > 1e-9) {
              return false;
            }
          }
          return true;
        };
        if (contains(0.0)) {
          hi = 0.0;
        } else {
          for (int it = 0; it < 60 && hi - lo > 1e-5; ++it) {
            const double mid = 0.5 * (lo + hi);
            if (contains(mid)) {
              hi = mid;
            } else {
              lo = mid;
            }
          }
        }
        worst_bisect_err = std::max(worst_bisect_err, std::fabs(hi - analytic));
        if (T == kMaxTotalTime) {
          std::printf("%s      {\"window_T_s\":%.1f,\"s0_mps\":%.2f,\"v_phys_mps\":%.2f,"
            "\"min_configured_max_speed_mps\":%.4f,\"numeric_bisection_mps\":%.4f,"
            "\"production_configured_mps\":%.2f,\"production_sufficient\":%s}",
            first_e ? "" : ",\n", T, s0, c.v_phys, analytic, hi, kConfiguredMaxSpeed,
            (kConfiguredMaxSpeed >= analytic - 1e-9) ? "true" : "false");
          first_e = false;
        }
      }
    }
  }
  std::printf("\n    ],\n");
  std::printf("    \"rows_note\": \"rows listed for T = %.1f s, the full production "
    "window; T = 2.0 and 3.0 were also bisected and agreed with the closed form\",\n",
    kMaxTotalTime);
  std::printf("    \"closed_form_vs_bisection_max_abs_err_mps\": %.3e\n", worst_bisect_err);
  std::printf("  },\n");

  // ==================================================================
  // Worked example, printed with every intermediate term
  // ==================================================================
  {
    const double s0 = 0.0, v_phys = 1.4, T = kMaxTotalTime;
    const double tcm = (kConfiguredMaxSpeed + s0) / kAmax;
    const double tcp = (v_phys + s0) / kAmax;
    const double bound = reachRadius(T, s0, B);
    const double truth = reachClosedForm(T, s0, v_phys, kAmax);
    std::printf("  \"worked_example\": {\n");
    std::printf("    \"s0_mps\": %.2f, \"v_phys_mps\": %.2f, \"total_time_s\": %.1f,\n",
      s0, v_phys, T);
    std::printf("    \"model_t_c_s\": %.4f, \"model_quadratic_part_m\": %.6f, "
      "\"model_linear_part_m\": %.6f, \"model_bound_m\": %.6f,\n",
      tcm, 0.5 * kAmax * tcm * tcm, (kConfiguredMaxSpeed + s0) * (T - tcm), bound);
    std::printf("    \"phys_t_c_s\": %.4f, \"true_quadratic_part_m\": %.6f, "
      "\"true_linear_part_m\": %.6f, \"true_reach_m\": %.6f,\n",
      tcp, 0.5 * kAmax * tcp * tcp, (v_phys + s0) * (T - tcp), truth);
    std::printf("    \"escape_m\": %.6f,\n", truth - bound);
    std::printf("    \"simulated_true_reach_m\": %.6f\n",
      simulateWorstCase(T, s0, v_phys, kAmax, kDtFine));
    std::printf("  },\n");
  }

  // ==================================================================
  // Monotonicity in s0 -- the separate, benign effect
  // ==================================================================
  bool mono_s0 = true;
  for (double s0 = 0.0; s0 < 5.0; s0 += 0.01) {
    for (double t : {0.5, 1.0, 2.0, 3.0, 4.0}) {
      if (reachRadius(t, s0 + 0.01, B) < reachRadius(t, s0, B) - 1e-15) {
        mono_s0 = false;
      }
    }
  }
  std::printf("  \"r_det_monotone_increasing_in_s0\": %s,\n", mono_s0 ? "true" : "false");
  std::printf("  \"monotonicity_implication\": \"an OVER-estimated filtered speed only "
    "enlarges the bound, so estimation overshoot is conservative, not a violation. This "
    "is also precisely why the superseded probe could not detect under-coverage: feeding "
    "the adversary's capability in as s0 inflated the bound in step with the threat.\",\n");

  std::printf("  \"summary\": {\n");
  std::printf("    \"in_domain_escapes\": %ld,\n", in_escapes);
  std::printf("    \"max_escape_m\": %.6f,\n", grid_max_escape);
  std::printf("    \"max_escape_total_time_s\": %.1f,\n", grid_max_escape_t);
  std::printf("    \"max_escape_s0_mps\": %.2f,\n", grid_max_escape_s0);
  std::printf("    \"max_escape_v_phys_mps\": %.2f,\n", grid_max_escape_v);
  std::printf("    \"earliest_onset_in_production_window_s\": %.4f\n",
    kConfiguredMaxSpeed / kAmax);
  std::printf("  }\n");
  std::printf("}\n");
  return 0;
}
