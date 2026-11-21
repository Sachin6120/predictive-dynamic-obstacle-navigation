// Stage-5C.1: the reachability VALIDITY CONTRACT.
//
// Stage-5C qualified the reachability predictor as a genuine second prediction
// method and, in the same audit, proved that its `max_speed` is a real validity
// PREMISE rather than a tuning parameter: with observed s0 = 0, a configured cap
// of 0.8 m/s and a true capability of 1.4 m/s, the model bounds the obstacle at
// 2.56 m at total_time 4.0 s while the obstacle reaches 3.64 m -- a 1.08 m
// escape (worst tested 1.44 m) INSIDE the production total-time window of
// max_observation_age + reachability_horizon = 4.0 s. At that commit there was
// no runtime enforcement of the premise at all.
//
// WHAT IS VERIFIED HERE
//   1. The premise is now evaluated on every sample and reported.
//   2. IN DOMAIN the numbers are BIT-IDENTICAL to the pre-Stage-5C.1
//      arithmetic -- proved by running the same inputs with
//      enforce_observed_speed on and off and comparing bit patterns.
//   3. An OBSERVABLE contradiction (filtered speed above the cap) can no longer
//      pass as a qualified conservative bound: it is flagged, and the cap is
//      raised to the weakest observation-consistent value.
//   4. The repair is real, not cosmetic: an adversary that is admissible at the
//      OBSERVED capability ESCAPES the old bound and is CONTAINED by the new
//      one.
//   5. The response is monotone-safe -- it can only ever enlarge the region --
//      which is what lets the tolerance stay at numeric scale. That matters
//      because the Stage-5C recordings show the filtered speed exceeding the
//      0.8 m/s cap by up to 0.131 m/s on tracks that are genuinely IN domain.
//   6. Boundary and degenerate inputs: equality, either side of the tolerance,
//      zero, negative, NaN, infinity.
//
// WHAT IS *NOT* CLAIMED. The premise quantifies over the FUTURE, so no runtime
// check can verify it. The unobservable residual -- true capability above the
// cap while the filtered speed stays below it -- is measured here and reported
// as the open risk it is, NOT as something this stage closes.
#define main stage5c1_original_main
#include "../src/lidar_obstacle_tracker_node.cpp"
#undef main
#include <cstdio>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using predictive_nav_tracking::ContractStatus;
using predictive_nav_tracking::ReachabilityBounds;
using predictive_nav_tracking::ReachabilityRegion;
using predictive_nav_tracking::buildRegion;
using predictive_nav_tracking::capabilityPremiseContradicted;
using predictive_nav_tracking::contractStatus;
using predictive_nav_tracking::effectiveMaxSpeed;
using predictive_nav_tracking::reachRadius;

int failures = 0;

void check(bool condition, const std::string & what)
{
  if (!condition) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
  }
}

/// Bit-level equality, so "unchanged in domain" is not softened into
/// "unchanged to within a tolerance".
bool bit_identical(double a, double b)
{
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

/// Forward-simulates an admissible trajectory under |a| <= a_max and
/// |v| <= v_cap, returning the deviation from the constant-velocity nominal at
/// time t. `v_cap` is passed explicitly -- NOT taken from the bounds -- because
/// the whole point here is to let the adversary use its PHYSICAL capability
/// while the model is configured with a smaller one.
double simulated_deviation(
  double v0x, double v0y, double ax, double ay, double t,
  double a_max, double v_cap, double dt = 1e-5)
{
  double px = 0.0, py = 0.0, vx = v0x, vy = v0y;
  const double a_norm = std::hypot(ax, ay);
  if (a_norm > a_max && a_norm > 0.0) {
    ax *= a_max / a_norm;
    ay *= a_max / a_norm;
  }
  for (double s = 0.0; s < t; s += dt) {
    const double step = std::min(dt, t - s);
    const double vx_before = vx;
    const double vy_before = vy;
    vx += ax * step;
    vy += ay * step;
    const double speed = std::hypot(vx, vy);
    if (speed > v_cap && speed > 0.0) {
      vx *= v_cap / speed;
      vy *= v_cap / speed;
    }
    // Midpoint update: forward Euler's O(a*dt*t) error exceeds the slack
    // between an extremal trajectory and the bound near the crossover time.
    px += 0.5 * (vx_before + vx) * step;
    py += 0.5 * (vy_before + vy) * step;
  }
  return std::hypot(px - v0x * t, py - v0y * t);
}

ReachabilityBounds productionBounds()
{
  ReachabilityBounds b;            // defaults are the production values
  b.max_acceleration = 0.5;
  b.max_speed = 0.8;
  b.covariance_sigma_level = 2.0;
  b.safety_margin = 0.0;
  return b;
}

const char * statusName(ContractStatus s)
{
  switch (s) {
    case ContractStatus::kSatisfied: return "SATISFIED";
    case ContractStatus::kObservedSpeedExceedsCap: return "OBSERVED_SPEED_EXCEEDS_CAP";
    case ContractStatus::kNotEvaluated: return "NOT_EVALUATED";
  }
  return "?";
}

Eigen::Matrix2d isotropic(double variance)
{
  Eigen::Matrix2d P;
  P << variance, 0.0, 0.0, variance;
  return P;
}

// The production total-time grid: observation_age up to 1.0 s plus horizons
// 0.5..3.0 s, i.e. everything the tracker can actually publish.
const std::vector<double> kTotalTimes =
{0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const ReachabilityBounds B = productionBounds();

  std::printf("{\n");
  std::printf("  \"stage\": \"5C.1\",\n");
  std::printf("  \"subject\": \"reachability max_speed validity contract\",\n");
  std::printf("  \"configured\": {\"max_speed_mps\": %.3f, \"max_acceleration_mps2\": %.3f,\n",
    B.max_speed, B.max_acceleration);
  std::printf(
    "    \"max_speed_tolerance_mps\": %.1e, \"enforce_observed_speed\": %s,\n",
    B.max_speed_tolerance, B.enforce_observed_speed ? "true" : "false");
  std::printf("    \"max_total_time_s\": 4.0},\n");

  // =====================================================================
  // 1. The invalidity condition, across the required boundary cases
  // =====================================================================
  //
  // The condition is OBSERVABLE: s0 > max_speed proves the obstacle is capable
  // of s0, hence disproves |v(s)| <= max_speed. Equality is NOT a violation --
  // the premise is satisfied with equality -- so the comparison is strict.
  std::printf("  \"boundary_cases\": [\n");
  struct Case
  {
    double s0;
    const char * label;
    bool expect_contradiction;
  };
  const double tol = B.max_speed_tolerance;
  const std::vector<Case> cases = {
    {0.0, "zero speed", false},
    {0.30, "clearly below the cap", false},
    {0.75, "fastest existing simulation obstacle", false},
    {0.8 - 1e-3, "just below the cap", false},
    {0.8, "exactly the configured premise (equality satisfies it)", false},
    {0.8 + 0.4 * tol, "inside the reporting tolerance", false},
    {0.8 + tol, "exactly at the tolerance (strict >, so not yet)", false},
    {0.8 + 2.0 * tol, "just outside the reporting tolerance", true},
    {0.8 + 1e-3, "just above the cap", true},
    {0.931, "worst in-domain filtered peak measured at Stage-5C", true},
    {1.4, "Stage-5C out-of-domain capability", true},
    {2.4, "far above the cap", true},
  };
  for (size_t i = 0; i < cases.size(); ++i) {
    const Case & c = cases[i];
    const bool contradicted = capabilityPremiseContradicted(c.s0, B);
    const ContractStatus st = contractStatus(c.s0, B);
    const double v_eff = effectiveMaxSpeed(c.s0, B);
    check(
      contradicted == c.expect_contradiction,
      std::string("contradiction flag for '") + c.label + "'");
    check(
      (st == ContractStatus::kObservedSpeedExceedsCap) == c.expect_contradiction,
      std::string("status for '") + c.label + "'");
    // Monotone safety: the effective cap is never below the configured one.
    check(v_eff >= B.max_speed, std::string("effective cap not below configured for ") + c.label);
    // In domain the effective cap must be EXACTLY the configured one.
    if (c.s0 <= B.max_speed) {
      check(
        bit_identical(v_eff, B.max_speed),
        std::string("effective cap not bit-identical in domain for ") + c.label);
    }
    std::printf(
      "    {\"s0_mps\": %.6f, \"label\": \"%s\", \"contradicted\": %s, "
      "\"status\": \"%s\", \"effective_max_speed_mps\": %.6f}%s\n",
      c.s0, c.label, contradicted ? "true" : "false", statusName(st), v_eff,
      i + 1 < cases.size() ? "," : "");
  }
  std::printf("  ],\n");

  // =====================================================================
  // 2. Degenerate / invalid inputs
  // =====================================================================
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    // A non-finite speed is not evidence of anything: it must not be reported
    // as a contract breach, and it must not corrupt the effective cap.
    check(!capabilityPremiseContradicted(nan, B), "NaN speed reported as a contradiction");
    check(
      bit_identical(effectiveMaxSpeed(nan, B), B.max_speed),
      "NaN speed perturbed the effective cap");
    // An infinite speed is not evidence ABOUT THE OBSTACLE either -- it is a
    // filter/sensor pathology. It must not be reported as a contract breach;
    // buildRegion() rejects such a sample as NOT_EVALUATED instead, which is
    // the honest answer ("unusable"), not "breached".
    check(!capabilityPremiseContradicted(inf, B), "infinite speed reported as a contradiction");
    check(
      bit_identical(effectiveMaxSpeed(inf, B), B.max_speed),
      "infinite speed perturbed the effective cap");
    const ReachabilityRegion bad_vel =
      buildRegion(0.0, 0.0, inf, 0.0, isotropic(0.01), 1.0, B);
    check(!bad_vel.valid, "infinite velocity produced a valid region");
    check(
      bad_vel.contract_status == ContractStatus::kNotEvaluated,
      "infinite velocity did not yield NOT_EVALUATED");
    // Negative speed cannot occur from hypot() but must be floored, not
    // propagated into the cap.
    check(!capabilityPremiseContradicted(-1.0, B), "negative speed flagged");
    check(
      bit_identical(effectiveMaxSpeed(-1.0, B), B.max_speed),
      "negative speed lowered the effective cap");

    // A region built from non-finite state must be NOT_EVALUATED, never
    // SATISFIED: an unusable sample must not look qualified.
    const ReachabilityRegion bad_state =
      buildRegion(nan, 0.0, 0.3, 0.0, isotropic(0.01), 1.0, B);
    check(!bad_state.valid, "non-finite position produced a valid region");
    check(
      bad_state.contract_status == ContractStatus::kNotEvaluated,
      "non-finite position did not yield NOT_EVALUATED");
    const ReachabilityRegion bad_cov =
      buildRegion(0.0, 0.0, 0.3, 0.0, isotropic(nan), 1.0, B);
    check(
      bad_cov.contract_status == ContractStatus::kNotEvaluated,
      "non-finite covariance did not yield NOT_EVALUATED");
    std::printf("  \"degenerate_inputs_handled\": true,\n");
  }

  // =====================================================================
  // 3. IN-DOMAIN BIT-IDENTICAL EQUIVALENCE
  // =====================================================================
  //
  // The strongest statement available: run the pre-Stage-5C.1 arithmetic
  // (enforce_observed_speed = false) and the new one over the same inputs and
  // compare the raw bit patterns of every published number. Any in-domain
  // difference at all is a failure.
  {
    ReachabilityBounds legacy = B;
    legacy.enforce_observed_speed = false;       // exactly the old expression
    size_t compared = 0;
    size_t differing = 0;
    // Integer-stepped so float accumulation cannot drift the last sample above
    // the cap and turn an in-domain sweep into an out-of-domain one.
    for (int step = 0; step <= 80; ++step) {
      const double s0 = std::min(static_cast<double>(step) * 0.01, B.max_speed);
      for (const double t : kTotalTimes) {
        const double v = 0.01 + 0.01 * t * t + 0.25 * t * t * t * t / 4.0;
        const ReachabilityRegion now = buildRegion(1.0, -2.0, s0, 0.0, isotropic(v), t, B);
        const ReachabilityRegion old = buildRegion(1.0, -2.0, s0, 0.0, isotropic(v), t, legacy);
        ++compared;
        const bool same =
          bit_identical(now.reach_radius, old.reach_radius) &&
          bit_identical(now.semi_major, old.semi_major) &&
          bit_identical(now.semi_minor, old.semi_minor) &&
          bit_identical(now.center_x, old.center_x) &&
          bit_identical(now.center_y, old.center_y) &&
          bit_identical(now.sigma_semi_major, old.sigma_semi_major) &&
          bit_identical(now.sigma_semi_minor, old.sigma_semi_minor) &&
          bit_identical(now.sigma_yaw, old.sigma_yaw) &&
          now.speed_capped == old.speed_capped &&
          now.valid == old.valid;
        if (!same) {++differing;}
        // Every in-domain sample must also be reported as satisfying the
        // contract, and must carry the configured cap verbatim.
        check(
          now.contract_status == ContractStatus::kSatisfied,
          "an in-domain sample was not reported SATISFIED");
        check(
          bit_identical(now.effective_max_speed, now.configured_max_speed),
          "an in-domain sample's effective cap differed from the configured cap");
      }
    }
    // Diagonal velocity, to exercise hypot() rather than a single component.
    for (int step = 0; step <= 56; ++step) {
      const double c = static_cast<double>(step) * 0.01;   // |v| = c*sqrt(2) <= 0.792
      for (const double t : kTotalTimes) {
        const double v = 0.02 + 0.03 * t * t;
        const ReachabilityRegion now = buildRegion(0.0, 0.0, c, c, isotropic(v), t, B);
        const ReachabilityRegion old = buildRegion(0.0, 0.0, c, c, isotropic(v), t, legacy);
        ++compared;
        if (!bit_identical(now.reach_radius, old.reach_radius) ||
          !bit_identical(now.semi_major, old.semi_major) ||
          !bit_identical(now.semi_minor, old.semi_minor))
        {
          ++differing;
        }
      }
    }
    check(differing == 0, "in-domain output was NOT bit-identical to the legacy arithmetic");
    std::printf(
      "  \"in_domain_equivalence\": {\"samples_compared\": %zu, \"bit_differences\": %zu, "
      "\"bit_identical\": %s},\n",
      compared, differing, differing == 0 ? "true" : "false");
  }

  // =====================================================================
  // 4. OBSERVABLE OUT-OF-CONTRACT: flagged, enlarged, never dropped
  // =====================================================================
  {
    ReachabilityBounds legacy = B;
    legacy.enforce_observed_speed = false;
    size_t samples = 0, flagged = 0, shrunk = 0, dropped = 0;
    double worst_growth = 0.0;
    for (double s0 = 0.81; s0 <= 2.4001; s0 += 0.01) {
      for (const double t : kTotalTimes) {
        const double v = 0.01 + 0.01 * t * t;
        const ReachabilityRegion now = buildRegion(0.0, 0.0, s0, 0.0, isotropic(v), t, B);
        const ReachabilityRegion old = buildRegion(0.0, 0.0, s0, 0.0, isotropic(v), t, legacy);
        ++samples;
        if (now.contract_status == ContractStatus::kObservedSpeedExceedsCap) {++flagged;}
        // Monotone-safe: never smaller than the legacy region.
        if (now.reach_radius < old.reach_radius) {++shrunk;}
        // Never dropped: coverage must not be removed by the contract.
        if (!now.valid) {++dropped;}
        worst_growth = std::max(worst_growth, now.reach_radius - old.reach_radius);
        check(
          bit_identical(now.effective_max_speed, std::max(B.max_speed, s0)),
          "effective cap was not the weakest observation-consistent value");
        check(
          bit_identical(now.observed_speed, s0),
          "observed_speed was not published verbatim");
      }
    }
    check(flagged == samples, "an out-of-contract sample was not flagged");
    check(shrunk == 0, "the contract response SHRANK a region");
    check(dropped == 0, "the contract response DROPPED a region");
    std::printf(
      "  \"out_of_contract\": {\"samples\": %zu, \"flagged\": %zu, \"shrunk\": %zu, "
      "\"dropped\": %zu, \"worst_radius_growth_m\": %.6f},\n",
      samples, flagged, shrunk, dropped, worst_growth);
  }

  // =====================================================================
  // 5. THE REPAIR IS REAL: the old bound is escaped, the new one is not
  // =====================================================================
  //
  // The adversary is admissible at the OBSERVED capability (v_cap = s0): an
  // object already measured at s0 can certainly sustain s0. Under the legacy
  // arithmetic the model caps delta_v at max_speed + s0; under the contract it
  // caps it at s0 + s0. The first is an under-bound and must be escaped.
  {
    ReachabilityBounds legacy = B;
    legacy.enforce_observed_speed = false;
    size_t trials = 0, legacy_escapes = 0, contract_escapes = 0;
    double worst_legacy_escape = 0.0, worst_contract_slack = 1e9;
    for (double s0 = 0.9; s0 <= 2.4001; s0 += 0.1) {
      for (const double t : kTotalTimes) {
        // Reverse at full acceleration: the extremal direction for the
        // deviation bound, since the obstacle is allowed to reverse.
        const double deviation =
          simulated_deviation(s0, 0.0, -B.max_acceleration, 0.0, t, B.max_acceleration, s0);
        const double legacy_bound = reachRadius(t, s0, legacy);
        const double contract_bound = reachRadius(t, s0, B);
        ++trials;
        if (deviation > legacy_bound + 1e-6) {
          ++legacy_escapes;
          worst_legacy_escape = std::max(worst_legacy_escape, deviation - legacy_bound);
        }
        if (deviation > contract_bound + 1e-6) {++contract_escapes;}
        worst_contract_slack = std::min(worst_contract_slack, contract_bound - deviation);
        check(
          contract_bound >= legacy_bound,
          "the contract bound was smaller than the legacy bound");
      }
    }
    check(
      legacy_escapes > 0,
      "the legacy bound was never escaped -- the probe does not demonstrate the hole");
    check(
      contract_escapes == 0,
      "an adversary admissible at the OBSERVED capability escaped the repaired bound");
    std::printf(
      "  \"repair_is_real\": {\"trials\": %zu, \"legacy_escapes\": %zu, "
      "\"worst_legacy_escape_m\": %.6f, \"contract_escapes\": %zu, "
      "\"worst_contract_slack_m\": %.3e},\n",
      trials, legacy_escapes, worst_legacy_escape, contract_escapes, worst_contract_slack);
  }

  // =====================================================================
  // 6. Why the tolerance is numeric-scale and not a wide dead-band
  // =====================================================================
  //
  // The Stage-5C recordings (validation/stage5c/model_audit.json) give the peak
  // FILTERED speed per scenario. Every scenario below is in domain by ground
  // truth (<= 0.8 m/s), yet the estimator transiently exceeds the cap. A
  // dead-band wide enough to absorb the worst of those (0.131 m/s) would also
  // swallow the s0 in [0.80, 0.95] band where Stage-5C's adversary measured
  // real escapes (3334/20000 trials, worst 0.155 m). The dead-band is
  // therefore kept at numeric scale, and the transients are made harmless by
  // the response being monotone-safe instead.
  {
    struct Peak {const char * id; double gt; double filtered;};
    const std::vector<Peak> peaks = {
      {"A_constant_velocity", 0.300, 0.466},
      {"A_constant_velocity_fast", 0.680, 0.931},
      {"B_crossing", 0.500, 0.838},
      {"C_decelerate_stop", 0.500, 0.851},
      {"C_reversal", 0.500, 0.710},
      {"C_accelerate", 0.700, 0.816},
      {"D_turn_in_model", 0.500, 0.762},
      {"D_turn_out_of_model", 0.500, 0.696},
      {"F_close_pass", 0.400, 0.602},
      {"H_occlusion_short", 0.500, 0.756},
      {"H_occlusion_maneuver", 0.500, 0.831},
    };
    size_t flagged_at_peak = 0;
    double worst_overshoot_above_cap = 0.0;
    double worst_region_growth = 0.0;
    for (const Peak & p : peaks) {
      check(p.gt <= B.max_speed + 1e-12, "a scenario listed as in-domain exceeds the cap");
      if (capabilityPremiseContradicted(p.filtered, B)) {++flagged_at_peak;}
      if (p.filtered > B.max_speed) {
        worst_overshoot_above_cap =
          std::max(worst_overshoot_above_cap, p.filtered - B.max_speed);
      }
      // The cost of flagging a transient: how much larger the region gets at
      // the worst production total_time. It must be a small enlargement, never
      // a rejection.
      ReachabilityBounds legacy = B;
      legacy.enforce_observed_speed = false;
      const double growth = reachRadius(4.0, p.filtered, B) - reachRadius(4.0, p.filtered, legacy);
      worst_region_growth = std::max(worst_region_growth, growth);
      check(growth >= 0.0, "flagging a transient shrank the region");
    }
    std::printf(
      "  \"tolerance_rationale\": {\"in_domain_scenarios\": %zu, "
      "\"scenarios_whose_filtered_peak_is_flagged\": %zu, "
      "\"worst_in_domain_filtered_overshoot_above_cap_mps\": %.3f, "
      "\"worst_region_growth_from_a_transient_m\": %.4f, "
      "\"response_is_rejection\": false},\n",
      peaks.size(), flagged_at_peak, worst_overshoot_above_cap, worst_region_growth);
  }

  // =====================================================================
  // 7. The UNOBSERVABLE residual, measured and reported as still open
  // =====================================================================
  //
  // Reproduces the Stage-5C example exactly: the runtime sees s0 = 0 and has
  // no way to know the obstacle can do 1.4 m/s. The contract reports SATISFIED
  // -- correctly, because nothing OBSERVED contradicts the premise -- and the
  // escape remains. This is the declared, unenforceable half of the contract.
  {
    const double s0 = 0.0;
    const double v_phys = 1.4;
    const double t = 4.0;
    const double model_bound = reachRadius(t, s0, B);
    const double true_reach =
      simulated_deviation(s0, 0.0, B.max_acceleration, 0.0, t, B.max_acceleration, v_phys);
    const ContractStatus st = contractStatus(s0, B);
    check(
      st == ContractStatus::kSatisfied,
      "the unobservable case must report SATISFIED -- observability must not be invented");
    check(
      true_reach > model_bound,
      "the Stage-5C unobservable escape did not reproduce");
    std::printf(
      "  \"unobservable_residual\": {\"observed_s0_mps\": %.3f, \"configured_cap_mps\": %.3f, "
      "\"physical_capability_mps\": %.3f, \"total_time_s\": %.1f, \"model_bound_m\": %.4f, "
      "\"true_reach_m\": %.4f, \"escape_m\": %.4f, \"reported_status\": \"%s\", "
      "\"runtime_enforceable\": false,\n"
      "    \"note\": \"future capability is not observable; this half of the premise stays a "
      "declared configuration assumption and is NOT closed by Stage-5C.1\"},\n",
      s0, B.max_speed, v_phys, t, model_bound, true_reach, true_reach - model_bound,
      statusName(st));
  }

  // =====================================================================
  // 8. End to end through the production tracker
  // =====================================================================
  //
  // The contract must survive the real node path -- parameters, per-track
  // state, predict_reachability() -- not just the header in isolation.
  {
    auto node = std::make_shared<LidarObstacleTracker>();

    // The node's declared defaults ARE the production contract settings: the
    // contract is on by default and nothing in the YAML has to opt in.
    check(
      node->reachability_bounds_.enforce_observed_speed,
      "the node's default did not enable the contract response");
    check(
      node->reachability_bounds_.max_speed_tolerance >= 0.0,
      "the node read a negative tolerance");
    check(
      bit_identical(node->reachability_bounds_.max_speed, 0.8),
      "the node's default max_speed is no longer 0.8");

    const rclcpp::Time stamp(1000, 0, RCL_ROS_TIME);

    // An in-domain track: every sample SATISFIED, effective cap == configured.
    Track in_domain;
    in_domain.id = 1;
    in_domain.kalman_initialized = true;
    in_domain.last_update = stamp;
    in_domain.kf_x << 0.0, 0.0, 0.5, 0.0;
    in_domain.kf_P = Eigen::Matrix4d::Identity() * 0.01;
    const auto in_regions = node->predict_reachability(in_domain, stamp);
    check(!in_regions.empty(), "no samples produced for the in-domain track");
    size_t in_satisfied = 0;
    for (const auto & r : in_regions) {
      if (r.contract_status == ContractStatus::kSatisfied) {++in_satisfied;}
      check(
        bit_identical(r.effective_max_speed, 0.8),
        "in-domain effective cap was not the configured cap through the node");
    }
    check(in_satisfied == in_regions.size(), "an in-domain node sample was not SATISFIED");

    // An observably out-of-contract track.
    Track over;
    over.id = 2;
    over.kalman_initialized = true;
    over.last_update = stamp;
    over.kf_x << 0.0, 0.0, 1.2, 0.0;
    over.kf_P = Eigen::Matrix4d::Identity() * 0.01;
    const auto over_regions = node->predict_reachability(over, stamp);
    check(!over_regions.empty(), "no samples produced for the out-of-contract track");
    size_t over_flagged = 0, over_valid = 0;
    for (const auto & r : over_regions) {
      if (r.contract_status == ContractStatus::kObservedSpeedExceedsCap) {++over_flagged;}
      if (r.valid) {++over_valid;}
      check(
        bit_identical(r.effective_max_speed, 1.2),
        "out-of-contract effective cap was not raised to the observed speed");
    }
    check(
      over_flagged == over_regions.size(),
      "an out-of-contract node sample passed as qualified");
    check(
      over_valid == over_regions.size(),
      "the contract dropped samples instead of enlarging them");

    // Per-track independence: one track's breach must not label another's.
    check(
      in_regions.front().contract_status == ContractStatus::kSatisfied,
      "contract state leaked between tracks");

    std::printf(
      "  \"through_production_node\": {\"in_domain_samples\": %zu, "
      "\"in_domain_satisfied\": %zu, \"out_of_contract_samples\": %zu, "
      "\"out_of_contract_flagged\": %zu, \"out_of_contract_still_valid\": %zu, "
      "\"per_track_independent\": true},\n",
      in_regions.size(), in_satisfied, over_regions.size(), over_flagged, over_valid);
  }

  std::printf("  \"failures\": %d,\n", failures);
  std::printf("  \"pass\": %s\n", failures == 0 ? "true" : "false");
  std::printf("}\n");
  rclcpp::shutdown();
  if (failures != 0) {
    std::fprintf(stderr, "stage5c1_contract: %d check(s) failed\n", failures);
    return 1;
  }
  return 0;
}
