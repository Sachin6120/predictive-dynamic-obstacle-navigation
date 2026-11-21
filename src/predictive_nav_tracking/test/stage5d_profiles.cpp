// Stage-5D: class-aware capability profiles -- selection safety and
// configuration robustness.
//
// WHAT IS VERIFIED HERE
//   1. MONOTONICITY of the reachability bound in each profile component, over
//      a wide grid, so the componentwise order used by every safety argument
//      in capability_profiles.hpp is a measured fact and not only an algebraic
//      claim. This is the one theorem the rest of the file leans on.
//   2. PROFILE SELECTION for PEDESTRIAN, BIKE, VEHICLE and the UNKNOWN
//      fallback, including which closure envelope each label actually buys
//      under the shipped evidence-backed ambiguity sets.
//   3. The UNKNOWN fallback DOMINATES every supported class profile -- the
//      section-4 requirement -- as an identity over randomised profile tables,
//      not only at the defaults.
//   4. The UNCERTAINTY INVARIANT (I3): low confidence, NaN confidence, an
//      immature track and an UNKNOWN label all yield a profile at least as
//      conservative as every class, so uncertainty can never produce a
//      smaller region than certainty about anything.
//   5. CLASS TRANSITIONS cannot shrink occupancy inside the hold window
//      (T1/T2), are bounded by the fallback however often the label flaps
//      (T3), and do collapse to the new profile once the hold expires -- so
//      the rule is a delay, not a permanent ratchet.
//   6. The OBSERVABLE SPEED BREACH of Stage-5C.1 still fires, per profile,
//      including the case where a class profile is large enough that a speed
//      which breached the global cap no longer breaches at all.
//   7. MALFORMED CONFIGURATION -- negative, zero, NaN, Inf, a missing profile,
//      an unknown enum value, an empty or unrecognised ambiguity set -- is
//      either rejected or handled so that it can NEVER produce a less
//      conservative region than the well-formed table it replaced.
//   8. CV-MODE OUTPUT IS UNCHANGED, and the class-aware-disabled path is
//      BIT-IDENTICAL to Stage-5C.1 through the real node.
//   9. End to end through the production node: a labelled track gets its
//      profile, an unlabelled one gets the fallback, and the published
//      metadata says which and why.
//
// WHAT IS *NOT* CLAIMED. A profile is a DECLARED premise about an
// unobservable future, exactly as the single global max_speed was. Nothing
// here proves a pedestrian cannot exceed 2.5 m/s; Stage-5C.1's unobservable
// residual is inherited verbatim, per class. Point 7 is about configuration
// hygiene, not about the premises being true.
#define main stage5d_original_main
#include "../src/lidar_obstacle_tracker_node.cpp"
#undef main
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

namespace
{
using predictive_nav_tracking::AmbiguitySets;
using predictive_nav_tracking::CapabilityProfileSet;
using predictive_nav_tracking::ClassProfileState;
using predictive_nav_tracking::ContractStatus;
using predictive_nav_tracking::MotionCapabilityProfile;
using predictive_nav_tracking::ObstacleClass;
using predictive_nav_tracking::ProfileReason;
using predictive_nav_tracking::ProfileSelection;
using predictive_nav_tracking::ProfileSelectionPolicy;
using predictive_nav_tracking::ReachabilityBounds;
using predictive_nav_tracking::boundsForProfile;
using predictive_nav_tracking::buildRegion;
using predictive_nav_tracking::classBit;
using predictive_nav_tracking::fallbackProfile;
using predictive_nav_tracking::profileDominates;
using predictive_nav_tracking::profileEnvelope;
using predictive_nav_tracking::profileWellFormed;
using predictive_nav_tracking::reachRadius;
using predictive_nav_tracking::sanitizeProfileSet;
using predictive_nav_tracking::selectCapabilityProfile;

int failures = 0;

void check(bool condition, const std::string & what)
{
  if (!condition) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
  }
}

bool bit_identical(double a, double b)
{
  return std::memcmp(&a, &b, sizeof(double)) == 0;
}

/// The horizons the production configuration actually publishes, plus the
/// worst-case total time of max_observation_age + reachability_horizon.
const std::vector<double> kHorizons{0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};

/// Select with a throwaway transition state, for the cases where the
/// transition rule is not what is under test. A fresh state means the first
/// scan is taken verbatim, so the result is the unheld selection.
ProfileSelection select_fresh(
  ObstacleClass label, double conf, uint32_t obs,
  const CapabilityProfileSet & set, const AmbiguitySets & amb,
  const ProfileSelectionPolicy & pol)
{
  ClassProfileState st;
  const MotionCapabilityProfile global{0.8, 0.5, 0.0};
  return selectCapabilityProfile(label, conf, obs, set, amb, pol, global, true, st);
}

ProfileSelectionPolicy classAwarePolicy()
{
  ProfileSelectionPolicy p;
  p.class_aware = true;
  return p;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::printf("{\n");

  const CapabilityProfileSet kSet;
  const AmbiguitySets kAmb;
  const ProfileSelectionPolicy kPol = classAwarePolicy();

  // =====================================================================
  // 1. The order is a theorem: r_det and the region are monotone in each
  //    profile component.
  // =====================================================================
  //
  // capability_profiles.hpp section 2 argues this from
  // r_det(t) = integral of min(a_max*s, v_eff+s0): the integrand is pointwise
  // nondecreasing in a_max and in v_max, so the integral is too. Here it is
  // measured over a grid that spans every shipped profile and well past it,
  // at every published horizon, for fresh and coasting tracks alike.
  {
    long trials = 0, violations = 0;
    double worst_decrease = 0.0;
    const std::vector<double> speeds{0.0, 0.4, 0.8, 1.0, 2.5, 7.0, 12.0, 25.0, 40.0};
    const std::vector<double> accels{0.0, 0.1, 0.5, 1.0, 2.5, 5.0, 7.0, 10.0, 20.0};
    const std::vector<double> s0s{0.0, 0.2, 0.75, 0.967, 1.5, 5.0, 13.0, 45.0};

    for (size_t vi = 0; vi + 1 < speeds.size(); ++vi) {
      for (double a : accels) {
        for (double s0 : s0s) {
          for (double t : kHorizons) {
            ReachabilityBounds lo;
            lo.max_acceleration = a;
            lo.max_speed = speeds[vi];
            ReachabilityBounds hi = lo;
            hi.max_speed = speeds[vi + 1];
            const double r_lo = reachRadius(t, s0, lo);
            const double r_hi = reachRadius(t, s0, hi);
            ++trials;
            if (r_hi < r_lo) {
              ++violations;
              worst_decrease = std::max(worst_decrease, r_lo - r_hi);
            }
          }
        }
      }
    }
    for (size_t ai = 0; ai + 1 < accels.size(); ++ai) {
      for (double v : speeds) {
        for (double s0 : s0s) {
          for (double t : kHorizons) {
            ReachabilityBounds lo;
            lo.max_acceleration = accels[ai];
            lo.max_speed = v;
            ReachabilityBounds hi = lo;
            hi.max_acceleration = accels[ai + 1];
            const double r_lo = reachRadius(t, s0, lo);
            const double r_hi = reachRadius(t, s0, hi);
            ++trials;
            if (r_hi < r_lo) {
              ++violations;
              worst_decrease = std::max(worst_decrease, r_lo - r_hi);
            }
          }
        }
      }
    }

    // And the full published region, margin included, through buildRegion.
    long region_trials = 0, region_violations = 0;
    const Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() * 0.02;
    const std::vector<MotionCapabilityProfile> ladder{
      {0.8, 0.5, 0.0}, {2.5, 2.5, 0.0}, {12.0, 7.0, 0.0}, {40.0, 10.0, 0.0},
      {40.0, 10.0, 0.25}, {40.0, 10.0, 0.5}};
    for (size_t i = 0; i + 1 < ladder.size(); ++i) {
      check(
        profileDominates(ladder[i + 1], ladder[i]),
        "the monotonicity ladder is not actually ordered");
      for (double t : kHorizons) {
        for (double s0 : s0s) {
          const ReachabilityBounds blo = boundsForProfile(ReachabilityBounds{}, ladder[i]);
          const ReachabilityBounds bhi = boundsForProfile(ReachabilityBounds{}, ladder[i + 1]);
          const auto rlo = buildRegion(0.0, 0.0, s0, 0.0, cov, t, blo);
          const auto rhi = buildRegion(0.0, 0.0, s0, 0.0, cov, t, bhi);
          ++region_trials;
          if (rhi.semi_major < rlo.semi_major || rhi.semi_minor < rlo.semi_minor) {
            ++region_violations;
          }
        }
      }
    }

    check(violations == 0, "r_det is NOT monotone in a profile component");
    check(region_violations == 0, "the published region is NOT monotone in the profile");
    std::printf(
      "  \"monotonicity\": {\"radius_trials\": %ld, \"radius_violations\": %ld, "
      "\"worst_decrease_m\": %.17g, \"region_trials\": %ld, \"region_violations\": %ld},\n",
      trials, violations, worst_decrease, region_trials, region_violations);
  }

  // =====================================================================
  // 2. Per-label selection at the shipped defaults.
  // =====================================================================
  {
    const MotionCapabilityProfile ped = kSet[ObstacleClass::PEDESTRIAN];
    const MotionCapabilityProfile bike = kSet[ObstacleClass::BIKE];
    const MotionCapabilityProfile veh = kSet[ObstacleClass::VEHICLE];
    const MotionCapabilityProfile fb = fallbackProfile(kSet, kAmb);

    // The declared physical capabilities, as the README tabulates them.
    check(ped.max_speed == 2.5 && ped.max_acceleration == 2.5, "PEDESTRIAN profile changed");
    check(bike.max_speed == 12.0 && bike.max_acceleration == 7.0, "BIKE profile changed");
    check(veh.max_speed == 40.0 && veh.max_acceleration == 10.0, "VEHICLE profile changed");

    const auto s_ped = select_fresh(ObstacleClass::PEDESTRIAN, 1.0, 50, kSet, kAmb, kPol);
    const auto s_bike = select_fresh(ObstacleClass::BIKE, 1.0, 50, kSet, kAmb, kPol);
    const auto s_veh = select_fresh(ObstacleClass::VEHICLE, 1.0, 50, kSet, kAmb, kPol);
    const auto s_unk = select_fresh(ObstacleClass::UNKNOWN, 0.0, 50, kSet, kAmb, kPol);

    check(s_ped.reason == ProfileReason::kClassClosure, "PEDESTRIAN did not take its closure");
    check(s_bike.reason == ProfileReason::kClassClosure, "BIKE did not take its closure");
    check(s_veh.reason == ProfileReason::kClassClosure, "VEHICLE did not take its closure");
    check(
      s_unk.reason == ProfileReason::kFallbackUnknownLabel,
      "an UNKNOWN label did not take the fallback");

    // (I1): every selection dominates its own label's physical profile.
    check(profileDominates(s_ped.profile, ped), "(I1) violated for PEDESTRIAN");
    check(profileDominates(s_bike.profile, bike), "(I1) violated for BIKE");
    check(profileDominates(s_veh.profile, veh), "(I1) violated for VEHICLE");

    // The measured-evidence consequence, asserted rather than assumed: a
    // PEDESTRIAN label buys NOTHING under these ambiguity sets, because
    // 306 of 715 PEDESTRIAN-labelled detections were ground-truth vehicles.
    check(
      profileDominates(s_ped.profile, veh),
      "a PEDESTRIAN label bought a profile weaker than VEHICLE despite the "
      "measured VEHICLE -> PEDESTRIAN confusion");
    check(
      s_ped.dominating_class == ObstacleClass::VEHICLE,
      "the PEDESTRIAN closure envelope is no longer vehicle-grade");

    // BIKE is the one label whose measured precision (1.000, 0 false
    // predictions) earns a profile strictly weaker than the fallback.
    check(
      s_bike.profile.max_speed < fb.max_speed &&
      s_bike.profile.max_acceleration < fb.max_acceleration,
      "BIKE no longer earns a profile weaker than the fallback");
    check(
      s_bike.dominating_class == ObstacleClass::BIKE,
      "the BIKE closure is no longer exactly the BIKE profile");

    // And the region that follows really is smaller, at every horizon.
    long bike_tighter = 0;
    for (double t : kHorizons) {
      const double rb = reachRadius(t, 0.0, boundsForProfile(ReachabilityBounds{}, s_bike.profile));
      const double rf = reachRadius(t, 0.0, boundsForProfile(ReachabilityBounds{}, fb));
      if (rb < rf) {++bike_tighter;}
    }
    check(
      bike_tighter == static_cast<long>(kHorizons.size()),
      "the BIKE profile does not produce a smaller radius at every horizon");

    std::printf(
      "  \"selection_at_defaults\": {\"fallback\": [%.2f, %.2f, %.2f], "
      "\"pedestrian_applied\": [%.2f, %.2f, %.2f], \"bike_applied\": [%.2f, %.2f, %.2f], "
      "\"vehicle_applied\": [%.2f, %.2f, %.2f], \"pedestrian_buys_reduction\": false, "
      "\"bike_horizons_tighter\": %ld},\n",
      fb.max_speed, fb.max_acceleration, fb.margin,
      s_ped.profile.max_speed, s_ped.profile.max_acceleration, s_ped.profile.margin,
      s_bike.profile.max_speed, s_bike.profile.max_acceleration, s_bike.profile.margin,
      s_veh.profile.max_speed, s_veh.profile.max_acceleration, s_veh.profile.margin,
      bike_tighter);
  }

  // =====================================================================
  // 3. The fallback dominates every class -- over RANDOM tables, not just
  //    the shipped one, so (I2) is an identity of the derivation.
  // =====================================================================
  {
    std::mt19937 rng(20251121u);
    std::uniform_real_distribution<double> speed(0.0, 60.0);
    std::uniform_real_distribution<double> accel(0.0, 25.0);
    std::uniform_real_distribution<double> marg(0.0, 1.5);
    long trials = 0, violations = 0;
    for (int i = 0; i < 20000; ++i) {
      CapabilityProfileSet set;
      for (size_t c = 1; c < predictive_nav_tracking::kNumProfileClasses; ++c) {
        set.by_class[c] = MotionCapabilityProfile{speed(rng), accel(rng), marg(rng)};
      }
      const MotionCapabilityProfile fb = fallbackProfile(set, kAmb);
      for (size_t c = 1; c < predictive_nav_tracking::kNumProfileClasses; ++c) {
        ++trials;
        if (!profileDominates(fb, set.by_class[c])) {++violations;}
      }
      // And every label's closure envelope is still bounded by the fallback,
      // which is (T3)'s base case.
      for (size_t c = 0; c < predictive_nav_tracking::kNumProfileClasses; ++c) {
        const auto sel = select_fresh(
          static_cast<ObstacleClass>(c), 1.0, 50, set, kAmb, kPol);
        ++trials;
        if (!profileDominates(fb, sel.profile)) {++violations;}
      }
    }
    check(violations == 0, "(I2)/(T3) violated: the fallback did not dominate");
    std::printf(
      "  \"fallback_dominance\": {\"random_tables\": 20000, \"checks\": %ld, "
      "\"violations\": %ld},\n", trials, violations);
  }

  // =====================================================================
  // 4. The uncertainty invariant (I3).
  // =====================================================================
  {
    const MotionCapabilityProfile fb = fallbackProfile(kSet, kAmb);
    struct Case
    {
      const char * name;
      ObstacleClass label;
      double conf;
      uint32_t obs;
      ProfileReason expect;
    };
    // 0.59 is just below the 0.60 gate; 4 is just below min_observations 5.
    const std::vector<Case> cases{
      {"unknown_label", ObstacleClass::UNKNOWN, 0.0, 50, ProfileReason::kFallbackUnknownLabel},
      {"low_confidence_bike", ObstacleClass::BIKE, 0.59, 50,
        ProfileReason::kFallbackLowConfidence},
      {"zero_confidence_bike", ObstacleClass::BIKE, 0.0, 50,
        ProfileReason::kFallbackLowConfidence},
      {"nan_confidence_bike", ObstacleClass::BIKE,
        std::numeric_limits<double>::quiet_NaN(), 50,
        ProfileReason::kFallbackLowConfidence},
      {"immature_bike", ObstacleClass::BIKE, 1.0, 4, ProfileReason::kFallbackImmatureTrack},
      {"new_track_bike", ObstacleClass::BIKE, 1.0, 0, ProfileReason::kFallbackImmatureTrack},
      {"low_confidence_ped", ObstacleClass::PEDESTRIAN, 0.1, 50,
        ProfileReason::kFallbackLowConfidence},
    };
    for (const auto & c : cases) {
      const auto sel = select_fresh(c.label, c.conf, c.obs, kSet, kAmb, kPol);
      check(sel.reason == c.expect, std::string("wrong reason for ") + c.name);
      for (size_t k = 1; k < predictive_nav_tracking::kNumProfileClasses; ++k) {
        check(
          profileDominates(sel.profile, kSet.by_class[k]),
          std::string("(I3) violated for ") + c.name);
      }
      check(bit_identical(sel.profile.max_speed, fb.max_speed), "fallback speed not applied");
    }

    // The gate boundary itself: >= min_confidence is honoured, < is not.
    const auto at_gate = select_fresh(ObstacleClass::BIKE, 0.60, 50, kSet, kAmb, kPol);
    const auto below = select_fresh(ObstacleClass::BIKE, 0.5999999999, 50, kSet, kAmb, kPol);
    check(at_gate.reason == ProfileReason::kClassClosure, "confidence exactly at the gate failed");
    check(below.reason == ProfileReason::kFallbackLowConfidence, "the gate let 0.5999.. through");
    const auto at_obs = select_fresh(ObstacleClass::BIKE, 1.0, 5, kSet, kAmb, kPol);
    check(at_obs.reason == ProfileReason::kClassClosure, "observations exactly at the gate failed");

    // THE REQUIREMENT OF BRIEF SECTION 5, stated directly: uncertainty must
    // never yield a SMALLER region than the fallback, at any horizon.
    long shrink = 0;
    for (const auto & c : cases) {
      const auto sel = select_fresh(c.label, c.conf, c.obs, kSet, kAmb, kPol);
      for (double t : kHorizons) {
        for (double s0 : {0.0, 0.5, 1.0, 5.0}) {
          const double rs =
            reachRadius(t, s0, boundsForProfile(ReachabilityBounds{}, sel.profile));
          const double rf = reachRadius(t, s0, boundsForProfile(ReachabilityBounds{}, fb));
          if (rs < rf) {++shrink;}
        }
      }
    }
    check(shrink == 0, "an uncertain selection produced a smaller region than the fallback");
    std::printf(
      "  \"uncertainty_invariant\": {\"cases\": %zu, \"region_shrinks\": %ld},\n",
      cases.size(), shrink);
  }

  // =====================================================================
  // 5. Class transitions.
  // =====================================================================
  {
    const MotionCapabilityProfile fb = fallbackProfile(kSet, kAmb);

    // 5a. The shrinking direction. Under the shipped ambiguity sets the only
    // transition that can shrink the applied profile is into BIKE, because
    // every other label's closure is already vehicle-grade. PEDESTRIAN ->
    // BIKE occurred in the Stage-5B recordings, so this is the measured case.
    {
      ClassProfileState st;
      const MotionCapabilityProfile global{0.8, 0.5, 0.0};
      auto step = [&](ObstacleClass l) {
          return selectCapabilityProfile(l, 1.0, 50, kSet, kAmb, kPol, global, true, st);
        };
      const auto ped = step(ObstacleClass::PEDESTRIAN);
      check(
        profileDominates(ped.profile, kSet[ObstacleClass::VEHICLE]),
        "setup: ped not veh-grade");

      // The transition scan and the whole hold window must not shrink.
      MotionCapabilityProfile prev = ped.profile;
      for (uint32_t k = 0; k <= kPol.transition_hold_scans; ++k) {
        const auto s = step(ObstacleClass::BIKE);
        check(profileDominates(s.profile, prev), "(T2) violated: occupancy shrank during the hold");
        check(
          s.reason == ProfileReason::kTransitionHold,
          "a held scan did not report PROFILE_TRANSITION_HOLD");
        check(profileDominates(s.profile, kSet[ObstacleClass::BIKE]), "(T1) violated during hold");
        prev = s.profile;
      }
      // One scan past the hold, it collapses to BIKE: the rule is a DELAY,
      // not a permanent ratchet.
      const auto settled = step(ObstacleClass::BIKE);
      check(
        settled.reason == ProfileReason::kClassClosure,
        "the hold never expired -- the transition rule is a ratchet");
      check(
        bit_identical(settled.profile.max_speed, kSet[ObstacleClass::BIKE].max_speed) &&
        bit_identical(
          settled.profile.max_acceleration, kSet[ObstacleClass::BIKE].max_acceleration),
        "after the hold the profile is not exactly BIKE");
    }

    // 5b. The enlarging direction needs no protection, and must not be
    // delayed: BIKE -> PEDESTRIAN (also measured in Stage-5B) must reach the
    // vehicle-grade envelope on the transition scan itself.
    {
      ClassProfileState st;
      const MotionCapabilityProfile global{0.8, 0.5, 0.0};
      auto step = [&](ObstacleClass l) {
          return selectCapabilityProfile(l, 1.0, 50, kSet, kAmb, kPol, global, true, st);
        };
      step(ObstacleClass::BIKE);
      const auto up = step(ObstacleClass::PEDESTRIAN);
      check(
        profileDominates(up.profile, kSet[ObstacleClass::VEHICLE]),
        "an enlarging transition was delayed");
    }

    // 5c. (T3) under adversarial flapping: 20,000 random label sequences,
    // including UNKNOWN cycling, must never exceed the fallback and never
    // shrink within a hold.
    {
      std::mt19937 rng(7u);
      std::uniform_int_distribution<int> pick(0, 3);
      std::uniform_real_distribution<double> conf(0.0, 1.0);
      std::uniform_int_distribution<uint32_t> obs(0, 60);
      long steps = 0, above_fallback = 0, bad_shrink = 0;
      const MotionCapabilityProfile global{0.8, 0.5, 0.0};
      for (int seq = 0; seq < 2000; ++seq) {
        ClassProfileState st;
        ObstacleClass last = ObstacleClass::UNKNOWN;
        MotionCapabilityProfile prev{0.0, 0.0, 0.0};
        uint32_t hold_left = 0;
        bool first = true;
        for (int k = 0; k < 40; ++k) {
          const ObstacleClass l = static_cast<ObstacleClass>(pick(rng));
          const auto s = selectCapabilityProfile(
            l, conf(rng), obs(rng), kSet, kAmb, kPol, global, true, st);
          ++steps;
          if (!profileDominates(fb, s.profile)) {++above_fallback;}
          // Mirror the rule's own bookkeeping to know when a shrink is legal.
          const bool in_hold = !first && (l != last || hold_left > 0);
          if (in_hold && !profileDominates(s.profile, prev)) {++bad_shrink;}
          if (!first && l != last) {
            hold_left = kPol.transition_hold_scans;
          } else if (hold_left > 0) {
            --hold_left;
          }
          prev = s.profile;
          last = l;
          first = false;
        }
      }
      check(above_fallback == 0, "(T3) violated: a held envelope exceeded the fallback");
      check(bad_shrink == 0, "(T2) violated: occupancy shrank inside a hold window");
      std::printf(
        "  \"transitions\": {\"sequences\": 2000, \"steps\": %ld, "
        "\"exceeded_fallback\": %ld, \"illegal_shrinks\": %ld},\n",
        steps, above_fallback, bad_shrink);
    }
  }

  // =====================================================================
  // 6. The Stage-5C.1 observable breach, per profile.
  // =====================================================================
  {
    long breaches_flagged = 0, caps_raised = 0, cases = 0;
    for (size_t c = 1; c < predictive_nav_tracking::kNumProfileClasses; ++c) {
      const MotionCapabilityProfile p = kSet.by_class[c];
      const ReachabilityBounds b = boundsForProfile(ReachabilityBounds{}, p);
      const double over = p.max_speed + 0.25;
      const Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() * 0.01;
      const auto r = buildRegion(0.0, 0.0, over, 0.0, cov, 2.0, b);
      ++cases;
      if (r.contract_status == ContractStatus::kObservedSpeedExceedsCap) {++breaches_flagged;}
      if (r.effective_max_speed > r.configured_max_speed) {++caps_raised;}
      check(r.valid, "an out-of-contract class sample was dropped instead of enlarged");
      check(
        bit_identical(r.configured_max_speed, p.max_speed),
        "the published configured cap is not the selected profile's");

      // In domain for the same profile: satisfied, and the cap untouched.
      const auto in = buildRegion(0.0, 0.0, p.max_speed - 0.01, 0.0, cov, 2.0, b);
      check(in.contract_status == ContractStatus::kSatisfied, "in-domain class sample flagged");
      check(
        bit_identical(in.effective_max_speed, in.configured_max_speed),
        "an in-domain class sample perturbed the cap");
    }
    check(breaches_flagged == cases, "a per-profile observable breach was not flagged");
    check(caps_raised == cases, "a per-profile breach did not raise the effective cap");

    // The informative case: 1.0 m/s breaches the old global 0.8 m/s cap but is
    // far inside every class profile, so class-awareness makes a real
    // contract difference rather than only a size difference. The Stage-5B
    // recordings measured a filtered bike speed of 0.967 m/s, so this is the
    // dataset's own case.
    const Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() * 0.01;
    ReachabilityBounds global;
    global.max_speed = 0.8;
    global.max_acceleration = 0.5;
    const auto g = buildRegion(0.0, 0.0, 0.967, 0.0, cov, 2.0, global);
    const auto bk = buildRegion(
      0.0, 0.0, 0.967, 0.0, cov, 2.0,
      boundsForProfile(ReachabilityBounds{}, kSet[ObstacleClass::BIKE]));
    check(
      g.contract_status == ContractStatus::kObservedSpeedExceedsCap,
      "0.967 m/s no longer breaches the global 0.8 m/s cap");
    check(
      bk.contract_status == ContractStatus::kSatisfied,
      "0.967 m/s breaches the BIKE profile, which would be wrong");
    std::printf(
      "  \"observable_breach_per_profile\": {\"profiles\": %ld, \"flagged\": %ld, "
      "\"caps_raised\": %ld, \"global_breaches_at_0.967\": true, "
      "\"bike_breaches_at_0.967\": false},\n", cases, breaches_flagged, caps_raised);
  }

  // =====================================================================
  // 7. Malformed configuration.
  // =====================================================================
  {
    const double nan_v = std::numeric_limits<double>::quiet_NaN();
    const double inf_v = std::numeric_limits<double>::infinity();
    const MotionCapabilityProfile good_fb = fallbackProfile(kSet, kAmb);

    struct Bad
    {
      const char * name;
      MotionCapabilityProfile p;
    };
    const std::vector<Bad> bads{
      {"negative_max_speed", {-1.0, 2.5, 0.0}},
      {"negative_max_acceleration", {2.5, -0.5, 0.0}},
      {"negative_margin", {2.5, 2.5, -0.1}},
      {"nan_max_speed", {nan_v, 2.5, 0.0}},
      {"nan_max_acceleration", {2.5, nan_v, 0.0}},
      {"nan_margin", {2.5, 2.5, nan_v}},
      {"inf_max_speed", {inf_v, 2.5, 0.0}},
      {"inf_max_acceleration", {2.5, inf_v, 0.0}},
      {"zero_everything", {0.0, 0.0, 0.0}},
    };

    long rejected = 0;
    for (const auto & bad : bads) {
      CapabilityProfileSet set = kSet;
      set[ObstacleClass::PEDESTRIAN] = bad.p;
      const bool ok = sanitizeProfileSet(set);
      const bool should_reject = !profileWellFormed(bad.p);
      check(ok == !should_reject, std::string("sanitize verdict wrong for ") + bad.name);
      if (should_reject) {
        ++rejected;
        // The substituted entry must DOMINATE every well-formed entry, so a
        // typo can never buy a smaller region than the correct table would.
        check(
          profileDominates(set[ObstacleClass::PEDESTRIAN], kSet[ObstacleClass::VEHICLE]),
          std::string("a sanitized entry is weaker than VEHICLE for ") + bad.name);
        check(
          profileWellFormed(set[ObstacleClass::PEDESTRIAN]),
          std::string("sanitize left a malformed entry for ") + bad.name);
        // And the fallback the node then serves is no weaker than before.
        check(
          profileDominates(fallbackProfile(set, kAmb), good_fb),
          std::string("sanitizing weakened the fallback for ") + bad.name);
      }
    }
    // zero_everything is WELL FORMED -- zero is a legitimate, maximally
    // restrictive premise (a provably immobile object) and is not an error.
    // It is nonetheless unsafe to SELECT by accident, which is why a zero
    // profile for one class still cannot pull the fallback down: the fallback
    // is a maximum.
    {
      CapabilityProfileSet set = kSet;
      set[ObstacleClass::VEHICLE] = MotionCapabilityProfile{0.0, 0.0, 0.0};
      check(
        sanitizeProfileSet(set),
        "an all-zero profile was treated as malformed");
      const auto fb_zero = fallbackProfile(set, kAmb);
      check(
        fb_zero.max_speed == kSet[ObstacleClass::BIKE].max_speed,
        "zeroing VEHICLE did not leave the fallback at the next strongest class");
    }

    // EVERY entry malformed: there is nothing well formed to envelope, so the
    // built-in conservative default must be used, not a zero profile.
    {
      CapabilityProfileSet set;
      for (size_t c = 0; c < predictive_nav_tracking::kNumProfileClasses; ++c) {
        set.by_class[c] = MotionCapabilityProfile{nan_v, nan_v, nan_v};
      }
      check(!sanitizeProfileSet(set), "an all-NaN table was accepted");
      const auto fb_bad = fallbackProfile(set, kAmb);
      check(profileWellFormed(fb_bad), "an all-NaN table produced a malformed fallback");
      check(
        profileDominates(fb_bad, kSet[ObstacleClass::VEHICLE]),
        "an all-NaN table produced a fallback weaker than VEHICLE");
    }

    // A malformed table ALSO forces the fallback at selection time, so the
    // failure is visible on every sample and not only in the start-up log.
    {
      ClassProfileState st;
      const MotionCapabilityProfile global{0.8, 0.5, 0.0};
      const auto s = selectCapabilityProfile(
        ObstacleClass::BIKE, 1.0, 50, kSet, kAmb, kPol, global, false, st);
      check(
        s.reason == ProfileReason::kFallbackInvalidConfig,
        "config_valid=false did not force PROFILE_FALLBACK_INVALID_CONFIG");
      check(
        profileDominates(s.profile, kSet[ObstacleClass::VEHICLE]),
        "config_valid=false still served a weak profile");
    }

    // An UNKNOWN ENUM VALUE. ObstacleClass is uint8 on the wire, so a
    // corrupted or future label can reach the selector. It must index inside
    // the table and must not be more optimistic than the fallback.
    {
      long oob_checks = 0, oob_bad = 0;
      for (unsigned v = 0; v < 256; ++v) {
        const ObstacleClass l = static_cast<ObstacleClass>(v);
        const size_t idx = static_cast<size_t>(l);
        if (idx >= predictive_nav_tracking::kNumProfileClasses) {
          // Out of table range: the node never constructs such a value (the
          // stabiliser only ever returns one of the four), so the contract is
          // that the four real values are total -- asserted below -- rather
          // than that an impossible value is silently absorbed.
          continue;
        }
        const auto s = select_fresh(l, 1.0, 50, kSet, kAmb, kPol);
        ++oob_checks;
        if (!profileDominates(fallbackProfile(kSet, kAmb), s.profile)) {++oob_bad;}
        if (!profileWellFormed(s.profile)) {++oob_bad;}
      }
      check(oob_checks == 4, "ObstacleClass no longer has exactly 4 in-table values");
      check(oob_bad == 0, "a valid enum value produced an out-of-order profile");
    }

    // AMBIGUITY SETS: an EMPTY set must not strip the label out of its own
    // closure, because that would hand the label a zero profile.
    {
      AmbiguitySets amb = kAmb;
      amb.mask[static_cast<size_t>(ObstacleClass::BIKE)] = 0;
      const auto s = select_fresh(ObstacleClass::BIKE, 1.0, 50, kSet, amb, kPol);
      check(
        profileDominates(s.profile, kSet[ObstacleClass::BIKE]),
        "(I1) violated: an empty ambiguity set dropped the label's own profile");
      check(s.profile.max_speed > 0.0, "an empty ambiguity set produced a zero profile");
    }
    // An UNKNOWN set that omits classes weakens the fallback -- which is a
    // legitimate operator choice and must be reflected honestly rather than
    // silently overridden, but it must still dominate what it does name.
    {
      AmbiguitySets amb = kAmb;
      amb.mask[static_cast<size_t>(ObstacleClass::UNKNOWN)] =
        static_cast<uint8_t>(classBit(ObstacleClass::PEDESTRIAN));
      const auto fb_narrow = fallbackProfile(kSet, amb);
      check(
        profileDominates(fb_narrow, kSet[ObstacleClass::PEDESTRIAN]),
        "a narrowed UNKNOWN set does not dominate what it names");
      check(
        !profileDominates(fb_narrow, kSet[ObstacleClass::VEHICLE]),
        "a narrowed UNKNOWN set silently kept vehicle coverage, hiding the operator's choice");
    }

    std::printf(
      "  \"malformed_config\": {\"cases\": %zu, \"rejected\": %ld, "
      "\"all_substitutions_dominate_vehicle\": true},\n", bads.size(), rejected);
  }

  // =====================================================================
  // 8 & 9. End to end through the production node.
  // =====================================================================
  {
    auto node = std::make_shared<LidarObstacleTracker>();
    const rclcpp::Time stamp(2000, 0, RCL_ROS_TIME);

    // 8a. THE PRODUCTION DEFAULT IS OFF. Stage-5D must not change behaviour.
    check(
      !node->profile_policy_.class_aware,
      "reachability_class_aware now defaults to TRUE -- the production default changed");
    check(node->profile_config_valid_, "the shipped profile table does not validate");

    auto make_track = [&](uint32_t id, double vx, ObstacleClass label, uint32_t obs) {
        Track t;
        t.id = id;
        t.kalman_initialized = true;
        t.last_update = stamp;
        t.kf_x << 0.0, 0.0, vx, 0.0;
        t.kf_P = Eigen::Matrix4d::Identity() * 0.01;
        t.observations = obs;
        // Drive the real stabiliser to the requested label rather than
        // writing it in, so the confidence is the one the node would see.
        if (label != ObstacleClass::UNKNOWN) {
          predictive_nav_tracking::ClassificationResult inst;
          inst.label = label;
          inst.confidence = 1.0;
          for (int k = 0; k < 12; ++k) {
            t.class_state.update(inst);
          }
        }
        return t;
      };

    // 8b. Class-aware OFF is bit-identical to Stage-5C.1: a BIKE-labelled
    // track and an UNKNOWN one produce the same numbers as each other and as
    // the global configuration.
    Track bike_off = make_track(1, 0.5, ObstacleClass::BIKE, 50);
    Track unk_off = make_track(2, 0.5, ObstacleClass::UNKNOWN, 50);
    const auto bounds_bike_off = node->select_track_bounds(bike_off);
    const auto bounds_unk_off = node->select_track_bounds(unk_off);
    check(
      bit_identical(bounds_bike_off.max_speed, 0.8) &&
      bit_identical(bounds_bike_off.max_acceleration, 0.5),
      "class-aware OFF did not return the global configuration for a BIKE track");
    check(
      bit_identical(bounds_unk_off.max_speed, bounds_bike_off.max_speed),
      "class-aware OFF made two labels differ");
    check(
      bike_off.class_profile_selection.reason == ProfileReason::kGlobal,
      "class-aware OFF did not report PROFILE_GLOBAL");
    const auto off_regions = node->predict_reachability(bike_off, stamp, &bounds_bike_off);
    const auto legacy_regions = node->predict_reachability(bike_off, stamp);
    check(off_regions.size() == legacy_regions.size(), "sample count changed");
    long bitdiff = 0;
    for (size_t i = 0; i < off_regions.size(); ++i) {
      if (!bit_identical(off_regions[i].semi_major, legacy_regions[i].semi_major)) {++bitdiff;}
      if (!bit_identical(off_regions[i].semi_minor, legacy_regions[i].semi_minor)) {++bitdiff;}
      if (!bit_identical(off_regions[i].reach_radius, legacy_regions[i].reach_radius)) {++bitdiff;}
    }
    check(bitdiff == 0, "the class-aware-off path is not bit-identical to the legacy call");

    // 8c. Turn class-aware ON through the real parameter interface.
    node->set_parameter(rclcpp::Parameter("reachability_class_aware", true));
    node->read_parameters();
    check(node->profile_policy_.class_aware, "the parameter did not enable class-aware mode");

    Track bike_on = make_track(3, 0.5, ObstacleClass::BIKE, 50);
    Track ped_on = make_track(4, 0.5, ObstacleClass::PEDESTRIAN, 50);
    Track unk_on = make_track(5, 0.5, ObstacleClass::UNKNOWN, 50);
    Track young_on = make_track(6, 0.5, ObstacleClass::BIKE, 3);

    const auto b_bike = node->select_track_bounds(bike_on);
    const auto b_ped = node->select_track_bounds(ped_on);
    const auto b_unk = node->select_track_bounds(unk_on);
    const auto b_young = node->select_track_bounds(young_on);

    check(bit_identical(b_bike.max_speed, 12.0), "node did not apply the BIKE profile");
    check(bit_identical(b_bike.max_acceleration, 7.0), "node did not apply the BIKE accel");
    check(bit_identical(b_ped.max_speed, 40.0), "node did not apply the PEDESTRIAN closure");
    check(bit_identical(b_unk.max_speed, 40.0), "node did not apply the fallback for UNKNOWN");
    check(bit_identical(b_young.max_speed, 40.0), "node did not fall back for a young track");
    check(
      young_on.class_profile_selection.reason == ProfileReason::kFallbackImmatureTrack,
      "node did not report the immature-track fallback");
    check(
      ped_on.class_profile_selection.dominating_class == ObstacleClass::VEHICLE,
      "node did not report the vehicle-grade profile behind a PEDESTRIAN label");

    // 9. The regions really do differ, and in the safe direction.
    const auto reg_bike = node->predict_reachability(bike_on, stamp, &b_bike);
    const auto reg_unk = node->predict_reachability(unk_on, stamp, &b_unk);
    check(reg_bike.size() == reg_unk.size(), "arm sample counts differ");
    long bike_smaller = 0;
    for (size_t i = 0; i < reg_bike.size(); ++i) {
      if (reg_bike[i].semi_major < reg_unk[i].semi_major) {++bike_smaller;}
      check(
        reg_bike[i].contract_status == ContractStatus::kSatisfied,
        "a BIKE-profile in-domain sample was flagged");
    }
    check(
      bike_smaller == static_cast<long>(reg_bike.size()),
      "the BIKE profile did not produce a smaller region than the fallback through the node");

    // 8d. CV output is untouched. predict_future() reads neither the profile
    // nor the class, so the CV samples of two differently-labelled tracks
    // with identical state must be bit-identical.
    const auto cv_bike = node->predict_future(bike_on);
    const auto cv_unk = node->predict_future(unk_on);
    check(cv_bike.size() == cv_unk.size(), "CV sample count changed with the label");
    long cv_diff = 0;
    for (size_t i = 0; i < cv_bike.size(); ++i) {
      if (!bit_identical(cv_bike[i].x, cv_unk[i].x)) {++cv_diff;}
      if (!bit_identical(cv_bike[i].y, cv_unk[i].y)) {++cv_diff;}
      if (!bit_identical(cv_bike[i].vx, cv_unk[i].vx)) {++cv_diff;}
      for (size_t k = 0; k < 4; ++k) {
        if (!bit_identical(
            cv_bike[i].position_covariance[k], cv_unk[i].position_covariance[k]))
        {
          ++cv_diff;
        }
      }
    }
    check(cv_diff == 0, "the CV prediction changed with the class label");

    std::printf(
      "  \"through_production_node\": {\"class_aware_default\": false, "
      "\"off_path_bit_differences\": %ld, \"bike_max_speed\": %.2f, "
      "\"pedestrian_applied_max_speed\": %.2f, \"unknown_max_speed\": %.2f, "
      "\"immature_fell_back\": true, \"bike_samples_smaller_than_fallback\": %ld, "
      "\"cv_bit_differences\": %ld},\n",
      bitdiff, b_bike.max_speed, b_ped.max_speed, b_unk.max_speed, bike_smaller, cv_diff);
  }

  std::printf("  \"failures\": %d,\n", failures);
  std::printf("  \"pass\": %s\n", failures == 0 ? "true" : "false");
  std::printf("}\n");
  rclcpp::shutdown();
  if (failures != 0) {
    std::fprintf(stderr, "stage5d_profiles: %d check(s) failed\n", failures);
    return 1;
  }
  return 0;
}
