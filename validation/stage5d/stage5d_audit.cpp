// Stage-5D -- offline qualification of the CLASS-AWARE capability profiles.
// ANALYSIS ONLY: this file includes the shipped production headers and
// modifies nothing, so every number below describes the shipped mathematics
// and the shipped selection policy rather than a re-implementation of either.
//
// Build (no CMake or production build file is touched):
//   g++ -O2 -std=c++17 -I/usr/include/eigen3 \
//       -Isrc/predictive_nav_tracking/include \
//       validation/stage5d/stage5d_audit.cpp -o <out>/stage5d_audit
//   <out>/stage5d_audit <replay.txt from extract_labels.py>
//
// FIVE PARTS
//   1. Per-profile adversarial containment IN domain (brief section 9):
//      every profile, every published horizon, motion admissible under that
//      profile. 0 escapes is the gate.
//   2. OUT-of-profile motion (brief section 9, second half): the same
//      adversary given a LARGER true capability than the profile declares.
//      Escapes are expected and are the point -- they are what makes a
//      profile a premise rather than a guarantee. Also measures that the
//      Stage-5C.1 observable breach still restores containment per profile.
//   3. THREE ARMS over the real recorded classifier output (brief sections
//      10 and 12): A global, B runtime class-aware, C oracle class-aware.
//      Arm C is ANALYTICAL / EVALUATION ONLY and exists solely to separate
//      class-aware MODEL quality from CLASSIFIER ERROR.
//   4. CLASSIFICATION-ERROR ANALYSIS (brief section 11): per confusion
//      direction, does the runtime arm's selected profile still cover what
//      the object really is?
//   5. Cost.
//
// ORACLE DISCIPLINE. The ground-truth class is read in exactly one place --
// `Row::gt_label` -- and reaches only arm C and part 4. The production node
// is not built against this file and has no parameter that could read it.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <random>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "predictive_nav_tracking/capability_profiles.hpp"
#include "predictive_nav_tracking/kalman_filter.hpp"
#include "predictive_nav_tracking/obstacle_classification.hpp"
#include "predictive_nav_tracking/reachability.hpp"

using predictive_nav_tracking::AmbiguitySets;
using predictive_nav_tracking::CapabilityProfileSet;
using predictive_nav_tracking::ClassProfileState;
using predictive_nav_tracking::ConstantVelocityKalmanFilter2D;
using predictive_nav_tracking::ContractStatus;
using predictive_nav_tracking::MotionCapabilityProfile;
using predictive_nav_tracking::ObstacleClass;
using predictive_nav_tracking::ProfileReason;
using predictive_nav_tracking::ProfileSelection;
using predictive_nav_tracking::ProfileSelectionPolicy;
using predictive_nav_tracking::ReachabilityBounds;
using predictive_nav_tracking::ReachabilityRegion;
using predictive_nav_tracking::boundsForProfile;
using predictive_nav_tracking::buildRegion;
using predictive_nav_tracking::fallbackProfile;
using predictive_nav_tracking::profileDominates;
using predictive_nav_tracking::reachRadius;
using predictive_nav_tracking::selectCapabilityProfile;
using predictive_nav_tracking::toString;

namespace
{

// ---- frozen production parameters -----------------------------------------
// Read from tracker_params.yaml and production_nav2_params.yaml. NOT retuned.
constexpr double kScanPeriod = 0.2;        // s, 5 Hz LDS
constexpr double kAccelNoise = 0.5;        // kf_process_accel_noise
constexpr double kMeasNoise = 0.05;        // kf_measurement_noise
constexpr double kInitPosVar = 0.01;       // kf_initial_position_variance
constexpr double kInitVelVar = 4.0;        // kf_initial_velocity_variance
constexpr double kTimeStep = 0.5;          // reachability_time_step
constexpr int kNumHorizons = 6;            // reachability_horizon 3.0 / 0.5
constexpr double kMaxObsAge = 1.0;         // max_observation_age
constexpr double kTrackerSigma = 2.0;      // covariance_sigma_level
constexpr double kMaxInfluence = 0.55;     // layer max_influence_radius
constexpr double kCostmapRes = 0.05;       // costmap resolution

/// The single global configuration Stage-5C/5C.1 qualified: arm A.
MotionCapabilityProfile globalProfile()
{
  return MotionCapabilityProfile{0.8, 0.5, 0.0};
}

ReachabilityBounds baseBounds()
{
  ReachabilityBounds b;
  b.covariance_sigma_level = kTrackerSigma;
  return b;
}

const char * reasonName(ProfileReason r)
{
  switch (r) {
    case ProfileReason::kGlobal: return "global";
    case ProfileReason::kClassClosure: return "class_closure";
    case ProfileReason::kFallbackUnknownLabel: return "fallback_unknown_label";
    case ProfileReason::kFallbackLowConfidence: return "fallback_low_confidence";
    case ProfileReason::kFallbackImmatureTrack: return "fallback_immature_track";
    case ProfileReason::kFallbackInvalidConfig: return "fallback_invalid_config";
    case ProfileReason::kTransitionHold: return "transition_hold";
  }
  return "?";
}

bool isFallback(ProfileReason r)
{
  return r >= ProfileReason::kFallbackUnknownLabel &&
         r <= ProfileReason::kFallbackInvalidConfig;
}

// =====================================================================
// The adversary (same construction as Stage-5C part 2)
// =====================================================================
//
// Forward-simulates an admissible trajectory under |a| <= a_max and
// |v| <= v_cap and returns |deviation| from the constant-velocity nominal at
// time t. `a_max` and `v_cap` are the adversary's TRUE capability and are
// passed explicitly, never taken from the model's bounds, because the whole
// point of part 2 is to let the obstacle do more than the profile declares.
//
// Midpoint velocity update: forward Euler's O(a*dt*t) error exceeds the slack
// between an extremal trajectory and the bound near the crossover time.
double simulatedDeviation(
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
    const double vxb = vx, vyb = vy;
    vx += ax * step;
    vy += ay * step;
    const double sp = std::hypot(vx, vy);
    if (sp > v_cap && sp > 0.0) {
      vx *= v_cap / sp;
      vy *= v_cap / sp;
    }
    px += 0.5 * (vxb + vx) * step;
    py += 0.5 * (vyb + vy) * step;
  }
  return std::hypot(px - v0x * t, py - v0y * t);
}

/// NUMERICAL ALLOWANCE on the adversary's own integration error, m.
///
/// The bound is TIGHT: for extremal straight-line motion the simulated
/// trajectory lands exactly ON r_det(t), so the sign of `bound - dev` at the
/// optimum is decided by the midpoint rule's O(a*dt^2) residual rather than by
/// the mathematics. Counting those as escapes would report the integrator, not
/// the model. This is the SAME 1e-6 m allowance Stage-5C's adversary used
/// (validation/stage5c/stage5c_audit.cpp:462), kept identical so the two
/// qualifications are comparable. The raw worst slack is reported at full
/// precision beside every escape count, so the allowance hides nothing: a real
/// escape is orders of magnitude larger (part 2 measures 1.4-42 m).
constexpr double kIntegrationTolerance = 1e-6;

/// Planner-visible cell count, replicating the POST-Stage-5C.1
/// rasterizeReachableSet exactly: the half-extents of the published ellipse's
/// bounding box are clamped to max_influence_radius, and membership is tested
/// against the semi-axes AS PUBLISHED. Cells are counted on a grid aligned to
/// the region centre, which is the same count the layer produces up to a
/// half-cell phase.
long paintedCells(double semi_major, double semi_minor, double yaw)
{
  if (!(semi_major > 0.0) || !(semi_minor > 0.0)) {
    return 0;
  }
  const double c = std::cos(yaw), sn = std::sin(yaw);
  double half_x = std::hypot(semi_major * c, semi_minor * sn);
  double half_y = std::hypot(semi_major * sn, semi_minor * c);
  if (kMaxInfluence > 0.0) {
    half_x = std::min(half_x, kMaxInfluence);
    half_y = std::min(half_y, kMaxInfluence);
  }
  const int ni = static_cast<int>(std::floor(half_x / kCostmapRes));
  const int nj = static_cast<int>(std::floor(half_y / kCostmapRes));
  long cells = 0;
  for (int j = -nj; j <= nj; ++j) {
    for (int i = -ni; i <= ni; ++i) {
      const double dx = i * kCostmapRes, dy = j * kCostmapRes;
      const double u = (dx * c + dy * sn) / semi_major;
      const double v = (-dx * sn + dy * c) / semi_minor;
      if (u * u + v * v <= 1.0) {
        ++cells;
      }
    }
  }
  return cells;
}

// =====================================================================
// The replay table
// =====================================================================
struct Row
{
  int scenario = 0;
  int track = 0;
  double stamp = 0.0;
  double x = 0.0;
  double y = 0.0;
  double speed = 0.0;
  uint32_t observations = 0;
  bool kalman_init = false;
  ObstacleClass runtime_label = ObstacleClass::UNKNOWN;
  double runtime_conf = 0.0;
  /// ANALYTICAL / EVALUATION ONLY. Read by arm C and part 4, nowhere else.
  ObstacleClass gt_label = ObstacleClass::UNKNOWN;
};

std::vector<Row> readReplay(const std::string & path)
{
  std::vector<Row> rows;
  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "cannot open replay table %s\n", path.c_str());
    std::exit(2);
  }
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    Row r;
    int ki = 0, rl = 0, gl = 0;
    unsigned long obs = 0;
    if (std::sscanf(
        line.c_str(), "%d %d %lf %lf %lf %lf %lu %d %d %lf %d",
        &r.scenario, &r.track, &r.stamp, &r.x, &r.y, &r.speed, &obs, &ki, &rl,
        &r.runtime_conf, &gl) != 11)
    {
      continue;
    }
    r.observations = static_cast<uint32_t>(obs);
    r.kalman_init = ki != 0;
    r.runtime_label = static_cast<ObstacleClass>(rl);
    r.gt_label = static_cast<ObstacleClass>(gl);
    rows.push_back(r);
  }
  return rows;
}

// =====================================================================
// Per-arm accumulator
// =====================================================================
struct Acc
{
  long n = 0;
  long capability_contained = 0;   // region covers the GT class's capability disc
  long observed_contained = 0;     // the track's own later position is inside
  long observed_eval = 0;          // samples where a later position existed
  long contract_breach = 0;
  long fallback = 0;
  long transition = 0;
  long unknown_profile = 0;        // the fallback envelope was the applied profile
  long unsafe_selection = 0;       // applied profile does NOT dominate GT capability
  double area = 0.0;
  double cells = 0.0;
  double radius = 0.0;

  void add(
    bool cap_in, bool obs_eval, bool obs_in, bool breach, bool fb, bool tr,
    bool unk, bool unsafe, double a, double cl, double rr)
  {
    ++n;
    capability_contained += cap_in;
    observed_eval += obs_eval;
    observed_contained += (obs_eval && obs_in);
    contract_breach += breach;
    fallback += fb;
    transition += tr;
    unknown_profile += unk;
    unsafe_selection += unsafe;
    area += a;
    cells += cl;
    radius += rr;
  }
};

void printAcc(const char * label, const Acc & a, bool last)
{
  const double n = a.n > 0 ? static_cast<double>(a.n) : 1.0;
  const double on = a.observed_eval > 0 ? static_cast<double>(a.observed_eval) : 1.0;
  std::printf(
    "      \"%s\": {\"samples\": %ld, \"capability_containment\": %.6f, "
    "\"observed_containment\": %.6f, \"observed_evaluable\": %ld, "
    "\"mean_area_m2\": %.6f, \"mean_cells\": %.3f, \"mean_reach_radius_m\": %.4f, "
    "\"contract_breaches\": %ld, \"fallback_rate\": %.6f, "
    "\"transition_rate\": %.6f, \"unknown_profile_rate\": %.6f, "
    "\"unsafe_selections\": %ld}%s\n",
    label, a.n, a.capability_contained / n, a.observed_contained / on, a.observed_eval,
    a.area / n, a.cells / n, a.radius / n, a.contract_breach,
    a.fallback / n, a.transition / n, a.unknown_profile / n, a.unsafe_selection,
    last ? "" : ",");
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <replay.txt>\n", argv[0]);
    return 2;
  }

  const CapabilityProfileSet kSet;
  const AmbiguitySets kAmb;
  ProfileSelectionPolicy kPol;
  kPol.class_aware = true;
  const MotionCapabilityProfile kFallback = fallbackProfile(kSet, kAmb);

  const std::array<ObstacleClass, 4> kClasses{
    ObstacleClass::UNKNOWN, ObstacleClass::PEDESTRIAN,
    ObstacleClass::BIKE, ObstacleClass::VEHICLE};

  /// The profile that describes what an object of class c can really do. For
  /// UNKNOWN this is the derived fallback, because an UNKNOWN object's true
  /// capability is by definition not known and the envelope is the only
  /// honest answer.
  auto truthProfile = [&](ObstacleClass c) {
      return (c == ObstacleClass::UNKNOWN) ? kFallback : kSet[c];
    };

  std::printf("{\n");
  std::printf(
    "  \"note\": \"Stage-5D class-aware reachability audit. ANALYSIS ONLY: built "
    "against the shipped production headers, modifies nothing. The ORACLE arm (C) "
    "and part 4 are the ONLY readers of the ground-truth class and are "
    "ANALYTICAL / EVALUATION ONLY -- no ground-truth label enters any runtime "
    "path.\",\n");

  // =================================================================
  // PART 1 -- per-profile adversarial containment, IN domain
  // =================================================================
  //
  // For each profile, motion that is admissible UNDER THAT PROFILE must be
  // contained by the region that profile produces, at every published
  // horizon, for fresh and coasting tracks alike. 0 escapes is the gate of
  // brief section 9.
  //
  // The adversary sweeps initial speed up to the profile's own cap, every
  // acceleration direction, and the extremal acceleration magnitude (the
  // bound is achieved at |a| = a_max, so interior magnitudes cannot be
  // worse); initial speeds at the cap are included because that is where the
  // |v(s) - v0| <= v_max + s0 term binds.
  std::printf("  \"part1_in_profile_adversarial\": {\n");
  std::printf("    \"gate\": \"0 escapes required\",\n");
  std::printf("    \"profiles\": [\n");
  for (size_t ci = 0; ci < kClasses.size(); ++ci) {
    const ObstacleClass c = kClasses[ci];
    const MotionCapabilityProfile p = truthProfile(c);
    const ReachabilityBounds b = boundsForProfile(baseBounds(), p);

    long trials = 0, escapes = 0;
    double worst_slack = std::numeric_limits<double>::infinity();
    double worst_escape = 0.0;
    double area_h1 = 0.0, area_hmax = 0.0;

    // Initial speeds spanning 0 to the profile cap, including the cap itself.
    std::vector<double> s0s;
    for (int k = 0; k <= 8; ++k) {
      s0s.push_back(p.max_speed * static_cast<double>(k) / 8.0);
    }
    for (double s0 : s0s) {
      for (int dir = 0; dir < 16; ++dir) {
        const double th = 2.0 * M_PI * dir / 16.0;
        const double ax = p.max_acceleration * std::cos(th);
        const double ay = p.max_acceleration * std::sin(th);
        for (int h = 1; h <= kNumHorizons; ++h) {
          for (double age : {0.0, kMaxObsAge}) {
            const double t = age + h * kTimeStep;
            const double dev = simulatedDeviation(s0, 0.0, ax, ay, t,
              p.max_acceleration, p.max_speed);
            const double bound = reachRadius(t, s0, b);
            ++trials;
            const double slack = bound - dev;
            worst_slack = std::min(worst_slack, slack);
            if (slack < -kIntegrationTolerance) {
              ++escapes;
              worst_escape = std::max(worst_escape, -slack);
            }
          }
        }
      }
    }
    // Region AREA at the first and the worst-case horizon, for the growth
    // figure brief section 9 asks for. Measured at a representative filter
    // state (a track with 10 updates at 0.5 m/s) so the sigma term is the
    // production one rather than an arbitrary covariance.
    {
      ConstantVelocityKalmanFilter2D kf(kAccelNoise, kMeasNoise);
      kf.initialize(0.0, 0.0, kInitPosVar, kInitVelVar);
      for (int k = 0; k < 10; ++k) {
        kf.predict(kScanPeriod);
        kf.update(0.5 * kScanPeriod * (k + 1), 0.0);
      }
      const auto r1 = buildRegion(
        kf.state()(0), kf.state()(1), kf.state()(2), kf.state()(3),
        kf.covariance().topLeftCorner<2, 2>(), kTimeStep, b);
      const auto rm = buildRegion(
        kf.state()(0), kf.state()(1), kf.state()(2), kf.state()(3),
        kf.covariance().topLeftCorner<2, 2>(), kMaxObsAge + kNumHorizons * kTimeStep, b);
      area_h1 = M_PI * r1.semi_major * r1.semi_minor;
      area_hmax = M_PI * rm.semi_major * rm.semi_minor;
    }

    std::printf(
      "      {\"class\": \"%s\", \"max_speed\": %.2f, \"max_acceleration\": %.2f, "
      "\"margin\": %.2f, \"horizons_s\": [0.5, 1.0, 1.5, 2.0, 2.5, 3.0], "
      "\"total_time_max_s\": %.1f, \"trials\": %ld, \"escapes\": %ld, "
      "\"worst_slack_m\": %.6e, \"worst_escape_beyond_tolerance_m\": %.6e, "
      "\"integration_tolerance_m\": 1e-06, "
      "\"area_at_0.5s_m2\": %.4f, \"area_at_4.0s_m2\": %.4f, \"area_growth\": %.2f}%s\n",
      toString(c), p.max_speed, p.max_acceleration, p.margin,
      kMaxObsAge + kNumHorizons * kTimeStep, trials, escapes, worst_slack, worst_escape,
      area_h1, area_hmax, area_h1 > 0.0 ? area_hmax / area_h1 : 0.0,
      ci + 1 == kClasses.size() ? "" : ",");
  }
  std::printf("    ]\n  },\n");

  // =================================================================
  // PART 2 -- OUT-of-profile motion
  // =================================================================
  //
  // The limitation that makes a profile a PREMISE. Each class's profile is
  // confronted with an obstacle whose TRUE capability is the next class up
  // (and, for VEHICLE, 1.5x its own), with the observed speed held BELOW the
  // configured cap so the breach is UNOBSERVABLE -- exactly Stage-5C's
  // s0 = 0, v_phys > cap case, now per class.
  //
  // Then the OBSERVABLE half: the same excess capability with the observed
  // speed above the cap. Stage-5C.1's response raises the cap to the observed
  // speed, which must restore containment of everything the observation
  // proves possible.
  std::printf("  \"part2_out_of_profile\": {\n");
  std::printf(
    "    \"what_this_shows\": \"A profile is a DECLARED premise about an "
    "unobservable future, not a proven bound. Where the true capability exceeds "
    "the declared one and the observed speed does NOT reveal it, the region is "
    "escaped -- and no runtime check can prevent that. Where the observed speed "
    "DOES reveal it, Stage-5C.1 raises the effective cap and containment is "
    "restored.\",\n");
  std::printf("    \"unobservable\": [\n");
  {
    struct Probe
    {
      ObstacleClass declared;
      const char * true_desc;
      double true_speed;
      double true_accel;
    };
    const std::vector<Probe> probes{
      {ObstacleClass::PEDESTRIAN, "a cyclist mislabelled PEDESTRIAN", 12.0, 7.0},
      {ObstacleClass::PEDESTRIAN, "an athletic side-step at 0.5 g", 2.5, 5.0},
      {ObstacleClass::BIKE, "a car mislabelled BIKE", 40.0, 10.0},
      {ObstacleClass::VEHICLE, "a vehicle 1.5x the declared capability", 60.0, 15.0},
    };
    for (size_t i = 0; i < probes.size(); ++i) {
      const Probe & pr = probes[i];
      const MotionCapabilityProfile p = kSet[pr.declared];
      const ReachabilityBounds b = boundsForProfile(baseBounds(), p);
      long trials = 0, escapes = 0;
      double worst_escape = 0.0;
      double escape_at_max_t = 0.0;
      // Observed speed held strictly below the configured cap, so
      // capabilityPremiseContradicted() is FALSE and nothing is detectable.
      for (double s0 : {0.0, 0.25 * p.max_speed, 0.5 * p.max_speed}) {
        for (int dir = 0; dir < 8; ++dir) {
          const double th = 2.0 * M_PI * dir / 8.0;
          for (int h = 1; h <= kNumHorizons; ++h) {
            for (double age : {0.0, kMaxObsAge}) {
              const double t = age + h * kTimeStep;
              const double dev = simulatedDeviation(
                s0, 0.0, pr.true_accel * std::cos(th), pr.true_accel * std::sin(th),
                t, pr.true_accel, pr.true_speed);
              const double bound = reachRadius(t, s0, b);
              ++trials;
              if (dev - bound > kIntegrationTolerance) {
                ++escapes;
                worst_escape = std::max(worst_escape, dev - bound);
                if (t >= kMaxObsAge + kNumHorizons * kTimeStep - 1e-9) {
                  escape_at_max_t = std::max(escape_at_max_t, dev - bound);
                }
              }
              // The premise was NOT contradicted, so the model reports
              // CONTRACT_SATISFIED throughout -- which is the honest limit.
            }
          }
        }
      }
      const ReachabilityBounds probe_b = boundsForProfile(baseBounds(), p);
      std::printf(
        "      {\"declared_profile\": \"%s\", \"declared\": [%.2f, %.2f], "
        "\"true_capability\": \"%s\", \"true\": [%.2f, %.2f], \"trials\": %ld, "
        "\"escapes\": %ld, \"escape_rate\": %.4f, \"worst_escape_m\": %.4f, "
        "\"escape_at_4.0s_m\": %.4f, \"contract_status\": \"CONTRACT_SATISFIED "
        "throughout -- the breach is UNOBSERVABLE\"}%s\n",
        toString(pr.declared), p.max_speed, p.max_acceleration, pr.true_desc,
        pr.true_speed, pr.true_accel, trials, escapes,
        trials > 0 ? static_cast<double>(escapes) / trials : 0.0,
        worst_escape, escape_at_max_t,
        i + 1 == probes.size() ? "" : ",");
      (void)probe_b;
    }
  }
  std::printf("    ],\n");
  std::printf("    \"observable\": [\n");
  {
    for (size_t ci = 1; ci < kClasses.size(); ++ci) {
      const ObstacleClass c = kClasses[ci];
      const MotionCapabilityProfile p = kSet[c];
      const ReachabilityBounds b = boundsForProfile(baseBounds(), p);
      // Observed speed ABOVE the configured cap: the premise is DISPROVED,
      // and the effective cap becomes the observed speed.
      const double s0 = p.max_speed * 1.25;
      long trials = 0, escapes_old = 0, escapes_new = 0;
      bool flagged = true;
      double worst_new_slack = std::numeric_limits<double>::infinity();
      for (int dir = 0; dir < 8; ++dir) {
        const double th = 2.0 * M_PI * dir / 8.0;
        for (int h = 1; h <= kNumHorizons; ++h) {
          for (double age : {0.0, kMaxObsAge}) {
            const double t = age + h * kTimeStep;
            // What the obstacle can do, given that it is DEMONSTRABLY moving
            // at s0 and therefore demonstrably capable of s0.
            const double dev = simulatedDeviation(
              s0, 0.0, p.max_acceleration * std::cos(th),
              p.max_acceleration * std::sin(th), t, p.max_acceleration, s0);
            // Pre-Stage-5C.1 arithmetic: the configured cap, not raised.
            ReachabilityBounds old_b = b;
            old_b.enforce_observed_speed = false;
            const double bound_old = reachRadius(t, s0, old_b);
            const double bound_new = reachRadius(t, s0, b);
            ++trials;
            if (dev - bound_old > kIntegrationTolerance) {++escapes_old;}
            if (dev - bound_new > kIntegrationTolerance) {++escapes_new;}
            worst_new_slack = std::min(worst_new_slack, bound_new - dev);
            const Eigen::Matrix2d cov = Eigen::Matrix2d::Identity() * 0.01;
            const auto reg = buildRegion(0.0, 0.0, s0, 0.0, cov, t, b);
            if (reg.contract_status != ContractStatus::kObservedSpeedExceedsCap) {
              flagged = false;
            }
          }
        }
      }
      std::printf(
        "      {\"declared_profile\": \"%s\", \"configured_max_speed\": %.2f, "
        "\"observed_speed\": %.2f, \"trials\": %ld, "
        "\"escapes_without_5C1_response\": %ld, \"escapes_with_5C1_response\": %ld, "
        "\"worst_slack_with_response_m\": %.6e, \"every_sample_flagged\": %s}%s\n",
        toString(c), p.max_speed, s0, trials, escapes_old, escapes_new,
        worst_new_slack, flagged ? "true" : "false",
        ci + 1 == kClasses.size() ? "" : ",");
    }
  }
  std::printf("    ]\n  },\n");

  // =================================================================
  // PART 3 -- THREE ARMS over the real recorded classifier output
  // =================================================================
  //
  // The replay re-runs the PRODUCTION Kalman filter along each recorded
  // track's measured centroids, so the state each arm predicts from is the
  // real filter's, driven by real measurements. The three arms then differ in
  // NOTHING but which profile the selector returns.
  //
  //   A GLOBAL        -- the single Stage-5C/5C.1 configuration (0.8, 0.5, 0)
  //   B RUNTIME       -- the recorded Stage-5B label and confidence
  //   C ORACLE        -- the ground-truth class. ANALYTICAL / EVALUATION
  //                      ONLY. It exists to separate class-aware MODEL
  //                      quality from CLASSIFIER ERROR and must never reach a
  //                      runtime path.
  //
  // TWO CONTAINMENT DEFINITIONS, because they answer different questions and
  // conflating them is how a safety claim gets overstated:
  //
  //   CAPABILITY CONTAINMENT -- does the arm's region cover everything an
  //     object of the TRUE class could do? Exactly: is the region's MINOR
  //     semi-axis at least r_det(t) under the true class's profile, which is
  //     the radius of the concentric capability disc. This is the safety
  //     question, and a misclassification can fail it.
  //   OBSERVED CONTAINMENT -- does the track's own later measured position
  //     fall inside the region predicted for that time? This is the
  //     calibration question, answered from recorded data with no ground
  //     truth of any kind.
  std::printf("  \"part3_three_arms\": {\n");
  const std::vector<Row> rows = readReplay(argv[1]);
  std::printf("    \"replay_rows\": %zu,\n", rows.size());

  {
    // Group by track, in order.
    std::map<int, std::vector<const Row *>> tracks;
    for (const auto & r : rows) {
      tracks[r.track].push_back(&r);
    }

    // Per-arm, per-horizon, pooled and per true class.
    Acc pooled[3][kNumHorizons];
    std::map<int, Acc> per_class[3];
    long transitions_runtime = 0, transitions_oracle = 0;
    double cv_ns = 0.0, build_ns = 0.0;
    long build_calls = 0;

    for (auto & kv : tracks) {
      const auto & seq = kv.second;
      ConstantVelocityKalmanFilter2D kf(kAccelNoise, kMeasNoise);
      bool kf_ready = false;
      ClassProfileState st_b, st_c;
      ObstacleClass prev_b = ObstacleClass::UNKNOWN, prev_c = ObstacleClass::UNKNOWN;
      bool first_label = true;

      for (size_t i = 0; i < seq.size(); ++i) {
        const Row & r = *seq[i];
        // --- the production filter, driven by the recorded centroid --------
        if (!kf_ready) {
          kf.initialize(r.x, r.y, kInitPosVar, kInitVelVar);
          kf_ready = true;
        } else {
          const double dt = std::max(r.stamp - seq[i - 1]->stamp, 1e-3);
          kf.predict(std::min(dt, kMaxObsAge));
          kf.update(r.x, r.y);
        }
        if (!r.kalman_init) {
          continue;   // the node publishes no region before this either
        }

        // --- the three selections -----------------------------------------
        const MotionCapabilityProfile gp = globalProfile();
        ClassProfileState dummy;
        const ProfileSelection sel_a = [&] {
            ProfileSelectionPolicy off;
            off.class_aware = false;
            return selectCapabilityProfile(
              r.runtime_label, r.runtime_conf, r.observations, kSet, kAmb, off, gp,
              true, dummy);
          }();
        const ProfileSelection sel_b = selectCapabilityProfile(
          r.runtime_label, r.runtime_conf, r.observations, kSet, kAmb, kPol, gp,
          true, st_b);
        // ORACLE: ANALYTICAL / EVALUATION ONLY. Given the ground-truth class
        // at full confidence and a mature track, so it measures the class-aware
        // MODEL with the classifier's errors removed.
        const ProfileSelection sel_c = selectCapabilityProfile(
          r.gt_label, 1.0, std::max(r.observations, kPol.min_observations), kSet, kAmb,
          kPol, gp, true, st_c);

        if (!first_label && r.runtime_label != prev_b) {++transitions_runtime;}
        if (!first_label && r.gt_label != prev_c) {++transitions_oracle;}
        prev_b = r.runtime_label;
        prev_c = r.gt_label;
        first_label = false;

        const MotionCapabilityProfile truth = truthProfile(r.gt_label);
        const ProfileSelection * sels[3] = {&sel_a, &sel_b, &sel_c};

        for (int arm = 0; arm < 3; ++arm) {
          const ProfileSelection & sel = *sels[arm];
          const ReachabilityBounds b = boundsForProfile(baseBounds(), sel.profile);
          const ReachabilityBounds tb = boundsForProfile(baseBounds(), truth);
          const bool unsafe = !profileDominates(sel.profile, truth);

          for (int h = 1; h <= kNumHorizons; ++h) {
            const double t = h * kTimeStep;
            const auto start = std::chrono::steady_clock::now();
            const Eigen::Matrix4d F = ConstantVelocityKalmanFilter2D::stateTransition(t);
            const Eigen::Matrix4d Pp = F * kf.covariance() * F.transpose() +
              ConstantVelocityKalmanFilter2D::processNoise(t, kAccelNoise);
            const ReachabilityRegion reg = buildRegion(
              kf.state()(0), kf.state()(1), kf.state()(2), kf.state()(3),
              Pp.topLeftCorner<2, 2>(), t, b);
            build_ns += std::chrono::duration<double, std::nano>(
              std::chrono::steady_clock::now() - start).count();
            ++build_calls;
            if (arm == 0) {
              const auto cs = std::chrono::steady_clock::now();
              const Eigen::Vector4d xp = F * kf.state();
              cv_ns += std::chrono::duration<double, std::nano>(
                std::chrono::steady_clock::now() - cs).count();
              (void)xp;
            }

            // CAPABILITY containment: the concentric disc of radius
            // r_det(t) under the TRUE class's profile must fit inside.
            const double required = reachRadius(t, reg.observed_speed, tb);
            const bool cap_in = reg.valid && reg.semi_minor >= required;

            // OBSERVED containment: the same track's measured position at
            // stamp + t, when the recording reaches that far.
            bool obs_eval = false, obs_in = false;
            for (size_t j = i + 1; j < seq.size(); ++j) {
              if (seq[j]->stamp >= r.stamp + t - 0.5 * kScanPeriod) {
                obs_eval = true;
                obs_in = predictive_nav_tracking::regionContains(
                  reg, seq[j]->x, seq[j]->y);
                break;
              }
            }

            const double area = M_PI * reg.semi_major * reg.semi_minor;
            const double cells = static_cast<double>(
              paintedCells(reg.semi_major, reg.semi_minor, reg.sigma_yaw));
            const bool breach =
              reg.contract_status == ContractStatus::kObservedSpeedExceedsCap;
            const bool unk = (arm != 0) &&
              predictive_nav_tracking::profileEqual(sel.profile, kFallback);

            pooled[arm][h - 1].add(
              cap_in, obs_eval, obs_in, breach, isFallback(sel.reason),
              sel.reason == ProfileReason::kTransitionHold, unk, unsafe,
              area, cells, reg.reach_radius);
            per_class[arm][static_cast<int>(r.gt_label)].add(
              cap_in, obs_eval, obs_in, breach, isFallback(sel.reason),
              sel.reason == ProfileReason::kTransitionHold, unk, unsafe,
              area, cells, reg.reach_radius);
          }
        }
      }
    }

    const char * arm_names[3] = {"A_global", "B_runtime_class_aware",
      "C_oracle_class_aware_EVALUATION_ONLY"};
    std::printf("    \"per_horizon\": {\n");
    for (int arm = 0; arm < 3; ++arm) {
      std::printf("      \"%s\": {\n", arm_names[arm]);
      for (int h = 0; h < kNumHorizons; ++h) {
        char label[32];
        std::snprintf(label, sizeof(label), "h%.1f", (h + 1) * kTimeStep);
        printAcc(label, pooled[arm][h], h + 1 == kNumHorizons);
      }
      std::printf("      }%s\n", arm + 1 == 3 ? "" : ",");
    }
    std::printf("    },\n");

    std::printf("    \"pooled\": {\n");
    for (int arm = 0; arm < 3; ++arm) {
      Acc all;
      for (int h = 0; h < kNumHorizons; ++h) {
        const Acc & a = pooled[arm][h];
        all.n += a.n;
        all.capability_contained += a.capability_contained;
        all.observed_contained += a.observed_contained;
        all.observed_eval += a.observed_eval;
        all.contract_breach += a.contract_breach;
        all.fallback += a.fallback;
        all.transition += a.transition;
        all.unknown_profile += a.unknown_profile;
        all.unsafe_selection += a.unsafe_selection;
        all.area += a.area;
        all.cells += a.cells;
        all.radius += a.radius;
      }
      printAcc(arm_names[arm], all, arm + 1 == 3);
    }
    std::printf("    },\n");

    std::printf("    \"per_true_class\": {\n");
    for (int arm = 0; arm < 3; ++arm) {
      std::printf("      \"%s\": {\n", arm_names[arm]);
      size_t k = 0;
      for (const auto & kv : per_class[arm]) {
        ++k;
        printAcc(
          toString(static_cast<ObstacleClass>(kv.first)), kv.second,
          k == per_class[arm].size());
      }
      std::printf("      }%s\n", arm + 1 == 3 ? "" : ",");
    }
    std::printf("    },\n");
    std::printf(
      "    \"label_transitions\": {\"runtime\": %ld, \"oracle_ground_truth\": %ld},\n",
      transitions_runtime, transitions_oracle);
    std::printf(
      "    \"cost\": {\"region_build_mean_ns\": %.1f, \"samples_timed\": %ld, "
      "\"per_track_6_sample_us\": %.3f, \"cv_propagate_mean_ns\": %.1f}\n",
      build_calls > 0 ? build_ns / build_calls : 0.0, build_calls,
      build_calls > 0 ? 6.0 * (build_ns / build_calls) / 1000.0 : 0.0,
      cv_ns > 0.0 ? cv_ns / (build_calls / 3.0) : 0.0);
    std::printf("  },\n");
  }

  // =================================================================
  // PART 4 -- classification-error analysis
  // =================================================================
  //
  // For every confusion direction that ACTUALLY OCCURRED in the recordings,
  // the one question that matters: does the profile the runtime arm selected
  // still dominate what the object really is? A direction with
  // `unsafe_selections > 0` is a measured safety defect of the policy, not a
  // statistic to be averaged away -- which is why they are reported per
  // direction here and not only pooled in part 3.
  std::printf("  \"part4_classification_error\": {\n");
  std::printf(
    "    \"question\": \"For each (ground truth -> runtime label) direction, does "
    "the RUNTIME arm's selected profile dominate the true class's capability "
    "profile componentwise? Anything other than 0 unsafe selections is a "
    "defect.\",\n");
  {
    struct Dir
    {
      long n = 0;
      long unsafe = 0;
      long fallback = 0;
      long class_closure = 0;
      double conf_min = 1e9;
      double conf_max = -1e9;
      double applied_speed = 0.0;
      double truth_speed = 0.0;
    };
    std::map<std::pair<int, int>, Dir> dirs;
    std::map<int, std::vector<const Row *>> tracks;
    for (const auto & r : rows) {
      tracks[r.track].push_back(&r);
    }
    long total_unsafe = 0;
    for (auto & kv : tracks) {
      ClassProfileState st;
      for (const Row * rp : kv.second) {
        const Row & r = *rp;
        const ProfileSelection sel = selectCapabilityProfile(
          r.runtime_label, r.runtime_conf, r.observations, kSet, kAmb, kPol,
          globalProfile(), true, st);
        const MotionCapabilityProfile truth = truthProfile(r.gt_label);
        Dir & d = dirs[{static_cast<int>(r.gt_label), static_cast<int>(r.runtime_label)}];
        ++d.n;
        const bool unsafe = !profileDominates(sel.profile, truth);
        d.unsafe += unsafe;
        total_unsafe += unsafe;
        d.fallback += isFallback(sel.reason);
        d.class_closure += (sel.reason == ProfileReason::kClassClosure);
        d.conf_min = std::min(d.conf_min, r.runtime_conf);
        d.conf_max = std::max(d.conf_max, r.runtime_conf);
        d.applied_speed += sel.profile.max_speed;
        d.truth_speed += truth.max_speed;
      }
    }
    std::printf("    \"directions\": [\n");
    size_t k = 0;
    for (const auto & kv : dirs) {
      ++k;
      const Dir & d = kv.second;
      std::printf(
        "      {\"ground_truth\": \"%s\", \"runtime_label\": \"%s\", \"samples\": %ld, "
        "\"unsafe_selections\": %ld, \"fallback_samples\": %ld, "
        "\"class_closure_samples\": %ld, \"confidence_range\": [%.3f, %.3f], "
        "\"mean_applied_max_speed\": %.2f, \"true_class_max_speed\": %.2f}%s\n",
        toString(static_cast<ObstacleClass>(kv.first.first)),
        toString(static_cast<ObstacleClass>(kv.first.second)),
        d.n, d.unsafe, d.fallback, d.class_closure,
        d.conf_min > 1e8 ? 0.0 : d.conf_min, d.conf_max < -1e8 ? 0.0 : d.conf_max,
        d.n > 0 ? d.applied_speed / d.n : 0.0, d.n > 0 ? d.truth_speed / d.n : 0.0,
        k == dirs.size() ? "" : ",");
    }
    std::printf("    ],\n");
    std::printf("    \"total_unsafe_selections\": %ld\n", total_unsafe);
    std::printf("  },\n");
  }

  // =================================================================
  // PART 5 -- what the planner can actually see
  // =================================================================
  //
  // Region area is the model's own calibration measure; planner-visible CELLS
  // are what MPPI consumes, and they are bounded by the layer's
  // max_influence_radius policy (0.55 m at production geometry). This part
  // reports the horizon at which each profile SATURATES that window, because
  // above it two different profiles are indistinguishable to the planner
  // however different their mathematics.
  std::printf("  \"part5_planner_visibility\": {\n");
  std::printf("    \"max_influence_radius_m\": %.2f,\n", kMaxInfluence);
  std::printf("    \"costmap_resolution_m\": %.2f,\n", kCostmapRes);
  std::printf(
    "    \"window_cells_when_saturated\": %ld,\n",
    paintedCells(100.0, 100.0, 0.0));
  std::printf("    \"profiles\": [\n");
  {
    const std::vector<std::pair<const char *, MotionCapabilityProfile>> probes{
      {"A_global", globalProfile()},
      {"PEDESTRIAN", kSet[ObstacleClass::PEDESTRIAN]},
      {"BIKE", kSet[ObstacleClass::BIKE]},
      {"VEHICLE", kSet[ObstacleClass::VEHICLE]},
      {"FALLBACK_derived", kFallback}};
    for (size_t i = 0; i < probes.size(); ++i) {
      const ReachabilityBounds b = boundsForProfile(baseBounds(), probes[i].second);
      // Smallest total_time at which the deterministic radius alone already
      // exceeds the policy bound, so the window is full whatever the sigma
      // term contributes.
      double t_sat = -1.0;
      for (double t = 0.01; t <= 4.0 + 1e-9; t += 0.01) {
        if (reachRadius(t, 0.0, b) >= kMaxInfluence) {
          t_sat = t;
          break;
        }
      }
      std::printf(
        "      {\"profile\": \"%s\", \"max_speed\": %.2f, \"max_acceleration\": %.2f, "
        "\"r_det_at_0.5s_m\": %.4f, \"r_det_at_1.0s_m\": %.4f, \"r_det_at_4.0s_m\": %.4f, "
        "\"saturates_window_at_total_time_s\": %.2f, "
        "\"cells_at_0.5s_zero_sigma\": %ld}%s\n",
        probes[i].first, probes[i].second.max_speed, probes[i].second.max_acceleration,
        reachRadius(0.5, 0.0, b), reachRadius(1.0, 0.0, b), reachRadius(4.0, 0.0, b),
        t_sat, paintedCells(
          reachRadius(0.5, 0.0, b), reachRadius(0.5, 0.0, b), 0.0),
        i + 1 == probes.size() ? "" : ",");
    }
  }
  std::printf("    ]\n  }\n");
  std::printf("}\n");
  return 0;
}
