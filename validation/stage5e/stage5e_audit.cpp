// Stage-5E -- FINAL three-arm PREDICTION-QUALITY audit, one replay set.
//
// ANALYSIS ONLY: this file includes the shipped production headers and
// modifies nothing. No new predictor, no new mathematics, no re-derivation --
// every region below is built by the SHIPPED `buildRegion()` and every
// containment test by the SHIPPED `regionContains()`, exactly as at Stage-5D.
//
// Build (no CMake or production build file is touched):
//   g++ -O2 -std=c++17 -I/usr/include/eigen3 \
//       -Isrc/predictive_nav_tracking/include \
//       validation/stage5e/stage5e_audit.cpp -o <out>/stage5e_audit
//   <out>/stage5e_audit <replay.txt from validation/stage5d/extract_labels.py>
//
// WHY THIS FILE EXISTS
// --------------------
// Stage-5E must put the three SHIPPABLE predictors on ONE containment axis.
// The two existing audits each cover a different pair on a different input
// set:
//
//   validation/stage5c/model_audit.json  part 3 -- CV vs GLOBAL reachability,
//       on synthetic in-model / out-of-model motion families;
//   validation/stage5d/profile_audit.json part 3 -- GLOBAL vs RUNTIME
//       class-aware vs ORACLE, on the recorded Stage-5B replay.
//
// Reading a CV-vs-class-aware comparison across those two is not legitimate.
// This audit therefore runs ALL THREE shippable arms over the SAME recorded
// Stage-5B replay rows the Stage-5D audit used, so every number in the
// Stage-5E containment row shares one input. The ORACLE arm is deliberately
// ABSENT: Stage-5E section 2 excludes it from the architecture comparison
// because it is not a deployable configuration. No ground-truth label is read
// by any arm; `Row::gt_label` is read only to define the TRUE capability a
// containment question is asked against, which is a property of the
// evaluation, not of any arm's input.
//
// THE THREE ARMS
//   A CV            -- the Stage-4C constant-velocity Kalman prediction, i.e.
//                      the covariance ellipse alone. Reported at BOTH sigma
//                      levels that exist in the system: k = 2.0, the
//                      tracker's published `uncertainty_ellipse_sigma`
//                      ("own"), and k = 1.5, the layer's production
//                      `sigma_level` ("consumed"). This is the SHIPPED
//                      PRODUCTION PREDICTOR.
//   B GLOBAL        -- the single Stage-5C/5C.1 qualified configuration
//                      (max_speed 0.8, max_acceleration 0.5, margin 0).
//   C CLASS-AWARE   -- the recorded Stage-5B label and confidence through the
//                      UNMODIFIED shipped selector with the shipped safe
//                      ambiguity/fallback policy (Stage-5D arm B).
//
// TWO CONTAINMENT DEFINITIONS, kept apart for the same reason Stage-5D kept
// them apart -- conflating them is how a safety claim gets overstated:
//
//   CAPABILITY CONTAINMENT -- does the arm's region cover everything an object
//     of the TRUE class could do within the horizon? Exactly: is the region's
//     MINOR semi-axis at least r_det(t) under the true class's profile. This
//     is the safety question. CV is not a reachable set and is EXPECTED to
//     fail it; the number is reported rather than omitted, because "CV does
//     not bound capability" is a finding, not an embarrassment.
//   OBSERVED CONTAINMENT -- does the track's own later MEASURED position fall
//     inside the region predicted for that time? This is the calibration
//     question, answered from recorded data with no ground truth of any kind,
//     and it is the question CV is built to answer.
//
// PROVENANCE OF THE REUSED CODE. `Row`, `readReplay`, `Acc`, `paintedCells`,
// `globalProfile`, `baseBounds`, `isFallback` and `truthProfile` are taken
// UNCHANGED from validation/stage5d/stage5d_audit.cpp; `insideCvEllipse`,
// `cvPaintedCells` and `ellipseArea` are taken UNCHANGED from
// validation/stage5c/stage5c_audit.cpp (lines 264-320), where they were
// qualified as replicating `rasterizeEllipse` exactly. Copying them verbatim
// is deliberate: re-deriving either rasteriser here would make the Stage-5E
// cell counts incomparable with the Stage-5C and Stage-5D ones.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
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
using predictive_nav_tracking::regionContains;
using predictive_nav_tracking::toString;

namespace
{

// ---- frozen production parameters -----------------------------------------
// Read from tracker_params.yaml and nav2_stage4f_params.yaml. NOT retuned.
// Identical to the Stage-5C and Stage-5D audit constants.
constexpr double kScanPeriod = 0.2;        // s, 5 Hz LDS
constexpr double kAccelNoise = 0.5;        // kf_process_accel_noise
constexpr double kMeasNoise = 0.05;        // kf_measurement_noise
constexpr double kInitPosVar = 0.01;       // kf_initial_position_variance
constexpr double kInitVelVar = 4.0;        // kf_initial_velocity_variance
constexpr double kTimeStep = 0.5;          // reachability_time_step
constexpr int kNumHorizons = 6;            // reachability_horizon 3.0 / 0.5
constexpr double kMaxObsAge = 1.0;         // max_observation_age
constexpr double kTrackerSigma = 2.0;      // uncertainty_ellipse_sigma (published)
constexpr double kLayerSigmaProd = 1.5;    // layer sigma_level (consumed)
constexpr double kMaxInfluence = 0.55;     // layer max_influence_radius
constexpr double kCostmapRes = 0.05;       // costmap resolution
constexpr double kMinPosVariance = 1e-6;   // layer min_position_variance guard

/// The single global configuration Stage-5C/5C.1 qualified: arm B.
/// From validation/stage5d/stage5d_audit.cpp, unchanged.
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

bool isFallback(ProfileReason r)
{
  return r >= ProfileReason::kFallbackUnknownLabel &&
         r <= ProfileReason::kFallbackInvalidConfig;
}

/// Planner-visible cell count for a REACHABILITY region, replicating the
/// post-Stage-5C.1 `rasterizeReachableSet` exactly. From
/// validation/stage5d/stage5d_audit.cpp, unchanged.
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

// ---- CV helpers, from validation/stage5c/stage5c_audit.cpp, unchanged ------

double ellipseArea(const Eigen::Matrix2d & cov, double k)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
  if (es.info() != Eigen::Success) {
    return 0.0;
  }
  const Eigen::Vector2d ev = es.eigenvalues().cwiseMax(kMinPosVariance);
  return M_PI * k * k * std::sqrt(ev(0) * ev(1));
}

bool insideCvEllipse(
  const Eigen::Vector2d & mu, const Eigen::Matrix2d & cov,
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

/// Replicates `rasterizeEllipse`'s painted-cell count EXACTLY: the bounding
/// box is clamped to max_influence_radius but the Mahalanobis test uses the
/// UNCLAMPED covariance -> painted region = ellipse INTERSECT clamped box.
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
      if (static_cast<double>(d.transpose() * inv * d) <= k * k) {
        ++cells;
      }
    }
  }
  return cells;
}

/// The CV region's minor semi-axis at sigma level k, for the capability
/// question. The covariance ellipse has NO reach term, so this is the whole
/// radius CV offers in its narrowest direction.
double cvSemiMinor(const Eigen::Matrix2d & cov, double k)
{
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(cov);
  if (es.info() != Eigen::Success) {
    return 0.0;
  }
  const Eigen::Vector2d ev = es.eigenvalues().cwiseMax(kMinPosVariance);
  return k * std::sqrt(ev(0));
}

// =====================================================================
// The replay table. From validation/stage5d/stage5d_audit.cpp, unchanged.
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
  /// Read ONLY to define the TRUE capability a containment question is asked
  /// against. No arm takes it as input -- there is no oracle arm here.
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
// Per-arm accumulator. Stage-5D's `Acc` plus the second CV sigma level.
// =====================================================================
struct Acc
{
  long n = 0;
  long capability_contained = 0;   // region covers the GT class's capability disc
  long observed_contained = 0;     // the track's own later position is inside
  long observed_contained_alt = 0; // CV only: the same at the OTHER sigma level
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
    bool cap_in, bool obs_eval, bool obs_in, bool obs_in_alt, bool breach,
    bool fb, bool tr, bool unk, bool unsafe, double a, double cl, double rr)
  {
    ++n;
    capability_contained += cap_in;
    observed_eval += obs_eval;
    observed_contained += (obs_eval && obs_in);
    observed_contained_alt += (obs_eval && obs_in_alt);
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

void printAcc(const char * label, const Acc & a, bool cv_arm, bool last)
{
  const double n = a.n > 0 ? static_cast<double>(a.n) : 1.0;
  const double on = a.observed_eval > 0 ? static_cast<double>(a.observed_eval) : 1.0;
  std::printf(
    "      \"%s\": {\"samples\": %ld, \"capability_containment\": %.6f, "
    "\"observed_containment\": %.6f, ",
    label, a.n, a.capability_contained / n, a.observed_contained / on);
  if (cv_arm) {
    // The CV arm exists at two sigma levels in the shipped system and both
    // are reported, because which one is "CV's containment" depends on
    // whether the question is about the tracker's published ellipse or the
    // one the costmap actually paints.
    std::printf("\"observed_containment_k1.5_consumed\": %.6f, ",
      a.observed_contained_alt / on);
  }
  std::printf(
    "\"observed_evaluable\": %ld, \"mean_area_m2\": %.6f, \"mean_cells\": %.3f, "
    "\"mean_reach_radius_m\": %.4f, \"contract_breaches\": %ld, "
    "\"fallback_rate\": %.6f, \"transition_rate\": %.6f, "
    "\"unknown_profile_rate\": %.6f, \"unsafe_selections\": %ld}%s\n",
    a.observed_eval, a.area / n, a.cells / n, a.radius / n, a.contract_breach,
    a.fallback / n, a.transition / n, a.unknown_profile / n, a.unsafe_selection,
    last ? "" : ",");
}

constexpr int kNumArms = 3;
const char * kArmNames[kNumArms] = {
  "A_cv_kalman", "B_global_reachability", "C_runtime_class_aware"};

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

  /// The profile that describes what an object of class c can really do. For
  /// UNKNOWN this is the derived fallback, because an UNKNOWN object's true
  /// capability is by definition not known and the envelope is the only honest
  /// answer. From validation/stage5d/stage5d_audit.cpp, unchanged.
  auto truthProfile = [&](ObstacleClass c) {
      return (c == ObstacleClass::UNKNOWN) ? kFallback : kSet[c];
    };

  std::printf("{\n");
  std::printf(
    "  \"note\": \"Stage-5E final three-arm prediction-quality audit. ANALYSIS "
    "ONLY: built against the shipped production headers, modifies nothing, "
    "introduces no predictor. All three SHIPPABLE arms -- CV-Kalman, global "
    "reachability and runtime class-aware reachability -- are evaluated over "
    "the SAME recorded Stage-5B replay rows the Stage-5D audit used, so the "
    "Stage-5E containment comparison shares one input set. There is NO ORACLE "
    "ARM: Stage-5E excludes it from the architecture comparison because it is "
    "not a deployable configuration. No arm reads the ground-truth class; "
    "gt_label is read only to define the TRUE capability that a containment "
    "question is asked against, which is a property of the evaluation.\",\n");
  std::printf(
    "  \"arms\": {\"A_cv_kalman\": \"Stage-4C constant-velocity Kalman "
    "covariance ellipse -- THE SHIPPED PRODUCTION PREDICTOR. Reported at "
    "k=2.0 (tracker uncertainty_ellipse_sigma, published) and k=1.5 (layer "
    "sigma_level, consumed).\", \"B_global_reachability\": \"single "
    "Stage-5C/5C.1 qualified configuration, max_speed 0.8, max_acceleration "
    "0.5, margin 0.\", \"C_runtime_class_aware\": \"recorded Stage-5B label "
    "and confidence through the UNMODIFIED shipped selector with the shipped "
    "safe ambiguity/fallback policy (= Stage-5D arm B).\"},\n");
  std::printf(
    "  \"containment_definitions\": {\"capability\": \"region minor semi-axis "
    ">= r_det(t) under the TRUE class profile: does the region cover "
    "everything an object of that class could do? The SAFETY question. CV is "
    "not a reachable set and is EXPECTED to fail it.\", \"observed\": \"the "
    "track's own later MEASURED position falls inside the region predicted "
    "for that time. The CALIBRATION question, answered with no ground truth "
    "of any kind.\"},\n");
  std::printf(
    "  \"frozen_config\": {\"scan_period_s\": %.2f, \"time_step_s\": %.2f, "
    "\"horizons\": %d, \"tracker_sigma_k\": %.2f, \"layer_sigma_k\": %.2f, "
    "\"max_influence_radius_m\": %.2f, \"costmap_resolution_m\": %.2f},\n",
    kScanPeriod, kTimeStep, kNumHorizons, kTrackerSigma, kLayerSigmaProd,
    kMaxInfluence, kCostmapRes);

  const std::vector<Row> rows = readReplay(argv[1]);
  std::printf("  \"replay_rows\": %zu,\n", rows.size());

  // Group by track, in order.
  std::map<int, std::vector<const Row *>> tracks;
  for (const auto & r : rows) {
    tracks[r.track].push_back(&r);
  }
  std::printf("  \"replay_tracks\": %zu,\n", tracks.size());

  Acc pooled[kNumArms][kNumHorizons];
  std::map<int, Acc> per_class[kNumArms];
  double cv_ns = 0.0, reach_ns = 0.0;
  long cv_calls = 0, reach_calls = 0;

  for (auto & kv : tracks) {
    const auto & seq = kv.second;
    ConstantVelocityKalmanFilter2D kf(kAccelNoise, kMeasNoise);
    bool kf_ready = false;
    ClassProfileState st_c;

    for (size_t i = 0; i < seq.size(); ++i) {
      const Row & r = *seq[i];
      // --- the production filter, driven by the recorded centroid ----------
      if (!kf_ready) {
        kf.initialize(r.x, r.y, kInitPosVar, kInitVelVar);
        kf_ready = true;
      } else {
        const double dt = std::max(r.stamp - seq[i - 1]->stamp, 1e-3);
        kf.predict(std::min(dt, kMaxObsAge));
        kf.update(r.x, r.y);
      }
      if (!r.kalman_init) {
        continue;   // the node publishes nothing before this either
      }

      // --- the two reachability selections ---------------------------------
      const MotionCapabilityProfile gp = globalProfile();
      ClassProfileState dummy;
      const ProfileSelection sel_b = [&] {
          ProfileSelectionPolicy off;
          off.class_aware = false;
          return selectCapabilityProfile(
            r.runtime_label, r.runtime_conf, r.observations, kSet, kAmb, off, gp,
            true, dummy);
        }();
      const ProfileSelection sel_c = selectCapabilityProfile(
        r.runtime_label, r.runtime_conf, r.observations, kSet, kAmb, kPol, gp,
        true, st_c);

      const MotionCapabilityProfile truth = truthProfile(r.gt_label);
      const ReachabilityBounds tb = boundsForProfile(baseBounds(), truth);

      for (int h = 1; h <= kNumHorizons; ++h) {
        const double t = h * kTimeStep;

        // The CV propagation, timed. This is exactly what the production
        // tracker does for every published prediction sample.
        const auto cv_start = std::chrono::steady_clock::now();
        const Eigen::Matrix4d F = ConstantVelocityKalmanFilter2D::stateTransition(t);
        const Eigen::Vector4d xp = F * kf.state();
        const Eigen::Matrix4d Pp = F * kf.covariance() * F.transpose() +
          ConstantVelocityKalmanFilter2D::processNoise(t, kAccelNoise);
        cv_ns += std::chrono::duration<double, std::nano>(
          std::chrono::steady_clock::now() - cv_start).count();
        ++cv_calls;

        const Eigen::Matrix2d Pcv = Pp.topLeftCorner<2, 2>();
        const Eigen::Vector2d mu(xp(0), xp(1));

        // --- the track's own later measured position, if the recording
        // reaches that far. ONE lookup, shared by all three arms, so the
        // observed-containment denominators are identical by construction.
        bool obs_eval = false;
        double gx = 0.0, gy = 0.0;
        for (size_t j = i + 1; j < seq.size(); ++j) {
          if (seq[j]->stamp >= r.stamp + t - 0.5 * kScanPeriod) {
            obs_eval = true;
            gx = seq[j]->x;
            gy = seq[j]->y;
            break;
          }
        }

        for (int arm = 0; arm < kNumArms; ++arm) {
          bool cap_in = false, obs_in = false, obs_in_alt = false;
          bool breach = false, fb = false, tr = false, unk = false;
          bool unsafe = false;
          double area = 0.0, cells = 0.0, radius = 0.0;

          if (arm == 0) {
            // ARM A -- CV. No capability premise of any kind, so the
            // capability question is answered against the covariance ellipse
            // alone and `unsafe_selection` is TRUE by construction: a
            // covariance ellipse does not dominate any class's capability.
            const double required = reachRadius(t, r.speed, tb);
            cap_in = cvSemiMinor(Pcv, kTrackerSigma) >= required;
            obs_in = obs_eval && insideCvEllipse(mu, Pcv, kTrackerSigma, gx, gy);
            obs_in_alt = obs_eval && insideCvEllipse(mu, Pcv, kLayerSigmaProd, gx, gy);
            area = ellipseArea(Pcv, kTrackerSigma);
            cells = static_cast<double>(
              cvPaintedCells(Pcv, kLayerSigmaProd, kMaxInfluence));
            radius = 0.0;          // CV has no reach term
            unsafe = true;
          } else {
            const ProfileSelection & sel = (arm == 1) ? sel_b : sel_c;
            const ReachabilityBounds b = boundsForProfile(baseBounds(), sel.profile);
            const auto rs = std::chrono::steady_clock::now();
            const ReachabilityRegion reg = buildRegion(
              kf.state()(0), kf.state()(1), kf.state()(2), kf.state()(3),
              Pcv, t, b);
            reach_ns += std::chrono::duration<double, std::nano>(
              std::chrono::steady_clock::now() - rs).count();
            ++reach_calls;
            const double required = reachRadius(t, reg.observed_speed, tb);
            cap_in = reg.valid && reg.semi_minor >= required;
            obs_in = obs_eval && regionContains(reg, gx, gy);
            obs_in_alt = obs_in;
            area = M_PI * reg.semi_major * reg.semi_minor;
            cells = static_cast<double>(
              paintedCells(reg.semi_major, reg.semi_minor, reg.sigma_yaw));
            radius = reg.reach_radius;
            breach = reg.contract_status == ContractStatus::kObservedSpeedExceedsCap;
            fb = isFallback(sel.reason);
            tr = sel.reason == ProfileReason::kTransitionHold;
            unk = (arm == 2) &&
              predictive_nav_tracking::profileEqual(sel.profile, kFallback);
            unsafe = !profileDominates(sel.profile, truth);
          }

          pooled[arm][h - 1].add(
            cap_in, obs_eval, obs_in, obs_in_alt, breach, fb, tr, unk, unsafe,
            area, cells, radius);
          per_class[arm][static_cast<int>(r.gt_label)].add(
            cap_in, obs_eval, obs_in, obs_in_alt, breach, fb, tr, unk, unsafe,
            area, cells, radius);
        }
      }
    }
  }

  std::printf("  \"per_horizon\": {\n");
  for (int arm = 0; arm < kNumArms; ++arm) {
    std::printf("    \"%s\": {\n", kArmNames[arm]);
    for (int h = 0; h < kNumHorizons; ++h) {
      char label[32];
      std::snprintf(label, sizeof(label), "h%.1f", (h + 1) * kTimeStep);
      printAcc(label, pooled[arm][h], arm == 0, h + 1 == kNumHorizons);
    }
    std::printf("    }%s\n", arm + 1 == kNumArms ? "" : ",");
  }
  std::printf("  },\n");

  std::printf("  \"pooled\": {\n");
  for (int arm = 0; arm < kNumArms; ++arm) {
    Acc all;
    for (int h = 0; h < kNumHorizons; ++h) {
      const Acc & a = pooled[arm][h];
      all.n += a.n;
      all.capability_contained += a.capability_contained;
      all.observed_contained += a.observed_contained;
      all.observed_contained_alt += a.observed_contained_alt;
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
    printAcc(kArmNames[arm], all, arm == 0, arm + 1 == kNumArms);
  }
  std::printf("  },\n");

  std::printf("  \"per_true_class\": {\n");
  for (int arm = 0; arm < kNumArms; ++arm) {
    std::printf("    \"%s\": {\n", kArmNames[arm]);
    size_t k = 0;
    for (const auto & kv : per_class[arm]) {
      ++k;
      printAcc(
        toString(static_cast<ObstacleClass>(kv.first)), kv.second, arm == 0,
        k == per_class[arm].size());
    }
    std::printf("    }%s\n", arm + 1 == kNumArms ? "" : ",");
  }
  std::printf("  },\n");

  std::printf(
    "  \"cost\": {\"cv_propagate_mean_ns\": %.1f, \"cv_calls\": %ld, "
    "\"reach_build_mean_ns\": %.1f, \"reach_calls\": %ld, "
    "\"reach_overhead_per_6_sample_track_us\": %.3f}\n",
    cv_calls > 0 ? cv_ns / cv_calls : 0.0, cv_calls,
    reach_calls > 0 ? reach_ns / reach_calls : 0.0, reach_calls,
    reach_calls > 0 ? (reach_ns / reach_calls) * kNumHorizons / 1000.0 : 0.0);
  std::printf("}\n");
  return 0;
}
