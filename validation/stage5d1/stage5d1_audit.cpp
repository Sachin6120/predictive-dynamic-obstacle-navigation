// Stage-5D.1 -- SEMANTIC CORRIDOR DISCRIMINABILITY AUDIT.
//
// ANALYSIS ONLY. This file includes the shipped production headers and
// modifies nothing: every profile selection, every reachability number and
// every rasterization below is computed by the production code, so the audit
// describes the shipped system rather than a re-implementation of it.
//
// Build (no CMake or production build file is touched):
//   g++ -O2 -std=c++17 -I/usr/include/eigen3 \
//       -Isrc/predictive_nav_tracking/include \
//       validation/stage5d1/stage5d1_audit.cpp -o <out>/stage5d1_audit
//   <out>/stage5d1_audit <replay.txt from validation/stage5d/extract_labels.py>
//
// THE QUESTION
// ------------
// Stage-5D qualified class-aware reachability mathematically but could not
// show that semantic class discrimination changes planner-visible geometry.
// Two candidate causes were named: the 0.55 m `max_influence_radius` costmap
// policy window, and the classifier ambiguity / conservative fallback policy.
// This audit separates them with a controlled four-arm comparison over ONE
// set of recorded tracks:
//
//   A  runtime classifier labels + current  window (0.55 m)   <- Stage-5D
//   B  ground-truth classes      + current  window (0.55 m)
//   C  runtime classifier labels + expanded window (3.00 m)
//   D  ground-truth classes      + expanded window (3.00 m)   <- upper bound
//
// ORACLE DISCIPLINE. The ground-truth class is read in exactly one place --
// `Row::gt_label` -- and reaches only the oracle arms (B, D) and the penalty
// analysis. The production node is not built against this file, does not read
// the replay table, and has no parameter that could.
//
// NOTHING IS TUNED. The expanded window is the layer's OWN declared default
// (`declareParameter("max_influence_radius", 3.0)`,
// predicted_obstacle_layer.cpp:44), which also happens to be the exact
// half-extent of the 6x6 m local costmap, i.e. the largest window this
// costmap can represent at all. No value was searched for and no navigation
// metric was consulted in choosing it; see part 1.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <set>
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
using predictive_nav_tracking::closureMask;
using predictive_nav_tracking::fallbackProfile;
using predictive_nav_tracking::maskEnvelope;
using predictive_nav_tracking::profileDominates;
using predictive_nav_tracking::profileEqual;
using predictive_nav_tracking::reachRadius;
using predictive_nav_tracking::selectCapabilityProfile;
using predictive_nav_tracking::toString;

namespace
{

// ---------------------------------------------------------------------
// Production constants, all read from the shipped configuration files.
// ---------------------------------------------------------------------
constexpr double kAccelNoise = 0.5;        // kf_process_accel_noise
constexpr double kMeasNoise = 0.05;        // kf_measurement_noise
constexpr double kInitPosVar = 0.01;       // kf_initial_position_variance
constexpr double kInitVelVar = 4.0;        // kf_initial_velocity_variance
constexpr double kTimeStep = 0.5;          // reachability_time_step
constexpr int kNumHorizons = 6;            // reachability_horizon 3.0 / 0.5
constexpr double kMaxObsAge = 1.0;         // max_observation_age
constexpr double kTrackerSigma = 2.0;      // covariance_sigma_level
constexpr double kScanPeriod = 0.2;        // s, 5 Hz LDS
constexpr double kCostmapRes = 0.05;       // costmap resolution

/// PRODUCTION policy window. nav2_stage4f_params.yaml:347 and
/// production_nav2_params.yaml:365. NOT changed by this stage.
constexpr double kWindowCurrent = 0.55;

/// ANALYSIS-ONLY expanded window. The layer's own declared default
/// (predicted_obstacle_layer.cpp:44) and the half-extent of the 6x6 m local
/// costmap, so it is simultaneously the un-tuned default and the ceiling of
/// what this costmap could ever expose. NOT written to any production file.
constexpr double kWindowExpanded = 3.00;

/// Local costmap, nav2_stage4f_params.yaml:256-258.
constexpr double kLocalCostmapSide = 6.0;
constexpr long kLocalCostmapCells = 14400;   // 120 x 120

/// The single global configuration Stage-5C/5C.1 qualified, kept for
/// reference only -- it is not one of the four arms.
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

/// Planner-visible cell count, replicating post-Stage-5C.1
/// `rasterizeReachableSet` exactly (predicted_obstacle_layer.cpp:532-620):
/// the half-extents of the published ellipse's bounding box are clamped to
/// `influence`, and membership is tested against the semi-axes AS PUBLISHED.
/// Counted on a grid aligned to the region centre, which is the layer's own
/// count up to a half-cell phase.
long paintedCells(double semi_major, double semi_minor, double yaw, double influence)
{
  if (!(semi_major > 0.0) || !(semi_minor > 0.0)) {
    return 0;
  }
  const double c = std::cos(yaw), sn = std::sin(yaw);
  double half_x = std::hypot(semi_major * c, semi_minor * sn);
  double half_y = std::hypot(semi_major * sn, semi_minor * c);
  if (influence > 0.0) {
    half_x = std::min(half_x, influence);
    half_y = std::min(half_y, influence);
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

/// Cells of the clamped iteration window itself, i.e. the count a FULLY
/// SATURATED region paints. When `paintedCells` equals this, the region is
/// indistinguishable from any larger region: the window, not the geometry, is
/// what the planner sees.
long windowCells(double semi_major, double semi_minor, double yaw, double influence)
{
  const double c = std::cos(yaw), sn = std::sin(yaw);
  double half_x = std::hypot(semi_major * c, semi_minor * sn);
  double half_y = std::hypot(semi_major * sn, semi_minor * c);
  if (influence > 0.0) {
    half_x = std::min(half_x, influence);
    half_y = std::min(half_y, influence);
  }
  const long ni = static_cast<long>(std::floor(half_x / kCostmapRes));
  const long nj = static_cast<long>(std::floor(half_y / kCostmapRes));
  return (2 * ni + 1) * (2 * nj + 1);
}

/// True when the clamped window is entirely inside the published ellipse, so
/// every cell of it is painted and the region's own extent is invisible.
/// This is the precise meaning of "saturated".
bool saturated(double semi_major, double semi_minor, double yaw, double influence)
{
  if (!(semi_major > 0.0) || !(semi_minor > 0.0) || !(influence > 0.0)) {
    return false;
  }
  return paintedCells(semi_major, semi_minor, yaw, influence) ==
         windowCells(semi_major, semi_minor, yaw, influence) &&
         std::hypot(semi_major * std::cos(yaw), semi_minor * std::sin(yaw)) > influence &&
         std::hypot(semi_major * std::sin(yaw), semi_minor * std::cos(yaw)) > influence;
}

/// First time at which r_det under `p` reaches `target`, or -1 beyond 60 s.
double crossTime(double target, double s0, const MotionCapabilityProfile & p)
{
  ReachabilityBounds b = boundsForProfile(baseBounds(), p);
  if (reachRadius(60.0, s0, b) < target) {
    return -1.0;
  }
  double lo = 0.0, hi = 60.0;
  for (int k = 0; k < 200; ++k) {
    const double m = 0.5 * (lo + hi);
    if (reachRadius(m, s0, b) < target) {lo = m;} else {hi = m;}
  }
  return hi;
}

// =====================================================================
// The replay table (identical format to Stage-5D)
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
  /// ANALYTICAL / EVALUATION ONLY. Read by the oracle arms and the penalty
  /// analysis, nowhere else.
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
  long capability_contained = 0;
  long observed_contained = 0;
  long observed_eval = 0;
  long contract_breach = 0;
  long fallback = 0;
  long unsafe = 0;           // applied profile does NOT dominate truth
  long exact = 0;            // applied profile == oracle-ideal profile
  long over = 0;             // strictly dominates it
  long saturated_samples = 0;
  double area = 0.0;
  double cells = 0.0;
  double win_cells = 0.0;
  double inflation = 0.0;    // area(arm) / area(oracle-ideal)
  double distinct = 0.0;     // distinct class corridors visible at this state
  double raster_ns = 0.0;
};

void printAcc(const char * label, const Acc & a, bool last)
{
  const double n = a.n > 0 ? static_cast<double>(a.n) : 1.0;
  const double on = a.observed_eval > 0 ? static_cast<double>(a.observed_eval) : 1.0;
  std::printf(
    "      \"%s\": {\"samples\": %ld, \"capability_containment\": %.6f, "
    "\"observed_containment\": %.6f, \"observed_evaluable\": %ld, "
    "\"mean_area_m2\": %.4f, \"mean_raster_cells\": %.3f, "
    "\"mean_window_cells\": %.3f, \"saturated_rate\": %.6f, "
    "\"mean_visible_distinct_class_corridors\": %.4f, "
    "\"contract_breaches\": %ld, \"fallback_rate\": %.6f, "
    "\"exact_vs_oracle\": %ld, \"over_conservative\": %ld, "
    "\"under_conservative\": %ld, \"mean_area_inflation_vs_oracle\": %.4f, "
    "\"mean_raster_us\": %.4f}%s\n",
    label, a.n, a.capability_contained / n, a.observed_contained / on,
    a.observed_eval, a.area / n, a.cells / n, a.win_cells / n,
    a.saturated_samples / n, a.distinct / n, a.contract_breach, a.fallback / n,
    a.exact, a.over, a.unsafe, a.inflation / n, a.raster_ns / n / 1000.0,
    last ? "" : ",");
}

void accumulate(Acc & dst, const Acc & s)
{
  dst.n += s.n;
  dst.capability_contained += s.capability_contained;
  dst.observed_contained += s.observed_contained;
  dst.observed_eval += s.observed_eval;
  dst.contract_breach += s.contract_breach;
  dst.fallback += s.fallback;
  dst.unsafe += s.unsafe;
  dst.exact += s.exact;
  dst.over += s.over;
  dst.saturated_samples += s.saturated_samples;
  dst.area += s.area;
  dst.cells += s.cells;
  dst.win_cells += s.win_cells;
  dst.inflation += s.inflation;
  dst.distinct += s.distinct;
  dst.raster_ns += s.raster_ns;
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

  /// What an object of class c can really do. UNKNOWN's honest answer is the
  /// derived fallback envelope, because an UNKNOWN object's true capability is
  /// by definition not known. Identical to Stage-5D's `truthProfile`.
  auto truthProfile = [&](ObstacleClass c) {
      return (c == ObstacleClass::UNKNOWN) ? kFallback : kSet[c];
    };

  /// ORACLE-IDEAL profile: the profile a PERFECT classifier would be entitled
  /// to, i.e. the class's own entry with a SINGLETON ambiguity set, because a
  /// perfect classifier has nothing to hedge against. This is the upper bound
  /// the brief's arms B and D ask for. It is identical to `truthProfile`.
  auto oracleIdeal = truthProfile;

  std::printf("{\n");
  std::printf(
    "  \"note\": \"Stage-5D.1 semantic corridor discriminability audit. ANALYSIS "
    "ONLY: built against the shipped production headers, modifies nothing. The "
    "ORACLE arms (B, D) and the penalty analysis are the ONLY readers of the "
    "ground-truth class and are ANALYTICAL / EVALUATION ONLY -- no ground-truth "
    "label enters any runtime path. The expanded window is applied ONLY inside "
    "this file's rasterization model; no production parameter is changed.\",\n");
  std::printf(
    "  \"windows\": {\"current_m\": %.2f, \"expanded_m\": %.2f, "
    "\"expanded_source\": \"the layer's own declareParameter default, "
    "predicted_obstacle_layer.cpp:44, which equals the half-extent of the 6x6 m "
    "local costmap\"},\n", kWindowCurrent, kWindowExpanded);

  // =================================================================
  // PART 1 -- WINDOW GEOMETRY AND SATURATION (brief sections 1 and 3)
  // =================================================================
  //
  // Purely analytic: when does each class's corridor outgrow a window, so
  // that the window and not the class is what the planner sees?
  //
  //   "binds"      r_det(t) >= R : the clamp starts truncating the iteration
  //                window, so the painted region stops growing in that axis.
  //   "saturates"  the whole clamped window lies inside the ellipse, so EVERY
  //                window cell is painted and two different classes produce
  //                BYTE-IDENTICAL rasters. For a near-circular region this is
  //                r_det(t) >= R*sqrt(2).
  //
  // No navigation metric is consulted anywhere in this part.
  std::printf("  \"part1_window_geometry\": {\n");
  std::printf(
    "    \"policy_role\": {\"what_it_clamps\": \"the ITERATION WINDOW half-extents "
    "half_x/half_y, post-Stage-5C.1; membership is tested against the semi-axes "
    "AS PUBLISHED, so it is a bound on WORK and on PLANNER-VISIBLE EXTENT, never "
    "a truncation of the published region\", "
    "\"declared_default_m\": 3.0, \"production_value_m\": 0.55, "
    "\"local_costmap_side_m\": %.1f, \"local_costmap_cells\": %ld},\n",
    kLocalCostmapSide, kLocalCostmapCells);

  const double kSweep[] = {0.55, 1.0, 1.5, 2.0, 3.0, 5.0};
  std::printf("    \"radius_sweep\": [\n");
  for (size_t ri = 0; ri < sizeof(kSweep) / sizeof(kSweep[0]); ++ri) {
    const double R = kSweep[ri];
    const long side = static_cast<long>(2 * std::floor(R / kCostmapRes) + 1);
    std::printf(
      "      {\"radius_m\": %.2f, \"window_side_cells\": %ld, "
      "\"window_cells\": %ld, \"frac_of_local_costmap\": %.4f, \"classes\": {",
      R, side, side * side,
      static_cast<double>(side * side) / static_cast<double>(kLocalCostmapCells));
    // s0 = 0.50 m/s: the campaign obstacle speed (run_nav.py crossing()
    // default), so the numbers describe the tracks the nav arms actually see.
    for (size_t ci = 0; ci < kClasses.size(); ++ci) {
      const ObstacleClass c = kClasses[ci];
      const MotionCapabilityProfile p = truthProfile(c);
      const double tb = crossTime(R, 0.50, p);
      const double ts = crossTime(R * std::sqrt(2.0), 0.50, p);
      std::printf(
        "\"%s\": {\"binds_at_s\": %.4f, \"saturates_at_s\": %.4f}%s",
        toString(c), tb, ts, ci + 1 == kClasses.size() ? "" : ", ");
    }
    const MotionCapabilityProfile gp = globalProfile();
    std::printf(
      ", \"GLOBAL_5C1_reference\": {\"binds_at_s\": %.4f, \"saturates_at_s\": %.4f}}}%s\n",
      crossTime(R, 0.50, gp), crossTime(R * std::sqrt(2.0), 0.50, gp),
      ri + 1 == sizeof(kSweep) / sizeof(kSweep[0]) ? "" : ",");
  }
  std::printf("    ],\n");

  // Deterministic corridor radius per class at every published horizon, the
  // table the brief's section 3 asks for before a radius is chosen.
  std::printf("    \"r_det_m_by_horizon_s0_0.50\": {\n");
  for (size_t ci = 0; ci < kClasses.size(); ++ci) {
    const ObstacleClass c = kClasses[ci];
    const ReachabilityBounds b = boundsForProfile(baseBounds(), truthProfile(c));
    std::printf("      \"%s\": [", toString(c));
    for (int h = 1; h <= kNumHorizons; ++h) {
      std::printf("%.4f%s", reachRadius(h * kTimeStep, 0.50, b),
        h == kNumHorizons ? "" : ", ");
    }
    std::printf("]%s\n", ci + 1 == kClasses.size() ? "" : ",");
  }
  std::printf("    },\n");

  // The structural fact that no window can repair: which DISTINCT profiles
  // each selection policy can actually express.
  std::printf("    \"expressible_profiles\": {\n");
  {
    std::printf("      \"closure_envelopes\": {");
    for (size_t ci = 0; ci < kClasses.size(); ++ci) {
      const ObstacleClass c = kClasses[ci];
      const MotionCapabilityProfile e = maskEnvelope(closureMask(c, kAmb), kSet);
      std::printf(
        "\"%s\": {\"max_speed\": %.1f, \"max_acceleration\": %.1f, "
        "\"equals_fallback\": %s}%s",
        toString(c), e.max_speed, e.max_acceleration,
        profileEqual(e, kFallback) ? "true" : "false",
        ci + 1 == kClasses.size() ? "" : ", ");
    }
    std::printf("},\n");
    std::set<std::pair<double, double>> runtime_distinct, oracle_distinct;
    for (const ObstacleClass c : kClasses) {
      const MotionCapabilityProfile e = maskEnvelope(closureMask(c, kAmb), kSet);
      runtime_distinct.insert({e.max_speed, e.max_acceleration});
      const MotionCapabilityProfile o = oracleIdeal(c);
      oracle_distinct.insert({o.max_speed, o.max_acceleration});
    }
    std::printf(
      "      \"runtime_closure_policy_distinct_profiles\": %zu,\n"
      "      \"oracle_ideal_distinct_profiles\": %zu,\n"
      "      \"interpretation\": \"A(PEDESTRIAN)={ped,bike,vehicle} and "
      "A(VEHICLE)={vehicle,unknown} both envelope to the fallback (40.0 m/s, "
      "10.0 m/s^2), so under the SHIPPED ambiguity policy the runtime system can "
      "express only TWO corridors -- BIKE and fallback -- and VEHICLE vs UNKNOWN "
      "is identical BY CONSTRUCTION at every radius. This bound is independent of "
      "max_influence_radius.\"\n",
      runtime_distinct.size(), oracle_distinct.size());
  }
  std::printf("    }\n  },\n");

  // =================================================================
  // The replay
  // =================================================================
  const std::vector<Row> rows = readReplay(argv[1]);

  // Arm indices.
  enum {A_RUN_CUR = 0, B_ORA_CUR = 1, C_RUN_EXP = 2, D_ORA_EXP = 3,
    B2_ORACLOS_CUR = 4, D2_ORACLOS_EXP = 5, kNumArms = 6};
  const char * arm_names[kNumArms] = {
    "A_runtime_current",
    "B_oracle_current_EVALUATION_ONLY",
    "C_runtime_expanded",
    "D_oracle_expanded_EVALUATION_ONLY",
    "B2_oracle_closure_current_EVALUATION_ONLY",
    "D2_oracle_closure_expanded_EVALUATION_ONLY"};
  const double arm_window[kNumArms] = {
    kWindowCurrent, kWindowCurrent, kWindowExpanded, kWindowExpanded,
    kWindowCurrent, kWindowExpanded};

  Acc pooled[kNumArms][kNumHorizons];
  std::map<int, Acc> per_class[kNumArms];

  // Pairwise raster separation, measured on the SAME track state: only the
  // profile differs, so any difference is semantic geometry and nothing else.
  struct Pair {ObstacleClass a, b; const char * name;};
  const std::vector<Pair> pairs{
    {ObstacleClass::PEDESTRIAN, ObstacleClass::BIKE, "PED_vs_BIKE"},
    {ObstacleClass::PEDESTRIAN, ObstacleClass::VEHICLE, "PED_vs_VEHICLE"},
    {ObstacleClass::BIKE, ObstacleClass::VEHICLE, "BIKE_vs_VEHICLE"},
    {ObstacleClass::VEHICLE, ObstacleClass::UNKNOWN, "VEHICLE_vs_UNKNOWN"},
    {ObstacleClass::PEDESTRIAN, ObstacleClass::UNKNOWN, "PED_vs_UNKNOWN"},
    {ObstacleClass::BIKE, ObstacleClass::UNKNOWN, "BIKE_vs_UNKNOWN"}};
  struct PairAcc
  {
    long n = 0;
    long differ_cur = 0, differ_exp = 0;
    double dcell_cur = 0.0, dcell_exp = 0.0;
    double cells_a_cur = 0.0, cells_b_cur = 0.0;
    double cells_a_exp = 0.0, cells_b_exp = 0.0;
    double area_ratio = 0.0;
  };
  std::vector<std::array<PairAcc, kNumHorizons>> pacc(pairs.size());

  // Per-class absolute geometry table (brief section 4).
  struct GeomAcc
  {
    long n = 0;
    double semi_major = 0.0, semi_minor = 0.0, area = 0.0;
    double cells_cur = 0.0, cells_exp = 0.0;
    long sat_cur = 0, sat_exp = 0;
  };
  std::map<int, std::array<GeomAcc, kNumHorizons>> geom;

  // Classifier penalty detail (brief section 5), keyed
  // (true class -> runtime label).
  struct PenAcc
  {
    long n = 0, exact = 0, over = 0, under = 0, fb = 0;
    double inflation = 0.0;
  };
  std::map<std::pair<int, int>, PenAcc> confusion;
  std::map<int, PenAcc> pen_by_true;
  std::map<std::string, long> reason_counts;

  {
    std::map<int, std::vector<const Row *>> tracks;
    for (const auto & r : rows) {
      tracks[r.track].push_back(&r);
    }
    std::printf("  \"replay\": {\"rows\": %zu, \"tracks\": %zu},\n",
      rows.size(), tracks.size());

    for (auto & kv : tracks) {
      const auto & seq = kv.second;
      ConstantVelocityKalmanFilter2D kf(kAccelNoise, kMeasNoise);
      bool kf_ready = false;
      ClassProfileState st_runtime, st_oracle_closure;

      for (size_t i = 0; i < seq.size(); ++i) {
        const Row & r = *seq[i];
        if (!kf_ready) {
          kf.initialize(r.x, r.y, kInitPosVar, kInitVelVar);
          kf_ready = true;
        } else {
          const double dt = std::max(r.stamp - seq[i - 1]->stamp, 1e-3);
          kf.predict(std::min(dt, kMaxObsAge));
          kf.update(r.x, r.y);
        }
        if (!r.kalman_init) {
          continue;
        }

        const MotionCapabilityProfile gp = globalProfile();

        // RUNTIME selection: the shipped selector, shipped policy, shipped
        // ambiguity sets, driven by the RECORDED classifier output. Arms A
        // and C share this selection exactly -- only the window differs.
        const ProfileSelection sel_runtime = selectCapabilityProfile(
          r.runtime_label, r.runtime_conf, r.observations, kSet, kAmb, kPol, gp,
          true, st_runtime);
        reason_counts[reasonName(sel_runtime.reason)]++;

        // ORACLE-IDEAL: perfect class, singleton ambiguity set. Arms B and D.
        const MotionCapabilityProfile prof_oracle = oracleIdeal(r.gt_label);

        // ORACLE-CLOSURE: the ground-truth label pushed through the UNMODIFIED
        // shipped selector (= Stage-5D's arm C). Reported alongside so the
        // effect of the ambiguity POLICY is separable from the effect of label
        // ERROR without weakening the policy anywhere.
        const ProfileSelection sel_oracle_closure = selectCapabilityProfile(
          r.gt_label, 1.0, std::max(r.observations, kPol.min_observations),
          kSet, kAmb, kPol, gp, true, st_oracle_closure);

        const MotionCapabilityProfile truth = truthProfile(r.gt_label);
        const MotionCapabilityProfile arm_prof[kNumArms] = {
          sel_runtime.profile, prof_oracle, sel_runtime.profile, prof_oracle,
          sel_oracle_closure.profile, sel_oracle_closure.profile};
        const bool arm_fb[kNumArms] = {
          isFallback(sel_runtime.reason), false, isFallback(sel_runtime.reason),
          false, isFallback(sel_oracle_closure.reason),
          isFallback(sel_oracle_closure.reason)};

        for (int h = 1; h <= kNumHorizons; ++h) {
          const double t = h * kTimeStep;
          const Eigen::Matrix4d F = ConstantVelocityKalmanFilter2D::stateTransition(t);
          const Eigen::Matrix4d Pp = F * kf.covariance() * F.transpose() +
            ConstantVelocityKalmanFilter2D::processNoise(t, kAccelNoise);
          const Eigen::Matrix2d Ppos = Pp.topLeftCorner<2, 2>();

          // --- the oracle region, needed as the inflation denominator ------
          const ReachabilityRegion reg_oracle = buildRegion(
            kf.state()(0), kf.state()(1), kf.state()(2), kf.state()(3),
            Ppos, t, boundsForProfile(baseBounds(), prof_oracle));
          const double area_oracle =
            M_PI * reg_oracle.semi_major * reg_oracle.semi_minor;

          // --- per-class geometry at THIS state (section 4) ---------------
          // Same track, same covariance, same time: the ONLY thing that
          // differs between classes is the capability profile.
          std::array<ReachabilityRegion, 4> by_class;
          for (size_t ci = 0; ci < kClasses.size(); ++ci) {
            by_class[ci] = buildRegion(
              kf.state()(0), kf.state()(1), kf.state()(2), kf.state()(3),
              Ppos, t, boundsForProfile(baseBounds(), truthProfile(kClasses[ci])));
          }
          for (size_t ci = 0; ci < kClasses.size(); ++ci) {
            const ReachabilityRegion & g = by_class[ci];
            GeomAcc & ga = geom[static_cast<int>(kClasses[ci])][h - 1];
            ++ga.n;
            ga.semi_major += g.semi_major;
            ga.semi_minor += g.semi_minor;
            ga.area += M_PI * g.semi_major * g.semi_minor;
            ga.cells_cur += static_cast<double>(
              paintedCells(g.semi_major, g.semi_minor, g.sigma_yaw, kWindowCurrent));
            ga.cells_exp += static_cast<double>(
              paintedCells(g.semi_major, g.semi_minor, g.sigma_yaw, kWindowExpanded));
            ga.sat_cur +=
              saturated(g.semi_major, g.semi_minor, g.sigma_yaw, kWindowCurrent);
            ga.sat_exp +=
              saturated(g.semi_major, g.semi_minor, g.sigma_yaw, kWindowExpanded);
          }

          // --- pairwise separation ----------------------------------------
          auto idx_of = [&](ObstacleClass c) {
              for (size_t k = 0; k < kClasses.size(); ++k) {
                if (kClasses[k] == c) {return k;}
              }
              return size_t{0};
            };
          for (size_t pi = 0; pi < pairs.size(); ++pi) {
            const ReachabilityRegion & ga = by_class[idx_of(pairs[pi].a)];
            const ReachabilityRegion & gb = by_class[idx_of(pairs[pi].b)];
            const long ca = paintedCells(
              ga.semi_major, ga.semi_minor, ga.sigma_yaw, kWindowCurrent);
            const long cb = paintedCells(
              gb.semi_major, gb.semi_minor, gb.sigma_yaw, kWindowCurrent);
            const long ea = paintedCells(
              ga.semi_major, ga.semi_minor, ga.sigma_yaw, kWindowExpanded);
            const long eb = paintedCells(
              gb.semi_major, gb.semi_minor, gb.sigma_yaw, kWindowExpanded);
            PairAcc & pa = pacc[pi][h - 1];
            ++pa.n;
            pa.differ_cur += (ca != cb);
            pa.differ_exp += (ea != eb);
            pa.dcell_cur += std::abs(static_cast<double>(ca - cb));
            pa.dcell_exp += std::abs(static_cast<double>(ea - eb));
            pa.cells_a_cur += static_cast<double>(ca);
            pa.cells_b_cur += static_cast<double>(cb);
            pa.cells_a_exp += static_cast<double>(ea);
            pa.cells_b_exp += static_cast<double>(eb);
            const double aa = ga.semi_major * ga.semi_minor;
            const double ab = gb.semi_major * gb.semi_minor;
            pa.area_ratio += (ab > 0.0) ? aa / ab : 0.0;
          }

          // --- the arms ----------------------------------------------------
          for (int arm = 0; arm < kNumArms; ++arm) {
            const MotionCapabilityProfile & p = arm_prof[arm];
            const double R = arm_window[arm];
            const ReachabilityRegion reg = buildRegion(
              kf.state()(0), kf.state()(1), kf.state()(2), kf.state()(3),
              Ppos, t, boundsForProfile(baseBounds(), p));

            const auto t0 = std::chrono::steady_clock::now();
            const long cells =
              paintedCells(reg.semi_major, reg.semi_minor, reg.sigma_yaw, R);
            const double raster_ns = std::chrono::duration<double, std::nano>(
              std::chrono::steady_clock::now() - t0).count();
            const long wcells =
              windowCells(reg.semi_major, reg.semi_minor, reg.sigma_yaw, R);
            const bool sat =
              saturated(reg.semi_major, reg.semi_minor, reg.sigma_yaw, R);

            // How many DISTINCT class corridors are visible at this state and
            // this window: the discriminability metric. 1 means the planner
            // cannot tell any two classes apart here.
            std::set<long> distinct;
            for (size_t ci = 0; ci < kClasses.size(); ++ci) {
              distinct.insert(
                paintedCells(by_class[ci].semi_major, by_class[ci].semi_minor,
                by_class[ci].sigma_yaw, R));
            }

            const ReachabilityBounds tb = boundsForProfile(baseBounds(), truth);
            const double required = reachRadius(t, reg.observed_speed, tb);
            const bool cap_in = reg.valid && reg.semi_minor >= required;

            bool obs_eval = false, obs_in = false;
            for (size_t j = i + 1; j < seq.size(); ++j) {
              if (seq[j]->stamp >= r.stamp + t - 0.5 * kScanPeriod) {
                obs_eval = true;
                obs_in = predictive_nav_tracking::regionContains(
                  reg, seq[j]->x, seq[j]->y);
                break;
              }
            }

            Acc s;
            s.n = 1;
            s.capability_contained = cap_in;
            s.observed_eval = obs_eval;
            s.observed_contained = obs_eval && obs_in;
            s.contract_breach =
              reg.contract_status == ContractStatus::kObservedSpeedExceedsCap;
            s.fallback = arm_fb[arm];
            s.unsafe = !profileDominates(p, truth);
            s.exact = profileEqual(p, prof_oracle);
            s.over = (!profileEqual(p, prof_oracle) &&
              profileDominates(p, prof_oracle));
            s.saturated_samples = sat;
            s.area = M_PI * reg.semi_major * reg.semi_minor;
            s.cells = static_cast<double>(cells);
            s.win_cells = static_cast<double>(wcells);
            s.inflation = (area_oracle > 0.0) ?
              (M_PI * reg.semi_major * reg.semi_minor) / area_oracle : 1.0;
            s.distinct = static_cast<double>(distinct.size());
            s.raster_ns = raster_ns;

            accumulate(pooled[arm][h - 1], s);
            accumulate(per_class[arm][static_cast<int>(r.gt_label)], s);
          }

          // --- classifier penalty, runtime vs oracle-ideal (section 5) -----
          if (h == 1) {    // once per sample, not once per horizon
            const bool exact = profileEqual(sel_runtime.profile, prof_oracle);
            const bool dom = profileDominates(sel_runtime.profile, prof_oracle);
            PenAcc & pb = pen_by_true[static_cast<int>(r.gt_label)];
            PenAcc & pc = confusion[{static_cast<int>(r.gt_label),
                static_cast<int>(r.runtime_label)}];
            for (PenAcc * pp : {&pb, &pc}) {
              ++pp->n;
              pp->exact += exact;
              pp->over += (!exact && dom);
              pp->under += !dom;
              pp->fb += isFallback(sel_runtime.reason);
            }
          }
        }
      }
    }
  }

  // Area inflation per penalty bucket, pooled over horizons, taken from the
  // arm accumulators so it is the same number the arms report.
  // =================================================================
  // PART 2 -- PROFILE SEPARATION BY HORIZON (brief section 4)
  // =================================================================
  std::printf("  \"part2_profile_separation\": {\n");
  std::printf(
    "    \"method\": \"Every class profile is evaluated on the SAME recorded "
    "track state, the same Kalman covariance and the same horizon, so the only "
    "difference between classes is the capability profile. Cells replicate "
    "post-Stage-5C.1 rasterizeReachableSet.\",\n");
  std::printf("    \"by_class\": {\n");
  {
    size_t k = 0;
    for (const auto & kv : geom) {
      ++k;
      std::printf("      \"%s\": {\n",
        toString(static_cast<ObstacleClass>(kv.first)));
      for (int h = 0; h < kNumHorizons; ++h) {
        const GeomAcc & g = kv.second[h];
        const double n = g.n > 0 ? static_cast<double>(g.n) : 1.0;
        std::printf(
          "        \"h%.1f\": {\"samples\": %ld, \"semi_major_m\": %.4f, "
          "\"semi_minor_m\": %.4f, \"area_m2\": %.4f, \"cells_at_0.55\": %.2f, "
          "\"cells_at_3.00\": %.2f, \"saturated_rate_0.55\": %.4f, "
          "\"saturated_rate_3.00\": %.4f}%s\n",
          (h + 1) * kTimeStep, g.n, g.semi_major / n, g.semi_minor / n,
          g.area / n, g.cells_cur / n, g.cells_exp / n,
          g.sat_cur / n, g.sat_exp / n, h + 1 == kNumHorizons ? "" : ",");
      }
      std::printf("      }%s\n", k == geom.size() ? "" : ",");
    }
  }
  std::printf("    },\n");
  std::printf("    \"pairwise\": {\n");
  for (size_t pi = 0; pi < pairs.size(); ++pi) {
    std::printf("      \"%s\": {\n", pairs[pi].name);
    for (int h = 0; h < kNumHorizons; ++h) {
      const PairAcc & pa = pacc[pi][h];
      const double n = pa.n > 0 ? static_cast<double>(pa.n) : 1.0;
      std::printf(
        "        \"h%.1f\": {\"samples\": %ld, "
        "\"raster_differs_rate_0.55\": %.6f, \"raster_differs_rate_3.00\": %.6f, "
        "\"mean_abs_cell_diff_0.55\": %.3f, \"mean_abs_cell_diff_3.00\": %.3f, "
        "\"mean_cells_0.55\": [%.2f, %.2f], \"mean_cells_3.00\": [%.2f, %.2f], "
        "\"mean_continuous_area_ratio\": %.4f}%s\n",
        (h + 1) * kTimeStep, pa.n, pa.differ_cur / n, pa.differ_exp / n,
        pa.dcell_cur / n, pa.dcell_exp / n,
        pa.cells_a_cur / n, pa.cells_b_cur / n,
        pa.cells_a_exp / n, pa.cells_b_exp / n,
        pa.area_ratio / n, h + 1 == kNumHorizons ? "" : ",");
    }
    std::printf("      }%s\n", pi + 1 == pairs.size() ? "" : ",");
  }
  std::printf("    }\n  },\n");

  // =================================================================
  // PART 3 -- CLASSIFIER PENALTY (brief section 5)
  // =================================================================
  std::printf("  \"part3_classifier_penalty\": {\n");
  std::printf(
    "    \"definition\": \"Per recorded sample: EXACT = the runtime-selected "
    "profile equals the oracle-ideal (true class, singleton ambiguity set) "
    "profile; OVER-CONSERVATIVE = it strictly dominates it; UNDER-CONSERVATIVE = "
    "it does not dominate it. UNDER-CONSERVATIVE must be 0.\",\n");
  std::printf("    \"selection_reasons\": {");
  {
    size_t k = 0;
    for (const auto & kv : reason_counts) {
      ++k;
      std::printf("\"%s\": %ld%s", kv.first.c_str(), kv.second,
        k == reason_counts.size() ? "" : ", ");
    }
  }
  std::printf("},\n");
  std::printf("    \"by_true_class\": {\n");
  {
    size_t k = 0;
    for (const auto & kv : pen_by_true) {
      ++k;
      const PenAcc & p = kv.second;
      const double n = p.n > 0 ? static_cast<double>(p.n) : 1.0;
      std::printf(
        "      \"%s\": {\"samples\": %ld, \"exact\": %ld, \"over_conservative\": "
        "%ld, \"under_conservative\": %ld, \"fallback\": %ld, \"exact_rate\": "
        "%.6f, \"over_rate\": %.6f, \"fallback_rate\": %.6f}%s\n",
        toString(static_cast<ObstacleClass>(kv.first)), p.n, p.exact, p.over,
        p.under, p.fb, p.exact / n, p.over / n, p.fb / n,
        k == pen_by_true.size() ? "" : ",");
    }
  }
  std::printf("    },\n");
  std::printf("    \"confusion_true_to_runtime_label\": {\n");
  {
    size_t k = 0;
    for (const auto & kv : confusion) {
      ++k;
      const PenAcc & p = kv.second;
      std::printf(
        "      \"%s->%s\": {\"samples\": %ld, \"exact\": %ld, "
        "\"over_conservative\": %ld, \"under_conservative\": %ld, "
        "\"fallback\": %ld}%s\n",
        toString(static_cast<ObstacleClass>(kv.first.first)),
        toString(static_cast<ObstacleClass>(kv.first.second)),
        p.n, p.exact, p.over, p.under, p.fb,
        k == confusion.size() ? "" : ",");
    }
  }
  std::printf("    }\n  },\n");

  // =================================================================
  // PART 4 -- THE FOUR ARMS (brief section 7)
  // =================================================================
  std::printf("  \"part4_four_arms\": {\n");
  std::printf(
    "    \"arms\": {\"A\": \"runtime labels + 0.55 m window (= Stage-5D "
    "production-candidate behaviour)\", \"B\": \"oracle-ideal class + 0.55 m\", "
    "\"C\": \"runtime labels + 3.00 m\", \"D\": \"oracle-ideal class + 3.00 m\", "
    "\"B2/D2\": \"ground-truth label pushed through the UNMODIFIED shipped "
    "selector (closure envelope, = Stage-5D arm C), at 0.55 m and 3.00 m -- "
    "reported so the ambiguity POLICY and the label ERROR are separable without "
    "weakening the policy\"},\n");
  std::printf("    \"per_horizon\": {\n");
  for (int arm = 0; arm < kNumArms; ++arm) {
    std::printf("      \"%s\": {\n", arm_names[arm]);
    for (int h = 0; h < kNumHorizons; ++h) {
      char label[32];
      std::snprintf(label, sizeof(label), "h%.1f", (h + 1) * kTimeStep);
      std::printf("  ");
      printAcc(label, pooled[arm][h], h + 1 == kNumHorizons);
    }
    std::printf("      }%s\n", arm + 1 == kNumArms ? "" : ",");
  }
  std::printf("    },\n");
  std::printf("    \"pooled\": {\n");
  for (int arm = 0; arm < kNumArms; ++arm) {
    Acc all;
    for (int h = 0; h < kNumHorizons; ++h) {
      accumulate(all, pooled[arm][h]);
    }
    printAcc(arm_names[arm], all, arm + 1 == kNumArms);
  }
  std::printf("    },\n");
  std::printf("    \"per_true_class\": {\n");
  for (int arm = 0; arm < kNumArms; ++arm) {
    std::printf("      \"%s\": {\n", arm_names[arm]);
    size_t k = 0;
    for (const auto & kv : per_class[arm]) {
      ++k;
      std::printf("  ");
      printAcc(toString(static_cast<ObstacleClass>(kv.first)), kv.second,
        k == per_class[arm].size());
    }
    std::printf("      }%s\n", arm + 1 == kNumArms ? "" : ",");
  }
  std::printf("    }\n  }\n");
  std::printf("}\n");
  return 0;
}
