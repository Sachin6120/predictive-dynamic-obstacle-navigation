// Stage-5B.1 integration regression, against the production translation unit.
//
// THE THINGS THIS FILE EXISTS TO PROVE:
//
//   1. THE FROZEN PARAMETERS ARE STILL FROZEN. cluster_distance is declared
//      at exactly 0.35 m, and the Stage-4G3 deblend parameters are unchanged.
//      Asserted against the node's own declared defaults, so an accidental
//      edit to the production file fails a test rather than a review.
//
//   2. WITH compound_enabled = false THE STAGE IS A PROVABLE NO-OP. The same
//      cluster list is pushed through apply_compound_association() with the
//      stage off, and the returned measurement vector must be IDENTICAL
//      element for element -- not merely equivalent.
//
//   3. WITH IT ENABLED, A BICYCLE IS RECOVERED, through the production
//      pipeline functions, not a test reimplementation: build_clusters()
//      produces the fragments, measurements_from_clusters() produces the
//      measurements and their cluster origins, and
//      apply_compound_association() produces the compound observation.
//
//   4. A NON-BICYCLE SCENE IS UNCHANGED EVEN WITH THE STAGE ENABLED, so
//      turning it on cannot perturb the rest of the validation set.
//
//   5. THE KALMAN / PREDICTION / REACHABILITY PATH IS UNTOUCHED. A fixed
//      measurement sequence is driven through the production association
//      path with the stage enabled and disabled, and the filter state,
//      covariance, CV predictions and reachable regions are compared BIT FOR
//      BIT -- the same contract stage5b_tracker_integration.cpp pins for
//      classification.
//
//   6. A DEBLENDED CHILD IS NEVER PAIRED. A Stage-4G3 split emits children
//      with no cluster origin, and the compound stage must skip them.

#define main stage5b1_original_main
#include "../src/lidar_obstacle_tracker_node.cpp"
#undef main
#include <cstdio>
#include <stdexcept>
#include <vector>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

namespace
{

constexpr double kInc = 2.0 * M_PI / 360.0;

struct Seg
{
  double ax, ay, bx, by;
};

std::vector<Seg> rect(double cx, double cy, double len, double wid, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hx = 0.5 * len;
  const double hy = 0.5 * wid;
  const double px[4] = {-hx, hx, hx, -hx};
  const double py[4] = {-hy, -hy, hy, hy};
  std::vector<Seg> out;
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    out.push_back(
      {cx + px[i] * c - py[i] * s, cy + px[i] * s + py[i] * c,
        cx + px[j] * c - py[j] * s, cy + px[j] * s + py[j] * c});
  }
  return out;
}

std::vector<Seg> operator+(std::vector<Seg> a, const std::vector<Seg> & b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

/// The Stage-5B actor_bike footprint: two 0.57 x 0.04 m wheel chords at
/// +/-0.525 m, i.e. a 1.05 m wheelbase and a 0.48 m hole.
std::vector<Seg> bike(double cx, double cy, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return rect(cx + 0.525 * c, cy + 0.525 * s, 0.57, 0.04, yaw) +
         rect(cx - 0.525 * c, cy - 0.525 * s, 0.57, 0.04, yaw);
}

/// Nearest-hit ray cast from the sensor origin, one return per beam.
std::vector<Point2D> scan_world(const std::vector<Seg> & segs, double sx = 0.0, double sy = 0.0)
{
  std::vector<Point2D> hits;
  const int n = static_cast<int>(std::lround(2.0 * M_PI / kInc));
  for (int i = 0; i < n; ++i) {
    const double t = -M_PI + i * kInc;
    const double dx = std::cos(t);
    const double dy = std::sin(t);
    double best = std::numeric_limits<double>::infinity();
    for (const auto & seg : segs) {
      const double ex = seg.bx - seg.ax;
      const double ey = seg.by - seg.ay;
      const double ax = seg.ax - sx;
      const double ay = seg.ay - sy;
      const double det = ex * dy - dx * ey;
      if (std::abs(det) < 1e-12) {
        continue;
      }
      const double u = (-ax * ey + ex * ay) / det;
      const double v = (dx * ay - dy * ax) / det;
      if (u > 1e-9 && v >= 0.0 && v <= 1.0 && u < best) {
        best = u;
      }
    }
    if (std::isfinite(best)) {
      hits.push_back({sx + best * dx, sy + best * dy});
    }
  }
  return hits;
}

rclcpp::Time at(double t)
{
  return rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
}

/// Run the PRODUCTION front-end over one world: primary clustering, then
/// Stage-4G3 deblending, then the Stage-5B.1 compound stage.
struct FrontEnd
{
  std::vector<Cluster> clusters;
  std::vector<size_t> origins;
  std::vector<Measurement> primary;
  std::vector<Measurement> compounded;
};

FrontEnd run_frontend(
  LidarObstacleTracker & node, const std::vector<Seg> & world, double stamp)
{
  FrontEnd fe;
  fe.clusters = node.build_clusters(scan_world(world));
  fe.primary = node.measurements_from_clusters(fe.clusters, at(stamp), &fe.origins);
  fe.compounded =
    node.apply_compound_association(fe.clusters, fe.origins, fe.primary, at(stamp));
  return fe;
}

bool same_measurements(const std::vector<Measurement> & a, const std::vector<Measurement> & b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].centroid.x != b[i].centroid.x || a[i].centroid.y != b[i].centroid.y) {
      return false;
    }
    if (a[i].features.valid != b[i].features.valid ||
      a[i].features.length != b[i].features.length ||
      a[i].features.thickness != b[i].features.thickness ||
      a[i].features.fill_ratio != b[i].features.fill_ratio ||
      a[i].features.max_gap != b[i].features.max_gap ||
      a[i].features.point_count != b[i].features.point_count)
    {
      return false;
    }
  }
  return true;
}

/// Everything the pre-Stage-5B.1 system produced for one track.
struct LegacyOutput
{
  Eigen::Vector4d kf_x;
  Eigen::Matrix4d kf_P;
  bool kalman_initialized{false};
  uint32_t age{0};
  uint32_t observations{0};
  uint32_t missed_count{0};
  std::vector<PredictionSample> predictions;
  std::vector<ReachabilityRegion> reachability;
};

/// A fixed sequence of ordinary single-object measurements through the
/// production association path. Nothing here is a fragment pair, so the
/// compound stage must make no difference at all.
LegacyOutput run_sequence(LidarObstacleTracker & node, bool compound)
{
  node.compound_params_.enabled = compound;
  node.compound_tracker_.clear();
  node.tracks_.clear();
  node.sensor_origin_ = {0.0, 0.0};
  for (int i = 0; i < 8; ++i) {
    Cluster c;
    c.points = scan_world(rect(3.0 - 0.06 * i, 0.0, 1.0, 0.5, 0.52));
    c.centroid = LidarObstacleTracker::centroid_of(c.points);
    c.diameter = LidarObstacleTracker::diameter_of(c.points);
    std::vector<Cluster> clusters{c};
    std::vector<size_t> origins;
    const auto primary =
      node.measurements_from_clusters(clusters, at(100.0 + 0.2 * i), &origins);
    const auto batch = node.apply_compound_association(
      clusters, origins, primary, at(100.0 + 0.2 * i));
    node.associate_measurements_and_update(batch, at(100.0 + 0.2 * i));
  }
  require(node.tracks_.size() == 1, "sequence should produce exactly one track");

  const Track & t = node.tracks_[0];
  LegacyOutput out;
  out.kf_x = t.kf_x;
  out.kf_P = t.kf_P;
  out.kalman_initialized = t.kalman_initialized;
  out.age = t.age;
  out.observations = t.observations;
  out.missed_count = t.missed_count;
  out.predictions = node.predict_future(t);
  out.reachability = node.predict_reachability(t, at(101.6));
  return out;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarObstacleTracker>();
  node->sensor_origin_ = {0.0, 0.0};
  node->classification_enabled_ = true;

  // ---- 1. THE FROZEN PARAMETERS ----------------------------------------
  {
    require(
      node->get_parameter("cluster_distance").as_double() == 0.35,
      "FROZEN: cluster_distance must be declared at exactly 0.35 m");
    require(
      node->cluster_distance_ == 0.35,
      "FROZEN: the live cluster_distance must be exactly 0.35 m");
    require(
      node->cluster_min_points_ == 3 && node->cluster_max_points_ == 200,
      "FROZEN: the cluster point-count envelope must be unchanged");
    require(
      node->cluster_max_diameter_ == 1.2,
      "FROZEN: cluster_max_diameter must be unchanged at 1.2 m");
    // Stage-4G3's own parameters.
    require(node->deblend_enabled_, "FROZEN: deblending must remain enabled by default");
    require(
      node->merge_min_cluster_diameter_ == 0.55 && node->merge_track_radius_ == 0.35,
      "FROZEN: the Stage-4G3 merge-detection parameters must be unchanged");
    require(
      node->deblend_max_child_diameter_ == 0.50 &&
      node->deblend_min_child_separation_ == 0.30 &&
      node->deblend_min_gap_ == 0.16,
      "FROZEN: the Stage-4G3 deblend geometry must be unchanged");
    // And the new stage must ship OFF.
    require(
      node->get_parameter("compound_enabled").as_bool() == false,
      "the compound stage must be declared DISABLED by default");
    require(
      !node->compound_params_.enabled,
      "the live compound stage must start disabled");
  }

  // ---- 2. DISABLED => PROVABLE NO-OP -----------------------------------
  {
    node->compound_params_.enabled = false;
    node->compound_tracker_.clear();
    const auto world = bike(3.0, 0.0, M_PI / 2);
    for (int k = 0; k < 6; ++k) {
      const FrontEnd fe = run_frontend(*node, world, 10.0 + 0.2 * k);
      require(
        fe.clusters.size() == 2,
        "no-op: the bicycle must still fragment into two primary clusters");
      require(
        same_measurements(fe.primary, fe.compounded),
        "no-op: with the stage disabled the measurement list must be IDENTICAL");
    }
    require(
      node->compound_tracker_.size() == 0,
      "no-op: a disabled stage must not accumulate any hypothesis state");
    require(
      node->compound_emitted_count_ == 0,
      "no-op: a disabled stage must emit nothing");
  }

  // ---- 3. ENABLED => THE BICYCLE IS RECOVERED --------------------------
  {
    node->compound_params_.enabled = true;
    node->compound_tracker_.clear();
    node->compound_emitted_count_ = 0;
    const auto world = bike(3.0, 0.0, M_PI / 2);

    // Scans 1 and 2: the geometry is good but support is still short of
    // min_support, so the fragments must still be reported separately.
    for (int k = 0; k < 2; ++k) {
      const FrontEnd fe = run_frontend(*node, world, 20.0 + 0.2 * k);
      require(
        fe.compounded.size() == 2,
        "recovery: below min_support the fragments must remain two measurements");
    }
    // Scan 3: support is reached and ONE compound observation replaces them.
    const FrontEnd fe = run_frontend(*node, world, 20.4);
    require(
      fe.compounded.size() == 1,
      "recovery: at min_support the two fragments must collapse to ONE observation");
    require(
      node->compound_emitted_count_ == 1,
      "recovery: exactly one compound observation should have been emitted");

    const Measurement & m = fe.compounded.front();
    require(m.features.valid, "recovery: the compound observation must carry valid geometry");
    require(
      m.features.length > 1.0 && m.features.length < 2.0,
      "recovery: the compound length must be bicycle-sized (~1.6 m)");
    require(
      m.features.max_gap >= 0.35,
      "recovery: the compound observation must retain the real internal gap");
    require(
      m.features.fill_ratio <= 0.90,
      "recovery: the compound observation must not read as solid");
    // The raw clusters are preserved -- the merge is not destructive.
    require(
      fe.clusters.size() == 2,
      "recovery: the PRIMARY clusters must survive unmodified");
    require(
      fe.primary.size() == 2,
      "recovery: the primary measurement list must survive unmodified");

    // And the UNCHANGED Stage-5B classifier must call it a BIKE.
    predictive_nav_tracking::TrackMotion motion;
    motion.speed = 0.25;
    motion.speed_valid = true;
    motion.observations = 10;
    const auto r = predictive_nav_tracking::classifyInstant(
      m.features, motion, node->classification_params_);
    require(
      r.label == predictive_nav_tracking::ObstacleClass::BIKE,
      "recovery: the compound observation must classify BIKE on the UNCHANGED classifier");
  }

  // ---- 4. A NON-BICYCLE SCENE IS UNCHANGED WITH THE STAGE ON -----------
  {
    node->compound_params_.enabled = true;
    node->compound_tracker_.clear();
    // Two separated 1.0 x 0.5 m bodies: real objects, far apart, nothing to
    // compound. Held for well over min_support scans.
    const auto world = rect(3.0, -1.2, 1.0, 0.5, 0.52) + rect(3.0, 1.2, 1.0, 0.5, 0.52);
    for (int k = 0; k < 8; ++k) {
      const FrontEnd fe = run_frontend(*node, world, 30.0 + 0.2 * k);
      require(
        same_measurements(fe.primary, fe.compounded),
        "unchanged: a scene with no fragment pair must pass through untouched");
    }
  }

  // ---- 5. THE KALMAN / PREDICTION / REACHABILITY CONTRACT --------------
  {
    const LegacyOutput on = run_sequence(*node, true);
    const LegacyOutput off = run_sequence(*node, false);

    require(on.kf_x == off.kf_x, "the compound stage changed the Kalman state");
    require(on.kf_P == off.kf_P, "the compound stage changed the Kalman covariance");
    require(
      on.kalman_initialized == off.kalman_initialized,
      "the compound stage changed kalman_initialized");
    require(on.age == off.age, "the compound stage changed track age");
    require(
      on.observations == off.observations,
      "the compound stage changed the observation count");
    require(
      on.missed_count == off.missed_count,
      "the compound stage changed missed_count");

    require(
      on.predictions.size() == off.predictions.size(),
      "the compound stage changed the number of CV prediction samples");
    for (size_t i = 0; i < on.predictions.size(); ++i) {
      const auto & a = on.predictions[i];
      const auto & b = off.predictions[i];
      require(a.time_from_now == b.time_from_now, "CV prediction time changed");
      require(a.x == b.x && a.y == b.y, "CV predicted position changed");
      require(a.vx == b.vx && a.vy == b.vy, "CV predicted velocity changed");
      require(
        a.position_covariance == b.position_covariance,
        "CV predicted position covariance changed");
    }

    require(
      on.reachability.size() == off.reachability.size(),
      "the compound stage changed the number of reachability samples");
    for (size_t i = 0; i < on.reachability.size(); ++i) {
      const auto & a = on.reachability[i];
      const auto & b = off.reachability[i];
      require(a.total_time == b.total_time, "reachability total_time changed");
      require(
        a.center_x == b.center_x && a.center_y == b.center_y,
        "reachability centre changed");
      require(a.vx == b.vx && a.vy == b.vy, "reachability velocity changed");
      require(a.reach_radius == b.reach_radius, "reachability radius changed");
      require(
        a.semi_major == b.semi_major && a.semi_minor == b.semi_minor,
        "reachability region axes changed");
      require(
        a.sigma_semi_major == b.sigma_semi_major &&
        a.sigma_semi_minor == b.sigma_semi_minor,
        "reachability sigma axes changed");
      require(a.safety_margin == b.safety_margin, "reachability safety margin changed");
      require(a.valid == b.valid, "reachability validity changed");
    }
  }

  // ---- 6. A DEBLENDED CHILD IS NEVER PAIRED ----------------------------
  {
    // Force a Stage-4G3 split: two confirmed tracks claiming one wide
    // cluster. The children carry kNoClusterOrigin, so even with the stage
    // enabled they must not be considered for compounding.
    node->compound_params_.enabled = true;
    node->compound_tracker_.clear();
    node->tracks_.clear();

    // Build two confirmed tracks either side of a merge.
    for (int i = 0; i < 5; ++i) {
      node->associate_and_update({{2.7, -0.30}, {2.7, 0.30}}, at(500.0 + 0.2 * i));
    }
    require(node->tracks_.size() == 2, "deblend setup: two tracks should exist");

    // One wide cluster spanning both, which deblending should split.
    Cluster wide;
    wide.points = scan_world(rect(2.7, 0.0, 0.10, 0.80, 0.0));
    wide.centroid = LidarObstacleTracker::centroid_of(wide.points);
    wide.diameter = LidarObstacleTracker::diameter_of(wide.points);
    std::vector<Cluster> clusters{wide};
    std::vector<size_t> origins;
    const auto primary =
      node->measurements_from_clusters(clusters, at(501.0), &origins);
    if (primary.size() == 2) {
      // It did split. Both children must carry no cluster origin...
      require(
        origins.size() == 2 &&
        origins[0] == LidarObstacleTracker::kNoClusterOrigin &&
        origins[1] == LidarObstacleTracker::kNoClusterOrigin,
        "deblend: a split child must carry no cluster origin");
      // ...so the compound stage must leave the pair alone.
      const auto out =
        node->apply_compound_association(clusters, origins, primary, at(501.0));
      require(
        same_measurements(primary, out),
        "deblend: deblended children must never be paired into a compound");
    }
  }

  std::fprintf(stderr, "stage5b1_compound_integration: all checks passed\n");
  rclcpp::shutdown();
  return 0;
}
