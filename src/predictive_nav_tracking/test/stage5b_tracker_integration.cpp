// Stage-5B integration regression, against the production translation unit.
//
// THE ONE THING THIS FILE EXISTS TO PROVE: enabling classification cannot
// change anything the pre-Stage-5B system produced.
//
// Stage-5B is specified as observation/metadata only -- class-aware prediction
// is explicitly a later stage -- so the acceptance criterion is not "the
// predictions still look reasonable" but "the predictions are IDENTICAL". That
// is checkable exactly, and this file checks it exactly: the same measurement
// sequence is driven through the production association path twice, once with
// classification on and once with it off, and the Kalman state, the state
// covariance, the Stage-4C constant-velocity predictions and the Stage-4G4
// reachable regions are compared BIT FOR BIT. A single differing bit fails the
// test. That is the same contract stage4g4_reachability.cpp pins for
// reachability-on vs reachability-off, applied to the new feature.
//
// The test is deliberately non-vacuous: it first asserts that the enabled run
// really does produce a label and real cluster features, so "identical" cannot
// be satisfied by classification silently doing nothing.
//
// It also pins the two properties that make the label safe to publish:
// classification state belongs to a track IDENTITY (a recycled ID inherits
// nothing), and a coasting scan contributes no class evidence.

#define main stage5b_original_main
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

/// Ray-cast a rectangular footprint, one return per beam, nearest hit only --
/// the same physical scan model the Stage-5B unit tests use. Produces the
/// member points of a cluster as the tracker would have received them.
std::vector<Point2D> scan_rect(
  double cx, double cy, double len, double wid, double yaw,
  double sx = 0.0, double sy = 0.0)
{
  struct Seg
  {
    double ax, ay, bx, by;
  };
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hx = 0.5 * len;
  const double hy = 0.5 * wid;
  const double px[4] = {-hx, hx, hx, -hx};
  const double py[4] = {-hy, -hy, hy, hy};
  std::vector<Seg> segs;
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    segs.push_back(
      {cx + px[i] * c - py[i] * s, cy + px[i] * s + py[i] * c,
        cx + px[j] * c - py[j] * s, cy + px[j] * s + py[j] * c});
  }
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

/// One scan of a 1.0 x 0.5 m body at a corner aspect, as a Measurement with
/// real geometry: a vehicle-class signature inside the frozen 1.2 m envelope.
///
/// The feature vector is produced by the node's OWN features_of(), not by
/// calling extractFeatures directly, so the measurement this test injects is
/// built by exactly the production code path -- including its respect for
/// classification_enabled_, which is the flag under test.
Measurement vehicle_measurement(
  const LidarObstacleTracker & node, double cx, double cy)
{
  Cluster cluster;
  cluster.points = scan_rect(cx, cy, 1.0, 0.5, 0.52);
  cluster.centroid = LidarObstacleTracker::centroid_of(cluster.points);
  cluster.diameter = LidarObstacleTracker::diameter_of(cluster.points);
  Measurement m;
  m.centroid = cluster.centroid;
  m.features = node.features_of(cluster);
  return m;
}

/// Everything the pre-Stage-5B system produced for one track.
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

rclcpp::Time at(double t)
{
  return rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
}

/// Drive a fixed sequence of vehicle measurements through the production
/// association path and capture the legacy outputs.
LegacyOutput run_sequence(LidarObstacleTracker & node, bool classification)
{
  node.classification_enabled_ = classification;
  node.tracks_.clear();
  node.sensor_origin_ = {0.0, 0.0};
  for (int i = 0; i < 8; ++i) {
    const double cx = 3.0 - 0.06 * i;
    std::vector<Measurement> batch{vehicle_measurement(node, cx, 0.0)};
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

  // ---- 1. The enabled run must really classify (anti-vacuity) ------------
  const LegacyOutput on = run_sequence(*node, true);
  {
    const Track & t = node->tracks_[0];
    require(
      t.class_features.valid,
      "classification enabled: cluster features should be valid");
    require(
      t.class_features.point_count >= 5,
      "classification enabled: the synthetic vehicle should return >= 5 beams");
    require(
      t.class_features.thickness > 0.3,
      "classification enabled: a corner aspect should show real depth");
    require(
      t.class_state.label() == predictive_nav_tracking::ObstacleClass::VEHICLE,
      "classification enabled: a vehicle-shaped cluster should stabilise to VEHICLE");
    require(
      t.class_state.confidence() > 0.0,
      "classification enabled: a held label should carry positive confidence");
  }

  // ---- 2. The disabled run must produce NO classification ----------------
  const LegacyOutput off = run_sequence(*node, false);
  {
    const Track & t = node->tracks_[0];
    require(
      !t.class_features.valid,
      "classification disabled: no cluster features should be computed");
    require(
      t.class_state.label() == predictive_nav_tracking::ObstacleClass::UNKNOWN,
      "classification disabled: the label must stay UNKNOWN");
    require(
      t.class_state.confidence() == 0.0,
      "classification disabled: confidence must be exactly zero");
  }

  // ---- 3. THE CONTRACT: every legacy output is bit-identical -------------
  require(on.kf_x == off.kf_x, "classification changed the Kalman state");
  require(on.kf_P == off.kf_P, "classification changed the Kalman covariance");
  require(
    on.kalman_initialized == off.kalman_initialized,
    "classification changed kalman_initialized");
  require(on.age == off.age, "classification changed track age");
  require(on.observations == off.observations, "classification changed observation count");
  require(on.missed_count == off.missed_count, "classification changed missed_count");

  require(
    on.predictions.size() == off.predictions.size(),
    "classification changed the number of CV prediction samples");
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
    "classification changed the number of reachability samples");
  for (size_t i = 0; i < on.reachability.size(); ++i) {
    const auto & a = on.reachability[i];
    const auto & b = off.reachability[i];
    require(a.total_time == b.total_time, "reachability total_time changed");
    require(a.center_x == b.center_x && a.center_y == b.center_y,
      "reachability centre changed");
    require(a.reach_radius == b.reach_radius, "reachability radius changed");
    require(a.semi_major == b.semi_major && a.semi_minor == b.semi_minor,
      "reachability region axes changed");
    require(a.sigma_semi_major == b.sigma_semi_major, "reachability sigma axis changed");
    require(a.valid == b.valid, "reachability validity changed");
  }

  // ---- 4. Position-only association is unchanged by Stage-5B ------------
  // The frozen entry point must behave exactly as before: it associates, it
  // filters, and it produces no class evidence because bare centroids carry
  // no geometry.
  {
    node->classification_enabled_ = true;
    node->tracks_.clear();
    for (int i = 0; i < 6; ++i) {
      node->associate_and_update({{0.3 * 0.2 * i, 0.0}}, at(200.0 + 0.2 * i));
    }
    require(node->tracks_.size() == 1, "position-only association should hold one track");
    require(
      node->tracks_[0].kalman_initialized,
      "position-only association should still initialise the filter");
    require(
      !node->tracks_[0].class_features.valid,
      "bare centroids must not fabricate cluster geometry");
    require(
      node->tracks_[0].class_state.label() == predictive_nav_tracking::ObstacleClass::UNKNOWN,
      "bare centroids must not produce a label");
  }

  // ---- 5. Class state belongs to a track IDENTITY ------------------------
  // A track that expires and is replaced must not pass its label on.
  {
    node->classification_enabled_ = true;
    node->tracks_.clear();
    node->sensor_origin_ = {0.0, 0.0};
    for (int i = 0; i < 8; ++i) {
      std::vector<Measurement> batch{vehicle_measurement(*node, 3.0 - 0.06 * i, 0.0)};
      node->associate_measurements_and_update(batch, at(300.0 + 0.2 * i));
    }
    const uint32_t first_id = node->tracks_[0].id;
    require(
      node->tracks_[0].class_state.label() ==
      predictive_nav_tracking::ObstacleClass::VEHICLE,
      "setup should hold a VEHICLE label");

    // Starve it until it expires, then feed the same geometry again.
    for (int i = 0; i < 10; ++i) {
      node->associate_measurements_and_update({}, at(302.0 + 0.2 * i));
      node->prune_stale_tracks(at(302.0 + 0.2 * i));
    }
    require(node->tracks_.empty(), "the starved track should have expired");

    std::vector<Measurement> batch{vehicle_measurement(*node, 3.0, 0.0)};
    node->associate_measurements_and_update(batch, at(320.0));
    require(node->tracks_.size() == 1, "a new track should have been created");
    require(node->tracks_[0].id != first_id, "the new track should have a new ID");
    require(
      node->tracks_[0].class_state.label() ==
      predictive_nav_tracking::ObstacleClass::UNKNOWN,
      "a new track ID must NOT inherit the previous track's label");
    require(
      node->tracks_[0].class_state.confidence() == 0.0,
      "a new track ID must start with zero class confidence");
  }

  // ---- 6. A coasting scan contributes no class evidence -----------------
  // It must DECAY the held label rather than reinforce it, and it must not
  // publish stale cluster geometry as though it had just been measured.
  {
    node->classification_enabled_ = true;
    node->tracks_.clear();
    node->sensor_origin_ = {0.0, 0.0};
    for (int i = 0; i < 10; ++i) {
      std::vector<Measurement> batch{vehicle_measurement(*node, 3.0 - 0.06 * i, 0.0)};
      node->associate_measurements_and_update(batch, at(400.0 + 0.2 * i));
    }
    require(
      node->tracks_[0].class_state.label() ==
      predictive_nav_tracking::ObstacleClass::VEHICLE,
      "setup should hold a VEHICLE label");
    const double before = node->tracks_[0].class_state.confidence();

    node->associate_measurements_and_update({}, at(402.0));
    require(
      node->tracks_[0].missed_count == 1,
      "the track should have recorded exactly one missed scan");
    require(
      !node->tracks_[0].class_features.valid,
      "a coasting scan must not report cluster geometry");
    require(
      node->tracks_[0].class_state.confidence() < before,
      "a coasting scan must DECAY class confidence, not hold or raise it");
  }

  std::fprintf(stderr, "stage5b_tracker_integration: all checks passed\n");
  rclcpp::shutdown();
  return 0;
}
