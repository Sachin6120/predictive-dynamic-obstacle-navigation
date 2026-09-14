// lidar_obstacle_tracker_node.cpp
//
// Stage-4B pipeline (unchanged):
//   /scan -> valid points -> transform to map frame -> reject points that
//   coincide with known-occupied static map cells -> cluster remaining
//   points -> gated nearest-neighbour association against existing tracks
//   -> publish TrackedObjectArray + RViz MarkerArray.
//
// Stage-4C additions:
//   - a per-track constant-velocity Kalman filter (see kalman_filter.hpp)
//     that becomes the published velocity/position source once a track has
//     enough observations for a real velocity estimate; the Stage-4B raw
//     finite-difference velocity remains available on every track for
//     RAW vs KALMAN evaluation.
//   - future trajectory prediction (0..prediction_horizon_) with covariance,
//     computed by peeking the filter forward without mutating live state.
//   - RViz markers for the raw measurement, filtered state, predicted path
//     and per-sample uncertainty ellipses.
//
// Deliberately no JPDA/MHT, no prediction-aware replanning, no costmap
// plugin: that is Stage-4D+.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

#include "predictive_nav_msgs/msg/tracked_object.hpp"
#include "predictive_nav_msgs/msg/tracked_object_array.hpp"
#include "predictive_nav_msgs/msg/tracked_object_prediction.hpp"
#include "predictive_nav_tracking/kalman_filter.hpp"

using std::placeholders::_1;
using predictive_nav_tracking::ConstantVelocityKalmanFilter2D;

namespace
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Observation
{
  rclcpp::Time stamp;
  double x{0.0};
  double y{0.0};
};

// One future state sample produced by peeking a track's Kalman filter
// forward; never derived from a mutated live filter.
/// One accepted connected component, with the member points retained so that
/// Stage-4G3 deblending can re-examine the raw geometry.
struct Cluster
{
  std::vector<Point2D> points;
  Point2D centroid;
  double diameter{0.0};
  /// True when the cluster was large enough to suspect two merged objects and
  /// at least two confirmed tracks claimed it, but the scan showed no evidence
  /// to split on. Diagnostic only: it never changes how the centroid is used.
  bool ambiguous_merge{false};
};

struct PredictionSample
{
  double time_from_now{0.0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
  std::array<double, 4> position_covariance{{0.0, 0.0, 0.0, 0.0}};  // row-major [xx, xy, yx, yy]
};

struct Track
{
  uint32_t id{0};

  // Kalman state x = [px, py, vx, vy] and covariance P, map frame.
  Eigen::Vector4d kf_x{Eigen::Vector4d::Zero()};
  Eigen::Matrix4d kf_P{Eigen::Matrix4d::Identity()};
  // True once >= min_observations_for_kalman_velocity_ measurements have
  // corrected the filter, i.e. the velocity estimate reflects real motion
  // rather than the zero-velocity initialization prior.
  bool kalman_initialized{false};

  // Stage-4B raw finite-difference baseline (unchanged semantics): kept on
  // every track so RAW vs KALMAN comparison remains possible.
  double raw_x{0.0};
  double raw_y{0.0};
  double raw_vx{0.0};
  double raw_vy{0.0};

  uint32_t age{0};
  uint32_t observations{0};
  uint32_t missed_count{0};
  rclcpp::Time last_update;
  std::deque<Observation> history;
};

}  // namespace

class LidarObstacleTracker : public rclcpp::Node
{
public:
  LidarObstacleTracker()
  : Node("lidar_obstacle_tracker")
  {
    declare_parameters();
    read_parameters();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    rclcpp::QoS map_qos(1);
    map_qos.transient_local();
    map_qos.reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&LidarObstacleTracker::map_callback, this, _1));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LidarObstacleTracker::scan_callback, this, _1));

    tracks_pub_ = this->create_publisher<predictive_nav_msgs::msg::TrackedObjectArray>(
      "tracked_objects", rclcpp::QoS(10));
    markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "tracked_objects/markers", rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "lidar_obstacle_tracker started: scan_topic=%s map_topic=%s static_reject_radius=%.2fm "
      "cluster_distance=%.2fm association_gate=%.2fm track_timeout=%.2fs | kalman: "
      "accel_noise=%.2fm/s^2 measurement_noise=%.3fm init_pos_var=%.3f init_vel_var=%.3f "
      "prediction_horizon=%.1fs step=%.1fs (%zu samples)",
      scan_topic_.c_str(), map_topic_.c_str(), static_reject_radius_, cluster_distance_,
      association_gate_, track_timeout_, kf_accel_noise_, kf_measurement_noise_,
      kf_initial_position_variance_, kf_initial_velocity_variance_, prediction_horizon_,
      prediction_time_step_, num_prediction_samples_);
  }

private:
  // ---------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------
  void declare_parameters()
  {
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("map_frame", "map");

    // --- Stage-4G1: range-aware static rejection -------------------------
    // `static_reject_radius` keeps its name and its Stage-4B default so no
    // existing YAML breaks. Its MEANING is now: the fixed radius when
    // range_aware_static_rejection is false (exact Stage-4B behaviour), and
    // the MINIMUM radius when it is true.
    this->declare_parameter<double>("static_reject_radius", 0.15);
    this->declare_parameter<bool>("range_aware_static_rejection", true);
    // Map discretisation + LiDAR range noise. One 0.05 m map cell plus ~3 sigma
    // of the simulated 0.01 m range noise. [m]
    this->declare_parameter<double>("base_static_margin", 0.06);
    // Translation component of the map->sensor pose error. [m]
    this->declare_parameter<double>("localization_margin", 0.06);
    // Angular uncertainty of the beam endpoint: half the beam angular spacing
    // plus the pose YAW uncertainty. Multiplied by beam range, this is the
    // range-dependent term the fixed radius was missing. [rad]
    this->declare_parameter<double>("angular_sampling_scale", 0.030);
    // Hard ceiling, so an anomalously long return can never blank a region. [m]
    this->declare_parameter<double>("max_static_reject_radius", 0.40);
    this->declare_parameter<int>("occupied_threshold", 65);
    this->declare_parameter<bool>("unknown_cells_are_static", false);

    this->declare_parameter<double>("cluster_distance", 0.35);
    this->declare_parameter<int>("cluster_min_points", 3);
    this->declare_parameter<int>("cluster_max_points", 200);
    this->declare_parameter<double>("cluster_max_diameter", 1.2);

    // --- Stage-4G3: track-aware cluster deblending -----------------------
    // A single connected cluster can contain the returns of two objects that
    // pass close to each other. The defaults below are derived from the
    // Stage-4G2 recordings (1,197 frames, 22 Gazebo runs), NOT from the
    // simulator's cylinder dimensions: accepted clusters carrying returns from
    // exactly one object never exceeded 0.468 m across, while every cluster
    // carrying returns from two objects was at least 0.785 m across. They are
    // parameters, not constants, so a deployment with differently sized
    // obstacles can rescale them.
    this->declare_parameter<bool>("deblend_enabled", true);
    this->declare_parameter<double>("merge_min_cluster_diameter", 0.55);
    this->declare_parameter<double>("merge_track_radius", 0.35);
    this->declare_parameter<int>("deblend_min_points", 3);
    this->declare_parameter<double>("deblend_max_child_diameter", 0.50);
    this->declare_parameter<double>("deblend_min_child_separation", 0.30);
    this->declare_parameter<double>("deblend_min_gap", 0.16);
    this->declare_parameter<bool>("deblend_use_mahalanobis", true);

    this->declare_parameter<double>("association_gate", 0.6);
    this->declare_parameter<double>("track_timeout", 1.0);
    this->declare_parameter<int>("max_missed_scans", 5);
    this->declare_parameter<int>("min_observations_to_publish", 3);
    this->declare_parameter<int>("velocity_history_size", 5);

    this->declare_parameter<bool>("publish_candidate_markers", true);
    this->declare_parameter<double>("velocity_marker_scale", 1.0);
    this->declare_parameter<double>("tf_lookup_timeout", 0.1);
    this->declare_parameter<bool>("profile_scans", false);  // Measurement only.

    // --- Stage-4C: constant-velocity Kalman filter -----------------------
    // 1-sigma unmodeled-acceleration noise (m/s^2) driving Q(dt); see
    // kalman_filter.hpp for the continuous white-noise-acceleration model.
    // 0.5 m/s^2 is a generic slow-indoor-obstacle default (not tuned to the
    // single recorded validation run): large enough that the filter
    // re-converges within roughly one second of an abrupt bounce reversal,
    // small enough to meaningfully smooth per-scan centroid noise on
    // straight segments.
    this->declare_parameter<double>("kf_process_accel_noise", 0.5);
    // 1-sigma position measurement noise (m) driving R. The cluster
    // centroid is an average over several LiDAR returns, so the per-scan
    // random component is well under typical planar-LiDAR range noise
    // (~1-3 cm/point); 0.05 m is a conservative round-number default.
    // NOTE: this is measurement *noise*, not the ~0.149 m surface-vs-center
    // bias documented for Stage-4B -- that bias is geometric and must not
    // be compensated here.
    this->declare_parameter<double>("kf_measurement_noise", 0.05);
    this->declare_parameter<double>("kf_initial_position_variance", 0.01);
    this->declare_parameter<double>("kf_initial_velocity_variance", 4.0);
    this->declare_parameter<double>("kf_min_dt", 1.0e-3);
    this->declare_parameter<double>("kf_max_dt", 1.0);
    // A track's velocity estimate is treated as real (rather than the
    // zero-velocity initialization prior) once this many measurements have
    // corrected the filter -- mirrors Stage-4B's own requirement of >= 2
    // history points before it trusts a finite-difference velocity.
    this->declare_parameter<int>("min_observations_for_kalman_velocity", 2);

    this->declare_parameter<double>("prediction_horizon", 3.0);
    this->declare_parameter<double>("prediction_time_step", 0.5);
    // Semi-axis scale factor for the uncertainty ellipse: semi-axis =
    // sigma * sqrt(eigenvalue of the 2x2 position covariance). sigma=2.0
    // encloses ~86% of probability mass for a 2D Gaussian
    // (1 - exp(-sigma^2/2)); this is a mathematically-defined confidence
    // region, not an arbitrary fixed-width corridor.
    this->declare_parameter<double>("uncertainty_ellipse_sigma", 2.0);
  }

  void read_parameters()
  {
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();

    static_reject_radius_ = this->get_parameter("static_reject_radius").as_double();
    range_aware_static_rejection_ =
      this->get_parameter("range_aware_static_rejection").as_bool();
    base_static_margin_ = this->get_parameter("base_static_margin").as_double();
    localization_margin_ = this->get_parameter("localization_margin").as_double();
    angular_sampling_scale_ = this->get_parameter("angular_sampling_scale").as_double();
    max_static_reject_radius_ =
      this->get_parameter("max_static_reject_radius").as_double();
    occupied_threshold_ = static_cast<int8_t>(this->get_parameter("occupied_threshold").as_int());
    unknown_cells_are_static_ = this->get_parameter("unknown_cells_are_static").as_bool();

    cluster_distance_ = this->get_parameter("cluster_distance").as_double();
    cluster_min_points_ = static_cast<size_t>(this->get_parameter("cluster_min_points").as_int());
    cluster_max_points_ = static_cast<size_t>(this->get_parameter("cluster_max_points").as_int());
    cluster_max_diameter_ = this->get_parameter("cluster_max_diameter").as_double();

    deblend_enabled_ = this->get_parameter("deblend_enabled").as_bool();
    merge_min_cluster_diameter_ =
      this->get_parameter("merge_min_cluster_diameter").as_double();
    merge_track_radius_ = this->get_parameter("merge_track_radius").as_double();
    deblend_min_points_ =
      static_cast<size_t>(this->get_parameter("deblend_min_points").as_int());
    deblend_max_child_diameter_ =
      this->get_parameter("deblend_max_child_diameter").as_double();
    deblend_min_child_separation_ =
      this->get_parameter("deblend_min_child_separation").as_double();
    deblend_min_gap_ = this->get_parameter("deblend_min_gap").as_double();
    deblend_use_mahalanobis_ = this->get_parameter("deblend_use_mahalanobis").as_bool();

    association_gate_ = this->get_parameter("association_gate").as_double();
    track_timeout_ = this->get_parameter("track_timeout").as_double();
    max_missed_scans_ = static_cast<uint32_t>(this->get_parameter("max_missed_scans").as_int());
    min_observations_to_publish_ =
      static_cast<uint32_t>(this->get_parameter("min_observations_to_publish").as_int());
    velocity_history_size_ =
      static_cast<size_t>(this->get_parameter("velocity_history_size").as_int());

    publish_candidate_markers_ = this->get_parameter("publish_candidate_markers").as_bool();
    velocity_marker_scale_ = this->get_parameter("velocity_marker_scale").as_double();
    tf_lookup_timeout_ = this->get_parameter("tf_lookup_timeout").as_double();

    kf_accel_noise_ = this->get_parameter("kf_process_accel_noise").as_double();
    kf_measurement_noise_ = this->get_parameter("kf_measurement_noise").as_double();
    kf_initial_position_variance_ = this->get_parameter("kf_initial_position_variance").as_double();
    kf_initial_velocity_variance_ = this->get_parameter("kf_initial_velocity_variance").as_double();
    kf_min_dt_ = this->get_parameter("kf_min_dt").as_double();
    kf_max_dt_ = this->get_parameter("kf_max_dt").as_double();
    min_observations_for_kalman_velocity_ = static_cast<uint32_t>(
      this->get_parameter("min_observations_for_kalman_velocity").as_int());

    prediction_horizon_ = this->get_parameter("prediction_horizon").as_double();
    prediction_time_step_ = this->get_parameter("prediction_time_step").as_double();
    uncertainty_ellipse_sigma_ = this->get_parameter("uncertainty_ellipse_sigma").as_double();
    num_prediction_samples_ = (prediction_time_step_ > 1e-6) ?
      static_cast<size_t>(std::lround(prediction_horizon_ / prediction_time_step_)) : 0;
  }

  // ---------------------------------------------------------------------
  // Map handling (unchanged)
  // ---------------------------------------------------------------------
  // -------------------------------------------------------------------
  // Stage-4G1: exact Euclidean distance transform of the static map.
  //
  // Felzenszwalb & Huttenlocher's O(n) squared-distance transform, one pass
  // down the columns and one across the rows. Exact, ~50 lines, no new
  // dependency. Computed once per map message; the per-scan-point query then
  // becomes a single array lookup instead of a neighbourhood search, which is
  // what makes a range-DEPENDENT radius affordable at all (a 0.40 m radius
  // would otherwise mean scanning 17x17 = 289 cells per point).
  // -------------------------------------------------------------------
  static void dt_1d(const std::vector<float> & f, std::vector<float> & d, int n)
  {
    static constexpr float kInf = std::numeric_limits<float>::max();
    std::vector<int> v(static_cast<size_t>(n), 0);
    std::vector<float> z(static_cast<size_t>(n) + 1, 0.0f);
    int k = 0;
    v[0] = 0;
    z[0] = -kInf;
    z[1] = kInf;
    for (int q = 1; q < n; ++q) {
      float s = ((f[q] + static_cast<float>(q) * q) -
        (f[v[k]] + static_cast<float>(v[k]) * v[k])) / static_cast<float>(2 * q - 2 * v[k]);
      while (s <= z[k]) {
        --k;
        s = ((f[q] + static_cast<float>(q) * q) -
          (f[v[k]] + static_cast<float>(v[k]) * v[k])) /
          static_cast<float>(2 * q - 2 * v[k]);
      }
      ++k;
      v[k] = q;
      z[k] = s;
      z[k + 1] = kInf;
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
      while (z[k + 1] < static_cast<float>(q)) {
        ++k;
      }
      const float dq = static_cast<float>(q - v[k]);
      d[q] = dq * dq + f[v[k]];
    }
  }

  void build_static_distance_field()
  {
    static constexpr float kInf = std::numeric_limits<float>::max();
    const auto & info = map_->info;
    const int w = static_cast<int>(info.width);
    const int h = static_cast<int>(info.height);
    if (w <= 0 || h <= 0) {
      map_distance_.clear();
      return;
    }

    // Seed set: cells the tracker considers STATIC. Unknown cells are included
    // only when unknown_cells_are_static is set, which reproduces the Stage-4B
    // semantics exactly rather than approximating them.
    std::vector<float> grid(static_cast<size_t>(w) * h, kInf);
    size_t n_static = 0;
    for (size_t i = 0; i < grid.size(); ++i) {
      const int8_t value = map_->data[i];
      const bool is_static = (value < 0) ? unknown_cells_are_static_ :
        (value >= occupied_threshold_);
      if (is_static) {
        grid[i] = 0.0f;
        ++n_static;
      }
    }
    if (n_static == 0) {
      map_distance_.assign(grid.size(), std::numeric_limits<float>::max());
      RCLCPP_WARN(get_logger(), "Static map has no occupied cells; nothing will be rejected");
      return;
    }

    std::vector<float> f(static_cast<size_t>(std::max(w, h)));
    std::vector<float> d(static_cast<size_t>(std::max(w, h)));

    for (int x = 0; x < w; ++x) {                       // columns
      for (int y = 0; y < h; ++y) {
        f[y] = grid[static_cast<size_t>(y) * w + x];
      }
      dt_1d(f, d, h);
      for (int y = 0; y < h; ++y) {
        grid[static_cast<size_t>(y) * w + x] = d[y];
      }
    }
    for (int y = 0; y < h; ++y) {                       // rows
      for (int x = 0; x < w; ++x) {
        f[x] = grid[static_cast<size_t>(y) * w + x];
      }
      dt_1d(f, d, w);
      for (int x = 0; x < w; ++x) {
        grid[static_cast<size_t>(y) * w + x] = d[x];
      }
    }

    map_distance_.resize(grid.size());
    const float res = static_cast<float>(info.resolution);
    for (size_t i = 0; i < grid.size(); ++i) {
      map_distance_[i] = std::sqrt(grid[i]) * res;      // cells -> metres
    }
    RCLCPP_INFO(
      get_logger(), "Static distance field built: %d x %d, %zu static cells",
      w, h, n_static);
  }

  /// Rejection tolerance for a return observed at `range` metres.
  ///
  /// r_reject(range) = clamp(base + localization + angular * range,
  ///                         static_reject_radius, max_static_reject_radius)
  ///
  /// The angular term is the one Stage-4B lacked. A pose yaw error of dtheta
  /// displaces a beam endpoint laterally by range * dtheta, so the mismatch
  /// between a wall return and the mapped wall grows with range while a fixed
  /// radius does not. Measured in the Stage-4F arena while driving: pose yaw
  /// error p95 = 0.0262 rad and beam half-spacing = 0.0087 rad, hence the
  /// 0.030 rad default.
  double static_reject_radius_for(double range) const
  {
    if (!range_aware_static_rejection_) {
      return static_reject_radius_;
    }
    const double r = base_static_margin_ + localization_margin_ +
      angular_sampling_scale_ * std::max(0.0, range);
    return std::clamp(r, static_reject_radius_, max_static_reject_radius_);
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
    build_static_distance_field();
    RCLCPP_INFO(
      get_logger(), "Received static map: %ux%u @ %.3fm/cell", msg->info.width,
      msg->info.height, msg->info.resolution);
  }

  /// True if the point is explained by the static map, given the range at
  /// which it was observed.
  bool is_static_point(double x, double y, double range) const
  {
    if (!map_ || map_distance_.empty()) {
      return true;
    }

    const auto & info = map_->info;
    const double res = info.resolution;
    const int mx = static_cast<int>(std::floor((x - info.origin.position.x) / res));
    const int my = static_cast<int>(std::floor((y - info.origin.position.y) / res));
    if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) ||
      my >= static_cast<int>(info.height))
    {
      // Outside the mapped area: unchanged from Stage-4B, nothing to explain
      // the return, so it is treated as a candidate dynamic point.
      return false;
    }

    const float dist = map_distance_[static_cast<size_t>(my) * info.width + mx];
    return static_cast<double>(dist) <= static_reject_radius_for(range);
  }

  // ---------------------------------------------------------------------
  // Scan processing (unchanged)
  // ---------------------------------------------------------------------
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    if (!map_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for static map on '%s' before tracking...",
        map_topic_.c_str());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        map_frame_, msg->header.frame_id, msg->header.stamp,
        rclcpp::Duration::from_seconds(tf_lookup_timeout_));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "TF %s->%s unavailable (%s); dropping scan",
        msg->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
      return;
    }

    const std::vector<Point2D> dynamic_points = extract_dynamic_points(*msg, transform);
    const auto cluster_start = std::chrono::steady_clock::now();
    std::vector<Cluster> clusters = build_clusters(dynamic_points);
    const auto cluster_end = std::chrono::steady_clock::now();

    log_static_rejection_stats(msg->header.stamp);

    const rclcpp::Time stamp(msg->header.stamp);
    // Stage-4G3: merge detection + track-aware deblending sit between
    // clustering and the unchanged association stage, so association always
    // sees one measurement per believed object.
    const auto deblend_start = std::chrono::steady_clock::now();
    const size_t splits_before = deblend_split_count_;
    const std::vector<Point2D> centroids = measurements_from_clusters(clusters, stamp);
    const auto deblend_end = std::chrono::steady_clock::now();
    const auto association_start = std::chrono::steady_clock::now();
    associate_and_update(centroids, stamp);
    const auto association_end = std::chrono::steady_clock::now();
    prune_stale_tracks(stamp);
    publish_tracks(stamp);
    publish_markers(stamp, centroids);
    const auto callback_end = std::chrono::steady_clock::now();
    if (get_parameter("profile_scans").as_bool()) {
      const auto us = [](auto a, auto b) {
          return std::chrono::duration<double, std::micro>(b - a).count();
        };
      RCLCPP_INFO(get_logger(),
        "G2_PERF %.9f scan_us=%.3f cluster_us=%.3f deblend_us=%.3f association_us=%.3f "
        "callback_us=%.3f clusters=%zu measurements=%zu tracks=%zu splits=%zu ambiguous=%zu",
        stamp.seconds(), us(callback_start, cluster_start), us(cluster_start, cluster_end),
        us(deblend_start, deblend_end),
        us(association_start, association_end), us(callback_start, callback_end),
        clusters.size(), centroids.size(), tracks_.size(),
        deblend_split_count_ - splits_before, ambiguous_merge_count_);
    }
  }

  /// Throttled runtime accounting for the static-rejection stage (Stage-4G1).
  void log_static_rejection_stats(const builtin_interfaces::msg::Time & stamp)
  {
    const rclcpp::Time now(stamp);
    if (last_reject_stats_log_.nanoseconds() == 0) {
      last_reject_stats_log_ = now;
      return;
    }
    if ((now - last_reject_stats_log_).seconds() < 5.0) {
      return;
    }
    last_reject_stats_log_ = now;
    if (reject_calls_ == 0) {
      return;
    }
    RCLCPP_INFO(
      get_logger(),
      "static rejection: scans=%lu mean=%.1fus max=%.1fus points_in=%lu "
      "rejected=%lu (%.2f%%) retained_last=%lu radius=[%.2f..%.2f]m",
      static_cast<unsigned long>(reject_calls_),
      reject_total_us_ / static_cast<double>(reject_calls_), reject_max_us_,
      static_cast<unsigned long>(points_in_total_),
      static_cast<unsigned long>(points_rejected_total_),
      points_in_total_ ? 100.0 * static_cast<double>(points_rejected_total_) /
      static_cast<double>(points_in_total_) : 0.0,
      static_cast<unsigned long>(points_retained_last_),
      static_reject_radius_for(0.0), static_reject_radius_for(20.0));
  }

  std::vector<Point2D> extract_dynamic_points(
    const sensor_msgs::msg::LaserScan & scan,
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    std::vector<Point2D> dynamic_points;
    dynamic_points.reserve(scan.ranges.size());

    const auto t_start = std::chrono::steady_clock::now();
    uint64_t considered = 0;

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const float range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;

      geometry_msgs::msg::PointStamped laser_pt;
      laser_pt.header = scan.header;
      laser_pt.point.x = range * std::cos(angle);
      laser_pt.point.y = range * std::sin(angle);
      laser_pt.point.z = 0.0;

      geometry_msgs::msg::PointStamped map_pt;
      tf2::doTransform(laser_pt, map_pt, transform);

      ++considered;
      if (!is_static_point(map_pt.point.x, map_pt.point.y, range)) {
        dynamic_points.push_back({map_pt.point.x, map_pt.point.y});
      }
    }

    const double elapsed_us =
      std::chrono::duration<double, std::micro>(
      std::chrono::steady_clock::now() - t_start).count();
    reject_total_us_ += elapsed_us;
    reject_max_us_ = std::max(reject_max_us_, elapsed_us);
    ++reject_calls_;
    points_in_total_ += considered;
    points_rejected_total_ += considered - dynamic_points.size();
    points_retained_last_ = dynamic_points.size();

    return dynamic_points;
  }

  /// Single-link connected components with the frozen Stage-4B filters. The
  /// only change from the checkpoint is that member points are retained.
  std::vector<Cluster> build_clusters(const std::vector<Point2D> & points) const
  {
    std::vector<Cluster> clusters;
    const size_t n = points.size();
    std::vector<bool> visited(n, false);

    for (size_t seed = 0; seed < n; ++seed) {
      if (visited[seed]) {
        continue;
      }
      std::vector<size_t> cluster_indices{seed};
      visited[seed] = true;

      for (size_t idx = 0; idx < cluster_indices.size(); ++idx) {
        const size_t current = cluster_indices[idx];
        for (size_t j = 0; j < n; ++j) {
          if (visited[j]) {
            continue;
          }
          const double d = std::hypot(
            points[current].x - points[j].x, points[current].y - points[j].y);
          if (d <= cluster_distance_) {
            visited[j] = true;
            cluster_indices.push_back(j);
          }
        }
      }

      if (cluster_indices.size() < cluster_min_points_ ||
        cluster_indices.size() > cluster_max_points_)
      {
        continue;
      }

      double sum_x = 0.0;
      double sum_y = 0.0;
      double min_x = points[cluster_indices.front()].x;
      double max_x = min_x;
      double min_y = points[cluster_indices.front()].y;
      double max_y = min_y;
      for (const auto idx : cluster_indices) {
        sum_x += points[idx].x;
        sum_y += points[idx].y;
        min_x = std::min(min_x, points[idx].x);
        max_x = std::max(max_x, points[idx].x);
        min_y = std::min(min_y, points[idx].y);
        max_y = std::max(max_y, points[idx].y);
      }
      const double diameter = std::hypot(max_x - min_x, max_y - min_y);
      if (diameter > cluster_max_diameter_) {
        continue;
      }

      Cluster cluster;
      cluster.points.reserve(cluster_indices.size());
      for (const auto idx : cluster_indices) {
        cluster.points.push_back(points[idx]);
      }
      cluster.centroid = {sum_x / static_cast<double>(cluster_indices.size()),
        sum_y / static_cast<double>(cluster_indices.size())};
      cluster.diameter = diameter;
      clusters.push_back(std::move(cluster));
    }
    return clusters;
  }

  static Point2D centroid_of(const std::vector<Point2D> & pts)
  {
    double sx = 0.0;
    double sy = 0.0;
    for (const auto & p : pts) {
      sx += p.x;
      sy += p.y;
    }
    const double inv = 1.0 / static_cast<double>(pts.size());
    return {sx * inv, sy * inv};
  }

  static double diameter_of(const std::vector<Point2D> & pts)
  {
    double min_x = pts.front().x;
    double max_x = min_x;
    double min_y = pts.front().y;
    double max_y = min_y;
    for (const auto & p : pts) {
      min_x = std::min(min_x, p.x);
      max_x = std::max(max_x, p.x);
      min_y = std::min(min_y, p.y);
      max_y = std::max(max_y, p.y);
    }
    return std::hypot(max_x - min_x, max_y - min_y);
  }

  // ---------------------------------------------------------------------
  // Stage-4G3: track-aware cluster deblending
  // ---------------------------------------------------------------------
  //
  // WHY this exists (measured, Stage-4G2 close perpendicular crossing): when
  // two objects pass within roughly 0.46-0.67 m of each other, their returns
  // fall into ONE 0.35 m-connected component for 4-5 consecutive scans. The
  // blended centroid was then handed to whichever track was nearer, the other
  // track received nothing, and after five misses it expired -- 2 ID switches
  // per trial. Greedy association was never the limiting factor: with only one
  // measurement there is nothing for any assignment rule, global or greedy, to
  // distribute.
  //
  // WHAT it does: for a cluster that is too large to be one object AND that at
  // least two already-confirmed tracks predict into, the member points are
  // partitioned by nearest predicted track and the partition is accepted only
  // if the SCAN ITSELF supports it. No ground truth is used; the seeds are the
  // tracker's own Kalman predictions, which are runtime state.
  //
  // WHAT it deliberately does not do: it never manufactures a measurement for
  // an object that is not visible. If one object physically occludes the other
  // the hidden object contributes no returns, no gap appears, the split is
  // refused, and the unobserved track coasts on prediction as before. It also
  // cannot create an identity that never existed: two seeds are required, so
  // objects that were merged from the moment they appeared stay one cluster.

  struct DeblendSeed
  {
    double x{0.0};
    double y{0.0};
    /// Inverse of the predicted position covariance + R, for Mahalanobis
    /// ownership. Falls back to identity if that matrix is not invertible.
    Eigen::Matrix2d information{Eigen::Matrix2d::Identity()};
  };

  /// Predicted positions/covariances of the CONFIRMED tracks, at scan time.
  /// Tentative tracks are excluded: an unconfirmed track is not yet evidence
  /// that an object exists, so it must not be able to carve up a cluster.
  std::vector<DeblendSeed> deblend_seeds(const rclcpp::Time & stamp) const
  {
    std::vector<DeblendSeed> seeds;
    for (const auto & track : tracks_) {
      if (track.observations < min_observations_to_publish_) {
        continue;
      }
      const double dt = clamp_dt((stamp - track.last_update).seconds());
      const Eigen::Matrix4d F = ConstantVelocityKalmanFilter2D::stateTransition(dt);
      const Eigen::Vector4d x = F * track.kf_x;
      const Eigen::Matrix4d P = F * track.kf_P * F.transpose() +
        ConstantVelocityKalmanFilter2D::processNoise(dt, kf_accel_noise_);

      DeblendSeed seed;
      seed.x = x(0);
      seed.y = x(1);
      Eigen::Matrix2d S = P.topLeftCorner<2, 2>();
      S(0, 0) += kf_measurement_noise_ * kf_measurement_noise_;
      S(1, 1) += kf_measurement_noise_ * kf_measurement_noise_;
      const double det = S.determinant();
      if (std::isfinite(det) && std::abs(det) > 1e-12) {
        seed.information = S.inverse();
      }
      seeds.push_back(seed);
    }
    return seeds;
  }

  /// Squared distance from a point to a seed, in whichever metric is enabled.
  static double seed_cost(
    const Point2D & p, const DeblendSeed & s, bool mahalanobis)
  {
    const Eigen::Vector2d d(p.x - s.x, p.y - s.y);
    return mahalanobis ? d.dot(s.information * d) : d.squaredNorm();
  }

  /// Attempts to split one cluster. Returns true and fills `children` when the
  /// scan supports two objects; returns false and leaves the cluster intact
  /// otherwise. `ambiguous` reports the "suspicious and claimed, but refused"
  /// case, which is a real merged observation of an unknown owner.
  bool try_deblend(
    const Cluster & cluster, const std::vector<DeblendSeed> & seeds,
    std::vector<Point2D> & children, bool & ambiguous) const
  {
    ambiguous = false;
    if (cluster.diameter <= merge_min_cluster_diameter_) {
      return false;   // Consistent with a single object; nothing to explain.
    }
    if (seeds.size() < 2) {
      return false;
    }

    // A seed only claims this cluster if some member point is close to it.
    // Distance to the NEAREST POINT, not to the centroid: during a merge the
    // blended centroid sits between the two objects and would otherwise look
    // equally near to both, which is exactly the ambiguity to be avoided.
    std::vector<size_t> claiming;
    for (size_t i = 0; i < seeds.size(); ++i) {
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto & p : cluster.points) {
        nearest = std::min(nearest, std::hypot(p.x - seeds[i].x, p.y - seeds[i].y));
      }
      if (nearest <= merge_track_radius_) {
        claiming.push_back(i);
      }
    }
    if (claiming.size() < 2) {
      return false;
    }

    // With more than two claimants, split only along the two furthest-apart
    // seeds -- the pair the cluster's elongation is actual evidence for. The
    // result is always a TWO-way split; any further claimant receives no
    // measurement this scan and coasts. Recursive or k-way splitting is
    // deliberately not attempted, because the single gap test below is
    // evidence for one dividing surface, not for several.
    size_t a = claiming[0];
    size_t b = claiming[1];
    double widest = -1.0;
    for (size_t i = 0; i < claiming.size(); ++i) {
      for (size_t j = i + 1; j < claiming.size(); ++j) {
        const double d = std::hypot(
          seeds[claiming[i]].x - seeds[claiming[j]].x,
          seeds[claiming[i]].y - seeds[claiming[j]].y);
        if (d > widest) {
          widest = d;
          a = claiming[i];
          b = claiming[j];
        }
      }
    }

    std::vector<Point2D> part_a;
    std::vector<Point2D> part_b;
    for (const auto & p : cluster.points) {
      if (seed_cost(p, seeds[a], deblend_use_mahalanobis_) <=
        seed_cost(p, seeds[b], deblend_use_mahalanobis_))
      {
        part_a.push_back(p);
      } else {
        part_b.push_back(p);
      }
    }

    ambiguous = true;   // Suspicious and claimed; downgraded below only on success.

    // Each side must be an acceptable measurement in its own right.
    if (part_a.size() < deblend_min_points_ || part_b.size() < deblend_min_points_) {
      return false;
    }
    if (diameter_of(part_a) > deblend_max_child_diameter_ ||
      diameter_of(part_b) > deblend_max_child_diameter_)
    {
      return false;
    }
    const Point2D ca = centroid_of(part_a);
    const Point2D cb = centroid_of(part_b);
    if (!std::isfinite(ca.x) || !std::isfinite(ca.y) ||
      !std::isfinite(cb.x) || !std::isfinite(cb.y))
    {
      return false;
    }
    if (std::hypot(ca.x - cb.x, ca.y - cb.y) < deblend_min_child_separation_) {
      return false;
    }

    // The decisive anti-fabrication guard: the returns themselves must show a
    // gap at the split surface, wider than the spacing seen between returns on
    // one object (measured p95 0.147 m, max 0.161 m). Without this, nearest-seed
    // partitioning would happily slice any solid blob in two.
    double gap = std::numeric_limits<double>::infinity();
    for (const auto & pa : part_a) {
      for (const auto & pb : part_b) {
        gap = std::min(gap, std::hypot(pa.x - pb.x, pa.y - pb.y));
      }
    }
    if (!(gap >= deblend_min_gap_)) {
      return false;
    }

    children.clear();
    children.push_back(ca);
    children.push_back(cb);
    ambiguous = false;
    return true;
  }

  /// Cluster centroids after deblending. With deblending disabled this is
  /// exactly the checkpoint's list of one centroid per accepted cluster.
  std::vector<Point2D> measurements_from_clusters(
    std::vector<Cluster> & clusters, const rclcpp::Time & stamp)
  {
    std::vector<Point2D> measurements;
    measurements.reserve(clusters.size() + 1);
    if (!deblend_enabled_) {
      for (const auto & c : clusters) {
        measurements.push_back(c.centroid);
      }
      return measurements;
    }

    const std::vector<DeblendSeed> seeds = deblend_seeds(stamp);
    std::vector<Point2D> children;
    for (auto & c : clusters) {
      bool ambiguous = false;
      if (try_deblend(c, seeds, children, ambiguous)) {
        measurements.insert(measurements.end(), children.begin(), children.end());
        ++deblend_split_count_;
      } else {
        // Unsplit merged clusters keep their single blended centroid, which
        // greedy association gives to exactly ONE track (its assignment is
        // one-to-one). The other track receives nothing and coasts. No centroid
        // is ever used to update two filters.
        measurements.push_back(c.centroid);
        c.ambiguous_merge = ambiguous;
        if (ambiguous) {
          ++ambiguous_merge_count_;
        }
      }
    }
    return measurements;
  }

  // ---------------------------------------------------------------------
  // Association / track maintenance
  // ---------------------------------------------------------------------
  double clamp_dt(double dt) const
  {
    return std::clamp(dt, kf_min_dt_, kf_max_dt_);
  }

  void associate_and_update(const std::vector<Point2D> & centroids, const rclcpp::Time & stamp)
  {
    const size_t n_tracks = tracks_.size();
    const size_t n_dets = centroids.size();

    // Gating uses the Kalman-predicted position (F(dt) * x), not a manual
    // vx*dt formula -- for an established track this is the filtered
    // velocity; for a brand-new track (v=0) it is identical to Stage-4B's
    // "no-motion" prediction.
    std::vector<Point2D> predicted(n_tracks);
    for (size_t t = 0; t < n_tracks; ++t) {
      const double dt = clamp_dt((stamp - tracks_[t].last_update).seconds());
      const Eigen::Vector4d predicted_state =
        ConstantVelocityKalmanFilter2D::stateTransition(dt) * tracks_[t].kf_x;
      predicted[t].x = predicted_state(0);
      predicted[t].y = predicted_state(1);
    }

    struct Candidate
    {
      double dist;
      size_t track_idx;
      size_t det_idx;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(n_tracks * n_dets);
    for (size_t t = 0; t < n_tracks; ++t) {
      for (size_t d = 0; d < n_dets; ++d) {
        const double dist = std::hypot(
          predicted[t].x - centroids[d].x, predicted[t].y - centroids[d].y);
        if (dist <= association_gate_) {
          candidates.push_back({dist, t, d});
        }
      }
    }
    std::sort(
      candidates.begin(), candidates.end(),
      [](const Candidate & a, const Candidate & b) {return a.dist < b.dist;});

    std::vector<bool> track_assigned(n_tracks, false);
    std::vector<bool> det_assigned(n_dets, false);
    for (const auto & c : candidates) {
      if (track_assigned[c.track_idx] || det_assigned[c.det_idx]) {
        continue;
      }
      track_assigned[c.track_idx] = true;
      det_assigned[c.det_idx] = true;
      update_track(tracks_[c.track_idx], centroids[c.det_idx], stamp);
    }

    for (size_t t = 0; t < n_tracks; ++t) {
      if (!track_assigned[t]) {
        tracks_[t].missed_count += 1;
        tracks_[t].age += 1;
      }
    }

    for (size_t d = 0; d < n_dets; ++d) {
      if (!det_assigned[d]) {
        tracks_.push_back(make_new_track(centroids[d], stamp, next_track_id_++));
      }
    }
  }

  Track make_new_track(const Point2D & det, const rclcpp::Time & stamp, uint32_t id) const
  {
    Track track;
    track.id = id;

    track.kf_x << det.x, det.y, 0.0, 0.0;
    track.kf_P.setZero();
    track.kf_P(0, 0) = kf_initial_position_variance_;
    track.kf_P(1, 1) = kf_initial_position_variance_;
    track.kf_P(2, 2) = kf_initial_velocity_variance_;
    track.kf_P(3, 3) = kf_initial_velocity_variance_;
    track.kalman_initialized = false;

    track.raw_x = det.x;
    track.raw_y = det.y;
    track.raw_vx = 0.0;
    track.raw_vy = 0.0;

    track.age = 1;
    track.observations = 1;
    track.missed_count = 0;
    track.last_update = stamp;
    track.history.push_back({stamp, det.x, det.y});
    return track;
  }

  void update_track(Track & track, const Point2D & det, const rclcpp::Time & stamp)
  {
    // --- Stage-4B raw finite-difference baseline (unchanged) --------------
    track.history.push_back({stamp, det.x, det.y});
    while (track.history.size() > velocity_history_size_) {
      track.history.pop_front();
    }
    if (track.history.size() >= 2) {
      const auto & oldest = track.history.front();
      const auto & newest = track.history.back();
      const double raw_dt = (newest.stamp - oldest.stamp).seconds();
      if (raw_dt > 1e-3) {
        track.raw_vx = (newest.x - oldest.x) / raw_dt;
        track.raw_vy = (newest.y - oldest.y) / raw_dt;
      }
    }
    track.raw_x = det.x;
    track.raw_y = det.y;

    // --- Stage-4C Kalman predict + update ---------------------------------
    const double dt = clamp_dt((stamp - track.last_update).seconds());
    const Eigen::Matrix4d F = ConstantVelocityKalmanFilter2D::stateTransition(dt);
    track.kf_x = F * track.kf_x;
    track.kf_P = F * track.kf_P * F.transpose() +
      ConstantVelocityKalmanFilter2D::processNoise(dt, kf_accel_noise_);

    Eigen::Matrix<double, 2, 4> H = Eigen::Matrix<double, 2, 4>::Zero();
    H(0, 0) = 1.0;
    H(1, 1) = 1.0;
    const Eigen::Matrix2d R =
      Eigen::Matrix2d::Identity() * (kf_measurement_noise_ * kf_measurement_noise_);

    const Eigen::Vector2d z(det.x, det.y);
    const Eigen::Vector2d y = z - H * track.kf_x;
    const Eigen::Matrix2d S = H * track.kf_P * H.transpose() + R;
    const Eigen::Matrix<double, 4, 2> K = track.kf_P * H.transpose() * S.inverse();

    track.kf_x = track.kf_x + K * y;
    track.kf_P = (Eigen::Matrix4d::Identity() - K * H) * track.kf_P;

    track.age += 1;
    track.observations += 1;
    track.missed_count = 0;
    track.last_update = stamp;

    if (track.observations >= min_observations_for_kalman_velocity_) {
      track.kalman_initialized = true;
    }
  }

  void prune_stale_tracks(const rclcpp::Time & stamp)
  {
    tracks_.erase(
      std::remove_if(
        tracks_.begin(), tracks_.end(),
        [&](const Track & t) {
          const double since_update = (stamp - t.last_update).seconds();
          return t.missed_count > max_missed_scans_ || since_update > track_timeout_;
        }),
      tracks_.end());
  }

  // ---------------------------------------------------------------------
  // Future trajectory prediction (Stage-4C)
  // ---------------------------------------------------------------------
  // Peeks the track's filter forward to t + i*prediction_time_step_ for
  // i = 1..num_prediction_samples_, independently from the live (kf_x,
  // kf_P) each time -- the live state is never modified.
  std::vector<PredictionSample> predict_future(const Track & track) const
  {
    std::vector<PredictionSample> samples;
    if (!track.kalman_initialized || num_prediction_samples_ == 0) {
      return samples;
    }
    samples.reserve(num_prediction_samples_);
    for (size_t i = 1; i <= num_prediction_samples_; ++i) {
      const double dt = static_cast<double>(i) * prediction_time_step_;
      const Eigen::Matrix4d F = ConstantVelocityKalmanFilter2D::stateTransition(dt);
      const Eigen::Vector4d xp = F * track.kf_x;
      const Eigen::Matrix4d Pp = F * track.kf_P * F.transpose() +
        ConstantVelocityKalmanFilter2D::processNoise(dt, kf_accel_noise_);

      PredictionSample s;
      s.time_from_now = dt;
      s.x = xp(0);
      s.y = xp(1);
      s.vx = xp(2);
      s.vy = xp(3);
      s.position_covariance = {Pp(0, 0), Pp(0, 1), Pp(1, 0), Pp(1, 1)};
      samples.push_back(s);
    }
    return samples;
  }

  // ---------------------------------------------------------------------
  // Publishing
  // ---------------------------------------------------------------------
  void publish_tracks(const rclcpp::Time & stamp)
  {
    predictive_nav_msgs::msg::TrackedObjectArray array_msg;
    array_msg.header.stamp = stamp;
    array_msg.header.frame_id = map_frame_;

    for (const auto & t : tracks_) {
      if (t.observations < min_observations_to_publish_) {
        continue;
      }
      predictive_nav_msgs::msg::TrackedObject obj;
      obj.id = t.id;

      const double pub_vx = t.kalman_initialized ? t.kf_x(2) : t.raw_vx;
      const double pub_vy = t.kalman_initialized ? t.kf_x(3) : t.raw_vy;

      obj.position.x = t.kf_x(0);
      obj.position.y = t.kf_x(1);
      obj.position.z = 0.0;
      obj.velocity.x = pub_vx;
      obj.velocity.y = pub_vy;
      obj.velocity.z = 0.0;
      obj.speed = std::hypot(pub_vx, pub_vy);

      obj.raw_position.x = t.raw_x;
      obj.raw_position.y = t.raw_y;
      obj.raw_position.z = 0.0;
      obj.raw_velocity.x = t.raw_vx;
      obj.raw_velocity.y = t.raw_vy;
      obj.raw_velocity.z = 0.0;

      obj.kalman_initialized = t.kalman_initialized;
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          obj.covariance[static_cast<size_t>(r * 4 + c)] = t.kf_P(r, c);
        }
      }

      obj.age = t.age;
      obj.observations = t.observations;
      obj.missed_count = t.missed_count;
      obj.stamp = t.last_update;

      for (const auto & s : predict_future(t)) {
        predictive_nav_msgs::msg::TrackedObjectPrediction pred;
        pred.time_from_now = s.time_from_now;
        pred.stamp = t.last_update + rclcpp::Duration::from_seconds(s.time_from_now);
        pred.position.x = s.x;
        pred.position.y = s.y;
        pred.position.z = 0.0;
        pred.velocity.x = s.vx;
        pred.velocity.y = s.vy;
        pred.velocity.z = 0.0;
        pred.position_covariance = s.position_covariance;
        obj.predictions.push_back(pred);
      }

      array_msg.tracks.push_back(obj);
    }
    tracks_pub_->publish(array_msg);
  }

  // Fills scale.x/y and orientation.z/w of `marker` with the sigma-scaled
  // 2D uncertainty ellipse of covariance block [[cxx, cxy], [cxy, cyy]].
  // Eigen-decomposition of the symmetric 2x2 block gives the ellipse's
  // principal axes directly -- this is a mathematically correct
  // representation of growing positional uncertainty, not a fixed-width
  // corridor.
  void fill_ellipse_from_covariance(
    visualization_msgs::msg::Marker & marker, double cxx, double cxy, double cyy) const
  {
    Eigen::Matrix2d cov;
    cov << cxx, cxy, cxy, cyy;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    Eigen::Vector2d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    const Eigen::Matrix2d eigenvectors = solver.eigenvectors();

    const double semi_major = uncertainty_ellipse_sigma_ * std::sqrt(eigenvalues(1));
    const double semi_minor = uncertainty_ellipse_sigma_ * std::sqrt(eigenvalues(0));
    const double angle = std::atan2(eigenvectors(1, 1), eigenvectors(0, 1));

    marker.scale.x = std::max(2.0 * semi_major, 0.02);
    marker.scale.y = std::max(2.0 * semi_minor, 0.02);
    marker.scale.z = 0.01;
    marker.pose.orientation.z = std::sin(angle / 2.0);
    marker.pose.orientation.w = std::cos(angle / 2.0);
  }

  void publish_markers(const rclcpp::Time & stamp, const std::vector<Point2D> & all_centroids)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    std::unordered_set<uint32_t> current_ids;

    for (const auto & t : tracks_) {
      if (t.observations < min_observations_to_publish_) {
        continue;
      }
      current_ids.insert(t.id);

      const double pub_vx = t.kalman_initialized ? t.kf_x(2) : t.raw_vx;
      const double pub_vy = t.kalman_initialized ? t.kf_x(3) : t.raw_vy;

      // Raw measurement (this scan's associated cluster centroid, unfiltered).
      visualization_msgs::msg::Marker measurement;
      measurement.header.frame_id = map_frame_;
      measurement.header.stamp = stamp;
      measurement.ns = "measurement";
      measurement.id = static_cast<int>(t.id);
      measurement.type = visualization_msgs::msg::Marker::SPHERE;
      measurement.action = visualization_msgs::msg::Marker::ADD;
      measurement.pose.position.x = t.raw_x;
      measurement.pose.position.y = t.raw_y;
      measurement.pose.position.z = 0.10;
      measurement.pose.orientation.w = 1.0;
      measurement.scale.x = 0.15;
      measurement.scale.y = 0.15;
      measurement.scale.z = 0.15;
      measurement.color.r = 1.0f;
      measurement.color.g = 1.0f;
      measurement.color.b = 1.0f;
      measurement.color.a = 0.9f;
      measurement.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(measurement);

      // Filtered current position.
      visualization_msgs::msg::Marker filtered;
      filtered.header.frame_id = map_frame_;
      filtered.header.stamp = stamp;
      filtered.ns = "filtered_position";
      filtered.id = static_cast<int>(t.id);
      filtered.type = visualization_msgs::msg::Marker::SPHERE;
      filtered.action = visualization_msgs::msg::Marker::ADD;
      filtered.pose.position.x = t.kf_x(0);
      filtered.pose.position.y = t.kf_x(1);
      filtered.pose.position.z = 0.15;
      filtered.pose.orientation.w = 1.0;
      filtered.scale.x = 0.3;
      filtered.scale.y = 0.3;
      filtered.scale.z = 0.3;
      filtered.color.r = 1.0f;
      filtered.color.g = 0.1f;
      filtered.color.b = 0.1f;
      filtered.color.a = 0.9f;
      filtered.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(filtered);

      visualization_msgs::msg::Marker label;
      label.header = filtered.header;
      label.ns = "label";
      label.id = static_cast<int>(t.id);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = t.kf_x(0);
      label.pose.position.y = t.kf_x(1);
      label.pose.position.z = 0.6;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.25;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 1.0f;
      char text[96];
      std::snprintf(
        text, sizeof(text), "ID %u  %.2f m/s%s", t.id, std::hypot(pub_vx, pub_vy),
        t.kalman_initialized ? " (KF)" : " (raw)");
      label.text = text;
      label.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(label);

      // Filtered velocity vector.
      visualization_msgs::msg::Marker velocity;
      velocity.header = filtered.header;
      velocity.ns = "velocity";
      velocity.id = static_cast<int>(t.id);
      velocity.type = visualization_msgs::msg::Marker::ARROW;
      velocity.action = visualization_msgs::msg::Marker::ADD;
      velocity.scale.x = 0.05;
      velocity.scale.y = 0.1;
      velocity.scale.z = 0.1;
      velocity.color.r = 0.1f;
      velocity.color.g = 0.8f;
      velocity.color.b = 1.0f;
      velocity.color.a = 0.9f;
      geometry_msgs::msg::Point start;
      start.x = t.kf_x(0);
      start.y = t.kf_x(1);
      start.z = 0.15;
      geometry_msgs::msg::Point end;
      end.x = t.kf_x(0) + pub_vx * velocity_marker_scale_;
      end.y = t.kf_x(1) + pub_vy * velocity_marker_scale_;
      end.z = 0.15;
      velocity.points.push_back(start);
      velocity.points.push_back(end);
      velocity.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(velocity);

      // Recent raw-measurement trail (shows measurement noise the filter smooths out).
      if (t.history.size() >= 2) {
        visualization_msgs::msg::Marker trail;
        trail.header = filtered.header;
        trail.ns = "trail";
        trail.id = static_cast<int>(t.id);
        trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
        trail.action = visualization_msgs::msg::Marker::ADD;
        trail.scale.x = 0.04;
        trail.color.r = 1.0f;
        trail.color.g = 0.6f;
        trail.color.b = 0.0f;
        trail.color.a = 0.8f;
        for (const auto & obs : t.history) {
          geometry_msgs::msg::Point p;
          p.x = obs.x;
          p.y = obs.y;
          p.z = 0.05;
          trail.points.push_back(p);
        }
        trail.lifetime = rclcpp::Duration::from_seconds(0.5);
        marker_array.markers.push_back(trail);
      }

      // Future trajectory prediction + growing uncertainty ellipses.
      const auto predictions = predict_future(t);
      if (!predictions.empty()) {
        visualization_msgs::msg::Marker path;
        path.header = filtered.header;
        path.ns = "prediction_path";
        path.id = static_cast<int>(t.id);
        path.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path.action = visualization_msgs::msg::Marker::ADD;
        path.scale.x = 0.03;
        path.color.r = 0.65f;
        path.color.g = 0.2f;
        path.color.b = 0.85f;
        path.color.a = 0.9f;
        geometry_msgs::msg::Point current_pt;
        current_pt.x = t.kf_x(0);
        current_pt.y = t.kf_x(1);
        current_pt.z = 0.15;
        path.points.push_back(current_pt);
        path.lifetime = rclcpp::Duration::from_seconds(0.5);

        for (size_t i = 0; i < predictions.size(); ++i) {
          const auto & s = predictions[i];
          geometry_msgs::msg::Point p;
          p.x = s.x;
          p.y = s.y;
          p.z = 0.15;
          path.points.push_back(p);

          visualization_msgs::msg::Marker ellipse;
          ellipse.header = filtered.header;
          ellipse.ns = "prediction_uncertainty";
          ellipse.id = static_cast<int>(t.id) * kPredictionMarkerIdStride + static_cast<int>(i);
          ellipse.type = visualization_msgs::msg::Marker::CYLINDER;
          ellipse.action = visualization_msgs::msg::Marker::ADD;
          ellipse.pose.position.x = s.x;
          ellipse.pose.position.y = s.y;
          ellipse.pose.position.z = 0.05;
          fill_ellipse_from_covariance(
            ellipse, s.position_covariance[0], s.position_covariance[1],
            s.position_covariance[3]);
          ellipse.color.r = 0.65f;
          ellipse.color.g = 0.2f;
          ellipse.color.b = 0.85f;
          ellipse.color.a = 0.25f;
          ellipse.lifetime = rclcpp::Duration::from_seconds(0.5);
          marker_array.markers.push_back(ellipse);
        }
        marker_array.markers.push_back(path);
      }
    }

    // Delete markers for tracks that no longer exist / are no longer confirmed.
    for (const auto & prev_id : previous_marker_ids_) {
      if (current_ids.count(prev_id)) {
        continue;
      }
      for (const char * ns :
        {"measurement", "filtered_position", "label", "velocity", "trail", "prediction_path"})
      {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = map_frame_;
        del.header.stamp = stamp;
        del.ns = ns;
        del.id = static_cast<int>(prev_id);
        del.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(del);
      }
      for (size_t i = 0; i < num_prediction_samples_; ++i) {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = map_frame_;
        del.header.stamp = stamp;
        del.ns = "prediction_uncertainty";
        del.id = static_cast<int>(prev_id) * kPredictionMarkerIdStride + static_cast<int>(i);
        del.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(del);
      }
    }
    previous_marker_ids_ = current_ids;

    if (publish_candidate_markers_) {
      visualization_msgs::msg::Marker candidates;
      candidates.header.frame_id = map_frame_;
      candidates.header.stamp = stamp;
      candidates.ns = "candidate_clusters";
      candidates.id = 0;
      candidates.type = visualization_msgs::msg::Marker::POINTS;
      candidates.action = visualization_msgs::msg::Marker::ADD;
      candidates.scale.x = 0.08;
      candidates.scale.y = 0.08;
      candidates.color.r = 1.0f;
      candidates.color.g = 1.0f;
      candidates.color.b = 0.0f;
      candidates.color.a = 0.6f;
      candidates.lifetime = rclcpp::Duration::from_seconds(0.5);
      for (const auto & c : all_centroids) {
        geometry_msgs::msg::Point p;
        p.x = c.x;
        p.y = c.y;
        p.z = 0.05;
        candidates.points.push_back(p);
      }
      marker_array.markers.push_back(candidates);
    }

    markers_pub_->publish(marker_array);
  }

  // ---------------------------------------------------------------------
  // Members
  // ---------------------------------------------------------------------
  // Spacing between per-track marker IDs in the "prediction_uncertainty"
  // namespace; must exceed num_prediction_samples_ so two tracks' ellipse
  // marker IDs never collide.
  static constexpr int kPredictionMarkerIdStride = 1000;

  std::string scan_topic_;
  std::string map_topic_;
  std::string map_frame_;

  double static_reject_radius_{0.15};
  // Stage-4G1 range-aware static rejection.
  bool range_aware_static_rejection_{true};
  double base_static_margin_{0.06};
  double localization_margin_{0.06};
  double angular_sampling_scale_{0.030};
  double max_static_reject_radius_{0.40};
  int8_t occupied_threshold_{65};
  bool unknown_cells_are_static_{false};

  double cluster_distance_{0.35};
  size_t cluster_min_points_{3};
  size_t cluster_max_points_{200};
  double cluster_max_diameter_{1.2};

  // --- Stage-4G3: track-aware cluster deblending ---
  bool deblend_enabled_{true};
  double merge_min_cluster_diameter_{0.55};
  double merge_track_radius_{0.35};
  size_t deblend_min_points_{3};
  double deblend_max_child_diameter_{0.50};
  double deblend_min_child_separation_{0.30};
  double deblend_min_gap_{0.16};
  bool deblend_use_mahalanobis_{true};
  size_t deblend_split_count_{0};
  size_t ambiguous_merge_count_{0};

  double association_gate_{0.6};
  double track_timeout_{1.0};
  uint32_t max_missed_scans_{5};
  uint32_t min_observations_to_publish_{3};
  size_t velocity_history_size_{5};

  bool publish_candidate_markers_{true};
  double velocity_marker_scale_{1.0};
  double tf_lookup_timeout_{0.1};

  double kf_accel_noise_{0.5};
  double kf_measurement_noise_{0.05};
  double kf_initial_position_variance_{0.01};
  double kf_initial_velocity_variance_{4.0};
  double kf_min_dt_{1.0e-3};
  double kf_max_dt_{1.0};
  uint32_t min_observations_for_kalman_velocity_{2};

  double prediction_horizon_{3.0};
  double prediction_time_step_{0.5};
  double uncertainty_ellipse_sigma_{2.0};
  size_t num_prediction_samples_{6};

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  /// Metres from each map cell to the nearest STATIC cell (Stage-4G1).
  std::vector<float> map_distance_;

  // Static-rejection runtime accounting (reported via a throttled log line).
  mutable double reject_total_us_{0.0};
  mutable double reject_max_us_{0.0};
  mutable uint64_t reject_calls_{0};
  mutable uint64_t points_in_total_{0};
  mutable uint64_t points_rejected_total_{0};
  mutable uint64_t points_retained_last_{0};
  rclcpp::Time last_reject_stats_log_;

  std::vector<Track> tracks_;
  uint32_t next_track_id_{1};
  std::unordered_set<uint32_t> previous_marker_ids_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<predictive_nav_msgs::msg::TrackedObjectArray>::SharedPtr tracks_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarObstacleTracker>());
  rclcpp::shutdown();
  return 0;
}
