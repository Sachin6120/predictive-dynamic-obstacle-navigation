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
// Stage-4G4 addition (ADDITIVE ONLY):
//   - an optional second future-motion representation: conservative
//     bounded-motion REACHABLE SETS (see reachability.hpp), published in the
//     new TrackedObject.reachability_predictions field. The Stage-4C
//     constant-velocity Gaussian prediction above is untouched -- same state,
//     same covariance, same anchoring, same field -- so every Stage-4D/4E/4F
//     result stays reproducible. Reachability differs in one further respect
//     that the legacy field deliberately does NOT adopt: it propagates through
//     a coasting track's observation age, so a stale track is never presented
//     as a fresh one.
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
#include "predictive_nav_msgs/msg/reachability_prediction.hpp"
#include "predictive_nav_tracking/kalman_filter.hpp"
#include "predictive_nav_tracking/compound_observation.hpp"
#include "predictive_nav_tracking/capability_profiles.hpp"
#include "predictive_nav_tracking/obstacle_classification.hpp"
#include "predictive_nav_tracking/reachability.hpp"

using std::placeholders::_1;
using predictive_nav_tracking::ConstantVelocityKalmanFilter2D;
using predictive_nav_tracking::ReachabilityBounds;
using predictive_nav_tracking::ReachabilityRegion;
using predictive_nav_tracking::ClassificationParams;
using predictive_nav_tracking::ClassificationResult;
using predictive_nav_tracking::ClassLabelStabilizer;
using predictive_nav_tracking::ClusterFeatures;
using predictive_nav_tracking::CompoundEvidence;
using predictive_nav_tracking::CompoundHypothesisTracker;
using predictive_nav_tracking::CompoundParams;
using predictive_nav_tracking::ObstacleClass;
using predictive_nav_tracking::AmbiguitySets;
using predictive_nav_tracking::CapabilityProfileSet;
using predictive_nav_tracking::ClassProfileState;
using predictive_nav_tracking::MotionCapabilityProfile;
using predictive_nav_tracking::ProfileReason;
using predictive_nav_tracking::ProfileSelection;
using predictive_nav_tracking::ProfileSelectionPolicy;

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

/// Stage-5B: one association-ready measurement. `centroid` is exactly the
/// quantity the frozen pipeline has always associated on -- association,
/// gating and the Kalman update read nothing else, so their behaviour is
/// unchanged. `features` is the scan-plane geometry of the cluster the
/// centroid came from, carried along purely so classification can attach to
/// the track the centroid updates. `features.valid` is false when no
/// trustworthy geometry exists for this measurement.
struct Measurement
{
  Point2D centroid;
  ClusterFeatures features;
  /// Stage-5B.1: true when this measurement is a COMPOUND OBSERVATION -- the
  /// pooled geometry of two primary clusters that the compound stage judged
  /// to be two slices of one rigid object. PROVENANCE ONLY: association,
  /// gating and the Kalman filter read `centroid` and never this flag, so it
  /// cannot change any track state. It exists so that a recovered object is
  /// auditable after the fact -- the validation set reports true and FALSE
  /// compound associations from this, rather than inferring them from the
  /// resulting geometry.
  bool compound{false};
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

  // --- Stage-5B: classification state, METADATA ONLY --------------------
  // Attached to this track's identity and therefore destroyed with it: a new
  // ID starts from UNKNOWN with no inherited evidence. Nothing in this block
  // is read by the Kalman filter, the CV prediction or the reachable sets.
  ClassLabelStabilizer class_state;
  /// This scan's unsmoothed decision (UNKNOWN on a coasting scan).
  ClassificationResult class_instant;
  /// The geometry `class_instant` was derived from (invalid when coasting).
  ClusterFeatures class_features;
  /// Stage-5B.1: true when THIS scan's associated measurement was a compound
  /// observation. Provenance only, reset on every coasting scan.
  bool class_from_compound{false};

  // --- Stage-5D: class-aware capability-profile state -------------------
  // The transition rule's memory (capability_profiles.hpp section 6). It is
  // attached to the track id, so a new id starts with no held envelope and
  // takes its first scan's selection -- which, because a new track is both
  // UNKNOWN and immature, is the conservative fallback. Nothing here is read
  // by the Kalman filter, the CV prediction or the classifier; it is read
  // only by predict_reachability().
  ClassProfileState class_profile_state;
  /// The profile decision made on the most recent published scan, kept so
  /// publish_tracks() can report it without recomputing and so a test can
  /// inspect it. Advisory copy of the result, not state.
  ProfileSelection class_profile_selection;
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

    // --- Stage-5B.1: gap-aware compound-observation recovery -------------
    // A SECONDARY association stage that re-links primary clusters which are
    // two slices of one rigid object (the measured case: a bicycle's two
    // wheel chords, 1.04 m apart with a 0.48-0.54 m hole between them, which
    // single-link clustering at 0.35 m necessarily splits).
    //
    // cluster_distance IS NOT TOUCHED. This stage only ever inspects pairs
    // of clusters that primary clustering already chose to keep apart, so
    // the Stage-4G3 close-object geometry -- whose inter-object gaps are
    // 0.12-0.35 m, i.e. BELOW cluster_distance -- never reaches it as a pair
    // at all. Every bound's derivation and measured margin is documented at
    // its point of definition in compound_observation.hpp.
    //
    // DEFAULT false. With this false the stage is a provable no-op and the
    // whole pipeline is bit-identical to the Stage-5B checkpoint; the
    // Stage-5B classification config turns it on.
    const CompoundParams compound_def;
    this->declare_parameter<bool>("compound_enabled", compound_def.enabled);
    this->declare_parameter<double>(
      "compound_fragment_length_min", compound_def.fragment_length_min);
    this->declare_parameter<double>(
      "compound_fragment_length_max", compound_def.fragment_length_max);
    this->declare_parameter<double>(
      "compound_fragment_thickness_max", compound_def.fragment_thickness_max);
    this->declare_parameter<double>("compound_alignment_min", compound_def.alignment_min);
    this->declare_parameter<double>("compound_separation_min", compound_def.separation_min);
    this->declare_parameter<double>("compound_separation_max", compound_def.separation_max);
    this->declare_parameter<double>("compound_gap_min", compound_def.gap_min);
    this->declare_parameter<double>("compound_gap_max", compound_def.gap_max);
    this->declare_parameter<double>("compound_union_length_min", compound_def.union_length_min);
    this->declare_parameter<double>("compound_union_length_max", compound_def.union_length_max);
    this->declare_parameter<double>(
      "compound_union_thickness_max", compound_def.union_thickness_max);
    this->declare_parameter<double>("compound_union_fill_max", compound_def.union_fill_max);
    this->declare_parameter<int>(
      "compound_min_support", static_cast<int>(compound_def.min_support));
    this->declare_parameter<double>(
      "compound_separation_tolerance", compound_def.separation_tolerance);
    this->declare_parameter<double>("compound_match_radius", compound_def.match_radius);
    this->declare_parameter<double>("compound_max_gap_seconds", compound_def.max_gap_seconds);

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

    // --- Stage-4G4: conservative bounded-motion reachable sets -----------
    // An ADDITIONAL future-motion representation published alongside the
    // untouched Stage-4C constant-velocity Gaussian prediction. Every value
    // below is a physical quantity chosen from the obstacle's motion
    // capability, not from a desired navigation outcome.
    //
    // max_acceleration: the acceleration bound the obstacle class is permitted
    //   to use. The Stage-4G4 scenario driver commands ramps at a known bound,
    //   and this is set to that bound. NOTE the measured limitation, recorded
    //   rather than hidden: Gazebo's VelocityControl plugin can also apply a
    //   STEP velocity change (measured |a| 5-9.5 m/s^2 in the Stage-4G3
    //   recordings), which no finite bound can cover at short horizon.
    // max_speed: |v| ceiling. Scenario obstacles run at 0.15-0.5 m/s; 0.8 m/s
    //   leaves headroom without licensing motion the models cannot perform.
    // reachability_horizon / _time_step: mirror the Stage-4C prediction grid so
    //   the two representations are compared at identical horizons.
    // max_observation_age: beyond this much time without a real measurement the
    //   region is marked invalid rather than grown without limit. Defaults to
    //   the existing track_timeout, so it never outlives the track itself.
    // base_safety_margin: deterministic body allowance. ZERO by default, so the
    //   published region measures the motion model alone and reachability gets
    //   no free inflation in the comparison against CV.
    // covariance_sigma_level: the k applied to the Stage-4C propagated position
    //   covariance for the separately reported STATISTICAL term. Matches the
    //   Stage-4D layer's sigma_level so both representations use the same k.
    this->declare_parameter<bool>("reachability_enabled", true);
    this->declare_parameter<double>("max_acceleration", 0.5);
    this->declare_parameter<double>("max_speed", 0.8);
    this->declare_parameter<double>("reachability_horizon", 3.0);
    this->declare_parameter<double>("reachability_time_step", 0.5);
    this->declare_parameter<double>("base_safety_margin", 0.0);
    this->declare_parameter<double>("max_observation_age", 1.0);
    this->declare_parameter<double>("covariance_sigma_level", 2.0);

    // --- Stage-5C.1: the max_speed VALIDITY CONTRACT ---------------------
    // max_speed above is a FUTURE CAPABILITY premise, not a tuning weight:
    // the reachable set is a bound only while |v(s)| <= max_speed holds for
    // the whole window. Stage-5C measured the cost of that being false --
    // observed s0 = 0, configured cap 0.8 m/s, true capability 1.4 m/s gives
    // a model reach of 2.56 m at t = 4.0 s against a true reach of 3.64 m, a
    // 1.08 m escape (worst tested 1.44 m) INSIDE the production total-time
    // window of max_observation_age + reachability_horizon = 4.0 s.
    //
    // WHAT IS ENFORCED HERE, AND WHAT CANNOT BE. The premise quantifies over
    // the FUTURE, so no runtime check can verify it and none is claimed. The
    // observable half is the contradiction: a filtered speed above the cap
    // proves the obstacle is capable of that speed and therefore disproves
    // the premise. That is detected, reported in
    // ReachabilityPrediction.contract_status, and answered by raising the cap
    // to the weakest observation-consistent value. The UNOBSERVABLE residual
    // -- true capability above the cap while the filtered speed stays below it
    // -- is NOT enforceable at runtime and remains a declared assumption of
    // this configuration: setting max_speed is asserting that no tracked
    // obstacle in the deployment can exceed it. Current simulation obstacles
    // run at <= 0.75 m/s, so 0.8 m/s holds for the qualified experiments.
    //
    // reachability_enforce_observed_speed: when true (default) a disproved
    //   premise is replaced by v_eff = max(max_speed, s0), which can only
    //   enlarge the region -- never drop it, never shrink it. In domain
    //   v_eff == max_speed and the output is bit-identical to Stage-5C.
    //   False restores the pre-Stage-5C.1 arithmetic and exists so the
    //   regression test can reproduce the old behaviour; it is not a
    //   production setting.
    // reachability_max_speed_tolerance: REPORTING dead-band only, so
    //   float-level jitter at exactly the cap is not announced as a breach.
    //   Deliberately numeric-scale (1e-6 m/s) and NOT wide enough to absorb
    //   estimator transients: the Stage-5C recordings show the filtered speed
    //   overshooting ground truth by up to 0.25 m/s and exceeding the cap by
    //   up to 0.131 m/s on tracks that are genuinely in domain, while
    //   Stage-5C's adversary measured real escapes for s0 in [0.80, 0.95]. A
    //   dead-band wide enough for the former would mask the latter. It is not
    //   needed for safety either, because the response above only ever
    //   enlarges the region, so reacting to a transient costs nothing.
    this->declare_parameter<bool>("reachability_enforce_observed_speed", true);
    this->declare_parameter<double>("reachability_max_speed_tolerance", 1e-6);

    // --- Stage-5D: class-aware motion-capability profiles ----------------
    // Replaces the ONE global (max_acceleration, max_speed, base_safety_
    // margin) triple above with a per-class table plus the policy that
    // decides which entry a track is entitled to. The reachability equation,
    // the Stage-5C.1 validity contract and the published region are
    // unchanged functions of whichever profile is selected -- see
    // capability_profiles.hpp, whose header comment carries the full
    // derivation, the measured classifier evidence behind the defaults and
    // the four invariants (I1)-(I3), (T1)-(T3).
    //
    // reachability_class_aware DEFAULTS TO FALSE, and that is the production
    // setting for this milestone. False returns the single global
    // configuration verbatim, so every published number is bit-identical to
    // Stage-5C.1. Production also keeps reachability_enabled false and the
    // costmap's prediction_mode cv_covariance, so Stage-5D changes no
    // production behaviour at all.
    //
    // THE PROFILE VALUES ARE REAL-WORLD PHYSICAL PRIORS, NOT THE SIMULATION
    // ENVELOPE. max_speed and max_acceleration are premises about an
    // object's FUTURE capability, so they must cover what the class can do,
    // not what the Gazebo actors were commanded to do. Every Stage-5B actor
    // ran at 0.25/0.50/0.75 m/s whatever its class and the filtered speed
    // never exceeded 0.967 m/s, so a simulation-fitted table would be
    // class-independent -- it would carry no semantic information -- and
    // would under-bound a real bicycle by a factor of 15.
    //
    //   max_speed 2.5 / 12.0 / 40.0 m/s are the SHIPPED Stage-5B per-class
    //     speed vetoes (ClassificationParams::*.speed_max), so the
    //     classifier's refusal threshold and the reachability premise are one
    //     number rather than two that can disagree.
    //   max_acceleration 2.5 / 7.0 / 10.0 m/s^2 are measured external values
    //     (pedestrian crossing accelerations, bicycle pitch-over-limited
    //     braking, ABS decelerations on dry asphalt), cited per field in
    //     capability_profiles.hpp and in validation/stage5d/STAGE5D_README.md.
    //   margin 0.0 m for every class, inherited deliberately from Stage-4G4's
    //     base_safety_margin: 0.0 so the region measures the motion model
    //     alone and the global-vs-class-aware comparison is not confounded.
    //
    // UNKNOWN HAS NO PARAMETERS BY DESIGN. Its profile is DERIVED as the
    // envelope over its ambiguity set, so it dominates every class it has to
    // cover by identity instead of by a chosen pair of numbers that could
    // drift. 382 of the 1,137 UNKNOWN detections in the Stage-5B recordings
    // were ground-truth VEHICLE, so an UNKNOWN object in this very dataset
    // demonstrably is a vehicle about a third of the time; the old global
    // 0.8 m/s would have under-bounded it by a factor of 50.
    const CapabilityProfileSet prof_def;
    const AmbiguitySets amb_def;
    const ProfileSelectionPolicy pol_def;
    this->declare_parameter<bool>("reachability_class_aware", pol_def.class_aware);
    declare_profile_params("pedestrian", prof_def[ObstacleClass::PEDESTRIAN]);
    declare_profile_params("bike", prof_def[ObstacleClass::BIKE]);
    declare_profile_params("vehicle", prof_def[ObstacleClass::VEHICLE]);

    // Per-label AMBIGUITY SET: the classes an object carrying that label
    // might really be. This is a property of the CLASSIFIER and its sensor,
    // measured, and is kept separate from the capability table above, which
    // is physics. Defaults come from the measured composition of each
    // predicted label over the Stage-5B recordings:
    //   PEDESTRIAN n=715 : PEDESTRIAN 381, VEHICLE 306, BIKE 28
    //   BIKE       n=277 : BIKE 277 -- precision 1.000, 0 false predictions
    //   VEHICLE    n=418 : VEHICLE 301, UNKNOWN 117
    //   UNKNOWN    n=1137: UNKNOWN 550, VEHICLE 382, PEDESTRIAN 197, BIKE 8
    // The label is forced into its own set in code, so omitting it from the
    // list cannot weaken invariant (I1).
    declare_ambiguity_param("unknown", amb_def[ObstacleClass::UNKNOWN]);
    declare_ambiguity_param("pedestrian", amb_def[ObstacleClass::PEDESTRIAN]);
    declare_ambiguity_param("bike", amb_def[ObstacleClass::BIKE]);
    declare_ambiguity_param("vehicle", amb_def[ObstacleClass::VEHICLE]);

    // reachability_profile_min_confidence 0.60: the measured FLOOR of the
    //   VEHICLE -> PEDESTRIAN confusion. Below it no label is trusted; above
    //   it the ambiguity sets, not the confidence, are what keep the
    //   selection conservative, because that confusion runs to 1.000 and is
    //   more confident than correct PEDESTRIAN classification (median 1.000
    //   against 0.856). The gate does the job it can do and is not asked to
    //   do the job it cannot.
    // reachability_profile_min_observations 5: the earliest observation at
    //   which the stabiliser can hold a label built from two consecutive
    //   confident scans, plus one.
    // reachability_profile_transition_hold_scans 3: 0.6 s at 5 Hz, one full
    //   classification_smoothing EWMA window (1/0.40 = 2.5 scans), so a
    //   shrink cannot be published until the new label has outlived the
    //   evidence window that produced the old one. 115 stable-label
    //   transitions occurred over 36 of 57 Stage-5B tracks, so this is a
    //   frequent event, not a hypothetical one.
    this->declare_parameter<double>(
      "reachability_profile_min_confidence", pol_def.min_confidence);
    this->declare_parameter<int>(
      "reachability_profile_min_observations",
      static_cast<int>(pol_def.min_observations));
    this->declare_parameter<int>(
      "reachability_profile_transition_hold_scans",
      static_cast<int>(pol_def.transition_hold_scans));

    // --- Stage-5B: dynamic obstacle classification -----------------------
    // Geometry/motion classification of tracked objects into
    // PEDESTRIAN / BIKE / VEHICLE / UNKNOWN. OBSERVATION ONLY: the label is
    // published in the additive TrackedObject.classification* fields and is
    // read by nothing in the prediction or costmap path, so disabling it
    // returns a byte-identical Stage-4G4 payload in every other field.
    //
    // Every threshold below is an interval on a physically measurable
    // dimension of the ~0.16 m LiDAR scan plane. Their derivation, units and
    // the reason each bound has the value it does are documented at the
    // point of definition in obstacle_classification.hpp -- the defaults
    // here mirror ClassificationParams and exist so a scenario can override
    // them without recompiling.
    this->declare_parameter<bool>("classification_enabled", true);
    const ClassificationParams cp_def;
    this->declare_parameter<int>(
      "classification_min_points", static_cast<int>(cp_def.min_points));
    this->declare_parameter<double>("classification_max_range", cp_def.max_range);
    this->declare_parameter<int>(
      "classification_min_observations", static_cast<int>(cp_def.min_observations));
    this->declare_parameter<double>("classification_min_accept_score", cp_def.min_accept_score);
    this->declare_parameter<double>("classification_min_margin", cp_def.min_margin);
    this->declare_parameter<double>("classification_thickness_floor", cp_def.thickness_floor);

    const ClassLabelStabilizer st_def;
    this->declare_parameter<double>("classification_smoothing", st_def.smoothing);
    this->declare_parameter<double>("classification_enter_threshold", st_def.enter_threshold);
    this->declare_parameter<double>("classification_exit_threshold", st_def.exit_threshold);
    this->declare_parameter<double>("classification_switch_margin", st_def.switch_margin);

    // Per-ASPECT geometric boxes, [lo, hi] in the units named in
    // ClusterFeatures: length/thickness/gap in m, fill dimensionless,
    // speed_max in m/s. A class with several aspects exposes one parameter
    // group per aspect, named for the view it describes.
    declare_class_model(aspect_names(), cp_def);
  }

  /// The parameter-group name of every (class, aspect) pair, in the order the
  /// aspects appear in ClassificationParams. Single-aspect classes use the
  /// bare class name so their parameters stay stable.
  static std::vector<std::vector<std::string>> aspect_names()
  {
    return {{"pedestrian"}, {"bike"}, {"vehicle_flank", "vehicle_corner"}};
  }

  void declare_class_model(
    const std::vector<std::vector<std::string>> & names, const ClassificationParams & d)
  {
    const std::vector<const predictive_nav_tracking::ClassModel *> models{
      &d.pedestrian, &d.bike, &d.vehicle};
    for (size_t c = 0; c < models.size(); ++c) {
      for (size_t a = 0; a < models[c]->aspects.size() && a < names[c].size(); ++a) {
        declare_class_box(names[c][a], models[c]->aspects[a]);
      }
    }
  }

  void read_class_model(
    const std::vector<std::vector<std::string>> & names, ClassificationParams & d)
  {
    const std::vector<predictive_nav_tracking::ClassModel *> models{
      &d.pedestrian, &d.bike, &d.vehicle};
    for (size_t c = 0; c < models.size(); ++c) {
      for (size_t a = 0; a < models[c]->aspects.size() && a < names[c].size(); ++a) {
        read_class_box(names[c][a], models[c]->aspects[a]);
      }
    }
  }

  /// Declares the four interval pairs plus the speed veto of one aspect box
  /// under the prefix "classification_<name>_*".
  void declare_class_box(
    const std::string & name, const predictive_nav_tracking::ClassBox & d)
  {
    const std::string k = "classification_" + name + "_";
    this->declare_parameter<double>(k + "length_lo", d.length_lo);
    this->declare_parameter<double>(k + "length_hi", d.length_hi);
    this->declare_parameter<double>(k + "length_soft", d.length_soft);
    this->declare_parameter<double>(k + "thickness_lo", d.thickness_lo);
    this->declare_parameter<double>(k + "thickness_hi", d.thickness_hi);
    this->declare_parameter<double>(k + "thickness_soft", d.thickness_soft);
    this->declare_parameter<double>(k + "fill_lo", d.fill_lo);
    this->declare_parameter<double>(k + "fill_hi", d.fill_hi);
    this->declare_parameter<double>(k + "fill_soft", d.fill_soft);
    this->declare_parameter<double>(k + "gap_lo", d.gap_lo);
    this->declare_parameter<double>(k + "gap_hi", d.gap_hi);
    this->declare_parameter<double>(k + "gap_soft", d.gap_soft);
    this->declare_parameter<double>(k + "speed_max", d.speed_max);
  }

  void read_class_box(const std::string & name, predictive_nav_tracking::ClassBox & b)
  {
    const std::string k = "classification_" + name + "_";
    b.length_lo = this->get_parameter(k + "length_lo").as_double();
    b.length_hi = this->get_parameter(k + "length_hi").as_double();
    b.length_soft = this->get_parameter(k + "length_soft").as_double();
    b.thickness_lo = this->get_parameter(k + "thickness_lo").as_double();
    b.thickness_hi = this->get_parameter(k + "thickness_hi").as_double();
    b.thickness_soft = this->get_parameter(k + "thickness_soft").as_double();
    b.fill_lo = this->get_parameter(k + "fill_lo").as_double();
    b.fill_hi = this->get_parameter(k + "fill_hi").as_double();
    b.fill_soft = this->get_parameter(k + "fill_soft").as_double();
    b.gap_lo = this->get_parameter(k + "gap_lo").as_double();
    b.gap_hi = this->get_parameter(k + "gap_hi").as_double();
    b.gap_soft = this->get_parameter(k + "gap_soft").as_double();
    b.speed_max = this->get_parameter(k + "speed_max").as_double();
  }

  // --- Stage-5D: class-aware capability-profile parameters ---------------

  /// Declares one class's capability triple under the prefix
  /// "reachability_profile_<name>_*". UNKNOWN deliberately has no group: its
  /// profile is DERIVED as the envelope over its ambiguity set, so it cannot
  /// be configured into being less conservative than a class it must cover.
  void declare_profile_params(
    const std::string & name, const MotionCapabilityProfile & d)
  {
    const std::string k = "reachability_profile_" + name + "_";
    this->declare_parameter<double>(k + "max_speed", d.max_speed);
    this->declare_parameter<double>(k + "max_acceleration", d.max_acceleration);
    this->declare_parameter<double>(k + "margin", d.margin);
  }

  void read_profile_params(const std::string & name, MotionCapabilityProfile & p)
  {
    const std::string k = "reachability_profile_" + name + "_";
    p.max_speed = this->get_parameter(k + "max_speed").as_double();
    p.max_acceleration = this->get_parameter(k + "max_acceleration").as_double();
    p.margin = this->get_parameter(k + "margin").as_double();
  }

  /// The class names a mask contains, as the parameter's string list.
  static std::vector<std::string> mask_to_names(uint8_t mask)
  {
    static const char * kNames[] = {"unknown", "pedestrian", "bike", "vehicle"};
    std::vector<std::string> out;
    for (size_t c = 0; c < predictive_nav_tracking::kNumProfileClasses; ++c) {
      if ((mask & predictive_nav_tracking::classBit(static_cast<ObstacleClass>(c))) != 0) {
        out.emplace_back(kNames[c]);
      }
    }
    return out;
  }

  void declare_ambiguity_param(const std::string & label, uint8_t d)
  {
    this->declare_parameter<std::vector<std::string>>(
      "reachability_ambiguity_" + label, mask_to_names(d));
  }

  /// Reads one ambiguity set. An UNRECOGNISED class name is rejected
  /// CONSERVATIVELY: it cannot be silently dropped, because dropping a name
  /// narrows the set and a narrower set means a weaker profile, so an
  /// operator typo would buy a smaller region. Instead the whole set falls
  /// back to "every class", which is the most conservative reading of "I do
  /// not understand this configuration", and the node logs an error.
  uint8_t read_ambiguity_param(const std::string & label)
  {
    const auto names = this->get_parameter("reachability_ambiguity_" + label).as_string_array();
    const uint8_t all = static_cast<uint8_t>(
      predictive_nav_tracking::classBit(ObstacleClass::UNKNOWN) |
      predictive_nav_tracking::classBit(ObstacleClass::PEDESTRIAN) |
      predictive_nav_tracking::classBit(ObstacleClass::BIKE) |
      predictive_nav_tracking::classBit(ObstacleClass::VEHICLE));
    uint8_t mask = 0;
    for (const auto & n : names) {
      if (n == "unknown") {
        mask |= predictive_nav_tracking::classBit(ObstacleClass::UNKNOWN);
      } else if (n == "pedestrian") {
        mask |= predictive_nav_tracking::classBit(ObstacleClass::PEDESTRIAN);
      } else if (n == "bike") {
        mask |= predictive_nav_tracking::classBit(ObstacleClass::BIKE);
      } else if (n == "vehicle") {
        mask |= predictive_nav_tracking::classBit(ObstacleClass::VEHICLE);
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "reachability_ambiguity_%s lists unknown class '%s'. The set is being replaced by "
          "ALL classes, because narrowing an ambiguity set weakens the selected profile and a "
          "configuration this node cannot read must never buy a smaller reachable set.",
          label.c_str(), n.c_str());
        return all;
      }
    }
    return mask;
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

    // --- Stage-5B.1: compound-observation recovery ----------------------
    compound_params_.enabled = this->get_parameter("compound_enabled").as_bool();
    compound_params_.fragment_length_min =
      this->get_parameter("compound_fragment_length_min").as_double();
    compound_params_.fragment_length_max =
      this->get_parameter("compound_fragment_length_max").as_double();
    compound_params_.fragment_thickness_max =
      this->get_parameter("compound_fragment_thickness_max").as_double();
    compound_params_.alignment_min = this->get_parameter("compound_alignment_min").as_double();
    compound_params_.separation_min = this->get_parameter("compound_separation_min").as_double();
    compound_params_.separation_max = this->get_parameter("compound_separation_max").as_double();
    compound_params_.gap_min = this->get_parameter("compound_gap_min").as_double();
    compound_params_.gap_max = this->get_parameter("compound_gap_max").as_double();
    compound_params_.union_length_min =
      this->get_parameter("compound_union_length_min").as_double();
    compound_params_.union_length_max =
      this->get_parameter("compound_union_length_max").as_double();
    compound_params_.union_thickness_max =
      this->get_parameter("compound_union_thickness_max").as_double();
    compound_params_.union_fill_max = this->get_parameter("compound_union_fill_max").as_double();
    compound_params_.min_support =
      static_cast<uint32_t>(this->get_parameter("compound_min_support").as_int());
    compound_params_.separation_tolerance =
      this->get_parameter("compound_separation_tolerance").as_double();
    compound_params_.match_radius = this->get_parameter("compound_match_radius").as_double();
    compound_params_.max_gap_seconds =
      this->get_parameter("compound_max_gap_seconds").as_double();
    compound_tracker_.setParams(compound_params_);

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

    reachability_enabled_ = this->get_parameter("reachability_enabled").as_bool();
    reachability_bounds_.max_acceleration = this->get_parameter("max_acceleration").as_double();
    reachability_bounds_.max_speed = this->get_parameter("max_speed").as_double();
    reachability_bounds_.safety_margin = this->get_parameter("base_safety_margin").as_double();
    reachability_bounds_.covariance_sigma_level =
      this->get_parameter("covariance_sigma_level").as_double();
    reachability_bounds_.enforce_observed_speed =
      this->get_parameter("reachability_enforce_observed_speed").as_bool();
    reachability_bounds_.max_speed_tolerance =
      std::max(this->get_parameter("reachability_max_speed_tolerance").as_double(), 0.0);

    // --- Stage-5D: class-aware capability profiles -----------------------
    profile_policy_.class_aware = this->get_parameter("reachability_class_aware").as_bool();
    profile_policy_.min_confidence =
      this->get_parameter("reachability_profile_min_confidence").as_double();
    profile_policy_.min_observations = static_cast<uint32_t>(
      std::max<int64_t>(this->get_parameter("reachability_profile_min_observations").as_int(), 0));
    profile_policy_.transition_hold_scans = static_cast<uint32_t>(
      std::max<int64_t>(
        this->get_parameter("reachability_profile_transition_hold_scans").as_int(), 0));

    read_profile_params("pedestrian", profile_set_[ObstacleClass::PEDESTRIAN]);
    read_profile_params("bike", profile_set_[ObstacleClass::BIKE]);
    read_profile_params("vehicle", profile_set_[ObstacleClass::VEHICLE]);
    // UNKNOWN's stored entry is never read -- the fallback is derived from the
    // ambiguity set -- but it is zeroed explicitly so a future reader of the
    // table cannot mistake a default for a configured value.
    profile_set_[ObstacleClass::UNKNOWN] = MotionCapabilityProfile{0.0, 0.0, 0.0};

    ambiguity_sets_.mask[static_cast<size_t>(ObstacleClass::UNKNOWN)] =
      read_ambiguity_param("unknown");
    ambiguity_sets_.mask[static_cast<size_t>(ObstacleClass::PEDESTRIAN)] =
      read_ambiguity_param("pedestrian");
    ambiguity_sets_.mask[static_cast<size_t>(ObstacleClass::BIKE)] =
      read_ambiguity_param("bike");
    ambiguity_sets_.mask[static_cast<size_t>(ObstacleClass::VEHICLE)] =
      read_ambiguity_param("vehicle");

    // Sanitize ONCE, here, not per scan. A malformed entry is replaced by a
    // dominating substitute so it can never produce an optimistic region, and
    // the failure is reported on every published sample as
    // PROFILE_FALLBACK_INVALID_CONFIG rather than only in this log line.
    profile_config_valid_ = predictive_nav_tracking::sanitizeProfileSet(profile_set_);
    if (!profile_config_valid_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "reachability capability profiles are MALFORMED (a non-finite or negative max_speed, "
        "max_acceleration or margin). Every track is now served the conservative fallback "
        "envelope and every published sample reports "
        "capability_profile_reason=PROFILE_FALLBACK_INVALID_CONFIG. Class-aware reachability "
        "is NOT qualified in this state -- fix the profile parameters.");
    }
    if (profile_policy_.class_aware) {
      const MotionCapabilityProfile fb =
        predictive_nav_tracking::fallbackProfile(profile_set_, ambiguity_sets_);
      RCLCPP_INFO(
        this->get_logger(),
        "Stage-5D class-aware reachability ENABLED. profiles (max_speed m/s, max_accel m/s^2, "
        "margin m): PEDESTRIAN %.2f/%.2f/%.2f BIKE %.2f/%.2f/%.2f VEHICLE %.2f/%.2f/%.2f "
        "FALLBACK(derived) %.2f/%.2f/%.2f; gates confidence>=%.2f observations>=%u "
        "transition_hold=%u scans. These are DECLARED future-capability premises, not proven "
        "bounds -- Stage-5C.1 applies unchanged.",
        profile_set_[ObstacleClass::PEDESTRIAN].max_speed,
        profile_set_[ObstacleClass::PEDESTRIAN].max_acceleration,
        profile_set_[ObstacleClass::PEDESTRIAN].margin,
        profile_set_[ObstacleClass::BIKE].max_speed,
        profile_set_[ObstacleClass::BIKE].max_acceleration,
        profile_set_[ObstacleClass::BIKE].margin,
        profile_set_[ObstacleClass::VEHICLE].max_speed,
        profile_set_[ObstacleClass::VEHICLE].max_acceleration,
        profile_set_[ObstacleClass::VEHICLE].margin,
        fb.max_speed, fb.max_acceleration, fb.margin,
        profile_policy_.min_confidence,
        static_cast<unsigned>(profile_policy_.min_observations),
        static_cast<unsigned>(profile_policy_.transition_hold_scans));
    }
    reachability_horizon_ = this->get_parameter("reachability_horizon").as_double();
    reachability_time_step_ = this->get_parameter("reachability_time_step").as_double();
    max_observation_age_ = this->get_parameter("max_observation_age").as_double();
    num_reachability_samples_ = (reachability_time_step_ > 1e-6) ?
      static_cast<size_t>(std::lround(reachability_horizon_ / reachability_time_step_)) : 0;

    // --- Stage-5B: classification --------------------------------------
    classification_enabled_ = this->get_parameter("classification_enabled").as_bool();
    classification_params_.min_points = static_cast<uint32_t>(
      this->get_parameter("classification_min_points").as_int());
    classification_params_.max_range =
      this->get_parameter("classification_max_range").as_double();
    classification_params_.min_observations = static_cast<uint32_t>(
      this->get_parameter("classification_min_observations").as_int());
    classification_params_.min_accept_score =
      this->get_parameter("classification_min_accept_score").as_double();
    classification_params_.min_margin =
      this->get_parameter("classification_min_margin").as_double();
    classification_params_.thickness_floor =
      this->get_parameter("classification_thickness_floor").as_double();
    read_class_model(aspect_names(), classification_params_);

    stabilizer_prototype_.smoothing =
      this->get_parameter("classification_smoothing").as_double();
    stabilizer_prototype_.enter_threshold =
      this->get_parameter("classification_enter_threshold").as_double();
    stabilizer_prototype_.exit_threshold =
      this->get_parameter("classification_exit_threshold").as_double();
    stabilizer_prototype_.switch_margin =
      this->get_parameter("classification_switch_margin").as_double();
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

    // Stage-5B: the sensor origin in the map frame and the beam spacing are
    // needed to express cluster geometry relative to the observer (range,
    // angular span, fill ratio). Both are taken from THIS scan and its
    // transform -- never assumed -- so a re-configured LiDAR is handled
    // automatically. Nothing else reads them.
    sensor_origin_.x = transform.transform.translation.x;
    sensor_origin_.y = transform.transform.translation.y;
    if (std::isfinite(msg->angle_increment) && std::abs(msg->angle_increment) > 0.0) {
      classification_params_.angle_increment = std::abs(msg->angle_increment);
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
    std::vector<size_t> measurement_origins;
    const std::vector<Measurement> primary_measurements =
      measurements_from_clusters(clusters, stamp, &measurement_origins);
    const auto deblend_end = std::chrono::steady_clock::now();
    // Stage-5B.1: the compound stage sits HERE -- after primary clustering and
    // Stage-4G3 deblending, before association -- so association still
    // receives one measurement per believed object. With
    // compound_enabled=false it returns `primary_measurements` unchanged.
    const auto compound_start = std::chrono::steady_clock::now();
    const size_t compounds_before = compound_emitted_count_;
    const std::vector<Measurement> measurements = apply_compound_association(
      clusters, measurement_origins, primary_measurements, stamp);
    const auto compound_end = std::chrono::steady_clock::now();
    const auto association_start = std::chrono::steady_clock::now();
    associate_measurements_and_update(measurements, stamp);
    const auto association_end = std::chrono::steady_clock::now();
    prune_stale_tracks(stamp);
    // Stage-4G4: reachability is computed inside publish_tracks and accumulated
    // into reachability_total_us_, so its cost is separable from the rest of
    // message construction.
    reachability_total_us_ = 0.0;
    const auto publish_start = std::chrono::steady_clock::now();
    publish_tracks(stamp);
    const auto publish_end = std::chrono::steady_clock::now();
    std::vector<Point2D> centroids;
    centroids.reserve(measurements.size());
    for (const auto & m : measurements) {
      centroids.push_back(m.centroid);
    }
    publish_markers(stamp, centroids);
    const auto callback_end = std::chrono::steady_clock::now();
    if (get_parameter("profile_scans").as_bool()) {
      const auto us = [](auto a, auto b) {
          return std::chrono::duration<double, std::micro>(b - a).count();
        };
      RCLCPP_INFO(get_logger(),
        "G2_PERF %.9f scan_us=%.3f cluster_us=%.3f deblend_us=%.3f compound_us=%.3f "
        "association_us=%.3f reach_us=%.3f publish_us=%.3f "
        "callback_us=%.3f clusters=%zu measurements=%zu tracks=%zu splits=%zu ambiguous=%zu "
        "compound_candidates=%zu compounds=%zu "
        "reach_contract_violations=%lu reach_contract_worst_speed=%.3f "
        "prof_global=%lu prof_class=%lu prof_fb_unknown=%lu prof_fb_lowconf=%lu "
        "prof_fb_immature=%lu prof_fb_badconfig=%lu prof_transition=%lu",
        stamp.seconds(), us(callback_start, cluster_start), us(cluster_start, cluster_end),
        us(deblend_start, deblend_end), us(compound_start, compound_end),
        us(association_start, association_end),
        reachability_total_us_, us(publish_start, publish_end),
        us(callback_start, callback_end),
        clusters.size(), measurements.size(), tracks_.size(),
        deblend_split_count_ - splits_before, ambiguous_merge_count_,
        compound_candidate_count_, compound_emitted_count_ - compounds_before,
        static_cast<unsigned long>(reachability_contract_violation_samples_),
        contract_violation_worst_speed_,
        // Stage-5D: cumulative published-sample counts per profile reason, so
        // fallback rate, UNKNOWN-profile rate and transition rate are
        // measurable from a live run and not only from an offline replay.
        static_cast<unsigned long>(profile_reason_samples_[0]),
        static_cast<unsigned long>(profile_reason_samples_[1]),
        static_cast<unsigned long>(profile_reason_samples_[2]),
        static_cast<unsigned long>(profile_reason_samples_[3]),
        static_cast<unsigned long>(profile_reason_samples_[4]),
        static_cast<unsigned long>(profile_reason_samples_[5]),
        static_cast<unsigned long>(profile_reason_samples_[6]));
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
  ///
  /// Stage-5B: each centroid now travels with the scan-plane geometry of the
  /// cluster it came from. The CENTROIDS THEMSELVES ARE UNCHANGED -- the
  /// split decision, the gap test and every returned position are computed
  /// exactly as before -- so association and the filter see the same
  /// measurements they always did.
  ///
  /// A DEBLENDED CHILD IS DELIBERATELY GIVEN INVALID FEATURES. Its point set
  /// was partitioned using the tracker's own Kalman predictions as seeds, so
  /// its apparent length, thickness and fill ratio are partly a function of
  /// the tracker's hypothesis rather than of the object. Classifying on that
  /// geometry would feed the tracker's belief back in as evidence. During a
  /// merge the affected tracks therefore contribute no new class evidence and
  /// hold their existing stabilised label while it decays, which is exactly
  /// what the hysteresis is for.
  /// Sentinel for "this measurement is not one whole primary cluster", used
  /// in the `origins` vector below.
  static constexpr size_t kNoClusterOrigin = std::numeric_limits<size_t>::max();

  std::vector<Measurement> measurements_from_clusters(
    std::vector<Cluster> & clusters, const rclcpp::Time & stamp,
    std::vector<size_t> * origins = nullptr)
  {
    // Stage-5B.1: `origins` records, per emitted measurement, WHICH primary
    // cluster it is the whole of -- or kNoClusterOrigin for a deblended
    // child, which is only a PART of one. The compound stage must never pair
    // a deblended child: its point set was partitioned using the tracker's
    // own Kalman predictions as seeds, so its geometry is partly a function
    // of the tracker's hypothesis, and building a second hypothesis on top of
    // it would compound two inferences. Recording the origin here, where the
    // distinction is already known, is what lets the compound stage enforce
    // that without re-deriving it.
    std::vector<Measurement> measurements;
    measurements.reserve(clusters.size() + 1);
    if (origins != nullptr) {
      origins->clear();
      origins->reserve(clusters.size() + 1);
    }
    const auto note_origin = [&origins](size_t idx) {
        if (origins != nullptr) {
          origins->push_back(idx);
        }
      };

    if (!deblend_enabled_) {
      for (size_t i = 0; i < clusters.size(); ++i) {
        measurements.push_back({clusters[i].centroid, features_of(clusters[i])});
        note_origin(i);
      }
      return measurements;
    }

    const std::vector<DeblendSeed> seeds = deblend_seeds(stamp);
    std::vector<Point2D> children;
    for (size_t i = 0; i < clusters.size(); ++i) {
      Cluster & c = clusters[i];
      bool ambiguous = false;
      if (try_deblend(c, seeds, children, ambiguous)) {
        for (const auto & child : children) {
          measurements.push_back({child, ClusterFeatures{}});
          note_origin(kNoClusterOrigin);
        }
        ++deblend_split_count_;
      } else {
        // Unsplit merged clusters keep their single blended centroid, which
        // greedy association gives to exactly ONE track (its assignment is
        // one-to-one). The other track receives nothing and coasts. No centroid
        // is ever used to update two filters.
        measurements.push_back({c.centroid, features_of(c)});
        note_origin(i);
        c.ambiguous_merge = ambiguous;
        if (ambiguous) {
          ++ambiguous_merge_count_;
        }
      }
    }
    return measurements;
  }

  // ---------------------------------------------------------------------
  // Stage-5B.1: gap-aware compound-observation recovery
  // ---------------------------------------------------------------------
  /// Replaces strongly-compatible FRAGMENT PAIRS with one compound
  /// observation, leaving every other measurement exactly as it was.
  ///
  /// WHAT THIS DOES NOT DO:
  ///   * it does not change cluster_distance or re-run clustering;
  ///   * it does not touch a Kalman filter, a track, or a class label;
  ///   * it does not look at any object's true class or model name;
  ///   * it never pairs a Stage-4G3 deblended child (see `origins` above);
  ///   * it never pairs two clusters whose gap is within the primary link
  ///     distance -- structurally impossible for two separate clusters, and
  ///     checked anyway by evaluateCompoundPair().
  ///
  /// A pair must satisfy nine geometric bounds AND hold them across
  /// `compound_min_support` consecutive scans with a stable separation. On
  /// acceptance the two fragment measurements are REPLACED by a single
  /// measurement carrying the pooled points' geometry, so association sees
  /// one measurement for one physical object -- which is the invariant the
  /// rest of the tracker is built on.
  ///
  /// The raw clusters are left untouched: `clusters` is const here, so the
  /// original measurements remain reconstructible and the decision auditable.
  std::vector<Measurement> apply_compound_association(
    const std::vector<Cluster> & clusters, const std::vector<size_t> & origins,
    const std::vector<Measurement> & measurements, const rclcpp::Time & stamp)
  {
    if (!compound_params_.enabled) {
      return measurements;           // provable no-op
    }
    if (measurements.size() < 2 || origins.size() != measurements.size()) {
      return measurements;
    }

    // Eligible: a whole primary cluster with usable shape statistics.
    std::vector<size_t> eligible;
    for (size_t i = 0; i < measurements.size(); ++i) {
      if (origins[i] != kNoClusterOrigin && measurements[i].features.valid) {
        eligible.push_back(i);
      }
    }
    if (eligible.size() < 2) {
      return measurements;
    }

    std::vector<CompoundHypothesisTracker::Candidate> candidates;
    for (size_t a = 0; a < eligible.size(); ++a) {
      for (size_t b = a + 1; b < eligible.size(); ++b) {
        const size_t ia = eligible[a];
        const size_t ib = eligible[b];
        // Cheap necessary condition first, from the summary features only.
        // screenCompoundPair() applies a SUBSET of the same bounds, so a pair
        // it rejects could not have been accepted; skipping it changes no
        // outcome and avoids the O(|A|*|B|) nearest-gap measurement and the
        // pooled-geometry extraction for the vast majority of pairs.
        if (predictive_nav_tracking::screenCompoundPair(
            measurements[ia].features, measurements[ib].features,
            compound_params_) != predictive_nav_tracking::kCompoundAccept)
        {
          continue;
        }
        const CompoundEvidence e = predictive_nav_tracking::evaluateCompoundPair(
          measurements[ia].features, measurements[ib].features,
          clusters[origins[ia]].points, clusters[origins[ib]].points,
          sensor_origin_.x, sensor_origin_.y, cluster_distance_,
          compound_params_, classification_params_);
        if (!e.geometry_ok) {
          continue;
        }
        CompoundHypothesisTracker::Candidate c;
        c.index_a = ia;
        c.index_b = ib;
        c.evidence = e;
        candidates.push_back(c);
      }
    }
    compound_candidate_count_ += candidates.size();
    if (candidates.empty()) {
      // Still advance the temporal state so that stale hypotheses expire and
      // support cannot survive a scan on which the geometry failed.
      compound_tracker_.update(stamp.seconds(), candidates);
      return measurements;
    }

    const std::vector<bool> emit = compound_tracker_.update(stamp.seconds(), candidates);

    // A cluster may belong to at most ONE compound observation. Strongest
    // evidence first, by collinearity then by stable index, so the outcome is
    // deterministic and independent of pair enumeration order.
    std::vector<size_t> order;
    for (size_t i = 0; i < candidates.size(); ++i) {
      if (emit[i]) {
        order.push_back(i);
      }
    }
    std::sort(
      order.begin(), order.end(),
      [&candidates](size_t l, size_t r) {
        if (candidates[l].evidence.alignment != candidates[r].evidence.alignment) {
          return candidates[l].evidence.alignment > candidates[r].evidence.alignment;
        }
        return l < r;
      });

    std::vector<bool> consumed(measurements.size(), false);
    std::vector<Measurement> compounds;
    for (const size_t k : order) {
      const auto & c = candidates[k];
      if (consumed[c.index_a] || consumed[c.index_b]) {
        continue;
      }
      consumed[c.index_a] = true;
      consumed[c.index_b] = true;
      Measurement m;
      m.centroid.x = c.evidence.union_features.centroid_x;
      m.centroid.y = c.evidence.union_features.centroid_y;
      m.features = c.evidence.union_features;
      m.compound = true;
      compounds.push_back(m);
      ++compound_emitted_count_;
      RCLCPP_DEBUG(
        get_logger(),
        "G5B1_COMPOUND %.9f sep=%.3f align=%.3f gap=%.3f ulen=%.3f uthk=%.3f ufill=%.2f",
        stamp.seconds(), c.evidence.separation, c.evidence.alignment, c.evidence.gap,
        c.evidence.union_length, c.evidence.union_thickness, c.evidence.union_fill);
    }
    if (compounds.empty()) {
      return measurements;
    }

    std::vector<Measurement> out;
    out.reserve(measurements.size());
    for (size_t i = 0; i < measurements.size(); ++i) {
      if (!consumed[i]) {
        out.push_back(measurements[i]);
      }
    }
    for (auto & m : compounds) {
      out.push_back(std::move(m));
    }
    return out;
  }

  /// Scan-plane geometry of one accepted cluster, relative to this scan's
  /// sensor origin. Returns an invalid feature set when classification is
  /// disabled, so the whole feature path costs nothing in that case.
  ClusterFeatures features_of(const Cluster & c) const
  {
    if (!classification_enabled_) {
      return ClusterFeatures{};
    }
    return predictive_nav_tracking::extractFeatures(
      c.points, sensor_origin_.x, sensor_origin_.y, classification_params_);
  }

  // ---------------------------------------------------------------------
  // Association / track maintenance
  // ---------------------------------------------------------------------
  double clamp_dt(double dt) const
  {
    return std::clamp(dt, kf_min_dt_, kf_max_dt_);
  }

  /// Position-only association. FROZEN SIGNATURE: this is the entry point
  /// the Stage-4G2/4G3/4G4 tests drive, and its behaviour is unchanged --
  /// bare centroids simply carry no cluster geometry, so the associated
  /// tracks receive no new class evidence on this call. A deliberately
  /// DISTINCT NAME is used for the feature-carrying form below rather than an
  /// overload, because `{{x, y}}` call sites would otherwise become
  /// ambiguous between the two parameter types.
  void associate_and_update(const std::vector<Point2D> & centroids, const rclcpp::Time & stamp)
  {
    std::vector<Measurement> measurements;
    measurements.reserve(centroids.size());
    for (const auto & c : centroids) {
      measurements.push_back({c, ClusterFeatures{}});
    }
    associate_measurements_and_update(measurements, stamp);
  }

  /// Stage-5B: association on measurements that carry their cluster geometry.
  /// Identical to the position-only form in every step that touches gating,
  /// assignment or the Kalman filter -- it reads `centroid` for all of those
  /// -- and additionally hands `features` to the classifier.
  void associate_measurements_and_update(
    const std::vector<Measurement> & measurements, const rclcpp::Time & stamp)
  {
    const size_t n_tracks = tracks_.size();
    const size_t n_dets = measurements.size();

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
          predicted[t].x - measurements[d].centroid.x,
          predicted[t].y - measurements[d].centroid.y);
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
      update_track(tracks_[c.track_idx], measurements[c.det_idx], stamp);
    }

    for (size_t t = 0; t < n_tracks; ++t) {
      if (!track_assigned[t]) {
        tracks_[t].missed_count += 1;
        tracks_[t].age += 1;
        // Stage-5B: a coasting scan carries no geometry, so it contributes no
        // class evidence -- but it still DECAYS the accumulated scores, so a
        // label fades out while an object stays unobserved instead of
        // persisting on stale evidence.
        if (classification_enabled_) {
          tracks_[t].class_features = ClusterFeatures{};
          tracks_[t].class_instant = ClassificationResult{};
          tracks_[t].class_from_compound = false;
          tracks_[t].class_state.update(tracks_[t].class_instant);
        }
      }
    }

    for (size_t d = 0; d < n_dets; ++d) {
      if (!det_assigned[d]) {
        tracks_.push_back(make_new_track(measurements[d], stamp, next_track_id_++));
      }
    }
  }

  Track make_new_track(
    const Measurement & meas, const rclcpp::Time & stamp, uint32_t id) const
  {
    const Point2D & det = meas.centroid;
    Track track;
    track.id = id;

    // Stage-5B: a brand-new ID starts from UNKNOWN with zero accumulated
    // evidence. The stabiliser is configured from the node's parameters but
    // inherits nothing from any previous track. With classification disabled
    // the geometry is not recorded either, so the published class_* fields are
    // exactly their defaults.
    track.class_state = stabilizer_prototype_;
    if (classification_enabled_) {
      track.class_features = meas.features;
      track.class_from_compound = meas.compound;
    }

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

  void update_track(Track & track, const Measurement & meas, const rclcpp::Time & stamp)
  {
    const Point2D & det = meas.centroid;
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

    // --- Stage-5B: classification (metadata only) -------------------------
    // Runs AFTER the filter update so the speed veto sees this scan's
    // filtered velocity, and writes only to the track's class_* fields. It
    // cannot affect kf_x, kf_P, the predictions or the reachable sets.
    if (classification_enabled_) {
      classify_track(track, meas);
    }
  }

  /// Folds one scan's geometry into `track`'s classification state.
  void classify_track(Track & track, const Measurement & meas) const
  {
    predictive_nav_tracking::TrackMotion motion;
    motion.speed = track.kalman_initialized ?
      std::hypot(track.kf_x(2), track.kf_x(3)) : std::hypot(track.raw_vx, track.raw_vy);
    motion.speed_valid = track.kalman_initialized;
    motion.observations = track.observations;

    track.class_features = meas.features;
    track.class_from_compound = meas.compound;
    track.class_instant = predictive_nav_tracking::classifyInstant(
      meas.features, motion, classification_params_);
    track.class_state.update(track.class_instant);
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
  // Conservative bounded-motion reachable sets (Stage-4G4)
  // ---------------------------------------------------------------------
  // Produces an ADDITIONAL, deterministic representation of the same track's
  // future. The live filter is not touched, and predict_future() above is not
  // touched: the two representations are computed independently from the same
  // (kf_x, kf_P).
  //
  // OBSERVATION AGE IS EXPLICIT AND CENTRAL. `scan_stamp` is now; the track's
  // filtered state belongs to `track.last_update`, which for a coasting track
  // is in the past. Every region therefore uses
  //     total_time = observation_age + horizon
  // for BOTH the constant-velocity nominal centre and the bound around it, so
  // a track that has not been seen for 0.8 s is expanded for 0.8 s of
  // unobserved motion instead of being presented as freshly measured. This is
  // exactly the semantic the legacy Stage-4C message deliberately does not
  // have, and it is why reachability is published in its own field rather than
  // written into the existing one.
  /// --- Stage-5D: choose and record one track's capability premises -----
  ///
  /// Advances the track's transition state, so it is called EXACTLY ONCE per
  /// track per scan, from publish_tracks(), before predict_reachability().
  /// Reads the ALREADY-STABILISED Stage-5B label that update_track() wrote
  /// earlier in this same scan, so there is no one-scan lag and no new
  /// plumbing, and the one decision is shared by every horizon sample of the
  /// track -- six samples of one track can never describe two profiles.
  ///
  /// With reachability_class_aware false this returns reachability_bounds_
  /// verbatim, so every published number is bit-identical to Stage-5C.1.
  ReachabilityBounds select_track_bounds(Track & track)
  {
    // Guarded on exactly the conditions predict_reachability() uses, so that
    // when reachability is off -- the PRODUCTION setting -- no selection runs,
    // no per-track profile state is advanced and no work is added to the
    // production path at all. Production is therefore inert, not merely
    // numerically equivalent.
    if (!reachability_enabled_ || !track.kalman_initialized ||
      num_reachability_samples_ == 0)
    {
      return reachability_bounds_;
    }
    const MotionCapabilityProfile global_profile{
      reachability_bounds_.max_speed,
      reachability_bounds_.max_acceleration,
      reachability_bounds_.safety_margin};
    track.class_profile_selection = predictive_nav_tracking::selectCapabilityProfile(
      track.class_state.label(), track.class_state.confidence(), track.observations,
      profile_set_, ambiguity_sets_, profile_policy_, global_profile,
      profile_config_valid_, track.class_profile_state);
    return predictive_nav_tracking::boundsForProfile(
      reachability_bounds_, track.class_profile_selection.profile);
  }

  /// `bounds` overrides the global configuration for this track; nullptr
  /// means "use the single global configuration", which is both the
  /// Stage-5C.1 behaviour and what the pre-Stage-5D tests exercise, so the
  /// legacy call shape stays available and stays exact.
  std::vector<ReachabilityRegion> predict_reachability(
    const Track & track, const rclcpp::Time & scan_stamp,
    const ReachabilityBounds * bounds = nullptr) const
  {
    std::vector<ReachabilityRegion> regions;
    if (!reachability_enabled_ || !track.kalman_initialized || num_reachability_samples_ == 0) {
      return regions;
    }
    const ReachabilityBounds & track_bounds =
      (bounds != nullptr) ? *bounds : reachability_bounds_;

    const double observation_age = std::max((scan_stamp - track.last_update).seconds(), 0.0);
    regions.reserve(num_reachability_samples_);

    for (size_t i = 1; i <= num_reachability_samples_; ++i) {
      const double horizon = static_cast<double>(i) * reachability_time_step_;
      const double total_time = observation_age + horizon;

      // Stage-4C propagated position covariance at the SAME total time, so the
      // statistical term is the identical mathematics the CV representation
      // uses -- only the deterministic term is new.
      const Eigen::Matrix4d F = ConstantVelocityKalmanFilter2D::stateTransition(total_time);
      const Eigen::Matrix4d Pp = F * track.kf_P * F.transpose() +
        ConstantVelocityKalmanFilter2D::processNoise(total_time, kf_accel_noise_);

      ReachabilityRegion region = predictive_nav_tracking::buildRegion(
        track.kf_x(0), track.kf_x(1), track.kf_x(2), track.kf_x(3),
        Pp.topLeftCorner<2, 2>(), total_time, track_bounds);

      // A region is only meaningful while the track's information is recent
      // enough to be worth extrapolating; past that it is published as invalid
      // rather than silently grown without limit.
      if (max_observation_age_ > 0.0 && observation_age > max_observation_age_) {
        region.valid = false;
      }
      regions.push_back(region);
    }
    return regions;
  }

  // ---------------------------------------------------------------------
  // Publishing
  // ---------------------------------------------------------------------
  void publish_tracks(const rclcpp::Time & stamp)
  {
    predictive_nav_msgs::msg::TrackedObjectArray array_msg;
    array_msg.header.stamp = stamp;
    array_msg.header.frame_id = map_frame_;

    // Stage-5D: non-const because predict_reachability() advances the track's
    // capability-profile transition state (capability_profiles.hpp section 6).
    // Nothing else in this loop mutates a track.
    for (auto & t : tracks_) {
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

      // --- Stage-5B: additive classification metadata --------------------
      // The published label is the STABILISED one; the raw per-scan guess is
      // carried alongside it so an evaluation can separate per-detection
      // accuracy from stable per-track accuracy. Every field below is new in
      // Stage-5B and is read by no existing consumer.
      obj.classification = static_cast<uint8_t>(t.class_state.label());
      obj.classification_confidence = t.class_state.confidence();
      obj.classification_instant = static_cast<uint8_t>(t.class_instant.label);
      obj.classification_instant_confidence = t.class_instant.confidence;

      obj.classification_features_valid = t.class_features.valid;
      obj.cluster_length = t.class_features.length;
      obj.cluster_thickness = t.class_features.thickness;
      obj.cluster_elongation = t.class_features.elongation;
      obj.cluster_point_count = t.class_features.point_count;
      obj.cluster_range = t.class_features.range;
      obj.cluster_angular_span = t.class_features.angular_span;
      obj.cluster_fill_ratio = t.class_features.fill_ratio;
      obj.cluster_max_gap = t.class_features.max_gap;
      obj.classification_from_compound = t.class_from_compound;

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

      // Stage-4G4: the additive reachability field. Note the different time
      // anchor -- these samples count from `stamp` (this scan), not from
      // t.last_update -- which is recorded in every sample's observation_age
      // and total_time so no consumer has to infer it.
      const auto reach_start = std::chrono::steady_clock::now();
      // Stage-5D: the profile decision and the regions it produced are timed
      // together, so reach_us keeps measuring the whole reachability cost.
      const ReachabilityBounds track_bounds = select_track_bounds(t);
      const std::vector<ReachabilityRegion> reach_regions =
        predict_reachability(t, stamp, &track_bounds);
      reachability_total_us_ +=
        std::chrono::duration<double, std::micro>(
        std::chrono::steady_clock::now() - reach_start).count();
      for (const auto & r : reach_regions) {
        predictive_nav_msgs::msg::ReachabilityPrediction reach;
        reach.time_from_now = r.total_time - (stamp - t.last_update).seconds();
        reach.stamp = stamp + rclcpp::Duration::from_seconds(reach.time_from_now);
        reach.observation_age = (stamp - t.last_update).seconds();
        reach.total_time = r.total_time;
        reach.position.x = r.center_x;
        reach.position.y = r.center_y;
        reach.position.z = 0.0;
        reach.velocity.x = r.vx;
        reach.velocity.y = r.vy;
        reach.velocity.z = 0.0;
        reach.reach_radius = r.reach_radius;
        reach.speed_capped = r.speed_capped;
        reach.sigma_semi_major = r.sigma_semi_major;
        reach.sigma_semi_minor = r.sigma_semi_minor;
        reach.sigma_yaw = r.sigma_yaw;
        reach.covariance_sigma_level = reachability_bounds_.covariance_sigma_level;
        reach.safety_margin = r.safety_margin;
        reach.semi_major = r.semi_major;
        reach.semi_minor = r.semi_minor;
        reach.valid = r.valid;
        // Stage-5C.1 validity-contract metadata. Published on every sample so
        // that a consumer can never mistake an out-of-contract region for a
        // qualified conservative bound.
        reach.observed_speed = r.observed_speed;
        reach.configured_max_speed = r.configured_max_speed;
        reach.effective_max_speed = r.effective_max_speed;
        reach.contract_status = static_cast<uint8_t>(r.contract_status);
        // Stage-5D capability-profile metadata. Which premises this sample was
        // built with, and why -- published on every sample because under
        // class-aware selection they are per-track and no longer readable from
        // a node parameter. configured_max_speed above already carries (P2) for
        // the selected profile, since boundsForProfile() wrote it into the
        // bounds buildRegion() used.
        reach.configured_max_acceleration =
          t.class_profile_selection.profile.max_acceleration;
        reach.capability_profile_class =
          static_cast<uint8_t>(t.class_profile_selection.dominating_class);
        reach.capability_profile_reason =
          static_cast<uint8_t>(t.class_profile_selection.reason);
        ++profile_reason_samples_[
          static_cast<size_t>(t.class_profile_selection.reason) % profile_reason_samples_.size()];
        if (r.contract_status ==
          predictive_nav_tracking::ContractStatus::kObservedSpeedExceedsCap)
        {
          ++reachability_contract_violation_samples_;
          contract_violation_worst_speed_ =
            std::max(contract_violation_worst_speed_, r.observed_speed);
          // Throttled, because a track above the cap stays above it for many
          // scans and six horizon samples each.
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "reachability contract: track %u filtered speed %.3f m/s exceeds configured "
            "max_speed %.3f m/s, which DISPROVES the model's capability premise. The cap "
            "was raised to %.3f m/s for this sample (region enlarged, never dropped) and "
            "contract_status=%u is published. Raise max_speed to this obstacle class's "
            "real capability -- the published region is NOT a qualified conservative "
            "bound at the configured value.",
            t.id, r.observed_speed, r.configured_max_speed, r.effective_max_speed,
            static_cast<unsigned>(r.contract_status));
        }
        obj.reachability_predictions.push_back(reach);
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

  // --- Stage-5B.1: gap-aware compound-observation recovery ---
  // Operates strictly between primary clustering and association. Holds no
  // track state and reads no class label.
  CompoundParams compound_params_{};
  CompoundHypothesisTracker compound_tracker_{};
  /// Diagnostic counters, published only through the G2_PERF log line.
  size_t compound_candidate_count_{0};   ///< pairs that passed the geometry
  size_t compound_emitted_count_{0};     ///< pairs that also passed the temporal gate

  // --- Stage-4G4: bounded-motion reachability ---
  bool reachability_enabled_{true};
  ReachabilityBounds reachability_bounds_{};
  double reachability_horizon_{3.0};
  double reachability_time_step_{0.5};
  double max_observation_age_{1.0};
  size_t num_reachability_samples_{6};
  double reachability_total_us_{0.0};
  // Stage-5C.1: how many published samples carried a DISPROVED capability
  // premise, and the worst filtered speed that disproved it. Cumulative over
  // the node's lifetime, reported in the throttled stats log, so a
  // mis-parameterised max_speed leaves a measurable trace instead of only a
  // warning that may be throttled away.
  uint64_t reachability_contract_violation_samples_{0};
  double contract_violation_worst_speed_{0.0};

  // --- Stage-5D: class-aware capability profiles ---
  // The ONLY new coupling between classification and prediction. Everything
  // here is configuration plus counters; the per-track transition state lives
  // on the Track so it dies with the track id.
  CapabilityProfileSet profile_set_{};
  AmbiguitySets ambiguity_sets_{};
  ProfileSelectionPolicy profile_policy_{};
  /// False when sanitizeProfileSet() had to replace a malformed entry. Forces
  /// the fallback on every track and is published on every sample.
  bool profile_config_valid_{true};
  /// Cumulative published-sample counts per ProfileReason, reported in the
  /// throttled stats log so fallback rate and transition rate are measurable
  /// in a live run rather than only offline. Indexed by the enum value.
  std::array<uint64_t, 7> profile_reason_samples_{};

  // --- Stage-5B: dynamic obstacle classification ---
  // OBSERVATION ONLY. Nothing in this block is read by the filter, the CV
  // prediction, the reachable sets or the costmap layer.
  bool classification_enabled_{true};
  ClassificationParams classification_params_{};
  /// Stabiliser settings a NEW track is born with; never itself updated.
  ClassLabelStabilizer stabilizer_prototype_{};
  /// This scan's LiDAR origin in the map frame, from the scan's own
  /// transform. All range-relative cluster features are measured from here.
  Point2D sensor_origin_{};

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
