#include "predictive_nav_costmap/predicted_obstacle_layer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
// tf2::getYaw(geometry_msgs::msg::Quaternion) resolves through
// tf2::impl::toQuaternion -> tf2::fromMsg(), which is only *declared* by
// tf2/impl/utils.hpp; its inline definition lives in tf2_geometry_msgs.
// Without this include the plugin builds fine and only fails at runtime,
// with an undefined-symbol error the first time a transform is needed.
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.hpp"

namespace predictive_nav_costmap
{

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::MAX_NON_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

void PredictedObstacleLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"PredictedObstacleLayer: failed to lock lifecycle node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("tracked_objects_topic", rclcpp::ParameterValue(std::string("/tracked_objects")));
  declareParameter("track_timeout", rclcpp::ParameterValue(1.0));
  declareParameter("track_array_timeout", rclcpp::ParameterValue(1.0));
  declareParameter("max_prediction_horizon", rclcpp::ParameterValue(3.0));
  declareParameter("sigma_level", rclcpp::ParameterValue(2.0));
  declareParameter("temporal_decay", rclcpp::ParameterValue(0.7));
  declareParameter("max_cost", rclcpp::ParameterValue(200));
  declareParameter("min_cost", rclcpp::ParameterValue(0));
  declareParameter("max_influence_radius", rclcpp::ParameterValue(3.0));
  declareParameter("min_position_variance", rclcpp::ParameterValue(0.0025));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.2));
  declareParameter("publish_debug_costmap", rclcpp::ParameterValue(false));
  declareParameter("stats_log_period", rclcpp::ParameterValue(5.0));
  // Stage-4G4: optional second representation. Default keeps the validated
  // Stage-4D CV behaviour exactly.
  declareParameter("prediction_mode", rclcpp::ParameterValue(std::string("cv_covariance")));
  declareParameter("max_observation_age", rclcpp::ParameterValue(1.0));

  getParameters();

  global_frame_ = layered_costmap_->getGlobalFrameID();
  rolling_window_ = layered_costmap_->isRolling();

  // NO_INFORMATION is this layer's "contributes nothing" value: under
  // updateWithMax() a NO_INFORMATION cell leaves the master grid untouched,
  // so an empty predictive layer can never erase static/obstacle/inflation
  // costs.
  setDefaultValue(NO_INFORMATION);
  matchSize();
  current_ = true;

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;
  tracks_sub_ = node->create_subscription<predictive_nav_msgs::msg::TrackedObjectArray>(
    tracked_objects_topic_, rclcpp::QoS(10),
    std::bind(&PredictedObstacleLayer::tracksCallback, this, std::placeholders::_1),
    sub_options);

  if (publish_debug_costmap_) {
    // Topic names may not contain '.', so the layer name is used as a
    // namespace segment rather than via getFullName().
    debug_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(
      name_ + "/debug_costmap", rclcpp::QoS(1).transient_local());
    debug_pub_->on_activate();
  }

  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&PredictedObstacleLayer::dynamicParametersCallback, this, std::placeholders::_1));

  last_stats_log_ = clock_->now();
  last_update_stamp_ = rclcpp::Time(0, 0, clock_->get_clock_type());

  RCLCPP_INFO(
    logger_,
    "PredictedObstacleLayer(%s): topic=%s frame=%s rolling=%d mode=%s sigma=%.2f "
    "horizon=%.2fs decay=%.2f cost=[%d,%d] influence_radius=%.2fm max_obs_age=%.2fs",
    name_.c_str(), tracked_objects_topic_.c_str(), global_frame_.c_str(),
    static_cast<int>(rolling_window_), prediction_mode_.c_str(),
    sigma_level_, max_prediction_horizon_, temporal_decay_,
    min_cost_, max_cost_, max_influence_radius_, max_observation_age_);
}

void PredictedObstacleLayer::getParameters()
{
  auto node = node_.lock();
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("tracked_objects_topic"), tracked_objects_topic_);
  node->get_parameter(getFullName("track_timeout"), track_timeout_);
  node->get_parameter(getFullName("track_array_timeout"), track_array_timeout_);
  node->get_parameter(getFullName("max_prediction_horizon"), max_prediction_horizon_);
  node->get_parameter(getFullName("sigma_level"), sigma_level_);
  node->get_parameter(getFullName("temporal_decay"), temporal_decay_);
  node->get_parameter(getFullName("max_cost"), max_cost_);
  node->get_parameter(getFullName("min_cost"), min_cost_);
  node->get_parameter(getFullName("max_influence_radius"), max_influence_radius_);
  node->get_parameter(getFullName("min_position_variance"), min_position_variance_);
  node->get_parameter(getFullName("transform_tolerance"), transform_tolerance_);
  node->get_parameter(getFullName("publish_debug_costmap"), publish_debug_costmap_);
  node->get_parameter(getFullName("stats_log_period"), stats_log_period_);
  node->get_parameter(getFullName("prediction_mode"), prediction_mode_);
  node->get_parameter(getFullName("max_observation_age"), max_observation_age_);
  if (prediction_mode_ != "cv_covariance" && prediction_mode_ != "reachability") {
    RCLCPP_WARN(
      logger_,
      "PredictedObstacleLayer(%s): unknown prediction_mode '%s'; falling back to "
      "'cv_covariance' (the validated Stage-4D behaviour)",
      name_.c_str(), prediction_mode_.c_str());
    prediction_mode_ = "cv_covariance";
  }
  reachability_mode_ = (prediction_mode_ == "reachability");

  // This layer must never assert a hard obstacle: MAX_NON_OBSTACLE (252) is
  // the highest value that is not INSCRIBED_INFLATED_OBSTACLE/LETHAL, so a
  // *predicted* (not sensed) occupancy can never be mistaken for a measured
  // lethal obstacle.
  if (max_cost_ > static_cast<int>(MAX_NON_OBSTACLE)) {
    RCLCPP_WARN(
      logger_, "PredictedObstacleLayer(%s): max_cost %d exceeds MAX_NON_OBSTACLE (%d); clamping",
      name_.c_str(), max_cost_, static_cast<int>(MAX_NON_OBSTACLE));
    max_cost_ = static_cast<int>(MAX_NON_OBSTACLE);
  }
  max_cost_ = std::clamp(max_cost_, 0, static_cast<int>(MAX_NON_OBSTACLE));
  min_cost_ = std::clamp(min_cost_, 0, max_cost_);
  sigma_level_ = std::max(sigma_level_, 1e-3);
  min_position_variance_ = std::max(min_position_variance_, 1e-9);
}

void PredictedObstacleLayer::matchSize()
{
  nav2_costmap_2d::CostmapLayer::matchSize();
}

void PredictedObstacleLayer::activate()
{
  if (debug_pub_) {
    debug_pub_->on_activate();
  }
}

void PredictedObstacleLayer::deactivate()
{
  if (debug_pub_) {
    debug_pub_->on_deactivate();
  }
}

void PredictedObstacleLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_tracks_.reset();
  resetMaps();
  // Leave previous_written_ intact: the next updateBounds() still has to
  // expand the master's update window over whatever this layer last wrote,
  // otherwise those master cells would keep a stale predictive cost.
  current_ = true;
}

void PredictedObstacleLayer::tracksCallback(
  predictive_nav_msgs::msg::TrackedObjectArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_tracks_ = std::move(msg);
}

void PredictedObstacleLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (rolling_window_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2.0, robot_y - getSizeInMetersY() / 2.0);
  }

  // Always expand the update window over the region written last cycle and
  // wipe this layer's own grid. The LayeredCostmap resets the master within
  // the union of all layers' bounds before re-applying every layer, so
  // including the previous region here is what actually lets stale
  // predictive cost disappear -- without ever touching cells this layer did
  // not write.
  if (previous_written_.valid) {
    touch(previous_written_.min_x, previous_written_.min_y, min_x, min_y, max_x, max_y);
    touch(previous_written_.max_x, previous_written_.max_y, min_x, min_y, max_x, max_y);
  }
  resetMaps();
  previous_written_.reset();
  cells_written_last_ = 0;

  const rclcpp::Time now = clock_->now();
  if (last_update_stamp_.nanoseconds() > 0) {
    const double period = (now - last_update_stamp_).seconds();
    if (period > 0.0 && period < 10.0) {
      update_period_total_s_ += period;
      ++update_period_samples_;
    }
  }
  last_update_stamp_ = now;
  ++update_count_;

  current_ = true;
  if (!enabled_) {
    return;
  }

  predictive_nav_msgs::msg::TrackedObjectArray::SharedPtr tracks;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    tracks = latest_tracks_;
  }
  if (!tracks || tracks->tracks.empty()) {
    return;
  }

  // Reject a stale array outright: if the tracker stopped publishing (node
  // died, sensor lost) the last message must not keep painting the costmap.
  const double array_age = (now - rclcpp::Time(tracks->header.stamp)).seconds();
  if (array_age > track_array_timeout_) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 5000,
      "PredictedObstacleLayer(%s): /tracked_objects is %.2fs old (> %.2fs); "
      "contributing no predictive cost", name_.c_str(), array_age, track_array_timeout_);
    return;
  }

  // Predictions are published in the tracker's frame (map); the costmap may
  // run in another frame (the local costmap is a rolling window in odom).
  // Both the mean and the covariance have to be taken into that frame:
  // mu' = R*mu + t and Sigma' = R*Sigma*R^T.
  Eigen::Matrix2d rotation = Eigen::Matrix2d::Identity();
  double translation_x = 0.0;
  double translation_y = 0.0;

  if (tracks->header.frame_id != global_frame_) {
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        global_frame_, tracks->header.frame_id, rclcpp::Time(tracks->header.stamp),
        rclcpp::Duration::from_seconds(transform_tolerance_));
    } catch (const tf2::TransformException &) {
      // Fall back to the latest available transform. map->odom is published
      // by AMCL and drifts slowly, so using the newest one rather than
      // dropping the cycle is the less surprising behaviour; a hard failure
      // is still handled below.
      try {
        transform = tf_->lookupTransform(
          global_frame_, tracks->header.frame_id,
          rclcpp::Time(0, 0, clock_->get_clock_type()));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000,
          "PredictedObstacleLayer(%s): TF %s->%s unavailable (%s); contributing no "
          "predictive cost this cycle", name_.c_str(), tracks->header.frame_id.c_str(),
          global_frame_.c_str(), ex.what());
        return;
      }
    }
    const double yaw = tf2::getYaw(transform.transform.rotation);
    rotation << std::cos(yaw), -std::sin(yaw), std::sin(yaw), std::cos(yaw);
    translation_x = transform.transform.translation.x;
    translation_y = transform.transform.translation.y;
  }

  // Stage-4G4 mode switch. The CV branch is the unmodified Stage-4D call.
  previous_written_ = reachability_mode_ ?
    rasterizeReachability(*tracks, rotation, translation_x, translation_y, now) :
    rasterizePredictions(*tracks, rotation, translation_x, translation_y, now);

  if (previous_written_.valid) {
    touch(previous_written_.min_x, previous_written_.min_y, min_x, min_y, max_x, max_y);
    touch(previous_written_.max_x, previous_written_.max_y, min_x, min_y, max_x, max_y);
  }
}

PredictedObstacleLayer::WorldBounds PredictedObstacleLayer::rasterizePredictions(
  const predictive_nav_msgs::msg::TrackedObjectArray & tracks,
  const Eigen::Matrix2d & rotation, double translation_x, double translation_y,
  const rclcpp::Time & now)
{
  WorldBounds written;

  for (const auto & track : tracks.tracks) {
    if (!track.kalman_initialized || track.predictions.empty()) {
      continue;
    }
    const double track_age = (now - rclcpp::Time(track.stamp)).seconds();
    if (track_age > track_timeout_) {
      continue;
    }

    for (const auto & prediction : track.predictions) {
      if (prediction.time_from_now < 0.0 || prediction.time_from_now > max_prediction_horizon_) {
        continue;
      }

      const auto & cov = prediction.position_covariance;
      if (!std::isfinite(prediction.position.x) || !std::isfinite(prediction.position.y) ||
        !std::isfinite(cov[0]) || !std::isfinite(cov[1]) || !std::isfinite(cov[2]) ||
        !std::isfinite(cov[3]))
      {
        continue;
      }

      // Use the symmetric part; a covariance must be symmetric and any
      // asymmetry here would only be numerical noise from transport.
      Eigen::Matrix2d sigma;
      const double cxy = 0.5 * (cov[1] + cov[2]);
      sigma << cov[0], cxy, cxy, cov[3];

      if (sigma(0, 0) <= 0.0 || sigma(1, 1) <= 0.0) {
        continue;
      }

      const Eigen::Vector2d mean_map(prediction.position.x, prediction.position.y);
      const Eigen::Vector2d mean = rotation * mean_map +
        Eigen::Vector2d(translation_x, translation_y);
      const Eigen::Matrix2d sigma_costmap = rotation * sigma * rotation.transpose();

      rasterizeEllipse(
        mean.x(), mean.y(), sigma_costmap, prediction.time_from_now, written);
    }
  }
  return written;
}

// ---------------------------------------------------------------------------
// Stage-4G4: deterministic reachable-set rasterization (reachability mode)
// ---------------------------------------------------------------------------
//
// COST SEMANTICS, stated explicitly because they differ from CV mode.
//   CV mode:            cost = min_cost + exp(-d_maha^2/2) * exp(-decay*t) * span
//                       -> a PROBABILITY DENSITY RATIO, peaked at the mean.
//   Reachability mode:  cost = min_cost + 1 * exp(-decay*t) * span inside the
//                       region, nothing outside
//                       -> SET MEMBERSHIP. A bounded-motion reachable set has
//                       no internal density: every point in it is equally
//                       reachable, so painting a peak would invent
//                       information the model does not contain.
// The temporal decay is retained in both modes and means the same thing in
// both: a policy statement that the layer commits less cost to more distant
// futures. It is NOT a probability, and it never reshapes the region itself.
//
// Both modes still respect max_cost < INSCRIBED_INFLATED_OBSTACLE, so a
// prediction can never be promoted to a sensed lethal obstacle, and both
// combine with updateWithMax().
PredictedObstacleLayer::WorldBounds PredictedObstacleLayer::rasterizeReachability(
  const predictive_nav_msgs::msg::TrackedObjectArray & tracks,
  const Eigen::Matrix2d & rotation, double translation_x, double translation_y,
  const rclcpp::Time & now)
{
  WorldBounds written;

  for (const auto & track : tracks.tracks) {
    if (!track.kalman_initialized || track.reachability_predictions.empty()) {
      continue;
    }
    // Same freshness rule as CV mode, so mode choice never changes which
    // tracks are eligible -- only how an eligible track is painted.
    const double track_age = (now - rclcpp::Time(track.stamp)).seconds();
    if (track_age > track_timeout_) {
      continue;
    }

    for (const auto & region : track.reachability_predictions) {
      // The producer already marks a region invalid past its own
      // max_observation_age; this layer additionally enforces its own bound so
      // a mis-parameterised tracker cannot make the costmap paint stale sets.
      if (!region.valid) {
        continue;
      }
      if (region.time_from_now < 0.0 || region.time_from_now > max_prediction_horizon_) {
        continue;
      }
      if (max_observation_age_ > 0.0 && region.observation_age > max_observation_age_) {
        continue;
      }
      if (!std::isfinite(region.position.x) || !std::isfinite(region.position.y) ||
        !std::isfinite(region.semi_major) || !std::isfinite(region.semi_minor) ||
        !std::isfinite(region.sigma_yaw))
      {
        continue;
      }
      if (region.semi_major <= 0.0 || region.semi_minor <= 0.0) {
        continue;
      }

      const Eigen::Vector2d center_map(region.position.x, region.position.y);
      const Eigen::Vector2d center = rotation * center_map +
        Eigen::Vector2d(translation_x, translation_y);
      // The region is an ellipse in the map frame; the map->costmap transform
      // here is a rotation, so only its orientation needs rotating.
      const double yaw_costmap =
        region.sigma_yaw + std::atan2(rotation(1, 0), rotation(0, 0));

      rasterizeReachableSet(
        center.x(), center.y(), region.semi_major, region.semi_minor, yaw_costmap,
        region.time_from_now, written);
    }
  }
  return written;
}

void PredictedObstacleLayer::rasterizeReachableSet(
  double center_x, double center_y, double semi_major, double semi_minor, double yaw,
  double time_from_now, WorldBounds & written)
{
  // Same costmap-policy bound as CV mode: this is not a statement about the
  // model, only about how far this layer will paint per sample.
  if (max_influence_radius_ > 0.0) {
    semi_major = std::min(semi_major, max_influence_radius_);
    semi_minor = std::min(semi_minor, max_influence_radius_);
  }

  const double c = std::cos(yaw);
  const double sn = std::sin(yaw);
  // Axis-aligned bounding box of a rotated ellipse.
  const double half_x = std::hypot(semi_major * c, semi_minor * sn);
  const double half_y = std::hypot(semi_major * sn, semi_minor * c);

  const double temporal_weight = std::exp(-temporal_decay_ * time_from_now);
  const double cost_span = static_cast<double>(max_cost_ - min_cost_);
  const int cost_value = static_cast<int>(
    std::lround(static_cast<double>(min_cost_) + temporal_weight * cost_span));
  if (cost_value <= 0) {
    return;
  }
  const unsigned char cost = static_cast<unsigned char>(
    std::clamp(cost_value, 0, static_cast<int>(MAX_NON_OBSTACLE)));

  int min_i = 0;
  int min_j = 0;
  int max_i = 0;
  int max_j = 0;
  worldToMapEnforceBounds(center_x - half_x, center_y - half_y, min_i, min_j);
  worldToMapEnforceBounds(center_x + half_x, center_y + half_y, max_i, max_j);

  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx = 0.0;
      double wy = 0.0;
      mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);

      const double dx = wx - center_x;
      const double dy = wy - center_y;
      const double u = (dx * c + dy * sn) / semi_major;
      const double v = (-dx * sn + dy * c) / semi_minor;
      if (u * u + v * v > 1.0) {
        continue;   // outside the reachable set: this layer says nothing here
      }

      const unsigned int mx = static_cast<unsigned int>(i);
      const unsigned int my = static_cast<unsigned int>(j);
      const unsigned char existing = getCost(mx, my);
      // Overlapping reachable sets (different horizons, different tracks) are
      // alternative future occupancies: keep the most conservative one, the
      // same max-composition rule CV mode uses.
      if (existing == NO_INFORMATION || cost > existing) {
        setCost(mx, my, cost);
      }
      if (existing == NO_INFORMATION) {
        ++cells_written_last_;
      }
      written.include(wx, wy);
    }
  }
}

void PredictedObstacleLayer::rasterizeEllipse(
  double mean_x, double mean_y, const Eigen::Matrix2d & covariance,
  double time_from_now, WorldBounds & written)
{
  // Eigen-decompose the symmetric covariance and floor its eigenvalues.
  // The floor is a numerical guard only: it keeps a (near-)singular
  // covariance invertible and guarantees an almost-stationary, very
  // confident prediction still rasterizes at least one cell rather than
  // degenerating to an empty or NaN region.
  const Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return;
  }
  Eigen::Vector2d eigenvalues = solver.eigenvalues();
  if (!eigenvalues.allFinite()) {
    return;
  }
  eigenvalues = eigenvalues.cwiseMax(min_position_variance_);
  const Eigen::Matrix2d eigenvectors = solver.eigenvectors();
  const Eigen::Matrix2d sigma =
    eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
  const Eigen::Matrix2d sigma_inverse = sigma.inverse();
  if (!sigma_inverse.allFinite()) {
    return;
  }

  // Axis-aligned bounding box of the sigma-level ellipse
  // {p : (p-mu)^T Sigma^-1 (p-mu) <= sigma_level^2} is exactly
  // mu +/- sigma_level * sqrt(diag(Sigma)).
  double half_x = sigma_level_ * std::sqrt(sigma(0, 0));
  double half_y = sigma_level_ * std::sqrt(sigma(1, 1));

  // Optional costmap-policy/performance bound. This is NOT a statistical
  // statement about the prediction: the underlying covariance is unchanged
  // and is still published in full on /tracked_objects. It only bounds how
  // far this layer is willing to paint per prediction, so a very
  // long-horizon, very uncertain prediction cannot cost an unbounded number
  // of cells. Set <= 0 to disable.
  if (max_influence_radius_ > 0.0) {
    half_x = std::min(half_x, max_influence_radius_);
    half_y = std::min(half_y, max_influence_radius_);
  }

  const double temporal_weight = std::exp(-temporal_decay_ * time_from_now);
  const double cost_span = static_cast<double>(max_cost_ - min_cost_);
  const double sigma_level_squared = sigma_level_ * sigma_level_;

  int min_i = 0;
  int min_j = 0;
  int max_i = 0;
  int max_j = 0;
  worldToMapEnforceBounds(mean_x - half_x, mean_y - half_y, min_i, min_j);
  worldToMapEnforceBounds(mean_x + half_x, mean_y + half_y, max_i, max_j);

  for (int j = min_j; j <= max_j; ++j) {
    for (int i = min_i; i <= max_i; ++i) {
      double wx = 0.0;
      double wy = 0.0;
      mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);

      const Eigen::Vector2d delta(wx - mean_x, wy - mean_y);
      const double mahalanobis_squared = delta.transpose() * sigma_inverse * delta;
      if (!std::isfinite(mahalanobis_squared) || mahalanobis_squared > sigma_level_squared) {
        continue;
      }

      // Spatial confidence is the Gaussian likelihood of this cell relative
      // to the predicted mean, p(x)/p(mu) = exp(-d^2/2): 1 at the mean and
      // falling off to exp(-sigma_level^2/2) at the ellipse boundary. Cost
      // is therefore highest where the obstacle is most likely to be.
      const double spatial_confidence = std::exp(-0.5 * mahalanobis_squared);
      const double weight = spatial_confidence * temporal_weight;
      const int cost_value = static_cast<int>(
        std::lround(static_cast<double>(min_cost_) + weight * cost_span));
      if (cost_value <= 0) {
        continue;
      }

      const unsigned char cost = static_cast<unsigned char>(
        std::clamp(cost_value, 0, static_cast<int>(MAX_NON_OBSTACLE)));
      const unsigned int mx = static_cast<unsigned int>(i);
      const unsigned int my = static_cast<unsigned int>(j);
      const unsigned char existing = getCost(mx, my);
      // Overlapping predictions (different horizons, different tracks) are
      // alternative future occupancies: keep the most conservative one.
      if (existing == NO_INFORMATION || cost > existing) {
        setCost(mx, my, cost);
      }
      if (existing == NO_INFORMATION) {
        ++cells_written_last_;
      }
      written.include(wx, wy);

      const double extent = std::hypot(wx - mean_x, wy - mean_y);
      max_extent_radius_ = std::max(max_extent_radius_, extent);
    }
  }
}

void PredictedObstacleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  const rclcpp::Time now = clock_->now();

  // Publish this layer's own contribution every cycle, including while
  // disabled -- updateBounds() has already wiped the grid, so subscribers
  // see an empty grid rather than being left with the last latched
  // non-empty one. Without this, "the contribution went away" is not
  // observable from outside.
  if (publish_debug_costmap_ && debug_pub_) {
    publishDebugCostmap(now);
  }

  if (!enabled_) {
    return;
  }

  const auto start = std::chrono::steady_clock::now();

  // Max-combination: a predictive cell can only ever raise a master cell,
  // never lower it, so static obstacles, live obstacle returns and inflation
  // can never be erased or weakened by this layer.
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);

  const double elapsed_us = std::chrono::duration<double, std::micro>(
    std::chrono::steady_clock::now() - start).count();
  update_costs_total_us_ += elapsed_us;
  update_costs_max_us_ = std::max(update_costs_max_us_, elapsed_us);
  ++update_costs_calls_;

  if (stats_log_period_ > 0.0 && (now - last_stats_log_).seconds() >= stats_log_period_) {
    const double mean_us = (update_costs_calls_ > 0) ?
      update_costs_total_us_ / static_cast<double>(update_costs_calls_) : 0.0;
    const double mean_rate = (update_period_samples_ > 0 && update_period_total_s_ > 0.0) ?
      static_cast<double>(update_period_samples_) / update_period_total_s_ : 0.0;
    RCLCPP_INFO(
      logger_,
      "PredictedObstacleLayer(%s) stats: updates=%lu rate=%.2fHz updateCosts mean=%.1fus "
      "max=%.1fus cells_written_last=%lu max_extent=%.2fm",
      name_.c_str(), static_cast<unsigned long>(update_count_), mean_rate, mean_us,
      update_costs_max_us_, static_cast<unsigned long>(cells_written_last_), max_extent_radius_);
    last_stats_log_ = now;
  }
}

void PredictedObstacleLayer::publishDebugCostmap(const rclcpp::Time & stamp)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = stamp;
  grid.header.frame_id = global_frame_;
  grid.info.resolution = static_cast<float>(getResolution());
  grid.info.width = getSizeInCellsX();
  grid.info.height = getSizeInCellsY();
  grid.info.origin.position.x = getOriginX();
  grid.info.origin.position.y = getOriginY();
  grid.info.origin.orientation.w = 1.0;
  grid.data.resize(static_cast<size_t>(grid.info.width) * grid.info.height);

  const unsigned char * chars = getCharMap();
  for (size_t i = 0; i < grid.data.size(); ++i) {
    const unsigned char cost = chars[i];
    grid.data[i] = (cost == NO_INFORMATION) ?
      static_cast<int8_t>(-1) :
      static_cast<int8_t>(std::lround(static_cast<double>(cost) * 100.0 / 252.0));
  }
  debug_pub_->publish(grid);
}

rcl_interfaces::msg::SetParametersResult PredictedObstacleLayer::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  // Runs on the node's executor thread. LayeredCostmap::updateMap() holds
  // the master costmap lock for the whole update, so taking it here gives
  // mutual exclusion with updateBounds()/updateCosts().
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(
    *layered_costmap_->getCostmap()->getMutex());

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const std::string & full_name = parameter.get_name();
    if (full_name == getFullName("enabled")) {
      enabled_ = parameter.as_bool();
    } else if (full_name == getFullName("track_timeout")) {
      track_timeout_ = parameter.as_double();
    } else if (full_name == getFullName("track_array_timeout")) {
      track_array_timeout_ = parameter.as_double();
    } else if (full_name == getFullName("max_prediction_horizon")) {
      max_prediction_horizon_ = parameter.as_double();
    } else if (full_name == getFullName("sigma_level")) {
      sigma_level_ = std::max(parameter.as_double(), 1e-3);
    } else if (full_name == getFullName("temporal_decay")) {
      temporal_decay_ = parameter.as_double();
    } else if (full_name == getFullName("max_cost")) {
      max_cost_ = std::clamp(
        static_cast<int>(parameter.as_int()), 0, static_cast<int>(MAX_NON_OBSTACLE));
      min_cost_ = std::min(min_cost_, max_cost_);
    } else if (full_name == getFullName("min_cost")) {
      min_cost_ = std::clamp(static_cast<int>(parameter.as_int()), 0, max_cost_);
    } else if (full_name == getFullName("max_influence_radius")) {
      max_influence_radius_ = parameter.as_double();
    } else if (full_name == getFullName("max_observation_age")) {
      max_observation_age_ = parameter.as_double();
    } else if (full_name == getFullName("prediction_mode")) {
      // Runtime-switchable so one Gazebo launch can serve both A/B arms, the
      // same way Stage-4E switched `enabled`. An unknown value falls back to
      // the validated CV path rather than silently painting nothing.
      const std::string requested = parameter.as_string();
      if (requested == "cv_covariance" || requested == "reachability") {
        prediction_mode_ = requested;
        reachability_mode_ = (requested == "reachability");
      } else {
        result.successful = false;
        result.reason = "prediction_mode must be 'cv_covariance' or 'reachability'";
      }
    }
  }
  return result;
}

}  // namespace predictive_nav_costmap

PLUGINLIB_EXPORT_CLASS(predictive_nav_costmap::PredictedObstacleLayer, nav2_costmap_2d::Layer)
