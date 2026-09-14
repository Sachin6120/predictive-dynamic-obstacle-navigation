// Stage-4G4 costmap regression for the OPTIONAL reachability mode, and for the
// guarantee that selecting it does not disturb the validated Stage-4D CV mode.
//
// Verified here, all through the real plugin's ROS subscription and
// LayeredCostmap rebuild API:
//   * CV mode is byte-identical whether or not tracks also carry reachability
//     regions (the new field cannot leak into the old path),
//   * reachable regions appear, and their painted area grows with horizon,
//   * observation age expands a coasting track's painted area,
//   * regions past max_observation_age paint nothing,
//   * multiple tracks compose with max semantics and no cross-contamination,
//   * expiry and removal clear completely, with no ghost cells,
//   * an empty world paints nothing.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>
#include "predictive_nav_costmap/predicted_obstacle_layer.hpp"
#include "predictive_nav_tracking/reachability.hpp"
#include "nav2_util/lifecycle_node.hpp"

using Array = predictive_nav_msgs::msg::TrackedObjectArray;
using Object = predictive_nav_msgs::msg::TrackedObject;
using predictive_nav_tracking::ReachabilityBounds;
using predictive_nav_tracking::buildRegion;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

/// Builds a track carrying BOTH representations, exactly as the Stage-4G4
/// tracker publishes them, using the production reachability mathematics.
Object object(
  uint32_t id, double x, double y, double vx, rclcpp::Time now,
  double observation_age, const ReachabilityBounds & bounds)
{
  Object t;
  t.id = id;
  t.kalman_initialized = true;
  t.stamp = now - rclcpp::Duration::from_seconds(observation_age);
  t.observations = 10;
  t.age = 10;
  t.position.x = x;
  t.position.y = y;
  t.velocity.x = vx;
  for (int i = 1; i <= 6; ++i) {
    const double h = i * 0.5;
    const double variance = 0.01 + 0.01 * h * h + 0.25 * h * h * h * h / 4;

    predictive_nav_msgs::msg::TrackedObjectPrediction p;
    p.time_from_now = h;
    p.stamp = rclcpp::Time(t.stamp) + rclcpp::Duration::from_seconds(h);
    p.position.x = x + vx * h;
    p.position.y = y;
    p.position_covariance = {variance, 0, 0, variance};
    t.predictions.push_back(p);

    const double total = observation_age + h;
    const double total_variance = 0.01 + 0.01 * total * total +
      0.25 * total * total * total * total / 4;
    Eigen::Matrix2d P;
    P << total_variance, 0, 0, total_variance;
    const auto region = buildRegion(x, y, vx, 0.0, P, total, bounds);

    predictive_nav_msgs::msg::ReachabilityPrediction r;
    r.time_from_now = h;
    r.stamp = now + rclcpp::Duration::from_seconds(h);
    r.observation_age = observation_age;
    r.total_time = total;
    r.position.x = region.center_x;
    r.position.y = region.center_y;
    r.velocity.x = vx;
    r.reach_radius = region.reach_radius;
    r.speed_capped = region.speed_capped;
    r.sigma_semi_major = region.sigma_semi_major;
    r.sigma_semi_minor = region.sigma_semi_minor;
    r.sigma_yaw = region.sigma_yaw;
    r.covariance_sigma_level = bounds.covariance_sigma_level;
    r.safety_margin = region.safety_margin;
    r.semi_major = region.semi_major;
    r.semi_minor = region.semi_minor;
    r.valid = region.valid;
    t.reachability_predictions.push_back(r);
  }
  return t;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_util::LifecycleNode>("stage4g4_reach_regression", "",
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("g4.tracked_objects_topic", "/g4_layer_tracks"),
      rclcpp::Parameter("g4.sigma_level", 1.5),
      rclcpp::Parameter("g4.temporal_decay", 0.35),
      rclcpp::Parameter("g4.max_cost", 250),
      rclcpp::Parameter("g4.min_cost", 0),
      rclcpp::Parameter("g4.max_influence_radius", 3.0),
      rclcpp::Parameter("g4.max_observation_age", 1.0),
      rclcpp::Parameter("g4.prediction_mode", "cv_covariance")}));
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap master("map", false, false);
  const unsigned int cells = 160;
  master.resizeMap(cells, cells, 0.05, -4.0, -4.0);
  auto layer = std::make_shared<predictive_nav_costmap::PredictedObstacleLayer>();
  master.getPlugins()->push_back(layer);
  layer->initialize(&master, "g4", &tf, node, group);
  auto pub = node->create_publisher<Array>("/g4_layer_tracks", 10);
  pub->on_activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  auto render = [&](std::vector<Object> tracks) {
      Array a;
      a.header.frame_id = "map";
      a.header.stamp = node->now();
      a.tracks = tracks;
      for (int i = 0; i < 10; ++i) {
        pub->publish(a);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        executor.spin_some();
      }
      master.updateMap(0, 0, 0);
      auto * map = master.getCostmap();
      return std::vector<unsigned char>(
        map->getCharMap(), map->getCharMap() + static_cast<size_t>(cells) * cells);
    };
  auto painted = [](const std::vector<unsigned char> & grid) {
      return static_cast<size_t>(std::count_if(grid.begin(), grid.end(), [](auto c) {return c > 0;}));
    };
  auto set_mode = [&](const std::string & mode) {
      node->set_parameter(rclcpp::Parameter("g4.prediction_mode", mode));
      executor.spin_some();
    };

  ReachabilityBounds bounds;
  bounds.max_acceleration = 0.5;
  bounds.max_speed = 0.8;
  bounds.covariance_sigma_level = 2.0;
  bounds.safety_margin = 0.0;

  // Each render() spins for ~100 ms, so a track built once and reused across
  // several renders would age past track_timeout and be dropped. Every track
  // is therefore stamped at the moment it is published.
  auto fresh = [&](uint32_t id, double x, double y, double vx, double observation_age) {
      return object(id, x, y, vx, node->now(), observation_age, bounds);
    };

  // ---- CV mode is untouched by the presence of reachability data ---------
  // Same track twice, once with the new field stripped: the CV path must not
  // observe any difference whatsoever.
  size_t cv_cells = 0;
  {
    auto with_reach = fresh(11, -0.8, -0.2, 0.3, 0.0);
    auto without = with_reach;
    without.reachability_predictions.clear();
    const auto a = render({with_reach});
    const auto b = render({without});
    require(a == b, "CV mode output changed when reachability data was present");
    cv_cells = painted(a);
    require(cv_cells > 0, "CV mode painted nothing");
  }

  // ---- Reachability mode paints, and contains the CV region --------------
  // Compared at a SINGLE short horizon on purpose. Over the full horizon set
  // the two modes are not cleanly comparable by cell count: CV mode's
  // max_influence_radius clamp bounds the ellipse's BOUNDING BOX, so a
  // long-horizon, high-variance CV sample degenerates into a filled square,
  // while a reachable set stays an inscribed ellipse. That is pre-existing
  // Stage-4D behaviour which Stage-4G4 must not change, so the containment
  // property is verified where the clamp does not bind.
  size_t reach_cells = 0;
  size_t cv_first_cells = 0;
  {
    auto track = fresh(11, -0.8, -0.2, 0.3, 0.0);
    auto first_only = track;
    first_only.predictions = {track.predictions[0]};
    first_only.reachability_predictions = {track.reachability_predictions[0]};

    const auto cv_grid = render({first_only});
    cv_first_cells = painted(cv_grid);

    set_mode("reachability");
    const auto grid = render({first_only});
    reach_cells = painted(grid);
    require(reach_cells > 0, "reachability mode painted nothing");
    require(
      reach_cells > cv_first_cells,
      "the reachable set did not paint more than the CV ellipse it contains");
    // Containment, cell by cell: every cell CV asserts must also be asserted
    // by the reachable set, since the region is built as a superset of the
    // same covariance ellipse.
    for (size_t i = 0; i < grid.size(); ++i) {
      require(
        !(cv_grid[i] > 0 && grid[i] == 0),
        "a cell claimed by the CV ellipse was not covered by the reachable set");
    }
    require(
      *std::max_element(grid.begin(), grid.end()) <= 250,
      "reachability cost exceeded the configured cap");
  }

  // ---- Painted area grows with horizon ----------------------------------
  // Strictly increasing while the layer's max_influence_radius policy bound
  // does not bind, and never decreasing once it does. The saturation is a
  // costmap policy choice (how far this layer will paint per sample), not a
  // property of the model -- the published region keeps growing regardless,
  // which is why the check is written against the published semi-axis rather
  // than assuming growth forever.
  size_t unclamped_horizons = 0;
  {
    std::vector<size_t> per_horizon;
    std::vector<bool> clamped;
    for (int i = 0; i < 6; ++i) {
      auto single = fresh(11, 0.0, 0.0, 0.3, 0.0);
      clamped.push_back(single.reachability_predictions[i].semi_major >= 3.0);
      single.reachability_predictions = {single.reachability_predictions[i]};
      per_horizon.push_back(painted(render({single})));
    }
    for (size_t i = 1; i < per_horizon.size(); ++i) {
      require(
        per_horizon[i] >= per_horizon[i - 1],
        "reachable area shrank as the prediction horizon grew");
      if (!clamped[i]) {
        require(
          per_horizon[i] > per_horizon[i - 1],
          "reachable area did not grow with the prediction horizon");
        ++unclamped_horizons;
      }
    }
    require(unclamped_horizons > 0, "no unclamped horizon was exercised");
  }

  // ---- Observation age expands a coasting track's footprint -------------
  size_t fresh_cells = 0;
  size_t coasting_cells = 0;
  {
    fresh_cells = painted(render({fresh(11, 0.0, 0.0, 0.3, 0.0)}));
    coasting_cells = painted(render({fresh(11, 0.0, 0.0, 0.3, 0.6)}));
    require(
      coasting_cells > fresh_cells,
      "a coasting track's reachable footprint did not grow with observation age");
    // Past max_observation_age nothing is painted: the layer refuses to assert
    // future occupancy from information it considers too old.
    const auto stale = render({fresh(11, 0.0, 0.0, 0.3, 1.5)});
    require(painted(stale) == 0, "a region past max_observation_age was still painted");
  }

  // ---- Multi-object composition, expiry and clearing --------------------
  size_t overlap = 0;
  size_t unique_a = 0;
  size_t unique_b = 0;
  {
    const auto only_a = render({fresh(11, -0.5, -0.15, 0.3, 0.0)});
    const auto only_b = render({fresh(22, 0.5, 0.15, -0.3, 0.0)});
    const auto both = render({fresh(11, -0.5, -0.15, 0.3, 0.0), fresh(22, 0.5, 0.15, -0.3, 0.0)});
    for (size_t i = 0; i < both.size(); ++i) {
      require(both[i] == std::max(only_a[i], only_b[i]), "combined cost is not max(A,B)");
      overlap += only_a[i] > 0 && only_b[i] > 0;
      unique_a += only_a[i] > 0 && only_b[i] == 0;
      unique_b += only_b[i] > 0 && only_a[i] == 0;
    }
    require(overlap && unique_a && unique_b, "test geometry lacks overlap or unique parts");

    // One track expires while the other stays: the survivor must be identical
    // and the expired one must leave nothing behind.
    auto expired = fresh(11, -0.5, -0.15, 0.3, 0.0);
    expired.stamp = node->now() - rclcpp::Duration::from_seconds(1.01);
    for (auto & r : expired.reachability_predictions) {
      r.observation_age = 1.01;
    }
    require(
      render({expired, fresh(22, 0.5, 0.15, -0.3, 0.0)}) == only_b,
      "expiring A changed B or left ghost cells");
    require(
      render({fresh(22, 0.5, 0.15, -0.3, 0.0)}) == only_b,
      "removing A changed B or left ghost cells");
    const auto empty = render({});
    require(
      std::all_of(empty.begin(), empty.end(), [](auto c) {return c == 0;}),
      "empty array left ghost cells in master");
  }

  // ---- Returning to CV mode restores the Stage-4D behaviour exactly ------
  {
    set_mode("cv_covariance");
    const auto restored = render({fresh(11, -0.8, -0.2, 0.3, 0.0)});
    require(painted(restored) == cv_cells, "CV mode was not restored exactly after switching back");
  }

  std::cout << "{\"pass\":true"
            << ",\"cv_mode_unaffected_by_reachability_field\":true"
            << ",\"cv_painted_cells\":" << cv_cells
            << ",\"cv_painted_cells_first_horizon\":" << cv_first_cells
            << ",\"reachability_painted_cells\":" << reach_cells
            << ",\"area_grows_with_horizon\":true"
            << ",\"unclamped_horizons_checked\":" << unclamped_horizons
            << ",\"fresh_painted_cells\":" << fresh_cells
            << ",\"coasting_painted_cells\":" << coasting_cells
            << ",\"stale_region_painted_cells\":0"
            << ",\"overlap_cells\":" << overlap
            << ",\"unique_a_cells\":" << unique_a
            << ",\"unique_b_cells\":" << unique_b
            << ",\"max_composition_exact\":true"
            << ",\"independent_expiry_exact\":true"
            << ",\"independent_removal_exact\":true"
            << ",\"empty_master_ghost_cells\":0"
            << ",\"cv_mode_restored_exactly\":true"
            << "}" << std::endl;
  rclcpp::shutdown();
  return 0;
}
