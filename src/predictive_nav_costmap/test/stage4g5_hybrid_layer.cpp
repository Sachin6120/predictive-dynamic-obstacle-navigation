// Stage-4G5 hybrid prediction-policy regression, through the real plugin's ROS
// subscription and LayeredCostmap rebuild API.
//
// The hybrid policy is only defensible if it is EXACTLY the two validated
// representations, selected per track -- never a third thing, never both at
// once. That is what this test pins, cell for cell:
//
//   * all tracks fresh    -> hybrid output is IDENTICAL to cv_covariance mode
//   * all tracks coasting -> hybrid output is IDENTICAL to reachability mode
//   * mixed               -> hybrid output is IDENTICAL to the max-composition
//                            of (CV for the fresh track) and (reachability for
//                            the coasting track), proving per-track selection
//                            with no global leakage and no duplicate region
//   * CV -> coasting -> reacquired: each step equals the corresponding
//                            single-representation render, so no stale corridor
//                            survives a switch in either direction
//   * expiry              -> zero cost, zero ghost cells
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

/// A track carrying BOTH representations, exactly as the Stage-4G4 tracker
/// publishes them. `observation_age` drives the hybrid policy.
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
  t.missed_count = static_cast<uint32_t>(std::lround(observation_age / 0.2));
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
    const double tv = 0.01 + 0.01 * total * total + 0.25 * total * total * total * total / 4;
    Eigen::Matrix2d P;
    P << tv, 0, 0, tv;
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
  auto node = std::make_shared<nav2_util::LifecycleNode>("stage4g5_hybrid_regression", "",
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("g5.tracked_objects_topic", "/g5_layer_tracks"),
      rclcpp::Parameter("g5.sigma_level", 1.5),
      rclcpp::Parameter("g5.temporal_decay", 0.35),
      rclcpp::Parameter("g5.max_cost", 250),
      rclcpp::Parameter("g5.min_cost", 0),
      rclcpp::Parameter("g5.max_influence_radius", 3.0),
      rclcpp::Parameter("g5.max_observation_age", 1.0),
      rclcpp::Parameter("g5.fresh_threshold", 0.1),
      rclcpp::Parameter("g5.prediction_mode", "cv_covariance")}));
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap master("map", false, false);
  const unsigned int cells = 160;
  master.resizeMap(cells, cells, 0.05, -4.0, -4.0);
  auto layer = std::make_shared<predictive_nav_costmap::PredictedObstacleLayer>();
  master.getPlugins()->push_back(layer);
  layer->initialize(&master, "g5", &tf, node, group);
  auto pub = node->create_publisher<Array>("/g5_layer_tracks", 10);
  pub->on_activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  using Grid = std::vector<unsigned char>;
  auto render = [&](std::vector<Object> tracks) {
      Array a;
      a.header.frame_id = "map";
      a.header.stamp = node->now();
      // The hybrid policy reads observation age as (header.stamp - track.stamp),
      // so the track stamps must be re-based onto THIS header to keep the ages
      // the caller asked for.
      for (auto & t : tracks) {
        const double age = t.reachability_predictions.empty() ? 0.0 :
          t.reachability_predictions.front().observation_age;
        t.stamp = rclcpp::Time(a.header.stamp) - rclcpp::Duration::from_seconds(age);
      }
      a.tracks = tracks;
      for (int i = 0; i < 10; ++i) {
        pub->publish(a);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        executor.spin_some();
      }
      master.updateMap(0, 0, 0);
      auto * map = master.getCostmap();
      return Grid(map->getCharMap(), map->getCharMap() + static_cast<size_t>(cells) * cells);
    };
  auto painted = [](const Grid & g) {
      return static_cast<size_t>(std::count_if(g.begin(), g.end(), [](auto c) {return c > 0;}));
    };
  auto set_mode = [&](const std::string & mode) {
      node->set_parameter(rclcpp::Parameter("g5.prediction_mode", mode));
      executor.spin_some();
    };
  auto compose_max = [](const Grid & a, const Grid & b) {
      Grid out(a.size());
      for (size_t i = 0; i < a.size(); ++i) {out[i] = std::max(a[i], b[i]);}
      return out;
    };

  ReachabilityBounds bounds;
  bounds.max_acceleration = 0.5;
  bounds.max_speed = 0.8;
  bounds.covariance_sigma_level = 2.0;
  bounds.safety_margin = 0.0;
  auto fresh = [&](uint32_t id, double x, double y, double vx) {
      return object(id, x, y, vx, node->now(), 0.0, bounds);
    };
  auto coasting = [&](uint32_t id, double x, double y, double vx, double age) {
      return object(id, x, y, vx, node->now(), age, bounds);
    };

  // ---- 1. All tracks fresh: hybrid IS cv_covariance ----------------------
  size_t hybrid_fresh_cells = 0;
  size_t cv_fresh_cells = 0;
  {
    set_mode("cv_covariance");
    const Grid cv = render({fresh(11, -0.8, -0.2, 0.3)});
    set_mode("hybrid");
    const Grid hy = render({fresh(11, -0.8, -0.2, 0.3)});
    require(cv == hy, "hybrid differs from cv_covariance while the track is fresh");
    cv_fresh_cells = painted(cv);
    hybrid_fresh_cells = painted(hy);
    require(cv_fresh_cells > 0, "cv mode painted nothing");
  }

  // ---- 2. All tracks coasting: hybrid IS reachability --------------------
  size_t hybrid_coast_cells = 0;
  size_t reach_coast_cells = 0;
  {
    set_mode("reachability");
    const Grid rc = render({coasting(11, -0.8, -0.2, 0.3, 0.4)});
    set_mode("hybrid");
    const Grid hy = render({coasting(11, -0.8, -0.2, 0.3, 0.4)});
    require(rc == hy, "hybrid differs from reachability while the track is coasting");
    reach_coast_cells = painted(rc);
    hybrid_coast_cells = painted(hy);
  }

  // ---- 2b. Conservatism during observation loss, measured like-for-like ---
  // Total painted cells cannot be compared ACROSS representations here: CV
  // mode's max_influence_radius clamp bounds the ellipse's BOUNDING BOX, so a
  // long-horizon high-variance CV sample degenerates into a filled square while
  // a reachable set stays an inscribed ellipse (pre-existing Stage-4D
  // behaviour, documented at Stage-4G4). The comparison is therefore made at
  // the single short horizon where neither representation is clamped.
  size_t one_horizon_fresh_cv = 0;
  size_t one_horizon_coasting_reach = 0;
  {
    auto first_horizon_only = [](Object t) {
        t.predictions = {t.predictions[0]};
        t.reachability_predictions = {t.reachability_predictions[0]};
        return t;
      };
    set_mode("hybrid");
    one_horizon_fresh_cv = painted(render({first_horizon_only(fresh(11, 0.0, 0.0, 0.3))}));
    one_horizon_coasting_reach =
      painted(render({first_horizon_only(coasting(11, 0.0, 0.0, 0.3, 0.4))}));
    require(one_horizon_coasting_reach > one_horizon_fresh_cv,
      "hybrid did not become more conservative during observation loss");
  }

  // ---- 3. Mixed fresh + coasting: per-track selection, no leakage --------
  // Rendered apart and composed with the layer's own max rule, then compared
  // against the single hybrid render. Equality proves each track chose
  // independently AND that neither track contributed two regions.
  size_t mixed_cells = 0;
  {
    set_mode("cv_covariance");
    const Grid cv_only_a = render({fresh(11, -1.2, -0.4, 0.3)});
    set_mode("reachability");
    const Grid rc_only_b = render({coasting(22, 1.2, 0.4, -0.3, 0.4)});
    set_mode("hybrid");
    const Grid mixed = render({fresh(11, -1.2, -0.4, 0.3), coasting(22, 1.2, 0.4, -0.3, 0.4)});
    require(mixed == compose_max(cv_only_a, rc_only_b),
      "mixed hybrid output is not exactly CV(fresh) composed with reachability(coasting)");
    mixed_cells = painted(mixed);

    // The negative control: if the policy leaked globally, the mixed render
    // would instead equal one representation applied to both tracks.
    set_mode("cv_covariance");
    const Grid all_cv = render({fresh(11, -1.2, -0.4, 0.3), coasting(22, 1.2, 0.4, -0.3, 0.4)});
    set_mode("reachability");
    const Grid all_rc = render({fresh(11, -1.2, -0.4, 0.3), coasting(22, 1.2, 0.4, -0.3, 0.4)});
    require(mixed != all_cv, "mixed hybrid collapsed to cv for both tracks");
    require(mixed != all_rc, "mixed hybrid collapsed to reachability for both tracks");
  }

  // ---- 4. Transition CV -> coasting -> reacquired ------------------------
  // Each step must equal the corresponding single-representation render, which
  // is the strongest available statement that nothing from the previous step
  // survived: any residual cell would break equality.
  size_t t_fresh = 0;
  size_t t_miss1 = 0;
  size_t t_miss2 = 0;
  size_t t_miss3 = 0;
  size_t t_reacq = 0;
  std::vector<size_t> growth_cells;
  {
    set_mode("cv_covariance");
    const Grid cv_ref = render({fresh(11, 0.0, 0.0, 0.3)});
    set_mode("reachability");
    const Grid rc1 = render({coasting(11, 0.0, 0.0, 0.3, 0.2)});
    const Grid rc2 = render({coasting(11, 0.0, 0.0, 0.3, 0.4)});
    const Grid rc3 = render({coasting(11, 0.0, 0.0, 0.3, 0.6)});

    set_mode("hybrid");
    const Grid h0 = render({fresh(11, 0.0, 0.0, 0.3)});
    const Grid h1 = render({coasting(11, 0.0, 0.0, 0.3, 0.2)});
    const Grid h2 = render({coasting(11, 0.0, 0.0, 0.3, 0.4)});
    const Grid h3 = render({coasting(11, 0.0, 0.0, 0.3, 0.6)});
    const Grid h4 = render({fresh(11, 0.0, 0.0, 0.3)});   // reacquired

    require(h0 == cv_ref, "hybrid fresh step is not the CV representation");
    require(h1 == rc1, "hybrid first-missed-scan step is not the reachability representation");
    require(h2 == rc2, "hybrid second-missed-scan step is not the reachability representation");
    require(h3 == rc3, "hybrid third-missed-scan step is not the reachability representation");
    require(h4 == cv_ref,
      "after reacquisition hybrid did not return EXACTLY to the CV representation "
      "(stale reachable occupancy survived)");

    t_fresh = painted(h0);
    t_miss1 = painted(h1);
    t_miss2 = painted(h2);
    t_miss3 = painted(h3);
    t_reacq = painted(h4);
    require(t_reacq == t_fresh, "reacquisition did not restore the fresh footprint exactly");
    // No prediction ever disappears mid-sequence.
    require(t_fresh > 0 && t_miss1 > 0 && t_miss2 > 0 && t_miss3 > 0 && t_reacq > 0,
      "a valid prediction temporarily disappeared during the transition");

    // Growth with observation age is asserted at the single unclamped horizon,
    // among the coasting steps only -- i.e. the same representation compared
    // with itself. Over the full horizon set every step saturates at
    // max_influence_radius and the counts are legitimately equal, which is a
    // costmap policy bound rather than a property of the model.
    auto first_only = [](Object t) {
        t.predictions = {t.predictions[0]};
        t.reachability_predictions = {t.reachability_predictions[0]};
        return t;
      };
    const size_t g1 = painted(render({first_only(coasting(11, 0.0, 0.0, 0.3, 0.2))}));
    const size_t g2 = painted(render({first_only(coasting(11, 0.0, 0.0, 0.3, 0.4))}));
    const size_t g3 = painted(render({first_only(coasting(11, 0.0, 0.0, 0.3, 0.6))}));
    require(g2 > g1 && g3 > g2,
      "hybrid did not grow monotonically with successive missed scans");
    growth_cells = {g1, g2, g3};
  }

  // ---- 5. Expiry clears everything --------------------------------------
  {
    set_mode("hybrid");
    require(painted(render({coasting(11, 0.0, 0.0, 0.3, 0.6)})) > 0, "setup should paint");
    const Grid expired = render({coasting(11, 0.0, 0.0, 0.3, 1.5)});
    require(painted(expired) == 0, "an expired track still painted predictive cost");
    const Grid empty = render({});
    require(std::all_of(empty.begin(), empty.end(), [](auto c) {return c == 0;}),
      "empty array left ghost cells in master");
  }

  // ---- 6. One track expires while another stays fresh --------------------
  {
    set_mode("cv_covariance");
    const Grid only_fresh = render({fresh(11, -1.2, -0.4, 0.3)});
    set_mode("hybrid");
    const Grid both = render({fresh(11, -1.2, -0.4, 0.3), coasting(22, 1.2, 0.4, -0.3, 1.5)});
    require(both == only_fresh,
      "an expired track changed the surviving track's cost or left ghost cells");
  }

  std::cout << "{\"pass\":true"
            << ",\"hybrid_equals_cv_when_fresh\":true"
            << ",\"hybrid_equals_reachability_when_coasting\":true"
            << ",\"per_track_selection_exact\":true"
            << ",\"no_global_mode_leakage\":true"
            << ",\"cv_fresh_cells\":" << cv_fresh_cells
            << ",\"hybrid_fresh_cells\":" << hybrid_fresh_cells
            << ",\"reachability_coasting_cells\":" << reach_coast_cells
            << ",\"hybrid_coasting_cells\":" << hybrid_coast_cells
            << ",\"mixed_cells\":" << mixed_cells
            << ",\"one_horizon_fresh_cv_cells\":" << one_horizon_fresh_cv
            << ",\"one_horizon_coasting_reach_cells\":" << one_horizon_coasting_reach
            << ",\"transition_cells\":[" << t_fresh << "," << t_miss1 << "," << t_miss2
            << "," << t_miss3 << "," << t_reacq << "]"
            << ",\"coasting_growth_cells_first_horizon\":[" << growth_cells[0] << ","
            << growth_cells[1] << "," << growth_cells[2] << "]"
            << ",\"reacquisition_restores_cv_exactly\":true"
            << ",\"no_prediction_gap_during_transition\":true"
            << ",\"expiry_clears_all\":true"
            << ",\"independent_expiry_exact\":true"
            << ",\"empty_master_ghost_cells\":0"
            << "}" << std::endl;
  rclcpp::shutdown();
  return 0;
}
