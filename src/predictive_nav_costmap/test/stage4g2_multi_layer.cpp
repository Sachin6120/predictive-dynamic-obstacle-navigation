// Direct regression of the real Stage-4D plugin through its ROS subscription
// and LayeredCostmap rebuild API. No production layer implementation changes.
#include <algorithm>
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <vector>
#include "predictive_nav_costmap/predicted_obstacle_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"

using Array = predictive_nav_msgs::msg::TrackedObjectArray;
using Object = predictive_nav_msgs::msg::TrackedObject;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

Object object(uint32_t id, double x, double y, double vx, rclcpp::Time now)
{
  Object t;
  t.id = id;
  t.kalman_initialized = true;
  t.stamp = now;
  t.observations = 10;
  t.age = 10;
  t.position.x = x;
  t.position.y = y;
  t.velocity.x = vx;
  for (int i = 1; i <= 6; ++i) {
    predictive_nav_msgs::msg::TrackedObjectPrediction p;
    p.time_from_now = i * 0.5;
    p.stamp = now + rclcpp::Duration::from_seconds(p.time_from_now);
    p.position.x = x + vx * p.time_from_now;
    p.position.y = y;
    const double h = p.time_from_now;
    const double variance = 0.01 + 0.01 * h * h + 0.25 * h * h * h * h / 4;
    p.position_covariance = {variance, 0, 0, variance};
    t.predictions.push_back(p);
  }
  return t;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_util::LifecycleNode>("stage4g2_layer_regression", "",
    rclcpp::NodeOptions().parameter_overrides({
      rclcpp::Parameter("g2.tracked_objects_topic", "/g2_layer_tracks"),
      rclcpp::Parameter("g2.sigma_level", 1.5),
      rclcpp::Parameter("g2.temporal_decay", 0.35),
      rclcpp::Parameter("g2.max_cost", 250),
      rclcpp::Parameter("g2.min_cost", 0),
      rclcpp::Parameter("g2.max_influence_radius", 0.55)}));
  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2_ros::Buffer tf(node->get_clock());
  nav2_costmap_2d::LayeredCostmap master("map", false, false);
  master.resizeMap(120, 120, 0.05, -3.0, -3.0);
  auto layer = std::make_shared<predictive_nav_costmap::PredictedObstacleLayer>();
  master.getPlugins()->push_back(layer);
  layer->initialize(&master, "g2", &tf, node, group);
  auto pub = node->create_publisher<Array>("/g2_layer_tracks", 10);
  pub->on_activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  auto render = [&](std::vector<Object> tracks) {
      Array a;
      a.header.frame_id = "map";
      a.header.stamp = node->now();
      a.tracks = tracks;
      // Spin actual subscription delivery, with a bounded discovery allowance.
      for (int i = 0; i < 10; ++i) {
        pub->publish(a);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        executor.spin_some();
      }
      master.updateMap(0, 0, 0);
      auto * map = master.getCostmap();
      return std::vector<unsigned char>(map->getCharMap(), map->getCharMap() + 120 * 120);
    };
  const auto now = node->now();
  auto a = object(11, -0.8, -0.2, 0.3, now);
  auto b = object(22, 0.8, 0.2, -0.3, now);
  const auto only_a = render({a});
  const auto only_b = render({b});
  const auto both = render({a, b});
  size_t overlap = 0, unique_a = 0, unique_b = 0;
  for (size_t i = 0; i < both.size(); ++i) {
    require(both[i] == std::max(only_a[i], only_b[i]), "combined cost is not max(A,B)");
    overlap += only_a[i] > 0 && only_b[i] > 0;
    unique_a += only_a[i] > 0 && only_b[i] == 0;
    unique_b += only_b[i] > 0 && only_a[i] == 0;
    require(both[i] <= 250, "prediction exceeded configured cost cap");
  }
  require(overlap && unique_a && unique_b, "test geometry lacks overlap or unique contributions");
  a.stamp = node->now() - rclcpp::Duration::from_seconds(1.01);
  b.stamp = node->now();
  const auto expired_a = render({a, b});
  require(expired_a == only_b, "expiring A changed B or left ghost cells");
  const auto removed_a = render({b});
  require(removed_a == only_b, "removing A changed B or left ghost cells");
  const auto empty = render({});
  require(std::all_of(empty.begin(), empty.end(), [](auto c) {return c == 0;}),
    "empty array left ghost cells in master");
  std::cout << "{\"pass\":true,\"overlap_cells\":" << overlap
            << ",\"unique_a_cells\":" << unique_a << ",\"unique_b_cells\":" << unique_b
            << ",\"max_composition_exact\":true,\"independent_expiry_exact\":true,"
            << "\"independent_removal_exact\":true,\"empty_master_ghost_cells\":0}" << std::endl;
  rclcpp::shutdown();
  return 0;
}
