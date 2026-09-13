// Stage-4D: an uncertainty-aware Nav2 costmap layer that converts the
// Stage-4C predicted obstacle trajectories on /tracked_objects into
// conservative future-occupancy cost.
//
// Derives from nav2_costmap_2d::CostmapLayer (not the bare Layer) because
// this layer must own and maintain its own grid of costs across cycles and
// clear them without touching other layers' contributions. CostmapLayer is
// itself a Costmap2D and provides updateWithMax()/touch()/matchSize(),
// which are exactly the primitives required for max-style combination and
// bounded stale-cost clearing. (API verified against the headers installed
// at /opt/ros/jazzy/include/nav2_costmap_2d on this machine -- not assumed
// from another ROS distribution.)
#ifndef PREDICTIVE_NAV_COSTMAP__PREDICTED_OBSTACLE_LAYER_HPP_
#define PREDICTIVE_NAV_COSTMAP__PREDICTED_OBSTACLE_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"

#include "predictive_nav_msgs/msg/tracked_object_array.hpp"

namespace predictive_nav_costmap
{

class PredictedObstacleLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  PredictedObstacleLayer() = default;
  ~PredictedObstacleLayer() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void reset() override;
  bool isClearable() override {return true;}

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void matchSize() override;

private:
  /// Axis-aligned world-coordinate bounding box of the cells this layer
  /// wrote. Kept in world coordinates so it stays valid across rolling-window
  /// origin shifts.
  struct WorldBounds
  {
    bool valid{false};
    double min_x{0.0};
    double min_y{0.0};
    double max_x{0.0};
    double max_y{0.0};

    void reset() {valid = false;}
    void include(double x, double y)
    {
      if (!valid) {
        min_x = max_x = x;
        min_y = max_y = y;
        valid = true;
        return;
      }
      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }
  };

  void getParameters();

  /// Subscription callback. Does no costmap work: it only swaps the latest
  /// message under a short mutex so the costmap update thread is never
  /// blocked on ROS work.
  void tracksCallback(predictive_nav_msgs::msg::TrackedObjectArray::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    std::vector<rclcpp::Parameter> parameters);

  /// Rasterizes every valid prediction of every fresh track into this
  /// layer's own grid. Returns the world bounding box actually written.
  WorldBounds rasterizePredictions(
    const predictive_nav_msgs::msg::TrackedObjectArray & tracks,
    const Eigen::Matrix2d & rotation, double translation_x, double translation_y,
    const rclcpp::Time & now);

  /// Rasterizes one prediction's sigma-level covariance ellipse.
  void rasterizeEllipse(
    double mean_x, double mean_y, const Eigen::Matrix2d & covariance,
    double time_from_now, WorldBounds & written);

  void publishDebugCostmap(const rclcpp::Time & stamp);

  // --- Parameters ------------------------------------------------------
  std::string tracked_objects_topic_;
  double track_timeout_{1.0};
  double track_array_timeout_{1.0};
  double max_prediction_horizon_{3.0};
  double sigma_level_{2.0};
  double temporal_decay_{0.7};
  int max_cost_{200};
  int min_cost_{0};
  double max_influence_radius_{3.0};
  double min_position_variance_{0.0025};
  double transform_tolerance_{0.2};
  bool publish_debug_costmap_{false};
  double stats_log_period_{5.0};

  // --- State -----------------------------------------------------------
  std::string global_frame_;
  bool rolling_window_{false};

  std::mutex data_mutex_;
  predictive_nav_msgs::msg::TrackedObjectArray::SharedPtr latest_tracks_;

  WorldBounds previous_written_;

  rclcpp::Subscription<predictive_nav_msgs::msg::TrackedObjectArray>::SharedPtr tracks_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr debug_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

  // --- Instrumentation (Stage-4D quantitative evidence) ----------------
  uint64_t update_count_{0};
  uint64_t update_costs_calls_{0};
  uint64_t cells_written_last_{0};
  double update_costs_total_us_{0.0};
  double update_costs_max_us_{0.0};
  double max_extent_radius_{0.0};
  rclcpp::Time last_stats_log_;
  rclcpp::Time last_update_stamp_;
  double update_period_total_s_{0.0};
  uint64_t update_period_samples_{0};
};

}  // namespace predictive_nav_costmap

#endif  // PREDICTIVE_NAV_COSTMAP__PREDICTED_OBSTACLE_LAYER_HPP_
