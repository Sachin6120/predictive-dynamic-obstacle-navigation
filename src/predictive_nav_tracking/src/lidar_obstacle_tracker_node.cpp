// lidar_obstacle_tracker_node.cpp
//
// Stage-4B pipeline:
//   /scan -> valid points -> transform to map frame -> reject points that
//   coincide with known-occupied static map cells -> cluster remaining
//   points -> gated nearest-neighbour association against existing tracks
//   -> velocity from short position history -> publish TrackedObjectArray
//   + RViz MarkerArray.
//
// Deliberately no Kalman filter / prediction model beyond simple constant-
// velocity extrapolation for the association gate. That is Stage-4C.

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

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

using std::placeholders::_1;

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

struct Track
{
  uint32_t id{0};
  double x{0.0};
  double y{0.0};
  double vx{0.0};
  double vy{0.0};
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
      "cluster_distance=%.2fm association_gate=%.2fm track_timeout=%.2fs",
      scan_topic_.c_str(), map_topic_.c_str(), static_reject_radius_, cluster_distance_,
      association_gate_, track_timeout_);
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

    this->declare_parameter<double>("static_reject_radius", 0.15);
    this->declare_parameter<int>("occupied_threshold", 65);
    this->declare_parameter<bool>("unknown_cells_are_static", false);

    this->declare_parameter<double>("cluster_distance", 0.35);
    this->declare_parameter<int>("cluster_min_points", 3);
    this->declare_parameter<int>("cluster_max_points", 200);
    this->declare_parameter<double>("cluster_max_diameter", 1.2);

    this->declare_parameter<double>("association_gate", 0.6);
    this->declare_parameter<double>("track_timeout", 1.0);
    this->declare_parameter<int>("max_missed_scans", 5);
    this->declare_parameter<int>("min_observations_to_publish", 3);
    this->declare_parameter<int>("velocity_history_size", 5);

    this->declare_parameter<bool>("publish_candidate_markers", true);
    this->declare_parameter<double>("velocity_marker_scale", 1.0);
    this->declare_parameter<double>("tf_lookup_timeout", 0.1);
  }

  void read_parameters()
  {
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();

    static_reject_radius_ = this->get_parameter("static_reject_radius").as_double();
    occupied_threshold_ = static_cast<int8_t>(this->get_parameter("occupied_threshold").as_int());
    unknown_cells_are_static_ = this->get_parameter("unknown_cells_are_static").as_bool();

    cluster_distance_ = this->get_parameter("cluster_distance").as_double();
    cluster_min_points_ = static_cast<size_t>(this->get_parameter("cluster_min_points").as_int());
    cluster_max_points_ = static_cast<size_t>(this->get_parameter("cluster_max_points").as_int());
    cluster_max_diameter_ = this->get_parameter("cluster_max_diameter").as_double();

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
  }

  // ---------------------------------------------------------------------
  // Map handling
  // ---------------------------------------------------------------------
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
    RCLCPP_INFO(
      get_logger(), "Received static map: %ux%u @ %.3fm/cell", msg->info.width,
      msg->info.height, msg->info.resolution);
  }

  // True if (x, y) in map frame lies within static_reject_radius_ of a cell
  // whose occupancy probability is >= occupied_threshold_ (i.e. it is
  // consistent with known static geometry and should be rejected before
  // clustering).
  bool is_static_point(double x, double y) const
  {
    if (!map_) {
      // No map yet: fail safe by treating everything as static so we never
      // spawn false tracks before the environment reference is available.
      return true;
    }

    const auto & info = map_->info;
    const double res = info.resolution;
    const double ox = info.origin.position.x;
    const double oy = info.origin.position.y;

    const int mx = static_cast<int>(std::floor((x - ox) / res));
    const int my = static_cast<int>(std::floor((y - oy) / res));

    const int radius_cells = static_cast<int>(std::ceil(static_reject_radius_ / res));

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const double cell_dist = std::hypot(dx * res, dy * res);
        if (cell_dist > static_reject_radius_) {
          continue;
        }
        const int cx = mx + dx;
        const int cy = my + dy;
        if (cx < 0 || cy < 0 || cx >= static_cast<int>(info.width) ||
          cy >= static_cast<int>(info.height))
        {
          continue;
        }
        const int8_t value = map_->data[static_cast<size_t>(cy) * info.width + cx];
        if (value < 0) {
          if (unknown_cells_are_static_) {
            return true;
          }
          continue;
        }
        if (value >= occupied_threshold_) {
          return true;
        }
      }
    }
    return false;
  }

  // ---------------------------------------------------------------------
  // Scan processing
  // ---------------------------------------------------------------------
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
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
    const std::vector<Point2D> centroids = cluster_points(dynamic_points);

    const rclcpp::Time stamp(msg->header.stamp);
    associate_and_update(centroids, stamp);
    prune_stale_tracks(stamp);
    publish_tracks(stamp);
    publish_markers(stamp, centroids);
  }

  std::vector<Point2D> extract_dynamic_points(
    const sensor_msgs::msg::LaserScan & scan,
    const geometry_msgs::msg::TransformStamped & transform) const
  {
    std::vector<Point2D> dynamic_points;
    dynamic_points.reserve(scan.ranges.size());

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

      if (!is_static_point(map_pt.point.x, map_pt.point.y)) {
        dynamic_points.push_back({map_pt.point.x, map_pt.point.y});
      }
    }
    return dynamic_points;
  }

  // Simple O(n^2) single-link clustering: adequate for a handful of
  // non-static returns per scan cleared out of an otherwise static scene.
  std::vector<Point2D> cluster_points(const std::vector<Point2D> & points) const
  {
    std::vector<Point2D> centroids;
    const size_t n = points.size();
    std::vector<bool> visited(n, false);

    for (size_t seed = 0; seed < n; ++seed) {
      if (visited[seed]) {
        continue;
      }
      std::vector<size_t> cluster_indices{seed};
      visited[seed] = true;

      // Grow the cluster with a simple frontier expansion.
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

      centroids.push_back(
        {sum_x / static_cast<double>(cluster_indices.size()),
          sum_y / static_cast<double>(cluster_indices.size())});
    }
    return centroids;
  }

  // ---------------------------------------------------------------------
  // Association / track maintenance
  // ---------------------------------------------------------------------
  void associate_and_update(const std::vector<Point2D> & centroids, const rclcpp::Time & stamp)
  {
    const size_t n_tracks = tracks_.size();
    const size_t n_dets = centroids.size();

    std::vector<Point2D> predicted(n_tracks);
    for (size_t t = 0; t < n_tracks; ++t) {
      const double dt = (stamp - tracks_[t].last_update).seconds();
      predicted[t].x = tracks_[t].x + tracks_[t].vx * dt;
      predicted[t].y = tracks_[t].y + tracks_[t].vy * dt;
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
    track.x = det.x;
    track.y = det.y;
    track.vx = 0.0;
    track.vy = 0.0;
    track.age = 1;
    track.observations = 1;
    track.missed_count = 0;
    track.last_update = stamp;
    track.history.push_back({stamp, det.x, det.y});
    return track;
  }

  void update_track(Track & track, const Point2D & det, const rclcpp::Time & stamp)
  {
    track.history.push_back({stamp, det.x, det.y});
    while (track.history.size() > velocity_history_size_) {
      track.history.pop_front();
    }

    if (track.history.size() >= 2) {
      const auto & oldest = track.history.front();
      const auto & newest = track.history.back();
      const double dt = (newest.stamp - oldest.stamp).seconds();
      if (dt > 1e-3) {
        track.vx = (newest.x - oldest.x) / dt;
        track.vy = (newest.y - oldest.y) / dt;
      }
    }

    track.x = det.x;
    track.y = det.y;
    track.age += 1;
    track.observations += 1;
    track.missed_count = 0;
    track.last_update = stamp;
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
      obj.position.x = t.x;
      obj.position.y = t.y;
      obj.position.z = 0.0;
      obj.velocity.x = t.vx;
      obj.velocity.y = t.vy;
      obj.velocity.z = 0.0;
      obj.speed = std::hypot(t.vx, t.vy);
      obj.age = t.age;
      obj.observations = t.observations;
      obj.missed_count = t.missed_count;
      obj.stamp = t.last_update;
      array_msg.tracks.push_back(obj);
    }
    tracks_pub_->publish(array_msg);
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

      visualization_msgs::msg::Marker centroid;
      centroid.header.frame_id = map_frame_;
      centroid.header.stamp = stamp;
      centroid.ns = "centroid";
      centroid.id = static_cast<int>(t.id);
      centroid.type = visualization_msgs::msg::Marker::SPHERE;
      centroid.action = visualization_msgs::msg::Marker::ADD;
      centroid.pose.position.x = t.x;
      centroid.pose.position.y = t.y;
      centroid.pose.position.z = 0.15;
      centroid.pose.orientation.w = 1.0;
      centroid.scale.x = 0.3;
      centroid.scale.y = 0.3;
      centroid.scale.z = 0.3;
      centroid.color.r = 1.0f;
      centroid.color.g = 0.1f;
      centroid.color.b = 0.1f;
      centroid.color.a = 0.9f;
      centroid.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(centroid);

      visualization_msgs::msg::Marker label;
      label.header = centroid.header;
      label.ns = "label";
      label.id = static_cast<int>(t.id);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position.x = t.x;
      label.pose.position.y = t.y;
      label.pose.position.z = 0.6;
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.25;
      label.color.r = 1.0f;
      label.color.g = 1.0f;
      label.color.b = 1.0f;
      label.color.a = 1.0f;
      char text[64];
      std::snprintf(text, sizeof(text), "ID %u  %.2f m/s", t.id, std::hypot(t.vx, t.vy));
      label.text = text;
      label.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(label);

      visualization_msgs::msg::Marker velocity;
      velocity.header = centroid.header;
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
      start.x = t.x;
      start.y = t.y;
      start.z = 0.15;
      geometry_msgs::msg::Point end;
      end.x = t.x + t.vx * velocity_marker_scale_;
      end.y = t.y + t.vy * velocity_marker_scale_;
      end.z = 0.15;
      velocity.points.push_back(start);
      velocity.points.push_back(end);
      velocity.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(velocity);

      if (t.history.size() >= 2) {
        visualization_msgs::msg::Marker trail;
        trail.header = centroid.header;
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
    }

    // Delete markers for tracks that no longer exist / are no longer confirmed.
    for (const auto & prev_id : previous_marker_ids_) {
      if (current_ids.count(prev_id)) {
        continue;
      }
      for (const char * ns : {"centroid", "label", "velocity", "trail"}) {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = map_frame_;
        del.header.stamp = stamp;
        del.ns = ns;
        del.id = static_cast<int>(prev_id);
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
  std::string scan_topic_;
  std::string map_topic_;
  std::string map_frame_;

  double static_reject_radius_{0.15};
  int8_t occupied_threshold_{65};
  bool unknown_cells_are_static_{false};

  double cluster_distance_{0.35};
  size_t cluster_min_points_{3};
  size_t cluster_max_points_{200};
  double cluster_max_diameter_{1.2};

  double association_gate_{0.6};
  double track_timeout_{1.0};
  uint32_t max_missed_scans_{5};
  uint32_t min_observations_to_publish_{3};
  size_t velocity_history_size_{5};

  bool publish_candidate_markers_{true};
  double velocity_marker_scale_{1.0};
  double tf_lookup_timeout_{0.1};

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
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
