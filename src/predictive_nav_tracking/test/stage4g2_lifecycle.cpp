// White-box lifecycle regression against the production translation unit.
// Synthetic centroids here are explicitly a UNIT test, not LiDAR detection evidence.
#define main stage4g2_original_main
#include "../src/lidar_obstacle_tracker_node.cpp"
#undef main
#include <iostream>
#include <stdexcept>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarObstacleTracker>();
  const auto time = [](double t) {return rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);};
  const auto step = [&](double t, std::vector<Point2D> observations) {
      node->associate_and_update(observations, time(t));
      node->prune_stale_tracks(time(t));
    };
  for (int i = 0; i < 6; ++i) {
    step(10 + i * 0.2, {{i * 0.04, -1}, {2 - i * 0.06, 1}});
  }
  require(node->tracks_.size() == 2, "two confirmed tracks missing");
  const auto a = node->tracks_[0];
  const auto b = node->tracks_[1];
  require(a.observations == 6 && b.observations == 6, "observation counts wrong");
  const auto predictions_b = node->predict_future(b);
  // Miss A for 4 scans; keep B measured. A's state remains at its last observation.
  for (int i = 1; i <= 4; ++i) {
    step(11 + i * 0.2, {{1.7 - i * 0.06, 1}});
  }
  require(node->tracks_[0].missed_count == 4, "missed scans not counted");
  require(node->tracks_[0].observations == 6, "prediction counted as observation");
  require(node->tracks_[0].last_update == a.last_update, "observation stamp changed on miss");
  require(node->tracks_[0].kf_x == a.kf_x && node->tracks_[0].kf_P == a.kf_P,
    "unmatched stored state unexpectedly changed");
  step(12.0, {{0.4, -1}, {1.4, 1}});
  require(node->tracks_[0].id == a.id && node->tracks_[0].missed_count == 0,
    "reacquisition did not retain ID after 4 misses");
  require(node->tracks_[0].observations == 7, "reacquisition count wrong");
  // Disappear completely: exactly 5 scans at <=1 s survive, sixth expires.
  for (int i = 1; i <= 5; ++i) {
    step(12 + i * 0.2, {});
    require(node->tracks_.size() == 2, "track expired before configured boundary");
    require(node->tracks_[0].missed_count == static_cast<uint32_t>(i), "miss count wrong");
  }
  step(13.2, {});
  require(node->tracks_.empty(), "stale track did not expire at sixth miss");
  step(13.4, {{0.68, -1}, {0.98, 1}});
  require(node->tracks_[0].id != a.id, "expired ID was reused");
  // Predicting/mutating A must not modify B or its future covariance.
  Track copy_a = a;
  copy_a.kf_P *= 3;
  copy_a.kf_x(2) += 2;
  const auto ignored = node->predict_future(copy_a);
  (void)ignored;
  const auto after_b = node->predict_future(b);
  for (size_t i = 0; i < predictions_b.size(); ++i) {
    require(predictions_b[i].x == after_b[i].x &&
      predictions_b[i].position_covariance == after_b[i].position_covariance,
      "shared prediction state");
  }
  std::cout << "{\"pass\":true,\"synthetic_unit_test\":true,"
            << "\"missed_scans_survived\":5,\"expiry_after_last_observation_s\":1.2,"
            << "\"reacquisition_same_id_after_misses\":4,\"new_id_after_expiry\":true,"
            << "\"observation_stamp_and_count_preserved_on_misses\":true,"
            << "\"prediction_state_independent\":true}" << std::endl;
  node.reset();
  rclcpp::shutdown();
  return 0;
}
