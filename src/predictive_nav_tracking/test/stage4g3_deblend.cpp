// Stage-4G3 track-aware deblending regression, against the production
// translation unit and against RECORDED Gazebo LiDAR geometry.
//
// The point sets below are the actual map-frame returns of the accepted
// cluster in the named Stage-4G2 close-crossing frames, reproduced
// bit-for-bit by an offline replay of the node's own static rejection and
// clustering. The seeds are the predicted positions of the confirmed tracks
// at that scan. Nothing here is ground truth: the `_truth` arrays are used
// ONLY to check where a split landed, never to produce one.
#define main stage4g3_original_main
#include "../src/lidar_obstacle_tracker_node.cpp"
#undef main
#include <iostream>
#include <stdexcept>
#include <utility>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

namespace
{
// Recorded Stage-4G2 crossing_01 frame at t0+3.078 s (cluster diameter 0.956 m).
const std::vector<Point2D> merge0_points{
     {-0.217918, -0.018992},
     {-0.204805, 0.031094},
     {-0.191887, 0.081650},
     {-0.140821, 0.134399},
     {-0.139792, -0.183214},
     {-0.190889, -0.129361},
     {-0.218609, -0.078055},
     {-0.232519, -0.027926},
     {-0.107504, 0.450474},
     {-0.133661, 0.499283},
     {-0.186321, 0.542522},
     {-0.195527, 0.593085},
     {-0.193665, 0.646155},
     {-0.208861, 0.695590},
     {-0.150272, 0.764086}};
const std::vector<std::pair<double, double>> merge0_seeds{{-0.196456, -0.023622}, {-0.171360, 0.605346}};
const std::vector<std::pair<double, double>> merge0_truth{{-0.0185, -0.0000}, {0.0000, 0.6686}};

// Recorded Stage-4G2 crossing_01 frame at t0+3.477 s (cluster diameter 0.849 m).
const std::vector<Point2D> merge1_points{
     {-0.016999, -0.020625},
     {-0.029905, 0.032732},
     {-0.011599, 0.086494},
     {0.070101, 0.143760},
     {-0.093705, 0.293714},
     {0.062489, -0.196135},
     {0.012762, -0.138796},
     {-0.047027, -0.082998},
     {-0.037999, -0.030126},
     {-0.160814, 0.338729},
     {-0.196984, 0.385676},
     {-0.213526, 0.434525},
     {-0.191763, 0.489500},
     {-0.182321, 0.543268},
     {-0.141208, 0.604197}};
const std::vector<std::pair<double, double>> merge1_seeds{{-0.001141, -0.025674}, {-0.173025, 0.442659}};
const std::vector<std::pair<double, double>> merge1_truth{{0.1736, -0.0000}, {0.0000, 0.4763}};

// Recorded Stage-4G2 crossing_01 frame at t0+3.879 s (cluster diameter 0.785 m).
const std::vector<Point2D> merge2_points{
     {0.156613, -0.022036},
     {0.157772, 0.034490},
     {-0.106896, 0.083934},
     {0.284165, -0.210295},
     {0.229920, -0.148857},
     {0.174004, -0.089367},
     {0.157750, -0.032341},
     {-0.161484, 0.133482},
     {-0.194792, 0.182514},
     {-0.213202, 0.231670},
     {-0.216350, 0.281800},
     {-0.195513, 0.334745},
     {-0.131274, 0.394388}};
const std::vector<std::pair<double, double>> merge2_seeds{{0.200555, -0.027776}, {-0.165617, 0.253491}};
const std::vector<std::pair<double, double>> merge2_truth{{0.3674, -0.0000}, {0.0000, 0.2821}};

// Recorded Stage-4G2 crossing_01 frame at t0+4.278 s (cluster diameter 0.331 m).
const std::vector<Point2D> occluded0_points{
     {-0.171533, -0.019369},
     {-0.199577, 0.031143},
     {-0.209510, 0.081177},
     {-0.166039, 0.133280},
     {-0.165837, 0.184307},
     {-0.122843, 0.238854},
     {-0.129265, -0.080629},
     {-0.170520, -0.028627}};
const std::vector<std::pair<double, double>> occluded0_seeds{{0.387260, -0.112473}, {-0.186099, 0.081571}};
const std::vector<std::pair<double, double>> occluded0_truth{{0.5601, -0.0000}, {0.0000, 0.0902}};

// Recorded Stage-4G2 crossing_01 frame at t0+4.677 s (cluster diameter 0.338 m).
const std::vector<Point2D> occluded1_points{
     {-0.190367, -0.019216},
     {-0.103678, 0.032041},
     {-0.147437, -0.285806},
     {-0.169063, -0.232430},
     {-0.218299, -0.178199},
     {-0.211304, -0.128415},
     {-0.208468, -0.078347},
     {-0.189036, -0.028418}};
const std::vector<std::pair<double, double>> occluded1_seeds{{0.579013, -0.156778}, {-0.173740, -0.112298}};
const std::vector<std::pair<double, double>> occluded1_truth{{0.7520, -0.0000}, {0.0000, -0.1021}};


/// Builds seeds the way deblend_seeds() does, but from explicit positions so
/// the geometry under test is exactly the recorded one.
std::vector<LidarObstacleTracker::DeblendSeed> make_seeds(
  const std::vector<std::pair<double, double>> & positions)
{
  std::vector<LidarObstacleTracker::DeblendSeed> seeds;
  for (const auto & p : positions) {
    LidarObstacleTracker::DeblendSeed s;
    s.x = p.first;
    s.y = p.second;
    seeds.push_back(s);
  }
  return seeds;
}

Cluster make_cluster(const std::vector<Point2D> & points)
{
  Cluster c;
  c.points = points;
  double sx = 0.0;
  double sy = 0.0;
  for (const auto & p : points) {
    sx += p.x;
    sy += p.y;
  }
  c.centroid = {sx / points.size(), sy / points.size()};
  double min_x = points.front().x, max_x = min_x;
  double min_y = points.front().y, max_y = min_y;
  for (const auto & p : points) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }
  c.diameter = std::hypot(max_x - min_x, max_y - min_y);
  return c;
}

double dist(const Point2D & a, const std::pair<double, double> & b)
{
  return std::hypot(a.x - b.first, a.y - b.second);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarObstacleTracker>();
  std::vector<Point2D> children;
  bool ambiguous = false;
  size_t case_a_split = 0;
  size_t case_b_refused = 0;
  double worst_child_error = 0.0;
  double worst_merged_centroid_error = 0.0;

  // ---- CASE A: both objects visible inside one cluster -> split ----------
  // Each recorded merged frame must split, and each child centroid must land
  // nearer to a DIFFERENT true object, within the 0.45 m Stage-4G2 scoring
  // gate. A split that put both children on one object would pass a naive
  // "did it split?" check and is explicitly rejected here.
  struct Frame
  {
    const std::vector<Point2D> * points;
    const std::vector<std::pair<double, double>> * seeds;
    const std::vector<std::pair<double, double>> * truth;
  };
  const std::vector<Frame> case_a{
    {&merge0_points, &merge0_seeds, &merge0_truth},
    {&merge1_points, &merge1_seeds, &merge1_truth},
    {&merge2_points, &merge2_seeds, &merge2_truth}
  };
  for (size_t i = 0; i < case_a.size(); ++i) {
    const auto cluster = make_cluster(*case_a[i].points);
    require(cluster.diameter > 0.55, "recorded merged cluster is not suspicious");
    const auto seeds = make_seeds(*case_a[i].seeds);
    require(seeds.size() == 2, "recorded merge frame should have two confirmed tracks");
    require(
      node->try_deblend(cluster, seeds, children, ambiguous),
      "observable merge (case A) was not deblended");
    require(children.size() == 2, "split did not produce two measurements");
    const auto & truth = *case_a[i].truth;
    const size_t near0 = dist(children[0], truth[0]) <= dist(children[0], truth[1]) ? 0 : 1;
    const size_t near1 = dist(children[1], truth[0]) <= dist(children[1], truth[1]) ? 0 : 1;
    require(near0 != near1, "both children were attributed to the same object");
    require(
      dist(children[0], truth[near0]) < 0.45 && dist(children[1], truth[near1]) < 0.45,
      "child centroid outside the scoring gate of its object");
    // The blended centroid it replaced sat between the objects; the children
    // must genuinely be better measurements than that single centroid.
    require(
      dist(children[0], truth[near0]) < dist(cluster.centroid, truth[near0]) &&
      dist(children[1], truth[near1]) < dist(cluster.centroid, truth[near1]),
      "split did not improve on the merged centroid");
    ++case_a_split;
    worst_child_error = std::max(
      worst_child_error,
      std::max(dist(children[0], truth[near0]), dist(children[1], truth[near1])));
    worst_merged_centroid_error = std::max(
      worst_merged_centroid_error,
      std::min(dist(cluster.centroid, truth[0]), dist(cluster.centroid, truth[1])));
  }

  // ---- CASE B: one object physically occludes the other -> NO split ------
  // Two confirmed tracks still predict into the region, but only one object
  // returns any points. Fabricating a second measurement here is exactly the
  // failure mode this guard exists to prevent.
  const std::vector<Frame> case_b{
    {&occluded0_points, &occluded0_seeds, &occluded0_truth},
    {&occluded1_points, &occluded1_seeds, &occluded1_truth}
  };
  for (size_t i = 0; i < case_b.size(); ++i) {
    const auto cluster = make_cluster(*case_b[i].points);
    const auto seeds = make_seeds(*case_b[i].seeds);
    require(seeds.size() == 2, "occlusion frame should still have two tracks");
    require(
      !node->try_deblend(cluster, seeds, children, ambiguous),
      "true occlusion (case B) was wrongly split into two measurements");
    ++case_b_refused;
  }

  // ---- A single object's cluster is never suspicious ---------------------
  {
    // The largest single-object cluster measured across all 22 Stage-4G2 runs
    // was 0.468 m across; a compact blob well inside that must not be touched
    // even with two seeds sitting on top of it.
    std::vector<Point2D> blob;
    for (int i = 0; i < 12; ++i) {
      blob.push_back({0.02 * i, 0.01 * i});
    }
    const auto cluster = make_cluster(blob);
    require(cluster.diameter < 0.55, "test blob is not a single-object size");
    const auto seeds = make_seeds({{0.0, 0.0}, {0.22, 0.11}});
    require(
      !node->try_deblend(cluster, seeds, children, ambiguous),
      "a single-object cluster was split");
    require(!ambiguous, "single-object cluster reported as an ambiguous merge");
  }

  // ---- A solid oversized cluster is NOT split without a gap --------------
  // Anti-fabrication: a large but continuous structure (the shape a static
  // residual or a wall fragment takes) must survive intact however tempting
  // the seed geometry is. This is what keeps deblending from inventing
  // dynamic objects in the Stage-4G1 static world.
  {
    std::vector<Point2D> solid;
    for (int i = 0; i < 30; ++i) {
      solid.push_back({0.03 * i, 0.0});   // 0.87 m long, 0.03 m spacing, no gap
    }
    const auto cluster = make_cluster(solid);
    require(cluster.diameter > 0.55, "solid test cluster is not suspicious");
    const auto seeds = make_seeds({{0.05, 0.0}, {0.82, 0.0}});
    require(
      !node->try_deblend(cluster, seeds, children, ambiguous),
      "a gapless cluster was split into two fabricated objects");
    require(ambiguous, "refused split of a claimed merge was not flagged ambiguous");
  }

  // ---- One confirmed track can never justify a split ---------------------
  // Objects that were merged from the moment they appeared have only one
  // track, so deblending must not manufacture a second identity.
  {
    const auto cluster = make_cluster(merge0_points);
    const auto seeds = make_seeds({merge0_seeds[0]});
    require(
      !node->try_deblend(cluster, seeds, children, ambiguous),
      "a split was invented from a single track");
  }

  // ---- Tentative tracks must not seed a split ----------------------------
  // deblend_seeds() only admits tracks with >= min_observations_to_publish.
  {
    const auto time = [](double t) {
        return rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
      };
    node->tracks_.clear();
    node->associate_and_update({{-0.196, -0.024}, {-0.171, 0.605}}, time(20.0));
    require(node->tracks_.size() == 2, "expected two tentative tracks");
    require(node->deblend_seeds(time(20.0)).empty(), "tentative track seeded a split");
    node->associate_and_update({{-0.196, -0.024}, {-0.171, 0.605}}, time(20.2));
    node->associate_and_update({{-0.196, -0.024}, {-0.171, 0.605}}, time(20.4));
    require(node->deblend_seeds(time(20.4)).size() == 2, "confirmed tracks not seeded");
  }

  // ---- An unsplit merged centroid updates exactly ONE track --------------
  // Stage-4G3 requirement: one ambiguous measurement must never correct two
  // independent filters. Greedy association is one-to-one, and this pins it.
  {
    const auto time = [](double t) {
        return rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
      };
    node->tracks_.clear();
    for (int i = 0; i < 4; ++i) {
      node->associate_and_update({{0.0, -0.25}, {0.0, 0.25}}, time(30.0 + 0.2 * i));
    }
    require(node->tracks_.size() == 2, "setup should hold two tracks");
    const auto before_a = node->tracks_[0].kf_x;
    const auto before_b = node->tracks_[1].kf_x;
    const uint32_t obs_a = node->tracks_[0].observations;
    const uint32_t obs_b = node->tracks_[1].observations;
    node->associate_and_update({{0.0, 0.0}}, time(30.8));   // one blended centroid
    const bool a_updated = node->tracks_[0].observations == obs_a + 1;
    const bool b_updated = node->tracks_[1].observations == obs_b + 1;
    require(a_updated != b_updated, "one centroid updated both tracks (or neither)");
    require(
      node->tracks_[0].missed_count + node->tracks_[1].missed_count == 1,
      "the unmatched track did not coast");
    // The coasting track's stored state must be untouched by the other's update.
    require(
      (a_updated ? node->tracks_[1].kf_x == before_b : node->tracks_[0].kf_x == before_a),
      "coasting track's state was modified by another track's measurement");
  }

  std::cout << "{\"pass\":true"
            << ",\"recorded_geometry\":true"
            << ",\"case_a_observable_merges_split\":" << case_a_split
            << ",\"case_b_occlusions_refused\":" << case_b_refused
            << ",\"worst_child_centroid_error_m\":" << worst_child_error
            << ",\"best_merged_centroid_error_m\":" << worst_merged_centroid_error
            << ",\"single_object_cluster_not_split\":true"
            << ",\"gapless_cluster_not_split\":true"
            << ",\"single_track_cannot_split\":true"
            << ",\"tentative_track_cannot_seed\":true"
            << ",\"one_centroid_updates_one_track\":true"
            << ",\"coasting_track_state_untouched\":true"
            << "}" << std::endl;
  rclcpp::shutdown();
  return 0;
}
