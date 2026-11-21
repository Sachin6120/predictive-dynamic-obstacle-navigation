// Stage-5C.1: COSTMAP REPRESENTATION FIDELITY for the reachability rasterizer.
//
// THE DEFECT THIS PINS. Until Stage-5C.1, rasterizeReachableSet() applied the
// max_influence_radius costmap-policy bound to `semi_major` / `semi_minor`
// themselves, and the membership test divides by those same variables. The
// planner therefore received a SMALLER ELLIPSE, not the published ellipse
// clipped to a window. CV mode applies the identical bound to the bounding BOX
// only and tests membership against the unclamped covariance, so the two modes
// were not treating the same policy bound the same way.
//
// At the production geometry (max_influence_radius 0.55 m, resolution 0.05 m)
// every horizon from 1.5 s on has semi-axes far above the bound, so:
//   CV           filled its whole clamped window   23 x 23 = 529 cells
//   reachability collapsed to the inscribed disc            377 cells
//   ratio 0.713 ~= pi/4
// Those are exactly the numbers recorded in validation/stage5c/model_audit.json
// ("cv_cells": 529.0, "reach_cells": 377.0, "cell_ratio": 0.713), i.e.
// reachability presented FEWER planner-visible cells than CV while describing a
// mathematically 2.16x larger region.
//
// HOW THIS TEST WORKS. Every case is driven through the real plugin, its real
// ROS subscription and the real LayeredCostmap rebuild. The expected cell set
// is then computed INDEPENDENTLY, twice:
//
//   NEW reference -- cell centre inside the ellipse AS PUBLISHED, and inside
//                    the policy window. This is the required invariant.
//   OLD reference -- cell centre inside the ellipse WITH THE AXES CLAMPED,
//                    i.e. a model of the pre-Stage-5C.1 behaviour, so the
//                    anomaly is reproduced mechanistically in the same binary
//                    that proves it fixed.
//
// The rasterizer must equal the NEW reference exactly -- no false negatives
// (Invariant F) and no false positives (Invariant S). The difference against
// the OLD reference is reported as the false-negative count before the fix.
//
// DISCRETIZATION RULE, stated explicitly as section 9 requires: a cell is
// painted if and only if ITS CENTRE lies inside the closed region. Cell (i,j)
// has centre (origin_x + (i+0.5)*res, origin_y + (j+0.5)*res). The test asserts
// that convention against the production Costmap2D before relying on it.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "predictive_nav_costmap/predicted_obstacle_layer.hpp"
#include "nav2_util/lifecycle_node.hpp"

using Array = predictive_nav_msgs::msg::TrackedObjectArray;
using Object = predictive_nav_msgs::msg::TrackedObject;
using Region = predictive_nav_msgs::msg::ReachabilityPrediction;

namespace
{
int failures = 0;

void check(bool condition, const std::string & what)
{
  if (!condition) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
  }
}

/// One deterministic ellipse to rasterize. The geometry is written straight
/// into the message rather than produced by buildRegion(), so that eccentric,
/// rotated and degenerate shapes can be specified exactly.
struct Case
{
  const char * name;
  double cx;
  double cy;
  double a;              // semi_major as published
  double b;              // semi_minor as published
  double yaw;            // rad
  double influence;      // max_influence_radius for this case; <= 0 disables it
  bool expect_clamp_binds;
};

/// Closed-ellipse membership, written out independently of production code.
bool inEllipse(double x, double y, double cx, double cy, double a, double b, double yaw)
{
  if (a <= 0.0 || b <= 0.0) {
    return false;
  }
  const double dx = x - cx;
  const double dy = y - cy;
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double u = (dx * c + dy * s) / a;
  const double v = (-dx * s + dy * c) / b;
  return u * u + v * v <= 1.0;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // Timeouts are deliberately huge: this test is about geometry, and a latched
  // track must stay eligible across many updateMap() calls so the benchmark can
  // iterate without republishing.
  auto node = std::make_shared<nav2_util::LifecycleNode>(
    "stage5c1_raster_fidelity", "",
    rclcpp::NodeOptions().parameter_overrides({
    rclcpp::Parameter("r.tracked_objects_topic", "/stage5c1_raster_tracks"),
    rclcpp::Parameter("r.prediction_mode", "reachability"),
    // temporal_decay 0 => uniform cost for every horizon, so cell COUNTS are
    // never confounded by a sample falling below the cost floor.
    rclcpp::Parameter("r.temporal_decay", 0.0),
    rclcpp::Parameter("r.max_cost", 250),
    rclcpp::Parameter("r.min_cost", 0),
    rclcpp::Parameter("r.max_influence_radius", 3.0),
    rclcpp::Parameter("r.max_prediction_horizon", 3.0),
    rclcpp::Parameter("r.max_observation_age", 1.0e6),
    rclcpp::Parameter("r.track_timeout", 1.0e6),
    rclcpp::Parameter("r.track_array_timeout", 1.0e6)}));

  auto group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2_ros::Buffer tf(node->get_clock());

  // 20 m x 20 m at 5 cm, origin (-10,-10): room for a large ellipse fully
  // inside AND for centres close enough to an edge that the window clips.
  const unsigned int kCells = 400;
  const double kRes = 0.05;
  const double kOriginX = -10.0;
  const double kOriginY = -10.0;
  nav2_costmap_2d::LayeredCostmap master("map", false, false);
  master.resizeMap(kCells, kCells, kRes, kOriginX, kOriginY);
  auto layer = std::make_shared<predictive_nav_costmap::PredictedObstacleLayer>();
  master.getPlugins()->push_back(layer);
  layer->initialize(&master, "r", &tf, node, group);

  auto pub = node->create_publisher<Array>("/stage5c1_raster_tracks", 10);
  pub->on_activate();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  // --- the discretization convention, asserted against production ---------
  {
    double wx = 0.0;
    double wy = 0.0;
    layer->mapToWorld(7u, 11u, wx, wy);
    const double expect_x = kOriginX + (7 + 0.5) * kRes;
    const double expect_y = kOriginY + (11 + 0.5) * kRes;
    check(
      std::abs(wx - expect_x) < 1e-12 && std::abs(wy - expect_y) < 1e-12,
      "mapToWorld is not origin + (index + 0.5) * resolution -- the documented "
      "cell-centre discretization rule does not hold");
  }

  // Publish one track carrying exactly one reachability region, then rebuild.
  auto render = [&](const std::vector<Region> & regions) {
      Array a;
      a.header.frame_id = "map";
      a.header.stamp = node->now();
      if (!regions.empty()) {
        Object t;
        t.id = 1;
        t.kalman_initialized = true;
        t.stamp = a.header.stamp;
        t.observations = 10;
        t.age = 10;
        t.reachability_predictions = regions;
        a.tracks.push_back(t);
      }
      for (int i = 0; i < 8; ++i) {
        pub->publish(a);
        std::this_thread::sleep_for(std::chrono::milliseconds(8));
        executor.spin_some();
      }
      master.updateMap(0.0, 0.0, 0.0);
    };

  // The layer IS a Costmap2D; reading its own grid isolates the rasterizer from
  // master-combination semantics entirely.
  auto layer_grid = [&]() {
      return std::vector<unsigned char>(
        layer->getCharMap(),
        layer->getCharMap() + static_cast<size_t>(kCells) * kCells);
    };

  // Determine the unpainted sentinel empirically rather than assuming it.
  render({});
  const std::vector<unsigned char> empty_grid = layer_grid();
  const unsigned char kUnpainted = empty_grid[0];
  {
    size_t non_default = 0;
    for (const auto c : empty_grid) {
      if (c != kUnpainted) {++non_default;}
    }
    check(non_default == 0, "an empty track array left painted cells in the layer grid");
  }

  auto one_region = [](const Case & k) {
      Region r;
      r.time_from_now = 1.0;
      r.observation_age = 0.0;
      r.total_time = 1.0;
      r.position.x = k.cx;
      r.position.y = k.cy;
      r.semi_major = k.a;
      r.semi_minor = k.b;
      r.sigma_yaw = k.yaw;
      r.reach_radius = k.a;            // provenance only; not read by the layer
      r.valid = true;
      r.contract_status = Region::CONTRACT_SATISFIED;
      r.configured_max_speed = 0.8;
      r.effective_max_speed = 0.8;
      return r;
    };

  const std::vector<Case> cases = {
    // name                         cx     cy     a      b     yaw   influence  clamp?
    {"fully_inside_no_clamp", 0.0, 0.0, 0.60, 0.40, 0.0, 3.0, false},
    // centre on a cell CORNER: the degenerate window-alignment case, where the
    // +-0.55 m policy bounds land exactly on cell boundaries and the window is
    // 24 cells wide rather than the generic 23.
    {"clamped_centre_on_cell_corner", 0.0, 0.0, 2.05, 2.05, 0.0, 0.55, true},
    // centre on a cell CENTRE: the generic alignment, and the one that
    // reproduces the Stage-5C production figures exactly.
    {"clamped_centre_on_cell_centre", 0.025, 0.025, 2.05, 2.05, 0.0, 0.55, true},
    {"rotated_clamped", 0.0, 0.0, 2.50, 1.00, 0.6, 0.55, true},
    {"rotated_no_clamp", 1.0, 1.0, 0.80, 0.30, 0.9, 3.0, false},
    {"highly_eccentric_no_clamp", 0.0, -2.0, 2.00, 0.08, 0.3, 3.0, false},
    {"highly_eccentric_clamped", 0.0, -2.0, 2.00, 0.08, 0.3, 0.55, true},
    {"small_sub_cell_ellipse", 0.3, 0.3, 0.06, 0.04, 0.0, 3.0, false},
    {"large_axes_influence_off", 0.0, 0.0, 5.00, 5.00, 0.0, 0.0, false},
    {"centre_near_map_edge", 9.90, 0.0, 0.60, 0.60, 0.0, 3.0, false},
    {"partially_outside_map", -9.80, -9.80, 1.50, 1.00, 0.2, 3.0, false},
    {"centre_outside_map", 10.40, 0.0, 1.00, 1.00, 0.0, 3.0, false},
  };

  std::printf("{\n");
  std::printf("  \"stage\": \"5C.1\",\n");
  std::printf("  \"subject\": \"reachability costmap rasterization fidelity\",\n");
  std::printf(
    "  \"map\": {\"cells_x\": %u, \"cells_y\": %u, \"resolution_m\": %.2f, "
    "\"origin_m\": [%.1f, %.1f]},\n", kCells, kCells, kRes, kOriginX, kOriginY);
  std::printf(
    "  \"discretization_rule\": \"a cell is painted iff its CENTRE lies inside the closed "
    "region; centre = origin + (index + 0.5) * resolution\",\n");
  std::printf("  \"cases\": [\n");

  size_t total_false_negatives_before = 0;
  size_t total_false_negatives_after = 0;
  size_t total_false_positives_after = 0;

  for (size_t ci = 0; ci < cases.size(); ++ci) {
    const Case & k = cases[ci];
    node->set_parameter(rclcpp::Parameter("r.max_influence_radius", k.influence));
    executor.spin_some();
    render({one_region(k)});
    const std::vector<unsigned char> grid = layer_grid();

    // ---- the two independent reference sets ------------------------------
    const double c = std::cos(k.yaw);
    const double s = std::sin(k.yaw);
    // True bounding half-extents of the rotated ellipse as published.
    const double true_half_x = std::hypot(k.a * c, k.b * s);
    const double true_half_y = std::hypot(k.a * s, k.b * c);
    // NEW: the policy bound clips the WINDOW.
    double win_half_x = true_half_x;
    double win_half_y = true_half_y;
    if (k.influence > 0.0) {
      win_half_x = std::min(win_half_x, k.influence);
      win_half_y = std::min(win_half_y, k.influence);
    }
    // OLD: the policy bound clipped the AXES, and the window followed from the
    // already-clamped axes.
    double old_a = k.a;
    double old_b = k.b;
    if (k.influence > 0.0) {
      old_a = std::min(old_a, k.influence);
      old_b = std::min(old_b, k.influence);
    }
    const double old_half_x = std::hypot(old_a * c, old_b * s);
    const double old_half_y = std::hypot(old_a * s, old_b * c);

    const bool clamp_binds = (k.influence > 0.0) &&
      (true_half_x > k.influence + 1e-12 || true_half_y > k.influence + 1e-12);
    check(
      clamp_binds == k.expect_clamp_binds,
      std::string("case '") + k.name + "': clamp-binding expectation is wrong");

    // Window in map indices. Computed by the test's own formula AND verified
    // against production's worldToMapEnforceBounds, so a drift in either is
    // caught.
    auto window = [&](double half_x, double half_y, int & lo_i, int & lo_j,
      int & hi_i, int & hi_j) {
        auto clampIdx = [&](double w, double origin) {
            const int raw = static_cast<int>(std::floor((w - origin) / kRes));
            return std::clamp(raw, 0, static_cast<int>(kCells) - 1);
          };
        lo_i = clampIdx(k.cx - half_x, kOriginX);
        lo_j = clampIdx(k.cy - half_y, kOriginY);
        hi_i = clampIdx(k.cx + half_x, kOriginX);
        hi_j = clampIdx(k.cy + half_y, kOriginY);
      };
    int lo_i = 0, lo_j = 0, hi_i = 0, hi_j = 0;
    window(win_half_x, win_half_y, lo_i, lo_j, hi_i, hi_j);
    {
      int p_lo_i = 0, p_lo_j = 0, p_hi_i = 0, p_hi_j = 0;
      layer->worldToMapEnforceBounds(k.cx - win_half_x, k.cy - win_half_y, p_lo_i, p_lo_j);
      layer->worldToMapEnforceBounds(k.cx + win_half_x, k.cy + win_half_y, p_hi_i, p_hi_j);
      check(
        p_lo_i == lo_i && p_lo_j == lo_j && p_hi_i == hi_i && p_hi_j == hi_j,
        std::string("case '") + k.name +
        "': the test's window formula disagrees with production worldToMapEnforceBounds");
      // No out-of-bounds indices can reach a write.
      check(
        p_lo_i >= 0 && p_lo_j >= 0 && p_hi_i < static_cast<int>(kCells) &&
        p_hi_j < static_cast<int>(kCells),
        std::string("case '") + k.name + "': production window escaped the map");
    }
    int o_lo_i = 0, o_lo_j = 0, o_hi_i = 0, o_hi_j = 0;
    window(old_half_x, old_half_y, o_lo_i, o_lo_j, o_hi_i, o_hi_j);

    size_t painted = 0, new_ref = 0, old_ref = 0;
    size_t false_neg_after = 0, false_pos_after = 0, false_neg_before = 0;
    for (unsigned int j = 0; j < kCells; ++j) {
      for (unsigned int i = 0; i < kCells; ++i) {
        const double wx = kOriginX + (i + 0.5) * kRes;
        const double wy = kOriginY + (j + 0.5) * kRes;
        const bool is_painted = grid[static_cast<size_t>(j) * kCells + i] != kUnpainted;

        const bool in_window = static_cast<int>(i) >= lo_i && static_cast<int>(i) <= hi_i &&
          static_cast<int>(j) >= lo_j && static_cast<int>(j) <= hi_j;
        const bool in_new = in_window && inEllipse(wx, wy, k.cx, k.cy, k.a, k.b, k.yaw);

        const bool in_old_window = static_cast<int>(i) >= o_lo_i &&
          static_cast<int>(i) <= o_hi_i && static_cast<int>(j) >= o_lo_j &&
          static_cast<int>(j) <= o_hi_j;
        const bool in_old = in_old_window &&
          inEllipse(wx, wy, k.cx, k.cy, old_a, old_b, k.yaw);

        if (is_painted) {++painted;}
        if (in_new) {++new_ref;}
        if (in_old) {++old_ref;}
        if (in_new && !is_painted) {++false_neg_after;}
        if (is_painted && !in_new) {++false_pos_after;}
        // A cell inside the qualified region and inside the window that the
        // OLD rasterizer dropped: a false negative before the fix.
        if (in_new && !in_old) {++false_neg_before;}
        // Invariant S: nothing outside the published ellipse may ever be
        // painted, regardless of windowing.
        if (is_painted) {
          check(
            inEllipse(wx, wy, k.cx, k.cy, k.a, k.b, k.yaw),
            std::string("case '") + k.name +
            "': a cell outside the published ellipse was painted");
        }
      }
    }

    check(
      false_neg_after == 0,
      std::string("case '") + k.name +
      "': cells inside the qualified region and inside the window were NOT painted");
    check(
      false_pos_after == 0,
      std::string("case '") + k.name + "': cells outside the region were painted");
    check(
      painted == new_ref,
      std::string("case '") + k.name + "': painted set does not equal the new reference");

    total_false_negatives_before += false_neg_before;
    total_false_negatives_after += false_neg_after;
    total_false_positives_after += false_pos_after;

    const double cell_area = kRes * kRes;
    std::printf(
      "    {\"name\": \"%s\", \"centre_m\": [%.2f, %.2f], \"semi_axes_m\": [%.3f, %.3f], "
      "\"yaw_rad\": %.3f, \"max_influence_radius_m\": %.2f, \"clamp_binds\": %s,\n"
      "     \"model_ellipse_area_m2\": %.4f, \"window_half_extents_m\": [%.4f, %.4f], "
      "\"window_indices\": [%d, %d, %d, %d],\n"
      "     \"painted_cells\": %zu, \"expected_cells\": %zu, \"rasterized_area_m2\": %.4f, "
      "\"cells_before_fix\": %zu,\n"
      "     \"false_negative_cells_before_fix\": %zu, \"false_negative_cells_after_fix\": %zu, "
      "\"false_positive_cells\": %zu}%s\n",
      k.name, k.cx, k.cy, k.a, k.b, k.yaw, k.influence, clamp_binds ? "true" : "false",
      M_PI * k.a * k.b, win_half_x, win_half_y, lo_i, lo_j, hi_i, hi_j,
      painted, new_ref, static_cast<double>(painted) * cell_area, old_ref,
      false_neg_before, false_neg_after, false_pos_after,
      ci + 1 < cases.size() ? "," : "");
  }
  std::printf("  ],\n");

  // =====================================================================
  // Degenerate geometry must paint nothing, not crash or paint garbage
  // =====================================================================
  {
    node->set_parameter(rclcpp::Parameter("r.max_influence_radius", 0.55));
    executor.spin_some();
    struct Bad {const char * name; double a; double b; bool valid;};
    const std::vector<Bad> bad = {
      {"zero_semi_major", 0.0, 0.5, true},
      {"zero_semi_minor", 0.5, 0.0, true},
      {"negative_axis", -0.5, 0.5, true},
      {"nan_axis", std::numeric_limits<double>::quiet_NaN(), 0.5, true},
      {"inf_axis", std::numeric_limits<double>::infinity(), 0.5, true},
      {"valid_false", 0.8, 0.8, false},
    };
    size_t clean = 0;
    for (const Bad & b : bad) {
      Case k{b.name, 0.0, 0.0, b.a, b.b, 0.0, 0.55, false};
      Region r = one_region(k);
      r.valid = b.valid;
      render({r});
      const std::vector<unsigned char> grid = layer_grid();
      size_t painted = 0;
      for (const auto cell : grid) {
        if (cell != kUnpainted) {++painted;}
      }
      check(painted == 0, std::string("degenerate case '") + b.name + "' painted cells");
      if (painted == 0) {++clean;}
    }
    std::printf(
      "  \"degenerate_geometry\": {\"cases\": %zu, \"painted_nothing\": %zu},\n",
      bad.size(), clean);
  }

  // =====================================================================
  // The Stage-5C anomaly, before and after, at production geometry
  // =====================================================================
  {
    node->set_parameter(rclcpp::Parameter("r.max_influence_radius", 0.55));
    executor.spin_some();
    // Semi-axes well above the 0.55 m policy bound -- the situation at every
    // horizon from 1.5 s on in production. The centre is placed on a CELL
    // CENTRE, which is the generic alignment and the one the Stage-5C run
    // recorded: the +-0.55 m bounds then fall strictly inside cell boundaries
    // and the window is 1.1/0.05 + 1 = 23 cells on a side.
    Case k{"anomaly", 0.025, 0.025, 2.05, 2.05, 0.0, 0.55, true};
    render({one_region(k)});
    const std::vector<unsigned char> grid = layer_grid();

    // Both references computed from the REAL grid, with no symmetry assumed:
    //   window  -- the policy window (what the corrected rasterizer must fill
    //              here, since the region strictly contains the window);
    //   disc    -- the window intersected with the CLAMPED ellipse, i.e. the
    //              pre-Stage-5C.1 behaviour.
    int lo_i = 0, lo_j = 0, hi_i = 0, hi_j = 0;
    layer->worldToMapEnforceBounds(k.cx - 0.55, k.cy - 0.55, lo_i, lo_j);
    layer->worldToMapEnforceBounds(k.cx + 0.55, k.cy + 0.55, hi_i, hi_j);
    // `disc` uses the same strict `<= 1` test the rasterizer uses. `disc_closed`
    // admits an exact-tangency tolerance. They differ by precisely the cells
    // whose centre lies at distance EXACTLY max_influence_radius from the
    // centre -- here the four axis-aligned ones at 0.55 m, which land on the
    // boundary to within one ulp. Both are reported, because which side of a
    // floating-point tie those four fall on is not a meaningful invariant and
    // the test must not assert one.
    size_t painted = 0, window_cells = 0, disc = 0, disc_closed = 0;
    for (unsigned int j = 0; j < kCells; ++j) {
      for (unsigned int i = 0; i < kCells; ++i) {
        if (grid[static_cast<size_t>(j) * kCells + i] != kUnpainted) {++painted;}
        if (static_cast<int>(i) < lo_i || static_cast<int>(i) > hi_i ||
          static_cast<int>(j) < lo_j || static_cast<int>(j) > hi_j)
        {
          continue;
        }
        ++window_cells;
        const double wx = kOriginX + (i + 0.5) * kRes;
        const double wy = kOriginY + (j + 0.5) * kRes;
        if (inEllipse(wx, wy, k.cx, k.cy, 0.55, 0.55, 0.0)) {++disc;}
        const double dx = wx - k.cx;
        const double dy = wy - k.cy;
        if (dx * dx + dy * dy <= 0.55 * 0.55 * (1.0 + 1e-9)) {++disc_closed;}
      }
    }
    const int side = hi_i - lo_i + 1;
    check(side == 23, "the production policy window is not 23 cells on a side");
    check(
      window_cells == 529,
      "the policy window is not the 529 cells Stage-5C recorded for CV");
    check(
      disc_closed == 377,
      "the inscribed disc, with exact-tangency cells admitted, is not the 377 cells "
      "Stage-5C recorded for reachability");
    check(
      disc == 373 && disc_closed - disc == 4,
      "the strict/closed disc counts do not differ by exactly the four "
      "axis-aligned tangency cells");
    check(
      painted == window_cells,
      "the corrected rasterizer did not fill the whole policy window where the "
      "region exceeds it");
    check(
      painted > disc,
      "the corrected rasterizer still paints no more than the inscribed disc");
    const size_t square = window_cells;
    std::printf(
      "  \"anomaly_reproduction\": {\"max_influence_radius_m\": 0.55, \"resolution_m\": %.2f,\n"
      "    \"model_ellipse_area_m2\": %.4f, \"policy_window_side_cells\": %d, "
      "\"policy_window_cells\": %zu,\n"
      "    \"before_fix_cells_inscribed_disc_strict\": %zu, "
      "\"before_fix_cells_inscribed_disc_closed\": %zu, \"after_fix_cells\": %zu,\n"
      "    \"before_fix_ratio_to_window\": %.4f, \"after_fix_ratio_to_window\": %.4f,\n"
      "    \"stage5c_recorded_cv_cells\": 529.0, \"stage5c_recorded_reach_cells\": 377.0,\n"
      "    \"stage5c_recorded_cell_ratio\": 0.713, \"reproduced_cell_ratio\": %.4f,\n"
      "    \"tangency_cells_between_strict_and_closed\": %zu},\n",
      kRes, M_PI * k.a * k.b, side, square, disc, disc_closed, painted,
      static_cast<double>(disc_closed) / static_cast<double>(square),
      static_cast<double>(painted) / static_cast<double>(square),
      static_cast<double>(disc_closed) / static_cast<double>(square),
      disc_closed - disc);
  }

  // =====================================================================
  // Cost of the fidelity fix
  // =====================================================================
  //
  // Six horizons at production geometry -- exactly what the tracker publishes
  // -- rebuilt repeatedly with the track latched. Reported as wall time for a
  // full master rebuild, which INCLUDES rasterization; the layer's own
  // updateCosts figure is captured separately in the Gazebo arm so it can be
  // compared like-for-like with the Stage-5C baseline.
  {
    node->set_parameter(rclcpp::Parameter("r.max_influence_radius", 0.55));
    node->set_parameter(rclcpp::Parameter("r.max_prediction_horizon", 3.0));
    executor.spin_some();
    std::vector<Region> six;
    for (int i = 1; i <= 6; ++i) {
      Case k{"h", 0.0, 0.0, 0.35 * i + 0.2, 0.35 * i + 0.2, 0.0, 0.55, true};
      Region r = one_region(k);
      r.time_from_now = 0.5 * i;
      r.total_time = 0.5 * i;
      six.push_back(r);
    }
    render(six);
    const int kIters = 400;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kIters; ++i) {
      master.updateMap(0.0, 0.0, 0.0);
    }
    const double per_update_us =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() /
      kIters;
    size_t painted = 0;
    for (const auto cell : layer_grid()) {
      if (cell != kUnpainted) {++painted;}
    }
    std::printf(
      "  \"performance\": {\"samples_per_track\": 6, \"painted_cells\": %zu, "
      "\"full_master_rebuild_us\": %.1f, \"iterations\": %d},\n",
      painted, per_update_us, kIters);
  }

  std::printf(
    "  \"totals\": {\"false_negative_cells_before_fix\": %zu, "
    "\"false_negative_cells_after_fix\": %zu, \"false_positive_cells\": %zu},\n",
    total_false_negatives_before, total_false_negatives_after, total_false_positives_after);
  std::printf("  \"failures\": %d,\n", failures);
  std::printf("  \"pass\": %s\n", failures == 0 ? "true" : "false");
  std::printf("}\n");

  rclcpp::shutdown();
  if (failures != 0) {
    std::fprintf(stderr, "stage5c1_raster_fidelity: %d check(s) failed\n", failures);
    return 1;
  }
  return 0;
}
