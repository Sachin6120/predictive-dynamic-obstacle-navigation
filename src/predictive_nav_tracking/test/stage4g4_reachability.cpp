// Stage-4G4 reachability regression, against the production translation unit.
//
// Covers the three things that could make the reachable set dishonest:
//   1. the bound is not actually a bound (a physically admissible trajectory
//      escapes it),
//   2. the speed cap is ignored, so the region licenses motion the obstacle
//      cannot perform,
//   3. observation age is not paid for, so a coasting track is published as
//      though it had just been measured.
// It also pins that the Stage-4C CV prediction is bit-identical whether
// reachability is on or off, and that reachability state is per-track.
//
// Ground truth is never used to PRODUCE a region: the simulated trajectories
// below exist only to check containment of regions built from track state.
#define main stage4g4_original_main
#include "../src/lidar_obstacle_tracker_node.cpp"
#undef main
#include <cstdio>
#include <iostream>
#include <random>
#include <stdexcept>

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

namespace
{
using predictive_nav_tracking::ReachabilityBounds;
using predictive_nav_tracking::buildRegion;
using predictive_nav_tracking::reachRadius;

/// Forward-simulates an admissible trajectory under |a| <= a_max, |v| <= v_max
/// and returns the deviation from the constant-velocity nominal at time t.
/// This is the adversary: it is allowed to do anything the bounds permit.
double simulated_deviation(
  double v0x, double v0y, double ax, double ay, double t,
  const ReachabilityBounds & bounds, double dt = 1e-4)
{
  double px = 0.0, py = 0.0, vx = v0x, vy = v0y;
  const double a_norm = std::hypot(ax, ay);
  if (a_norm > bounds.max_acceleration) {          // keep the adversary legal
    ax *= bounds.max_acceleration / a_norm;
    ay *= bounds.max_acceleration / a_norm;
  }
  for (double s = 0.0; s < t; s += dt) {
    const double step = std::min(dt, t - s);
    const double vx_before = vx;
    const double vy_before = vy;
    vx += ax * step;
    vy += ay * step;
    const double speed = std::hypot(vx, vy);
    if (speed > bounds.max_speed) {                // enforce the speed cap
      vx *= bounds.max_speed / speed;
      vy *= bounds.max_speed / speed;
    }
    // Midpoint (trapezoidal) position update. Forward Euler here is only
    // first-order and its O(a*dt*t) error is larger than the slack between an
    // extremal trajectory and the bound, which would produce a spurious
    // failure at the crossover time rather than a real one.
    px += 0.5 * (vx_before + vx) * step;
    py += 0.5 * (vy_before + vy) * step;
  }
  return std::hypot(px - v0x * t, py - v0y * t);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarObstacleTracker>();

  ReachabilityBounds bounds;
  bounds.max_acceleration = 0.5;
  bounds.max_speed = 0.8;
  bounds.covariance_sigma_level = 2.0;
  bounds.safety_margin = 0.0;

  // ---- 1. The bound actually bounds admissible motion -------------------
  // Randomized adversarial search over start speeds, acceleration directions
  // and horizons. Every escape would be a real safety defect.
  size_t containment_trials = 0;
  double worst_slack = std::numeric_limits<double>::infinity();
  {
    std::mt19937 rng(20260914);
    std::uniform_real_distribution<double> speed(0.0, bounds.max_speed);
    std::uniform_real_distribution<double> angle(-M_PI, M_PI);
    std::uniform_real_distribution<double> horizon(0.1, 4.0);
    for (int i = 0; i < 3000; ++i) {
      const double s0 = speed(rng);
      const double heading = angle(rng);
      const double v0x = s0 * std::cos(heading);
      const double v0y = s0 * std::sin(heading);
      const double a_dir = angle(rng);
      const double t = horizon(rng);
      const double deviation = simulated_deviation(
        v0x, v0y, bounds.max_acceleration * std::cos(a_dir),
        bounds.max_acceleration * std::sin(a_dir), t, bounds);
      const double bound = reachRadius(t, s0, bounds);
      // 1e-8 absorbs the midpoint integrator's residual error. The extremal
      // (straight-line reversal) trajectory meets the bound exactly below the
      // crossover time, so this tolerance is genuinely tight.
      require(deviation <= bound + 1e-8, "an admissible trajectory escaped the reachable bound");
      worst_slack = std::min(worst_slack, bound - deviation);
      ++containment_trials;
    }
  }

  // ---- 2. The speed cap binds, and only ever shrinks the region ----------
  {
    // Free bounded acceleration would give 0.5*a*t^2 = 0.5*0.5*16 = 4.0 m at
    // t = 4 s; the cap must produce strictly less.
    const double capped = reachRadius(4.0, 0.0, bounds);
    require(capped < 0.5 * bounds.max_acceleration * 16.0, "speed cap did not bind at t=4s");
    bool flagged = false;
    reachRadius(4.0, 0.0, bounds, &flagged);
    require(flagged, "speed_capped flag not raised when the cap bound the radius");

    ReachabilityBounds uncapped = bounds;
    uncapped.max_speed = 1e6;
    require(
      reachRadius(4.0, 0.0, uncapped) > capped,
      "the speed cap grew the region instead of shrinking it");
    // Below the crossover the closed form must be exactly 0.5*a*t^2.
    require(
      std::abs(reachRadius(0.5, 0.0, bounds) - 0.5 * 0.5 * 0.25) < 1e-12,
      "sub-crossover radius is not 0.5*a_max*t^2");
    // A zero-length horizon is a point, and a_max = 0 removes the term.
    require(reachRadius(0.0, 0.3, bounds) == 0.0, "zero horizon produced a non-zero radius");
    ReachabilityBounds inert = bounds;
    inert.max_acceleration = 0.0;
    require(reachRadius(3.0, 0.3, inert) == 0.0, "a_max = 0 produced a non-zero radius");
  }

  // ---- 3. Monotonicity in time and in the acceleration bound -------------
  {
    double previous = -1.0;
    for (double t = 0.0; t <= 3.0; t += 0.25) {
      const double r = reachRadius(t, 0.3, bounds);
      require(r >= previous - 1e-12, "reach radius is not monotone in time");
      previous = r;
    }
    ReachabilityBounds loose = bounds;
    loose.max_acceleration = 1.0;
    require(
      reachRadius(2.0, 0.3, loose) > reachRadius(2.0, 0.3, bounds),
      "a larger acceleration bound did not produce a larger region");
  }

  // ---- 4. Deterministic and statistical terms stay separable -------------
  {
    Eigen::Matrix2d P;
    P << 0.04, 0.0, 0.0, 0.01;                    // 0.2 m / 0.1 m sigma
    const auto region = buildRegion(1.0, 2.0, 0.3, 0.0, P, 2.0, bounds);
    require(region.valid, "region should be valid");
    require(std::abs(region.center_x - 1.6) < 1e-12, "centre is not p0 + v0*t");
    require(std::abs(region.center_y - 2.0) < 1e-12, "centre is not p0 + v0*t");
    require(
      std::abs(region.sigma_semi_major - 2.0 * 0.2) < 1e-9,
      "statistical semi-major is not k*sqrt(lambda_max)");
    require(
      std::abs(region.sigma_semi_minor - 2.0 * 0.1) < 1e-9,
      "statistical semi-minor is not k*sqrt(lambda_min)");
    require(
      std::abs(
        region.semi_major -
        (region.sigma_semi_major + region.reach_radius + region.safety_margin)) < 1e-12,
      "published semi-major is not the sum of its declared parts");
    // With a_max = 0 the region must collapse EXACTLY onto the CV sigma
    // ellipse: no hidden inflation anywhere in the deterministic path.
    ReachabilityBounds inert = bounds;
    inert.max_acceleration = 0.0;
    const auto cv_only = buildRegion(1.0, 2.0, 0.3, 0.0, P, 2.0, inert);
    require(
      std::abs(cv_only.semi_major - 2.0 * 0.2) < 1e-9 &&
      std::abs(cv_only.semi_minor - 2.0 * 0.1) < 1e-9,
      "a_max = 0 did not collapse the region onto the CV sigma ellipse");
  }

  // ---- 5. Observation age is paid for, on a genuinely coasting track -----
  // Build a real track through the production association path, then stop
  // feeding it and watch what the node publishes.
  double age_paid = 0.0;
  double fresh_radius = 0.0;
  double coasting_radius = 0.0;
  {
    const auto time = [](double t) {
        return rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
      };
    node->tracks_.clear();
    for (int i = 0; i < 6; ++i) {
      node->associate_and_update({{0.3 * 0.2 * i, 0.0}}, time(10.0 + 0.2 * i));
    }
    require(node->tracks_.size() == 1, "setup should hold exactly one track");
    require(node->tracks_[0].kalman_initialized, "track should be Kalman-initialized");

    const auto fresh = node->predict_reachability(node->tracks_[0], time(11.0));
    require(!fresh.empty(), "no reachability samples were produced");
    require(
      std::abs(fresh.front().total_time - 0.5) < 1e-9,
      "a freshly observed track must have total_time == horizon");
    fresh_radius = fresh.front().reach_radius;

    // Same track, same stored state, but 0.8 s of unobserved time.
    const auto stale = node->predict_reachability(node->tracks_[0], time(11.8));
    require(
      std::abs(stale.front().total_time - 1.3) < 1e-9,
      "observation age was not added to the horizon");
    coasting_radius = stale.front().reach_radius;
    require(
      coasting_radius > fresh_radius,
      "a coasting track's region did not grow with observation age");
    age_paid = 0.8;

    // The track's own stored state must be untouched by prediction.
    const auto before = node->tracks_[0].kf_x;
    const auto before_P = node->tracks_[0].kf_P;
    node->predict_reachability(node->tracks_[0], time(12.5));
    require(
      node->tracks_[0].kf_x == before && node->tracks_[0].kf_P == before_P,
      "predicting reachability mutated the live filter state");

    // Past max_observation_age the region is published invalid, not grown.
    const auto expired = node->predict_reachability(node->tracks_[0], time(12.6));
    require(!expired.empty(), "samples should still be emitted past the age limit");
    require(!expired.front().valid, "a region past max_observation_age was not marked invalid");
  }

  // ---- 6. The Stage-4C CV prediction is unaffected by reachability -------
  {
    const auto cv_on = node->predict_future(node->tracks_[0]);
    node->reachability_enabled_ = false;
    const auto cv_off = node->predict_future(node->tracks_[0]);
    const auto none = node->predict_reachability(
      node->tracks_[0], rclcpp::Time(static_cast<int64_t>(11.0 * 1e9), RCL_ROS_TIME));
    node->reachability_enabled_ = true;
    require(none.empty(), "reachability_enabled=false still produced samples");
    require(cv_on.size() == cv_off.size(), "CV prediction count changed with the mode");
    for (size_t i = 0; i < cv_on.size(); ++i) {
      require(
        cv_on[i].x == cv_off[i].x && cv_on[i].y == cv_off[i].y &&
        cv_on[i].vx == cv_off[i].vx && cv_on[i].vy == cv_off[i].vy &&
        cv_on[i].position_covariance == cv_off[i].position_covariance,
        "the Stage-4C CV prediction is not bit-identical with reachability enabled");
    }
  }

  // ---- 7. Multi-track independence: no shared reachability state ---------
  {
    const auto time = [](double t) {
        return rclcpp::Time(static_cast<int64_t>(t * 1e9), RCL_ROS_TIME);
      };
    node->tracks_.clear();
    for (int i = 0; i < 6; ++i) {
      const double t = 0.2 * i;
      node->associate_and_update(
        {{0.5 * t, -1.0}, {-0.2 * t, 1.0}, {0.0, 2.0 + 0.3 * t}}, time(20.0 + t));
    }
    require(node->tracks_.size() == 3, "setup should hold three tracks");

    // Feed only tracks 0 and 2 on the next scan, so track 1 coasts while the
    // others are freshly observed -- the mixed case Stage-4G4 requires.
    node->associate_and_update({{0.5 * 1.2, -1.0}, {0.0, 2.36}}, time(21.2));
    const size_t coasting = (node->tracks_[0].missed_count > 0) ? 0 :
      (node->tracks_[1].missed_count > 0 ? 1 : 2);
    require(node->tracks_[coasting].missed_count == 1, "expected exactly one coasting track");

    std::vector<std::vector<ReachabilityRegion>> regions;
    for (const auto & track : node->tracks_) {
      regions.push_back(node->predict_reachability(track, time(21.2)));
    }
    for (size_t i = 0; i < regions.size(); ++i) {
      require(!regions[i].empty(), "a track produced no reachability samples");
      for (size_t j = i + 1; j < regions.size(); ++j) {
        require(
          regions[i].front().center_x != regions[j].front().center_x ||
          regions[i].front().center_y != regions[j].front().center_y,
          "two tracks produced identical reachable centres (shared state)");
      }
    }
    // The coasting track -- and only it -- must have paid an observation age.
    for (size_t i = 0; i < regions.size(); ++i) {
      const double total = regions[i].front().total_time;
      if (i == coasting) {
        require(total > 0.5 + 1e-9, "the coasting track did not pay its observation age");
      } else {
        require(std::abs(total - 0.5) < 1e-9, "a freshly observed track paid an age it did not owe");
      }
    }
  }

  std::printf(
    "{\"pass\":true,\"containment_trials\":%zu,\"worst_containment_slack_m\":%.6f,"
    "\"speed_cap_binds\":true,\"monotone_in_time_and_a_max\":true,"
    "\"terms_separable\":true,\"a_max_zero_collapses_to_cv_ellipse\":true,"
    "\"observation_age_paid_s\":%.2f,\"fresh_reach_radius_m\":%.6f,"
    "\"coasting_reach_radius_m\":%.6f,\"live_state_untouched\":true,"
    "\"invalid_past_max_observation_age\":true,\"cv_prediction_bit_identical\":true,"
    "\"multi_track_independent\":true}\n",
    containment_trials, worst_slack, age_paid, fresh_radius, coasting_radius);
  rclcpp::shutdown();
  return 0;
}
