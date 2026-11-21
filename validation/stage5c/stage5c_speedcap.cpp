// Stage-5C -- focused probe of the ONE model-assumption boundary found in the
// Stage-4G4 reachability bound.
//
// The bound r_det(t) is derived from TWO premises:
//     |a(s)| <= max_acceleration      and      |v(s)| <= max_speed
// The second premise is what makes |v(s) - v0| <= max_speed + s0 valid. If the
// PHYSICAL obstacle is faster than max_speed, that premise fails and the
// published region is no longer an outer bound. Nothing in reachability.hpp or
// the tracker detects this: max_speed is a configured constant, never compared
// against the track's own measured speed.
//
// This probe measures how large the resulting containment violation is as a
// function of the obstacle's true speed, for the frozen a_max = 0.5 m/s^2.
//
// NOTE on a DIFFERENT and benign effect, measured separately below: an
// OVER-estimated s0 (filtered speed above the true speed) only ENLARGES the
// bound, because r_det is monotone increasing in s0. Estimation overshoot is
// therefore conservative and is not a defect.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include <Eigen/Dense>

#include "predictive_nav_tracking/reachability.hpp"

using predictive_nav_tracking::ReachabilityBounds;
using predictive_nav_tracking::reachRadius;

namespace
{
// Admissible-under-reality trajectory: respects |a| <= a_max and a TRUE speed
// ceiling of `true_v_max`, which may exceed the configured bounds.max_speed.
double deviation(
  double v0x, double v0y, double ax, double ay, double t, double a_max,
  double true_v_max, double dt = 1e-5)
{
  double px = 0, py = 0, vx = v0x, vy = v0y;
  const double an = std::hypot(ax, ay);
  if (an > a_max) {
    ax *= a_max / an;
    ay *= a_max / an;
  }
  for (double s = 0; s < t; s += dt) {
    const double step = std::min(dt, t - s);
    const double vxb = vx, vyb = vy;
    vx += ax * step;
    vy += ay * step;
    const double sp = std::hypot(vx, vy);
    if (sp > true_v_max) {
      vx *= true_v_max / sp;
      vy *= true_v_max / sp;
    }
    px += 0.5 * (vxb + vx) * step;
    py += 0.5 * (vyb + vy) * step;
  }
  return std::hypot(px - v0x * t, py - v0y * t);
}
}  // namespace

int main()
{
  ReachabilityBounds B;
  B.max_acceleration = 0.5;
  B.max_speed = 0.8;          // frozen production value
  B.covariance_sigma_level = 2.0;
  B.safety_margin = 0.0;

  std::printf("{\n  \"configured_max_speed_mps\": %.2f,\n", B.max_speed);
  std::printf("  \"configured_max_acceleration_mps2\": %.2f,\n", B.max_acceleration);
  std::printf("  \"note\": \"escape_m is how far an admissible trajectory gets OUTSIDE "
    "the published deterministic bound; 0 means the bound held\",\n");
  std::printf("  \"sweep\": [\n");

  // Reference speeds: sim suite max 0.75; Stage-5B classes 0.25/0.50/0.75;
  // real pedestrian ~1.4; real cyclist ~4.2 (15 km/h).
  const std::vector<std::pair<double, const char *>> speeds = {
    {0.68, "stage4g sim max (g7/g8 scenarios)"},
    {0.75, "stage5b fastest commanded speed"},
    {0.80, "exactly at the configured cap"},
    {0.90, "12% over cap"},
    {1.10, "slow walk, real world"},
    {1.40, "typical pedestrian, real world"},
    {2.00, "fast walk / jog"},
    {3.00, "slow cyclist"},
    {4.20, "cyclist 15 km/h"},
  };

  std::mt19937 rng(20261121);
  std::uniform_real_distribution<double> angle(-M_PI, M_PI);

  for (size_t i = 0; i < speeds.size(); ++i) {
    const double v_true = speeds[i].first;
    double worst_escape = 0.0;
    double worst_t = 0.0;
    long escapes = 0, trials = 0;
    // escape at each published horizon
    std::vector<double> per_h(6, 0.0);
    for (int hi = 0; hi < 6; ++hi) {
      const double t = (hi + 1) * 0.5;
      double worst_h = 0.0;
      for (int k = 0; k < 400; ++k) {
        const double hd = angle(rng);
        const double ad = angle(rng);
        // Start at the true speed; the adversary accelerates in any direction.
        const double dev = deviation(
          v_true * std::cos(hd), v_true * std::sin(hd),
          B.max_acceleration * std::cos(ad), B.max_acceleration * std::sin(ad),
          t, B.max_acceleration, v_true);
        const double bound = reachRadius(t, v_true, B);
        const double esc = dev - bound;
        ++trials;
        if (esc > 1e-6) {
          ++escapes;
          worst_h = std::max(worst_h, esc);
          if (esc > worst_escape) {
            worst_escape = esc;
            worst_t = t;
          }
        }
      }
      per_h[hi] = worst_h;
    }
    std::printf("    {\"true_speed_mps\":%.2f,\"label\":\"%s\",\"trials\":%ld,"
      "\"escape_rate\":%.4f,\"worst_escape_m\":%.4f,\"worst_escape_at_t_s\":%.1f,"
      "\"worst_escape_by_horizon_m\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f]}%s\n",
      v_true, speeds[i].second, trials, static_cast<double>(escapes) / trials,
      worst_escape, worst_t, per_h[0], per_h[1], per_h[2], per_h[3], per_h[4], per_h[5],
      (i + 1 == speeds.size()) ? "" : ",");
  }
  std::printf("  ],\n");

  // --- the benign effect: monotonicity in s0 ------------------------------
  bool mono_s0 = true;
  for (double s0 = 0.0; s0 < 5.0; s0 += 0.01) {
    for (double t : {0.5, 1.0, 2.0, 3.0}) {
      if (reachRadius(t, s0 + 0.01, B) < reachRadius(t, s0, B) - 1e-15) {
        mono_s0 = false;
      }
    }
  }
  std::printf("  \"r_det_monotone_increasing_in_s0\": %s,\n", mono_s0 ? "true" : "false");
  std::printf("  \"implication\": \"overestimated filtered speed only enlarges the "
    "bound, so estimation overshoot is conservative, not a defect\",\n");

  // --- what max_speed WOULD be needed for the real-world classes ----------
  std::printf("  \"min_max_speed_for_validity\": [\n");
  for (size_t i = 0; i < speeds.size(); ++i) {
    std::printf("    {\"true_speed_mps\":%.2f,\"required_max_speed_mps\":%.2f,"
      "\"r_det_at_1s_configured_m\":%.4f,\"r_det_at_1s_required_m\":%.4f}%s\n",
      speeds[i].first, speeds[i].first,
      reachRadius(1.0, speeds[i].first, B),
      [&] {
        ReachabilityBounds R = B;
        R.max_speed = speeds[i].first;
        return reachRadius(1.0, speeds[i].first, R);
      }(),
      (i + 1 == speeds.size()) ? "" : ",");
  }
  std::printf("  ]\n}\n");
  return 0;
}
