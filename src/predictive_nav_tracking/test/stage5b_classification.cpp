// Stage-5B: unit tests for obstacle classification.
//
// HOW THESE TESTS GENERATE INPUT
// ------------------------------
// Wherever a real object can express the case, the input is produced by
// RAY-CASTING a 2D LiDAR against a polygonal footprint: beams are swept at
// the sensor's own angular spacing, each beam returns its NEAREST hit and
// nothing else, and the configured range noise is added. That is what makes
// the fixtures trustworthy -- point count, spacing, angular span, solidity
// and self-occlusion all fall out of the geometry and the range, exactly as
// they do on the robot, instead of being chosen by hand. An earlier draft of
// these tests placed points directly on circular arcs; it produced two
// returns on one bearing and a fill_ratio of 1.63, which no LiDAR can report.
//
// Pure feature-vector inputs are used only where constructing a geometry
// would obscure the point: the evidence gates, the trapezoid itself, the
// class-box boundaries and the stabiliser.
//
// WHAT THEY ASSERT
// ----------------
// The contract, not an accuracy figure: that the features mean what they are
// documented to mean, that each class is recognised from its own physical
// signature, that UNKNOWN is preferred when evidence is thin or ambiguous,
// that confidence stays in [0, 1], and that the stabiliser cannot oscillate.
// Accuracy on simulator data is the job of the Gazebo validation set.

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "predictive_nav_tracking/obstacle_classification.hpp"

using predictive_nav_tracking::ClassBox;
using predictive_nav_tracking::ClassificationParams;
using predictive_nav_tracking::ClassificationResult;
using predictive_nav_tracking::ClassLabelStabilizer;
using predictive_nav_tracking::ClusterFeatures;
using predictive_nav_tracking::ObstacleClass;
using predictive_nav_tracking::TrackMotion;
using predictive_nav_tracking::classifyInstant;
using predictive_nav_tracking::extractFeatures;
using predictive_nav_tracking::toString;
using predictive_nav_tracking::trapezoid;

namespace
{

int g_failures = 0;
int g_checks = 0;

void require(bool ok, const std::string & what)
{
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
  }
}

struct P
{
  double x;
  double y;
};

struct Seg
{
  double ax, ay, bx, by;
};

constexpr double kInc = 2.0 * M_PI / 360.0;   // rad, TB3 LDS beam spacing
constexpr double kNoise = 0.01;               // m, 1-sigma range noise

/// Deterministic pseudo-random range noise. A fixed linear congruential
/// sequence, so every run of this test sees exactly the same scan.
class Jitter
{
public:
  double next()
  {
    state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
    const double u = static_cast<double>((state_ >> 11) & 0x1FFFFFULL) /
      static_cast<double>(0x200000ULL);              // [0, 1)
    return (2.0 * u - 1.0) * 1.732;                  // zero-mean, unit variance
  }

private:
  uint64_t state_{0x5B5Bu};
};

/// Sweep a full 2D LiDAR revolution against `segs` from (sx, sy): one return
/// per beam, nearest hit only, plus range noise.
std::vector<P> scan_world(
  const std::vector<Seg> & segs, double sx = 0.0, double sy = 0.0, bool noisy = true)
{
  Jitter jitter;
  std::vector<P> hits;
  const int n = static_cast<int>(std::lround(2.0 * M_PI / kInc));
  for (int i = 0; i < n; ++i) {
    const double t = -M_PI + i * kInc;
    const double dx = std::cos(t);
    const double dy = std::sin(t);
    double best = std::numeric_limits<double>::infinity();
    for (const auto & s : segs) {
      const double ex = s.bx - s.ax;
      const double ey = s.by - s.ay;
      const double ax = s.ax - sx;
      const double ay = s.ay - sy;
      const double det = ex * dy - dx * ey;
      if (std::abs(det) < 1e-12) {
        continue;
      }
      const double u = (-ax * ey + ex * ay) / det;   // distance along the beam
      const double v = (dx * ay - dy * ax) / det;    // position along the segment
      if (u > 1e-9 && v >= 0.0 && v <= 1.0 && u < best) {
        best = u;
      }
    }
    if (std::isfinite(best)) {
      const double r = noisy ? best + kNoise * jitter.next() : best;
      hits.push_back({sx + r * dx, sy + r * dy});
    }
  }
  return hits;
}

/// Rectangular footprint: centre (cx, cy), `len` along its own x axis, `wid`
/// along its y axis, rotated by `yaw`.
std::vector<Seg> rect(double cx, double cy, double len, double wid, double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hx = 0.5 * len;
  const double hy = 0.5 * wid;
  const double px[4] = {-hx, hx, hx, -hx};
  const double py[4] = {-hy, -hy, hy, hy};
  std::vector<Seg> out;
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    out.push_back(
      {cx + px[i] * c - py[i] * s, cy + px[i] * s + py[i] * c,
        cx + px[j] * c - py[j] * s, cy + px[j] * s + py[j] * c});
  }
  return out;
}

/// Regular polygon approximating a cylinder of radius `r`.
std::vector<Seg> circle(double cx, double cy, double r, int sides = 24)
{
  std::vector<Seg> out;
  for (int i = 0; i < sides; ++i) {
    const double a0 = 2.0 * M_PI * i / sides;
    const double a1 = 2.0 * M_PI * (i + 1) / sides;
    out.push_back(
      {cx + r * std::cos(a0), cy + r * std::sin(a0),
        cx + r * std::cos(a1), cy + r * std::sin(a1)});
  }
  return out;
}

std::vector<Seg> operator+(std::vector<Seg> a, const std::vector<Seg> & b)
{
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

// --- reference actor footprints, as the scan plane at ~0.16 m sees them ----
//
// PEDESTRIAN: two legs, each a 0.12 m square proxy for a calf, 0.28 m apart
// centre-to-centre (a mid-stride stance).
std::vector<Seg> pedestrian_at(double x, double y = 0.0)
{
  return rect(x, y - 0.14, 0.12, 0.12, 0.0) + rect(x, y + 0.14, 0.12, 0.12, 0.0);
}

// BIKE: the two wheel CHORDS. At a 0.16 m scan height a 0.67 m wheel is cut
// 0.175 m below its hub, giving a chord of 2*sqrt(0.335^2 - 0.175^2) = 0.57 m
// along the direction of travel and only a tyre width (0.04 m) thick. The
// wheelbase is 1.05 m, so the two chords leave a ~0.48 m hole between them.
std::vector<Seg> bike_at(double x, double y = 0.0, double yaw = 0.0)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double off = 0.525;
  return rect(x - off * s, y + off * c, 0.04, 0.57, yaw) +
         rect(x + off * s, y - off * c, 0.04, 0.57, yaw);
}

// VEHICLE: a 3.8 x 1.7 m car footprint, and a compact 1.0 x 0.5 m body that
// fits inside the frozen tracker's 1.2 m cluster-diameter envelope.
std::vector<Seg> car_at(double x, double y = 0.0, double yaw = 0.0)
{
  return rect(x, y, 3.8, 1.7, yaw);
}

std::vector<Seg> compact_vehicle_at(double x, double y = 0.0, double yaw = 0.0)
{
  return rect(x, y, 1.0, 0.5, yaw);
}

/// A mature, moving track: old enough to classify, with a trusted velocity.
TrackMotion moving(double speed = 1.0)
{
  TrackMotion m;
  m.speed = speed;
  m.speed_valid = true;
  m.observations = 10;
  return m;
}

ClusterFeatures feats(const std::vector<Seg> & world, const ClassificationParams & p)
{
  return extractFeatures(scan_world(world), 0.0, 0.0, p);
}

}  // namespace

int main()
{
  const ClassificationParams p;

  // =====================================================================
  // 1. Feature extraction
  // =====================================================================
  {
    // A 2.0 m flat plate broadside at 3 m: length must recover the physical
    // extent, the slice must read as solid, and there must be no internal gap.
    const auto f = feats(rect(3.0, 0.0, 0.02, 2.0, 0.0), p);
    require(f.valid, "extract: broadside plate should be valid");
    require(std::abs(f.length - 2.0) < 0.10, "extract: length should recover 2.0 m");
    require(f.thickness < 0.10, "extract: a flat broadside plate should be thin");
    require(std::abs(f.range - 3.0) < 0.10, "extract: range should be ~3.0 m");
    require(f.fill_ratio > 0.85, "extract: a solid surface should have fill_ratio ~1");
    require(f.max_gap < 0.12, "extract: a solid surface should show no large gap");
  }
  {
    // Range normalisation: the SAME object further away returns fewer beams
    // but must still read as solid and keep its measured length.
    const auto near = feats(rect(1.5, 0.0, 0.02, 0.60, 0.0), p);
    const auto far = feats(rect(5.0, 0.0, 0.02, 0.60, 0.0), p);
    require(far.point_count < near.point_count, "extract: farther object returns fewer beams");
    require(far.angular_span < near.angular_span, "extract: farther object subtends less angle");
    require(far.fill_ratio > 0.80, "extract: fill_ratio must stay ~1 with range");
    require(std::abs(far.length - 0.60) < 0.12, "extract: length must stay ~0.60 m with range");
  }
  {
    // max_gap must see THROUGH a structured object. A bike's two wheel chords
    // leave a ~0.48 m hole; a solid plate of the same overall length does not.
    const auto bike = feats(bike_at(3.0), p);
    const auto solid = feats(rect(3.0, 0.0, 0.02, 1.60, 0.0), p);
    require(bike.valid && solid.valid, "extract: both fixtures should be valid");
    require(bike.max_gap > 0.35, "extract: max_gap should expose a bike's empty middle");
    require(solid.max_gap < 0.15, "extract: a solid plate of equal length has no gap");
    require(
      bike.fill_ratio < solid.fill_ratio - 0.15,
      "extract: a gapped silhouette must read less solid than a plate");
  }
  {
    // A corner view must report real depth on the minor axis; a flank view of
    // the same body must not.
    const auto corner = feats(car_at(4.0, 0.0, 0.52), p);
    const auto flank = feats(car_at(4.0, 0.0, M_PI_2), p);
    require(corner.thickness > 0.35, "extract: a corner view should show real depth");
    require(flank.thickness < 0.20, "extract: a flank view should show little depth");
  }
  {
    // Degenerate input: fewer than 3 points leaves the shape undefined.
    const std::vector<P> two{{1.0, 0.0}, {1.0, 0.1}};
    require(!extractFeatures(two, 0.0, 0.0, p).valid, "extract: 2 points must be invalid");
    const std::vector<P> none{};
    require(!extractFeatures(none, 0.0, 0.0, p).valid, "extract: 0 points must be invalid");
    const std::vector<P> bad{{1.0, 0.0}, {std::nan(""), 0.0}, {1.0, 0.1}, {1.0, 0.2}};
    require(!extractFeatures(bad, 0.0, 0.0, p).valid, "extract: NaN input must be invalid");
  }
  {
    // A cluster straddling the +/-pi bearing wrap must not report a span of
    // nearly 2*pi: the object is behind the sensor, on the -x axis.
    const auto f = feats(rect(-3.0, 0.0, 0.02, 0.60, 0.0), p);
    require(f.valid, "extract: wrapped cluster should be valid");
    require(f.angular_span < 0.5, "extract: bearing wrap must not inflate angular_span");
    require(f.fill_ratio > 0.80, "extract: wrapped solid surface should still read solid");
    require(std::abs(f.length - 0.60) < 0.12, "extract: wrapped length should still be 0.60 m");
  }
  {
    // fill_ratio is a FRACTION OF AVAILABLE BEAMS and must therefore never
    // exceed 1, over any geometry, aspect or range. Measured on the Stage-5B
    // validation set before the clamp was added, 977 of 2816 detections
    // reported above 1.05 (up to 1.37) because this LiDAR's first and last
    // beams sit 0.18 increments apart, and every one of those detections was
    // being rejected by the classes' fill_hi bounds. This pins the bound so
    // the artefact cannot return.
    bool bounded = true;
    double worst = 0.0;
    for (double range = 1.0; range <= 6.0; range += 0.25) {
      for (double yaw = 0.0; yaw < M_PI; yaw += 0.2) {
        const std::vector<std::vector<Seg>> worlds{
          pedestrian_at(range), bike_at(range, 0.0, yaw), car_at(range, 0.0, yaw),
          compact_vehicle_at(range, 0.0, yaw), circle(range, 0.0, 0.45),
          rect(range, 0.0, 0.02, 0.75, 0.0)};
        for (const auto & w : worlds) {
          const auto f = feats(w, p);
          worst = std::max(worst, f.fill_ratio);
          if (f.fill_ratio > 1.0) {
            bounded = false;
          }
        }
      }
    }
    require(bounded, "extract: fill_ratio must never exceed 1.0");
    require(worst > 0.8, "extract: the sweep should include near-solid clusters");
  }
  {
    // Translation invariance: the measured SHAPE must not depend on where the
    // sensor happens to sit, only the range-relative terms should.
    const auto a = extractFeatures(scan_world(bike_at(3.0, 0.0)), 0.0, 0.0, p);
    const auto b = extractFeatures(
      scan_world(bike_at(8.0, 5.0), 5.0, 5.0), 5.0, 5.0, p);
    require(std::abs(a.length - b.length) < 0.10, "extract: length must be frame-independent");
    require(std::abs(a.max_gap - b.max_gap) < 0.10, "extract: max_gap must be frame-independent");
    require(std::abs(a.range - b.range) < 0.10, "extract: range must be measured from the sensor");
  }

  // =====================================================================
  // 2. Pedestrian
  // =====================================================================
  {
    // Two legs at 2.5 m: compact, moderately filled, small stride gap.
    const auto f = feats(pedestrian_at(2.5), p);
    const auto r = classifyInstant(f, moving(1.2), p);
    require(
      r.label == ObstacleClass::PEDESTRIAN,
      std::string("pedestrian: two legs at 2.5 m should be PEDESTRIAN, got ") +
      toString(r.label) + " (" + r.reject_reason + ")");
    require(r.confidence > 0.0 && r.confidence <= 1.0, "pedestrian: confidence in (0,1]");
  }
  {
    // Legs together (one 0.25 m blob) must also read as a pedestrian.
    const auto f = feats(rect(2.0, 0.0, 0.25, 0.25, 0.0), p);
    const auto r = classifyInstant(f, moving(0.8), p);
    require(
      r.label == ObstacleClass::PEDESTRIAN,
      std::string("pedestrian: a single compact blob should be PEDESTRIAN, got ") +
      toString(r.label) + " (" + r.reject_reason + ")");
  }
  {
    // The speed veto: pedestrian geometry at 9 m/s is not a pedestrian.
    const auto f = feats(pedestrian_at(2.5), p);
    const auto r = classifyInstant(f, moving(9.0), p);
    require(
      r.label != ObstacleClass::PEDESTRIAN,
      "pedestrian: 9 m/s must veto the PEDESTRIAN hypothesis");
  }
  {
    // DOCUMENTED SENSOR LIMIT, asserted so it cannot regress silently: a
    // pedestrian's legs subtend too few beams to classify at long range, and
    // the correct answer there is UNKNOWN rather than a guess.
    const auto near = classifyInstant(feats(pedestrian_at(2.5), p), moving(1.2), p);
    const auto far = classifyInstant(feats(pedestrian_at(5.5), p), moving(1.2), p);
    require(near.label == ObstacleClass::PEDESTRIAN, "pedestrian: classifiable at 2.5 m");
    require(
      far.label == ObstacleClass::UNKNOWN,
      "pedestrian: a far, sparse pedestrian must be UNKNOWN, not guessed");
  }

  // =====================================================================
  // 3. Bike
  // =====================================================================
  {
    // Two wheel chords, 1.05 m wheelbase, broadside at 3 m.
    const auto f = feats(bike_at(3.0), p);
    const auto r = classifyInstant(f, moving(4.0), p);
    require(
      r.label == ObstacleClass::BIKE,
      std::string("bike: broadside bike at 3 m should be BIKE, got ") +
      toString(r.label) + " (" + r.reject_reason + ")");
    require(r.confidence > 0.0 && r.confidence <= 1.0, "bike: confidence in (0,1]");
  }
  {
    // Same at 4 m: the wheel chords still resolve.
    const auto r = classifyInstant(feats(bike_at(4.0), p), moving(4.0), p);
    require(
      r.label == ObstacleClass::BIKE,
      std::string("bike: broadside bike at 4 m should be BIKE, got ") + toString(r.label));
  }
  {
    // The GAP is what makes a bike. Fill the middle in and the same envelope
    // must stop being a bike: a solid 1.6 m thin surface is not a bicycle.
    const auto f = feats(rect(3.0, 0.0, 0.02, 1.60, 0.0), p);
    const auto r = classifyInstant(f, moving(4.0), p);
    require(
      r.label != ObstacleClass::BIKE,
      std::string("bike: a SOLID 1.6 m surface must not be BIKE, got ") + toString(r.label));
  }

  // =====================================================================
  // 4. Vehicle
  // =====================================================================
  {
    // Flank view of a 3.8 x 1.7 m car at 4 m: long, solid, ungapped.
    const auto f = feats(car_at(4.0, 0.0, M_PI_2), p);
    const auto r = classifyInstant(f, moving(3.0), p);
    require(
      r.label == ObstacleClass::VEHICLE,
      std::string("vehicle: car flank should be VEHICLE, got ") +
      toString(r.label) + " (" + r.reject_reason + ")");
    require(r.confidence > 0.0 && r.confidence <= 1.0, "vehicle: confidence in (0,1]");
  }
  {
    // Corner view of the same car: shorter, but with real depth.
    const auto r = classifyInstant(feats(car_at(4.0, 0.0, 0.52), p), moving(3.0), p);
    require(
      r.label == ObstacleClass::VEHICLE,
      std::string("vehicle: car corner should be VEHICLE, got ") + toString(r.label));
  }
  {
    // The corner ASPECT is what makes a compact vehicle separable at all --
    // and it is the only vehicle view that fits inside the frozen tracker's
    // 1.2 m cluster-diameter envelope.
    const auto r = classifyInstant(
      feats(compact_vehicle_at(3.0, 0.0, 0.52), p), moving(1.0), p);
    require(
      r.label == ObstacleClass::VEHICLE,
      std::string("vehicle: compact corner view should be VEHICLE, got ") +
      toString(r.label) + " (" + r.reject_reason + ")");
  }
  {
    // A vehicle must never be mistaken for a bike.
    bool any_bike = false;
    for (double yaw = 0.0; yaw < M_PI; yaw += 0.15) {
      for (double range = 2.5; range <= 5.5; range += 0.5) {
        const auto r = classifyInstant(feats(car_at(range, 0.0, yaw), p), moving(3.0), p);
        if (r.label == ObstacleClass::BIKE) {
          any_bike = true;
        }
      }
    }
    require(!any_bike, "vehicle: a car must not classify BIKE from any aspect or range");
  }

  // =====================================================================
  // 5. UNKNOWN behaviour -- the classifier must decline, not guess
  // =====================================================================
  {
    // Too few returns for the shape statistics to mean anything.
    ClusterFeatures f;
    f.valid = true;
    f.point_count = 3;          // below min_points (5)
    f.length = 0.3;
    f.thickness = 0.1;
    f.range = 2.0;
    f.fill_ratio = 0.9;
    const auto r = classifyInstant(f, moving(), p);
    require(r.label == ObstacleClass::UNKNOWN, "unknown: too few points must be UNKNOWN");
    require(std::string(r.reject_reason) == "too_few_points", "unknown: reason too_few_points");
    require(r.confidence == 0.0, "unknown: confidence must be exactly 0");
  }
  {
    // Beyond max_range the geometry cannot be recovered.
    auto f = feats(rect(2.0, 0.0, 0.25, 0.25, 0.0), p);
    f.range = 9.0;
    const auto r = classifyInstant(f, moving(), p);
    require(r.label == ObstacleClass::UNKNOWN, "unknown: beyond max_range must be UNKNOWN");
    require(std::string(r.reject_reason) == "out_of_range", "unknown: reason out_of_range");
  }
  {
    // A track with fewer observations than min_observations must not be
    // labelled at all, however good the geometry is.
    TrackMotion young = moving(1.0);
    young.observations = 1;
    const auto r = classifyInstant(feats(pedestrian_at(2.5), p), young, p);
    require(r.label == ObstacleClass::UNKNOWN, "unknown: a young track must be UNKNOWN");
    require(std::string(r.reject_reason) == "track_too_young", "unknown: reason track_too_young");
  }
  {
    // An invalid feature set can never produce a label.
    const ClusterFeatures f;   // valid == false
    const auto r = classifyInstant(f, moving(), p);
    require(r.label == ObstacleClass::UNKNOWN, "unknown: invalid features must be UNKNOWN");
    require(std::string(r.reject_reason) == "invalid_features", "unknown: reason invalid_features");
  }
  {
    // Geometry belonging to no class: a 0.9 m round planter is too long for a
    // pedestrian, has no wheel gap, is too short for a vehicle flank and too
    // shallow for a vehicle corner.
    const auto r = classifyInstant(feats(circle(3.0, 0.0, 0.45), p), moving(0.0), p);
    require(
      r.label == ObstacleClass::UNKNOWN,
      std::string("unknown: a 0.9 m round planter fits no class, got ") + toString(r.label));
  }
  {
    // A 0.75 m thin plate likewise fits nothing.
    const auto r = classifyInstant(feats(rect(2.5, 0.0, 0.02, 0.75, 0.0), p), moving(0.5), p);
    require(
      r.label == ObstacleClass::UNKNOWN,
      std::string("unknown: a 0.75 m plate fits no class, got ") + toString(r.label));
  }

  // =====================================================================
  // 6. Confidence bounds
  // =====================================================================
  {
    // Over a wide sweep of real geometries, confidence and every class score
    // must stay in [0, 1], and confidence must be exactly 0 for UNKNOWN.
    bool bounded = true;
    bool unknown_zero = true;
    int labelled = 0;
    for (double range = 1.5; range <= 6.0; range += 0.5) {
      for (double yaw = 0.0; yaw < M_PI; yaw += 0.3) {
        std::vector<std::vector<Seg>> worlds{
          pedestrian_at(range), bike_at(range, 0.0, yaw),
          car_at(range, 0.0, yaw), compact_vehicle_at(range, 0.0, yaw),
          circle(range, 0.0, 0.45), rect(range, 0.0, 0.02, 0.75, 0.0)};
        for (const auto & w : worlds) {
          const auto r = classifyInstant(feats(w, p), moving(1.0), p);
          if (!(r.confidence >= 0.0 && r.confidence <= 1.0)) {
            bounded = false;
          }
          if (r.label == ObstacleClass::UNKNOWN && r.confidence != 0.0) {
            unknown_zero = false;
          }
          if (r.label != ObstacleClass::UNKNOWN) {
            ++labelled;
          }
          for (const double sc : r.scores) {
            if (!(sc >= 0.0 && sc <= 1.0)) {
              bounded = false;
            }
          }
        }
      }
    }
    require(bounded, "confidence: all confidences and scores must lie in [0,1]");
    require(unknown_zero, "confidence: UNKNOWN must carry confidence exactly 0");
    require(labelled > 50, "confidence: the sweep must produce many real labels");
  }
  {
    // trapezoid() itself: 1 inside, 0 beyond the ramp, monotone across it.
    require(trapezoid(0.5, 0.0, 1.0, 0.1) == 1.0, "trapezoid: inside -> 1");
    require(trapezoid(1.05, 0.0, 1.0, 0.1) > 0.0, "trapezoid: on the ramp -> (0,1)");
    require(trapezoid(1.05, 0.0, 1.0, 0.1) < 1.0, "trapezoid: on the ramp -> (0,1)");
    require(trapezoid(1.20, 0.0, 1.0, 0.1) == 0.0, "trapezoid: beyond the ramp -> 0");
    require(trapezoid(-0.20, 0.0, 1.0, 0.1) == 0.0, "trapezoid: below the ramp -> 0");
    require(trapezoid(1.05, 0.0, 1.0, 0.0) == 0.0, "trapezoid: zero soft -> hard indicator");
    require(trapezoid(std::nan(""), 0.0, 1.0, 0.1) == 0.0, "trapezoid: NaN -> 0");
    require(
      trapezoid(5.0, 0.0, std::numeric_limits<double>::infinity(), 0.1) == 1.0,
      "trapezoid: an infinite upper bound must never bind");
    bool mono = true;
    double prev = 1.0;
    for (double v = 1.0; v <= 1.11; v += 0.01) {
      const double cur = trapezoid(v, 0.0, 1.0, 0.1);
      if (cur > prev + 1e-12) {
        mono = false;
      }
      prev = cur;
    }
    require(mono, "trapezoid: must be non-increasing across the upper ramp");
  }

  // =====================================================================
  // 7. Temporal stabilisation
  // =====================================================================
  {
    // A confident label must be adopted within a bounded number of scans,
    // and must then be HELD through a single contradictory scan.
    ClassLabelStabilizer st;
    ClassificationResult ped;
    ped.label = ObstacleClass::PEDESTRIAN;
    ped.confidence = 0.9;

    int adopt = -1;
    for (int i = 1; i <= 10; ++i) {
      if (st.update(ped) == ObstacleClass::PEDESTRIAN) {
        adopt = i;
        break;
      }
    }
    require(adopt > 0, "stabilizer: a confident label must eventually be adopted");
    require(adopt <= 3, "stabilizer: adoption must take at most 3 confident scans");

    ClassificationResult bike;
    bike.label = ObstacleClass::BIKE;
    bike.confidence = 0.9;
    require(
      st.update(bike) == ObstacleClass::PEDESTRIAN,
      "stabilizer: ONE contradictory scan must not flip the label");
  }
  {
    // Sustained contradictory evidence MUST eventually win -- hysteresis is
    // not permanence.
    ClassLabelStabilizer st;
    ClassificationResult ped;
    ped.label = ObstacleClass::PEDESTRIAN;
    ped.confidence = 0.9;
    for (int i = 0; i < 10; ++i) {
      st.update(ped);
    }
    require(st.label() == ObstacleClass::PEDESTRIAN, "stabilizer: setup should hold PEDESTRIAN");

    ClassificationResult bike;
    bike.label = ObstacleClass::BIKE;
    bike.confidence = 0.9;
    int flip = -1;
    for (int i = 1; i <= 20; ++i) {
      if (st.update(bike) == ObstacleClass::BIKE) {
        flip = i;
        break;
      }
    }
    require(flip > 0, "stabilizer: sustained contrary evidence must take the label over");
    require(flip >= 2, "stabilizer: the switch must require more than one scan");
  }
  {
    // Alternating evidence must NOT produce an alternating label. This is the
    // property switch_margin is derived to guarantee, so it is checked at the
    // worst case (confidence 1.0) as well as below it.
    for (const double c : {0.6, 0.8, 1.0}) {
      ClassLabelStabilizer st;
      ClassificationResult a;
      a.label = ObstacleClass::PEDESTRIAN;
      a.confidence = c;
      ClassificationResult b;
      b.label = ObstacleClass::BIKE;
      b.confidence = c;

      std::vector<ObstacleClass> seen;
      for (int i = 0; i < 60; ++i) {
        seen.push_back(st.update((i % 2 == 0) ? a : b));
      }
      size_t changes = 0;
      for (size_t i = 1; i < seen.size(); ++i) {
        if (seen[i] != seen[i - 1]) {
          ++changes;
        }
      }
      require(
        changes <= 2,
        "stabilizer: alternating evidence must not oscillate (c=" +
        std::to_string(c) + ", changes=" + std::to_string(changes) + ")");
    }
  }
  {
    // A label must FADE when the object stops being recognisable: UNKNOWN
    // scans contribute no evidence and decay what is there.
    ClassLabelStabilizer st;
    ClassificationResult ped;
    ped.label = ObstacleClass::PEDESTRIAN;
    ped.confidence = 0.9;
    for (int i = 0; i < 10; ++i) {
      st.update(ped);
    }
    require(st.label() == ObstacleClass::PEDESTRIAN, "stabilizer: setup should hold PEDESTRIAN");

    const ClassificationResult nothing;   // UNKNOWN, confidence 0
    int to_unknown = -1;
    for (int i = 1; i <= 30; ++i) {
      if (st.update(nothing) == ObstacleClass::UNKNOWN) {
        to_unknown = i;
        break;
      }
    }
    require(to_unknown > 0, "stabilizer: a label must decay to UNKNOWN without evidence");
    require(to_unknown > 1, "stabilizer: one missing scan must not drop the label");
    require(st.confidence() == 0.0, "stabilizer: UNKNOWN must report zero confidence");
  }
  {
    // Confidence must stay in [0,1] and track the held label.
    ClassLabelStabilizer st;
    ClassificationResult v;
    v.label = ObstacleClass::VEHICLE;
    v.confidence = 1.0;
    bool bounded = true;
    for (int i = 0; i < 50; ++i) {
      st.update(v);
      if (!(st.confidence() >= 0.0 && st.confidence() <= 1.0)) {
        bounded = false;
      }
    }
    require(bounded, "stabilizer: confidence must stay in [0,1]");
    require(st.label() == ObstacleClass::VEHICLE, "stabilizer: should hold VEHICLE");
    require(st.confidence() > 0.5, "stabilizer: sustained max evidence should be confident");
  }
  {
    // A fresh stabiliser starts from UNKNOWN with no inherited evidence, so a
    // recycled track ID can never inherit the previous object's label.
    const ClassLabelStabilizer st;
    require(st.label() == ObstacleClass::UNKNOWN, "stabilizer: must start UNKNOWN");
    require(st.confidence() == 0.0, "stabilizer: must start with zero confidence");
    require(st.score(ObstacleClass::PEDESTRIAN) == 0.0, "stabilizer: must start with zero scores");
  }

  // =====================================================================
  // 8. Boundary conditions on the class boxes
  // =====================================================================
  {
    const ClassBox & ped = p.pedestrian.aspects.front();
    // Exactly ON a box edge the membership is 1.0 by definition (the interval
    // is closed), so a feature set sitting on pedestrian length_hi must not
    // be rejected for its length.
    ClusterFeatures f;
    f.valid = true;
    f.point_count = 12;
    f.range = 2.0;
    f.length = ped.length_hi;      // 0.55 m
    f.thickness = 0.20;
    f.fill_ratio = 0.8;
    f.max_gap = 0.10;
    const auto on_edge = classifyInstant(f, moving(1.0), p);
    require(
      on_edge.label == ObstacleClass::PEDESTRIAN,
      "boundary: a feature set exactly on pedestrian length_hi must still be PEDESTRIAN");

    // Just past the far end of the soft ramp it must be refused, not stretched.
    f.length = ped.length_hi + ped.length_soft + 0.05;
    const auto past = classifyInstant(f, moving(1.0), p);
    require(
      past.label != ObstacleClass::PEDESTRIAN,
      "boundary: beyond the soft ramp PEDESTRIAN must be refused");
  }
  {
    // The bike gap bound, approached from below: a gap just under the ramp
    // must not yield BIKE, one at the bound must.
    const ClassBox & bike = p.bike.aspects.front();
    ClusterFeatures f;
    f.valid = true;
    f.point_count = 20;
    f.range = 3.0;
    f.length = 1.50;
    f.thickness = 0.05;
    f.fill_ratio = 0.70;
    f.max_gap = bike.gap_lo;                       // exactly on the bound
    require(
      classifyInstant(f, moving(4.0), p).label == ObstacleClass::BIKE,
      "boundary: a gap exactly on bike gap_lo must be BIKE");
    f.max_gap = bike.gap_lo - bike.gap_soft - 0.01;   // below the ramp
    require(
      classifyInstant(f, moving(4.0), p).label != ObstacleClass::BIKE,
      "boundary: a gap below the ramp must not be BIKE");
  }
  {
    // thickness_floor must stop elongation exploding for a perfectly flat
    // slice -- it is published, so it must at least stay finite and bounded.
    const auto f = feats(rect(2.0, 0.0, 0.02, 0.30, 0.0), p);
    require(std::isfinite(f.elongation), "boundary: elongation must stay finite");
    require(
      f.elongation <= f.length / p.thickness_floor + 1e-9,
      "boundary: elongation must be capped by thickness_floor");
  }
  {
    // Determinism: identical input must give an identical answer.
    const auto f = feats(bike_at(3.0), p);
    const auto r1 = classifyInstant(f, moving(4.0), p);
    const auto r2 = classifyInstant(f, moving(4.0), p);
    require(
      r1.label == r2.label && r1.confidence == r2.confidence,
      "boundary: classification must be deterministic");
  }
  {
    // An unproven velocity must not trigger the speed veto: while
    // speed_valid is false the reported speed is a prior, not a measurement.
    TrackMotion m;
    m.speed = 50.0;         // nonsense, but not yet trusted
    m.speed_valid = false;
    m.observations = 10;
    const auto r = classifyInstant(feats(pedestrian_at(2.5), p), m, p);
    require(
      r.label == ObstacleClass::PEDESTRIAN,
      "boundary: an untrusted speed must not veto a class");
  }
  {
    // The margin rule must fire: two classes that both fit must yield
    // UNKNOWN rather than an arbitrary winner.
    ClassificationParams tied = p;
    // Make the bike and pedestrian boxes identical, so any geometry fitting
    // one fits the other exactly and the margin can never be met.
    tied.bike.aspects.front() = tied.pedestrian.aspects.front();
    ClusterFeatures f;
    f.valid = true;
    f.point_count = 12;
    f.range = 2.0;
    f.length = 0.30;
    f.thickness = 0.10;
    f.fill_ratio = 0.80;
    f.max_gap = 0.10;
    const auto r = classifyInstant(f, moving(1.0), tied);
    require(r.label == ObstacleClass::UNKNOWN, "boundary: a perfect tie must be UNKNOWN");
    require(std::string(r.reject_reason) == "ambiguous", "boundary: reason ambiguous");
  }

  std::fprintf(
    stderr, "stage5b_classification: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
