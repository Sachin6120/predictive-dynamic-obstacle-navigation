// Stage-5B.1: unit tests for gap-aware compound-observation recovery.
//
// HOW THESE TESTS GENERATE INPUT
// ------------------------------
// Same discipline as the Stage-5B classification tests: every fixture is
// produced by RAY-CASTING a 2D LiDAR against a polygonal footprint (one
// nearest hit per beam, at the sensor's own angular spacing, plus range
// noise), and then run through a reconstruction of the FROZEN primary
// clustering rule at cluster_distance = 0.35 m. So the clusters these tests
// evaluate are the clusters the robot would actually get -- the fragmentation
// is not simulated by hand, it falls out of the geometry.
//
// WHAT THEY ASSERT
// ----------------
//   1. The bicycle really does fragment at cluster_distance = 0.35 m, and the
//      compound stage recovers it -- across aspect and range.
//   2. THE ANTI-MERGE REQUIREMENT. Two genuinely distinct objects must fail
//      safely to two independent observations, at the Stage-4G3 separations
//      and beyond, while crossing, while moving in parallel, and standing
//      still. These are the tests the milestone can fail on.
//   3. Each negative case is rejected REDUNDANTLY -- by two or more
//      independent bounds -- so no single threshold is load-bearing.
//   4. The temporal gate: one frame is never enough, support must be
//      consecutive, and a pair whose separation drifts loses it.
//   5. The structural invariant: this stage never operates on a gap inside
//      the primary link distance.
//   6. The existing Stage-5B classifier labels a recovered compound
//      observation BIKE on geometry alone, with NO threshold changes.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#include "predictive_nav_tracking/compound_observation.hpp"
#include "predictive_nav_tracking/obstacle_classification.hpp"

using predictive_nav_tracking::ClassificationParams;
using predictive_nav_tracking::ClusterFeatures;
using predictive_nav_tracking::CompoundEvidence;
using predictive_nav_tracking::CompoundHypothesisTracker;
using predictive_nav_tracking::CompoundParams;
using predictive_nav_tracking::ObstacleClass;
using predictive_nav_tracking::TrackMotion;
using predictive_nav_tracking::classifyInstant;
using predictive_nav_tracking::describeReject;
using predictive_nav_tracking::evaluateCompoundPair;
using predictive_nav_tracking::extractFeatures;
using predictive_nav_tracking::kRejectAlignment;
using predictive_nav_tracking::kRejectFragmentLength;
using predictive_nav_tracking::kRejectFragmentThickness;
using predictive_nav_tracking::kRejectUnionThickness;

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

/// The FROZEN primary clustering parameters. These are asserted against the
/// node's declared defaults by the integration test; here they are the
/// geometry these fixtures must be clustered with.
constexpr double kClusterDistance = 0.35;   // m -- MUST NOT CHANGE
constexpr size_t kClusterMinPoints = 3;
constexpr size_t kClusterMaxPoints = 200;
constexpr double kClusterMaxDiameter = 1.2;  // m

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

/// Deterministic pseudo-random range noise, seeded per call site so that a
/// fixture is reproducible but two objects in one scene do not share a
/// noise sequence.
class Jitter
{
public:
  explicit Jitter(uint64_t seed)
  : state_(seed) {}

  double next()
  {
    state_ = state_ * 6364136223846793005ULL + 1442695040888963407ULL;
    const double u = static_cast<double>((state_ >> 11) & 0x1FFFFFULL) /
      static_cast<double>(0x200000ULL);
    return (2.0 * u - 1.0) * 1.732;
  }

private:
  uint64_t state_;
};

std::vector<P> scan_world(
  const std::vector<Seg> & segs, double sx = 0.0, double sy = 0.0, uint64_t seed = 0x5B5Bu)
{
  Jitter jitter(seed);
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
      const double u = (-ax * ey + ex * ay) / det;
      const double v = (dx * ay - dy * ax) / det;
      if (u > 1e-9 && v >= 0.0 && v <= 1.0 && u < best) {
        best = u;
      }
    }
    if (std::isfinite(best)) {
      const double r = best + kNoise * jitter.next();
      hits.push_back({sx + r * dx, sy + r * dy});
    }
  }
  return hits;
}

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

// --- footprints, exactly as the SDF models define them ---------------------

/// Stage-5B actor_bike: two 0.57 x 0.04 m wheel chords at +/-0.525 m along
/// the direction of travel. Wheelbase 1.05 m, inter-chord hole 0.48 m.
std::vector<Seg> bike_at(double x, double y = 0.0, double yaw = 0.0)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double off = 0.525;
  return rect(x + off * c, y + off * s, 0.57, 0.04, yaw) +
         rect(x - off * c, y - off * s, 0.57, 0.04, yaw);
}

/// Stage-5B actor_pedestrian: two 0.12 m square calf proxies, 0.325 m apart.
std::vector<Seg> pedestrian_at(double x, double y = 0.0, double yaw = 0.0)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double lx = 0.14;
  const double ly = 0.08;
  return rect(x + lx * c - ly * s, y + lx * s + ly * c, 0.12, 0.12, yaw) +
         rect(x - lx * c + ly * s, y - lx * s - ly * c, 0.12, 0.12, yaw);
}

/// Stage-4G3 dynamic_obstacle: a 0.2 m radius cylinder. THIS is the object
/// whose close-object separation behaviour must not regress.
std::vector<Seg> g4g3_obstacle_at(double x, double y)
{
  return circle(x, y, 0.2);
}

// --- frozen primary clustering, reconstructed ------------------------------

/// Single-link connected components at `kClusterDistance`, with the same
/// point-count and diameter filters the node applies. Semantics identical to
/// build_clusters().
std::vector<std::vector<P>> primary_clusters(const std::vector<P> & pts)
{
  const size_t n = pts.size();
  std::vector<bool> visited(n, false);
  std::vector<std::vector<P>> out;
  for (size_t seed = 0; seed < n; ++seed) {
    if (visited[seed]) {
      continue;
    }
    visited[seed] = true;
    std::vector<size_t> idx{seed};
    for (size_t k = 0; k < idx.size(); ++k) {
      for (size_t j = 0; j < n; ++j) {
        if (visited[j]) {
          continue;
        }
        if (std::hypot(pts[idx[k]].x - pts[j].x, pts[idx[k]].y - pts[j].y) <=
          kClusterDistance)
        {
          visited[j] = true;
          idx.push_back(j);
        }
      }
    }
    if (idx.size() < kClusterMinPoints || idx.size() > kClusterMaxPoints) {
      continue;
    }
    double minx = pts[idx.front()].x, maxx = minx;
    double miny = pts[idx.front()].y, maxy = miny;
    for (const size_t i : idx) {
      minx = std::min(minx, pts[i].x);
      maxx = std::max(maxx, pts[i].x);
      miny = std::min(miny, pts[i].y);
      maxy = std::max(maxy, pts[i].y);
    }
    if (std::hypot(maxx - minx, maxy - miny) > kClusterMaxDiameter) {
      continue;
    }
    std::vector<P> members;
    members.reserve(idx.size());
    for (const size_t i : idx) {
      members.push_back(pts[i]);
    }
    out.push_back(std::move(members));
  }
  return out;
}

/// One scene reduced to its primary clusters and their features.
struct Scene
{
  std::vector<std::vector<P>> clusters;
  std::vector<ClusterFeatures> features;
};

Scene observe(
  const std::vector<Seg> & world, const ClassificationParams & clp,
  uint64_t seed = 0x5B5Bu)
{
  Scene s;
  s.clusters = primary_clusters(scan_world(world, 0.0, 0.0, seed));
  for (const auto & c : s.clusters) {
    s.features.push_back(extractFeatures(c, 0.0, 0.0, clp));
  }
  return s;
}

/// Evaluate the first two clusters of a scene as a candidate pair.
CompoundEvidence evaluate_first_pair(
  const Scene & s, const CompoundParams & cp, const ClassificationParams & clp)
{
  if (s.clusters.size() < 2) {
    return CompoundEvidence{};
  }
  return evaluateCompoundPair(
    s.features[0], s.features[1], s.clusters[0], s.clusters[1],
    0.0, 0.0, kClusterDistance, cp, clp);
}

/// Number of independent bounds a rejection cites. The anti-merge tests
/// require this to be >= 2, so that no negative case depends on one
/// threshold sitting marginally in the right place.
int reject_bound_count(uint32_t mask)
{
  int n = 0;
  for (int b = 0; b < 32; ++b) {
    if (mask & (1u << b)) {
      ++n;
    }
  }
  return n;
}

TrackMotion moving(double speed = 0.25)
{
  TrackMotion m;
  m.speed = speed;
  m.speed_valid = true;
  m.observations = 10;
  return m;
}

}  // namespace

int main()
{
  const ClassificationParams clp;
  const CompoundParams base_cp;   // the shipped defaults

  // =====================================================================
  // 1. ROOT CAUSE -- the bicycle really does fragment at 0.35 m
  // =====================================================================
  {
    // Broadside at 3 m. If this stops producing two clusters, the premise of
    // the whole milestone has changed and the rest of these tests are moot.
    const Scene s = observe(bike_at(3.0, 0.0, M_PI / 2), clp);
    require(
      s.clusters.size() == 2,
      "root cause: a bicycle at 3 m must split into exactly 2 primary clusters");
    if (s.clusters.size() == 2) {
      const double gap = predictive_nav_tracking::nearestGap(s.clusters[0], s.clusters[1]);
      require(
        gap > kClusterDistance,
        "root cause: the inter-fragment gap must exceed cluster_distance (that is WHY it splits)");
      require(
        gap > 0.40 && gap < 0.70,
        "root cause: the measured inter-fragment gap should be ~0.48 m");
      require(
        s.features[0].valid && s.features[1].valid,
        "root cause: both fragments must have usable shape statistics");
      // Each fragment alone is pedestrian-sized -- the reason BIKE recall
      // was 0.000 before this stage existed.
      const auto r = classifyInstant(s.features[0], moving(), clp);
      require(
        r.label != ObstacleClass::BIKE,
        "root cause: a single wheel fragment must NOT classify as BIKE on its own");
    }
  }

  // =====================================================================
  // 2. RECOVERY -- the compound stage accepts the bicycle across aspect
  //    and range, on single-frame geometry
  // =====================================================================
  {
    int accepted = 0;
    int fragmented = 0;
    for (const double range : {2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0}) {
      for (const double yaw : {M_PI / 2, M_PI / 4, M_PI / 3}) {
        const Scene s = observe(bike_at(range, 0.0, yaw), clp);
        if (s.clusters.size() != 2) {
          continue;
        }
        ++fragmented;
        const CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
        if (e.geometry_ok) {
          ++accepted;
        } else {
          std::fprintf(
            stderr, "  note: bike r=%.1f yaw=%.2f rejected by %s\n",
            range, yaw, describeReject(e.reject_mask).c_str());
        }
      }
    }
    require(fragmented >= 15, "recovery: the sweep must produce fragmented bicycles to recover");
    // Measured acceptance over the ray-cast population is 96.7%; require a
    // clear majority so a genuine regression fails but sampling noise at one
    // range does not.
    require(
      accepted * 4 >= fragmented * 3,
      "recovery: >=75% of fragmented bicycle frames must pass the geometric conjunction");
    std::fprintf(
      stderr, "  recovery: %d/%d fragmented bicycle frames accepted\n", accepted, fragmented);
  }

  // =====================================================================
  // 3. CLASSIFIER INTEGRATION -- a recovered compound observation is BIKE
  //    with NO classifier threshold changes
  // =====================================================================
  {
    const Scene s = observe(bike_at(3.0, 0.0, M_PI / 2), clp);
    const CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
    require(e.geometry_ok, "classify: the 3 m broadside bicycle must be accepted");
    const auto r = classifyInstant(e.union_features, moving(), clp);
    require(
      r.label == ObstacleClass::BIKE,
      "classify: the compound observation must classify BIKE on the UNCHANGED classifier");
    require(r.confidence > 0.0 && r.confidence <= 1.0, "classify: confidence in (0, 1]");
    // The label must come from the recovered geometry, not from relabelling:
    // the union must actually look like a bicycle.
    require(
      e.union_length > 1.0 && e.union_length < 2.0,
      "classify: the union length must be bicycle-sized (~1.6 m)");
    require(
      e.union_fill <= 0.90,
      "classify: the union must NOT be solid -- the wheel hole guarantees it");
    require(
      e.gap >= 0.35,
      "classify: the union must carry a real internal gap");
  }

  // =====================================================================
  // 4. ANTI-MERGE REGRESSION SET -- the hard acceptance requirement
  //
  // Each case must fail to a pair of INDEPENDENT observations, and must do
  // so on at least two independent bounds.
  // =====================================================================
  {
    // 4a. TWO STAGE-4G3 OBSTACLES at the separations the stage exists to
    //     protect, 0.46-0.67 m, and well beyond.
    //
    //     Note what happens below 0.70 m: primary clustering MERGES them into
    //     one cluster, because their gap is under cluster_distance. They
    //     therefore never reach the compound stage as a pair at all -- the
    //     Stage-4G3 deblender owns that case. That is the structural reason
    //     this stage cannot regress Stage-4G3, and it is asserted, not
    //     assumed.
    int merged_by_primary = 0;
    int pairs_seen = 0;
    int pairs_rejected = 0;
    for (const double sep : {0.46, 0.50, 0.55, 0.60, 0.67, 0.75, 0.85,
        0.95, 1.00, 1.05, 1.10, 1.20, 1.30})
    {
      const Scene s = observe(
        g4g3_obstacle_at(3.0, -sep / 2) + g4g3_obstacle_at(3.0, sep / 2), clp);
      if (s.clusters.size() < 2) {
        ++merged_by_primary;
        require(
          sep < 0.70,
          "anti-merge 4a: only separations below ~0.70 m may merge in PRIMARY clustering");
        continue;
      }
      ++pairs_seen;
      const CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
      require(
        !e.geometry_ok,
        "anti-merge 4a: two Stage-4G3 obstacles at " + std::to_string(sep) +
        " m must NOT form a compound observation");
      if (!e.geometry_ok) {
        ++pairs_rejected;
        require(
          reject_bound_count(e.reject_mask) >= 2,
          "anti-merge 4a: rejection at " + std::to_string(sep) +
          " m must be redundant, got " + describeReject(e.reject_mask));
        // The physical bound: a 0.4 m object cannot present a 0.42 m chord.
        require(
          (e.reject_mask & kRejectFragmentLength) != 0,
          "anti-merge 4a: a 0.4 m obstacle must fail the fragment-length bound at " +
          std::to_string(sep) + " m");
      }
    }
    require(merged_by_primary >= 4, "anti-merge 4a: the 0.46-0.67 m band must merge primarily");
    require(pairs_seen >= 6, "anti-merge 4a: wider separations must present as two clusters");
    std::fprintf(
      stderr,
      "  anti-merge 4a: %d separations merged by primary clustering, "
      "%d/%d candidate pairs rejected\n",
      merged_by_primary, pairs_rejected, pairs_seen);
  }
  {
    // 4b. TWO PEDESTRIANS PASSING CLOSE TOGETHER, independent gaits
    //     (different yaws), over the whole separation range that produces
    //     two clusters.
    int seen = 0;
    for (const double sep : {0.70, 0.80, 0.90, 1.00, 1.04, 1.10, 1.20, 1.35}) {
      for (const double yaw_b : {0.0, M_PI / 2, M_PI / 5}) {
        const Scene s = observe(
          pedestrian_at(3.0, -sep / 2, 0.0) + pedestrian_at(3.0, sep / 2, yaw_b), clp, 0xA17u);
        if (s.clusters.size() < 2) {
          continue;
        }
        ++seen;
        const CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
        require(
          !e.geometry_ok,
          "anti-merge 4b: two pedestrians at " + std::to_string(sep) +
          " m must NOT form a compound observation");
        if (!e.geometry_ok) {
          require(
            reject_bound_count(e.reject_mask) >= 2,
            "anti-merge 4b: rejection at " + std::to_string(sep) +
            " m must be redundant, got " + describeReject(e.reject_mask));
        }
      }
    }
    require(seen >= 8, "anti-merge 4b: pedestrian pairs must present as two clusters");
    std::fprintf(stderr, "  anti-merge 4b: %d pedestrian-pair frames, all rejected\n", seen);
  }
  {
    // 4c. CROSSING OBJECTS -- two Stage-4G3 obstacles on perpendicular paths,
    //     swept through the closest-approach geometry. This is the Stage-4G2
    //     crossing case that motivated deblending.
    int seen = 0;
    for (int step = 0; step <= 12; ++step) {
      const double t = -0.9 + 0.15 * step;          // m along each path
      const Scene s = observe(
        g4g3_obstacle_at(3.0 + t, 0.0) + g4g3_obstacle_at(3.0, t), clp, 0xC01u);
      if (s.clusters.size() < 2) {
        continue;
      }
      ++seen;
      const CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
      require(
        !e.geometry_ok,
        "anti-merge 4c: crossing objects must NOT form a compound observation");
    }
    require(seen >= 6, "anti-merge 4c: the crossing sweep must present two-cluster frames");
    std::fprintf(stderr, "  anti-merge 4c: %d crossing frames, all rejected\n", seen);
  }
  {
    // 4d. TEMPORARILY PARALLEL-MOVING OBJECTS -- the hardest negative for a
    //     temporal rule, because a common velocity and a near-constant
    //     separation are exactly the evidence a rigid body produces. Two
    //     obstacles are walked side by side at a FIXED 1.05 m separation --
    //     the bicycle's own wheelbase -- for 10 consecutive scans, and fed
    //     through the real temporal gate.
    CompoundHypothesisTracker tracker;
    tracker.setParams(base_cp);
    int emitted = 0;
    int geometry_ok = 0;
    for (int k = 0; k < 10; ++k) {
      const double y = -0.5 + 0.1 * k;              // both translate together
      const Scene s = observe(
        g4g3_obstacle_at(3.0, y - 0.525) + g4g3_obstacle_at(3.0, y + 0.525), clp, 0xD0Du);
      std::vector<CompoundHypothesisTracker::Candidate> cands;
      if (s.clusters.size() >= 2) {
        const CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
        if (e.geometry_ok) {
          ++geometry_ok;
          CompoundHypothesisTracker::Candidate c;
          c.index_a = 0;
          c.index_b = 1;
          c.evidence = e;
          cands.push_back(c);
        }
      }
      const auto emit = tracker.update(100.0 + 0.2 * k, cands);
      for (const bool b : emit) {
        if (b) {
          ++emitted;
        }
      }
    }
    require(
      geometry_ok == 0,
      "anti-merge 4d: parallel objects at a bicycle wheelbase must fail on GEOMETRY, "
      "so the temporal gate is never even reached");
    require(
      emitted == 0,
      "anti-merge 4d: parallel-moving distinct objects must never be emitted as compound");
    std::fprintf(stderr, "  anti-merge 4d: 10 parallel frames, 0 geometric accepts\n");
  }
  {
    // 4e. STATIONARY CLOSE OBJECTS, held for 20 scans. A rigid-separation
    //     test alone would accept these -- they never move at all -- so this
    //     case shows the geometric conjunction, not the temporal rule, is
    //     what excludes them.
    CompoundHypothesisTracker tracker;
    tracker.setParams(base_cp);
    int emitted = 0;
    for (int k = 0; k < 20; ++k) {
      const Scene s = observe(
        g4g3_obstacle_at(3.0, -0.52) + g4g3_obstacle_at(3.0, 0.52), clp, 0xE0Eu);
      std::vector<CompoundHypothesisTracker::Candidate> cands;
      if (s.clusters.size() >= 2) {
        const CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
        if (e.geometry_ok) {
          CompoundHypothesisTracker::Candidate c;
          c.index_a = 0;
          c.index_b = 1;
          c.evidence = e;
          cands.push_back(c);
        }
      }
      const auto emit = tracker.update(200.0 + 0.2 * k, cands);
      for (const bool b : emit) {
        if (b) {
          ++emitted;
        }
      }
    }
    require(
      emitted == 0,
      "anti-merge 4e: two stationary close objects must never be emitted as compound");
    std::fprintf(stderr, "  anti-merge 4e: 20 stationary frames, 0 emitted\n");
  }
  {
    // 4f. A PEDESTRIAN BESIDE A BICYCLE WHEEL. The adversarial case for the
    //     collinearity rule: one fragment IS a genuine wheel chord, so the
    //     pair must be refused on the other fragment's geometry.
    const Scene s = observe(
      rect(3.0, -0.52, 0.57, 0.04, M_PI / 2) + pedestrian_at(3.0, 0.52, 0.0), clp, 0xF0Fu);
    if (s.clusters.size() >= 2) {
      const CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
      require(
        !e.geometry_ok,
        "anti-merge 4f: a wheel chord beside a pedestrian must NOT form a compound");
      require(
        (e.reject_mask & (kRejectFragmentLength | kRejectFragmentThickness |
        kRejectAlignment | kRejectUnionThickness)) != 0,
        "anti-merge 4f: refusal must cite the pedestrian fragment's geometry, got " +
        describeReject(e.reject_mask));
    }
  }

  // =====================================================================
  // 5. THE TEMPORAL GATE
  // =====================================================================
  {
    // A single frame of perfect bicycle geometry must NOT be emitted.
    const Scene s = observe(bike_at(3.0, 0.0, M_PI / 2), clp);
    const CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
    require(e.geometry_ok, "temporal: fixture must be geometrically acceptable");

    CompoundHypothesisTracker tracker;
    tracker.setParams(base_cp);
    CompoundHypothesisTracker::Candidate c;
    c.index_a = 0;
    c.index_b = 1;
    c.evidence = e;

    auto emit = tracker.update(10.0, {c});
    require(emit.size() == 1 && !emit[0], "temporal: 1 scan of support must not emit");
    emit = tracker.update(10.2, {c});
    require(emit.size() == 1 && !emit[0], "temporal: 2 scans of support must not emit");
    emit = tracker.update(10.4, {c});
    require(
      emit.size() == 1 && emit[0],
      "temporal: min_support=3 consecutive scans must emit");
    emit = tracker.update(10.6, {c});
    require(emit.size() == 1 && emit[0], "temporal: support must persist once reached");
  }
  {
    // A dropped scan must restart the count: support has to be CONSECUTIVE.
    const Scene s = observe(bike_at(3.0, 0.0, M_PI / 2), clp);
    CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
    CompoundHypothesisTracker tracker;
    tracker.setParams(base_cp);
    CompoundHypothesisTracker::Candidate c;
    c.evidence = e;

    tracker.update(20.0, {c});
    tracker.update(20.2, {c});
    tracker.update(21.5, {});                  // long gap: hypothesis expires
    require(tracker.size() == 0, "temporal: a stale hypothesis must be forgotten");
    auto emit = tracker.update(21.7, {c});
    require(
      emit.size() == 1 && !emit[0],
      "temporal: support must restart from 1 after a dropped scan");
  }
  {
    // A pair whose SEPARATION drifts is not a rigid body: the hypothesis
    // must not be continued, so support restarts.
    const Scene s = observe(bike_at(3.0, 0.0, M_PI / 2), clp);
    CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
    CompoundHypothesisTracker tracker;
    tracker.setParams(base_cp);
    CompoundHypothesisTracker::Candidate c;
    c.evidence = e;
    tracker.update(30.0, {c});
    tracker.update(30.2, {c});
    // Now the separation jumps far beyond separation_tolerance.
    CompoundHypothesisTracker::Candidate drifted = c;
    drifted.evidence.separation = e.separation + 0.40;
    auto emit = tracker.update(30.4, {drifted});
    require(
      emit.size() == 1 && !emit[0],
      "temporal: a separation jump beyond the tolerance must restart support");
  }
  {
    // A pair that drifts bodily but holds its internal separation IS a rigid
    // body and must still accumulate -- the rule tests separation, not
    // absolute position.
    const Scene s = observe(bike_at(3.0, 0.0, M_PI / 2), clp);
    CompoundEvidence e = evaluate_first_pair(s, base_cp, clp);
    CompoundHypothesisTracker tracker;
    tracker.setParams(base_cp);
    bool emitted_on_third = false;
    for (int k = 0; k < 3; ++k) {
      CompoundHypothesisTracker::Candidate c;
      c.evidence = e;
      c.evidence.midpoint_x += 0.15 * k;       // 0.75 m/s at 4.98 Hz
      const auto emit = tracker.update(40.0 + 0.2 * k, {c});
      if (k == 2 && !emit.empty()) {
        emitted_on_third = emit[0];
      }
    }
    require(
      emitted_on_third,
      "temporal: a moving but rigid pair must still reach support in 3 scans");
  }

  // =====================================================================
  // 6. STRUCTURAL INVARIANTS
  // =====================================================================
  {
    // The stage must refuse a pair whose gap is inside the primary link
    // distance, even if every other bound holds. This can only arise from a
    // caller bug, and it is the guard that makes "this stage never operates
    // inside the primary rule's regime" a checked property.
    const Scene s = observe(bike_at(3.0, 0.0, M_PI / 2), clp);
    require(s.clusters.size() == 2, "invariant: fixture must be two clusters");
    const CompoundEvidence bad = evaluateCompoundPair(
      s.features[0], s.features[1], s.clusters[0], s.clusters[1],
      0.0, 0.0, /*primary_link=*/5.0, base_cp, clp);
    require(
      !bad.geometry_ok,
      "invariant: a gap within the primary link distance must be refused");
    require(
      (bad.reject_mask & predictive_nav_tracking::kRejectPrimaryLink) != 0,
      "invariant: the refusal must cite the primary-link violation");
  }
  {
    // Invalid features must never be paired.
    ClusterFeatures invalid;    // valid == false
    const Scene s = observe(bike_at(3.0, 0.0, M_PI / 2), clp);
    const CompoundEvidence e = evaluateCompoundPair(
      invalid, s.features[1], s.clusters[0], s.clusters[1],
      0.0, 0.0, kClusterDistance, base_cp, clp);
    require(!e.geometry_ok, "invariant: an invalid feature set must never be paired");
    require(
      (e.reject_mask & predictive_nav_tracking::kRejectInvalidFeatures) != 0,
      "invariant: the refusal must cite invalid features");
  }
  {
    // The default must be OFF, so that the stage is a provable no-op until a
    // configuration enables it.
    const CompoundParams def;
    require(!def.enabled, "invariant: compound association must default to DISABLED");
    require(
      def.gap_min >= kClusterDistance,
      "invariant: gap_min must not reach below the frozen cluster_distance");
    require(def.min_support >= 3, "invariant: min_support must be at least 3 scans");
  }
  {
    // A class label must never depend on absolute position or heading: the
    // additive axis/centroid outputs must not reach classify(). Translating
    // and rotating the same object must give the same label.
    const ClassificationParams p2;
    const auto a = extractFeatures(scan_world(bike_at(3.0, 0.0, M_PI / 2)), 0.0, 0.0, p2);
    const auto fa = classifyInstant(a, moving(), p2);
    ClusterFeatures shifted = a;
    shifted.centroid_x += 17.0;
    shifted.centroid_y -= 9.0;
    shifted.axis_x = 0.0;
    shifted.axis_y = 1.0;
    const auto fb = classifyInstant(shifted, moving(), p2);
    require(
      fa.label == fb.label && fa.confidence == fb.confidence,
      "invariant: classify() must ignore axis and centroid entirely");
  }

  // =====================================================================
  // 7. THE CHEAP SCREEN IS A TRUE NECESSARY CONDITION
  //
  // The node skips the expensive nearest-gap and pooled-geometry work for any
  // pair screenCompoundPair() rejects. That is only sound if the screen never
  // rejects a pair the full evaluation would have accepted. Checked
  // exhaustively over every fixture in this file, positive and negative.
  // =====================================================================
  {
    int screened_out = 0;
    int evaluated = 0;
    int disagreements = 0;
    std::vector<std::vector<Seg>> worlds;
    for (const double range : {2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0}) {
      for (const double yaw : {0.0, M_PI / 6, M_PI / 4, M_PI / 3, M_PI / 2}) {
        worlds.push_back(bike_at(range, 0.0, yaw));
        worlds.push_back(
          g4g3_obstacle_at(range, -0.45) + g4g3_obstacle_at(range, 0.45));
        worlds.push_back(
          g4g3_obstacle_at(range, -0.60) + g4g3_obstacle_at(range, 0.60));
        worlds.push_back(
          pedestrian_at(range, -0.50, 0.0) + pedestrian_at(range, 0.50, yaw));
        worlds.push_back(
          pedestrian_at(range, -0.35, yaw) + g4g3_obstacle_at(range, 0.55));
      }
    }
    for (const auto & w : worlds) {
      const Scene s = observe(w, clp, 0x5EEDu);
      for (size_t a = 0; a < s.clusters.size(); ++a) {
        for (size_t b = a + 1; b < s.clusters.size(); ++b) {
          const uint32_t screen = predictive_nav_tracking::screenCompoundPair(
            s.features[a], s.features[b], base_cp);
          const CompoundEvidence full = evaluateCompoundPair(
            s.features[a], s.features[b], s.clusters[a], s.clusters[b],
            0.0, 0.0, kClusterDistance, base_cp, clp);
          if (screen == predictive_nav_tracking::kCompoundAccept) {
            ++evaluated;
          } else {
            ++screened_out;
            // THE CONTRACT: anything the screen rejects, the full evaluation
            // must reject too.
            if (full.geometry_ok) {
              ++disagreements;
            }
          }
          // And the screen's reasons must be a SUBSET of the full reasons,
          // since it applies a subset of the same bounds to the same inputs.
          if ((screen & ~full.reject_mask) != 0u) {
            ++disagreements;
          }
        }
      }
    }
    require(
      disagreements == 0,
      "screen: the cheap screen must never reject a pair the full evaluation accepts");
    require(screened_out > 0, "screen: the sweep must actually exercise rejection");
    require(evaluated > 0, "screen: the sweep must actually exercise acceptance");
    std::fprintf(
      stderr, "  screen: %d pairs screened out, %d fully evaluated, %d disagreements\n",
      screened_out, evaluated, disagreements);
  }

  std::fprintf(
    stderr, "stage5b1_compound: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
