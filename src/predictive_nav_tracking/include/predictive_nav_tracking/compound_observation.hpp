// Stage-5B.1: gap-aware compound-observation recovery for fragmented objects.
//
// WHAT THIS IS
// ------------
// A conservative SECONDARY association stage that sits AFTER the frozen
// primary clustering and BEFORE the frozen association/tracking stage. It
// looks for pairs of separate primary clusters that are, on measured geometry
// and measured temporal behaviour, two slices of ONE rigid object, and emits a
// single COMPOUND OBSERVATION for such a pair.
//
// WHAT THIS IS NOT
// ----------------
// It is NOT a relaxation of clustering. `cluster_distance` is untouched and
// the primary connected-component rule runs exactly as before. It is NOT a
// general cluster merge: it accepts only pairs that satisfy a conjunction of
// nine independent bounds AND hold that conjunction over consecutive scans.
// It does NOT read ground truth, object class, or Gazebo model names.
//
// THE PROBLEM, MEASURED
// ---------------------
// A bicycle at a ~0.16 m scan plane is cut below both hubs, so the sensor sees
// two wheel CHORDS separated by the empty span where the frame passes above
// the plane. Measured on the Stage-5B validation set (291 two-track frames
// over 6 scenarios): the two fragments sit 1.037-1.043 m apart (the wheelbase)
// and their nearest returns are ~0.48-0.54 m apart. Since that gap exceeds
// cluster_distance = 0.35 m, single-link clustering splits the bicycle into
// two clusters, each ~0.51 m long and ~0.03 m thick -- i.e. each one is
// pedestrian-sized, and the downstream classifier correctly reports what it
// was given: two pedestrian-like objects. BIKE recall was therefore 0.000.
//
// WHY RAISING cluster_distance IS NOT THE FIX
// -------------------------------------------
// Stage-4G3 established, on 1,197 recorded frames, that two genuinely
// distinct objects at centre separations of 0.46-0.76 m produce an
// inter-object gap of 0.12-0.35 m. Raising cluster_distance above the
// bicycle's 0.48 m gap would therefore also merge those, destroying the
// close-object separation that Stage-4G3 exists to protect. That parameter
// stays at 0.35 m.
//
// WHY THIS STAGE IS STRUCTURALLY SAFE
// -----------------------------------
// It only ever considers pairs of clusters that primary clustering ALREADY
// chose to keep apart. By construction, two separate primary clusters have a
// nearest-point gap strictly greater than cluster_distance. The Stage-4G3
// hazard geometry lives at gaps of 0.12-0.35 m -- BELOW cluster_distance --
// which means it arrives at this stage as ONE cluster and is handled by the
// Stage-4G3 deblender, never as a candidate pair here. This stage cannot
// re-merge what Stage-4G3 protects, because it never sees it as two things.
// `requireGapAbovePrimaryLink` enforces that invariant at runtime rather than
// leaving it as a comment.
//
// THE DISCRIMINATOR, AND ITS MEASURED MARGIN
// ------------------------------------------
// Ray-cast reference geometry (360 beams, 6.28 rad span, nearest hit only,
// 0.01 m range noise, random aspect and range over 2.0-5.5 m), 333 bicycle
// frames against 633 genuinely-distinct close-pair frames (two 0.4 m
// Stage-4G3 cylinders, and two Stage-5B pedestrians, at separations that do
// produce two clusters):
//
//   feature                        bicycle pair      distinct-object pairs
//   ---------------------------------------------------------------------
//   per-fragment thickness, m      0.002 - 0.071     0.022 - 0.185
//   longer fragment length, m      0.382 - 0.587     0.131 - 0.392
//   |cos| fragment axis vs pair    0.992 - 1.000     0.003 - 0.999
//   union thickness, m             0.007 - 0.073     0.048 - 0.457
//   centroid separation, m         0.932 - 1.157     0.490 - 1.637
//
// Three bounds separate the populations, and they are PHYSICALLY, not
// empirically, motivated:
//
//   1. FRAGMENT LENGTH >= 0.42 m. A wheel chord at this plane is
//      2*sqrt(r^2 - (r-h)^2) = 0.57 m for r = 0.335 m, h = 0.16 m. A 0.4 m
//      diameter object CANNOT present a principal-axis extent above ~0.40 m,
//      so this bound excludes the entire Stage-4G3 obstacle population by
//      geometry rather than by threshold choice. Measured: every one of 341
//      cylinder-pair frames failed it (max 0.392 m).
//
//   2. UNION THICKNESS <= 0.08 m. Both wheels of a bicycle lie in ONE plane,
//      so the union of the two fragments is still a line. Two separate
//      objects do not share a plane and their union acquires real depth.
//      Measured: every one of 292 pedestrian-pair frames failed it
//      (min 0.090 m vs bicycle max 0.073 m -- the populations do not touch).
//
//   3. COLLINEARITY |cos| >= 0.98 for BOTH fragments. Each wheel chord lies
//      along the direction of travel, which is also the direction from one
//      wheel to the other, so a bicycle's fragment axes and its pair axis are
//      the same line. Two independent objects have no such constraint.
//
// Each negative frame failed 2-3 of these bounds independently, not one, so no
// single threshold is load-bearing. Acceptance: 322/333 bicycle frames
// (96.7%), 0/633 distinct-object frames (0.0%).
//
// WHY A SINGLE FRAME IS NOT ENOUGH
// --------------------------------
// The nine geometric bounds above describe an instantaneous silhouette, and
// two unrelated objects can momentarily fall inside all of them. What they
// cannot do is stay there: a RIGID body holds its fragment separation
// constant, whereas two independently-moving objects do not. Measured on the
// real validation data, a bicycle's fragment separation has a standard
// deviation of 0.017-0.025 m over 28-77 frames. This stage therefore requires
// the geometric conjunction to hold on `min_support` CONSECUTIVE scans with
// the separation stable to within `separation_tolerance`.
//
// min_support = 3 is chosen to match the tracker's own
// min_observations_to_publish = 3 and the classifier's min_observations = 3,
// so a compound observation becomes classifiable at exactly the scan on which
// any track becomes publishable -- the temporal requirement costs NO extra
// latency relative to the existing pipeline. At the measured 4.98 Hz that is
// 0.60 s, and the shortest bicycle scenario in the validation set still
// supplies 28 frames.

#ifndef PREDICTIVE_NAV_TRACKING__COMPOUND_OBSERVATION_HPP_
#define PREDICTIVE_NAV_TRACKING__COMPOUND_OBSERVATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "predictive_nav_tracking/obstacle_classification.hpp"

namespace predictive_nav_tracking
{

/// Every bound this stage applies, with its unit and physical meaning. All
/// are parameters rather than constants, so a deployment with a different
/// sensor height or differently sized objects can rescale them.
struct CompoundParams
{
  /// Master switch. DEFAULT false: with this false the stage is a provable
  /// no-op and the pipeline is bit-identical to the Stage-5B checkpoint.
  bool enabled{false};

  // --- per-fragment gates: "is this slice a wheel chord?" ------------------

  /// Minimum principal-axis extent of EACH fragment, m. A wheel chord at a
  /// 0.16 m plane measures 0.57 m nominal, 0.478-0.587 m as sampled. A 0.4 m
  /// diameter object cannot exceed ~0.40 m, so this is the bound that
  /// excludes the Stage-4G3 obstacle by geometry.
  double fragment_length_min{0.42};

  /// Maximum principal-axis extent of each fragment, m. Above this the slice
  /// is longer than any bicycle wheel and the pair is something else.
  double fragment_length_max{0.80};

  /// Maximum perpendicular extent of each fragment, m. A tyre is ~0.04 m
  /// across; this allows noise and oblique sampling but excludes any slice
  /// with real depth.
  double fragment_thickness_max{0.10};

  // --- pair gates: "are these two slices one rigid frame?" -----------------

  /// Minimum |cos| between EACH fragment's principal axis and the
  /// fragment-to-fragment direction, dimensionless in [0, 1]. 0.98 is 11.5
  /// degrees. Measured bicycle minimum 0.992; pedestrian pairs reach 0.953 at
  /// the 95th percentile.
  double alignment_min{0.98};

  /// Admissible centroid separation, m -- a plausible WHEELBASE. 1.05 m
  /// nominal, 0.932-1.157 m as sampled over all aspects and ranges.
  double separation_min{0.80};
  double separation_max{1.35};

  /// Admissible nearest-point gap between the two fragments, m. `gap_min` is
  /// a floor only: the true lower bound is the primary cluster_distance,
  /// which `requireGapAbovePrimaryLink` enforces. `gap_max` bounds how much
  /// empty space the hypothesis is willing to span.
  double gap_min{0.35};
  double gap_max{0.80};

  // --- union gates: "is the combined footprint a bicycle?" -----------------

  /// Admissible principal-axis extent of the UNION of both fragments, m.
  /// Wheelbase plus two chords: 1.62 m nominal, 1.334-1.647 m as sampled.
  double union_length_min{1.00};
  double union_length_max{2.00};

  /// Maximum perpendicular extent of the union, m. THE decisive bound against
  /// two separate objects: both wheels share one plane, two objects do not.
  /// Measured bicycle max 0.073; pedestrian-pair min 0.090.
  double union_thickness_max{0.08};

  /// Maximum fill ratio of the union, dimensionless. A bicycle cannot be
  /// solid -- the hole between the wheels guarantees it. Measured 0.667-0.978.
  double union_fill_max{0.90};

  // --- temporal gates ------------------------------------------------------

  /// Consecutive scans the geometric conjunction must hold before a compound
  /// observation is emitted, count. See the header note on why this is 3.
  uint32_t min_support{3};

  /// Maximum change in fragment separation between consecutive supporting
  /// scans, m. The rigid-body test. Measured per-scenario standard deviation
  /// 0.017-0.025 m, so 0.12 m is ~5 sigma -- loose enough never to drop a
  /// real bicycle, far tighter than two independently-moving objects hold.
  double separation_tolerance{0.12};

  /// Maximum movement of the pair MIDPOINT between consecutive scans, m, used
  /// to recognise the same hypothesis again. At 4.98 Hz an actor at the
  /// validation set's fastest 0.75 m/s moves 0.151 m per scan, so 0.35 m
  /// carries a 2.3x margin while staying below the association gate.
  double match_radius{0.35};

  /// A hypothesis not re-observed within this interval is forgotten, s.
  /// Slightly over one scan period at 4.98 Hz, so support must be
  /// CONSECUTIVE: one dropped scan restarts the count.
  double max_gap_seconds{0.30};
};

/// Which bound(s) a candidate pair failed. Bitmask so that a rejection can be
/// reported with its full reason set rather than just the first failure --
/// this is what makes the heuristic auditable, and it is how the regression
/// tests assert that a negative case is rejected REDUNDANTLY rather than by a
/// single marginal threshold.
enum CompoundRejectBits : uint32_t
{
  kCompoundAccept           = 0u,
  kRejectInvalidFeatures    = 1u << 0,
  kRejectFragmentLength     = 1u << 1,
  kRejectFragmentThickness  = 1u << 2,
  kRejectAlignment          = 1u << 3,
  kRejectSeparation         = 1u << 4,
  kRejectGap                = 1u << 5,
  kRejectUnionLength        = 1u << 6,
  kRejectUnionThickness     = 1u << 7,
  kRejectUnionFill          = 1u << 8,
  kRejectPrimaryLink        = 1u << 9,
};

/// The measured quantities behind one accept/reject decision. Every field is
/// something the sensor produced this scan; nothing here is ground truth.
struct CompoundEvidence
{
  /// True when `reject_mask == kCompoundAccept`, i.e. every geometric bound
  /// held. This is the SINGLE-FRAME verdict only -- the temporal requirement
  /// is applied by CompoundHypothesisTracker.
  bool geometry_ok{false};
  uint32_t reject_mask{kRejectInvalidFeatures};

  double separation{0.0};        ///< m, centroid to centroid
  double alignment{0.0};         ///< min |cos| over the two fragments
  double gap{0.0};               ///< m, nearest member point to nearest member point
  double union_length{0.0};      ///< m
  double union_thickness{0.0};   ///< m
  double union_fill{0.0};        ///< dimensionless
  double midpoint_x{0.0};        ///< m, map frame
  double midpoint_y{0.0};        ///< m, map frame

  /// Geometry of the POOLED member points, i.e. what the compound
  /// observation looks like as a single cluster. Produced by the same
  /// extractFeatures() the classifier consumes, so an accepted compound
  /// observation is described by exactly the code that will later classify
  /// it -- there is no second, approximate geometry path. Its centroid is
  /// the compound observation's measured POSITION.
  ClusterFeatures union_features{};
};

/// Human-readable reason set, for logs and test failure messages.
inline std::string describeReject(uint32_t mask)
{
  if (mask == kCompoundAccept) {
    return "accept";
  }
  std::string s;
  const auto add = [&s](const char * name) {
      if (!s.empty()) {
        s += "|";
      }
      s += name;
    };
  if (mask & kRejectInvalidFeatures) {add("invalid_features");}
  if (mask & kRejectFragmentLength) {add("fragment_length");}
  if (mask & kRejectFragmentThickness) {add("fragment_thickness");}
  if (mask & kRejectAlignment) {add("alignment");}
  if (mask & kRejectSeparation) {add("separation");}
  if (mask & kRejectGap) {add("gap");}
  if (mask & kRejectUnionLength) {add("union_length");}
  if (mask & kRejectUnionThickness) {add("union_thickness");}
  if (mask & kRejectUnionFill) {add("union_fill");}
  if (mask & kRejectPrimaryLink) {add("primary_link_violation");}
  return s;
}

/// Minimal point type used to pool two clusters' member points. Matches the
/// `.x`/`.y` duck type extractFeatures() accepts.
struct Point2DLike
{
  double x{0.0};
  double y{0.0};
};

/// Nearest-point distance between two point sets, m. This is the gap the
/// PRIMARY clustering rule itself measured when it decided to keep the two
/// clusters apart, so comparing it against cluster_distance is exact rather
/// than approximate.
template<typename PointContainer>
inline double nearestGap(const PointContainer & a, const PointContainer & b)
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & p : a) {
    for (const auto & q : b) {
      best = std::min(best, std::hypot(p.x - q.x, p.y - q.y));
    }
  }
  return best;
}

/// CHEAP NECESSARY CONDITION on a candidate pair, from the summary features
/// alone -- no member points are touched.
///
/// WHY THIS EXISTS: evaluateCompoundPair() below measures the nearest-point
/// gap between the two clusters and re-extracts the geometry of their pooled
/// points, which costs O(|A| * |B|) distance computations plus one feature
/// extraction. Enumerating every pair of clusters in a busy scan and paying
/// that for all of them would make this stage the most expensive thing in the
/// scan callback, for no benefit: the overwhelming majority of pairs are
/// excluded by bounds that cost a handful of floating-point operations.
///
/// The bounds applied here are exactly the subset of the full conjunction
/// that depends only on `fa` and `fb`, and they are applied IDENTICALLY in
/// evaluateCompoundPair(). This function is therefore a pure necessary
/// condition: skipping a pair it rejects cannot change any outcome, only the
/// work done to reach it. The full evaluation remains the single source of
/// truth, and the tests drive that, not this.
///
/// Returns the partial reject mask: kCompoundAccept means "worth measuring
/// properly", NOT "accepted".
inline uint32_t screenCompoundPair(
  const ClusterFeatures & fa, const ClusterFeatures & fb, const CompoundParams & cp)
{
  if (!fa.valid || !fb.valid) {
    return kRejectInvalidFeatures;
  }
  uint32_t mask = kCompoundAccept;

  const double dx = fb.centroid_x - fa.centroid_x;
  const double dy = fb.centroid_y - fa.centroid_y;
  const double sep = std::hypot(dx, dy);
  if (!(sep > 1e-9) || !std::isfinite(sep)) {
    return kRejectSeparation;
  }
  if (!(sep >= cp.separation_min && sep <= cp.separation_max)) {
    mask |= kRejectSeparation;
  }

  const double len_lo = std::min(fa.length, fb.length);
  const double len_hi = std::max(fa.length, fb.length);
  if (!(len_lo >= cp.fragment_length_min && len_hi <= cp.fragment_length_max)) {
    mask |= kRejectFragmentLength;
  }
  if (!(std::max(fa.thickness, fb.thickness) <= cp.fragment_thickness_max)) {
    mask |= kRejectFragmentThickness;
  }

  const double ux = dx / sep;
  const double uy = dy / sep;
  const double cos_a = std::abs(fa.axis_x * ux + fa.axis_y * uy);
  const double cos_b = std::abs(fb.axis_x * ux + fb.axis_y * uy);
  if (!(std::min(cos_a, cos_b) >= cp.alignment_min)) {
    mask |= kRejectAlignment;
  }
  return mask;
}

/// Single-frame evaluation of one candidate pair.
///
/// `fa`, `fb` are the ALREADY-EXTRACTED features of the two primary clusters
/// (so the principal axes and centroids come from the same code path the
/// classifier uses), `pa`, `pb` their member points, and `sx, sy` the sensor
/// origin for this scan.
///
/// `primary_link` is the live `cluster_distance`. The returned evidence is
/// rejected with kRejectPrimaryLink if the measured gap does not exceed it,
/// which can only happen if this function is handed two clusters that primary
/// clustering would have joined -- a caller bug. Checking it here means the
/// invariant "this stage never operates inside the primary rule's regime" is
/// enforced by code rather than asserted in prose.
template<typename PointContainer>
inline CompoundEvidence evaluateCompoundPair(
  const ClusterFeatures & fa, const ClusterFeatures & fb,
  const PointContainer & pa, const PointContainer & pb,
  double sx, double sy, double primary_link,
  const CompoundParams & cp, const ClassificationParams & clp)
{
  (void)sx;
  (void)sy;
  CompoundEvidence e;
  if (!fa.valid || !fb.valid) {
    e.reject_mask = kRejectInvalidFeatures;
    return e;
  }

  uint32_t mask = kCompoundAccept;

  // --- pair geometry ------------------------------------------------------
  const double dx = fb.centroid_x - fa.centroid_x;
  const double dy = fb.centroid_y - fa.centroid_y;
  const double sep = std::hypot(dx, dy);
  e.separation = sep;
  e.midpoint_x = 0.5 * (fa.centroid_x + fb.centroid_x);
  e.midpoint_y = 0.5 * (fa.centroid_y + fb.centroid_y);
  if (!(sep > 1e-9) || !std::isfinite(sep)) {
    e.reject_mask = kRejectSeparation;
    return e;
  }
  const double ux = dx / sep;
  const double uy = dy / sep;

  // |dot|, because a principal axis has no preferred sign.
  const double cos_a = std::abs(fa.axis_x * ux + fa.axis_y * uy);
  const double cos_b = std::abs(fb.axis_x * ux + fb.axis_y * uy);
  e.alignment = std::min(cos_a, cos_b);

  e.gap = nearestGap(pa, pb);

  // --- union geometry, measured on the pooled member points ---------------
  // Deliberately recomputed with the SAME extractFeatures() the classifier
  // uses, rather than estimated from the two fragments' summary statistics:
  // the compound observation must be described by exactly the code that will
  // later classify it.
  std::vector<Point2DLike> pooled;
  pooled.reserve(
    static_cast<size_t>(std::distance(std::begin(pa), std::end(pa))) +
    static_cast<size_t>(std::distance(std::begin(pb), std::end(pb))));
  for (const auto & p : pa) {
    pooled.push_back({p.x, p.y});
  }
  for (const auto & p : pb) {
    pooled.push_back({p.x, p.y});
  }
  const ClusterFeatures uf = extractFeatures(pooled, sx, sy, clp);
  if (!uf.valid) {
    e.reject_mask = kRejectInvalidFeatures;
    return e;
  }
  e.union_features = uf;
  e.union_length = uf.length;
  e.union_thickness = uf.thickness;
  e.union_fill = uf.fill_ratio;

  // --- the conjunction ----------------------------------------------------
  // Every bound is evaluated; none short-circuits. A rejected pair therefore
  // reports EVERY bound it failed, which is what lets the regression tests
  // show that a negative case is excluded redundantly.
  const double len_lo = std::min(fa.length, fb.length);
  const double len_hi = std::max(fa.length, fb.length);
  if (!(len_lo >= cp.fragment_length_min && len_hi <= cp.fragment_length_max)) {
    mask |= kRejectFragmentLength;
  }
  if (!(std::max(fa.thickness, fb.thickness) <= cp.fragment_thickness_max)) {
    mask |= kRejectFragmentThickness;
  }
  if (!(e.alignment >= cp.alignment_min)) {
    mask |= kRejectAlignment;
  }
  if (!(sep >= cp.separation_min && sep <= cp.separation_max)) {
    mask |= kRejectSeparation;
  }
  if (!(e.gap >= cp.gap_min && e.gap <= cp.gap_max)) {
    mask |= kRejectGap;
  }
  if (!(uf.length >= cp.union_length_min && uf.length <= cp.union_length_max)) {
    mask |= kRejectUnionLength;
  }
  if (!(uf.thickness <= cp.union_thickness_max)) {
    mask |= kRejectUnionThickness;
  }
  if (!(uf.fill_ratio <= cp.union_fill_max)) {
    mask |= kRejectUnionFill;
  }
  // The structural invariant. Two separate primary clusters always satisfy
  // this; a violation means the caller fabricated a pair.
  if (!(e.gap > primary_link)) {
    mask |= kRejectPrimaryLink;
  }

  e.reject_mask = mask;
  e.geometry_ok = (mask == kCompoundAccept);
  return e;
}

/// Temporal gate over single-frame candidates.
///
/// Holds one entry per pair hypothesis currently under observation, keyed by
/// nothing: hypotheses are re-recognised frame to frame by their MIDPOINT and
/// their SEPARATION, both of which a rigid body preserves. Nothing here
/// touches a track, a Kalman filter or a class label -- this is an
/// independent, stateless-per-object accumulator whose only output is
/// "this pair has now held together for long enough".
class CompoundHypothesisTracker
{
public:
  struct Hypothesis
  {
    double midpoint_x{0.0};
    double midpoint_y{0.0};
    double separation{0.0};
    uint32_t support{0};        ///< consecutive supporting scans, count
    double last_stamp{0.0};     ///< s
    bool matched_this_scan{false};
  };

  /// One candidate pair, as the caller found it this scan.
  struct Candidate
  {
    size_t index_a{0};          ///< caller's cluster index
    size_t index_b{0};
    CompoundEvidence evidence;
  };

  /// Feed this scan's geometrically-accepted candidates and get back, for
  /// each, whether it has now accumulated enough consecutive support to be
  /// emitted as a compound observation.
  ///
  /// `stamp` is seconds. Candidates whose geometry failed must NOT be passed:
  /// a scan on which the conjunction broke is a scan on which support is
  /// lost, and the caller signals that simply by omitting the pair.
  std::vector<bool> update(double stamp, const std::vector<Candidate> & candidates)
  {
    for (auto & h : hypotheses_) {
      h.matched_this_scan = false;
    }

    std::vector<bool> emit(candidates.size(), false);

    for (size_t i = 0; i < candidates.size(); ++i) {
      const CompoundEvidence & e = candidates[i].evidence;
      Hypothesis * best = nullptr;
      double best_d = std::numeric_limits<double>::infinity();
      for (auto & h : hypotheses_) {
        if (h.matched_this_scan) {
          continue;
        }
        // Consecutive-scan requirement: a hypothesis that went unobserved for
        // longer than one scan period cannot be continued, only restarted.
        if (stamp - h.last_stamp > params_.max_gap_seconds) {
          continue;
        }
        // The rigid-body test. Both terms must hold: a pair may drift
        // bodily (the object is moving) but its internal separation may not.
        if (std::abs(h.separation - e.separation) > params_.separation_tolerance) {
          continue;
        }
        const double d = std::hypot(h.midpoint_x - e.midpoint_x, h.midpoint_y - e.midpoint_y);
        if (d > params_.match_radius) {
          continue;
        }
        if (d < best_d) {
          best_d = d;
          best = &h;
        }
      }

      if (best != nullptr) {
        best->matched_this_scan = true;
        best->support = (best->support == std::numeric_limits<uint32_t>::max()) ?
          best->support : best->support + 1;
        best->midpoint_x = e.midpoint_x;
        best->midpoint_y = e.midpoint_y;
        best->separation = e.separation;
        best->last_stamp = stamp;
        emit[i] = (best->support >= params_.min_support);
      } else {
        Hypothesis h;
        h.midpoint_x = e.midpoint_x;
        h.midpoint_y = e.midpoint_y;
        h.separation = e.separation;
        h.support = 1;
        h.last_stamp = stamp;
        h.matched_this_scan = true;
        hypotheses_.push_back(h);
        emit[i] = (h.support >= params_.min_support);
      }
    }

    // Forget hypotheses that have gone stale. A pair that breaks the
    // conjunction for even one scan therefore loses all accumulated support.
    hypotheses_.erase(
      std::remove_if(
        hypotheses_.begin(), hypotheses_.end(),
        [&](const Hypothesis & h) {
          return stamp - h.last_stamp > params_.max_gap_seconds;
        }),
      hypotheses_.end());

    return emit;
  }

  void setParams(const CompoundParams & p) {params_ = p;}
  const CompoundParams & params() const {return params_;}
  size_t size() const {return hypotheses_.size();}
  void clear() {hypotheses_.clear();}
  const std::vector<Hypothesis> & hypotheses() const {return hypotheses_;}

private:
  CompoundParams params_;
  std::vector<Hypothesis> hypotheses_;
};

}  // namespace predictive_nav_tracking

#endif  // PREDICTIVE_NAV_TRACKING__COMPOUND_OBSERVATION_HPP_
