// Stage-5B: geometry/motion obstacle classification for 2D LiDAR tracks.
//
// WHAT THIS IS
// ------------
// A deterministic, fully explainable classifier that assigns one of
//   PEDESTRIAN / BIKE / VEHICLE / UNKNOWN
// to an existing dynamic-obstacle track, using ONLY quantities a single-plane
// 2D LiDAR and the existing constant-velocity tracker actually measure.
//
// WHAT THIS IS NOT
// ----------------
// It is NOT a learned model, NOT a neural network, and NOT semantic
// perception. It does not use object height, because the sensor is a single
// horizontal scan plane and height is simply not observable (see SCAN PLANE
// below). It does not alter the track state, the Kalman filter, the
// predictions or the reachable sets: classification is pure metadata derived
// from them.
//
// SCAN PLANE -- WHAT A 2D LIDAR ACTUALLY SEES
// -------------------------------------------
// The TurtleBot3 waffle LDS sits ~0.16 m above the floor and sweeps one
// horizontal plane (360 beams over 2*pi, i.e. 0.017453 rad between beams).
// At 0.16 m the beam does NOT cut an object at its widest or most
// recognisable point; it cuts it near the ground:
//
//   pedestrian : ankles / lower calves. One or two small blobs, together
//                spanning at most a stride width. COMPACT and SMALL.
//   bicycle    : the two tyre contact patches, separated by the wheelbase.
//                LONG, very THIN, and with a large EMPTY GAP in the middle
//                where the frame passes above the scan plane.
//   vehicle    : the lower body panel / bumper and the tyres. LONG, and --
//                because a corner view exposes two faces -- genuinely THICK,
//                with a CONTINUOUS return and no large internal gap.
//
// Those three signatures separate on geometry alone, which is the entire
// basis of this classifier. Every threshold below is an interval on a
// physically measurable dimension of that slice, with its unit stated.
//
// WHY SPEED IS ONLY A WEAK PRIOR
// ------------------------------
// Real pedestrians, cyclists and cars occupy very different speed bands, and
// `speed_max` per class below encodes that. In THIS simulation, however, the
// scenario driver commands every actor at a comparable 0.15-0.50 m/s, so the
// measured speed distributions overlap almost completely and speed carries
// almost no discriminative information. It is therefore used only as a VETO
// on physically impossible motion (a 'pedestrian' at 8 m/s is not a
// pedestrian), never as positive evidence. Geometry decides. This is a
// property of the available data and is reported as such rather than tuned
// around.
//
// DECISION RULE
// -------------
// Each class owns a box of trapezoidal membership intervals, one per feature.
// A feature's membership is 1.0 inside the physically expected interval and
// ramps linearly to 0.0 across a soft band outside it, so a cluster near a
// boundary degrades smoothly instead of flipping. A class's score is the
// MINIMUM of its feature memberships -- a conjunction: "long AND thin AND
// gapped" must all hold for BIKE, and one disqualifying feature is enough to
// rule the class out. The winning class is accepted only when
//
//   (a) enough evidence exists at all   : point_count >= min_points
//                                         and range <= max_range
//   (b) its score is decisive           : best >= min_accept_score
//   (c) it beats its rival clearly      : best - second >= min_margin
//
// and UNKNOWN is returned otherwise. (b) and (c) are what keep the classifier
// honest: a cluster that is merely "not obviously anything" stays UNKNOWN
// rather than being forced into the nearest semantic class.
//
// Confidence is reported as best * (1 - second/best) in [0, 1]: it rises with
// absolute fit and falls as a rival class becomes competitive. UNKNOWN
// returned for lack of evidence carries confidence 0.0.

#ifndef PREDICTIVE_NAV_TRACKING__OBSTACLE_CLASSIFICATION_HPP_
#define PREDICTIVE_NAV_TRACKING__OBSTACLE_CLASSIFICATION_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace predictive_nav_tracking
{

/// Semantic label. Values are wire-stable: they are published directly in
/// TrackedObject.classification and must never be renumbered.
enum class ObstacleClass : uint8_t
{
  UNKNOWN = 0,
  PEDESTRIAN = 1,
  BIKE = 2,
  VEHICLE = 3,
};

/// Number of semantic (non-UNKNOWN) classes.
constexpr size_t kNumSemanticClasses = 3;

inline const char * toString(ObstacleClass c)
{
  switch (c) {
    case ObstacleClass::PEDESTRIAN: return "PEDESTRIAN";
    case ObstacleClass::BIKE: return "BIKE";
    case ObstacleClass::VEHICLE: return "VEHICLE";
    default: return "UNKNOWN";
  }
}

/// Geometric descriptor of ONE cluster in ONE scan, in the map frame.
///
/// Every field is computed from the cluster's member beam endpoints and the
/// sensor origin for that scan. Nothing here requires information a 2D LiDAR
/// does not provide; in particular there is no height, no volume and no
/// vertical extent.
struct ClusterFeatures
{
  /// Extent along the cluster's principal axis, m. Second-moment
  /// (PCA) major axis of the member points, measured as the full
  /// min-to-max projection onto that axis -- the object's apparent LENGTH in
  /// the scan plane.
  double length{0.0};

  /// Extent along the perpendicular (minor) axis, m. The apparent THICKNESS
  /// or DEPTH of the slice. A flat surface seen broadside gives ~0; a corner
  /// view of a box-shaped body gives a real positive depth.
  double thickness{0.0};

  /// length / max(thickness, thickness_floor), dimensionless (>= 1).
  ///
  /// DIAGNOSTIC ONLY -- published and logged, but deliberately NOT used in
  /// the decision rule. Measured on ray-cast reference geometry, the minor
  /// axis of any flat or broadside view is zero to within the sensor's 0.01 m
  /// range noise, so `thickness` sits on `thickness_floor` and this ratio
  /// degenerates to 20x`length`: it carries no information that `length` does
  /// not already carry (a 3.8 m car flank measures 76, a 0.3 m leg 6). Where
  /// the minor axis IS informative -- a corner view of a box-shaped body --
  /// `thickness` is used directly instead. Including the ratio in the
  /// conjunction would add a constraint that only tracks absolute size.
  double elongation{1.0};

  /// Number of beam returns in the cluster, count.
  uint32_t point_count{0};

  /// Euclidean distance from the sensor origin to the cluster centroid, m.
  /// Governs how many beams can possibly strike the object, so it is required
  /// to interpret point_count and fill_ratio.
  double range{0.0};

  /// Angular extent subtended at the sensor by the cluster, rad
  /// (max bearing - min bearing over the member points, unwrapped).
  double angular_span{0.0};

  /// point_count divided by the number of beams that could have struck the
  /// cluster's angular span, dimensionless in (0, 1].
  ///
  ///   expected_beams = angular_span / angle_increment + 1
  ///   fill_ratio     = min(1, point_count / expected_beams)
  ///
  /// THE CLAMP IS NOT COSMETIC. A fraction of available beams that returned
  /// cannot exceed 1, but the raw quotient can, because `angular_span` is
  /// measured from the returns' own bearings while `angle_increment` is the
  /// driver's nominal spacing, and the two disagree when a scan contains two
  /// beams at nearly the same bearing. This LiDAR does: its 360 samples span
  /// 6.28 rad, not 2*pi, so beam 0 and beam 359 are 2*pi - 6.28 = 0.0032 rad
  /// apart -- 0.18 of one increment. A cluster straddling that direction
  /// (which, for a sensor looking down its own x axis, means anything
  /// straight ahead) therefore contributes one extra return while adding only
  /// 0.18 increments of span, deflating expected_beams by ~0.8 beams.
  /// Measured on the Stage-5B validation set BEFORE the clamp: 977 of 2816
  /// detections reported fill_ratio above 1.05, up to 1.37, and the observed
  /// span deficit was 0.81 increments -- matching the 0.82 the near-duplicate
  /// beam pair predicts. Those detections were being rejected by the
  /// `fill_hi` bounds of every class, i.e. an estimator artefact was being
  /// read as "this object is impossibly solid".
  ///
  /// This is the SOLIDITY of the slice and is the single most useful feature
  /// available: a continuous surface (car body panel) returns on essentially
  /// every beam that crosses it (ratio ~1), whereas an object that is mostly
  /// empty space within its own silhouette (a bicycle: two tyres and air)
  /// returns on only a fraction (ratio well below 1). It is range-normalised
  /// by construction, because both numerator and denominator scale with how
  /// densely the object is sampled.
  double fill_ratio{0.0};

  /// Largest Euclidean distance between two angularly adjacent member points,
  /// m. Measures internal STRUCTURE: the empty span between a bicycle's front
  /// and rear wheel, or between a pedestrian's two legs. A continuous body
  /// panel has a small value (one beam spacing at that range).
  double max_gap{0.0};

  /// False when the cluster was too small for the shape statistics above to
  /// mean anything (fewer than 3 points, or a degenerate/non-finite
  /// geometry). An invalid feature set always classifies UNKNOWN.
  bool valid{false};

  // --- Stage-5B.1: ADDITIVE POSE OUTPUTS, NOT CLASSIFIER INPUTS ----------
  //
  // The principal axis and centroid are already computed inside
  // extractFeatures() in order to produce `length` and `thickness`. They are
  // reported here so that the Stage-5B.1 compound-observation stage can test
  // whether two clusters are COLLINEAR without re-implementing the same
  // eigen decomposition -- two copies of that math is a real drift hazard.
  //
  // THEY ARE NOT USED BY classify() AND NEVER WILL BE. A class label must not
  // depend on where in the map an object happens to sit, nor on its absolute
  // heading; both of those are properties of the scene, not of the object.
  // `classify()` reads length/thickness/fill_ratio/max_gap/point_count/range
  // and nothing else, exactly as before this stage was added.

  /// Unit principal (major) axis of the member points, map frame. Sign is
  /// arbitrary -- the eigenvector of a point set has no preferred direction --
  /// so every consumer must use |dot| rather than dot. Left at (0, 0) when
  /// `valid` is false.
  double axis_x{0.0};
  double axis_y{0.0};

  /// Arithmetic mean of the member points, map frame, m. Left at (0, 0) when
  /// `valid` is false. This is the same quantity the tracker uses as the
  /// cluster's measured position.
  double centroid_x{0.0};
  double centroid_y{0.0};
};

/// The motion context the classifier is allowed to see. Deliberately tiny:
/// classification must not become a second tracker.
struct TrackMotion
{
  /// Filtered track speed, m/s: hypot of the Kalman velocity once the filter
  /// is initialised, else the raw finite-difference speed.
  double speed{0.0};

  /// True once the per-track Kalman filter holds a real velocity estimate.
  /// While false, `speed` is the zero-velocity prior or a 2-sample finite
  /// difference and the speed veto is not applied.
  bool speed_valid{false};

  /// Successful associations so far, count. Used only to withhold a label
  /// from a track too young to have stable geometry.
  uint32_t observations{0};
};

/// One ASPECT of one class: the admissible box, in physical units, for ONE
/// way that class can project into a horizontal scan plane.
///
/// Each [lo, hi] pair is the interval the feature is PHYSICALLY expected to
/// occupy for that aspect in a ~0.16 m scan plane; `*_soft` is the width of
/// the linear ramp outside it. Setting a bound to +inf disables it.
struct ClassBox
{
  double length_lo{0.0};
  double length_hi{std::numeric_limits<double>::infinity()};
  double length_soft{0.10};          // m

  double thickness_lo{0.0};
  double thickness_hi{std::numeric_limits<double>::infinity()};
  double thickness_soft{0.10};       // m

  double fill_lo{0.0};
  double fill_hi{std::numeric_limits<double>::infinity()};
  double fill_soft{0.15};            // dimensionless

  double gap_lo{0.0};
  double gap_hi{std::numeric_limits<double>::infinity()};
  double gap_soft{0.08};             // m

  /// Physically implausible speed for this class, m/s. Exceeding it drives
  /// the class score to zero (the VETO described in the header comment).
  /// Applied only when TrackMotion::speed_valid.
  double speed_max{std::numeric_limits<double>::infinity()};
};

/// One class, as one or more ASPECTS.
///
/// WHY MORE THAN ONE BOX PER CLASS: a horizontal slice of a rigid body is
/// view-dependent, and for a box-shaped object the two views are genuinely
/// different measurements rather than two ends of one interval. A car
/// observed along its flank gives a LONG, flat, zero-depth return; the same
/// car observed at its corner gives a SHORTER return with real depth. No
/// single conjunction of intervals covers both without also admitting a
/// bicycle, so the class is expressed as a disjunction over aspects and
/// scored with the MAXIMUM. A class with one aspect (pedestrian, bike)
/// behaves exactly as a single box.
struct ClassModel
{
  std::vector<ClassBox> aspects;
};

/// All tunables, every one with a stated unit and physical meaning.
///
/// The geometric intervals come from the dimensions of the slice each class
/// presents at a 0.16 m scan plane, NOT from fitting to classifier output.
struct ClassificationParams
{
  // --- evidence gates ------------------------------------------------------
  /// Minimum beam returns before any semantic label is considered, count.
  /// Below ~5 points the PCA axes are dominated by the 0.01 m range noise and
  /// length/thickness are meaningless.
  uint32_t min_points{5};

  /// Maximum centroid range at which a semantic label is considered, m.
  /// Beyond this a pedestrian-sized object subtends fewer than ~min_points
  /// beams (a 0.30 m object at 6 m subtends ~0.05 rad, i.e. only ~3 beams), so geometry
  /// cannot be recovered and UNKNOWN is the correct answer.
  double max_range{6.0};

  /// Minimum successful associations before a label is emitted, count.
  /// Matches the tracker's own min_observations_to_publish so a track becomes
  /// classifiable exactly when it becomes publishable.
  uint32_t min_observations{3};

  // --- decision thresholds -------------------------------------------------
  /// Minimum winning score to accept a semantic label, dimensionless [0,1].
  /// 0.5 means the worst-fitting feature of the winning class must still sit
  /// at least halfway through its soft ramp.
  double min_accept_score{0.50};

  /// Minimum lead the winner must hold over the runner-up, dimensionless.
  /// Prevents committing when two class boxes both fit (e.g. a short vehicle
  /// slice that also satisfies BIKE).
  double min_margin{0.15};

  /// Floor on thickness when forming the elongation ratio, m. One beam
  /// spacing at 2 m (0.035 m) plus the 0.01 m range noise: below this the
  /// minor axis is noise, and dividing by it would manufacture an unbounded
  /// elongation.
  double thickness_floor{0.05};

  /// Beam angular spacing used to convert angular_span into an expected beam
  /// count, rad. 2*pi/360 for the TB3 LDS. The node overwrites this from the
  /// live LaserScan's angle_increment, so it is never assumed.
  double angle_increment{2.0 * M_PI / 360.0};

  // --- per-class aspect models -------------------------------------------
  //
  // Every bound below is an interval on a dimension of the ~0.16 m scan slice
  // and was checked against ray-cast reference geometry (one return per beam,
  // nearest hit only, 0.017453 rad spacing) for each class at 2-6 m. The
  // three decisive features are LENGTH (absolute size), MAX_GAP (internal
  // structure) and FILL_RATIO (solidity); THICKNESS is used as an upper bound
  // to keep thin objects out of the vehicle class, and as the lower bound
  // that defines the vehicle corner aspect.

  // PEDESTRIAN -- ankles/calves at 0.16 m.
  //   length    [0.04, 0.55] m : one leg is ~0.10-0.15 m across; two legs at
  //                              full stride span ~0.45 m. Measured 0.41-0.43 m
  //                              for a two-leg slice at 2.5-4 m.
  //   thickness [0, 0.40] m    : a leg is a small convex surface; it cannot
  //                              present vehicle-like depth.
  //   fill      [0.35, 1.05]   : legs-apart gives ~0.55, legs-together ~1.0.
  //                              The lower bound rejects a silhouette that is
  //                              mostly empty space (a bicycle).
  //   gap       [0, 0.30] m    : a stride gap; measured 0.26 m at 2.5 m.
  //   speed_max 2.5 m/s        : a sprint. Faster is not a pedestrian.
  ClassModel pedestrian{{ClassBox{
    /*length*/ 0.04, 0.55, 0.10,
    /*thickness*/ 0.0, 0.40, 0.10,
    /*fill*/ 0.35, 1.05, 0.15,
    /*gap*/ 0.0, 0.30, 0.08,
    /*speed_max*/ 2.5}}};

  // BIKE -- the two wheels. At 0.16 m the scan plane cuts each 0.67 m wheel
  // well below its hub, so each wheel presents a CHORD along the direction of
  // travel: 2*sqrt(r^2 - (r - 0.16)^2) = 0.57 m for r = 0.335 m, only a tyre
  // width thick. With a 1.05 m wheelbase the pair spans ~1.6 m with a ~0.5 m
  // hole between the chords where the frame passes above the plane.
  //   length    [0.85, 2.00] m : wheelbase plus chords. Measured 1.55-1.60 m
  //                              broadside at 3-4 m.
  //   thickness [0, 0.45] m    : both wheels lie in one plane; a bicycle
  //                              cannot present depth.
  //   fill      [0, 0.90]      : CANNOT be solid -- the hole guarantees it.
  //                              Measured 0.70-0.71 broadside.
  //   gap       [0.35, 1.60] m : THE defining feature. Measured 0.52-0.56 m;
  //                              a solid surface of the same length measures
  //                              0.05-0.07 m. The lower bound sits far above
  //                              the within-object spacing at these ranges.
  //   speed_max 12 m/s         : a fast descent.
  ClassModel bike{{ClassBox{
    /*length*/ 0.85, 2.00, 0.15,
    /*thickness*/ 0.0, 0.45, 0.10,
    /*fill*/ 0.0, 0.90, 0.15,
    /*gap*/ 0.35, 1.60, 0.10,
    /*speed_max*/ 12.0}}};

  // VEHICLE -- the lower body panel and tyres, as TWO aspects.
  //
  // Aspect 1, FLANK/END view: the body is a long continuous surface and the
  // slice has no measurable depth.
  //   length    [1.80, 8.00] m : above any bicycle's apparent length, so the
  //                              two classes do not overlap here. Measured
  //                              3.79 m flank and 1.70 m end-on for a
  //                              3.8 x 1.7 m car.
  //   fill      [0.55, 1.05]   : a continuous panel returns on every beam
  //                              crossing it. Measured 1.00.
  //   gap       [0, 0.45] m    : no hole in a body panel. Measured 0.04-0.07 m.
  //
  // Aspect 2, CORNER view: two faces are exposed, so the slice is SHORTER but
  // genuinely THICK. This aspect is what makes a compact vehicle separable at
  // all, and it is the only one available inside the frozen tracker's 1.2 m
  // cluster-diameter envelope.
  //   length    [0.70, 8.00] m : a corner view of a compact body. Measured
  //                              1.09 m for a 1.0 x 0.5 m footprint at 3 m.
  //   thickness [0.35, 3.00] m : the decisive term. Measured 0.42 m for that
  //                              compact corner and 1.59 m for a car corner;
  //                              a pedestrian measures <= 0.10 m and a
  //                              bicycle ~0 m, so neither can reach it.
  //   fill / gap               : as aspect 1 -- still a solid body.
  //   speed_max 40 m/s         : ~145 km/h.
  ClassModel vehicle{{
    ClassBox{
      /*length*/ 1.80, 8.00, 0.25,
      /*thickness*/ 0.0, 3.00, 0.12,
      /*fill*/ 0.55, 1.05, 0.15,
      /*gap*/ 0.0, 0.45, 0.10,
      /*speed_max*/ 40.0},
    ClassBox{
      /*length*/ 0.70, 8.00, 0.20,
      /*thickness*/ 0.35, 3.00, 0.12,
      /*fill*/ 0.55, 1.05, 0.15,
      /*gap*/ 0.0, 0.45, 0.10,
      /*speed_max*/ 40.0}}};
};

/// Extract the scan-plane geometry of one cluster.
///
/// `points` is any container whose elements expose `.x` and `.y` (the node's
/// Point2D and a test's own struct both qualify); the coordinates are the
/// cluster's member beam endpoints in the map frame. `sx, sy` is the SENSOR
/// ORIGIN for that same scan, in the same frame -- it is required because
/// every range-dependent quantity (range, angular_span, fill_ratio) is
/// defined relative to the observer, not to the world.
///
/// Method, all of it ordinary 2D statistics on the member points:
///
///   centroid       : arithmetic mean.
///   principal axis : unit eigenvector of the 2x2 second-moment (covariance)
///                    matrix of the points belonging to the larger
///                    eigenvalue, obtained in closed form. `length` and
///                    `thickness` are the full min-to-max projections onto
///                    that axis and its normal, so they are the real extents
///                    of the slice rather than standard deviations.
///   angular_span   : the points are sorted by bearing from the sensor, and
///                    the span is taken over the SHORTEST arc containing them
///                    all, which makes it correct for a cluster straddling
///                    the +/-pi bearing wrap.
///   max_gap        : the largest Euclidean step between points that are
///                    adjacent in that bearing order, i.e. the widest hole
///                    the sensor saw THROUGH the object, not merely the
///                    widest spacing in an arbitrary ordering.
///
/// `valid` is set false, and all shape fields are left at zero, for fewer
/// than 3 points or any non-finite input, because the axes are then
/// undefined.
template<typename PointContainer>
inline ClusterFeatures extractFeatures(
  const PointContainer & points, double sx, double sy, const ClassificationParams & p)
{
  ClusterFeatures f;
  const size_t n = static_cast<size_t>(std::distance(std::begin(points), std::end(points)));
  f.point_count = static_cast<uint32_t>(n);
  if (n < 3 || !std::isfinite(sx) || !std::isfinite(sy)) {
    return f;
  }

  double cx = 0.0;
  double cy = 0.0;
  for (const auto & q : points) {
    if (!std::isfinite(q.x) || !std::isfinite(q.y)) {
      return f;
    }
    cx += q.x;
    cy += q.y;
  }
  const double inv = 1.0 / static_cast<double>(n);
  cx *= inv;
  cy *= inv;

  // 2x2 second-moment matrix [[sxx, sxy], [sxy, syy]] about the centroid.
  double sxx = 0.0;
  double sxy = 0.0;
  double syy = 0.0;
  for (const auto & q : points) {
    const double dx = q.x - cx;
    const double dy = q.y - cy;
    sxx += dx * dx;
    sxy += dx * dy;
    syy += dy * dy;
  }
  sxx *= inv;
  sxy *= inv;
  syy *= inv;

  // Closed-form principal eigenvector of a symmetric 2x2 matrix. The larger
  // eigenvalue's eigenvector is the direction of maximum spread.
  const double tr = sxx + syy;
  const double diff = sxx - syy;
  const double disc = std::sqrt(std::max(0.0, diff * diff + 4.0 * sxy * sxy));
  const double lambda1 = 0.5 * (tr + disc);
  double ax = 0.0;
  double ay = 0.0;
  if (std::abs(sxy) > 1e-12) {
    ax = lambda1 - syy;
    ay = sxy;
  } else {
    // Already axis-aligned: pick whichever axis carries more spread.
    ax = (sxx >= syy) ? 1.0 : 0.0;
    ay = (sxx >= syy) ? 0.0 : 1.0;
  }
  const double an = std::hypot(ax, ay);
  if (!(an > 0.0) || !std::isfinite(an)) {
    return f;
  }
  ax /= an;
  ay /= an;

  // Full extents along the principal axis and its normal.
  double umin = std::numeric_limits<double>::infinity();
  double umax = -std::numeric_limits<double>::infinity();
  double vmin = std::numeric_limits<double>::infinity();
  double vmax = -std::numeric_limits<double>::infinity();
  for (const auto & q : points) {
    const double dx = q.x - cx;
    const double dy = q.y - cy;
    const double u = dx * ax + dy * ay;
    const double v = -dx * ay + dy * ax;
    umin = std::min(umin, u);
    umax = std::max(umax, u);
    vmin = std::min(vmin, v);
    vmax = std::max(vmax, v);
  }
  f.length = umax - umin;
  f.thickness = vmax - vmin;
  f.elongation = f.length / std::max(f.thickness, p.thickness_floor);

  // Stage-5B.1: report the axis and centroid this function already derived.
  // Purely additive -- see ClusterFeatures for why classify() cannot read
  // them.
  f.axis_x = ax;
  f.axis_y = ay;
  f.centroid_x = cx;
  f.centroid_y = cy;

  f.range = std::hypot(cx - sx, cy - sy);

  // Bearings from the sensor, sorted, so the angular span and the internal
  // gaps are both measured in the order the beams actually swept.
  std::vector<std::pair<double, size_t>> bearings;
  bearings.reserve(n);
  {
    size_t i = 0;
    for (const auto & q : points) {
      bearings.emplace_back(std::atan2(q.y - sy, q.x - sx), i++);
    }
  }
  std::sort(bearings.begin(), bearings.end());

  // Shortest arc containing every point: the complement of the largest
  // angular gap between bearing-adjacent points (wrapping included).
  double largest_angular_gap = 0.0;
  size_t gap_at = 0;
  for (size_t i = 0; i < n; ++i) {
    const size_t j = (i + 1) % n;
    double d = bearings[j].first - bearings[i].first;
    if (j == 0) {
      d += 2.0 * M_PI;               // the wrap-around step
    }
    if (d > largest_angular_gap) {
      largest_angular_gap = d;
      gap_at = i;
    }
  }
  f.angular_span = std::max(0.0, 2.0 * M_PI - largest_angular_gap);

  const double inc = (p.angle_increment > 0.0) ? p.angle_increment : (2.0 * M_PI / 360.0);
  const double expected = f.angular_span / inc + 1.0;
  // Clamped at 1: a fraction of available beams cannot exceed all of them.
  // See ClusterFeatures::fill_ratio for why the raw quotient can.
  f.fill_ratio = (expected > 0.0) ?
    std::min(1.0, static_cast<double>(n) / expected) : 0.0;

  // Largest Euclidean step between bearing-adjacent points, taken around the
  // contiguous arc only: the step ACROSS the excluded gap (from the last
  // point of the arc back to the first) is the empty space behind the
  // object, not a hole in it, so it is skipped.
  {
    // Index the points once so bearing order can address them.
    std::vector<std::pair<double, double>> xy;
    xy.reserve(n);
    for (const auto & q : points) {
      xy.emplace_back(q.x, q.y);
    }
    double gmax = 0.0;
    for (size_t k = 0; k + 1 < n; ++k) {
      const size_t i = (gap_at + 1 + k) % n;
      const size_t j = (gap_at + 2 + k) % n;
      const auto & a = xy[bearings[i].second];
      const auto & b = xy[bearings[j].second];
      gmax = std::max(gmax, std::hypot(a.first - b.first, a.second - b.second));
    }
    f.max_gap = gmax;
  }

  f.valid = std::isfinite(f.length) && std::isfinite(f.thickness) &&
    std::isfinite(f.range) && std::isfinite(f.fill_ratio) && std::isfinite(f.max_gap);
  return f;
}

/// One classification decision plus the evidence behind it.
struct ClassificationResult
{
  ObstacleClass label{ObstacleClass::UNKNOWN};

  /// [0, 1]. 0.0 whenever `label` is UNKNOWN for lack of evidence.
  double confidence{0.0};

  /// Raw per-class conjunction scores, indexed by ObstacleClass value.
  /// Published nowhere; kept so a test or a log can explain a decision.
  std::array<double, 4> scores{{0.0, 0.0, 0.0, 0.0}};

  /// Why UNKNOWN was returned, for diagnostics. Empty when a label was
  /// accepted.
  const char * reject_reason{""};
};

/// Trapezoidal membership of `v` in [lo, hi] with a linear ramp of width
/// `soft` on each side. Returns 1.0 inside the interval, 0.0 beyond the ramp.
///
/// A non-positive `soft` degenerates to a hard indicator function.
inline double trapezoid(double v, double lo, double hi, double soft)
{
  if (!std::isfinite(v)) {
    return 0.0;
  }
  if (v >= lo && v <= hi) {
    return 1.0;
  }
  if (soft <= 0.0) {
    return 0.0;
  }
  const double d = (v < lo) ? (lo - v) : (v - hi);
  if (!std::isfinite(d)) {
    return 0.0;
  }
  return std::clamp(1.0 - d / soft, 0.0, 1.0);
}

/// Conjunction score of one class box against one feature set, in [0, 1].
/// The MINIMUM over features: every feature must be satisfied for the class
/// to score well, and any single disqualifying feature rules it out.
inline double boxScore(
  const ClusterFeatures & f, const TrackMotion & m, const ClassBox & b)
{
  if (m.speed_valid && m.speed > b.speed_max) {
    return 0.0;      // kinematic veto
  }
  double s = 1.0;
  s = std::min(s, trapezoid(f.length, b.length_lo, b.length_hi, b.length_soft));
  s = std::min(s, trapezoid(f.thickness, b.thickness_lo, b.thickness_hi, b.thickness_soft));
  s = std::min(s, trapezoid(f.fill_ratio, b.fill_lo, b.fill_hi, b.fill_soft));
  s = std::min(s, trapezoid(f.max_gap, b.gap_lo, b.gap_hi, b.gap_soft));
  return s;
}

/// Score one class against one feature set: the MAXIMUM over the class's
/// aspects, since the aspects are alternative views of the same object.
/// A class with no aspects scores 0.
inline double modelScore(
  const ClusterFeatures & f, const TrackMotion & m, const ClassModel & model)
{
  double best = 0.0;
  for (const auto & b : model.aspects) {
    best = std::max(best, boxScore(f, m, b));
  }
  return best;
}

/// Classify ONE scan's cluster geometry for ONE track. Stateless and
/// deterministic: identical inputs always give an identical result.
inline ClassificationResult classifyInstant(
  const ClusterFeatures & f,
  const TrackMotion & m,
  const ClassificationParams & p)
{
  ClassificationResult r;

  if (!f.valid) {
    r.reject_reason = "invalid_features";
    return r;
  }
  if (f.point_count < p.min_points) {
    r.reject_reason = "too_few_points";
    return r;
  }
  if (!(f.range <= p.max_range)) {
    r.reject_reason = "out_of_range";
    return r;
  }
  if (m.observations < p.min_observations) {
    r.reject_reason = "track_too_young";
    return r;
  }

  r.scores[static_cast<size_t>(ObstacleClass::PEDESTRIAN)] = modelScore(f, m, p.pedestrian);
  r.scores[static_cast<size_t>(ObstacleClass::BIKE)] = modelScore(f, m, p.bike);
  r.scores[static_cast<size_t>(ObstacleClass::VEHICLE)] = modelScore(f, m, p.vehicle);

  // Winner and runner-up over the semantic classes only.
  size_t best_i = 1;
  double best = r.scores[1];
  for (size_t i = 2; i <= kNumSemanticClasses; ++i) {
    if (r.scores[i] > best) {
      best = r.scores[i];
      best_i = i;
    }
  }
  double second = 0.0;
  for (size_t i = 1; i <= kNumSemanticClasses; ++i) {
    if (i != best_i) {
      second = std::max(second, r.scores[i]);
    }
  }

  if (best < p.min_accept_score) {
    r.reject_reason = "below_accept_score";
    return r;
  }
  if (best - second < p.min_margin) {
    r.reject_reason = "ambiguous";
    return r;
  }

  r.label = static_cast<ObstacleClass>(best_i);
  r.confidence = std::clamp(best * (1.0 - second / best), 0.0, 1.0);
  return r;
}

/// Temporal label stabiliser for ONE track.
///
/// WHY: a single scan of a partially occluded or obliquely viewed object can
/// score for the wrong class, and an unfiltered label would flicker
/// frame-to-frame. This accumulates evidence on the track that already
/// exists -- it holds no position, no velocity and no association state, and
/// is therefore not a second tracker.
///
/// HOW: one exponentially-weighted score per semantic class,
///
///   S_c <- (1 - alpha) * S_c + alpha * (c == instant_label ? confidence : 0)
///
/// with alpha = `smoothing`. A scan that returns UNKNOWN for lack of evidence
/// contributes no positive evidence to any class but still DECAYS all of
/// them, so a label fades out when the object stops being recognisable
/// instead of persisting forever.
///
/// WHEN THE PUBLISHED LABEL CHANGES (hysteresis):
///   * It is set from UNKNOWN to class c as soon as S_c >= `enter_threshold`.
///   * Once c is held, it is REPLACED by a rival d only when
///         S_d >= enter_threshold  AND  S_d >= S_c + `switch_margin`.
///   * It reverts to UNKNOWN when S_c < `exit_threshold`.
/// Because enter_threshold > exit_threshold and a switch additionally
/// requires a clear lead, a label cannot oscillate between two classes on
/// alternating scans.
class ClassLabelStabilizer
{
public:
  /// Exponential smoothing factor alpha, dimensionless (0, 1]. 0.4 gives an
  /// effective window of ~1/alpha = 2.5 scans, i.e. 0.5 s at the LDS's 5 Hz:
  /// long enough to reject a single bad scan, short enough to label a track
  /// within its first second.
  double smoothing{0.40};

  /// Smoothed score needed to ADOPT a class, dimensionless [0, 1].
  double enter_threshold{0.45};

  /// Smoothed score below which the held class is DROPPED, dimensionless.
  /// Strictly below enter_threshold: the difference is the hysteresis band.
  double exit_threshold{0.25};

  /// Extra lead a rival must hold over the incumbent to take the label over,
  /// dimensionless.
  ///
  /// DERIVED, not tuned. Under perfectly alternating evidence between two
  /// classes at confidence c, the two smoothed scores settle into
  /// anti-phase steady states
  ///   hi = c / (2 - alpha),   lo = (1 - alpha) c / (2 - alpha)
  /// so the rival leads the incumbent by at most
  ///   hi - lo = c * alpha / (2 - alpha)
  /// which for alpha = 0.4 and the worst case c = 1 is exactly 0.25.
  /// Requiring a lead STRICTLY GREATER than that bound makes alternating
  /// evidence provably incapable of flipping the label, at any confidence.
  /// 0.30 is that bound plus margin. Sustained (non-alternating) contrary
  /// evidence is unaffected: the incumbent's score decays towards 0 while the
  /// rival's rises towards c, so the lead grows past 0.30 and the handover
  /// happens.
  double switch_margin{0.30};

  /// Fold one scan's instantaneous decision in and return the stable label.
  ObstacleClass update(const ClassificationResult & instant)
  {
    const double a = std::clamp(smoothing, 0.0, 1.0);
    const size_t inst = static_cast<size_t>(instant.label);
    for (size_t c = 1; c <= kNumSemanticClasses; ++c) {
      const double evidence =
        (c == inst && instant.label != ObstacleClass::UNKNOWN) ? instant.confidence : 0.0;
      scores_[c] = (1.0 - a) * scores_[c] + a * evidence;
    }

    size_t best_i = 1;
    for (size_t c = 2; c <= kNumSemanticClasses; ++c) {
      if (scores_[c] > scores_[best_i]) {
        best_i = c;
      }
    }
    const double best = scores_[best_i];
    const size_t held = static_cast<size_t>(label_);

    if (label_ == ObstacleClass::UNKNOWN) {
      if (best >= enter_threshold) {
        label_ = static_cast<ObstacleClass>(best_i);
      }
    } else if (scores_[held] < exit_threshold) {
      // The incumbent's own evidence has decayed away. Hand over only to a
      // rival that independently qualifies; otherwise fall back to UNKNOWN.
      if (best_i != held && best >= enter_threshold) {
        label_ = static_cast<ObstacleClass>(best_i);
      } else {
        label_ = ObstacleClass::UNKNOWN;
      }
    } else if (
      best_i != held && best >= enter_threshold &&
      best >= scores_[held] + switch_margin)
    {
      label_ = static_cast<ObstacleClass>(best_i);
    }

    return label_;
  }

  ObstacleClass label() const {return label_;}

  /// Smoothed confidence of the currently held label, [0, 1]. Zero for
  /// UNKNOWN.
  double confidence() const
  {
    return (label_ == ObstacleClass::UNKNOWN) ?
           0.0 : std::clamp(scores_[static_cast<size_t>(label_)], 0.0, 1.0);
  }

  double score(ObstacleClass c) const {return scores_[static_cast<size_t>(c)];}

private:
  std::array<double, 4> scores_{{0.0, 0.0, 0.0, 0.0}};
  ObstacleClass label_{ObstacleClass::UNKNOWN};
};

}  // namespace predictive_nav_tracking

#endif  // PREDICTIVE_NAV_TRACKING__OBSTACLE_CLASSIFICATION_HPP_
