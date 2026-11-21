// Stage-5D: class-aware MOTION CAPABILITY PROFILES for bounded-motion
// reachable sets.
//
// WHAT THIS IS
// ------------
// A per-semantic-class table of the two premises the Stage-4G4 reachability
// bound rests on -- `max_speed` (P2) and `max_acceleration` (P1), see
// reachability.hpp -- plus a deterministic body margin, together with the
// policy that decides WHICH entry of that table a given track is entitled to.
// It replaces the one global `ReachabilityBounds{max_acceleration, max_speed,
// safety_margin}` with a selection, and nothing else: the reachability
// mathematics, the Stage-5C.1 validity contract and the published region are
// byte-for-byte the same functions of whichever profile is selected.
//
// WHAT THIS IS NOT
// ----------------
// It is not a classifier, not a probability, and not a confidence model. It
// consumes the already-stabilised Stage-5B label and its smoothed confidence
// and makes one decision from them. It also does not make the prediction more
// trustworthy: a profile is still a DECLARED CAPABILITY PREMISE about an
// object's unobservable future, exactly as the single global `max_speed` was.
// Attaching a class name to the premise changes who the premise is about, not
// whether it has been proved.
//
// =====================================================================
// 1. WHY A PROFILE CANNOT BE SELECTED FROM THE LABEL ALONE
// =====================================================================
//
// The Stage-5B classifier is imperfect in a way that is MEASURED, not
// assumed. From the retained Stage-5B recordings (2,547 rows, 57 tracks;
// validation/stage5b/raw/, reproduced in
// validation/stage5d/CLASS_REACHABILITY_BOUNDARY_AUDIT.md section 1.8), the
// composition of each PREDICTED label is
//
//   predicted PEDESTRIAN  n=715 : PEDESTRIAN 381, VEHICLE 306, BIKE 28
//   predicted BIKE        n=277 : BIKE 277                      <-- pure
//   predicted VEHICLE     n=418 : VEHICLE 301, UNKNOWN 117
//   predicted UNKNOWN     n=1137: UNKNOWN 550, VEHICLE 382, PEDESTRIAN 197,
//                                 BIKE 8
//
// and the confidence attached to the dangerous direction is
//
//   PEDESTRIAN -> PEDESTRIAN : min 0.253, median 0.856
//   VEHICLE    -> PEDESTRIAN : min 0.640, median 1.000   <-- MORE confident
//
// A vehicle mislabelled PEDESTRIAN is therefore reported MORE confidently
// than a correctly labelled pedestrian, and the misclassification's MINIMUM
// confidence sits above the correct classification's MEDIAN. No threshold on
// `classification_confidence` can admit true pedestrians while excluding
// those vehicles. The mechanism is physical and understood: a 1.0 x 0.5 m
// compact body viewed end-on in ONE horizontal plane returns a 0.42-0.45 m
// long, 0.02-0.03 m thick, fully-filled slice, which is a pedestrian-sized
// return. It is the same family of 2D-LiDAR observability limit already
// documented for the six head-on bicycle scenarios.
//
// So a confidence gate is NECESSARY (an immature or weakly-held label must
// not buy a weak profile) but NOT SUFFICIENT. The sufficient condition has to
// come from the label's measured PRECISION, which is what the ambiguity sets
// below encode.
//
// =====================================================================
// 2. THE SAFETY ORDER, AND WHY IT IS A THEOREM
// =====================================================================
//
// Define the componentwise order on profiles
//
//   A >= B  <=>  A.max_speed        >= B.max_speed
//           AND  A.max_acceleration >= B.max_acceleration
//           AND  A.margin           >= B.margin
//
// CLAIM. A >= B implies region(A, t) contains region(B, t) for every t, every
// track state and every observed speed.
//
// PROOF. reachability.hpp derives
//
//   r_det(t) = integral_0^t min(a_max * s, v_eff + s0) ds,
//
// with v_eff = max(v_max, s0) >= v_max (Stage-5C.1). The integrand
// min(a_max*s, v_eff+s0) is pointwise nondecreasing in a_max and in v_max
// (because max(., s0) is nondecreasing and min(., .) is nondecreasing in each
// argument), so the integral is nondecreasing in both. The published
// semi-axes are sigma_term + r_det + margin with a sigma_term that does not
// depend on the profile at all, so both semi-axes are nondecreasing in each
// of the three components, and the ellipses are concentric and co-oriented.
// Hence containment. QED.
//
// `envelope(A, B)` below is the componentwise maximum, i.e. the least upper
// bound of this order. The claim is what makes every safety argument in this
// file a one-line consequence: to show a selection is at least as
// conservative as another, it is enough to show it dominates componentwise,
// and `envelope` dominates both its arguments by construction. It is asserted
// numerically over a wide grid by `stage5d_profiles` and `stage5d_audit`
// rather than only argued here.
//
// =====================================================================
// 3. AMBIGUITY SETS -- THE PROFILE A LABEL IS WORTH
// =====================================================================
//
// `CapabilityProfileSet` answers "what can a PEDESTRIAN do"; it is physics
// and it is the same whatever classifier produced the word.
// `AmbiguitySets` answers the different question "what might an object this
// classifier calls PEDESTRIAN actually BE", which is a property of the
// classifier and its sensor, and the two must not be conflated in one number.
//
// For a predicted label c, the applied profile is the envelope over c's
// ambiguity set A(c) -- every class a c-labelled object might really be. The
// label itself is always a member (enforced in code), so
//
//   selected(c) >= profile(c)                                          (I1)
//
// and the bound stays valid for every class the object might really be,
// which is the property that survives a misclassification instead of being
// invalidated by it.
//
// The defaults come from the measured composition in section 1 above:
//
//   A(PEDESTRIAN) = {PEDESTRIAN, BIKE, VEHICLE}   306/715 were vehicles
//   A(BIKE)       = {BIKE}                        277/277 were bicycles:
//                                                 precision 1.000 per
//                                                 detection AND per track,
//                                                 0 false BIKE predictions
//   A(VEHICLE)    = {VEHICLE, UNKNOWN}            117/418 were the planter
//   A(UNKNOWN)    = all four                      382/1137 were vehicles
//
// They are configuration, not constants, because they describe a measured
// classifier at a measured operating point: a better classifier earns
// narrower sets, and must then re-measure them to claim them.
//
// CONSEQUENCE, STATED PLAINLY. Under these defaults A(PEDESTRIAN) contains
// VEHICLE, so the PEDESTRIAN envelope EQUALS the vehicle-grade envelope and
// a PEDESTRIAN label buys no reduction at all. BIKE is the only label whose
// measured precision earns a profile weaker than the fallback. That is a
// result about this classifier, and it is reported rather than engineered
// around -- the alternative, trusting the label, is precisely the 306-sample
// VEHICLE -> PEDESTRIAN confusion above.
//
// =====================================================================
// 4. UNKNOWN IS THE FALLBACK, AND IT IS THE TOP OF THE ORDER
// =====================================================================
//
// UNKNOWN is not a fourth kind of object. The Stage-5B recordings show the
// label is overloaded three ways -- not yet decided (the dominant case: the
// median track is UNKNOWN for its first 16 observations), cannot be decided
// (too few points, beyond 6 m, ambiguous), and decided to be none of the
// three -- and that 382 of 1,137 UNKNOWN detections were ground-truth
// VEHICLE. An UNKNOWN object in this very dataset demonstrably IS a vehicle a
// third of the time.
//
// So the UNKNOWN profile is not chosen; it is DERIVED as the envelope over
// A(UNKNOWN) = every supported class:
//
//   fallback = envelope over { profile(c) : c in A(UNKNOWN) } >= profile(c)
//                                                    for every class c  (I2)
//
// This is what section 4 of the Stage-5D brief requires, as an identity
// rather than as a chosen pair of numbers, and it is why the old global
// 0.8 m/s is NOT reused for UNKNOWN: 0.8 m/s under-bounds a bicycle by a
// factor of 15 and a car by 50.
//
// =====================================================================
// 5. THE UNCERTAINTY INVARIANT
// =====================================================================
//
// Whenever the selection is UNCERTAIN -- UNKNOWN label, confidence below
// `min_confidence`, or a track younger than `min_observations` -- the
// fallback is substituted outright. With (I2) that gives
//
//   uncertain  =>  selected >= profile(c) for EVERY class c             (I3)
//
// so uncertainty can never produce a smaller reachable set than certainty
// about any class, which is the invariant section 5 of the brief demands.
// Certainty about a class may legitimately produce a smaller set -- that is
// the entire point of class-awareness -- and is gated on (I1) plus the
// label's measured precision.
//
// =====================================================================
// 6. TRANSITIONS
// =====================================================================
//
// Label changes are frequent, not hypothetical: 115 stable-label transitions
// over 36 of 57 Stage-5B tracks (63%), including PEDESTRIAN -> BIKE, which
// under the defaults above is the one transition direction that SHRINKS the
// applied profile.
//
// The rule is the minimum that makes a shrink impossible without warning:
// on any change of the stable label, and for `transition_hold_scans`
// subsequent scans at the new label, publish the ENVELOPE of what was held
// and what is now selected. Formally, with H_k the held profile after scan k
// and C_k the scan's selection,
//
//   H_k = C_k                        if the track is new
//       = envelope(H_{k-1}, C_k)     if the label changed, or a hold is live
//       = C_k                        otherwise
//
// Three properties follow directly from `envelope` being the least upper
// bound:
//
//   (T1) H_k >= C_k always: the published profile is never weaker than the
//        current selection;
//   (T2) on the transition scan and for the whole hold window,
//        H_k >= H_{k-1}: occupancy cannot shrink at all;
//   (T3) H_k <= fallback always, by (I2) and induction, so the envelope
//        cannot grow without bound however often the label flaps.
//
// A shrink can therefore only occur on the first scan after a full
// `transition_hold_scans` run of scans at ONE stable label, on top of the
// Stage-5B hysteresis (enter 0.45 / exit 0.25 / switch margin 0.30) that the
// label already had to survive. Nothing more elaborate is added, because
// nothing more is needed to make the shrink bounded, delayed and visible in
// `capability_profile_reason`.
//
// =====================================================================
// 7. MALFORMED CONFIGURATION
// =====================================================================
//
// A profile table is operator-supplied, so it can be wrong. The rule is that
// a malformed entry must never be able to produce an OPTIMISTIC prediction:
// `sanitizeProfileSet()` replaces any non-finite or negative entry with the
// envelope of the entries that are well-formed, and if none is, with the
// built-in conservative default. It reports what it did so the node can log
// an error and publish `PROFILE_FALLBACK_INVALID_CONFIG` rather than
// silently substituting. This runs once when parameters are read, not per
// scan.
#ifndef PREDICTIVE_NAV_TRACKING__CAPABILITY_PROFILES_HPP_
#define PREDICTIVE_NAV_TRACKING__CAPABILITY_PROFILES_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#include "predictive_nav_tracking/obstacle_classification.hpp"
#include "predictive_nav_tracking/reachability.hpp"

namespace predictive_nav_tracking
{

/// Number of entries in a class-indexed table: UNKNOWN plus the semantic
/// classes, indexed by the ObstacleClass value itself.
constexpr size_t kNumProfileClasses = kNumSemanticClasses + 1;

/// One class's declared motion capability. Every field is a DECLARED bound on
/// what an object of that class can do, chosen from the object's assumed
/// capability and never from a desired navigation outcome. "Declared" is
/// meant literally: see `CapabilityProfileSet` for which of the fields rests
/// on independent evidence (max_acceleration) and which is a configured
/// conservative prior (max_speed).
struct MotionCapabilityProfile
{
  /// m/s. Premise (P2) of reachability.hpp for this class: |v(s)| <=
  /// max_speed over the whole prediction window. A FUTURE CAPABILITY claim,
  /// not an observed velocity -- Stage-5C.1 applies unchanged, including the
  /// observable-contradiction response.
  double max_speed{0.0};

  /// m/s^2. Premise (P1): isotropic |a(u)| <= max_acceleration over the whole
  /// window. Isotropic means it bounds braking, side-stepping and turning
  /// alike, so the governing figure is the largest magnitude the class can
  /// produce in ANY direction -- for wheeled classes that is braking, not
  /// propulsion.
  double max_acceleration{0.0};

  /// m. Deterministic additive body allowance, mapped onto
  /// ReachabilityBounds::safety_margin.
  double margin{0.0};
};

/// True when every field is finite and non-negative. A profile that fails
/// this is not merely unusual, it is unusable: NaN propagates through
/// reachRadius() into the published semi-axes and a negative bound is not a
/// bound at all.
inline bool profileWellFormed(const MotionCapabilityProfile & p)
{
  return std::isfinite(p.max_speed) && p.max_speed >= 0.0 &&
         std::isfinite(p.max_acceleration) && p.max_acceleration >= 0.0 &&
         std::isfinite(p.margin) && p.margin >= 0.0;
}

/// Componentwise order of section 2: true when `a` is at least as
/// conservative as `b` in every component, hence region(a) contains
/// region(b) at every horizon.
inline bool profileDominates(
  const MotionCapabilityProfile & a, const MotionCapabilityProfile & b)
{
  return a.max_speed >= b.max_speed &&
         a.max_acceleration >= b.max_acceleration &&
         a.margin >= b.margin;
}

/// Exact componentwise equality. Used only to name the applied envelope after
/// the fact (`dominatingClass`), never to make a safety decision.
inline bool profileEqual(
  const MotionCapabilityProfile & a, const MotionCapabilityProfile & b)
{
  return a.max_speed == b.max_speed &&
         a.max_acceleration == b.max_acceleration &&
         a.margin == b.margin;
}

/// Least upper bound of the order: the componentwise maximum. Dominates both
/// arguments by construction, which is the whole of the transition and
/// fallback safety argument.
inline MotionCapabilityProfile profileEnvelope(
  const MotionCapabilityProfile & a, const MotionCapabilityProfile & b)
{
  MotionCapabilityProfile out;
  out.max_speed = std::max(a.max_speed, b.max_speed);
  out.max_acceleration = std::max(a.max_acceleration, b.max_acceleration);
  out.margin = std::max(a.margin, b.margin);
  return out;
}

/// The class-indexed capability table.
///
/// DEFAULTS ARE CONSERVATIVE CONFIGURED CAPABILITY PRIORS, NOT THE SIMULATION
/// ENVELOPE, and that is deliberate. Three kinds of number must not be
/// conflated, and only the third is independent evidence:
///
///   (a) SIMULATION-QUALIFIED OPERATING ENVELOPE -- what the Gazebo actors
///       were commanded and observed to do. Reported separately in section
///       3.A of validation/stage5d/STAGE5D_README.md; never a default here.
///   (b) CONFIGURED CONSERVATIVE CAPABILITY PRIOR -- a premise chosen to be
///       hard to exceed, asserted rather than proved. `max_speed` is this.
///   (c) INDEPENDENTLY SOURCED PHYSICAL EVIDENCE -- an external measurement
///       this repository did not produce. `max_acceleration` is this.
///
/// `max_speed` and `max_acceleration` are premises
/// about an object's future capability (reachability.hpp), so they must cover
/// what the class CAN do, not what the Gazebo actors happened to be commanded
/// to do. Every Stage-5B actor was commanded at 0.25/0.50/0.75 m/s regardless
/// of class and the filtered speed never exceeded 0.967 m/s, so a
/// simulation-fitted table would be CLASS-INDEPENDENT -- it would carry no
/// semantic information at all -- and would under-bound a real bicycle by a
/// factor of 15. Section 4 of the Stage-5D brief forbids exactly that.
/// validation/stage5d/STAGE5D_README.md tabulates the simulation-qualified
/// envelope separately and states which of the two each result belongs to.
///
/// EVIDENCE PER FIELD (full citations in the README):
///
/// max_speed: a CONSERVATIVE CONFIGURED CAPABILITY PRIOR ALIGNED WITH THE
///   CLASSIFIER'S PER-CLASS SPEED GATES -- taken from the SHIPPED Stage-5B
///   per-class speed vetoes,
///   ClassificationParams::{pedestrian,bike,vehicle}.speed_max = 2.5 / 12.0 /
///   40.0 m/s. THESE ARE NOT MEASURED VALUES AND NOT LITERATURE-PROVEN
///   PHYSICAL MAXIMA. Nothing in or out of this repository establishes them as
///   the fastest a member of each class can travel, and Stage-5D does not
///   claim it; they are retained because they are conservative here and
///   because alignment with the gate is self-consistency, not a new constant:
///   the classifier
///   already REFUSES a PEDESTRIAN label to anything observed above 2.5 m/s,
///   so a held PEDESTRIAN label is already an assertion that |v| <= 2.5, and
///   premise (P2) with max_speed = 2.5 asserts the same thing over the
///   prediction window. The two thresholds coincide by construction, which
///   also means the classifier's veto and
///   `capabilityPremiseContradicted()` fire at the same speed.
///
/// max_acceleration: INDEPENDENTLY SOURCED PHYSICAL EVIDENCE -- measured
///   external values, cited below, because the repository holds no per-class
///   acceleration evidence of its own. This is the one column of the table
///   with an evidence basis separate from the classifier's configuration.
///   PEDESTRIAN 2.5 m/s^2 -- measured crossing accelerations average
///     0.68 m/s^2 with a maximum of 1.44 m/s^2 (arXiv:1610.00029), so this is
///     1.7x the measured maximum and about 0.25 g of foot traction. DECLARED
///     RESIDUAL: an athletic standing-start or side-step is traction-limited
///     nearer 0.5 g and can exceed it transiently; that is an out-of-profile
///     case, measured by the Stage-5D out-of-domain probe and reported, in
///     the same way Stage-4G4 reports the 5-9.5 m/s^2 Gazebo step-velocity
///     residual rather than hiding it.
///   BIKE 7.0 m/s^2 -- bicycle braking is pitch-over limited at a calculated
///     5.4-7.0 m/s^2, and measured emergency front-or-both-brake
///     decelerations are 0.40-0.71 g, i.e. 3.9-7.0 m/s^2
///     (SAE 2020-01-0876). The top of the measured range is taken because the
///     bound is isotropic and must cover braking, which dominates a
///     bicycle's propulsion by several times.
///   VEHICLE 10.0 m/s^2 -- measured ABS decelerations for 20 late-model
///     vehicles on dry asphalt and concrete average 0.871-1.081 g, i.e.
///     8.5-10.6 m/s^2 (SAE 2023-01-0616). 10.0 sits inside that measured
///     band near its top.
///
/// margin: 0.0 for every class, inherited deliberately from Stage-4G4's
///   `base_safety_margin: 0.0` so that the published region measures the
///   motion model alone and the global-vs-class-aware comparison is not
///   confounded by a free additive term. The field exists, is per-class and is
///   configurable; the README tabulates each Stage-5B actor's scan-plane body
///   half-extent as the value a deployment should set.
///
/// UNKNOWN IS NOT STORED. It is derived as the envelope over its ambiguity
/// set, see `fallbackProfile()`, so it cannot drift out of step with the
/// classes it has to dominate.
struct CapabilityProfileSet
{
  std::array<MotionCapabilityProfile, kNumProfileClasses> by_class{{
    /* UNKNOWN    */ MotionCapabilityProfile{0.0, 0.0, 0.0},
    /* PEDESTRIAN */ MotionCapabilityProfile{2.5, 2.5, 0.0},
    /* BIKE       */ MotionCapabilityProfile{12.0, 7.0, 0.0},
    /* VEHICLE    */ MotionCapabilityProfile{40.0, 10.0, 0.0},
  }};

  MotionCapabilityProfile & operator[](ObstacleClass c)
  {
    return by_class[static_cast<size_t>(c)];
  }
  const MotionCapabilityProfile & operator[](ObstacleClass c) const
  {
    return by_class[static_cast<size_t>(c)];
  }
};

/// Bit for one class in an ambiguity mask.
inline uint8_t classBit(ObstacleClass c)
{
  return static_cast<uint8_t>(1u << static_cast<unsigned>(c));
}

/// For each predicted label, the set of classes an object carrying that label
/// might really be -- a property of the CLASSIFIER, measured, not of physics.
/// See section 3 for the measured composition each default comes from.
///
/// The label is forced into its own set by `closureMask()`, so a
/// configuration that omits it cannot weaken invariant (I1).
struct AmbiguitySets
{
  std::array<uint8_t, kNumProfileClasses> mask{{
    /* UNKNOWN    */ static_cast<uint8_t>(
      classBit(ObstacleClass::UNKNOWN) | classBit(ObstacleClass::PEDESTRIAN) |
      classBit(ObstacleClass::BIKE) | classBit(ObstacleClass::VEHICLE)),
    /* PEDESTRIAN */ static_cast<uint8_t>(
      classBit(ObstacleClass::PEDESTRIAN) | classBit(ObstacleClass::BIKE) |
      classBit(ObstacleClass::VEHICLE)),
    /* BIKE       */ static_cast<uint8_t>(classBit(ObstacleClass::BIKE)),
    /* VEHICLE    */ static_cast<uint8_t>(
      classBit(ObstacleClass::VEHICLE) | classBit(ObstacleClass::UNKNOWN)),
  }};

  uint8_t operator[](ObstacleClass c) const {return mask[static_cast<size_t>(c)];}
};

/// `label`'s ambiguity set with `label` itself forced in.
inline uint8_t closureMask(ObstacleClass label, const AmbiguitySets & amb)
{
  return static_cast<uint8_t>(amb[label] | classBit(label));
}

/// Envelope of every profile named by `mask`. An empty mask yields the zero
/// profile, which is the only honest answer for "no class at all" and which
/// `selectCapabilityProfile` never produces because the label is always in
/// its own closure.
inline MotionCapabilityProfile maskEnvelope(
  uint8_t mask, const CapabilityProfileSet & set)
{
  MotionCapabilityProfile out{0.0, 0.0, 0.0};
  for (size_t c = 0; c < kNumProfileClasses; ++c) {
    if ((mask & classBit(static_cast<ObstacleClass>(c))) != 0) {
      out = profileEnvelope(out, set.by_class[c]);
    }
  }
  return out;
}

/// The conservative fallback: the envelope over UNKNOWN's ambiguity set,
/// which by default is every class. Derived rather than configured, so (I2)
/// -- fallback dominates every class profile -- holds by identity whenever
/// A(UNKNOWN) names every class.
inline MotionCapabilityProfile fallbackProfile(
  const CapabilityProfileSet & set, const AmbiguitySets & amb)
{
  return maskEnvelope(closureMask(ObstacleClass::UNKNOWN, amb), set);
}

/// Replace every malformed entry with the envelope of the well-formed ones,
/// or with `CapabilityProfileSet{}`'s built-in conservative default if none
/// is well-formed. Returns true when nothing had to be changed.
///
/// Conservative by construction: a substituted entry dominates every entry
/// the operator supplied correctly, so a typo cannot buy a smaller region.
/// Call once when parameters are read; it is not a per-scan guard.
inline bool sanitizeProfileSet(CapabilityProfileSet & set)
{
  MotionCapabilityProfile good{0.0, 0.0, 0.0};
  bool any_good = false;
  bool all_good = true;
  for (size_t c = 0; c < kNumProfileClasses; ++c) {
    // UNKNOWN's stored entry is unused (the fallback is derived), but a
    // non-finite value there would still reach maskEnvelope, so it is
    // sanitized like any other.
    if (profileWellFormed(set.by_class[c])) {
      good = profileEnvelope(good, set.by_class[c]);
      any_good = true;
    } else {
      all_good = false;
    }
  }
  if (all_good) {
    return true;
  }
  const CapabilityProfileSet builtin;
  const MotionCapabilityProfile replacement = any_good ?
    good : maskEnvelope(
    static_cast<uint8_t>(
      classBit(ObstacleClass::PEDESTRIAN) | classBit(ObstacleClass::BIKE) |
      classBit(ObstacleClass::VEHICLE)), builtin);
  for (size_t c = 0; c < kNumProfileClasses; ++c) {
    if (!profileWellFormed(set.by_class[c])) {
      set.by_class[c] = replacement;
    }
  }
  return false;
}

/// How much certainty a track must show before its label is allowed to buy a
/// profile weaker than the fallback.
struct ProfileSelectionPolicy
{
  /// Master switch. False reproduces the single global Stage-5C.1
  /// configuration exactly, and is the production default.
  bool class_aware{false};

  /// Minimum smoothed `classification_confidence` before a semantic label is
  /// honoured, dimensionless [0, 1]. 0.60 is the measured floor of the
  /// VEHICLE -> PEDESTRIAN confusion (section 1): BELOW it no label is
  /// trusted at all, and ABOVE it the ambiguity sets -- not the confidence --
  /// are what keep the selection conservative, because the confusion extends
  /// to 1.000. The gate is therefore doing the job it can do (rejecting
  /// weakly-held labels) and is not asked to do the job it cannot.
  double min_confidence{0.60};

  /// Minimum successful associations before a semantic label is honoured,
  /// count. 5 is the earliest observation at which the Stage-5B stabiliser
  /// can hold a label it reached from two consecutive confident scans
  /// (instant labels begin at observation 3; two scans at confidence 1.0
  /// cross enter_threshold 0.45 at 0.400 then 0.640), plus one, so a label
  /// must have survived at least one further scan. Measured first-label
  /// observation over the Stage-5B tracks: min 4, median 16.
  uint32_t min_observations{5};

  /// Scans after a stable-label change during which the published profile
  /// stays the envelope of the old and the new, count. 3 scans is 0.6 s at
  /// the 5 Hz LDS, one full `smoothing = 0.40` EWMA window (1/alpha = 2.5
  /// scans), so a shrink cannot be published until the new label has
  /// outlived the evidence window that produced the old one.
  uint32_t transition_hold_scans{3};
};

/// Why the applied profile is what it is. Values match the PROFILE_*
/// constants in ReachabilityPrediction.msg.
///
/// SINGLE ENCODING, following the Stage-5C.1 precedent that two encodings of
/// one fact can disagree: there is no separate `fallback_used` boolean.
/// "A conservative substitution happened" is exactly
/// `reason >= kFallbackUnknownLabel`.
enum class ProfileReason : uint8_t
{
  /// Class-aware selection is disabled: the single global configuration was
  /// used and every number is bit-identical to Stage-5C.1.
  kGlobal = 0,
  /// The stable label's own closure envelope was applied.
  kClassClosure = 1,
  /// Label was UNKNOWN; the fallback envelope was substituted.
  kFallbackUnknownLabel = 2,
  /// Label was semantic but its confidence was below min_confidence.
  kFallbackLowConfidence = 3,
  /// Label was semantic and confident but the track was younger than
  /// min_observations.
  kFallbackImmatureTrack = 4,
  /// The configured profile table was malformed and had been sanitized.
  kFallbackInvalidConfig = 5,
  /// A stable-label change is within its hold window: the envelope of the
  /// previously held profile and the current selection was applied, and it
  /// dominates both.
  kTransitionHold = 6,
};

/// Per-track state the transition rule needs. Lives on the track, so it dies
/// with the track id and a new id starts from scratch -- the same lifetime
/// rule the Stage-5B class state already has.
struct ClassProfileState
{
  ObstacleClass last_label{ObstacleClass::UNKNOWN};
  MotionCapabilityProfile held{};
  uint32_t hold_scans_left{0};
  bool initialized{false};
};

/// One profile decision, with everything a consumer or an audit needs to
/// reconstruct it.
struct ProfileSelection
{
  /// The profile actually applied, after closure, fallback and transition.
  MotionCapabilityProfile profile{};
  /// The stable label the decision was made from.
  ObstacleClass label{ObstacleClass::UNKNOWN};
  /// The smoothed confidence the decision was made from.
  double confidence{0.0};
  /// Which class's own profile the applied envelope coincides with, or
  /// UNKNOWN when it coincides with none (a strict envelope). Naming only.
  ObstacleClass dominating_class{ObstacleClass::UNKNOWN};
  ProfileReason reason{ProfileReason::kGlobal};
};

/// The member of `mask` whose profile equals `p` exactly, preferring the
/// strongest such class, or UNKNOWN when none does. Reporting only: it makes
/// "a PEDESTRIAN label was seen but a VEHICLE-grade profile was applied"
/// legible on the wire without a second class field.
inline ObstacleClass dominatingClass(
  const MotionCapabilityProfile & p, uint8_t mask, const CapabilityProfileSet & set)
{
  for (size_t c = kNumProfileClasses; c-- > 0; ) {
    const ObstacleClass oc = static_cast<ObstacleClass>(c);
    if ((mask & classBit(oc)) != 0 && profileEqual(set.by_class[c], p)) {
      return oc;
    }
  }
  return ObstacleClass::UNKNOWN;
}

/// Select the capability profile for one track on one scan, and advance that
/// track's transition state.
///
/// `global` is the single Stage-5C.1 configuration, returned verbatim when
/// class-aware selection is off so that the legacy path is bit-identical
/// rather than merely equivalent.
///
/// `config_valid` is the result of `sanitizeProfileSet()` from parameter-read
/// time: false forces the fallback and reports
/// `kFallbackInvalidConfig`, so a malformed table is visible on every sample
/// instead of only in a start-up log.
///
/// Guarantees, all of them consequences of `profileEnvelope` being the least
/// upper bound of the order in section 2:
///   (I1) result >= profile(label)                       always
///   (I3) result >= profile(c) for every class c         whenever uncertain
///   (T1) result >= this scan's unheld selection         always
///   (T2) result >= the previously held profile          during a hold
///   (T3) result <= fallback                             always
inline ProfileSelection selectCapabilityProfile(
  ObstacleClass stable_label,
  double confidence,
  uint32_t observations,
  const CapabilityProfileSet & set,
  const AmbiguitySets & amb,
  const ProfileSelectionPolicy & policy,
  const MotionCapabilityProfile & global,
  bool config_valid,
  ClassProfileState & state)
{
  ProfileSelection out;
  out.label = stable_label;
  out.confidence = confidence;

  if (!policy.class_aware) {
    out.profile = global;
    out.reason = ProfileReason::kGlobal;
    out.dominating_class = ObstacleClass::UNKNOWN;
    // The transition state is deliberately left untouched: the global path
    // must not acquire hidden per-track state, so switching class-aware on at
    // runtime starts every track from a clean, conservative first scan.
    return out;
  }

  const MotionCapabilityProfile fallback = fallbackProfile(set, amb);
  const uint8_t fallback_mask = closureMask(ObstacleClass::UNKNOWN, amb);

  MotionCapabilityProfile candidate;
  uint8_t candidate_mask;
  ProfileReason reason;

  if (!config_valid) {
    candidate = fallback;
    candidate_mask = fallback_mask;
    reason = ProfileReason::kFallbackInvalidConfig;
  } else if (stable_label == ObstacleClass::UNKNOWN) {
    candidate = fallback;
    candidate_mask = fallback_mask;
    reason = ProfileReason::kFallbackUnknownLabel;
  } else if (!(confidence >= policy.min_confidence)) {
    // Written as !(>=) so a NaN confidence takes the conservative branch.
    candidate = fallback;
    candidate_mask = fallback_mask;
    reason = ProfileReason::kFallbackLowConfidence;
  } else if (observations < policy.min_observations) {
    candidate = fallback;
    candidate_mask = fallback_mask;
    reason = ProfileReason::kFallbackImmatureTrack;
  } else {
    candidate_mask = closureMask(stable_label, amb);
    candidate = maskEnvelope(candidate_mask, set);
    reason = ProfileReason::kClassClosure;
  }

  // --- transition rule (section 6) --------------------------------------
  bool held_applied = false;
  if (!state.initialized) {
    state.held = candidate;
    state.hold_scans_left = 0;
  } else if (stable_label != state.last_label) {
    state.held = profileEnvelope(state.held, candidate);
    state.hold_scans_left = policy.transition_hold_scans;
    held_applied = true;
  } else if (state.hold_scans_left > 0) {
    state.held = profileEnvelope(state.held, candidate);
    --state.hold_scans_left;
    held_applied = true;
  } else {
    state.held = candidate;
  }
  state.last_label = stable_label;
  state.initialized = true;

  out.profile = state.held;
  // A hold that happens not to have enlarged anything is not reported as a
  // hold: the reason names what the published numbers ARE, so it must not
  // claim an envelope the sample does not carry.
  out.reason = (held_applied && !profileEqual(state.held, candidate)) ?
    ProfileReason::kTransitionHold : reason;
  out.dominating_class = dominatingClass(out.profile, fallback_mask, set);
  return out;
}

/// Map a selected profile onto the Stage-4G4/5C.1 bounds, leaving every other
/// field of `base` -- sigma level, tolerance, enforce_observed_speed -- alone.
/// This is the ONLY coupling point between Stage-5D and the reachability
/// mathematics: the equation, the validity contract and the published region
/// are unchanged functions of whatever lands here.
inline ReachabilityBounds boundsForProfile(
  const ReachabilityBounds & base, const MotionCapabilityProfile & p)
{
  ReachabilityBounds out = base;
  out.max_speed = p.max_speed;
  out.max_acceleration = p.max_acceleration;
  out.safety_margin = p.margin;
  return out;
}

}  // namespace predictive_nav_tracking

#endif  // PREDICTIVE_NAV_TRACKING__CAPABILITY_PROFILES_HPP_
