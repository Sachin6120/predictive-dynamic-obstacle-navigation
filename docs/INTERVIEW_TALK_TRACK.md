# Interview talk track

Spoken-form explanations and answers for this project. Every number is from
committed evidence under `validation/`.

---

## 30 seconds

I built a ROS 2 navigation system that avoids moving obstacles *before* the
conflict rather than after. A single 2D LiDAR feeds a C++ tracker that clusters,
tracks and classifies each obstacle — pedestrian, bicycle, vehicle or unknown —
and predicts where it will be over the next three seconds, with uncertainty. That
prediction goes into Nav2 through a costmap plugin I wrote, so the stock MPPI
controller plans against future occupancy instead of last-seen position. I
implemented three different predictors, compared them in a 132-trial campaign,
and shipped the simplest one — because the measurements said the expensive one
bought nothing.

---

## 2 minutes

The problem with reactive obstacle avoidance is that your warning time shrinks as
the obstacle gets faster — you only see it once it is already close. So the system
tracks every dynamic obstacle with its own Kalman filter and propagates it forward,
then paints that predicted future occupancy into the Nav2 local costmap as soft,
time-decaying cost. It is combined with `updateWithMax` and capped below the
inscribed-obstacle value, so a prediction can add cost on top of sensed geometry
but can never masquerade as a sensed lethal obstacle. No Nav2 or MPPI source was
modified — it is a plugin plus one critic weight.

On top of that I added semantic classification from LiDAR cluster geometry, and
then the interesting question: does knowing *what* the obstacle is let you predict
*better*? To answer that I implemented reachability prediction — instead of a
probabilistic ellipse, a bounded set of everywhere the obstacle could physically
reach given a declared speed and acceleration capability — in two forms: one
global capability premise for all obstacles, and per-class capability profiles.

The final campaign ran all three predictors over the same nine conditions, same
map, same tracker binary, same controller — 135 trials, 132 scored. The result was
a tie on every navigation metric: every pairwise difference sat inside its
two-sigma band. The only thing that resolved was how much costmap each predictor
painted, and the class-aware one painted the most. So I kept constant-velocity
Kalman in production, kept global reachability as a secondary signal because it
can report when its own validity premise is violated — which a covariance ellipse
structurally cannot — and documented class-aware reachability as a research
result.

The reason it failed is the part I find most interesting, and I diagnosed it
rather than guessed: the class profiles produce large isotropic regions, the
production influence window is only 0.55 m, and at that window every class paints
an identical 529 of 529 cells. The semantics are real in the geometry and
invisible to the planner. When I widened the window so the classes *were*
distinguishable, navigation got worse — eight collisions where there had been
zero — because the corridor stopped being a corridor and became a blanket over
the whole local costmap, and MPPI had no gradient left to steer on.

---

## Likely technical questions

### 1. Why did CV-Kalman win the production decision?

Not because it was safer — it is the only arm that recorded a collision, one in 45
trials, and I counted it. It won because the decision rule said an incumbent is
not displaced by a tie, and this was a tie: every navigation metric, for every
pair of arms, fell inside its two-sigma-of-the-difference band. The differences
that *did* resolve were costs, not benefits — class-aware painted 203 more cells
than CV with no outcome to show for it.

The positive case for CV is structural. Inside the 0.55 m window, CV still has
graded cost structure through the first second — 54 cells at 0.5 s, 260 at 1.0 s,
out of 529 — while global saturates from 1.0 s and class-aware is saturated
already at 0.5 s. For a robot moving at 0.48 m/s, that first second is the horizon
the controller actually acts on, and a saturated field carries no information the
planner can use.

### 2. What is the difference between probabilistic covariance and reachability?

They answer different questions. The Kalman covariance is a *likelihood*
statement: given my motion model and noise model, here is where the obstacle
probably is, and I consume it at a chosen sigma level — 1.5σ in production. It has
no premise that can be falsified; if the obstacle does something the model never
contemplated, the ellipse is simply wrong and says nothing about it.

Reachability is a *worst-case set* statement: given a declared capability — a max
speed and a max acceleration — here is the complete region the obstacle could
occupy at time t. That premise is explicit, so it is falsifiable at runtime: the
system fires a contract breach when observed speed exceeds the declared cap, names
the track and the speeds, and enlarges the region. That signal is the reason I
kept global reachability deployed as a secondary representation, even though it
is not what the planner consumes.

The two are not commensurable and I never combine them into one figure of merit.
CV's observed containment of 0.9783 at k = 1.5 is a calibration property;
class-aware's capability containment of 1.0000 is a set property. Putting them in
the same column would be a category error.

### 3. Why can conservative prediction hurt navigation?

Because a cost field only helps if it has gradient. A conservative predictor
covers more of the space, and at some point it covers *all* of the space the
planner can see — at which point every candidate trajectory looks equally bad and
the controller has nothing to optimise against.

I measured exactly that. At a 3.00 m influence window the class-aware corridor
covered 101.7% of the local costmap; mean predictive coverage went from about 5%
of the window to about 51%, success fell from 35/35 to 29/36, collisions went from
0 to 8, collision-monitor interventions from 0 to 35, and navigation time from
19.6 s to 30.6 s. Some trials simply stalled until timeout. And critically it was
not a compute problem — layer update cost rose only from 25.7 to 29.0 µs with zero
missed deadlines. Expanded conservatism was affordable and destructive at the same
time.

### 4. What are the limits of classifying obstacles from a 2D LiDAR?

Hard ones. A planar scan gives you one horizontal slice of the near face of an
object, so the features available are essentially length, thickness, elongation
and point count at a given range. Bicycles are cleanly separable — long, thin, one
consistent signature, precision 1.000 in my data. Pedestrians and vehicles are
not: a vehicle's visible near face at some aspect angles looks exactly like a
pedestrian, and that is geometry, not a tuning failure. My per-detection accuracy
is 55.7%, per-track 63.2%.

I made the classifier decline rather than guess — it reports UNKNOWN when the
feature conjunction is not satisfied — which means the live class-aware fallback
rate was 0.4193: the class closes on only about 58% of samples. That is honest,
but it also means the class-aware premise was not even exercised as designed on
the remainder. The fix is sensor fusion — a camera — not a better threshold on 2D
geometry.

### 5. How does the Nav2 costmap plugin work?

It is a `nav2_costmap_2d::Layer` subclass registered through pluginlib, so it is
loaded by name from the costmap parameter file and participates in the normal
update cycle. It subscribes to the tracker's `TrackedObjectArray`, and on each
`updateCosts` call it rasterizes every track's predicted regions over the horizon
into the costmap.

Three design rules matter. First, it combines with `updateWithMax`, so it can only
raise cost above what sensed geometry already asserts, never lower or replace it.
Second, cost is capped below `INSCRIBED_INFLATED_OBSTACLE`, so a prediction is
never promoted to a sensed lethal obstacle — the planner may route through it if
everything else is worse. Third, cost decays with the prediction time, so the
near-term prediction dominates.

`max_influence_radius` clamps the rasterization *iteration window* — the loop
bounds — not the published geometry. That distinction was a fix I made during the
contract and raster-fidelity audit: before it, the clamp was applied to the
semi-axes themselves, which made the painted region a smaller ellipse rather than
the true ellipse clipped to the window. The cost is 25 µs mean per update against
a 200 ms budget, with zero rate shortfalls across the whole campaign.

---

## If asked "what would you do next?"

Anisotropic reachable sets are the obvious one — the current regions are
isotropic discs, which is why class differences vanish once the disc exceeds the
policy window. A directional set conditioned on heading would keep the class
information inside the window where the planner can see it. Beyond that:
intent-conditioned prediction, a CTRV or IMM motion model instead of pure constant
velocity, camera–LiDAR fusion to fix the classification ceiling, and real-robot
validation — everything here is simulation.
