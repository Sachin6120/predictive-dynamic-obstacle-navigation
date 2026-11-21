#!/usr/bin/env python3
"""
Stage-5B classification validation set: the scenario definitions.

BOUNDED BY DESIGN. This is a classification dataset, not a navigation one: no
goal is ever sent, the robot never moves, and none of the 448 Stage-4F
navigation trials are rerun. What varies is only what classification depends
on -- how far away the object is, what aspect it presents to the sensor, and
how fast it is moving.

THE ROBOT IS STATIONARY ON PURPOSE. A driving robot adds AMCL pose error,
which the Stage-4G1 audit measured at a p95 of 0.118 m while moving, and that
error leaks into the static-rejection stage as false dynamic returns. Holding
the robot still removes that confound, so a misclassification is attributable
to the classifier and the sensor rather than to localisation drift. The
separate navigation integration check is what exercises the moving case.

WHAT EACH PATTERN MEASURES
--------------------------
crossing  Lateral traverse, actor yawed along its direction of travel, so the
          robot sees its FLANK. Sweeps bearing over tens of degrees. This is
          the aspect that carries the most geometric information.
approach  Head-on closing motion, actor facing the robot, so the robot sees
          its END. Sweeps range downwards, which is what exposes the
          sparse-at-range limit.
recede    The same axis travelled outbound, so range sweeps the other way and
          the track is born near and lost far. Included because track age and
          range are correlated in `approach` and anti-correlated here.
oblique   A diagonal traverse, yawed along its direction of travel, chosen so
          that the travel line is neither radial nor tangential to the robot.
          The actor's long axis therefore sits at roughly 45 deg to the line
          of sight and the robot sees a CORNER aspect -- the only view that
          gives a box-shaped body measurable depth.

GEOMETRY IS FOOTPRINT-AWARE. Start pose and travel are not shared constants:
they are derived per (actor, pattern) from the actor's own world-frame
bounding box at its yaw, so that NO PART of any actor ever enters a wall, a
localisation pillar or the robot's own footprint. A 3.8 m car therefore gets a
shorter crossing traverse than a pedestrian, and that is reported as a sample
count rather than hidden -- it is a consequence of putting a car-sized object
in a 10 x 8 m room.

Speeds 0.25 / 0.50 / 0.75 m/s span the band the Stage-4G4 scenario driver
commands. NOTE, and it is reported rather than worked around: that band is the
same for every class, so the measured speed distributions overlap almost
completely and speed carries nearly no discriminative information in this
dataset. Geometry decides. Real pedestrians, cyclists and cars separate on
speed; simulated ones here do not.
"""
import math

from actors import ACTORS, gt_class_of   # noqa: F401  (re-exported for callers)

#: Robot pose for every classification scenario, map frame. The Stage-4F start
#: pose, so arena, map and localisation prior are the validated ones.
ROBOT = (-3.0, 0.0, 0.0)

#: Region ANY PART of an actor may occupy, map frame, m.
#: The Stage-4F room is x in [-5, 5], y in [-4, 4] with 0.30 m localisation
#: pillars centred at |y| = 3.0. X_MIN keeps actors clear of the robot's own
#: footprint at (-3.0, 0.0); |y| <= 2.6 clears the pillars and their inflation.
X_MIN, X_MAX = -1.5, 4.6
Y_MIN, Y_MAX = -2.6, 2.6

#: pattern -> (unit travel direction, actor yaw, nominal travel m, fixed axis)
#: 'fixed' names the coordinate held constant and its value.
#:
#: EVERY PATTERN YAWS THE ACTOR ALONG ITS DIRECTION OF TRAVEL, which is both
#: what a real pedestrian, cyclist or car does and what keeps the commanded
#: motion a pure forward (body +x) velocity. An earlier revision yawed the
#: oblique actor across its travel direction, which required a large body-Y
#: command; measured against ground-truth odometry those runs covered only
#: 0.8-1.1 m of their 3.6 m travel, so the lateral command was not being
#: followed. Aligning yaw with travel removes that dependence entirely, and
#: the corner aspect is obtained geometrically instead -- by choosing a travel
#: LINE that is oblique to the robot's line of sight.
PATTERNS = {
    'crossing': ((0.0, 1.0), math.pi / 2, 4.6, ('x', 0.3)),
    'approach': ((-1.0, 0.0), math.pi, 4.4, ('y', 0.0)),
    'recede': ((1.0, 0.0), 0.0, 4.4, ('y', 0.0)),
    'oblique': ((1.0, 1.0), math.pi / 4, 3.6, None),
}

SPEEDS = (0.25, 0.50, 0.75)


def footprint_extent(name):
    """Actor footprint extent along its own x and y axes, m."""
    _, parts = ACTORS[name]
    xs, ys = [], []
    for part in parts:
        if part[0] == 'box':
            _, dx, dy, ox, oy = part
            xs += [ox - dx / 2.0, ox + dx / 2.0]
            ys += [oy - dy / 2.0, oy + dy / 2.0]
        else:
            _, r, ox, oy = part
            xs += [ox - r, ox + r]
            ys += [oy - r, oy + r]
    return max(xs) - min(xs), max(ys) - min(ys)


def world_half_extent(name, yaw):
    """World-frame axis-aligned half-extents of `name` rotated by `yaw`, m."""
    length, width = footprint_extent(name)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    return 0.5 * (length * c + width * s), 0.5 * (length * s + width * c)


def _span(lo, hi, half):
    """Interval the actor CENTRE may occupy on one axis, or None if it cannot fit."""
    a, b = lo + half, hi - half
    return (a, b) if b >= a else None


def plan(name, pattern):
    """
    Start pose and travel distance for one (actor, pattern), or None.

    Returns (x0, y0, yaw, travel). The actor starts at one end of the span its
    own bounding box allows and travels towards the other, capped at the
    pattern's nominal travel. None when the actor cannot fit the pattern at
    all, which is itself a reportable result.
    """
    (ux, uy), yaw, nominal, fixed = PATTERNS[pattern]
    hx, hy = world_half_extent(name, yaw)
    sx, sy = _span(X_MIN, X_MAX, hx), _span(Y_MIN, Y_MAX, hy)
    if sx is None or sy is None:
        return None

    # Hold the pattern's fixed coordinate, clamped into the feasible span.
    cx = min(max(fixed[1], sx[0]), sx[1]) if fixed and fixed[0] == 'x' else None
    cy = min(max(fixed[1], sy[0]), sy[1]) if fixed and fixed[0] == 'y' else None

    # Travel is limited by whichever axis the motion actually consumes.
    limits = []
    if abs(ux) > 1e-9:
        limits.append((sx[1] - sx[0]) / abs(ux))
    if abs(uy) > 1e-9:
        limits.append((sy[1] - sy[0]) / abs(uy))
    travel = min([nominal] + limits)
    if travel <= 0.25:
        return None

    # Start at the end of the span the motion leads away from.
    x0 = cx if cx is not None else (sx[1] if ux < 0 else sx[0])
    y0 = cy if cy is not None else (sy[1] if uy < 0 else sy[0])
    if cx is None and abs(ux) < 1e-9:
        x0 = 0.5 * (sx[0] + sx[1])
    if cy is None and abs(uy) < 1e-9:
        y0 = 0.5 * (sy[0] + sy[1])
    if abs(uy) > 1e-9 and cy is None:
        y0 = sy[1] if uy < 0 else sy[0]
    if abs(ux) > 1e-9 and cx is None:
        x0 = sx[1] if ux < 0 else sx[0]
    return x0, y0, yaw, travel


def scenarios():
    """Every feasible (actor, pattern, speed) scenario, deterministically ordered."""
    out = []
    for actor in sorted(ACTORS):
        for pattern in ('crossing', 'approach', 'recede', 'oblique'):
            planned = plan(actor, pattern)
            if planned is None:
                continue
            x0, y0, yaw, travel = planned
            (ux, uy), _, _, _ = PATTERNS[pattern]
            norm = math.hypot(ux, uy)
            for speed in SPEEDS:
                out.append({
                    'id': f'{actor}__{pattern}__v{speed:.2f}',
                    'actor': actor,
                    'gt_class': gt_class_of(actor),
                    'pattern': pattern,
                    'speed': speed,
                    'x0': round(x0, 4),
                    'y0': round(y0, 4),
                    'yaw': round(yaw, 6),
                    'vx': round(speed * ux / norm, 4),
                    'vy': round(speed * uy / norm, 4),
                    'travel': round(travel, 3),
                    'duration': round(travel / speed, 2),
                })
    return out


def main():
    """Print the scenario table."""
    rows = scenarios()
    skipped = [(a, p) for a in sorted(ACTORS)
               for p in PATTERNS if plan(a, p) is None]
    print(f'{len(rows)} scenarios over {len(ACTORS)} actors, '
          f'{len(PATTERNS)} patterns, {len(SPEEDS)} speeds')
    if skipped:
        print(f'infeasible (actor does not fit the pattern): {skipped}')
    for s in rows:
        print(f"  {s['id']:46s} gt={s['gt_class']:10s} "
              f"start=({s['x0']:+.2f},{s['y0']:+.2f}) yaw={s['yaw']:.3f} "
              f"v=({s['vx']:+.2f},{s['vy']:+.2f}) travel={s['travel']:.2f}m "
              f"t={s['duration']:.1f}s")


if __name__ == '__main__':
    main()
