#!/usr/bin/env python3
"""Stage-4G7 geometry designer: find wall/obstacle layouts where the observation
gap and the conflict actually coincide.

Stage-4G6's null was partly structural -- the gap ended seconds before the robot
was at risk. Rather than discover that again after hours of Gazebo, this solves
the line-of-sight problem analytically first: for a candidate layout it reports
when the LiDAR loses the target behind static geometry, when the robot-target
closest approach occurs, and how much of the pre-conflict interval is spent
unobserved. Only layouts that pass are simulated.

The robot is modelled as moving along y = 0 at a constant nominal speed. That is
an approximation (the real robot slows near obstacles), so every layout selected
here is re-verified against the actual Gazebo recording.
"""
import argparse
import itertools
import json
import math


def seg_blocks(p, q, box):
    """True if segment p->q intersects the axis-aligned box (cx, cy, sx, sy)."""
    cx, cy, sx, sy = box
    xmin, xmax = cx - sx / 2, cx + sx / 2
    ymin, ymax = cy - sy / 2, cy + sy / 2
    # Liang-Barsky
    t0, t1 = 0.0, 1.0
    dx, dy = q[0] - p[0], q[1] - p[1]
    for num, den in ((xmin - p[0], dx), (p[0] - xmax, -dx),
                     (ymin - p[1], dy), (p[1] - ymax, -dy)):
        if den == 0:
            if num > 0:
                return False
            continue
        t = num / den
        if den > 0:
            if t > t1:
                return False
            t0 = max(t0, t)
        else:
            if t < t0:
                return False
            t1 = min(t1, t)
    return t0 <= t1


def path_clear(walls, obs, duration=30.0, dt=0.1, obstacle_radius=0.20, margin=0.05):
    """True if the target never drives into static geometry.

    Added after a pilot showed the target physically colliding with the occluder
    and stopping short of the route: the visibility solver alone will happily
    propose a path straight through a wall, which silently destroys the
    scenario. Every layout must pass this before it is simulated in Gazebo.
    """
    keep = obstacle_radius + margin
    for k in range(int(duration / dt) + 1):
        ox, oy = obs(k * dt)
        for cx, cy, sx, sy in walls:
            dx = max(abs(ox - cx) - sx / 2, 0.0)
            dy = max(abs(oy - cy) - sy / 2, 0.0)
            if math.hypot(dx, dy) < keep:
                return False
    return True


def simulate(walls, robot_x0, robot_speed, obs, duration=30.0, dt=0.2,
             sensor_dy=0.0, obstacle_radius=0.20, robot_radius=0.22):
    """obs(t) -> (x, y). Returns the per-scan visibility/geometry timeline."""
    rows = []
    for k in range(int(duration / dt) + 1):
        t = k * dt
        rx = robot_x0 + robot_speed * t
        if rx > 3.0:
            rx = 3.0
        ox, oy = obs(t)
        # The target is visible if ANY part of its silhouette is unblocked; the
        # tracker needs >= 3 returns, so sample a few points across the cylinder.
        seen = 0
        for frac in (-0.7, -0.35, 0.0, 0.35, 0.7):
            ang = math.atan2(oy, ox - rx) + math.pi / 2
            px = ox + frac * obstacle_radius * math.cos(ang)
            py = oy + frac * obstacle_radius * math.sin(ang)
            if not any(seg_blocks((rx, sensor_dy), (px, py), w) for w in walls):
                seen += 1
        gap = math.hypot(ox - rx, oy) - obstacle_radius - robot_radius
        rows.append(dict(t=round(t, 3), rx=round(rx, 3), ox=round(ox, 3),
                         oy=round(oy, 3), seen=seen, visible=seen >= 3,
                         clearance=round(gap, 3)))
    return rows


def summarise(rows):
    vis = [r for r in rows if r['visible']]
    first_vis = vis[0]['t'] if vis else None
    # Longest contiguous hidden run that STARTS after the target was first seen.
    runs, cur = [], []
    started = False
    for r in rows:
        if r['visible']:
            started = True
            if cur:
                runs.append(cur)
                cur = []
        elif started:
            cur.append(r['t'])
    if cur:
        runs.append(cur)
    gap = max(runs, key=len) if runs else []
    closest = min(rows, key=lambda r: r['clearance'])
    out = dict(first_visible_s=first_vis,
               visible_before_gap_s=(round(gap[0] - first_vis, 2)
                                     if gap and first_vis is not None else None),
               gap_start_s=gap[0] if gap else None,
               gap_end_s=gap[-1] if gap else None,
               gap_duration_s=round(gap[-1] - gap[0] + .2, 2) if gap else 0.0,
               closest_t=closest['t'], closest_clearance=closest['clearance'])
    # conflict_overlap: fraction of the pre-conflict approach spent unobserved.
    if gap and first_vis is not None and closest['t'] > gap[0]:
        window = closest['t'] - gap[0]
        hidden = min(out['gap_duration_s'], window)
        out['conflict_overlap'] = round(hidden / window, 3) if window > 0 else None
        out['reacquire_to_closest_s'] = round(closest['t'] - gap[-1], 2)
    else:
        out['conflict_overlap'] = 0.0
        out['reacquire_to_closest_s'] = None
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out')
    ap.add_argument('--robot-speed', type=float, default=0.40)
    args = ap.parse_args()

    WALL_T = 0.15
    results = []
    # Blind corner: a north-south wall standing just north of the route, with
    # its southern tip forming the corner the target comes around.
    for tip_y, wall_top, wall_x, col_dx, speed, start_y in itertools.product(
            (0.35, 0.5), (1.6, 2.0), (1.2, 1.4), (0.55, 0.7), (0.5, 0.65, 0.8), (2.4, 2.8)):
        walls = [(wall_x, (tip_y + wall_top) / 2, WALL_T, wall_top - tip_y)]
        ox = wall_x + col_dx
        rows = simulate(walls, -3.05, args.robot_speed,
                        lambda t, ox=ox, s=speed, y0=start_y: (ox, y0 - s * t))
        s = summarise(rows)
        s.update(kind='corner', wall_x=wall_x, tip_y=tip_y, wall_top=wall_top,
                 col_x=round(ox, 2), speed=speed, start_y=start_y)
        results.append(s)

    good = [r for r in results
            if r['gap_duration_s'] >= .35 and r['gap_duration_s'] <= 1.15
            and (r['visible_before_gap_s'] or 0) >= 2.0
            and (r['conflict_overlap'] or 0) >= .5
            and r['closest_clearance'] < .9]
    good.sort(key=lambda r: (-r['conflict_overlap'], r['gap_duration_s']))
    print(f'{len(results)} layouts, {len(good)} viable\n')
    print(f"{'gap':>5s} {'overlap':>8s} {'vis_before':>10s} {'reacq->close':>12s} "
          f"{'closest':>8s} | wall_x tip top col speed start")
    for r in good[:25]:
        print(f"{r['gap_duration_s']:5.2f} {r['conflict_overlap']:8.2f} "
              f"{r['visible_before_gap_s']:10.2f} {r['reacquire_to_closest_s']:12.2f} "
              f"{r['closest_clearance']:8.3f} | {r['wall_x']} {r['tip_y']} {r['wall_top']} "
              f"{r['col_x']} {r['speed']} {r['start_y']}")
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(dict(all=results, viable=good), f, indent=1)


if __name__ == '__main__':
    main()
