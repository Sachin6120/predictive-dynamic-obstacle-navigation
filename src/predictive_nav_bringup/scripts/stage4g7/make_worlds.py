#!/usr/bin/env python3
"""Generate the two Stage-4G7 occlusion worlds AND their occupancy maps.

Same contract as make_stage4f_world.py: the Gazebo world and the map AMCL
localises against are emitted from ONE geometry list per variant, so they cannot
drift apart. That matters more here than at Stage-4F, because the new occluders
are the whole point of the experiment: a wall present in the SDF but missing
from the PGM would be rejected by neither the tracker's static filter nor AMCL,
and would appear as a permanent false dynamic obstacle.

Both variants are the validated Stage-4F 10 x 8 m room with its four
localisation pillars, plus static occluding geometry north of the robot's route:

  corner  - a single 0.50 x 0.15 m stub at (1.40, 0.90). The target descends
            just east of it and comes around its corner into the route.
  doorway - a 0.30 m jamb at (1.05, 0.90) and a 1.10 m wall at (2.35, 0.90),
            leaving a 0.60 m doorway at x in [1.20, 1.80] that the target
            descends through.

Every occluder's south face sits at y = 0.825, i.e. at least inflation_radius
(0.70 m) from the route centre line, so the robot's route keeps the ~zero static
inflation cost that Stage-4F was built to guarantee. Nothing about the room the
earlier stages validated is changed; geometry is only added.
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
from make_stage4f_world import (  # noqa: E402  (shared, single source of truth)
    BOX_TEMPLATE, GEOMETRY as STAGE4F_GEOMETRY, MAP_MIN, MAP_MAX, RES, WALL_H,
    WALL_T, WORLD_HEADER)

# Every occluder sits at y = 0.85, so its south face is at y = 0.775 -- at or
# beyond inflation_radius (0.70 m) from the route centre line, preserving the
# ~zero static inflation on the robot's route that Stage-4F was built to give.
#
# The three corner variants differ ONLY in the stub's width, which is the single
# dimension that sets the observation-gap duration (0.40 / 0.60 / 0.80 s for
# 0.20 / 0.25 / 0.30 m). Everything else -- room, pillars, occluder position,
# target path -- is identical across them, so gap duration is varied through
# GEOMETRY rather than through any predictor parameter.
# Every occluder sits at y = 0.85, so its south face is at y = 0.775 -- at or
# beyond inflation_radius (0.70 m) from the route centre line, preserving the
# ~zero static inflation on the robot's route that Stage-4F was built to give.
#
# c30 / c45 are the blind-corner variants and differ ONLY in the stub's width,
# which is the single dimension that sets the observation-gap duration. Room,
# pillars, occluder position and target path are identical across them, so gap
# duration is varied through GEOMETRY, never through a predictor parameter.
#
# `door` is a real doorway: two walls leaving a 0.70 m gap at x in [-0.45, 0.25].
# Because the robot's line of sight to a northern target crosses y = 0.85 well
# to the WEST early on and sweeps EAST as the robot advances, a doorway hides
# such a target from the start and reveals it late -- the opposite order from a
# corner. It is therefore the never-observed / expiry case, not a coasting one.
OCCLUDERS = {
    'c30': [('occ_corner', -0.80, 0.85, 0.30, WALL_T)],
    'c45': [('occ_corner', -0.80, 0.85, 0.45, WALL_T)],
    'door': [('occ_wall_west', -2.525, 0.85, 4.15, WALL_T),
             ('occ_wall_east', 2.425, 0.85, 4.35, WALL_T)],
}

def write_world(path, geometry):
    parts = [WORLD_HEADER]
    for name, cx, cy, sx, sy in geometry:
        parts.append(BOX_TEMPLATE.format(name=name, cx=cx, cy=cy, hz=WALL_H / 2.0,
                                         sx=sx, sy=sy, sz=WALL_H))
    parts.append('\n  </world>\n</sdf>\n')
    with open(path, 'w') as f:
        f.write(''.join(parts))
    return path


def write_map(pgm_path, yaml_path, geometry):
    w = int(round((MAP_MAX[0] - MAP_MIN[0]) / RES))
    h = int(round((MAP_MAX[1] - MAP_MIN[1]) / RES))
    FREE, OCC, UNK = 254, 0, 205
    grid = [[UNK] * w for _ in range(h)]
    inner = (-5.0 + WALL_T / 2.0, -4.0 + WALL_T / 2.0,
             5.0 - WALL_T / 2.0, 4.0 - WALL_T / 2.0)
    for row in range(h):
        wy = MAP_MIN[1] + (row + 0.5) * RES
        for col in range(w):
            wx = MAP_MIN[0] + (col + 0.5) * RES
            if inner[0] <= wx <= inner[2] and inner[1] <= wy <= inner[3]:
                grid[row][col] = FREE
    for _name, cx, cy, sx, sy in geometry:
        c0 = int((cx - sx / 2.0 - MAP_MIN[0]) / RES)
        c1 = int((cx + sx / 2.0 - MAP_MIN[0]) / RES)
        r0 = int((cy - sy / 2.0 - MAP_MIN[1]) / RES)
        r1 = int((cy + sy / 2.0 - MAP_MIN[1]) / RES)
        for row in range(max(r0, 0), min(r1 + 1, h)):
            for col in range(max(c0, 0), min(c1 + 1, w)):
                grid[row][col] = OCC
    data = bytearray()
    for row in range(h - 1, -1, -1):
        data.extend(bytes(grid[row]))
    with open(pgm_path, 'wb') as f:
        f.write(b'P5\n')
        f.write(b'# GENERATED by scripts/stage4g7/make_worlds.py\n')
        f.write(f'{w} {h}\n255\n'.encode())
        f.write(bytes(data))
    with open(yaml_path, 'w') as f:
        f.write('# GENERATED by scripts/stage4g7/make_worlds.py -- do not edit by hand.\n'
                f'image: {os.path.basename(pgm_path)}\n'
                f'resolution: {RES:.6f}\n'
                f'origin: [{MAP_MIN[0]:.6f}, {MAP_MIN[1]:.6f}, 0.000000]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return w, h


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pkg-dir', default='src/predictive_nav_bringup')
    args = ap.parse_args()
    worlds = os.path.join(args.pkg_dir, 'worlds')
    maps = os.path.join(args.pkg_dir, 'maps')
    os.makedirs(worlds, exist_ok=True)
    os.makedirs(maps, exist_ok=True)

    for variant, extra in OCCLUDERS.items():
        geometry = list(STAGE4F_GEOMETRY) + extra
        name = f'stage4g7_{variant}'
        write_world(os.path.join(worlds, f'{name}.sdf.xacro'), geometry)
        w, h = write_map(os.path.join(maps, f'{name}.pgm'),
                         os.path.join(maps, f'{name}.yaml'), geometry)
        south = min(cy - sy / 2 for _n, _cx, cy, _sx, sy in extra)
        print(f'{name}: {len(extra)} occluder(s), map {w}x{h} px @ {RES} m, '
              f'nearest occluder face at y={south:.3f} '
              f'({south:.3f} >= inflation_radius 0.70: '
              f'{"OK" if south >= 0.70 else "TOO CLOSE TO ROUTE"})')


if __name__ == '__main__':
    main()
