#!/usr/bin/env python3
"""Stage-5D navigation campaign: global vs runtime class-aware reachability.

Runs the bounded Gazebo comparison against class-specific obstacle bodies.

WHAT VARIES, AND IT IS ONE THING
--------------------------------
The two arms differ in exactly one line of one file:

    tracker_params = stage5d_global_tracker.yaml     (arm GLOBAL)
                   = stage5d_classaware_tracker.yaml (arm CLASS_AWARE)

and those two files differ in EXACTLY ONE LINE,
`reachability_class_aware: false -> true` (verified by diff in
STAGE5D_README.md). Everything else is shared BY CONSTRUCTION, because both
arms go through the same launch invocation with the same values: map, world,
start, goal, obstacle body, obstacle trajectory, release time, sensor model,
AMCL, the Stage-4C Kalman filter, the CV predictor, the Stage-5B classifier,
compound clustering, the reachability mathematics and the Stage-5C.1 contract,
MPPI and every Nav2 parameter, costmap geometry and resolution, and
`prediction_mode: reachability` applied at runtime. MPPI IS NOT RETUNED PER
ARM -- both arms read the UNMODIFIED Stage-4F params_file.

Both arms consume REACHABILITY, not CV, because the question is whether
semantic class improves the reachability corridor. The CV production arm is
unchanged and untouched; it is not one of these arms and is not re-run here.

THE OBSTACLE BODIES ARE THE STAGE-5B ACTORS
-------------------------------------------
Generated from the single ACTORS table that also supplies their ground-truth
class, so geometry cannot disagree with the label. Each is spawned under the
entity name `dynamic_obstacle`, so the existing bridge, the existing
ground-truth odometry path and stage4f_trial.py all work unchanged.

Each actor is yawed ALONG its direction of travel, which is both what the real
object does and what makes it classifiable: a 2D LiDAR measures one horizontal
slice, so a bicycle not yawed along its travel presents its end rather than its
two wheel chords and is not recognisable at all -- the documented head-on
bicycle limitation.

`actor_vehicle_compact` (1.0 x 0.5 m) is the vehicle body, not
`actor_vehicle` (3.8 x 1.7 m): the frozen tracker rejects any cluster whose
bounding-box diagonal exceeds cluster_max_diameter 1.2 m, so the compact body
is the only VEHICLE-class object the unmodified clustering can represent at
all. That is a Stage-5B finding carried forward, not a Stage-5D choice.

GROUND TRUTH is consumed by the trial runner for scoring and for commanding the
scripted obstacle only, exactly as in Stage-4F, and never reaches /scan, the
tracker, the costmap layer, the planner or the controller.

    scripts/stage5d/run_nav.py --out DIR [--trials 3] [--conditions a,b]
    scripts/stage5d/run_nav.py --list
"""
import argparse
import importlib.util
import math
import os
import time

from ament_index_python.packages import (get_package_prefix,
                                         get_package_share_directory)


def _load_bench():
    """Import run_stage4f_bench from the installed share directory.

    Reused rather than reimplemented so that trial execution -- one full
    simulator restart per trial on its own ROS_DOMAIN_ID, straggler cleanup,
    result collection, infrastructure-fault detection -- is the SAME code path
    that produced every Stage-4F and Stage-5C.1 number.
    """
    share = get_package_share_directory('predictive_nav_bringup')
    lib = os.path.join(
        get_package_prefix('predictive_nav_bringup'), 'lib',
        'predictive_nav_bringup')
    for path in (os.path.join(lib, 'run_stage4f_bench.py'),
                 os.path.join(share, 'scripts', 'run_stage4f_bench.py'),
                 os.path.join(os.path.dirname(os.path.abspath(__file__)), '..',
                              'run_stage4f_bench.py')):
        if os.path.exists(path):
            spec = importlib.util.spec_from_file_location('s4f_bench', path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    raise SystemExit('cannot locate run_stage4f_bench.py')


BENCH = _load_bench()

#: Route, obstacle release rule and predictive policy are the Stage-4F ones,
#: imported rather than restated so they cannot drift.
ROUTE = BENCH.ROUTE
POLICY = BENCH.POLICY
SPAWN_DIST = BENCH.SPAWN_DIST
ARRIVAL_OFFSET = BENCH.ARRIVAL_OFFSET
T_ROBOT_TO_CONFLICT = BENCH.T_ROBOT_TO_CONFLICT

#: Ground-truth class of each actor body, for reporting only. It is the same
#: mapping scripts/stage5b/actors.py defines; it is restated here because this
#: file is installed without that module on its path, and a mismatch would be
#: caught by STAGE5D_README.md's per-class breakdown rather than silently used.
ACTOR_GT_CLASS = {
    'actor_pedestrian': 'PEDESTRIAN',
    'actor_bike': 'BIKE',
    'actor_vehicle_compact': 'VEHICLE',
    'actor_planter': 'UNKNOWN',
}


def _model(actor):
    share = get_package_share_directory('predictive_nav_bringup')
    return os.path.join(share, 'models', 'stage5b', f'{actor}.sdf')


def crossing(name, actor, speed=0.50, from_south=True):
    """Build a perpendicular crossing condition for `actor`, yawed along travel.

    The Stage-4F release rule: the obstacle spawns SPAWN_DIST from the conflict
    point and is released so that it arrives ARRIVAL_OFFSET seconds ahead of
    the robot, whatever its speed. Speed is therefore not a separate
    experiment, and the geometry is identical for every actor -- only the BODY
    changes between conditions, which is the point.
    """
    heading = math.pi / 2 if from_south else -math.pi / 2
    sign = -1.0 if from_south else 1.0
    return {
        'name': name, 'scenario': 'perpendicular', **ROUTE,
        'cross_x': '0.0', 'cross_y': '0.0',
        'obstacle_start_x': '0.0',
        'obstacle_start_y': f'{sign * SPAWN_DIST:.4f}',
        'obstacle_speed': f'{speed:.4f}',
        'obstacle_heading': f'{heading:.6f}',
        # Yawed along travel: the aspect that carries the geometric
        # information a 2D slice can carry.
        'obstacle_yaw': f'{heading:.6f}',
        'obstacle_travel': f'{2 * SPAWN_DIST:.2f}',
        'obstacle_trigger_delay':
            f'{T_ROBOT_TO_CONFLICT - ARRIVAL_OFFSET - SPAWN_DIST / speed:.3f}',
        'spawn_obstacle': 'True',
        'obstacle_model': _model(actor),
        # The Stage-5B actors are prisms over z in [0, 0.45] in their own
        # frame, so they sit on the floor at z = 0.
        'obstacle_spawn_z': '0.0',
        'actor': actor,
    }


def longitudinal(name, actor, speed=0.50, lane_y=0.25):
    """Build a head-on longitudinal condition down the route, offset `lane_y`.

    The Stage-4F `headon` geometry: the obstacle drives -x down a lane offset
    0.25 m from the route, so it passes through the robot's corridor but a
    ~0.45 m lateral shift clears it -- the conflict is real without being
    geometrically unavoidable. Released at t = 0.
    """
    return {
        'name': name, 'scenario': 'headon', **ROUTE,
        'cross_x': '-0.40', 'cross_y': '0.0',
        'obstacle_start_x': '3.0', 'obstacle_start_y': f'{lane_y:.2f}',
        'obstacle_speed': f'{speed:.4f}',
        'obstacle_heading': f'{math.pi:.6f}',
        'obstacle_yaw': f'{math.pi:.6f}',
        'obstacle_travel': '6.0', 'obstacle_trigger_delay': '0.0',
        'spawn_obstacle': 'True',
        'obstacle_model': _model(actor),
        'obstacle_spawn_z': '0.0',
        'actor': actor,
    }


#: The six conditions brief section 13 asks for. A multi-obstacle condition is
#: NOT included and its absence is reported rather than worked around: the
#: shared launch spawns exactly one scripted obstacle, and adding a second
#: would change the launch for every earlier stage.
CONDITIONS = {
    'ped_cross': crossing('ped_cross', 'actor_pedestrian'),
    'ped_longitudinal': longitudinal('ped_longitudinal', 'actor_pedestrian'),
    'bike_cross': crossing('bike_cross', 'actor_bike'),
    'veh_cross': crossing('veh_cross', 'actor_vehicle_compact'),
    'veh_longitudinal': longitudinal('veh_longitudinal', 'actor_vehicle_compact'),
    'unknown_cross': crossing('unknown_cross', 'actor_planter'),
}

#: The two arms. The ONLY difference is the tracker params file, and those two
#: files differ in EXACTLY ONE LINE, `reachability_class_aware` (verified by
#: diff in STAGE5D_README.md). `params_file` is the UNMODIFIED Stage-4F file in
#: both arms -- hence MPPI, every critic weight, the planner and the costmap
#: geometry are shared -- and `layer_prediction_mode: reachability` is applied
#: at runtime, identically, in both.
ARMS = {
    'global': 'stage5d_global_tracker.yaml',
    'classaware': 'stage5d_classaware_tracker.yaml',
}


def main():
    """Run the campaign."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out')
    ap.add_argument('--conditions', default=','.join(CONDITIONS))
    ap.add_argument('--arms', default='global,classaware')
    ap.add_argument('--trials', type=int, default=3)
    ap.add_argument('--start-index', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--nav-timeout', type=float, default=60.0)
    ap.add_argument('--domain-base', type=int, default=40)
    ap.add_argument('--list', action='store_true')
    args = ap.parse_args()

    if args.list:
        for name, c in CONDITIONS.items():
            print(f"{name:18s} actor={c['actor']:22s} "
                  f"gt={ACTOR_GT_CLASS[c['actor']]:10s} "
                  f"scenario={c['scenario']:14s} speed={c['obstacle_speed']} "
                  f"start=({c['obstacle_start_x']},{c['obstacle_start_y']}) "
                  f"yaw={float(c['obstacle_yaw']) * 180 / math.pi:6.1f}deg "
                  f"release={c['obstacle_trigger_delay']}s")
        return
    if not args.out:
        raise SystemExit('--out is required')

    tracking = get_package_share_directory('predictive_nav_tracking')
    bringup = get_package_share_directory('predictive_nav_bringup')
    # ONE params_file for BOTH arms, and it is the UNMODIFIED Stage-4F file,
    # so MPPI, every critic weight, the planner, the costmap geometry and the
    # resolution cannot differ between arms even by accident. The costmap's
    # prediction_mode is switched to `reachability` at RUNTIME instead, in
    # both arms identically, through the mechanism the layer documents for it.
    params_file = os.path.join(bringup, 'config', 'nav2_stage4f_params.yaml')
    arm_params = {}
    for arm, yaml_name in ARMS.items():
        path = os.path.join(tracking, 'config', yaml_name)
        if not os.path.exists(path):
            raise SystemExit(f'missing tracker params for arm {arm}: {path}')
        arm_params[arm] = path
    if not os.path.exists(params_file):
        raise SystemExit(f'missing nav2 params: {params_file}')

    out_dir = os.path.abspath(args.out)
    log_dir = os.path.join(out_dir, 'logs')
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)

    conds = [c.strip() for c in args.conditions.split(',') if c.strip()]
    arms = [a.strip() for a in args.arms.split(',') if a.strip()]
    for c in conds:
        if c not in CONDITIONS:
            raise SystemExit(f'unknown condition {c}; try --list')
    for a in arms:
        if a not in ARMS:
            raise SystemExit(f'unknown arm {a}')

    # Arms are INTERLEAVED within each trial index, so machine drift or
    # thermal throttling over a 40-minute campaign cannot bias one arm.
    plan = []
    for cname in conds:
        for i in range(args.start_index, args.start_index + args.trials):
            for arm in arms:
                plan.append((cname, arm, i))

    print(f'Stage-5D navigation campaign: {len(plan)} trials '
          f'({len(conds)} conditions x {len(arms)} arms x {args.trials}) '
          f'-> {out_dir}', flush=True)
    BENCH.kill_stragglers()

    t0 = time.time()
    for n, (cname, arm, i) in enumerate(plan, 1):
        cond = dict(CONDITIONS[cname])
        actor = cond.pop('actor')
        extra = {
            'tracker_params': arm_params[arm],
            'params_file': params_file,
            # Identical in both arms: the comparison is between two
            # reachability CONFIGURATIONS, not between reachability and CV.
            'layer_prediction_mode': 'reachability',
            'obstacle_model': cond.pop('obstacle_model'),
            'obstacle_spawn_z': cond.pop('obstacle_spawn_z'),
        }
        tid = f'{cname}_{arm}_{i:02d}'
        print(f'[{n}/{len(plan)}] {tid} actor={actor} '
              f'gt={ACTOR_GT_CLASS[actor]} (elapsed {time.time() - t0:.0f}s)',
              flush=True)
        # mode=predictive: the predicted_obstacle_layer is ENABLED in both
        # arms. The reactive arm is a Stage-4F control and is not part of this
        # comparison.
        BENCH.run_trial(
            cond, 'predictive', tid, out_dir, log_dir, extra,
            args.timeout, args.domain_base + (n % 20), args.nav_timeout)
    print(f'done in {time.time() - t0:.0f}s', flush=True)


if __name__ == '__main__':
    main()
