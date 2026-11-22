#!/usr/bin/env python3
"""Stage-5E FINAL three-arm navigation campaign: CV vs global vs class-aware.

ANALYSIS ONLY. Nothing in this file or in validation/stage5e/ is installed,
referenced by a launch default, or reachable from a production configuration.
The production default remains `prediction_mode: cv_covariance` with
`production_tracker_params.yaml`, both untouched -- Stage-5E section 14.

THE THREE ARMS, AND THE ONE THING THAT VARIES
---------------------------------------------
Exactly one thing varies: WHICH PREDICTOR THE COSTMAP LAYER CONSUMES, and the
tracker publication flag that necessarily accompanies it.

    arm CV          tracker stage5e_cv_tracker.yaml        layer cv_covariance
                    (= production_tracker_params.yaml + profile_scans, i.e.
                     reachability_enabled FALSE -- the shipped production
                     configuration)
    arm GLOBAL      tracker stage5d_global_tracker.yaml    layer reachability
                    (reachability_enabled TRUE, class_aware FALSE -- the
                     Stage-5C/5C.1 qualified global reachable set)
    arm CLASSAWARE  tracker stage5d_classaware_tracker.yaml layer reachability
                    (reachability_enabled TRUE, class_aware TRUE -- the
                     Stage-5D qualified runtime classifier + the shipped safe
                     ambiguity/fallback policy)

Verified by comment-stripped diff (STAGE5E_README.md section 2):

    CV      vs GLOBAL     : ONE line, reachability_enabled false -> true
    GLOBAL  vs CLASSAWARE : ONE line, reachability_class_aware false -> true

Every other tracker parameter -- Stage-4G1 static rejection, Stage-4B
clustering, Stage-4G3 deblending, association, lifecycle, the Stage-4C Kalman
filter and CV predictor, the Stage-5B classifier, Stage-5B.1 compound
clustering, the Stage-5D capability-profile table and the ambiguity policy --
is IDENTICAL in all three files. The classifier therefore executes identically
in all three arms; in arm CV its output simply has no consumer.

NO ORACLE ARM. Stage-5E section 2: the oracle is an evaluation-only
upper-bound measurement from Stage-5D/5D.1, not a deployable configuration, so
it does not belong in an architecture comparison. It is not run here.

WINDOW: 0.55 m in ALL THREE ARMS -- the production
`policy_max_influence_radius`, which is also POLICY's value in
run_stage4f_bench.py. It is passed explicitly anyway so that each trial records
it and the analyser can cross-check it per trial. The 3.00 m analysis window is
a Stage-5D.1 diagnostic and is not an arm here.

WHAT IS HELD FIXED
------------------
Everything else, by construction, because every arm goes through the SAME
launch invocation with the SAME values and the SAME UNMODIFIED Stage-4F
params_file: map, world, start, goal, route, obstacle body, obstacle
trajectory, release time, sensor model, AMCL, Nav2, MPPI and every critic
weight, the costmap geometry and resolution, the prediction horizon, the
Stage-5C.1 contract and the frozen Stage-4F predictive policy. MPPI IS NOT
RETUNED PER ARM. No YAML under src/ is written.

SCENARIOS ARE IMPORTED, NOT INVENTED
------------------------------------
    * the six class conditions are IMPORTED from the installed Stage-5D runner
      (`CONDITIONS`), so the geometry cannot drift between Stage-5D, Stage-5D.1
      and Stage-5E;
    * `ped_oblique` and `ped_close_pass` are the VALIDATED Stage-4F `oblique`
      and `noconflict` condition dictionaries, taken from
      run_stage4f_bench.CONDITIONS and given a Stage-5B actor body plus a yaw
      along travel. No geometry number is restated here;
    * `ped_corner` is the VALIDATED Stage-4G8 `g8_corner` definition -- target
      start (-0.25, 3.4), velocity (0, -0.68), world `stage4g7_c30` -- read out
      of scripts/stage4g2/scenarios.json at run time rather than copied, and
      expressed in the Stage-4F one-way scripted-obstacle form.

NOT RUN, AND WHY (reported, not worked around)
----------------------------------------------
    * REVERSAL / ACCELERATION-CHANGE (brief condition 9): the shared trial
      runner drives the scripted obstacle ONE-WAY at constant velocity
      (stage4f_trial.py:659, "scripted obstacle (one-way, no reversal)").
      Producing a reversal would mean changing the scorer every earlier stage's
      numbers were produced with.
    * MULTI-OBSTACLE (brief condition 10, explicitly conditional on harness
      support): the shared launch spawns exactly ONE scripted obstacle. Adding
      a second would change the launch for every earlier stage.
      Stage-5D already declined this condition for the same reason.

    validation/stage5e/run_nav5.py --out DIR [--trials 5] [--arms cv,global,classaware]
    validation/stage5e/run_nav5.py --list
"""
import argparse
import importlib.util
import json
import math
import os
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _find(fname, *subdirs):
    """The installed copy first, then the source tree."""
    lib = os.path.join(
        get_package_prefix('predictive_nav_bringup'), 'lib', 'predictive_nav_bringup')
    cands = [os.path.join(lib, fname)]
    for sub in subdirs:
        cands.append(os.path.join(HERE, '..', '..', 'src', 'predictive_nav_bringup',
                                  'scripts', sub, fname))
    cands.append(os.path.join(HERE, '..', '..', 'src', 'predictive_nav_bringup',
                              'scripts', fname))
    for p in cands:
        if os.path.exists(p):
            return p
    raise SystemExit(f'cannot locate {fname}')


#: The Stage-5D runner, imported for CONDITIONS, ACTOR_GT_CLASS and the actor
#: model resolver, so the six class conditions are literally the same objects
#: Stage-5D and Stage-5D.1 used.
S5D = _load('s5d_run_nav', _find('run_nav.py', 'stage5d'))
BENCH = S5D.BENCH
ACTOR_GT_CLASS = S5D.ACTOR_GT_CLASS

#: The VALIDATED Stage-4G8 scenario table, read rather than restated.
SCENARIOS_JSON = os.path.join(
    HERE, '..', '..', 'src', 'predictive_nav_bringup', 'scripts', 'stage4g2',
    'scenarios.json')


def _actor_fields(actor):
    """The three launch arguments that put a Stage-5B actor body on the stage."""
    return {
        'obstacle_model': S5D._model(actor),
        # The Stage-5B actors are prisms over z in [0, 0.45] in their own
        # frame, so they sit on the floor at z = 0.
        'obstacle_spawn_z': '0.0',
        'actor': actor,
    }


def _from_stage4f(cond_name, actor):
    """A VALIDATED Stage-4F condition, given a Stage-5B body and a travel yaw.

    The geometry -- start, speed, heading, travel, release time, conflict point
    -- is whatever run_stage4f_bench.CONDITIONS already says it is; nothing is
    restated here. Only the BODY is substituted, and the body is yawed ALONG
    its direction of travel, which is both what the real object does and what
    makes it classifiable from one horizontal LiDAR slice (the Stage-5B
    aspect finding).
    """
    cond = dict(BENCH.CONDITIONS[cond_name])
    cond['name'] = f'ped_{cond_name}'
    cond['obstacle_yaw'] = cond['obstacle_heading']
    cond.update(_actor_fields(actor))
    return cond


def _g8_corner(actor):
    """The VALIDATED Stage-4G8 `g8_corner` blind-corner scenario, Stage-4F form.

    Read out of scripts/stage4g2/scenarios.json so the numbers cannot drift:
    one target starting at (-0.25, 3.4) with velocity (0, -0.68) in the
    Stage-4G7 `stage4g7_c30` world, whose 0.30 x 0.15 m stub at (-0.80, 0.85)
    hides it until it is already near the route. Released at goal acceptance,
    which is the instant the Stage-4G2 worker released it, so the timing is
    g8_corner's and not a new one.

    The target descends at x = -0.25. The occluder's east face is at
    x = -0.65 and the pedestrian body is 0.28 m across its hips, so the body
    clears the stub by 0.26 m -- the Stage-4G7 path_clear() requirement, which
    `g8_corner` already satisfied with a wider body.
    """
    with open(SCENARIOS_JSON) as handle:
        defs = json.load(handle)
    g8 = defs['g8_corner']
    if len(g8['objects']) != 1:
        raise SystemExit('g8_corner is no longer a single-object scenario')
    obj = g8['objects'][0]
    sx, sy = obj['start']
    vx, vy = obj['velocity']
    speed = math.hypot(vx, vy)
    heading = math.atan2(vy, vx)
    world = g8['world']
    # Travel far enough to cross the route and leave it: the target starts
    # |sy| above y = 0 and the Stage-4F harness drives it one-way.
    travel = abs(sy) + 1.1
    cond = {
        'name': 'ped_corner', 'scenario': 'perpendicular', **BENCH.ROUTE,
        # The conflict point is where the target's track meets the route.
        'cross_x': f'{sx:.4f}', 'cross_y': '0.0',
        'obstacle_start_x': f'{sx:.4f}', 'obstacle_start_y': f'{sy:.4f}',
        'obstacle_speed': f'{speed:.4f}',
        'obstacle_heading': f'{heading:.6f}',
        'obstacle_yaw': f'{heading:.6f}',
        'obstacle_travel': f'{travel:.2f}',
        # g8_corner released its target at trial start, i.e. at goal
        # acceptance, which is `0.0` in the Stage-4F harness.
        'obstacle_trigger_delay': '0.0',
        'spawn_obstacle': 'True',
        **_actor_fields(actor),
        # The world and its occupancy map are always taken as a PAIR from the
        # one Stage-4G7 generator, so they cannot describe different rooms.
        'world_name': world,
    }
    return cond


#: The nine conditions. Six are the Stage-5D class conditions, imported
#: unchanged; three are validated Stage-4F / Stage-4G8 definitions given a
#: Stage-5B body. Brief conditions 9 (reversal) and 10 (multi-obstacle) are
#: NOT here and the module docstring says why.
CONDITIONS = dict(S5D.CONDITIONS)
CONDITIONS['ped_oblique'] = _from_stage4f('oblique', 'actor_pedestrian')
CONDITIONS['ped_close_pass'] = _from_stage4f('noconflict', 'actor_pedestrian')
CONDITIONS['ped_corner'] = _g8_corner('actor_pedestrian')

#: arm -> (tracker params file, layer prediction mode). The tracker file is the
#: ONLY thing that differs between CV and GLOBAL, and between GLOBAL and
#: CLASSAWARE, apart from the layer mode CV necessarily implies.
ARMS = {
    'cv': ('stage5e_cv_tracker.yaml', 'cv_covariance', 'validation'),
    'global': ('stage5d_global_tracker.yaml', 'reachability', 'share'),
    'classaware': ('stage5d_classaware_tracker.yaml', 'reachability', 'share'),
}

#: The production influence window, identical in every arm.
WINDOW_M = '0.55'


def tracker_path(arm):
    """Resolve an arm's tracker params file."""
    name, _mode, where = ARMS[arm]
    if where == 'validation':
        return os.path.join(HERE, name)
    return os.path.join(
        get_package_share_directory('predictive_nav_tracking'), 'config', name)


def main():
    """Run the three-arm campaign."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out')
    ap.add_argument('--conditions', default=','.join(CONDITIONS))
    ap.add_argument('--arms', default='cv,global,classaware')
    ap.add_argument('--trials', type=int, default=5)
    ap.add_argument('--start-index', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--nav-timeout', type=float, default=60.0)
    ap.add_argument('--domain-base', type=int, default=80)
    ap.add_argument('--list', action='store_true')
    args = ap.parse_args()

    if args.list:
        for name, c in CONDITIONS.items():
            gt = ACTOR_GT_CLASS[c['actor']]
            print(f"{name:18s} actor={c['actor']:22s} gt={gt:10s} "
                  f"scenario={c['scenario']:14s} "
                  f"world={c.get('world_name', 'stage4f_benchmark'):20s} "
                  f"speed={c['obstacle_speed']} "
                  f"start=({c['obstacle_start_x']},{c['obstacle_start_y']}) "
                  f"yaw={float(c['obstacle_yaw']) * 180 / math.pi:7.1f}deg "
                  f"release={c['obstacle_trigger_delay']}s")
        for arm, (yaml_name, mode, _w) in ARMS.items():
            print(f'arm {arm:11s} tracker={yaml_name:34s} layer_mode={mode}')
        return
    if not os.environ.get('STAGE5E_OUT') and not args.out:
        raise SystemExit('--out is required')

    bringup = get_package_share_directory('predictive_nav_bringup')
    # ONE params_file for ALL THREE arms, and it is the UNMODIFIED Stage-4F
    # file, so MPPI, every critic weight, the planner, the costmap geometry and
    # the resolution cannot differ between arms even by accident.
    params_file = os.path.join(bringup, 'config', 'nav2_stage4f_params.yaml')
    if not os.path.exists(params_file):
        raise SystemExit(f'missing nav2 params: {params_file}')
    for arm in ARMS:
        path = tracker_path(arm)
        if not os.path.exists(path):
            raise SystemExit(f'missing tracker params for arm {arm}: {path}')

    out_dir = os.path.abspath(args.out or os.environ['STAGE5E_OUT'])
    log_dir = os.path.join(out_dir, 'logs')
    os.makedirs(log_dir, exist_ok=True)

    conds = [c.strip() for c in args.conditions.split(',') if c.strip()]
    arms = [a.strip().lower() for a in args.arms.split(',') if a.strip()]
    for c in conds:
        if c not in CONDITIONS:
            raise SystemExit(f'unknown condition {c}; try --list')
    for a in arms:
        if a not in ARMS:
            raise SystemExit(f'unknown arm {a}; one of {sorted(ARMS)}')

    # Arms are INTERLEAVED within each trial index, so machine drift or thermal
    # throttling over a multi-hour campaign cannot bias one arm.
    plan = []
    for cname in conds:
        for i in range(args.start_index, args.start_index + args.trials):
            for arm in arms:
                plan.append((cname, arm, i))

    print(f'Stage-5E three-arm campaign: {len(plan)} trials '
          f'({len(conds)} conditions x {len(arms)} arms x {args.trials}) '
          f'-> {out_dir}', flush=True)
    BENCH.kill_stragglers()

    t0 = time.time()
    for n, (cname, arm, i) in enumerate(plan, 1):
        cond = dict(CONDITIONS[cname])
        actor = cond.pop('actor')
        gt = ACTOR_GT_CLASS[actor]
        _yaml, mode, _where = ARMS[arm]
        extra = {
            'tracker_params': tracker_path(arm),
            'params_file': params_file,
            # The ONE costmap-side difference, and it is the predictor the
            # layer consumes -- the question Stage-5E exists to answer.
            'layer_prediction_mode': mode,
            # Identical in every arm; passed explicitly so each trial records
            # it and the analyser can cross-check it.
            'policy_max_influence_radius': WINDOW_M,
            'obstacle_model': cond.pop('obstacle_model'),
            'obstacle_spawn_z': cond.pop('obstacle_spawn_z'),
        }
        world = cond.pop('world_name', None)
        if world:
            # World and map as a PAIR from the one Stage-4G7 generator.
            extra['world'] = os.path.join(bringup, 'worlds', f'{world}.sdf.xacro')
            extra['map'] = os.path.join(bringup, 'maps', f'{world}.yaml')
            for p in (extra['world'], extra['map']):
                if not os.path.exists(p):
                    raise SystemExit(f'missing world/map asset: {p}')
        tid = f'{cname}_{arm}_{i:02d}'
        print(f'[{n}/{len(plan)}] {tid} actor={actor} gt={gt} '
              f'arm={arm} layer={mode} world={world or "stage4f_benchmark"} '
              f'(elapsed {time.time() - t0:.0f}s)', flush=True)
        BENCH.run_trial(
            cond, 'predictive', tid, out_dir, log_dir, extra,
            args.timeout, args.domain_base + (n % 20), args.nav_timeout)
    print(f'done in {time.time() - t0:.0f}s', flush=True)


if __name__ == '__main__':
    main()
