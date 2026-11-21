#!/usr/bin/env python3
"""Stage-5D.1 FOUR-ARM navigation campaign: class source x policy window.

ANALYSIS ONLY. Nothing in this file is installed, referenced by a launch
default, or reachable from a production configuration.

THE FOUR ARMS, AND THE TWO THINGS THAT VARY
-------------------------------------------
Exactly two factors vary, crossed:

    class source  : RUNTIME (the shipped Stage-5B classifier + the shipped
                     Stage-5D ambiguity/fallback policy)
                 vs ORACLE  (the capability premise a perfect classifier would
                     be entitled to, supplied as a CONFIGURED GLOBAL constant,
                     see below)
    policy window : max_influence_radius 0.55 m (PRODUCTION)
                 vs max_influence_radius 3.00 m (ANALYSIS ONLY -- the layer's
                     own declareParameter default, and the half-extent of the
                     6x6 m local costmap, so the largest window this costmap
                     can represent at all)

    A  runtime / 0.55     B  oracle / 0.55
    C  runtime / 3.00     D  oracle / 3.00

HOW THE ORACLE ARM AVOIDS A GROUND-TRUTH LEAK
---------------------------------------------
The oracle arms load an oracle_*_tracker.yaml, which is
stage5d_global_tracker.yaml with ONLY `max_speed` and `max_acceleration`
changed to the true class's Stage-5D capability entry. `class_aware` stays
FALSE, so the node takes the bit-identical Stage-5C.1 GLOBAL path and has NO
class input of any kind: it reads two configured constants, exactly as it has
since Stage-4G4.

Which file to load is decided HERE, outside the robot, from the scenario
definition -- this runner already knows which actor body it is spawning,
because it spawns it. No label, no classifier output and no ground-truth field
crosses into the tracker, the costmap layer, the planner or the controller.

The oracle arms are therefore an UPPER-BOUND MEASUREMENT, not a deployable
configuration: a real robot does not know the class of an obstacle it has not
classified.

WHAT IS HELD FIXED
------------------
Everything else, by construction, because every arm goes through the same
launch invocation with the same values and the SAME UNMODIFIED Stage-4F
params_file: map, world, start, goal, route, obstacle body, obstacle
trajectory, release time, sensor model, AMCL, the Stage-4C Kalman filter, the
Stage-5B classifier, compound clustering, the reachability mathematics, the
Stage-5C.1 contract, the prediction horizon, MPPI and every critic weight,
Nav2, costmap geometry and resolution, the capability profile table and the
ambiguity policy. MPPI IS NOT RETUNED PER ARM.

The six conditions are IMPORTED from validation's Stage-5D runner rather than
restated, so the scenario geometry cannot drift between the two stages.

    validation/stage5d1/run_nav4.py --out DIR [--trials 3] [--arms a,b,c,d]
    validation/stage5d1/run_nav4.py --list
"""
import argparse
import importlib.util
import os
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _find(fname):
    """The installed copy first, then the source tree."""
    lib = os.path.join(
        get_package_prefix('predictive_nav_bringup'), 'lib', 'predictive_nav_bringup')
    for p in (os.path.join(lib, fname),
              os.path.join(HERE, '..', '..', 'src', 'predictive_nav_bringup',
                           'scripts', 'stage5d', fname),
              os.path.join(HERE, '..', '..', 'src', 'predictive_nav_bringup',
                           'scripts', fname)):
        if os.path.exists(p):
            return p
    raise SystemExit(f'cannot locate {fname}')


#: The Stage-5D runner, imported for CONDITIONS and ACTOR_GT_CLASS so the
#: scenario geometry is literally the same objects Stage-5D used.
S5D = _load('s5d_run_nav', _find('run_nav.py'))
BENCH = S5D.BENCH
CONDITIONS = S5D.CONDITIONS
ACTOR_GT_CLASS = S5D.ACTOR_GT_CLASS

#: True class -> the oracle capability premise that class is entitled to.
#: UNKNOWN maps to the fallback envelope because an UNKNOWN object's true
#: capability is by definition not known, and the envelope is the only honest
#: answer -- the same `truthProfile` rule the offline audit uses.
ORACLE_YAML = {
    'PEDESTRIAN': 'oracle_ped_tracker.yaml',
    'BIKE': 'oracle_bike_tracker.yaml',
    'VEHICLE': 'oracle_fallback_tracker.yaml',
    'UNKNOWN': 'oracle_fallback_tracker.yaml',
}

#: arm -> (class source, max_influence_radius)
ARMS = {
    'a': ('runtime', '0.55'),
    'b': ('oracle', '0.55'),
    'c': ('runtime', '3.00'),
    'd': ('oracle', '3.00'),
}


def main():
    """Run the four-arm campaign."""
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out')
    ap.add_argument('--conditions', default=','.join(CONDITIONS))
    ap.add_argument('--arms', default='a,b,c,d')
    ap.add_argument('--trials', type=int, default=3)
    ap.add_argument('--start-index', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--nav-timeout', type=float, default=60.0)
    ap.add_argument('--domain-base', type=int, default=60)
    ap.add_argument('--list', action='store_true')
    args = ap.parse_args()

    if args.list:
        for name, c in CONDITIONS.items():
            gt = ACTOR_GT_CLASS[c['actor']]
            print(f"{name:18s} actor={c['actor']:22s} gt={gt:10s} "
                  f"oracle_yaml={ORACLE_YAML[gt]}")
        for a, (src, win) in ARMS.items():
            print(f"arm {a.upper()}: class_source={src:8s} max_influence_radius={win}")
        return
    if not args.out:
        raise SystemExit('--out is required')

    tracking = get_package_share_directory('predictive_nav_tracking')
    bringup = get_package_share_directory('predictive_nav_bringup')
    # ONE params_file for ALL FOUR arms, and it is the UNMODIFIED Stage-4F
    # file, so MPPI, every critic weight, the planner, the costmap geometry and
    # the resolution cannot differ between arms even by accident. The window is
    # varied through the layer's OWN documented runtime parameter instead, and
    # this file is NOT written to.
    params_file = os.path.join(bringup, 'config', 'nav2_stage4f_params.yaml')
    if not os.path.exists(params_file):
        raise SystemExit(f'missing nav2 params: {params_file}')
    runtime_yaml = os.path.join(
        tracking, 'config', 'stage5d_classaware_tracker.yaml')
    if not os.path.exists(runtime_yaml):
        raise SystemExit(f'missing runtime tracker params: {runtime_yaml}')
    for y in set(ORACLE_YAML.values()):
        if not os.path.exists(os.path.join(HERE, y)):
            raise SystemExit(f'missing oracle tracker params: {y}')

    out_dir = os.path.abspath(args.out)
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

    print(f'Stage-5D.1 four-arm campaign: {len(plan)} trials '
          f'({len(conds)} conditions x {len(arms)} arms x {args.trials}) '
          f'-> {out_dir}', flush=True)
    BENCH.kill_stragglers()

    t0 = time.time()
    for n, (cname, arm, i) in enumerate(plan, 1):
        cond = dict(CONDITIONS[cname])
        actor = cond.pop('actor')
        gt = ACTOR_GT_CLASS[actor]
        source, window = ARMS[arm]
        tracker = (runtime_yaml if source == 'runtime'
                   else os.path.join(HERE, ORACLE_YAML[gt]))
        extra = {
            'tracker_params': tracker,
            'params_file': params_file,
            # Identical in every arm: this is a comparison between reachability
            # CONFIGURATIONS, not between reachability and CV.
            'layer_prediction_mode': 'reachability',
            # The ONE policy value that differs between the window arms. Sent
            # as a launch argument and applied by the layer's own documented
            # runtime parameter path; no YAML is modified.
            'policy_max_influence_radius': window,
            'obstacle_model': cond.pop('obstacle_model'),
            'obstacle_spawn_z': cond.pop('obstacle_spawn_z'),
        }
        tid = f'{cname}_{arm}_{i:02d}'
        print(f'[{n}/{len(plan)}] {tid} actor={actor} gt={gt} '
              f'source={source} window={window} '
              f'(elapsed {time.time() - t0:.0f}s)', flush=True)
        BENCH.run_trial(
            cond, 'predictive', tid, out_dir, log_dir, extra,
            args.timeout, args.domain_base + (n % 20), args.nav_timeout)
    print(f'done in {time.time() - t0:.0f}s', flush=True)


if __name__ == '__main__':
    main()
