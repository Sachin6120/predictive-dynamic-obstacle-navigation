#!/usr/bin/env python3
"""Stage-4F benchmark driver: run the scenario matrix, both arms, one command.

Every trial is a full simulator restart on its own ROS_DOMAIN_ID, so no trial
can inherit another's state. Arms are interleaved (R,P,R,P,...) so machine
drift cannot bias one of them.

    run_stage4f_bench.py --out DIR --conditions perp_050,oblique,headon,noconflict
    run_stage4f_bench.py --out DIR --conditions perp_025,perp_050,perp_075
    run_stage4f_bench.py --out DIR --conditions nominal --modes reactive --trials 3
    run_stage4f_bench.py --list

Obstacle release rule (the single systematic rule behind the speed study):
every conflict scenario spawns the obstacle SPAWN_DIST = 3.0 m from the
conflict point, and releases it at

    t_release = t_robot_to_conflict - ARRIVAL_OFFSET - SPAWN_DIST / v

so that the obstacle reaches the conflict point a fixed ARRIVAL_OFFSET ahead of
the robot whatever the obstacle's speed. At 0.25 m/s that release time is negative, i.e. the obstacle
is already under way when the goal is issued; the trial runner handles that as
a pre-roll. This keeps spawn geometry fixed and makes speed the only variable,
rather than turning each speed into a different experiment.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import time

# Measured obstacle-free time for the robot to reach the conflict point:
# 7.05 / 7.05 / 7.10 s over 3 `--conditions nominal` runs (sd 0.02 s). Used
# only to schedule the scripted obstacle; it never enters the navigation stack.
T_ROBOT_TO_CONFLICT = 7.07
SPAWN_DIST = 3.0

# The obstacle is timed to reach the conflict point ARRIVAL_OFFSET seconds
# BEFORE the robot would, rather than at the same instant.
#
# Why not simultaneous: with exact simultaneous arrival a 0.49 m/s robot cannot
# geometrically avoid a 0.5 m/s crosser at all. A perpendicular crosser is not
# ON the robot's path until the last moment -- the two bodies conflict only
# inside 0.42 m -- so ordinary reactive sensing gets no usable warning and the
# reactive arm collides every time (measured: -0.20 m penetration). Benchmarking
# against a baseline that is doomed by construction proves nothing.
#
# For a robot that does NOT react, closest approach is
#     sep_min = v_robot * v_obs * offset / sqrt(v_robot^2 + v_obs^2)
# giving 0.22 / 0.35 / 0.41 m at 0.25 / 0.50 / 0.75 m/s for offset = 1.0 s.
# All three are below the 0.42 m contact threshold, so the conflict is real and
# the robot is genuinely forced to act at every speed -- but a modest brake or a
# ~0.2 m lateral shift clears it, so a late-reacting reactive baseline can still
# succeed. That is the fair comparison Stage-4F section 12 asks for.
ARRIVAL_OFFSET = 1.0

# Route and arena are shared by every condition.
ROUTE = {
    'start_x': '-3.0', 'start_y': '0.0', 'start_yaw': '0.0',
    'goal_x': '3.0', 'goal_y': '0.0', 'goal_yaw': '0.0',
}


def crossing(name, speed, heading_deg, spawn_angle_deg, travel=5.0):
    """A conflict scenario whose obstacle starts SPAWN_DIST from the origin at
    `spawn_angle_deg` and drives along `heading_deg` through it."""
    sa = math.radians(spawn_angle_deg)
    return {
        'name': name,
        'scenario': 'perpendicular' if heading_deg == 90 else 'oblique',
        **ROUTE,
        'cross_x': '0.0', 'cross_y': '0.0',
        'obstacle_start_x': f'{SPAWN_DIST * math.cos(sa):.4f}',
        'obstacle_start_y': f'{SPAWN_DIST * math.sin(sa):.4f}',
        'obstacle_speed': f'{speed:.4f}',
        'obstacle_heading': f'{math.radians(heading_deg):.6f}',
        'obstacle_travel': f'{travel:.2f}',
        'obstacle_trigger_delay':
            f'{T_ROBOT_TO_CONFLICT - ARRIVAL_OFFSET - SPAWN_DIST / speed:.3f}',
        'spawn_obstacle': 'True',
    }


CONDITIONS = {
    # --- A. perpendicular crossing, three speeds (the speed study) ---
    'perp_025': crossing('perp_025', 0.25, 90, -90),
    'perp_050': crossing('perp_050', 0.50, 90, -90),
    'perp_075': crossing('perp_075', 0.75, 90, -90),

    # --- B. oblique crossing: obstacle runs the 45 deg diagonal through the
    # conflict point, so the robot/obstacle velocity angle is 45 deg, not 90.
    # Checks the result is not an artefact of axis-aligned motion.
    'oblique': crossing('oblique', 0.50, 45, -135, travel=4.5),

    # --- C. head-on / opposing. The obstacle drives -x down a lane offset
    # +0.25 m from the route, so it passes through the robot's corridor but the
    # geometry does not force a collision: a ~0.45 m lateral shift clears it,
    # and there is 2.3 m of zero-cost space on either side to do it in.
    # They close at (0.46 + 0.50) m/s from 6 m apart, meeting near x = -0.4.
    'headon': {
        'name': 'headon', 'scenario': 'headon', **ROUTE,
        'cross_x': '-0.40', 'cross_y': '0.0',
        'obstacle_start_x': '3.0', 'obstacle_start_y': '0.25',
        'obstacle_speed': '0.50', 'obstacle_heading': f'{math.pi:.6f}',
        'obstacle_travel': '6.0', 'obstacle_trigger_delay': '0.0',
        'spawn_obstacle': 'True',
    },

    # --- D. no-conflict control. Same obstacle, same speed, opposing direction,
    # but in a parallel lane 1.5 m off the route: continuously visible, tracked
    # and predicted right next to the robot, yet its predicted path never
    # intersects the route. Guards against the layer being a generic penalty
    # field rather than a prediction of a specific conflict.
    'noconflict': {
        'name': 'noconflict', 'scenario': 'noconflict', **ROUTE,
        'cross_x': '0.0', 'cross_y': '0.0',
        'obstacle_start_x': '3.0', 'obstacle_start_y': '1.5',
        'obstacle_speed': '0.50', 'obstacle_heading': f'{math.pi:.6f}',
        'obstacle_travel': '6.0', 'obstacle_trigger_delay': '0.0',
        'spawn_obstacle': 'True',
    },

    # --- baseline: no obstacle at all. Defines the nominal trajectory, the
    # reaction thresholds, and T_ROBOT_TO_CONFLICT above.
    'nominal': {
        'name': 'nominal', 'scenario': 'nominal', **ROUTE,
        'cross_x': '0.0', 'cross_y': '0.0',
        'obstacle_start_x': '0.0', 'obstacle_start_y': '-3.0',
        'obstacle_speed': '0.0', 'obstacle_heading': '1.5707963',
        'obstacle_travel': '0.0', 'obstacle_trigger_delay': '0.0',
        'spawn_obstacle': 'False',
    },
}

# Frozen Stage-4F predictive policy -- identical for EVERY primary trial, every
# scenario and every obstacle speed. Applied in both arms (inert when the layer
# is disabled) so the arms differ only in `enabled`.
POLICY = {
    'policy_max_prediction_horizon': '3.0',
    'policy_sigma_level': '1.5',
    'policy_temporal_decay': '0.35',
    'policy_max_cost': '250',
    'policy_min_cost': '0',
    'policy_max_influence_radius': '0.55',
    'nominal_speed': '0.48',
}

PATTERNS = ('gz sim', 'gz-sim', 'ruby.*gz', 'parameter_bridge',
            'robot_state_publisher', 'rviz2', 'lidar_obstacle_tracker',
            'stage4f_trial', 'stage4e_trial', 'component_container',
            'publish_initial_pose', 'ros_gz_bridge', 'ros_gz_sim',
            'stage4f_bringup')


def kill_stragglers(verify_timeout=45.0):
    """Kill and VERIFY. A surviving component_container keeps its Nav2
    lifecycle nodes on the ROS graph, which makes the next trial's bringup fail
    and every goal get rejected -- a failure mode that masquerades as a
    navigation failure."""
    deadline = time.time() + verify_timeout
    while True:
        for pat in PATTERNS:
            subprocess.run(['pkill', '-9', '-f', pat],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2.0)
        alive = [p for p in PATTERNS
                 if subprocess.run(['pgrep', '-f', p], stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL).returncode == 0]
        if not alive or time.time() > deadline:
            if alive:
                print(f'     WARNING: still alive after kill: {alive}', flush=True)
            break
    time.sleep(3.0)


def run_trial(cond, mode, trial_id, out_dir, log_dir, extra, timeout, domain_id,
              nav_timeout):
    out_path = os.path.join(out_dir, f'{trial_id}.json')
    log_path = os.path.join(log_dir, f'{trial_id}.log')
    c = dict(cond)
    c.pop('name')
    spawn = c.pop('spawn_obstacle')

    cmd = ['ros2', 'launch', 'predictive_nav_bringup', 'stage4f_bringup.launch.py',
           'headless:=True', 'use_rviz:=False',
           f'mode:={mode}', f'trial_id:={trial_id}', f'out_path:={out_path}',
           f'spawn_obstacle:={spawn}', f'nav_timeout:={nav_timeout}']
    for k, v in {**c, **POLICY, **extra}.items():
        cmd.append(f'{k}:={v}')

    env = dict(os.environ, ROS_DOMAIN_ID=str(domain_id))
    print(f'  -> {trial_id} [{mode}] domain={domain_id}', flush=True)
    with open(log_path, 'w') as log:
        log.write(f'ROS_DOMAIN_ID={domain_id}\n' + ' '.join(cmd) + '\n\n')
        log.flush()
        proc = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True, env=env)
        try:
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            print('     launch timeout; killing', flush=True)
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
            proc.wait(timeout=30)

    kill_stragglers()

    if not os.path.exists(out_path):
        print('     NO RESULT FILE', flush=True)
        return None
    if 'Failed to bring up all requested nodes' in open(log_path,
                                                        errors='ignore').read():
        print('     INFRASTRUCTURE FAULT: Nav2 bringup failed; trial invalid',
              flush=True)
    with open(out_path) as f:
        r = json.load(f)
    m = r.get('metrics', {})
    print(f"     {m.get('nav_status')} t={m.get('nav_time')} "
          f"clr={m.get('min_clearance')} react={m.get('primary_reaction_time')} "
          f"lat={m.get('max_abs_lateral_offset_approach')} "
          f"resp={m.get('response_class')} pred={m.get('max_pred_cells')}",
          flush=True)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out')
    ap.add_argument('--conditions', default='perp_050')
    ap.add_argument('--modes', default='reactive,predictive')
    ap.add_argument('--trials', type=int, default=10)
    ap.add_argument('--start-index', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--nav-timeout', type=float, default=60.0)
    ap.add_argument('--domain-base', type=int, default=60)
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--tag', default='',
                    help='suffix appended to trial ids, for ablation runs')
    ap.add_argument('--list', action='store_true')
    args = ap.parse_args()

    if args.list:
        for name, c in CONDITIONS.items():
            print(f"{name:12s} scenario={c['scenario']:14s} "
                  f"speed={c['obstacle_speed']:>6s} "
                  f"heading={float(c['obstacle_heading']) * 180 / math.pi:6.1f}deg "
                  f"start=({c['obstacle_start_x']},{c['obstacle_start_y']}) "
                  f"release={c['obstacle_trigger_delay']:>7s}s")
        return
    if not args.out:
        raise SystemExit('--out is required')

    out_dir = os.path.abspath(args.out)
    log_dir = os.path.join(out_dir, 'logs')
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    extra = dict(kv.split('=', 1) for kv in args.set)
    modes = [m.strip() for m in args.modes.split(',') if m.strip()]
    conds = [c.strip() for c in args.conditions.split(',') if c.strip()]
    for c in conds:
        if c not in CONDITIONS:
            raise SystemExit(f'unknown condition {c}; try --list')

    plan = []
    for cname in conds:
        for i in range(args.start_index, args.start_index + args.trials):
            for m in modes:
                plan.append((cname, m, i))

    print(f'Stage-4F benchmark: conditions={conds} modes={modes} '
          f'trials={args.trials} -> {out_dir}', flush=True)
    kill_stragglers()

    t0 = time.time()
    for n, (cname, mode, i) in enumerate(plan, 1):
        tid = f'{cname}_{mode}_{i:02d}{args.tag}'
        print(f'[{n}/{len(plan)}] {tid}  (elapsed {time.time() - t0:.0f}s)', flush=True)
        run_trial(CONDITIONS[cname], mode, tid, out_dir, log_dir, extra,
                  args.timeout, args.domain_base + (n % 20), args.nav_timeout)
    print(f'done in {time.time() - t0:.0f}s', flush=True)


if __name__ == '__main__':
    main()
