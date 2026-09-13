#!/usr/bin/env python3
"""Stage-4E batch driver: run N trials of one or both arms from ONE scenario
definition.

Every trial is a full simulator restart, so each arm starts from a bitwise
identical world: same robot spawn pose, same obstacle park pose, same Nav2
params file, same tracker. The only thing the `--modes` flag changes is the
boolean the trial runner writes to `predicted_obstacle_layer.enabled`.

Usage
  run_stage4e_ab.py --out DIR --trials 10 --modes reactive,predictive
  run_stage4e_ab.py --out DIR --trials 3 --scenario nominal --modes reactive
  run_stage4e_ab.py --out DIR --trials 5 --scenario noconflict
"""
import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time

# One scenario definition, shared by both arms. Anything the launch file
# exposes can be overridden on the command line, but the A/B runs use these.
# Route direction matters. Driving -x puts the corridor's only really tight
# pinch (0.13 m lateral margin at x = 0.90-1.30) in the first 0.75 m, where the
# robot traverses it undisturbed and at speed. In the original +x direction that
# pinch sat AFTER the crossing, and every single timeout in a 10+10 batch --
# both arms alike -- was the robot stalling at x = 0.58 on the pinch entrance
# once the conflict was already resolved. That is a route artifact with nothing
# to do with prediction, and it was corrupting the success-rate metric.
SCENARIO = {
    'crossing': {
        'start_x': '1.75', 'start_y': '-0.55', 'start_yaw': '3.14159',
        'goal_x': '-1.75', 'goal_y': '-0.55', 'goal_yaw': '3.14159',
        'cross_x': '-0.55', 'cross_y': '-0.55',
        # The obstacle continues to y = +0.90 and halts there: 1.45 m clear of
        # the robot's corridor, i.e. outside the 0.90 m reach of its own
        # inflation, so once it has crossed it stops influencing the route at
        # all. Parking it at +0.35 left residual inflation over the corridor's
        # narrowest pinch (x 0.9-1.3, only 0.13 m lateral margin) and deadlocked
        # BOTH arms. Its column is clear to y = +1.10 (min clearance 0.35 m).
        'obstacle_start_y': '-2.05', 'obstacle_stop_y': '0.90',
        # 0.25 m/s is the Stage-4B/4C validated tracker regime. The obstacle
        # covers the 1.50 m from its park pose to the robot's corridor in
        # 6.00 s, and is released 1.15 s after goal acceptance so it arrives at
        # the crossing at t = 7.15 s -- the measured obstacle-free time for the
        # robot to reach the same point (7.0-7.3 s, mean 7.13 s over 3 nominal
        # runs). Worst case: simultaneous arrival, i.e. a genuine conflict if
        # the robot does not act on the prediction.
        'obstacle_speed': '0.25', 'obstacle_dir': '1.0',
        'obstacle_trigger_delay': '1.15',
        'spawn_obstacle': 'True',
    },
    # Control: same obstacle, same column, same release schedule, but driven
    # AWAY from the robot's corridor. Tracked and predicted, never in conflict.
    'noconflict': {
        'start_x': '1.75', 'start_y': '-0.55', 'start_yaw': '3.14159',
        'goal_x': '-1.75', 'goal_y': '-0.55', 'goal_yaw': '3.14159',
        'cross_x': '-0.55', 'cross_y': '-0.55',
        'obstacle_start_y': '-1.30', 'obstacle_stop_y': '-2.20',
        'obstacle_speed': '0.25', 'obstacle_dir': '-1.0',
        'obstacle_trigger_delay': '1.15',
        'spawn_obstacle': 'True',
    },
    # Baseline: no obstacle at all. Defines the nominal trajectory/speed used
    # to set the reaction thresholds.
    'nominal': {
        'start_x': '1.75', 'start_y': '-0.55', 'start_yaw': '3.14159',
        'goal_x': '-1.75', 'goal_y': '-0.55', 'goal_yaw': '3.14159',
        'cross_x': '-0.55', 'cross_y': '-0.55',
        'obstacle_start_y': '-2.05', 'obstacle_stop_y': '0.35',
        'obstacle_speed': '0.0', 'obstacle_dir': '1.0',
        'obstacle_trigger_delay': '0.0',
        'spawn_obstacle': 'False',
    },
}

# Frozen Stage-4E predictive policy. Applied in BOTH arms (in the reactive arm
# the layer is disabled, so the values are inert) so the arms are configured
# identically apart from `enabled`.
POLICY = {
    'policy_max_prediction_horizon': '3.0',
    'policy_sigma_level': '1.5',
    'policy_temporal_decay': '0.35',
    'policy_max_cost': '250',
    'policy_min_cost': '200',
    'policy_max_influence_radius': '0.55',
    # Reaction thresholds, frozen from the nominal runs before any A/B trial.
    'nominal_speed': '0.48',
}


PATTERNS = ('gz sim', 'gz-sim', 'ruby.*gz', 'parameter_bridge',
            'robot_state_publisher', 'rviz2', 'lidar_obstacle_tracker',
            'stage4e_trial', 'component_container', 'publish_initial_pose',
            'ros_gz_bridge', 'ros_gz_sim', 'stage4e_bringup')


def kill_stragglers(verify_timeout=45.0):
    """Guarantee no process from a previous trial survives into the next.

    This matters more than it looks. A surviving component_container keeps its
    Nav2 lifecycle nodes on the ROS graph, so the next trial's bringup finds
    duplicate node names, reports "Failed to bring up all requested nodes",
    and every NavigateToPose goal is rejected -- or worse, two controllers
    publish to /cmd_vel at once and the robot deadlocks mid-corridor. Both
    were observed before this was tightened, and they corrupt trials in ways
    that are easy to mistake for genuine navigation failures.
    """
    deadline = time.time() + verify_timeout
    while True:
        for pat in PATTERNS:
            subprocess.run(['pkill', '-9', '-f', pat],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2.0)
        alive = [pat for pat in PATTERNS
                 if subprocess.run(['pgrep', '-f', pat],
                                   stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL).returncode == 0]
        if not alive:
            break
        if time.time() > deadline:
            print(f'     WARNING: still alive after kill: {alive}', flush=True)
            break
    time.sleep(3.0)


def run_trial(mode, scenario, trial_id, out_dir, log_dir, extra, timeout, domain_id):
    out_path = os.path.join(out_dir, f'{trial_id}.json')
    log_path = os.path.join(log_dir, f'{trial_id}.log')
    sc = dict(SCENARIO[scenario])
    spawn = sc.pop('spawn_obstacle')

    cmd = ['ros2', 'launch', 'predictive_nav_bringup', 'stage4e_bringup.launch.py',
           'headless:=True', 'use_rviz:=False',
           f'mode:={mode}', f'scenario:={scenario}',
           f'trial_id:={trial_id}', f'out_path:={out_path}',
           f'spawn_obstacle:={spawn}']
    for k, v in {**sc, **POLICY, **extra}.items():
        cmd.append(f'{k}:={v}')

    # A distinct ROS_DOMAIN_ID per trial means that even if teardown somehow
    # leaves something running, it cannot be discovered by the next trial.
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain_id))

    print(f'  -> {trial_id} [{mode}/{scenario}] domain={domain_id}', flush=True)
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
        print('     NO RESULT FILE (trial failed to produce output)', flush=True)
        return None
    if 'Failed to bring up all requested nodes' in open(log_path, errors='ignore').read():
        print('     INFRASTRUCTURE FAULT: Nav2 bringup failed; trial invalid',
              flush=True)
    with open(out_path) as f:
        r = json.load(f)
    m = r.get('metrics', {})
    print(f"     status={m.get('nav_status')} t={m.get('nav_time')} "
          f"clr={m.get('min_clearance')} react={m.get('primary_reaction_time')} "
          f"pred_cells={m.get('max_pred_cells')}", flush=True)
    return r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--trials', type=int, default=10)
    ap.add_argument('--modes', default='reactive,predictive')
    ap.add_argument('--scenario', default='crossing', choices=list(SCENARIO))
    ap.add_argument('--start-index', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=300.0)
    ap.add_argument('--nav-timeout', type=float, default=60.0,
                    help='per-trial navigation timeout; nominal is ~10 s')
    ap.add_argument('--set', action='append', default=[],
                    help='extra launch arg, key=value (repeatable)')
    ap.add_argument('--domain-base', type=int, default=40,
                    help='ROS_DOMAIN_ID base; each trial uses base + (n %% 20)')
    ap.add_argument('--interleave', action='store_true',
                    help='alternate arms instead of running one arm then the '
                         'other, so slow machine drift cannot bias one arm')
    args = ap.parse_args()

    out_dir = os.path.abspath(args.out)
    log_dir = os.path.join(out_dir, 'logs')
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(log_dir, exist_ok=True)
    extra = dict(kv.split('=', 1) for kv in args.set)
    extra.setdefault('nav_timeout', str(args.nav_timeout))
    modes = [m.strip() for m in args.modes.split(',') if m.strip()]

    plan = []
    for i in range(args.start_index, args.start_index + args.trials):
        for m in modes:
            plan.append((m, i))
    if not args.interleave:
        plan.sort(key=lambda x: (modes.index(x[0]), x[1]))

    print(f'Stage-4E batch: scenario={args.scenario} modes={modes} '
          f'trials={args.trials} -> {out_dir}', flush=True)
    kill_stragglers()

    t_start = time.time()
    for n, (mode, i) in enumerate(plan, 1):
        tid = f'{args.scenario}_{mode}_{i:02d}'
        print(f'[{n}/{len(plan)}] {tid}  (elapsed {time.time()-t_start:.0f}s)',
              flush=True)
        run_trial(mode, args.scenario, tid, out_dir, log_dir, extra, args.timeout,
                  args.domain_base + (n % 20))

    print(f'done in {time.time()-t_start:.0f}s', flush=True)


if __name__ == '__main__':
    main()
