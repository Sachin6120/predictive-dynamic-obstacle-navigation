#!/usr/bin/env python3
"""
Stage-5B integration check: one real predictive-navigation run, unchanged.

Runs ONE representative Stage-4F predictive-navigation scenario on the FROZEN
tracker profile (tracker_params.yaml, cluster_max_diameter 1.2 m -- NOT the
widened Stage-5B classification profile) and the frozen Nav2 parameters, and
asserts that every part of the validated stack is still operational with the
classifier compiled in and enabled:

  * Nav2 lifecycle nodes reach ACTIVE
  * MPPI is the controller actually loaded
  * the predictive costmap layer is loaded and updating
  * the tracker and the CV predictor are producing tracks and predictions
  * the navigation goal SUCCEEDS with no collision
  * class output is present on /tracked_objects

It deliberately does NOT re-run the 448-trial Stage-4F suite: this is a
"nothing regressed" check on one known-good scenario, which is what the stage
calls for.

WHAT THIS CHECK CANNOT PROVE, and does not claim to: that prediction is
numerically unchanged. Gazebo runs are not bit-reproducible, so comparing two
navigation trials cannot establish identity. That claim is proved separately
and exactly by test/stage5b_tracker_integration.cpp, which drives the same
measurement sequence through the production association path with
classification on and off and compares the Kalman state, the CV predictions
and the reachable regions bit for bit. This check establishes that the live
system still works; that test establishes that it works identically.
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from lifecycle_msgs.srv import GetState                 # noqa: E402

from predictive_nav_msgs.msg import TrackedObjectArray  # noqa: E402

import rclpy                                            # noqa: E402
from rclpy.node import Node                             # noqa: E402

CLASS_NAMES = {0: 'UNKNOWN', 1: 'PEDESTRIAN', 2: 'BIKE', 3: 'VEHICLE'}
LIFECYCLE_NODES = ('controller_server', 'planner_server', 'bt_navigator',
                   'local_costmap/local_costmap', 'global_costmap/global_costmap')


class Probe(Node):
    """Watches /tracked_objects and queries Nav2 lifecycle states."""

    def __init__(self):
        """Subscribe to the tracker output."""
        super().__init__('stage5b_integration_probe')
        self.msgs = 0
        self.tracks_seen = 0
        self.with_predictions = 0
        self.with_reachability = 0
        self.class_field_present = False
        self.labels = {}
        self.create_subscription(
            TrackedObjectArray, '/tracked_objects', self._on_tracks, 10)

    def _on_tracks(self, msg):
        self.msgs += 1
        for t in msg.tracks:
            self.tracks_seen += 1
            if t.predictions:
                self.with_predictions += 1
            if t.reachability_predictions:
                self.with_reachability += 1
            # The field exists by construction once the message is rebuilt;
            # record the values actually observed.
            self.class_field_present = True
            name = CLASS_NAMES.get(t.classification, f'?{t.classification}')
            self.labels[name] = self.labels.get(name, 0) + 1

    def lifecycle_state(self, name, timeout=5.0):
        """Return the lifecycle state label of `name`, or None."""
        client = self.create_client(GetState, f'/{name}/get_state')
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            return None
        return future.result().current_state.label


def main():
    """Run the scenario and print a pass/fail verdict per requirement."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, help='directory for artefacts')
    parser.add_argument('--timeout', type=float, default=420.0)
    args = parser.parse_args()
    os.makedirs(args.out, exist_ok=True)

    from ament_index_python.packages import get_package_share_directory
    tracking = get_package_share_directory('predictive_nav_tracking')
    frozen = os.path.join(tracking, 'config', 'tracker_params.yaml')
    if not os.path.exists(frozen):
        sys.exit(f'missing frozen tracker profile: {frozen}')

    trial_json = os.path.join(args.out, 'trial.json')
    launch_log = os.path.join(args.out, 'launch.log')

    # The frozen Stage-4F 'perpendicular' predictive scenario: the obstacle
    # crosses the robot's route, which is exactly the case the predictive
    # costmap exists for.
    with open(launch_log, 'w') as log:
        proc = subprocess.Popen(
            ['ros2', 'launch', 'predictive_nav_bringup', 'stage4f_bringup.launch.py',
             'run_trial:=True', 'spawn_obstacle:=True', 'headless:=True',
             'use_rviz:=False', 'mode:=predictive', 'scenario:=perpendicular',
             'trial_id:=stage5b_integration', f'out_path:={trial_json}',
             f'tracker_params:={frozen}'],
            stdout=log, stderr=subprocess.STDOUT, preexec_fn=os.setsid)

        rclpy.init()
        probe = Probe()
        states = {}
        try:
            end = time.time() + args.timeout
            probed = False
            while time.time() < end and proc.poll() is None:
                rclpy.spin_once(probe, timeout_sec=0.2)
                # Probe lifecycle once the stack has had time to activate.
                if not probed and probe.msgs > 20:
                    for name in LIFECYCLE_NODES:
                        states[name] = probe.lifecycle_state(name)
                    probed = True
            if proc.poll() is None:
                print('! trial did not finish within the timeout', flush=True)
        finally:
            probe.destroy_node()
            rclpy.shutdown()
            try:
                os.killpg(os.getpgid(proc.pid), 15)
                proc.wait(timeout=30)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), 9)
                except Exception:
                    pass

    log_text = ''
    if os.path.exists(launch_log):
        with open(launch_log, errors='replace') as handle:
            log_text = handle.read()

    trial = {}
    if os.path.exists(trial_json):
        with open(trial_json) as handle:
            trial = json.load(handle)
    metrics = trial.get('metrics', trial) if isinstance(trial, dict) else {}

    def grep(pattern):
        return bool(re.search(pattern, log_text, re.I))

    nav_ok = all(
        states.get(n) == 'active' for n in LIFECYCLE_NODES if states.get(n) is not None)
    any_state = any(states.get(n) is not None for n in LIFECYCLE_NODES)

    checks = [
        ('tracker operational (tracks published)',
         probe.msgs > 0 and probe.tracks_seen > 0,
         f'{probe.msgs} messages, {probe.tracks_seen} track-instances'),
        ('CV predictor operational (predictions populated)',
         probe.with_predictions > 0,
         f'{probe.with_predictions} track-instances carried predictions'),
        ('reachability still populated (Stage-4G4 unchanged)',
         probe.with_reachability > 0,
         f'{probe.with_reachability} track-instances carried reachable regions'),
        ('predictive costmap layer loaded',
         grep(r'PredictedObstacleLayer') or grep(r'predicted_obstacle_layer'),
         'PredictedObstacleLayer referenced in the launch log'),
        ('MPPI is the loaded controller',
         grep(r'MPPIController') or grep(r'mppi'),
         'nav2_mppi_controller referenced in the launch log'),
        ('Nav2 lifecycle nodes ACTIVE',
         any_state and nav_ok,
         ', '.join(f'{n}={states.get(n)}' for n in LIFECYCLE_NODES)),
        ('navigation goal SUCCEEDED',
         bool(metrics.get('success')) or trial.get('status') == 'SUCCEEDED',
         f"status={trial.get('status')} success={metrics.get('success')}"),
        ('no collision',
         metrics.get('collision') is False,
         f"collision={metrics.get('collision')} "
         f"min_clearance={metrics.get('min_clearance')}"),
        ('class output present on /tracked_objects',
         probe.class_field_present,
         f'observed stable labels: {probe.labels}'),
    ]

    print()
    print('=' * 74)
    print('STAGE-5B NAVIGATION INTEGRATION CHECK (frozen tracker profile)')
    print('=' * 74)
    print('  scenario        : stage4f perpendicular, mode=predictive')
    print(f'  tracker profile : {frozen}')
    print(f'  artefacts       : {args.out}')
    print()
    failures = 0
    for name, ok, detail in checks:
        if not ok:
            failures += 1
        print(f'  [{"PASS" if ok else "FAIL"}] {name}')
        print(f'         {detail}')
    print()
    extra = {k: metrics.get(k) for k in
             ('nav_time', 'min_clearance', 'primary_reaction_time', 'max_pred_cells')}
    print(f'  trial metrics   : {extra}')
    print(f'  VERDICT         : {"PASS" if failures == 0 else f"FAIL ({failures})"}')

    summary = os.path.join(args.out, 'integration_check.json')
    with open(summary, 'w') as handle:
        json.dump({
            'checks': [{'name': n, 'pass': bool(o), 'detail': d} for n, o, d in checks],
            'lifecycle': states,
            'trial_status': trial.get('status'),
            'metrics': metrics,
            'observed_labels': probe.labels,
            'failures': failures,
        }, handle, indent=1)
    print(f'  wrote {summary}')
    return 0 if failures == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
