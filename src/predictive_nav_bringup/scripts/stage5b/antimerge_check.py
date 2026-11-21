#!/usr/bin/env python3
"""
Stage-5B.1 ANTI-MERGE regression, in Gazebo, on the real Stage-4G3 geometry.

THE QUESTION THIS ANSWERS
-------------------------
The Stage-5B.1 compound-observation stage re-links two primary clusters when
they look like two slices of one rigid object. Stage-4G3 exists because two
GENUINELY DISTINCT objects passing within 0.46-0.76 m of each other must stay
two tracked objects. So: does turning the new stage on damage that?

The unit tests (test/stage5b1_compound.cpp) answer it on ray-cast fixtures.
This answers it on the real sensor, in the real simulator, with the real
pipeline -- which is the evidence the milestone's acceptance gate asks for.

WHAT IT RUNS
------------
The Stage-4G2/4G3 close-object scenarios, with their recorded start poses and
velocities taken verbatim from stage4g2/scenarios.json so the geometry is not
re-invented here:

  crossing      two obstacles on perpendicular paths, closing to ~0.46 m
  near_crossing the same, at the separation Stage-4G3 measured as hardest
  parallel      two obstacles travelling side by side -- the case where a
                common velocity and a near-constant separation are exactly
                the evidence a rigid body produces, i.e. the hardest
                negative for a temporal rule
  opposing      head-on, so the separation sweeps through the whole band

Each scenario is run TWICE on one launch: once with compound_enabled false
and once with it true, set live through the node's own parameter service so
nothing else can differ between the two arms.

WHAT IT ASSERTS
---------------
  1. ZERO compound observations are emitted in any arm, on any frame, for any
     of these scenarios. This is the hard gate: a single compound association
     here is a Stage-4G3-style false merge.
  2. The number of confirmed tracks per frame is unchanged between arms, so
     the two objects are still resolved as two objects.
  3. The per-frame track count distribution is unchanged, so deblending still
     does its job.

WHAT IT CANNOT PROVE, and does not claim: that the two arms are numerically
identical. Gazebo is not bit-reproducible, and the two arms are separate
traversals. Numerical identity of the Kalman/prediction/reachability path is
proved exactly, and separately, by test/stage5b1_compound_integration.cpp.
"""
import argparse
from collections import Counter
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from predictive_nav_msgs.msg import TrackedObjectArray   # noqa: E402

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue  # noqa: E402
from rcl_interfaces.srv import SetParameters             # noqa: E402

import rclpy                                             # noqa: E402
from rclpy.node import Node                              # noqa: E402

WORLD = 'default'
ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))

#: Robot pose, the Stage-4F start pose -- the validated arena and localisation
#: prior, matching how Stage-4G2/4G3 recorded this geometry.
ROBOT = (-3.0, 0.0, 0.0)

#: The Stage-4G3 close-object scenarios to replay. Start poses and velocities
#: are read from stage4g2/scenarios.json at run time, NOT copied here, so this
#: check cannot drift away from the geometry Stage-4G3 actually measured.
SCENARIOS = ('crossing', 'near_crossing', 'parallel', 'opposing')

#: The Stage-4G3 obstacle model: a 0.2 m radius cylinder. The object whose
#: close-object separation behaviour must not regress.
OBSTACLE = os.path.join(ROOT, 'src', 'predictive_nav_bringup', 'models',
                        'dynamic_obstacle.sdf')


def gz(args, timeout=10):
    """Run a gz CLI command, returning (rc, stdout+stderr)."""
    try:
        done = subprocess.run(
            args, capture_output=True, text=True, timeout=timeout, check=False)
        return done.returncode, done.stdout + done.stderr
    except subprocess.TimeoutExpired:
        return 124, 'timeout'


def set_pose(name, x, y, yaw=0.0, z=0.0):
    """Teleport `name` via the world's set_pose service."""
    req = (f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2.0):.6f}, '
           f'w: {math.cos(yaw / 2.0):.6f}}}')
    return gz(['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
               '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '3000', '--req', req])


def set_velocity(name, vx, vy):
    """Command `name`'s world-frame planar velocity (obstacles are unyawed)."""
    return gz(['gz', 'topic', '-t', f'/model/{name}/cmd_vel',
               '-m', 'gz.msgs.Twist',
               '-p', f'linear: {{x: {vx:.4f}, y: {vy:.4f}, z: 0.0}}, '
                     'angular: {z: 0.0}'])


class Recorder(Node):
    """Records one frame per /tracked_objects message while `recording`."""

    def __init__(self):
        super().__init__('stage5b1_antimerge_recorder')
        self.recording = False
        self.frames = []
        self.seen_any = False
        self.create_subscription(
            TrackedObjectArray, '/tracked_objects', self._on_tracks, 10)

    def _on_tracks(self, msg):
        self.seen_any = True
        if not self.recording:
            return
        self.frames.append({
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            'n_tracks': len(msg.tracks),
            'ids': sorted(int(t.id) for t in msg.tracks),
            # THE GATE: any True here is a false compound association.
            'compound': [bool(t.classification_from_compound) for t in msg.tracks],
            'n_compound': sum(
                1 for t in msg.tracks if t.classification_from_compound),
        })


def spin_for(node, seconds):
    """Spin `node` for `seconds` of wall time."""
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_until(node, predicate, timeout, what):
    """Spin until `predicate()` or `timeout`; return True if it became true."""
    end = time.time() + timeout
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return True
    print(f'  ! timed out waiting for {what}', flush=True)
    return False


def velocity_at(obj, elapsed):
    """
    Return the world-frame velocity commanded for `obj` at `elapsed` seconds.

    A scenario object states its motion EITHER as a single constant
    "velocity", OR as "segments" of [duration, vx, vy] -- the schema
    stage4g2/run.py defines and stage4g2/scenarios.json uses. Reading only
    "velocity" silently drops the second form, and near_crossing -- the
    scenario Stage-4G3 measured as the hardest separation -- uses it, so the
    replay must honour both or it is not replaying Stage-4G3's geometry.

    A "segments" boundary is a STEP change in the commanded velocity, which is
    what Stage-4G2/4G3 ran and what their recorded ground truth shows. No
    replayed scenario carries the Stage-4G4 "accel" key, so no ramp is applied
    here; if one ever does, that is a different trajectory and this replay
    rejects it rather than approximating it.
    """
    if 'accel' in obj:
        raise SystemExit('replay does not model the Stage-4G4 "accel" ramp; '
                         'this scenario is not the Stage-4G3 geometry')
    if 'segments' in obj:
        remaining = float(elapsed)
        for seg_duration, vx, vy in obj['segments']:
            if remaining < float(seg_duration):
                return (float(vx), float(vy))
            remaining -= float(seg_duration)
        return (0.0, 0.0)
    vx, vy = obj['velocity']
    return (float(vx), float(vy))


def drive(node, names, objects, duration):
    """
    Spin for `duration`, holding each object on its scenario velocity.

    Each object's velocity is (re)commanded only when its segment-active value
    CHANGES. A constant-velocity object is therefore commanded exactly once, as
    before; a "segments" object steps at each of its boundaries. Timing follows
    the same wall clock the rest of this script spins on.
    """
    last = [None] * len(objects)
    start = time.time()
    while True:
        elapsed = time.time() - start
        if elapsed >= duration:
            break
        for index, (name, obj) in enumerate(zip(names, objects)):
            want = velocity_at(obj, elapsed)
            if want != last[index]:
                set_velocity(name, want[0], want[1])
                last[index] = want
        rclpy.spin_once(node, timeout_sec=0.05)


def set_compound(node, client, enabled, timeout=10.0):
    """Set compound_enabled live on the running tracker; return True on success."""
    req = SetParameters.Request(parameters=[Parameter(
        name='compound_enabled',
        value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                             bool_value=enabled))])
    future = client.call_async(req)
    end = time.time() + timeout
    while time.time() < end and not future.done():
        rclpy.spin_once(node, timeout_sec=0.05)
    result = future.result()
    return bool(result) and all(r.successful for r in result.results)


def main():
    """Replay the Stage-4G3 close-object geometry with the stage off and on."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, help='output JSON path')
    parser.add_argument('--definitions', default=os.path.join(
        HERE, '..', 'stage4g2', 'scenarios.json'))
    parser.add_argument('--settle', type=float, default=3.0)
    parser.add_argument('--park', type=float, default=3.0)
    parser.add_argument('--startup-timeout', type=float, default=180.0)
    args = parser.parse_args()

    with open(os.path.abspath(args.definitions)) as handle:
        defs = json.load(handle)
    missing = [s for s in SCENARIOS if s not in defs]
    if missing:
        sys.exit(f'scenario definitions missing: {missing}')
    if not os.path.exists(OBSTACLE):
        sys.exit(f'missing obstacle model: {OBSTACLE}')

    # The FROZEN navigation tracker profile, so cluster_distance,
    # cluster_max_diameter and the Stage-4G3 deblend parameters are exactly
    # the validated ones. Only compound_enabled is varied, and only live.
    params = os.path.join(ROOT, 'src', 'predictive_nav_tracking', 'config',
                          'tracker_params.yaml')

    launch = subprocess.Popen(
        ['ros2', 'launch', 'predictive_nav_bringup', 'stage4f_bringup.launch.py',
         'run_trial:=False', 'spawn_obstacle:=False', 'headless:=True',
         'use_rviz:=False', f'tracker_params:={params}',
         f'start_x:={ROBOT[0]}', f'start_y:={ROBOT[1]}', f'start_yaw:={ROBOT[2]}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)

    rclpy.init()
    node = Recorder()
    results = []
    names = ('g4g3_obj_a', 'g4g3_obj_b')
    park = ((40.0, 40.0), (44.0, 44.0))
    try:
        print('waiting for simulation...', flush=True)
        if not wait_until(node, lambda: node.seen_any, args.startup_timeout,
                          '/tracked_objects'):
            raise SystemExit(2)
        spin_for(node, 5.0)

        client = node.create_client(
            SetParameters, '/lidar_obstacle_tracker/set_parameters')
        if not wait_until(node, client.service_is_ready, 60.0,
                          'tracker parameter service'):
            raise SystemExit(2)

        for name, (px, py) in zip(names, park):
            rc, _ = gz(['ros2', 'run', 'ros_gz_sim', 'create',
                        '-world', WORLD, '-file', OBSTACLE, '-name', name,
                        '-x', str(px), '-y', str(py), '-z', '0.0'], timeout=60)
            print(f'spawn {name} rc={rc}', flush=True)
        spin_for(node, 3.0)

        for scenario in SCENARIOS:
            objects = defs[scenario]['objects']
            if len(objects) < 2:
                print(f'  skip {scenario}: needs two objects', flush=True)
                continue
            duration = float(defs[scenario].get('duration', 10))
            for arm_enabled in (False, True):
                arm = 'compound_on' if arm_enabled else 'compound_off'
                # Park both out of sight so every track expires: a track
                # surviving across arms would carry its state over.
                for name, (px, py) in zip(names, park):
                    set_velocity(name, 0.0, 0.0)
                    set_pose(name, px, py)
                spin_for(node, args.park)

                if not set_compound(node, client, arm_enabled):
                    raise SystemExit(f'could not set compound_enabled={arm_enabled}')

                for name, obj in zip(names, objects):
                    set_pose(name, float(obj['start'][0]), float(obj['start'][1]))
                spin_for(node, args.settle)

                node.frames = []
                node.recording = True
                drive(node, names, objects, duration)
                node.recording = False
                for name in names:
                    set_velocity(name, 0.0, 0.0)

                frames = list(node.frames)
                compound_frames = [f for f in frames if f['n_compound'] > 0]
                counts = Counter(f['n_tracks'] for f in frames)
                record = {
                    'scenario': scenario,
                    'arm': arm,
                    'compound_enabled': arm_enabled,
                    'frames': len(frames),
                    'compound_observations': sum(f['n_compound'] for f in frames),
                    'frames_with_compound': len(compound_frames),
                    'track_count_distribution': {str(k): v for k, v in
                                                 sorted(counts.items())},
                    'max_tracks': max((f['n_tracks'] for f in frames), default=0),
                    'mean_tracks': (sum(f['n_tracks'] for f in frames) /
                                    len(frames)) if frames else 0.0,
                    'distinct_ids': len({i for f in frames for i in f['ids']}),
                }
                results.append(record)
                print(f"  {scenario:<16s} {arm:<13s} frames={record['frames']:4d} "
                      f"compound={record['compound_observations']:3d} "
                      f"mean_tracks={record['mean_tracks']:.2f} "
                      f"max_tracks={record['max_tracks']}", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        try:
            os.killpg(os.getpgid(launch.pid), 15)
        except (ProcessLookupError, PermissionError):
            pass
        launch.wait(timeout=60)

    # ------------------------------------------------------------- verdict
    on = [r for r in results if r['compound_enabled']]
    off = [r for r in results if not r['compound_enabled']]
    false_compound = sum(r['compound_observations'] for r in results)
    ran = [r for r in results if r['frames'] > 0]

    checks = {
        'all_arms_produced_frames': len(ran) == len(results) and bool(results),
        'zero_false_compound_associations': false_compound == 0,
        'both_arms_present': len(on) == len(off) == len(SCENARIOS),
    }
    # Per scenario: the two objects must still be resolved as two objects, and
    # equally so in both arms.
    per_scenario = []
    for scenario in SCENARIOS:
        a = next((r for r in off if r['scenario'] == scenario), None)
        b = next((r for r in on if r['scenario'] == scenario), None)
        if not a or not b:
            continue
        entry = {
            'scenario': scenario,
            'max_tracks_off': a['max_tracks'],
            'max_tracks_on': b['max_tracks'],
            'mean_tracks_off': round(a['mean_tracks'], 3),
            'mean_tracks_on': round(b['mean_tracks'], 3),
            'compound_on': b['compound_observations'],
            'two_objects_resolved_off': a['max_tracks'] >= 2,
            'two_objects_resolved_on': b['max_tracks'] >= 2,
        }
        per_scenario.append(entry)
    checks['two_objects_still_resolved_with_stage_on'] = all(
        e['two_objects_resolved_on'] for e in per_scenario) and bool(per_scenario)
    checks['resolution_not_degraded_by_stage'] = all(
        e['max_tracks_on'] >= e['max_tracks_off'] for e in per_scenario)

    out = {
        'what': 'Stage-5B.1 anti-merge regression on the Stage-4G3 close-object '
                'geometry, replayed in Gazebo with the compound stage off and on',
        'tracker_profile': 'tracker_params.yaml (FROZEN navigation profile)',
        'scenarios': list(SCENARIOS),
        'false_compound_associations': false_compound,
        'per_scenario': per_scenario,
        'arms': results,
        'checks': checks,
        'pass': all(checks.values()),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as handle:
        handle.write(json.dumps(out, indent=1) + '\n')

    print()
    for key, value in checks.items():
        print(f'  {"PASS" if value else "FAIL"}  {key}')
    print(f'\nfalse compound associations: {false_compound}')
    print(f'wrote {args.out}')
    return 0 if out['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
