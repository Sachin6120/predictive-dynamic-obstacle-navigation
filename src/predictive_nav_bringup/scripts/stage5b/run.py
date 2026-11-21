#!/usr/bin/env python3
"""
Stage-5B classification trial runner: drive one actor, record the tracker.

WHAT IT DOES. Brings up the validated Stage-4F simulation (room, map, robot,
AMCL, Nav2, tracker) with NO navigation goal and NO Stage-4B obstacle, spawns
ONE Stage-5B actor, and runs every scenario for that actor in turn, recording
each published TrackedObject together with the ground-truth class the actor's
MODEL IDENTITY implies. One JSONL row per (scan, track).

GROUND TRUTH NEVER COMES FROM THE CLASSIFIER. The `gt_class` written on every
row is `actors.gt_class_of(<spawned model name>)`. The classifier's own output
is recorded in separate fields and is never fed back into the label.

WHY THE ACTOR IS PARKED OUT OF SIGHT BETWEEN SCENARIOS. Each scenario must
start from a clean tracker state, otherwise a label accumulated in the
previous run would carry into the next through a surviving track. Simply
teleporting to the next start pose is NOT sufficient, and the first run of
this harness proved it: a track whose last filtered velocity points towards
the new start pose keeps coasting on its constant-velocity prediction, and if
that prediction reaches the teleported actor within association_gate it
RE-ASSOCIATES, so the same track ID -- and its accumulated class evidence --
continues into the next scenario. Two of sixty scenarios were contaminated
this way, both of them a `recede` run following an `approach` run, i.e.
exactly the case where the old track was already moving towards the new pose.

The actor is therefore PARKED far outside the arena first, where the walls
occlude it and it produces no returns at all. With no measurement to
associate, every track covering it misses, and after max_missed_scans it is
pruned -- so the next scenario provably starts with no track. Only then is the
actor teleported to the start pose. The runner still records whether any track
ID was shared with the previous scenario, so contamination cannot pass
unnoticed if the parking interval is ever made too short.

BODY-FRAME VELOCITY. Gazebo's VelocityControl writes the model-level
LinearVelocityCmd component, which is interpreted in the MODEL's own frame,
so the world-frame velocity each scenario asks for is rotated into the actor's
yaw before being commanded.

MOTION IS VERIFIED AGAINST GROUND TRUTH, NOT AGAINST THE TRACKER. Whether an
actor actually traversed its scenario is checked using the actor's own Gazebo
odometry, bridged in for this purpose alone. Using the tracker's reported
position would be circular -- the tracker is the thing under test, and an
object it never detected would be indistinguishable from an object that never
moved. With ground-truth odometry the two are separable, which matters
because NOT DETECTING an object at all is one of the results this stage is
meant to measure.
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from nav_msgs.msg import OccupancyGrid                  # noqa: E402
from nav_msgs.msg import Odometry                       # noqa: E402

from predictive_nav_msgs.msg import TrackedObjectArray  # noqa: E402

import rclpy                                            # noqa: E402
from rclpy.node import Node                             # noqa: E402
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,  # noqa: E402
                       QoSReliabilityPolicy)

from actors import gt_class_of                          # noqa: E402,I100,I201

from scenarios import ROBOT, scenarios                  # noqa: E402,I100,I201

WORLD = 'default'
#: Where an actor is parked between scenarios, map frame. Far outside the
#: 10 x 8 m room, so the walls occlude it and it contributes no scan returns.
PARK = (40.0, 40.0)
#: TrackedObject.classification value -> label name. Must match the CLASS_*
#: constants in TrackedObject.msg.
CLASS_NAMES = {0: 'UNKNOWN', 1: 'PEDESTRIAN', 2: 'BIKE', 3: 'VEHICLE'}


def gz(args, timeout=10):
    """Run a gz CLI command, returning (rc, stdout)."""
    try:
        done = subprocess.run(
            args, capture_output=True, text=True, timeout=timeout, check=False)
        return done.returncode, done.stdout + done.stderr
    except subprocess.TimeoutExpired:
        return 124, 'timeout'


def set_pose(name, x, y, yaw, z=0.0):
    """Teleport `name` to (x, y, z, yaw) via the world's set_pose service."""
    req = (f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
           f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2.0):.6f}, '
           f'w: {math.cos(yaw / 2.0):.6f}}}')
    return gz(['gz', 'service', '-s', f'/world/{WORLD}/set_pose',
               '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
               '--timeout', '3000', '--req', req])


def set_body_velocity(name, vx_body, vy_body):
    """Command `name`'s velocity in its OWN frame (see the module docstring)."""
    return gz(['gz', 'topic', '-t', f'/model/{name}/cmd_vel',
               '-m', 'gz.msgs.Twist',
               '-p', f'linear: {{x: {vx_body:.4f}, y: {vy_body:.4f}, z: 0.0}}, '
                     'angular: {z: 0.0}'])


def world_to_body(vx, vy, yaw):
    """Rotate a world-frame planar velocity into a body frame yawed by `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    return vx * c + vy * s, -vx * s + vy * c


class Recorder(Node):
    """Collects /tracked_objects and the map, and exposes the latest frame."""

    def __init__(self):
        """Subscribe to the latched map and to the tracker's output."""
        super().__init__('stage5b_recorder')
        self.frames = []
        self.recording = False
        self.tag = {}
        self.have_map = False
        latched = QoSProfile(
            depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.create_subscription(
            TrackedObjectArray, '/tracked_objects', self._on_tracks, 10)
        self.last_track_time = 0.0
        self.last_ids = set()
        # Ground-truth actor odometry, used ONLY to verify that a scenario
        # actually ran. Never used as a classification label and never
        # consumed by the tracker.
        self.gt = []
        self.gt_recording = False

    def subscribe_gt(self, actor):
        """Subscribe to the bridged ground-truth odometry of `actor`."""
        self.create_subscription(
            Odometry, f'/model/{actor}/odometry', self._on_gt, 20)

    def _on_gt(self, msg):
        self._gt_seen = True
        if self.gt_recording:
            self.gt.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def gt_peek(self):
        """Report whether any ground-truth odometry message has arrived."""
        return getattr(self, '_gt_seen', False)

    def _on_map(self, _msg):
        self.have_map = True

    def _on_tracks(self, msg):
        self.last_track_time = time.time()
        self.last_ids = {t.id for t in msg.tracks}
        if not self.recording:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        for t in msg.tracks:
            row = dict(self.tag)
            row.update({
                'stamp': stamp,
                'track_id': int(t.id),
                'x': t.position.x,
                'y': t.position.y,
                'speed': t.speed,
                'kalman_initialized': bool(t.kalman_initialized),
                'age': int(t.age),
                'observations': int(t.observations),
                'missed_count': int(t.missed_count),
                # --- Stage-5B classifier output (NEVER used as ground truth) --
                'pred_class': CLASS_NAMES[t.classification],
                'pred_conf': t.classification_confidence,
                'pred_class_instant': CLASS_NAMES[t.classification_instant],
                'pred_conf_instant': t.classification_instant_confidence,
                # --- the geometry the decision was made on -------------------
                'features_valid': bool(t.classification_features_valid),
                'cluster_length': t.cluster_length,
                'cluster_thickness': t.cluster_thickness,
                'cluster_elongation': t.cluster_elongation,
                'cluster_point_count': int(t.cluster_point_count),
                'cluster_range': t.cluster_range,
                'cluster_angular_span': t.cluster_angular_span,
                'cluster_fill_ratio': t.cluster_fill_ratio,
                'cluster_max_gap': t.cluster_max_gap,
                # --- Stage-5B.1 provenance: was this a COMPOUND observation?
                # Read straight off the message, never inferred from geometry,
                # so true and FALSE compound associations are both countable.
                'from_compound': bool(t.classification_from_compound),
            })
            self.frames.append(row)


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


def main():
    """Run every scenario for one actor and write the recorded rows."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--actor', required=True)
    parser.add_argument('--out', required=True, help='output JSONL path')
    parser.add_argument(
        '--tracker-params', default='',
        help='tracker params YAML; default is the Stage-5B classification profile')
    parser.add_argument('--settle', type=float, default=3.0,
                        help='s held still at the start pose before recording')
    parser.add_argument('--park', type=float, default=3.0,
                        help='s parked out of sight, to force every track to expire')
    parser.add_argument('--startup-timeout', type=float, default=180.0)
    args = parser.parse_args()

    rows = [s for s in scenarios() if s['actor'] == args.actor]
    if not rows:
        sys.exit(f'no scenarios for actor {args.actor}')
    gt = gt_class_of(args.actor)

    from ament_index_python.packages import get_package_share_directory
    bringup = get_package_share_directory('predictive_nav_bringup')
    tracking = get_package_share_directory('predictive_nav_tracking')
    params = args.tracker_params or os.path.join(
        tracking, 'config', 'stage5b_classification_tracker.yaml')
    model = os.path.join(bringup, 'models', 'stage5b', f'{args.actor}.sdf')
    for path in (params, model):
        if not os.path.exists(path):
            sys.exit(f'missing required file: {path}')

    launch = subprocess.Popen(
        ['ros2', 'launch', 'predictive_nav_bringup', 'stage4f_bringup.launch.py',
         'run_trial:=False', 'spawn_obstacle:=False', 'headless:=True',
         'use_rviz:=False', f'tracker_params:={params}',
         f'start_x:={ROBOT[0]}', f'start_y:={ROBOT[1]}',
         f'start_yaw:={ROBOT[2]}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)

    rclpy.init()
    node = Recorder()
    out = []
    bridges = []
    scenario_ids = set()
    try:
        print(f'[{args.actor}] waiting for simulation...', flush=True)
        if not wait_until(node, lambda: node.have_map, args.startup_timeout, '/map'):
            raise SystemExit(2)
        # The tracker only publishes once it has map + TF + scan, so a first
        # /tracked_objects message is the real readiness signal.
        if not wait_until(
                node, lambda: node.last_track_time > 0.0, args.startup_timeout,
                '/tracked_objects'):
            raise SystemExit(2)
        spin_for(node, 5.0)   # let AMCL converge on the stationary robot

        rc, _ = gz(['ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', WORLD, '-file', model, '-name', args.actor,
                    '-x', str(rows[0]['x0']), '-y', str(rows[0]['y0']),
                    '-z', '0.0'], timeout=60)
        print(f'[{args.actor}] spawn rc={rc}', flush=True)
        spin_for(node, 3.0)

        bridge = subprocess.Popen(
            ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             f'/model/{args.actor}/odometry@nav_msgs/msg/Odometry'
             f'[gz.msgs.Odometry'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid)
        bridges.append(bridge)
        node.subscribe_gt(args.actor)
        if not wait_until(node, lambda: bool(node.gt_peek()), 30.0,
                          'ground-truth odometry'):
            raise SystemExit(2)

        for i, s in enumerate(rows, 1):
            # --- reset: stop, park out of sight so every track expires, then
            # --- move to the start pose and let a fresh track establish -------
            set_body_velocity(args.actor, 0.0, 0.0)
            set_pose(args.actor, PARK[0], PARK[1], 0.0)
            spin_for(node, args.park)
            set_pose(args.actor, s['x0'], s['y0'], s['yaw'])
            spin_for(node, args.settle)
            prev_ids = set(scenario_ids)

            vbx, vby = world_to_body(s['vx'], s['vy'], s['yaw'])
            node.tag = {
                'scenario': s['id'], 'actor': args.actor, 'gt_class': gt,
                'pattern': s['pattern'], 'speed': s['speed'],
            }
            node.frames = []
            node.gt = []
            node.recording = True
            node.gt_recording = True
            set_body_velocity(args.actor, vbx, vby)
            spin_for(node, s['duration'])
            node.recording = False
            node.gt_recording = False
            set_body_velocity(args.actor, 0.0, 0.0)

            frames = list(node.frames)
            # Did the ACTOR traverse its scenario? Ground truth only.
            gt_moved = 0.0
            if len(node.gt) >= 2:
                gxs = [g[0] for g in node.gt]
                gys = [g[1] for g in node.gt]
                gt_moved = math.hypot(max(gxs) - min(gxs), max(gys) - min(gys))
            ok = gt_moved >= 0.75 * s['travel']

            ids = {f['track_id'] for f in frames}
            scenario_ids = ids
            # A track ID shared with the PREVIOUS scenario would mean the
            # previous run's accumulated class evidence leaked into this one.
            reused = bool(ids & prev_ids)
            for f in frames:
                f['scenario_ok'] = ok
                f['reused_track_id'] = reused
            out.extend(frames)
            detected = 'tracked' if frames else 'NEVER-DETECTED'
            print(f"  [{i:2d}/{len(rows)}] {s['id']:46s} "
                  f"rows={len(frames):4d} gt_moved={gt_moved:.2f}/{s['travel']:.2f}m "
                  f"{'OK' if ok else 'NOT-RUN'} {detected}"
                  f"{' REUSED-ID' if reused else ''}", flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for proc in bridges + [launch]:
            try:
                os.killpg(os.getpgid(proc.pid), 15)
                proc.wait(timeout=30)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), 9)
                except Exception:
                    pass

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, 'w') as handle:
        for row in out:
            handle.write(json.dumps(row) + '\n')
    print(f'[{args.actor}] wrote {len(out)} rows -> {args.out}', flush=True)


if __name__ == '__main__':
    main()
