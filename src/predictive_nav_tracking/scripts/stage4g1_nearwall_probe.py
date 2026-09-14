#!/usr/bin/env python3
"""Stage-4G1: does range-aware static rejection erase dynamic obstacles near walls?

A range-dependent rejection radius is wider at long range, so the thing it could
plausibly break is detection of a genuine moving obstacle that passes close to
mapped geometry. This measures exactly that, and it does so in ONE simulator run
rather than one run per clearance: the obstacle is driven on a diagonal so its
clearance to the wall sweeps continuously from metres down to overlap, and
detection is then binned by the instantaneous clearance.

For every published scan it records the obstacle's true wall clearance and
whether the tracker had a track matching the obstacle's true position. Ground
truth is used for scoring and for driving the scripted obstacle only.

    ros2 run predictive_nav_tracking stage4g1_nearwall_probe.py --ros-args \
        -p out_path:=/tmp/nearwall.json
"""
import json
import math
import os
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from predictive_nav_msgs.msg import TrackedObjectArray

# Clearance here is SURFACE-to-surface: obstacle cylinder surface to the nearest
# static map cell.
CLEARANCE_BINS = [(-9, 0.0), (0.0, 0.1), (0.1, 0.2), (0.2, 0.3), (0.3, 0.4),
                  (0.4, 0.6), (0.6, 1.0), (1.0, 99)]
OBSTACLE_RADIUS = 0.20
MATCH_GATE_M = 0.60


class NearWallProbe(Node):

    def __init__(self):
        super().__init__('stage4g1_nearwall_probe')
        p = self.declare_parameter
        p('out_path', '')
        p('duration', 40.0)
        p('vx', 0.10)
        p('vy', 0.12)
        p('stop_y', 3.80)
        p('occupied_threshold', 65)

        self.P = {n: self.get_parameter(n).value for n in
                  ('out_path', 'duration', 'vx', 'vy', 'stop_y',
                   'occupied_threshold')}

        self.edt = None
        self.info = None
        self.obs = None
        self.tracks = None
        self.samples = []

        qos_tl = QoSProfile(depth=1)
        qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_tl.reliability = QoSReliabilityPolicy.RELIABLE
        qos_s = QoSProfile(depth=5)
        qos_s.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos_tl)
        self.create_subscription(Odometry, '/model/dynamic_obstacle/odometry',
                                 self._obs_cb, qos_s)
        self.create_subscription(TrackedObjectArray, '/tracked_objects',
                                 self._tracks_cb, 10)
        self.cmd = self.create_publisher(Twist, '/model/dynamic_obstacle/cmd_vel', 10)

    def _map_cb(self, m):
        from scipy.ndimage import distance_transform_edt
        self.info = m.info
        g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        occ = g >= self.P['occupied_threshold']
        self.edt = distance_transform_edt(~occ) * m.info.resolution
        self.get_logger().info('EDT ready')

    def _obs_cb(self, m):
        self.obs = (m.pose.pose.position.x, m.pose.pose.position.y)

    def _tracks_cb(self, m):
        """One sample per published track array: the obstacle's true clearance
        to the wall, and whether any track matched its true position."""
        if self.obs is None or self.edt is None:
            return
        ox, oy = self.obs
        res = self.info.resolution
        cx = int((ox - self.info.origin.position.x) / res)
        cy = int((oy - self.info.origin.position.y) / res)
        H, W = self.edt.shape
        if not (0 <= cx < W and 0 <= cy < H):
            return
        clearance = float(self.edt[cy, cx]) - OBSTACLE_RADIUS

        best = None
        for t in m.tracks:
            d = math.hypot(t.position.x - ox, t.position.y - oy)
            if best is None or d < best[0]:
                best = (d, t.id, bool(t.kalman_initialized))
        matched = best is not None and best[0] <= MATCH_GATE_M
        self.samples.append({
            'clearance': round(clearance, 4),
            'obs': [round(ox, 3), round(oy, 3)],
            'matched': matched,
            'err': round(best[0], 4) if matched else None,
            'id': best[1] if matched else None,
            'n_tracks': len(m.tracks),
        })

    def drive(self):
        t = Twist()
        if self.obs is not None and self.obs[1] >= self.P['stop_y']:
            t.linear.x = 0.0
            t.linear.y = 0.0
        else:
            t.linear.x = float(self.P['vx'])
            t.linear.y = float(self.P['vy'])
        self.cmd.publish(t)


def summarize(node):
    S = node.samples
    out = {'n_samples': len(S)}
    if not S:
        return out
    bins = []
    for lo, hi in CLEARANCE_BINS:
        sel = [s for s in S if lo <= s['clearance'] < hi]
        if not sel:
            continue
        matched = [s for s in sel if s['matched']]
        errs = [s['err'] for s in matched]
        ids = sorted({s['id'] for s in matched})
        bins.append({
            'clearance_bin_m': f'{lo:.1f}-{hi:.1f}' if lo >= 0 else f'<0 (overlapping)',
            'n_frames': len(sel),
            'n_detected': len(matched),
            'detection_rate': round(len(matched) / len(sel), 3),
            'mean_position_error_m': round(float(np.mean(errs)), 4) if errs else None,
            'distinct_track_ids': len(ids),
        })
    out['clearance_bins'] = bins
    det = [s for s in S if s['matched']]
    out['overall'] = {
        'n_frames': len(S),
        'n_detected': len(det),
        'detection_rate': round(len(det) / len(S), 3),
        'distinct_track_ids': len(sorted({s['id'] for s in det})),
    }
    # Smallest clearance bin that still detects reliably (>= 90% of frames).
    reliable = [b for b in bins if b['detection_rate'] >= 0.9 and
                not b['clearance_bin_m'].startswith('<0')]
    out['smallest_reliable_clearance_bin_m'] = (
        min((b['clearance_bin_m'] for b in reliable),
            key=lambda z: float(z.split('-')[0])) if reliable else None)
    return out


def main():
    rclpy.init()
    node = NearWallProbe()
    end = time.monotonic() + float(node.P['duration'])
    last = 0.0
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        now = time.monotonic()
        if now - last > 0.1:
            node.drive()
            last = now
    t = Twist()
    node.cmd.publish(t)

    out = summarize(node)
    if node.P['out_path']:
        os.makedirs(os.path.dirname(node.P['out_path']) or '.', exist_ok=True)
        with open(node.P['out_path'], 'w') as f:
            json.dump(out, f, indent=1)
    print(json.dumps(out, indent=1))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
