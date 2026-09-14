#!/usr/bin/env python3
"""Stage-4G1 diagnostic: measure static-return residuals as a function of range.

Run this in a world with NO dynamic obstacle. Every LiDAR return is then, by
construction, a static (mapped) return, so any point the tracker's static
rejection RETAINS is a false positive. For every point this records:

  * the raw beam range r,
  * the true Euclidean distance from the transformed map-frame point to the
    nearest occupied map cell (from a Euclidean distance transform of the map),
  * whether the CURRENT fixed-radius rule would retain it.

and bins all of it by range. It also logs the AMCL pose error (TF map->base_link
versus the simulator's exact pose), so the range-dependent term can be
attributed to a measured cause instead of an assumed one.

This measures ONLY perception geometry. It never publishes to the navigation
stack, and ground truth is used for evaluation only.

    ros2 run predictive_nav_tracking stage4g1_static_probe.py \
        --ros-args -p out_path:=/tmp/probe.json -p duration:=40.0
"""
import json
import math
import os
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

RANGE_BINS = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6), (6, 8), (8, 99)]


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class StaticProbe(Node):

    def __init__(self):
        super().__init__('stage4g1_static_probe')
        p = self.declare_parameter
        p('out_path', '')
        p('duration', 40.0)
        p('scan_topic', '/scan')
        p('map_topic', '/map')
        p('map_frame', 'map')
        p('occupied_threshold', 65)
        # Old rule, reproduced for the "before" column.
        p('legacy_static_reject_radius', 0.15)
        # New range-aware rule, reproduced for the "after" column so both are
        # scored on exactly the same points from a single run.
        p('base_static_margin', 0.06)
        p('localization_margin', 0.06)
        p('angular_sampling_scale', 0.030)
        p('max_static_reject_radius', 0.40)
        # Ground-truth robot spawn pose, for the localisation-error measurement.
        p('start_x', -3.0)
        p('start_y', 0.0)
        p('start_yaw', 0.0)

        self.P = {n: self.get_parameter(n).value for n in
                  ('out_path', 'duration', 'scan_topic', 'map_topic', 'map_frame',
                   'occupied_threshold', 'legacy_static_reject_radius',
                   'base_static_margin', 'localization_margin',
                   'angular_sampling_scale', 'max_static_reject_radius',
                   'start_x', 'start_y', 'start_yaw')}

        self.edt = None          # metres to nearest occupied cell, per cell
        self.info = None
        self.odom = None
        self.n_scans = 0
        self.records = []        # (range, true_dist)
        self.n_tracks_seen = []  # tracks published per scan, if any
        self.pose_err = []       # (dx, dy, dyaw)
        self.angle_increment = None

        qos_tl = QoSProfile(depth=1)
        qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_tl.reliability = QoSReliabilityPolicy.RELIABLE
        qos_s = QoSProfile(depth=5)
        qos_s.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(OccupancyGrid, self.P['map_topic'],
                                 self._map_cb, qos_tl)
        self.create_subscription(LaserScan, self.P['scan_topic'],
                                 self._scan_cb, qos_s)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_s)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _odom_cb(self, m):
        self.odom = m

    def _map_cb(self, m):
        """Precompute a Euclidean distance transform: metres from each cell to
        the nearest OCCUPIED cell. Done once; the per-point query is then a
        single array lookup rather than a neighbourhood search."""
        from scipy.ndimage import distance_transform_edt
        self.info = m.info
        w, h = m.info.width, m.info.height
        grid = np.array(m.data, dtype=np.int16).reshape(h, w)
        occupied = grid >= self.P['occupied_threshold']
        if not occupied.any():
            self.get_logger().error('map has no occupied cells')
            return
        self.edt = distance_transform_edt(~occupied) * m.info.resolution
        self.get_logger().info(
            f'map {w}x{h} @ {m.info.resolution} m, {int(occupied.sum())} occupied '
            f'cells; EDT ready')

    def _scan_cb(self, msg):
        if self.edt is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.P['map_frame'], msg.header.frame_id, msg.header.stamp,
                timeout=Duration(seconds=0.1))
        except Exception:
            return
        self.angle_increment = msg.angle_increment

        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        th = yaw_of(tf.transform.rotation)
        c, s = math.cos(th), math.sin(th)

        res = self.info.resolution
        ox = self.info.origin.position.x
        oy = self.info.origin.position.y
        H, W = self.edt.shape
        n = len(msg.ranges)
        for i in range(n):
            r = msg.ranges[i]
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            a = msg.angle_min + i * msg.angle_increment
            lx = r * math.cos(a)
            ly = r * math.sin(a)
            mx = c * lx - s * ly + dx
            my = s * lx + c * ly + dy
            cx = int(math.floor((mx - ox) / res))
            cy = int(math.floor((my - oy) / res))
            if cx < 0 or cy < 0 or cx >= W or cy >= H:
                continue
            d = float(self.edt[cy, cx])
            self.records.append((float(r), d))

        # localisation error: AMCL (TF map->base_link) vs the simulator's exact
        # pose (known spawn composed with exact odometry). Evaluation only.
        if self.odom is not None:
            try:
                tb = self.tf_buffer.lookup_transform(
                    self.P['map_frame'], 'base_link', rclpy.time.Time(),
                    timeout=Duration(seconds=0.05))
                op = self.odom.pose.pose.position
                oyaw = yaw_of(self.odom.pose.pose.orientation)
                y0 = self.P['start_yaw']
                cc, ss = math.cos(y0), math.sin(y0)
                gx = self.P['start_x'] + cc * op.x - ss * op.y
                gy = self.P['start_y'] + ss * op.x + cc * op.y
                gyaw = y0 + oyaw
                ex = tb.transform.translation.x - gx
                ey = tb.transform.translation.y - gy
                eth = math.atan2(math.sin(yaw_of(tb.transform.rotation) - gyaw),
                                 math.cos(yaw_of(tb.transform.rotation) - gyaw))
                self.pose_err.append((ex, ey, eth))
            except Exception:
                pass

        self.n_scans += 1


def summarize(node):
    recs = node.records
    out = {
        'n_scans': node.n_scans,
        'n_points': len(recs),
        'angle_increment_rad': node.angle_increment,
        'legacy_static_reject_radius': node.P['legacy_static_reject_radius'],
    }
    if not recs:
        return out

    R = np.array([r[0] for r in recs])
    D = np.array([r[1] for r in recs])
    # Old rule: one fixed radius at every range.
    L = D > node.P['legacy_static_reject_radius']
    # New rule: clamp(base + loc + ang*range, min, max).
    NEW_T = np.clip(node.P['base_static_margin'] + node.P['localization_margin']
                    + node.P['angular_sampling_scale'] * R,
                    node.P['legacy_static_reject_radius'],
                    node.P['max_static_reject_radius'])
    N = D > NEW_T
    out['new_rule'] = {
        'base_static_margin': node.P['base_static_margin'],
        'localization_margin': node.P['localization_margin'],
        'angular_sampling_scale': node.P['angular_sampling_scale'],
        'max_static_reject_radius': node.P['max_static_reject_radius'],
        'min_static_reject_radius': node.P['legacy_static_reject_radius'],
    }

    def q(v, p):
        return round(float(np.percentile(v, p)), 4) if len(v) else None

    bins = []
    for lo, hi in RANGE_BINS:
        m = (R >= lo) & (R < hi)
        if not m.any():
            continue
        d = D[m]
        bins.append({
            'range_bin_m': f'{lo}-{hi}',
            'n_points': int(m.sum()),
            'dist_to_occupied_mean_m': round(float(d.mean()), 4),
            'dist_to_occupied_p50_m': q(d, 50),
            'dist_to_occupied_p95_m': q(d, 95),
            'dist_to_occupied_p99_m': q(d, 99),
            'dist_to_occupied_max_m': round(float(d.max()), 4),
            'reject_radius_at_bin_mid_m': round(float(np.clip(
                node.P['base_static_margin'] + node.P['localization_margin']
                + node.P['angular_sampling_scale'] * ((lo + min(hi, 10)) / 2.0),
                node.P['legacy_static_reject_radius'],
                node.P['max_static_reject_radius'])), 4),
            'legacy_retained': int(L[m].sum()),
            'legacy_residual_rate': round(float(L[m].mean()), 6),
            'new_retained': int(N[m].sum()),
            'new_residual_rate': round(float(N[m].mean()), 6),
        })
    out['range_bins'] = bins
    out['overall'] = {
        'dist_to_occupied_mean_m': round(float(D.mean()), 4),
        'dist_to_occupied_p99_m': q(D, 99),
        'dist_to_occupied_max_m': round(float(D.max()), 4),
        'legacy_retained': int(L.sum()),
        'legacy_residual_rate': round(float(L.mean()), 6),
        'new_retained': int(N.sum()),
        'new_residual_rate': round(float(N.mean()), 6),
    }
    if node.pose_err:
        P = np.array(node.pose_err)
        out['localisation_error'] = {
            'n': len(P),
            'trans_mean_m': round(float(np.hypot(P[:, 0], P[:, 1]).mean()), 4),
            'trans_p95_m': round(float(np.percentile(np.hypot(P[:, 0], P[:, 1]), 95)), 4),
            'trans_max_m': round(float(np.hypot(P[:, 0], P[:, 1]).max()), 4),
            'yaw_mean_rad': round(float(np.abs(P[:, 2]).mean()), 5),
            'yaw_p95_rad': round(float(np.percentile(np.abs(P[:, 2]), 95)), 5),
            'yaw_max_rad': round(float(np.abs(P[:, 2]).max()), 5),
        }
        # If yaw error is the range-dependent driver, residual distance should
        # grow like r * yaw_error. Report the implied coefficient per bin.
        yaw95 = out['localisation_error']['yaw_p95_rad']
        for b in out['range_bins']:
            lo, hi = b['range_bin_m'].split('-')
            rmid = (float(lo) + float(hi)) / 2.0
            b['predicted_p95_from_yaw_m'] = round(rmid * yaw95, 4)
    return out


def main():
    rclpy.init()
    node = StaticProbe()
    end = time.monotonic() + float(node.P['duration'])
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    out = summarize(node)
    path = node.P['out_path']
    if path:
        os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
        with open(path, 'w') as f:
            json.dump(out, f, indent=1)
        node.get_logger().info(f'wrote {path}')
    print(json.dumps(out, indent=1))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
