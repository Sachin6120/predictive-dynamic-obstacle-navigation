#!/usr/bin/env python3
"""Small evaluation helper for Stage-4B validation (not part of the tracker).

Two modes:
  --mode static   Runs against a stationary world (no moving obstacle) and
                   counts any track that ever reaches the tracker's
                   'min_observations_to_publish' threshold. Any such track is
                   a false persistent dynamic detection of static geometry.

  --mode dynamic  Matches each published TrackedObjectArray against the
                   moving obstacle's ground-truth odometry (nearest track by
                   position, within a plausibility gate) and reports mean
                   position/velocity error, ID-switch count, and how
                   dominant the majority track ID was over the run.

Prints a JSON summary to stdout.
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from predictive_nav_msgs.msg import TrackedObjectArray

MATCH_DISTANCE_THRESHOLD_M = 1.0


class TrackingEvaluator(Node):

    def __init__(self, mode: str):
        super().__init__('tracking_evaluator')
        self.mode = mode
        self.frames = 0
        self.ground_truth = None
        self.pos_errors = []
        self.vel_errors = []
        self.matched_id_history = []
        self.all_seen_ids = set()

        self.create_subscription(
            TrackedObjectArray, '/tracked_objects', self.tracks_callback, 10)
        if mode == 'dynamic':
            self.create_subscription(
                Odometry, '/model/dynamic_obstacle/odometry', self.gt_callback, 10)

    def gt_callback(self, msg: Odometry):
        self.ground_truth = msg

    def tracks_callback(self, msg: TrackedObjectArray):
        self.frames += 1
        for t in msg.tracks:
            self.all_seen_ids.add(t.id)

        if self.mode == 'static' or not msg.tracks or self.ground_truth is None:
            return

        gt_pos = self.ground_truth.pose.pose.position
        gt_vel = self.ground_truth.twist.twist.linear

        best = min(
            msg.tracks,
            key=lambda t: math.hypot(t.position.x - gt_pos.x, t.position.y - gt_pos.y))
        dist = math.hypot(best.position.x - gt_pos.x, best.position.y - gt_pos.y)
        if dist > MATCH_DISTANCE_THRESHOLD_M:
            return

        self.pos_errors.append(dist)
        self.vel_errors.append(math.hypot(best.velocity.x - gt_vel.x, best.velocity.y - gt_vel.y))
        self.matched_id_history.append(best.id)

    def summary(self) -> dict:
        result = {
            'mode': self.mode,
            'frames_observed': self.frames,
            'distinct_track_ids_seen': len(self.all_seen_ids),
        }
        if self.mode == 'static':
            result['false_persistent_tracks'] = len(self.all_seen_ids)
            return result

        id_switches = 0
        prev = None
        for tid in self.matched_id_history:
            if prev is not None and tid != prev:
                id_switches += 1
            prev = tid

        dominant_id = None
        dominant_fraction = None
        if self.matched_id_history:
            dominant_id = max(set(self.matched_id_history), key=self.matched_id_history.count)
            dominant_fraction = (
                self.matched_id_history.count(dominant_id) / len(self.matched_id_history))

        result.update({
            'matched_frames': len(self.matched_id_history),
            'id_switches': id_switches,
            'dominant_track_id': dominant_id,
            'dominant_track_id_fraction': dominant_fraction,
            'mean_position_error_m': (
                sum(self.pos_errors) / len(self.pos_errors) if self.pos_errors else None),
            'mean_velocity_error_mps': (
                sum(self.vel_errors) / len(self.vel_errors) if self.vel_errors else None),
        })
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['static', 'dynamic'], required=True)
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--output', type=str, default=None)
    args = parser.parse_args()

    rclpy.init()
    node = TrackingEvaluator(args.mode)
    start = time.time()
    try:
        while rclpy.ok() and (time.time() - start) < args.duration:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass

    text = json.dumps(node.summary(), indent=2)
    print(text)
    if args.output:
        with open(args.output, 'w') as f:
            f.write(text)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
