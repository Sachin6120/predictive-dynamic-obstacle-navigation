#!/usr/bin/env python3
"""Stage-4D evaluation: measures what the predictive costmap layer actually
writes, independently of the tracker that feeds it.

Subscribes to the layer's own debug OccupancyGrid (its contribution ONLY,
not the combined master costmap) plus /tracked_objects, and records a
per-cycle timeline of:

  * number of predictive cells with cost > 0
  * maximum predictive cost in the grid
  * centroid of the predictive region (cost-weighted, costmap frame)
  * maximum predictive extent (m) from that centroid
  * number of fresh tracks and their prediction-mean centroid, transformed
    from the tracker frame into the costmap frame via TF

The prediction-centroid vs costmap-region-centroid comparison is the
coordinate-correctness check: the layer transforms mean AND covariance from
the tracker's map frame into the costmap's frame (odom for the rolling local
costmap), so a systematic offset between the two would show up here.

Timeline entries are timestamped so post-hoc questions can be answered
directly, e.g. "how long after the last track did predictive cost reach
zero" (track-expiry clearing) and "were there any predictive cells left
behind after a direction reversal" (ghost corridors).

Prints a JSON summary to stdout; --timeline writes the raw per-cycle series.
"""
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from predictive_nav_msgs.msg import TrackedObjectArray

import tf2_ros


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def mean(values):
    return sum(values) / len(values) if values else None


class Stage4dEvaluator(Node):

    def __init__(self, args):
        super().__init__('stage4d_evaluator')
        self.args = args
        self.timeline = []
        self.latest_tracks = None
        self.latest_tracks_wall = None
        self.grid_msgs = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(
            OccupancyGrid, args.debug_topic, self._on_grid, 10)
        self.create_subscription(
            TrackedObjectArray, '/tracked_objects', self._on_tracks, 10)

    def _on_tracks(self, msg: TrackedObjectArray):
        self.latest_tracks = msg
        self.latest_tracks_wall = time.time()

    def _frame_transform(self, target_frame, source_frame):
        """Returns (tx, ty, cos_yaw, sin_yaw) or None."""
        if target_frame == source_frame:
            return (0.0, 0.0, 1.0, 0.0)
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
        except Exception:
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (tf.transform.translation.x, tf.transform.translation.y,
                math.cos(yaw), math.sin(yaw))

    def _prediction_points_in(self, frame_id):
        """Cost-free reference: every predicted mean of every track,
        transformed into `frame_id`, as a list of (time_from_now, x, y)."""
        msg = self.latest_tracks
        if msg is None or not msg.tracks:
            return None
        transform = self._frame_transform(frame_id, msg.header.frame_id)
        if transform is None:
            return None
        tx, ty, cos_a, sin_a = transform

        points = []
        for t in msg.tracks:
            for p in t.predictions:
                x, y = p.position.x, p.position.y
                points.append((
                    p.time_from_now,
                    cos_a * x - sin_a * y + tx,
                    sin_a * x + cos_a * y + ty))
        return points or None

    def _prediction_centroid_in(self, frame_id):
        """Mean of all predicted means, transformed into `frame_id`.
        Returns (x, y, n_predictions) or None."""
        points = self._prediction_points_in(frame_id)
        if not points:
            return None
        return (sum(p[1] for p in points) / len(points),
                sum(p[2] for p in points) / len(points),
                len(points))

    def _on_grid(self, msg: OccupancyGrid):
        self.grid_msgs += 1
        res = msg.info.resolution
        ox = msg.info.origin.position.x
        oy = msg.info.origin.position.y
        width = msg.info.width

        total_weight = 0.0
        sum_x = 0.0
        sum_y = 0.0
        cells = 0
        max_cost = 0
        peak_cell = None
        points = []
        for idx, value in enumerate(msg.data):
            if value <= 0:
                continue
            cells += 1
            mx = idx % width
            my = idx // width
            wx = ox + (mx + 0.5) * res
            wy = oy + (my + 0.5) * res
            if value > max_cost:
                max_cost = value
                peak_cell = (wx, wy)
            sum_x += wx * value
            sum_y += wy * value
            total_weight += value
            points.append((wx, wy))

        centroid = None
        max_extent = 0.0
        bbox = None
        if total_weight > 0.0:
            centroid = (sum_x / total_weight, sum_y / total_weight)
            for (wx, wy) in points:
                max_extent = max(max_extent, math.hypot(wx - centroid[0], wy - centroid[1]))
            bbox = (min(p[0] for p in points), min(p[1] for p in points),
                    max(p[0] for p in points), max(p[1] for p in points))

        pred_centroid = self._prediction_centroid_in(msg.header.frame_id)

        n_tracks = len(self.latest_tracks.tracks) if self.latest_tracks else 0
        tracks_age = None
        if self.latest_tracks_wall is not None:
            tracks_age = time.time() - self.latest_tracks_wall

        entry = {
            'wall_time': time.time(),
            'grid_stamp': stamp_to_sec(msg.header.stamp),
            'frame_id': msg.header.frame_id,
            'predictive_cells': cells,
            'max_cost': max_cost,
            'centroid': centroid,
            'bbox': bbox,
            'peak_cell': peak_cell,
            'max_extent_m': max_extent,
            'n_tracks_last_msg': n_tracks,
            'tracks_msg_age_s': tracks_age,
            'prediction_centroid': pred_centroid[:2] if pred_centroid else None,
            'n_predictions': pred_centroid[2] if pred_centroid else 0,
        }
        if centroid and pred_centroid:
            entry['centroid_offset_m'] = math.hypot(
                centroid[0] - pred_centroid[0], centroid[1] - pred_centroid[1])

        # Unconfounded coordinate check: the highest-cost cell must sit at the
        # nearest-horizon predicted mean (that prediction has the largest
        # temporal weight and the tightest covariance, so it dominates the
        # cost peak). Unlike the region centroid, this is unaffected by the
        # rolling window clipping the far-horizon ellipses.
        pred_points = self._prediction_points_in(msg.header.frame_id)
        if peak_cell and pred_points:
            nearest = min(pred_points, key=lambda p: p[0])
            entry['nearest_horizon_s'] = nearest[0]
            entry['nearest_prediction'] = (nearest[1], nearest[2])
            entry['peak_offset_m'] = math.hypot(
                peak_cell[0] - nearest[1], peak_cell[1] - nearest[2])
            # The layer rasterizes the tracks message it held at its own
            # update, while this evaluator may already hold a newer one. With
            # the obstacle moving, that skew alone shifts the comparison, so
            # the strict coordinate check only uses near-synchronous samples.
            entry['track_grid_skew_s'] = abs(
                stamp_to_sec(msg.header.stamp) - stamp_to_sec(self.latest_tracks.header.stamp))
            # A rolling local costmap is only a few metres wide. When the
            # predicted mean falls outside it, the true cost peak is clipped
            # away and the brightest *visible* cell sits on the window edge,
            # so such samples say nothing about coordinate correctness.
            entry['nearest_prediction_in_grid'] = (
                ox <= nearest[1] <= ox + width * res and
                oy <= nearest[2] <= oy + msg.info.height * res)
        self.timeline.append(entry)

    # ------------------------------------------------------------------
    def summary(self) -> dict:
        cells_series = [e['predictive_cells'] for e in self.timeline]
        nonzero = [e for e in self.timeline if e['predictive_cells'] > 0]
        offsets = [e['centroid_offset_m'] for e in self.timeline
                   if e.get('centroid_offset_m') is not None]
        peak_offsets = [e['peak_offset_m'] for e in self.timeline
                        if e.get('peak_offset_m') is not None]
        sync_peak_offsets = [
            e['peak_offset_m'] for e in self.timeline
            if e.get('peak_offset_m') is not None and
            e.get('track_grid_skew_s') is not None and
            e['track_grid_skew_s'] <= self.args.sync_tolerance and
            e.get('nearest_prediction_in_grid')]
        clipped_samples = sum(
            1 for e in self.timeline
            if e.get('peak_offset_m') is not None and not e.get('nearest_prediction_in_grid'))

        # Time from the last frame that still had tracks to the first frame
        # with zero predictive cells that stays zero (expiry clearing).
        clear_latency = None
        last_with_tracks = None
        for e in self.timeline:
            if e['n_predictions'] > 0:
                last_with_tracks = e
            elif last_with_tracks is not None and e['predictive_cells'] == 0:
                clear_latency = e['wall_time'] - last_with_tracks['wall_time']
                break

        # Residual predictive cells at the very end of the run, when there
        # were no predictions to justify them: a ghost check.
        trailing_ghost_cells = 0
        for e in reversed(self.timeline):
            if e['n_predictions'] > 0:
                break
            trailing_ghost_cells = max(trailing_ghost_cells, e['predictive_cells'])

        return {
            'mode': 'stage4d',
            'debug_topic': self.args.debug_topic,
            'grid_messages': self.grid_msgs,
            'frames_with_predictive_cost': len(nonzero),
            'frames_total': len(self.timeline),
            'predictive_cells': {
                'mean': mean(cells_series),
                'max': max(cells_series) if cells_series else 0,
                'min': min(cells_series) if cells_series else 0,
            },
            'max_cost_observed': max([e['max_cost'] for e in self.timeline], default=0),
            'max_extent_m': max([e['max_extent_m'] for e in self.timeline], default=0.0),
            'centroid_offset_m': {
                'n': len(offsets),
                'mean': mean(offsets),
                'max': max(offsets) if offsets else None,
                'note': 'confounded by rolling-window clipping of far-horizon ellipses',
            },
            'peak_offset_m': {
                'n': len(peak_offsets),
                'mean': mean(peak_offsets),
                'max': max(peak_offsets) if peak_offsets else None,
                'note': 'peak predictive cell vs nearest-horizon predicted mean; '
                        'includes samples where the evaluator holds a newer tracks '
                        'message than the layer rasterized',
            },
            'peak_offset_synchronized_m': {
                'n': len(sync_peak_offsets),
                'sync_tolerance_s': self.args.sync_tolerance,
                'mean': mean(sync_peak_offsets),
                'max': max(sync_peak_offsets) if sync_peak_offsets else None,
                'samples_excluded_prediction_outside_window': clipped_samples,
                'note': 'strict coordinate-correctness check (map->costmap-frame '
                        'transform of mean and covariance); near-synchronous samples '
                        'whose predicted mean lies inside the rolling window',
            },
            'clear_latency_after_last_prediction_s': clear_latency,
            'trailing_ghost_cells': trailing_ghost_cells,
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument(
        '--debug-topic', type=str,
        default='/local_costmap/predicted_obstacle_layer/debug_costmap')
    parser.add_argument('--output', type=str, default=None)
    parser.add_argument('--timeline', type=str, default=None,
                        help='optional path to dump the raw per-cycle timeline as JSON')
    parser.add_argument('--sync-tolerance', type=float, default=0.05,
                        help='max |grid stamp - tracks stamp| (s) for a sample to count '
                             'towards the strict coordinate-correctness check')
    args = parser.parse_args()

    rclpy.init()
    node = Stage4dEvaluator(args)
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
    if args.timeline:
        with open(args.timeline, 'w') as f:
            json.dump(node.timeline, f, indent=1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
