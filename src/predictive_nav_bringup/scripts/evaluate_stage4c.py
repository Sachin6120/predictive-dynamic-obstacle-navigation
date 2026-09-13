#!/usr/bin/env python3
"""Stage-4C evaluation: RAW vs KALMAN current-state tracking, and future
trajectory prediction (ADE/FDE by horizon), against simulator ground truth.

Extends (does not replace) Stage-4B's scripts/evaluate_tracking.py, which
remains the source of truth for the original static/dynamic regression
numbers. This script additionally requires the Stage-4C TrackedObject
fields (raw_position, raw_velocity, kalman_initialized, predictions[]).

Ground truth is simulator-only: it is never fed into the tracker, only used
here, offline, for validation.

Matching (current-state, per published TrackedObjectArray frame):
  Same policy as Stage-4B -- nearest published track to ground truth
  position (using the track's primary/filtered `position`), gated at
  --match-gate meters. RAW and KALMAN current-state errors are computed
  from that SAME matched track's raw_position/raw_velocity vs
  position/velocity fields against the SAME ground-truth sample, so the
  comparison is apples-to-apples.

Prediction (ADE/FDE):
  Every time the matched track publishes non-empty `predictions`, each
  predicted (time_from_now, x, y) is stored as a pending prediction keyed by
  its ABSOLUTE target timestamp (prediction made_at time + time_from_now).
  Ground-truth odometry samples are buffered separately. A pending
  prediction is only scored once ground truth samples bracketing its target
  timestamp have arrived, by linearly interpolating ground truth position
  to that exact timestamp -- predictions are never compared against the
  ground truth available at prediction time, only against the future truth
  at the corresponding future time.

  Reports:
    - per-horizon displacement error (mean / RMSE) at each of
      --target-horizons (default 0.5, 1.0, 2.0, 3.0 s), pooled across all
      prediction events.
    - classic ADE/FDE per full predicted trajectory (mean error over all
      sampled horizons = ADE; error at the largest/final sampled horizon =
      FDE), averaged over all trajectories.
  Both are additionally split into "near_reversal" (the trajectory's
  prediction was MADE within --reversal-window seconds after a detected
  ground-truth direction reversal on --motion-axis) vs "steady_state", per
  the requirement to surface constant-velocity-model breakdown after a
  bounce separately rather than averaging it away.

Prints a JSON summary to stdout.
"""
import argparse
import math
import json
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from predictive_nav_msgs.msg import TrackedObjectArray

MATCH_DISTANCE_THRESHOLD_M_DEFAULT = 1.0


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def mean(values):
    return sum(values) / len(values) if values else None


def rmse(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


class GroundTruthBuffer:
    """Time-ordered (t, x, y, vx, vy) samples with linear interpolation."""

    def __init__(self, max_len=20000):
        self.samples = deque(maxlen=max_len)

    def append(self, t, x, y, vx, vy):
        self.samples.append((t, x, y, vx, vy))

    def earliest(self):
        return self.samples[0][0] if self.samples else None

    def latest(self):
        return self.samples[-1][0] if self.samples else None

    def interpolate(self, t):
        """Returns (x, y) at time t via linear interpolation between the two
        bracketing samples, or None if t falls outside the buffered range."""
        if not self.samples or t < self.samples[0][0] or t > self.samples[-1][0]:
            return None
        prev = self.samples[0]
        for sample in self.samples:
            if sample[0] >= t:
                if sample[0] == prev[0]:
                    return (sample[1], sample[2])
                ratio = (t - prev[0]) / (sample[0] - prev[0])
                x = prev[1] + ratio * (sample[1] - prev[1])
                y = prev[2] + ratio * (sample[2] - prev[2])
                return (x, y)
            prev = sample
        return (self.samples[-1][1], self.samples[-1][2])


class Stage4cEvaluator(Node):

    def __init__(self, args):
        super().__init__('stage4c_evaluator')
        self.args = args
        self.frames = 0
        self.matched_frames = 0
        self.all_seen_ids = set()
        self.matched_id_history = []

        # Each entry: dict(near_reversal, raw_pos, raw_vel, kf_pos, kf_vel)
        self.current_state_samples = []

        self.gt = GroundTruthBuffer()
        self.pending_predictions = []   # list of dicts, see _on_tracks
        self.scored_horizon_errors = {h: [] for h in args.target_horizons}
        self.scored_horizon_errors_near_reversal = {h: [] for h in args.target_horizons}
        self.trajectory_errors = []              # list of dict(ade, fde, near_reversal)
        self.dropped_predictions = 0
        self._trajectory_scratch = {}            # (track_id, made_at) -> {errors, near_reversal}

        self.reversal_events = []   # absolute timestamps of detected reversals
        self._last_axis_sign = None

        self.create_subscription(TrackedObjectArray, '/tracked_objects', self._on_tracks, 10)
        self.create_subscription(
            Odometry, '/model/dynamic_obstacle/odometry', self._on_odometry, 10)

    # ------------------------------------------------------------------
    def _on_odometry(self, msg: Odometry):
        t = stamp_to_sec(msg.header.stamp)
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.gt.append(t, p.x, p.y, v.x, v.y)

        axis_v = v.x if self.args.motion_axis == 'x' else v.y
        if abs(axis_v) > self.args.reversal_speed_threshold:
            sign = 1.0 if axis_v > 0 else -1.0
            if self._last_axis_sign is not None and sign != self._last_axis_sign:
                self.reversal_events.append(t)
            self._last_axis_sign = sign

        self._score_pending_predictions()

    def _near_reversal(self, made_at_time):
        for r_t in self.reversal_events:
            if 0.0 <= made_at_time - r_t <= self.args.reversal_window:
                return True
        return False

    def _score_pending_predictions(self):
        still_pending = []
        for pred in self.pending_predictions:
            if pred['target_time'] > self.gt.latest():
                still_pending.append(pred)
                continue
            xy = self.gt.interpolate(pred['target_time'])
            if xy is None:
                # Target time already older than the buffered ground-truth
                # window (buffer overflow / run ended); cannot score.
                self.dropped_predictions += 1
                continue
            gx, gy = xy
            err = math.hypot(pred['x'] - gx, pred['y'] - gy)
            pred['error'] = err
            self._finalize_prediction(pred)
        self.pending_predictions = still_pending

    def _finalize_prediction(self, pred):
        near_rev = self._near_reversal(pred['made_at'])
        for h in self.args.target_horizons:
            if abs(pred['time_from_now'] - h) < 1e-6:
                self.scored_horizon_errors[h].append(pred['error'])
                if near_rev:
                    self.scored_horizon_errors_near_reversal[h].append(pred['error'])

        traj = self._trajectory_scratch.setdefault(
            (pred['track_id'], pred['made_at']), {'errors': [], 'near_reversal': near_rev})
        traj['errors'].append((pred['time_from_now'], pred['error']))

    # ------------------------------------------------------------------
    def _flush_trajectories(self):
        """Called at shutdown: any trajectory whose samples are all scored
        contributes one ADE (mean over its sampled horizons) and one FDE
        (error at its largest sampled horizon)."""
        for (_track_id, _made_at), traj in self._trajectory_scratch.items():
            if not traj['errors']:
                continue
            traj['errors'].sort(key=lambda e: e[0])
            errors_only = [e[1] for e in traj['errors']]
            fde = traj['errors'][-1][1]
            self.trajectory_errors.append({
                'ade': mean(errors_only),
                'fde': fde,
                'n_samples': len(errors_only),
                'near_reversal': traj['near_reversal'],
            })

    # ------------------------------------------------------------------
    def _on_tracks(self, msg: TrackedObjectArray):
        self.frames += 1
        for t in msg.tracks:
            self.all_seen_ids.add(t.id)

        if not msg.tracks or not self.gt.samples:
            return

        frame_t = stamp_to_sec(msg.header.stamp)
        gt_now = self.gt.interpolate(frame_t)
        if gt_now is None:
            # Fall back to the latest buffered sample if the scan timestamp
            # is marginally ahead of the last odometry sample received so far.
            last = self.gt.samples[-1]
            gt_now = (last[1], last[2])
        gt_x, gt_y = gt_now
        gt_last = self.gt.samples[-1]
        gt_vx, gt_vy = gt_last[3], gt_last[4]

        best = min(msg.tracks, key=lambda t: math.hypot(t.position.x - gt_x, t.position.y - gt_y))
        dist = math.hypot(best.position.x - gt_x, best.position.y - gt_y)
        if dist > self.args.match_gate:
            return

        self.matched_frames += 1
        self.matched_id_history.append(best.id)

        kf_vel_err = math.hypot(best.velocity.x - gt_vx, best.velocity.y - gt_vy)
        raw_dist = math.hypot(best.raw_position.x - gt_x, best.raw_position.y - gt_y)
        raw_vel_err = math.hypot(best.raw_velocity.x - gt_vx, best.raw_velocity.y - gt_vy)
        self.current_state_samples.append({
            'near_reversal': self._near_reversal(frame_t),
            'raw_pos': raw_dist,
            'raw_vel': raw_vel_err,
            'kf_pos': dist,
            'kf_vel': kf_vel_err,
        })

        made_at = stamp_to_sec(best.stamp)
        for p in best.predictions:
            target_time = stamp_to_sec(p.stamp)
            self.pending_predictions.append({
                'track_id': best.id,
                'made_at': made_at,
                'time_from_now': p.time_from_now,
                'target_time': target_time,
                'x': p.position.x,
                'y': p.position.y,
            })
        self._score_pending_predictions()

    # ------------------------------------------------------------------
    def summary(self) -> dict:
        self._flush_trajectories()

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

        def horizon_stats(errors_by_h):
            return {
                str(h): {
                    'n': len(errs),
                    'mae': mean(errs),
                    'rmse': rmse(errs),
                } for h, errs in errors_by_h.items()
            }

        all_traj = self.trajectory_errors
        near_rev_traj = [t for t in all_traj if t['near_reversal']]
        steady_traj = [t for t in all_traj if not t['near_reversal']]

        def traj_stats(trajs):
            return {
                'n_trajectories': len(trajs),
                'ade_mean': mean([t['ade'] for t in trajs]),
                'fde_mean': mean([t['fde'] for t in trajs]),
            }

        def current_state_stats(samples):
            return {
                'n': len(samples),
                'raw_finite_difference': {
                    'position_mae_m': mean([s['raw_pos'] for s in samples]),
                    'position_rmse_m': rmse([s['raw_pos'] for s in samples]),
                    'velocity_mae_mps': mean([s['raw_vel'] for s in samples]),
                    'velocity_rmse_mps': rmse([s['raw_vel'] for s in samples]),
                },
                'kalman_filtered': {
                    'position_mae_m': mean([s['kf_pos'] for s in samples]),
                    'position_rmse_m': rmse([s['kf_pos'] for s in samples]),
                    'velocity_mae_mps': mean([s['kf_vel'] for s in samples]),
                    'velocity_rmse_mps': rmse([s['kf_vel'] for s in samples]),
                },
            }

        near_rev_samples = [s for s in self.current_state_samples if s['near_reversal']]
        steady_samples = [s for s in self.current_state_samples if not s['near_reversal']]

        return {
            'mode': 'stage4c',
            'frames_observed': self.frames,
            'matched_frames': self.matched_frames,
            'distinct_track_ids_seen': len(self.all_seen_ids),
            'id_switches': id_switches,
            'dominant_track_id': dominant_id,
            'dominant_track_id_fraction': dominant_fraction,
            'reversal_events_detected': len(self.reversal_events),
            'current_state': {
                'all': current_state_stats(self.current_state_samples),
                'near_reversal': current_state_stats(near_rev_samples),
                'steady_state': current_state_stats(steady_samples),
            },
            'prediction': {
                'by_horizon_all': horizon_stats(self.scored_horizon_errors),
                'by_horizon_near_reversal': horizon_stats(self.scored_horizon_errors_near_reversal),
                'ade_fde_all_trajectories': traj_stats(all_traj),
                'ade_fde_near_reversal': traj_stats(near_rev_traj),
                'ade_fde_steady_state': traj_stats(steady_traj),
                'dropped_predictions_end_of_run': self.dropped_predictions,
            },
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--output', type=str, default=None)
    parser.add_argument('--match-gate', type=float, default=MATCH_DISTANCE_THRESHOLD_M_DEFAULT)
    parser.add_argument('--motion-axis', choices=['x', 'y'], default='x')
    parser.add_argument('--reversal-window', type=float, default=1.0,
                         help='seconds after a detected reversal during which a prediction '
                              'is tagged near_reversal')
    parser.add_argument('--reversal-speed-threshold', type=float, default=0.05,
                         help='m/s; ground-truth speed below this is ignored for reversal '
                              'sign detection (avoids noise-triggered false reversals near v=0)')
    parser.add_argument('--target-horizons', type=float, nargs='+',
                         default=[0.5, 1.0, 2.0, 3.0])
    args = parser.parse_args()

    rclpy.init()
    node = Stage4cEvaluator(args)
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
