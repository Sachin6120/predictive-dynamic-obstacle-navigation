#!/usr/bin/env python3
"""Stage-4E single-trial runner: reactive vs predictive A/B on one scenario.

One process drives one trial end to end and writes one machine-readable JSON
result. The SAME code path runs both arms; `mode` only decides whether
`predicted_obstacle_layer.enabled` is set True or False on the local costmap
before the goal is sent. Nothing else differs between MODE A and MODE B --
same params file, same plugins loaded, same tracker, same obstacle schedule --
so the layer's boolean is the single independent variable.

Responsibilities
  1. Wait for the Nav2 stack, TF and the tracker to be live.
  2. Apply the Stage-4E predictive policy + the mode's `enabled` value.
  3. Send one NavigateToPose goal.
  4. Drive the moving obstacle on a deterministic schedule keyed to goal
     acceptance (t=0), one-way, no reversal.
  5. Record commanded velocity, ground-truth poses, tracker output, the
     predictive layer's own debug grid, global plans and collision-monitor
     state at 20 Hz.
  6. Score fixed, pre-declared reaction events and safety metrics.

Ground truth (Gazebo model odometry for the obstacle, the robot's own
simulator odometry) is consumed ONLY here, for evaluation and for commanding
the scripted obstacle. It is never republished into /scan, /tracked_objects,
the costmap layer, the planner or the controller.
"""
import json
import math
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

import tf2_ros
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

from predictive_nav_msgs.msg import TrackedObjectArray

LOCAL_COSTMAP_NODE = '/local_costmap/local_costmap'
LAYER = 'predicted_obstacle_layer'

ROBOT_RADIUS = 0.22      # nav2_predictive_params.yaml local_costmap.robot_radius
OBSTACLE_RADIUS = 0.20   # models/dynamic_obstacle.sdf cylinder radius

# Clearance is measured against the robot's ACTUAL simulated collision
# geometry, not Nav2's circumscribed planning radius. gz_waffle.sdf.xacro gives
# base_collision as a 0.265 x 0.265 m box at (-0.064, 0) in base_link; the
# y half-extent is widened to 0.153 m to enclose the wheel collisions
# (+-0.144 +- 0.009). The waffle is markedly asymmetric -- 0.197 m of body
# behind base_link but only 0.069 m in front -- so a circumscribed circle
# (0.237 m) overstates frontal overlap by ~0.09 m and reports phantom
# collisions for an obstacle passing in front of a stopped robot.
FOOTPRINT_CX = -0.064
FOOTPRINT_HX = 0.1325
FOOTPRINT_HY = 0.153


def footprint_clearance(rx, ry, ryaw, ox, oy):
    """Surface-to-surface distance (m) between the robot's oriented footprint
    box and the obstacle cylinder. <= 0 means the bodies actually intersect."""
    c, s_ = math.cos(-ryaw), math.sin(-ryaw)
    dx, dy = ox - rx, oy - ry
    lx = c * dx - s_ * dy - FOOTPRINT_CX
    ly = s_ * dx + c * dy
    ex = max(abs(lx) - FOOTPRINT_HX, 0.0)
    ey = max(abs(ly) - FOOTPRINT_HY, 0.0)
    return math.hypot(ex, ey) - OBSTACLE_RADIUS


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Stage4ETrial(Node):

    def __init__(self):
        super().__init__('stage4e_trial')

        p = self.declare_parameter
        p('trial_id', 'trial')
        p('mode', 'predictive')            # 'reactive' | 'predictive'
        p('scenario', 'crossing')          # 'crossing' | 'noconflict' | 'nominal'
        p('out_path', '')

        # --- scenario geometry (map frame) ---
        p('start_x', -1.90)
        p('start_y', -0.55)
        p('start_yaw', 0.0)
        p('goal_x', 1.75)
        p('goal_y', -0.55)
        p('goal_yaw', 0.0)
        p('cross_x', 0.55)                 # obstacle column == crossing point x
        p('cross_y', -0.55)                # robot corridor     == crossing point y
        p('obstacle_start_y', -2.05)
        p('obstacle_stop_y', 0.90)
        p('obstacle_speed', 0.25)
        p('obstacle_trigger_delay', 0.0)   # s after goal acceptance
        p('obstacle_dir', 1.0)             # +1 => +y (crossing), -1 => -y (no-conflict)

        # --- Stage-4E predictive policy (applied at runtime, both modes) ---
        p('policy_max_prediction_horizon', 2.0)
        p('policy_sigma_level', 1.2)
        p('policy_temporal_decay', 0.7)
        p('policy_max_cost', 250)
        p('policy_min_cost', 0)
        p('policy_max_influence_radius', 2.0)

        # --- pre-declared reaction thresholds (frozen before final testing) ---
        # Measured from 3 obstacle-free nominal runs (validation/stage4e_nominal.json):
        # commanded cruise 0.480 m/s (min 0.446), |w| <= 0.094 rad/s, lateral
        # deviation from the corridor <= 0.032 m, crossing reached at t=6.00 s.
        # Each threshold sits ~3x outside the nominal envelope so it cannot be
        # tripped by ordinary corridor following.
        p('nominal_speed', 0.48)           # m/s, measured from nominal runs
        p('speed_reaction_fraction', 0.60) # R1: v_cmd < f * nominal_speed (0.288)
        p('reaction_sustain_s', 0.25)
        p('yaw_reaction_threshold', 0.30)  # R2: |w_cmd| rad/s (nominal max 0.094)
        p('lateral_reaction_threshold', 0.10)   # R3: |y - corridor_y| m (nominal max 0.032)
        p('stop_speed_threshold', 0.05)    # R4: v_cmd below this == stopped
        p('crossing_zone_radius', 0.35)    # m, "at the crossing point"

        p('settle_timeout', 180.0)
        p('nav_timeout', 90.0)
        p('record_hz', 20.0)
        p('post_goal_record_s', 1.0)

        self.P = {n: self.get_parameter(n).value for n in [
            'trial_id', 'mode', 'scenario', 'out_path', 'start_x', 'start_y', 'start_yaw',
            'goal_x', 'goal_y', 'goal_yaw', 'cross_x', 'cross_y', 'obstacle_start_y',
            'obstacle_stop_y', 'obstacle_speed', 'obstacle_trigger_delay', 'obstacle_dir',
            'policy_max_prediction_horizon', 'policy_sigma_level', 'policy_temporal_decay',
            'policy_max_cost', 'policy_min_cost', 'policy_max_influence_radius',
            'nominal_speed', 'speed_reaction_fraction', 'reaction_sustain_s',
            'yaw_reaction_threshold', 'lateral_reaction_threshold', 'stop_speed_threshold',
            'crossing_zone_radius', 'settle_timeout', 'nav_timeout', 'record_hz',
            'post_goal_record_s']}

        # --- live state ---
        self.odom = None            # robot simulator odometry (odom frame)
        self.obs_odom = None        # obstacle ground truth (gz world == map)
        self.cmd_nav = None         # controller_server output  (/cmd_vel_nav)
        self.cmd_out = None         # post collision-monitor    (/cmd_vel)
        self.cmd_nav_stamp = 0.0
        self.tracks = None
        self.tracks_stamp = 0.0
        self.debug_grid = None
        self.debug_grid_seq = 0
        self.last_debug_seq_used = -1
        self.master = None
        self.master_seq = 0
        self.last_master_seq_used = -1
        self.plans = []
        self.plan_count = 0
        self.cm_state = None
        self.cm_nonzero_count = 0

        self.samples = []
        self.goal_t0 = None
        self.nav_status = 'NOT_STARTED'
        self.nav_time = None

        qos_sensor = QoSProfile(depth=10)
        qos_sensor.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_tl = QoSProfile(depth=1)
        qos_tl.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos_tl.reliability = QoSReliabilityPolicy.RELIABLE

        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_sensor)
        self.create_subscription(Odometry, '/model/dynamic_obstacle/odometry',
                                 self._obs_cb, qos_sensor)
        self.create_subscription(Twist, '/cmd_vel_nav', self._cmd_nav_cb, 10)
        self.create_subscription(Twist, '/cmd_vel', self._cmd_out_cb, 10)
        self.create_subscription(TrackedObjectArray, '/tracked_objects',
                                 self._tracks_cb, 10)
        self.create_subscription(
            OccupancyGrid, f'/local_costmap/{LAYER}/debug_costmap',
            self._debug_cb, qos_tl)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        # Raw master local costmap (true 0-255 costs, not the 0-100 rescaled
        # OccupancyGrid). This is what MPPI's CostCritic actually reads, so it
        # is the only place the question "did the predictive layer change what
        # the controller sees?" can be answered.
        self.create_subscription(Costmap, '/local_costmap/costmap_raw',
                                 self._master_cb, 1)

        self.obs_cmd_pub = self.create_publisher(
            Twist, '/model/dynamic_obstacle/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.param_client = self.create_client(
            SetParameters, f'{LOCAL_COSTMAP_NODE}/set_parameters')

        try:
            from nav2_msgs.msg import CollisionMonitorState
            self.create_subscription(CollisionMonitorState,
                                     '/collision_monitor_state', self._cm_cb, 10)
        except Exception:
            self.get_logger().warn('CollisionMonitorState unavailable; skipping')

    # ---------------- callbacks ----------------
    def _odom_cb(self, m):
        self.odom = m

    def _obs_cb(self, m):
        self.obs_odom = m

    def _cmd_nav_cb(self, m):
        self.cmd_nav = m
        self.cmd_nav_stamp = time.monotonic()

    def _cmd_out_cb(self, m):
        self.cmd_out = m

    def _tracks_cb(self, m):
        self.tracks = m
        self.tracks_stamp = time.monotonic()

    def _debug_cb(self, m):
        self.debug_grid = m
        self.debug_grid_seq += 1

    def _master_cb(self, m):
        self.master = m
        self.master_seq += 1

    def _plan_cb(self, m):
        self.plan_count += 1
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in m.poses]
        self.plans.append((time.monotonic(), pts))

    def _cm_cb(self, m):
        self.cm_state = m
        if getattr(m, 'action_type', 0) != 0:
            self.cm_nonzero_count += 1

    # ---------------- helpers ----------------
    def robot_map_pose(self):
        """Ground-truth robot pose in the map frame.

        The simulator's odometry is exact and its frame is anchored at the
        known spawn pose (yaw 0), so this is ground truth, not AMCL's estimate.
        Used for evaluation only.
        """
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        yaw = yaw_of(self.odom.pose.pose.orientation)
        # The simulator anchors the odom frame at the spawn pose INCLUDING its
        # orientation, so odom->map is a rotation by start_yaw, not just a
        # translation. Getting this wrong silently mirrors the trajectory for
        # any route that does not start facing +x.
        y0 = self.P['start_yaw']
        c, sn = math.cos(y0), math.sin(y0)
        return (self.P['start_x'] + c * p.x - sn * p.y,
                self.P['start_y'] + sn * p.x + c * p.y,
                y0 + yaw)

    def obstacle_map_pose(self):
        if self.obs_odom is None:
            return None
        p = self.obs_odom.pose.pose.position
        return (p.x, p.y)

    def predictive_cells(self, rp=None):
        """Summarise the layer's own debug grid, in the map frame via TF.

        Returns (n_cells, peak, near_crossing, peak_near_crossing,
                 peak_ahead, cells_ahead, frac_window).

        `*_ahead` samples only the band the MPPI controller can actually reach
        within its own 2.8 s / ~1.4 m horizon (0.2-1.4 m in front of the robot,
        +-0.15 m laterally -- the strip a forward trajectory's sampled centre
        points occupy; a wider band would straddle the pillars bounding the
        0.70 m corridor and saturate on static cost). That is the diagnostic that matters: cost painted
        outside the reachable set cannot change the controller's choice, and
        cost painted UNIFORMLY across the whole window adds the same constant
        to every sampled trajectory, which MPPI's softmax cancels exactly.
        `frac_window` therefore detects the "footprint too broad" failure mode.
        """
        g = self.debug_grid
        empty = (0, 0, 0, 0, 0, 0, 0.0)
        if g is None:
            return empty
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', g.header.frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except Exception:
            return empty
        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        th = yaw_of(tf.transform.rotation)
        c, s = math.cos(th), math.sin(th)

        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        cx, cy = self.P['cross_x'], self.P['cross_y']
        rz = self.P['crossing_zone_radius']
        w = g.info.width
        total = len(g.data)

        rx = ry = ryaw = None
        if rp is not None:
            rx, ry, ryaw = rp
            rc, rs = math.cos(-ryaw), math.sin(-ryaw)

        n = peak = near = peak_near = peak_ahead = cells_ahead = 0
        for i, v in enumerate(g.data):
            if v <= 0:
                continue
            n += 1
            if v > peak:
                peak = v
            lx = ox + (i % w + 0.5) * res
            ly = oy + (i // w + 0.5) * res
            mx = c * lx - s * ly + dx
            my = s * lx + c * ly + dy
            if (mx - cx) ** 2 + (my - cy) ** 2 <= rz * rz:
                near += 1
                if v > peak_near:
                    peak_near = v
            if rx is not None:
                bx = rc * (mx - rx) - rs * (my - ry)
                by = rs * (mx - rx) + rc * (my - ry)
                if 0.2 <= bx <= 1.4 and abs(by) <= 0.15:
                    cells_ahead += 1
                    if v > peak_ahead:
                        peak_ahead = v
        return (n, peak, near, peak_near, peak_ahead, cells_ahead,
                round(n / total, 4) if total else 0.0)

    def master_band(self, rp):
        """Master-costmap statistics inside the robot's MPPI-reachable band
        (0.2-1.4 m ahead, +-0.15 m lateral). Returns (peak, mean, n_ge_180).

        This is the ground truth for "did enabling the layer change what the
        controller reads?", because updateWithMax means the layer only matters
        where it EXCEEDS the cost the other layers already wrote there."""
        m = self.master
        if m is None or rp is None:
            return (0, 0.0, 0)
        info = m.metadata
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        w = info.size_x
        rx, ry, ryaw = rp
        # costmap_raw is published in the layer's own global frame (odom);
        # convert the robot's map pose into it with the same TF the layer uses.
        try:
            tf = self.tf_buffer.lookup_transform(
                'odom', 'map', rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except Exception:
            return (0, 0.0, 0)
        th = yaw_of(tf.transform.rotation)
        c, s_ = math.cos(th), math.sin(th)
        orx = c * rx - s_ * ry + tf.transform.translation.x
        ory = s_ * rx + c * ry + tf.transform.translation.y
        oryaw = ryaw + th
        rc, rs = math.cos(-oryaw), math.sin(-oryaw)

        peak = 0
        total = 0
        n = 0
        n_hi = 0
        for i, v in enumerate(m.data):
            if v == 255:          # NO_INFORMATION
                continue
            cxm = ox + (i % w + 0.5) * res
            cym = oy + (i // w + 0.5) * res
            bx = rc * (cxm - orx) - rs * (cym - ory)
            by = rs * (cxm - orx) + rc * (cym - ory)
            if 0.2 <= bx <= 1.4 and abs(by) <= 0.15:
                n += 1
                total += v
                if v > peak:
                    peak = v
                if v >= 180:
                    n_hi += 1
        return (peak, round(total / n, 1) if n else 0.0, n_hi)

    def set_layer_params(self):
        if not self.param_client.wait_for_service(timeout_sec=30.0):
            raise RuntimeError('local_costmap set_parameters unavailable')

        def dbl(name, v):
            return Parameter(name=f'{LAYER}.{name}', value=ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE, double_value=float(v)))

        def integer(name, v):
            return Parameter(name=f'{LAYER}.{name}', value=ParameterValue(
                type=ParameterType.PARAMETER_INTEGER, integer_value=int(v)))

        params = [
            dbl('max_prediction_horizon', self.P['policy_max_prediction_horizon']),
            dbl('sigma_level', self.P['policy_sigma_level']),
            dbl('temporal_decay', self.P['policy_temporal_decay']),
            integer('max_cost', self.P['policy_max_cost']),
            integer('min_cost', self.P['policy_min_cost']),
            dbl('max_influence_radius', self.P['policy_max_influence_radius']),
            Parameter(name=f'{LAYER}.enabled', value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL,
                bool_value=(self.P['mode'] == 'predictive'))),
        ]
        req = SetParameters.Request(parameters=params)
        fut = self.param_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if fut.result() is None:
            raise RuntimeError('set_parameters timed out')
        bad = [r.reason for r in fut.result().results if not r.successful]
        if bad:
            raise RuntimeError(f'set_parameters rejected: {bad}')
        self.get_logger().info(
            f"layer configured: enabled={self.P['mode'] == 'predictive'} "
            f"horizon={self.P['policy_max_prediction_horizon']} "
            f"sigma={self.P['policy_sigma_level']} max_cost={self.P['policy_max_cost']}")

    def command_obstacle(self, vy):
        t = Twist()
        t.linear.y = float(vy)
        self.obs_cmd_pub.publish(t)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---------------- phases ----------------
    def wait_ready(self):
        """Block until odometry, TF map->base_link, the tracker and (in
        predictive mode) the layer's debug grid are all live."""
        deadline = time.monotonic() + self.P['settle_timeout']
        need_obstacle = self.P['scenario'] != 'nominal'
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is None:
                continue
            if need_obstacle and self.obs_odom is None:
                continue
            if not self.tf_buffer.can_transform(
                    'map', 'base_link', rclpy.time.Time(),
                    timeout=Duration(seconds=0.05)):
                continue
            if self.tracks is None:
                continue
            if self.debug_grid is None:
                continue
            if not self.nav_client.wait_for_server(timeout_sec=0.5):
                continue
            # hold the obstacle at its park pose while we settle
            if need_obstacle:
                self.command_obstacle(0.0)
            return True
        return False

    def run(self):
        ok = self.wait_ready()
        if not ok:
            self.nav_status = 'SETUP_TIMEOUT'
            return
        self.set_layer_params()
        # let the layer's one-shot clear / first paint settle
        self.spin_for(2.0)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.P['goal_x'])
        goal.pose.pose.position.y = float(self.P['goal_y'])
        goal.pose.pose.orientation.z = math.sin(self.P['goal_yaw'] / 2.0)
        goal.pose.pose.orientation.w = math.cos(self.P['goal_yaw'] / 2.0)

        # bt_navigator advertises navigate_to_pose before its lifecycle
        # transition to `active` completes, so a goal sent the instant the
        # server appears is intermittently rejected. Retry rather than losing
        # the trial; the obstacle stays parked (zero velocity) meanwhile, so
        # retries do not perturb the scenario.
        gh = None
        for attempt in range(10):
            send_fut = self.nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_fut, timeout_sec=20.0)
            gh = send_fut.result()
            if gh is not None and gh.accepted:
                break
            self.get_logger().warn(f'goal not accepted (attempt {attempt + 1}); retrying')
            self.command_obstacle(0.0)
            self.spin_for(2.0)
            goal.pose.header.stamp = self.get_clock().now().to_msg()
        if gh is None or not gh.accepted:
            self.nav_status = 'GOAL_REJECTED'
            return

        self.goal_t0 = time.monotonic()
        result_fut = gh.get_result_async()

        period = 1.0 / self.P['record_hz']
        next_sample = self.goal_t0
        deadline = self.goal_t0 + self.P['nav_timeout']
        obstacle_running = False
        obstacle_done = False
        finished_at = None

        v_obs = self.P['obstacle_speed'] * self.P['obstacle_dir']
        y_stop = self.P['obstacle_stop_y']
        moving = self.P['scenario'] in ('crossing', 'noconflict')

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.005)
            now = time.monotonic()
            t = now - self.goal_t0

            # --- scripted obstacle (one-way, no reversal) ---
            if moving and not obstacle_done:
                if not obstacle_running and t >= self.P['obstacle_trigger_delay']:
                    obstacle_running = True
                    self.get_logger().info(f'obstacle released at t={t:.2f}s')
                if obstacle_running:
                    op = self.obstacle_map_pose()
                    if op is not None and (
                            (self.P['obstacle_dir'] > 0 and op[1] >= y_stop) or
                            (self.P['obstacle_dir'] < 0 and op[1] <= y_stop)):
                        obstacle_done = True
                        self.command_obstacle(0.0)
                    else:
                        self.command_obstacle(v_obs)
                else:
                    self.command_obstacle(0.0)

            # --- 20 Hz sampling ---
            if now >= next_sample:
                next_sample += period
                self.record(t)

            if result_fut.done() and finished_at is None:
                res = result_fut.result()
                code = res.status
                self.nav_status = {
                    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
                    GoalStatus.STATUS_ABORTED: 'ABORTED',
                    GoalStatus.STATUS_CANCELED: 'CANCELED',
                }.get(code, f'STATUS_{code}')
                self.nav_time = t
                finished_at = now
            if finished_at is not None and now - finished_at >= self.P['post_goal_record_s']:
                break
            if now >= deadline:
                self.nav_status = 'TIMEOUT'
                self.nav_time = t
                break

        if moving:
            self.command_obstacle(0.0)
            self.spin_for(0.3)
            self.command_obstacle(0.0)

    def record(self, t):
        rp = self.robot_map_pose()
        op = self.obstacle_map_pose()
        if rp is None:
            return
        cn = self.cmd_nav
        co = self.cmd_out
        # A stale /cmd_vel_nav (controller silent) is recorded as zero rather
        # than as the last value, so "stopped" is not hidden by a held sample.
        fresh = (time.monotonic() - self.cmd_nav_stamp) < 0.5
        v_nav = (cn.linear.x if (cn is not None and fresh) else 0.0)
        w_nav = (cn.angular.z if (cn is not None and fresh) else 0.0)

        if self.master_seq != self.last_master_seq_used:
            self.last_master_seq_used = self.master_seq
            self._cached_master = self.master_band(rp)
        m_peak, m_mean, m_hi = getattr(self, '_cached_master', (0, 0.0, 0))

        if self.debug_grid_seq != self.last_debug_seq_used:
            self.last_debug_seq_used = self.debug_grid_seq
            self._cached_pred = self.predictive_cells(rp)
        (npred, peak, near, peak_near, peak_ahead, cells_ahead,
         frac_win) = getattr(self, '_cached_pred', (0, 0, 0, 0, 0, 0, 0.0))

        # Tracker state: nearest track to the obstacle's true position, so the
        # LiDAR-surface-vs-centre bias does not pick the wrong track. Ground
        # truth is used ONLY to select which track to log, never fed back.
        ntr = trk_speed = trk_x = trk_y = pfx = pfy = None
        trk_kf = False
        ntr = len(self.tracks.tracks) if self.tracks else 0
        if self.tracks and self.tracks.tracks and op is not None:
            tk = min(self.tracks.tracks,
                     key=lambda k: (k.position.x - op[0]) ** 2 + (k.position.y - op[1]) ** 2)
            trk_x, trk_y = round(tk.position.x, 4), round(tk.position.y, 4)
            trk_speed = round(tk.speed, 4)
            trk_kf = bool(tk.kalman_initialized)
            if tk.predictions:
                far = tk.predictions[-1]
                pfx = round(far.position.x, 4)
                pfy = round(far.position.y, 4)

        sep = None
        clr = None
        if op is not None:
            sep = math.hypot(rp[0] - op[0], rp[1] - op[1])
            clr = footprint_clearance(rp[0], rp[1], rp[2], op[0], op[1])

        self.samples.append({
            't': round(t, 4),
            'rx': round(rp[0], 4), 'ry': round(rp[1], 4), 'ryaw': round(rp[2], 4),
            'ox': round(op[0], 4) if op else None,
            'oy': round(op[1], 4) if op else None,
            'v_nav': round(v_nav, 4), 'w_nav': round(w_nav, 4),
            'v_out': round(co.linear.x, 4) if co else 0.0,
            'w_out': round(co.angular.z, 4) if co else 0.0,
            'v_odom': round(self.odom.twist.twist.linear.x, 4),
            'sep': round(sep, 4) if sep else None,
            'clr': round(clr, 4) if clr is not None else None,
            'n_tracks': ntr, 'trk_speed': trk_speed,
            'trk_x': trk_x, 'trk_y': trk_y, 'trk_kf': trk_kf,
            'trk_pred_far_x': pfx, 'trk_pred_far_y': pfy,
            'pred_cells': npred, 'pred_peak': peak, 'pred_near_cross': near,
            'pred_peak_near_cross': peak_near, 'pred_peak_ahead': peak_ahead,
            'pred_cells_ahead': cells_ahead, 'pred_frac_window': frac_win,
            'mst_peak_ahead': m_peak, 'mst_mean_ahead': m_mean,
            'mst_hi_ahead': m_hi,
            'plan_count': self.plan_count,
        })

    # ---------------- scoring ----------------
    def score(self):
        S = self.samples
        P = self.P
        cx, cy = P['cross_x'], P['cross_y']
        out = {}

        if not S:
            return {'error': 'no samples'}

        def first_sustained(pred, sustain):
            """First t where pred() holds continuously for `sustain` seconds."""
            start = None
            for s in S:
                if pred(s):
                    if start is None:
                        start = s['t']
                    elif s['t'] - start >= sustain:
                        return start
                else:
                    start = None
            return None

        sustain = P['reaction_sustain_s']
        v_thresh = P['speed_reaction_fraction'] * P['nominal_speed']

        # Scoring only opens once the robot has actually reached cruise. The
        # first ~1.3 s after goal acceptance is the startup acceleration ramp,
        # during which v_cmd is legitimately below the reaction threshold in
        # EVERY run including the obstacle-free nominal ones; counting that as
        # a "reaction" would report t~0 for every trial in both arms.
        t_cruise = next((s['t'] for s in S
                         if s['v_nav'] >= 0.8 * P['nominal_speed']), None)
        out['cruise_established_t'] = t_cruise
        if t_cruise is None:
            S = []
        else:
            S = [s for s in S if s['t'] >= t_cruise]
        if not S:
            S = self.samples   # degenerate: never reached cruise
            out['cruise_established_t'] = None

        # Reaction events are only meaningful while the robot is still
        # APPROACHING the crossing: a slowdown after passing it is goal
        # deceleration, not avoidance. Signed so the test is independent of
        # which way along the corridor the route runs.
        travel = 1.0 if P['goal_x'] >= P['start_x'] else -1.0

        def approaching(s):
            return travel * (s['rx'] - cx) < 0.0

        r1 = first_sustained(
            lambda s: approaching(s) and s['v_nav'] < v_thresh, sustain)
        r2 = first_sustained(
            lambda s: approaching(s) and abs(s['w_nav']) > P['yaw_reaction_threshold'],
            sustain)
        r3 = first_sustained(
            lambda s: approaching(s) and abs(s['ry'] - cy) > P['lateral_reaction_threshold'],
            sustain)
        r4 = first_sustained(
            lambda s: approaching(s) and s['v_nav'] < P['stop_speed_threshold'], sustain)

        def at(t):
            if t is None:
                return None
            best = min(S, key=lambda s: abs(s['t'] - t))
            d_cross_robot = math.hypot(best['rx'] - cx, best['ry'] - cy)
            d_cross_obs = (math.hypot(best['ox'] - cx, best['oy'] - cy)
                           if best['ox'] is not None else None)
            return {
                't': round(t, 3),
                'robot': [best['rx'], best['ry']],
                'obstacle': ([best['ox'], best['oy']]
                             if best['ox'] is not None else None),
                'robot_dist_to_crossing': round(d_cross_robot, 3),
                'obstacle_dist_to_crossing': (round(d_cross_obs, 3)
                                              if d_cross_obs is not None else None),
                'separation': best['sep'],
                'v_nav': best['v_nav'],
            }

        out['reactions'] = {
            'R1_speed_reduction': at(r1),
            'R2_angular': at(r2),
            'R3_lateral_deviation': at(r3),
            'R4_stop': at(r4),
        }
        out['primary_reaction_time'] = r1

        # --- advance-warning evidence (criterion 4) ---
        rz = P['crossing_zone_radius']
        t_pred_cross = next((s['t'] for s in S if s['pred_near_cross'] > 0), None)
        t_pred_any = next((s['t'] for s in S if s['pred_cells'] > 0), None)
        t_obs_cross = next(
            (s['t'] for s in S
             if s['ox'] is not None and math.hypot(s['ox'] - cx, s['oy'] - cy) <= rz),
            None)
        out['advance_warning'] = {
            'first_predictive_cost_t': t_pred_any,
            'first_predictive_cost_at_crossing_t': t_pred_cross,
            'first_obstacle_at_crossing_t': t_obs_cross,
            'lead_s': (round(t_obs_cross - t_pred_cross, 3)
                       if (t_pred_cross is not None and t_obs_cross is not None)
                       else None),
        }

        # --- safety / performance ---
        seps = [s['sep'] for s in S if s['sep'] is not None]
        clrs = [s['clr'] for s in S if s.get('clr') is not None]
        min_sep = min(seps) if seps else None
        min_clr = min(clrs) if clrs else None
        out['min_center_separation'] = round(min_sep, 4) if min_sep else None
        # Headline safety metric: true body-to-body gap.
        out['min_clearance'] = round(min_clr, 4) if min_clr is not None else None
        out['collision'] = bool(min_clr is not None and min_clr <= 0.0)
        # Secondary, deliberately pessimistic: Nav2's own circumscribed
        # planning radius. Reported so a conservative reader can see how close
        # the encounter came to violating the planner's safety model.
        out['min_planning_radius_margin'] = (
            round(min_sep - ROBOT_RADIUS - OBSTACLE_RADIUS, 4)
            if min_sep is not None else None)

        path_len = 0.0
        for a, b in zip(S, S[1:]):
            path_len += math.hypot(b['rx'] - a['rx'], b['ry'] - a['ry'])
        out['path_length'] = round(path_len, 4)

        # stops (on the controller's own command, before the collision monitor)
        stops = []
        in_stop = None
        for s in S:
            if s['v_nav'] < P['stop_speed_threshold']:
                if in_stop is None:
                    in_stop = s['t']
            else:
                if in_stop is not None and s['t'] - in_stop >= sustain:
                    stops.append([round(in_stop, 3), round(s['t'] - in_stop, 3)])
                in_stop = None
        if in_stop is not None and S[-1]['t'] - in_stop >= sustain:
            stops.append([round(in_stop, 3), round(S[-1]['t'] - in_stop, 3)])
        out['stops'] = stops
        out['n_stops'] = len(stops)
        out['stop_duration_total'] = round(sum(d for _, d in stops), 3)

        approach = [s for s in S if approaching(s)]
        out['min_v_nav_approach'] = (round(min(s['v_nav'] for s in approach), 4)
                                     if approach else None)
        out['mean_v_nav_approach'] = (
            round(sum(s['v_nav'] for s in approach) / len(approach), 4)
            if approach else None)
        out['max_abs_w_nav_approach'] = (
            round(max(abs(s['w_nav']) for s in approach), 4) if approach else None)
        out['max_lateral_dev_approach'] = (
            round(max(abs(s['ry'] - cy) for s in approach), 4) if approach else None)
        # command variation: mean |dv/dt| of the controller's linear command
        dv = [abs(b['v_nav'] - a['v_nav']) for a, b in zip(S, S[1:])]
        out['cmd_variation'] = round(sum(dv) / len(dv), 5) if dv else None

        out['max_pred_cells'] = max(s['pred_cells'] for s in S)
        out['max_pred_frac_window'] = max(s.get('pred_frac_window', 0.0) for s in S)
        out['mean_pred_frac_window'] = round(
            sum(s.get('pred_frac_window', 0.0) for s in S) / len(S), 4)
        out['max_pred_peak_near_cross'] = max(
            s.get('pred_peak_near_cross', 0) for s in S)
        out['max_pred_peak_ahead'] = max(s.get('pred_peak_ahead', 0) for s in S)
        t_ahead = next((s['t'] for s in S if s.get('pred_peak_ahead', 0) > 0), None)
        out['first_predictive_cost_ahead_t'] = t_ahead
        # What the controller actually saw in its reachable band.
        out['max_master_peak_ahead'] = max(s.get('mst_peak_ahead', 0) for s in S)
        out['max_master_hi_ahead'] = max(s.get('mst_hi_ahead', 0) for s in S)
        appr = [s for s in S if approaching(s)]
        out['mean_master_mean_ahead_approach'] = (
            round(sum(s.get('mst_mean_ahead', 0.0) for s in appr) / len(appr), 1)
            if appr else None)
        t_hi = next((s['t'] for s in S if s.get('mst_hi_ahead', 0) >= 20), None)
        out['first_master_hi_ahead_t'] = t_hi
        out['max_pred_peak'] = max(s['pred_peak'] for s in S)
        out['max_pred_near_cross'] = max(s['pred_near_cross'] for s in S)
        out['frames_with_pred_cost'] = sum(1 for s in S if s['pred_cells'] > 0)
        out['max_tracks'] = max(s['n_tracks'] or 0 for s in S)
        out['frames_tracked'] = sum(1 for s in S if s['n_tracks'])
        out['frames_kf_converged'] = sum(1 for s in S if s.get('trk_kf'))
        spd = [s['trk_speed'] for s in S
               if s.get('trk_kf') and s.get('trk_speed') is not None]
        out['tracked_speed_mean'] = round(sum(spd) / len(spd), 4) if spd else None
        out['tracked_speed_max'] = round(max(spd), 4) if spd else None

        out['plan_publications'] = self.plan_count
        out['global_path_changes'] = self.count_path_changes()
        out['collision_monitor_interventions'] = self.cm_nonzero_count

        out['nav_status'] = self.nav_status
        out['nav_time'] = round(self.nav_time, 3) if self.nav_time else None
        out['success'] = (self.nav_status == 'SUCCEEDED')
        return out

    @staticmethod
    def _curve_deviation(a, b):
        """Max distance from any point of curve `a` to the polyline `b`.

        Index-matched comparison is wrong here: as the robot advances the plan
        gets shorter, so matched indices drift along the route and report a
        large "change" even when the route is geometrically identical. Nearest
        point-to-segment distance is invariant to that re-parameterisation.
        """
        worst = 0.0
        for px, py in a:
            best = float('inf')
            for (x1, y1), (x2, y2) in zip(b, b[1:]):
                vx, vy = x2 - x1, y2 - y1
                L2 = vx * vx + vy * vy
                t = 0.0 if L2 == 0.0 else max(
                    0.0, min(1.0, ((px - x1) * vx + (py - y1) * vy) / L2))
                best = min(best, math.hypot(px - (x1 + t * vx), py - (y1 + t * vy)))
            worst = max(worst, best)
        return worst

    def count_path_changes(self):
        """Count GLOBAL plans that are a geometrically different ROUTE from
        their predecessor.

        Deliberately NOT called 'replans': the BT re-invokes the planner at a
        fixed rate, so nearly every publication is a fresh plan of the SAME
        route. Only a >0.15 m geometric deviation of the new plan from the old
        one (nearest-point, so shortening does not count) is a route change.
        """
        changes = 0
        prev = None
        for _, pts in self.plans:
            if len(pts) < 2:
                continue
            if prev is not None:
                # subsample for cost; 0.15 m threshold >> 0.05 m plan spacing
                a = pts[::5] or pts
                if self._curve_deviation(a, prev) > 0.15:
                    changes += 1
            prev = pts
        return changes


def main():
    rclpy.init()
    node = Stage4ETrial()
    err = None
    try:
        node.run()
    except Exception as exc:  # noqa: BLE001 - recorded into the result file
        err = f'{type(exc).__name__}: {exc}'
        node.get_logger().error(err)

    try:
        metrics = node.score()
    except Exception as exc:  # noqa: BLE001
        metrics = {'error': f'scoring failed: {exc}'}

    result = {
        'trial_id': node.P['trial_id'],
        'mode': node.P['mode'],
        'scenario': node.P['scenario'],
        'wall_time': time.strftime('%Y-%m-%dT%H:%M:%S'),
        'error': err,
        'config': {k: node.P[k] for k in node.P if k not in ('out_path',)},
        'metrics': metrics,
    }
    out = node.P['out_path']
    if out:
        os.makedirs(os.path.dirname(out), exist_ok=True)
        with open(out, 'w') as f:
            json.dump(result, f, indent=1)
        series_path = out.replace('.json', '_series.json')
        with open(series_path, 'w') as f:
            json.dump({'trial_id': node.P['trial_id'], 'mode': node.P['mode'],
                       'samples': node.samples}, f)
        node.get_logger().info(f'wrote {out}')
    print(json.dumps({'trial': node.P['trial_id'], 'mode': node.P['mode'],
                      'status': node.nav_status,
                      'metrics': {k: metrics.get(k) for k in
                                  ('success', 'collision', 'min_clearance',
                                   'nav_time', 'primary_reaction_time',
                                   'max_pred_cells')}}, indent=1))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
