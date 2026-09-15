#!/usr/bin/env python3
"""Gazebo Stage-4G2/4G4 trials. GT stays in this simulation/evaluation process.

Stage-4G4 extends this runner ADDITIVELY, so every Stage-4G2/4G3 scenario and
result reproduces unchanged:
  * scenario objects may carry an optional "accel" (m/s^2). Without it a
    "segments" boundary is a STEP velocity change, which is what Stage-4G2/4G3
    used and what the measured GT shows (|a| 5-9.5 m/s^2 at a boundary). With
    it the commanded velocity RAMPS toward the segment target at that bound, so
    the obstacle's dynamics are physically bounded and known -- the premise a
    reachable set needs.
  * the recorded tracker frames additionally carry the new
    reachability_predictions field.
  * --layer-mode selects the predictive costmap arm (reactive / cv_covariance /
    reachability) by setting live parameters, the same way Stage-4E switched
    its A/B arms. Default "keep" touches nothing.

No global pkill: each subprocess has its own process group, and only groups
created by this runner are stopped. Each trial uses an isolated DDS domain
and Gazebo partition. Existing Stage-4F configuration is loaded unchanged.
Raw evidence is exclusive-created in the requested output directory.
"""
import argparse
import hashlib
import json
import math
import os
import random
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def seconds(s):
    return s.sec + s.nanosec * 1e-9


def group_alive(pgid):
    # A launch leader can exit while Nav2 children are still shutting down.
    # Zombies require their parent to reap them, not another signal.
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            fields = (entry/'stat').read_text().rsplit(')',1)[1].split()
            if int(fields[2]) == pgid and fields[0] != 'Z':
                return True
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return False


def stop(p):
    if p is None:
        return
    for sig, timeout in [(signal.SIGINT, 8), (signal.SIGTERM, 3), (signal.SIGKILL, 2)]:
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            p.poll()  # reap the leader, but inspect the whole owned group
            if not group_alive(p.pid):
                return
            time.sleep(.05)
    raise RuntimeError(f'Owned process group {p.pid} survived SIGKILL')


def run(args, scenarios):
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    sequence = 0
    for name in args.scenarios.split(','):
        for trial in range(1, args.trials + 1):
            trial_dir = out / f'{name}_{trial:02d}'
            trial_dir.mkdir()  # never silently overwrite baseline evidence
            sequence += 1
            # Deterministic per-trial randomisation: the seed is derived from the
            # base seed and the trial index, and is recorded in provenance, so a
            # jittered trial is exactly reproducible.
            trial_seed = args.seed + sequence
            scenario_def = json.loads(json.dumps(scenarios[name]))
            jitter_applied = None
            if args.jitter > 0:
                rng = random.Random(trial_seed)
                j = args.jitter
                jitter_applied = []
                for obj in scenario_def.get('objects', []):
                    dx = rng.uniform(-0.15, 0.15) * j
                    dy = rng.uniform(-0.15, 0.15) * j
                    ds = 1.0 + rng.uniform(-0.12, 0.12) * j
                    dt = rng.uniform(-0.8, 0.8) * j
                    obj['start'] = [obj['start'][0] + dx, obj['start'][1] + dy]
                    if 'velocity' in obj:
                        obj['velocity'] = [v * ds for v in obj['velocity']]
                    if 'segments' in obj:
                        obj['segments'] = [[d, vx * ds, vy * ds]
                                           for d, vx, vy in obj['segments']]
                    # A positive dt delays the object by holding it still first.
                    if dt > 0:
                        obj.setdefault('segments', [[scenario_def['duration'],
                                                     *obj.get('velocity', [0, 0])]])
                        obj['segments'] = [[dt, 0.0, 0.0]] + obj['segments']
                        obj.pop('velocity', None)
                    jitter_applied.append(dict(dx=round(dx, 4), dy=round(dy, 4),
                                               speed_scale=round(ds, 4),
                                               delay_s=round(max(dt, 0.0), 4)))
            env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain + sequence),
                       GZ_PARTITION=f'stage4g2_{os.getpid()}_{name}_{trial}',
                       ROS_LOG_DIR=str(trial_dir / 'ros_logs'))
            provenance = {
                'head': subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                'stage4f_reference': subprocess.check_output(['git','rev-parse','stage4f-validated^{commit}'],cwd=ROOT,text=True).strip(),
                'definition': scenario_def, 'layer_mode': args.layer_mode,
                'production': bool(args.production), 'scan_noise_sigma': args.scan_noise,
                'pose_offset': [args.pose_dx, args.pose_dy, args.pose_dyaw],
                'jitter': args.jitter, 'trial_seed': trial_seed,
                'jitter_applied': jitter_applied,
                'world': scenarios[name].get('world', 'stage4f_benchmark'),
                'ros_domain_id': env['ROS_DOMAIN_ID'], 'gz_partition': env['GZ_PARTITION'],
                'sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [Path(__file__).resolve(), HERE/'score.py',
                        ROOT/'src/predictive_nav_tracking/src/lidar_obstacle_tracker_node.cpp',
                        ROOT/'src/predictive_nav_tracking/config/tracker_params.yaml']}}
            (trial_dir/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
            # The worker reads the (possibly jittered) definition from the trial
            # directory, so what ran is exactly what was recorded.
            (trial_dir/'definition.json').write_text(
                json.dumps({name: scenario_def}, indent=1)+'\n')
            print(f'START {trial_dir.name}', flush=True)
            launch = None
            worker = None
            noise = None
            try:
                with (trial_dir / 'launch.log').open('w') as log:
                    # Stage-4G7: a scenario may name its own static world. The
                    # world and its map are always taken as a PAIR from one
                    # generator, so they cannot describe different rooms.
                    launch_args = [
                        'ros2', 'launch', 'predictive_nav_bringup',
                        'stage4f_bringup.launch.py', 'spawn_obstacle:=False',
                        'run_trial:=False', 'headless:=True', 'use_rviz:=False']
                    if args.production:
                        launch_args += [
                            f'params_file:={ROOT}/src/predictive_nav_bringup/config/'
                            'production_nav2_params.yaml',
                            f'tracker_params:={ROOT}/src/predictive_nav_tracking/config/'
                            'production_tracker_params.yaml']
                    if args.scan_noise > 0:
                        launch_args += ['tracker_scan_topic:=/scan_noisy']
                    if args.pose_dx or args.pose_dy or args.pose_dyaw:
                        launch_args += [f'initialpose_dx:={args.pose_dx}',
                                        f'initialpose_dy:={args.pose_dy}',
                                        f'initialpose_dyaw:={args.pose_dyaw}']
                    world = scenarios[name].get('world')
                    if world:
                        launch_args += [
                            f'world:={ROOT}/src/predictive_nav_bringup/worlds/{world}.sdf.xacro',
                            f'map:={ROOT}/src/predictive_nav_bringup/maps/{world}.yaml']
                    launch = subprocess.Popen(
                        launch_args, env=env, stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True)
                    if args.scan_noise > 0:
                        noise = subprocess.Popen([
                            sys.executable,
                            str(ROOT / 'src/predictive_nav_bringup/scripts/stage4g8/'
                                       'scan_noise.py'),
                            '--sigma', str(args.scan_noise), '--seed', str(trial_seed)],
                            env=env, stdout=log, stderr=subprocess.STDOUT,
                            start_new_session=True)
                    with (trial_dir / 'worker.log').open('w') as wlog:
                        worker = subprocess.Popen([
                            sys.executable, str(Path(__file__).resolve()),
                            '--worker', name, '--out', str(trial_dir),
                            '--definitions', str(trial_dir / 'definition.json'),
                            '--layer-mode', args.layer_mode], env=env,
                            stdout=wlog, stderr=subprocess.STDOUT, start_new_session=True)
                        worker.wait(timeout=300)
                        if worker.returncode:
                            raise RuntimeError((trial_dir / 'worker.log').read_text()[-4000:])
            except Exception as exc:
                # Record the failure IN the trial directory and continue. A
                # simulation/setup failure must never delete the rest of a
                # batch, and must stay visible to the analysis rather than
                # being silently dropped.
                (trial_dir / 'setup_failure.json').write_text(json.dumps(dict(
                    trial=trial_dir.name, scenario=name, layer_mode=args.layer_mode,
                    error=type(exc).__name__, detail=str(exc)[-2000:]), indent=2) + '\n')
                print(f'FAILED {trial_dir.name}: {type(exc).__name__}', flush=True)
                continue
            finally:
                stop(worker)
                stop(noise)
                stop(launch)
            subprocess.run([sys.executable, str(HERE / 'score.py'), str(trial_dir)], check=True)
            print(f'DONE {trial_dir.name}', flush=True)


def worker(args, scenario):
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from rclpy.time import Time
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from visualization_msgs.msg import MarkerArray
    from predictive_nav_msgs.msg import TrackedObjectArray
    from nav2_msgs.action import NavigateToPose
    from lifecycle_msgs.srv import GetState
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    import tf2_ros

    rclpy.init(args=[])
    n = Node('stage4g2_evaluation')
    n.set_parameters([rclpy.parameter.Parameter('use_sim_time', value=True)])
    out = Path(args.out)
    objects = scenario['objects']
    gt = [[] for _ in objects]
    arrays, clusters, scans, grids, odoms = [], {}, [], [], []
    pending_scans = []
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer, n)
    pubs, subs, bridges = [], [], []
    action = ActionClient(n, NavigateToPose, '/navigate_to_pose')
    t0 = None
    latest_tracks = None
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def odom_record(m):
        p, v = m.pose.pose.position, m.twist.twist.linear
        return [seconds(m.header.stamp), p.x, p.y, v.x, v.y]

    def on_tracks(m):
        nonlocal latest_tracks
        latest_tracks = m
        if t0 is None:
            return
        rows = []
        for tr in m.tracks:
            rows.append(dict(id=tr.id, p=[tr.position.x,tr.position.y],
                v=[tr.velocity.x,tr.velocity.y], raw=[tr.raw_position.x,tr.raw_position.y],
                stamp=seconds(tr.stamp), age=tr.age, observations=tr.observations,
                missed=tr.missed_count, covariance=list(tr.covariance),
                predictions=[dict(dt=p.time_from_now, stamp=seconds(p.stamp),
                    p=[p.position.x,p.position.y], covariance=list(p.position_covariance))
                    for p in tr.predictions],
                reachability=[dict(dt=r.time_from_now, stamp=seconds(r.stamp),
                    age=r.observation_age, total=r.total_time,
                    p=[r.position.x,r.position.y], v=[r.velocity.x,r.velocity.y],
                    reach_radius=r.reach_radius, capped=bool(r.speed_capped),
                    sigma=[r.sigma_semi_major,r.sigma_semi_minor,r.sigma_yaw],
                    k=r.covariance_sigma_level, margin=r.safety_margin,
                    axes=[r.semi_major,r.semi_minor], valid=bool(r.valid))
                    for r in tr.reachability_predictions]))
        arrays.append(dict(t=seconds(m.header.stamp), tracks=rows))

    def on_markers(m):
        if t0 is None:
            return
        for marker in m.markers:
            if marker.ns == 'candidate_clusters':
                clusters[f'{seconds(marker.header.stamp):.6f}'] = [[p.x,p.y] for p in marker.points]

    def on_scan(m):
        if t0 is not None:
            pending_scans.append(m)

    def process_scan(m):
        try:
            tr = buffer.lookup_transform('map', m.header.frame_id, Time.from_msg(m.header.stamp))
        except Exception:
            return False  # retry after the corresponding TF arrives
        q, p = tr.transform.rotation, tr.transform.translation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        a = m.angle_min + np.arange(len(m.ranges))*m.angle_increment + yaw
        r = np.array(m.ranges)
        ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max)
        points = np.column_stack((p.x+r[ok]*np.cos(a[ok]), p.y+r[ok]*np.sin(a[ok])))
        # Stage-4G4: the robot's own map-frame pose at the SAME stamp, so
        # footprint clearance can be measured against the real oriented
        # collision box rather than approximated from the sensor position.
        robot = None
        try:
            rb = buffer.lookup_transform('map', 'base_footprint', Time.from_msg(m.header.stamp))
            rq, rp = rb.transform.rotation, rb.transform.translation
            robot = [rp.x, rp.y,
                     math.atan2(2*(rq.w*rq.z+rq.x*rq.y), 1-2*(rq.y*rq.y+rq.z*rq.z))]
        except Exception:
            pass
        scans.append(dict(t=seconds(m.header.stamp), sensor=[p.x,p.y], robot=robot,
                          points=points.tolist()))
        return True

    def drain_scans():
        pending_scans[:] = [m for m in pending_scans if not process_scan(m)]

    scan_timer = n.create_timer(.1, drain_scans)

    def on_grid(m):
        if t0 is None:
            return
        # Keep compact positive-cell coordinates for actual layer evidence.
        cells = np.array(m.data).reshape(m.info.height, m.info.width)
        iy, ix = np.nonzero(cells > 0)
        represented = []
        eligible = []
        if latest_tracks:
            try:
                tr = buffer.lookup_transform(m.header.frame_id, 'map', Time())
                q, p = tr.transform.rotation, tr.transform.translation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                for track in latest_tracks.tracks:
                    inside = painted = False
                    for pr in track.predictions:
                        x = math.cos(yaw)*pr.position.x-math.sin(yaw)*pr.position.y+p.x
                        y = math.sin(yaw)*pr.position.x+math.cos(yaw)*pr.position.y+p.y
                        mx = int(math.floor((x-m.info.origin.position.x)/m.info.resolution))
                        my = int(math.floor((y-m.info.origin.position.y)/m.info.resolution))
                        if 0 <= mx < m.info.width and 0 <= my < m.info.height:
                            inside = True
                            # .15 m neighbourhood bounds scan/layer scheduling mismatch.
                            painted |= bool(np.any(cells[max(0,my-3):my+4,max(0,mx-3):mx+4]>0))
                    if inside: eligible.append(track.id)
                    if painted: represented.append(track.id)
            except Exception:
                pass
        grids.append(dict(t=seconds(m.header.stamp), frame=m.header.frame_id,
            eligible_ids=eligible, represented_ids=represented,
            origin=[m.info.origin.position.x,m.info.origin.position.y],
            resolution=m.info.resolution, size=[m.info.width,m.info.height],
            cells=np.column_stack((ix,iy,cells[iy,ix])).tolist()))

    subs += [n.create_subscription(TrackedObjectArray, '/tracked_objects', on_tracks, 20),
             n.create_subscription(MarkerArray, '/tracked_objects/markers', on_markers, 20),
             n.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data),
             n.create_subscription(OccupancyGrid,
                 '/local_costmap/predicted_obstacle_layer/debug_costmap', on_grid, qos),
             n.create_subscription(Odometry, '/odom', lambda m: odoms.append(odom_record(m)), 20)]

    def spin_until(condition, timeout=180):
        deadline = time.monotonic()+timeout
        while not condition():
            if time.monotonic() > deadline:
                raise TimeoutError('Stage-4G2 readiness timeout')
            rclpy.spin_once(n, timeout_sec=.05)

    def service(client, request, timeout=15):
        spin_until(lambda: client.service_is_ready(), timeout)
        f = client.call_async(request)
        spin_until(f.done, timeout)
        return f.result()

    lifecycle = n.create_client(GetState, '/controller_server/get_state')
    spin_until(lambda: action.server_is_ready() and latest_tracks is not None)
    spin_until(lambda: service(lifecycle, GetState.Request()).current_state.id == 3)
    param = n.create_client(SetParameters, '/lidar_obstacle_tracker/set_parameters')
    req = SetParameters.Request(parameters=[Parameter(name='profile_scans',
        value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))])
    assert all(x.successful for x in service(param, req).results)

    # Stage-4G4 navigation arms. One launch serves all three; only these live
    # parameters differ, so the controller, planner, costmap geometry, tracker
    # and scenario are provably identical across arms.
    layer_mode = getattr(args, 'layer_mode', 'keep')
    if layer_mode != 'keep':
        layer = n.create_client(SetParameters, '/local_costmap/local_costmap/set_parameters')
        params = [Parameter(name='predicted_obstacle_layer.enabled',
            value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                bool_value=layer_mode != 'reactive'))]
        if layer_mode != 'reactive':
            params.append(Parameter(name='predicted_obstacle_layer.prediction_mode',
                value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                    string_value=layer_mode)))
        results = service(layer, SetParameters.Request(parameters=params)).results
        bad = [r.reason for r in results if not r.successful]
        if bad:
            raise RuntimeError(f'predictive layer configuration rejected: {bad}')
        print(f'predictive layer arm: {layer_mode}', flush=True)
    print('Nav2 and tracker ready', flush=True)
    try:
        for i, obj in enumerate(objects):
            name = f'g2_obstacle_{i}'
            model = ET.parse(ROOT / 'src/predictive_nav_bringup/models/dynamic_obstacle.sdf')
            model.getroot().find('model').set('name', name)
            model.getroot().find('.//robot_base_frame').text = name
            path = out / f'{name}.sdf'
            model.write(path)
            log = (out / f'{name}.log').open('w')
            bridges.append(subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
                f'/model/{name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/model/{name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True))
            log.close()
            pubs.append(n.create_publisher(Twist, f'/model/{name}/cmd_vel', 10))
            subs.append(n.create_subscription(Odometry, f'/model/{name}/odometry',
                lambda m, i=i: gt[i].append(odom_record(m)), 100))
            result = subprocess.run(['ros2','run','ros_gz_sim','create','-file',str(path),
                '-name',name,'-x',str(obj['start'][0]),'-y',str(obj['start'][1]),'-z','0.3'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20, text=True)
            if result.returncode:
                raise RuntimeError(result.stdout)
        spin_until(lambda: all(len(g)>10 for g in gt))
        # Hold all models stationary until discovery, localization and confirmation settle.
        settle = n.get_clock().now().nanoseconds*1e-9
        spin_until(lambda: n.get_clock().now().nanoseconds*1e-9-settle >= 2.)
        t0 = n.get_clock().now().nanoseconds*1e-9
        result_future = None
        goal_handle = None
        if scenario.get('navigate'):
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = n.get_clock().now().to_msg()
            goal.pose.pose.position.x = 3.
            goal.pose.pose.orientation.w = 1.
            f = action.send_goal_async(goal)
            spin_until(f.done)
            goal_handle = f.result()
            assert goal_handle.accepted
            result_future = goal_handle.get_result_async()
        commanded = [(0.0, [0.0, 0.0]) for _ in objects]
        # The wall-clock guard must scale with the scenario: a fixed 120 s budget
        # aborts any scenario longer than that as a "stalled clock", which is what
        # happened to the Stage-4G8 150 s endurance run.
        wall_deadline = time.monotonic()+max(120, scenario['duration']*2.5)
        while n.get_clock().now().nanoseconds*1e-9-t0 < scenario['duration']:
            if time.monotonic()>wall_deadline:
                raise TimeoutError('Simulation clock stalled')
            elapsed = n.get_clock().now().nanoseconds*1e-9-t0
            for i,(pub,obj) in enumerate(zip(pubs,objects)):
                v = obj.get('velocity', [0,0])
                if 'segments' in obj:
                    remaining = elapsed
                    v = [0,0]
                    for duration,vx,vy in obj['segments']:
                        if remaining < duration:
                            v = [vx,vy]
                            break
                        remaining -= duration
                v = [float(v[0]), float(v[1])]
                accel = obj.get('accel')
                if accel:
                    # Ramp the COMMAND toward the segment target at a known
                    # bound instead of stepping. This is what makes the
                    # obstacle's acceleration a declared physical quantity;
                    # without it the VelocityControl plugin applies the target
                    # instantaneously. The ramp is integrated on the commanded
                    # value, not on odometry, so it stays deterministic.
                    prev_t, prev_v = commanded[i]
                    step = max(elapsed - prev_t, 0.0) * float(accel)
                    dx, dy = v[0]-prev_v[0], v[1]-prev_v[1]
                    gap = math.hypot(dx, dy)
                    if gap > step and gap > 0.0:
                        v = [prev_v[0] + dx*step/gap, prev_v[1] + dy*step/gap]
                commanded[i] = (elapsed, v)
                cmd = Twist()
                cmd.linear.x, cmd.linear.y = v
                pub.publish(cmd)
            rclpy.spin_once(n, timeout_sec=.025)
        for pub in pubs:
            pub.publish(Twist())
        end = n.get_clock().now().nanoseconds*1e-9
        spin_until(lambda: n.get_clock().now().nanoseconds*1e-9-end > .3)
        lifecycle_end = service(lifecycle, GetState.Request()).current_state.id
        nav = dict(requested=bool(scenario.get('navigate')), lifecycle_end=lifecycle_end,
            status=result_future.result().status if result_future and result_future.done() else None,
            robot_distance=math.hypot(odoms[-1][1]-odoms[0][1],odoms[-1][2]-odoms[0][2]) if odoms else 0)
        raw = dict(scenario=args.worker, definition=scenario, t0=t0, end=t0+scenario['duration'],
            layer_mode=layer_mode, world=scenario.get('world', 'stage4f_benchmark'),
            gt=gt, arrays=arrays, clusters=clusters, scans=scans, grids=grids, nav=nav)
        with (out/'raw.json').open('x') as f:
            json.dump(raw,f)
        print(f'Recorded {len(arrays)} tracker frames; navigation {nav}', flush=True)
    finally:
        for pub in pubs:
            pub.publish(Twist())
        for p in bridges:
            stop(p)
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out', required=True)
    p.add_argument('--definitions', default=str(HERE/'scenarios.json'))
    p.add_argument('--scenarios', default='single,parallel,opposing,crossing,crossing_resolved,near_crossing,occlusion')
    p.add_argument('--trials', type=int, default=1)
    p.add_argument('--domain', type=int, default=92)
    # --- Stage-4G8 robustness dimensions. All default to the validated
    # behaviour, so every earlier stage reproduces unchanged.
    p.add_argument('--production', action='store_true',
                   help='use the FINAL production configuration '
                        '(production_nav2_params.yaml + production_tracker_params.yaml)')
    p.add_argument('--scan-noise', type=float, default=0.0,
                   help='extra LiDAR range sigma (m) injected on the TRACKER input only')
    p.add_argument('--pose-dx', type=float, default=0.0)
    p.add_argument('--pose-dy', type=float, default=0.0)
    p.add_argument('--pose-dyaw', type=float, default=0.0,
                   help='controlled localisation error: the robot spawns at the true pose '
                        'but AMCL is initialised at true + (dx, dy, dyaw)')
    p.add_argument('--jitter', type=float, default=0.0,
                   help='per-trial random perturbation scale for obstacle start time, '
                        'position and speed; 0 disables. Seeded per trial and logged.')
    p.add_argument('--seed', type=int, default=4008,
                   help='base RNG seed; each trial uses seed + trial index')
    p.add_argument('--layer-mode', default='keep',
                   choices=['keep','reactive','cv_covariance','reachability','hybrid'],
                   help='predictive costmap arm; "keep" leaves the launch configuration alone')
    p.add_argument('--worker')
    args=p.parse_args()
    definitions=json.loads(Path(args.definitions).read_text())
    worker(args, definitions[args.worker]) if args.worker else run(args,definitions)
