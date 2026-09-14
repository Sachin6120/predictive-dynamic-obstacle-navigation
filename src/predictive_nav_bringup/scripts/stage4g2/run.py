#!/usr/bin/env python3
"""Gazebo Stage-4G2 trials. GT stays in this simulation/evaluation process.

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
            env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain + sequence),
                       GZ_PARTITION=f'stage4g2_{os.getpid()}_{name}_{trial}',
                       ROS_LOG_DIR=str(trial_dir / 'ros_logs'))
            provenance = {
                'head': subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
                'stage4f_reference': subprocess.check_output(['git','rev-parse','stage4f-validated^{commit}'],cwd=ROOT,text=True).strip(),
                'definition': scenarios[name],
                'ros_domain_id': env['ROS_DOMAIN_ID'], 'gz_partition': env['GZ_PARTITION'],
                'sha256': {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in [Path(__file__).resolve(), HERE/'score.py',
                        ROOT/'src/predictive_nav_tracking/src/lidar_obstacle_tracker_node.cpp',
                        ROOT/'src/predictive_nav_tracking/config/tracker_params.yaml']}}
            (trial_dir/'provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
            print(f'START {trial_dir.name}', flush=True)
            launch = None
            worker = None
            try:
                with (trial_dir / 'launch.log').open('w') as log:
                    launch = subprocess.Popen([
                        'ros2', 'launch', 'predictive_nav_bringup',
                        'stage4f_bringup.launch.py', 'spawn_obstacle:=False',
                        'run_trial:=False', 'headless:=True', 'use_rviz:=False'],
                        env=env, stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True)
                    with (trial_dir / 'worker.log').open('w') as wlog:
                        worker = subprocess.Popen([
                            sys.executable, str(Path(__file__).resolve()),
                            '--worker', name, '--out', str(trial_dir),
                            '--definitions', args.definitions], env=env,
                            stdout=wlog, stderr=subprocess.STDOUT, start_new_session=True)
                        worker.wait(timeout=240)
                        if worker.returncode:
                            raise RuntimeError((trial_dir / 'worker.log').read_text()[-4000:])
            finally:
                stop(worker)
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
                    for p in tr.predictions]))
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
        scans.append(dict(t=seconds(m.header.stamp), sensor=[p.x,p.y], points=points.tolist()))
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

    def spin_until(condition, timeout=100):
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
        wall_deadline = time.monotonic()+120
        while n.get_clock().now().nanoseconds*1e-9-t0 < scenario['duration']:
            if time.monotonic()>wall_deadline:
                raise TimeoutError('Simulation clock stalled')
            elapsed = n.get_clock().now().nanoseconds*1e-9-t0
            for pub,obj in zip(pubs,objects):
                v = obj.get('velocity', [0,0])
                if 'segments' in obj:
                    remaining = elapsed
                    v = [0,0]
                    for duration,vx,vy in obj['segments']:
                        if remaining < duration:
                            v = [vx,vy]
                            break
                        remaining -= duration
                cmd = Twist()
                cmd.linear.x, cmd.linear.y = map(float,v)
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
    p.add_argument('--worker')
    args=p.parse_args()
    definitions=json.loads(Path(args.definitions).read_text())
    worker(args, definitions[args.worker]) if args.worker else run(args,definitions)
