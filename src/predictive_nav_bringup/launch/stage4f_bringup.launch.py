"""Stage-4F bringup: one benchmark trial in the wide arena, either arm.

Same stack as Stage-4E (simulation + Nav2 + Stage-4B/4C tracker + the
predictive costmap layer) but pointed at:

  * worlds/stage4f_benchmark.sdf.xacro and maps/stage4f_benchmark.yaml -- the
    10 x 8 m arena, generated together by scripts/make_stage4f_world.py so the
    Gazebo world and the localisation map cannot drift apart;
  * config/nav2_stage4f_params.yaml -- Stage-4E's parameters with a 6 x 6 m
    local costmap, matched voxel sensing range, and min_cost 0.

The obstacle is described by a start point, a heading and a travel distance,
which covers the perpendicular, oblique, head-on and no-conflict classes with
one code path. It is spawned stationary and held there by the trial runner
until its release time (which may be negative, i.e. before the goal).

`mode` is passed through to the trial runner, which flips
`predicted_obstacle_layer.enabled`. Both arms load the identical params file
and the identical plugin set.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


ARG_NAMES = (
    'headless', 'use_rviz', 'mode', 'scenario', 'trial_id', 'out_path',
    'params_file', 'world', 'map', 'spawn_obstacle', 'run_trial',
    'start_x', 'start_y', 'start_yaw', 'goal_x', 'goal_y', 'goal_yaw',
    'cross_x', 'cross_y',
    'obstacle_start_x', 'obstacle_start_y', 'obstacle_speed',
    'obstacle_heading', 'obstacle_travel', 'obstacle_trigger_delay',
    'nominal_speed',
    'policy_max_prediction_horizon', 'policy_sigma_level',
    'policy_temporal_decay', 'policy_max_cost', 'policy_min_cost',
    'policy_max_influence_radius', 'nav_timeout',
)


def generate_launch_description():
    bringup_dir = get_package_share_directory('predictive_nav_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tracking_dir = get_package_share_directory('predictive_nav_tracking')

    a = {n: LaunchConfiguration(n) for n in ARG_NAMES}

    decls = [
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('mode', default_value='predictive',
                              description="'reactive' or 'predictive'"),
        DeclareLaunchArgument('scenario', default_value='perpendicular'),
        DeclareLaunchArgument('trial_id', default_value='trial'),
        DeclareLaunchArgument('out_path', default_value=''),
        DeclareLaunchArgument('run_trial', default_value='True'),
        DeclareLaunchArgument('spawn_obstacle', default_value='True'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config',
                                       'nav2_stage4f_params.yaml')),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(bringup_dir, 'worlds',
                                       'stage4f_benchmark.sdf.xacro')),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(bringup_dir, 'maps',
                                       'stage4f_benchmark.yaml')),
        # Route: straight along y = 0, 6 m, through the arena centre. The whole
        # route and a +-2.3 m band around it carry zero static inflation cost.
        DeclareLaunchArgument('start_x', default_value='-3.0'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        DeclareLaunchArgument('start_yaw', default_value='0.0'),
        DeclareLaunchArgument('goal_x', default_value='3.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0'),
        DeclareLaunchArgument('cross_x', default_value='0.0'),
        DeclareLaunchArgument('cross_y', default_value='0.0'),
        DeclareLaunchArgument('obstacle_start_x', default_value='0.0'),
        DeclareLaunchArgument('obstacle_start_y', default_value='-3.0'),
        DeclareLaunchArgument('obstacle_speed', default_value='0.50'),
        DeclareLaunchArgument('obstacle_heading', default_value='1.5707963'),
        DeclareLaunchArgument('obstacle_travel', default_value='5.0'),
        DeclareLaunchArgument('obstacle_trigger_delay', default_value='0.0'),
        DeclareLaunchArgument('nominal_speed', default_value='0.48'),
        # Frozen Stage-4F predictive policy (Stage-4E's, with min_cost 0).
        DeclareLaunchArgument('policy_max_prediction_horizon', default_value='3.0'),
        DeclareLaunchArgument('policy_sigma_level', default_value='1.5'),
        DeclareLaunchArgument('policy_temporal_decay', default_value='0.35'),
        DeclareLaunchArgument('policy_max_cost', default_value='250'),
        DeclareLaunchArgument('policy_min_cost', default_value='0'),
        DeclareLaunchArgument('policy_max_influence_radius', default_value='0.55'),
        DeclareLaunchArgument('nav_timeout', default_value='60.0'),
    ]

    tb3_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'headless': a['headless'],
            'use_rviz': 'False',
            'world': a['world'],
            'map': a['map'],
            'x_pose': a['start_x'],
            'y_pose': a['start_y'],
            'yaw': a['start_yaw'],
            'params_file': a['params_file'],
        }.items(),
    )

    initial_pose_publisher = Node(
        package='predictive_nav_bringup', executable='publish_initial_pose.py',
        name='initial_pose_publisher', output='screen',
        parameters=[{'x': a['start_x'], 'y': a['start_y'],
                     'yaw': a['start_yaw'], 'wait_timeout_sec': 60.0}],
    )

    spawn_dynamic_obstacle = Node(
        package='ros_gz_sim', executable='create', name='spawn_dynamic_obstacle',
        output='screen', condition=IfCondition(a['spawn_obstacle']),
        arguments=['-file', os.path.join(bringup_dir, 'models', 'dynamic_obstacle.sdf'),
                   '-name', 'dynamic_obstacle',
                   '-x', a['obstacle_start_x'], '-y', a['obstacle_start_y'],
                   '-z', '0.3'],
    )

    dynamic_obstacle_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='dynamic_obstacle_bridge', output='screen',
        condition=IfCondition(a['spawn_obstacle']),
        parameters=[{'config_file': os.path.join(
            bringup_dir, 'config', 'dynamic_obstacle_bridge.yaml')}],
    )

    lidar_obstacle_tracker = Node(
        package='predictive_nav_tracking', executable='lidar_obstacle_tracker_node',
        name='lidar_obstacle_tracker', output='screen',
        parameters=[os.path.join(tracking_dir, 'config', 'tracker_params.yaml')],
    )

    trial = Node(
        package='predictive_nav_bringup', executable='stage4f_trial.py',
        name='stage4f_trial', output='screen',
        condition=IfCondition(a['run_trial']),
        parameters=[{n: a[n] for n in (
            'trial_id', 'mode', 'scenario', 'out_path',
            'start_x', 'start_y', 'start_yaw', 'goal_x', 'goal_y', 'goal_yaw',
            'cross_x', 'cross_y', 'obstacle_start_x', 'obstacle_start_y',
            'obstacle_speed', 'obstacle_heading', 'obstacle_travel',
            'obstacle_trigger_delay', 'nominal_speed',
            'policy_max_prediction_horizon', 'policy_sigma_level',
            'policy_temporal_decay', 'policy_max_cost', 'policy_min_cost',
            'policy_max_influence_radius', 'nav_timeout')}],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'stage4f.rviz')],
        condition=IfCondition(a['use_rviz']),
    )

    ld = LaunchDescription(decls)
    ld.add_action(tb3_simulation)
    ld.add_action(initial_pose_publisher)
    ld.add_action(TimerAction(period=4.0, actions=[spawn_dynamic_obstacle]))
    ld.add_action(TimerAction(period=5.0, actions=[dynamic_obstacle_bridge]))
    ld.add_action(lidar_obstacle_tracker)
    ld.add_action(TimerAction(period=8.0, actions=[trial]))
    ld.add_action(rviz)
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=trial,
                      on_exit=[EmitEvent(event=Shutdown(reason='trial complete'))])))
    return ld
