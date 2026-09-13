"""Stage-4E bringup: one deterministic crossing-conflict trial, either arm.

Same stack as Stage-4D (simulation + Nav2 + Stage-4B/4C tracker + the
predictive costmap layer) but pointed at config/nav2_stage4e_params.yaml and
with the Stage-4D back-and-forth `obstacle_mover` REPLACED by the trial
runner's own one-way scripted obstacle. That removes the reversal transient
from the experiment and keys obstacle release to goal acceptance, so t=0 means
the same thing in both arms.

The obstacle is spawned but held stationary at its park pose until the trial
runner releases it; `spawn_obstacle:=False` gives the no-obstacle nominal run.

`mode` is passed straight through to the trial runner, which flips
`predicted_obstacle_layer.enabled` accordingly. Both arms load the identical
params file and the identical set of plugins.
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


def generate_launch_description():
    bringup_dir = get_package_share_directory('predictive_nav_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tracking_dir = get_package_share_directory('predictive_nav_tracking')

    args = {n: LaunchConfiguration(n) for n in (
        'headless', 'use_rviz', 'mode', 'scenario', 'trial_id', 'out_path',
        'params_file', 'spawn_obstacle', 'run_trial',
        'start_x', 'start_y', 'start_yaw', 'goal_x', 'goal_y', 'goal_yaw',
        'cross_x', 'cross_y',
        'obstacle_start_y', 'obstacle_stop_y', 'obstacle_speed', 'obstacle_dir',
        'obstacle_trigger_delay', 'nominal_speed',
        'policy_max_prediction_horizon', 'policy_sigma_level',
        'policy_temporal_decay', 'policy_max_cost', 'policy_min_cost',
        'policy_max_influence_radius',
        'nav_timeout')}

    # Scenario geometry defaults. The crossing point (cross_x, cross_y) is the
    # most open junction of the tb3_sandbox pillar grid (0.57 m clearance to
    # the nearest static obstacle), where the robot's straight corridor
    # y = -0.55 meets the obstacle's column x = 0.55.
    decls = [
        DeclareLaunchArgument('headless', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('mode', default_value='predictive',
                              description="'reactive' or 'predictive'"),
        DeclareLaunchArgument('scenario', default_value='crossing',
                              description="'crossing', 'noconflict' or 'nominal'"),
        DeclareLaunchArgument('trial_id', default_value='trial'),
        DeclareLaunchArgument('out_path', default_value=''),
        DeclareLaunchArgument('run_trial', default_value='True'),
        DeclareLaunchArgument('spawn_obstacle', default_value='True'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config',
                                       'nav2_stage4e_params.yaml')),
        DeclareLaunchArgument('start_x', default_value='1.75'),
        DeclareLaunchArgument('start_y', default_value='-0.55'),
        DeclareLaunchArgument('start_yaw', default_value='3.14159'),
        DeclareLaunchArgument('goal_x', default_value='-1.75'),
        DeclareLaunchArgument('goal_y', default_value='-0.55'),
        DeclareLaunchArgument('goal_yaw', default_value='3.14159'),
        DeclareLaunchArgument('cross_x', default_value='-0.55'),
        DeclareLaunchArgument('cross_y', default_value='-0.55'),
        DeclareLaunchArgument('obstacle_start_y', default_value='-2.05'),
        DeclareLaunchArgument('obstacle_stop_y', default_value='0.90'),
        DeclareLaunchArgument('obstacle_speed', default_value='0.25'),
        DeclareLaunchArgument('obstacle_dir', default_value='1.0'),
        DeclareLaunchArgument('obstacle_trigger_delay', default_value='1.15'),
        DeclareLaunchArgument('nominal_speed', default_value='0.48'),
        DeclareLaunchArgument('policy_max_prediction_horizon', default_value='3.0'),
        DeclareLaunchArgument('policy_sigma_level', default_value='1.5'),
        DeclareLaunchArgument('policy_temporal_decay', default_value='0.35'),
        DeclareLaunchArgument('policy_max_cost', default_value='250'),
        DeclareLaunchArgument('policy_min_cost', default_value='0'),
        DeclareLaunchArgument('policy_max_influence_radius', default_value='0.55'),
        DeclareLaunchArgument('nav_timeout', default_value='90.0'),
    ]

    tb3_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'headless': args['headless'],
            'use_rviz': 'False',
            'x_pose': args['start_x'],
            'y_pose': args['start_y'],
            'yaw': args['start_yaw'],
            'params_file': args['params_file'],
        }.items(),
    )

    initial_pose_publisher = Node(
        package='predictive_nav_bringup', executable='publish_initial_pose.py',
        name='initial_pose_publisher', output='screen',
        parameters=[{'x': args['start_x'], 'y': args['start_y'],
                     'yaw': args['start_yaw'], 'wait_timeout_sec': 60.0}],
    )

    # Spawned at the crossing column, parked below the robot's corridor. Held
    # at zero velocity by the trial runner until goal acceptance.
    spawn_dynamic_obstacle = Node(
        package='ros_gz_sim', executable='create', name='spawn_dynamic_obstacle',
        output='screen', condition=IfCondition(args['spawn_obstacle']),
        arguments=['-file', os.path.join(bringup_dir, 'models', 'dynamic_obstacle.sdf'),
                   '-name', 'dynamic_obstacle',
                   '-x', args['cross_x'], '-y', args['obstacle_start_y'], '-z', '0.3'],
    )

    dynamic_obstacle_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='dynamic_obstacle_bridge', output='screen',
        condition=IfCondition(args['spawn_obstacle']),
        parameters=[{'config_file': os.path.join(
            bringup_dir, 'config', 'dynamic_obstacle_bridge.yaml')}],
    )

    lidar_obstacle_tracker = Node(
        package='predictive_nav_tracking', executable='lidar_obstacle_tracker_node',
        name='lidar_obstacle_tracker', output='screen',
        parameters=[os.path.join(tracking_dir, 'config', 'tracker_params.yaml')],
    )

    trial = Node(
        package='predictive_nav_bringup', executable='stage4e_trial.py',
        name='stage4e_trial', output='screen',
        condition=IfCondition(args['run_trial']),
        parameters=[{
            'trial_id': args['trial_id'],
            'mode': args['mode'],
            'scenario': args['scenario'],
            'out_path': args['out_path'],
            'start_x': args['start_x'], 'start_y': args['start_y'],
            'start_yaw': args['start_yaw'],
            'goal_x': args['goal_x'], 'goal_y': args['goal_y'],
            'goal_yaw': args['goal_yaw'],
            'cross_x': args['cross_x'], 'cross_y': args['cross_y'],
            'obstacle_start_y': args['obstacle_start_y'],
            'obstacle_stop_y': args['obstacle_stop_y'],
            'obstacle_speed': args['obstacle_speed'],
            'obstacle_dir': args['obstacle_dir'],
            'obstacle_trigger_delay': args['obstacle_trigger_delay'],
            'nominal_speed': args['nominal_speed'],
            'policy_max_prediction_horizon': args['policy_max_prediction_horizon'],
            'policy_sigma_level': args['policy_sigma_level'],
            'policy_temporal_decay': args['policy_temporal_decay'],
            'policy_max_cost': args['policy_max_cost'],
            'policy_min_cost': args['policy_min_cost'],
            'policy_max_influence_radius': args['policy_max_influence_radius'],
            'nav_timeout': args['nav_timeout'],
        }],
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'stage4e.rviz')],
        condition=IfCondition(args['use_rviz']),
    )

    ld = LaunchDescription(decls)
    ld.add_action(tb3_simulation)
    ld.add_action(initial_pose_publisher)
    ld.add_action(TimerAction(period=4.0, actions=[spawn_dynamic_obstacle]))
    ld.add_action(TimerAction(period=5.0, actions=[dynamic_obstacle_bridge]))
    ld.add_action(lidar_obstacle_tracker)
    ld.add_action(TimerAction(period=8.0, actions=[trial]))
    ld.add_action(rviz)
    # The trial runner owns the trial's lifetime: when it exits, tear the
    # whole stack down so the batch driver can start the next trial from a
    # guaranteed-clean simulator.
    ld.add_action(RegisterEventHandler(
        OnProcessExit(target_action=trial,
                      on_exit=[EmitEvent(event=Shutdown(reason='trial complete'))])))
    return ld
