"""Stage-4B bringup: Nav2 TB3 simulation (Stage-4A baseline, unmodified) +
a scripted moving obstacle + the LiDAR dynamic-obstacle tracker + RViz.

This launch file is the project-owned fix for the Stage-4A known issue:
instead of requiring a human to publish /initialpose at the right moment, it
launches a subscription-gated one-shot publisher (see
scripts/publish_initial_pose.py) so every run localizes deterministically.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('predictive_nav_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    tracking_dir = get_package_share_directory('predictive_nav_tracking')

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    spawn_obstacle = LaunchConfiguration('spawn_obstacle')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Whether to run gz sim without its own GUI client')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Whether to start RViz with the Stage-4B tracking display config')
    declare_x_pose_cmd = DeclareLaunchArgument('x_pose', default_value='-2.00')
    declare_y_pose_cmd = DeclareLaunchArgument('y_pose', default_value='-0.50')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.00')
    declare_spawn_obstacle_cmd = DeclareLaunchArgument(
        'spawn_obstacle', default_value='True',
        description='Whether to spawn and drive the Stage-4B moving obstacle. '
                     'Set False for the stationary-world false-positive validation run.')

    # 1. Stage-4A baseline: the unmodified, validated Nav2 TB3 simulation.
    #    We only pass launch arguments; nothing under nav2_bringup is edited.
    tb3_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'headless': headless,
            'use_rviz': 'False',  # we launch our own RViz with tracker displays
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
        }.items(),
    )

    # 2. Project-owned fix for the Stage-4A localization-timing issue: block
    #    until AMCL is actually subscribed, then publish exactly once.
    initial_pose_publisher = Node(
        package='predictive_nav_bringup',
        executable='publish_initial_pose.py',
        name='initial_pose_publisher',
        output='screen',
        parameters=[{
            'x': x_pose,
            'y': y_pose,
            'yaw': yaw,
            'wait_timeout_sec': 60.0,
        }],
    )

    # 3. Spawn the Stage-4B moving obstacle into the running world. Delayed
    #    slightly so the gz world/create service is guaranteed to be up.
    spawn_dynamic_obstacle = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_dynamic_obstacle',
        output='screen',
        condition=IfCondition(spawn_obstacle),
        arguments=[
            '-file', os.path.join(bringup_dir, 'models', 'dynamic_obstacle.sdf'),
            '-name', 'dynamic_obstacle',
            # Spawned in the robot's own row (y=-0.5), which map inspection
            # confirmed is a clear, pillar-free corridor with guaranteed
            # line-of-sight from the spawn pose -- unlike a y-sweep, which
            # crosses the sandbox world's grid of static pillar obstacles.
            '-x', '-1.0', '-y', '-0.5', '-z', '0.3',
        ],
    )

    # 4. Bridge only the moving obstacle's cmd_vel / ground-truth odometry.
    #    Kept separate from nav2_bringup's own robot bridge.
    dynamic_obstacle_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='dynamic_obstacle_bridge',
        output='screen',
        condition=IfCondition(spawn_obstacle),
        parameters=[{
            'config_file': os.path.join(bringup_dir, 'config', 'dynamic_obstacle_bridge.yaml'),
        }],
    )

    # 5. Deterministic driver for the moving obstacle (simulation helper,
    #    not part of the tracker under test).
    obstacle_mover = Node(
        package='predictive_nav_bringup',
        executable='obstacle_mover.py',
        name='obstacle_mover',
        output='screen',
        condition=IfCondition(spawn_obstacle),
        parameters=[{
            'odom_topic': '/model/dynamic_obstacle/odometry',
            'cmd_vel_topic': '/model/dynamic_obstacle/cmd_vel',
            'axis': 'axis_x',
            'speed': 0.25,
            'bound_min': -1.5,
            'bound_max': 0.5,
        }],
    )

    # 6. The Stage-4B tracker under test.
    lidar_obstacle_tracker = Node(
        package='predictive_nav_tracking',
        executable='lidar_obstacle_tracker_node',
        name='lidar_obstacle_tracker',
        output='screen',
        parameters=[os.path.join(tracking_dir, 'config', 'tracker_params.yaml')],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'stage4b.rviz')],
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription()
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_spawn_obstacle_cmd)

    ld.add_action(tb3_simulation)
    ld.add_action(initial_pose_publisher)
    ld.add_action(TimerAction(period=4.0, actions=[spawn_dynamic_obstacle]))
    ld.add_action(TimerAction(period=5.0, actions=[dynamic_obstacle_bridge]))
    ld.add_action(TimerAction(period=6.0, actions=[obstacle_mover]))
    ld.add_action(lidar_obstacle_tracker)
    ld.add_action(rviz)

    return ld
