"""Stage-4D bringup: the validated Stage-4B/4C stack plus the predictive
Nav2 costmap layer.

Identical to stage4b_bringup.launch.py except that it hands Nav2 a
project-owned params file (config/nav2_predictive_params.yaml) which adds
predictive_nav_costmap::PredictedObstacleLayer to the local costmap. Nothing
under /opt/ros is modified: nav2_bringup's tb3_simulation_launch.py already
exposes a 'params_file' launch argument for exactly this purpose.

The 'params_file' argument is overridable so the same launch file can be
reused for the global-costmap reusability demonstration without editing
anything here.
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
    params_file = LaunchConfiguration('params_file')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless', default_value='True',
        description='Whether to run gz sim without its own GUI client')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Whether to start RViz with the tracking/prediction display config')
    declare_x_pose_cmd = DeclareLaunchArgument('x_pose', default_value='-2.00')
    declare_y_pose_cmd = DeclareLaunchArgument('y_pose', default_value='-0.50')
    declare_yaw_cmd = DeclareLaunchArgument('yaw', default_value='0.00')
    declare_spawn_obstacle_cmd = DeclareLaunchArgument(
        'spawn_obstacle', default_value='True',
        description='Whether to spawn and drive the moving obstacle. '
                    'Set False for the stationary-world regression run.')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_predictive_params.yaml'),
        description='Project-owned Nav2 parameters file carrying the predictive costmap '
                    'layer configuration. Override to run the global-costmap variant.')

    # 1. Stage-4A baseline simulation + Nav2, now with project-owned params.
    tb3_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={
            'headless': headless,
            'use_rviz': 'False',
            'x_pose': x_pose,
            'y_pose': y_pose,
            'yaw': yaw,
            'params_file': params_file,
        }.items(),
    )

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

    spawn_dynamic_obstacle = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_dynamic_obstacle',
        output='screen',
        condition=IfCondition(spawn_obstacle),
        arguments=[
            '-file', os.path.join(bringup_dir, 'models', 'dynamic_obstacle.sdf'),
            '-name', 'dynamic_obstacle',
            '-x', '-1.0', '-y', '-0.5', '-z', '0.3',
        ],
    )

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

    # Stage-4B/4C tracker + Kalman predictor, unchanged.
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
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'stage4d.rviz')],
        condition=IfCondition(use_rviz),
    )

    ld = LaunchDescription()
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_x_pose_cmd)
    ld.add_action(declare_y_pose_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_spawn_obstacle_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(tb3_simulation)
    ld.add_action(initial_pose_publisher)
    ld.add_action(TimerAction(period=4.0, actions=[spawn_dynamic_obstacle]))
    ld.add_action(TimerAction(period=5.0, actions=[dynamic_obstacle_bridge]))
    ld.add_action(TimerAction(period=6.0, actions=[obstacle_mover]))
    ld.add_action(lidar_obstacle_tracker)
    ld.add_action(rviz)

    return ld
