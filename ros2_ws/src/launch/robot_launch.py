from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler, GroupAction
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Locate YAML configuration files
    lidar_config = os.path.join(
        FindPackageShare('simple_autonomous_robot').find('simple_autonomous_robot'),
        'config',
        'lidar_params.yaml'
    )

    controller_config = os.path.join(
        FindPackageShare('simple_autonomous_robot').find('simple_autonomous_robot'),
        'config',
        'controller_params.yaml'
    )

    # Declare Launch Arguments for dynamic control
    declare_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.3',
        description='Linear speed of the robot'
    )

    declare_angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.7',
        description='Angular speed for obstacle avoidance'
    )

    declare_obstacle_distance_arg = DeclareLaunchArgument(
        'obstacle_distance_threshold',
        default_value='1.0',
        description='Minimum distance to detect obstacles'
    )

    declare_recovery_behavior_arg = DeclareLaunchArgument(
        'recovery_behavior',
        default_value='true',
        description='Enable recovery behavior on obstacle detection'
    )

    # LIDAR Sensor Node (publishes scan data)
    lidar_node = Node(
        package='simple_autonomous_robot',
        executable='lidar_sensor',
        name='lidar_sensor',
        output='screen',
        parameters=[lidar_config]
    )

    # Robot Controller Node (subscribes to LIDAR and controls movement)
    controller_node = Node(
        package='simple_autonomous_robot',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[controller_config, {
            'linear_speed': LaunchConfiguration('linear_speed'),
            'angular_speed': LaunchConfiguration('angular_speed'),
            'obstacle_distance_threshold': LaunchConfiguration('obstacle_distance_threshold')
        }]
    )

    # Behavior: Restart controller if it crashes
    restart_controller_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=controller_node,
            on_exit=[
                LogInfo(msg='Controller crashed. Restarting...'),
                controller_node
            ]
        )
    )

    # Group action to launch nodes together
    launch_group = GroupAction([
        lidar_node,
        TimerAction(
            period=2.0,  # Delay starting controller by 2 seconds (ensure LIDAR initializes first)
            actions=[controller_node]
        )
    ])

    # Launch description to return all actions
    return LaunchDescription([
        declare_speed_arg,
        declare_angular_speed_arg,
        declare_obstacle_distance_arg,
        declare_recovery_behavior_arg,
        LogInfo(msg=TextSubstitution(text="Launching Simple Autonomous Robot...")),
        launch_group,
        restart_controller_handler
    ])
