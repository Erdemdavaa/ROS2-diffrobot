import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    
    diffrobot_description = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('diffrobot_description'),
            'launch',
            'diffrobot_description_launch.py'
        ]),
        launch_arguments={
            'use_sim_time' : 'true'
        }.items()
    )

    diffrobot_simulation_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('diffrobot_simulation'),
            'launch',
            'gz_include_launch.py'
        ])
    )

    diffrobot_simulation_bridge = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('diffrobot_simulation'),
            'launch',
            'gz_bridge_include_bridge.py'
        ])
    )

    start_gazebo_ros_spawner = Node(
        name='diff_robot_spawner',
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', "diff_robot",
            '-topic', "robot_description",
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.01',
        ],
        output='screen',
    )


    return LaunchDescription([
        diffrobot_description,
        diffrobot_simulation_launch,
        diffrobot_simulation_bridge,
        start_gazebo_ros_spawner,      
    ])