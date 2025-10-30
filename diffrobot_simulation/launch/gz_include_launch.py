import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    ros_gz_launch_dir = os.path.join(ros_gz_sim_dir, 'launch')

    world_pkg_name = 'diffrobot_simulation'
    world_file_subpath = 'worlds/diffrobot_example_world.sdf'
    world_file = os.path.join(get_package_share_directory(world_pkg_name),world_file_subpath)


    return LaunchDescription([
        IncludeLaunchDescription(
            #PathJoinSubstitution(os.path.join(ros_gz_launch_dir, 'gz_sim.launch.py')),
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ]),
            launch_arguments={
                'gz_args': ['', world_file],
                'on_exit_shutdown' : 'true'
            }.items(),
        )
    ])