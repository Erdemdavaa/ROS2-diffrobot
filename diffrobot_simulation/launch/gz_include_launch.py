import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Locate Gazebo (ros_gz_sim) and your world
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    world_pkg_name = 'diffrobot_simulation'
    world_file = os.path.join(
        get_package_share_directory(world_pkg_name),
        'worlds',
        'diffrobot_example_world.sdf'
    )

    # Launch Gazebo with your world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}'
        }.items(),
    )

    return LaunchDescription([
        gazebo
    ])
