from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to slam_toolbox package
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Path to your config yaml
    slam_params = os.path.join(
        get_package_share_directory('diffrobot_slam'),
        'config',
        'slam_params.yaml'
    )

    return LaunchDescription([

        # Include SLAM toolbox's main launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={'params_file': slam_params}.items()
        )
    ])
