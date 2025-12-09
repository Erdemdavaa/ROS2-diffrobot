# diffrobot_navigation/launch/localization_launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('diffrobot_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    declared_arguments = [
        DeclareLaunchArgument('map', default_value=os.path.join(pkg_share, 'maps', 'map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg_share, 'config', 'localization_params.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
    ]

    map_arg = LaunchConfiguration('map')
    params_file_arg = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_arg,
            'params_file': params_file_arg,
            'use_sim_time': use_sim_time
        }.items()
    )

    ld = LaunchDescription()
    for arg in declared_arguments:
        ld.add_action(arg)
    ld.add_action(localization_launch)
    return ld
