from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix=['xterm -e']  # opens in new terminal window
        # no remapping here
    )

    return LaunchDescription([teleop])
