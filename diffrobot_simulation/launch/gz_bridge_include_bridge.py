import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/world/empty/model/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            'cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            'odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            'scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            'scan/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/world/empty/model/joint_state', 'joint_states'),
        ],
        output='screen',
    )

    return LaunchDescription([
        gz_bridge
    ])

