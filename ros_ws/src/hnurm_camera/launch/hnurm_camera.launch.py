import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_dir = get_package_share_directory('hnurm_camera')
    params_file = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(camera_dir, 'params', 'default.yaml'),
            description='Full path to the ROS2 parameters file to use for the camera'
        ),
        Node(
            package='hnurm_camera',
            executable='hnurm_camera_node',
            output='screen',
            parameters=[params_file]
        )
    ])
