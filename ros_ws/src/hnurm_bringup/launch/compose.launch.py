import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    hnurm_bringup_dir = get_package_share_directory('hnurm_bringup')

    # getting all params
    hnurm_uart_dir = get_package_share_directory('hnurm_uart')
    hnurm_camera_dir = get_package_share_directory('hnurm_camera')
    armor_detector_dir = get_package_share_directory('armor_detector')
    armor_solver_dir = get_package_share_directory('armor_solver')

    params_file_uart = LaunchConfiguration('uart_params_file')
    params_file_camera = LaunchConfiguration('camera_params_file')
    params_file_detect = LaunchConfiguration('detect_params_file')
    params_file_armor = LaunchConfiguration('armor_params_file')

    hnurm_bringup_ld = LaunchDescription([
        DeclareLaunchArgument(
            'uart_params_file',
            default_value=os.path.join(hnurm_uart_dir, 'params', 'default.yaml'),
            description='uart params file'
        ),

        DeclareLaunchArgument(
            'camera_params_file',
            default_value=os.path.join(hnurm_camera_dir, 'params', 'default.yaml'),
            description='camera params file'
        ),

        DeclareLaunchArgument(
            'detect_params_file',
            default_value=os.path.join(armor_detector_dir, 'params', 'default.yaml'),
            description='detect params file'
        ),

        DeclareLaunchArgument(
            'armor_params_file',
            default_value=os.path.join(armor_solver_dir, 'params', 'default.yaml'),
            description='armor params file'
        ),

        Node(
            package='hnurm_bringup',
            executable='compose_node',
            output='screen',
            parameters=[params_file_uart, params_file_camera, params_file_detect, params_file_armor]
        ),
    ])

    # now return the launch description
    return LaunchDescription([
        hnurm_bringup_ld
    ])
