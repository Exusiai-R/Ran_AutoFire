import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

calib_param = {
    "fExposureTime": 20000.0,  # 相机曝光时间
    'fGain': 15.0  # 相机增益
}

def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('hnurm_bringup')
    camera_dir = get_package_share_directory('hnurm_camera')
    
    # 加载launch_params.yaml
    launch_params_file = os.path.join(bringup_dir, 'config', 'launch_params.yaml')
    
    # camera_node自己的参数文件
    camera_params_file = os.path.join(camera_dir, 'params', 'default.yaml')
    
    # 从launch_params.yaml中获取camera_id和camera_info_url
    import yaml
    with open(launch_params_file, 'r') as f:
        launch_params = yaml.safe_load(f)
    
    camera_id = launch_params.get('camera_id', '2BDFA7447491')
    camera_info_url_package = launch_params.get('camera_info_url', f"package://hnurm_bringup/config/camera_info/{camera_id}.yaml")
    
    # 将package URL转换为文件系统路径
    # package://hnurm_bringup/config/camera_info/<camera_id>.yaml -> <bringup_share_dir>/config/camera_info/
    if camera_info_url_package.startswith('package://hnurm_bringup/'):
        relative_path = camera_info_url_package[len('package://hnurm_bringup/'):]
        # 获取目录路径（去掉文件名）
        relative_dir = os.path.dirname(relative_path)
        camera_info_dir = os.path.join(bringup_dir, relative_dir)
    else:
        camera_info_dir = ""
    
    # 创建相机节点
    camera_node = Node(
        package='hnurm_camera',
        executable='hnurm_camera_node',
        parameters=[camera_params_file, launch_params_file, calib_param]
    )
    
    # 创建标定节点
    camera_calibration_node = Node(
        package='camera_calibration',
        executable='cameracalibrator',
        parameters=[
            {
                'camera_info_url': camera_info_dir,  # 传递目录路径（文件系统路径）
                'camera_id': camera_id
            }
        ],
        arguments=['--pattern', 'charuco', '--size', '12x9', '--square', '0.03', '--aruco_dict', '5x5_1000', '--charuco_marker_size', '0.0225', '--no-service-check']
    )
    
    return [camera_node, camera_calibration_node]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
