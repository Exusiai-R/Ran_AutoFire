import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import ComposableNodeContainer, Node
from launch.actions import TimerAction
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess # 添加 ExecuteProcess

def generate_launch_description():
    
    hnurm_uart_dir = get_package_share_directory('hnurm_uart')
    hnurm_camera_dir = get_package_share_directory('hnurm_camera')
    armor_detector_dir = get_package_share_directory('armor_detector')
    armor_solver_dir = get_package_share_directory('armor_solver')
    hnurm_virtual_camera_dir = get_package_share_directory('hnurm_virtual_camera')
    hnurm_virtual_uart_dir = get_package_share_directory('hnurm_virtual_uart')

    # 声明参数
    declare_uart_param = DeclareLaunchArgument(
        'uart_params_file',
        default_value=os.path.join(hnurm_uart_dir, 'params', 'default.yaml'),
        description='uart params file'
    )
    declare_camera_param = DeclareLaunchArgument(
        'camera_params_file',
        default_value=os.path.join(hnurm_camera_dir, 'params', 'default.yaml'),
        description='camera params file'
    )
    declare_armor_detector_param = DeclareLaunchArgument(
        'armor_detector_params_file',
        default_value=os.path.join(armor_detector_dir, 'params', 'default.yaml'),
        description='armor detector params file'
    )
    declare_armor_solver_param = DeclareLaunchArgument(
        'armor_solver_params_file',
        default_value=os.path.join(armor_solver_dir, 'params', 'default.yaml'),
        description='armor solver params file'
    )
    declare_virtual_camera_param = DeclareLaunchArgument(
        'virtual_camera_params_file',
        default_value=os.path.join(hnurm_virtual_camera_dir, 'params', 'default.yaml'),
        description='virtual camera params file'
    )
    declare_virtual_uart_param = DeclareLaunchArgument(
        'virtual_uart_params_file',
        default_value=os.path.join(hnurm_virtual_uart_dir, 'params', 'default.yaml'),
        description='virtual uart params file'
    )
    # ===== 新增：启动 Python 内参发布脚本 =====
    bringup_dir = get_package_share_directory('hnurm_bringup')
    yaml_path = os.path.join(bringup_dir, 'config', 'camera_info', '2BDFA7447491.yaml')
    publisher_script = os.path.join(hnurm_virtual_camera_dir, 'launch', 'camera_info_publisher.py')

    camera_info_pub = ExecuteProcess(
        cmd=['python3', publisher_script, '--ros-args', '-p', f'camera_info_url:={yaml_path}'],
        output='screen'
    )
    
    delay_camera_info_pub = TimerAction(
        period=1.5,
        actions=[camera_info_pub],
    )
    # ==========================================

    # 参数声明后再用 LaunchConfiguration
    params_file_uart = LaunchConfiguration('uart_params_file')
    params_file_camera = LaunchConfiguration('camera_params_file')
    params_file_armor_detector = LaunchConfiguration('armor_detector_params_file')
    params_file_armor_solver = LaunchConfiguration('armor_solver_params_file')
    params_file_virtual_camera = LaunchConfiguration('virtual_camera_params_file')
    params_file_virtual_uart = LaunchConfiguration('virtual_uart_params_file')

    uart_node = Node(
        package='hnurm_uart',
        executable='hnurm_uart_node',
        output='screen',
        parameters=[params_file_uart]
    )

    camera_node = ComposableNode(
        package='hnurm_camera',
        plugin='hnurm::CameraNode',
        name='camera_node',
        parameters=[params_file_camera],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    detect_model_path = os.path.join(armor_detector_dir, 'model')
    model_path = os.path.join(detect_model_path, "mlp.onnx")
    label_path = os.path.join(detect_model_path, "label.txt")
    yolov5_path = os.path.join(detect_model_path, "yolov5.xml")

    additional_param = {
        'model_path': model_path,
        'label_path': label_path,
        'yolov5_path': yolov5_path
    }

    armor_detector_node = ComposableNode(
        package='armor_detector',
        plugin='hnurm::ArmorDetectorNode',
        name='armor_detector_node',
        parameters=[params_file_armor_detector, additional_param],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    camera_detector_node = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_isolated',
        #composable_node_descriptions=[  camera_node,  armor_detector_node],
        composable_node_descriptions=[armor_detector_node],
        output='both',
        emulate_tty=True
    )

    armor_solver_node = Node(
        package='armor_solver',
        executable='armor_solver_node',
        output='screen',
        parameters=[params_file_armor_solver]
    )

    virtual_camera_node = Node(
        package='hnurm_virtual_camera',
        executable='hnurm_virtual_camera_node',
        output='screen',
        parameters=[params_file_virtual_camera]
    )

    virtual_uart_node = Node(
        package='hnurm_virtual_uart',
        executable='hnurm_virtual_uart_node',
        output='screen',
        parameters=[params_file_virtual_uart]
    )

    delay_uart_node = TimerAction(
        period=1.5,
        actions=[uart_node],
    )

    delay_camera_detector_node = TimerAction(
        period=1.5,
        actions=[camera_detector_node],
    )

    delay_armor_detector_node = TimerAction(
        period=1.5,
        actions=[armor_detector_node],
    )

    delay_armor_solver_node = TimerAction(
        period=2.0,
        actions=[armor_solver_node],
    )

    delay_virtual_camera_node = TimerAction(
        period=1.5,
        actions=[virtual_camera_node],
    )

    delay_virtual_uart_node = TimerAction(
        period=1.5,
        actions=[virtual_uart_node],
    ) 

    # 正常启动
    # return LaunchDescription([
    #     declare_uart_param,
    #     declare_camera_param,
    #     declare_armor_detector_param,
    #     declare_armor_solver_param,
    #     # declare_virtual_camera_param,
    #     # declare_virtual_uart_param,
    #     delay_uart_node,
    #     # delay_virtual_uart_node,
    #     # delay_virtual_camera_node,
    #     # delay_armor_detector_node,
    #     delay_camera_detector_node,
    #     delay_armor_solver_node
    #     ])

    # 使用虚拟串口节点
    # return LaunchDescription([
    #     # declare_uart_param,
    #     declare_camera_param,
    #     declare_armor_detector_param,
    #     declare_armor_solver_param,
    #     # declare_virtual_camera_param,
    #     declare_virtual_uart_param,
    #     # delay_uart_node,
    #     delay_virtual_uart_node,
    #     # delay_virtual_camera_node,
    #     # delay_armor_detector_node,
    #     delay_camera_detector_node,
    #     delay_armor_solver_node
    # ])

    return LaunchDescription([
        # declare_uart_param,
        # declare_camera_param,
        declare_armor_detector_param,
        declare_armor_solver_param,
        declare_virtual_camera_param,
        declare_virtual_uart_param,
        # delay_uart_node,
        delay_virtual_uart_node,
        delay_virtual_camera_node,
        
        delay_camera_info_pub, # <---- 【把这个加进来】
        
        # delay_armor_detector_node,
        delay_camera_detector_node,
        delay_armor_solver_node
    ])

    # 使用仿真器
    # return LaunchDescription([
    #     # declare_uart_param,
    #     # declare_camera_param,
    #     declare_armor_detector_param,
    #     declare_armor_solver_param,
    #     # declare_virtual_camera_param,
    #     # declare_virtual_uart_param,
    #     # delay_uart_node,
    #     # delay_virtual_uart_node,
    #     # delay_virtual_camera_node,
    #     # delay_armor_detector_node,
    #     delay_camera_detector_node,
    #     delay_armor_solver_node
    # ])