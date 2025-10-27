import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    pkg_path = get_package_share_directory('estimate_markers_poses')
    param_config = os.path.join(pkg_path, 'config', 'isaac_apriltag.yaml')

    set_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    use_sim_time = LaunchConfiguration("use_sim_time")

    composable_nodes = [
        # Image Rectifier - Remove Distortion
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect')
            ],
            parameters=[{'use_sim_time': True}]
        ),
        
        # Apriltag Detector - Detecte and estimate markers poses
        ComposableNode(
            package='apriltag_ros',
            plugin='AprilTagNode', 
            name='apriltag',
            namespace='', 
            remappings=[
                ('image_rect', 'image_rect'),
                ('camera_info', 'camera_info')
            ],
            parameters=[param_config, {'use_sim_time': True}],
            extra_arguments=[{'use_intra_process_comms': True}]
        )        
    ]

    container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([set_use_sim_time,container])