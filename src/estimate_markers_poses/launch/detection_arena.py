import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    pkg_path = get_package_share_directory('estimate_markers_poses')
    tag_param = os.path.join(pkg_path, 'config', 'lima_apriltag.yaml')
    pose_param = os.path.join(pkg_path, 'config', 'camera_pose.yaml')
    rviz_scene = os.path.join(pkg_path, 'rviz', 'tf_visualization.rviz')

    set_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    use_sim_time = LaunchConfiguration("use_sim_time")

    composable_nodes = [
        # Image Rectifier - Remove Distortion
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_node',
            remappings=[
                ('image', 'camera/image_raw'),
                ('camera_info', 'camera/camera_info'),
                ('image_rect', 'image_rect')
            ],
            parameters=[{'image_transport': 'compressed'},{'use_sim_time': False}]
        ),
        
        # Apriltag Detector - Detecte and estimate markers poses
        ComposableNode(
            package='apriltag_ros',
            plugin='AprilTagNode', 
            name='apriltag',
            namespace='', 
            remappings=[
                ('image_rect', 'image_rect'),
                ('camera_info', 'camera/camera_info')
            ],
            parameters=[tag_param, {'use_sim_time': False}],
            extra_arguments=[{'use_intra_process_comms': True}]
        )        
    ]

    container = ComposableNodeContainer(
        name='image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        parameters=[{'use_sim_time': False}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_scene], 
        parameters=[{'use_sim_time': use_sim_time}] 
    )

    camera_pose_publisher = Node(
        package='estimate_markers_poses',
        executable='publish_camera_pose.py',
        name='camera_pose_publisher',
        parameters=[{'config_file': pose_param}],
        output='screen',
    )

    return LaunchDescription([set_use_sim_time,
                              container,
                              camera_pose_publisher,
                              rviz_node])