import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    package_name = 'build_obstacles_footprint'
    pkg_share_dir = get_package_share_directory(package_name)
    obstacle_config_file = os.path.join(pkg_share_dir,'config','obstacles.yaml')

    declare_use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='True')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # definir a ação de iniciar o Nó
    polygon_builder_node = Node(
        package=package_name,
        executable='build_polygons.py', 
        name='polygon_builder_node',
        output='screen', # Mostra o log (print/get_logger) no terminal
        parameters=[
            obstacle_config_file,          # Passa o arquivo YAML
            {'use_sim_time': use_sim_time}  # Passa o parâmetro use_sim_time
        ]
    )

    
    return LaunchDescription([
        declare_use_sim_time_arg,
        polygon_builder_node
    ])