"""
Launch file para Cartographer SLAM con el Forklift
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Directorio de configuración
    config_dir = '/root/ros2_ws/src/config'
    
    # Argumentos
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # TF estático: scan -> base_link (el LIDAR está 1.2m arriba del base_link)
    static_tf_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_to_base_link',
        arguments=['0', '0', '1.2', '0', '0', '0', 'base_link', 'scan']
    )
    
    # TF estático: front_scan -> base_link
    static_tf_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='front_scan_to_base_link',
        arguments=['1.2', '0', '0.4', '0', '0', '0', 'base_link', 'front_scan']
    )
    
    # TF estático: rear_scan -> base_link
    static_tf_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rear_scan_to_base_link',
        arguments=['-0.8', '0', '0.4', '0', '0', '3.14159', 'base_link', 'rear_scan']
    )
    
    # Nodo de Cartographer
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'forklift_cartographer.lua'
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ]
    )
    
    # Nodo de ocupancy grid (genera el mapa)
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        static_tf_scan,
        static_tf_front,
        static_tf_rear,
        cartographer_node,
        occupancy_grid_node,
    ])
