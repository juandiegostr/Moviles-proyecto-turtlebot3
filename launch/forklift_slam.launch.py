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
            ('imu', '/imu'),
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
        cartographer_node,
        occupancy_grid_node,
    ])
