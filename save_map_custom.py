#!/usr/bin/env python3
"""
Script para guardar el mapa de Cartographer
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import cv2
import numpy as np
import yaml
import sys
import os

class MapSaver(Node):
    def __init__(self, map_name):
        super().__init__('map_saver_custom')
        
        self.map_name = map_name
        self.map_received = False
        
        # QoS flexible para recibir de diferentes publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        
        # También probar con QoS por defecto
        self.sub1 = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos
        )
        
        self.sub2 = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.get_logger().info('Esperando mapa en /map...')
        
    def map_callback(self, msg):
        if self.map_received:
            return
            
        self.map_received = True
        self.get_logger().info(f'Mapa recibido: {msg.info.width}x{msg.info.height}')
        
        # Convertir a imagen
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin
        
        # Crear imagen (invertida porque ROS usa origen abajo-izquierda)
        img = np.zeros((height, width), dtype=np.uint8)
        
        for i, cell in enumerate(msg.data):
            y = i // width
            x = i % width
            
            if cell == -1:  # Desconocido
                img[height - 1 - y, x] = 205
            elif cell == 0:  # Libre
                img[height - 1 - y, x] = 254
            else:  # Ocupado
                img[height - 1 - y, x] = 0
        
        # Guardar imagen
        pgm_file = f'{self.map_name}.pgm'
        yaml_file = f'{self.map_name}.yaml'
        
        cv2.imwrite(pgm_file, img)
        
        # Crear archivo YAML
        yaml_data = {
            'image': os.path.basename(pgm_file),
            'resolution': float(resolution),
            'origin': [float(origin.position.x), float(origin.position.y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        
        with open(yaml_file, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        
        self.get_logger().info(f'✅ Mapa guardado:')
        self.get_logger().info(f'   {pgm_file}')
        self.get_logger().info(f'   {yaml_file}')
        
        rclpy.shutdown()


def main():
    map_dir = '/root/ros2_ws/maps'
    map_name = sys.argv[1] if len(sys.argv) > 1 else 'warehouse'
    full_path = os.path.join(map_dir, map_name)
    
    os.makedirs(map_dir, exist_ok=True)
    
    rclpy.init()
    node = MapSaver(full_path)
    
    # Esperar máximo 10 segundos
    timeout = 10.0
    start = node.get_clock().now()
    
    while rclpy.ok() and not node.map_received:
        rclpy.spin_once(node, timeout_sec=0.1)
        elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
        if elapsed > timeout:
            node.get_logger().error('❌ Timeout esperando mapa')
            break
    
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
