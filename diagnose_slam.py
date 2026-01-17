#!/usr/bin/env python3
"""
Diagnóstico de topics para SLAM
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Imu

class Diagnostics(Node):
    def __init__(self):
        super().__init__('slam_diagnostics')
        
        self.scan_count = 0
        self.odom_count = 0
        self.imu_count = 0
        self.map_count = 0
        
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        
        self.timer = self.create_timer(2.0, self.report)
        self.get_logger().info('Diagnosticando topics...')
        
    def scan_cb(self, msg):
        self.scan_count += 1
        
    def odom_cb(self, msg):
        self.odom_count += 1
        
    def imu_cb(self, msg):
        self.imu_count += 1
        
    def map_cb(self, msg):
        self.map_count += 1
        self.get_logger().info(f'MAP recibido: {msg.info.width}x{msg.info.height}')
        
    def report(self):
        print(f"\n📊 Estado de topics (mensajes recibidos):")
        print(f"   /scan (LIDAR 360°):  {self.scan_count} {'✅' if self.scan_count > 0 else '❌'}")
        print(f"   /odom (Odometría):   {self.odom_count} {'✅' if self.odom_count > 0 else '❌'}")
        print(f"   /imu (IMU):          {self.imu_count} {'✅' if self.imu_count > 0 else '❌'}")
        print(f"   /map (Mapa):         {self.map_count} {'✅' if self.map_count > 0 else '❌'}")
        
        # Reset
        self.scan_count = 0
        self.odom_count = 0
        self.imu_count = 0
        self.map_count = 0

def main():
    rclpy.init()
    node = Diagnostics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
