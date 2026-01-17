#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════════╗
║        🚜 FORKLIFT AUTONOMOUS PICK & PLACE 🚜                  ║
║                                                               ║
║   Programa que navega automáticamente a un pallet,           ║
║   lo "recoge" y vuelve a la posición inicial.                ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

# Posiciones de los pallets en el almacén (x, y)
PALLET_POSITIONS = {
    'pallet_1': (-17.0, 8.0),
    'pallet_2': (-15.0, 8.0),
    'pallet_3': (-13.0, 8.0),
    'pallet_4': (-17.0, 5.0),
    'pallet_5': (-15.0, 5.0),
}

# Posición de inicio/descarga
HOME_POSITION = (-15.0, 0.0)

# Offset para insertar horquillas (distancia extra que avanza después de llegar al pallet)
FORK_INSERT_OFFSET = 1.2  # metros - ajusta según el tamaño del pallet

# Tolerancias
DISTANCE_TOLERANCE = 0.3  # metros (más preciso)
ANGLE_TOLERANCE = 0.08    # radianes (más preciso)

# Velocidades
MAX_LINEAR_VEL = 1.0   # m/s
MAX_ANGULAR_VEL = 0.8  # rad/s


class ForkliftAutonomous(Node):
    def __init__(self):
        super().__init__('forklift_autonomous')
        
        # Publisher para comandos de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber para odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Estado actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        # Estado de la misión
        self.carrying_pallet = False
        
        self.get_logger().info('🚜 Forklift Autonomous Node iniciado!')
        
    def odom_callback(self, msg):
        """Callback para actualizar la posición actual"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extraer yaw del quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_received = True
        
    def get_distance_to(self, target_x, target_y):
        """Calcula la distancia al objetivo"""
        return math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
    
    def get_angle_to(self, target_x, target_y):
        """Calcula el ángulo hacia el objetivo"""
        return math.atan2(target_y - self.current_y, target_x - self.current_x)
    
    def normalize_angle(self, angle):
        """Normaliza el ángulo entre -pi y pi"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop(self):
        """Detiene el robot"""
        msg = Twist()
        self.cmd_pub.publish(msg)
        
    def navigate_to(self, target_x, target_y, description=""):
        """Navega hacia una posición objetivo"""
        self.get_logger().info(f'📍 Navegando a ({target_x:.1f}, {target_y:.1f}) {description}')
        
        rate = self.create_rate(20)  # 20 Hz
        
        while rclpy.ok():
            if not self.odom_received:
                self.get_logger().info('⏳ Esperando odometría...')
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
                
            distance = self.get_distance_to(target_x, target_y)
            
            # ¿Llegamos?
            if distance < DISTANCE_TOLERANCE:
                self.stop()
                self.get_logger().info(f'✅ ¡Llegamos! Distancia: {distance:.2f}m')
                return True
            
            # Calcular ángulo hacia objetivo
            target_angle = self.get_angle_to(target_x, target_y)
            angle_error = self.normalize_angle(target_angle - self.current_yaw)
            
            # Crear mensaje de velocidad
            msg = Twist()
            
            # Si el ángulo es grande, primero girar
            if abs(angle_error) > ANGLE_TOLERANCE:
                # Girar hacia el objetivo
                msg.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angle_error * 2.0))
                # Avanzar un poco si el ángulo no es muy grande
                if abs(angle_error) < 0.5:
                    msg.linear.x = MAX_LINEAR_VEL * 0.5
            else:
                # Avanzar hacia el objetivo
                msg.linear.x = min(MAX_LINEAR_VEL, distance * 0.5)
                msg.angular.z = angle_error * 1.0  # Corrección pequeña
            
            self.cmd_pub.publish(msg)
            
            # Mostrar progreso cada segundo aprox
            self.get_logger().info(
                f'   📊 Pos: ({self.current_x:.1f}, {self.current_y:.1f}) | '
                f'Dist: {distance:.1f}m | Ang: {math.degrees(angle_error):.0f}°',
                throttle_duration_sec=1.0
            )
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        return False
    
    def pick_pallet(self):
        """Las horquillas están insertadas - mantener contacto"""
        self.get_logger().info('🔧 Horquillas insertadas en el pallet')
        time.sleep(0.5)
        self.carrying_pallet = True
        self.get_logger().info('✅ ¡Listo para mover el pallet empujándolo!')
    
    def align_to(self, target_x, target_y):
        """Gira para alinearse con el objetivo"""
        self.get_logger().info(f'🎯 Alineando hacia ({target_x:.1f}, {target_y:.1f})')
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            
            target_angle = self.get_angle_to(target_x, target_y)
            angle_error = self.normalize_angle(target_angle - self.current_yaw)
            
            if abs(angle_error) < ANGLE_TOLERANCE:
                self.stop()
                self.get_logger().info(f'✅ Alineado! Error: {math.degrees(angle_error):.1f}°')
                return True
            
            msg = Twist()
            msg.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angle_error * 2.0))
            self.cmd_pub.publish(msg)
            
        return False
    
    def move_forward(self, distance):
        """Avanza (o retrocede si es negativo) una distancia específica"""
        direction = "Avanzando" if distance > 0 else "Retrocediendo"
        self.get_logger().info(f'🚜 {direction} {abs(distance):.1f}m...')
        
        start_x = self.current_x
        start_y = self.current_y
        target_distance = abs(distance)
        
        # Velocidad fija para el movimiento
        speed = 0.4  # m/s - velocidad constante
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            
            traveled = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            
            if traveled >= target_distance - 0.05:  # Tolerancia de 5cm
                self.stop()
                self.get_logger().info(f'✅ Movimiento completado! Distancia: {traveled:.2f}m')
                return True
            
            msg = Twist()
            msg.linear.x = speed if distance > 0 else -speed
            self.cmd_pub.publish(msg)
            
            # Log de progreso
            self.get_logger().info(
                f'   🚜 Avanzado: {traveled:.2f}m / {target_distance:.2f}m',
                throttle_duration_sec=0.5
            )
            
        return False
        
    def drop_pallet(self):
        """Simula soltar un pallet"""
        self.get_logger().info('🔧 Bajando horquillas...')
        time.sleep(1)
        self.get_logger().info('📦 Soltando pallet...')
        time.sleep(1)
        self.get_logger().info('🔧 Subiendo horquillas...')
        time.sleep(1)
        self.carrying_pallet = False
        self.get_logger().info('✅ ¡Pallet entregado!')
        
    def execute_mission(self, pallet_name):
        """Ejecuta una misión completa: ir al pallet, empujarlo a HOME"""
        if pallet_name not in PALLET_POSITIONS:
            self.get_logger().error(f'❌ Pallet "{pallet_name}" no encontrado!')
            self.get_logger().info(f'   Pallets disponibles: {list(PALLET_POSITIONS.keys())}')
            return False
        
        pallet_x, pallet_y = PALLET_POSITIONS[pallet_name]
        
        self.get_logger().info('='*50)
        self.get_logger().info(f'🎯 MISIÓN: Mover {pallet_name} a HOME')
        self.get_logger().info('='*50)
        
        # Calcular punto de aproximación (detrás del pallet, para empujarlo hacia HOME)
        # Necesitamos posicionarnos al lado opuesto de HOME respecto al pallet
        angle_pallet_to_home = math.atan2(HOME_POSITION[1] - pallet_y, HOME_POSITION[0] - pallet_x)
        
        # Punto de aproximación: detrás del pallet (lado opuesto a HOME)
        approach_distance = 2.0  # metros detrás del pallet
        approach_x = pallet_x - approach_distance * math.cos(angle_pallet_to_home)
        approach_y = pallet_y - approach_distance * math.sin(angle_pallet_to_home)
        
        # Fase 1: Ir al punto de aproximación (detrás del pallet)
        self.get_logger().info(f'\n📍 FASE 1: Posicionándose detrás del pallet')
        if not self.navigate_to(approach_x, approach_y, "[POSICIÓN DE EMPUJE]"):
            return False
        
        # Fase 2: Alinearse hacia el pallet (y hacia HOME)
        self.get_logger().info(f'\n🎯 FASE 2: Alineando con pallet y HOME')
        if not self.align_to(pallet_x, pallet_y):
            return False
        
        # Fase 3: Avanzar para contactar con el pallet
        self.get_logger().info(f'\n🔧 FASE 3: Contactando con pallet')
        if not self.move_forward(approach_distance - 0.3):  # Llegar hasta el pallet
            return False
        
        self.pick_pallet()
        
        # Fase 4: Empujar el pallet hasta HOME
        self.get_logger().info(f'\n🚜 FASE 4: Empujando pallet hacia HOME')
        distance_to_home = math.sqrt((HOME_POSITION[0] - self.current_x)**2 + 
                                      (HOME_POSITION[1] - self.current_y)**2)
        if not self.move_forward(distance_to_home + 1.0):  # Empujar hasta HOME y un poco más
            return False
        
        # Fase 5: Retroceder y dejar el pallet
        self.get_logger().info(f'\n🔙 FASE 5: Retrocediendo y dejando pallet')
        if not self.move_forward(-2.0):  # Retroceder 2 metros
            return False
        
        self.drop_pallet()
        
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('🎉 ¡MISIÓN COMPLETADA! Pallet movido a HOME')
        self.get_logger().info('='*50)
        
        return True


def main():
    rclpy.init()
    node = ForkliftAutonomous()
    
    print(__doc__)
    print("\n" + "="*60)
    print("  Pallets disponibles:")
    for name, pos in PALLET_POSITIONS.items():
        print(f"    - {name}: posición ({pos[0]}, {pos[1]})")
    print("="*60)
    
    # Esperar a recibir odometría
    print("\n⏳ Esperando conexión con la simulación...")
    while not node.odom_received and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    
    print(f"✅ Conectado! Posición actual: ({node.current_x:.1f}, {node.current_y:.1f})")
    
    # Menú interactivo
    while rclpy.ok():
        print("\n" + "-"*40)
        print("Opciones:")
        print("  1-5: Ir a pallet_1 hasta pallet_5")
        print("  h: Volver a HOME")
        print("  q: Salir")
        print("-"*40)
        
        try:
            choice = input("Selecciona opción: ").strip().lower()
        except EOFError:
            break
            
        if choice == 'q':
            break
        elif choice == 'h':
            node.navigate_to(HOME_POSITION[0], HOME_POSITION[1], "[HOME]")
        elif choice in ['1', '2', '3', '4', '5']:
            pallet_name = f'pallet_{choice}'
            node.execute_mission(pallet_name)
        else:
            print("❌ Opción no válida")
    
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
    print("\n✅ Programa terminado.")


if __name__ == '__main__':
    main()
