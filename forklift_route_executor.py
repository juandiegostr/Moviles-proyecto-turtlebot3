#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════════════════════════╗
║           🚜 FORKLIFT ROUTE EXECUTOR - EJECUTOR DE RUTAS 🚜                   ║
║                                                                               ║
║   Nodo ROS2 que ejecuta rutas planificadas con el Route Planner.              ║
║   Lee el archivo de ruta JSON y navega a cada waypoint, ejecutando            ║
║   las acciones de recoger/soltar pallets según corresponda.                   ║
║                                                                               ║
╚═══════════════════════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
import time
import json
import os
import sys

# Configuración
DEFAULT_ROUTE_FILE = "/root/ros2_ws/src/ruta3.json"

# Tolerancias de navegación
DISTANCE_TOLERANCE = 0.4  # metros
ANGLE_TOLERANCE = 0.1     # radianes

# Velocidades
MAX_LINEAR_VEL = 0.8   # m/s
MAX_ANGULAR_VEL = 0.7  # rad/s
APPROACH_VEL = 0.3     # m/s (velocidad de aproximación)

# Distancias para pick/drop
FORK_INSERT_DISTANCE = 1.2  # metros para insertar horquillas


class ForkliftRouteExecutor(Node):
    def __init__(self):
        super().__init__('forklift_route_executor')
        
        # Publisher para comandos de velocidad
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Publisher para estado (para UI)
        self.status_pub = self.create_publisher(String, '/forklift/status', 10)
        
        # Subscriber para odometría
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Estado actual del robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        # Estado de la misión
        self.carrying_pallet = False
        self.current_step = 0
        self.route = []
        self.paused = False
        self.abort = False
        
        self.get_logger().info('🚜 Forklift Route Executor iniciado!')
        self.publish_status("IDLE", "Esperando ruta...")
        
    def publish_status(self, state, message):
        """Publica el estado actual"""
        msg = String()
        msg.data = json.dumps({
            'state': state,
            'message': message,
            'step': self.current_step,
            'total_steps': len(self.route),
            'carrying_pallet': self.carrying_pallet,
            'position': {'x': self.current_x, 'y': self.current_y, 'yaw': self.current_yaw}
        })
        self.status_pub.publish(msg)
        
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
        
    def navigate_to(self, target_x, target_y, name="", is_approach=False):
        """Navega hacia una posición objetivo"""
        self.get_logger().info(f'📍 Navegando a {name} ({target_x:.1f}, {target_y:.1f})')
        self.publish_status("NAVIGATING", f"Navegando a {name}")
        
        max_vel = APPROACH_VEL if is_approach else MAX_LINEAR_VEL
        
        while rclpy.ok() and not self.abort:
            if self.paused:
                self.stop()
                time.sleep(0.1)
                continue
                
            if not self.odom_received:
                self.get_logger().info('⏳ Esperando odometría...')
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
                
            distance = self.get_distance_to(target_x, target_y)
            
            # ¿Llegamos?
            if distance < DISTANCE_TOLERANCE:
                self.stop()
                self.get_logger().info(f'✅ Llegamos a {name}! Distancia: {distance:.2f}m')
                self.publish_status("ARRIVED", f"Llegamos a {name}")
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
                    msg.linear.x = max_vel * 0.3
            else:
                # Avanzar hacia el objetivo
                msg.linear.x = min(max_vel, distance * 0.5)
                msg.angular.z = angle_error * 1.0  # Corrección pequeña
            
            self.cmd_pub.publish(msg)
            
            # Log de progreso
            self.get_logger().info(
                f'   📊 Dist: {distance:.1f}m | Ang: {math.degrees(angle_error):.0f}°',
                throttle_duration_sec=1.0
            )
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        return False
    
    def align_to_angle(self, target_yaw):
        """Gira para alinearse con un ángulo específico"""
        self.get_logger().info(f'🎯 Alineando a {math.degrees(target_yaw):.0f}°')
        
        while rclpy.ok() and not self.abort:
            rclpy.spin_once(self, timeout_sec=0.05)
            
            angle_error = self.normalize_angle(target_yaw - self.current_yaw)
            
            if abs(angle_error) < ANGLE_TOLERANCE:
                self.stop()
                self.get_logger().info(f'✅ Alineado!')
                return True
            
            msg = Twist()
            msg.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angle_error * 2.0))
            self.cmd_pub.publish(msg)
            
        return False
    
    def move_forward(self, distance, speed=APPROACH_VEL):
        """Avanza (o retrocede si es negativo) una distancia específica"""
        direction = "Avanzando" if distance > 0 else "Retrocediendo"
        self.get_logger().info(f'🚜 {direction} {abs(distance):.1f}m...')
        
        start_x = self.current_x
        start_y = self.current_y
        target_distance = abs(distance)
        
        while rclpy.ok() and not self.abort:
            rclpy.spin_once(self, timeout_sec=0.05)
            
            traveled = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            
            if traveled >= target_distance - 0.05:
                self.stop()
                self.get_logger().info(f'✅ Movimiento completado!')
                return True
            
            msg = Twist()
            msg.linear.x = speed if distance > 0 else -speed
            self.cmd_pub.publish(msg)
            
        return False
    
    def execute_pick(self, waypoint):
        """Ejecuta acción de recoger pallet"""
        name = waypoint.get('name', 'pallet')
        self.get_logger().info(f'📦 === RECOGIENDO {name} ===')
        self.publish_status("PICKING", f"Recogiendo {name}")
        
        if self.carrying_pallet:
            self.get_logger().warn('⚠️ Ya tienes un pallet! Soltándolo primero...')
            self.execute_drop(waypoint)
        
        # 1. Avanzar para insertar horquillas
        self.get_logger().info('🔧 Insertando horquillas...')
        self.move_forward(FORK_INSERT_DISTANCE, APPROACH_VEL)
        
        # 2. Simular levantar pallet
        self.get_logger().info('⬆️ Levantando pallet...')
        time.sleep(1.5)
        
        self.carrying_pallet = True
        self.get_logger().info(f'✅ {name} recogido!')
        self.publish_status("CARRYING", f"Transportando {name}")
        
        return True
    
    def execute_drop(self, waypoint):
        """Ejecuta acción de soltar pallet"""
        name = waypoint.get('name', 'posición')
        self.get_logger().info(f'📤 === SOLTANDO PALLET en {name} ===')
        self.publish_status("DROPPING", f"Soltando en {name}")
        
        if not self.carrying_pallet:
            self.get_logger().warn('⚠️ No hay pallet que soltar!')
            return False
        
        # 1. Simular bajar pallet
        self.get_logger().info('⬇️ Bajando pallet...')
        time.sleep(1.0)
        
        # 2. Retroceder para sacar horquillas
        self.get_logger().info('🔧 Retirando horquillas...')
        self.move_forward(-FORK_INSERT_DISTANCE, APPROACH_VEL)
        
        self.carrying_pallet = False
        self.get_logger().info(f'✅ Pallet dejado en {name}!')
        self.publish_status("EMPTY", "Sin pallet")
        
        return True
    
    def load_route(self, filepath):
        """Carga una ruta desde archivo JSON"""
        if not os.path.exists(filepath):
            self.get_logger().error(f'❌ Archivo de ruta no encontrado: {filepath}')
            return False
        
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            
            # Verificar si es formato del planificador (con nodes y route)
            if isinstance(data, dict) and 'nodes' in data and 'route' in data:
                # Formato del Route Planner
                nodes = {int(k): v for k, v in data['nodes'].items()}
                route_steps = data['route']
                
                # Convertir a formato ejecutable
                self.route = []
                for step in route_steps:
                    node_id = step['node_id']
                    node = nodes.get(node_id)
                    if node:
                        self.route.append({
                            'x': node['x'],
                            'y': node['y'],
                            'name': node['name'],
                            'action': step.get('action', 'none')
                        })
                    else:
                        self.get_logger().warn(f'⚠️ Nodo {node_id} no encontrado, saltando...')
            elif isinstance(data, list):
                # Formato simple (lista de waypoints)
                self.route = data
            else:
                self.get_logger().error('❌ Formato de archivo no reconocido')
                return False
            
            self.get_logger().info(f'📂 Ruta cargada: {len(self.route)} waypoints')
            for i, wp in enumerate(self.route):
                action_str = f" [{wp['action'].upper()}]" if wp.get('action', 'none') != 'none' else ""
                self.get_logger().info(f'   {i+1}. {wp["name"]} ({wp["x"]:.1f}, {wp["y"]:.1f}){action_str}')
            
            return True
        except Exception as e:
            self.get_logger().error(f'❌ Error cargando ruta: {e}')
            import traceback
            traceback.print_exc()
            return False
    
    def execute_route(self):
        """Ejecuta la ruta completa"""
        if not self.route:
            self.get_logger().error('❌ No hay ruta cargada!')
            return False
        
        self.get_logger().info('='*60)
        self.get_logger().info('🚀 INICIANDO EJECUCIÓN DE RUTA')
        self.get_logger().info('='*60)
        self.publish_status("EXECUTING", "Iniciando ruta...")
        
        self.current_step = 0
        self.abort = False
        
        for i, waypoint in enumerate(self.route):
            if self.abort:
                self.get_logger().warn('⚠️ Ruta abortada!')
                break
            
            self.current_step = i + 1
            name = waypoint.get('name', f'WP_{i+1}')
            x = waypoint['x']
            y = waypoint['y']
            action = waypoint.get('action', 'none')
            
            self.get_logger().info(f'\n{"="*50}')
            self.get_logger().info(f'📍 PASO {i+1}/{len(self.route)}: {name}')
            self.get_logger().info(f'{"="*50}')
            
            # Navegar al waypoint
            success = self.navigate_to(x, y, name)
            
            if not success:
                self.get_logger().error(f'❌ Error navegando a {name}')
                break
            
            # Ejecutar acción si corresponde
            if action == 'pick':
                self.execute_pick(waypoint)
            elif action == 'drop':
                self.execute_drop(waypoint)
            
            # Pequeña pausa entre waypoints
            time.sleep(0.5)
        
        # Fin de la ruta
        self.stop()
        
        if not self.abort:
            self.get_logger().info('\n' + '='*60)
            self.get_logger().info('🎉 ¡RUTA COMPLETADA EXITOSAMENTE!')
            self.get_logger().info('='*60)
            self.publish_status("COMPLETED", "Ruta completada!")
        else:
            self.publish_status("ABORTED", "Ruta abortada")
        
        return True


def print_banner():
    """Imprime el banner del programa"""
    print("""
╔═══════════════════════════════════════════════════════════════════════════════╗
║           🚜 FORKLIFT ROUTE EXECUTOR - EJECUTOR DE RUTAS 🚜                   ║
╚═══════════════════════════════════════════════════════════════════════════════╝
""")


def main():
    print_banner()
    
    rclpy.init()
    node = ForkliftRouteExecutor()
    
    # Cargar archivo de ruta
    route_file = DEFAULT_ROUTE_FILE
    if len(sys.argv) > 1:
        route_file = sys.argv[1]
    
    print(f"\n📂 Archivo de ruta: {route_file}")
    
    if not node.load_route(route_file):
        print("❌ No se pudo cargar la ruta. Asegúrate de crear una ruta con el Route Planner.")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Esperar odometría
    print("\n⏳ Esperando conexión con la simulación...")
    while not node.odom_received and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    
    print(f"✅ Conectado! Posición actual: ({node.current_x:.1f}, {node.current_y:.1f})")
    
    # Menú interactivo
    print("\n" + "-"*50)
    print("Comandos:")
    print("  ENTER : Iniciar ejecución de ruta")
    print("  p     : Pausar/Reanudar")
    print("  a     : Abortar ruta")
    print("  q     : Salir")
    print("-"*50)
    
    import threading
    
    # Flag para controlar el hilo
    running = True
    
    def input_handler():
        nonlocal running
        while running and rclpy.ok():
            try:
                cmd = input().strip().lower()
                if cmd == '':
                    if not node.route:
                        print("❌ No hay ruta cargada")
                    else:
                        # Ejecutar en hilo separado
                        exec_thread = threading.Thread(target=node.execute_route)
                        exec_thread.start()
                elif cmd == 'p':
                    node.paused = not node.paused
                    state = "PAUSADO ⏸️" if node.paused else "REANUDADO ▶️"
                    print(f"\n{state}")
                elif cmd == 'a':
                    node.abort = True
                    print("\n⚠️ Abortando ruta...")
                elif cmd == 'q':
                    running = False
                    node.abort = True
                    break
            except EOFError:
                break
    
    # Iniciar hilo de entrada
    input_thread = threading.Thread(target=input_handler, daemon=True)
    input_thread.start()
    
    # Spin principal
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    
    running = False
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
    print("\n✅ Programa terminado.")


if __name__ == '__main__':
    main()
