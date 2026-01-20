#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════════════════════════╗
║           🚜 FORKLIFT ROUTE EXECUTOR - EJECUTOR DE RUTAS 🚜                   ║
║                                                                               ║
║   Nodo ROS2 que ejecuta rutas planificadas con el Route Planner.              ║
║   Lee el archivo de ruta JSON y navega a cada waypoint, ejecutando            ║
║   las acciones de recoger/soltar pallets usando la API de mvsim.              ║
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
import threading

# Cliente mvsim para mover objetos (agarre de pallets)
try:
    from mvsim_comms import pymvsim_comms
    from mvsim_msgs import SrvSetPose_pb2, SrvGetPose_pb2
    MVSIM_AVAILABLE = True
except ImportError:
    MVSIM_AVAILABLE = False
    print("⚠️  mvsim_comms no disponible - función de agarre será simulada")

# Configuración
DEFAULT_ROUTE_FILE = "/root/ros2_ws/src/ruta3.json"

# Tolerancias de navegación
DISTANCE_TOLERANCE = 0.4  # metros
ANGLE_TOLERANCE = 0.1     # radianes

# Velocidades
MAX_LINEAR_VEL = 0.8   # m/s
MAX_ANGULAR_VEL = 0.7  # rad/s
APPROACH_VEL = 0.3     # m/s (velocidad de aproximación)

# Offset del pallet cuando está enganchado
GRASP_OFFSET_X = 1.5    # metros delante del forklift
GRASP_OFFSET_Z = 0.15   # altura del pallet levantado

# Lista de pallets disponibles
PALLET_POSITIONS = {
    'pallet_1': (-17.0, 8.0),
    'pallet_2': (-15.0, 8.0),
    'pallet_3': (-13.0, 8.0),
    'pallet_4': (-17.0, 5.0),
    'pallet_5': (-15.0, 5.0),
    'pallet_6': (-17.0, -8.0),
    'pallet_7': (-15.0, -8.0),
    'pallet_8': (-13.0, -8.0),
    'pallet_9': (-17.0, -5.0),
    'pallet_10': (4.0, 6.0),
    'pallet_11': (4.0, 4.8),
    'pallet_12': (12.0, -6.0),
    'pallet_13': (12.0, -4.9),
}


class PalletGrasper:
    """Maneja el enganche y desenganche de pallets via mvsim API"""
    
    def __init__(self):
        self.client = None
        self.connected = False
        self.grasped_pallet = None
        self.running = False
        self.update_thread = None
        
        # Posición del forklift
        self.forklift_x = 0.0
        self.forklift_y = 0.0
        self.forklift_yaw = 0.0
        
        # Posiciones actuales de los pallets
        self.pallet_positions = dict(PALLET_POSITIONS)
        
    def connect(self):
        """Conecta al servidor mvsim"""
        if not MVSIM_AVAILABLE:
            print("⚠️  mvsim_comms no disponible")
            return False
            
        try:
            self.client = pymvsim_comms.mvsim.Client()
            self.client.setName("forklift_route_executor_grasper")
            self.client.connect()
            self.connected = True
            print("✅ Sistema de agarre conectado a mvsim")
            return True
        except Exception as e:
            print(f"⚠️  Error conectando a mvsim: {e}")
            self.connected = False
            return False
    
    def find_nearest_pallet(self):
        """Encuentra el pallet más cercano al forklift"""
        nearest = None
        min_dist = float('inf')
        
        for pallet_name, (px, py) in self.pallet_positions.items():
            dist = math.sqrt((px - self.forklift_x)**2 + (py - self.forklift_y)**2)
            if dist < min_dist:
                min_dist = dist
                nearest = pallet_name
        
        return nearest, min_dist
    
    def find_pallet_by_name(self, name):
        """Busca un pallet por nombre (parcial o completo)"""
        name_lower = name.lower().replace('_', '').replace('-', '').replace(' ', '')
        
        for pallet_name in self.pallet_positions.keys():
            pallet_lower = pallet_name.lower().replace('_', '')
            if name_lower in pallet_lower or pallet_lower in name_lower:
                return pallet_name
        
        # Buscar por número
        import re
        numbers = re.findall(r'\d+', name)
        if numbers:
            num = numbers[0]
            for pallet_name in self.pallet_positions.keys():
                if num in pallet_name:
                    return pallet_name
        
        return None
    
    def set_object_pose(self, name, x, y, z, yaw):
        """Mueve un objeto a una posición"""
        if not self.connected:
            return False
        try:
            req = SrvSetPose_pb2.SrvSetPose()
            req.objectId = name
            req.pose.x = x
            req.pose.y = y
            req.pose.z = z
            req.pose.yaw = yaw
            req.pose.pitch = 0.0
            req.pose.roll = 0.0
            self.client.callService('set_pose', req.SerializeToString())
            return True
        except Exception as e:
            print(f"⚠️  Error moviendo objeto: {e}")
            return False
    
    def grasp(self, pallet_name):
        """Engancha un pallet específico"""
        if self.grasped_pallet:
            return False, f"Ya tienes enganchado: {self.grasped_pallet}"
        
        if not self.connected:
            if not self.connect():
                return False, "No conectado a mvsim"
        
        # Verificar que el pallet existe
        if pallet_name not in self.pallet_positions:
            return False, f"Pallet '{pallet_name}' no existe"
        
        self.grasped_pallet = pallet_name
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
        return True, f"✅ Pallet '{pallet_name}' ENGANCHADO"
    
    def grasp_nearest(self):
        """Engancha el pallet más cercano"""
        if self.grasped_pallet:
            return False, f"Ya tienes enganchado: {self.grasped_pallet}"
        
        nearest, dist = self.find_nearest_pallet()
        
        if nearest is None:
            return False, "No se encontraron pallets"
        
        if dist > 3.0:
            return False, f"Pallet más cercano ({nearest}) está a {dist:.1f}m (máx: 3.0m)"
        
        return self.grasp(nearest)
    
    def release(self):
        """Desengancha el pallet actual y lo deja en el suelo"""
        if not self.grasped_pallet:
            return False, "No hay pallet enganchado"
        
        released = self.grasped_pallet
        
        # Detener el hilo de actualización
        self.running = False
        if self.update_thread:
            self.update_thread.join(timeout=0.5)
        
        # Dejar el pallet en el suelo (z=0)
        if self.connected:
            cos_yaw = math.cos(self.forklift_yaw)
            sin_yaw = math.sin(self.forklift_yaw)
            final_x = self.forklift_x + GRASP_OFFSET_X * cos_yaw
            final_y = self.forklift_y + GRASP_OFFSET_X * sin_yaw
            
            self.set_object_pose(released, final_x, final_y, 0.08, self.forklift_yaw)
            
            # Actualizar posición guardada
            self.pallet_positions[released] = (final_x, final_y)
        
        self.grasped_pallet = None
        return True, f"✅ Pallet '{released}' SOLTADO"
    
    def update_forklift_pose(self, x, y, yaw):
        """Actualiza la posición del forklift"""
        self.forklift_x = x
        self.forklift_y = y
        self.forklift_yaw = yaw
    
    def _update_loop(self):
        """Actualiza la posición del pallet enganchado continuamente"""
        while self.running and self.grasped_pallet:
            try:
                cos_yaw = math.cos(self.forklift_yaw)
                sin_yaw = math.sin(self.forklift_yaw)
                
                # Posición del pallet delante del forklift
                global_x = self.forklift_x + GRASP_OFFSET_X * cos_yaw
                global_y = self.forklift_y + GRASP_OFFSET_X * sin_yaw
                
                # Actualizar posición guardada del pallet
                self.pallet_positions[self.grasped_pallet] = (global_x, global_y)
                
                self.set_object_pose(
                    self.grasped_pallet,
                    global_x,
                    global_y,
                    GRASP_OFFSET_Z,
                    self.forklift_yaw
                )
                time.sleep(0.02)  # 50 Hz
            except:
                time.sleep(0.1)


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
        self.current_step = 0
        self.route = []
        self.paused = False
        self.abort = False
        
        # Sistema de agarre de pallets (REAL)
        self.grasper = PalletGrasper()
        
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
            'carrying_pallet': self.grasper.grasped_pallet,
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
        
        # Actualizar posición en el grasper
        self.grasper.update_forklift_pose(self.current_x, self.current_y, self.current_yaw)
        
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
                msg.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angle_error * 2.0))
                if abs(angle_error) < 0.5:
                    msg.linear.x = max_vel * 0.3
            else:
                msg.linear.x = min(max_vel, distance * 0.5)
                msg.angular.z = angle_error * 1.0
            
            self.cmd_pub.publish(msg)
            
            self.get_logger().info(
                f'   📊 Dist: {distance:.1f}m | Ang: {math.degrees(angle_error):.0f}°',
                throttle_duration_sec=1.0
            )
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        return False
    
    def execute_pick(self, waypoint):
        """Ejecuta acción de recoger pallet"""
        name = waypoint.get('name', 'pallet')
        self.get_logger().info(f'📦 === RECOGIENDO EN {name} ===')
        self.publish_status("PICKING", f"Recogiendo en {name}")
        
        # Buscar el pallet por nombre
        pallet_name = self.grasper.find_pallet_by_name(name)
        
        if pallet_name:
            self.get_logger().info(f'🎯 Pallet identificado: {pallet_name}')
            success, msg = self.grasper.grasp(pallet_name)
        else:
            # Si no se encuentra por nombre, buscar el más cercano
            self.get_logger().info(f'🔍 Buscando pallet más cercano...')
            success, msg = self.grasper.grasp_nearest()
        
        self.get_logger().info(msg)
        
        if success:
            self.publish_status("CARRYING", f"Transportando pallet")
            time.sleep(0.5)  # Pequeña pausa para visualizar
        
        return success
    
    def execute_drop(self, waypoint):
        """Ejecuta acción de soltar pallet"""
        name = waypoint.get('name', 'posición')
        self.get_logger().info(f'📤 === SOLTANDO PALLET EN {name} ===')
        self.publish_status("DROPPING", f"Soltando en {name}")
        
        success, msg = self.grasper.release()
        self.get_logger().info(msg)
        
        if success:
            self.publish_status("EMPTY", "Sin pallet")
            time.sleep(0.5)
        
        return success
    
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
                nodes = {int(k): v for k, v in data['nodes'].items()}
                route_steps = data['route']
                
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
            if action != 'none':
                action_text = "📦 RECOGER" if action == 'pick' else "📤 SOLTAR"
                self.get_logger().info(f'   Acción: {action_text}')
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
            if self.grasper.grasped_pallet:
                self.get_logger().info(f'📦 Nota: Aún tienes enganchado: {self.grasper.grasped_pallet}')
            self.get_logger().info('='*60)
            self.publish_status("COMPLETED", "Ruta completada!")
        else:
            self.publish_status("ABORTED", "Ruta abortada")
        
        return True


def print_banner():
    print("""
╔═══════════════════════════════════════════════════════════════════════════════╗
║           🚜 FORKLIFT ROUTE EXECUTOR - EJECUTOR DE RUTAS 🚜                   ║
║                                                                               ║
║   Ejecuta rutas con AGARRE REAL de pallets usando mvsim API                  ║
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
        print("❌ No se pudo cargar la ruta.")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Conectar sistema de agarre
    print("\n🔧 Conectando sistema de agarre...")
    node.grasper.connect()
    
    # Esperar odometría
    print("\n⏳ Esperando conexión con la simulación...")
    while not node.odom_received and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    
    print(f"✅ Conectado! Posición actual: ({node.current_x:.1f}, {node.current_y:.1f})")
    
    # Menú
    print("\n" + "-"*50)
    print("Comandos:")
    print("  ENTER : Iniciar ejecución de ruta")
    print("  p     : Pausar/Reanudar")
    print("  a     : Abortar ruta")
    print("  q     : Salir")
    print("-"*50)
    
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
                        exec_thread = threading.Thread(target=node.execute_route)
                        exec_thread.start()
                elif cmd == 'p':
                    node.paused = not node.paused
                    state = "⏸️ PAUSADO" if node.paused else "▶️ REANUDADO"
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
    
    input_thread = threading.Thread(target=input_handler, daemon=True)
    input_thread.start()
    
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    
    running = False
    
    # Liberar pallet si está enganchado
    if node.grasper.grasped_pallet:
        print(f"\n📤 Soltando pallet {node.grasper.grasped_pallet}...")
        node.grasper.release()
    
    try:
        node.stop()
    except:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
    print("\n✅ Programa terminado.")


if __name__ == '__main__':
    main()
