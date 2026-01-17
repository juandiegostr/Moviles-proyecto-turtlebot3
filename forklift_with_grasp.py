#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════════╗
║    🚜 FORKLIFT CON AGARRE REALISTA (via mvsim API) 🚜         ║
║                                                               ║
║   Este programa usa la API interna de mvsim para mover        ║
║   los pallets junto con el forklift, simulando un agarre     ║
║   físico realista.                                           ║
║                                                               ║
║   ¡El pallet se mueve junto con el forklift!                 ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import threading

# Cliente mvsim para mover objetos
from mvsim_comms import pymvsim_comms
from mvsim_msgs import SrvSetPose_pb2, SrvGetPose_pb2

# ═══════════════════════════════════════════════════════════════
# CONFIGURACIÓN
# ═══════════════════════════════════════════════════════════════

# Posiciones de los pallets en el almacén
# IMPORTANTE: Los nombres deben coincidir exactamente con los del archivo XML
PALLET_POSITIONS = {
    # Zona de recepción (noroeste)
    'pallet_1': (-17.0, 8.0),
    'pallet_2': (-15.0, 8.0),
    'pallet_3': (-13.0, 8.0),
    'pallet_4': (-17.0, 5.0),
    'pallet_5': (-15.0, 5.0),
    # Zona de expedición (suroeste)
    'pallet_6': (-17.0, -8.0),
    'pallet_7': (-15.0, -8.0),
    'pallet_8': (-13.0, -8.0),
    'pallet_9': (-17.0, -5.0),
    # Pallets en pasillos
    'pallet_10': (4.0, 6.0),
    'pallet_11': (4.0, 4.8),
    'pallet_12': (12.0, -6.0),
    'pallet_13': (12.0, -4.9),
}

# Posición de inicio/descarga
HOME_POSITION = (-15.0, 0.0)

# Offset del pallet relativo al forklift cuando está "agarrado"
# (delante del forklift, a distancia de las horquillas)
GRASP_OFFSET_X = 1.5  # metros delante del forklift
GRASP_OFFSET_Z = 0.15  # altura del pallet cuando está levantado

# Tolerancias de navegación
DISTANCE_TOLERANCE = 0.3
ANGLE_TOLERANCE = 0.08

# Velocidades
MAX_LINEAR_VEL = 1.0
MAX_ANGULAR_VEL = 0.8


class ForkliftGraspSimulator:
    """
    Clase que simula el agarre moviendo el pallet con el forklift
    usando la API de mvsim directamente.
    """
    def __init__(self):
        self.client = None
        self.connected = False
        self.grasped_object = None  # Nombre del objeto agarrado
        self.grasp_offset = (GRASP_OFFSET_X, 0.0, GRASP_OFFSET_Z)  # x, y, z relativo
        self.running = False
        self.update_thread = None
        
        # Posición actual del forklift (actualizada externamente)
        self.forklift_x = 0.0
        self.forklift_y = 0.0
        self.forklift_yaw = 0.0
        
    def connect(self):
        """Conecta al servidor mvsim"""
        try:
            self.client = pymvsim_comms.mvsim.Client()
            self.client.setName("forklift_grasp_simulator")
            print("🔌 Conectando a mvsim...")
            self.client.connect()
            self.connected = True
            print("✅ Conectado a mvsim correctamente")
            return True
        except Exception as e:
            print(f"❌ Error conectando a mvsim: {e}")
            self.connected = False
            return False
    
    def set_object_pose(self, object_name, x, y, z, yaw):
        """Mueve un objeto a una posición específica"""
        if not self.connected:
            return False
            
        try:
            req = SrvSetPose_pb2.SrvSetPose()
            req.objectId = object_name
            req.pose.x = x
            req.pose.y = y
            req.pose.z = z
            req.pose.yaw = yaw
            req.pose.pitch = 0.0
            req.pose.roll = 0.0
            
            self.client.callService('set_pose', req.SerializeToString())
            return True
        except Exception as e:
            print(f"⚠️ Error moviendo {object_name}: {e}")
            return False
    
    def grasp(self, object_name):
        """Agarra un objeto (comienza a sincronizarlo con el forklift)"""
        if self.grasped_object:
            print(f"⚠️ Ya hay un objeto agarrado: {self.grasped_object}")
            return False
            
        self.grasped_object = object_name
        self.running = True
        
        # Iniciar thread de actualización
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
        
        print(f"🤏 ¡Objeto '{object_name}' agarrado!")
        return True
    
    def release(self):
        """Suelta el objeto agarrado"""
        if not self.grasped_object:
            print("⚠️ No hay objeto agarrado para soltar")
            return False
            
        released = self.grasped_object
        self.running = False
        self.grasped_object = None
        
        if self.update_thread:
            self.update_thread.join(timeout=1.0)
            
        print(f"📦 Objeto '{released}' soltado")
        return True
    
    def update_forklift_pose(self, x, y, yaw):
        """Actualiza la posición del forklift (llamar desde odom callback)"""
        self.forklift_x = x
        self.forklift_y = y
        self.forklift_yaw = yaw
    
    def _update_loop(self):
        """Loop que actualiza la posición del objeto agarrado"""
        while self.running and self.grasped_object:
            try:
                # Calcular posición del pallet relativa al forklift
                # El pallet está delante del forklift
                cos_yaw = math.cos(self.forklift_yaw)
                sin_yaw = math.sin(self.forklift_yaw)
                
                # Transformar offset local a global
                ox, oy, oz = self.grasp_offset
                global_x = self.forklift_x + ox * cos_yaw - oy * sin_yaw
                global_y = self.forklift_y + ox * sin_yaw + oy * cos_yaw
                
                # Mover el pallet
                self.set_object_pose(
                    self.grasped_object,
                    global_x,
                    global_y,
                    oz,  # Altura cuando está levantado
                    self.forklift_yaw
                )
                
                time.sleep(0.02)  # 50 Hz de actualización
                
            except Exception as e:
                print(f"⚠️ Error en update loop: {e}")
                time.sleep(0.1)


class ForkliftWithGrasp(Node):
    """
    Nodo ROS2 para control del forklift con simulación de agarre
    """
    def __init__(self):
        super().__init__('forklift_with_grasp')
        
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
        
        # Simulador de agarre
        self.grasp_sim = ForkliftGraspSimulator()
        
        self.get_logger().info('🚜 Forklift con Agarre iniciado!')
        
    def connect_grasp_simulator(self):
        """Conecta el simulador de agarre a mvsim"""
        return self.grasp_sim.connect()
        
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
        
        # Actualizar posición en el simulador de agarre
        self.grasp_sim.update_forklift_pose(
            self.current_x,
            self.current_y,
            self.current_yaw
        )
        
    def get_distance_to(self, target_x, target_y):
        return math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)
    
    def get_angle_to(self, target_x, target_y):
        return math.atan2(target_y - self.current_y, target_x - self.current_x)
    
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def stop(self):
        msg = Twist()
        self.cmd_pub.publish(msg)
        
    def navigate_to(self, target_x, target_y, description=""):
        """Navega hacia una posición objetivo"""
        self.get_logger().info(f'📍 Navegando a ({target_x:.1f}, {target_y:.1f}) {description}')
        
        while rclpy.ok():
            if not self.odom_received:
                self.get_logger().info('⏳ Esperando odometría...')
                rclpy.spin_once(self, timeout_sec=0.1)
                continue
                
            distance = self.get_distance_to(target_x, target_y)
            
            if distance < DISTANCE_TOLERANCE:
                self.stop()
                self.get_logger().info(f'✅ ¡Llegamos! Distancia: {distance:.2f}m')
                return True
            
            target_angle = self.get_angle_to(target_x, target_y)
            angle_error = self.normalize_angle(target_angle - self.current_yaw)
            
            msg = Twist()
            
            if abs(angle_error) > ANGLE_TOLERANCE:
                msg.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angle_error * 2.0))
                if abs(angle_error) < 0.5:
                    msg.linear.x = MAX_LINEAR_VEL * 0.5
            else:
                msg.linear.x = min(MAX_LINEAR_VEL, distance * 0.5)
                msg.angular.z = angle_error * 1.0
            
            self.cmd_pub.publish(msg)
            
            self.get_logger().info(
                f'   📊 Pos: ({self.current_x:.1f}, {self.current_y:.1f}) | '
                f'Dist: {distance:.1f}m | Ang: {math.degrees(angle_error):.0f}°',
                throttle_duration_sec=1.0
            )
            
            rclpy.spin_once(self, timeout_sec=0.05)
        
        return False
    
    def align_to(self, target_x, target_y):
        """Gira para alinearse con el objetivo"""
        self.get_logger().info(f'🎯 Alineando hacia ({target_x:.1f}, {target_y:.1f})')
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            
            target_angle = self.get_angle_to(target_x, target_y)
            angle_error = self.normalize_angle(target_angle - self.current_yaw)
            
            if abs(angle_error) < ANGLE_TOLERANCE:
                self.stop()
                return True
            
            msg = Twist()
            msg.angular.z = max(-MAX_ANGULAR_VEL, min(MAX_ANGULAR_VEL, angle_error * 2.0))
            self.cmd_pub.publish(msg)
            
        return False
    
    def move_forward(self, distance):
        """Avanza una distancia específica"""
        direction = "Avanzando" if distance > 0 else "Retrocediendo"
        self.get_logger().info(f'🚜 {direction} {abs(distance):.1f}m...')
        
        start_x = self.current_x
        start_y = self.current_y
        target_distance = abs(distance)
        speed = 0.4 if distance > 0 else -0.4
        
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            
            traveled = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            
            if traveled >= target_distance - 0.05:
                self.stop()
                return True
            
            msg = Twist()
            msg.linear.x = speed
            self.cmd_pub.publish(msg)
            
        return False
    
    def execute_pick_and_place(self, pallet_name):
        """Ejecuta una misión de pick & place con agarre realista"""
        
        if pallet_name not in PALLET_POSITIONS:
            self.get_logger().error(f'❌ Pallet "{pallet_name}" no existe')
            return False
        
        pallet_x, pallet_y = PALLET_POSITIONS[pallet_name]
        home_x, home_y = HOME_POSITION
        
        print("\n" + "="*60)
        print(f"  🎯 MISIÓN: Recoger {pallet_name} y llevarlo a HOME")
        print("="*60 + "\n")
        
        # Paso 1: Navegar cerca del pallet
        approach_x = pallet_x - 2.0  # Acercarse desde un lado
        self.navigate_to(approach_x, pallet_y, "(aproximación)")
        
        # Paso 2: Alinearse con el pallet
        self.align_to(pallet_x, pallet_y)
        time.sleep(0.5)
        
        # Paso 3: Avanzar para insertar horquillas
        self.get_logger().info("🔧 Insertando horquillas...")
        self.move_forward(1.5)
        time.sleep(0.5)
        
        # Paso 4: ¡AGARRAR EL PALLET!
        print("\n🤏 ¡AGARRANDO PALLET!")
        self.grasp_sim.grasp(pallet_name)
        time.sleep(1.0)  # Dar tiempo para que el agarre se estabilice
        
        # Paso 5: Retroceder un poco para despejar el área
        self.get_logger().info("⬅️ Retrocediendo con el pallet...")
        self.move_forward(-1.0)
        
        # Paso 6: Navegar a HOME (el pallet va CON el forklift!)
        self.navigate_to(home_x, home_y, "(llevando pallet a HOME)")
        
        # Paso 7: Soltar el pallet
        print("\n📦 ¡SOLTANDO PALLET!")
        self.grasp_sim.release()
        time.sleep(0.5)
        
        # Paso 8: Retroceder para despejar
        self.move_forward(-1.5)
        
        print("\n" + "="*60)
        print("  ✅ ¡MISIÓN COMPLETADA!")
        print("="*60 + "\n")
        
        return True


def main():
    rclpy.init()
    
    node = ForkliftWithGrasp()
    
    # Esperar odometría
    print("⏳ Esperando odometría...")
    while not node.odom_received and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.5)
    print("✅ Odometría recibida")
    
    # Conectar simulador de agarre
    print("\n🔌 Conectando al simulador de agarre...")
    if not node.connect_grasp_simulator():
        print("❌ No se pudo conectar al simulador de agarre")
        print("   Asegúrate de que mvsim esté corriendo")
        rclpy.shutdown()
        return
    
    # Menú interactivo
    while rclpy.ok():
        print("\n" + "="*60)
        print("  🚜 FORKLIFT CON AGARRE REALISTA (via mvsim API) 🚜")
        print("="*60)
        print("\n📦 Pallets disponibles:")
        print("  ┌── Zona Recepción (noroeste) ──")
        print("  │  1: pallet_1 (-17, 8)")
        print("  │  2: pallet_2 (-15, 8)")
        print("  │  3: pallet_3 (-13, 8)")
        print("  │  4: pallet_4 (-17, 5)")
        print("  │  5: pallet_5 (-15, 5)")
        print("  │")
        print("  ├── Zona Expedición (suroeste) ──")
        print("  │  6: pallet_6 (-17, -8)")
        print("  │  7: pallet_7 (-15, -8)")
        print("  │  8: pallet_8 (-13, -8)")
        print("  │  9: pallet_9 (-17, -5)")
        print("  │")
        print("  └── Pasillos ──")
        print("     10: pallet_10 (4, 6)")
        print("     11: pallet_11 (4, 4.8)")
        print("     12: pallet_12 (12, -6)")
        print("     13: pallet_13 (12, -4.9)")
        print(f"\n🏠 HOME: {HOME_POSITION}")
        print("\n📍 Posición actual: ({:.1f}, {:.1f})".format(node.current_x, node.current_y))
        print("\n⌨️  Opciones:")
        print("  1-13 → Recoger pallet por número")
        print("  q    → Salir")
        
        try:
            choice = input("\n¿Qué pallet quieres recoger? ").strip().lower()
            
            if choice == 'q':
                break
            elif choice.isdigit() and 1 <= int(choice) <= 13:
                pallet_name = f'pallet_{choice}'
                if pallet_name in PALLET_POSITIONS:
                    node.execute_pick_and_place(pallet_name)
                else:
                    print(f"❌ Pallet {pallet_name} no encontrado")
            else:
                print("❌ Opción no válida (escribe 1-13 o 'q')")
                
        except KeyboardInterrupt:
            break
    
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
    print("\n👋 ¡Hasta luego!")


if __name__ == '__main__':
    main()
