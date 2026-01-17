#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════════╗
║        🚜 FORKLIFT TELEOP SIMPLE - Control Preciso 🚜         ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║   MOVIMIENTO:                                                 ║
║   ┌───┐                                                       ║
║   │ W │  W = Avanzar                                          ║
║   ├───┤  S = Retroceder                                       ║
║   │ S │  A = Girar izquierda (paso a paso)                    ║
║   └───┘  D = Girar derecha (paso a paso)                      ║
║   ┌───┬───┐                                                   ║
║   │ A │ D │  X / ESPACIO = Parar                              ║
║   └───┴───┘                                                   ║
║                                                               ║
║   PALLET:                                                     ║
║   ┌───┐                                                       ║
║   │ G │  G = Enganchar pallet más cercano                     ║
║   ├───┤  H = Desenganchar/Soltar pallet                       ║
║   │ H │                                                       ║
║   └───┘                                                       ║
║                                                               ║
║   1-9 = Enganchar pallet específico (pallet_1 a pallet_9)     ║
║                                                               ║
║   ESC = Salir                                                 ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
import termios
import tty
import select
import threading
import math
import time

# Cliente mvsim para mover objetos (agarre de pallets)
try:
    from mvsim_comms import pymvsim_comms
    from mvsim_msgs import SrvSetPose_pb2, SrvGetPose_pb2
    MVSIM_AVAILABLE = True
except ImportError:
    MVSIM_AVAILABLE = False
    print("⚠️  mvsim_comms no disponible - función de agarre deshabilitada")


# ═══════════════════════════════════════════════════════════════
# CONFIGURACIÓN
# ═══════════════════════════════════════════════════════════════

# Velocidades
LINEAR_SPEED = 1.0      # m/s para avanzar/retroceder
TURN_DEGREES = 5.0      # Grados por cada pulsación de A/D
TURN_SPEED = 0.8        # rad/s para el giro

# Offset del pallet cuando está enganchado
GRASP_OFFSET_X = 1.5    # metros delante del forklift
GRASP_OFFSET_Z = 0.15   # altura del pallet levantado

# Lista de pallets disponibles con sus posiciones iniciales
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
PALLET_NAMES = list(PALLET_POSITIONS.keys())

# Distancia máxima para enganchar un pallet (metros)
MAX_GRASP_DISTANCE = 3.0


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
        
        # Posiciones actuales de los pallets (se actualizan cuando se mueven)
        self.pallet_positions = dict(PALLET_POSITIONS)
        
    def connect(self):
        """Conecta al servidor mvsim"""
        if not MVSIM_AVAILABLE:
            return False
            
        try:
            self.client = pymvsim_comms.mvsim.Client()
            self.client.setName("forklift_pallet_grasper")
            self.client.connect()
            self.connected = True
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
        except:
            return False
    
    def grasp(self, pallet_name):
        """Engancha un pallet"""
        if self.grasped_pallet:
            return False, f"Ya tienes enganchado: {self.grasped_pallet}"
        
        if not self.connected:
            if not self.connect():
                return False, "No conectado a mvsim"
        
        self.grasped_pallet = pallet_name
        self.running = True
        self.update_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.update_thread.start()
        return True, f"Pallet '{pallet_name}' enganchado"
    
    def grasp_nearest(self):
        """Engancha el pallet más cercano"""
        if self.grasped_pallet:
            return False, f"Ya tienes enganchado: {self.grasped_pallet}"
        
        nearest, dist = self.find_nearest_pallet()
        
        if nearest is None:
            return False, "No se encontraron pallets"
        
        if dist > MAX_GRASP_DISTANCE:
            return False, f"Pallet más cercano ({nearest}) está a {dist:.1f}m (máx: {MAX_GRASP_DISTANCE}m)"
        
        return self.grasp(nearest)
    
    def release(self):
        """Desengancha el pallet actual"""
        if not self.grasped_pallet:
            return False, "No hay pallet enganchado"
        
        released = self.grasped_pallet
        self.running = False
        self.grasped_pallet = None
        
        if self.update_thread:
            self.update_thread.join(timeout=0.5)
        
        return True, f"Pallet '{released}' soltado"
    
    def update_forklift_pose(self, x, y, yaw):
        """Actualiza la posición del forklift"""
        self.forklift_x = x
        self.forklift_y = y
        self.forklift_yaw = yaw
    
    def _update_loop(self):
        """Actualiza la posición del pallet enganchado"""
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
                time.sleep(0.02)
            except:
                time.sleep(0.1)


class ForkliftSimpleTeleop(Node):
    def __init__(self):
        super().__init__('forklift_simple_teleop')
        
        # Publisher para cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber para odometría
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        
        # Estado del robot
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
        # Estado de movimiento
        self.moving_forward = False
        self.moving_backward = False
        self.turning = False
        self.turn_target = 0.0
        
        # Sistema de agarre
        self.grasper = PalletGrasper()
        
        # Timer para control continuo
        self.timer = self.create_timer(0.02, self.control_loop)
        
        self.running = True
        self.get_logger().info('🚜 Forklift Simple Teleop iniciado!')
        
    def odom_callback(self, msg):
        """Actualiza posición desde odometría"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Extraer yaw del quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_received = True
        
        # Actualizar posición en el grasper
        self.grasper.update_forklift_pose(
            self.current_x, self.current_y, self.current_yaw)
    
    def normalize_angle(self, angle):
        """Normaliza ángulo a [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def control_loop(self):
        """Loop de control principal"""
        msg = Twist()
        
        if self.turning and self.odom_received:
            # Ejecutando giro discreto
            angle_error = self.normalize_angle(self.turn_target - self.current_yaw)
            
            if abs(angle_error) < 0.02:  # ~1 grado de tolerancia
                # Giro completado
                self.turning = False
                msg.angular.z = 0.0
            else:
                # Continuar girando
                msg.angular.z = TURN_SPEED if angle_error > 0 else -TURN_SPEED
                
        elif self.moving_forward:
            msg.linear.x = LINEAR_SPEED
            
        elif self.moving_backward:
            msg.linear.x = -LINEAR_SPEED
        
        self.cmd_pub.publish(msg)
    
    def start_forward(self):
        """Comienza a avanzar"""
        self.moving_forward = True
        self.moving_backward = False
        self.turning = False
        
    def start_backward(self):
        """Comienza a retroceder"""
        self.moving_backward = True
        self.moving_forward = False
        self.turning = False
        
    def turn_left(self):
        """Gira a la izquierda un paso fijo"""
        if not self.odom_received:
            return
        self.moving_forward = False
        self.moving_backward = False
        self.turning = True
        self.turn_target = self.normalize_angle(
            self.current_yaw + math.radians(TURN_DEGREES))
        
    def turn_right(self):
        """Gira a la derecha un paso fijo"""
        if not self.odom_received:
            return
        self.moving_forward = False
        self.moving_backward = False
        self.turning = True
        self.turn_target = self.normalize_angle(
            self.current_yaw - math.radians(TURN_DEGREES))
    
    def stop(self):
        """Detiene todo movimiento"""
        self.moving_forward = False
        self.moving_backward = False
        self.turning = False
        msg = Twist()
        self.cmd_pub.publish(msg)
    
    def grasp_pallet(self, pallet_name):
        """Engancha un pallet específico"""
        return self.grasper.grasp(pallet_name)
    
    def grasp_nearest_pallet(self):
        """Engancha el pallet más cercano"""
        return self.grasper.grasp_nearest()
    
    def release_pallet(self):
        """Suelta el pallet enganchado"""
        return self.grasper.release()
    
    def get_status(self):
        """Devuelve el estado actual"""
        if self.grasper.grasped_pallet:
            pallet_status = f"🔗 {self.grasper.grasped_pallet}"
        else:
            pallet_status = "⚪ Sin pallet"
        
        if self.turning:
            move_status = "↻ Girando..."
        elif self.moving_forward:
            move_status = "↑ Avanzando"
        elif self.moving_backward:
            move_status = "↓ Retrocediendo"
        else:
            move_status = "■ Parado"
        
        return pallet_status, move_status
    
    def shutdown(self):
        """Apaga el nodo"""
        self.running = False
        self.stop()
        self.grasper.running = False


def get_key(settings, timeout=0.1):
    """Lee una tecla del teclado"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_status(node):
    """Imprime el estado actual en una línea"""
    pallet, move = node.get_status()
    pos = f"({node.current_x:.1f}, {node.current_y:.1f})"
    yaw_deg = math.degrees(node.current_yaw)
    
    status = f"\r\033[K  {move} | {pallet} | Pos: {pos} | Yaw: {yaw_deg:.0f}°  "
    print(status, end='', flush=True)


def main():
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = ForkliftSimpleTeleop()
    
    # Spinner en thread separado
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Conectar al grasper
    if MVSIM_AVAILABLE:
        if node.grasper.connect():
            print("✅ Sistema de agarre conectado")
        else:
            print("⚠️  Sistema de agarre no disponible")
    
    print(__doc__)
    print("\n" + "="*65)
    print("  🎮 Controla el forklift. ESC para salir.")
    print("="*65 + "\n")
    
    try:
        while node.running:
            key = get_key(settings)
            
            # Ignorar teclas vacías
            if not key:
                print_status(node)
                continue
            
            if key == '\x1b':  # ESC
                print("\n\n🛑 Saliendo...")
                break
                
            elif key.lower() == 'w':
                node.start_forward()
                
            elif key.lower() == 's':
                node.start_backward()
                
            elif key.lower() == 'a':
                node.turn_left()
                
            elif key.lower() == 'd':
                node.turn_right()
                
            elif key.lower() == 'x' or key == ' ':
                node.stop()
                
            elif key.lower() == 'g':
                # Enganchar el pallet más cercano
                success, msg = node.grasp_nearest_pallet()
                print(f"\n{'✅' if success else '❌'} {msg}")
                
            elif key.lower() == 'h':
                # Soltar pallet
                success, msg = node.release_pallet()
                print(f"\n{'✅' if success else '❌'} {msg}")
                
            elif key.isdigit() and key != '0':
                # Enganchar pallet específico (1-9)
                pallet_name = f'pallet_{key}'
                success, msg = node.grasp_pallet(pallet_name)
                print(f"\n{'✅' if success else '❌'} {msg}")
            
            print_status(node)
            
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("\n✅ Teleop cerrado.\n")


if __name__ == '__main__':
    main()
