#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════════╗
║           🚜 FORKLIFT TELEOP - Control de Teclado 🚜           ║
╠═══════════════════════════════════════════════════════════════╣
║                                                               ║
║   Controles de Movimiento:                                    ║
║   ┌───┬───┬───┐                                               ║
║   │ Q │ W │ E │   Q/E = Girar izq/der mientras avanza         ║
║   ├───┼───┼───┤   W = Adelante                                ║
║   │ A │ S │ D │   S = Atrás                                   ║
║   ├───┼───┼───┤   A/D = Girar izq/der en sitio                ║
║   │ Z │ X │ C │   Z/C = Girar izq/der mientras retrocede      ║
║   └───┴───┴───┘   X = PARAR                                   ║
║                                                               ║
║   Velocidad:                                                  ║
║   ┌───┬───┐                                                   ║
║   │ + │ - │  +/- = Aumentar/Reducir velocidad lineal          ║
║   └───┴───┘                                                   ║
║   ┌───┬───┐                                                   ║
║   │ * │ / │  *// = Aumentar/Reducir velocidad angular         ║
║   └───┴───┘                                                   ║
║                                                               ║
║   ESPACIO = Parada de emergencia                              ║
║   ESC = Salir                                                 ║
║                                                               ║
╚═══════════════════════════════════════════════════════════════╝
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import threading

# Mapeo de teclas a movimientos (linear_x, angular_z)
MOVE_BINDINGS = {
    'w': (1.0, 0.0),    # Adelante
    's': (-1.0, 0.0),   # Atrás
    'a': (0.0, 1.0),    # Girar izquierda
    'd': (0.0, -1.0),   # Girar derecha
    'q': (1.0, 1.0),    # Adelante + izquierda
    'e': (1.0, -1.0),   # Adelante + derecha
    'z': (-1.0, 1.0),   # Atrás + izquierda
    'c': (-1.0, -1.0),  # Atrás + derecha
    'x': (0.0, 0.0),    # Parar
    ' ': (0.0, 0.0),    # Parada de emergencia
}

# Teclas para ajustar velocidad
SPEED_BINDINGS = {
    '+': (0.1, 0.0),    # Aumentar velocidad lineal
    '-': (-0.1, 0.0),   # Reducir velocidad lineal
    '*': (0.0, 0.1),    # Aumentar velocidad angular
    '/': (0.0, -0.1),   # Reducir velocidad angular
}


class ForkliftTeleop(Node):
    def __init__(self):
        super().__init__('forklift_teleop')
        
        # Publisher para cmd_vel (QoS compatible con mvsim)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Velocidades actuales
        self.linear_speed = 1.0   # m/s
        self.angular_speed = 0.8  # rad/s
        
        # Velocidad actual del robot
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Timer para publicar continuamente a 50 Hz
        self.timer = self.create_timer(0.02, self.publish_twist)
        
        # Estado
        self.running = True
        
        self.get_logger().info('🚜 Forklift Teleop iniciado!')
        
    def publish_twist(self):
        """Publica el mensaje Twist actual"""
        msg = Twist()
        msg.linear.x = self.current_linear * self.linear_speed
        msg.angular.z = self.current_angular * self.angular_speed
        self.publisher.publish(msg)
    
    def publish_now(self):
        """Publica inmediatamente (para respuesta rápida)"""
        self.publish_twist()
        
    def stop(self):
        """Detiene el robot"""
        self.current_linear = 0.0
        self.current_angular = 0.0
        msg = Twist()
        self.publisher.publish(msg)
        
    def shutdown(self):
        """Apaga el nodo"""
        self.running = False
        self.stop()


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
    """Imprime el estado actual"""
    bar_length = 20
    
    # Barra de velocidad lineal
    linear_percent = abs(node.current_linear)
    linear_bar = '█' * int(linear_percent * bar_length) + '░' * (bar_length - int(linear_percent * bar_length))
    linear_dir = '↑' if node.current_linear > 0 else ('↓' if node.current_linear < 0 else '○')
    
    # Barra de velocidad angular
    angular_percent = abs(node.current_angular)
    angular_bar = '█' * int(angular_percent * bar_length) + '░' * (bar_length - int(angular_percent * bar_length))
    angular_dir = '←' if node.current_angular > 0 else ('→' if node.current_angular < 0 else '○')
    
    # Limpiar línea y mostrar estado
    status = f"\r\033[K  {linear_dir} Lineal: [{linear_bar}] {node.current_linear * node.linear_speed:+.2f} m/s  |  {angular_dir} Angular: [{angular_bar}] {node.current_angular * node.angular_speed:+.2f} rad/s  |  Max: {node.linear_speed:.1f} m/s, {node.angular_speed:.1f} rad/s"
    print(status, end='', flush=True)


def main():
    # Guardar configuración del terminal
    settings = termios.tcgetattr(sys.stdin)
    
    # Inicializar ROS2
    rclpy.init()
    node = ForkliftTeleop()
    
    # Spinner en thread separado
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print(__doc__)
    print("\n" + "="*65)
    print("  🎮 Usa las teclas para controlar el forklift. ESC para salir.")
    print("="*65 + "\n")
    
    try:
        while node.running:
            key = get_key(settings)
            
            if key == '\x1b':  # ESC
                print("\n\n🛑 Saliendo...")
                break
            elif key in MOVE_BINDINGS:
                linear, angular = MOVE_BINDINGS[key]
                node.current_linear = linear
                node.current_angular = angular
                node.publish_now()  # Publicar inmediatamente
                
                if key == ' ':
                    print("\n⚠️  ¡PARADA DE EMERGENCIA!")
                    
            elif key in SPEED_BINDINGS:
                linear_delta, angular_delta = SPEED_BINDINGS[key]
                node.linear_speed = max(0.1, min(3.0, node.linear_speed + linear_delta))
                node.angular_speed = max(0.1, min(2.0, node.angular_speed + angular_delta))
                
            print_status(node)
            
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        # Restaurar terminal y limpiar
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        print("\n✅ Forklift Teleop cerrado correctamente.\n")


if __name__ == '__main__':
    main()
