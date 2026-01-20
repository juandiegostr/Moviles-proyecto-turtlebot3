#!/bin/bash
#
# ╔═══════════════════════════════════════════════════════════════╗
# ║     🚜 EJECUTOR DE RUTAS DEL FORKLIFT 🚜                      ║
# ╚═══════════════════════════════════════════════════════════════╝
#
# Este script ejecuta la ruta planificada con el Route Planner.
# Requiere que la simulación esté corriendo (MVSim o Gazebo).
#

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║     🚜 FORKLIFT ROUTE EXECUTOR - EJECUTOR DE RUTAS 🚜        ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# Directorio del script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROUTE_FILE="${1:-$SCRIPT_DIR/current_route.json}"

# Source ROS2
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash 2>/dev/null || true

# Verificar que existe el archivo de ruta
if [ ! -f "$ROUTE_FILE" ]; then
    echo "❌ No se encontró archivo de ruta: $ROUTE_FILE"
    echo ""
    echo "Por favor, primero crea una ruta usando el Route Planner:"
    echo "  ./run_route_planner.sh"
    echo ""
    exit 1
fi

echo "📂 Archivo de ruta: $ROUTE_FILE"
echo ""
echo "Verificando conexión con la simulación..."

# Verificar que hay topics de ROS2 activos
ros2 topic list 2>/dev/null | grep -q "/cmd_vel"
if [ $? -ne 0 ]; then
    echo "⚠️  No se detectaron topics de ROS2 activos."
    echo "   Asegúrate de que la simulación esté corriendo."
    echo ""
    echo "   Para iniciar la simulación:"
    echo "   Terminal 1: mvsim launch mvsim_warehouse.xml"
    echo ""
fi

echo "🚀 Iniciando ejecutor de rutas..."
echo ""

cd "$SCRIPT_DIR"
python3 forklift_route_executor.py "$ROUTE_FILE"
