#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  🗺️  MAPEADO CON SLAM_TOOLBOX
# ═══════════════════════════════════════════════════════════════
#
# Este script lanza:
# 1. Simulador mvsim
# 2. slam_toolbox para crear el mapa
# 3. RViz para visualización
# 4. Teleop para controlar el forklift
#
# Para guardar el mapa (en otra terminal):
#   ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/mi_mapa
#

echo "╔════════════════════════════════════════════════════════╗"
echo "║      🗺️  MAPEADO CON SLAM_TOOLBOX 🗺️                    ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

WORLD_FILE="/root/ros2_ws/src/mvsim_warehouse.xml"
SLAM_CONFIG="/root/ros2_ws/src/config/slam.yaml"

# Source ROS2
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash 2>/dev/null

# Verificar archivos
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ Error: No se encuentra $WORLD_FILE"
    exit 1
fi

if [ ! -f "$SLAM_CONFIG" ]; then
    echo "❌ Error: No se encuentra $SLAM_CONFIG"
    exit 1
fi

# Lanzar mvsim SIN fake_localization (importante!)
echo "🌍 [1/4] Lanzando simulador mvsim..."
ros2 launch mvsim launch_world.launch.py \
    world_file:=$WORLD_FILE \
    headless:=False \
    do_fake_localization:=False &
MVSIM_PID=$!
sleep 6

if ! ps -p $MVSIM_PID > /dev/null 2>&1; then
    echo "❌ Error: mvsim no se inició"
    exit 1
fi
echo "✅ mvsim iniciado"

# Lanzar slam_toolbox
echo ""
echo "🗺️  [2/4] Lanzando slam_toolbox..."
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=$SLAM_CONFIG &
SLAM_PID=$!
sleep 3
echo "✅ slam_toolbox iniciado"

# Lanzar RViz
echo ""
echo "📊 [3/4] Lanzando RViz..."
rviz2 &
RVIZ_PID=$!
sleep 2

echo ""
echo "🎮 [4/4] Lanzando Teleop..."
echo ""
echo "════════════════════════════════════════════════════════"
echo "  ✅ Todo listo!"
echo ""
echo "  📋 En RViz:"
echo "     1. Add -> By Topic -> /map -> Map"
echo "     2. Add -> By Topic -> /scan -> LaserScan"
echo ""
echo "  🎮 Controla con W/A/S/D, recorre todo el almacén"
echo ""
echo "  💾 Para GUARDAR el mapa (otra terminal):"
echo "     source /opt/ros/humble/setup.bash"
echo "     ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/warehouse"
echo ""
echo "  Pulsa ESC para cerrar"
echo "════════════════════════════════════════════════════════"
echo ""

python3 /root/ros2_ws/src/forklift_simple_teleop.py

# Limpiar
echo ""
echo "🛑 Cerrando..."
kill $MVSIM_PID $SLAM_PID $RVIZ_PID 2>/dev/null
wait 2>/dev/null
echo "✅ Todo cerrado"
