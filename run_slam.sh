#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  🗺️  FORKLIFT SLAM - Creación de Mapa del Almacén
# ═══════════════════════════════════════════════════════════════
#
# Este script lanza:
# 1. Simulador mvsim con sensores
# 2. Cartographer para SLAM
# 3. RViz para visualización
#
# USO: ./run_slam.sh
# Luego conduce el forklift por todo el almacén para crear el mapa.
#

echo "╔════════════════════════════════════════════════════════╗"
echo "║        🗺️  FORKLIFT SLAM - Mapeo del Almacén 🗺️         ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

WORLD_FILE="/root/ros2_ws/src/mvsim_warehouse.xml"
CONFIG_DIR="/root/ros2_ws/src/config"
LAUNCH_DIR="/root/ros2_ws/src/launch"

# Source ROS2
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash 2>/dev/null

# Verificar archivos
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ Error: No se encuentra $WORLD_FILE"
    exit 1
fi

# Lanzar mvsim
echo "🌍 [1/4] Lanzando simulador mvsim..."
ros2 launch mvsim launch_world.launch.py \
    world_file:=$WORLD_FILE \
    headless:=False \
    do_fake_localization:=True &
MVSIM_PID=$!
sleep 6

# Verificar mvsim
if ! ps -p $MVSIM_PID > /dev/null 2>&1; then
    echo "❌ Error: mvsim no se inició correctamente"
    exit 1
fi
echo "✅ mvsim iniciado"

# Lanzar Cartographer
echo ""
echo "🗺️  [2/4] Lanzando Cartographer SLAM..."
ros2 launch $LAUNCH_DIR/forklift_slam.launch.py use_sim_time:=true &
CARTO_PID=$!
sleep 3
echo "✅ Cartographer iniciado"

# Lanzar RViz
echo ""
echo "📊 [3/4] Lanzando RViz..."
ros2 run rviz2 rviz2 -d $CONFIG_DIR/slam_rviz.rviz &
RVIZ_PID=$!
sleep 2

# Lanzar teleop
echo ""
echo "🎮 [4/4] Lanzando Teleop..."
echo ""
echo "════════════════════════════════════════════════════════"
echo "  ✅ Todo listo!"
echo ""
echo "  🎮 Controla el forklift con W/A/S/D"
echo "  🗺️  El mapa se genera en tiempo real en RViz"
echo ""
echo "  Cuando termines de mapear, en OTRA terminal:"
echo "     cd /root/ros2_ws/src && ./save_map.sh"
echo ""
echo "  Pulsa ESC en el teleop para cerrar todo"
echo "════════════════════════════════════════════════════════"
echo ""

python3 /root/ros2_ws/src/forklift_simple_teleop.py

# Limpiar al salir
echo ""
echo "🛑 Cerrando..."
kill $MVSIM_PID 2>/dev/null
kill $CARTO_PID 2>/dev/null
kill $RVIZ_PID 2>/dev/null
wait 2>/dev/null
echo "✅ Todo cerrado"
