#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  🚜 FORKLIFT SIMPLE TELEOP - Control Preciso con Agarre
# ═══════════════════════════════════════════════════════════════

echo "╔════════════════════════════════════════════════════════╗"
echo "║       🚜 FORKLIFT SIMPLE TELEOP 🚜                     ║"
echo "║       Control preciso + Enganche de pallets            ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

# Verificar que existe el mundo
WORLD_FILE="/root/ros2_ws/src/mvsim_warehouse.xml"
if [ ! -f "$WORLD_FILE" ]; then
    echo "❌ Error: No se encuentra $WORLD_FILE"
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash 2>/dev/null

echo "🌍 Lanzando simulador mvsim..."
echo ""

# Lanzar mvsim en background
ros2 launch mvsim launch_world.launch.py \
    world_file:=$WORLD_FILE \
    headless:=False \
    do_fake_localization:=True &

MVSIM_PID=$!

# Esperar a que mvsim esté listo
echo "⏳ Esperando a que mvsim inicie (5 segundos)..."
sleep 5

# Verificar que mvsim está corriendo
if ! ps -p $MVSIM_PID > /dev/null 2>&1; then
    echo "❌ Error: mvsim no se inició correctamente"
    exit 1
fi

echo "✅ mvsim iniciado correctamente"
echo ""

# Lanzar teleop
python3 /root/ros2_ws/src/forklift_simple_teleop.py

# Cuando el usuario salga, cerrar mvsim
echo "🛑 Cerrando simulación..."
kill $MVSIM_PID 2>/dev/null
wait $MVSIM_PID 2>/dev/null

echo "✅ Todo cerrado correctamente."
