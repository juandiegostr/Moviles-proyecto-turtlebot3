#!/bin/bash
# ═══════════════════════════════════════════════════════════════
#  🚜 LANZADOR DEL FORKLIFT CON AGARRE REALISTA
# ═══════════════════════════════════════════════════════════════
#
# Este script lanza mvsim y luego el programa de pick & place
# con agarre realista usando la API de mvsim.
#

echo "╔════════════════════════════════════════════════════════╗"
echo "║    🚜 FORKLIFT SIMULATOR CON AGARRE REALISTA 🚜        ║"
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
echo "   (Espera a que aparezca la ventana del simulador)"
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
echo "════════════════════════════════════════════════════════"
echo ""

# Lanzar el programa de pick & place
echo "🚜 Lanzando programa de Pick & Place con agarre..."
echo ""
python3 /root/ros2_ws/src/forklift_with_grasp.py

# Cuando termine, matar mvsim
echo ""
echo "🛑 Deteniendo simulador..."
kill $MVSIM_PID 2>/dev/null
wait $MVSIM_PID 2>/dev/null

echo "👋 ¡Hasta pronto!"
