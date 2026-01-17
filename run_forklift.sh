#!/bin/bash
#
# 🚜 Script de lanzamiento del Forklift Warehouse
# Inicia la simulación y el control de teclado
#

# Colores
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║           🚜 FORKLIFT WAREHOUSE SIMULATOR 🚜                  ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Source ROS2
echo -e "${YELLOW}[1/3] Configurando entorno ROS2...${NC}"
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash 2>/dev/null

# Lanzar simulación en background
echo -e "${YELLOW}[2/3] Iniciando simulación mvsim...${NC}"
ros2 launch mvsim launch_world.launch.py world_file:=/root/ros2_ws/src/mvsim_warehouse.xml &
MVSIM_PID=$!

# Esperar a que mvsim arranque
echo -e "${YELLOW}[3/3] Esperando a que la simulación cargue...${NC}"
sleep 8

# Verificar que mvsim está corriendo
if ps -p $MVSIM_PID > /dev/null 2>&1; then
    echo -e "${GREEN}✅ Simulación iniciada correctamente!${NC}"
    echo ""
    echo -e "${BLUE}Iniciando control de teclado...${NC}"
    echo ""
    sleep 1
    
    # Lanzar teleop
    python3 /root/ros2_ws/src/forklift_teleop.py
    
    # Cuando el usuario salga del teleop, cerrar mvsim
    echo -e "${YELLOW}Cerrando simulación...${NC}"
    kill $MVSIM_PID 2>/dev/null
    wait $MVSIM_PID 2>/dev/null
else
    echo -e "${YELLOW}⚠️  La simulación tardó en cargar. Intentando continuar...${NC}"
    python3 /root/ros2_ws/src/forklift_teleop.py
fi

echo -e "${GREEN}✅ Todo cerrado correctamente.${NC}"
