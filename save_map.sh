#!/bin/bash
# Guarda el mapa del simulador (sin necesidad de SLAM)
#
# USO: ./save_map.sh [nombre_mapa]
# Por defecto guarda como "warehouse"
# IMPORTANTE: Solo necesita que mvsim esté corriendo (run_simple_teleop.sh)

source /opt/ros/humble/setup.bash

MAP_NAME=${1:-warehouse}

echo "🗺️  Guardando mapa del simulador como '$MAP_NAME'..."
python3 /root/ros2_ws/src/save_simul_map.py $MAP_NAME
