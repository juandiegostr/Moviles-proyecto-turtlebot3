#!/bin/bash
# Guarda el mapa actual generado por SLAM
#
# USO: ./save_map.sh [nombre_mapa]
# Por defecto guarda como "warehouse"
# IMPORTANTE: Ejecutar mientras SLAM está corriendo!

source /opt/ros/humble/setup.bash

MAP_NAME=${1:-warehouse}

echo "💾 Guardando mapa como '$MAP_NAME'..."
python3 /root/ros2_ws/src/save_map_custom.py $MAP_NAME
