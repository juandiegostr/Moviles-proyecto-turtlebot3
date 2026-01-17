#!/bin/bash
# Script para ejecutar el teleop simple del forklift

cd /root/ros2_ws
source install/setup.bash

echo "🚜 Iniciando Forklift Simple Teleop..."
echo ""
echo "Asegúrate de que mvsim esté corriendo en otra terminal:"
echo "  ros2 launch forklift_robot forklift_mvsim.launch.py"
echo ""

python3 src/forklift_simple_teleop.py
