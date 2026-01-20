#!/bin/bash
#
# ╔═══════════════════════════════════════════════════════════════╗
# ║     🗺️  LANZADOR DEL PLANIFICADOR DE RUTAS 🗺️                 ║
# ╚═══════════════════════════════════════════════════════════════╝
#
# Este script lanza la interfaz gráfica del planificador de rutas
# para el forklift.
#

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║     🗺️  FORKLIFT ROUTE PLANNER - PLANIFICADOR DE RUTAS 🗺️     ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# Directorio del script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Verificar que existe Pillow (necesario para cargar imágenes PGM)
python3 -c "from PIL import Image" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️  Instalando Pillow (necesario para cargar imágenes)..."
    pip3 install Pillow --quiet
fi

# Verificar que existe tkinter
python3 -c "import tkinter" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️  Instalando tkinter..."
    apt-get update && apt-get install -y python3-tk
fi

echo "🚀 Iniciando planificador de rutas..."
echo ""
echo "Instrucciones:"
echo "  1. El mapa muestra el almacén con los 13 pallets y HOME"
echo "  2. Usa 'Añadir Nodo' para crear waypoints intermedios"
echo "  3. Usa 'Añadir a Ruta' para añadir nodos a la ruta"
echo "  4. Selecciona cada paso y elige la acción (recoger/soltar)"
echo "  5. Guarda la ruta y ejecútala con el ejecutor"
echo ""

cd "$SCRIPT_DIR"
python3 forklift_route_planner.py
