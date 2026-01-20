#!/usr/bin/env python3
"""
╔═══════════════════════════════════════════════════════════════════════════════╗
║           🗺️  FORKLIFT ROUTE PLANNER - PLANIFICADOR DE RUTAS 🗺️               ║
║                                                                               ║
║   Interfaz gráfica para:                                                      ║
║   • Visualizar el mapa del almacén con pallets                                ║
║   • Crear nodos de ruta (waypoints)                                           ║
║   • Definir rutas completas con acciones (pick/drop)                          ║
║   • Exportar rutas para ejecución por el robot                                ║
║                                                                               ║
╚═══════════════════════════════════════════════════════════════════════════════╝
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog, simpledialog
from PIL import Image, ImageTk, ImageDraw
import json
import os
import math

# ============================
# CONFIGURACIÓN DEL ALMACÉN
# ============================

# Dimensiones del almacén (del XML)
WAREHOUSE_X_MIN = -20
WAREHOUSE_X_MAX = 25
WAREHOUSE_Y_MIN = -15
WAREHOUSE_Y_MAX = 15

# Tamaño del mapa en el canvas
CANVAS_WIDTH = 900
CANVAS_HEIGHT = 600

# Calcular escala
WORLD_WIDTH = WAREHOUSE_X_MAX - WAREHOUSE_X_MIN  # 45
WORLD_HEIGHT = WAREHOUSE_Y_MAX - WAREHOUSE_Y_MIN  # 30
SCALE_X = CANVAS_WIDTH / WORLD_WIDTH
SCALE_Y = CANVAS_HEIGHT / WORLD_HEIGHT

# Posiciones de los pallets (del XML mvsim_warehouse.xml)
PALLET_POSITIONS = {
    'pallet_1': (-17.0, 8.0),
    'pallet_2': (-15.0, 8.0),
    'pallet_3': (-13.0, 8.0),
    'pallet_4': (-17.0, 5.0),
    'pallet_5': (-15.0, 5.0),
    'pallet_6': (-17.0, -8.0),
    'pallet_7': (-15.0, -8.0),
    'pallet_8': (-13.0, -8.0),
    'pallet_9': (-17.0, -5.0),
    'pallet_10': (4.0, 6.0),
    'pallet_11': (4.0, 4.8),
    'pallet_12': (12.0, -6.0),
    'pallet_13': (12.0, -4.9),
}

# Posiciones de las estanterías (del XML)
SHELF_POSITIONS = [
    # Zona Norte
    (0, 10), (0, 13), (8, 10), (8, 13), (16, 10), (16, 13),
    # Zona Sur
    (0, -10), (0, -13), (8, -10), (8, -13), (16, -10), (16, -13),
    # Zona Este
    (22, 5), (22, -5),
]

# Cajas y obstáculos
OBSTACLES = [
    {'type': 'box', 'pos': (-5, 3), 'size': 0.4},
    {'type': 'box', 'pos': (-4.5, 3.3), 'size': 0.4},
    {'type': 'box', 'pos': (-5.5, 2.8), 'size': 0.6},
    {'type': 'box', 'pos': (10, 0), 'size': 0.8},
    {'type': 'box', 'pos': (6, -2), 'size': 0.6},
    {'type': 'tote', 'pos': (-3, 7), 'size': 0.5},
    {'type': 'tote', 'pos': (-2, 7.3), 'size': 0.5},
    {'type': 'tote', 'pos': (5, -7), 'size': 0.5},
    {'type': 'pallet_jack', 'pos': (-10, 12), 'size': 1.2},
    {'type': 'pallet_jack', 'pos': (-8, -12), 'size': 1.2},
]

# Posición HOME
HOME_POSITION = (-15.0, 0.0)


class ForkliftRoutePlanner:
    def __init__(self, root):
        self.root = root
        self.root.title("🚜 Forklift Route Planner - Planificador de Rutas")
        self.root.geometry("1400x900")
        self.root.configure(bg='#2b2b2b')
        
        # Datos de la aplicación
        self.nodes = {}  # {id: {'x': x, 'y': y, 'name': name, 'type': type}}
        self.route = []  # Lista de {'node_id': id, 'action': 'none'|'pick'|'drop'}
        self.node_counter = 0
        self.selected_node = None
        self.dragging_node = None
        
        # Modos
        self.mode = tk.StringVar(value="view")  # view, add_node, add_route, delete
        
        # Cargar imagen del mapa si existe
        self.map_image = None
        self.map_photo = None
        
        self.setup_ui()
        self.load_default_nodes()
        self.draw_map()
        
    def setup_ui(self):
        """Configura la interfaz de usuario"""
        # Frame principal
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Estilo
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#2b2b2b')
        style.configure('TLabel', background='#2b2b2b', foreground='white')
        style.configure('TButton', padding=6)
        style.configure('Header.TLabel', font=('Helvetica', 14, 'bold'))
        style.configure('Toolbutton', padding=8)
        
        # ===== Panel izquierdo: Mapa =====
        left_frame = ttk.Frame(main_frame)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Título del mapa
        map_title = ttk.Label(left_frame, text="🗺️ MAPA DEL ALMACÉN", style='Header.TLabel')
        map_title.pack(pady=(0, 10))
        
        # Canvas del mapa
        canvas_frame = ttk.Frame(left_frame)
        canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        self.canvas = tk.Canvas(
            canvas_frame, 
            width=CANVAS_WIDTH, 
            height=CANVAS_HEIGHT,
            bg='#404040',
            highlightthickness=2,
            highlightbackground='#666666'
        )
        self.canvas.pack(pady=5)
        
        # Eventos del canvas
        self.canvas.bind('<Button-1>', self.on_canvas_click)
        self.canvas.bind('<B1-Motion>', self.on_canvas_drag)
        self.canvas.bind('<ButtonRelease-1>', self.on_canvas_release)
        self.canvas.bind('<Motion>', self.on_canvas_motion)
        
        # Info de coordenadas
        self.coord_label = ttk.Label(left_frame, text="Coordenadas: (0.0, 0.0)")
        self.coord_label.pack()
        
        # Barra de herramientas
        toolbar = ttk.Frame(left_frame)
        toolbar.pack(fill=tk.X, pady=10)
        
        ttk.Label(toolbar, text="Modo:").pack(side=tk.LEFT, padx=5)
        
        modes = [
            ("👁️ Ver", "view"),
            ("➕ Añadir Nodo", "add_node"),
            ("🔗 Añadir a Ruta", "add_route"),
            ("🗑️ Eliminar", "delete"),
        ]
        
        for text, mode in modes:
            rb = ttk.Radiobutton(
                toolbar, 
                text=text, 
                value=mode, 
                variable=self.mode,
                style='Toolbutton'
            )
            rb.pack(side=tk.LEFT, padx=3)
        
        # Botones de acción
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(btn_frame, text="🔄 Refrescar Mapa", command=self.draw_map).pack(side=tk.LEFT, padx=3)
        ttk.Button(btn_frame, text="📥 Cargar Mapa PGM", command=self.load_map_image).pack(side=tk.LEFT, padx=3)
        ttk.Button(btn_frame, text="🧹 Limpiar Todo", command=self.clear_all).pack(side=tk.LEFT, padx=3)
        
        # ===== Panel derecho: Controles =====
        right_frame = ttk.Frame(main_frame, width=450)
        right_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=(20, 0))
        right_frame.pack_propagate(False)
        
        # ----- Sección de Nodos -----
        nodes_title = ttk.Label(right_frame, text="📍 NODOS CREADOS", style='Header.TLabel')
        nodes_title.pack(pady=(0, 10))
        
        # Lista de nodos
        nodes_list_frame = ttk.Frame(right_frame)
        nodes_list_frame.pack(fill=tk.X, pady=5)
        
        self.nodes_listbox = tk.Listbox(
            nodes_list_frame, 
            height=8, 
            bg='#3c3c3c', 
            fg='white',
            selectbackground='#0078d4',
            font=('Consolas', 10)
        )
        self.nodes_listbox.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        nodes_scroll = ttk.Scrollbar(nodes_list_frame, orient=tk.VERTICAL, command=self.nodes_listbox.yview)
        nodes_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.nodes_listbox.config(yscrollcommand=nodes_scroll.set)
        
        self.nodes_listbox.bind('<<ListboxSelect>>', self.on_node_select)
        
        # Botones de nodos
        node_btns = ttk.Frame(right_frame)
        node_btns.pack(fill=tk.X, pady=5)
        
        ttk.Button(node_btns, text="✏️ Renombrar", command=self.rename_node).pack(side=tk.LEFT, padx=2)
        ttk.Button(node_btns, text="🗑️ Eliminar", command=self.delete_selected_node).pack(side=tk.LEFT, padx=2)
        
        # ----- Sección de Ruta -----
        ttk.Separator(right_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15)
        
        route_title = ttk.Label(right_frame, text="🛤️ RUTA PLANIFICADA", style='Header.TLabel')
        route_title.pack(pady=(0, 10))
        
        # Lista de ruta
        route_list_frame = ttk.Frame(right_frame)
        route_list_frame.pack(fill=tk.X, pady=5)
        
        self.route_listbox = tk.Listbox(
            route_list_frame, 
            height=10, 
            bg='#3c3c3c', 
            fg='white',
            selectbackground='#28a745',
            font=('Consolas', 10)
        )
        self.route_listbox.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        route_scroll = ttk.Scrollbar(route_list_frame, orient=tk.VERTICAL, command=self.route_listbox.yview)
        route_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.route_listbox.config(yscrollcommand=route_scroll.set)
        
        self.route_listbox.bind('<<ListboxSelect>>', self.on_route_select)
        
        # Acciones en punto de ruta
        action_frame = ttk.LabelFrame(right_frame, text="Acción en punto seleccionado")
        action_frame.pack(fill=tk.X, pady=10)
        
        self.action_var = tk.StringVar(value="none")
        actions = [
            ("⏺️ Solo pasar", "none"),
            ("📦 RECOGER Pallet", "pick"),
            ("📤 SOLTAR Pallet", "drop"),
        ]
        
        for text, action in actions:
            rb = ttk.Radiobutton(
                action_frame, 
                text=text, 
                value=action, 
                variable=self.action_var,
                command=self.update_route_action
            )
            rb.pack(anchor=tk.W, padx=10, pady=2)
        
        # Botones de ruta
        route_btns = ttk.Frame(right_frame)
        route_btns.pack(fill=tk.X, pady=5)
        
        ttk.Button(route_btns, text="⬆️", command=self.move_route_up, width=3).pack(side=tk.LEFT, padx=2)
        ttk.Button(route_btns, text="⬇️", command=self.move_route_down, width=3).pack(side=tk.LEFT, padx=2)
        ttk.Button(route_btns, text="❌ Quitar", command=self.remove_from_route).pack(side=tk.LEFT, padx=2)
        ttk.Button(route_btns, text="🧹 Limpiar Ruta", command=self.clear_route).pack(side=tk.LEFT, padx=2)
        
        # ----- Sección de Guardar/Cargar -----
        ttk.Separator(right_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15)
        
        io_title = ttk.Label(right_frame, text="💾 GUARDAR / CARGAR", style='Header.TLabel')
        io_title.pack(pady=(0, 10))
        
        io_btns = ttk.Frame(right_frame)
        io_btns.pack(fill=tk.X, pady=5)
        
        ttk.Button(io_btns, text="💾 Guardar Ruta", command=self.save_route).pack(side=tk.LEFT, padx=3)
        ttk.Button(io_btns, text="📂 Cargar Ruta", command=self.load_route).pack(side=tk.LEFT, padx=3)
        
        # ----- Sección de Ejecución -----
        ttk.Separator(right_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15)
        
        exec_title = ttk.Label(right_frame, text="🚜 EJECUTAR RUTA", style='Header.TLabel')
        exec_title.pack(pady=(0, 10))
        
        exec_frame = ttk.Frame(right_frame)
        exec_frame.pack(fill=tk.X, pady=5)
        
        self.exec_btn = ttk.Button(
            exec_frame, 
            text="▶️ INICIAR NAVEGACIÓN", 
            command=self.execute_route,
            style='Accent.TButton'
        )
        self.exec_btn.pack(fill=tk.X, pady=5)
        
        # Info de la ruta
        self.route_info_label = ttk.Label(right_frame, text="Ruta: 0 pasos | Distancia: 0.0m")
        self.route_info_label.pack(pady=5)
        
        # ----- Leyenda -----
        ttk.Separator(right_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15)
        
        legend_title = ttk.Label(right_frame, text="📋 LEYENDA", style='Header.TLabel')
        legend_title.pack(pady=(0, 5))
        
        legend_items = [
            ("🟠 Pallet", "#FF8C00"),
            ("🔵 Nodo intermedio", "#00BFFF"),
            ("🟢 Home", "#32CD32"),
            ("⬛ Estantería", "#666666"),
            ("🟫 Caja/Obstáculo", "#8B4513"),
        ]
        
        legend_frame = ttk.Frame(right_frame)
        legend_frame.pack(fill=tk.X)
        
        for text, color in legend_items:
            item = ttk.Label(legend_frame, text=text, foreground=color)
            item.pack(anchor=tk.W, padx=10)
    
    def world_to_canvas(self, x, y):
        """Convierte coordenadas del mundo a coordenadas del canvas"""
        cx = (x - WAREHOUSE_X_MIN) * SCALE_X
        cy = CANVAS_HEIGHT - (y - WAREHOUSE_Y_MIN) * SCALE_Y  # Invertir Y
        return cx, cy
    
    def canvas_to_world(self, cx, cy):
        """Convierte coordenadas del canvas a coordenadas del mundo"""
        x = cx / SCALE_X + WAREHOUSE_X_MIN
        y = (CANVAS_HEIGHT - cy) / SCALE_Y + WAREHOUSE_Y_MIN
        return x, y
    
    def load_default_nodes(self):
        """Carga los nodos por defecto (pallets y HOME)"""
        # Añadir HOME
        self.add_node(HOME_POSITION[0], HOME_POSITION[1], "HOME", "home")
        
        # Añadir pallets
        for name, (x, y) in PALLET_POSITIONS.items():
            self.add_node(x, y, name.upper(), "pallet")
    
    def add_node(self, x, y, name=None, node_type="waypoint"):
        """Añade un nodo en la posición dada"""
        self.node_counter += 1
        node_id = self.node_counter
        
        if name is None:
            name = f"WP_{node_id}"
        
        self.nodes[node_id] = {
            'x': x,
            'y': y,
            'name': name,
            'type': node_type
        }
        
        self.update_nodes_list()
        self.draw_map()
        return node_id
    
    def update_nodes_list(self):
        """Actualiza la lista de nodos en la UI"""
        self.nodes_listbox.delete(0, tk.END)
        
        # Ordenar por tipo
        sorted_nodes = sorted(
            self.nodes.items(), 
            key=lambda x: (x[1]['type'] != 'home', x[1]['type'] != 'pallet', x[1]['name'])
        )
        
        for node_id, node in sorted_nodes:
            type_icon = "🏠" if node['type'] == 'home' else "📦" if node['type'] == 'pallet' else "📍"
            self.nodes_listbox.insert(
                tk.END, 
                f"{type_icon} [{node_id}] {node['name']} ({node['x']:.1f}, {node['y']:.1f})"
            )
    
    def update_route_list(self):
        """Actualiza la lista de ruta en la UI"""
        self.route_listbox.delete(0, tk.END)
        
        for i, step in enumerate(self.route):
            node_id = step['node_id']
            action = step['action']
            node = self.nodes.get(node_id, {})
            name = node.get('name', f'Node {node_id}')
            
            action_icon = "📦" if action == 'pick' else "📤" if action == 'drop' else "➡️"
            action_text = " [RECOGER]" if action == 'pick' else " [SOLTAR]" if action == 'drop' else ""
            
            self.route_listbox.insert(tk.END, f"{i+1}. {action_icon} {name}{action_text}")
        
        # Actualizar info de ruta
        self.update_route_info()
    
    def update_route_info(self):
        """Actualiza la información de la ruta"""
        total_distance = 0
        prev_node = None
        
        for step in self.route:
            node_id = step['node_id']
            node = self.nodes.get(node_id)
            if node and prev_node:
                dist = math.sqrt((node['x'] - prev_node['x'])**2 + (node['y'] - prev_node['y'])**2)
                total_distance += dist
            prev_node = node
        
        picks = sum(1 for s in self.route if s['action'] == 'pick')
        drops = sum(1 for s in self.route if s['action'] == 'drop')
        
        self.route_info_label.config(
            text=f"Ruta: {len(self.route)} pasos | Distancia: {total_distance:.1f}m | 📦{picks} 📤{drops}"
        )
    
    def draw_map(self):
        """Dibuja el mapa completo"""
        self.canvas.delete("all")
        
        # Fondo (imagen PGM si está cargada)
        if self.map_photo:
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.map_photo)
        else:
            # Dibujar suelo
            self.canvas.create_rectangle(0, 0, CANVAS_WIDTH, CANVAS_HEIGHT, fill='#505050', outline='')
        
        # Dibujar grid
        self.draw_grid()
        
        # Dibujar paredes
        self.draw_walls()
        
        # Dibujar estanterías
        for sx, sy in SHELF_POSITIONS:
            cx, cy = self.world_to_canvas(sx, sy)
            # Estantería como rectángulo alargado
            self.canvas.create_rectangle(
                cx - 30, cy - 15, cx + 30, cy + 15,
                fill='#555555', outline='#777777', width=2
            )
            self.canvas.create_text(cx, cy, text="📦📦📦", font=('Helvetica', 8))
        
        # Dibujar obstáculos
        for obs in OBSTACLES:
            cx, cy = self.world_to_canvas(obs['pos'][0], obs['pos'][1])
            size = obs['size'] * SCALE_X / 2
            color = '#8B4513' if obs['type'] == 'box' else '#4169E1' if obs['type'] == 'tote' else '#FFD700'
            self.canvas.create_rectangle(
                cx - size, cy - size, cx + size, cy + size,
                fill=color, outline='#333333'
            )
        
        # Dibujar ruta (líneas entre nodos)
        if len(self.route) > 1:
            points = []
            for step in self.route:
                node = self.nodes.get(step['node_id'])
                if node:
                    cx, cy = self.world_to_canvas(node['x'], node['y'])
                    points.extend([cx, cy])
            
            if len(points) >= 4:
                self.canvas.create_line(points, fill='#00FF00', width=3, arrow=tk.LAST, smooth=True)
        
        # Dibujar nodos
        for node_id, node in self.nodes.items():
            self.draw_node(node_id, node)
        
        # Numerar pasos de la ruta
        for i, step in enumerate(self.route):
            node = self.nodes.get(step['node_id'])
            if node:
                cx, cy = self.world_to_canvas(node['x'], node['y'])
                # Número del paso
                self.canvas.create_oval(cx + 10, cy - 25, cx + 30, cy - 5, fill='#32CD32', outline='white')
                self.canvas.create_text(cx + 20, cy - 15, text=str(i + 1), fill='white', font=('Helvetica', 9, 'bold'))
    
    def draw_grid(self):
        """Dibuja una cuadrícula de referencia"""
        # Líneas verticales cada 5 metros
        for x in range(WAREHOUSE_X_MIN, WAREHOUSE_X_MAX + 1, 5):
            cx, _ = self.world_to_canvas(x, 0)
            self.canvas.create_line(cx, 0, cx, CANVAS_HEIGHT, fill='#3a3a3a', dash=(2, 4))
            # Etiqueta
            self.canvas.create_text(cx, CANVAS_HEIGHT - 10, text=str(x), fill='#888888', font=('Helvetica', 8))
        
        # Líneas horizontales cada 5 metros
        for y in range(WAREHOUSE_Y_MIN, WAREHOUSE_Y_MAX + 1, 5):
            _, cy = self.world_to_canvas(0, y)
            self.canvas.create_line(0, cy, CANVAS_WIDTH, cy, fill='#3a3a3a', dash=(2, 4))
            # Etiqueta
            self.canvas.create_text(15, cy, text=str(y), fill='#888888', font=('Helvetica', 8))
    
    def draw_walls(self):
        """Dibuja las paredes del almacén"""
        # Borde exterior
        x1, y1 = self.world_to_canvas(WAREHOUSE_X_MIN, WAREHOUSE_Y_MAX)
        x2, y2 = self.world_to_canvas(WAREHOUSE_X_MAX, WAREHOUSE_Y_MIN)
        self.canvas.create_rectangle(x1, y1, x2, y2, outline='#8B0000', width=4)
    
    def draw_node(self, node_id, node):
        """Dibuja un nodo individual"""
        cx, cy = self.world_to_canvas(node['x'], node['y'])
        node_type = node['type']
        
        # Determinar color y tamaño según tipo
        if node_type == 'home':
            color = '#32CD32'  # Verde
            size = 18
            icon = "🏠"
        elif node_type == 'pallet':
            color = '#FF8C00'  # Naranja
            size = 15
            icon = "📦"
        else:
            color = '#00BFFF'  # Azul claro
            size = 12
            icon = "📍"
        
        # Resaltar si está seleccionado
        if self.selected_node == node_id:
            self.canvas.create_oval(
                cx - size - 5, cy - size - 5, cx + size + 5, cy + size + 5,
                outline='#FFFF00', width=3
            )
        
        # Resaltar si está en la ruta
        in_route = any(step['node_id'] == node_id for step in self.route)
        if in_route:
            self.canvas.create_oval(
                cx - size - 3, cy - size - 3, cx + size + 3, cy + size + 3,
                outline='#00FF00', width=2
            )
        
        # Dibujar nodo
        self.canvas.create_oval(
            cx - size, cy - size, cx + size, cy + size,
            fill=color, outline='white', width=2,
            tags=f"node_{node_id}"
        )
        
        # Icono/texto
        self.canvas.create_text(cx, cy, text=icon, font=('Helvetica', 10))
        
        # Nombre del nodo
        self.canvas.create_text(
            cx, cy + size + 12, 
            text=node['name'], 
            fill='white', 
            font=('Helvetica', 8, 'bold'),
            anchor=tk.N
        )
    
    def on_canvas_click(self, event):
        """Maneja clicks en el canvas"""
        wx, wy = self.canvas_to_world(event.x, event.y)
        
        # Buscar si se clickeó en un nodo existente
        clicked_node = self.find_node_at(event.x, event.y)
        
        mode = self.mode.get()
        
        if mode == "add_node":
            # Añadir nuevo nodo waypoint
            name = simpledialog.askstring("Nuevo Nodo", "Nombre del nodo:", initialvalue=f"WP_{self.node_counter + 1}")
            if name:
                self.add_node(wx, wy, name, "waypoint")
        
        elif mode == "add_route":
            if clicked_node:
                # Añadir nodo a la ruta
                self.route.append({'node_id': clicked_node, 'action': 'none'})
                self.update_route_list()
                self.draw_map()
        
        elif mode == "delete":
            if clicked_node:
                node = self.nodes.get(clicked_node)
                if node and node['type'] not in ['home', 'pallet']:
                    del self.nodes[clicked_node]
                    # Quitar de la ruta también
                    self.route = [s for s in self.route if s['node_id'] != clicked_node]
                    self.update_nodes_list()
                    self.update_route_list()
                    self.draw_map()
                else:
                    messagebox.showwarning("Aviso", "No se pueden eliminar nodos de pallet o HOME")
        
        elif mode == "view":
            if clicked_node:
                self.selected_node = clicked_node
                self.dragging_node = clicked_node if self.nodes[clicked_node]['type'] == 'waypoint' else None
                self.draw_map()
                # Seleccionar en la lista
                self.select_node_in_list(clicked_node)
    
    def on_canvas_drag(self, event):
        """Maneja arrastre en el canvas (mover nodos waypoint)"""
        if self.dragging_node:
            wx, wy = self.canvas_to_world(event.x, event.y)
            # Limitar a los bordes del almacén
            wx = max(WAREHOUSE_X_MIN + 1, min(WAREHOUSE_X_MAX - 1, wx))
            wy = max(WAREHOUSE_Y_MIN + 1, min(WAREHOUSE_Y_MAX - 1, wy))
            
            self.nodes[self.dragging_node]['x'] = wx
            self.nodes[self.dragging_node]['y'] = wy
            self.draw_map()
    
    def on_canvas_release(self, event):
        """Maneja soltar el click"""
        if self.dragging_node:
            self.update_nodes_list()
            self.dragging_node = None
    
    def on_canvas_motion(self, event):
        """Muestra las coordenadas del cursor"""
        wx, wy = self.canvas_to_world(event.x, event.y)
        self.coord_label.config(text=f"Coordenadas: ({wx:.1f}, {wy:.1f})")
    
    def find_node_at(self, cx, cy, radius=20):
        """Busca un nodo cerca de las coordenadas del canvas"""
        for node_id, node in self.nodes.items():
            nx, ny = self.world_to_canvas(node['x'], node['y'])
            dist = math.sqrt((cx - nx)**2 + (cy - ny)**2)
            if dist <= radius:
                return node_id
        return None
    
    def select_node_in_list(self, node_id):
        """Selecciona un nodo en la lista"""
        for i in range(self.nodes_listbox.size()):
            item = self.nodes_listbox.get(i)
            if f"[{node_id}]" in item:
                self.nodes_listbox.selection_clear(0, tk.END)
                self.nodes_listbox.selection_set(i)
                self.nodes_listbox.see(i)
                break
    
    def on_node_select(self, event):
        """Maneja selección en la lista de nodos"""
        selection = self.nodes_listbox.curselection()
        if selection:
            item = self.nodes_listbox.get(selection[0])
            # Extraer ID del texto
            import re
            match = re.search(r'\[(\d+)\]', item)
            if match:
                node_id = int(match.group(1))
                self.selected_node = node_id
                self.draw_map()
    
    def on_route_select(self, event):
        """Maneja selección en la lista de ruta"""
        selection = self.route_listbox.curselection()
        if selection:
            idx = selection[0]
            if idx < len(self.route):
                step = self.route[idx]
                self.action_var.set(step['action'])
                self.selected_node = step['node_id']
                self.draw_map()
    
    def update_route_action(self):
        """Actualiza la acción del paso de ruta seleccionado"""
        selection = self.route_listbox.curselection()
        if selection:
            idx = selection[0]
            if idx < len(self.route):
                self.route[idx]['action'] = self.action_var.get()
                self.update_route_list()
                # Mantener selección
                self.route_listbox.selection_set(idx)
    
    def rename_node(self):
        """Renombra el nodo seleccionado"""
        if self.selected_node and self.selected_node in self.nodes:
            node = self.nodes[self.selected_node]
            new_name = simpledialog.askstring("Renombrar Nodo", "Nuevo nombre:", initialvalue=node['name'])
            if new_name:
                node['name'] = new_name
                self.update_nodes_list()
                self.update_route_list()
                self.draw_map()
    
    def delete_selected_node(self):
        """Elimina el nodo seleccionado"""
        if self.selected_node and self.selected_node in self.nodes:
            node = self.nodes[self.selected_node]
            if node['type'] in ['home', 'pallet']:
                messagebox.showwarning("Aviso", "No se pueden eliminar nodos de pallet o HOME")
                return
            
            if messagebox.askyesno("Confirmar", f"¿Eliminar nodo '{node['name']}'?"):
                del self.nodes[self.selected_node]
                self.route = [s for s in self.route if s['node_id'] != self.selected_node]
                self.selected_node = None
                self.update_nodes_list()
                self.update_route_list()
                self.draw_map()
    
    def move_route_up(self):
        """Mueve el elemento seleccionado hacia arriba en la ruta"""
        selection = self.route_listbox.curselection()
        if selection and selection[0] > 0:
            idx = selection[0]
            self.route[idx], self.route[idx-1] = self.route[idx-1], self.route[idx]
            self.update_route_list()
            self.route_listbox.selection_set(idx - 1)
            self.draw_map()
    
    def move_route_down(self):
        """Mueve el elemento seleccionado hacia abajo en la ruta"""
        selection = self.route_listbox.curselection()
        if selection and selection[0] < len(self.route) - 1:
            idx = selection[0]
            self.route[idx], self.route[idx+1] = self.route[idx+1], self.route[idx]
            self.update_route_list()
            self.route_listbox.selection_set(idx + 1)
            self.draw_map()
    
    def remove_from_route(self):
        """Elimina el paso seleccionado de la ruta"""
        selection = self.route_listbox.curselection()
        if selection:
            idx = selection[0]
            del self.route[idx]
            self.update_route_list()
            self.draw_map()
    
    def clear_route(self):
        """Limpia toda la ruta"""
        if messagebox.askyesno("Confirmar", "¿Limpiar toda la ruta?"):
            self.route = []
            self.update_route_list()
            self.draw_map()
    
    def clear_all(self):
        """Limpia todos los nodos waypoint y la ruta"""
        if messagebox.askyesno("Confirmar", "¿Limpiar todos los waypoints y la ruta?"):
            # Mantener HOME y pallets
            self.nodes = {k: v for k, v in self.nodes.items() if v['type'] in ['home', 'pallet']}
            self.route = []
            self.update_nodes_list()
            self.update_route_list()
            self.draw_map()
    
    def load_map_image(self):
        """Carga una imagen PGM del mapa"""
        filepath = filedialog.askopenfilename(
            title="Cargar mapa",
            initialdir="/root/ros2_ws/maps",
            filetypes=[("PGM files", "*.pgm"), ("PNG files", "*.png"), ("All files", "*.*")]
        )
        
        if filepath:
            try:
                img = Image.open(filepath)
                # Redimensionar al tamaño del canvas
                img = img.resize((CANVAS_WIDTH, CANVAS_HEIGHT), Image.Resampling.LANCZOS)
                # Invertir verticalmente para coincidir con coordenadas ROS
                img = img.transpose(Image.FLIP_TOP_BOTTOM)
                # Convertir a formato Tkinter
                self.map_image = img
                self.map_photo = ImageTk.PhotoImage(img)
                self.draw_map()
                messagebox.showinfo("Éxito", "Mapa cargado correctamente")
            except Exception as e:
                messagebox.showerror("Error", f"No se pudo cargar el mapa: {e}")
    
    def save_route(self):
        """Guarda la ruta en un archivo JSON"""
        if not self.route:
            messagebox.showwarning("Aviso", "La ruta está vacía")
            return
        
        filepath = filedialog.asksaveasfilename(
            title="Guardar ruta",
            initialdir="/root/ros2_ws/src",
            defaultextension=".json",
            filetypes=[("JSON files", "*.json")]
        )
        
        if filepath:
            data = {
                'nodes': self.nodes,
                'route': self.route
            }
            
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=2)
            
            messagebox.showinfo("Éxito", f"Ruta guardada en:\n{filepath}")
    
    def load_route(self):
        """Carga una ruta desde un archivo JSON"""
        filepath = filedialog.askopenfilename(
            title="Cargar ruta",
            initialdir="/root/ros2_ws/src",
            filetypes=[("JSON files", "*.json")]
        )
        
        if filepath:
            try:
                with open(filepath, 'r') as f:
                    data = json.load(f)
                
                # Convertir keys de nodos a int
                self.nodes = {int(k): v for k, v in data['nodes'].items()}
                self.route = data['route']
                self.node_counter = max(self.nodes.keys()) if self.nodes else 0
                
                self.update_nodes_list()
                self.update_route_list()
                self.draw_map()
                
                messagebox.showinfo("Éxito", "Ruta cargada correctamente")
            except Exception as e:
                messagebox.showerror("Error", f"No se pudo cargar la ruta: {e}")
    
    def execute_route(self):
        """Exporta y ejecuta la ruta"""
        if not self.route:
            messagebox.showwarning("Aviso", "La ruta está vacía")
            return
        
        # Preparar datos para el ejecutor
        route_data = []
        for step in self.route:
            node = self.nodes.get(step['node_id'])
            if node:
                route_data.append({
                    'x': node['x'],
                    'y': node['y'],
                    'name': node['name'],
                    'action': step['action']
                })
        
        # Guardar en archivo temporal
        route_file = "/root/ros2_ws/src/current_route.json"
        with open(route_file, 'w') as f:
            json.dump(route_data, f, indent=2)
        
        messagebox.showinfo(
            "Ruta Exportada", 
            f"Ruta guardada en:\n{route_file}\n\n"
            f"Para ejecutar, abre un terminal y ejecuta:\n"
            f"cd /root/ros2_ws/src && python3 forklift_route_executor.py"
        )


def main():
    root = tk.Tk()
    app = ForkliftRoutePlanner(root)
    root.mainloop()


if __name__ == '__main__':
    main()
