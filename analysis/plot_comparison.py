import csv
import os
import sys
import glob
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms

_ANALYSIS_DIR  = os.path.dirname(os.path.abspath(__file__))
_CONTROLLER_DIR = os.path.join(_ANALYSIS_DIR, '..', 'controllers', 'controlador')

archivos_csv = glob.glob(os.path.join(_CONTROLLER_DIR, 'trajectory_*.csv'))

if not archivos_csv: # Fallback por si están en la misma carpeta que este script
    archivos_csv = glob.glob(os.path.join(_ANALYSIS_DIR, 'trajectory_*.csv'))

if not archivos_csv:
    print("ERROR: No se encontró ningún archivo CSV de trayectoria.")
    sys.exit(1)

archivo_mas_reciente = max(archivos_csv, key=os.path.getmtime)

# Extraer el nombre del escenario
nombre_base = os.path.basename(archivo_mas_reciente)
ESCENARIO_ACTUAL = nombre_base.replace('trajectory_', '').replace('.csv', '')

print(f">>> Auto-detectado: Graficando el último escenario simulado ({ESCENARIO_ACTUAL}) <<<")

# =============================================================================
# 1. CONFIGURACIÓN MAESTRA DEL ESCENARIO
# =============================================================================
# Cambia esto a "SIMPLE" o "COMPLEJO" según lo que quieras graficar

if ESCENARIO_ACTUAL == "SIMPLE":
    LIMITE_ARENA = 0.55 
    TITULO_GRAFICO = 'Escenario Simple (1x1)'
    OBSTACULOS = [
    {"t": [ 0.23,    0.41],     "s": [0.2, 0.2], "ang": 0.523599},
    {"t": [-0.26,   -0.40],     "s": [0.2, 0.2], "ang": 0.0},
    {"t": [-0.28,    0.27],     "s": [0.3, 0.1], "ang": 0.0},
    {"t": [ 0.33,   -0.12],     "s": [0.1, 0.4], "ang": 0.0},
    {"t": [-0.352219, -0.0529289], "s": [0.3, 0.2], "ang": 0.785398},
    {"t": [ 0.07,   -0.14],     "s": [0.2, 0.2], "ang": 0.0},
    ]
elif ESCENARIO_ACTUAL == "COMPLEJO":
    LIMITE_ARENA = 1.05  # Arena 2x2m (+ margen visual)
    TITULO_GRAFICO = 'Escenario Complejo (2x2)'
    OBSTACULOS = [
        {"t": [ 0.19,       -0.56],     "s": [0.6, 0.3], "ang": 0.0},
        {"t": [ 0.34,       -0.11],     "s": [0.6, 0.3], "ang": -1.570795},
        {"t": [ 0.59,       -0.24],     "s": [0.3, 0.2], "ang": -1.570795},
        # Recordatorio: El bloque invisible en [0.34, -0.86] no se dibuja aquí para
        # evidenciar en el gráfico por qué el A* falló y forzó la evasión reactiva.
        {"t": [ 0.90,        0.34],     "s": [0.3, 0.2], "ang": -1.570795},
        {"t": [ 0.0100693,   0.589526], "s": [0.1, 0.3], "ang": -0.523595},
        {"t": [ 0.19,        0.34],     "s": [0.6, 0.3], "ang": 0.0},
        {"t": [-0.89993,    -0.562913], "s": [0.1, 0.2], "ang": 1.5708},
        {"t": [-0.5, -0.1],             "s": [0.52, 0.52], "ang": 0.0}, # Cilindro 1
        {"t": [-0.8,  0.79],            "s": [0.80, 0.80], "ang": 0.0}, # Cilindro 2
    ]
elif ESCENARIO_ACTUAL == "MUY_COMPLEJO":
    LIMITE_ARENA = 1.55  # Arena 2x2m (+ margen visual)
    TITULO_GRAFICO = 'Escenario Muy Complejo (3x3)'
    OBSTACULOS = [
        # Bloques obs1 (Cuadrados y rectángulos pequeños)
        {"t": [-0.16,     -1.15],     "s": [0.3, 0.3], "ang": 0.0},
        {"t": [-0.33,      0.10],     "s": [0.5, 0.5], "ang": 0.0},
        {"t": [ 0.31,     -1.00],     "s": [0.3, 0.3], "ang": 0.0},
        {"t": [ 0.02,      0.11],     "s": [0.3, 0.3], "ang": 0.0},
        {"t": [ 0.31,     -0.41],     "s": [0.3, 0.3], "ang": 0.0},
        {"t": [ 0.61,      1.04],     "s": [0.3, 0.3], "ang": 0.0},
        {"t": [ 0.40,      0.65],     "s": [0.3, 0.3], "ang": 0.0},
        {"t": [ 1.27,      0.66],     "s": [0.5, 0.3], "ang": 0.0},
        {"t": [ 0.95,     -1.22],     "s": [0.5, 0.3], "ang": 0.0},
        {"t": [ 1.1607,   -0.0587],   "s": [0.5, 0.1], "ang": 2.6179},
        {"t": [-0.9690,    0.2124],   "s": [0.5, 0.1], "ang": 3.1415},
        {"t": [-0.6650,    0.0085],   "s": [0.5, 0.1], "ang": -2.3561},

        # Bloques b (Muros grandes y divisores diagonales)
        {"t": [-1.03,     -1.15],     "s": [1.2, 0.7], "ang": 0.0},
        # Corrección de eje Z invertido (-1) a rotación estándar (+1)
        {"t": [-0.3037,   -0.4490],   "s": [1.1, 0.2], "ang": 1.0472}, 
        {"t": [ 0.4025,   -0.2140],   "s": [1.1, 0.2], "ang": 1.5708},
        {"t": [-0.0374,    0.3159],   "s": [0.7, 0.2], "ang": 1.5708},
        {"t": [ 0.3125,    0.7659],   "s": [0.9, 0.2], "ang": -3.1415},
        {"t": [ 0.5625,    0.0959],   "s": [0.5, 0.2], "ang": -3.1415},
        {"t": [ 0.6343,    1.0037],   "s": [0.7, 0.1], "ang": -1.0471},
        # Corrección de eje Z invertido
        {"t": [ 0.1743,    1.0696],   "s": [0.8, 0.1], "ang": 0.5236}, 
        {"t": [ 1.3114,    1.1989],   "s": [1.3, 0.4], "ang": 2.3562},
        {"t": [-0.6368,    1.1998],   "s": [1.5, 0.5], "ang": 0.7854},
    ]
else:
    raise ValueError("ESCENARIO_ACTUAL no válido. Usa 'SIMPLE' o 'COMPLEJO'.")

# =============================================================================
# 2. RUTAS DE ARCHIVOS CSV
# =============================================================================
_ANALYSIS_DIR  = os.path.dirname(os.path.abspath(__file__))
# Asegúrate de que las carpetas coincidan con tu estructura real
_CONTROLLER_DIR = os.path.join(_ANALYSIS_DIR, '..', 'controllers', 'controlador')

# Nombres vinculados automáticamente al escenario
PLANNED_CSV = os.path.join(os.path.dirname(archivo_mas_reciente), f'planned_path_{ESCENARIO_ACTUAL}.csv')
TRAJECTORY_CSV = archivo_mas_reciente
OUTPUT_PNG   = os.path.join(_ANALYSIS_DIR, f'trajectory_analysis_{ESCENARIO_ACTUAL}.png')

# Fallback por si los archivos se guardan en la misma carpeta que el script
if not os.path.exists(PLANNED_CSV):
    PLANNED_CSV = os.path.join(_ANALYSIS_DIR, f'planned_path_{ESCENARIO_ACTUAL}.csv')
    TRAJECTORY_CSV = os.path.join(_ANALYSIS_DIR, f'trajectory_{ESCENARIO_ACTUAL}.csv')

def load_planned(path):
    points = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            points.append((float(row['x_planned']), float(row['y_planned'])))
    return points

def load_trajectory(path):
    xs, ys, states = [], [], []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            xs.append(float(row['x_actual']))
            ys.append(float(row['y_actual']))
            states.append(int(row['nav_state']))
    return xs, ys, states

if not os.path.exists(PLANNED_CSV) or not os.path.exists(TRAJECTORY_CSV):
    print(f"ERROR: CSVs no encontrados para el escenario {ESCENARIO_ACTUAL}.")
    print(f"Buscando en: {PLANNED_CSV}")
    sys.exit(1)

planned = load_planned(PLANNED_CSV)
traj_x, traj_y, traj_states = load_trajectory(TRAJECTORY_CSV)

# =============================================================================
# 3. GRÁFICO 2D
# =============================================================================
fig, ax = plt.subplots(1, 1, figsize=(8, 8))

# Límites dinámicos según el escenario
ax.set_xlim(-LIMITE_ARENA, LIMITE_ARENA)
ax.set_ylim(-LIMITE_ARENA, LIMITE_ARENA)
ax.set_aspect('equal')
ax.set_xlabel('X [m]', fontsize=11)
ax.set_ylabel('Y [m]', fontsize=11)
ax.set_title(TITULO_GRAFICO, fontsize=14, fontweight='bold')

# Filtrar puntos de evasión (nav_state != 0)
evasion_x = [traj_x[i] for i in range(len(traj_states)) if traj_states[i] != 0]
evasion_y = [traj_y[i] for i in range(len(traj_states)) if traj_states[i] != 0]

# Dibujar obstáculos del escenario actual
for o in OBSTACULOS:
    w, h = o['s'][0], o['s'][1]
    rect = patches.Rectangle((o['t'][0] - w/2, o['t'][1] - h/2), w, h, 
                             linewidth=1, edgecolor='#cc2222', facecolor='#ffcccc', zorder=2)
    # Aplicar rotación alrededor del centro del obstáculo
    t = transforms.Affine2D().rotate_around(o['t'][0], o['t'][1], o['ang']) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)

# Dibujar eventos de evasión
if evasion_x:
    ax.scatter(evasion_x, evasion_y, c='orange', s=10, alpha=0.8, zorder=4, label='Zona de Evasión (Kalman)')

# Dibujar Trayectoria Real (Odometría)
ax.plot(traj_x, traj_y, color='#cc0000', linewidth=1.8, zorder=5, label='Trayectoria ejecutada (Odometría)')

px = [pt[0] for pt in planned]
py = [pt[1] for pt in planned]
ax.plot(px, py, 'b--', linewidth=2.0, zorder=6, label='Ruta A*')

ax.plot(planned[0][0], planned[0][1], 'g*', markersize=18, zorder=7, label='Inicio')
ax.plot(planned[-1][0], planned[-1][1], marker='*', color='gold', markersize=18, zorder=7, markeredgecolor='k', label='Meta')

ax.legend(loc='upper right', framealpha=0.9)
plt.grid(True, linestyle=':', alpha=0.6)
plt.savefig(OUTPUT_PNG, dpi=150, bbox_inches='tight')
print(f"Gráfico guardado en: {OUTPUT_PNG}")
plt.show()