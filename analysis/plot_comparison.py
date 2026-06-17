import csv
import os
import sys
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.transforms as transforms

# =============================================================================
# CONFIGURACIÓN
# =============================================================================
_ANALYSIS_DIR    = os.path.dirname(os.path.abspath(__file__))
_CONTROLLER_DIR  = os.path.join(_ANALYSIS_DIR, '..', 'controllers', 'robot_controller')
PLANNED_CSV      = os.path.join(_CONTROLLER_DIR, 'planned_path.csv')
TRAJECTORY_CSV   = os.path.join(_CONTROLLER_DIR, 'trajectory.csv')
OUTPUT_PNG       = os.path.join(_ANALYSIS_DIR, 'trajectory_analysis_1x1.png')

if not os.path.exists(PLANNED_CSV):
    PLANNED_CSV = os.path.join(_ANALYSIS_DIR, 'planned_path.csv')
    TRAJECTORY_CSV = os.path.join(_ANALYSIS_DIR, 'trajectory.csv')

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
    print("ERROR: CSVs no encontrados.")
    sys.exit(1)

planned = load_planned(PLANNED_CSV)
traj_x, traj_y, traj_states = load_trajectory(TRAJECTORY_CSV)

# =============================================================================
# GRÁFICO (Ajustado a la arena 1x1)
# =============================================================================
fig, ax = plt.subplots(1, 1, figsize=(8, 8))
# Límites ajustados a la arena pequeña
ax.set_xlim(-0.55, 0.55)
ax.set_ylim(-0.55, 0.55)
ax.set_aspect('equal')
ax.set_xlabel('X [m]', fontsize=11)
ax.set_ylabel('Y [m]', fontsize=11)
ax.set_title('Proyecto Final: Escenario 1x1 - A* y Evasión', fontsize=14, fontweight='bold')

evasion_x = [traj_x[i] for i in range(len(traj_states)) if traj_states[i] != 0]
evasion_y = [traj_y[i] for i in range(len(traj_states)) if traj_states[i] != 0]

# Los nuevos 6 obstáculos
obstaculos = [
    {"t": [0.23, 0.41], "s": [0.2, 0.2], "ang": 0.523599},
    {"t": [-0.26, -0.4], "s": [0.2, 0.2], "ang": 0},
    {"t": [-0.28, 0.27], "s": [0.3, 0.1], "ang": 0},
    {"t": [0.33, -0.12], "s": [0.1, 0.4], "ang": 0},
    {"t": [-0.352219, -0.0529289], "s": [0.3, 0.2], "ang": 0.785398},
    {"t": [0.07, -0.14], "s": [0.2, 0.2], "ang": 0}
]

for o in obstaculos:
    w, h = o['s'][0], o['s'][1]
    rect = patches.Rectangle((o['t'][0] - w/2, o['t'][1] - h/2), w, h, linewidth=1, edgecolor='#cc2222', facecolor='#ffcccc', zorder=2)
    t = transforms.Affine2D().rotate_around(o['t'][0], o['t'][1], o['ang']) + ax.transData
    rect.set_transform(t)
    ax.add_patch(rect)

if evasion_x:
    ax.scatter(evasion_x, evasion_y, c='orange', s=8, alpha=0.6, zorder=4, label='Evasión')

ax.plot(traj_x, traj_y, color='#cc0000', linewidth=1.5, zorder=5, label='Trayectoria estimada (odometría)')

px = [pt[0] for pt in planned]
py = [pt[1] for pt in planned]
ax.plot(px, py, 'b--', linewidth=2.0, zorder=6, label='Ruta A*')

ax.plot(planned[0][0], planned[0][1], 'g*', markersize=18, zorder=7, label='Inicio')
ax.plot(planned[-1][0], planned[-1][1], marker='*', color='gold', markersize=18, zorder=7, markeredgecolor='k', label='Meta')

ax.legend(loc='upper right')
plt.grid(True, linestyle=':', alpha=0.6)
plt.savefig(OUTPUT_PNG, dpi=150, bbox_inches='tight')
print(f"Gráfico guardado en: {OUTPUT_PNG}")
plt.show()