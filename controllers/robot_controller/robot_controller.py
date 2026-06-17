from controller import Robot
import math
import csv
import heapq
import os

# =============================================================================
# CONFIGURACIÓN MAESTRA DEL ESCENARIO
# =============================================================================
# 'SIMPLE' o 'COMPLEJO' o 'MUY_COMPLEJO'
ESCENARIO_ACTUAL = "MUY_COMPLEJO" 

TAMANO_CELDA = 0.05
INFLACION    = 0.05

if ESCENARIO_ACTUAL == "SIMPLE":
    ARENA_HALF = 0.5  # Mundo de 1x1 metros
    GRID_N     = int(round((ARENA_HALF * 2) / TAMANO_CELDA)) # 20x20
    
    START_X, START_Y, START_PHI = 0.0611, -0.4, 1.5638
    META_X,  META_Y             = -0.413, 0.413   # esquina superior izquierda, en zona claramente libre
    
    # Lista REAL de obstáculos (centro [x,y], tamaño [ancho,alto], rotación rad).
    # Debe coincidir con los Solid de worlds/escenario_simple.wbt.
    # Lista REAL de obstáculos extraída del nuevo mundo
    OBSTACULOS = [
    {"t": [ 0.23,    0.41],     "s": [0.2, 0.2], "ang": 0.523599},
    {"t": [-0.26,   -0.40],     "s": [0.2, 0.2], "ang": 0.0},
    {"t": [-0.28,    0.27],     "s": [0.3, 0.1], "ang": 0.0},
    {"t": [ 0.33,   -0.12],     "s": [0.1, 0.4], "ang": 0.0},
    {"t": [-0.352219, -0.0529289], "s": [0.3, 0.2], "ang": 0.785398},
    {"t": [ 0.07,   -0.14],     "s": [0.2, 0.2], "ang": 0.0},
    ]

elif ESCENARIO_ACTUAL == "COMPLEJO":
    ARENA_HALF = 1.0  # Mundo de 2x2 metros
    GRID_N     = int(round((ARENA_HALF * 2) / TAMANO_CELDA)) # 40x40
    
    # Posición inicial basada en tu e-puck (-0.87, -0.87)
    START_X, START_Y, START_PHI = -0.87, -0.87, 1.5708
    
    # Meta en la esquina superior derecha (esquivando los cilindros y bloques)
    META_X,  META_Y             = 0.80, 0.80  
    
    OBSTACULOS = [
        # Bloques rectangulares
        {"t": [ 0.19,       -0.56],     "s": [0.6, 0.3], "ang": 0.0},
        {"t": [ 0.34,       -0.11],     "s": [0.6, 0.3], "ang": -1.570795},
        {"t": [ 0.59,       -0.24],     "s": [0.3, 0.2], "ang": -1.570795},
        # "bloque_invisible" en [0.34, -0.86] fue OMITIDO INTENCIONALMENTE para forzar evasión
        {"t": [ 0.90,        0.34],     "s": [0.3, 0.2], "ang": -1.570795},
        {"t": [ 0.0100693,   0.589526], "s": [0.1, 0.3], "ang": -0.523595},
        {"t": [ 0.19,        0.34],     "s": [0.6, 0.3], "ang": 0.0},
        {"t": [-0.89993,    -0.562913], "s": [0.1, 0.2], "ang": 1.5708},
        
        # Cilindros (aproximados como cajas usando el diámetro: radio * 2)
        {"t": [-0.5, -0.1],  "s": [0.52, 0.52], "ang": 0.0}, # Cilindro 1 (r=0.26)
        {"t": [-0.8,  0.79], "s": [0.80, 0.80], "ang": 0.0}, # Cilindro 2 (r=0.40)
    ]
elif ESCENARIO_ACTUAL == "MUY_COMPLEJO":
    ARENA_HALF = 1.5  # Mundo enorme de 3x3 metros
    GRID_N     = int(round((ARENA_HALF * 2) / TAMANO_CELDA)) # 60x60
    
    # Posición inicial exacta de tu e-puck
    START_X, START_Y, START_PHI = -0.0027, -1.436, -1.5838
    META_X,  META_Y = -1.25, 1.25 
    
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
    raise ValueError("ESCENARIO_ACTUAL no válido. Usa 'SIMPLE', 'COMPLEJO' o 'MUY_COMPLEJO'.")

def celda_a_mundo(row, col):
    """Centro (x, y) de la celda (fila, columna)."""
    x = -ARENA_HALF + (col + 0.5) * TAMANO_CELDA
    y =  ARENA_HALF - (row + 0.5) * TAMANO_CELDA
    return x, y

def mundo_a_celda(x, y):
    """Celda (fila, columna) que contiene la coordenada de mundo (x, y)."""
    col = int(round((x + ARENA_HALF) / TAMANO_CELDA - 0.5))
    row = int(round((ARENA_HALF - y) / TAMANO_CELDA - 0.5))
    col = max(0, min(GRID_N - 1, col))
    row = max(0, min(GRID_N - 1, row))
    return row, col

def construir_mapa(obstaculos, inflacion):
    """Rasteriza los obstáculos reales (con rotación e inflación) a una grilla."""
    mapa = [[0] * GRID_N for _ in range(GRID_N)]
    for row in range(GRID_N):
        for col in range(GRID_N):
            cx, cy = celda_a_mundo(row, col)
            # Margen contra las paredes de la arena
            if abs(cx) > ARENA_HALF - inflacion or abs(cy) > ARENA_HALF - inflacion:
                mapa[row][col] = 1
                continue
            for o in obstaculos:
                dx = cx - o["t"][0]
                dy = cy - o["t"][1]
                a = -o["ang"]  # llevar el centro de la celda al marco local del obstáculo
                lx = dx * math.cos(a) - dy * math.sin(a)
                ly = dx * math.sin(a) + dy * math.cos(a)
                if abs(lx) <= o["s"][0] / 2 + inflacion and abs(ly) <= o["s"][1] / 2 + inflacion:
                    mapa[row][col] = 1
                    break
    return mapa

MAPA_GRILLA = construir_mapa(OBSTACULOS, INFLACION)

# =============================================================================
# ALGORITMO A*  (8 vecinos, sin corte de esquinas)
# =============================================================================
def heuristic(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])

def a_star(mapa, start, goal):
    filas = len(mapa)
    cols = len(mapa[0])
    vecinos = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(-1,-1),(1,-1),(-1,1)]

    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data[::-1]

        close_set.add(current)
        for i, j in vecinos:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < filas and 0 <= neighbor[1] < cols:
                if mapa[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue

            # En diagonal, no cortar esquinas: ambas celdas ortogonales deben estar libres
            if i != 0 and j != 0:
                if mapa[current[0] + i][current[1]] == 1 or mapa[current[0]][current[1] + j] == 1:
                    continue

            tentative_g = gscore[current] + math.hypot(i, j)
            if neighbor in close_set and tentative_g >= gscore.get(neighbor, 0):
                continue
            if tentative_g < gscore.get(neighbor, 0) or neighbor not in [n[1] for n in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g
                fscore[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

def generar_waypoints(start_nodo, goal_nodo):
    ruta_nodos = a_star(MAPA_GRILLA, start_nodo, goal_nodo)
    waypoints = []
    if ruta_nodos:
        for nodo in ruta_nodos:
            waypoints.append(celda_a_mundo(nodo[0], nodo[1]))
        print("Ruta A* planificada con éxito.")
    else:
        print("ERROR: A* no encontró ruta.")
    return waypoints

# =============================================================================
# HARDWARE DEL ROBOT Y CINEMÁTICA
# =============================================================================
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())
WHEEL_RADIUS = 0.0205
WHEEL_BASE   = 0.052
MAX_SPEED    = 6.28 # rad/s

V_LIN_MAX        = 0.06  # m/s
TURN_SPEED_RAD   = MAX_SPEED * 0.6
BACKUP_SPEED_RAD = MAX_SPEED * 0.4

left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_encoder  = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

front_right_sensor = robot.getDevice('ps0')
front_left_sensor  = robot.getDevice('ps7')
diag_right_sensor  = robot.getDevice('ps1')
diag_left_sensor   = robot.getDevice('ps6')
left_sensor        = robot.getDevice('ps2')
right_sensor       = robot.getDevice('ps5')
for s in [front_right_sensor, front_left_sensor, diag_right_sensor, diag_left_sensor, left_sensor, right_sensor]:
    s.enable(TIME_STEP)

# =============================================================================
# PLANIFICACIÓN GLOBAL
# =============================================================================
# Pose REAL de partida del robot en Webots (translation/rotation del E-puck en el .wbt).
# La odometría se inicializa aquí; el robot NO vuelve a leer su posición real (solo odometría).
#START_X, START_Y, START_PHI = 0.0, -0.37, 1.5708
# START_X, START_Y, START_PHI = 0.0611, -0.4, 1.5638
# META_X,  META_Y             = -0.04, 0.28   # esquina superior izquierda, en zona claramente libre

NODO_INICIO = mundo_a_celda(START_X, START_Y)
NODO_META   = mundo_a_celda(META_X, META_Y)

# La inflación puede tapar la celda de inicio o meta: las forzamos libres.
MAPA_GRILLA[NODO_INICIO[0]][NODO_INICIO[1]] = 0
MAPA_GRILLA[NODO_META[0]][NODO_META[1]]     = 0

waypoints = generar_waypoints(NODO_INICIO, NODO_META)
current_wp_index = 0
RADIO_TOLERANCIA = 0.025  # ajustado (grilla 5 cm); tracking ceñido para no recortar esquinas
LLEGO_A_LA_META  = False

# =============================================================================
# ODOMETRÍA INICIAL
# =============================================================================
x_k   = START_X
y_k   = START_Y
phi_k = START_PHI

x_k_init, y_k_init = x_k, y_k
prev_enc_left, prev_enc_right = 0.0, 0.0
first_iteration = True

# =============================================================================
# CSV Y VARIABLES
# =============================================================================
_SCRIPT_DIR      = os.path.dirname(os.path.abspath(__file__))
PLANNED_PATH_CSV = os.path.join(_SCRIPT_DIR, f'planned_path_{ESCENARIO_ACTUAL}.csv')
TRAJECTORY_CSV   = os.path.join(_SCRIPT_DIR, f'trajectory_{ESCENARIO_ACTUAL}.csv')

with open(PLANNED_PATH_CSV, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['waypoint_index', 'grid_row', 'grid_col', 'x_planned', 'y_planned'])
    w.writerow([0, NODO_INICIO[0], NODO_INICIO[1], f"{x_k_init:.5f}", f"{y_k_init:.5f}"])
    ruta_nodos = a_star(MAPA_GRILLA, NODO_INICIO, NODO_META)
    if ruta_nodos:
        for i, nodo in enumerate(ruta_nodos):
            xp, yp = celda_a_mundo(nodo[0], nodo[1])
            w.writerow([i + 1, nodo[0], nodo[1], f"{xp:.5f}", f"{yp:.5f}"])

_traj_file   = open(TRAJECTORY_CSV, 'w', newline='')
_traj_writer = csv.writer(_traj_file)
_traj_writer.writerow(['step', 'sim_time_s', 'x_actual', 'y_actual', 'phi_k', 'nav_state', 'current_wp_index', 'd_est', 'obstacle_detected', 'left_speed', 'right_speed'])

STATE_FOLLOW_PATH = 0
STATE_BACKING     = 1
STATE_TURNING     = 2
nav_state = STATE_FOLLOW_PATH

BACKUP_STEPS         = 15
MIN_TURN_STEPS       = 20
MAX_TURN_STEPS       = 120
ESCAPE_TURN_STEPS    = 90
STUCK_WINDOW         = 600
STUCK_THRESHOLD      = 4
TIEMPO_MAXIMO_ATASCADO = 150
MAX_ITERATIONS       = 20000

tiempo_sin_avanzar = 0
backup_count       = 0
turn_steps_done    = 0
turn_max_current   = MIN_TURN_STEPS
is_turning_left    = False
turn_history       = []
is_escape_turn     = False
iteracion          = 0

d_est = 0.05
P, Q, R = 1.0, 0.001, 0.0005
left_speed = 0.0
right_speed = 0.0

print(f"Start: {NODO_INICIO} | Meta: {NODO_META} | Total WP: {len(waypoints)}")

# =============================================================================
# LOOP PRINCIPAL
# =============================================================================
while robot.step(TIME_STEP) != -1 and not LLEGO_A_LA_META:

    # ── A. ODOMETRÍA ──
    val_enc_left  = left_encoder.getValue()
    val_enc_right = right_encoder.getValue()

    if first_iteration:
        prev_enc_left, prev_enc_right = val_enc_left, val_enc_right
        first_iteration = False

    delta_theta_left  = val_enc_left  - prev_enc_left
    delta_theta_right = val_enc_right - prev_enc_right
    prev_enc_left, prev_enc_right = val_enc_left, val_enc_right

    ds_l = WHEEL_RADIUS * delta_theta_left
    ds_r = WHEEL_RADIUS * delta_theta_right
    ds   = (ds_l + ds_r) / 2.0
    dphi = (ds_r - ds_l) / WHEEL_BASE

    x_k   += ds * math.cos(phi_k + dphi / 2.0)
    y_k   += ds * math.sin(phi_k + dphi / 2.0)
    phi_k += dphi
    phi_k  = math.atan2(math.sin(phi_k), math.cos(phi_k))

    # ── B. FILTRO KALMAN ──
    v_fr, v_fl = front_right_sensor.getValue(), front_left_sensor.getValue()
    v_dr, v_dl = diag_right_sensor.getValue(),  diag_left_sensor.getValue()
    v_l,  v_r  = left_sensor.getValue(),         right_sensor.getValue()

    # Umbral ALTO a propósito: como la ruta A* es provadamente libre (inflación 5 cm),
    # solo activamos la evasión a ciegas ante una colisión INMINENTE (sensor IR ~<1 cm).
    # Así el robot confía en su ruta y no abandona el pasillo (lo que arruinaba la odometría).
    UMBRAL_RAW = 2000.0
    max_sensor_val = max(v_fr, v_fl, v_dr, v_dl)
    z_k = max_sensor_val / 4095.0 

    d_pred = d_est 
    P_pred = P + Q
    K_k    = P_pred / (P_pred + R)
    d_est  = d_pred + K_k * (z_k - d_pred)
    P      = (1 - K_k) * P_pred

    intensidad_umbral = UMBRAL_RAW / 4095.0
    obstacle_detected = d_est >= intensidad_umbral

    # ── C. VERIFICAR WAYPOINT ──
    if current_wp_index < len(waypoints):
        target_x, target_y = waypoints[current_wp_index]
        dist_to_target = math.hypot(target_x - x_k, target_y - y_k)
        if dist_to_target < RADIO_TOLERANCIA:
            current_wp_index += 1
            if current_wp_index >= len(waypoints):
                print("¡META FINAL ALCANZADA!")
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                LLEGO_A_LA_META = True
                break
    else:
        break

    # ── D. MÁQUINA DE ESTADOS ──
    if nav_state == STATE_FOLLOW_PATH:
        if obstacle_detected:
            tiempo_sin_avanzar += 1
            turn_history.append(iteracion)
            turn_history = [t for t in turn_history if iteracion - t < STUCK_WINDOW]

            if len(turn_history) >= STUCK_THRESHOLD or tiempo_sin_avanzar > TIEMPO_MAXIMO_ATASCADO:
                is_turning_left  = not is_turning_left
                turn_max_current = ESCAPE_TURN_STEPS
                is_escape_turn   = True
                turn_history     = []
                tiempo_sin_avanzar = 0
            else:
                is_turning_left  = ((v_r + v_dr) > (v_l + v_dl))
                turn_max_current = MAX_TURN_STEPS
                is_escape_turn   = False

            backup_count    = BACKUP_STEPS
            turn_steps_done = 0
            nav_state       = STATE_BACKING
            left_speed, right_speed = -BACKUP_SPEED_RAD, -BACKUP_SPEED_RAD
        else:
            tiempo_sin_avanzar = 0
            target_angle = math.atan2(target_y - y_k, target_x - x_k)
            angle_error  = math.atan2(math.sin(target_angle - phi_k), math.cos(target_angle - phi_k))

            # Stop-and-Turn
            if abs(angle_error) > 0.15: 
                v_lin = 0.0 
                omega = 3.0 * angle_error
            else:
                v_lin = V_LIN_MAX 
                omega = 2.0 * angle_error

            v_l = v_lin - (omega * WHEEL_BASE / 2.0)
            v_r = v_lin + (omega * WHEEL_BASE / 2.0)

            left_speed  = v_l / WHEEL_RADIUS
            right_speed = v_r / WHEEL_RADIUS

    elif nav_state == STATE_BACKING:
        backup_count -= 1
        if backup_count <= 0:
            nav_state = STATE_TURNING
            left_speed  = -TURN_SPEED_RAD if is_turning_left else  TURN_SPEED_RAD
            right_speed =  TURN_SPEED_RAD if is_turning_left else -TURN_SPEED_RAD
        else:
            left_speed, right_speed = -BACKUP_SPEED_RAD, -BACKUP_SPEED_RAD

    elif nav_state == STATE_TURNING:
        turn_steps_done += 1
        done = (turn_steps_done >= turn_max_current) if is_escape_turn else (turn_steps_done >= turn_max_current or (turn_steps_done >= MIN_TURN_STEPS and not obstacle_detected))
        if done:
            nav_state = STATE_FOLLOW_PATH
            is_escape_turn = False
            tiempo_sin_avanzar = 0
        else:
            left_speed  = -TURN_SPEED_RAD if is_turning_left else  TURN_SPEED_RAD
            right_speed =  TURN_SPEED_RAD if is_turning_left else -TURN_SPEED_RAD

    left_speed  = max(min(left_speed,  MAX_SPEED), -MAX_SPEED)
    right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)

    sim_time = iteracion * TIME_STEP / 1000.0
    _traj_writer.writerow([iteracion, f"{sim_time:.4f}", f"{x_k:.5f}", f"{y_k:.5f}", f"{phi_k:.5f}", nav_state, current_wp_index, f"{d_est:.5f}", int(obstacle_detected), f"{left_speed:.4f}", f"{right_speed:.4f}"])
    
    iteracion += 1
    if iteracion >= MAX_ITERATIONS:
        break

_traj_file.close()