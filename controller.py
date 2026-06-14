from controller import Robot
import math
import csv
import heapq

# Algoritmo A*
# Esta matriz representa tu mundo en Webots 10x10.  
# Se ajusta
# 0 = Libre, 1 = Obstáculo.
MAPA_GRILLA = [
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 0], # Fila 0 (Y = 0.4 a 0.5) - Obstáculo superior derecho
    [0, 0, 0, 0, 0, 0, 1, 1, 1, 0], # Fila 1 (Y = 0.3 a 0.4)
    [1, 1, 1, 1, 0, 0, 0, 0, 0, 0], # Fila 2 (Y = 0.2 a 0.3) - Obstáculo superior izquierdo
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0], # Fila 3 (Y = 0.1 a 0.2)
    [1, 1, 1, 0, 0, 0, 0, 1, 1, 0], # Fila 4 (Y = 0.0 a 0.1) - Obstáculos centrales
    [1, 1, 1, 0, 0, 1, 1, 1, 1, 0], # Fila 5 (Y = -0.1 a 0.0) 
    [1, 1, 1, 0, 0, 1, 1, 1, 1, 0], # Fila 6 (Y = -0.2 a -0.1) 
    [0, 0, 0, 0, 0, 1, 1, 1, 1, 0], # Fila 7 (Y = -0.3 a -0.2) 
    [0, 1, 1, 1, 0, 0, 0, 0, 0, 0], # Fila 8 (Y = -0.4 a -0.3) - Obstáculo inferior izquierdo
    [0, 1, 1, 1, 0, 0, 0, 0, 0, 0]  # Fila 9 (Y = -0.5 a -0.4) - El robot parte en Fila 9, Columna 5
]
TAMANO_CELDA = 0.10  # 10 centímetros

def heuristic(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])

def a_star(mapa, start, goal):
    filas = len(mapa)
    cols = len(mapa[0])
    vecinos = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)] # 8 direcciones
    
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
            # Validar límites
            if 0 <= neighbor[0] < filas and 0 <= neighbor[1] < cols:
                if mapa[neighbor[0]][neighbor[1]] == 1:
                    continue
            else:
                continue
                
            tentative_g_score = gscore[current] + math.hypot(i, j)
            
            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
                
            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False # No hay ruta

# Convertir nodos de la matriz (fila, col) a coordenadas globales (x, y) de Webots
def generar_waypoints(start_nodo, goal_nodo):
    ruta_nodos = a_star(MAPA_GRILLA, start_nodo, goal_nodo)
    waypoints = []
    if ruta_nodos:
        for nodo in ruta_nodos:
            # Fórmulas de traslación de Grilla a Webots
            # El centro de la primera celda (0,0) está en X = -0.45, Y = 0.45
            x_mundo = (nodo[0] * TAMANO_CELDA) - 0.45
            y_mundo = 0.45 - (nodo[1] * TAMANO_CELDA)
            
            waypoints.append((x_mundo, y_mundo))
        print("Ruta planificada exitosamente:", waypoints)
    else:
        print("ERROR: A* no encontró ruta.")
    return waypoints

# Configuración básica robot (Lab 2)
robot = Robot()
TIME_STEP = int(robot.getBasicTimeStep())
WHEEL_RADIUS = 0.0205
WHEEL_BASE = 0.052  # Distancia entre ruedas (L)
MAX_SPEED = 6.28

ADVANCE_SPEED = MAX_SPEED * 0.4 
BACKUP_SPEED = MAX_SPEED * 0.4
TURN_SPEED = MAX_SPEED * 0.6

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

left_encoder = robot.getDevice('left wheel sensor')
right_encoder = robot.getDevice('right wheel sensor')
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

front_right_sensor = robot.getDevice('ps0')
front_left_sensor = robot.getDevice('ps7')
diag_right_sensor = robot.getDevice('ps1')
diag_left_sensor = robot.getDevice('ps6')
left_sensor = robot.getDevice('ps2')
right_sensor = robot.getDevice('ps5')

for s in [front_right_sensor, front_left_sensor, diag_right_sensor, diag_left_sensor, left_sensor, right_sensor]:
    s.enable(TIME_STEP)

# Variables globales y odometría
# =============================
x_k, y_k, phi_k = 0.0, 0.0, 0.0 # Posición global inicial (Odometría)
prev_enc_left, prev_enc_right = 0.0, 0.0
first_iteration = True

# Variables Kalman y Lab 2
MODO_NAVEGACION = "KALMAN"
d_est = 0.05
P, Q, R = 1.0, 0.001, 0.0005

# Máquina de estados actualizada
STATE_FOLLOW_PATH = 0  # Reemplaza a STATE_ADVANCING
STATE_BACKING = 1
STATE_TURNING = 2
nav_state = STATE_FOLLOW_PATH

# Parámetros Lab 2
SAFE_DISTANCE = 0.045
BACKUP_STEPS = 15
MIN_TURN_STEPS = 20
MAX_TURN_STEPS = 120
ESCAPE_TURN_STEPS = 90
STUCK_WINDOW = 600
STUCK_THRESHOLD = 4
TIEMPO_MAXIMO_ATASCADO = 150

tiempo_sin_avanzar = 0
backup_count = 0
turn_steps_done = 0
turn_max_current = MIN_TURN_STEPS
is_turning_left = False
turn_history = []
is_escape_turn = False
iteracion = 0

# planificación global
# ==========================================
# El robot parte físicamente cerca de X=0.06, Y=-0.4 (Fila 9, Columna 5 en la matriz)
NODO_INICIO = (9, 5) 

# Le diremos que vaya a la esquina superior izquierda, que está libre
NODO_META = (1, 1)

waypoints = generar_waypoints(NODO_INICIO, NODO_META)
current_wp_index = 0
RADIO_TOLERANCIA = 0.08  # 8 cm de tolerancia euclidiana para llegar a un nodo
LLEGO_A_LA_META = False

print(f"Modo de navegación activo: {MODO_NAVEGACION}")

# loop principal
while robot.step(TIME_STEP) != -1 and not LLEGO_A_LA_META:
    
    # --- A. LECTURA Y ODOMETRÍA ---
    val_enc_left = left_encoder.getValue()
    val_enc_right = right_encoder.getValue()

    if first_iteration:
        prev_enc_left = val_enc_left
        prev_enc_right = val_enc_right
        first_iteration = False

    delta_theta_left = val_enc_left - prev_enc_left
    delta_theta_right = val_enc_right - prev_enc_right
    prev_enc_left = val_enc_left
    prev_enc_right = val_enc_right

    ds_l = WHEEL_RADIUS * delta_theta_left
    ds_r = WHEEL_RADIUS * delta_theta_right
    ds = (ds_l + ds_r) / 2.0
    dphi = (ds_r - ds_l) / WHEEL_BASE

    # Actualizar coordenadas (Ecuaciones del proyecto)
    x_k += ds * math.cos(phi_k + dphi / 2.0)
    y_k += ds * math.sin(phi_k + dphi / 2.0)
    phi_k += dphi
    phi_k = math.atan2(math.sin(phi_k), math.cos(phi_k))

    # --- B. LECTURA DE SENSORES Y KALMAN ---
    v_fr, v_fl = front_right_sensor.getValue(), front_left_sensor.getValue()
    v_dr, v_dl = diag_right_sensor.getValue(), diag_left_sensor.getValue()
    v_l, v_r = left_sensor.getValue(), right_sensor.getValue()

    dist_fr = 0.05 * (1.0 - (v_fr / 4095.0))
    dist_fl = 0.05 * (1.0 - (v_fl / 4095.0))
    dist_dr = 0.05 * (1.0 - (v_dr / 4095.0))
    dist_dl = 0.05 * (1.0 - (v_dl / 4095.0))
    z_k = min(dist_fr, dist_fl, dist_dr, dist_dl)

    # Kalman (Lab 2)
    delta_d_k = -ds
    d_pred = d_est + delta_d_k
    P_pred = P + Q
    K_k = P_pred / (P_pred + R)
    d_est = d_pred + K_k * (z_k - d_pred)
    P = (1 - K_k) * P_pred

    obstacle_detected = d_est <= SAFE_DISTANCE

    # --- C. TOLERANCIA EUCLIDIANA ---
    if current_wp_index < len(waypoints):
        target_x, target_y = waypoints[current_wp_index]
        dist_to_target = math.hypot(target_x - x_k, target_y - y_k)
        
        # Si entra en el radio, avanza al siguiente punto
        if dist_to_target < RADIO_TOLERANCIA:
            current_wp_index += 1
            print(f"[It {iteracion}] Llegó al Waypoint {current_wp_index}")
            
            if current_wp_index >= len(waypoints):
                print("\n¡META FINAL ALCANZADA! El robot está dentro de la tolerancia euclidiana.")
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                LLEGO_A_LA_META = True
                break

    # --- D. MÁQUINA DE ESTADOS (Fusión Lab 2 y A*) ---
    if nav_state == STATE_FOLLOW_PATH:
        if obstacle_detected:
            # Lógica de evasión Lab 2
            tiempo_sin_avanzar += 1
            turn_history.append(iteracion)
            turn_history = [t for t in turn_history if iteracion - t < STUCK_WINDOW]

            bucle_infinito = len(turn_history) >= STUCK_THRESHOLD
            atascado_tiempo = tiempo_sin_avanzar > TIEMPO_MAXIMO_ATASCADO

            if bucle_infinito or atascado_tiempo:
                print(f">>> ¡Atascado! Forzando escape 180°")
                is_turning_left = not is_turning_left
                turn_max_current = ESCAPE_TURN_STEPS
                is_escape_turn = True
                turn_history = []
                tiempo_sin_avanzar = 0 
            else:
                peso_derecho = v_r + v_dr
                peso_izquierdo = v_l + v_dl
                is_turning_left = (peso_derecho > peso_izquierdo)
                turn_max_current = MAX_TURN_STEPS
                is_escape_turn = False

            backup_count = BACKUP_STEPS
            turn_steps_done = 0
            nav_state = STATE_BACKING
            left_speed, right_speed = -BACKUP_SPEED, -BACKUP_SPEED
        else:
            # Seguimiento Proporcional al Waypoint (Línea A)
            tiempo_sin_avanzar = 0
            target_angle = math.atan2(target_y - y_k, target_x - x_k)
            angle_error = target_angle - phi_k
            angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
            
            Kp = 4.0
            omega = Kp * angle_error
            v_lineal = ADVANCE_SPEED
            
            if abs(angle_error) > 0.4:
                v_lineal = ADVANCE_SPEED * 0.3 # Desacelera en curvas fuertes
                
            left_speed = v_lineal - (omega * WHEEL_BASE / 2.0)
            right_speed = v_lineal + (omega * WHEEL_BASE / 2.0)

    elif nav_state == STATE_BACKING:
        backup_count -= 1
        if backup_count > 0:
            left_speed, right_speed = -BACKUP_SPEED, -BACKUP_SPEED
        else:
            nav_state = STATE_TURNING
            left_speed = -TURN_SPEED if is_turning_left else TURN_SPEED
            right_speed = TURN_SPEED if is_turning_left else -TURN_SPEED

    elif nav_state == STATE_TURNING:
        turn_steps_done += 1
        min_done = turn_steps_done >= MIN_TURN_STEPS
        max_reached = turn_steps_done >= turn_max_current
        front_clear = d_est > SAFE_DISTANCE

        done = max_reached if is_escape_turn else (max_reached or (min_done and front_clear))

        if done:
            nav_state = STATE_FOLLOW_PATH  # Termina evasión, vuelve a seguir la ruta
            is_escape_turn = False
            tiempo_sin_avanzar = 0
        else:
            left_speed = -TURN_SPEED if is_turning_left else TURN_SPEED
            right_speed = TURN_SPEED if is_turning_left else -TURN_SPEED

    # Saturación y asignación
    left_speed = max(min(left_speed, MAX_SPEED), -MAX_SPEED)
    right_speed = max(min(right_speed, MAX_SPEED), -MAX_SPEED)
    
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    iteracion += 1