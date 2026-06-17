# Navegación Autónoma con Planificación de Rutas A* en Webots

**Línea seleccionada:** Planificación de Rutas

## Integrantes del grupo

| Nombre |
|--------|
| Felipe Astudillo (Programador, análisis de datos, documentador) |
| Martina Sandoval (Programador, diseño del entorno) |
| Julian Guerrero (Documentador, análisis de datos) |

---

## Contenidos

1. [Objetivo del proyecto](#objetivo-del-proyecto)
2. [Relación con los Laboratorios 1 y 2](#relación-con-los-laboratorios-1-y-2)
3. [Descripción del robot, sensores y actuadores](#descripción-del-robot-sensores-y-actuadores)
4. [Descripción de los escenarios de prueba](#descripción-de-los-escenarios-de-prueba)
5. [Explicación del algoritmo implementado](#explicación-del-algoritmo-implementado)
6. [Diagrama de flujo de la solución](#diagrama-de-flujo-de-la-solución)
7. [Resultados obtenidos y métricas de desempeño](#resultados-obtenidos-y-métricas-de-desempeño)
8. [Videos de la simulación](#videos-de-la-simulación)
9. [Instrucciones para ejecutar la simulación](#instrucciones-para-ejecutar-la-simulación)
10. [Conclusiones, limitaciones y posibles mejoras](#conclusiones-limitaciones-y-posibles-mejoras)

---

## Objetivo del proyecto

Desarrollar un sistema de navegación autónoma híbrido para el robot diferencial E-puck en el simulador Webots. El proyecto implementa una capa deliberativa utilizando el algoritmo A* para la planificación global de rutas sobre un mapa discretizado, y una capa reactiva de evasión de obstáculos basada en sensores infrarrojos. El rendimiento de la arquitectura se evalúa empíricamente en tres entornos de complejidad creciente, contrastando la trayectoria cinemática real (odometría) frente a la ruta ideal planificada.

---

## Relación con los Laboratorios 1 y 2

Este proyecto final integra y extiende los conceptos y herramientas desarrollados en los dos laboratorios previos del curso. La siguiente tabla resume la progresión acumulativa:

| Laboratorio | Aporte al proyecto final |
|-------------|--------------------------|
| **Lab 1** – Cinemática del robot diferencial | Modelo cinemático, plataforma e-puck, control de velocidades |
| **Lab 2** – Navegación reactiva y filtrado | Filtro de Kalman, máquina de estados reactiva, anti-atasco |
| **Proyecto Final** | Planificación global A* sobre mapa discretizado + capas anteriores |

### Desde el Laboratorio 1

En el Lab 1 se estudió el comportamiento cinemático del e-puck al variar las velocidades de las ruedas izquierda $v_l$ y derecha $v_r$:

- **Modelo cinemático:** $v = (v_r + v_l) / 2$, $\omega = (v_r - v_l) / L$  
  Este mismo modelo es la base de la odometría del proyecto final. En cada paso de 32 ms se calcula `ds = (ds_l + ds_r) / 2` y `dφ = (ds_r - ds_l) / L` para estimar la pose `(x, y, φ)` del robot.

- **Control de movimiento:** Los tres patrones explorados en Lab 1 reaparecen directamente en el controlador Stop-and-Turn del proyecto final:
  - Línea recta (`vr = vl`) → fase de avance hacia el waypoint.
  - Giro en el eje (`vr = −vl`) → fase de reorientación del Stop-and-Turn y de la evasión reactiva.
  - Curva (`vr ≠ vl`) → corrección suave de ángulo durante el avance.

- **Perturbaciones:** Las pruebas con ruido en los actuadores del Lab 1 evidenciaron la necesidad de complementar el control cinemático con retroalimentación sensorial, motivando el filtrado desarrollado en Lab 2 e incorporado al proyecto final.

### Desde el Laboratorio 2

El Lab 2 implementó un sistema de navegación reactiva para el mismo e-puck, con sensores IR y encoders. Sus contribuciones se reutilizan directamente en el proyecto final:

1. **Filtro de Kalman escalar:** Se reutiliza la estructura del filtro de Kalman del Lab 2 con los mismos parámetros (`Q = 0.001`, `R = 0.0005`). En el proyecto final el filtro suaviza la **intensidad IR normalizada** (`z_k = max(ps0,ps1,ps6,ps7) / 4095`) para reducir falsas detecciones. A diferencia del Lab 2 (donde la predicción incorporaba el avance por encoder), aquí la predicción es un modelo de proceso constante (`d_pred = d_est`), ya que la ruta A* garantiza que el robot no debería acercarse a obstáculos durante el avance normal.

2. **Máquina de estados reactiva:** La secuencia ADVANCING → BACKING → TURNING del Lab 2 se hereda como la capa de evasión reactiva del proyecto final (FOLLOW_PATH → BACKING → TURNING), con los mismos 15 pasos de retroceso y el rango de 20–120 pasos de giro.

3. **Mecanismo anti-atasco:** La lógica de detección de bucle (≥ 4 giros en una ventana de 600 iteraciones) y el giro de escape forzado de 90 pasos se transfieren sin modificación.

4. **Configuración de sensores:** El mismo tiempo de muestreo (Ts = 32 ms), radio de rueda (`R = 0.0205 m`) y distribución de los sensores IR (`ps0–ps7`) se mantienen constantes a lo largo de los tres trabajos.

---

## Descripción del robot, sensores y actuadores

**Plataforma de simulación:** Webots R2023b  
**Robot:** E-puck (tracción diferencial)

### Cinemática y Actuadores

| Parámetro / Dispositivo | Especificación |
|-------------------------|---------------|
| Motor izquierdo y derecho | Velocidad máxima: 6.28 rad/s |
| Radio de rueda ($R$) | 20.5 mm |
| Distancia entre ejes ($L$) | 52 mm |

### Sensores

| Sensor | Uso en el proyecto |
|--------|--------------------|
| 8 sensores IR de proximidad (`ps0`–`ps7`) | Detección de obstáculos imprevistos |
| Encoders de ruedas (izquierdo y derecho) | Odometría para estimar posición `(x, y, φ)` |

Los sensores frontales y diagonales (`ps0`, `ps1`, `ps6`, `ps7`) se fusionan mediante un **Filtro de Kalman escalar** para suavizar las lecturas ruidosas y estimar la proximidad al obstáculo más cercano. El filtro trabaja sobre intensidad normalizada (0 = sin obstáculo, 1 = contacto); el obstáculo se considera detectado cuando la estimación supera el umbral de `2000/4095 ≈ 0.488`.

### Percepción y Filtrado

Para la evasión reactiva, se utilizan los sensores IR frontales y diagonales (`ps0`, `ps1`, `ps6`, `ps7`). Para mitigar el ruido inherente de los sensores, se extrae la lectura de mayor proximidad y se le aplica un **Filtro de Kalman escalar unidimensional**, permitiendo obtener una estimación robusta de la distancia al obstáculo más inminente. La pose del robot $(x, y, \phi)$ se estima internamente mediante los encoders de las ruedas.

**Modelo de Odometría (Paso de simulación: 32 ms):**
```
ds_l = R · Δθ_izquierda
ds_r = R · Δθ_derecha
ds   = (ds_l + ds_r) / 2
dφ   = (ds_r - ds_l) / L

x_{k+1} = x_k + ds · cos(φ_k + dφ/2)
y_{k+1} = y_k + ds · sin(φ_k + dφ/2)
φ_{k+1} = φ_k + dφ
```
Donde `R = 0.0205 m` (radio) y `L = 0.052 m` (base).

---

## Descripción de los escenarios de prueba

Se crearon tres mundos en Webots de complejidad creciente. Todos comparten el mismo controlador; el escenario se selecciona con la variable `ESCENARIO_ACTUAL` en `controllers/robot_controller/robot_controller.py`.

| Escenario | Archivo | Arena | Grilla | Inicio → Meta | Obstáculos |
|-----------|---------|-------|--------|---------------|-----------|
| **SIMPLE** | `worlds/simple.wbt` | 1×1 m | 20×20 | (0.06, -0.40) → (-0.41, 0.41) | 6 bloques |
| **COMPLEJO** | `worlds/complejo.wbt` | 2×2 m | 40×40 | (-0.87, -0.87) → (0.80, 0.80) | 7 bloques + 2 cilindros |
| **MUY_COMPLEJO** | `worlds/complejo2.wbt` | 3×3 m | 60×60 | (-0.00, -1.44) → (-1.25, 1.25) | 22 obstáculos mixtos |

Todos los obstáculos se definen con posición `(x, y)`, tamaño `(ancho, alto)` y ángulo de rotación. La grilla usa celdas de **5 cm** con una inflación de seguridad de **5 cm** alrededor de cada obstáculo.

---

## Explicación del algoritmo implementado

El sistema combina tres capas de control:

### 1. Planificación global: A*

Se construye un mapa binario (`MAPA_GRILLA`) rasterizando los obstáculos del mundo con rotación e inflación. Sobre este mapa se ejecuta A* con:

- **8 vecinos** (movimiento en las 4 direcciones cardinales y las 4 diagonales)
- **Sin corte de esquinas:** en movimientos diagonales, ambas celdas ortogonales deben estar libres
- **Heurística:** distancia euclidiana al nodo meta
- **Costo de arista:** distancia euclidiana entre celdas (1.0 para ortogonal, √2 para diagonal)

El resultado es una lista de **waypoints** (coordenadas de mundo) que el robot debe seguir en orden.

### 2. Seguimiento de waypoints: Stop-and-Turn

Para cada waypoint activo se calcula el error de orientación. Si el error supera **0.15 rad**, el robot se detiene y gira en el lugar antes de avanzar:

```
error_ángulo = atan2(sin(θ_objetivo - φ_k), cos(θ_objetivo - φ_k))

Si |error_ángulo| > 0.15 rad:
    v_lin = 0                    # Detener traslación
    ω     = 3.0 · error_ángulo   # Solo girar

Si no:
    v_lin = 0.06 m/s             # Avanzar
    ω     = 2.0 · error_ángulo   # Corrección suave

v_izq = (v_lin - ω·L/2) / R
v_der = (v_lin + ω·L/2) / R
```

Un waypoint se da por alcanzado cuando la distancia euclidiana al robot es menor a **2.5 cm**.

### 3. Evasión reactiva

Como el mapa A* infla obstáculos, el robot confía en la ruta planificada. La evasión solo se activa ante colisiones inminentes: cuando la **estimación Kalman de intensidad** supera `2000/4095 ≈ 0.488` (equivalente a un obstáculo a menos de ~1 cm). El umbral es deliberadamente alto para evitar que el robot abandone la ruta planificada ante falsas detecciones. El sistema usa una **máquina de 3 estados**:

| Estado | Acción |
|--------|--------|
| `FOLLOW_PATH` | Navegación normal hacia waypoints |
| `BACKING` | Retrocede 15 pasos para desengancharse del obstáculo |
| `TURNING` | Gira hasta que el camino quede despejado (20–120 pasos); detecta el lado menos bloqueado por los sensores |

Un mecanismo anti-atasco detecta si el robot activa evasión `≥ 4` veces en una ventana de 600 pasos y ejecuta un giro de escape forzado de 90 pasos en la dirección opuesta a la última.

---

## Diagrama de flujo de la solución

```mermaid
flowchart TD
    A([Inicio]) --> B[Cargar configuración del escenario]
    B --> C[Rasterizar obstáculos → MAPA_GRILLA]
    C --> D[A*: Planificar ruta global]
    D --> E{¿Ruta encontrada?}
    E -- No --> F([Error: sin ruta disponible])
    E -- Sí --> G[Inicializar odometría y guardar CSV de ruta]
    G --> H{¿Meta alcanzada?}
    H -- Sí --> I([Fin: misión completada])
    H -- No --> J[Leer encoders → Actualizar odometría]
    J --> K[Leer sensores IR → Filtro de Kalman]
    K --> L{nav_state}
    L -- FOLLOW_PATH --> M{¿Obstáculo detectado?}
    M -- No --> N[Stop-and-Turn hacia waypoint actual]
    M -- Sí --> O[nav_state = BACKING\nRetroceder 15 pasos]
    N --> P[Aplicar velocidades a motores]
    O --> P
    L -- BACKING --> Q{¿Retroceso completo?}
    Q -- No --> R[Continuar retroceso]
    Q -- Sí --> S[nav_state = TURNING\nSeleccionar dirección de giro]
    R --> P
    S --> P
    L -- TURNING --> T{¿Giro completo o camino libre?}
    T -- No --> U[Continuar girando]
    T -- Sí --> V[nav_state = FOLLOW_PATH]
    U --> P
    V --> P
    P --> W[Registrar fila en trajectory_*.csv]
    W --> H
```

### Pseudocódigo del A*

```
función A_STAR(mapa, inicio, meta):
    abiertos  ← cola de prioridad con (f=h(inicio,meta), inicio)
    cerrados  ← conjunto vacío
    g[inicio] ← 0
    padre[·]  ← {}

    mientras abiertos no vacío:
        actual ← extraer nodo con menor f

        si actual == meta:
            reconstruir y retornar camino desde padre[·]

        cerrados.agregar(actual)

        para cada vecino en 8_vecinos(actual):
            si vecino fuera de límites o mapa[vecino] == OCUPADO:
                continuar
            si es diagonal y alguna celda ortogonal está ocupada:
                continuar   // sin corte de esquinas

            g_tentativo ← g[actual] + distancia_euclidiana(actual, vecino)

            si vecino en cerrados y g_tentativo ≥ g[vecino]:
                continuar

            si g_tentativo < g[vecino] o vecino no en abiertos:
                padre[vecino]  ← actual
                g[vecino]      ← g_tentativo
                f[vecino]      ← g_tentativo + distancia_euclidiana(vecino, meta)
                abiertos.insertar(f[vecino], vecino)

    retornar SIN_RUTA
```

---

## Resultados obtenidos y métricas de desempeño

Los **waypoints** son los nodos intermedios de la ruta calculada por A* — coordenadas del mundo real que el robot debe alcanzar en orden para ir del punto de inicio a la meta. Su avance se determina midiendo la distancia entre la pose estimada por odometría y cada waypoint activo; cuando esa distancia cae bajo 2.5 cm, el waypoint se da por completado y se avanza al siguiente.

Las métricas se calculan a partir de los archivos CSV generados durante la simulación.

| Escenario | Waypoints A* | Pasos de simulación | Duración | Activaciones de evasión | Éxito real |
|-----------|-------------|---------------------|----------|------------------------|------------|
| SIMPLE | 25 | 901 | 28.8 s | 35 | ✓ |
| COMPLEJO | 57 | 1,687 | 54.0 s | 0 | ✓ |
| MUY_COMPLEJO | 122 | 7,337 | 234.8 s | 1,890 | ✗ |

> **Nota sobre MUY_COMPLEJO:** el contador de waypoints completados (basado en odometría) **no refleja la realidad física**. En la simulación real, el robot acumuló tanta deriva odométrica desde el inicio que su trayectoria física divergió completamente de la estimada: el robot se desplazó en una dirección incorrecta y permaneció en su mayor parte cerca del punto de partida, mientras la odometría "creía" estar recorriendo la ruta planificada. Las 1,890 activaciones de evasión introdujeron rotaciones sucesivas que corrompieron irreversiblemente la estimación de ángulo.

**Observaciones:**
- En **SIMPLE**, la evasión se activa 35 veces debido al espacio reducido (1×1 m) y la cercanía de los obstáculos.
- En **COMPLEJO**, la ruta A* es suficientemente buena para llegar sin ninguna evasión reactiva.
- En **MUY_COMPLEJO**, el sistema **falló**. La combinación de un entorno muy denso (22 obstáculos, arena 3×3 m) con una misión larga (≈235 s) superó la tolerancia a la deriva del sistema. Sin relocalización ni corrección de pose, el error acumulado hizo que el robot físico no completara la ruta.

### Trayectorias ejecutadas

**Escenario SIMPLE (1×1 m)**
![Trayectoria SIMPLE](analysis/Simple/trajectory_analysis_SIMPLE.png)
En este gráfico se observa una alta fidelidad general entre la trayectoria odométrica (línea roja) y la ruta global planificada por A* (línea azul discontinua). Sin embargo, dada la estrechez del entorno (1×1 m) y la proximidad de los obstáculos, el robot experimenta múltiples activaciones de la capa de evasión reactiva (puntos naranjas) principalmente al tomar las curvas y evadir las esquinas. Esto demuestra que la inflación de 5 cm en la grilla es un límite estricto, lo que obliga al sistema de control local a realizar correcciones finas y retrocesos para evitar colisiones reales en los vértices, compensando las limitaciones del mapa discreto.

**Escenario COMPLEJO (2×2 m)**
![Trayectoria COMPLEJO](analysis/Complejo/trajectory_analysis_COMPLEJO.png)
Este gráfico destaca por la ausencia total de activaciones reactivas (cero puntos naranjas). Al contar con un espacio más amplio (2×2 m) y pasillos más holgados entre los cilindros y bloques, la capa deliberativa pura (control Stop-and-Turn) es suficiente para guiar al robot desde el inicio hasta la meta. La trayectoria ejecutada (roja) se superpone casi de manera perfecta a la ruta A* (azul), validando la efectividad geométrica de la heurística. Además, confirma que en misiones de duración moderada y sin estrecheces críticas, el error acumulado por la cinemática diferencial se mantiene en rangos completamente tolerables.

**Escenario MUY COMPLEJO (3×3 m)**
![Trayectoria MUY COMPLEJO](analysis/Muy_Complejo/trajectory_analysis_MUY_COMPLEJO.png)
Aquí se evidencia gráficamente el problema de la deriva odométrica (odometry drift) en misiones de larga duración (más de 7,300 pasos de simulación). En la primera mitad del recorrido, la trayectoria real (roja) sigue fielmente el plan global (azul). Sin embargo, a medida que avanza y realiza múltiples giros, el error angular se acumula, provocando una divergencia severa. Esta desincronización entre la pose estimada y el mapa real hace que el planificador "choque" virtualmente la ruta contra las paredes, lo que gatilla una cantidad masiva de activaciones de evasión por los sensores IR (alta densidad de puntos naranjas) y rutinas de escape anti-atasco. A pesar del error cinemático acumulado, la robustez de la FSM reactiva logra mantener al robot operando hasta que su odometría interna registra la llegada a la meta.

> **Leyenda de los gráficos:**  
> — Línea azul discontinua: ruta planificada por A*  
> — Línea roja continua: trayectoria real ejecutada (odometría)  
> — Puntos naranjas: activaciones del sistema de evasión reactiva  
> — ★ Verde: posición inicial | ★ Dorado: posición meta

---

## Videos de la simulación

| Escenario | Enlace |
|-----------|--------|
| SIMPLE | [Video Escenario Simple](https://drive.google.com/file/d/1pN5vyCnL4VWmhJzzlpsHVfoB3APdUmuT/view?usp=drive_link) |
| COMPLEJO | [Video Escenario Complejo](https://drive.google.com/file/d/1ZoAkilCsNLIigbdXc8-LEwCP-40yA8SJ/view?usp=drive_link) |
| MUY COMPLEJO | [Video Escenario Muy Complejo](https://drive.google.com/file/d/1nCmMZZ0z0TqugLZ9ELGUfmGIuj_97Smh/view?usp=drive_link) |

---

## Instrucciones para ejecutar la simulación

### Estructura de carpetas

```
Robotica_final/
├── controllers/
│   └── robot_controller/
│       └── robot_controller.py        ← controlador principal (A* + odometría + Kalman)
├── worlds/
│   ├── simple.wbt                     ← escenario 1×1 m
│   ├── complejo.wbt                   ← escenario 2×2 m
│   └── complejo2.wbt                  ← escenario 3×3 m
├── analysis/
│   ├── plot_comparison.py             ← script de visualización
│   ├── Simple/
│   │   ├── planned_path_SIMPLE.csv
│   │   ├── trajectory_SIMPLE.csv
│   │   └── trajectory_analysis_SIMPLE.png
│   ├── Complejo/
│   │   ├── planned_path_COMPLEJO.csv
│   │   ├── trajectory_COMPLEJO.csv
│   │   └── trajectory_analysis_COMPLEJO.png
│   └── Muy_Complejo/
│       ├── planned_path_MUY_COMPLEJO.csv
│       ├── trajectory_MUY_COMPLEJO.csv
│       └── trajectory_analysis_MUY_COMPLEJO.png
├── readme_lab_1.md
├── readme_lab_2.md
└── README.md
```

### Requisitos

- [Webots R2023b](https://cyberbotics.com/) (o versión compatible)
- Python 3.8+
- Dependencias Python:

```bash
pip install pandas matplotlib numpy
```

### Pasos

**1. Clonar el repositorio**

```bash
git clone <URL_DEL_REPOSITORIO>
cd Robotica_final
```

**2. Seleccionar el escenario**

Editar la línea 11 de `controllers/robot_controller/robot_controller.py`:

```python
# Opciones: "SIMPLE", "COMPLEJO", "MUY_COMPLEJO"
ESCENARIO_ACTUAL = "SIMPLE"
```

**3. Abrir el mundo correspondiente en Webots**

| Escenario | Archivo a abrir |
|-----------|----------------|
| SIMPLE | `worlds/simple.wbt` |
| COMPLEJO | `worlds/complejo.wbt` |
| MUY_COMPLEJO | `worlds/complejo2.wbt` |

En Webots: `File → Open World` y seleccionar el archivo `.wbt`.

**4. Ejecutar la simulación**

Presionar el botón **Play** (▶) en Webots. El controlador Python se inicia automáticamente. Los archivos CSV se generan en:

```
controllers/robot_controller/planned_path_<ESCENARIO>.csv
controllers/robot_controller/trajectory_<ESCENARIO>.csv
```

**5. Visualizar resultados**

Copiar los CSV generados por el controlador a la subcarpeta correspondiente en `analysis/`:

```bash
# Ejemplo para el escenario SIMPLE
cp controllers/robot_controller/planned_path_SIMPLE.csv analysis/Simple/
cp controllers/robot_controller/trajectory_SIMPLE.csv   analysis/Simple/
```

Luego ejecutar el script de visualización desde la raíz del repositorio:

```bash
python analysis/plot_comparison.py
```

El script detecta automáticamente el escenario del CSV más reciente y guarda la imagen en `analysis/trajectory_analysis_<ESCENARIO>.png`.

---

## Conclusiones, limitaciones y posibles mejoras

### Conclusiones

- **A* produce rutas viables y la inflación de 5 cm es suficiente para los escenarios simples.** Se planificaron 25 y 57 waypoints para SIMPLE y COMPLEJO respectivamente, completando ambas misiones. El escenario COMPLEJO es el más limpio: llega a la meta sin activar la evasión reactiva en ningún momento (0 activaciones), lo que demuestra que la ruta planificada es globalmente óptima y libre de colisiones.

- **La acumulación de error odométrico es el factor limitante en misiones largas.** En MUY_COMPLEJO el robot falló: físicamente se desplazó en una dirección incorrecta y permaneció cerca del punto de partida, mientras la odometría reportaba estar recorriendo la ruta. Las 1,890 activaciones de evasión introdujeron rotaciones sucesivas que corrompieron la estimación de ángulo sin posibilidad de corrección, haciendo que la trayectoria estimada y la real divergieran completamente.

- **La estrategia Stop-and-Turn garantiza precisión en el seguimiento de waypoints** sin un controlador PID, pero detener la traslación ante errores > 0.15 rad aumenta el tiempo de misión (7,337 pasos para 122 waypoints en MUY_COMPLEJO).

- **La capa reactiva del Lab 2 aporta robustez frente a imprevistos**, pero su acción masiva en entornos muy densos (1,890 activaciones) es también la principal fuente de deriva. En escenarios de baja densidad (COMPLEJO), el sistema funciona con 0 activaciones y alta precisión.

### Limitaciones

| Limitación | Descripción |
|-----------|-------------|
| **Deriva odométrica** | La odometría acumula error sin corrección. En MUY_COMPLEJO (≈235 s, 1,890 activaciones de evasión) la divergencia fue suficiente para que el robot no alcanzara la meta real, aunque la estimación interna indicara éxito. |
| **Mapa estático** | El mapa se construye al inicio con los obstáculos del archivo `.wbt`. Obstáculos dinámicos o no declarados no se integran al plan. |
| **Evasión reactiva simple** | El esquema retroceso-giro puede fallar en pasillos estrechos o ante obstáculos irregulares, generando bucles de atasco. |
| **Sin relocalización** | El robot no tiene forma de corregir su pose estimada; no se usan landmarks ni GPS. |
| **Mapa de celdas cuadradas** | La discretización a 5 cm introduce un error geométrico en curvas y diagonales largas. |

### Posibles mejoras

- **Replanificación dinámica (D\*):** Actualizar la ruta en tiempo real ante obstáculos no mapeados, sin recalcular desde cero.
- **Control PID de orientación:** Reemplazar el Stop-and-Turn por un controlador proporcional-integral-derivativo para trayectorias más suaves y rápidas.
- **Fusión sensorial con IMU:** Combinar la odometría con una unidad de medición inercial para reducir la deriva en misiones largas.
- **SLAM (Simultaneous Localization and Mapping):** Construir y actualizar el mapa mientras el robot navega, eliminando la dependencia del mapa estático preprogramado.
- **Campos potenciales artificiales:** Sustituir o complementar la evasión reactiva con un campo repulsivo alrededor de los obstáculos para trayectorias de evasión más fluidas.
Trabajo Futuro (Mejoras Propuestas)
Implementación de SLAM: Integrar algoritmos de Simultaneous Localization and Mapping u odometría visual para corregir la pose en tiempo real y mitigar el error acumulado.
