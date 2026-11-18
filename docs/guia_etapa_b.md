# Guía de ejecución de la etapa B

Paquete `nav/`: localización sobre un mapa conocido y navegación a un objetivo. Las decisiones y sus datos están en [decisiones/](decisiones/) (004 a 012). El plan de las visitas al laboratorio está en [robot_real_checklist.md](robot_real_checklist.md).

Todos los comandos van desde la carpeta `nav/`.

## Qué hay

| Pieza | Archivo | Qué hace |
|---|---|---|
| Localización | `nav/core/localization.py` | Filtro de partículas sobre el mapa |
| Planificador | `nav/core/planner.py` | Mapa de costos y A* |
| Seguidor | `nav/core/follower.py` | Pure pursuit y giros en el lugar |
| Máquina de estados | `nav/core/navigator.py` | Decide qué hace el robot |
| Nodos ROS | `nav/ros/localization_node.py`, `nav/ros/navigator_node.py` | Conectan lo anterior con ROS |
| Launch | `launch/nav.launch.py` | Lanza los dos nodos y RViz |
| Parámetros | `config/cafe.yaml` (simulador), `config/real.yaml` (robot) | Todos los valores, con comentarios |

Obstáculos que no están en el mapa ([008](decisiones/008-obstaculos-no-mapeados.md)): el lidar los anota en una capa aparte (`nav/core/obstacles.py`). Si uno tapa el camino, el robot frena y planifica de nuevo. Si no hay paso, espera hasta 15 s y abandona el objetivo.

## Sin ROS

Tests (importan `slam.core`, por eso el `PYTHONPATH`):

```bash
PYTHONPATH=../slam:. python3 -m unittest discover -s tests
```

Con `uv`, sin instalar nada: `PYTHONPATH=../slam:. uv run --no-project --with-requirements requirements.txt python -m unittest discover -s tests`.

## Con ROS

Enlazar `slam/` y `nav/` dentro de `src/` del workspace y compilar los dos:

```bash
colcon build --symlink-install --packages-select slam nav
source install/setup.bash
```

En el contenedor del RosMaster, `slam/scripts/sync_container.sh` copia y compila los dos paquetes. La parte de `nav` de ese script no está probada con Docker.

### Robot real

```bash
ros2 launch nav nav.launch.py map_file:=/ruta/al/mapa.yaml config_file:=$PWD/config/real.yaml \
  base_frame:=base_footprint odom_topic:=/odom
```

`base_footprint` es el frame de la base del robot del laboratorio, medido el 2026-11-16. Después, en RViz: marcar la pose con **2D Pose Estimate** y el objetivo con **2D Goal Pose**.

| Argumento | Para qué |
|---|---|
| `map_file` | Mapa PGM + YAML, el que guarda el SLAM de la etapa A |
| `config_file` | Parámetros |
| `base_frame` | Frame de la base. Tiene que ser el `child_frame_id` de la odometría |
| `odom_topic` | Tópico de odometría |
| `odom_child_frame` | Solo si la odometría trae otro `child_frame_id` y su pose igual es la de la base |
| `navigate:=false` | Lanza solo la localización. El robot se maneja con teleop |
| `publish_tf`, `publish_map` | En `true` por defecto: el nodo publica `map → odom` y `/map`. No tener el SLAM corriendo a la vez, publica lo mismo |
| `rviz:=false` | Sin RViz |

Si el `child_frame_id` de la odometría no coincide, el nodo lo dice en el log y no estima.

### Simulador en macOS (café, Gazebo Classic)

Una terminal por comando, desde la raíz del workspace de simulación, cada una con `pixi run -e classic bash` y `source install_classic/setup.bash`:

```bash
ros2 launch custom_map cafe.launch.py rviz:=False            # gui:=false sin ventana
ros2 launch nav nav.launch.py map_file:=$PWD/custom_map/maps/my_map.yaml \
  config_file:=$PWD/nav/config/cafe.yaml odom_topic:=/calc_odom odom_child_frame:=calc_base \
  publish_tf:=false publish_map:=false use_sim_time:=true
```

`/calc_odom` es la odometría con deriva y sale con `child_frame_id: calc_base`. `publish_tf` y `publish_map` van en `false` porque el launch del café ya publica un `map → odom` fijo y el mapa. Al terminar, apagar todo con `./run stop`.

Prueba automática contra la pose real del simulador, con los nodos ya lanzados:

```bash
python nav/scripts/sim_live_check.py --goal 3.1 -2.4 -1.5
```

Publica un `/initialpose` corrido 36 cm, manda el objetivo e imprime el error real al llegar.

### Parada de seguridad por perfil

`navigator.stop_distance` se mide desde el centro de la base. Café usa **0,40 m** y `sensor.min_range=0.25` m: con el láser adelantado 0,043 m, el retorno frontal debe poder activar la parada antes de entrar en el alcance descartado por el chasis. El perfil real usa 0,25 m y mínimo de 0,12 m; falta verificar su huella física. Ambos usan semiancho de 0,20 m. La parada actúa al avanzar y no espera la confirmación de la capa de obstáculos.

En Café, elegir objetivos con espacio delante del robot; los objetivos muy próximos a una pared pueden terminar en `BLOCKED`. La parada conserva una zona ciega bajo el mínimo del sensor: comenzar con el frente despejado. Ver [decisión 008](decisiones/008-obstaculos-no-mapeados.md#corrección-del-perfil-café-en-la-revisión-del-pr-4). La corrección del perfil se verificó con tests; todavía no se repitió en ROS/Gazebo.

## Qué mirar

| Tópico | Contenido |
|---|---|
| `/nav/pose` | Pose del robot en el mapa |
| `/nav/particles` | Nube de partículas |
| `/nav/localization_status` | JSON: `state` (`WAITING_INITIAL_POSE`, `NOT_CONVERGED`, `LOCALIZED`, `LOST`), dispersión, `update_seconds`, `scan_score`, `unexplained_fraction` |

`LOST` aparece cuando `unexplained_fraction` (fracción de los puntos del scan a más de 20 cm de toda pared del mapa) supera 0,25: la pose no explica lo que ve el lidar. Hay que marcar la pose de nuevo. Justo después del click el límite es 0,15: si lo supera, el log dice `Click rejected` y el nodo queda en `LOST` hasta el próximo click ([004](decisiones/004-localizacion-en-mapa-conocido.md#detección-de-perdido)).

| `/nav/plan` | Camino planificado |
| `/nav/state` | JSON: estado de la máquina, si la parada de seguridad está activa y los últimos eventos |

Para guardar el estado durante una corrida: `ros2 topic echo /nav/localization_status > loc_status.log` y `ros2 topic echo /nav/state > nav_state.log`.

## Analizar un bag

Los scripts están en `scripts/`. El primero necesita ROS, el resto solo Python.

```bash
# 1. Bag a .npz (entorno con ROS)
python scripts/bag_to_npz.py bag_aula bag_aula.npz --base-frame base_footprint

# 2. Correr la localización sobre la grabación
PYTHONPATH=../slam:. python3 scripts/replay_localization.py --input bag_aula.npz \
  --map /ruta/al/mapa.yaml --config config/real.yaml --output loc.json --initial-pose 0 0 0

# 3. Repetibilidad sobre las marcas del piso
python3 scripts/mark_repeatability.py bag_aula.npz loc.json

# 4. Comparar con la trayectoria del SLAM sobre el mismo bag
python3 scripts/compare_trajectories.py loc.json /ruta/al/samples.json
```

`replay_localization.py` acepta `--seed`, `--set grupo.campo=valor` para probar un parámetro sin editar el YAML, y `--initial-offset` para simular un click con error. Si la grabación es de simulación y trae la pose real, informa el error contra ella. `chairs_sim.py` arma la prueba sintética de sillas movidas ([011](decisiones/011-entorno-que-cambia.md)).

Probado con el bag real del Parcial: 1188 scans extraídos, 839 actualizaciones, y tres semillas que coinciden entre sí a 1,4 cm.
