# Guía de ejecución de la etapa C

Paquete `mission/`: exploración del aula, detección de conos rojos y aproximación al cono, sobre el mapa de la etapa A y la navegación de la etapa B. Decisiones en [014](decisiones/014-exploracion-por-puntos-de-observacion.md) a [017](decisiones/017-ubicacion-y-confirmacion-del-cono.md).

## Sin ROS

Desde `mission/`:

```bash
PYTHONPATH=../slam:../nav:. python3 -m unittest discover -s tests
```

Necesita NumPy, SciPy, PyYAML y OpenCV (`requirements.txt`). Con la imagen del curso, que ya los tiene, desde la raíz del repo:

```bash
docker run --rm -e PYTHONDONTWRITEBYTECODE=1 -v "$PWD":/ws:ro -w /ws/mission yahboom_rosmaster:latest \
  bash -c 'PYTHONPATH=../slam:../nav:. python3 -m unittest discover -s tests'
```

## Gazebo

### Diferencias con el robot real

La cámara simulada del curso no es la Astra: 87 grados de campo de visión, 424 × 240 px, profundidad solo hasta 1,5 m y 2 imágenes por segundo. Por eso la simulación usa `mission/config/sim.yaml` (alcance 1,4 m) y el plan tiene unos 22 puntos en lugar de 10. La navegación usa `nav/config/cafe.yaml`, el perfil de simulación de la etapa B.

### Escenarios

Mundos generados por `mission/scripts/make_world.py` a partir del mapa del aula (`intento3_corregido`): cada obstáculo del mapa es una caja, así el lidar simulado ve lo mismo que vio el real. Los conos son cinco cilindros escalonados, 30 cm de alto, porque Gazebo Fortress no tiene geometría de cono.

| Escenario | Qué prueba |
|---|---|
| `aula_conos` | Un cono rojo en el pasillo central y distractores naranja, azul y amarillo. Tiene que ir al rojo e ignorar el resto |
| `aula_hueco` | Una pared extra con un hueco de 15 cm y el cono rojo detrás. Lo ve por el hueco y tiene que rodear por arriba a la izquierda |
| `aula_punto` | Un cono rojo justo sobre un punto de observación. Tiene que verlo en el camino y frenar antes, sin chocarlo |

El launch del curso hace aparecer al robot siempre en el origen del mundo. El origen del mapa del aula está a 15 cm de una pared, así que los mundos están corridos: el robot aparece en **(−0,8; 0) del mapa, mirando a +x**.

### Preparar

Una vez, y cada vez que cambie el código o un escenario. Desde la raíz del repo, en el host:

```bash
# Mundos y mapas de los escenarios (quedan en mission/worlds/, no se versionan)
docker run --rm -u "$(id -u):$(id -g)" -e PYTHONDONTWRITEBYTECODE=1 -v "$PWD":/ws -w /ws/mission \
  yahboom_rosmaster:latest bash -c 'for s in aula_conos aula_hueco aula_punto; do
    PYTHONPATH=../slam:../nav:. python3 scripts/make_world.py \
      ../sesiones/20261116/mapa/intento3_corregido/map.yaml worlds/$s.yaml worlds/$s; done'

# Contenedor del curso y paquetes slam, nav y mission
./archivos_ros/container.sh start
./slam/scripts/sync_container.sh
```

El sincronizador tiene que terminar listando `mission_node` entre los ejecutables de `mission`.

### Correr un escenario

Tres terminales dentro del contenedor (`./archivos_ros/container.sh enter`). En cada una, `ESC=aula_conos` (o `aula_hueco`, `aula_punto`).

**Terminal 1, Gazebo:**

```bash
ros2 launch yahboom_rosmaster_gazebo rosmaster_gazebo_fortress.launch.py \
  world:=$HOME/rosmaster_ws/src/mission/worlds/$ESC.sdf rviz:=false motion_profile:=ideal motion_bias:=false
```

Antes de seguir, en otra terminal:

- `ros2 topic hz /cam_1/color/image_raw` tiene que dar unos 2 Hz. Si no publica, la cámara no se está renderizando.
- `ros2 topic echo /cam_1/color/image_raw --once --field header` tiene que mostrar un `frame_id` terminado en `_optical_frame` (en el URDF del curso, `cam_1_depth_optical_frame`). La misión supone el frame óptico: z hacia adelante, x a la derecha, y hacia abajo. Si muestra otro frame, los conos quedan mal ubicados en el mapa: avisar antes de seguir.

**Terminal 2, navegación de la etapa B, misión y RViz:**

```bash
ros2 launch mission mission.launch.py use_sim_time:=true \
  map_file:=$HOME/rosmaster_ws/src/mission/worlds/${ESC}_map.yaml \
  config_file:=$(ros2 pkg prefix mission)/share/mission/config/sim.yaml \
  nav_config_file:=$(ros2 pkg prefix nav)/share/nav/config/cafe.yaml \
  2>&1 | tee /tmp/mision_$ESC.log
```

**Terminal 3, pose inicial y estado:**

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -0.8, y: 0.0}, orientation: {w: 1.0}}}}"
ros2 topic echo /mission/state --field data | tee /tmp/estado_$ESC.log
```

También se puede marcar la pose con **2D Pose Estimate** en RViz: en (−0,8; 0), mirando a +x.

En RViz, agregar con **Add → By topic**: `/mission/markers` (puntos de observación en azul; conos candidatos en rojo, más opacos cuanto más probables, y verdes al confirmarse) y `/mission/debug_image` (la imagen con las detecciones: verde si se ubicó en el mapa, amarillo si no).

### Qué tiene que pasar

1. Con la pose inicial, el log dice `Exploration plan: {...}` y el robot va al primer punto.
2. En cada punto gira hacia lo que falta ver (`LOOK_AROUND`).
3. Al ver algo rojo: `red candidate at (...), stopping to look`, frena y gira hacia el candidato.
4. Al confirmarse: `cone confirmed at (...)` y va a pararse a 0,45 m del cono, mirándolo.
5. Al llegar: `reached the cone at (...)` y estado `DONE`.

Si nunca lo encuentra termina en `EXHAUSTED`.

### Qué traer de cada corrida

- `/tmp/mision_$ESC.log` y `/tmp/estado_$ESC.log`, copiados al host con `docker cp yahboom_rosmaster:/tmp/mision_$ESC.log .`.
- Capturas de RViz y de Gazebo al encontrar el cono y al terminar.
- El tiempo desde la pose inicial hasta `DONE` o `EXHAUSTED`.
- Dónde quedó el robot respecto del cono: `ros2 topic echo /ground_truth/odom --once` da la pose real en el frame del mundo. En el mapa es la del mundo más (−0,8; 0).

### Si algo no anda

| Síntoma | Qué revisar |
|---|---|
| Al entrar al contenedor: `Permission denied` en `/tmp/rosmaster-x11-...` y `no X cookie found`; Gazebo y RViz no abren ventanas | Después de reiniciar la PC, Docker vuelve a crear esa carpeta como `root`. En el host: `sudo chown "$USER:$USER" /tmp/rosmaster-x11-$(id -u)-yahboom_rosmaster` y volver a entrar. Para evitarlo: `mkdir -p` de esa carpeta antes de `container.sh start` |
| El log de la misión dice `waiting for localization` y no avanza | Falta la pose inicial, o la localización no convergió: `ros2 topic echo /nav/state --once` |
| `No camera pose` en el log | TF entre `map` y el frame de la imagen: `ros2 run tf2_ros tf2_echo map cam_1_depth_optical_frame` |
| Los marcadores de los conos aparecen en lugares raros, lejos del cono real | El `frame_id` de la imagen no es el óptico (ver la comprobación de arriba) |
| `/mission/debug_image` no muestra recuadros con el cono a la vista | Umbrales del detector (`detector:` en `sim.yaml`) o luz de Gazebo: traer una captura de esa imagen |
| Recuadros amarillos y nunca verdes | La profundidad no da un valor válido: el cono está a más de 1,4 m, o la profundidad no viene en el formato esperado |
| El robot no se mueve aunque la misión manda objetivos | `ros2 topic echo /nav/state`: el navegador puede estar `BLOCKED` o esperando la localización |

Para cortar: Ctrl+C en la terminal 2 y después en la 1.
