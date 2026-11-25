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

## Probar la visión sobre un bag del robot

`mission/scripts/check_bag.py` pasa las imágenes de un bag grabado con el limitador de la cámara (`/mission/cam/...`, `/odom`, `/tf_static`) por el detector, la medición con profundidad y el filtro de Bayes, sin navegación. Deja en `--output` un resumen, el gráfico de los conos medidos sobre el recorrido y muestras con los recuadros. Necesita ROS para leer el bag. Desde la raíz del repo, con el bag fuera del repo:

```bash
docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$PWD":/ws -v ~/ruta/bag_conos_2:/bag:ro -w /ws/mission \
  yahboom_rosmaster:latest bash -c 'source /opt/ros/humble/setup.bash && \
  PYTHONPATH=../slam:../nav:.:$PYTHONPATH python3 scripts/check_bag.py /bag --output /tmp/check && cat /tmp/check/summary.json'
```

Para guardar las salidas, cambiar `--output` a una carpeta dentro de `/ws`. Resultados de `bag_conos_2` en la [decisión 017](decisiones/017-ubicacion-y-confirmacion-del-cono.md#cámara-del-robot-bag_conos_2-2026-11-24).

## Gazebo

### Diferencias con el robot real

La cámara simulada del curso no es la Astra: 87 grados de campo de visión, 424 × 240 px, profundidad solo hasta 1,5 m y 2 imágenes por segundo. Por eso la simulación usa `mission/config/sim.yaml` (alcance 1,4 m) y el plan tiene unos 22 puntos en lugar de 10. La navegación usa `nav/config/cafe.yaml`, el perfil de simulación de la etapa B.

### Escenarios

Mundos generados por `mission/scripts/make_world.py` a partir del mapa del aula (`intento3_corregido`): cada obstáculo del mapa es una caja, así el lidar simulado ve lo mismo que vio el real. Los conos son cinco cilindros escalonados, 30 cm de alto, porque Gazebo Fortress no tiene geometría de cono.

| Escenario | Qué prueba |
|---|---|
| `aula_conos` | Un cono rojo en el pasillo central y distractores naranja, azul y amarillo. Tiene que ir al rojo e ignorar el resto |
| `aula_hueco` | Una pared extra con un hueco de 15 cm y el cono rojo detrás. Lo ve por el hueco y tiene que rodear por arriba a la izquierda |
| `aula_punto` | Un cono rojo sobre el camino entre los dos primeros puntos de observación. Tiene que verlo y aproximarse sin chocarlo. Con el cono justo sobre un punto no lo encontraba ([015](decisiones/015-maquina-de-estados-de-la-mision.md#decisiones-menores)) |
| `aula_lejos` | Un cono rojo en la esquina de arriba a la izquierda, cerca del último punto del recorrido. Tiene que recorrer casi toda el aula sin trabarse hasta encontrarlo; da el tiempo de una misión completa |

El launch del curso hace aparecer al robot siempre en el origen del mundo. El origen del mapa del aula está a 15 cm de una pared, así que los mundos están corridos: el robot aparece en **(−0,8; 0) del mapa, mirando a +x**.

### Preparar

Una vez, y cada vez que cambie el código o un escenario. Desde la raíz del repo, en el host:

```bash
# Mundos y mapas de los escenarios (quedan en mission/worlds/, no se versionan)
docker run --rm -u "$(id -u):$(id -g)" -e PYTHONDONTWRITEBYTECODE=1 -v "$PWD":/ws -w /ws/mission \
  yahboom_rosmaster:latest bash -c 'for s in aula_conos aula_hueco aula_punto aula_lejos; do
    PYTHONPATH=../slam:../nav:. python3 scripts/make_world.py \
      ../sesiones/20261116/mapa/intento3_corregido/map.yaml worlds/$s.yaml worlds/$s; done'

# Contenedor del curso y paquetes slam, nav y mission
./archivos_ros/container.sh start
./slam/scripts/sync_container.sh
```

El sincronizador tiene que terminar listando `mission_node` entre los ejecutables de `mission`.

### Correr un escenario

Tres terminales dentro del contenedor (`./archivos_ros/container.sh enter`). En cada una, `ESC=aula_conos` (o `aula_hueco`, `aula_punto`, `aula_lejos`).

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
| El navegador dice `new initial pose` pero la localización nunca dice `Initial pose received`; RViz no muestra el robot | La pose inicial se mandó antes de que la localización estuviera escuchando. Mandarla de nuevo, siempre después de ver `Localization with 1000 particles ... waiting for /initialpose` |
| `No camera pose` en el log | TF entre `map` y el frame de la imagen: `ros2 run tf2_ros tf2_echo map cam_1_depth_optical_frame` |
| Los marcadores de los conos aparecen en lugares raros, lejos del cono real | El `frame_id` de la imagen no es el óptico (ver la comprobación de arriba) |
| `/mission/debug_image` no muestra recuadros con el cono a la vista | Umbrales del detector (`detector:` en `sim.yaml`) o luz de Gazebo: traer una captura de esa imagen |
| Recuadros amarillos y nunca verdes | La profundidad no da un valor válido: el cono está a más de 1,4 m, o la profundidad no viene en el formato esperado |
| El robot no se mueve aunque la misión manda objetivos | `ros2 topic echo /nav/state`: el navegador puede estar `BLOCKED` o esperando la localización |

Para cortar: Ctrl+C en la terminal 2 y después en la 1.

## Robot real

Lo aprendido en las visitas del 2026-11-24, en la que se grabó `bag_conos_2`, y del 2026-11-25, con las dos corridas de la misión en `sesiones/20261125/`. La PC del laboratorio tiene ROS 2 Humble instalado directo y el workspace del grupo en `~/ivnafra/ws`. Los pasos de cada visita y qué traer están en [robot_real_checklist.md](robot_real_checklist.md).

### Cómo está armado el robot

- **Drivers:** arrancan solos al prender el robot, sin lanzar nada. Para verlo desde la PC alcanza con usar **su** `ROS_DOMAIN_ID`. El 16/11 fue 11; el 25/11, 12, con el robot en `172.23.16.102` y el contenedor `rosmaster_humble`. Si `ros2 node list` sale vacío con el domain que se probó, es el domain equivocado.
- **ROS en el robot:** no está instalado en el robot, solo adentro de un contenedor Docker, que se ve con `docker ps` por `ssh`. Las credenciales del `ssh` son las de la cátedra.
- **Cámara:** a 30 Hz y sin comprimir, color y profundidad ocupan unos 16 MB/s y **saturan el wifi**. Con el limitador corriendo en la PC, un bag de 9 minutos quedó con 61 imágenes y 83 scans. El limitador tiene que correr **adentro del contenedor del robot**. Aun así, con la misión a 3 Hz el `/scan` llegaba a la PC a 1,1–1,7 Hz en lugar de 7: el navegador cancelaba cada objetivo y la misión salteó los 10 puntos en 37 s. **Para la misión, 0,5 Hz**: el scan llegó a 7,2 Hz, con un hueco máximo de 0,17 s, y el cono se confirmó en unos 2 s (25/11). Para grabar un bag adentro del contenedor, sin la misión, 3 Hz no molesta: las imágenes no viajan por wifi si nadie en la PC se suscribe.
- **Profundidad:** no mide a menos de unos 0,6 m. Color 320 × 240 y profundidad `32FC1` en metros, las dos en `cam_1_color_optical_frame`.
- **Evitar en RViz** las imágenes de `/cam_1/...`: según la cátedra, un RViz remoto por wifi sube la latencia de todos los suscriptores. Mirar `/mission/debug_image`.

### 1. PC: preparar

Una vez por visita:

```bash
cd ~/ivnafra/TP_final && git fetch && git checkout feat/mission && git pull
ln -sfn ~/ivnafra/TP_final/mission ~/ivnafra/ws/src/mission     # ruta absoluta: un enlace relativo no resolvía y colcon ignoraba el paquete
cd ~/ivnafra/ws && colcon build --symlink-install --packages-select slam nav mission
```

En **cada terminal** de la PC, con el domain del robot:

```bash
source ~/ivnafra/ws/env.sh <domain>
ros2 node list               # /sllidar_node, /driver_node, /astra_sensor_adapter, ...
ros2 topic hz /scan          # unos 7 Hz
```

### 2. Robot: el limitador de la cámara

Desde la PC, copiar el paquete al robot y de ahí al contenedor:

```bash
scp -r ~/ivnafra/TP_final/mission <usuario>@<ip del robot>:~/ivnafra_mission
ssh <usuario>@<ip del robot>
docker ps                                            # nombre del contenedor, columna NAMES
docker cp ~/ivnafra_mission <contenedor>:/tmp/ivnafra_mission
docker exec -it <contenedor> bash
```

Adentro del contenedor:

```bash
source /opt/ros/*/setup.bash
export ROS_DOMAIN_ID=<domain>          # si no está definido
cd /tmp/ivnafra_mission && python3 -m mission.ros.camera_throttle_node --ros-args -p rate_hz:=0.5
```

`rate_hz:=0.5` para la misión; `3.0` solo para grabar un bag sin la misión (ver arriba). Cada 5 s escribe cuántos cuadros recibe y publica. Desde la PC, `ros2 topic hz /mission/cam/color/image_raw` tiene que dar la frecuencia elegida, y `ros2 topic hz /scan` seguir en unos 7 Hz. Queda corriendo toda la visita.

### 3. Bag de la cámara (opcional)

Para calibrar el detector en casa con `mission/scripts/check_bag.py`, con el limitador a 3 Hz y sin la misión corriendo. Se graba adentro del contenedor, así no se pierde nada por el wifi; en otra terminal, `docker exec -it <contenedor> bash` y el mismo `source`:

```bash
cd /tmp && ros2 bag record -o ivnafra_bag_<nombre> /scan /odom /tf /tf_static \
  /mission/cam/color/image_raw /mission/cam/color/camera_info \
  /mission/cam/depth/image_raw /mission/cam/depth/camera_info
```

Manejar con `ros2 run teleop_twist_keyboard teleop_twist_keyboard` desde la PC, a 0,15–0,2 m/s (bajar con `z`). Después, `ros2 bag info` del bag y sacarlo: `docker cp <contenedor>:/tmp/ivnafra_bag_<nombre> ~/` en el robot y `scp -r` a la PC. Unos 1,6 MB por segundo grabado.

### 4. Misión

Con el limitador corriendo en el contenedor y el teleop cerrado (si publica en `/cmd_vel`, compite con el navegador). En la PC:

```bash
ros2 launch mission mission.launch.py \
  map_file:=$HOME/ivnafra/TP_final/sesiones/20261116/mapa/intento3_corregido/map.yaml \
  color_topic:=/mission/cam/color/image_raw depth_topic:=/mission/cam/depth/image_raw \
  camera_info_topic:=/mission/cam/color/camera_info camera_rate:=0.5 \
  2>&1 | tee /tmp/mision_<corrida>.log
```

Usa `mission/config/real.yaml` y `nav/config/real.yaml`. Un nombre de log por corrida: con el mismo nombre, la siguiente pisa la anterior. En RViz: **2D Pose Estimate** con la pose real del robot (el 25/11 el primer clic se rechazó en las dos corridas: mandarlo de nuevo), y agregar `/mission/markers` y `/mission/debug_image`. En otra terminal, `ros2 topic echo /mission/state --field data | tee /tmp/estado_<corrida>.log`.

Tiene que pasar lo mismo que en Gazebo (ver [Qué tiene que pasar](#qué-tiene-que-pasar)), con el robot a 0,45 m del cono al final. El 25/11: 130 s y 154 s de la pose inicial a `DONE`. Tener a mano Ctrl+C en la terminal de la misión para frenarlo.

Para el video, repetir una corrida que haya salido bien filmando con el celular de principio a fin, sin cortes y con un cono de otro color a la vista.

### 5. Antes de irse

Cortar el limitador y borrar lo copiado. Adentro del contenedor:

```bash
rm -rf /tmp/ivnafra_mission /tmp/ivnafra_bag_*
```

En el robot, fuera del contenedor: `rm -rf ~/ivnafra_mission ~/ivnafra_bag_*`. Copiar al pendrive los bags, los `/tmp/mision_*.log` y `/tmp/estado_*.log`, capturas y el video. Si se tocó algo en la PC del laboratorio (por ejemplo `nav/config/real.yaml`), guardar el `git diff` y descartarlo con `git checkout -- <archivo>`. Apagar el robot con `sudo shutdown -h now` por `ssh` y esperar a que el lidar deje de girar.

### Si algo no anda en el robot

| Síntoma | Qué revisar |
|---|---|
| `ros2 node list` vacío desde la PC, el robot responde al ping | Domain equivocado: probar con el del robot en cada terminal y `ros2 daemon stop; ros2 daemon start` |
| El teleop publica (`ros2 topic echo /cmd_vel` muestra valores) y el robot no se mueve | Interruptor de los motores, batería o ruedas en el aire. Probar con `ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}}"` |
| `/mission/cam/...` a mucho menos de lo pedido, o `/scan` con huecos | El limitador está corriendo en la PC y no en el contenedor, o alguien mira `/cam_1/...` en RViz |
| La misión saltea los puntos uno tras otro y el navegador cancela los objetivos | `/scan` llega lento a la PC (`ros2 topic hz /scan`, tiene que dar unos 7 Hz): bajar el limitador a 0,5 Hz |
| `Click rejected: 45 % of the scan is far from every mapped wall` | Volver a marcar la pose; el 25/11 el segundo clic convergió en 2 iteraciones |
| colcon: `ignoring unknown package mission` | El enlace `ws/src/mission` no resuelve: rehacerlo con la ruta absoluta y la rama `feat/mission` |
| Los nodos del robot desaparecen a mitad de la visita | El 25/11 la cámara se volvió a enumerar por USB y se cayeron los drivers; reiniciar el robot. Causa sin confirmar |
| Recuadros amarillos con el cono a menos de 0,6 m | Normal: la cámara no mide tan cerca. El cono ya se ubicó desde más lejos |
| Sin recuadro con el cono rojo a la vista | Umbrales del detector en `real.yaml` (`sat_min`, `val_min`): traer una captura de `/mission/debug_image` o un bag |
