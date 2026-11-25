# Guía de ejecución del SLAM

Todos los comandos se corren desde la carpeta `slam/` del repositorio, que es el paquete ROS. Los parámetros están en [parametros.md](parametros.md) y el alcance de lo validado en [validacion.md](validacion.md).

La [revisión del 16/11/2026](revision_20261116.md) compara los cambios de Iván e incluye recetas específicas para el bag del Parcial y Café Fortress.

Desde la corrección de deriva con el robot quieto ([012](decisiones/012-integracion-con-el-robot-quieto.md)), el SLAM integra solo por desplazamiento o giro. Antes de usar un YAML externo o archivado, quitar `update.max_interval`: el cargador lo rechaza. Los perfiles instalados, incluidos `cafe_compare_*.yaml`, ya están migrados. Resincronizar y construir antes de lanzar la versión nueva.

## 1. Prueba reproducible sin ROS

Desde `~/TP_final/slam`:

```bash
python3 -m pip install -r requirements.txt
python3 -m slam.toy.cli toy --output artifacts/toy --seed 7 --check
python3 -m unittest discover -s tests -v
```

En una instalación Ubuntu con paquetes del sistema se pueden obtener las dependencias con `sudo apt install python3-numpy python3-scipy python3-yaml`. El ejemplo usa un entorno de oficina de 9 × 7 m, con una partición interior, una puerta y una trayectoria cerrada que recorre ambos lados. Simula ruido de rango, odometría imperfecta y una referencia geométrica independiente de las partículas. Los scans son instantáneos: el paso de traslación de 0,28 m y el período sintético de 0,25 s definen el muestreo del benchmark; no constituyen un ensayo físico de barrido a 1,12 m/s.

La salida en `artifacts/toy` contiene:

| Archivo | Contenido |
| --- | --- |
| `map.pgm`, `map.yaml` | Mapa elegido, listo para cargar con un map server. |
| `ground_truth.pgm`, `ground_truth.yaml` | Geometría de referencia para comparar. |
| `trajectory.csv` | Poses de referencia, odometría y estimación. |
| `overview.svg` | Comparación visual del mapa y la trayectoria. |
| `metrics.json` | Precisión/recall de obstáculos, errores de pose, cobertura, conectividad y costo medido. |

`--check` comprueba los criterios de aceptación sintéticos implementados por el ejemplo. Aprobarlos verifica ese escenario; la aceptación de la entrega requiere además el entorno de Gazebo, el rosbag y el mapa del robot físico.

Para explorar otra resolución o cantidad de partículas:

```bash
python3 -m slam.toy.cli toy --output artifacts/toy_coarse --seed 7 --resolution 0.10
python3 -m slam.toy.cli toy --output artifacts/toy_dense --seed 7 --particles 48
```

Los argumentos específicos del generador incluyen `--sensor-noise`, `--odom-translation-noise`, `--odom-rotation-noise`, `--odom-scale`, `--beams` y `--scan-step`. Modifican los datos sintéticos; los coeficientes del filtro se configuran por separado. Consultar los valores y unidades con:

```bash
python3 -m slam.toy.cli toy --help
```

## 2. Instalar el paquete ROS

### Usar el contenedor habitual del RosMaster

El contenedor `yahboom_rosmaster` que se abre con `~/PRA_00/container.sh enter` usa el usuario `alumno` y el workspace `/home/alumno/rosmaster_ws`. Su montaje actual incluye los paquetes de `PRA_00`, pero no la carpeta `~/TP_final` del host. Para incorporar este paquete, ejecutar **en el host**:

```bash
cd ~/TP_final
./slam/scripts/sync_container.sh
```

El script copia los archivos del paquete a `/home/alumno/rosmaster_ws/src/slam` y construye `slam` y `nav` con colcon. Conserva los otros paquetes y el contenedor existente. El 10 de noviembre de 2026 se verificó la construcción en este contenedor habitual: colcon terminó correctamente, ROS encontró el paquete en `/home/alumno/rosmaster_ws/install/slam` y enumeró sus cuatro ejecutables. La apertura de nodos, RViz y Gazebo desde esta sesión interactiva queda a cargo del usuario; esta comprobación no los lanzó.

### Entorno virtual y Numba en el contenedor habitual

Para sincronizar, construir y preparar Numba en una sola operación, **en el host**:

```bash
cd ~/TP_final
./slam/scripts/sync_container.sh --with-numba
./slam/scripts/container_env.sh status
~/PRA_00/container.sh enter
```

Si los paquetes ya están sincronizados, basta con
`./slam/scripts/container_env.sh setup`. La preparación se puede repetir:
reutiliza el entorno administrado y no duplica la entrada en `.bashrc`.
Requiere el contenedor existente encendido, Python 3.10 y pip en el host, la misma
arquitectura en host/contenedor y acceso del host al índice de paquetes. Descarga
ruedas binarias de pip 24.0, Numba 0.57.1 y llvmlite 0.40.1; las instala sin red
dentro de `~/rosmaster_ws/.venv`. No requiere pip global ni `ensurepip` en el
contenedor y no modifica la imagen Docker.

El entorno usa [`venv --system-site-packages`](https://docs.python.org/3.10/library/venv.html)
para reutilizar NumPy, SciPy, PyYAML y las dependencias de ROS del sistema. Las
restricciones de NumPy/Numba son las de
[`requirements-acceleration-humble.txt`](../slam/requirements-acceleration-humble.txt)
y la [tabla oficial de compatibilidad](https://numba.readthedocs.io/en/stable/user/installing.html#version-support-information).
Si faltan dependencias base compatibles, la instalación falla sin sustituirlas
en el Python global.

Las terminales nuevas abiertas con el helper habitual activan el entorno
automáticamente. También se puede usar `./slam/scripts/container_env.sh enter`
en el host: delega la entrada al helper de `PRA_00`, que conserva la gestión de X11.
En una terminal que ya estaba abierta, cargarlo una vez:

```bash
source ~/rosmaster_ws/.tp_env/scripts/tp_env.bash
```

**Dentro del contenedor**:

```bash
tp-env status          # Python, versiones, backend y caché; no arranca nodos
tp-env off             # Salir del venv y restaurar sus variables anteriores
tp-env on              # Cargar ROS/overlay y activar Numba
tp-env numpy           # Mantener el venv y forzar NumPy en próximos procesos
tp-env on              # Volver a Numba en próximos procesos
python3 -m pip list     # Consultar paquetes visibles del entorno
```

`tp-env on` debe preceder a cualquier comando `ros2 launch` de esta guía; la
activación automática lo hace en las terminales nuevas. Funciona con los cuatro
modos del launcher. Los cambios afectan procesos futuros: detener y relanzar
un SLAM existente para cambiar su backend. Usar `tp-env off` para salir en lugar
de `deactivate`, porque también restaura `PYTHONPATH` y las variables de Numba.
Al activar se cargan ROS y el overlay; al salir permanecen disponibles.

Los ejecutables creados por colcon usan `/usr/bin/python3`. Por eso el helper
añade el `site-packages` del venv a `PYTHONPATH`, además de activar su Python:
también los nodos lanzados por ROS encuentran Numba. La caché JIT se guarda en
`~/rosmaster_ws/.cache/numba`. El archivo `COLCON_IGNORE` evita que colcon explore
el venv. La preparación sólo importa dependencias: no ejecuta nodos, replays,
RViz ni Gazebo. Tanto el entorno como la caché están dentro del contenedor y se
pierden si se elimina; pueden recrearse con `setup`.

### Abrir una sesión y ejecutar

Después entrar como siempre:

```bash
~/PRA_00/container.sh enter
```

Y ejecutar **dentro del contenedor**:

```bash
cd ~/rosmaster_ws
source install/setup.bash
ros2 launch slam slam.launch.py mode:=toy rviz:=true output_path:=/home/alumno/rosmaster_ws/maps/slam_toy
```

El modo toy publica los sensores sintéticos y abre RViz. Al terminar el recorrido se puede guardar desde otra terminal del mismo contenedor, cargando el overlay:

```bash
source ~/rosmaster_ws/install/setup.bash
ros2 service call /slam/save_map std_srvs/srv/Trigger '{}'
```

Los archivos `slam_toy.pgm`, `.yaml` y `.json` quedan en `/home/alumno/rosmaster_ws/maps`, dentro del contenedor. Esa carpeta no está montada en `~/TP_final`: los resultados no se copian automáticamente al host y se perderían al eliminar el contenedor.

Como alternativa, detener primero el launcher toy con **Ctrl+C** y ejecutar Gazebo desde ese mismo entorno:

```bash
ros2 launch slam slam.launch.py mode:=simulation rviz:=true output_path:=/home/alumno/rosmaster_ws/maps/slam_simulation
```

No ejecutar ambos launchers simultáneamente: comparten tópicos, reloj y TF. En simulación se conduce el robot con el teleop del RosMaster para explorar el mundo, como se detalla en la sección 4.

La copia del paquete no es un montaje de los fuentes. Tras editar código, YAML o launchfiles en `~/TP_final`, detener el launcher y volver a ejecutar `./slam/scripts/sync_container.sh` en el host. Cargar otra vez `~/rosmaster_ws/install/setup.bash` antes de relanzar. Si sólo se quiere actualizar la copia sin construir todavía:

```bash
cd ~/TP_final
./slam/scripts/sync_container.sh --sync-only
```

En ese caso, antes de ejecutar ROS completar la construcción dentro del contenedor:

```bash
cd ~/rosmaster_ws
colcon build --symlink-install --packages-select slam nav
source install/setup.bash
```

### Instalar en otro entorno con ROS 2 Humble

En Ubuntu 22.04, con ROS 2 Humble instalado y su entorno cargado:

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-numpy python3-yaml ros-humble-rviz2 ros-humble-rosbag2
```

El repositorio contiene dos paquetes `ament_python`, `slam/` y `nav/`. Pueden enlazarse en `src` de un workspace. Por ejemplo, estando en su raíz:

```bash
TP_SLAM_SOURCE="$(pwd)"
mkdir -p ~/slam_ws/src
ln -s "$TP_SLAM_SOURCE" ~/slam_ws/src/slam
ln -s "$TP_SLAM_SOURCE/../nav" ~/slam_ws/src/nav
cd ~/slam_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select slam nav
source install/setup.bash
```

Si el enlace ya existe, se omite `ln -s`. Cada terminal nueva debe cargar `/opt/ros/humble/setup.bash` y `~/slam_ws/install/setup.bash`. El modo `simulation` puede requerir además el workspace de los drivers y la simulación del RosMaster; el paquete de SLAM no necesita ese backend para procesar datos existentes.

## 3. Contrato de entrada

Se necesita un tópico `sensor_msgs/msg/LaserScan`, una pose odométrica temporalmente compatible y la extrínseca entre la base y el frame del láser. El valor de `base_frame` por defecto es `base_footprint`, consistente con el RosMaster simulado; se puede seleccionar `base_link` si ése es el marco de la base en los datos. Los nombres reales se inspeccionan antes de lanzar:

```bash
ros2 topic list -t
ros2 topic info /scan --verbose
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo base_footprint laser
```

Cambiar `base_footprint` y `laser` por los frames efectivos. La cadena esperada es:

```text
map --(slam)--> odom --(base / simulación / bag)--> base_footprint --(URDF o estática calibrada)--> laser
```

`odom_source:=topic` consume `/odom` de tipo `nav_msgs/msg/Odometry` e interpola la pose al timestamp de la lectura. `odom_source:=tf` obtiene `odom → base` directamente de TF. Para mensajes de odometría se verifica el marco padre y el `child_frame_id`. Si este último describe un punto distinto de la base, debe existir la transformación de montaje correspondiente o configurarse explícitamente la convención correcta. Cambiar sólo el nombre de un tópico no convierte una pose de `calc_base` a `base_link`.

La extrínseca se consulta por TF; no se supone silenciosamente que el láser coincide con el centro del robot. Si el robot o el bag ya publica su árbol, mantener `publish_scan_tf:=false` y `publish_odom_tf:=false`. Estos publicadores auxiliares se habilitan únicamente para completar una transformación ausente y conocida.

El frame `map` se fija a la pose inicial configurada, por defecto `(0,0,0)`. El primer mensaje de odometría sirve de referencia para los incrementos; sus coordenadas absolutas no deben confundirse con la pose inicial del mapa. La grilla no se carga como mapa conocido al iniciar SLAM: se construye desde desconocido.

## 4. Launcher único

El punto de entrada es:

```bash
ros2 launch slam slam.launch.py mode:=toy
```

Los modos seleccionan la fuente de lecturas y el reloj. La configuración del algoritmo se comparte mediante `config_file`.

Para consultar todos los argumentos del launcher:

```bash
ros2 launch slam slam.launch.py --show-args
```

| Argumento | Default | Uso |
| --- | --- | --- |
| `mode` | `toy` | `toy`, `simulation`, `bag` o `real`. |
| `config_file` | `config/default.yaml` instalado | YAML con hiperparámetros del núcleo. |
| `scan_topic` | `/scan` | Tópico `LaserScan`. |
| `odom_topic` | `/odom` | Tópico `Odometry`, usado con `odom_source:=topic`. |
| `odom_source` | `topic` | `topic` con interpolación, o `tf`. |
| `map_frame` | `map` | Marco del mapa creado. |
| `odom_frame` | `odom` | Marco odométrico. |
| `base_frame` | `base_footprint` | Marco planar de la base. |
| `output_path` | `./maps/slam` | Prefijo de archivos; conviene usar ruta absoluta. |
| `rviz` | `true` | Abre la configuración RViz incluida. |
| `ground_truth_topic` | `auto` | `/toy/ground_truth` (`PoseStamped`) en toy, `/ground_truth/odom` (`Odometry`) en simulation; deshabilitado en bag/real. Vacío deshabilita; otro tópico debe respetar el tipo del modo. |
| `use_sim_time` | `auto` | `true` para toy/simulation/bag, `false` para real; puede fijarse explícitamente. |
| `world` | `worlds/office.sdf` instalado | Mundo de Gazebo, usado si inicia el simulador. |
| `start_simulator` | `true` | Sólo actúa en `simulation`; poner `false` para conectarse a uno existente. |
| `headless` | `false` | Ejecutar Gazebo sin interfaz gráfica. |
| `bag_path` | vacío | Ruta de un bag ROS 2, obligatoria en modo bag. |
| `bag_rate` | `0.5` | Factor de velocidad del player, positivo. |
| `toy_rate` | `10.0` | Lecturas sintéticas publicadas por segundo de pared; no altera los timestamps de su trayectoria. |
| `publish_odom_tf` | `false` | Publicar `odom → base` desde la odometría de tópico; usar sólo si falta ese enlace. |
| `publish_scan_tf` | `false` | Publicar una extrínseca láser estática explícita; no usar con el toy ni duplicar URDF. |
| `laser_frame` | `laser_frame` | Frame del sensor para el toy o la extrínseca auxiliar. |
| `laser_x`, `laser_y`, `laser_z` | `0.0` | Traslación del montaje respecto de la base, en m; reemplazar por calibración real si se habilita. |
| `laser_yaw` | `0.0` | Orientación del montaje en radianes, con roll/pitch cero. |
| `publish_period` | `1.0` | Intervalo mínimo entre publicaciones del mapa, en segundos de datos. |
| `tf_wait_timeout` | `2.0` | Espera máxima por TF/odometría para un scan, en segundos de pared. |
| `odom_history_seconds` | `30.0` | Ventana de historial de odometría y TF, en segundos. |
| `scan_queue_size` | `30` | Máximo de scans pendientes; al desbordar se descarta el más antiguo. |
| `max_path_poses` | `5000` | Cantidad máxima de poses publicadas para visualizar trayectoria. |

Los frames se escriben sin `/` inicial. Las extrínsecas auxiliares de este launcher suponen un sensor horizontal; un montaje con roll/pitch requiere describir correctamente TF y revisar la validez de la proyección planar.

### Recorrer la vista y comparar trayectorias en RViz

La configuración incluye las herramientas **Move Camera**, **Focus Camera**, **Select**, **Measure** y el panel **Views**, con vista `TopDownOrtho` y marco fijo `map`. Con Move Camera seleccionado: rueda para zoom, botón central o **Shift + arrastrar con el izquierdo** para desplazar la vista; arrastrar con el izquierdo rota el plano. En Views se pueden ajustar `Scale`, `X`, `Y` y `Angle`. En una sesión con la configuración anterior, el botón **+** de la barra superior permite agregar **Move Camera**; Views se agrega desde **Panels → Add New Panel**.

| Color | Tópico | Contenido |
| --- | --- | --- |
| Cian | `/path`, `/slam_pose` | Trayectoria ancestral de la hipótesis elegida y su pose actual con covarianza. |
| Naranja | `/slam/odom_path` | Odometría cruda de la base, evaluada en cada scan integrado. Funciona con fuente topic y TF. |
| Verde | `/slam/ground_truth_path` | Referencia del toy o Gazebo interpolada al timestamp de esos mismos scans. |

Las referencias se alinean **una sola vez**, en el primer scan integrado: `T_map_source = T_initial · inverse(T_source_base(t0))`. La alineación no usa las correcciones posteriores `map → odom`, por lo que conserva la deriva. Esta comparación mide error relativo desde el inicio, no error absoluto de la pose inicial. El ground truth sólo alimenta visualización; no entra en el filtro. Para Odometry se exige TF del child a la base si difieren; en Fortress el plugin instalado declara `world` y `base_footprint`.

Iniciar la fuente de ground truth antes del mapeo y mantener el robot quieto al arrancar. Si faltan muestras que encierren `t0`, la línea verde queda vacía y se avisa; no se extrapola ni se inventa ground truth. En bag/real queda vacía por defecto. Las tres trayectorias se borran con `/slam/reset`; las referencias guardan como máximo `max_path_poses` (5000) y el buffer de ground truth usa `odom_history_seconds` (30 s). Pausar mapeo también pausa el muestreo de referencias.

Para instalar cambios ejecutar `./slam/scripts/sync_container.sh` en el host. Luego cargar `source ~/rosmaster_ws/install/setup.bash` dentro del contenedor y reiniciar el launcher con la misma receta (toy o café/simulation) que se estaba usando. Guardar antes con `/slam/save_map` si se desea conservar el mapa actual. Para abrir sólo la nueva configuración sin reiniciar el mapeador: **File → Open Config** y elegir `~/rosmaster_ws/install/slam/share/slam/rviz/slam.rviz`; las nuevas referencias requieren reiniciar también el nodo SLAM.

Mover la cámara no conduce al robot. En A se usa teleop externo. La navegación por objetivos está implementada parcialmente en el paquete `nav`; ver [guia_etapa_b.md](guia_etapa_b.md).

Validación del 10/11/2026: 31 pruebas del núcleo aprobadas en host y Humble; el toy verificó las 118 poses de ambas referencias contra su fuente analítica y el borrado al resetear. RViz cargó los nuevos plugins en una pantalla Xvfb y dominio ROS separado, sin errores; no se reinició la sesión gráfica activa. Toy y bag sintético con odometría por tópico y por TF integraron 118 scans cada uno, borraron las referencias al resetear y exportaron PGM idénticos. Evidencia ROS de esta ampliación en `artifacts/ros_reference_paths/`. Una consulta de 8 s al `/ground_truth/odom` del contenedor habitual no recibió muestras en ese momento; la comparación completa sobre un recorrido de Gazebo sigue pendiente.

### Ejemplo sintético con ROS

```bash
ros2 launch slam slam.launch.py mode:=toy rviz:=true output_path:=/tmp/slam/toy
```

Este modo permite inspeccionar el mismo filtro con tópicos y TF, sin depender del robot. El ejemplo Python de la sección 1 es la vía para obtener las métricas contra ground truth.

### Simulación

```bash
ros2 launch slam slam.launch.py mode:=simulation rviz:=true output_path:=/tmp/slam/simulation
```

Se puede usar cualquier mundo si el simulador publica el contrato de entrada. Para conectarse a una simulación que ya está ejecutándose:

```bash
ros2 launch slam slam.launch.py mode:=simulation start_simulator:=false scan_topic:=/scan odom_topic:=/odom base_frame:=base_link output_path:=/tmp/slam/otro_mundo
```

Para iniciar otro mundo con el backend del RosMaster:

```bash
ros2 launch slam slam.launch.py mode:=simulation world:=/ruta/absoluta/mundo.sdf config_file:=/ruta/absoluta/config.yaml headless:=true rviz:=false
```

El backend esperado es `yahboom_rosmaster_gazebo`, launchfile `rosmaster_gazebo_fortress.launch.py`, con Gazebo Fortress. Se incluye directamente el robot/mundo y se evita incorporar los launchers de mapas estáticos de otros TPs. La disponibilidad de ese paquete se comprueba con `ros2 pkg prefix yahboom_rosmaster_gazebo`.

Usar el frame de base que realmente publica el RosMaster; algunos modelos incorporan `base_footprint`. Mantener una sola fuente de `/clock` y una sola autoridad de `map → odom`. Los launchers anteriores de otros TPs que arrancan un map server o una transformación fija `map → odom` deben adaptarse para ejecutar sólo el robot y el mundo.

La exploración de A se realiza conduciendo el robot con el teleop del RosMaster y relevando todas las habitaciones y pasillos. Este paquete no implementa exploración autónoma ni envía velocidades. Recorrer a baja velocidad, mirar puertas desde ambos lados y cerrar un lazo ayuda a evaluar consistencia. La autonomía de exploración se incorpora en C.

#### Probar el mundo café de `custom_map`

Detener con Ctrl+C cualquier launcher toy o simulación anterior. Para disponer de la configuración del café, resincronizar el paquete **en el host** y entrar al contenedor habitual:

```bash
cd ~/TP_final
./slam/scripts/sync_container.sh
~/PRA_00/container.sh enter
```

Luego ejecutar **dentro del contenedor**:

```bash
source ~/rosmaster_ws/install/setup.bash
CAFE_SHARE="$(ros2 pkg prefix custom_map)/share/custom_map"
export IGN_GAZEBO_RESOURCE_PATH="$CAFE_SHARE/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="$CAFE_SHARE/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
ros2 launch slam slam.launch.py mode:=simulation world:="$CAFE_SHARE/worlds/cafe.world" config_file:="$(ros2 pkg prefix slam)/share/slam/config/cafe.yaml" rviz:=true output_path:=/home/alumno/rosmaster_ws/maps/cafe
```

Los modelos locales `custom_cafe` y `custom_cafe_table` se referencian con `model://`; por eso se agregan sus rutas de recursos preservando los valores existentes. Se verificó que el mundo y sus modelos están instalados en el contenedor, sin ejecutar todavía este mundo.

`config/cafe.yaml` usa el caso D con scan matching, 16 partículas y `prior_xy=0.025` (ver `docs/decisiones/002-propuesta-con-scan-matching.md`), amplía la grilla a **16 × 32 m**, con origen `(-8,-16)` y resolución de 5 cm, y fija `sensor.min_range=0.25` para descartar retornos del propio chasis medidos a 0,14–0,20 m en Gazebo. El café se extiende aproximadamente de y=-13,6 a y=12,4 m considerando piso y paredes: la grilla toy de y=-8 a y=8 lo truncaría. El SLAM crea un mapa nuevo; no carga el mapa grabado de `custom_map`.

No ejecutar a la vez `ros2 launch custom_map cafe.launch.py`: ese launcher inicia su propio map server y una transformación fija `map → odom`, que entrarían en conflicto con las salidas de SLAM. Una vez abierto Gazebo, explorar con el teleop del RosMaster y guardar con `/slam/save_map` como en la sección 5. La salida `cafe.pgm/.yaml/.json` queda dentro de `~/rosmaster_ws/maps` en el contenedor.

Para comparar varias corridas, elegir un prefijo nuevo **antes de cada launch**:
`output_path` se vuelve a escribir al guardar y también al cerrar ordenadamente.
El nombre fijo `cafe` no conserva versiones anteriores. Por ejemplo:

```bash
CAFE_RUN="$HOME/rosmaster_ws/maps/cafe_lento_$(date +%Y%m%d_%H%M%S)"
ros2 launch slam slam.launch.py mode:=simulation \
  world:="$CAFE_SHARE/worlds/cafe.world" \
  config_file:="$(ros2 pkg prefix slam)/share/slam/config/cafe.yaml" \
  rviz:=true output_path:="$CAFE_RUN"
```

En la corrida rápida, cambiar `cafe_lento` por `cafe_rapido`. Conservar los tres
archivos de cada prefijo y copiarlos al host antes de eliminar el contenedor.
La velocidad reportada por teleop es una consigna; medirla exige guardar
odometría o GT con timestamps. El PGM/YAML/JSON final permite analizar geometría
y diagnósticos, pero no reconstruir trayectorias ni calcular RMSE de pose.

### Rosbag

Inspeccionar primero el bag:

```bash
ros2 bag info /ruta/al/bag
```

Luego iniciar la reproducción desde el launcher:

```bash
ros2 launch slam slam.launch.py mode:=bag bag_path:=/ruta/al/bag bag_rate:=0.5 scan_topic:=/scan odom_source:=topic odom_topic:=/odom output_path:=/tmp/slam/bag rviz:=true
```

El modo bag usa tiempo simulado y el player publica `/clock`. Bajar `bag_rate` ayuda si el procesamiento tarda más que el intervalo entre lecturas. No altera la velocidad física registrada, ni elimina la distorsión de movimiento del LIDAR. Si el bag contiene `/tf` con la odometría correcta, también puede usarse `odom_source:=tf`. Si contiene sólo `/odom`, la integración puede publicar el enlace auxiliar mediante `publish_odom_tf:=true`, siempre que otro nodo no lo publique.

El bag debe aportar la extrínseca del láser por `/tf_static`/URDF o mediante una transformación estática calibrada. No agregar una identidad para ocultar errores de frames. El launcher reproduce únicamente el tópico del scan, el de odometría y TF, junto con su `/clock`; no reproduce `/cmd_vel`. Los TF del bag pasan por tópicos internos `/slam/bag_tf` y `/slam/bag_tf_static`; un relay filtra enlaces de localización previos que involucren `map` y padres incompatibles de `odom`. Así no compiten con el `map → odom` calculado por este filtro. La reproducción usa los overrides QoS de `config/bag_qos.yaml` para conservar las transformaciones estáticas.

Para grabar un nuevo bag, seleccionar datos de mapeo y conservar la extrínseca:

```bash
ros2 bag record -o bag_entrega_a /scan /odom /tf /tf_static
```

Adaptar nombres al sistema y verificar que `/tf_static` quedó registrado. El procesamiento sigue siendo online durante la reproducción; no es un optimizador batch. Según la consigna, elegir un método realmente offline exige grabar un bag propio.

### Robot real

Primero ejecutar los drivers, la odometría y la publicación del URDF según el setup del RosMaster. Con `/scan`, `/odom` y TF comprobados:

```bash
ros2 launch slam slam.launch.py mode:=real scan_topic:=/scan odom_source:=topic \
  odom_topic:=/odom base_frame:=base_footprint output_path:=/tmp/slam/aula rviz:=true \
  config_file:=$PWD/config/parcial_bag_scan_match.yaml
```

Es el comando usado en el laboratorio el 16/11. En ese robot la base es `base_footprint` (con `base_link` el nodo descarta todos los scans) y hace falta el perfil con el mapa de 48 × 40 m, porque el de `default.yaml` (16 × 16 m) no alcanza. El modo real usa reloj de pared. Los equipos deben compartir una referencia temporal coherente; escaneos y odometría desfasados deforman las paredes aunque las transformaciones espaciales estén bien calibradas. Manejar el robot con el teleop ya validado y guardar el bag de la sesión para comparar configuraciones sobre las mismas lecturas.

## 5. Guardar y controlar el mapeador

Los servicios son `std_srvs/srv/Trigger`:

```bash
ros2 service call /slam/save_map std_srvs/srv/Trigger '{}'
ros2 service call /slam/pause std_srvs/srv/Trigger '{}'
ros2 service call /slam/resume std_srvs/srv/Trigger '{}'
ros2 service call /slam/reset std_srvs/srv/Trigger '{}'
```

`save_map` escribe el prefijo configurado con `output_path`: `<prefijo>.pgm` y `<prefijo>.yaml`. También se intenta guardar al cerrar ordenadamente con Ctrl+C. Guardar explícitamente antes de detener un experimento evita depender del cierre. Una pausa detiene la incorporación de lecturas; no frena el robot. `resume` conserva el desplazamiento odométrico acumulado; si el robot se movió mucho durante la pausa, su incertidumbre puede superar lo que las partículas representan. `reset` borra el mapa y las hipótesis de la sesión, por lo que se debe guardar antes si se quieren conservar.

La exportación es trinaría: ocupado negro, libre blanco y desconocido gris. Las celdas no observadas y las observadas sin confianza suficiente quedan desconocidas; el umbral libre es `0.25` y el ocupado toma `sensor.occupied_threshold`. No se exporta una inflación de obstáculos: B debe aplicarla según la huella del robot. El PGM invierte el orden vertical de las filas para que coincida con el origen inferior izquierdo indicado en el YAML.

La publicación incluye `/map` (`nav_msgs/OccupancyGrid`), `/slam_pose`, `/particles`, `/path` y `/slam/status`, además de la transformación `map → odom`. El estado permite distinguir espera de datos, funcionamiento, pausa y error. El mapa para B debe guardarse después de explorar todo el entorno y comprobar puertas, paredes y zonas desconocidas.

Para inspeccionar estado y la pose publicada:

```bash
ros2 topic echo /slam/status
ros2 topic echo /slam_pose
```

El diagnóstico informa scans integrados, omitidos por movimiento, rechazados por tiempo o datos, ESS, remuestreos, tiempo de actualización, rayos que salen de la grilla y colas/descartes del adaptador. Un salto temporal hacia atrás exige `/slam/reset` antes de una nueva sesión. La covarianza describe la dispersión de la población; una población empobrecida puede estar muy concentrada y equivocada, por lo que ese valor no reemplaza las métricas geométricas.

## 8. Problemas frecuentes

### Perfil real

`config/parcial_bag_scan_match.yaml` es el perfil para bags y robot real: 8
partículas, `prior_xy=0.05`, `sample_xy=0.01`, grilla de 48 × 40 m. Es la
configuración que generó el mapa final del laboratorio. Indicar ese archivo con
`config_file` en los modos bag/real y resincronizar antes de relanzar. Entre el
18/11 y el 25/11 el archivo tuvo 16 partículas (E16); la
[decisión 014](decisiones/014-perfil-real-8-particulas.md) explica por qué volvió a 8.

### Perfil Café D y aceleración numérica opcional

El usuario eligió D como perfil de trabajo de Café el 18/11/2026: `cafe.yaml`
selecciona 16 partículas, `prior_xy=0.025`, `sample_xy=0.01` y conserva el resto
del sensor/movimiento. No se traslada al robot real. La evidencia es una semilla
y replay del núcleo; faltan robustez y ROS1×.

Para disponer del backend medido, usar el entorno administrado de la sección 2.
En el host:

```bash
./slam/scripts/sync_container.sh --with-numba
./slam/scripts/container_env.sh status
```

El estado debe imprimir `SLAM backend: numba`. El camino NumPy conserva la
configuración D pero no el costo medido con JIT; no afirmar tiempo real por esa
equivalencia.

La implementación incorporada desde el worktree de profiling agrega kernels propios compilables con Numba para
consultas del scan matcher, distancias del likelihood y escritura de la grilla.
La [comparación manual de Café](comparacion_cafe_manual.md) permite al usuario
seleccionar original u optimizado sin sustituir el paquete instalado y ensayar
ocho/dieciséis partículas y cada hiperparámetro por separado.
El [ciclo colaborativo Café 01](ciclo_hitl_cafe_01.md) documenta los cinco replays
de núcleo aprobados, las métricas v2 y el punto de revisión visual humana.
La misma evaluación sobre el bag físico del 16/11 está registrada en
[el ciclo bag16 01](ciclo_hitl_bag16_01.md), en un worktree separado. Sus
replays headless no validan colas/descartes ROS ni cambian el perfil físico
instalado. El análisis sin GT usa `slam/scripts/analyze_bag_cycle.py`.
El [ciclo bag16 02](ciclo_hitl_bag16_02.md) documenta el ensayo autorizado de
24 partículas, la comprobación obligatoria de Numba y los argumentos para
compararlo con el control E16 sin desplazar los checkpoints del ciclo anterior.
Si Numba no está instalado o `SLAM_DISABLE_NUMBA=1`, usa NumPy. No cambia el
modelo ni los parámetros del sensor. La ruta compilada usa `fastmath=false` y
calienta sus firmas antes de recibir sensores; el primer arranque puede tardar
más por la compilación. `NUMBA_CACHE_DIR` puede seleccionar una carpeta writable
si los fuentes están montados read-only.

La combinación evaluada en ROS2 Humble/Python3.10 fue NumPy1.21.5,
Numba0.57.1 y llvmlite0.40.1. El archivo
`slam/requirements-acceleration-humble.txt` conserva esas restricciones. No
instalarlo sobre el Python global del host. Para ejecutar el camino NumPy de
control, dentro del contenedor y antes del launch:

```bash
tp-env numpy
```

`tp-env on` vuelve a habilitar Numba. También se puede fijar
`SLAM_DISABLE_NUMBA=1` manualmente. El sincronizador sólo instala este entorno
cuando recibe `--with-numba`; una sincronización habitual conserva el entorno
existente. El núcleo sigue ejecutándose sin Numba con los requirements habituales. Los métodos de
profiling y la condición de integración están en
[perfilado_20261118.md](perfilado_20261118.md).

| Síntoma | Comprobación concreta |
| --- | --- |
| Espera indefinida sin mapa | Confirmar scans válidos, odometría temporalmente compatible y extrínseca base-láser. |
| Errores de TF al reproducir | Comprobar `/clock`, `use_sim_time`, frames y registro de `/tf_static`; no consultar la última TF en lugar del timestamp del scan. |
| No llegan lecturas | Revisar nombres y QoS con `ros2 topic info --verbose`; los sensores suelen publicar best effort. |
| Mapa que salta o árbol TF que parpadea | Buscar dos publicadores de `map → odom` o de `odom → base`. |
| Paredes dobles | Revisar extrínseca, desfase temporal, giro durante el barrido y deriva; después ajustar el filtro. |
| Puertas cerradas artificialmente | Revisar resolución, espesor de impacto, ruido y cobertura desde ambos lados. |
| Mapa truncado | Ampliar `map.width/height` y ajustar origen; el mapa tiene extensión fija. |
| Retraso creciente con el bag | Reducir `bag_rate`, haces o partículas; medir tiempo por scan. |
| Se perdió la hipótesis correcta | Reiniciar y repetir con mejor odometría/recorrido o mayor diversidad; esta versión no hace optimización global retrospectiva. |

## Navegación B después del merge de los PR

La navegación usa perfiles propios en `nav/config/`, separados de los del SLAM. En Café, `navigator.stop_distance=0.40` m conserva una franja frontal detectable con `sensor.min_range=0.25` m y la extrínseca del láser; no copiar los 0,25 m del perfil real a Café. Resincronizar y construir ambos paquetes antes de ejecutar la versión nueva. Los comandos y las limitaciones para objetivos cerca de paredes están en la [guía de B](guia_etapa_b.md#parada-de-seguridad-por-perfil). La validación de esta corrección fue unitaria; no se lanzaron ROS, RViz, Gazebo ni replays.


La navegación también corta avance y giro al perder el LIDAR. En real el
umbral inicial es 0,50 s de silencio local y en Café es 0,45 s de simulación.
Para recuperar el viaje se requiere un objetivo nuevo. Ver [guía de B](guia_etapa_b.md#pérdida-del-lidar).
