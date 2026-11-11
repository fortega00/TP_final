# Entrega A: ejecución y configuración

Se implementa FastSLAM con mapas de ocupación en grilla: cada partícula conserva su pose y su propio mapa en log-odds. El LIDAR actualiza las celdas mediante un modelo inverso; la ponderación usa la concordancia con obstáculos del mapa previo. Los algoritmos de movimiento, medición, mapeo y filtrado son propios; NumPy aporta operaciones numéricas, PyYAML carga configuración y ROS proporciona transporte, TF y visualización. No se delega SLAM a GMapping, SLAM Toolbox, Cartographer u otra biblioteca.

La base de integración es **ROS 2 Humble**. El núcleo también funciona con Python 3 sin instalar ROS. [docs/architecture.md](docs/architecture.md) describe los módulos y estados; [docs/entrega_a.md](docs/entrega_a.md) responde las preguntas de la consigna y distingue resultados medidos de validaciones pendientes.

## 1. Prueba reproducible sin ROS

Desde la raíz del proyecto:

```bash
python3 -m pip install numpy PyYAML
python3 -m tp_slam.cli toy --output artifacts/toy --seed 7 --check
python3 -m unittest discover -s tests -v
```

En una instalación Ubuntu con paquetes del sistema se pueden obtener las dependencias con `sudo apt install python3-numpy python3-yaml`. El ejemplo usa un entorno de oficina de 9 × 7 m, con una partición interior, una puerta y una trayectoria cerrada que recorre ambos lados. Simula ruido de rango, odometría imperfecta y una referencia geométrica independiente de las partículas. Los scans son instantáneos: el paso de traslación de 0,28 m y el período sintético de 0,25 s definen el muestreo del benchmark; no constituyen un ensayo físico de barrido a 1,12 m/s.

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
python3 -m tp_slam.cli toy --output artifacts/toy_coarse --seed 7 --resolution 0.10
python3 -m tp_slam.cli toy --output artifacts/toy_dense --seed 7 --particles 48
```

Los argumentos específicos del generador incluyen `--sensor-noise`, `--odom-translation-noise`, `--odom-rotation-noise`, `--odom-scale`, `--beams` y `--scan-step`. Modifican los datos sintéticos; los coeficientes del filtro se configuran por separado. Consultar los valores y unidades con:

```bash
python3 -m tp_slam.cli toy --help
```

## 2. Instalar el paquete ROS

### Usar el contenedor habitual del RosMaster

El contenedor `yahboom_rosmaster` que se abre con `~/PRA_00/container.sh enter` usa el usuario `alumno` y el workspace `/home/alumno/rosmaster_ws`. Su montaje actual incluye los paquetes de `PRA_00`, pero no la carpeta `~/TP_final` del host. Para incorporar este paquete, ejecutar **en el host**:

```bash
cd ~/TP_final
./scripts/sync_container.sh
```

El script copia los archivos del paquete a `/home/alumno/rosmaster_ws/src/tp_slam` y construye únicamente `tp_slam` con colcon. Conserva los otros paquetes y el contenedor existente. El 10 de noviembre de 2026 se verificó la construcción en este contenedor habitual: colcon terminó correctamente, ROS encontró el paquete en `/home/alumno/rosmaster_ws/install/tp_slam` y enumeró sus cuatro ejecutables. La apertura de nodos, RViz y Gazebo desde esta sesión interactiva queda a cargo del usuario; esta comprobación no los lanzó.

Después entrar como siempre:

```bash
~/PRA_00/container.sh enter
```

Y ejecutar **dentro del contenedor**:

```bash
cd ~/rosmaster_ws
source install/setup.bash
ros2 launch tp_slam slam.launch.py mode:=toy rviz:=true output_path:=/home/alumno/rosmaster_ws/maps/tp_slam_toy
```

El modo toy publica los sensores sintéticos y abre RViz. Al terminar el recorrido se puede guardar desde otra terminal del mismo contenedor, cargando el overlay:

```bash
source ~/rosmaster_ws/install/setup.bash
ros2 service call /slam/save_map std_srvs/srv/Trigger '{}'
```

Los archivos `tp_slam_toy.pgm`, `.yaml` y `.json` quedan en `/home/alumno/rosmaster_ws/maps`, dentro del contenedor. Esa carpeta no está montada en `~/TP_final`: los resultados no se copian automáticamente al host y se perderían al eliminar el contenedor.

Como alternativa, detener primero el launcher toy con **Ctrl+C** y ejecutar Gazebo desde ese mismo entorno:

```bash
ros2 launch tp_slam slam.launch.py mode:=simulation rviz:=true output_path:=/home/alumno/rosmaster_ws/maps/tp_slam_simulation
```

No ejecutar ambos launchers simultáneamente: comparten tópicos, reloj y TF. En simulación se conduce el robot con el teleop del RosMaster para explorar el mundo, como se detalla en la sección 4.

La copia del paquete no es un montaje de los fuentes. Tras editar código, YAML o launchfiles en `~/TP_final`, detener el launcher y volver a ejecutar `./scripts/sync_container.sh` en el host. Cargar otra vez `~/rosmaster_ws/install/setup.bash` antes de relanzar. Si sólo se quiere actualizar la copia sin construir todavía:

```bash
cd ~/TP_final
./scripts/sync_container.sh --sync-only
```

En ese caso, antes de ejecutar ROS completar la construcción dentro del contenedor:

```bash
cd ~/rosmaster_ws
colcon build --symlink-install --packages-select tp_slam
source install/setup.bash
```

### Instalar en otro entorno con ROS 2 Humble

En Ubuntu 22.04, con ROS 2 Humble instalado y su entorno cargado:

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-numpy python3-yaml ros-humble-rviz2 ros-humble-rosbag2
```

El repositorio es un paquete `ament_python`. Puede colocarse o enlazarse en `src` de un workspace. Por ejemplo, estando en su raíz:

```bash
TP_SLAM_SOURCE="$(pwd)"
mkdir -p ~/tp_slam_ws/src
ln -s "$TP_SLAM_SOURCE" ~/tp_slam_ws/src/tp_slam
cd ~/tp_slam_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select tp_slam
source install/setup.bash
```

Si el enlace ya existe, se omite `ln -s`. Cada terminal nueva debe cargar `/opt/ros/humble/setup.bash` y `~/tp_slam_ws/install/setup.bash`. El modo `simulation` puede requerir además el workspace de los drivers y la simulación del RosMaster; el paquete de SLAM no necesita ese backend para procesar datos existentes.

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
map --(tp_slam)--> odom --(base / simulación / bag)--> base_footprint --(URDF o estática calibrada)--> laser
```

`odom_source:=topic` consume `/odom` de tipo `nav_msgs/msg/Odometry` e interpola la pose al timestamp de la lectura. `odom_source:=tf` obtiene `odom → base` directamente de TF. Para mensajes de odometría se verifica el marco padre y el `child_frame_id`. Si este último describe un punto distinto de la base, debe existir la transformación de montaje correspondiente o configurarse explícitamente la convención correcta. Cambiar sólo el nombre de un tópico no convierte una pose de `calc_base` a `base_link`.

La extrínseca se consulta por TF; no se supone silenciosamente que el láser coincide con el centro del robot. Si el robot o el bag ya publica su árbol, mantener `publish_scan_tf:=false` y `publish_odom_tf:=false`. Estos publicadores auxiliares se habilitan únicamente para completar una transformación ausente y conocida.

El frame `map` se fija a la pose inicial configurada, por defecto `(0,0,0)`. El primer mensaje de odometría sirve de referencia para los incrementos; sus coordenadas absolutas no deben confundirse con la pose inicial del mapa. La grilla no se carga como mapa conocido al iniciar SLAM: se construye desde desconocido.

## 4. Launcher único

El punto de entrada es:

```bash
ros2 launch tp_slam slam.launch.py mode:=toy
```

Los modos seleccionan la fuente de lecturas y el reloj. La configuración del algoritmo se comparte mediante `config_file`.

Para consultar todos los argumentos del launcher:

```bash
ros2 launch tp_slam slam.launch.py --show-args
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

Para instalar cambios ejecutar `./scripts/sync_container.sh` en el host. Luego cargar `source ~/rosmaster_ws/install/setup.bash` dentro del contenedor y reiniciar el launcher con la misma receta (toy o café/simulation) que se estaba usando. Guardar antes con `/slam/save_map` si se desea conservar el mapa actual. Para abrir sólo la nueva configuración sin reiniciar el mapeador: **File → Open Config** y elegir `~/rosmaster_ws/install/tp_slam/share/tp_slam/rviz/slam.rviz`; las nuevas referencias requieren reiniciar también el nodo SLAM.

Mover la cámara no conduce al robot. La conducción sigue siendo por teleop externo; seleccionar objetivos de navegación en RViz corresponde a la Parte B, todavía no implementada.

Validación del 10/11/2026: 31 pruebas del núcleo aprobadas en host y Humble; el toy verificó las 118 poses de ambas referencias contra su fuente analítica y el borrado al resetear. RViz cargó los nuevos plugins en una pantalla Xvfb y dominio ROS separado, sin errores; no se reinició la sesión gráfica activa. Toy y bag sintético con odometría por tópico y por TF integraron 118 scans cada uno, borraron las referencias al resetear y exportaron PGM idénticos. Evidencia ROS de esta ampliación en `artifacts/ros_reference_paths/`. Una consulta de 8 s al `/ground_truth/odom` del contenedor habitual no recibió muestras en ese momento; la comparación completa sobre un recorrido de Gazebo sigue pendiente.

### Ejemplo sintético con ROS

```bash
ros2 launch tp_slam slam.launch.py mode:=toy rviz:=true output_path:=/tmp/tp_slam/toy
```

Este modo permite inspeccionar el mismo filtro con tópicos y TF, sin depender del robot. El ejemplo Python de la sección 1 es la vía para obtener las métricas contra ground truth.

### Simulación

```bash
ros2 launch tp_slam slam.launch.py mode:=simulation rviz:=true output_path:=/tmp/tp_slam/simulation
```

Se puede usar cualquier mundo si el simulador publica el contrato de entrada. Para conectarse a una simulación que ya está ejecutándose:

```bash
ros2 launch tp_slam slam.launch.py mode:=simulation start_simulator:=false scan_topic:=/scan odom_topic:=/odom base_frame:=base_link output_path:=/tmp/tp_slam/otro_mundo
```

Para iniciar otro mundo con el backend del RosMaster:

```bash
ros2 launch tp_slam slam.launch.py mode:=simulation world:=/ruta/absoluta/mundo.sdf config_file:=/ruta/absoluta/config.yaml headless:=true rviz:=false
```

El backend esperado es `yahboom_rosmaster_gazebo`, launchfile `rosmaster_gazebo_fortress.launch.py`, con Gazebo Fortress. Se incluye directamente el robot/mundo y se evita incorporar los launchers de mapas estáticos de otros TPs. La disponibilidad de ese paquete se comprueba con `ros2 pkg prefix yahboom_rosmaster_gazebo`.

Usar el frame de base que realmente publica el RosMaster; algunos modelos incorporan `base_footprint`. Mantener una sola fuente de `/clock` y una sola autoridad de `map → odom`. Los launchers anteriores de otros TPs que arrancan un map server o una transformación fija `map → odom` deben adaptarse para ejecutar sólo el robot y el mundo.

La exploración de A se realiza conduciendo el robot con el teleop del RosMaster y relevando todas las habitaciones y pasillos. Este paquete no implementa exploración autónoma ni envía velocidades. Recorrer a baja velocidad, mirar puertas desde ambos lados y cerrar un lazo ayuda a evaluar consistencia. La autonomía de exploración se incorpora en C.

#### Probar el mundo café de `custom_map`

Detener con Ctrl+C cualquier launcher toy o simulación anterior. Para disponer de la configuración del café, resincronizar el paquete **en el host** y entrar al contenedor habitual:

```bash
cd ~/TP_final
./scripts/sync_container.sh
~/PRA_00/container.sh enter
```

Luego ejecutar **dentro del contenedor**:

```bash
source ~/rosmaster_ws/install/setup.bash
CAFE_SHARE="$(ros2 pkg prefix custom_map)/share/custom_map"
export IGN_GAZEBO_RESOURCE_PATH="$CAFE_SHARE/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="$CAFE_SHARE/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
ros2 launch tp_slam slam.launch.py mode:=simulation world:="$CAFE_SHARE/worlds/cafe.world" config_file:="$(ros2 pkg prefix tp_slam)/share/tp_slam/config/cafe.yaml" rviz:=true output_path:=/home/alumno/rosmaster_ws/maps/cafe
```

Los modelos locales `custom_cafe` y `custom_cafe_table` se referencian con `model://`; por eso se agregan sus rutas de recursos preservando los valores existentes. Se verificó que el mundo y sus modelos están instalados en el contenedor, sin ejecutar todavía este mundo.

`config/cafe.yaml` conserva el modelo de movimiento y las 24 partículas, amplía la grilla a **16 × 32 m**, con origen `(-8,-16)` y resolución de 5 cm, y fija `sensor.min_range=0.25` para descartar retornos del propio chasis medidos a 0,14–0,20 m en Gazebo. El café se extiende aproximadamente de y=-13,6 a y=12,4 m considerando piso y paredes: la grilla toy de y=-8 a y=8 lo truncaría. El SLAM crea un mapa nuevo; no carga el mapa grabado de `custom_map`.

No ejecutar a la vez `ros2 launch custom_map cafe.launch.py`: ese launcher inicia su propio map server y una transformación fija `map → odom`, que entrarían en conflicto con las salidas de SLAM. Una vez abierto Gazebo, explorar con el teleop del RosMaster y guardar con `/slam/save_map` como en la sección 5. La salida `cafe.pgm/.yaml/.json` queda dentro de `~/rosmaster_ws/maps` en el contenedor.

### Rosbag

Inspeccionar primero el bag:

```bash
ros2 bag info /ruta/al/bag
```

Luego iniciar la reproducción desde el launcher:

```bash
ros2 launch tp_slam slam.launch.py mode:=bag bag_path:=/ruta/al/bag bag_rate:=0.5 scan_topic:=/scan odom_source:=topic odom_topic:=/odom output_path:=/tmp/tp_slam/bag rviz:=true
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
ros2 launch tp_slam slam.launch.py mode:=real scan_topic:=/scan odom_source:=topic odom_topic:=/odom base_frame:=base_link output_path:=/tmp/tp_slam/aula rviz:=true
```

El modo real usa reloj de pared. Los equipos deben compartir una referencia temporal coherente; escaneos y odometría desfasados deforman las paredes aunque las transformaciones espaciales estén bien calibradas. Manejar el robot con el teleop ya validado y guardar el bag de la sesión para comparar configuraciones sobre las mismas lecturas.

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

## 6. Hiperparámetros del filtro

Crear una copia de `config/default.yaml` y pasar su ruta absoluta con `config_file:=/ruta/a/config.yaml`. Es un YAML del núcleo con grupos `map`, `sensor`, `motion`, `filter`, `update` y `scan_match` (opcional, ver `docs/decisiones/002-propuesta-con-scan-matching.md`), compartido por CLI y ROS. Los campos desconocidos se rechazan para evitar que un error de escritura deje un parámetro sin aplicar. Ejemplo de una configuración parcial válida:

```yaml
map:
  resolution: 0.05
  width: 16.0
  height: 16.0
  origin_x: -8.0
  origin_y: -8.0
sensor:
  p_hit: 0.70
  p_free: 0.35
filter:
  particles: 24
  seed: 7
```

Los campos omitidos toman los valores de las tablas. Son un punto de partida para el toy, no una calibración del robot real.

Los valores finales se eligieron después de comparar alternativas en el toy. La corrida base de semilla 7 obtuvo precision de obstáculos de 0,9032, recall de 1,0 —con tolerancia de una celda de 5 cm— y RMSE de posición de 2,46 cm. El detalle, las variantes que fallaron y las limitaciones están en [el informe](docs/entrega_a.md) y en `artifacts/tuning`. La odometría de ese toy ya es precisa: su RMSE fue 2,01 cm, menor que el de SLAM; no se presenta como una mejora general de localización.

### Geometría y prior

| Campo | Default | Unidad y efecto |
| --- | ---: | --- |
| `map.resolution` | `0.05` | m/celda. Permite representar puertas y paredes finas con más detalle que 10 cm. |
| `map.width` | `16.0` | Ancho en metros, redondeado hacia arriba a celdas completas. |
| `map.height` | `16.0` | Alto en metros. |
| `map.origin_x` | `-8.0` | Coordenada x del extremo inferior izquierdo de la grilla, en m. |
| `map.origin_y` | `-8.0` | Coordenada y del extremo inferior izquierdo, en m. |
| `map.prior` | `0.5` | Probabilidad inicial; la máscara de observación mantiene unknown separado. |

La extensión es fija. Antes de mapear un entorno mayor, ampliar dimensiones y elegir el origen de modo que la trayectoria y las paredes quepan. No se rellena automáticamente el exterior ni se usa la frontera como obstáculo. El consumo de grillas crece aproximadamente con `partículas × ancho × alto / resolución²`.

### Modelo inverso del sensor

| Campo | Default | Unidad y efecto |
| --- | ---: | --- |
| `sensor.p_hit` | `0.70` | Probabilidad inversa de ocupación para un impacto. |
| `sensor.p_free` | `0.35` | Probabilidad inversa de ocupación para espacio atravesado. |
| `sensor.log_odds_min` | `-4.0` | Límite inferior de evidencia; corresponde aproximadamente a p=0,018. |
| `sensor.log_odds_max` | `4.0` | Límite superior; corresponde aproximadamente a p=0,982. |
| `sensor.hit_thickness` | `0.03` | Espesor radial total de la zona de impacto, en m. |
| `sensor.min_range` | `0.12` | Rango mínimo utilizable, en m, combinado con el del mensaje. |
| `sensor.max_range` | `8.0` | Alcance máximo de mapeo, en m, combinado con el del mensaje. |
| `sensor.max_beams` | `180` | Máximo de haces integrados, seleccionados a lo largo del campo angular. |

Se exige `0 < p_free < prior < p_hit < 1`. La actualización suma el logit de la probabilidad inversa **menos el logit del prior** y luego satura. Una observación de la misma celda no se acumula decenas de veces sólo porque varios haces del mismo scan la atraviesen; se deduplican las celdas por lectura y el impacto tiene precedencia sobre evidencia libre de esa lectura.

Las lecturas `NaN`, `-inf`, no positivas, demasiado cercanas o finitas fuera del rango físico declarado se descartan. Un `+inf` representa ausencia de retorno. Por defecto (`sensor.no_return_is_free: false`) no aporta evidencia, porque en el lidar real un `+inf` aparece con vidrio, superficies oscuras o rasantes y no garantiza espacio libre. Con `no_return_is_free: true` (perfil de Gazebo, `cafe.yaml`) libera el tramo observable sin crear un obstáculo en el extremo. Un valor finito igual al alcance útil libera el tramo y tampoco crea obstáculo. Si se limita el alcance configurado por debajo de un retorno válido, sólo se usa evidencia libre hasta ese límite.

### Modelo de likelihood

| Campo | Default | Unidad y efecto |
| --- | ---: | --- |
| `sensor.likelihood_beams` | `60` | Máximo de extremos usados para ponderar partículas. |
| `sensor.sigma_hit` | `0.10` | Desvío espacial, en m, de la concordancia con obstáculos. |
| `sensor.z_hit` | `0.95` | Peso de la componente de impacto. |
| `sensor.z_rand` | `0.05` | Peso de la componente uniforme; debe ser positivo. |
| `sensor.likelihood_max_dist` | `0.60` | Truncamiento de distancia al obstáculo, en m. |
| `sensor.likelihood_temperature` | `1.0` | Multiplica la suma de log-likelihoods y permite moderar la confianza. |
| `sensor.occupied_threshold` | `0.65` | Umbral para considerar una celda obstáculo en la ponderación. |

La mezcla exige `z_hit + z_rand = 1`. Para un impacto a distancia `d` del obstáculo más próximo se usa una puntuación proporcional a `z_hit·exp(-d²/(2·sigma_hit²)) + z_rand/rango_máximo`. Se evalúa sobre el mapa de cada partícula antes de integrar el scan. El primer scan arranca sin obstáculos previos y no discrimina hipótesis por concordancia artificial con su propio mapa.

Reducir `sigma_hit` o aumentar cantidad de haces/temperatura vuelve la ponderación más selectiva; con odometría ruidosa puede concentrar el peso en una hipótesis equivocada. Aumentarlos en sentido opuesto puede suavizar diferencias hasta que las partículas casi no se corrijan con el LIDAR.

### Movimiento, partículas y actualización

| Campo | Default | Unidad y efecto |
| --- | ---: | --- |
| `motion.alpha1` | `0.0001` | Coeficiente de varianza del ruido de traslación por traslación. |
| `motion.alpha2` | `0.00001` | Varianza de traslación por rotación. |
| `motion.alpha3` | `0.0001` | Varianza de rotación por rotación. |
| `motion.alpha4` | `0.0001` | Varianza de rotación por traslación. |
| `filter.particles` | `24` | Cantidad fija de hipótesis; aumenta costo y diversidad. |
| `filter.seed` | `7` | Semilla entera no negativa. |
| `filter.resample_ess_ratio` | `0.5` | Remuestreo si ESS es menor que esta fracción de N. |
| `filter.initial_x` | `0.0` | Posición inicial del robot en `map`, en m. |
| `filter.initial_y` | `0.0` | Posición inicial y, en m. |
| `filter.initial_yaw` | `0.0` | Orientación inicial en radianes. |
| `filter.initial_std_xy` | `0.0` | Desvío de muestreo inicial por eje, en m. |
| `filter.initial_std_yaw` | `0.0` | Desvío angular inicial, en radianes. |
| `update.min_translation` | `0.08` | Desplazamiento neto mínimo desde el último scan integrado, en m. |
| `update.min_rotation` | `0.06` | Giro neto mínimo desde el último integrado, en radianes. |
| `update.max_interval` | `1.0` | Máximo intervalo entre integraciones, en segundos de datos. |

Los coeficientes `alpha` parametrizan **varianzas**, no desvíos estándar. Para un incremento relativo `(dx,dy,dtheta)`, la varianza por eje de traslación es `alpha1·(dx²+dy²) + alpha2·dtheta²`; la angular es `alpha3·dtheta² + alpha4·(dx²+dy²)`. El modelo conserva movimiento lateral; no supone una base exclusivamente diferencial. Se integra un scan al superar cualquiera de los umbrales de desplazamiento/giro/tiempo. Los scans omitidos no reinician la referencia, de modo que movimientos pequeños pueden acumularse hasta superar el umbral. La semilla no compensa datos perdidos o diferencias en el orden temporal de ROS.

## 7. Cómo ajustar y evaluar

1. Verificar unidades, frames, timestamps, extrínseca y odometría antes de tocar la likelihood. Una extrínseca equivocada genera paredes dobles que no se arreglan aumentando partículas.
2. Elegir extensión y resolución para cubrir el mundo completo y las puertas relevantes. Mantener la referencia geométrica y el recorrido cuando se comparan resoluciones.
3. Estimar ruido y deriva de la odometría, incluyendo desplazamientos laterales, giros y retroceso; ajustar `alpha` para representar la incertidumbre observada.
4. Ajustar el modelo inverso y el espesor de impacto sin ocultar aperturas reales. Una probabilidad de ocupación extrema satura rápido y dificulta corregir falsos obstáculos.
5. Ajustar `sigma_hit`, haces y temperatura observando ESS, pérdida de diversidad, consistencia geométrica y costo. Probar más de una semilla.
6. Guardar parámetros, bag/trayectoria, mapa y métricas de cada corrida. Comparar precisión/recall de paredes con tolerancia métrica, error de trayectoria cuando exista ground truth, ancho de puertas y conectividad después de inflar por el robot.

Para B, comprobar caminos sobre celdas conocidas libres e inflar obstáculos por la huella y un margen. Las métricas de conectividad del toy hacen una comprobación conservadora con disco; no sustituyen el planificador ni certifican cualquier ruta del entorno real. La zona desconocida no debe convertirse en libre para conseguir que una ruta exista.

## 8. Problemas frecuentes

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

## 9. Alcance de la validación

| Prueba | Estado y alcance |
| --- | --- |
| Núcleo y utilidades | 31 pruebas unitarias aprobadas; cubren sensor, grilla, pesos/remuestreo, estados, tiempo, odometría y toy. |
| Benchmark sin ROS | Semillas 7, 11 y 19 aprobadas con los defaults; variantes de ruido muestran fallos de calidad documentados. |
| Build ROS 2 Humble | `colcon build` y carga de argumentos del launcher comprobados en la imagen Docker disponible del RosMaster. |
| ROS con toy y bag sintético | Tres integraciones aprobadas: toy con odometría de tópico, rosbag2 sintético con tópico y con TF. Cada una integró 118 scans sin descartes, con PGM idéntico; servicios, reset y cierre limpio comprobados. [Evidencia](artifacts/ros/README.md). |
| Gazebo café | Recorrido guiado completo, RMSE 2,063 cm, cobertura SDF aproximada observada 99,863 %, 74/74 poses muestreadas conectadas. Perfil ideal; no demuestra corrección de deriva severa. |
| Rosbag real del Parcial | Corrida completa con perfil específico: separación inicio–fin 0,842 m vs 17,855 m de odometría. Persisten discontinuidades y ruido; no certificar mapa definitivo para navegación. |
| Robot físico vivo | Pendiente de ejecución y verificación geométrica del aula. |

La consigna pide un mapa real exportado y adecuado para navegación. El paquete prepara los modos de adquisición y los criterios de evaluación; el mapa del aula sólo puede acreditarse con una sesión física.

Con la imagen Docker existente de los TPs anteriores se puede repetir la validación completa aislada:

```bash
./scripts/validate_ros.sh
```

El script construye el paquete dentro de un contenedor temporal, valida el SDF y ejecuta las pruebas de núcleo, toy ROS y rosbag2 sintético. Monta los fuentes en sólo lectura y escribe evidencia en `artifacts/ros`. No modifica la imagen. Se puede cambiar `TASK_IMAGE` o `TASK_RESULTS`; el script necesita Docker y una imagen con ROS 2 Humble, colcon y las dependencias declaradas. No requiere red durante la ejecución.

## 10. Corridas integrales de café y del bag del Parcial

El procedimiento, las limitaciones de cada métrica y los resultados están en [docs/experimentos_a.md](docs/experimentos_a.md). El recorrido guiado se aprobó para la evidencia de A; sus puntos sólo alimentan al controlador de prueba. El filtro sigue comenzando con mapa desconocido. `ideal` y `motion_bias=false` se mantienen en café.

Para reproducir los experimentos en el contenedor habitual, primero sincronizar y copiar los auxiliares **desde el host**:

```bash
cd ~/TP_final
./scripts/sync_container.sh
docker cp scripts/run_a_experiment.py yahboom_rosmaster:/tmp/tp_final_run_a.py
docker cp scripts/cafe_route.yaml yahboom_rosmaster:/tmp/tp_final_cafe_route.yaml
~/PRA_00/container.sh enter
```

Dentro del contenedor, con directorios de salida nuevos y dominios/pantallas libres:

```bash
source ~/rosmaster_ws/install/setup.bash
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
python3 /tmp/tp_final_run_a.py --mode simulation \
  --config "$(ros2 pkg prefix tp_slam)/share/tp_slam/config/cafe.yaml" \
  --route /tmp/tp_final_cafe_route.yaml --max-sim-seconds 900 \
  --output /tmp/entrega_a/cafe --domain 96 --display :96
```

El ejecutor inicia su pantalla Xvfb y RViz, conduce en la simulación, exporta el mapa y guarda una captura al terminar el recorrido. Sus estados y puntos se registran en las muestras. Si se omite `--route`, utiliza el seguidor izquierdo experimental, que no garantiza cobertura.

En otra sesión independiente puede ejecutarse el bag real, cuya ruta debe existir **dentro** del contenedor:

```bash
python3 /tmp/tp_final_run_a.py --mode bag \
  --config "$(ros2 pkg prefix tp_slam)/share/tp_slam/config/parcial_bag_tuned.yaml" \
  --bag /tmp/real_rosbag_input --bag-rate 0.15 --timeout 2400 \
  --output /tmp/entrega_a/rosbag --domain 97 --display :97
```

El perfil del bag es específico del experimento: 128 partículas, alpha `(0.10,0.01,0.10,0.10)`, sigma 0,15 m, `p_hit=0.75`, `p_free=0.40` y espesor 0,08 m. No cambia los defaults del toy o del robot real ni constituye calibración general. `parcial_bag.yaml` conserva los defaults con una grilla ampliada para reproducir la referencia que falla.

Después de terminar, traer las salidas **al host** y analizarlas:

```bash
docker cp yahboom_rosmaster:/tmp/entrega_a ./artifacts/entrega_a_repeticion
python3 -m pip install -r requirements-evaluation.txt
python3 scripts/analyze_a_experiment.py artifacts/entrega_a_repeticion/cafe \
  --cafe-package /home/fran/PRA_00/pkg/custom_map
python3 scripts/analyze_a_experiment.py artifacts/entrega_a_repeticion/rosbag
```

El análisis requiere NumPy/PyYAML, Pillow para leer el PGM y Matplotlib para las figuras. No incorpora bibliotecas de SLAM. En café se calcula error respecto del GT y cobertura aproximada desde el SDF; en el bag no se informa RMSE absoluto porque no hay GT. Conectividad de poses visitadas e inspección de un mapa no certifican navegación entre cualquier par de puntos.

## Referencias

- Thrun, Burgard y Fox, *Probabilistic Robotics*, MIT Press, 2005. Capítulos sobre movimiento, sensores, occupancy grids y FastSLAM. [Ficha de la editorial](https://mitpress.mit.edu/9780262201629/probabilistic-robotics/).
- Montemerlo et al., *FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem*, 2002. [Publicación original](https://robots.stanford.edu/papers/montemerlo.fastslam-tr.html).
- Grisetti, Stachniss y Burgard, *Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters*, IEEE T-RO, 2007. [PDF de los autores](https://www.ipb.uni-bonn.de/wp-content/papercite-data/pdf/grisetti07tro.pdf). Orienta el RBPF en grillas y el remuestreo selectivo; su propuesta con scan matching queda como extensión.
- ROS 2: [LaserScan de Humble](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg) y [rosbag2 Humble](https://github.com/ros2/rosbag2/tree/humble).
