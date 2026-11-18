# Comparación manual: aceleración, partículas y propuesta

Preparado el 18/11/2026. El usuario ejecuta todas las sesiones ROS/RViz/Gazebo.
No se ejecutaron pruebas al preparar estos perfiles ni se fusionó el worktree.
La rama original es `main` (código b9d82a1); la optimizada es
`perf/slam-profile-20261118`, implementación cb806b6 y evidencia 38ad161.

## Comparaciones propuestas

| Caso | Código | Partículas | prior_xy | sample_xy | Perfil |
| --- | --- | ---: | ---: | ---: | --- |
| A | Original | 8 | 0.05 | 0.01 | cafe_compare_8.yaml |
| B | Optimizado | 8 | 0.05 | 0.01 | cafe_compare_8.yaml |
| C | Optimizado | 16 | 0.05 | 0.01 | cafe_compare_16.yaml |
| D | Optimizado | 16 | 0.025 | 0.01 | cafe_compare_16_prior.yaml |
| E | Optimizado | 16 | 0.05 | 0.005 | cafe_compare_16_sample.yaml |

A/B aísla código/recepción, B/C cambia solo la cantidad de partículas y C/D/E
examina cada hiperparámetro por separado. Si D o E resulta mejor, repetir ese
mismo perfil con código original permite contrastar las dos implementaciones
a igual configuración; los cuatro perfiles sirven con ambas versiones.

Los perfiles son copias completas de `cafe.yaml`: mantienen semilla 7, grilla,
sensor, actualización y movimiento. No se alteró el perfil normal. No combinar
todavía ambos cambios ni agregar partículas a la comparación A/B. Numba puede
apagar JIT con `SLAM_DISABLE_NUMBA=1`, pero eso **no recupera la versión original**:
el worktree conserva optimizaciones NumPy, caché de paths y cambios de recepción.

La aceleración del núcleo medida en Café fue 2,60× con ocho partículas. Dieciséis
es un primer ensayo razonable, no una garantía de tiempo real. Veinticuatro
triplica partículas respecto de ocho y puede superar el ahorro; medir antes.
Más partículas representan más hipótesis, pero no eliminan errores compartidos
por un matcher sesgado. Un prior más fuerte favorece la odometría ideal; menos
sample_xy reduce dispersión y puede reducir diversidad. No extender resultados
al robot sin evaluarlo con el bag del 16/11.

## Preparar las dos versiones sin reemplazar el paquete instalado

Con el contenedor habitual ya iniciado **por el usuario**, desde el host:

```bash
TP_COMPARE_WT=/tmp/tp-final-slam-profile-20261118
docker exec -u alumno yahboom_rosmaster mkdir -p \
  /home/alumno/rosmaster_ws/slam_compare/baseline \
  /home/alumno/rosmaster_ws/slam_compare/optimized \
  /home/alumno/rosmaster_ws/slam_compare/config
docker cp /home/fran/TP_final/slam/slam \
  yahboom_rosmaster:/home/alumno/rosmaster_ws/slam_compare/baseline/
docker cp "$TP_COMPARE_WT/slam/slam" \
  yahboom_rosmaster:/home/alumno/rosmaster_ws/slam_compare/optimized/
docker cp "$TP_COMPARE_WT/slam/config/." \
  yahboom_rosmaster:/home/alumno/rosmaster_ws/slam_compare/config/
```

Esto copia el código actual del árbol principal como control. Si se editó ese
código después de b9d82a1, registrar también ese diff antes de compararlo. Los
archivos copiados pueden quedar propiedad de root; el cache JIT se dirige a una
carpeta del usuario.

Dentro del contenedor Humble/Python3.10, con NumPy1.21.5 y SciPy del sistema,
instalar los aceleradores en una carpeta separada (sin modificar NumPy global):

```bash
source /opt/ros/humble/setup.bash
source ~/rosmaster_ws/install/setup.bash
export TP_COMPARE=~/rosmaster_ws/slam_compare
python3 -m pip install --no-deps --target "$TP_COMPARE/jit" \
  numba==0.57.1 llvmlite==0.40.1
export NUMBA_CACHE_DIR="$TP_COMPARE/numba_cache"
unset SLAM_DISABLE_NUMBA
```

En la terminal que lanzará SLAM elegir una sola implementación; estas variables
se fijan antes del launch y cambiarán los imports de sus procesos hijos:

```bash
# Caso A: original. Para B/C/D/E, cambiar baseline por optimized.
export TP_COMPARE=~/rosmaster_ws/slam_compare
export PYTHONPATH="$TP_COMPARE/baseline:$TP_COMPARE/jit${PYTHONPATH:+:$PYTHONPATH}"
python3 -c 'import slam; print(slam.__file__)'
```

Debe imprimir `.../slam_compare/baseline/slam/__init__.py` o su equivalente
`optimized`. En la versión optimizada confirmar además:

```bash
python3 -c 'from slam.core.acceleration import BACKEND; print(BACKEND)'
```

Debe imprimir `numba`; si imprime `numpy` no se está midiendo el JIT. Numba se
calienta al construir el filtro: mantener quieto el robot hasta recibir mapa.
Registrar el tiempo de arranque aparte del costo de procesar scans.

## Lanzar una sesión manual y registrar evidencia

En esa misma terminal, reutilizar el launch Café que ya funcionó, sustituyendo
`config_file` por el perfil deseado. Por ejemplo, caso A:

```bash
CAFE_SHARE="$(ros2 pkg prefix custom_map)/share/custom_map"
export IGN_GAZEBO_RESOURCE_PATH="$CAFE_SHARE/models${IGN_GAZEBO_RESOURCE_PATH:+:$IGN_GAZEBO_RESOURCE_PATH}"
export GZ_SIM_RESOURCE_PATH="$CAFE_SHARE/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export TP_COMPARE_CASE="A_$(date +%Y%m%d_%H%M%S)"
ros2 launch slam slam.launch.py mode:=simulation \
  world:="$CAFE_SHARE/worlds/cafe.world" \
  config_file:="$TP_COMPARE/config/cafe_compare_8.yaml" \
  rviz:=true output_path:="$TP_COMPARE/results/$TP_COMPARE_CASE/map"
```

Detener la sesión anterior antes de cada launch. El launcher instalado sigue
seleccionando Fortress ideal sin sesgos. El teleop se ejecuta igual que antes:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.30 -p turn:=0.50
```

Usar consignas y recorrido fijos al comparar. Las sesiones teleop independientes
sirven para observar comportamiento; **la comparación cuantitativa principal
debe usar la misma grabación**. Grabar una sesión antes de conducir, desde otra
terminal con ROS/overlay cargados; elegir un nombre nuevo por grabación:

```bash
export TP_COMPARE=~/rosmaster_ws/slam_compare
mkdir -p "$TP_COMPARE/bags"
ros2 bag record -o "$TP_COMPARE/bags/cafe_$(date +%Y%m%d_%H%M%S)" \
  --qos-profile-overrides-path "$TP_COMPARE/config/bag_qos.yaml" \
  /clock /scan /odom /tf /tf_static /ground_truth/odom /cmd_vel \
  /path /slam/odom_path /slam/ground_truth_path /slam/status
```

Cerrar el recorder con Ctrl+C, verificar los conteos y `/tf_static` con
`ros2 bag info`. Al repetir un bag, apagar Gazebo y reproducir solo entradas,
GT y reloj; excluir `cmd_vel`, mapa, paths y estado grabados. El launcher bag
actual filtra TF previos y excluye velocidades, pero no reproduce GT: para
medir RMSE se debe configurar explícitamente su reproducción y observación.
No ejecutar el bag completo con todos sus tópicos, porque competirían con el
estimador nuevo. El bag Café crudo del profiling no está incluido en Git y no
reapareció al reconstruir el worktree; localizar otra copia antes de usar esa
receta histórica. Los NPZ y resultados versionados sí están disponibles.

Guardar cada mapa explícitamente tras frenar y pausar, como en la guía principal.
Medir calidad y costo por separado: RMSE contra GT sobre timestamps comunes,
paredes/puertas/cobertura, descartes por causa, edad y cola de scans, P50/P95 del
callback completo, CPU y memoria. El original no publica los nuevos tiempos:
para comparar costo completo usar instrumentación equivalente o perfil externo;
no comparar su `update_seconds` del núcleo con el callback del candidato.

La condición de fusión sigue vigente: demostrar mejora del mapa, no solo CPU.
Mantener la rama experimental aislada hasta revisar resultados.
