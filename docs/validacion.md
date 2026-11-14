# Alcance de la validación y corridas integrales

## 9. Alcance de la validación

| Prueba | Estado y alcance |
| --- | --- |
| Núcleo y utilidades | 31 pruebas unitarias aprobadas; cubren sensor, grilla, pesos/remuestreo, estados, tiempo, odometría y toy. |
| Benchmark sin ROS | Semillas 7, 11 y 19 aprobadas con los defaults; variantes de ruido muestran fallos de calidad documentados. |
| Build ROS 2 Humble | `colcon build` y carga de argumentos del launcher comprobados en la imagen Docker disponible del RosMaster. |
| ROS con toy y bag sintético | Tres integraciones aprobadas: toy con odometría de tópico, rosbag2 sintético con tópico y con TF. Cada una integró 118 scans sin descartes, con PGM idéntico; servicios, reset y cierre limpio comprobados. [Evidencia](../slam/artifacts/ros/README.md). |
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

El procedimiento, las limitaciones de cada métrica y los resultados están en [experimentos_a.md](experimentos_a.md). El recorrido guiado se aprobó para la evidencia de A; sus puntos sólo alimentan al controlador de prueba. El filtro sigue comenzando con mapa desconocido. `ideal` y `motion_bias=false` se mantienen en café.

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
  --config "$(ros2 pkg prefix slam)/share/slam/config/cafe.yaml" \
  --route /tmp/tp_final_cafe_route.yaml --max-sim-seconds 900 \
  --output /tmp/entrega_a/cafe --domain 96 --display :96
```

El ejecutor inicia su pantalla Xvfb y RViz, conduce en la simulación, exporta el mapa y guarda una captura al terminar el recorrido. Sus estados y puntos se registran en las muestras. Si se omite `--route`, utiliza el seguidor izquierdo experimental, que no garantiza cobertura.

En otra sesión independiente puede ejecutarse el bag real, cuya ruta debe existir **dentro** del contenedor:

```bash
python3 /tmp/tp_final_run_a.py --mode bag \
  --config "$(ros2 pkg prefix slam)/share/slam/config/parcial_bag_tuned.yaml" \
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
