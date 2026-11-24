# Contexto e instrucciones del TP Final

Actualizado: 18 de noviembre de 2026. Workspace: `/home/fran/TP_final`.

## Organización y consignas

- Grupo de tres integrantes. El TP tiene etapas A, B y C progresivas; cada entrega incluye paquetes ROS con launchfiles e informe.
- Leer las consignas originales `consignas/PRA_TPFinal_Parte_A.pdf`, `consignas/PRA_TPFinal_Parte_B.pdf` y `consignas/PRA_TPFinal_Parte_C.pdf`. La documentación del proyecto no sustituye sus requisitos.
- Robot: Yahboom RosMaster X3 con ruedas mecanum. **ROS 2 Humble**, confirmado por el usuario. Sensores disponibles: LIDAR, cámara de profundidad y RGB.
- **A:** construir mapa de ocupación y estimar pose con sensores/odometría; evaluar primero simulación, después rosbag real y finalmente robot vivo. Se exige geometría nítida, cobertura completa, resolución y utilidad para rutas seguras. Exportar mapa real e incluir las diez respuestas del PDF en el informe.
- **B:** localización probabilística en mapa conocido, `/initialpose` (`PoseWithCovarianceStamped`), `/goal_pose` (`PoseStamped`), planificación, control, orientación final, nuevos objetivos, obstáculos no mapeados, recuperación y máquina de estados.
- **C:** el PDF ya está disponible: robot real, exploración autónoma y búsqueda selectiva de conos rojos. Las detecciones se envían al planificador; ver un cono por una abertura no implica paso transitable. Requiere video e informe.

## Decisiones y restricciones del usuario

- La primera implementación aborda **A** con **FastSLAM, grillas de ocupación, log-odds y modelo inverso del sensor**, influida por *Probabilistic Robotics* de Thrun, Burgard y Fox.
- **No usar bibliotecas que resuelvan SLAM o algoritmos centrales similares.** Filtro, movimiento, likelihood, modelo inverso, grilla, pesos, ESS, remuestreo y estados son propios. NumPy/PyYAML resuelven cálculo general/configuración; ROS aporta transporte, TF y visualización. No sustituir el núcleo por GMapping, Cartographer, SLAM Toolbox, filtros o planificadores ajenos.
- Mantener módulos y máquina de estados explícita. Permitir propuestas con scan matching y sensores adicionales sin mezclarlos con ROS.
- Mantener `docs/guia_ejecucion.md` y `docs/parametros.md` con comandos para los cuatro modos e hiperparámetros completos con defaults reproducibles. Actualizar este archivo al cambiar interfaces, decisiones, comandos o validaciones.
- Repositorio creado a pedido del usuario: **https://github.com/fortega00/TP_final**, rama `main`, remoto `origin`, originalmente privado. El 18/11 GitHub informó visibilidad pública y el usuario autorizó explícitamente «publicar en el repositorio público» los cambios y artefactos del PR de E16. Mantener la visibilidad pública actual salvo nuevo pedido; no cambiarla automáticamente.
- Distinguir evidencia sintética de pruebas físicas. No afirmar calidad del aula o del bag real sin ejecutarlos.
- El usuario ejecuta personalmente las pruebas y observa RViz/Gazebo. No lanzar pruebas, replay ni simuladores por cuenta propia; ayudar con comandos y analizar sus resultados cuando lo pida. Solo si el usuario lo pide explicitamente entonces se pueden ejecutar pruebas en RViz/Gazebo/Ros etc.

- El usuario ejecuta las próximas comparaciones de Café en RViz/Gazebo. Preparación del 18/11: perfiles `cafe_compare_8`, `cafe_compare_16`, `cafe_compare_16_prior` y `cafe_compare_16_sample`, todos completos y sin cambiar `cafe.yaml`. Matriz, selección de revisiones por PYTHONPATH y captura en `docs/comparacion_cafe_manual.md`. No se autoriza lanzar estas pruebas durante la preparación; no fusionar la rama salvo mejora demostrada del mapa.
- Autorización posterior explícita: implementar y ejecutar el ciclo colaborativo Café 01, cinco replays del núcleo con semilla 7 y análisis v2 (inflación 0,22 m, extrínseca del NPZ, GT común y series de tiempo/ESS). Detenerse al presentar mapas para evaluación humana; no ejecutar ROS1×, simulaciones ni otros ciclos automáticamente. Fuentes/resultados en esta rama experimental; no fusionar por CPU. Procedimiento en `docs/ciclo_hitl_cafe_01.md`.
- Cierre original Café 01, anterior a la selección de D documentada abajo: cinco casos completos, 1458 integraciones cada uno y cinco tests analíticos de métricas aprobados. A/B originales/optimizados con ocho partículas idénticos bit a bit; 136,50→51,83 s. RMSE GT común A/B 15,31 cm; C16 9,68 cm; D16 prior_xy=0,025: 6,12 cm y P95 núcleo 79,7 ms; E16 sample_xy=0,005: 9,08 cm. D mejora métricas, E tiene posible discontinuidad en puerta: no elegir finalistas hasta inspección humana. Evidencia íntegra en `slam/artifacts/hitl_cafe_01/`. Detenerse aquí; sin merge, nuevas semillas, ROS ni nuevo ciclo.
## Implementación

Un paquete ROS 2 por etapa. La etapa A es el paquete `ament_python` **`slam`**, en la carpeta `slam/`. Adentro, `slam/slam/core/` tiene los algoritmos y no importa ROS, `slam/slam/ros/` tiene los nodos y `slam/slam/toy/` el mundo sintético. Las etapas B y C van en `nav/` y `mission/` con la misma separación `core/` y `ros/`, y pueden importar de `slam.core`. Las rutas de la tabla son relativas a `slam/`.

Comentarios y docstrings en español; identificadores, mensajes de error y commits en inglés.

| Ruta | Responsabilidad |
| --- | --- |
| `slam/core/config.py`, `config/default.yaml` | Configuración validada; defaults idénticos. |
| `config/cafe.yaml` | Perfil de grilla 16 × 32 m para el mundo café de `custom_map`. |
| `slam/core/geometry.py`, `slam/core/types.py` | SE(2), `Pose2D`, `Scan` con timestamp y extrínseca. |
| `slam/core/motion.py` | Movimiento holonómico y protocolo de propuestas. |
| `slam/core/sensor.py` | Haces, likelihood de extremos y protocolo de medición. |
| `slam/core/grid.py` | Grilla independiente, modelo inverso y máscara observada. |
| `slam/core/filter.py` | RBPF, log-pesos, ESS, remuestreo y selección coherente. |
| `slam/core/state_machine.py` | `WAITING`, `RUNNING`, `PAUSED`, `ERROR`, `STOPPED`. |
| `slam/core/io.py` | PGM/YAML trinarío compatible con map server. |
| `slam/core/odometry.py`, `slam/ros/ros_node.py` | Interpolación, TF, colas, publicaciones y servicios. |
| `slam/ros/tf_filter.py` | Filtra TF de localización obsoletos de bags. |
| `slam/toy/world.py`, `slam/toy/cli.py`, `slam/ros/toy_node.py` | Toy analítico, benchmark y publicador ROS. |
| `launch/slam.launch.py` | Launcher único para toy/simulation/bag/real. |
| `worlds/office.sdf`, `rviz/slam.rviz` | Oficina Gazebo y visualización. |
| `tests/`, `scripts/validate_ros.sh` | Pruebas de núcleo e integración ROS aislada. |
| `scripts/sync_container.sh` | Copia y construcción del paquete en el contenedor habitual del RosMaster. |
| `scripts/tune_toy.py`, `artifacts/` | Ablaciones y evidencia. |
| `scripts/run_a_experiment.py`, `scripts/cafe_route.yaml` | Ejecutor ROS aislado y recorrido guiado de evaluación, separado del SLAM. |
| `scripts/analyze_a_experiment.py`, `scripts/replay_bag_core.py` | Métricas y screening del núcleo sobre entradas reales alineadas. |
| `../docs/experimentos_a.md` | Procedimiento y resultados de café y del bag real del Parcial. |
| `../docs/guia_ejecucion.md`, `../docs/parametros.md`, `../docs/architecture.md`, `../docs/entrega_a.md` | Ejecución, parámetros, arquitectura e informe con diez respuestas. |

Etapa B, paquete `nav` (rutas relativas a `nav/`). Decisiones y datos en `docs/decisiones/004` a `012`, ejecución en `docs/guia_etapa_b.md`, plan del robot real en `docs/robot_real_checklist.md`, bitácora de la sesión del 16/11 en `docs/sesion_robot_20261116.md` (robot en `172.23.16.78`, `ROS_DOMAIN_ID=11`, base `base_footprint`; mapa del aula elegido en `sesiones/20261116/mapa/intento3_corregido`, sin medidas con cinta). El nodo de localización informa `LOST` si más de `localization.lost_fraction` (25 %) del scan cae a más de 20 cm de toda pared, o más de `click_lost_fraction` (15 %) justo después del click (decisión 004). Quedan dos visitas al laboratorio como máximo, repartidas en `docs/robot_real_checklist.md`. Evasión de obstáculos no mapeados implementada (008): capa aparte en `nav/core/obstacles.py`, el mapa estático no se modifica. La parada de seguridad (`navigator.stop_distance`) bajó de 0,40 a 0,25 m porque impedía llegar a objetivos cerca de una pared (008).

| Ruta | Responsabilidad |
| --- | --- |
| `nav/core/config.py`, `config/cafe.yaml` | Configuración validada. Los grupos `sensor`, `motion` y `scan_match` son los de `slam.core`. |
| `nav/core/map.py` | Carga PGM/YAML trinario como `OccupancyGrid` de `slam.core`. |
| `nav/core/localization.py` | MCL sobre mapa conocido: ráfaga inicial, compuerta de movimiento, ESS, inyección, scan matching opcional. |
| `nav/core/planner.py` | Mapa de costos (desconocido es obstáculo, inflación dura, costo de cercanía) y A*. |
| `nav/core/follower.py` | Pure pursuit con avance y giro, y giros en el lugar. |
| `nav/core/navigator.py` | Máquina de estados de la navegación y parada de seguridad. |
| `nav/ros/localization_node.py`, `nav/ros/navigator_node.py`, `launch/nav.launch.py` | Nodos y launch. `/nav/pose`, `/nav/plan`, `/nav/state`, `/nav/localization_status`. |
| `scripts/bag_to_npz.py`, `scripts/replay_localization.py` | Extraen un bag y corren la localización sin ROS sobre la grabación. |
| `scripts/mark_repeatability.py`, `scripts/compare_trajectories.py` | Medidas sin pose real: repetibilidad sobre marcas del piso y comparación con el SLAM. |
| `scripts/sim_live_check.py`, `scripts/chairs_sim.py` | Prueba en vivo contra la pose real del simulador y prueba sintética de sillas movidas. |
| `tests/` | `PYTHONPATH=../slam:. python3 -m unittest discover -s tests` |

Etapa C, paquete `mission` (rutas relativas a `mission/`), en desarrollo. Exploración sobre el mapa conocido con puntos de observación, detector de conos rojos con OpenCV solo para operaciones genéricas de imagen, y envío del cono confirmado al navegador de B por `/goal_pose`; no modifica `nav`. Tests: `PYTHONPATH=../slam:../nav:. python3 -m unittest discover -s tests`. `slam/scripts/sync_container.sh` copia y construye también `mission`.

| Ruta | Responsabilidad |
| --- | --- |
| `mission/core/throttle.py` | Baja la frecuencia de la cámara y acompaña cada cuadro de color con la profundidad más cercana en el tiempo. |
| `mission/ros/camera_throttle_node.py`, `launch/camera_throttle.launch.py` | Republica `/cam_1/...` bajo `/mission/cam/...` (3 Hz por defecto) para grabar `bag_conos` sin saturar el wifi. Corre sin compilar con `python3 -m mission.ros.camera_throttle_node`. |
| `mission/core/config.py`, `config/real.yaml` | Configuración validada de la misión. El mapa de costos no se duplica: se usa el grupo `planner` del YAML de `nav`. |
| `mission/core/exploration.py` | Puntos de observación sobre el mapa conocido (cobertura codiciosa con visibilidad 2D), orden del recorrido con distancias por zona transitable y `CoverageTracker` para girar solo hacia lo no visto. Mesas y bancos (el lidar ve solo patas y bordes) se detectan del mismo mapa en `prepare_map`: las paredes (obstáculos que tocan el exterior desconocido) se separan de los muebles, y solo los muebles pasan por un cierre morfológico `clutter_radius=0.65`; los puntos guardan holgura `viewpoint_clearance=0.50` (`config/real.yaml`). Sin intervención manual. Decisión 014. En el aula: 11 puntos, 98,9 % y 35,7 m. |
| `mission/core/mission.py` | Máquina de estados de la misión (`WAIT_LOCALIZATION`, `EXPLORE`, `LOOK_AROUND`, `CONFIRM`, `APPROACH`, `DONE`, `EXHAUSTED`). Manda objetivos al navegador de B; el resultado de cada objetivo se deduce de la geometría cuando `/nav/state` vuelve a `IDLE`, sin leer el texto de los eventos. Decisión 015. |
| `scripts/plot_viewpoints.py`, `artifacts/` | Dibuja el plan sobre un mapa (matplotlib, `requirements-evaluation.txt`). |

### Invariantes

- Cada partícula mantiene mapa y trayectoria ancestral propios. El remuestreo no comparte arrays mutables.
- Evaluar `z[t]` sobre `m[t-1]` **antes** de escribirlo. Normalizar log-pesos de forma estable.
- Conservar un snapshot elegido antes de remuestrear; mapa, pose y path deben corresponder a la misma hipótesis. No promediar grillas ni seleccionar arbitrariamente un descendiente con pesos reiniciados.
- Actualizar `L += logit(p_inverso) - logit(prior)` y saturar. Unknown es una máscara separada del prior.
- Contar una celda como máximo una vez por scan; impacto prevalece sobre libre. NaN, cero, negativos e inválidos no aportan evidencia. El alcance máximo finito despeja sin inventar impacto. `+inf` no aporta evidencia salvo con `sensor.no_return_is_free: true` (simulación), donde despeja igual que el alcance máximo.
- Un retorno exterior no crea paredes en el borde. Grilla finita; diagnosticar truncamiento y ampliar/resetear.
- Conservar movimiento lateral y marcha atrás. `alpha` parametriza varianzas del incremento SE(2).
- Interpolar odometría al timestamp del primer haz y usar extrínseca `base → laser`; no sustituir TF faltante por identidad ni usar TF reciente para ponderar una lectura pasada.
- Acumular movimiento desde el último scan integrado incluso con omisiones/pausa. Saltos de reloj hacia atrás requieren reset.
- La propuesta distribuida usa scan matching propio opcional. La interfaz admite corrección `log(p/q)`, pero `ScanMatchProposal` devuelve cero: es una aproximación documentada en 002 y un hallazgo pendiente de la revisión del 16/11, no FastSLAM 2.0 exacto. La propuesta odométrica sigue disponible.

### Defaults ajustados con el toy

- Grilla 16 × 16 m, origen `(-8,-8)`, celda 0,05 m, prior 0,5.
- 8 partículas con propuesta de scan matching (`docs/decisiones/002-propuesta-con-scan-matching.md`), semilla 7, remuestreo con ESS < 0,5 N; pose inicial `(0,0,0)` sin dispersión.
- Sensor: `p_hit=0.70`, `p_free=0.35`, log-odds `[-4,4]`, espesor 0,08 m, rango 0,12–8 m, 180 haces de mapa/60 de likelihood, sigma 0,15 m, mezcla 0,95/0,05, distancia máxima 0,60 m, temperatura 1, ocupado 0,65.
- Movimiento: `alpha1=0.0001`, `alpha2=0.00001`, `alpha3=0.0001`, `alpha4=0.0001`.
- Incorporación solo por movimiento: 0,08 m o 0,06 rad (`docs/decisiones/012-integracion-con-el-robot-quieto.md`).
- Valores calificados sólo para el toy; **no calibrados en robot real**. Todas las opciones y fórmulas están en `docs/parametros.md`.
- El usuario pidió conservar el perfil cinemático `ideal`. El 10/11 aprobó usar un recorrido guiado para cerrar la prueba de café, después de que el seguidor izquierdo necesitara recuperaciones. No agregar `stress` ni sesgos de movimiento a esa sesión.
- El perfil café usa `sensor.min_range=0.25`: se observaron retornos del propio chasis a 0,14–0,20 m. Los defaults generales permanecen en 0,12 m.

## Ejecución

Sin ROS, desde `slam/`:

```bash
python3 -m pip install -r requirements.txt
python3 -m unittest discover -s tests -v
python3 -m slam.toy.cli toy --config config/default.yaml --output artifacts/toy --seed 7 --check
python3 scripts/tune_toy.py --output artifacts/tuning
```

Instalar/enlazar esta carpeta como paquete dentro de `src/` de un workspace. Ejecutar `colcon build --symlink-install --packages-select slam` y cargar ROS y overlay en cada terminal. Ver preparación completa en `docs/guia_ejecucion.md`.

### Contenedor habitual del usuario

El usuario trabaja entrando con `/home/fran/PRA_00/container.sh enter` al contenedor existente `yahboom_rosmaster`, como `alumno`, en `/home/alumno/rosmaster_ws`. El montaje actual sólo enlaza `/home/fran/PRA_00/pkg` con `src/pkg`; **`TP_final` no está montado**. Para incorporar cambios sin recrear el contenedor ni tocar los otros TPs:

```bash
# En el host
cd ~/TP_final
./scripts/sync_container.sh
~/PRA_00/container.sh enter

# Dentro del contenedor
cd ~/rosmaster_ws
source install/setup.bash
ros2 launch slam slam.launch.py mode:=toy rviz:=true output_path:=/home/alumno/rosmaster_ws/maps/slam_toy
```

El script copia sólo archivos del paquete a `~/rosmaster_ws/src/slam` y construye únicamente ese paquete por defecto. `--sync-only` omite la construcción; luego ejecutar `colcon build --symlink-install --packages-select slam` dentro del workspace. Tras editar los fuentes del host hay que resincronizar, construir y cargar de nuevo el overlay. El 10 de noviembre de 2026 se verificaron sincronización y build en el contenedor habitual: colcon aprobó, el prefijo instalado es `/home/alumno/rosmaster_ws/install/slam` y ROS enumeró los cuatro ejecutables. No se lanzaron nodos, RViz ni Gazebo durante esa comprobación; la evidencia de ejecución anterior corresponde al contenedor aislado.

Guardar con `/slam/save_map` genera PGM/YAML/JSON en `~/rosmaster_ws/maps` dentro del contenedor. Esa salida **no se persiste automáticamente en el host** y se pierde si se elimina el contenedor. Para probar Gazebo, detener el toy con Ctrl+C y ejecutar `ros2 launch slam slam.launch.py mode:=simulation rviz:=true output_path:=/home/alumno/rosmaster_ws/maps/slam_simulation`. No ejecutar ambos modos a la vez porque comparten sensores, reloj y TF. La guía completa está en `docs/guia_ejecucion.md`.

### Modos del launcher

```bash
ros2 launch slam slam.launch.py mode:=toy rviz:=false output_path:=/tmp/slam/toy
ros2 launch slam slam.launch.py mode:=simulation world:=/ruta/mundo.sdf
ros2 launch slam slam.launch.py mode:=bag bag_path:=/ruta/bag bag_rate:=0.5
ros2 launch slam slam.launch.py mode:=real base_frame:=base_link
ros2 service call /slam/save_map std_srvs/srv/Trigger '{}'
```

`simulation start_simulator:=false` se conecta a un simulador existente. Para iniciar Gazebo usa directamente `yahboom_rosmaster_gazebo/rosmaster_gazebo_fortress.launch.py`, Fortress, mundo configurable, perfil `ideal` y `motion_bias=false`. El backend/driver es externo. La exploración A se hace con teleop externo; este paquete no publica velocidades.

Para el café de los TPs anteriores, ver la receta de `docs/guia_ejecucion.md` en «Probar el mundo café de custom_map». Resincronizar antes para instalar `config/cafe.yaml`. Dentro del contenedor, el share verificado de `custom_map` es `/home/alumno/rosmaster_ws/install/custom_map/share/custom_map`: existen `worlds/cafe.world` y `models`. Agregar `models` a `IGN_GAZEBO_RESOURCE_PATH` y `GZ_SIM_RESOURCE_PATH`, preservando los valores previos, y lanzar `slam` con ese world y el YAML café instalado. La grilla tiene ancho 16 m, alto 32 m, origen `(-8,-16)` y celda 0,05 m; la grilla toy truncaría el café en Y. Se verificaron rutas y assets sin ejecutar el mundo. Detener toy/simulaciones anteriores y no usar `custom_map/cafe.launch.py`, que agrega map server y TF fijo incompatibles con SLAM.

Transporte por defecto: `/scan`, `/odom`, `odom_source=topic`, frames `map`, `odom`, **`base_footprint`**. Para robot real seleccionar los frames efectivos. Otro `child_frame_id` requiere TF hacia la base; `/calc_odom` con child `calc_base` no es intercambiable sólo cambiando el tópico. Alternativa `odom_source:=tf`.

`use_sim_time` automático (false en real, true en los demás). En bag se reproducen únicamente scan, odometría y TF, y el player publica reloj. Un relay elimina la localización grabada; **no se reproduce `/cmd_vel`**. No ejecutar otro map server ni autoridad `map → odom`. Extrínsecas auxiliares sólo con `publish_scan_tf:=true` y valores calibrados; `publish_odom_tf:=true` sólo si falta ese enlace.

Salidas: `/map` transient-local, `/slam_pose` (`PoseWithCovarianceStamped`), `/particles`, `/path` ancestral, `/slam/status` JSON y TF `map → odom`. Trigger: `/slam/save_map`, `/slam/pause`, `/slam/resume`, `/slam/reset`. Reset descarta la sesión; guardar primero. Pausar mapeo no frena al robot.

Exportación `<output_path>.pgm`, `.yaml`, `.json`: libre p<0,25, ocupado según umbral configurable, resto desconocido. Para B aplicar inflación por huella; el mapa exportado no viene inflado.

## Entorno y evidencia

- Host Python 3.10, NumPy 2.2.6, PyYAML 5.4.1; sin ROS en host.
- Imagen Docker existente `yahboom_rosmaster:latest`: Humble, NumPy 1.21.5, colcon y Fortress. `./scripts/validate_ros.sh` valida en contenedor aislado, fuentes read-only, resultados aparte y red deshabilitada. No modifica imagen ni otros TPs.
- Se consultaron `/home/fran/TP_Parcial` y `/home/fran/PRA_00` para convenciones. `custom_map cafe/real` arranca map server y TF fijo; no incluirlo directamente. No se modificaron trabajos anteriores ni se adoptaron sus instrucciones locales como reglas para este workspace.
- 31 pruebas unitarias aprobadas en host/contenedor; `colcon build`, argumentos del launcher y SDF aprobados.
- Toy semilla 7: 119 scans/118 integrados; precision a una celda 0,9032, recall 1, RMSE 2,46 cm, cierre 1,82 cm, conectividad 100 % con radio 22 cm + margen 5 cm. Odometría sola RMSE 2,01 cm: no afirmar mejora de pose en esa corrida. IoU exacta 0,3642 expone diferencias de espesor/alineación.
- Semillas 11 y 19 también aprobaron sin retuning (RMSE 2,88/2,37 cm). Ablaciones con más ruido deterioran paredes y fallan precisión; conservar esos fallos.
- Scans toy instantáneos; paso 0,28 m/período 0,25 s es muestreo, no ensayo físico de velocidad/intrabarrido.
- Integración ROS aprobada en tres modos: toy con odometría por tópico, bag sintético por tópico y bag sintético por TF. Cada uno integró 118 scans sin descartes y exportó PGM idéntico. Servicios, transiciones inválidas, reset, filtro de TF/cmd_vel y cierre limpio comprobados; evidencia en `artifacts/ros/`. Ejecutar `./scripts/validate_ros.sh` para reproducir.
- El 10/11 se encontró el **bag físico original** del Parcial en `/tmp/real_rosbag_input` dentro del contenedor habitual: 163,858 s, 1188 scans y 1633 odometrías. Su DB3 pesa 511.655.936 bytes; SHA256 `3cc3858ea91a8c866e7b60f7c2cd2d01b6356dae7a680ade7885335d6cd2cc86`. No tiene GT; el cierre no es error absoluto. No cargar el mapa conocido del Parcial en el filtro de SLAM.
- Corrida ROS inicial del bag: 857 scans integrados, 331 omitidos por movimiento, cero descartes; separación inicio–fin SLAM 17,738 m y odometría 17,855 m. El mapa pierde paredes y **no aprueba A**. Evidencia en `artifacts/entrega_a_20261110/rosbag/`.
- Screening real sin ROS, semilla 7: 64 partículas/alpha base 0,03/sigma 0,15 dejó separación 2,365 m y sólo 16,19 % de extremos a 10 cm; 128 partículas/alpha base 0,10/sigma 0,15 dejó 0,880 m y 54,94 %. Mayor persistencia inversa con 96 partículas dejó 1,282 m y 61,44 %, pero engrosó/deformó paredes. No elegir sólo por cierre ni presentar screening como validación ROS.
- **Sesiones ROS completas:** evidencia persistida en el host, `artifacts/entrega_a_20261110/cafe/` y `rosbag_tuned/`; ambas tienen `rviz.png`, PGM/YAML, muestras y `metrics.json`. Café: dominio/display 96, recorrido guiado completo, RMSE 2,063 cm, error final 1,300 cm, cobertura SDF aproximada observada 99,863 %, libre 99,819 %, conectividad visitada 74/74, cero descartes tras reset. Odometría ideal RMSE 0,040 mm: no afirmar mejora de pose. El PGM al cerrar agregó un scan estacionario respecto del path capturado (1467/1466 integraciones, 1 s); el ejecutor actual pausa antes de exportar.
- Bag ajustado: dominio/display 97, 128 partículas, alpha `(0,10;0,01;0,10;0,10)`, sigma 0,15, p_hit 0,75, p_free 0,40, espesor 0,08. Reproducción 0,2× reducida a 0,15×; 856 scans integrados, 325 omitidos, siete descartes iniciales por TF, cola final cero. Separación inicio–fin SLAM 0,842 m vs odometría 17,855 m. Ajuste interno: 82,03 % de extremos a 10 cm, mediana 2,66 cm, P95 55,48 cm. Conectividad visitada 42/43. Mejora clara, pero persisten discontinuidades/ruido: **no certificar mapa definitivo ni A completamente aprobada**. Comparación con mapa anterior sólo posterior, sin usarlo en SLAM y sin tratarlo como GT.
- Ambas sesiones cerraron Xvfb, launcher y RViz con código cero. 31 unit tests aprobados. Scripts y reproducción en `A.md` sección 10; análisis y límites en `docs/experimentos_a.md`. Los fuentes del ejecutor usado están archivados en cada corrida; el ejecutor actual mejora espera inicial de TF, buffer del observador y congelación final.
- **Pendientes:** resolver/calificar discontinuidades del mapa del bag, calibración física y mapeo vivo del aula. Los pasos de ejecución y medición de simulación/bag están documentados; no confundir completar corridas con cumplir toda la calidad geométrica de A.

## Límites y próximos pasos

Sin deskew, fusión profundidad/RGB/IMU, relocalización global robusta, optimización retrospectiva ni mapas dinámicos/3D. Scan matching aproximado implementado; navegación B parcial y C pendiente. Revisitar puede seleccionar una hipótesis superviviente; no reconstruye trayectorias eliminadas ni reajusta poses pasadas. Memoria densa O(N·área/resolución²), más trayectorias crecientes.

Antes de B/C registrar sesiones físicas y verificar puertas, paredes, cobertura y huella. El mapa toy no certifica la geometría del laboratorio. Consultar `docs/entrega_a.md` para fundamentos y respuestas.

## Revisión del 16/11/2026

- Comparación `18a7784..1c14cf2`: 14 commits de Iván; HEAD confirmado igual a `origin/main` mediante consulta remota. Resumen, hallazgos y recetas actualizadas en `docs/revision_20261116.md`. No se modificaron algoritmos durante la revisión.
- Desde la raíz del repo el sincronizador es `./slam/scripts/sync_container.sh`; copia y construye `slam` y `nav`. Los scripts de experimentos están en `slam/scripts/` y no se copian al contenedor con el sincronizador. Las rutas de scripts/artifacts de secciones históricas deben interpretarse respecto de `slam/`.
- SciPy es una nueva dependencia declarada y necesaria aun con matcher desactivado por los imports actuales. La imagen Humble disponible incluye SciPy 1.8.0.
- Defaults Python y `slam/config/default.yaml` difieren: Python conserva 24 partículas, matcher false, espesor 0,03 y sigma 0,10; YAML usa 8, true, 0,08 y 0,15. Cargar perfiles completos para reproducir. No declarar defaults idénticos hasta corregirlo.
- Host aislado: 37 tests A y 28 tests B aprobados. Toy default semilla 7: RMSE 2,915 cm, precisión a una celda 0,8361, recall 1, conectividad 100 %. No reemplaza la evidencia anterior de otra configuración.
- Hallazgo B: sin LIDAR pero con odometría, la pose publicada mantiene timestamps recientes y el navegador no controla edad del scan/corrección. Corregir y verificar antes de navegación física. Evasión de obstáculos nuevos sigue pendiente.
- Evidencia de revisión en `slam/artifacts/review_20261116/`; incluye fallos y resultados de las corridas. El validador ROS original aprobó toy y bag sintético por tópico, pero falló bag por TF (114 de 118 integraciones). Repetición a 2× también falló bajo carga; una variante a 0,5× aprobó con 118 integraciones, servicios/reset y PGM idéntico a toy/bag tópico. No generalizar aprobación del validador original.
- Se conservaron los resultados originales del 10/11 y no se arrancó ni modificó el contenedor habitual desde la revisión (al inicio estaba detenido; al cierre se observó activo por una acción externa). Las pruebas nuevas se ejecutan en un contenedor temporal separado.

- Bag nuevo con scan matching: 857 integraciones, 331 omitidos, cero descartes, cola final cero, separación inicio–fin 0,705 m, consistencia a 10 cm 95,03 %, conectividad 42/43. Replay del núcleo y ROS exportan PGM idéntico. Sin GT; no certifica A.
- Café Fortress ideal nuevo: 1438 integraciones, cero descartes tras reset, RMSE 16,330 cm contra Gazebo frente a 2,063 cm anterior; cobertura observada aproximada 99,837 %, conectividad 69/72. Regresión del perfil nuevo en ideal. Terminó por límite de 600 s, 31/32 waypoints; no afirmar recorrido completo ni error de cierre. La receta futura usa 660 s (no ejecutada en esta revisión).
- B en mapa propio del mismo bag: 839 actualizaciones, ráfaga de tres iteraciones, RMSE contra SLAM 8,1 cm tras ajuste rígido; coherencia interna, no exactitud. Ambos nodos arrancan; cierre limpio con SIGINT al launcher, interrupciones en destroy_node con SIGINT al grupo entero.

## Café manual del 18/11/2026

- Ejecutado por el usuario con teleop, perfil ideal y `cafe.yaml` actual. Exportación y análisis estático persistidos en `slam/artifacts/cafe_teleop_20261118/`; no se repitieron pruebas ni se cambiaron algoritmos/perfiles.
- Mapa visualmente coherente: 961 integraciones, 55 descartes del adaptador, ESS final 6,178/8. Concordancia de ocupados con cajas SDF aproximadas: 94,95 % a dos celdas (10 cm), 98,20 % a tres (15 cm). Puerta inferior 1,15 m frente a 1,09 m nominales. No certificar cobertura total por esta referencia ni RMSE de pose: no se guardaron trayectorias/bag de esta corrida.
- Usuario reportó subir consignas de teleop hasta 0,52 m/s y 1,04 rad/s; no son velocidades medidas ni un máximo sostenido validado. Última actualización 0,162 s, sin distribución temporal. Revisar sincronización antes de retocar el núcleo; no extrapolar velocidades de Fortress ideal al LIDAR físico sin deskew. Detalle, límites y propuestas sin aplicar en el README de la corrida.

## Worktree de profiling del 18/11/2026

- Pedido posterior explícito: commitear todo y abrir PR, incluyendo los cambios locales y la evidencia del worktree. Rama de integración `feat/slam-e16-evaluation-20261118`, basada también en `origin/main` con PR #3/#4. Los perfiles instalables `bag16_compare_*` eliminan `update.max_interval`; el perfil E16 seleccionado conserva N16/prior0.05/sample0.005 con compuerta solo por movimiento. Las métricas históricas siguen siendo anteriores a esa corrección, sin nueva validación del algoritmo actual. No modificar las capturas/configs archivadas. La decisión E16 pasa a 013 para reservar 012 a la compuerta de reposo ya incorporada en main. Publicación pública de estos cambios autorizada expresamente por el usuario; sin merge automático del PR ni nuevas pruebas ROS.

- Selección posterior a los ciclos bag16 01/02 y revisión visual («Se ven todos muy parecidos. Cambia el yaml por la config que mejor dio»): se adopta **E16** en `slam/config/parcial_bag_scan_match.yaml`, 16 partículas, `prior_xy=0.05`, `sample_xy=0.005`. El perfil anterior del 16/11 queda en sus exportaciones históricas. Frente a su replay8, E16 mejora modestamente endpoints reservados94,04→94,71 % y P95residuo12,16→10,72cm, pero cuesta más: P95núcleo observado72,5→122,2ms en ciclos con distinta carga. No afirmar más rapidez, exactitud física, mejora global de navegabilidad ni capacidad ROS1×. Numba requerido para reproducir tiempos; el YAML no activa el backend. Decisión/evidencia resumida en `docs/decisiones/013-perfil-real-e16.md` y `slam/artifacts/perfil_real_e16_20261118/selection.json`; evidencia completa en `exp/rosbag-20261116-cycle01` commit `68ebc13`. Se integra solo el perfil elegido y su documentación; no se fusiona toda la rama ni se autorizan nuevas corridas. Café D y demás perfiles permanecen separados.

Las restricciones de los cierres de ciclos siguientes son históricas, anteriores a la selección E16 y al pedido posterior de incluir el trabajo en un PR.

- Nueva autorización posterior al cierre del ciclo 01: el usuario solicita probar 24 partículas con los mismos ejes para mejorar el mapa. Ciclo bag16 02 en el mismo worktree, cuatro corridas semilla7: control E16 actual y F24 base/D24 prior0.025/E24 sample0.005. Numba obligatorio, mismo bag/NPZ, checkpoints e interior seed83 fijos del ciclo01. A/B/H reutilizados; no repetir otros estimadores. Detalle `docs/ciclo_hitl_bag16_02.md`. Solo núcleo aislado; detener para revisión humana, sin merge, nuevas semillas ni ROS1× automáticos.
- Cierre bag16 02: cuatro corridas completas, 963 integraciones cada una y backend Numba confirmado. E16 reproduce E anterior bit a bit; P95 actual122,2ms con menor carga. N24 F/D/E: endpoints reservados93,97/76,40/92,86 % frente a94,71 % E16; P95 núcleo185,6/188,7/201,3ms. No mejora de consistencia observada con24 en semilla7. D24 acorta separación inicio-fin y aumenta checkpoints admisibles, pero empeora residuos: requiere revisión humana de geometría, no selección automática. Ocho tests de métricas aprobados, contenedor aislado detenido. Evidencia `slam/artifacts/hitl_bag16_02/`; sin perfil físico adoptado, merge ni nuevas semillas/ROS1×.

- Pedido posterior explícito: evaluar el bag del 16/11 mientras el usuario prueba D en Café, siguiendo los criterios anteriores y en un worktree aparte. Ciclo acotado en `exp/rosbag-20261116-cycle01`, `/tmp/tp-final-bag-20261116-cycle01`, cinco casos A–E semilla 7 (original/optimizado8, optimizado16, prior0.025, sample0.005), entrada `bag_mapa2` y referencia histórica `intento3`. Solo núcleo headless, contenedor aislado y prioridad baja; no modificar el contenedor habitual ni el árbol principal. No adoptar perfiles físicos, fusionar, ejecutar nuevas semillas/ciclos o ROS1× sin revisión humana. Detalle en `docs/ciclo_hitl_bag16_01.md`.
- Cierre bag16 01: cinco casos completos, 963 integraciones y 2650 omisiones cada uno, A/B grilla/trayectoria/PGM idénticos. E mejora modestamente endpoints ≤10 cm 95,46→95,90 % y reservados 94,04→94,71 % (P95 12,16→10,72 cm); D casi sin mejora. No afirmar mejor mapa histórico global sin inspección humana/GT. Origen no admite huella22cm en H/A/B/C/D/E; diagnóstico interior común conecta todos los checkpoints admisibles, manteniendo el fallo del origen. Carga del host cambió: tiempos D/E altos no certifican tiempo real ni costo intrínseco de esos parámetros. Ocho tests analíticos de métricas aprobados; contenedor aislado detenido. Evidencia íntegra `slam/artifacts/hitl_bag16_01/`, revisión humana pendiente, sin merge ni perfil físico adoptado.

- Selección posterior del usuario tras Café 01: «Excelente, usemos D entonces». Se adopta D como perfil de trabajo de Café junto con las optimizaciones: `cafe.yaml` usa 16 partículas, `prior_xy=0.025`, `sample_xy=0.01`; default y perfiles físicos permanecen separados. La incorporación al árbol principal se apoya en la mejora medida y esa selección humana. No se autoriza otro ciclo ni pruebas ROS/físicas; faltan semillas adicionales y ROS1×. Registro humano en `slam/artifacts/hitl_cafe_01/report/comparison.json` y ejecución Numba en la guía.

- El usuario autorizó explícitamente al agente paralelo a ejecutar ROS2/perfilado, siempre en worktree y sin fusionar salvo mejora de estimación del mapa. Trabajo aislado en `perf/slam-profile-20261118`, `/tmp/tp-final-slam-profile-20261118`, contenedor propio sin red; no se modificó el árbol principal ni el contenedor habitual. Código `cb806b6`, sin merge ni push.
- Numba opcional compila bucles propios de matcher, modelo inverso y likelihood; NumPy permanece disponible. Se conserva contenido/frecuencia de trayectorias con una caché de mensajes. Diagnóstico por causa de descarte y descarte inmediato de scans anteriores a todo el historial odométrico; sin cambiar defaults de colas/timeouts/hiperparámetros. Detalles y requisitos en `docs/perfilado_20261118.md`.
- Café y bag16/11 completos en el núcleo: grilla cruda, trayectoria y PGM idénticos; aceleración 2,60×/2,16×. En ROS Café4×: descartes 512→31 de arranque, RMSE GT común15,64→15,31 cm; precision/recall geométricos SDF bajan ligeramente. **Conclusión histórica del profiling previo al ciclo 01:** la optimización sola no había demostrado mejora del mapa; la adopción posterior de D se documenta arriba. No hubo par ROS1× ni prueba física nueva. Evidencia en `slam/artifacts/profile_20261118/`; NPZ conservados; el bag crudo no reapareció al reconstruir el worktree temporal.

## Entorno virtual del contenedor, 18/11/2026

- A pedido del usuario se preparó Numba en el contenedor habitual, sin lanzar nodos, replays, RViz ni Gazebo. `./slam/scripts/sync_container.sh --with-numba` sincroniza/construye `slam` y `nav` y ejecuta la preparación; `./slam/scripts/container_env.sh setup|enter|status` gestiona el entorno por separado.
- Entorno `/home/alumno/rosmaster_ws/.venv`, Python 3.10 con `--system-site-packages` y `COLCON_IGNORE`. Numba 0.57.1, llvmlite 0.40.1 y pip 24.0 se instalan con ruedas descargadas en el host, sin pip global ni ensurepip en el contenedor. NumPy 1.21.5, SciPy 1.8.0 y PyYAML 5.4.1 siguen siendo los del sistema. No se modifica la imagen ni el helper de PRA_00.
- `.bashrc` carga `~/rosmaster_ws/.tp_env/scripts/tp_env.bash`; las terminales nuevas activan ROS, overlay y venv. En terminales existentes, hacer `source` de ese archivo. `tp-env on|off|numpy|status` permite activar, salir restaurando variables, forzar NumPy y consultar versiones/backend. `on` antepone el site-packages del venv a `PYTHONPATH` para los nodos con shebang `/usr/bin/python3`; caché en `~/rosmaster_ws/.cache/numba`. Cambiar backend requiere relanzar procesos existentes.
- Construcción de ambos paquetes e imports de Numba/llvmlite/rclpy aprobados; estado `SLAM backend: numba`. Se verificaron activación automática en Bash interactivo, backend NumPy con `tp-env numpy`, backend Numba desde `/usr/bin/python3` y retorno al Python del sistema con `tp-env off`. Esto verifica instalación, no una nueva corrida ni tiempos de ejecución. El entorno/caché no están montados en el host y se pierden al eliminar el contenedor. Detalles en `docs/guia_ejecucion.md` y `docs/parametros.md`.

## Auditoría de ruido y corridas manuales posteriores, 18/11/2026

- Antecedentes de ruido revisados desde archivos, sin nuevos replays ni pruebas: `docs/ruido_odometria.md`. Separar perturbación de la odometría de entrada y aumento de incertidumbre del filtro. Toy histórico sin matcher: RMSE 2,46→5,37 cm y precisión a 5 cm 90,32→67,05 % al aumentar ruido/sesgo de entrada. Los screenings del bag cambiaron simultáneamente partículas y `alpha`; no aíslan su efecto. La decisión 002 contiene más resultados reportados por Iván, con ejecutores en su repo personal. Falta evaluar D ante odometría degradada.
- El usuario reportó dos corridas nuevas de Café (lenta/rápida) y una con choque que debe excluirse. Se localizaron logs de launch de 21:16, 21:33 y 21:37 según el contenedor; sólo quedó la exportación `maps/cafe.{pgm,yaml,json}` de 21:42. Confirmó que es **la rápida** y que todas usaron el mismo `output_path`: la lenta quedó sobrescrita. No tratar la corrida manual de las 13:14 ni Café 01 como esa corrida lenta. Evidencia final copiada a `slam/artifacts/cafe_manual_20261118_fast/`.
- `output_path` fijo sobrescribe la exportación al guardar y al cierre limpio; no hay versionado de mapas. La guía agrega prefijos con nombre de caso y timestamp para comparaciones futuras. No se cambió el algoritmo ni el comportamiento del guardado.
- Análisis estático de la rápida, sin SLAM/ROS/replays/simulación: 95,12 % de ocupados a ≤10 cm de cajas SDF aproximadas; puerta inferior 1,05 m frente a 1,09 m nominales; libre navegable conectado 164,94 m² con radio 0,27 m y desconocido/exterior bloqueados. Faltan patas de mesa en (-1,5; -5,5) y (2,4; -9,0) m; otras dos aparecen desplazadas 13–15 cm. Se confirmó sección visual de las patas de 4,16 cm a altura del LIDAR. No certificar seguridad sólo por conectividad ni cobertura completa con las cajas, especialmente arriba donde difieren de las mallas.
- Rápida: 653 integraciones, 351 omisiones, 42 descartes por odometría de 1046 scans recibidos; ESS final 11,85/16. Mediana/P95 de núcleo 235/265 ms y `drain` 265/317 ms sobre las últimas 512 integraciones. Sensor configurado 5 Hz; falta factor de tiempo real/series para afirmar capacidad sostenida a 1×. Consignas aportadas hasta 1,73 m/s y 3,45 rad/s, sin velocidades medidas. No hay bag/trayectorias ni RMSE de pose. No se modificó D ni se inició otro ciclo. Informe y propuestas en el README de la corrida y `docs/experimentos_a.md`.

## Revisión de PR del 18/11/2026

- PR #3 de Iván: integración solo por movimiento y detección `LOST` por residuo. Al combinar con main se migraron también los cuatro `cafe_compare_*.yaml` para quitar `update.max_interval`; se documentó la migración de YAML externos. Una prueba de carga de todos los perfiles instalados detectó los cuatro fallos antes de corregirlos. Las evidencias históricas conservan su código/configuración originales; no son una validación nueva del perfil D con esta compuerta.
- Revisión sin nodos ROS, replays, RViz, Gazebo ni robot. PR #4 queda sin merge por la interacción entre `sensor.min_range=0.25` y `navigator.stop_distance=0.25` en Café: con la extrínseca registrada `(0.043, 0, pi)`, un retorno frontal a 0.20 m del láser (0.243 m del centro) no activa la parada.
- Validación del resultado de PR #3 + main: suite A, 53 tests (46 aprobados y 7 omitidos por falta de ROS/Numba); suite B, 29 aprobados. La combinación de PR #4 + main pasó 36 tests B, pero no cubrían el perfil Café con su extrínseca y el umbral de parada reducido.

## Corrección y aprobación del PR #4, 18/11/2026

- A pedido del usuario se corrigió el perfil de navegación Café: `nav/config/cafe.yaml` restaura `navigator.stop_distance=0.40`, conservando mínimo de 0,25 m y extrínseca registrada `(0.043, 0, pi)`. La parada actúa antes de la zona descartada; los retornos dentro del mínimo siguen sin aportar detección. El perfil real/default conserva parada 0,25 m y mínimo 0,12 m, pendiente de huella física. La mayor parada de Café conserva la limitación de objetivos cercanos a paredes.
- Cuatro regresiones usan el perfil Café y su extrínseca; los casos de parada fallaban antes de la corrección. Se documentan parámetros y límites en las guías de ejecución/B, parámetros y decisiones 008/009. Validación sin ROS, RViz, Gazebo, replays ni robot; no se actualizó el contenedor habitual.
- Resultado de la combinación con main: suite A, 53 tests (46 aprobados y 7 omitidos por dependencias ROS/Numba); suite B, 40 aprobados, incluidos los cuatro casos Café. `git diff --check` limpio.


## Parada de navegación por pérdida de LIDAR

- `navigator.scan_timeout`: 0,50 s con reloj monotónico de recepción en real, 0,45 s con tiempo simulado en Café. El sello del scan real solo ordena mensajes de la fuente. Usar `WAIT_SCAN` para el corte de avance y giro, con aviso en terminal, cancelación del objetivo y plan vacío. Al recuperar datos hace falta pose reciente localizada y un objetivo nuevo. Ver `docs/guia_etapa_b.md` y `docs/parametros.md`.
- El valor real es inicial. La calibración de jitter y recorrido de frenado se mide en el aula antes de usarlo para evaluar seguridad física. Un salto hacia atrás del reloj simulado requiere relanzar navegación y localización.

- Validación en vivo del watchdog en Café Fortress el 20/11: Humble, mapa D
  fijo, perfil Café, contenedor separado sin red. Corte del relay del scan
  durante `FOLLOWING`, con odometría y pose aún activas. `Twist` cero 0,50 s
  de simulación desde la captura del último scan y 0,409 s desde el envío del
  relay para ese scan. `WAIT_SCAN`, plan vacío, una advertencia y marcha solo
  al mandar otro objetivo. Build de ambos paquetes y 45 pruebas B aprobadas.
  Resumen en `nav/artifacts/lidar_cut_cafe/summary.json`. El margen de
  frenado físico sigue pendiente de medición.

- Sesión manual de Café del 20/11: dos objetivos cerca de `(-3,4; 8,3)`
  alcanzaron `ALIGNING` y se cancelaron con más de 1,1 s simulados sin un scan
  utilizable. Durante la repetición, un suscriptor independiente midió un
  intervalo máximo de 0,310 s de pared. En el mapa D archivado, el A* original
  tardó 1,30–1,43 s para esos objetivos desde un inicio aproximado. El nodo de
  navegación ahora ejecuta A* en otro hilo, con el robot quieto durante
  `PLANNING`, el watchdog activo y descarte de resultados obsoletos. Falta la
  repetición del usuario en Café Fortress con el cambio instalado.

- Diagnóstico `LOST` en marcha: el navegador publica las celdas confirmadas de su capa dinámica por `/nav/obstacle_cells`. El localizador consulta celdas cercanas solo para retornos sin explicar por el mapa fijo, con capturas de scans anteriores y recientes. `localization.min_static_match_fraction=0.50` exige apoyo del mapa fijo para usar la capa. Tras `/initialpose` se limpia esa evidencia y se mantiene el límite estático de 0,15. `/nav/localization_status` informa el residuo fijo y el usado para decidir. Falta medir latencia y verificar el caso manual de Café. Ver `docs/decisiones/004-localizacion-en-mapa-conocido.md`.
