# Contexto e instrucciones del TP Final

Actualizado: 16 de noviembre de 2026. Workspace: `/home/fran/TP_final`.

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
- Repositorio creado a pedido del usuario: **https://github.com/fortega00/TP_final**, privado, rama `main`, remoto `origin`. Mantener esa visibilidad salvo pedido expreso de publicación pública.
- Distinguir evidencia sintética de pruebas físicas. No afirmar calidad del aula o del bag real sin ejecutarlos.

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

Etapa B, paquete `nav` (rutas relativas a `nav/`). Decisiones y datos en `docs/decisiones/004` a `011`, ejecución en `docs/guia_etapa_b.md`, plan del robot real en `docs/robot_real_checklist.md`, bitácora de la sesión del 16/11 en `docs/sesion_robot_20261116.md` (robot en `172.23.16.78`, `ROS_DOMAIN_ID=11`, base `base_footprint`; mapa del aula elegido en `sesiones/20261116/mapa/intento3`, revisado solo a ojo). Falta la evasión de obstáculos no mapeados (008): hoy el robot solo frena.

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
- Incorporación por movimiento/tiempo: 0,08 m, 0,06 rad o 1 s.
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
