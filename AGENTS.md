# Contexto e instrucciones del TP Final

Actualizado: 10 de noviembre de 2026. Workspace: `/home/fran/TP_final`.

## Organización y consignas

- Grupo de tres integrantes. El TP tiene etapas A, B y C progresivas; cada entrega incluye paquetes ROS con launchfiles e informe.
- Leer las consignas originales `PRA_TPFinal_Parte_A.pdf`, `PRA_TPFinal_Parte_B.pdf` y `PRA_TPFinal_Parte_C.pdf`. La documentación del proyecto no sustituye sus requisitos.
- Robot: Yahboom RosMaster X3 con ruedas mecanum. **ROS 2 Humble**, confirmado por el usuario. Sensores disponibles: LIDAR, cámara de profundidad y RGB.
- **A:** construir mapa de ocupación y estimar pose con sensores/odometría; evaluar primero simulación, después rosbag real y finalmente robot vivo. Se exige geometría nítida, cobertura completa, resolución y utilidad para rutas seguras. Exportar mapa real e incluir las diez respuestas del PDF en el informe.
- **B:** localización probabilística en mapa conocido, `/initialpose` (`PoseWithCovarianceStamped`), `/goal_pose` (`PoseStamped`), planificación, control, orientación final, nuevos objetivos, obstáculos no mapeados, recuperación y máquina de estados.
- **C:** el PDF ya está disponible: robot real, exploración autónoma y búsqueda selectiva de conos rojos. Las detecciones se envían al planificador; ver un cono por una abertura no implica paso transitable. Requiere video e informe.

## Decisiones y restricciones del usuario

- La primera implementación aborda **A** con **FastSLAM, grillas de ocupación, log-odds y modelo inverso del sensor**, influida por *Probabilistic Robotics* de Thrun, Burgard y Fox.
- **No usar bibliotecas que resuelvan SLAM o algoritmos centrales similares.** Filtro, movimiento, likelihood, modelo inverso, grilla, pesos, ESS, remuestreo y estados son propios. NumPy/PyYAML resuelven cálculo general/configuración; ROS aporta transporte, TF y visualización. No sustituir el núcleo por GMapping, Cartographer, SLAM Toolbox, filtros o planificadores ajenos.
- Mantener módulos y máquina de estados explícita. Permitir propuestas con scan matching y sensores adicionales sin mezclarlos con ROS.
- Mantener `A.md` con comandos para los cuatro modos e hiperparámetros completos con defaults reproducibles. Actualizar este archivo al cambiar interfaces, decisiones, comandos o validaciones.
- Repositorio creado a pedido del usuario: **https://github.com/fortega00/TP_final**, privado, rama `main`, remoto `origin`. Mantener esa visibilidad salvo pedido expreso de publicación pública.
- Distinguir evidencia sintética de pruebas físicas. No afirmar calidad del aula o del bag real sin ejecutarlos.

## Implementación

Paquete ROS 2 `ament_python` **`tp_slam`**, situado en la raíz del repositorio. El núcleo es importable directamente desde aquí sin ROS.

| Ruta | Responsabilidad |
| --- | --- |
| `tp_slam/config.py`, `config/default.yaml` | Configuración validada; defaults idénticos. |
| `tp_slam/geometry.py`, `types.py` | SE(2), `Pose2D`, `Scan` con timestamp y extrínseca. |
| `tp_slam/motion.py` | Movimiento holonómico y protocolo de propuestas. |
| `tp_slam/sensor.py` | Haces, likelihood de extremos y protocolo de medición. |
| `tp_slam/grid.py` | Grilla independiente, modelo inverso y máscara observada. |
| `tp_slam/filter.py` | RBPF, log-pesos, ESS, remuestreo y selección coherente. |
| `tp_slam/state_machine.py` | `WAITING`, `RUNNING`, `PAUSED`, `ERROR`, `STOPPED`. |
| `tp_slam/io.py` | PGM/YAML trinarío compatible con map server. |
| `tp_slam/odometry.py`, `ros_node.py` | Interpolación, TF, colas, publicaciones y servicios. |
| `tp_slam/tf_filter.py` | Filtra TF de localización obsoletos de bags. |
| `tp_slam/toy.py`, `cli.py`, `toy_node.py` | Toy analítico, benchmark y publicador ROS. |
| `launch/slam.launch.py` | Launcher único para toy/simulation/bag/real. |
| `worlds/office.sdf`, `rviz/slam.rviz` | Oficina Gazebo y visualización. |
| `tests/`, `scripts/validate_ros.sh` | Pruebas de núcleo e integración ROS aislada. |
| `scripts/sync_container.sh` | Copia y construcción del paquete en el contenedor habitual del RosMaster. |
| `scripts/tune_toy.py`, `artifacts/` | Ablaciones y evidencia. |
| `A.md`, `docs/architecture.md`, `docs/entrega_a.md` | Ejecución, arquitectura e informe con diez respuestas. |

### Invariantes

- Cada partícula mantiene mapa y trayectoria ancestral propios. El remuestreo no comparte arrays mutables.
- Evaluar `z[t]` sobre `m[t-1]` **antes** de escribirlo. Normalizar log-pesos de forma estable.
- Conservar un snapshot elegido antes de remuestrear; mapa, pose y path deben corresponder a la misma hipótesis. No promediar grillas ni seleccionar arbitrariamente un descendiente con pesos reiniciados.
- Actualizar `L += logit(p_inverso) - logit(prior)` y saturar. Unknown es una máscara separada del prior.
- Contar una celda como máximo una vez por scan; impacto prevalece sobre libre. NaN, cero, negativos e inválidos no aportan evidencia. `+inf` y alcance máximo despejan sin inventar impacto.
- Un retorno exterior no crea paredes en el borde. Grilla finita; diagnosticar truncamiento y ampliar/resetear.
- Conservar movimiento lateral y marcha atrás. `alpha` parametriza varianzas del incremento SE(2).
- Interpolar odometría al timestamp del primer haz y usar extrínseca `base → laser`; no sustituir TF faltante por identidad ni usar TF reciente para ponderar una lectura pasada.
- Acumular movimiento desde el último scan integrado incluso con omisiones/pausa. Saltos de reloj hacia atrás requieren reset.
- Propuesta actual odométrica, **sin scan matching ni FastSLAM 2.0**. Una propuesta diferente debe devolver corrección de importancia `log(p/q)`.

### Defaults ajustados con el toy

- Grilla 16 × 16 m, origen `(-8,-8)`, celda 0,05 m, prior 0,5.
- 24 partículas, semilla 7, remuestreo con ESS < 0,5 N; pose inicial `(0,0,0)` sin dispersión.
- Sensor: `p_hit=0.70`, `p_free=0.35`, log-odds `[-4,4]`, espesor 0,03 m, rango 0,12–8 m, 180 haces de mapa/60 de likelihood, sigma 0,10 m, mezcla 0,95/0,05, distancia máxima 0,60 m, temperatura 1, ocupado 0,65.
- Movimiento: `alpha1=0.0001`, `alpha2=0.00001`, `alpha3=0.0001`, `alpha4=0.0001`.
- Incorporación por movimiento/tiempo: 0,08 m, 0,06 rad o 1 s.
- Valores calificados sólo para el toy; **no calibrados en robot real**. Todas las opciones y fórmulas están en `A.md`.

## Ejecución

Sin ROS, desde la raíz:

```bash
python3 -m pip install -r requirements.txt
python3 -m unittest discover -s tests -v
python3 -m tp_slam.cli toy --config config/default.yaml --output artifacts/toy --seed 7 --check
python3 scripts/tune_toy.py --output artifacts/tuning
```

Instalar/enlazar esta carpeta como paquete dentro de `src/` de un workspace. Ejecutar `colcon build --symlink-install --packages-select tp_slam` y cargar ROS y overlay en cada terminal. Ver preparación completa en `A.md`.

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
ros2 launch tp_slam slam.launch.py mode:=toy rviz:=true output_path:=/home/alumno/rosmaster_ws/maps/tp_slam_toy
```

El script copia sólo archivos del paquete a `~/rosmaster_ws/src/tp_slam` y construye únicamente ese paquete por defecto. `--sync-only` omite la construcción; luego ejecutar `colcon build --symlink-install --packages-select tp_slam` dentro del workspace. Tras editar los fuentes del host hay que resincronizar, construir y cargar de nuevo el overlay. El 10 de noviembre de 2026 se verificaron sincronización y build en el contenedor habitual: colcon aprobó, el prefijo instalado es `/home/alumno/rosmaster_ws/install/tp_slam` y ROS enumeró los cuatro ejecutables. No se lanzaron nodos, RViz ni Gazebo durante esa comprobación; la evidencia de ejecución anterior corresponde al contenedor aislado.

Guardar con `/slam/save_map` genera PGM/YAML/JSON en `~/rosmaster_ws/maps` dentro del contenedor. Esa salida **no se persiste automáticamente en el host** y se pierde si se elimina el contenedor. Para probar Gazebo, detener el toy con Ctrl+C y ejecutar `ros2 launch tp_slam slam.launch.py mode:=simulation rviz:=true output_path:=/home/alumno/rosmaster_ws/maps/tp_slam_simulation`. No ejecutar ambos modos a la vez porque comparten sensores, reloj y TF. La guía completa está en `A.md`.

### Modos del launcher

```bash
ros2 launch tp_slam slam.launch.py mode:=toy rviz:=false output_path:=/tmp/slam/toy
ros2 launch tp_slam slam.launch.py mode:=simulation world:=/ruta/mundo.sdf
ros2 launch tp_slam slam.launch.py mode:=bag bag_path:=/ruta/bag bag_rate:=0.5
ros2 launch tp_slam slam.launch.py mode:=real base_frame:=base_link
ros2 service call /slam/save_map std_srvs/srv/Trigger '{}'
```

`simulation start_simulator:=false` se conecta a un simulador existente. Para iniciar Gazebo usa directamente `yahboom_rosmaster_gazebo/rosmaster_gazebo_fortress.launch.py`, Fortress, mundo configurable, perfil `ideal` y `motion_bias=false`. El backend/driver es externo. La exploración A se hace con teleop externo; este paquete no publica velocidades.

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
- **Pendientes:** recorrido completo en Gazebo, bag real, calibración y mapeo vivo del aula. No hay bag físico disponible aquí. No presentar bag sintético como evidencia real.

## Límites y próximos pasos

Sin deskew, scan matching, fusión profundidad/RGB/IMU, relocalización global, optimización retrospectiva, mapas dinámicos/3D ni navegación B/C. Revisitar puede seleccionar una hipótesis superviviente; no reconstruye trayectorias eliminadas ni reajusta poses pasadas. Memoria densa O(N·área/resolución²), más trayectorias crecientes.

Antes de B/C registrar sesiones físicas y verificar puertas, paredes, cobertura y huella. El mapa toy no certifica la geometría del laboratorio. Consultar `docs/entrega_a.md` para fundamentos y respuestas.
