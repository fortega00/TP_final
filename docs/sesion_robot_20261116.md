# Bitácora de la sesión con el robot real, 2026-11-16

Qué hicimos en el laboratorio con el RosMaster X3, qué aprendimos y cómo repetirlo. El plan original está en [robot_real_checklist.md](robot_real_checklist.md) y los datos crudos en [sesiones/20261116/](../sesiones/20261116/README.md).

Se completaron los pasos 2, 5 y 6 del checklist: verificar datos, grabar bags y construir el mapa. Quedaron pendientes los pasos 7 a 10. Los mapas se revisaron solo a ojo; no hay comparación contra medidas con cinta.

## Datos de conexión

| Dato | Valor |
|---|---|
| IP del robot | `172.23.16.78` |
| `ROS_DOMAIN_ID` del robot | `11` |
| IP de la PC del laboratorio | `172.23.16.197` (wifi), `172.18.146.13` (cable) |
| Usuario de la PC | `alumno1` (cuenta compartida con otros grupos) |
| ROS en la PC | Humble nativo en `/opt/ros/humble`, sin Docker, `rmw_fastrtps_cpp` |
| Ping PC a robot | unos 13 ms |

El `.bashrc` de la PC deja cada terminal nueva en el domain 0 (`local_mode 0`). En el domain 0 el robot no se ve: solo aparecen `/rosout` y `/parameter_events`. Hay que pasar al 11 en cada terminal.

## Preparación de la PC

Workspace propio, sin tocar los de otros grupos (`tp_final_ws`, `yahboom_test_ws`, `robav_ws`):

```bash
mkdir -p ~/ivnafra/ws/src && cd ~/ivnafra/ws
ln -sfn ~/ivnafra/TP_final/slam src/slam
ln -sfn ~/ivnafra/TP_final/nav src/nav
colcon build --symlink-install --packages-select slam nav
```

Compiló sin errores. Tests en la PC: 37 de `slam` y 28 de `nav`, todos aprobados (NumPy 1.21.5, PyYAML 5.4.1).

En cada terminal nueva:

```bash
source ~/ivnafra/ws/env.sh 11
```

Carga ROS, el overlay, fija el domain y define `$TP` (el repo) y `$SESION` (`~/ivnafra/sesion_20261116`). Los dos scripts están copiados en `sesiones/20261116/scripts/`.

## Qué publica el robot

Nodos: `/driver_node`, `/base_node`, `/sllidar_node`, `/robot_state_publisher`, `/imu_filter_madgwick`, `/astra_sensor_adapter`, `/_hardware/astra/camera`, `/system_shutdown_listener`.

| Dato | Valor |
|---|---|
| `/scan` | 7,3 Hz, 1080 haces, frame `laser_link`, rango declarado 0,05 a 12 m, QoS best-effort |
| `/odom` | 9,9 Hz, `frame_id: odom`, `child_frame_id: base_footprint`, QoS reliable |
| TF `odom → base_footprint` | 10 Hz |
| TF `base_footprint → base_link → laser_link` | x = 0,043 m, z = 0,181 m, yaw 180° |
| Scan de muestra | 933 haces válidos, 147 `inf`, mínimo 0,42 m |
| Diferencia de reloj PC y robot | 0,16 a 0,33 s |
| Batería | 10,6 V al empezar (`/voltage`). Es bajo si el pack es 3S (12,6 V lleno): tener el cargador a mano |
| Otros tópicos | `/scan_filtered`, `/imu/data`, `/imu/data_raw`, `/imu/mag`, `/vel_raw`, `/joint_states`, `/cam_1/color/image_raw`, `/cam_1/depth/image_raw`, `/cam_1/depth/color/points`, `/Buzzer`, `/RGBLight` |

Lo que cambia respecto de lo que suponía el checklist:

- **La base es `base_footprint`, no `base_link`.** Todos los comandos van con `base_frame:=base_footprint`. Es el default del launch de `slam`.
- **El lidar está montado girado 180°.** El TF lo corrige; no hay que tocar nada.
- **La altura del lidar según el URDF es 18,1 cm.** Falta confirmarla con cinta y compararla con la base de las sillas (paso 3).
- **No hay retornos del chasis** por debajo de 0,25 m en el scan de muestra, así que `min_range: 0.12` no molestó.
- **Usamos `/scan`**, no `/scan_filtered`.

## Cómo corrimos el SLAM

```bash
cd $TP/slam
ros2 launch slam slam.launch.py mode:=real scan_topic:=/scan odom_source:=topic \
  odom_topic:=/odom base_frame:=base_footprint output_path:=/tmp/slam/aula rviz:=true \
  config_file:=$PWD/config/parcial_bag_scan_match.yaml
```

Bag, estado y guardado:

```bash
cd $SESION/bags && ros2 bag record -o bag_<nombre> /scan /odom /tf /tf_static /cmd_vel
ros2 topic echo /slam/status --full-length > /tmp/slam/status.log
ros2 service call /slam/save_map std_srvs/srv/Trigger '{}'
```

Para no abrir cuatro terminales, `~/ivnafra/ws/session.sh`:

```bash
~/ivnafra/ws/session.sh start bag_mapa3    # bag + SLAM + RViz + log de estado
~/ivnafra/ws/session.sh ps                 # qué está corriendo
~/ivnafra/ws/session.sh stop               # baja todo; no guarda el mapa
```

Parámetros de `parcial_bag_scan_match.yaml`: grilla 48 × 40 m con origen `(-16, -20)` y celda 0,05 m; 8 partículas, semilla 7; scan matching activado con ventana ±0,10 m y ±0,06 rad; actualización cada 0,08 m, 0,06 rad o 1 s (la condición de tiempo se sacó después, ver [012](decisiones/012-integracion-con-el-robot-quieto.md)); `p_hit` 0,75, `p_free` 0,40; `no_return_is_free: false`. El detalle completo queda en el `aula.json` de cada mapa.

Resto de los valores de esa configuración:

- **Sensor:** log-odds ±4, espesor 0,08 m, rango 0,12 a 8 m, 180 haces de mapa y 60 de likelihood, `sigma_hit` 0,15, mezcla 0,95/0,05, distancia máxima 0,6 m, umbral de ocupado 0,65.
- **Movimiento:** `alpha1` 0,003, `alpha2` 0,0003, `alpha3` 0,003, `alpha4` 0,003. Solo se usan hasta que el mapa tiene celdas suficientes para el scan matching.
- **Filtro:** remuestreo con ESS < 0,5 N, pose inicial `(0,0,0)` sin dispersión.
- **Scan matching:** pasos de 0,025 m y 0,01 rad; `sigma` 0,05; prior 0,05 m y 0,03 rad; muestreo 0,01 m y 0,005 rad; 180 haces, 20 efectivos; mínimo 200 celdas ocupadas.

### Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` adelante, `,` atrás, `j` y `l` giran, `u` y `o` avanzan girando, `k` frena, `q` sube la velocidad un 10 %, `Shift` + `J` o `L` mueve de costado. Arranca en 0,5 m/s: bajar con `z` hasta unos 0,15 m/s antes de mapear. La terminal del teleop tiene que tener el foco.

### Cámara

```bash
ros2 run rqt_image_view rqt_image_view /cam_1/color/image_raw
```

Desde el desplegable se puede pasar a `/cam_1/depth/image_raw` para ver la profundidad. También se ve dentro de RViz: Add, By topic, `/cam_1/color/image_raw`, Image.

El robot solo publica la imagen sin comprimir. Por wifi ocupa mucho ancho de banda y puede hacer perder scans, que viajan best-effort. Usarla para una mirada rápida y cerrarla antes de mapear.

## Corridas

| Bag | Mapa | Duración | Scans | Integrados | Resultado |
|---|---|---|---|---|---|
| (regrabado encima) | `mapa/intento1` | | | 200 | ESS 6,1 de 8; 1392 scans salteados por falta de movimiento. Cobertura parcial (54 m² libres), parte de abajo torcida unos 15 a 20°. Descartado |
| `bag_intento_descartado` | no tiene | 109 s | 816 | | El robot se movió sin querer y había dos SLAM corriendo. Descartado |
| `bag_mapa` | `mapa/intento2` | 734 s | 5381 | 1104 | Sala completa, 108,5 m² libres. Paredes gruesas, contorno algo inclinado |
| `bag_mapa2` | `mapa/intento3` | 475 s | 3614 | 962 | Sala completa, 108,2 m² libres. Paredes rectas. Reemplazado por `mapa/intento3_corregido` para la etapa B (ver Análisis posterior) |

En las dos corridas completas:

- `state` siempre `RUNNING`, cero scans descartados, cero en cola, ninguna partícula fuera del mapa.
- `update_seconds` entre 0,08 y 0,09 s en esta PC. El scan llega cada 0,137 s: alcanza, pero no sobra para subir mucho las partículas o los haces.
- Remuestreos: 148 en `intento2` y 93 en `intento3`. `update_seconds` final 0,085 s y 0,092 s.
- Mensajes de `/cmd_vel` en el bag: 2361 en `bag_mapa` y 3571 en `bag_mapa2`. Odometrías: 7340 y 4750. Los dos tienen `/tf_static`.
- El lazo cerró sin paredes dobles visibles.
- La sala mide unos 12 × 11 m en el mapa y se ven las tres filas de mesas con sus sillas.
- Los dos mapas están girados 180° entre sí porque el robot arrancó mirando para lados opuestos. El `(0,0,0)` de cada mapa es donde estaba el robot al lanzar.
- En `intento3` sale un abanico de rayos por una esquina. Parece una puerta abierta; falta confirmarlo.

## Lo que aprendimos

1. **Domain 11 en cada terminal.** Un `save_map` lanzado en el domain 0 se queda colgado esperando un servicio que no ve. Lo mismo pasa con los comandos lanzados con `!` desde Claude Code: corren en el domain 0.
2. **`base_footprint`.** Si el `base_frame` no coincide con el `child_frame_id` de la odometría, el nodo descarta todo.
3. **Un solo SLAM a la vez.** En un reinicio quedaron dos nodos `/slam` vivos, publicando el mismo `/map` y el mismo `map → odom`. Antes de relanzar, `session.sh ps` o `ros2 node list` tiene que mostrar cero o un `/slam`.
4. **El robot quieto y en la cruz antes de lanzar.** El mapa nace donde está el robot. Si se lo mueve sin querer, conviene reiniciar.
5. **Giros lentos.** El primer mapa se torció. El scan matching corrige hasta 0,06 rad por actualización y el lidar da 7 Hz.
6. **El bag se cierra con Ctrl+C (SIGINT).** Hasta entonces no existe `metadata.yaml` y `ros2 bag info` falla. `ros2 bag record -o` no acepta una carpeta que ya existe.
7. **Guardar antes de bajar.** `session.sh stop` y Ctrl+C no guardan el mapa.
8. **`/tmp/slam` se pisa en cada corrida** y se pierde al reiniciar la PC. Después de cada guardado, copiar a `$SESION/mapa/intentoN/`.
9. **Los bags se grabaron en la PC por wifi.** Puede faltar algún scan. Grabar también en el robot sería más seguro.
10. **Con el bag completo, los parámetros se prueban offline.** No hace falta volver a manejar el robot para comparar configuraciones.
11. **Git en la PC del laboratorio.** No tiene identidad configurada (se pasa con `git -c user.name=... -c user.email=...`) ni `git lfs`. El `.gitignore` del repo excluye `*.db3` y `logs/`: los bags se agregan con `git add -f`. `bag_mapa` pesa 57 MB y GitHub avisa por encima de 50 MB (el límite es 100 MB por archivo).
12. **Push desde la PC del laboratorio.** El remoto es HTTPS y no hay `gh` ni claves SSH. GitHub no acepta la contraseña de la cuenta: en `Password` va un token personal con permiso `repo` (github.com/settings/tokens). El `!` de Claude Code no sirve para el push porque no puede pedir credenciales; hay que usar una terminal normal. Como la cuenta `alumno1` es compartida, no guardar el token (`credential.helper store`) y revocarlo al terminar. La rama `sesion-robot-20261116` se subió el mismo 16/11.
13. **No bajar procesos con `pkill -f` desde un script o una línea compuesta.** El patrón también coincide con la línea de comando del propio shell y lo mata antes de terminar. `session.sh` lista los procesos con `ps` y descarta los `bash`. A mano, usar `kill -INT <pid>`.

## Análisis posterior

Hecho el mismo 16/11 con los bags, sin el robot: replays offline del núcleo con 3 a 8 semillas por bag y comparación de los dos mapas entre sí. No hay medidas con cinta, entonces todo esto mide consistencia, no exactitud.

Lo que se confirma:

- Los replays offline reproducen los mapas en vivo.
- La odometría sola termina 1,45 m y 14° corrida en `bag_mapa`, y 1,89 m y 21° en `bag_mapa2`. El SLAM vuelve al lugar de partida con unos 10 cm de diferencia.
- `intento2` e `intento3`, alineados con un giro de 177°, coinciden: 83 % de las celdas ocupadas a 10 cm o menos y 95 % a 20 cm o menos. La diferencia está en una pared larga, 20 a 30 cm, y el torcido es `intento2`.
- El planificador de la etapa B encuentra una sola zona transitable de 70 m² y llega a todos los pasillos. La localización sobre `intento3` sigue al SLAM a 3,7 cm con su propio bag y a 4,6 a 4,9 cm con el otro, sin perderse.

Lo que corrige a esta bitácora:

- **Manejamos mucho más rápido que 0,15 m/s.** Mediana de 0,37 a 0,42 m/s, picos de 0,5 a 0,6 m/s y giros de hasta 1 rad/s. A 7,4 Hz, un scan abarca hasta 7 cm y 8° de movimiento.
- **Las paredes miden unos 20 cm de grosor en los dos mapas**, no son finas. La velocidad es la causa más probable, sin probar.
- **Con el robot quieto la pose del SLAM derivaba**: 17 cm en 215 s en `bag_mapa2`. El `(0,0)` de `intento3` está a unos 20 cm del lugar físico de partida. Corregido en el código, ver [012](decisiones/012-integracion-con-el-robot-quieto.md). El mapa para la etapa B pasa a ser `sesiones/20261116/mapa/intento3_corregido`, regenerado del mismo bag.
- **El robot arrancó y terminó las dos corridas casi tocando una pared.** En `bag_mapa2` estaba de frente, con 40 % de los haces sin retorno. El "sin retornos por debajo de 0,25 m" de arriba salió de un solo scan: quieto contra la pared, 19 % de los haces miden menos de 0,30 m.
- **El 62 % de `bag_mapa2` y el 80 % de `bag_mapa` es el robot quieto.**
- `bag_mapa` tiene un hueco de 22 s sin scans en t = 430 s, con el robot quieto. `bag_mapa2` perdió unos 5 scans.

Para la próxima sesión:

- Manejar a 0,15 m/s de verdad y girar despacio. Mirar la velocidad que imprime el teleop.
- Arrancar a 0,5 m o más de cualquier pared, sobre una cruz marcada, mirando hacia la sala.
- Hacer las medidas con cinta de los pasos 3, 4 y 6.
- Dejar el robot quieto unos minutos con el SLAM en vivo para confirmar la corrección de [012](decisiones/012-integracion-con-el-robot-quieto.md).
- Arrancar la navegación a 0,5 m o más de cualquier pared y sin nada a menos de 40 cm por delante.
- En el paso 9, sumar un click con 90° de error y mirar los puntos rojos del lidar contra las paredes.

## Pruebas offline del 17/11

Hechas con los bags y el mapa `intento3_corregido`, sin el robot. Los scripts están en `tp4_final/review/` del repo personal (`lab_partb_loc.py`, `lab_partb_nav.py`, `lab_ros_node_check.py`).

**SLAM corregido en el nodo ROS.** `bag_mapa2` reproducido a velocidad real por `slam_node` en la Mac da lo mismo que el replay del núcleo: 691 integraciones, 2922 scans omitidos por movimiento, las mismas poses y el mismo `map.pgm` byte a byte. En los 263 s que el robot está quieto, `/slam_pose` se mueve 0,00 cm. Se descartó un solo scan, el primero, por llegar antes que la primera odometría. La Mac, con otros procesos a la vez, tardó 0,17 s por actualización y acumuló hasta 12 scans en cola (límite 30) sin perder ninguno. En la PC del laboratorio fueron 0,08 a 0,09 s.

**Localización con clicks equivocados.** 82 corridas del filtro de la etapa B sobre los dos bags, contra la trayectoria del SLAM del mismo bag:

| Click inicial | Corridas | Resultado |
|---|---|---|
| Correcto, 8 semillas por bag | 16 | 3,4 a 3,5 cm (`bag_mapa2`) y 6,1 a 6,3 cm (`bag_mapa`, con ajuste rígido) |
| Hasta 1,5 m de error, o hasta 45°, o 1 m en cada eje más 45° | 60 | Igual que el click correcto. La ráfaga inicial lo resuelve con el robot quieto |
| 90° de error | 6 | 3 no se recuperan y terminan a 8 m. Las otras 3 se recuperan cuando el robot empieza a moverse |

En las corridas perdidas el filtro no avisa: la ráfaga dice que convergió y la nube queda cerrada (3 cm). Lo que las delata es el residuo del scan contra el mapa: 20 a 34 cm perdido contra 1 cm ubicado. Es la medición que faltaba en [004](decisiones/004-localizacion-en-mapa-conocido.md). Usar el residuo para pasar a `LOST` es una decisión abierta.

**Planificador.** Sobre la zona transitable de 70,6 m²: 200 pares de puntos al azar, todos con camino, largo mediano 1,24 veces la línea recta, distancia mínima a un obstáculo 0,22 m. Mediana de 43 ms por plan, con la máquina ocupada. 50 objetivos en celdas libres pegadas a una pared: los 50 rechazados.

**Seguidor.** Máquina de estados más pure pursuit sobre un modelo cinemático, 20 objetivos desde el lugar de partida del 16/11: llega a los 20, nunca pasa de 0,20 m/s ni de 0,80 rad/s, se aparta del camino 9 cm como máximo y tarda 42 s de mediana. Con ruido de 4 cm y 1° en la pose también llega a los 20, con hasta 15 cm de desvío. No prueba la dinámica del robot real.

**Lugar de partida.** La pose de partida del 16/11 está a 10 a 15 cm de la pared y no es transitable (hacen falta 22 cm). El planificador corre el inicio 10 a 15 cm, dentro del límite de 30 cm, y planifica igual. En `bag_mapa2` el robot arrancó con la pared a 11 cm por delante, dentro del rectángulo de la parada de seguridad. Manejando, la parada se habría activado en el 1 % de los scans.

## Pendiente

- Paso 3: medir con cinta la altura del lidar, el ancho del robot y la base de una silla.
- Paso 4: marcar las cruces y medir su distancia a dos paredes.
- Paso 6: medir dos o tres paredes o puertas y compararlas con `intento3_corregido`.
- Pasos 7 y 8: recorridos sobre las cruces, con y sin sillas movidas.
- Paso 9: localización en vivo. Cerrar el SLAM antes.
- Paso 10: ir a un objetivo.

Comandos de los pasos 9 y 10 con los nombres de este robot:

```bash
cd $TP/nav
ros2 launch nav nav.launch.py map_file:=$TP/sesiones/20261116/mapa/intento3_corregido/map.yaml \
  config_file:=$PWD/config/real.yaml base_frame:=base_footprint odom_topic:=/odom navigate:=false
```

Sin `navigate:=false` para el paso 10. Ninguno de los dos se probó todavía en el robot.

- Analizar offline `bag_mapa` y `bag_mapa2` con los comandos de [Analizar un bag](guia_etapa_b.md#analizar-un-bag).
