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
| Batería | 10,6 V al empezar (`/voltage`) |
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

Parámetros de `parcial_bag_scan_match.yaml`: grilla 48 × 40 m con origen `(-16, -20)` y celda 0,05 m; 8 partículas, semilla 7; scan matching activado con ventana ±0,10 m y ±0,06 rad; actualización cada 0,08 m, 0,06 rad o 1 s; `p_hit` 0,75, `p_free` 0,40; `no_return_is_free: false`. El detalle completo queda en el `aula.json` de cada mapa.

### Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

`i` adelante, `,` atrás, `j` y `l` giran, `k` frena, `Shift` + `J` o `L` mueve de costado. Arranca en 0,5 m/s: bajar con `z` hasta unos 0,15 m/s antes de mapear. La terminal del teleop tiene que tener el foco.

### Cámara

```bash
ros2 run rqt_image_view rqt_image_view /cam_1/color/image_raw
```

El robot solo publica la imagen sin comprimir. Por wifi ocupa mucho ancho de banda y puede hacer perder scans, que viajan best-effort. Usarla para una mirada rápida y cerrarla antes de mapear.

## Corridas

| Bag | Mapa | Duración | Scans | Integrados | Resultado |
|---|---|---|---|---|---|
| (regrabado encima) | `mapa/intento1` | | | 200 | Cobertura parcial (54 m² libres), parte de abajo torcida unos 15 a 20°. Descartado |
| `bag_intento_descartado` | no tiene | 109 s | 816 | | El robot se movió sin querer y había dos SLAM corriendo. Descartado |
| `bag_mapa` | `mapa/intento2` | 734 s | 5381 | 1104 | Sala completa, 108,5 m² libres. Paredes gruesas, contorno algo inclinado |
| `bag_mapa2` | `mapa/intento3` | 475 s | 3614 | 962 | Sala completa, 108,2 m² libres. Paredes rectas y finas. **Elegido para la etapa B** |

En las dos corridas completas:

- `state` siempre `RUNNING`, cero scans descartados, cero en cola, ninguna partícula fuera del mapa.
- `update_seconds` entre 0,08 y 0,09 s en esta PC. El scan llega cada 0,137 s: alcanza, pero no sobra para subir mucho las partículas o los haces.
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
11. **Git en la PC del laboratorio.** No tiene identidad configurada (se pasa con `git -c user.name=... -c user.email=...`) ni `git lfs`. El `.gitignore` del repo excluye `*.db3` y `logs/`: los bags se agregan con `git add -f`. `bag_mapa` pesa 57 MB y GitHub avisa por encima de 50 MB.

## Pendiente

- Paso 3: medir con cinta la altura del lidar, el ancho del robot y la base de una silla.
- Paso 4: marcar las cruces y medir su distancia a dos paredes.
- Paso 6: medir dos o tres paredes o puertas y compararlas con `intento3`.
- Pasos 7 y 8: recorridos sobre las cruces, con y sin sillas movidas.
- Paso 9: localización en vivo. Cerrar el SLAM antes.
- Paso 10: ir a un objetivo.

Comandos de los pasos 9 y 10 con los nombres de este robot:

```bash
cd $TP/nav
ros2 launch nav nav.launch.py map_file:=$SESION/mapa/intento3/aula.yaml \
  config_file:=$PWD/config/real.yaml base_frame:=base_footprint odom_topic:=/odom navigate:=false
```

Sin `navigate:=false` para el paso 10. Ninguno de los dos se probó todavía en el robot.

- Actualizar `robot_real_checklist.md` con `base_footprint` y el domain.
- Analizar offline `bag_mapa` y `bag_mapa2` con los comandos de [Analizar un bag](guia_etapa_b.md#analizar-un-bag).
