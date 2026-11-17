# Plan para la sesión con el robot real

Se usó en la sesión del 2026-11-16 ([bitácora](sesion_robot_20261116.md)), que llegó hasta el paso 6, y quedó actualizado con lo que se aprendió ahí para la próxima. Los pasos están en orden y cada uno dice para qué sirve. Si el tiempo no alcanza, se corta desde el final: los pasos 1 a 5 son los que no se pueden recuperar después.

Llevar cinta métrica, cinta de papel para marcar el piso y algo para anotar.

## 1. Ordenar la sala

Empujar las sillas con ruedas debajo de sus mesas y sacar del paso lo que no vaya a quedar ahí.

Para qué: las sillas nunca quedan dos veces en el mismo lugar. Debajo de la mesa es la posición más repetible, y es la que conviene que guarde el mapa ([011](decisiones/011-entorno-que-cambia.md)).

## 2. Verificar que llegan los datos

1. Levantar drivers, odometría y URDF del RosMaster.
1. En cada terminal de la PC, pasar al domain del robot: `source ~/ivnafra/ws/env.sh 11`. Las terminales nuevas arrancan en el domain 0 y ahí el robot no se ve.
2. `ros2 topic hz /scan` y `ros2 topic hz /odom`.
3. `ros2 run tf2_ros tf2_echo base_footprint laser_link`.
4. Confirmar el frame de la base, el tópico de odometría y el `child_frame_id` de sus mensajes.

Para qué: en el simulador los frames y el tópico de odometría son distintos a los del robot. Si un nombre está mal, el nodo descarta todos los scans y parece que no hace nada.

Con los paquetes `slam` y `nav` ya compilados en el workspace ([guía de la etapa B](guia_etapa_b.md)).

Medido el 2026-11-16 ([bitácora](sesion_robot_20261116.md)): robot en `172.23.16.78`, `ROS_DOMAIN_ID=11`, base `base_footprint`, odometría en `/odom` con `child_frame_id: base_footprint`, lidar en `laser_link`, `/scan` a 7,3 Hz y `/odom` a 10 Hz. Los comandos de abajo ya usan esos nombres. Si el robot o su configuración cambian, volver a verificarlos.

## 3. Medir el robot y una silla

Con la cinta: altura del lidar sobre el piso, ancho del robot, ancho de la base con ruedas de una silla y altura de esa base.

Para qué: si la base de la silla queda por debajo del plano del lidar, el robot solo ve la columna y puede enganchar la base. El margen de inflación del planificador sale de estas medidas.

## 4. Marcar el piso

Pegar tres o cuatro cruces de cinta en lugares separados, una en el punto de partida. Medir la distancia de cada cruz a dos paredes y anotarla.

Para qué: en el robot real no hay pose verdadera. Las marcas son la referencia para medir repetibilidad y error de llegada ([010](decisiones/010-metricas-de-evaluacion.md)).

## 5. Grabar un bag de cada corrida

Antes de cada paso que mueva al robot, en una terminal aparte:

```bash
ros2 bag record -o bag_<nombre> /scan /odom /tf /tf_static /cmd_vel
```

Al terminar, `ros2 bag info bag_<nombre>` tiene que mostrar mensajes en `/tf_static`.

Para qué: es lo más importante de la sesión. Con el bag se repite todo offline las veces que haga falta, aunque la corrida en vivo salga mal. Todas las tablas de las decisiones se llenan con replays.

## 6. Construir el mapa (etapa A)

Dejar el robot quieto sobre la cruz de partida, mirando a una zona con paredes. El mapa arranca en `(0,0,0)` ahí.

Desde la carpeta `slam/`, con el paquete compilado en el workspace del robot:

```bash
ros2 launch slam slam.launch.py mode:=real scan_topic:=/scan odom_source:=topic \
  odom_topic:=/odom base_frame:=base_footprint output_path:=/tmp/slam/aula rviz:=true \
  config_file:=$PWD/config/parcial_bag_scan_match.yaml
ros2 topic echo /slam/status > /tmp/slam/status.log     # en otra terminal
```

Va `parcial_bag_scan_match.yaml` y no `default.yaml` por el tamaño del mapa: el default cubre 8 m para cada lado y en el bag del Parcial el robot se alejó 9,5 m. Si todas las partículas salen del mapa, el filtro aborta.

Antes de manejar, dejar el robot quieto un minuto y mirar `/slam/status`: `processed_scans` no tiene que subir, `skipped_motion` sí, y la pose en RViz no se tiene que mover. Es la prueba en el robot de la corrección de [012](decisiones/012-integracion-con-el-robot-quieto.md).

Manejo:

- a 0,15 m/s y con giros lentos. El 16/11 la mediana fue 0,4 m/s y las paredes salieron de 20 cm de grosor;
- arrancar a 0,5 m o más de cualquier pared, mirando hacia la sala;
- mirar cada puerta desde los dos lados;
- recorrer todas las zonas y volver a la cruz de partida para cerrar el lazo.

Mirar en `/slam/status`:

| Campo | Valor esperado |
|---|---|
| `update_seconds` | Menor que el intervalo entre actualizaciones. En la Mac fue 0,073 s |
| `queued_scans` | 0 o 1. Si crece, la computadora no llega: bajar `filter.particles` de 8 a 4 y repetir |
| `dropped_scans` | Sin crecer después del arranque |
| `state` | Nunca `ERROR` |

Guardar antes de apagar:

```bash
ros2 service call /slam/save_map std_srvs/srv/Trigger '{}'
```

Para qué: la etapa B se evalúa sobre un mapa hecho por nuestro SLAM, y todo lo que sigue lo usa. Medir con la cinta dos o tres paredes o puertas para comparar con el mapa.

## 7. Recorrido de referencia sobre las marcas

Con teleop y el bag grabando, llevar el robot a cada cruz, frenarlo unos segundos encima y volver a la de partida. Repetir el recorrido completo dos veces más.

Para qué: da varias pasadas por los mismos puntos físicos. Offline, la localización tiene que dar la misma pose cada vez que el robot estuvo sobre la misma cruz.

## 8. Prueba de sillas movidas

Sacar cinco o seis sillas de debajo de las mesas, dejarlas en lugares distintos y repetir el recorrido del paso 7 con otro bag. Sacar una foto de la sala antes y después.

Para qué: mide cuánto empeora la localización cuando la sala no coincide con el mapa. En la prueba sintética el error subió 2 a 8 cm sin perderse ([011](decisiones/011-entorno-que-cambia.md)). Son los datos de las preguntas 4 y 7 del informe.

## 9. Localización en vivo (etapa B)

Cerrar el SLAM del paso 6. Desde la carpeta `nav/`, con el mapa guardado:

```bash
ros2 launch nav nav.launch.py map_file:=/tmp/slam/aula.yaml config_file:=$PWD/config/real.yaml \
  base_frame:=base_footprint odom_topic:=/odom navigate:=false
ros2 topic echo /nav/localization_status > /tmp/slam/loc_status.log     # en otra terminal
```

`navigate:=false` lanza solo la localización, el robot se maneja con teleop. Si en la sesión no se hizo un mapa nuevo, usar `map_file:=$TP/sesiones/20261116/mapa/intento3_corregido/map.yaml`, también en el paso 10.

1. En RViz, marcar la pose con **2D Pose Estimate**. La nube de flechas azules se tiene que cerrar sobre el robot y los puntos rojos del lidar tienen que caer sobre las paredes del mapa.
2. Manejar con teleop y mirar que la pose siga al robot.
3. Repetir el click con errores a propósito: 30 cm, 1 m, 45° y 90°. Offline, hasta 1,5 m o 45° queda bien ubicado; con 90° se pierde la mitad de las veces sin avisar.

Anotar para cada click si quedó bien ubicado. El log del nodo dice cuántas iteraciones y milisegundos tardó, y `update_seconds` en el estado es el tiempo por actualización.

Si la computadora no llega (`update_seconds` cerca de 0,1 s o más), bajar `localization.particles` de 1000 a 300 en `config/real.yaml`. En la Mac 1000 partículas tardan 5 ms y 300 tardan 1,5 ms.

Para qué: es la primera vez que el filtro corre en la computadora del robot y con odometría real. En simulación quedó bien ubicado con clicks de hasta 1 m o 45°, y con 2 m falló la mitad de las veces sin avisar ([004](decisiones/004-localizacion-en-mapa-conocido.md)). Por eso hay que mirar los puntos rojos contra las paredes antes de confiar en la pose.

## 10. Ir a un objetivo (etapa B)

Cerrar el launch del paso 9 y lanzarlo completo:

```bash
ros2 launch nav nav.launch.py map_file:=/tmp/slam/aula.yaml config_file:=$PWD/config/real.yaml \
  base_frame:=base_footprint odom_topic:=/odom
ros2 topic echo /nav/state > /tmp/slam/nav_state.log                    # en otra terminal
```

0. Dejar el robot a 0,5 m o más de cualquier pared y sin nada a menos de 40 cm por delante. En el lugar donde arrancaron las corridas del 16/11 el robot estaba a 15 cm de la pared: el planificador igual arranca (corre el inicio 10 a 15 cm, hasta la celda transitable más cercana), pero el robot gira en el lugar pegado a la pared y la puede tocar. Además, con el robot de frente a la pared la parada de seguridad ya está activa antes de salir.
1. Marcar la pose con **2D Pose Estimate** y verificar que quedó bien ubicado, como en el paso 9.
2. Primer objetivo cerca, a 1 o 2 m en línea recta y sin nada en el medio, con **2D Goal Pose**. En RViz aparece el camino en naranja. El robot gira hacia el camino, avanza y al llegar gira al ángulo del objetivo.
3. Después, objetivos sobre las cruces. Al llegar, medir con la cinta la distancia del centro del robot a la cruz y anotar el ángulo.
4. Una vez, marcar otro objetivo con el robot en marcha: tiene que replanificar.
5. Una vez, marcar un objetivo adentro de una pared: lo tiene que rechazar y quedarse quieto.

Tener una mano en el teleop o en el botón de parada. El robot va a 0,2 m/s como máximo. La evasión de obstáculos no entra en esta sesión: si algo queda a menos de 40 cm delante, frena, espera hasta 15 s y si no se despeja abandona el objetivo. Para frenarlo a mano, Ctrl+C en el launch.

Para qué: es el resultado de punta a punta que evalúa la consigna. El error de llegada con cinta es la única medida de exactitud que hay en el robot real. En el simulador llegó a 13 a 22 cm y 4 a 5° del objetivo ([007](decisiones/007-seguimiento-del-camino.md)).

## Antes de irse

Copiar fuera del robot: todos los bags, `/tmp/slam/aula.pgm` y `.yaml`, los `.log` de `/tmp/slam/`, una captura de RViz y las fotos.

Anotar:

- si la sala y el punto de partida son los mismos que en el bag del Parcial;
- las medidas de los pasos 3 y 4 y las de las paredes;
- la velocidad aproximada de manejo y cualquier patinada o choque.

## Después de la sesión

Cada bag se convierte y se analiza offline con los cuatro comandos de [Analizar un bag](guia_etapa_b.md#analizar-un-bag): extraer, correr la localización, medir la repetibilidad sobre las cruces y comparar con el SLAM. Con eso se llenan las tablas pendientes de las decisiones 004, 010 y 011.
