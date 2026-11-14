# Plan para la sesión con el robot real

Sesión del lunes 2026-11-16. Los pasos están en orden y cada uno dice para qué sirve. Si el tiempo no alcanza, se corta desde el final: los pasos 1 a 5 son los que no se pueden recuperar después.

Llevar cinta métrica, cinta de papel para marcar el piso y algo para anotar.

## 1. Ordenar la sala

Empujar las sillas con ruedas debajo de sus mesas y sacar del paso lo que no vaya a quedar ahí.

Para qué: las sillas nunca quedan dos veces en el mismo lugar. Debajo de la mesa es la posición más repetible, y es la que conviene que guarde el mapa ([011](decisiones/011-entorno-que-cambia.md)).

## 2. Verificar que llegan los datos

1. Levantar drivers, odometría y URDF del RosMaster.
2. `ros2 topic hz /scan` y `ros2 topic hz /odom`.
3. `ros2 run tf2_ros tf2_echo base_link laser`, con los nombres de frames que publique el robot.
4. Anotar el frame de la base (`base_link` o `base_footprint`), el tópico de odometría y el `child_frame_id` de sus mensajes.

Para qué: en el simulador los frames y el tópico de odometría son distintos a los del robot. Si un nombre está mal, el nodo descarta todos los scans y parece que no hace nada.

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
  odom_topic:=/odom base_frame:=base_link output_path:=/tmp/slam/aula rviz:=true \
  config_file:=$PWD/config/parcial_bag_scan_match.yaml
ros2 topic echo /slam/status > /tmp/slam/status.log     # en otra terminal
```

Va `parcial_bag_scan_match.yaml` y no `default.yaml` por el tamaño del mapa: el default cubre 8 m para cada lado y en el bag del Parcial el robot se alejó 9,5 m. Si todas las partículas salen del mapa, el filtro aborta.

Manejo:

- despacio, sobre todo en los giros. El scan matching busca hasta 10 cm y 0,06 rad por actualización;
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

Con el mapa del paso 6, lanzar el nodo de localización, marcar la pose con 2D Pose Estimate en RViz y manejar con teleop mirando que la pose siga al robot. Repetir el click con errores a propósito: 30 cm, 1 m y 45°.

Anotar cuánto tarda en ubicarse después de cada click, el tiempo por actualización del filtro y si en algún caso queda ubicado en un lugar equivocado.

Para qué: es la primera vez que el filtro corre en la computadora del robot y con odometría real. En simulación quedó bien ubicado con clicks de hasta 1 m o 45° ([004](decisiones/004-localizacion-en-mapa-conocido.md)).

El nodo todavía no existe. El comando se agrega acá cuando esté probado en el simulador.

## 10. Ir a un objetivo (etapa B)

Con el robot ubicado, marcar un objetivo con 2D Goal Pose sobre una de las cruces. Al llegar, medir con la cinta la distancia del robot a la cruz y el error de ángulo. Repetir con tres objetivos, y una vez cambiar el objetivo con el robot en marcha.

Tener el teleop o el botón de parada a mano. La evasión de obstáculos no entra en esta sesión: si aparece algo en el camino, el robot solo frena.

Para qué: es el resultado de punta a punta que evalúa la consigna. El error de llegada con cinta es la única medida de exactitud que hay en el robot real.

El planificador, el seguidor y la máquina de estados todavía no existen. El comando se agrega acá cuando estén probados en el simulador.

## Antes de irse

Copiar fuera del robot: todos los bags, `/tmp/slam/aula.pgm` y `.yaml`, `status.log`, una captura de RViz y las fotos.

Anotar:

- si la sala y el punto de partida son los mismos que en el bag del Parcial;
- las medidas de los pasos 3 y 4 y las de las paredes;
- la velocidad aproximada de manejo y cualquier patinada o choque.
