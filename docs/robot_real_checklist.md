# Guía de laboratorio

Quedan **dos visitas como máximo** para terminar el TP. Esta guía reparte todo lo que necesita el robot real entre esas dos. Lo que pasó en la primera sesión está en la [bitácora del 16/11](sesion_robot_20261116.md).

| Visita | Objetivo | Tiene que estar listo antes de ir |
|---|---|---|
| 1 | Cerrar las etapas A y B en el robot y traer imágenes para la etapa C | [PR 3](https://github.com/fortega00/TP_final/pull/3) en `main`. Evasión de obstáculos ([008](decisiones/008-obstaculos-no-mapeados.md)) probada en el simulador |
| 2 | Misión de la etapa C con video, y repetir lo que haya fallado en la visita 1 | Detector de conos probado con las imágenes de la visita 1. Exploración probada en el simulador |

Si la evasión de obstáculos no llega a la visita 1, su prueba pasa a la visita 2. Nada más se puede correr: lo que no salga en la visita 2 queda sin medir.

## Tres reglas

1. **Grabar un bag de todo lo que mueva al robot.** Con el bag se repite offline las veces que haga falta. Sin bag, la corrida se perdió.
2. **Seguir el orden.** Los pasos van de lo irrecuperable a lo que se puede repetir. Si falta tiempo, se corta desde el final.
3. **Anotar en el momento**: medidas, velocidad de manejo, choques, patinadas y qué bag corresponde a qué prueba.

## Antes de salir de casa

- Los tests pasan (`slam` y `nav`) y `main` tiene todo lo que se va a probar.
- Lo nuevo corrió al menos una vez en el simulador o sobre un bag.
- Llevar cinta métrica, cinta de papel para el piso y algo para anotar.
- Visita 1: pedir a la cátedra conos rojos y de otros colores para grabarlos.

## Al llegar (las dos visitas)

1. Empujar las sillas con ruedas debajo de sus mesas. Es la posición más repetible y la que guarda el mapa ([011](decisiones/011-entorno-que-cambia.md)).
2. Encender el robot y levantar sus drivers.
3. En la PC, actualizar y compilar:

   ```bash
   cd ~/ivnafra/TP_final && git pull
   cd ~/ivnafra/ws && colcon build --symlink-install --packages-select slam nav
   ```

4. En **cada terminal nueva**: `source ~/ivnafra/ws/env.sh 11`. Las terminales arrancan en el domain 0 y ahí el robot no se ve.
5. Verificar que llegan los datos:

   ```bash
   ros2 topic hz /scan        # unos 7,3 Hz
   ros2 topic hz /odom        # unos 10 Hz
   ros2 run tf2_ros tf2_echo base_footprint laser_link
   ```

Valores de este robot, medidos el 16/11: IP `172.23.16.78`, `ROS_DOMAIN_ID=11`, base `base_footprint`, lidar `laser_link`, odometría en `/odom`. Si un nombre no coincide, el nodo descarta todos los scans y parece que no hace nada.

Para grabar, en una terminal aparte y antes de mover el robot:

```bash
ros2 bag record -o bag_<nombre> /scan /odom /tf /tf_static /cmd_vel
```

Al terminar, `ros2 bag info bag_<nombre>` tiene que mostrar mensajes en `/tf_static`.

## Visita 1

### 1. Medir y marcar

- Con la cinta: altura del lidar sobre el piso, ancho del robot, ancho y altura de la base con ruedas de una silla.
- Pegar tres o cuatro cruces de cinta en el piso, separadas, todas a **0,5 m o más de cualquier pared**. Una es el punto de partida. Medir la distancia de cada cruz a dos paredes.
- Medir dos o tres paredes o puertas de la sala.

Para qué: en el robot real no hay pose verdadera. Estas medidas son la única referencia de exactitud del mapa y de la llegada ([010](decisiones/010-metricas-de-evaluacion.md)). El 16/11 no se hicieron.

### 2. Mapa nuevo, manejando lento (etapa A)

Robot quieto sobre la cruz de partida, mirando hacia la sala. El mapa arranca en `(0,0,0)` ahí. Con el bag grabando, desde `slam/`:

```bash
ros2 launch slam slam.launch.py mode:=real scan_topic:=/scan odom_source:=topic \
  odom_topic:=/odom base_frame:=base_footprint output_path:=/tmp/slam/aula rviz:=true \
  config_file:=$PWD/config/parcial_bag_scan_match.yaml
ros2 topic echo /slam/status > /tmp/slam/status.log     # en otra terminal
```

1. **Prueba de robot quieto**: no tocar el teleop durante un minuto. En `/slam/status`, `processed_scans` no sube y `skipped_motion` sí. La pose en RViz no se mueve. Confirma la corrección de [012](decisiones/012-integracion-con-el-robot-quieto.md).
2. Manejar a **0,15 m/s** y con giros lentos, mirando la velocidad que imprime el teleop. El 16/11 la mediana fue 0,4 m/s y las paredes salieron de 20 cm de grosor.
3. Mirar cada puerta desde los dos lados, recorrer todas las zonas y volver a la cruz de partida.
4. Guardar antes de apagar: `ros2 service call /slam/save_map std_srvs/srv/Trigger '{}'`

| Campo de `/slam/status` | Valor esperado |
|---|---|
| `update_seconds` | Menor que 0,13 s. El 16/11 fue 0,08 a 0,09 s |
| `queued_scans` | 0 o 1. Si crece, bajar `filter.particles` de 8 a 4 y repetir |
| `state` | Nunca `ERROR` |

Listo cuando: el mapa cierra bien al volver a la partida y las paredes se ven más finas que en `intento3_corregido`. Si sale peor, la etapa B sigue con `$TP/sesiones/20261116/mapa/intento3_corregido/map.yaml`.

Para qué: es el mapa final de la etapa A. Además define si las paredes gruesas eran por la velocidad y deja el bag para decidir entre las opciones A y C de [012](decisiones/012-integracion-con-el-robot-quieto.md).

### 3. Imágenes para la etapa C

Con los conos repartidos por la sala (rojos y de otros colores, alguno visible a través de un hueco):

```bash
ros2 topic list | grep -i -E "image|camera|depth"
ros2 topic hz <tópico de imagen>
ros2 bag record -o bag_conos /scan /odom /tf /tf_static <tópicos de imagen y de camera_info>
```

Manejar con teleop mirando los conos de cerca, de lejos, de costado y a contraluz. Anotar la altura y el diámetro de un cono, y el nombre del frame de la cámara.

Listo cuando: `ros2 bag info bag_conos` muestra mensajes en los tópicos de imagen.

Para qué: el detector de conos se desarrolla en casa y sin estas imágenes no hay con qué probarlo. **No irse sin este bag**: es lo único que la visita 2 no puede recuperar.

### 4. Localización en vivo (etapa B)

Cerrar el SLAM. Con el bag grabando, desde `nav/`:

```bash
ros2 launch nav nav.launch.py map_file:=/tmp/slam/aula.yaml config_file:=$PWD/config/real.yaml \
  base_frame:=base_footprint odom_topic:=/odom navigate:=false
ros2 topic echo /nav/localization_status > /tmp/slam/loc_status.log     # en otra terminal
```

1. Marcar la pose con **2D Pose Estimate**. La nube azul se cierra sobre el robot y los puntos rojos del lidar caen sobre las paredes del mapa.
2. Manejar con teleop y mirar que la pose siga al robot.
3. Repetir el click con errores a propósito y anotar `state` y `scan_residual` de cada uno:

| Click | Resultado esperado (medido offline) |
|---|---|
| 30 cm, 1 m o 45° de error | `LOCALIZED`, `scan_residual` de 1 a 5 cm |
| 90° de error | `LOST` la mitad de las veces, `scan_residual` mayor que 10 cm. El nodo pide un click nuevo |

Si `update_seconds` llega a 0,1 s, bajar `localization.particles` de 1000 a 300 en `config/real.yaml`.

Listo cuando: los clicks con poco error quedan `LOCALIZED` y un click malo nunca queda `LOCALIZED` con los puntos rojos fuera de las paredes.

### 5. Ir a un objetivo (etapa B)

Robot a **0,5 m o más de cualquier pared** y sin nada a menos de 40 cm por delante. Mismo launch sin `navigate:=false`, con el bag grabando y `ros2 topic echo /nav/state > /tmp/slam/nav_state.log`.

1. Marcar la pose y verificar que quedó `LOCALIZED`.
2. Primer objetivo cerca, a 1 o 2 m en línea recta, con **2D Goal Pose**. El robot gira hacia el camino naranja, avanza y al llegar gira al ángulo pedido.
3. Objetivos sobre las cruces. Al llegar, medir con la cinta la distancia del centro del robot a la cruz y anotar el ángulo.
4. Marcar otro objetivo con el robot en marcha: replanifica.
5. Marcar un objetivo adentro de una pared: lo rechaza y queda quieto.

Tener una mano en el teleop. El robot va a 0,2 m/s como máximo. Para frenarlo, Ctrl+C en el launch.

Listo cuando: llega a tres cruces sin tocar nada. En el simulador llegó a 13 a 22 cm y 4 a 5° ([007](decisiones/007-seguimiento-del-camino.md)).

### 6. Obstáculo no mapeado (etapa B)

Solo si la evasión ([008](decisiones/008-obstaculos-no-mapeados.md)) está implementada. Poner una caja en medio de un camino ya recorrido y mandar el mismo objetivo. Repetir con la caja en un pasillo angosto.

Listo cuando: el robot rodea la caja y llega. Sin la evasión, lo esperado es que frene, espere 15 s y abandone el objetivo.

### 7. Recorrido sobre las cruces y sillas movidas

1. Con teleop, llevar el robot a cada cruz, frenarlo unos segundos encima y volver a la partida. Tres vueltas, un bag.
2. Sacar cinco o seis sillas de debajo de las mesas, sacar una foto y repetir con otro bag.

Para qué: mide offline la repetibilidad de la localización y cuánto empeora con la sala cambiada ([011](decisiones/011-entorno-que-cambia.md)). Es lo primero que se corta si falta tiempo: la visita 2 lo puede repetir.

## Visita 2

### 1. Lo que quedó de la visita 1

Primero lo que falló o no se llegó a hacer, en el mismo orden.

### 2. Misión de la etapa C

Los pasos concretos se escriben cuando la etapa C esté implementada. Lo que tiene que salir de esta visita, según la consigna:

- **Video** del robot explorando solo, viendo un cono rojo y navegando hasta él. Es un entregable.
- Un bag de cada intento, con las imágenes.
- Una corrida con conos de otros colores a la vista: no tiene que ir hacia ellos.
- Una corrida con un cono rojo visible a través de un hueco: tiene que rodear la pared, no atravesarla.
- Anotado qué falló en cada intento (percepción, localización, planificación o control). Es la pregunta 2 del informe.

## Antes de irse (las dos visitas)

- Guardar todo en `sesiones/<fecha>/` del repo: bags, mapa (`aula.pgm`, `aula.yaml`), los `.log` de `/tmp/slam/`, una captura de RViz, fotos y videos. `git push` desde la PC, o copiar a un pendrive.
- Anotar las medidas, la velocidad de manejo y cualquier choque o patinada.

## Después de cada visita

Cada bag se convierte y se analiza con los comandos de [Analizar un bag](guia_etapa_b.md#analizar-un-bag). Con eso se completan las tablas pendientes de las decisiones 004, 010, 011 y 012, y se escribe la bitácora de la visita.
