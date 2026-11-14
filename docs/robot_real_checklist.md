# Checklist para la sesión con el robot real

Lo más importante es grabar un bag propio de toda la sesión. Con el bag se puede repetir todo offline, aunque la corrida en vivo salga mal.

## Antes de mover el robot

1. Levantar drivers, odometría y URDF del RosMaster.
2. Confirmar que llegan los datos: `ros2 topic hz /scan`, `ros2 topic hz /odom`, y que existe el TF entre la base y el láser (`ros2 run tf2_ros tf2_echo base_link laser`, con los nombres de frames que publique el robot).
3. Anotar el frame de la base (`base_link` o `base_footprint`) para pasarlo en `base_frame`.
4. Dejar el robot quieto en el punto de partida. El mapa arranca en `(0,0,0)` ahí.

## Grabar el bag (primero, y durante toda la sesión)

```bash
ros2 bag record -o bag_aula /scan /odom /tf /tf_static
```

Al terminar, `ros2 bag info bag_aula` tiene que mostrar mensajes en `/tf_static`.

## Lanzar el SLAM

Desde la carpeta `slam/` del repositorio, con el paquete `slam` ya compilado en el workspace del robot:

```bash
ros2 launch slam slam.launch.py mode:=real scan_topic:=/scan odom_source:=topic \
  odom_topic:=/odom base_frame:=base_link output_path:=/tmp/slam/aula rviz:=true \
  config_file:=$PWD/config/parcial_bag_scan_match.yaml
```

Uso `parcial_bag_scan_match.yaml` y no `default.yaml` por el tamaño del mapa. El default cubre 8 m para cada lado del punto de partida, y en el bag del Parcial el robot se alejó 9,5 m. Si todas las partículas salen del mapa, el filtro aborta. El perfil del bag cubre 48 × 40 m y es el que está validado con datos del robot real.

En otra terminal, guardar el estado del nodo para medir el tiempo real:

```bash
ros2 topic echo /slam/status > /tmp/slam/status.log
```

## Manejo

- Despacio, sobre todo en los giros. El scan matching busca hasta 10 cm y 0,06 rad alrededor de la odometría en cada actualización.
- Mirar cada puerta desde los dos lados.
- Recorrer todas las zonas y volver al punto de partida para cerrar el lazo.
- Arrancar mirando a una zona con paredes. Hasta tener 200 celdas ocupadas el filtro usa solo odometría.

## Qué mirar mientras corre

En `/slam/status`:

| Campo | Valor esperado |
|---|---|
| `update_seconds` | Menor que el intervalo entre actualizaciones. En la Mac fue 0,073 s de mediana, con 0,19 s disponibles en el bag |
| `queued_scans` | 0 o 1. Si crece, la computadora no llega |
| `dropped_scans` | Sin crecer después del arranque |
| `state` | Nunca `ERROR` |

Si la computadora no llega: bajar `filter.particles` de 8 a 4 (en el bag dio la misma precisión) y repetir.

## Antes de apagar

```bash
ros2 service call /slam/save_map std_srvs/srv/Trigger '{}'
```

Copiar fuera del robot: el bag, `/tmp/slam/aula.pgm` y `.yaml`, `status.log` y una captura de RViz.

## Qué anotar

- Si la sala y el punto de partida son los mismos que en el bag del Parcial.
- Medidas reales de dos o tres paredes o puertas con cinta métrica, para comparar con el mapa.
- Velocidad aproximada de manejo y cualquier patinada o choque.
