# Corrida corta de la misión, 2026-11-25

Primera corrida completa de la etapa C en el robot real: exploró, detectó el cono rojo, lo confirmó y llegó hasta él. Hay video, filmado con el celular (no está en el repo).

## Configuración

- Rama `feat/mission`, commit en `commit.txt`. Cambio sin commitear al momento de la corrida en `cambios_sin_commit.diff`: `navigator.scan_timeout` de 0,50 a 1,0 s en `nav/config/real.yaml`.
- `nav_real.yaml` y `mission_real.yaml`: copias de las configuraciones usadas.
- Mapa: `sesiones/20261116/mapa/intento3_corregido/map.yaml`.
- Robot en `172.23.16.102`, `ROS_DOMAIN_ID=12`, contenedor `rosmaster_humble`.
- Cámara: `camera_throttle_node` adentro del contenedor con `rate_hz:=0.5`; misión con `camera_rate:=0.5`.

## Resultado (de `mision.log`)

| Evento | Valor |
|---|---|
| Pose inicial | Primer 2D Pose Estimate rechazado (45 % del scan lejos de las paredes); el segundo convergió en 2 iteraciones, 5 ms |
| Plan | 10 puntos, 31,9 m, cobertura 96,2 % |
| Puntos visitados antes del cono | 3, con giro (`LOOK_AROUND`) en cada uno |
| Cono | Candidato y confirmación a (−2,89; 3,23), probabilidad 0,999, 2,1 s entre candidato y confirmación |
| Llegada | `reached the cone`, el navegador informa 10 cm y −1,8° del objetivo de aproximación (−2,44; 3,23; 180°) |
| Duración | 130 s desde la pose aceptada hasta `DONE` |
| Lidar | Cero cortes de `Lidar unavailable` |
| Parada de seguridad | 7 veces `PLANNING -> BLOCKED` yendo al punto 2 (−2,72; −0,72); a los 15 s abandonó el objetivo y llegó a un objetivo cercano (−2,64; −0,80) |
| Cámara | 2 avisos `No camera pose within 0.2 s of the image` |

El cono se ubicó a ojo, no en la posición del plan (−4,5; 3,2), así que la diferencia con (−2,89; 3,23) no indica un error. No hay medida con cinta de la posición real del cono.

## Problemas antes de la corrida

1. El enlace `ws/src/mission` era relativo y no resolvía: colcon ignoraba el paquete. Se cambió por la ruta absoluta.
2. Con el throttle a 3 Hz el `/scan` llegaba a la PC a 1,1 a 1,7 Hz (el robot publicaba a 7 Hz). El navegador cancelaba cada objetivo y la misión salteó los 10 puntos en 37 s. Sin la cámara y luego con el throttle a 0,5 Hz, el scan llegó a 7,2 Hz con un hueco máximo de 0,17 s.
3. A mitad de la mañana se cayeron los drivers del robot. El contenedor seguía arriba; `dmesg` mostró que la cámara se volvió a enumerar por USB unos minutos antes. Se reinició el robot. Batería después del reinicio: 10,4 V. No se confirmó la causa.

## Archivos

- `mision.log`: salida de la terminal de la misión (misión, localización, navegador y RViz).
- `estado.log`: `/mission/state` durante la corrida.
- `ros_log/`: log del launch de ROS.
