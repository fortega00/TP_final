# Corrida larga de la misión, 2026-11-25

Segunda corrida completa de la etapa C en el robot real, con el cono contra la pared de arriba, entre los puntos de observación 5 y 6. Misma configuración que la [corrida corta](../corrida_corta/README.md): throttle de la cámara a 0,5 Hz, `camera_rate:=0.5`, `scan_timeout` 1,0 s. Hay video del robot y grabación de pantalla (no están en el repo).

El log se grabó con el mismo nombre que la corta (`/tmp/mision_corta.log`); la corta ya estaba copiada antes. Esta corrida empieza con el launch de las 10:31.

## Resultado (de `mision.log`)

| Evento | Valor |
|---|---|
| Pose inicial | Primer 2D Pose Estimate rechazado (44 %); el segundo convergió en 2 iteraciones, 6 ms |
| Plan | 10 puntos, 31,9 m, cobertura 96,2 % |
| Puntos visitados | 1, 2, 4 y 5, con giro en cada uno |
| Punto 3 (−1,72; 3,28) | 7 veces `PLANNING -> BLOCKED`; a los 15 s `obstacle did not clear in time, goal dropped` y la misión lo salteó |
| Cono | Candidato a (−5,66; 6,75), confirmado a (−5,63; 6,75) en 2,1 s; probabilidad final 1,0 |
| Llegada | `reached the cone`; el navegador informa 7 cm y −1,0° del objetivo de aproximación (−5,20; 6,63; 165°) |
| Duración | 154 s desde la pose aceptada hasta `DONE` |
| Lidar | Cero cortes de `Lidar unavailable` |
| Localización | Nunca `LOST` |
| Cámara | Cero avisos `No camera pose` |

El cono se ubicó a ojo; no hay medida con cinta de su posición real.

Errores en el log, ninguno detuvo la corrida: el aviso de shaders de RViz al arrancar, el primer click rechazado y tres `Lookup would require extrapolation` de RViz (TF unos milisegundos atrasado para dibujar).

## Archivos

- `mision.log`: salida de la terminal de la misión.
- `estado.log`: `/mission/state` durante la corrida.
- `ros_log/`: log del launch y logs por nodo (`python3_55101` misión, `55103` localización, `55105` navegador, `rviz2_55107`).
- `nav_real.yaml`, `mission_real.yaml`, `commit.txt`, `cambios_sin_commit.diff`: configuración y código de la corrida.
