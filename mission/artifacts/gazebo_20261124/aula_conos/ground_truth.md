# aula_conos: pose real al terminar

Corrida del 2026-11-24 en Gazebo Fortress, perfil `mission/config/sim.yaml` con `approach_distance: 0.60`. Logs `mision_captura.log` y `estado_captura.log`; capturas `rviz.png` y `rviz_final.png`. Medida con `ros2 topic echo /ground_truth/odom --once` después de `APPROACH -> DONE`.

`mision.log` y `estado.log` son de la primera corrida del escenario, a las 00:20, con `approach_distance: 0.45`: también terminó en `DONE`, sin capturas ni pose real.

El mundo está corrido en −spawn: pose en el mapa = pose en el mundo + (−0,8; 0).

| | x (m) | y (m) |
|---|---|---|
| Robot, pose real en el mundo | −4,518 | 3,033 |
| Robot, pose real en el mapa | −5,318 | 3,033 |
| Objetivo de aproximación (misión) | −5,24 | 3,04 |
| Cono rojo, posición estimada al terminar | −4,66 | 3,20 |
| Cono rojo, posición en el escenario | −4,60 | 3,20 |

- `DONE` 8,75 min después de la pose inicial. Un solo candidato en toda la misión, el rojo: los conos naranja, azul y amarillo no frenaron al robot.
- Error de la posición estimada del cono: 6 cm.
- Distancia real del robot al objetivo: 8 cm; el navegador informó 7 cm.
- Distancia real del robot al cono: 0,74 m (pedida: 0,60 m). El objetivo quedó a 0,60 m del cono estimado y a 0,66 m del real, y el robot frenó 8 cm antes del objetivo, del lado del cono.
