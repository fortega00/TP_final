# aula_hueco: pose real al terminar

Corrida del 2026-11-24 en Gazebo Fortress, perfil `mission/config/sim.yaml` con `approach_distance: 0.60`. Medida con `ros2 topic echo /ground_truth/odom --once` después de `APPROACH -> DONE`.

El mundo está corrido en −spawn: pose en el mapa = pose en el mundo + (−0,8; 0).

| | x (m) | y (m) |
|---|---|---|
| Robot, pose real en el mundo | −0,596 | 5,189 |
| Robot, pose real en el mapa | −1,396 | 5,189 |
| Objetivo de aproximación (misión) | −1,38 | 5,11 |
| Cono, posición estimada al terminar | −0,96 | 4,87 |
| Cono, posición en el escenario | −0,90 | 4,85 |

- Error de la posición estimada del cono: 6 cm.
- Distancia real del robot al cono: 0,60 m (pedida: 0,60 m).
- Distancia real del robot al objetivo: 8 cm; el navegador informó 7 cm.
- El robot terminó del lado del cono de la pared con hueco (y = 5,19 > 4,55): vio el cono por el hueco desde abajo y rodeó la pared.
