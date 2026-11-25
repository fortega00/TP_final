# aula_punto: pose real al terminar

Corrida del 2026-11-24 en Gazebo Fortress, perfil `mission/config/sim.yaml` con `approach_distance: 0.60`, cono en (−1,22; −1,95), sobre el camino entre los dos primeros puntos de observación. Medida con `ros2 topic echo /ground_truth/odom --once` después de `APPROACH -> DONE`.

El mundo está corrido en −spawn: pose en el mapa = pose en el mundo + (−0,8; 0).

| | x (m) | y (m) |
|---|---|---|
| Robot, pose real en el mundo | −0,252 | −1,219 |
| Robot, pose real en el mapa | −1,052 | −1,219 |
| Objetivo de aproximación (misión) | −1,03 | −1,31 |
| Cono, posición estimada al terminar | −1,19 | −1,89 |
| Cono, posición en el escenario | −1,22 | −1,95 |

- Lo vio en el camino al primer punto, 10 s después de la pose inicial, antes de llegar. `DONE` a los 25 s. No fue al segundo punto ni pasó por `BLOCKED`.
- Error de la posición estimada del cono: 7 cm.
- Distancia real del robot al objetivo: 9 cm; el navegador informó 8 cm.
- Distancia real del robot al cono: 0,75 m (pedida: 0,60 m). El objetivo quedó a 0,60 m del cono estimado, pero a 0,67 m del real por el error de la estimación, y el robot frenó 9 cm antes del objetivo, del lado del cono. Las dos diferencias van en la misma dirección.
- Corrida anterior con el cono justo sobre el segundo punto, en la que no lo encontró: `../aula_punto_cono_en_punto/`.
