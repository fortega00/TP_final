# aula_lejos: pose real al terminar

Corrida del 2026-11-24 en Gazebo Fortress, perfil `mission/config/sim.yaml` con `approach_distance: 0.60`, cono en (−8,5; 6,9), en la esquina de arriba a la izquierda, cerca del último punto del recorrido. Con la búsqueda de la pose de aproximación sin holgura cuando no hay lugar con ella ([017](../../../../docs/decisiones/017-ubicacion-y-confirmacion-del-cono.md)). Medida con `ros2 topic echo /ground_truth/odom --once` después de `APPROACH -> DONE`.

El mundo está corrido en −spawn: pose en el mapa = pose en el mundo + (−0,8; 0).

| | x (m) | y (m) |
|---|---|---|
| Robot, pose real en el mundo | −7,127 | 6,551 |
| Robot, pose real en el mapa | −7,927 | 6,551 |
| Objetivo de aproximación (misión) | −7,98 | 6,52 |
| Cono, posición estimada al terminar | −8,40 | 6,94 |
| Cono, posición en el escenario | −8,50 | 6,90 |

- Recorrió el plan sin puntos salteados ni objetivos abandonados. Vio el cono 13 min 28 s después de la pose inicial, al llegar al último punto. `DONE` a los 13 min 46 s.
- Error de la posición estimada del cono: 11 cm, el mayor de los escenarios (6–7 cm en los otros). El cono estaba en una esquina, a 1,3 m.
- Distancia real del robot al objetivo: 6 cm; el navegador informó 9 cm.
- Distancia real del robot al cono: 0,67 m (pedida: 0,60 m). Se acercó en diagonal, a 135°, porque en la esquina no hay lugar con 0,5 m de holgura.
- Primer intento, antes de la corrección: confirmó el cono y lo abandonó como inalcanzable, `EXHAUSTED` con el 96,9 % del piso visto (`../aula_lejos_intento1/`).
