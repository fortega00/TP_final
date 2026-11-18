# 008. Obstáculos que no están en el mapa

Estado: decidida el 2026-11-14, implementada el 2026-11-17 en `nav/nav/core/obstacles.py` y `nav/nav/core/navigator.py`. Probada con tests, con los bags del aula y en un simulador sobre el mapa del aula. Sin probar en el robot real.

En una línea: el lidar anota los obstáculos nuevos en una capa aparte del mapa; si uno tapa el camino, el robot frena y planifica de nuevo con A* sobre mapa más capa.

## Problema

Si el robot encuentra en su camino un obstáculo que no estaba en el mapa, tiene que evitarlo y completar el objetivo. Los obstáculos chicos son obligatorios y los complejos opcionales.

La detección es igual en todas las opciones: llevar los puntos del lidar al mapa con la pose estimada y marcar los que quedan lejos de toda pared conocida.

## Política

| Opción | A favor | En contra |
|---|---|---|
| A. Frenar, agregar el obstáculo al mapa de costos y replanificar con A* | Reusa el planificador, el desvío tiene las mismas garantías de distancia. Resuelve obstáculos complejos si existe algún camino | Un error de pose ubica mal el obstáculo. Necesita una regla para olvidarlo |
| B. Reactivo, chocar y girar como en la Parte 0 | El código existe | Sale del camino sin garantía de volver y puede oscilar |
| C. Planificador local reactivo (campos potenciales, VFH, DWA) | Movimiento continuo sin frenar | Un algoritmo nuevo, con mínimos locales y mucho ajuste |

Elegí A. Debajo hay siempre una parada de seguridad: si el lidar ve algo demasiado cerca en la dirección de avance, la velocidad es cero.

## Memoria del obstáculo

| Opción | A favor | En contra |
|---|---|---|
| A. Hasta el próximo objetivo | Simple y evita oscilaciones | Un obstáculo que se fue sigue bloqueando |
| B. Liberar por trazado de rayos | Una celda se libera cuando un rayo del lidar la atraviesa. Maneja bien obstáculos que se mueven | Más código |
| C. Sin memoria, solo el último scan | Lo mínimo | El robot puede replanificar hacia un obstáculo que dejó de ver |

Elegí B. Es la misma actualización en log-odds que hace la grilla del SLAM (`slam/core/grid.py`).

## Dónde se guardan los obstáculos

[011](011-entorno-que-cambia.md) había elegido un mapa de costos vivo, donde el lidar también libera celdas del mapa estático. Lo medí con los bags del aula antes de implementarlo y lo cambié.

| Opción | A favor | En contra |
|---|---|---|
| A. Mapa de costos vivo: el lidar agrega y libera cualquier celda | Una silla del mapa que ya no está deja de bloquear | En los bags del aula, sin ningún obstáculo nuevo, el lidar borró del 15 al 38 % de las celdas de pared |
| B. Mapa estático fijo y una capa aparte para los obstáculos nuevos | Las paredes no se tocan. Menos código: la capa es una `OccupancyGrid` del SLAM | Una silla del mapa que se fue sigue bloqueando su lugar |

Elegí B. Las paredes del mapa miden unos 20 cm de grosor y la pose tiene unos centímetros de error, así que muchos rayos atraviesan celdas de pared y las liberan. Si las sillas movidas resultan un problema en el aula, la salida es volver a mapear, no borrar paredes.

## Parámetros

Los cuatro están en el grupo `obstacles` de `nav/config/*.yaml`. Los datos salen de `bag_mapa2` y `bag_mapa` del 16/11, donde no había obstáculos nuevos: todo lo que la capa marca ahí es una falsa alarma (o uno de nosotros caminando).

| Parámetro | Valor | Por qué | Alternativas medidas |
|---|---|---|---|
| `wall_distance` | 0,20 m | Un impacto más cerca que esto de una pared del mapa es esa pared. Es el valor más grande que sigue siendo seguro: lo que queda sin detectar sobresale menos de 20 cm de la pared, y el planificador ya no pasa a menos de 22 cm | 0,10 m: 4 a 10 veces más celdas falsas. 0,30 m: menos falsas, pero deja sin ver obstáculos que sobresalen más que la inflación |
| `hits_to_mark` | 3 scans | Una celda es obstáculo cuando la vieron ocupada 3 scans (0,4 s a 7,4 Hz, 8 cm de avance) | 1 scan: 3 a 7 veces más celdas falsas. 4 scans: casi igual que 3 |
| `max_range` | 3 m | Lejos, un error de 1° en la pose corre el punto varios centímetros. 3 m son 15 s de marcha | 4 m: el doble de celdas falsas en `bag_mapa`. 2 m: parecido a 3 m |
| `navigator.retry_period` | 2 s | Sin camino por obstáculos, cada cuánto se planifica de nuevo. Un plan fallido cuesta hasta 1 s | Sin medir |

Con estos valores y una pose de un scan de antigüedad (0,135 s), la capa tuvo en promedio 3 celdas falsas en `bag_mapa2` y 19 en `bag_mapa`, de 768 000. Tomando como camino los 3 m que el robot recorrió después, habría quedado tapado 0 veces en `bag_mapa2` y 5 veces en `bag_mapa`. Las alternativas de la tabla están medidas igual. Actualizar la capa tarda 0,9 ms por scan.

La pose con la que se proyecta el scan tiene que ser reciente: con una pose de 2 scans de antigüedad (0,27 s) las celdas falsas suben a 18 y 84. El nodo usa la última `/nav/pose`, que llega a 10 Hz. En esos bags se manejó al doble de la velocidad de la navegación.

## Cuándo se replanifica

| Opción | A favor | En contra |
|---|---|---|
| A. Solo cuando un obstáculo nuevo queda a menos de la inflación (22 cm) del camino que falta | Un plan por obstáculo. Chequeo barato | Ninguna |
| B. En cada cambio de la capa | Camino siempre óptimo | A* tarda 43 ms de mediana y hasta 1 s en este mapa, y el lidar da 7 scans por segundo |
| C. Cada cierto tiempo | Simple | Reacciona tarde o planifica de más |

Elegí A.

## Cómo quedó

1. Cada scan actualiza la capa, también con el robot quieto y sin objetivo.
2. Con el robot en marcha, si una celda de la capa queda a menos de 22 cm del camino que falta: frena y planifica con mapa más capa.
3. Si hay camino, gira y sigue. Si no hay, mira la causa: si el mapa solo tampoco tiene camino rechaza el objetivo; si lo tapan obstáculos nuevos pasa a `BLOCKED`, prueba de nuevo cada 2 s y a los 15 s abandona ([009](009-maquina-de-estados.md)).
4. Un `/initialpose` nuevo vacía la capa: lo anotado con la pose anterior quedó en lugares equivocados.
5. La parada de seguridad sigue debajo de todo.

## Datos

Simulador sin ROS sobre el mapa del aula (`tp4_final/review/lab_obstacle_sim.py` del repo personal): robot cinemático, lidar por trazado de rayos con la geometría del robot real, pose con error de unos 3 cm y 1° y un ciclo de atraso. 150 rutas al azar de 3 a 8 m por escenario. El obstáculo es un cilindro puesto sobre la ruta.

| Escenario | Llega | Choques | Distancia mínima al obstáculo | Replanificaciones | Parada por replanificar |
|---|---|---|---|---|---|
| Sin obstáculo | 122 de 150 (81 %) | 0 | no aplica | 0 | no aplica |
| Caja de 40 cm | 125 de 150 (83 %) | 0 | 24 cm del borde al centro del robot | 1 de mediana, 7 como máximo | 0,4 s de mediana |
| Obstáculo de 90 cm | 120 de 148 (81 %) | 0 | 23 cm | 3 de mediana, 33 como máximo | 0,5 s de mediana |

En las 2 rutas donde el obstáculo grande no dejaba paso, el robot esperó y abandonó el objetivo sin tocarlo.

La evasión no baja la tasa de llegada. El 19 % que no llega tampoco llega sin obstáculos, por la parada de seguridad: ver el punto abierto.

### En vivo en Gazebo (café), con los nodos ROS

Caja de 30 cm que no está en el mapa, puesta sobre el camino. Pose inicial con 36 cm de error.

| Corrida | Llega | Error real al llegar | Distancia mínima al borde de la caja | Replanificaciones |
|---|---|---|---|---|
| Sin caja (2) | sí | 22 a 23 cm | no aplica | 0 |
| Caja 2 m adelante, puesta antes del objetivo | sí | 24 cm | 0,65 m | 0: ya estaba anotada y el primer plan la rodea |
| Caja que aparece con el robot en marcha, 1,9 m adelante | sí | 16 cm | 1,05 m | 1. Detectada a los 0,5 s, avanza de nuevo 1,5 s después |
| Caja 3,2 m adelante, al lado del objetivo | sí | 22 cm | 0,35 m | 1 |
| Caja que cierra el único pasillo | no, como se espera | no aplica | 1,57 m | Espera, 7 reintentos y abandona a los 14,9 s |

Sin choques. La primera tanda, con la detección de perdido anterior, terminó en `LOST` en 3 de 5 corridas con caja: por eso cambió esa señal ([004](004-localizacion-en-mapa-conocido.md#detección-de-perdido)).

Una celda que el lidar ya vio libre muchas veces tarda más de 3 scans en marcarse: en Gazebo, de 0,6 a 1,6 s. Probé no acumular evidencia de libre (marca siempre en 3 scans) y lo descarté: en el simulador del aula subió las replanificaciones (máximo 92 contra 33) y los fracasos lejos del objetivo (10 contra 7).

## Punto abierto: la parada de seguridad frena contra paredes del mapa

El rectángulo de la parada llega a 40 cm del centro del robot, y el planificador acepta objetivos y pasos a 22 cm de una pared. Un objetivo cerca de una pared, de frente, no se puede alcanzar: el robot frena, espera 15 s y abandona. Antes no se veía porque las pruebas sin lidar no activaban la parada.

| `navigator.stop_distance` | Llega sin obstáculo | Con caja | Con obstáculo grande | Choques |
|---|---|---|---|---|
| 0,40 m (actual) | 81 % | 83 % | 81 % | 0 |
| 0,30 m | 93 % | 89 % | 90 % | 0 |
| 0,25 m | 96 % | 95 % | 94 % | 0 |

Opciones: bajar `stop_distance` (un número), o que la parada ignore los impactos que son paredes del mapa (más código, depende de la pose). Falta el largo real del robot, que se mide en la visita 1, para saber cuánto margen queda delante del chasis. Sin decidir.

## Sin probar

- El robot real: paso 6 de la [guía de laboratorio](../robot_real_checklist.md).
- Obstáculos más bajos que el plano del lidar, por ejemplo la base de una silla.
- Obstáculos que se mueven mientras el robot los rodea.
