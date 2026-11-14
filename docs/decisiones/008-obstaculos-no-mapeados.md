# 008. Obstáculos que no están en el mapa

Estado: decidida el 2026-11-14 y ampliada el mismo día por [011](011-entorno-que-cambia.md). Sin implementar. No entra en la sesión con el robot real del 2026-11-16, donde solo va la parada de seguridad.

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

## Ampliación: también se liberan celdas del mapa estático

Tal como estaba, solo se liberaban los obstáculos detectados en vivo. El laboratorio tiene sillas con ruedas que cambian de lugar entre sesiones, y una silla guardada en el mapa seguiría bloqueando un paso libre. Por eso el mapa de costos es una grilla viva que arranca del mapa estático, y el lidar agrega y libera cualquier celda. Las alternativas y los datos están en [011](011-entorno-que-cambia.md). La localización sigue usando el mapa fijo.

## Datos

Pendiente: obstáculos chicos evitados sobre intentos, tiempo desde la detección hasta retomar la marcha, y distancia mínima al obstáculo.
