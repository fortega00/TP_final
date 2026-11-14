# 006. Mapa de costos y planificador

Estado: decidida el 2026-11-14. A* implementado en `nav/nav/core/planner.py` y probado en el simulador. La comparación contra Theta* está pendiente.

## Problema

La consigna pide un camino sin choques entre la pose actual y el objetivo, que además no pase demasiado cerca de los objetos "de ser posible", y evalúa la velocidad con que se genera.

## Celdas desconocidas

| Opción | A favor | En contra |
|---|---|---|
| A. Tratarlas como obstáculo | El robot solo pasa por espacio verificado | Un objetivo marcado en zona desconocida se rechaza |
| B. Tratarlas como libres | Caminos más cortos, cualquier objetivo es alcanzable | Manda al robot a un lugar que nadie verificó |

Elegí A. El rechazo de un objetivo inalcanzable lo maneja la máquina de estados.

## Distancia a las paredes

| Opción | A favor | En contra |
|---|---|---|
| A. Solo inflación dura (radio del robot más un margen) | Un parámetro | Con margen grande se cierran las puertas, con margen chico el camino roza las paredes |
| B. Inflación dura con margen chico más un costo que decae con la distancia a los obstáculos | El camino se centra en los pasillos y aun así entra por una puerta. Es lo que dice "de ser posible" | Dos parámetros más |

Elegí B. El costo sale de la transformada de distancia del mapa, la misma que usa la localización.

## Algoritmo

| Opción | A favor | En contra |
|---|---|---|
| A. A* sobre la grilla de 8 vecinos | Óptimo para el costo, respeta el costo de cercanía. Rápido con heurística | Caminos en zigzag a múltiplos de 45°, que el seguidor suaviza |
| B. Theta* (lo elegido en el TP2) | Caminos en cualquier ángulo, más cortos y rectos | Los atajos por línea de vista saltean el costo de cercanía y rozan las esquinas infladas |
| C. Dijkstra | El más simple | Más lento sin ninguna ventaja |

Elegí A como base y comparar contra B antes de cerrarlo. Los dos son Python puro, así que se miden offline sobre el mismo mapa de costos.

## Datos

Valores usados: radio del robot 0,17 m más margen de 0,05 m, costo de cercanía con peso 5 que decae en 0,50 m. El radio hay que medirlo en el robot.

Un caso que apareció al implementar: si el robot arranca dentro de la zona inflada (pegado a una pared, o con un error chico de pose), el camino empieza en la celda transitable más cercana, hasta 0,30 m. Si no hay ninguna a esa distancia, se rechaza.

A* en la Mac, Python puro:

| Mapa | Inicio y objetivo | Tiempo (ms) | Largo (m) | Distancia mínima a obstáculos (m) | Distancia media (m) |
|---|---|---|---|---|---|
| Café, 190 × 449 celdas | (0, 0) a (3,1, -8,2) | 20 | 9,52 | 0,71 | 1,17 |
| Café | (0, 0) a (-3,5, -8,5) | 25 | 9,91 | 0,73 | 1,38 |
| Café | (3,1, -2,4) a (-3,5, -2,7) | 3 | 6,76 | 0,79 | 1,43 |

Armar el mapa de costos tarda 2 ms.

Pendiente: la misma tabla para Theta*, y repetirla sobre el mapa del laboratorio.
