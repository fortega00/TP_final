# 006. Mapa de costos y planificador

Estado: decidida el 2026-11-14. Sin implementar. La elección entre A* y Theta* se cierra con una comparación offline.

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

Pendiente, para A* y Theta* sobre el mismo mapa y los mismos pares inicio y objetivo: tiempo de planificación, largo del camino, distancia mínima y media a los obstáculos.
