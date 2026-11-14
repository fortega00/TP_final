# 009. Máquina de estados de la navegación

Estado: decidida el 2026-11-14. Sin implementar.

## Estados

| Estado | Qué hace el robot | Sale cuando |
|---|---|---|
| `WAIT_INITIAL_POSE` | Nada, el filtro no arrancó | Llega `/initialpose` → `LOCALIZING` |
| `LOCALIZING` | Quieto, ráfaga inicial del filtro | Convergió → `IDLE`, o `PLANNING` si hay un objetivo guardado. Tope de iteraciones → `WAIT_INITIAL_POSE` |
| `IDLE` | Quieto, ubicado | Llega `/goal_pose` → `PLANNING` |
| `PLANNING` | Quieto, corre el planificador | Hay camino → `ALIGNING`. No hay camino, ver abajo |
| `ALIGNING` | Gira hacia el camino | Error de orientación chico → `FOLLOWING` |
| `FOLLOWING` | Pure pursuit | Cerca del objetivo → `FINAL_ALIGN`. Camino bloqueado → `BLOCKED`. Nube demasiado dispersa → `LOST` |
| `FINAL_ALIGN` | Gira al ángulo del objetivo | Error de ángulo chico → `IDLE` |
| `BLOCKED` | Quieto, agrega el obstáculo al mapa de costos | Enseguida → `PLANNING` |
| `LOST` | Quieto, espera que el filtro vuelva a converger | Convergió → `PLANNING`. Se acabó el tiempo → `WAIT_INITIAL_POSE` |

Eventos globales: un `/goal_pose` nuevo en cualquier estado en movimiento lleva a `PLANNING`, y un `/initialpose` nuevo en cualquier estado frena al robot y lleva a `LOCALIZING`.

## Casos que eran una decisión

**Llega un `/initialpose` mientras navega.**

| Opción | A favor | En contra |
|---|---|---|
| A. Conservar el objetivo y replanificar | El usuario estaba corrigiendo la pose, no cancelando el viaje | Una transición más |
| B. Descartar el objetivo | Más simple | Hay que volver a marcar el objetivo |

Elegí A.

**Llega un `/goal_pose` antes de estar ubicado.**

| Opción | A favor | En contra |
|---|---|---|
| A. Guardarlo y planificar al terminar de ubicarse | Coherente con "un objetivo nuevo reemplaza al anterior" | Un dato más que mantener |
| B. Rechazarlo con un error en el log | Menos estados que razonar | Hay que volver a marcarlo |

Elegí A. Vale el último objetivo recibido.

**El planificador no encuentra camino.**

| Opción | A favor | En contra |
|---|---|---|
| A. Rechazar o esperar según la causa | Si el mapa estático solo ya no tiene camino (objetivo en una pared o en zona desconocida) se rechaza enseguida. Si lo bloquean solo obstáculos detectados, se espera y se reintenta mientras el trazado de rayos puede liberarlos, con un tope de tiempo | Hay que planificar dos veces para saber la causa |
| B. Rechazar siempre enseguida | Lo más simple | Una persona parada en una puerta cancela el viaje |

Elegí A.

**Se pierde la localización en el camino** (la nube se dispersa más que un umbral).

| Opción | A favor | En contra |
|---|---|---|
| A. Frenar, volver a converger y replanificar | Seguro, es el manejo de incertidumbre que evalúa la consigna | Una zona mal mapeada puede dejar al robot parado, por eso el tope de tiempo |
| B. Bajar la velocidad y seguir | Nunca se queda parado | Avanza con una pose en la que no confía |
| C. Ignorarlo | Lo más simple | Riesgo de choque |

Elegí A.

## Punto abierto

La dispersión de la nube sola no alcanza para saber si el robot está bien ubicado: la ráfaga inicial puede cerrarse en un lugar equivocado ([004](004-localizacion-en-mapa-conocido.md)). La condición de salida de `LOCALIZING` y la de entrada a `LOST` se definen cuando esté medida una señal mejor.

## Datos

Pendiente: diagrama final y registro de transiciones de una corrida completa.
