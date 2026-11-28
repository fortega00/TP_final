# 009. Máquina de estados de la navegación

Estado: decidida el 2026-11-14. Implementada en `nav/nav/core/navigator.py` y probada en el simulador en vivo, con dos diferencias que se detallan abajo.

## Estados

| Estado | Qué hace el robot | Sale cuando |
|---|---|---|
| `WAIT_SCAN` | Quieto y sin objetivo, faltan scans utilizables | Vuelve un scan utilizable → se comprueba localización. Tras un corte, el usuario debe enviar un objetivo nuevo con pose reciente y localizada |
| `WAIT_INITIAL_POSE` | Nada, el filtro no arrancó | Llega `/initialpose` → `LOCALIZING` |
| `LOCALIZING` | Quieto, ráfaga inicial del filtro | Convergió → `IDLE`, o `PLANNING` si hay un objetivo guardado. Tope de iteraciones → `WAIT_INITIAL_POSE` |
| `IDLE` | Quieto, ubicado | Llega `/goal_pose` → `PLANNING` |
| `PLANNING` | Quieto, corre el planificador | Hay camino → `ALIGNING`. No hay camino, ver abajo |
| `ALIGNING` | Gira hacia el camino | Error de orientación chico → `FOLLOWING` |
| `FOLLOWING` | Pure pursuit | Cerca del objetivo → `FINAL_ALIGN`. Obstáculo nuevo sobre el camino → `PLANNING`. Algo dentro de la parada de seguridad → `BLOCKED`. Nube demasiado dispersa → `LOST` |
| `FINAL_ALIGN` | Gira al ángulo del objetivo | Error de ángulo chico → `IDLE` |
| `BLOCKED` | Quieto: un obstáculo no deja avanzar | Se despejó, o pasaron 2 s → `PLANNING`. 15 s sin poder avanzar → abandona el objetivo, `IDLE` |
| `LOST` | Quieto, espera que el filtro vuelva a converger | Convergió → `PLANNING`. Se acabó el tiempo → `WAIT_INITIAL_POSE` |

La falta de scans utilizables lleva a `WAIT_SCAN`, cancela el objetivo y borra el camino. En los demás estados, un `/goal_pose` nuevo durante el movimiento lleva a `PLANNING`. Un `/initialpose` nuevo frena y vuelve a comprobar la localización. Durante `WAIT_SCAN` se conserva la espera del sensor.

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

## Cómo quedó implementada

Dos diferencias con el diseño original:

- Los obstáculos nuevos no se agregan en `BLOCKED` sino todo el tiempo, en una capa aparte ([008](008-obstaculos-no-mapeados.md)). Si uno tapa el camino, el robot pasa directo a `PLANNING`. `BLOCKED` queda para dos casos: no hay camino por obstáculos nuevos, o la parada de seguridad ve algo en un rectángulo delante del robot, de 40 cm de ancho y largo `navigator.stop_distance` desde el centro: 40 cm en Café, 25 cm en el perfil real. Café conserva mayor distancia por el filtro del chasis y la extrínseca del láser ([008](008-obstaculos-no-mapeados.md#corrección-del-perfil-café-en-la-revisión-del-pr-4)). La parada solo aplica al avanzar, no a los giros en el lugar.
- La ráfaga inicial la hace el nodo de localización. El navegador mira el estado que ese nodo publica: `LOCALIZING` mientras dice `NOT_CONVERGED` y `WAIT_INITIAL_POSE` mientras espera un click o se perdió.

Registro de una corrida en el simulador con dos objetivos seguidos:

```
WAIT_INITIAL_POSE -> LOCALIZING -> IDLE
goal (3.19, -8.17, -87 deg)
IDLE -> PLANNING -> ALIGNING -> FOLLOWING -> FINAL_ALIGN
goal reached, 8 cm and 1.8 deg from it
FINAL_ALIGN -> IDLE
goal (0.00, 0.00, 0 deg)
IDLE -> PLANNING -> ALIGNING -> FOLLOWING -> FINAL_ALIGN
goal reached, 14 cm and -2.6 deg from it
FINAL_ALIGN -> IDLE
```

Los demás casos (objetivo antes de ubicarse, objetivo nuevo en marcha, `/initialpose` en marcha, objetivo inalcanzable, obstáculo, localización perdida) están cubiertos por los tests de `nav/tests/test_navigation.py` con un robot simulado, no en vivo.

## Punto abierto

La dispersión de la nube sola no alcanza para saber si el robot está bien ubicado: la ráfaga inicial puede cerrarse en un lugar equivocado ([004](004-localizacion-en-mapa-conocido.md)). Resuelto el 2026-11-17: el nodo de localización informa `LOST` cuando más del 25 % del scan cae lejos de las paredes del mapa, o más del 15 % justo después del click ([004](004-localizacion-en-mapa-conocido.md#detección-de-perdido)), y con eso la máquina frena y espera un `/initialpose` nuevo.

## Objetivo en el lugar del robot

Corregido el 2026-11-24, al probar la etapa C en Gazebo: la misión pide girar en el lugar con un objetivo en la posición actual y otro ángulo ([015](015-maquina-de-estados-de-la-mision.md)). El navegador pasaba por `ALIGNING` hacia un camino de largo cero, cuya dirección sale de milímetros de error de la pose, así que el robot giraba hacia cualquier lado antes del giro pedido. Lo mismo pasaba con un **2D Goal Pose** marcado sobre el robot. Ahora, si el objetivo está a menos de `follower.goal_tolerance_xy` del robot, `PLANNING` pasa directo a `FINAL_ALIGN`. Test: `test_goal_on_the_robot_only_turns_in_place`.

## Datos

Pendiente: diagrama final, y probar en vivo los casos que hoy solo cubren los tests.
