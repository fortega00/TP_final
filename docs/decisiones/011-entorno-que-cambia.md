# 011. Un laboratorio que cambia entre sesiones

Estado: decidida el 2026-11-14, con una prueba sintética en simulación. Falta la prueba con el robot real del 2026-11-16.

## Problema

El laboratorio tiene mesas con sillas con ruedas. Las sillas nunca quedan dos veces en el mismo lugar, entonces el mapa guardado y el laboratorio del día del experimento siempre difieren un poco. Los algoritmos tienen que ser simples y tolerar esa diferencia.

## Dónde afecta

| Parte | Qué pasa con una silla que se movió | Exposición |
|---|---|---|
| Localización | El scan tiene puntos que el mapa no explica y el mapa tiene celdas de silla donde ya no hay nada | Baja, ver los datos |
| Planificación | Una silla guardada en el mapa bloquea para siempre un paso que está libre | Alta |
| Marcha | Una silla donde el mapa dice libre es un obstáculo no mapeado | Alta |

La localización tolera el cambio por el modelo de sensor que ya usa. El campo de verosimilitud solo pregunta si cada punto del scan cae cerca de un obstáculo del mapa. Un punto sin explicación cuesta una cantidad acotada (el término uniforme `z_rand`) y una celda ocupada del mapa donde ya no hay nada no cuesta nada. Un modelo de haces con trazado de rayos penalizaría cada silla movida.

## Decisiones

**Cómo se arma el mapa.**

| Opción | A favor | En contra |
|---|---|---|
| A. Ordenar las sillas debajo de las mesas y mapear la sala como está | El mapa guarda la posición más repetible de las sillas. Sin algoritmo extra | Depende de acordarse de ordenar |
| B. Mapear como está y borrar del mapa las manchas chicas aisladas | Mapa más limpio | Un paso más de procesamiento, que también puede borrar obstáculos finos reales, como patas de mesa |
| C. Mapear como está | Sin preparación | El mapa guarda las sillas donde cayeron |

Elegí A.

**Si el mapa de costos puede liberar celdas del mapa estático.**

| Opción | A favor | En contra |
|---|---|---|
| A. Mapa de costos vivo que arranca del mapa estático | El lidar agrega y libera celdas con la misma actualización en log-odds del SLAM. Un solo mecanismo cubre obstáculos nuevos y sillas que se fueron | Un error de pose puede borrar una pared real por un rato |
| B. Mapa estático fijo, solo se agregan obstáculos | Más simple de razonar | Una silla movida sigue bloqueando su lugar viejo |

Elegí A. La localización sigue usando el mapa fijo. Esto cambia la decisión [008](008-obstaculos-no-mapeados.md).

**Cambiada el 2026-11-17 a B**, con datos: en los bags del aula el lidar borró del 15 al 38 % de las celdas de pared. Detalle en [008](008-obstaculos-no-mapeados.md#dónde-se-guardan-los-obstáculos).

**Si se mide con el robot real.** Sí: grabar un bag, mover las sillas y grabar otro sobre el mismo recorrido. Son datos directos para las preguntas 4 y 7 del informe.

## Datos de simulación

Prueba sintética sobre la misma grabación y el mismo mapa de [004](004-localizacion-en-mapa-conocido.md). Las sillas son discos de 20 cm de radio en celdas libres, a más de 60 cm del recorrido. `nav/scripts/chairs_sim.py` las agrega a los scans (sillas en la sala que no están en el mapa), al mapa (sillas en el mapa que ya no están en la sala), o a los dos con posiciones distintas (sillas que se movieron). El click inicial tiene 30 cm y 9° de error. 1000 partículas, semillas 1 / 2 / 7.

RMSE de posición en cm:

| Caso | Sillas | Tramo 1 | Tramo 2 |
|---|---|---|---|
| Sin cambios | 0 | 11,5 / 11,6 / 11,4 | 6,1 / 6,8 / 6,9 |
| Solo en el mapa | 10 | 11,4 / 11,4 / 11,4 | 6,7 / 6,3 / 6,8 |
| Solo en el mapa | 60 | 11,3 / 11,3 / 11,2 | 5,8 / 5,7 / 6,0 |
| Solo en la sala | 10 | 12,8 / 12,8 / 12,9 | 8,9 / 9,0 / 8,9 |
| Solo en la sala | 60 | 14,4 / 14,6 / 14,7 | 8,2 / 7,9 / 7,8 |
| Se movieron | 10 | 13,9 / 13,9 / 13,7 | 10,0 / 11,6 / 9,8 |
| Se movieron | 30 | 15,2 / 15,6 / 15,7 | 14,6 / 13,7 / 13,7 |
| Se movieron | 60 | 15,7 / 19,6 / 20,1 | 13,5 / 13,9 / 13,7 |

Lecturas:

- Sillas que están en el mapa y no en la sala no cambian nada, como predice el modelo.
- Sillas nuevas en la sala suman 1 a 3 cm.
- Sillas que se movieron suman 2 a 8 cm. Es peor que la suma de los dos casos anteriores porque una silla nueva cerca de una silla vieja del mapa se asocia con ella y tira de la pose.
- Con 60 sillas, que es un caso extremo para 80 m², el error llega a 20 cm y la ráfaga inicial arranca hasta 40 cm corrida, pero el filtro no se pierde en ninguna corrida.
- El residuo del scan contra el mapa sube de 3 cm a 17 o 26 cm con las sillas. Mide cuánto cambió la sala, no cuánto se equivoca la pose.

Límite de la prueba: el piso de 6 a 11 cm viene del mapa de los profesores y los discos son más grandes que lo que el lidar ve de una silla real.

## Riesgo físico a verificar

La base con ruedas de una silla es más ancha que su columna y puede quedar por debajo del plano del lidar. Si el lidar solo ve la columna, el robot puede enganchar la base. El margen de inflación tiene que cubrir esa diferencia. Hay que medir la altura del lidar contra una silla.

## Pendiente

| Medición | Robot real |
|---|---|
| Diferencia de trayectoria entre el bag con sillas ordenadas y el bag con sillas movidas, sobre el mismo mapa | pendiente |
| Altura del lidar, ancho de la base de una silla y ancho de lo que el lidar ve de ella | pendiente |
