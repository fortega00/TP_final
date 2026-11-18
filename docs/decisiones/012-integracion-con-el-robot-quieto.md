# 012. Integración con el robot quieto

Estado: implementada en la rama `fix/parked-drift` (2026-11-16) y probada offline con los dos bags de la [sesión del 16/11](../sesion_robot_20261116.md). No se corrió todavía en el nodo ROS ni en el robot.

## Problema

El filtro integraba un scan si el robot se había movido 0,08 m, girado 0,06 rad, o si había pasado 1 s desde la última integración (`update.max_interval`). Con el robot quieto, esa tercera condición dispara una integración por segundo. Cada integración pasa por la propuesta de scan matching, que devuelve la mejor pose más un ruido chico (`sample_xy` 0,01 m), y después dibuja el scan en el mapa desde esa pose. La pose camina sola un poco en cada paso y el mapa cercano se reescribe con ella, entonces no hay nada que la traiga de vuelta.

Medido en los bags del laboratorio, con la odometría exactamente en cero y los scans sin cambios:

| Bag | Tiempo quieto al inicio | Integraciones en ese tiempo | Deriva de la pose (8 semillas) |
|---|---|---|---|
| `bag_mapa` | 455 s | 412 | mediana 5,3 cm, máximo 7,6 cm |
| `bag_mapa2` | 215 s | 207 | mediana 13,1 cm, máximo 18,2 cm |

En `bag_mapa2` el robot estaba de frente a una pared a unos 10 cm, con 40 % de los haces sin retorno, y la deriva fue a lo largo de esa pared. Consecuencia: el `(0,0)` de `mapa/intento3` quedó a unos 20 cm del lugar físico de partida.

La deriva no explica el grosor de las paredes (unos 20 cm): recortar la parte quieta de los bags no lo cambia.

## Alternativas

| Opción | A favor | En contra |
|---|---|---|
| A. Sacar la integración por tiempo: solo se integra si el robot se movió | El cambio más chico: se borra una condición y un parámetro. Coincide con lo que ya decía el comentario del filtro (integrar con el robot quieto repite la misma evidencia) | El mapa inicial sale de un solo scan. Los cambios de la sala con el robot quieto no se mapean |
| B. Dejar el código y recortar a mano los bags | Sin cambios de código | No sirve en vivo. La etapa C tiene pausas |
| C. Mantener la integración por tiempo, pero solo actualizar los mapas desde la pose actual, sin muestrear una pose nueva | Sin deriva y el mapa inicial se refuerza mientras el robot espera | Un camino más en el filtro (actualiza mapas sin predecir ni pesar) y se conserva el parámetro |

Elegí A.

## Datos

Replays offline del núcleo con `config/parcial_bag_scan_match.yaml`, 8 semillas (7, 1, 2, 3, 4, 5, 6, 8) por bag. "Entre semillas" compara todos los pares: RMS de la distancia entre trayectorias y fracción de celdas ocupadas de un mapa que tienen una celda ocupada del otro a 5 cm o menos. No hay pose real: esto mide repetibilidad, no exactitud.

| Bag | Versión | Integraciones | Deriva quieto, mediana | Trayectorias entre semillas, mediana / máximo | Celdas ocupadas a 5 cm, mediana / mínimo |
|---|---|---|---|---|---|
| `bag_mapa2` | antes | 963 | 13,1 cm | 8,3 / 18,5 cm | 0,89 / 0,59 |
| `bag_mapa2` | A | 691 | 0 cm | 6,4 / 10,0 cm | 0,95 / 0,84 |
| `bag_mapa2` | C (prototipo) | 691 | 0 cm | 6,6 / 11,3 cm | 0,94 / 0,83 |
| `bag_mapa` | antes | 1111 | 5,3 cm | 7,6 / 12,8 cm | 0,93 / 0,79 |
| `bag_mapa` | A | 585 | 0 cm | 12,5 / 18,7 cm | 0,84 / 0,63 |
| `bag_mapa` | C (prototipo) | 585 | 0 cm | 7,5 / 15,7 cm | 0,94 / 0,74 |

Lecturas:

- A y C eliminan la deriva en los dos bags.
- En `bag_mapa2`, el bag del mapa elegido, A mejora la repetibilidad y C da lo mismo que A.
- En `bag_mapa`, A empeora la repetibilidad y C la deja como antes. Hipótesis sin probar: con A el mapa inicial sale de un solo scan de 180 haces, y en ese bag el robot arranca a 0,4 a 0,6 m/s contra ese mapa pobre. Antes, las 412 integraciones quietas lo reforzaban.
- La pose final de `bag_mapa2` (el robot volvió al lugar de partida) pasa de `(0,07, 0,25)` m a `(-0,01, 0,12)` m, con las 8 semillas dentro de 7 cm.

Queda abierto si conviene pasar a C. El dato a favor viene de una sola corrida, manejada más rápido de lo recomendado. El prototipo de C no está en el repo.

## Qué cambió

- `slam/slam/core/filter.py`: el filtro descarta el scan si el robot no superó `min_translation` ni `min_rotation`, sin excepción por tiempo.
- `update.max_interval` ya no existe. Un YAML que lo tenga falla al cargar, con el error de parámetro desconocido.
- Prueba nueva en `slam/tests/test_scan_match.py`: robot quieto 5 minutos con el mismo scan a 7,4 Hz. Con el código anterior la pose terminaba a 5 cm del origen y la prueba fallaba.

## Mapa regenerado

`sesiones/20261116/mapa/intento3_corregido/` es el mapa de `bag_mapa2` con el código corregido (semilla 7, replay offline del núcleo). Comparado con `intento3`:

| Medición | `intento3` | `intento3_corregido` |
|---|---|---|
| Zona libre | 108,2 m² | 108,6 m² |
| Lugar físico de partida, según la localización de los dos bags | `(-0,02, 0,20)` m | `(0,02 a 0,09, 0,07)` m |
| Distancia libre en el origen | 0,05 m | 0,15 m |
| Zona transitable con `nav/config/real.yaml` | 70,3 m² en una sola pieza | 70,6 m² en una sola pieza |
| Localización de `bag_mapa2` contra su SLAM, directo / con ajuste rígido | 13,7 / 3,7 cm | 4,0 / 3,6 cm |
| Localización de `bag_mapa` (otra corrida) contra su SLAM, con ajuste rígido | 4,6 a 4,9 cm | 5,2 cm |

El origen sigue sin ser transitable porque el robot arrancó casi tocando la pared. El planificador mueve el inicio a la celda transitable más cercana.

## Pendiente

| Medición | Estado |
|---|---|
| Correr el nodo ROS con el cambio sobre un bag | hecho el 17/11 con `bag_mapa2`: mismo resultado que el replay, 0,00 cm de movimiento con el robot quieto ([bitácora](../sesion_robot_20261116.md#pruebas-offline-del-1711)) |
| Robot quieto varios minutos en el laboratorio, con el SLAM en vivo | pendiente |
| Decidir entre A y C con una corrida más, manejada a 0,15 m/s | pendiente |

Los replays y los scripts de comparación están en `tp4_final/review/` del repo personal (`lab_parked_fix.py`, `lab_map_checks.py`, `lab_map_register.py`).
