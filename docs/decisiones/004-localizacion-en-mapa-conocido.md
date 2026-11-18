# 004. Localización en mapa conocido (etapa B)

Estado: implementada en la rama `nav` (2026-11-14), medida sin ROS sobre una grabación del simulador y probada con el nodo ROS en el simulador en vivo y con el bag real del Parcial. Falta repetir las mediciones con un mapa hecho por nuestro SLAM y la sesión con el robot real del 2026-11-16. Las tablas marcadas como pendientes se completan con esos datos.

## Problema

La etapa B pide estimar la pose del robot en el mapa de la etapa A con un filtro probabilístico, usando odometría y lidar. El robot arranca sin saber dónde está y recibe una pose aproximada por `/initialpose` (la herramienta 2D Pose Estimate de RViz, que siempre manda un desvío de 0,5 m y 15°).

## Método

| Opción | A favor | En contra |
|---|---|---|
| A. Filtro de partículas (MCL) con campo de verosimilitud | Usa la grilla directamente, tolera un click impreciso, puede recuperarse | Más hiperparámetros |
| B. EKF | Barato, pocos parámetros | Necesita un scan matcher adelante para tener una medición y es unimodal, un click malo no se recupera |
| C. FastSLAM con el mapa congelado | Sin código nuevo | Es MCL cargando un mapa por partícula |

Elegí A. Está en `nav/nav/core/localization.py` y reusa de `slam.core` el modelo de movimiento, la preparación del scan, la fórmula del campo de verosimilitud, el remuestreo sistemático y el scan matching. El mapa se carga como una `OccupancyGrid` de `slam.core`, así el modelo de sensor es el mismo que construyó el mapa.

Como el mapa es uno solo y no cambia, las partículas son un arreglo `(N, 3)` y el campo de distancias se calcula una vez. Eso permite pesar todas las partículas juntas: 1000 partículas tardan 4 a 5 ms por actualización en la Mac.

## Modificaciones al filtro básico

| Modificación | Para qué | Estado |
|---|---|---|
| Remuestreo solo con ESS bajo | No destruir hipótesis sin necesidad | Adoptada |
| Compuerta de movimiento | Con el robot quieto, scans repetidos vuelven al filtro demasiado seguro | Adoptada |
| Ráfaga inicial con el robot quieto | La compuerta impide converger justo después del click | Adoptada |
| Inyección de partículas al azar (MCL aumentado) | Recuperarse de un click malo | Implementada, sin beneficio medido todavía |
| Corregir el incremento de odometría con scan matching | Converger con menos ruido | Implementada detrás de `scan_match.enabled`, sin beneficio medido todavía |
| Cantidad de partículas adaptativa (KLD) | Menos cómputo | Descartada, el filtro ya es barato |

**Ráfaga inicial.** Después de cada `/initialpose` se pesa la nube con el scan actual, se remuestrea y se suma un ruido chico (3 cm, 0,02 rad), varias veces. Termina cuando la nube mide menos de 10 cm y 0,10 rad y además una iteración más mueve la estimación menos de 1 cm. La segunda condición hace falta: con la nube ancha todo el peso cae en una sola partícula, la dispersión ponderada da cero y la ráfaga cortaba en la primera iteración a 6 u 8 cm de la pose real. Con la condición queda a 1 o 2 cm en la sala de prueba.

**Scan matching.** En SLAM cada partícula ajustaba su pose contra su propio mapa. Acá todas comparten el mapa, entonces se ajusta una sola vez por scan alrededor de la estimación anterior más la odometría, y el movimiento hasta la pose ajustada reemplaza al incremento de las ruedas para todas las partículas. Solo se usa con la nube convergida.

## Datos de simulación

Grabación `tp4_final/review/data/sim_drift_inputs.npz` del repo personal: café en Gazebo Classic, odometría con deriva (`/calc_odom`), pose real del simulador como referencia. Mapa: el del café que traen los profesores (`custom_map/maps/my_map.yaml`), porque todavía no hay uno hecho con nuestro SLAM. Ese mapa no cubre el fondo del café (termina en y = -11,1 m y el recorrido llega a -12,2 m), así que uso los dos tramos que quedan adentro: scans 0 a 360 (20 m) y 670 a 1015 (19,5 m). Cuando todas las partículas salen del mapa el filtro lanza un error, no sigue en silencio.

Se reproduce desde `nav/` con:

```bash
PYTHONPATH=../slam:. python scripts/replay_localization.py --input <npz> --map <yaml> \
  --config config/cafe.yaml --output /tmp/run.json --last 360 --seed 7
```

**Seguimiento** (click sin error, 1000 partículas, semillas 1 / 2 / 7):

| Configuración | Tramo 1, RMSE (cm) | Tramo 2, RMSE (cm) | ms por actualización |
|---|---|---|---|
| Solo odometría | 173,6 | 184,6 | |
| MCL base | 11,3 / 11,9 / 11,5 | 6,8 / 6,6 / 6,0 | 5 |
| Con scan matching | 11,8 / 12,0 / 12,3 | 6,1 / 5,8 / 6,3 | 10 |
| Sin inyección | 11,6 / 11,5 / 11,6 | 6,3 / 7,0 / 6,6 | 5 |
| 300 partículas | 44,2 / 11,6 / 11,5 | 6,1 / 7,4 / 5,9 | 1,5 |
| 3000 partículas | 11,4 / 11,4 / 11,4 | 6,1 / 6,1 / 6,3 | 18 |
| alpha 0,005 | 11,7 / 12,1 / 11,9 | 7,7 / 7,7 / 8,2 | 5 |
| alpha 0,1 | 11,3 / 11,2 / 11,3 | 6,0 / 5,9 / 5,8 | 5 |
| sigma_hit 0,05 | 12,4 / 12,4 / 12,4 | 6,1 / 5,9 / 5,7 | 5 |
| sigma_hit 0,20 | 11,2 / 11,4 / 11,1 | 6,6 / 8,2 / 7,4 | 5 |

El error de yaw es de 0,4° a 0,8° en todas las filas.

Lecturas:

- El filtro corrige la deriva: de 175 cm a 6 o 12 cm.
- El error casi no cambia con ningún parámetro. En el tramo 1 el scan encaja mejor en el mapa desde la pose estimada (3,4 cm de residuo medio) que desde la pose real (5,3 cm), y el error es un corrimiento lateral estable de 8,5 cm. Eso indica que el piso de error viene del mapa de los profesores, que no coincide del todo con el mundo simulado, y no del filtro. El filtro se ubica respecto del mapa.
- Scan matching no mejora y cuesta el doble. La inyección no cambia el seguimiento.
- Con 300 partículas una semilla arrancó 90 cm corrida (ver abajo) y tardó en recuperarse.

**Click inicial con error** (5 semillas por tramo, 10 corridas por fila, 1000 partículas):

| Error del click | Iteraciones de la ráfaga | Error después de la ráfaga (cm) | Corridas que terminan bien ubicadas |
|---|---|---|---|
| 0 | 2 a 7 | 2,0 a 7,1 | 10 de 10 |
| 30 cm y 9° | 4 a 7 | 1,7 a 5,9 | 10 de 10 |
| 60 cm y 14° | 3 a 10 | 1,7 a 7,3 | 10 de 10 |
| 1 m y 17° | 3 a 9 | 1,6 a 8,1, una en 69 | 10 de 10 |
| 0 cm y 45° | 6 a 23 | 1,7 a 7,8, una en 68 | 10 de 10 |
| 2 m y 29° | 6 a 18 | 65 a 212 | 5 de 9, una abortó fuera del mapa |
| 2 m y 29°, sin inyección | 6 a 18 | 65 a 212 | 6 de 9 |

La ráfaga tarda unos 10 ms. Hasta 1 m de error el robot queda ubicado; cuando la ráfaga cae en un falso encaje a 69 cm, el seguimiento lo corrige al andar. Con 2 m la ráfaga siempre cae en una pose equivocada y la mitad de las corridas no se recupera.

Dos problemas abiertos que muestran estos datos:

1. La ráfaga informa que convergió también cuando encajó en un lugar equivocado. La dispersión de la nube sola no alcanza como medida de confianza.
2. La inyección uniforme en todo el espacio libre no recupera un click a 2 m. Una partícula al azar tiene que caer a pocos centímetros y pocos grados de la pose real para sobrevivir, y con 200 por actualización eso casi no pasa.

## Decisiones de implementación

Las cuatro quedaron confirmadas el 2026-11-14.

| Tema | Elegido | Alternativa | Por qué |
|---|---|---|---|
| Scan matching | Una vez por scan, sobre la estimación | Por partícula, como en el SLAM | Cuesta unos 9 ms por partícula y todas comparten el mismo mapa |
| Cálculo de pesos | Todas las partículas juntas sobre un campo de distancias calculado una vez, con la misma fórmula y parámetros de `slam.core` | Llamar al modelo de `slam.core` partícula por partícula | Ese modelo recalcula las celdas ocupadas del mapa en cada llamada, está pensado para mapas que cambian |
| Fin de la ráfaga | Nube chica y estimación estable | Solo nube chica | Solo con la nube chica cortaba en la primera iteración sobre una única partícula |
| Inyección | Uniforme sobre las celdas libres, con tope de 20 % de las partículas | Alrededor de la estimación | Es la versión del libro. Los datos de abajo muestran que no alcanza, queda abierta |

## Nodo ROS en vivo y bag real

Nodo `nav/ros/localization_node.py` en Gazebo Classic sin ventana, café, odometría con deriva (`/calc_odom`), mapa de los profesores, 1000 partículas. `scripts/sim_live_check.py` publica un `/initialpose` corrido 36 cm y 9° y compara `/nav/pose` con la pose real:

| Corrida | Después de la ráfaga (cm) | RMSE en marcha (cm) | Máximo (cm) | Ráfaga |
|---|---|---|---|---|
| Ida y vuelta de 2 m con teleop automático | 6,2 | 8,3 | 10,3 | 9 iteraciones, 23 ms |
| Un objetivo a 4 m | 6,1 | 6,0 | 8,0 | 6 iteraciones |
| Dos objetivos, 15 m en total | 14,3 | 12,5 | 20,0 | |

El filtro tardó 8 a 18 ms por actualización con el simulador corriendo en la misma máquina.

Bag real del Parcial sobre el mapa del laboratorio de los profesores (`my_map3`), pose inicial `(0, 0, 0)`, sin ROS: 839 actualizaciones, 4,5 ms cada una, la ráfaga converge en 3 a 6 iteraciones. Tres semillas coinciden entre sí a 1,4 cm de RMSE. La pose final es `(0,93, 0,21)`, la misma que dio mi filtro del TP3 sobre ese bag `(0,89, 0,20)`. Contra la trayectoria del SLAM de la etapa A sobre el mismo bag hay 8,3 cm de RMSE con ajuste rígido.

## Métrica sin pose real

Para el robot real pensaba usar el residuo del scan contra el mapa (distancia media de los extremos del scan a la pared más cercana, desde la pose estimada). En simulación, con el filtro ya convergido, el residuo no sigue al error real: la correlación entre los dos dio 0,41 en un tramo y -0,73 en el otro. Para lo que sí sirve es para distinguir ubicado de perdido, ver la sección siguiente.

Las medidas que lo reemplazan en el robot real están en [010](010-metricas-de-evaluacion.md).

## Detección de perdido

Decidida e implementada el 2026-11-17. La señal se cambió el mismo día, al sumar la evasión de obstáculos.

Problema: con un click muy equivocado (90° en el aula, 2 m en el café) la ráfaga se cierra en una pose falsa. La nube queda concentrada, el filtro dice que convergió y el robot navegaría con una pose a metros de la real.

Regla: el nodo informa `LOST` cuando más del 25 % de los puntos del scan cae a más de 20 cm de toda pared del mapa (`localization.lost_fraction` y `localization.unexplained_distance`). La máquina de estados frena y espera un `/initialpose` nuevo. Si la fracción vuelve a bajar, el estado vuelve solo a `LOCALIZED`.

| Señal | Falsas alarmas, bien ubicado | Detecta perdido, por actualización | Con un obstáculo cerca que no está en el mapa |
|---|---|---|---|
| A. Mirar los puntos del lidar en RViz | no aplica | Depende de que alguien mire | no aplica |
| B. Distancia media a las paredes mayor que 10 cm | 0 % | 100 % aula, 94 % café | **Falla**: una caja de 40 cm a 25 cm sube la media a 10 cm. En Gazebo, 3 de 5 corridas con caja terminaron en `LOST` |
| C. Mediana de la distancia mayor que 5 cm | 0 % | 97 % aula, **41 % café** | No la mueve |
| D. Fracción de puntos a más de 20 cm de toda pared, mayor que 25 % | 0 % | 98 % aula, 72 % café | Una caja de 40 cm a 25 cm da 15 a 18 %. Un obstáculo de 90 cm a 25 cm da 23 a 25 % |

Elegí D. Primero había elegido B, que separa mejor los dos casos pero choca con la evasión: un obstáculo tapa pocos rayos y queda muy lejos de las paredes, así que mueve mucho la media. La mediana (C) no ve un falso encaje, porque un falso encaje es justamente una pose que explica más de la mitad del scan.

El 72 % es por actualización. Por episodio alcanza: en las 10 corridas perdidas (6 del aula, 4 del café) D marcó `LOST` en la primera actualización, y el robot frena con la primera.

Datos: aula real, bags del 16/11 con click correcto y con 90° de error, 3822 actualizaciones ubicado y 2140 perdido, contra la corrida bien ubicada. Café simulado con pose real, 2590 ubicado y 547 perdido. Scripts en `tp4_final/review/` del repo personal (`lab_residual_statistic.py`).

### Prueba en vivo en Gazebo (café), con los nodos ROS

| Caso | `unexplained_fraction` | Estado |
|---|---|---|
| Bien ubicado, sin caja (4 corridas) | hasta 0,03 | `LOCALIZED` |
| Bien ubicado, rodeando una caja de 30 cm (3 corridas) | 0,07 a 0,21 | `LOCALIZED`. Con la media, estas tres terminaban en `LOST` |
| Click a 2 m, la ráfaga recupera la pose real (2 de 5) | 0,00 a 0,01 | `LOCALIZED` |
| Click a 2 m, la ráfaga cae 1 a 3 m corrida a lo largo de un pasillo (3 de 5) | 0,22 a 0,24 | `LOCALIZED`, **no lo detecta** |

### Límite conocido y punto abierto

La regla detecta errores gruesos (los 90° del aula, siempre) y no molesta al rodear una caja. No detecta un falso encaje corrido a lo largo de un pasillo, que explica casi el 80 % del scan: en el café quedó en 0,22 a 0,24, pegado a lo que da una caja cerca con la pose correcta (0,21). Con una sola fracción no hay umbral que separe esos dos casos.

Opciones, sin decidir:

| Opción | A favor | En contra |
|---|---|---|
| Dejar 25 % y seguir mirando los puntos del lidar en RViz después de cada click | Sin cambios. En el aula el click falla solo con errores gruesos, que sí detecta | El falso encaje de pasillo queda a ojo |
| Umbral más estricto (15 %) solo en la ráfaga del click, 25 % en marcha | Detecta los 3 casos del café | Un click con una caja a menos de 50 cm daría `LOST` |
| Bajar a 20 % siempre | Detecta los 3 casos del café | Rodear una caja de cerca puede dar `LOST` (0,21 medido) |

Los datos para decidir salen de la visita 1: los clicks equivocados del paso 4 y la caja del paso 6 de la [guía de laboratorio](../robot_real_checklist.md), anotando `unexplained_fraction`. Tampoco está medido con sillas movidas en el aula real.

## Pendiente

- Repetir todas las tablas con un mapa hecho por nuestro SLAM.
- Medir si el puntaje del scan separa un encaje correcto de uno falso, y con eso decidir cómo validar la ráfaga.
- Decidir qué hacer con la inyección: dejarla, sacarla o inyectar alrededor de la estimación en lugar de en todo el mapa.
- Barridos de velocidad y de frecuencia de actualización (preguntas 6 y 9 del informe).

| Medición | Simulación, mapa propio | Robot real |
|---|---|---|
| RMSE de seguimiento | pendiente | no hay pose real |
| Tiempo e iteraciones de la ráfaga | pendiente | pendiente |
| Error al llegar al objetivo | pendiente | pendiente |
| ms por actualización en la computadora del robot | | pendiente |
