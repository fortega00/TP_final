# 002. Propuesta con scan matching

Estado: implementada en la rama `fix/slam-review` (2026-11-11). Activada con 8 partículas en `config/default.yaml`, `config/cafe.yaml` y `config/parcial_bag_scan_match.yaml`. Un YAML sin el grupo `scan_match` usa la propuesta de odometría.

## Problema

El filtro movía cada partícula con el incremento de odometría más ruido (FastSLAM 1.0) y dejaba que los pesos descartaran las malas. La odometría es mucho menos precisa que el lidar, entonces para que alguna partícula caiga cerca de la pose real hace falta mucho ruido y muchas partículas:

- Con la config por defecto (24 partículas, ruido de 1 % del paso) el filtro sigue a la odometría. En la simulación con deriva las partículas se van del mapa y la corrida aborta.
- En el bag real solo funciona con 128 partículas y ruido de 32 % del paso, a 0,44 s por actualización contra 0,19 s disponibles.

## Alternativas

**A. Solo ajustar ruido y cantidad de partículas.** Bag real, semillas 7 / 1 / 2:

| alpha | Partículas | RMSE de posición (cm) | Peor recall a 10 cm | Peor precisión a 10 cm |
|---|---|---|---|---|
| 0,003 | 16 a 128 | 358 a 535 | 0,17 | 0,06 |
| 0,01 | 16 a 128 | 202 a 296 | 0,19 | 0,09 |
| 0,03 | 128 | 23,7 / 21,0 / 24,8 | 0,80 | 0,70 |
| 0,1 | 16 | 19,6 / 18,8 / 14,4 | 0,76 | 0,70 |
| 0,1 | 32 | 13,4 / 18,4 / 12,4 | 0,87 | 0,81 |
| 0,1 | 64 | 12,9 / 13,9 / 14,5 | 0,89 | 0,85 |
| 0,1 | 128 | 9,5 / 12,8 / 7,9 | 0,88 | 0,84 |

No hay un punto barato que funcione: con poco ruido diverge por metros y con ruido suficiente la calidad recién aparece con 64 a 128 partículas.

**B. Propuesta con scan matching** (la idea de GMapping y FastSLAM 2.0). Antes de pesar, cada partícula:

1. toma como estimación su pose anterior compuesta con el incremento de odometría, sin ruido;
2. prueba todas las poses de una ventana chica alrededor (±10 cm en pasos de 2,5 cm, ±0,06 rad en pasos de 0,01) y puntúa cada una según qué tan cerca caen los extremos del scan de las celdas ocupadas de su propio mapa, más un prior gaussiano que la mantiene cerca de la odometría cuando el scan no alcanza para decidir (un pasillo liso);
3. se queda con la mejor y le suma un ruido chico (1 cm, 0,005 rad) para conservar diversidad.

El resto del filtro (pesos, actualización del mapa, remuestreo) no cambia. Mientras el mapa tiene menos de 200 celdas ocupadas no hay contra qué comparar y se usa la propuesta de odometría.

## Decisión

Opción B, en `tp_slam/scan_match.py`, enchufada en la interfaz `Proposal` que ya tenía el filtro. Queda activada en los perfiles de simulación, robot real y bag, con 8 partículas. La propuesta de odometría sigue en el código (`scan_match.enabled: false`) y sirve de comparación; `parcial_bag.yaml` y `parcial_bag_tuned.yaml` la conservan para reproducir los resultados anteriores.

También se acepta `scipy` en el núcleo, solo para la transformada de distancia. No es una librería de SLAM.

## Datos

Las referencias son independientes del SLAM: en el bag real, el mapa del laboratorio de los profesores y mi localización del TP3 sobre ese mapa; en simulación, la pose real de Gazebo. Scripts en `tp4_final/review/` del repo personal.

**Bag real** (857 actualizaciones, semillas 7 / 1 / 2):

| Configuración | Partículas | RMSE de posición (cm) | Error máximo (cm) | Recall a 10 cm | Precisión a 10 cm |
|---|---|---|---|---|---|
| Odometría, ajustada | 128 | 9,5 / 12,8 / 7,9 | 21,0 / 27,3 / 22,2 | 0,96 / 0,88 / 0,98 | 0,91 / 0,84 / 0,95 |
| Scan matching | 4 | 7,4 / 6,4 / 7,6 | 24,3 / 29,4 / 26,8 | 0,98 / 0,98 / 0,98 | 0,97 / 0,95 / 0,97 |
| Scan matching | 8 | 8,0 / 7,2 / 8,0 | 27,2 / 26,3 / 20,7 | 0,98 / 0,97 / 0,97 | 0,96 / 0,96 / 0,95 |
| Scan matching | 16 | 7,4 / 6,7 / 5,9 | 19,1 / 23,0 / 20,1 | 0,97 / 0,98 / 0,97 | 0,95 / 0,97 / 0,96 |
| Scan matching, ventana ±15 cm y ±0,09 rad | 8 | 8,3 / 6,9 / 7,7 | 22,2 / 26,3 / 24,3 | 0,96 / 0,98 / 0,96 | 0,94 / 0,96 / 0,94 |

El resultado no depende de la cantidad de partículas ni del tamaño de la ventana. Solo el 0,01 % de los ajustes tocó el borde de la ventana de ±10 cm.

**Simulación con deriva** (café en Gazebo Classic, recorrido de 54,7 m con vuelta al inicio, 556 actualizaciones). La odometría sola termina con 667 cm de RMSE y hasta 120° de error de yaw:

| Configuración | Partículas | RMSE de posición contra la pose real (cm) | Error máximo (cm) |
|---|---|---|---|
| Odometría, default (`cafe.yaml`) | 24 | aborta: todas las partículas salen del mapa | |
| Odometría, ajustada (alpha 0,1) | 128 | 13,3 / 16,2 / 53,9 | 25,1 / 29,6 / 83,7 |
| Scan matching | 8 | 14,6 / 16,8 / 12,8 | 24,7 / 33,1 / 21,9 |

**Tiempo por actualización** (MacBook Air, una sola corrida a la vez):

| Configuración | Bag real (disponible: 0,19 s) | Simulación (disponible: 0,36 s) |
|---|---|---|
| Odometría ajustada, 128 partículas | 0,44 s (medido con tres corridas en paralelo) | 0,96 s (ídem) |
| Scan matching, 8 partículas | 0,074 s | 0,050 s |

**Nodo ROS en vivo** (Gazebo Classic sin interfaz en macOS, café, mismo recorrido de 54,7 m, odometría con deriva por `/calc_odom`, 8 partículas, una corrida):

| Medida | Valor |
|---|---|
| RMSE de posición de la odometría contra la pose real | 854 cm |
| RMSE de posición de `/slam_pose` contra la pose real | 8,8 cm (máximo 16,8, final 5,6) |
| Actualizaciones procesadas | 566 |
| Tiempo por actualización | mediana 0,073 s, percentil 95 0,093 s, máximo 0,175 s |
| Scans en cola (máximo) | 1 |
| Scans descartados | 1, al arrancar, antes de tener odometría |

El nodo sigue el ritmo del simulador sin acumular cola. Falta medirlo en la computadora del robot.

**Toy** (odometría casi perfecta, 2,0 cm de RMSE): acá scan matching no ayuda. RMSE de posición 3,8 cm contra 2,6 cm de la propuesta de odometría con 24 partículas, y precisión de celdas ocupadas 0,76 contra 0,88. La búsqueda trabaja en pasos de 2,5 cm sobre celdas de 5 cm, entonces cuando la odometría es mejor que eso agrega un poco de ruido. Es el costo conocido de usarla por defecto.

## Parámetros sensibles

- `scan_match.sigma` (qué distancia extremo-pared cuenta como ajuste). Con el prototipo, 8 partículas y tres semillas: 0,03 m da 6,6 a 11,0 cm de RMSE; 0,05 m da 7,2 a 8,0; 0,07 m da 9,3 a 11,8; 0,10 m falla en dos semillas (45 y 47 cm). Queda en 0,05 m, una celda.
- El puntaje tiene que saturar para los extremos lejos de toda pared conocida (zonas recién vistas). La primera versión usaba la distancia al cuadrado sin saturar y esos extremos arrastraban la pose: 22 cm de RMSE. Por eso se usa la misma mezcla que el campo de verosimilitud (`z_hit` gaussiana más `z_rand` uniforme).
- `sensor.hit_thickness`: con 0,03 m en lugar de 0,08 m el resultado empeora (RMSE 11,5 / 13,3 / 7,4 cm, recall 0,90 / 0,83 / 0,93). Queda en 0,08 m. El grosor de las paredes se trata aparte.

## Limitaciones

- El peso exacto de esta propuesta integra la verosimilitud sobre la ventana. Acá la corrección es 0 y cada partícula se pesa con la verosimilitud en su pose ajustada. Es una simplificación habitual, hay que decirla en el informe.
- La ventana supone que la odometría se equivoca menos de 10 cm y 0,06 rad por paso. Vale para el bag y para la simulación, no está probado con movimientos más bruscos.
- Con odometría casi perfecta (toy) empeora un poco el resultado, ver arriba.
- No está probado en el robot real ni en su computadora.
