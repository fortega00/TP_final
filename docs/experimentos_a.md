# Evaluación de A: café y bag real del Parcial

Fecha: 10 de noviembre de 2026. Esta evaluación distingue completar una ejecución de aprobar la calidad del mapa exigida por la consigna. Los resultados de los ajustes sin ROS son screening; sólo las corridas con el launcher validan transporte, TF, exportación y RViz.

## Procedimiento y referencias

Cada sesión inicia un filtro nuevo, con mapas desconocidos y pose `(0,0,0)` sin dispersión inicial. Se separan dominios ROS, particiones de Gazebo, pantallas Xvfb y directorios de salida. El ground truth sólo alimenta la evaluación y su trayectoria de referencia; no interviene en el filtro ni en el control. Las capturas `rviz.png` son capturas reales de la pantalla virtual de RViz, sin reconstruir la interfaz a partir de una imagen del mapa.

Los archivos se conservan en [artifacts/entrega_a_20261110](../artifacts/entrega_a_20261110). Cada corrida incluye mapa PGM/YAML, configuración y diagnósticos, muestras, scans comprimidos, logs y vista de RViz. `metrics.json` y `map_and_paths.png` se generan con `scripts/analyze_a_experiment.py`.

### Café

Se utiliza el `cafe.world` instalado en `custom_map`, Gazebo Fortress, odometría de ruedas `/odom`, perfil de contacto `ideal` y `motion_bias=false`. El núcleo conserva 24 partículas y los coeficientes de movimiento del toy. `config/cafe.yaml` amplía la grilla a 16 × 32 m, con celda de 5 cm. Durante el diagnóstico se midieron retornos del chasis a 0,14–0,20 m: este perfil descarta rangos inferiores a 0,25 m, tanto en el filtro como en el controlador experimental.

El controlador usa `/scan` con su extrínseca real y odometría para completar los giros. Sus estados son aproximación, giro a la derecha, seguimiento izquierdo y giro de esquina izquierda. La velocidad máxima es 0,25 m/s, el giro máximo 0,40 rad/s y la separación deseada de la pared 0,70 m. Cuando no encuentra pared después de aproximadamente un cuarto de giro, vuelve a buscar una pared avanzando. Esto evita quedar orbitando en una zona abierta. No es un explorador de fronteras ni garantiza cobertura completa. Se detiene el avance si faltan datos de sensor.

Los intentos previos de diagnóstico se excluyen de la evidencia final: permitieron detectar retornos del chasis, recuperación insuficiente al perder una pared y necesidad de no reutilizar pantallas/particiones antes de cerrar los procesos anteriores. La sesión final utiliza un dominio y una pantalla nuevos y realiza un reset estando quieta antes de comenzar a conducir.

La pose se compara con el GT mediante una transformación fija al comienzo de la sesión, sin ajustar trayectorias retrospectivamente. Se informa el RMSE de la **trayectoria ancestral de la hipótesis final**, que no equivale a una secuencia de estimaciones online independientes.

La cobertura estructural se estima rasterizando cajas de colisión del SDF a la altura del LIDAR y calculando espacio alcanzable desde el origen, con radio de 22 cm más margen de 5 cm. Se informa cobertura conocida/libre y concordancia de obstáculos con tolerancia de 15 cm. Esta referencia es aproximada: el LIDAR GPU usa mallas visuales, que pueden diferir de las cajas de colisión. No se la presenta como una grilla exacta de ground truth. La extensión del piso limita el dominio de cobertura.

### Bag real del Parcial

Entrada original: `/tmp/real_rosbag_input` dentro de `yahboom_rosmaster`, correspondiente a `rosbag2_2026_09_07-08_51_02`. Duración de almacenamiento 163,858 s; 1188 scans y 1633 mensajes de odometría. Los hashes del DB3 y de sus metadatos están en [provenance.json](../artifacts/entrega_a_20261110/provenance.json).

Se utilizan `/scan`, `/odom` y TF grabados. El bag aporta `odom → base_footprint`, `base_footprint → base_link` y `base_link → laser_link`, con traslación planar del láser `(0,0435; 0,0000526)` m y yaw π. No se reproduce `/cmd_vel`, no se carga el mapa conocido del Parcial ni su filtro de localización. La grilla de `config/parcial_bag.yaml` incluye la extensión completa de la odometría cruda y el alcance del sensor; no pretende que el ambiente real tenga ese tamaño.

El bag no tiene GT. La distancia inicio–fin no es error absoluto de pose: el robot no necesariamente volvió exactamente a la pose inicial. La consistencia de extremos contra el mapa final utiliza las mismas mediciones que construyeron ese mapa y puede favorecer sobreajuste; se etiqueta explícitamente como métrica interna. No reemplaza la inspección de paredes y puertas ni una referencia física independiente. Reproducir más lento reduce carga de cómputo, pero conserva el movimiento y la distorsión originales del barrido.

## Resultados iniciales del bag

La corrida ROS con los parámetros del toy y sólo la grilla ampliada completó los 1188 scans: 857 integrados, 331 omitidos por el umbral de movimiento y cero descartes de transporte/sincronización. Duró 341,54 s de pared a velocidad de reproducción 0,5×. La distancia inicio–fin fue 17,738 m para SLAM y 17,855 m para odometría. Sólo 6,62 % de los extremos evaluados quedó a menos de 10 cm de un obstáculo del PGM exportado. El mapa perdió paredes y **no aprueba la calidad de A**, aunque la integración ROS funcione.

Evidencia: [métricas](../artifacts/entrega_a_20261110/rosbag/metrics.json), [RViz](../artifacts/entrega_a_20261110/rosbag/rviz.png), [mapa exportado](../artifacts/entrega_a_20261110/rosbag/map.yaml).

El screening de 64 partículas con mayor incertidumbre de movimiento redujo la distancia inicio–fin a 2,365 m, pero la concordancia interna de extremos a 10 cm fue sólo 16,19 % y siguieron faltando paredes. Esto muestra por qué no se selecciona una configuración sólo por su cierre.

## Resultados finales de las dos sesiones ROS

| Medida | Café guiado, perfil ideal | Bag real ajustado |
| --- | ---: | ---: |
| Partículas | 24 | 128 |
| Tiempo hasta captura/exportación | 998,73 s | 927,55 s |
| Poses de la trayectoria capturada | 1466 | 856 |
| RMSE de posición contra GT | 2,063 cm | No disponible: sin GT |
| RMSE angular contra GT | 0,1944° | No disponible: sin GT |
| Error final de posición contra GT | 1,300 cm | No disponible: sin GT |
| Separación inicio–fin SLAM | 6,234 cm | 0,842 m |
| Separación inicio–fin de referencia | GT: 6,677 cm | Odometría: 17,855 m |
| Extremos a menos de 10 cm de obstáculo exportado, métrica interna | 99,991 % | 82,033 % |
| Conectividad de poses visitadas muestreadas, unknown bloqueado | 74/74 | 42/43 |

### Café

El recorrido guiado se completó y volvió cerca del punto de partida; no fuerza la orientación inicial al terminar. La trayectoria GT midió 101,71 m y 612,4 s entre sus poses integradas. La cobertura **aproximada**, respecto del espacio alcanzable de las cajas SDF, fue 99,863 % observada y 99,819 % libre. La precision de ocupados fue 98,226 % y el recall de superficies alcanzables 96,534 %, con tolerancia de 15 cm. El mapa muestra paredes y sectores superiores definidos. Esto valida esta prueba de simulación en condiciones ideales, sin certificar cualquier ruta posible.

La odometría de ruedas tuvo RMSE de 0,040 mm, menor que el del filtro: el perfil ideal es extremadamente favorable y no demuestra corrección de deriva real. Se respetó la decisión de conservar ese perfil. No se registraron descartes después del reset inicial. Se diagnosticaron 21.965 rayos truncados al mirar al exterior desde la puerta; la grilla contiene el piso evaluado y no se inventan paredes en su frontera.

El estado capturado tenía 1466 integraciones y 1598 omisiones por movimiento. El exportado al cerrar agregó una lectura estacionaria: 1467 integraciones, con timestamp 1 s posterior al último path capturado. `metrics.json` conserva ambos snapshots. El ejecutor actual pausa el filtro antes de guardar para evitar ese desfase en repeticiones.

- [Captura real de RViz](../artifacts/entrega_a_20261110/cafe/rviz.png)
- [Mapa con trayectorias](../artifacts/entrega_a_20261110/cafe/map_and_paths.png)
- [Errores contra GT](../artifacts/entrega_a_20261110/cafe/pose_errors.png)
- [Cobertura y referencia aproximada](../artifacts/entrega_a_20261110/cafe/coverage.png)
- [Mapa PGM/YAML](../artifacts/entrega_a_20261110/cafe/map.yaml) y [métricas](../artifacts/entrega_a_20261110/cafe/metrics.json)

### Bag real

La validación ROS final utilizó `config/parcial_bag_tuned.yaml`: 128 partículas, alpha `(0.10,0.01,0.10,0.10)`, sigma 0,15 m, `p_hit=0.75`, `p_free=0.40` y espesor 0,08 m. Se integraron 856 scans, se omitieron 325 por movimiento y se descartaron siete al inicio mientras aún faltaba la extrínseca grabada del láser. Esa fase era estacionaria. No quedaron scans en cola. Se reprodujo primero a 0,2× y se redujo a 0,15× mediante el servicio del player para vaciar la cola; el cambio está en `rate_change.json`. Las repeticiones documentadas usan 0,15× desde el inicio y mayor espera de TF.

El mapa recupera la estructura principal y varios obstáculos interiores. La distancia inicio–fin se redujo de 17,855 m en odometría a 0,842 m en SLAM. La mediana de distancia de extremos al mapa exportado fue 2,66 cm, la media 8,38 cm y el P95 55,48 cm. Es ajuste interno, no exactitud absoluta. La comparación posterior con `my_map3` muestra diferencias en el sector inferior izquierdo; ese mapa anterior **no se cargó en el filtro y no es GT**.

**La ejecución del paso rosbag está completada y es reproducible, pero no se certifica todavía el mapa como definitivo para navegación:** persisten discontinuidades y ruido exterior, y una pose visitada de las 43 muestreadas no quedó conectada bajo inflación conservadora. No se da por cumplida toda la calidad geométrica de A únicamente por la mejora del cierre. El mapeo vivo y la verificación física del aula siguen pendientes.

- [Captura real de RViz](../artifacts/entrega_a_20261110/rosbag_tuned/rviz.png)
- [Mapa con trayectorias](../artifacts/entrega_a_20261110/rosbag_tuned/map_and_paths.png)
- [Comparación con el mapa del Parcial, sin alineación ajustada](../artifacts/entrega_a_20261110/rosbag_tuned/reference_comparison.png)
- [Mapa PGM/YAML](../artifacts/entrega_a_20261110/rosbag_tuned/map.yaml) y [métricas](../artifacts/entrega_a_20261110/rosbag_tuned/metrics.json)

Ambas sesiones terminaron con códigos de salida cero para Xvfb, launcher y RViz. Se ejecutaron en dominios independientes sobre el mismo host, por lo que sus tiempos incluyen carga concurrente. Una medición durante la ejecución registró máximos residentes de aproximadamente 121 MiB para el nodo de café y 1021 MiB para el del bag. Las 31 pruebas unitarias del núcleo siguieron aprobando. Los scripts de evaluación usan sólo bibliotecas generales y ROS, sin bibliotecas de SLAM.
