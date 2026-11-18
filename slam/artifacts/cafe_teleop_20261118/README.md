# Café manual del 18/11/2026

El usuario ejecutó Gazebo Fortress, SLAM y teleop; guardó el mapa, pausó el
estimador y cerró el launch. Esta evaluación posterior solo lee su exportación
y logs. No se volvieron a ejecutar SLAM, simuladores ni pruebas. Código de
referencia del repo: `b9d82a1`. No se modificaron algoritmos ni perfiles.

## Archivos y referencia

Se copiaron PGM/YAML/JSON desde `yahboom_rosmaster`, prefijo
`/home/alumno/rosmaster_ws/maps/cafe_teleop_20261118_131436`, y los logs de esa
sesión. Así el mapa también queda persistido en el host. `metrics.json` es el
análisis estático; `map_comparison.png` superpone el mapa azul con las cajas
SDF naranjas, sin rotación ni traslación ajustadas. Los hashes de los tres SDF
del host coinciden con los instalados en el contenedor.

La referencia es aproximada: usa cajas de colisión a z=0,19 m y el sensor GPU
observa mallas visuales. En particular hay diferencias en la esquina superior
izquierda y elementos estrechos. La cobertura calculada sobre el piso incluye
una franja exterior bajo la puerta inferior; no es una medida exacta de cuánto
del interior se recorrió. No interpretar el recall como paredes reales omitidas
sin inspección adicional. Se presupone el inicio quieto en el spawn original.

## Resultado

- El mapa es visualmente coherente: paredes principales continuas, sectores
  superiores reconocibles, puerta inferior abierta y pocas celdas aisladas.
- 961 scans integrados, 1548 omitidos por movimiento, 169 remuestreos, ESS final
  6,178/8, ningún scan inválido ni rechazo de timestamp del núcleo. Hubo **55
  descartes del adaptador**. El log registra falta de odometría que encierre el
  timestamp al arranque y hacia la mitad de la sesión; no todos se pueden
  atribuir al inicio. El log está limitado por throttling, por lo que no permite
  asignar la causa individual de los 55 descartes.
- 208,16 m² libres y 9,86 m² ocupados. El 94,95 % de las celdas ocupadas está
  dentro de una dilatación circular de dos celdas (10 cm) de la referencia
  rasterizada; a tres celdas (15 cm), 98,20 %. Esta tolerancia suma el efecto de
  discretizar y rasterizar cajas, no es un RMSE de pared ni de pose.
- Abertura inferior: 1,15 m en una banda y∈[-11,08;-10,88], frente a 1,09 m
  nominales de las cajas SDF. Las cajas difieren de las mallas visuales.
- La componente desde el spawn contiene 99,99 % de las celdas candidatas
  transitables con inflación de 27 cm de obstáculos y desconocido. Mide
  conectividad interna del mapa: no exactitud respecto del mundo, recorrido
  efectivamente visitado, huella calibrada ni seguridad de cualquier objetivo.
- Se observa un pequeño desvío/engrosamiento de la pared derecha. Los 154 rayos
  truncados son compatibles con mirar por la abertura exterior; el interior
  principal cabe en la grilla. No se infiere su causa exacta sin los scans.
- El último `update_seconds` es 0,162 s. Es **una sola actualización**, sin
  distribución ni costo completo del adaptador. No prueba rendimiento sostenido.
- El nodo SLAM cerró con código cero; RViz abortó durante el cierre (-6), según
  `launch.log`. Esto se distingue de la calidad del mapa ya exportado.

No quedaron bag ni trayectorias en estos archivos. No se puede calcular RMSE
contra Gazebo, error de retorno, velocidad física o latencia durante el recorrido.
El usuario aportó los límites de teleop: se subieron de 0,15/0,30 a **0,52 m/s y
1,04 rad/s**. Son consignas sin timestamps ni duración, no velocidades medidas
ni una validación sostenida a su máximo.

## Expectativa y ajustes propuestos, sin aplicar

Para Café ideal se propone como objetivo de trabajo: paredes principales a
5–10 cm, puertas conservadas y sin paredes dobles. Con trayectoria guardada,
buscar RMSE de pose ≤5 cm y retorno sin desdoblar la geometría. Son objetivos
de ingeniería del proyecto, no garantías matemáticas ni umbrales del PDF.

La configuración del modelo instala un LIDAR GPU de 5 Hz. A las consignas máximas
reportadas hay 10,4 cm y 11,9° entre scans consecutivos, si se alcanzaran y no se
perdieran mensajes. La ventana del matcher es sobre la **predicción odométrica**;
sus ±10 cm/±0,06 rad no son un límite directo de desplazamiento ni velocidad.
La odometría ideal facilita mucho la tarea. La simulación no valida la distorsión
temporal del barrido físico; el núcleo no hace deskew. ROS distingue el tiempo
entre haces (`time_increment`) del intervalo entre scans (`scan_time`):
[definición de LaserScan de Humble](https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/LaserScan.msg).
El [bridge de Humble](https://github.com/gazebosim/ros_gz/blob/humble/ros_gz_bridge/src/convert/sensor_msgs.cpp)
deja ambos en cero para mensajes de Gazebo; no obtener una duración real de
barrido a partir de esos ceros.

Mantener el perfil actual como referencia y grabar scans, odometría, TF,
`/ground_truth/odom`, `/cmd_vel`, `/path`, `/slam/odom_path`,
`/slam/ground_truth_path` y `/slam/status` en la siguiente sesión manual. La
grabación necesita iniciarse antes de conducir y confirmar que captura TF
estáticos transient-local. El usuario ejecuta las pruebas.

Primero revisar recepción/sincronización, antes de tocar el núcleo. Ensayar una
espera mayor (`tf_wait_timeout`: 2→10 s) y comprobar cola/edad y descartes; una
cola mayor puede absorber ráfagas, pero no corrige pérdida de odometría ni falta
de capacidad sostenida. No declararlo una solución ya verificada.

Si interesa afinar el perfil ideal, comparar una variable a la vez sobre la misma
grabación: `scan_match.prior_xy` 0,05→0,025 m, luego `prior_yaw` 0,03→0,015 rad;
alternativamente, `sample_xy` 0,01→0,005 m o `sample_yaw` 0,005→0,0025 rad. Un
prior más fuerte puede preservar mejor la odometría ideal; menos ruido puede
reducir diversidad. Ambas son hipótesis que requieren evaluación y no se deben
trasladar al robot automáticamente. Mantener por ahora celda de 5 cm y ocho
partículas. Aumentar partículas no está justificado por este mapa y cuesta CPU.

Para el próximo ensayo reproducible en Café: fijar consignas por tramo y separar
avance de giro; 0,30 m/s y 0,50 rad/s son un punto de comparación propuesto, no
un máximo. La corrida actual ya alcanzó consignas mayores. Para el robot real,
empezar con 0,15 m/s y 0,30 rad/s y medir el barrido y las velocidades efectivas;
esas cifras tampoco son límites certificados.

Recalcular solo el análisis estático desde el host:

```bash
python3 slam/artifacts/cafe_teleop_20261118/analyze_saved_map.py
```

Requiere NumPy, PyYAML, Pillow y Matplotlib, y la copia local de `custom_map`
indicada en el script. No importa el núcleo del SLAM ni ROS.
