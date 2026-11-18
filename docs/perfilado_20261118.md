# Profiling aislado y descartes de scans — 18/11/2026

Trabajo solicitado expresamente a un agente paralelo, con autorización para ROS2.
Worktree `/tmp/tp-final-slam-profile-20261118`, rama `perf/slam-profile-20261118`,
base `b9d82a1`. El contenedor de trabajo es `tp-final-profile-20261118`, sin red,
con fuentes read-only, dominios 121–127 y display 121. No se modificó el árbol
principal ni el contenedor habitual del usuario. **No se integra automáticamente:**
el usuario exige mejora de la estimación del mapa, no solo velocidad de cálculo.

## Qué se puede concluir sobre los 55 descartes originales

La corrida manual del usuario conserva mapa/configuración y un log con throttle,
pero no bag, trayectorias ni historial de estado. El contador 55 agrupa varias
causas: no se pueden asignar retroactivamente las 55 a una única causa. Los
cuatro avisos guardados dicen que falta odometría que encierre el timestamp, al
arranque y hacia la mitad del recorrido. No certifican que todos sean TF ni
overflow, y tampoco que todos correspondan al arranque.

Se identificó y verificó una espera inútil: si el scan es anterior a la muestra
odometría más antigua, nunca será interpolable, porque el buffer solo admite
tiempos crecientes. El baseline espera `tf_wait_timeout` por cada uno y bloquea
sucesores que sí tienen odometría. El candidato descarta ese caso inmediatamente.
Si falta una muestra **posterior**, conserva la espera y nunca extrapola.

También se verificó saturación de cómputo con entradas reproducidas a mayor
ritmo: el executor atiende cálculos y recepción en el mismo hilo. La cola se llena
y pierde scans. El costo del callback incluye reconstruir/publicar trayectorias;
`update_seconds` del núcleo por sí solo no describe la capacidad del adaptador.

El candidato agrega contadores separados de descarte y un resumen de tiempos de
núcleo, publicación y callback completo, sobre las últimas 512 integraciones.
Las exportaciones guardan estos diagnósticos. No se cambiaron defaults de cola,
timeout, cantidad de partículas, pasos del matcher ni parámetros probabilísticos.

## Optimizaciones propias

1. El matcher calcula trigonometría para 13 orientaciones en lugar de repetirla
   para 1053 combinaciones de orientación y traslación. Evalúa exp/log sobre el
   campo de distancias una vez por celda y consulta el resultado por candidata.
2. Un bucle propio opcional Numba consulta ese campo sin matrices de índices y
   máscaras. La reducción final sigue siendo NumPy, conservando su orden.
3. El modelo inverso propio compila el muestreo de rayos y deduplica con marcas.
   Cada celda recibe evidencia una sola vez, impacto prevalece, se conserva la
   máscara observada, saturación, prior y el tratamiento de impactos exteriores.
4. El likelihood compila la búsqueda sobre el mismo vecindario circular y usa
   las mismas distancias a centros de celda. La mezcla y suma final son NumPy.
5. El adaptador reutiliza mensajes de los puntos inmutables que conserva el
   prefijo ancestral seleccionado. No cambia contenido ni frecuencia de los
   paths; la caché conserva solo la trayectoria actual y se vacía con reset.
   Las referencias dejan de copiar la lista completa al agregar cada punto.

Numba es opcional, sin `fastmath`, sin paralelizar partículas y sin bibliotecas
externas de SLAM. La alternativa NumPy sigue disponible, incluyendo
`SLAM_DISABLE_NUMBA=1`. Las firmas se calientan antes de recibir sensores. La
combinación probada es Python3.10, NumPy1.21.5, SciPy1.8.0, Numba0.57.1,
llvmlite0.40.1. Numba0.58.1 rechazó NumPy1.21 y se conservó ese intento fallido;
no se actualizó NumPy para favorecer resultados.

## Entradas y comparación

Se grabó una sesión nueva de Café Fortress **ideal**, sin sesgos ni perfil stress,
con el recorrido guiado de 32 waypoints. Se completaron los 32 y 101,7 m. El bag
incluye scans, odometría, TF, GT y comandos; los comandos nunca se reproducen.
Hay 3123 scans en el bag, 3092 con odometría/GT interpolables, 618 s de datos
alineados. Los 31 anteriores a la primera odometría se identifican por separado.

Cada par baseline/candidato usa la misma grabación, semilla 7, ocho partículas
y `cafe.yaml` sin cambios. El player a 4× genera carga computacional; **no es un
ensayo de velocidad física a 4×**. Se guarda un primer par exploratorio y una
repetición secuencial sin otros ensayos del agente en paralelo. El primer baseline
coincidió durante unos 14 s con un smoke ROS toy del agente, por eso la repetición
es la comparación principal. La carga externa del host no está fijada.

El GT nunca entra al estimador. La calidad de pose también se compara sobre
timestamps comunes contra el GT completo extraído, sin ajuste rígido estimado.
Se usa solo la transformación conocida de la pose GT inicial para expresar la
estimación en el mundo. La referencia de paredes rasteriza cajas SDF a z=0,19 m:
es aproximada, porque el LIDAR GPU observa mallas visuales.

El bag físico `sesiones/20261116/bags/bag_mapa2` se extrae con TF estáticos
reales y odometría interpolada. Quedan 3613 scans y uno sin soporte temporal.
No tiene GT y no se carga el mapa exportado en el SLAM. Su comparación valida
preservación de resultados y costo, no mejora de exactitud física.

## Mediciones ya verificadas

En el bag físico completo, baseline y candidato integran 963 scans y producen
PGM idéntico. Tiempo del núcleo: **107,33→49,66 s**; mediana por actualización
**110,81→51,76 ms**, P95 **123,88→59,15 ms**. El candidato usa más memoria por
Numba/LLVM: pico RSS del proceso **126,7→235,6 MiB**. Calentamiento del constructor
del candidato: 1,45 s en esa ejecución, informado aparte del replay. Son tiempos
del núcleo, no capacidad completa de ROS ni medidas del robot.

El perfil inicial de 200 scans atribuyó aproximadamente 80 % del costo del
núcleo a la propuesta. El ejecutor de profiling materializa los arrays NPZ antes
de medir: acceder repetidamente a `NpzFile[key]` descomprimía todo el array y
ocultaba el costo verdadero. Se conserva también ese primer perfil del ejecutor.

La sesión Café nueva en vivo del baseline dio RMSE 14,15 cm, 1443 integraciones
y un descarte después del reset estacionario; su exportación y path tienen el
mismo timestamp. No es la corrida manual del usuario ni reemplaza sus resultados.

El primer par ROS a 4× dio 548→31 descartes y RMSE muestreado 33,75→15,94 cm.
Los 31 del candidato corresponden al arranque: 30 anteriores al historial y
un overflow previo a disponer de odometría. Es evidencia exploratoria; la
repetición y las métricas geométricas determinan si se cumple la condición de
integración. Más velocidad con mapa idéntico en el núcleo no basta por sí sola.

## Cierre: condición de fusión todavía no demostrada

La repetición limpia y secuencial ya finalizó. **No se recomienda fusionar este
worktree bajo la condición estricta del usuario.** La aceleración y recuperación
de scans están demostradas; la mejora geométrica del mapa no es uniforme.

| Medida | Baseline | Candidato |
| --- | ---: | ---: |
| Café núcleo, tiempo de replay | 124,57 s | 47,89 s |
| Café núcleo, mediana / P95 por integración | 90,53 / 112,81 ms | 33,45 / 38,41 ms |
| Café núcleo, grilla cruda / trayectoria / PGM | Referencia | Idénticos bit a bit |
| Bag físico núcleo, tiempo | 107,33 s | 49,66 s |
| Bag físico, grilla cruda / trayectoria / PGM | Referencia | Idénticos bit a bit |
| ROS Café4×, scans integrados | 1293 | 1458 |
| ROS Café4×, descartes | 512 | 31 de arranque |
| ROS Café4×, RMSE posición sobre GT común | 15,637 cm | 15,310 cm |
| ROS Café4×, RMSE yaw sobre GT común | 1,277° | 0,862° |
| ROS Café4×, precision ocupados SDF a 15 cm | 93,222 % | 93,052 % |
| ROS Café4×, recall superficies SDF a 15 cm | 89,152 % | 89,039 % |
| ROS Café4×, cobertura observada aproximada | 99,861 % | 99,879 % |
| ROS Café4×, cobertura libre aproximada | 99,821 % | 99,842 % |
| ROS Café4×, celdas libres sobre obstáculos SDF | 1930 | 1803 |
| ROS Café4×, CPU del nodo | 184,81 s | 137,52 s |
| ROS Café4×, RSS máximo muestreado | 102,11 MiB | 191,52 MiB |

El GT común usa todos los timestamps de scans dentro de ambas trayectorias,
con la transformación conocida de la pose inicial, sin ajuste a posteriori. En
los replays del núcleo el RMSE común es exactamente igual: 15,310 cm en ambos.
La mejora bajo carga ROS procede de conservar mediciones, no de cambiar el
estimador matemático. El candidato recupera el resultado completo del núcleo.

La conectividad de poses visitadas fue 100 % en ambos mapas, pero se muestrearon
65/73 puntos respectivamente por diferir las integraciones: ese dato no es una
comparación sobre el mismo conjunto de puntos y no se usa como prueba adicional
de mejora. La comparación de SDF es aproximada, y su precision/recall disminuyen
ligeramente. Por tanto, no convertir una mejora de CPU, descartes o RMSE pequeño
en una certificación de mapa superior. El primer par exploratorio fue menos
favorable al baseline y no reproduce la magnitud de mejora en la repetición.

El perfil de publicación antes/después de la caché (últimas 512 integraciones,
ambos candidatos con los mismos datos completos) pasó de mediana/P95
44,31/69,43 ms a 19,31/41,19 ms. El callback completo pasó de 87,92/116,23 ms a
63,23/85,25 ms. Se conserva el contenido/frecuencia de los mensajes, y ambos
candidatos producen el mismo PGM. No es una medición de navegación física.

**No se ejecutó un par ROS1× baseline/candidato**, ni una nueva sesión física.
Los resultados de 4× prueban capacidad frente a carga artificial del transporte;
no certifican velocidad del robot ni una mejora en el caso habitual a 1×.

Evidencia persistida en
[slam/artifacts/profile_20261118](../slam/artifacts/profile_20261118/README.md),
con hashes, configuraciones, grillas, trayectorias, perfiles, logs e inputs NPZ.
El bag crudo Café de 86 MiB se conserva localmente en el worktree y está excluido
del commit por la regla `*.db3` del repositorio. No eliminar el worktree si se
quiere repetir exactamente la carga ROS. El código está en `cb806b6`; el commit
de cierre solo agrega evidencia y documentación. No se hizo merge ni push.

Validaciones: 46 tests de A con Numba, 28 de B con y sin Numba, y 45 tests A del
camino NumPy anterior a añadir el test de caché (dos casos JIT omitidos).
Smoke ROS toy: 118 integraciones, cero descartes, servicios y reset aprobados,
cierre limpio; ese smoke precede la caché, que se verificó con test de cambio de
ancestralidad y los dos replays Café posteriores. Los replays completos baseline
y candidato cierran sus tres procesos con código cero.

## Reproducción

Los scripts nuevos son:

- `slam/scripts/profile_replay.py`: mismo NPZ/config por revisión, tiempos de
  cada etapa, CPU, RSS, grillas crudas, trayectoria y PGM. `--cprofile` conserva
  el perfil; quitarlo para el benchmark de tiempo principal.
- `slam/scripts/profile_ros_replay.py`: bag en dominio aislado, diagnóstico,
  paths, CPU/RSS muestreados, pause/save y cierre. No publica velocidades ni
  lanza Gazebo/RViz. `PYTHONPATH` selecciona baseline o candidato.
- `slam/scripts/compare_profile_runs.py`: analiza evidencia guardada y GT a los
  mismos timestamps; no ejecuta el SLAM. La comparación del mapa usa además
  `analyze_a_experiment.py`.

Ejemplo dentro del contenedor de evaluación, después de cargar ROS/overlay:

```bash
export PYTHONPATH=/ruta/revision/slam:$PYTHONPATH
python3 /worktree/slam/scripts/profile_replay.py \
  --input /tmp/profile_results/cafe_inputs.npz \
  --config /worktree/slam/config/cafe.yaml \
  --output /tmp/profile_results/nueva_corrida
python3 /worktree/slam/scripts/profile_ros_replay.py \
  --bag /tmp/profile_results/cafe_input2 \
  --config /worktree/slam/config/cafe.yaml \
  --output /tmp/profile_results/nuevo_replay_ros --domain 128 --rate 4 --truth
```

Las carpetas de salida deben ser nuevas. El bag completo queda preservado
localmente; los hashes y resúmenes acompañan la evidencia. Antes de cualquier
fusión se revisan la repetición, calidad del mapa, preservación del núcleo y
tests. No se afirma que esta optimización arregle la aproximación `log(p/q)=0`
del matcher ni la ausencia de deskew.
