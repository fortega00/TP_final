# Evidencia del worktree de profiling — 18/11/2026

**Conclusión: condición estricta de fusión todavía no demostrada. Mantener la rama
aislada.** Mejora de CPU y transporte demostrada; estimación del núcleo idéntica
y métricas geométricas mixtas bajo carga ROS. No hubo merge, push ni modificación
del árbol principal/contenedor habitual del usuario.

El código candidato está en `cb806b6`, base `b9d82a1`, rama
`perf/slam-profile-20261118`. El [informe](../../../docs/perfilado_20261118.md)
explica método, límites, dependencias y reproducción. El manifiesto con hashes
está en [manifest.json](manifest.json).

| Evidencia | Archivos |
| --- | --- |
| Comparación principal ROS Café4×, repetición limpia | [Comparación](results/cafe_ros_comparison.json), [baseline](results/cafe_ros_baseline4x_repeat/metrics.json), [candidato](results/cafe_ros_candidate4x_cached/metrics.json) |
| Café núcleo, igualdad bit a bit y tiempos | [Comparación](results/cafe_core_comparison.json), [baseline](results/cafe_core_baseline/timing.json), [candidato](results/cafe_core_candidate/timing.json) |
| Bag físico16/11 núcleo, igualdad bit a bit y tiempos | [Comparación](results/physical_core_comparison.json), [baseline](results/physical_core_baseline/timing.json), [candidato](results/physical_core_candidate/timing.json) |
| Grabación Café completa de 32 waypoints, baseline vivo | [Métricas](results/cafe_baseline_live2/metrics.json), [RViz](results/cafe_baseline_live2/rviz.png) |
| Perfil inicial y candidatos intermedios | `results/core_*_profile*/profile.pstats`, `timing.json` |
| Validaciones | [Tests finales](results/tests_final2.log), [smoke ROS toy](results/ros_toy_candidate/summary.json) |
| Exportación original de los 55 descartes | [Archivo de entrada manual](manual_input/README.md) |

Resultados principales:

- Café núcleo: 124,57→47,89 s; mapas, log-odds, máscara y trayectoria idénticos.
- Bag físico núcleo: 107,33→49,66 s, también idénticos; sin GT físico.
- ROS4×: 512→31 descartes y RMSE sobre GT común15,637→15,310 cm. Precision SDF
  93,222→93,052 % y recall89,152→89,039 %: no mejora uniforme del mapa.
- Cobertura observada99,861→99,879 %, libre99,821→99,842 %. La referencia SDF es
  aproximada. Conectividad100 % en cada muestreo propio; los conjuntos de puntos
  difieren, por lo que no se usa esa cifra como comparación común.
- RSS muestreado del nodo ROS102,11→191,52 MiB: la aceleración paga memoria de
  Numba/LLVM. CPU184,81→137,52 s para un bag cuyo player limita tiempo de pared.

El primer par ROS4× es exploratorio: el baseline coincidió brevemente con un
smoke toy del agente. Se preserva y se usa la repetición secuencial como evidencia
principal. No presentar el salto inicial33,75→15,94 cm como resultado repetible.

El ensayo4× acelera la entrega de mensajes; no cambia la velocidad física de las
lecturas ni es una prueba del robot real. No hay un par comparativo ROS1×. La
mejora operativa consiste en conservar los scans que el baseline pierde bajo
carga. Los kernels conservan el estimador matemático sobre datos completos.

## Entradas reproducibles

[results/cafe_inputs.npz](results/cafe_inputs.npz) contiene 3092 scans con odometría
y GT interpolados, más extrínseca real de simulación. El bag crudo conserva3123
scans, de los cuales31 preceden el soporte odométrico. Se conserva en
`results/cafe_input2/cafe_input2_0.db3` **solo localmente**, excluido del commit por
`*.db3`; su metadata y SHA256 sí se versionan. No borrar este worktree si se desea
repetir exactamente las pruebas ROS. El input del bag físico es
[results/physical_mapa2.npz](results/physical_mapa2.npz), extraído de los datos ya
versionados de la sesión del16/11, sin cargar mapas previos en SLAM.

Las copias NPZ idénticas se reemplazaron por enlaces relativos dentro de esta
carpeta, después de verificar SHA256 byte a byte. Se registran en
[deduplicated_inputs.json](deduplicated_inputs.json). Los paths absolutos que
aparecen en JSON/logs son rutas históricas del contenedor propio; para recalcular
comparaciones desde el host usar las rutas actuales, por ejemplo:

```bash
python3 slam/scripts/compare_profile_runs.py \
  --baseline slam/artifacts/profile_20261118/results/cafe_ros_baseline4x_repeat \
  --candidate slam/artifacts/profile_20261118/results/cafe_ros_candidate4x_cached \
  --input slam/artifacts/profile_20261118/results/cafe_inputs.npz \
  --output /tmp/cafe_ros_comparison.json
```

El análisis estático no ejecuta ROS ni SLAM. `profile_ros_replay.py` requiere ROS
Humble y el bag crudo; `profile_replay.py` requiere los paquetes numéricos y la
revisión seleccionada por `PYTHONPATH`. Toda futura ejecución interactiva en el
contenedor habitual queda a cargo del usuario.
