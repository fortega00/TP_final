# Evidencia de la revisión del 16/11/2026

Fuentes revisados: `1c14cf2063ecea9e170bc1e191f7022f0e28102c`, comparados con `18a7784`. Ver [el informe de revisión](../../../docs/revision_20261116.md) para resultados, límites y comandos interactivos. No se cambiaron algoritmos.

| Ruta | Alcance |
| --- | --- |
| `manifest.json` | Versión, entorno y alcance de validación |
| `toy_core/` | Toy sin ROS, semilla 7, perfil default completo |
| `ros/toy/`, `ros/bag/` | Smoke tests aprobados con servicios/reset |
| `ros/bag_tf/` | Fallo original del validador a 2×: 114 integraciones |
| `ros_bag_tf_repeat/` | Repetición a 2× bajo carga: timeout con 102 integraciones |
| `ros_bag_tf_slow/` | Variante a 0,5× aprobada, 118 integraciones |
| `ros_smoke_tf_slow.py` | Copia del smoke test, único cambio: velocidad de bag 2 → 0,5 |
| `ros_map_comparison.json` | Hashes iguales del toy, bag tópico y bag TF lento |
| `bag_scan_match/` | Intento sin Xvfb instalado: falló antes de lanzar ROS, no es un resultado de SLAM |
| `bag_scan_match_ros/` | Bag físico completo, scan matching, 8 partículas, ROS Humble |
| `parcial_inputs.npz` | Extracción del bag original con odometría interpolada y extrínseca de TF |
| `bag_core_scan_match/` | Mismas entradas/configuración sin ROS; el PGM coincide con ROS |
| `core_ros_comparison.json` | Comparación exacta de exportaciones y poses finales |
| `localization_own_bag_map.json` | MCL de B sobre nuestro mapa generado con el mismo bag |
| `localization_vs_slam.json` | Acuerdo entre MCL y SLAM; no es exactitud contra GT |
| `nav_tests_humble.log` | 28 pruebas de B en Humble |
| `nav_launch_smoke.json`, `.log` | Arranque aprobado; cierre por SIGINT al grupo produjo KeyboardInterrupt en ambos nodos; no prueba navegación |
| `nav_launch_parent_signal.json`, `.log` | Arranque y cierre limpios enviando SIGINT solo al launcher |
| `reproduce_stale_lidar.py`, `stale_lidar.json` | Hallazgo de frescura de LIDAR, callbacks reales con publicadores simulados |
| `cafe_scan_match_ros/` | Recorrido guiado de Café Fortress, perfil ideal, sin sesgos |

El contenedor habitual estaba detenido al iniciar. Esta revisión no lo arrancó ni modificó; al finalizar se observó activo y se dejó intacto. Se copiaron el bag y los recursos necesarios para un contenedor temporal distinto. La imagen original no se modificó. En el contenedor temporal se instalaron `xvfb`, `python3-pyqt5`, `python3-pil` y `python3-matplotlib`, necesarios para el ejecutor y el análisis. Los fuentes y el bag se montaron de solo lectura.

Para reproducir los núcleos desde la raíz del repo, con el entorno Python preparado:

```bash
(cd slam && python3 -m unittest discover -s tests -v)
(cd nav && PYTHONPATH=../slam:. python3 -m unittest discover -s tests -v)
(cd slam && python3 -m slam.toy.cli toy --config config/default.yaml \
  --output /tmp/review_toy_new --seed 7 --check)
(cd slam && PYTHONPATH=. python3 scripts/replay_bag_core.py \
  --input artifacts/review_20261116/parcial_inputs.npz \
  --config config/parcial_bag_scan_match.yaml --output /tmp/review_bag_core_new)
(cd nav && PYTHONPATH=../slam:. python3 scripts/replay_localization.py \
  --input ../slam/artifacts/review_20261116/parcial_inputs.npz \
  --map ../slam/artifacts/review_20261116/bag_scan_match_ros/map.yaml \
  --config config/real.yaml --initial-pose 0 0 0 --output /tmp/review_mcl_new.json)
```

Las carpetas de salida del toy/replay deben ser nuevas. En ROS, `executor.py` y `processes.json` de cada experimento conservan el ejecutor y argumentos exactos. Sus rutas `/repo` y `/results` son montajes del contenedor temporal: reemplazarlas por las del entorno propio. Las recetas del informe resuelven esa diferencia.

No comparar los tiempos con los de la Mac de Iván: hubo varias pruebas concurrentes y renderizado por software. El bag no tiene ground truth; sus residuos usan los mismos datos del mapeo. La referencia geométrica SDF del café es aproximada. Se preservan fallos y resultados originales del 10/11 fuera de esta carpeta.
