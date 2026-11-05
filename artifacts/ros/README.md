# Integración ROS 2 Humble

Ejecutada el 5 de noviembre de 2026 en `yahboom_rosmaster:latest`, sin red y con los fuentes montados en sólo lectura. Reproducir desde la raíz con `./scripts/validate_ros.sh`. Se compila el paquete, se ejecutan 31 pruebas unitarias, se valida el SDF y se lanzan tres pruebas de integración:

| Directorio | Fuente | Odometría | Resultado |
| --- | --- | --- | --- |
| `toy` | Publicador sintético ROS, 10 scans/s de pared | Tópico interpolado | Aprueba |
| `bag` | rosbag2 generado con el mismo toy, reproducción a ×2 | Tópico interpolado | Aprueba |
| `bag_tf` | El mismo rosbag2 sintético | TF al tiempo del scan | Aprueba |

Cada prueba integró 118 de 119 scans; uno se omitió por movimiento. No hubo descartes ni cola pendiente. El PGM guardado fue idéntico en las tres corridas: [comparación y hashes](comparison.json).

Los scripts comprueban descubrimiento de servicios, pausa/reanudación y rechazo de transiciones inválidas, pose/covarianza finitas, mapa publicado, exportación PGM/YAML, reset a mapa desconocido y preservación del archivo guardado después del reset y apagado. El bag contiene deliberadamente un TF de localización obsoleto y un tópico de velocidad; las pruebas verifican que no llegan a los tópicos de salida. El cierre no necesitó SIGKILL ni produjo errores de procesos hijos.

Los `summary.json` conservan los argumentos exactos, parámetros y conteos. Los `launch.log` registran los procesos reales; los prefijos `/results` pertenecen al montaje temporal del contenedor. Los mapas/diagnósticos se copiaron después de las pruebas. El bag se genera en el contenedor y no se versiona.

Esta evidencia valida transporte e integración con datos **sintéticos**. No corresponde a un rosbag del robot físico ni a un recorrido de Gazebo. Las métricas geométricas del CLI están documentadas por separado en `artifacts/toy` y `artifacts/tuning`; el transporte ROS serializa rangos como float32 y usa otra versión de NumPy, por lo que no se exige identidad de partículas con la corrida del host.
