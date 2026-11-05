# Evidencia sintética de entrega A

`toy/` contiene la corrida por defecto, `tuning/` cuatro ablaciones con la misma
semilla y `seeds/` dos semillas adicionales sin cambiar hiperparámetros.
Cada directorio de corrida incluye el mapa PGM/YAML, ground truth independiente,
trayectoria CSV, vista SVG y métricas JSON con todos los parámetros usados.

```bash
python3 -m tp_slam.cli toy --output artifacts/toy --seed 7 --check
python3 scripts/tune_toy.py --output artifacts/tuning --seed 7
python3 -m tp_slam.cli toy --output artifacts/seeds/seed11 --seed 11 --check
python3 -m tp_slam.cli toy --output artifacts/seeds/seed19 --seed 19 --check
```

Los tiempos son mediciones de la ejecución del filtro, varían entre máquinas y no
incluyen exportación ni evaluación. Las métricas del mapa se calculan leyendo el
PGM/YAML guardado, con la clasificación trinary de `map_server`: desconocido y
probabilidad intermedia no se consideran espacio navegable. La conectividad usa
un radio de robot de 22 cm más 5 cm de margen, inflación circular conservadora y
búsqueda de cuatro vecinos. Este chequeo evalúa el mapa, no implementa navegación.

La precisión y el recall toleran un desplazamiento de una celda, cuya distancia
física depende de la resolución; la IoU es exacta y penaliza tanto obstáculos
faltantes como celdas ocupadas adicionales. La cobertura se mide sobre todo el
espacio libre real de la oficina; no sólo sobre lo observado por el robot.

Las semillas 7, 11 y 19 pasan los umbrales del escenario sencillo. Las ablaciones
con LiDAR de mayor ruido y odometría más ruidosa fallan el umbral de precisión de
paredes, aun cuando el recorrido sigue conectado. Estos resultados hacen visible
la sensibilidad del enfoque sin scan matching. No prueban desempeño en Gazebo,
en rosbag real ni en el RosMaster físico; esas tres validaciones deben realizarse
con los procedimientos de `A.md`.

`exploratory_search.json` conserva pruebas preliminares anteriores a corregir la
discretización de la banda ocupada. Tiene valor histórico y no debe usarse como
resultado del código final; las comparaciones reproducibles están en `tuning/`.
