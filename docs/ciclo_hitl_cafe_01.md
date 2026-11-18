# Ciclo colaborativo Café 01 — 18/11/2026

El usuario aprobó implementar y ejecutar el plan de cinco casos. Esta autorización
incluye únicamente el replay del núcleo, sus chequeos analíticos y análisis de
resultados. No incluye nuevas simulaciones, ROS1×, pruebas físicas ni ciclos
siguientes. El punto de control es la revisión visual humana al finalizar.

## Diseño y datos

| ID | Implementación | Partículas | prior_xy | sample_xy |
| --- | --- | ---: | ---: | ---: |
| A | Original | 8 | 0.05 | 0.01 |
| B | Optimizada | 8 | 0.05 | 0.01 |
| C | Optimizada | 16 | 0.05 | 0.01 |
| D | Optimizada | 16 | 0.025 | 0.01 |
| E | Optimizada | 16 | 0.05 | 0.005 |

Semilla 7, resto de parámetros de `cafe.yaml`, mismas entradas completas y
ejecución secuencial. Los controles A/B se repiten para medirlos en el mismo
entorno y registrar las nuevas series por integración. Los otros tres casos
son experimentos nuevos. No es un barrido de semillas ni se combina prior y ruido.

Se reutiliza el NPZ Café de 3092 scans con odometría y GT de una sesión Fortress
ideal anterior del agente. No es la corrida manual de los 55 descartes. El
baseline usa los fuentes sin optimizar del árbol principal, cuyo núcleo no
cambió respecto de b9d82a1. El candidato usa los kernels del worktree. La
instrumentación no modifica el estimador. Numba0.57.1 y llvmlite0.40.1 se toman
del entorno ya preparado; no se cambian dependencias del host ni del contenedor
habitual. `SLAM_DISABLE_NUMBA=1` no recupera el código original completo.

## Métricas e interfaces

`profile_replay.py` conserva ahora `updates.npz`: columnas `stamp`,
`core_seconds`, `ess_ratio`, `resamples`, una fila por integración. Su JSON de
tiempos incluye proporciones sobre 90 y 137 ms y ubicación/versiones del runtime.
Los tiempos incluyen `FastSLAM.process` e instrumentación por etapa; excluyen
lectura de NPZ, importación, calentamiento del constructor, exportación y ROS.
El máximo RSS del proceso sí incluye importación/JIT y se informa por separado.

`analyze_a_experiment.py` agrega:

- `--input-npz`: extrínseca registrada y checkpoints comunes de GT.
- `--inflation-radius`: radio métrico, por defecto 0,27 para compatibilidad de
  invocaciones; este ciclo siempre pasa **0,22** explícitamente.
- `--compare`: directorios de los otros casos, además del posicional original.
- `--report-dir`: carpeta nueva para JSON, CSV y láminas comparativas.

El análisis usa `experiment_metrics.py`, independiente del SLAM y de ROS.
La evaluación nueva declara versión 2: el radio no se redondea a 25 cm, ocupado,
desconocido y exterior impiden paso de la huella, y los extremos se comparan
contra centros de celdas ocupadas sin truncar distancias. Se seleccionan hasta
30 haces válidos por scan integrado con la misma regla en todos los casos.
El modo comparativo exige timestamps de integración idénticos para esta métrica.
Los resultados antiguos no se sobrescriben ni se consideran idénticos a v2.

El RMSE se calcula sobre timestamps comunes de entrada, con GT y transformación
inicial conocida, sin ajuste rígido posterior. Se interpola la trayectoria
ancestral final y se informa ese alcance; no son estimaciones online de pose
independientes. Los checkpoints de conectividad son GT cada veinte scans de la
misma entrada. SDF de colisiones es referencia aproximada del mapa, porque el
LIDAR GPU observa mallas visuales. Endpoints sobre scans de construcción son
consistencia interna, no validación independiente.

Los cinco casos conservan tiempos, mapas, grillas, paths, configuración, trazas
ESS y logs. El informe incluye vistas con/sin trayectorias, detalle de pared
derecha, sector superior y puerta inferior, y curvas de costo/ESS.

## Reproducción del ciclo aprobado

El contenedor propio `tp-final-profile-20261118` monta este worktree en
`/worktree:ro` y los assets en `/custom_map:ro`; no se lanzan nodos ROS.
El código original se copia a `/tmp/hitl_baseline/slam`. Resultados dentro del
contenedor en `/tmp/hitl_cafe_01`, posteriormente archivados en el worktree.

```bash
docker exec tp-final-profile-20261118 bash /worktree/slam/scripts/run_hitl_cafe_01.sh
docker exec tp-final-profile-20261118 python3 /worktree/slam/tests/test_experiment_metrics.py
docker exec -e MPLCONFIGDIR=/tmp/hitl_mpl_cache tp-final-profile-20261118 \
  python3 /worktree/slam/scripts/analyze_a_experiment.py /tmp/hitl_cafe_01/A \
  --compare /tmp/hitl_cafe_01/B /tmp/hitl_cafe_01/C /tmp/hitl_cafe_01/D /tmp/hitl_cafe_01/E \
  --input-npz /worktree/slam/artifacts/profile_20261118/results/cafe_inputs.npz \
  --inflation-radius 0.22 --cafe-package /custom_map \
  --report-dir /tmp/hitl_cafe_01/report
```

La carpeta de salida debe ser nueva. No ejecutar estos comandos para iniciar
otro ciclo sin acordarlo. Las instrucciones reproducen el ciclo 01 y requieren
el snapshot de control, fuentes, dependencias y montaje documentados arriba.

## Revisión conjunta

Antes de elegir finalistas se cruzan métricas con observaciones humanas sobre
paredes dobles, pasillos, habitaciones, deformación y puertas. Todos los casos
quedan en investigar hasta esa revisión. Se conserva `human_observations` en
el registro JSON para incorporar literalmente los comentarios del usuario.
Ningún umbral geométrico orientativo descarta automáticamente un candidato.
La condición de fusión sigue vigente: no se integra por ahorrar CPU solamente.

Resultados e informe: `slam/artifacts/hitl_cafe_01/report/README.md`.

## Cierre medido

Los cinco casos completaron 1458 integraciones sobre la misma entrada. Los cinco
chequeos analíticos de evaluación pasaron. A/B conservaron grilla, trayectoria
y PGM bit a bit; tiempo 136,50→51,83 s. RMSE común A/B 15,31 cm, C 9,68 cm,
D 6,12 cm y E 9,08 cm. P95 del núcleo A 125,3 ms, B 41,6 ms, C 91,5 ms,
D 79,7 ms y E 79,5 ms. El presupuesto 137 ms corresponde al objetivo del
LIDAR físico de 7,3 Hz; la entrada Café proviene del simulador a 5 Hz.

D tiene las mejores métricas conjuntas en este ciclo, pero la evaluación humana
sigue pendiente. E muestra una posible discontinuidad en la pared junto a la
puerta inferior que las métricas globales no deben ocultar. Conectividad 100 %
sobre los mismos 155 checkpoints no certifica todas las paredes ni rutas.
Todos quedan en investigar. No se fusionó ni se inició otro ciclo.
