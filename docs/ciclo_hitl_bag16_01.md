# Ciclo colaborativo 01 sobre el bag del 16/11

**Registro histórico anterior a la compuerta solo por movimiento.** Las
configuraciones y fuentes archivados conservan `update.max_interval=1`.
Los YAML instalables `bag16_compare_*` se migraron al esquema actual y las
métricas de este documento no validan esa combinación nueva. Para reproducir
exactamente este ciclo, preparar los fuentes/configuración del commit
`68ebc13` y el núcleo original `b9d82a1`, como en la receta histórica de abajo.
El runner rechaza el núcleo posterior para evitar mezclar implementaciones.

Autorizado el 18/11/2026 mediante el pedido de probar el bag mientras el usuario
evalúa D en Café, siguiendo los criterios anteriores y en un worktree aparte.
Rama `exp/rosbag-20261116-cycle01`, worktree
`/tmp/tp-final-bag-20261116-cycle01`, base `dd2b302`. No fusionar ni cambiar
perfiles físicos de uso antes de revisar los mapas con el usuario.

Se reutiliza el contenedor aislado `tp-final-profile-20261118`, sin red. Se
copian los fuentes del nuevo worktree a `/tmp/bag16_source` dentro del contenedor.
No se modifica `yahboom_rosmaster` ni se publican tópicos ROS. Los cinco
estimadores se ejecutan secuencialmente con `nice -n 15`, un hilo de BLAS/OMP.
El usuario puede estar usando Gazebo en el mismo host: los tiempos son
observaciones de ese entorno compartido, sin garantía de benchmark aislado.

## Hipótesis y matriz

| Caso | Código | Partículas | prior_xy m | sample_xy m | Hipótesis |
|---|---|---:|---:|---:|---|
| H | Mapa guardado 16/11 | 8 | 0,050 | 0,010 | Referencia histórica cualitativa |
| A | Original `b9d82a1` | 8 | 0,050 | 0,010 | Reproducir configuración histórica sobre entrada fija |
| B | Optimizado `dd2b302` | 8 | 0,050 | 0,010 | Acelerar conservando estimación exactamente |
| C | Optimizado | 16 | 0,050 | 0,010 | Aprovechar cómputo para mantener más hipótesis |
| D | Optimizado | 16 | 0,025 | 0,010 | Probar si la mejora de prior en Café se transfiere al bag |
| E | Optimizado | 16 | 0,050 | 0,005 | Reducir ruido de propuesta sin endurecer el prior |

Semilla 7 en todas. Los cuatro YAML completos `bag16_compare_*` parten de
`parcial_bag_scan_match.yaml`, idéntico al `config` de `intento3/aula.json`.
Conservan grilla física 48 × 40 m, sensor real y `no_return_is_free=false`;
no se copia el YAML Café completo al robot. Estimación inicial de cómputo:
unos 10 minutos para las cinco corridas y el análisis. No se ejecutan nuevos
ciclos, otras semillas ni validación ROS1× automáticamente.

## Entrada y comparación histórica

Bag: `sesiones/20261116/bags/bag_mapa2`, 3614 scans, 4750 odometrías.
Mapa histórico elegido para B: `sesiones/20261116/mapa/intento3/aula.*`.
Se extrae nuevamente con `nav/scripts/bag_to_npz.py`, usando `base_footprint`
y TF estático real. No se leen comandos para mover el robot.

Resultado: 3613 scans interpolables de 1080 haces; uno excluido por no tener
odometría antes y después. NPZ SHA256
`50946170bcc0bbb8874979f04299213e8551865513369cf30cc72a48c6229ebc`,
idéntico al profiling previo. Extrínseca plana `(0.0435, 0.0000526, pi)`.

El histórico integró 962 scans; el replay A integra 963. Ambos terminan en el
mismo scan `1794855669.285052`; los últimos tres scans crudos posteriores
quedan fuera de la integración por la compuerta de movimiento/tiempo. Su
trayectoria ancestral y selección exacta de scans no se guardaron. No se
presentan endpoints históricos usando poses de otra corrida, ni se exige
identidad con el replay A. El log de estado tiene muestras del último tiempo
de actualización, no una serie de cada integración.

## Métricas y límites

- Endpoints v2: 30 retornos finitos por scan, mismos timestamps integrados,
  extrínseca del NPZ, distancia sin truncar a centros de celdas ocupadas.
  Mediana, P95 y fracción ≤10 cm. Consistencia interna, no exactitud absoluta.
- Comprobación adicional en scans omitidos por la compuerta odométrica:
  no se usaron en matcher/likelihood/mapa. Se interpolan las poses ancestrales
  y se aplica la misma métrica. Datos correlacionados de la misma sesión;
  no reemplaza un bag de validación independiente ni ground truth.
- Navegabilidad: inflación métrica 22 cm de ocupado, desconocido y exterior.
  Componentes conexas con vecindad de cuatro. Mismos timestamps cada 20 scans
  de entrada, puntos comunes de la estimación A y también puntos propios.
  Ninguno representa posiciones reales verificadas. Para H se asume el mismo
  arranque estacionario y orientación; no se ajusta el mapa a posteriori.
- Trayectoria: longitud y separación/orientación inicio-fin, sin atribuir
  error absoluto ni error de cierre conocido. No hay GT, ATE/RMSE ni recall
  de paredes/cobertura física verificables en este bag.
- Tiempos de todas las integraciones: mediana, P95, máximo, >90/>137 ms;
  calentamiento separado, RSS, ESS/N completo y remuestreos. El núcleo no
  mide colas, TF, descartes ni capacidad del adaptador ROS en tiempo real.
- Mapas con mismos ejes, escala, representación y detalles regionales.
  Comparación visual de H, B, C, D, E; A/B se comprueban bit a bit por separado.

Scripts: `slam/scripts/run_hitl_bag16_01.sh`, `profile_replay.py` y
`analyze_bag_cycle.py`. Evidencia en `slam/artifacts/hitl_bag16_01/` del
worktree. Al terminar se detiene el contenedor aislado y se pide revisión
humana antes de elegir un perfil o comenzar otra iteración.

## Reproducción del ciclo aprobado

Con los fuentes del worktree copiados a `/tmp/bag16_source`, el núcleo original
de `git archive b9d82a1 slam/slam` en `/tmp/hitl_baseline/slam`, el bag montado
read-only en `/physical_input` y Numba0.57.1/llvmlite0.40.1 en
`/tmp/profile_site057`, estos son los comandos dentro del contenedor aislado:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH=/tmp/bag16_source/slam:/tmp/profile_site057
mkdir -p /tmp/hitl_bag16_01/input
python3 /tmp/bag16_source/nav/scripts/bag_to_npz.py \
  /physical_input/bags/bag_mapa2 /tmp/hitl_bag16_01/input/bag_mapa2.npz \
  --base-frame base_footprint
bash /tmp/bag16_source/slam/scripts/run_hitl_bag16_01.sh
python3 /tmp/bag16_source/slam/scripts/analyze_bag_cycle.py \
  /tmp/hitl_bag16_01 --history /tmp/hitl_bag16_01/H
```

`H` debe contener una copia de `intento3/aula.*` para el análisis posterior.
El script rechaza sobrescribir A; el analizador rechaza sobrescribir `report`.
Los resultados archivados no se borran para volver a ejecutar comandos.
La importación de ROS se usa solo para deserializar el bag, sin reproducirlo.

## Resultados y diagnóstico del agente

Las cinco corridas terminaron: 963 integraciones, 2650 omisiones por movimiento,
cero stamps rechazados, scans inválidos, rayos truncados o partículas fuera de
grilla. Un scan fue excluido al extraer, antes del filtro. No se midieron
descartes/colas TF de ROS. Hay 2647 scans reservados dentro de la trayectoria;
los tres restantes están después del último scan integrado y no se extrapolan.

| Caso | Endpoints ≤10 cm % | P95 cm | Reservados ≤10 cm % | P95 reservados cm | Núcleo med/P95 ms | ESS/N medio | Remuestreos |
|---|---:|---:|---:|---:|---:|---:|---:|
| A | 95,462 | 9,21 | 94,042 | 12,16 | 132,3 / 165,6 | 0,716 | 106 |
| B | 95,462 | 9,21 | 94,042 | 12,16 | 56,4 / 72,5 | 0,716 | 106 |
| C | 95,528 | 9,14 | 93,880 | 12,31 | 111,8 / 289,6 | 0,696 | 131 |
| D | 95,469 | 9,10 | 94,040 | 13,32 | 327,3 / 371,9 | 0,684 | 145 |
| E | 95,898 | 8,21 | 94,711 | 10,72 | 345,9 / 404,2 | 0,704 | 119 |

A/B son idénticos bit a bit en grilla, trayectoria y PGM. Más partículas por sí
solas no muestran una mejora clara. D no reproduce el salto observado en Café:
la odometría física puede necesitar correcciones que un prior más fuerte
penaliza; es una hipótesis, no una calibración demostrada. E reduce las colas
de residuos en ambas selecciones de scans, con una mejora modesta. La distancia
inicio-fin es A/B 25,53 cm, C 29,04, D 23,06, E 21,33; no es error absoluto ni
un cierre medido con marcas.

La carga cambió durante C y fue elevada en D/E. El registro del host llegó a
loadavg de un minuto 13,55 en ocho hilos lógicos. No atribuir el salto temporal
C→D a los hiperparámetros ni elegir un perfil para ROS a partir de esta tabla.
Se conservan tiempos observados completos: >137 ms A 38,63 %, B 0 %, C 16,61 %,
D/E 99,90 %. Total de los cinco replays: 960,66 s (16,01 min), más preparación
y análisis; la estimación inicial fue menor. Hace falta una medición con carga
controlada y ROS1×, después de la evaluación humana.

El origen `(0,0)` es libre en su celda, pero no admite toda la huella de 22 cm
en ningún mapa, incluido H. Se conserva el resultado estricto desde el origen
(0 %), sin liberar celdas ni mover silenciosamente la pose. Como diagnóstico
adicional se toma un único punto interior, el primer checkpoint libre con
huella completa en **todos** los mapas: índice 83, posición de A
`(-0.351714, -0.118170)` y stamp `1794855412.5038261`. Desde él quedan
conectados H 83/181 checkpoints, A/B 89/181 y C/D/E 88/181. Todos los puntos
que admiten la huella pertenecen a esa componente (100 % de los admisibles en
cada mapa); los demás fallan por huella, no por una barrera entre componentes.
Es diagnóstico sobre posiciones estimadas y no certifica puertas ni navegación
desde una pose física concreta. Se archiva también el primer análisis con el
origen bloqueado para explicar la ampliación del diagnóstico.

Visualmente se mantiene el contorno de la sala y las tres filas de mesas. E
parece algo más limpio en parte de la pared superior; quedan irregularidades
en mesas y rayos por la abertura superior. No hay una mejora geométrica grande
demostrada frente a H. Una inclinación global entre mapas nativos puede ser
solo una diferencia de frame, no deformación del aula. No sustituir el mapa
elegido el 16/11 solo por un mayor porcentaje de endpoints.

Los cinco tests previos de métricas y tres casos analíticos del diagnóstico
del bag aprobaron. Los fuentes realmente ejecutados se verificaron por SHA256
contra el worktree y la revisión original; Numba0.57.1, NumPy1.21.5,
SciPy1.8.0, Python3.10.12. El contenedor aislado quedó detenido. No se tocó
el contenedor habitual, el árbol principal ni los perfiles físicos de uso.

**Estado:** todos los casos siguen en investigar; E es una señal prometedora
para revisar, no un finalista elegido. Evaluación humana pendiente, sin merge,
otras semillas, nuevo ciclo ni validación ROS.

- [Comparación de cinco mapas](../slam/artifacts/hitl_bag16_01/report/maps.png)
- [Detalle de paredes, mesas y esquinas](../slam/artifacts/hitl_bag16_01/report/details.png)
- [Métricas completas y diagnóstico](../slam/artifacts/hitl_bag16_01/report/comparison.json)
- [Tiempos y ESS](../slam/artifacts/hitl_bag16_01/report/timing_and_ess.png)
