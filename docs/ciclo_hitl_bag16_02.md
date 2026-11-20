# Ciclo colaborativo bag16/11 02: 24 partículas

**Evidencia histórica previa a la compuerta solo por movimiento.** Los
resultados usan `update.max_interval=1` y los fuentes/configuración del commit
`68ebc13`. Los perfiles instalables se migraron al esquema actual; no se
atribuyen estas métricas a la nueva compuerta. Para repetir las corridas
históricas, preparar aquella revisión; el runner rechaza el núcleo posterior.

Selección posterior del usuario tras revisar los mapas: los ve muy parecidos
y pide adoptar la configuración que mejor dio, priorizando rapidez/utilidad.
Se adopta E16 en el YAML habitual del robot/bag; ver
[decisión 013](decisiones/013-perfil-real-e16.md). La mejora de residuos es
modesta y ocho partículas son más rápidas. Los resultados, estados de revisión
y restricciones que siguen conservan el cierre original del ciclo, anterior
a esa selección. No se ejecutaron nuevas pruebas al adoptar el YAML.

El usuario pidió explícitamente: «subamos a 24 particulas y probemos como queda
el mapa», conservando los ejes de evaluación anteriores. Continúa en el
worktree `/tmp/tp-final-bag-20261116-cycle01`, rama
`exp/rosbag-20261116-cycle01`, base del ciclo `704b1f5`. No se fusiona ni se
sustituye el perfil físico habitual antes de la revisión humana.

## Hipótesis y corridas autorizadas

| Caso | Partículas | prior_xy m | sample_xy m | Propósito |
|---|---:|---:|---:|---|
| E16 | 16 | 0,050 | 0,005 | Control nuevo con carga actual; reproduce E del ciclo 01 |
| F24 | 24 | 0,050 | 0,010 | Efecto de aumentar partículas respecto de C16 |
| D24 | 24 | 0,025 | 0,010 | Efecto de aumentar partículas respecto de D16 |
| E24 | 24 | 0,050 | 0,005 | Efecto de aumentar partículas respecto de E16 |

Cuatro corridas nuevas, secuenciales y semilla 7. A/B de ocho partículas y H
se reutilizan del ciclo 01 sin repetir el estimador; sus tiempos pertenecen a
otra condición de carga y quedan identificados como reutilizados. Se repite
E16 porque el ciclo anterior cambió fuertemente de carga durante las corridas:
es necesario un control temporal cercano a E24. Estimación inicial: 8–15 min.
No se combina prior reducido con sample reducido en este ciclo.

La hipótesis es que más partículas mantengan hipótesis útiles y mejoren paredes,
mesas o consistencia sin degradar la transitabilidad. Más partículas también
pueden no aportar mejoras, y su mayor costo puede superar el presupuesto del
LiDAR. No se elige solo por cierre, endpoints ni rapidez.

## Reproducibilidad y evaluación

- Mismo `bag_mapa2` y NPZ SHA256
  `50946170bcc0bbb8874979f04299213e8551865513369cf30cc72a48c6229ebc`.
  3613 scans alineados; el mapa histórico no entra al filtro.
- Mismos parámetros físicos salvo N/prior/sample de la tabla. Los perfiles
  completos nuevos son `config/bag16_compare_24{,_prior,_sample}.yaml`.
- Backend Numba obligatorio: el runner aborta si `BACKEND != 'numba'`.
  NumPy1.21.5, Numba0.57.1, llvmlite0.40.1; fuente del núcleo sin cambios.
- Mismas métricas v2: 30 endpoints por scan, extrínseca real del NPZ, distancia
  a centros sin truncar; scans reservados y poses interpoladas correlacionadas,
  no validación física independiente ni GT.
- Mismos puntos de navegación del baseline A y mismos timestamps, inflación
  22 cm de ocupado/desconocido/exterior. Se fija el punto interior del ciclo 01,
  índice 83 y posición `(-0.351714186, -0.118170402)`, en lugar de buscar un
  punto favorable para los mapas nuevos. Si un mapa nuevo bloquea esa huella,
  se registra el fallo. Se conserva además el resultado estricto desde `(0,0)`.
- Mediana/P95/máximo y >90/>137 ms por integración; ESS/N, remuestreos, RSS,
  calentamiento separado. Se registra carga al empezar y terminar cada caso.
- No hay GT, RMSE ni cobertura física verificable. La distancia inicio-fin
  sigue siendo separación relativa, no error absoluto ni cierre verificado.
- Mapas H/E16/F24/D24/E24 con misma escala/orientación y regiones de detalle.
- E16 debe reproducir bit a bit E anterior: mapa crudo, trayectoria y PGM.
  Las métricas de calidad de casos anteriores se reutilizan; no se interpretan
  sus tiempos como si se hubieran medido en este ciclo.

Se usa `tp-final-profile-20261118`, aislado y sin red; fuentes copiados del
worktree a `/tmp/bag16_source`. `nice -n 15`, BLAS/OMP un hilo. Sin nodos ROS,
replay ROS1×, Gazebo, robot vivo ni cambios al contenedor habitual o a `main`.

Runner: `slam/scripts/run_hitl_bag16_02.sh`. Analizador:

```bash
PYTHONPATH=/tmp/bag16_source/slam:/tmp/profile_site057 \
python3 /tmp/bag16_source/slam/scripts/analyze_bag_cycle.py \
  /tmp/hitl_bag16_02 --history /tmp/hitl_bag16_02/H \
  --cases A B E16 F24 D24 E24 --plot-cases H E16 F24 D24 E24 \
  --interior-seed-index 83 --reused-cases A B \
  --repeat-check E16 /tmp/hitl_bag16_01/E \
  --title 'Bag 16/11 — ciclo 02, 24 partículas; revisión humana pendiente'
```

El analizador conserva su interfaz del ciclo 01 por defecto. Las opciones
nuevas cambian selección/presentación de casos y fijan el checkpoint; no
modifican las fórmulas de las métricas. Evidencia: `slam/artifacts/hitl_bag16_02/`.
Se detiene después de presentar resultados y mapas para evaluación humana.

## Resultados

Las cuatro corridas nuevas completaron 963 integraciones y 2650 omisiones por
la compuerta de movimiento. Cero stamps rechazados, scans inválidos, rayos
truncados o partículas fuera de grilla. El runner confirmó `numba` antes de
empezar y cada timing.json registra ese backend. No se midieron descartes TF,
colas o capacidad del adaptador ROS.

| Caso | Endpoints ≤10 cm % | P95 cm | Reservados ≤10 cm % | P95 reservados cm | Núcleo med/P95 ms | >137 ms % | ESS/N medio | Remuestreos |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| E16 | 95,898 | 8,21 | 94,711 | 10,72 | 107,8 / 122,2 | 0,42 | 0,704 | 119 |
| F24 | 95,452 | 9,23 | 93,973 | 13,27 | 163,8 / 185,6 | 93,25 | 0,695 | 134 |
| D24 | 87,210 | 15,49 | 76,396 | 19,11 | 164,4 / 188,7 | 99,07 | 0,680 | 161 |
| E24 | 95,320 | 9,50 | 92,861 | 12,18 | 166,4 / 201,3 | 98,13 | 0,702 | 124 |

E16 repitió exactamente la grilla cruda, trayectoria y PGM de E anterior. El
P95 pasó de 404,2 a 122,2 ms al volver a medir con menor carga. Esta repetición
no altera la conclusión de calidad del ciclo 01; proporciona una comparación
temporal más útil. El nuevo loadavg de un minuto estuvo entre 1,06 y 1,95 al
empezar/terminar casos (el ciclo anterior llegó a 13,55). Eso no equivale a
un benchmark con CPU aislada; se conserva todo el registro en `run.log`.

**No hay mejora de consistencia al subir a 24 partículas en esta semilla.**
F24 no supera C16 anterior; D24 empeora claramente frente a D16; E24 empeora
frente al control E16. La cantidad de partículas cambia las realizaciones
aleatorias y la hipótesis seleccionada, por lo que una sola semilla no
justifica descartar definitivamente N24. La propuesta con scan matching
sigue siendo aproximada (`log(p/q)=0`); aumentar N no garantiza una mejora
monótona de todos los criterios.

El costo crece de forma consistente: 24 partículas tienen mediana 164–166 ms
y P95 186–201 ms, fuera del presupuesto de 137 ms por integración. Muchos
scans son omitidos por movimiento; esta distribución por integración no prueba
por sí sola acumulación ni descartes en ROS. No se certifica ROS1× ni se cambia
automáticamente el ritmo del sensor para acomodar el resultado. RSS máximo
E16 272,2 MiB; F24/D24 336,7/336,8 MiB; E24 330,0 MiB. Calentamiento del
constructor medido aparte, alrededor de 0,11–0,12 s con caché JIT existente.
Tiempo total de las cuatro corridas: 581,31 s (9,69 min), más análisis.

## Geometría, trayectoria y contradicciones

Todos los mapas mantienen el contorno general y tres filas de mesas.
D24 puede parecer más recto en una vista general, pero esa apariencia no
coincide con sus residuos. Hay diferencias locales de espesor, continuidad
y ruido en bordes de mesas; no se han medido paredes reales para decidir
cuál es físicamente más fiel. Las inclinaciones globales en el frame nativo
no se interpretan automáticamente como deformación.

La separación inicio-fin es E16 21,33 cm, F24 21,14, D24 17,33 y E24 21,82.
D24 tiene la separación más pequeña y los peores endpoints: no elegir por
ese proxy de cierre. No se conoce error absoluto ni cierre con marcas físicas.

Con los mismos 181 checkpoints y seed83 fijo, E16/F24/D24/E24 tienen
88/84/104/86 puntos admisibles, respectivamente; todos los admisibles quedan
conectados. En el recorrido estimado propio son 89/89/163/88. D24 aumenta
la cantidad de huellas admisibles pero pierde consistencia sensorial: esa
contradicción requiere evaluación humana, no una elección automática. El
origen inicial sigue bloqueado por huella en todos; no se corrige artificialmente
la grilla. Los puntos estimados, el espacio libre y la conectividad no son GT
ni medidas de cobertura real verificada.

**Estado:** el agente conserva E16 como referencia cuantitativa de costo y
consistencia para comparar; no elige finalista ni adopta un perfil físico.
Todos los casos siguen en investigar hasta revisar juntos H/E16/F24/D24/E24,
en especial mesas y paredes de D24 frente a E16. No se ejecutan automáticamente
otras semillas, refinamientos ni validación ROS. Ocho tests analíticos de
métricas aprobados y contenedor aislado detenido después de copiar la evidencia.

- [Mapas a igual escala](../slam/artifacts/hitl_bag16_02/report/maps.png)
- [Detalles de paredes y mesas](../slam/artifacts/hitl_bag16_02/report/details.png)
- [Mapas y trayectorias](../slam/artifacts/hitl_bag16_02/report/maps_and_paths.png)
- [Tiempos y ESS](../slam/artifacts/hitl_bag16_02/report/timing_and_ess.png)
- [Comparación completa](../slam/artifacts/hitl_bag16_02/report/comparison.json)
