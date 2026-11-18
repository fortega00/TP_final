# Café HITL 01 — D elegido por el usuario

**Selección del 18/11/2026:** el usuario indicó «Excelente, usemos D entonces».
D se adopta como perfil de trabajo para Café con el código optimizado.
La elección no autoriza nuevos experimentos ni certifica el robot real.
Los comentarios de revisión pendiente que siguen describen el cierre previo
a esta selección; las métricas medidas permanecen intactas.

| Caso | N | RMSE cm | Endpoints ≤10 cm % | P95 extremo cm | Conectados % | Núcleo med/P95 ms | >137 ms % | RSS MiB |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| A | 8 | 15.31 | 92.43 | 13.35 | 100.00 | 94.0/125.3 | 0.69 | 84.2 |
| B | 8 | 15.31 | 92.43 | 13.35 | 100.00 | 35.8/41.6 | 0.00 | 196.4 |
| C | 16 | 9.68 | 94.55 | 10.58 | 100.00 | 70.1/91.5 | 0.00 | 188.4 |
| D | 16 | 6.12 | 98.29 | 7.42 | 100.00 | 69.7/79.7 | 0.00 | 197.9 |
| E | 16 | 9.08 | 96.83 | 8.13 | 100.00 | 68.5/79.5 | 0.00 | 196.5 |

La comparación completa está en comparison.json y comparison.csv.
Mapas: maps.png; trayectorias: maps_and_paths.png; ampliaciones: details.png.
Los tres paneles de detalle usan la misma escala entre casos para cada región.
Métrica de endpoints v2: extrínseca real del NPZ, centros de celdas y sin truncamiento.
Inflación 22 cm de ocupado, desconocido y exterior; puntos GT comunes.
El costo medido corresponde al núcleo, no al callback completo ROS.
A/B compara datos completos; si son iguales la rapidez no cuenta como mejora intrínseca.
Los umbrales son orientativos. No se ha elegido finalista ni autorizado fusión.

## Observaciones humanas

Pendientes: paredes dobles, ensanchamiento, habitaciones, pasillos y puerta inferior.
Estado de todos los casos: investigar hasta la revisión conjunta.

## Identificación y diagnóstico preliminar

A: original con ocho partículas; B: optimizado con ocho; C: optimizado con
dieciséis; D: C con prior_xy=0,025; E: C con sample_xy=0,005. El resto de los
parámetros coincide y todos usan la semilla 7.

- **A/B:** grilla cruda, trayectoria ancestral y PGM idénticos bit a bit. Tiempo
  de replay 136,50→51,83 s (2,63×), con pico RSS 84,2→196,4 MiB. La aceleración
  por sí sola conserva la estimación.
- **B/C:** más partículas reducen RMSE 15,31→9,68 cm y elevan endpoints a 10 cm
  92,43→94,55 %. El P95 del núcleo sube 41,6→91,5 ms: la ganancia tiene costo.
- **C/D:** prior de traslación más fuerte reduce RMSE 9,68→6,12 cm, P95 del residuo
  10,58→7,42 cm y mejora precisión/recall SDF. Es el caso con mejores métricas
  conjuntas de este ciclo; no es todavía una configuración finalista aprobada.
- **C/E:** menos ruido de propuesta reduce RMSE a 9,08 cm y P95 del residuo a
  8,13 cm. La inspección preliminar sugiere una discontinuidad junto al lateral
  izquierdo de la puerta inferior: revisar la tercera fila de details.png.
- Los cinco mapas conectan el 100 % de los **mismos 155 checkpoints GT** con
  inflación de 22 cm. Esta cifra no prueba integridad de todas las paredes.
- Cobertura observada aproximada: A/B 99,562 %, C 99,519 %, D 99,549 %, E 99,530 %.
  No mejora uniformemente toda métrica; la superficie evaluada y la inflación
  difieren de los informes históricos. Las diferencias pequeñas requieren
  interpretación; no usarlas como veto automático.
- ESS/N medio 0,662–0,680; mínimos 0,070–0,131 y 212–241 remuestreos. Hay
  concentraciones transitorias de pesos, no evidencia aquí de divergencia técnica.
  La curva timing_and_ess.png conserva los episodios para diagnóstico.
- Todos integraron los mismos 1458 scans, sin rechazos de timestamp, scans
  inválidos ni partículas fuera de la grilla. No se midieron pérdidas o colas
  ROS: esta ejecución fue exclusivamente del núcleo.

D está cerca del objetivo orientativo, aunque su RMSE supera 5 cm. El costo
medido no incluye publicación/recepción ROS y no certifica operación a 7,3 Hz
en el robot. Una sola semilla y una grabación no prueban robustez.

## Punto de control humano

Inspeccionar especialmente la unión del sector superior, pared derecha, puerta
inferior y discontinuidades de la pared inferior. Comparar D con C y E: ¿se
pierde geometría relevante pese a mejorar la tabla? Registrar las observaciones
en human_observations de comparison.json antes de acordar otro ciclo.

No se ejecutaron semillas nuevas, bags físicos ni ROS1×. No hubo merge ni push.
