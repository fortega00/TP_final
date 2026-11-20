# Bag 16/11 — ciclo 01, revisión humana pendiente

| Caso | N | prior / sample m | Endpoints ≤10 cm % | Reservados ≤10 cm % | P95 extremos cm | Conectividad interior % | Med/P95 núcleo ms | >137 ms % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| A | 8 | 0.050 / 0.010 | 95.46 | 94.04 | 9.21 | 49.17 | 132.3/165.6 | 38.63 |
| B | 8 | 0.050 / 0.010 | 95.46 | 94.04 | 9.21 | 49.17 | 56.4/72.5 | 0.00 |
| C | 16 | 0.050 / 0.010 | 95.53 | 93.88 | 9.14 | 48.62 | 111.8/289.6 | 16.61 |
| D | 16 | 0.025 / 0.010 | 95.47 | 94.04 | 9.10 | 48.62 | 327.3/371.9 | 99.90 |
| E | 16 | 0.050 / 0.005 | 95.90 | 94.71 | 8.21 | 48.62 | 345.9/404.2 | 99.90 |

H es el mapa guardado intento3 el 16/11: no es ground truth.
A y B usan la configuración histórica; A original, B optimizado. C/D/E optimizados.
Endpoints v2: 30 haces por scan, extrínseca del bag, distancia a centros sin truncar.
2647 scans reservados: no integrados; poses interpoladas y datos correlacionados.
Conectividad: inflación 22 cm, ocupado/desconocido/exterior bloqueados.
Puntos comunes del replay A, además del recorrido propio: ambos son estimaciones.
El origen inicial no admite la huella de 22 cm en H/A/B/C/D/E: conectividad desde el origen 0 %. Sin liberar celdas artificialmente.
La columna interior usa el primer checkpoint seguro en todos los mapas; se archivan índice, pose y ambos resultados.
No hay RMSE ni error absoluto. Separación inicio-fin no certifica cierre.
Núcleo con prioridad baja en host compartido; no mide TF, colas ni descartes ROS.
Mapas en coordenadas nativas: misma escala, orientación, colores y sin ajuste al histórico.
Archivos: maps.png, maps_and_paths.png, details.png, timing_and_ess.png, navigation.png.
Detalles completos en comparison.json y comparison.csv.

## Evaluación humana

Pendiente: paredes dobles, continuidad de paredes, mesas, pasillos y puerta.
No se cambia el perfil físico de uso ni se fusiona este worktree.

## Diagnóstico del agente

E mejora modestamente endpoints y scans reservados. D no reproduce la ventaja de Café.
Más partículas por sí solas no mejoran claramente la consistencia. A/B son idénticos bit a bit.
No hay superioridad global demostrada sobre H: se necesita inspección humana del contorno y mesas.
Los checkpoints admisibles para la huella están todos conectados desde el punto interior común.
El origen sigue bloqueado por la huella, aunque su celda central es libre. No se liberan celdas.
Carga variable del host durante C/D/E: no atribuir sus tiempos solo a los parámetros ni certificar ROS1×.
Ocho tests analíticos únicos aprobados; resultados, fuentes efectivos y dependencias archivados.
Contenedor de pruebas detenido; sin cambios al contenedor habitual ni al árbol principal.

## Revisión solicitada

Comparar H, B y E: pared superior, encuentros de esquinas, bordes de mesas, continuidad y pasillos.
Todos los casos siguen en investigar; no se elige finalista ni se autoriza otro ciclo automáticamente.
