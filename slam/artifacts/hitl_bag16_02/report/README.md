# Bag 16/11 — ciclo 02, 24 partículas; revisión humana pendiente

| Caso | N | prior / sample m | Endpoints ≤10 cm % | Reservados ≤10 cm % | P95 extremos cm | Conectividad interior % | Med/P95 núcleo ms | >137 ms % |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| A | 8 | 0.050 / 0.010 | 95.46 | 94.04 | 9.21 | 49.17 | 132.3/165.6 | 38.63 |
| B | 8 | 0.050 / 0.010 | 95.46 | 94.04 | 9.21 | 49.17 | 56.4/72.5 | 0.00 |
| E16 | 16 | 0.050 / 0.005 | 95.90 | 94.71 | 8.21 | 48.62 | 107.8/122.2 | 0.42 |
| F24 | 24 | 0.050 / 0.010 | 95.45 | 93.97 | 9.23 | 46.41 | 163.8/185.6 | 93.25 |
| D24 | 24 | 0.025 / 0.010 | 87.21 | 76.40 | 15.49 | 57.46 | 164.4/188.7 | 99.07 |
| E24 | 24 | 0.050 / 0.005 | 95.32 | 92.86 | 9.50 | 47.51 | 166.4/201.3 | 98.13 |

H es el mapa guardado intento3 el 16/11: no es ground truth.
A y B usan la configuración histórica; A original, B optimizado. Las demás variantes son optimizadas.
Endpoints v2: 30 haces por scan, extrínseca del bag, distancia a centros sin truncar.
2647 scans reservados: no integrados; poses interpoladas y datos correlacionados.
Conectividad: inflación 22 cm, ocupado/desconocido/exterior bloqueados.
Puntos comunes del replay A, además del recorrido propio: ambos son estimaciones.
El resultado estricto desde el origen se conserva en comparison.json; no se liberan celdas artificialmente.
La columna interior usa el checkpoint documentado en comparison.json; se archivan índice, pose y ambos resultados.
No hay RMSE ni error absoluto. Separación inicio-fin no certifica cierre.
Núcleo con prioridad baja en host compartido; no mide TF, colas ni descartes ROS.
Mapas en coordenadas nativas: misma escala, orientación, colores y sin ajuste al histórico.
Archivos: maps.png, maps_and_paths.png, details.png, timing_and_ess.png, navigation.png.
Detalles completos en comparison.json y comparison.csv.

## Evaluación humana

Pendiente: paredes dobles, continuidad de paredes, mesas, pasillos y puerta.
No se cambia el perfil físico de uso ni se fusiona este worktree.

## Diagnóstico del agente

No hay mejora de endpoints con 24 partículas en semilla 7. E16 conserva los mejores residuos y menor costo de esta comparación.
D24 mejora separación inicio-fin y cantidad de checkpoints admisibles, pero empeora mucho los residuos. No seleccionar por esos proxies.
Los contornos generales se conservan; revisar mesas y continuidad de paredes con el usuario.
Las cuatro corridas nuevas usaron Numba; E16 reproduce bit a bit E del ciclo01.
A/B son evidencia reutilizada: sus tiempos no se midieron con la carga de este ciclo.
La carga actual fue menor y más estable; los casos N24 superan el presupuesto de 137 ms por integración.
No se midieron colas, TF ni descartes ROS; esto no certifica ni descarta por sí solo toda la capacidad del adaptador con omisiones por movimiento.
Todos los puntos admisibles por huella de 22 cm pertenecen a una componente interior común; el origen continúa bloqueado por huella.
Semilla 7 solamente, sin GT: ninguna variante se adopta ni se descarta definitivamente. Revisión humana pendiente.
