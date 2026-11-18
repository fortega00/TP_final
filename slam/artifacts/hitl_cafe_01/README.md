# Ciclo aprobado Café HITL 01

Cinco ejecuciones del núcleo, semilla 7, con entradas idénticas.

- [Informe y revisión humana pendiente](report/README.md)
- [Cinco mapas](report/maps.png) · [Detalles](report/details.png)
- [Trayectorias y GT](report/maps_and_paths.png) · [Costo y ESS](report/timing_and_ess.png)
- [Tabla completa CSV](report/comparison.csv) · [Registro JSON](report/comparison.json)
- `manifest.json`: autorización, datos, runtime y hashes de fuentes.
- `hypotheses.json`: hipótesis y diagnóstico preliminar.
- `A` a `E`: mapas, configuración, grilla, trayectoria, métricas y trazas por actualización.
- `input/cafe_inputs.npz`: copia única de la entrada; scans.npz de cada caso es un enlace relativo verificado.
- `source`: snapshots de los ejecutores, métricas y chequeos utilizados.

Cierre: detenerse para inspección visual humana. No hubo nuevo ciclo, ROS, simulación, fusión ni push.
