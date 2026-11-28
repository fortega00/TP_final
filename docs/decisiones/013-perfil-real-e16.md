# 013 — Perfil de trabajo E16 para robot y bags reales

Estado: reemplazada por la [018](018-perfil-real-8-particulas.md) el 2026-11-25. El perfil real volvió a 8 partículas.

El 18/11/2026 el usuario revisó las comparativas y comentó que los mapas se
ven muy parecidos. Pidió adoptar la configuración que mejor dio, priorizando
un FastSLAM rápido y útil. Se adopta **E16** en
`slam/config/parcial_bag_scan_match.yaml`: 16 partículas, `prior_xy=0.05` y
`sample_xy=0.005`. El perfil de Café D sigue separado.

Al preparar el PR se incorporó el `main` con la
[compuerta solo por movimiento](012-integracion-con-el-robot-quieto.md).
El YAML instalable ya no contiene `update.max_interval`. Las métricas de abajo
son anteriores a esa corrección: no validan E16 con la compuerta actual ni
reemplazan el mapa `intento3_corregido` incorporado por esa revisión.

## Evidencia y costo de la elección

Bag físico `sesiones/20261116/bags/bag_mapa2`, semilla 7, mismo input alineado,
mapas propios y backend Numba. Comparación contra el **replay de la
configuración** usada el 16/11; el mapa histórico no es ground truth y no
conserva trayectoria para calcularle estos residuos.

| Métrica | Configuración 16/11, 8 partículas | E16 |
|---|---:|---:|
| Endpoints integrados ≤10 cm | 95,46 % | 95,90 % |
| Endpoints de scans reservados ≤10 cm | 94,04 % | 94,71 % |
| P95 de residuos reservados | 12,16 cm | 10,72 cm |
| Mediana/P95 del núcleo | 56,4/72,5 ms | 107,8/122,2 ms |

**E16 mejora modestamente la consistencia, pero es más lento que ocho
partículas.** Los tiempos se midieron en ciclos con distinta carga: no son un
benchmark aislado ni una comparación directa contra la PC del laboratorio.
E16 se repitió con mapa crudo, trayectoria y PGM idénticos; las variantes N24
ensayadas empeoraron los residuos y dieron P95 de 186–201 ms.

Se elige como compromiso de calidad/costo frente a esas variantes de 24. Ocho
partículas siguen siendo la alternativa más rápida medida. El P95 de E16
quedó debajo de 137 ms en ese replay del núcleo, con 0,42 % de integraciones
por encima; faltan el costo del adaptador y la validación ROS1×. No certificar
capacidad en vivo únicamente por ese número.

No hay GT ni error absoluto de pose. Los scans reservados no se integraron,
pero sus poses se interpolan de la trayectoria estimada y son datos
correlacionados de la misma sesión. Con inflación de 22 cm, el origen queda
bloqueado por huella en ambos mapas; los checkpoints admisibles están
conectados desde el punto interior común. No se demuestra una mejora global
de navegabilidad ni cobertura física.

## Reproducción y alcance

N/prior/sample, semilla, sensor, grilla y movimiento coinciden con el caso E16
medido. La actualización conserva los umbrales de desplazamiento/giro y adopta
la eliminación de la incorporación por tiempo del `main` actual. La selección
no cambia el YAML default, Café ni los perfiles históricos de odometría.

Para usar el YAML actualizado en el contenedor habitual, desde el host:

```bash
cd ~/TP_final
./slam/scripts/sync_container.sh --with-numba
./slam/scripts/container_env.sh status
```

El estado debe informar `SLAM backend: numba`. El YAML no activa Numba por sí
solo; es una selección del entorno Python. En los launch de bag/real indicar
`config_file:=/ruta/al/parcial_bag_scan_match.yaml`; los procesos existentes
deben relanzarse para leerlo. Esta adopción no lanzó pruebas ni modificó el
contenedor habitual automáticamente.

[Registro de selección y métricas](../../slam/artifacts/perfil_real_e16_20261118/selection.json).
Evidencia completa de los ciclos en la rama `exp/rosbag-20261116-cycle01`,
commit `68ebc13`, worktree `/tmp/tp-final-bag-20261116-cycle01`.
Pendientes: robustez con otras semillas y validación ROS1×. La adopción del
perfil no autoriza nuevas corridas ni una fusión automática de toda la rama.
