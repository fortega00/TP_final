# Corridas de A — 10/11/2026

- Café: [RViz](cafe/rviz.png), [mapa y trayectorias](cafe/map_and_paths.png), [errores GT](cafe/pose_errors.png), [cobertura](cafe/coverage.png), [PGM/YAML](cafe/map.yaml), [métricas](cafe/metrics.json).
- Bag real ajustado: [RViz](rosbag_tuned/rviz.png), [mapa y trayectorias](rosbag_tuned/map_and_paths.png), [comparación con mapa anterior](rosbag_tuned/reference_comparison.png), [PGM/YAML](rosbag_tuned/map.yaml), [métricas](rosbag_tuned/metrics.json).
- Referencia ROS del bag con defaults: `rosbag/` (falla calidad).
- Screening sin ROS: `screen_64/`, `screen_128/`, `screen_robust/`. No son corridas ROS ni GT.
- Entrada derivada de los 1188 scans y odometría alineada: `bag_inputs.npz`; identificación del original: `provenance.json`.

Café: RMSE 2,063 cm y cobertura observada aproximada 99,863 %. Bag: separación inicio–fin 0,842 m frente a 17,855 m de odometría; conserva discontinuidades y ruido, por lo que no se certifica como mapa definitivo de navegación. Los dos procesos principales terminaron con código cero. Los archivos están persistidos en el host y no dependen de mantener el contenedor encendido.

Informe y límites: [docs/experimentos_a.md](../../docs/experimentos_a.md). Reproducción: [A.md](../../A.md), sección 10.
