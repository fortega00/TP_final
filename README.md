# TP final: SLAM del RosMaster

Repositorio privado: [fortega00/TP_final](https://github.com/fortega00/TP_final).

Primera implementación de la entrega A: FastSLAM basado en un filtro de partículas Rao–Blackwellizado, con una grilla de ocupación independiente por partícula, actualización en log-odds y modelo inverso del LIDAR. El núcleo funciona sin ROS; el paquete `tp_slam` lo integra con ROS 2 Humble mediante una máquina de estados.

Desde la raíz del proyecto, con Python 3, NumPy y PyYAML:

```bash
python3 -m tp_slam.cli toy --output artifacts/toy --seed 7 --check
```

El ejemplo genera un mapa PGM/YAML, una figura SVG, la trayectoria y métricas reproducibles. Es una prueba sintética; las validaciones con Gazebo, el rosbag y el RosMaster físico se registran por separado.

- [A.md](A.md): instalación, launcher para cada modo, parámetros y diagnóstico.
- [Arquitectura](docs/architecture.md): módulos, máquina de estados y puntos de extensión.
- [Informe de la entrega A](docs/entrega_a.md): decisiones, evaluación y respuestas a las diez preguntas de la consigna.
- [AGENTS.md](AGENTS.md): contexto y convenciones para continuar las etapas A, B y C.

Las consignas originales están en los tres PDF de la raíz. El alcance implementado es el mapeo de A; B y C utilizarán estos mapas para navegación y exploración.
