# TP Final: SLAM y navegación autónoma

Trabajo final de Principios de la Robótica Autónoma (I-402), con un Yahboom RosMaster X3 y ROS 2 Humble. Las consignas están en [consignas/](consignas/).

| Etapa | Qué pide | Carpeta | Estado |
|---|---|---|---|
| A | Construir el mapa y estimar la pose (SLAM) | `slam/` | Implementada. Mapa del aula hecho con el robot real el 16/11, sin medidas con cinta |
| B | Localizarse en el mapa, planificar y navegar | `nav/` | Localización y navegación a un objetivo probadas en el simulador. Con evasión de obstáculos no mapeados. Falta correrla en el robot real |
| C | Explorar y buscar conos rojos con la cámara | `mission/` | Sin empezar |

## Organización

Hay un paquete ROS por etapa. Adentro de cada uno, los algoritmos y ROS están separados:

```
slam/                 paquete ROS de la etapa A
  slam/core/          algoritmos, sin ROS: filtro, grilla, sensor, movimiento, scan matching
  slam/ros/           nodos ROS: reciben scans y odometría, llaman al core y publican
  slam/toy/           mundo sintético para probar el algoritmo en segundos
  config/             perfiles de parámetros (YAML)
  launch/             un launch para toy, simulación, bag y robot real
  tests/              tests del core
  scripts/            replay de bags, experimentos, utilidades
  artifacts/          resultados guardados de las corridas
docs/                 guías, decisiones e informe
consignas/            los tres PDF de la materia
```

El método de la etapa A es FastSLAM: un filtro de partículas donde cada partícula lleva su propia grilla de ocupación, con scan matching para ubicar las partículas. Para leer el código conviene empezar por `slam/slam/core/filter.py`, que tiene los cinco pasos del algoritmo.

## Prueba rápida sin ROS

```bash
cd slam
python3 -m pip install -r requirements.txt
python3 -m unittest discover -s tests
python3 -m slam.toy.cli toy --config config/default.yaml --output /tmp/slam_toy --seed 7 --check
```

Etapa B, desde `nav/` (importa `slam.core`, por eso el `PYTHONPATH`):

```bash
cd nav
PYTHONPATH=../slam:. python3 -m unittest discover -s tests
```

## Con ROS

Enlazar la carpeta `slam/` dentro de `src/` de un workspace y compilar:

```bash
colcon build --symlink-install --packages-select slam
ros2 launch slam slam.launch.py mode:=toy
```

## Documentación

- [Guía de ejecución](docs/guia_ejecucion.md): instalación y launch para cada modo de la etapa A.
- [Guía de la etapa B](docs/guia_etapa_b.md): cómo correr la localización y la navegación, y cómo analizar un bag.
- [Parámetros](docs/parametros.md): qué hace cada uno y cómo ajustarlos.
- [Arquitectura](docs/architecture.md): módulos, máquina de estados y puntos de extensión.
- [Decisiones](docs/decisiones/): alternativas y datos de cada decisión de diseño.
- [Guía de laboratorio](docs/robot_real_checklist.md): qué hacer en cada una de las dos visitas que quedan, en orden y con el motivo de cada paso.
- [macOS](docs/macos.md): cómo correr el núcleo y el nodo ROS en Mac.
- [Informe de la etapa A](docs/entrega_a.md) y [experimentos](docs/experimentos_a.md).
- [Informe de la etapa B](docs/entrega_b.md), borrador.
- [Validación](docs/validacion.md): qué está probado y qué no.
- [AGENTS.md](AGENTS.md): contexto y convenciones para herramientas de IA.
