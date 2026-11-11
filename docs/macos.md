# Correr en macOS (Apple Silicon)

Gazebo Fortress no renderiza en macOS, así que el contenedor no sirve acá. Hay dos caminos, probados el 2026-11-11.

## Núcleo sin ROS (cualquier sistema operativo)

El núcleo, los tests y el replay offline del bag solo necesitan Python y [uv](https://docs.astral.sh/uv/):

```bash
uv run --no-project --with-requirements requirements-evaluation.txt python -m unittest discover -s tests
PYTHONPATH=. uv run --no-project --with-requirements requirements-evaluation.txt \
  python scripts/replay_bag_core.py --input artifacts/entrega_a_20261110/bag_inputs.npz \
  --config config/parcial_bag_tuned.yaml --output /tmp/replay
```

## Nodo ROS con Gazebo Classic (pixi / RoboStack)

Requiere el workspace de simulación de la materia con el entorno `classic` de pixi. Los comandos van desde la raíz de ese workspace.

Compilar (una vez, y de nuevo si cambian launch o entry points):

```bash
ln -s /ruta/a/TP_final tp_slam
pixi run -e classic colcon build --symlink-install --build-base build_classic \
  --install-base install_classic --packages-select tp_slam
```

En cada terminal:

```bash
pixi run -e classic bash
source install_classic/setup.bash
```

Toy, sin Gazebo:

```bash
ros2 launch tp_slam slam.launch.py mode:=toy rviz:=false
```

Café con odometría con deriva (una terminal por comando):

```bash
ros2 launch custom_map cafe.launch.py rviz:=False gui:=false
python scripts/macos_calc_odom_relay.py
ros2 launch tp_slam slam.launch.py mode:=simulation start_simulator:=false rviz:=false \
  odom_topic:=/slam_odom ground_truth_topic:=/odom \
  config_file:=$PWD/tp_slam/config/cafe.yaml
```

## Diferencias con el contenedor

- En Classic `/odom` es la pose real del simulador y no hay `/ground_truth/odom`. La odometría con deriva es `/calc_odom`.
- `/calc_odom` sale con `child_frame_id: calc_base`. El nodo compone la odometría con el TF entre ese frame y `base_footprint`, y eso devuelve la pose real. `scripts/macos_calc_odom_relay.py` republica el tópico como `/slam_odom` con `child_frame_id: base_footprint`. Sin el relay el SLAM no ve ninguna deriva.
- `cafe.launch.py` levanta un `map_server` que también publica `/map`. Para ver el mapa del SLAM en RViz hay que apagar ese nodo o remapear.
- `ros2 topic echo --once` se cuelga con tópicos volátiles. Para guardar el mapa: `ros2 service call /slam/save_map std_srvs/srv/Trigger`.
