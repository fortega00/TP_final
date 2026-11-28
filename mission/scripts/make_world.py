#!/usr/bin/env python3
"""Mundo de Gazebo Fortress a partir de un mapa del aula y un escenario con conos.

Sin ROS. Cada obstáculo del mapa se levanta como una caja, así el lidar
simulado ve lo mismo que vio el real y la localización funciona sobre el mismo
mapa. El escenario (YAML) agrega conos de colores y paredes extra:

    spawn: [-0.8, 0.0]          # punto del mapa donde aparece el robot
    walls:                      # rectángulos [x0, y0, x1, y1] en m, frame del mapa
      - [-2.3, 4.45, -0.9, 4.55]
    cones:
      - {x: -4.8, y: 3.2, color: red}

Se escribe también una copia del mapa (`<salida>_map.pgm/.yaml`), con las
paredes extra si las hay, para que la navegación las conozca y para tener el
mapa al lado del mundo dentro del contenedor.

El launch del curso hace aparecer al robot siempre en el origen del mundo,
mirando a +x. Para que aparezca en el punto `spawn` del mapa, el mundo entero
se corre en -spawn; el mapa no cambia. La pose inicial para la localización es
entonces (spawn, 0 rad). El origen del mapa del aula está a 15 cm de una pared:
ahí no entra el robot.

Desde `mission/`:

    PYTHONPATH=../slam:../nav:. python3 scripts/make_world.py \
        ../sesiones/20261116/mapa/intento3_corregido/map.yaml worlds/aula_conos.yaml worlds/aula_conos
"""

import argparse
from pathlib import Path

import numpy as np
import yaml

from nav.core.map import load_map, occupied_cells

COLORS = {"red": (0.85, 0.05, 0.05), "orange": (1.0, 0.45, 0.0), "yellow": (0.95, 0.85, 0.1),
          "green": (0.1, 0.7, 0.2), "blue": (0.1, 0.3, 0.9)}
WALL_HEIGHT = 0.5
# Lado de las celdas de colisión, en celdas del mapa (4 x 5 cm = 20 cm).
COLLISION_CELLS = 4
# Cono escalonado: Gazebo Fortress no tiene geometría de cono. Cinco cilindros
# de radio decreciente, 30 cm de alto y 20 cm de diámetro en la base.
CONE_HEIGHT, CONE_BASE, CONE_TOP, CONE_STEPS = 0.30, 0.10, 0.025, 5

HEADER = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>100</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.8 0.8 0.8 1</specular></material>
        </visual>
      </link>
    </model>
"""


def material(rgb):
    r, g, b = rgb
    return (f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
            f"<specular>0.1 0.1 0.1 1</specular></material>")


def rectangles(mask):
    """Rectángulos (fila0, fila1, col0, col1) que cubren la máscara: tramos por fila unidos hacia arriba."""
    open_runs, done = {}, []
    for row in range(mask.shape[0]):
        cols = np.flatnonzero(np.diff(np.concatenate(([0], mask[row].astype(np.int8), [0]))))
        runs = set(zip(cols[0::2], cols[1::2]))
        for run in list(open_runs):
            if run not in runs:
                done.append((open_runs.pop(run), row, *run))
        for run in runs:
            open_runs.setdefault(run, row)
    done.extend((start, mask.shape[0], *run) for run, start in open_runs.items())
    return done


def box_geometry(sx, sy):
    return f"<geometry><box><size>{sx:.3f} {sy:.3f} {WALL_HEIGHT}</size></box></geometry>"


def wall_model(name, visuals, collisions):
    """Un modelo estático: cajas visuales y de colisión, cada una (cx, cy, sx, sy).

    El lidar y la cámara ven las visuales, que siguen el mapa celda a celda.
    Las colisiones solo evitan que el robot atraviese algo: van más gruesas y
    son muchas menos, porque la física revisa choques cada milisegundo.
    """
    parts = []
    for i, (cx, cy, sx, sy) in enumerate(visuals):
        parts.append(f'      <visual name="v{i}"><pose>{cx:.3f} {cy:.3f} {WALL_HEIGHT / 2} 0 0 0</pose>'
                     f'{box_geometry(sx, sy)}{material((0.6, 0.6, 0.6))}</visual>\n')
    for i, (cx, cy, sx, sy) in enumerate(collisions):
        parts.append(f'      <collision name="c{i}"><pose>{cx:.3f} {cy:.3f} {WALL_HEIGHT / 2} 0 0 0</pose>'
                     f'{box_geometry(sx, sy)}</collision>\n')
    return (f'    <model name="{name}">\n      <static>true</static>\n      <link name="link">\n'
            + "".join(parts) + "      </link>\n    </model>\n")


def cone_model(name, x, y, rgb):
    parts = []
    step = CONE_HEIGHT / CONE_STEPS
    for i in range(CONE_STEPS):
        radius = CONE_BASE + (CONE_TOP - CONE_BASE) * i / (CONE_STEPS - 1)
        geometry = f"<geometry><cylinder><radius>{radius:.3f}</radius><length>{step:.3f}</length></cylinder></geometry>"
        z = step * (i + 0.5)
        parts.append(f'      <collision name="c{i}"><pose>0 0 {z:.3f} 0 0 0</pose>{geometry}</collision>\n'
                     f'      <visual name="v{i}"><pose>0 0 {z:.3f} 0 0 0</pose>{geometry}{material(rgb)}</visual>\n')
    return (f'    <model name="{name}">\n      <static>true</static>\n      <pose>{x} {y} 0 0 0 0</pose>\n'
            f'      <link name="link">\n' + "".join(parts) + "      </link>\n    </model>\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("map_yaml")
    parser.add_argument("scenario_yaml")
    parser.add_argument("output", help="prefijo de salida, sin extensión")
    args = parser.parse_args()
    scenario = yaml.safe_load(Path(args.scenario_yaml).read_text()) or {}
    output = Path(args.output)

    grid = load_map(args.map_yaml)
    c = grid.config
    occupied = occupied_cells(grid)
    extra = np.zeros_like(occupied)
    xs = c.origin_x + (np.arange(grid.width) + 0.5) * c.resolution
    ys = c.origin_y + (np.arange(grid.height) + 0.5) * c.resolution
    for x0, y0, x1, y1 in scenario.get("walls") or []:
        extra |= ((ys >= y0) & (ys <= y1))[:, None] & ((xs >= x0) & (xs <= x1))[None, :]

    sx, sy = scenario.get("spawn") or (0.0, 0.0)
    solid = occupied | extra

    def boxes(mask, cell):
        return [(c.origin_x + (c0 + c1) / 2 * cell - sx, c.origin_y + (r0 + r1) / 2 * cell - sy,
                 (c1 - c0) * cell, (r1 - r0) * cell) for r0, r1, c0, c1 in rectangles(mask)]

    # Colisiones en celdas de COLLISION_CELLS x COLLISION_CELLS: una celda gruesa
    # es sólida si tiene alguna celda ocupada. Agranda cada obstáculo hasta
    # 20 cm, menos que la holgura de la navegación (22 cm).
    k = COLLISION_CELLS
    h, w = -(-solid.shape[0] // k) * k, -(-solid.shape[1] // k) * k
    padded = np.zeros((h, w), dtype=bool)
    padded[:solid.shape[0], :solid.shape[1]] = solid
    coarse = padded.reshape(h // k, k, w // k, k).any(axis=(1, 3))
    visuals, collisions = boxes(solid, c.resolution), boxes(coarse, k * c.resolution)
    world = [HEADER.format(name=output.name), wall_model("aula", visuals, collisions)]
    for i, cone in enumerate(scenario.get("cones") or []):
        world.append(cone_model(f"cone_{i}_{cone['color']}", cone["x"] - sx, cone["y"] - sy, COLORS[cone["color"]]))
    world.append("  </world>\n</sdf>\n")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.with_suffix(".sdf").write_text("".join(world))
    print(f"{output.with_suffix('.sdf')}: {len(visuals)} visual boxes, {len(collisions)} collision boxes, "
          f"{len(scenario.get('cones') or [])} cones; "
          f"initial pose in the map: ({sx}, {sy}, 0)")

    # Copia del mapa, con las paredes extra, en el formato de map_server.
    meta = yaml.safe_load(Path(args.map_yaml).read_text())
    data = (Path(args.map_yaml).parent / meta["image"]).read_bytes()
    header_end = data.index(b"255\n") + 4
    image = np.frombuffer(data, dtype=np.uint8, offset=header_end).reshape(grid.height, grid.width).copy()
    image[np.flipud(extra)] = 0
    pgm = output.parent / f"{output.name}_map.pgm"
    pgm.write_bytes(data[:header_end] + image.tobytes())
    meta["image"] = pgm.name
    pgm.with_suffix(".yaml").write_text(yaml.safe_dump(meta, sort_keys=False))
    print(f"{pgm.with_suffix('.yaml')}: map with {int(extra.sum())} extra wall cells")


if __name__ == "__main__":
    main()
