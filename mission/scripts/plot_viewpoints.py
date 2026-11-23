#!/usr/bin/env python3
"""Dibuja el plan de exploración sobre un mapa: puntos, recorrido y cobertura.

Sin ROS. Necesita matplotlib (requirements-evaluation.txt). Desde `mission/`:

    PYTHONPATH=../slam:../nav:. python3 scripts/plot_viewpoints.py \
        ../sesiones/20261116/mapa/intento3_corregido/map.yaml \
        --nav-config ../nav/config/real.yaml --start 0 0 --output viewpoints.png
"""

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from nav.core.config import NavConfig
from nav.core.map import free_cells, load_map, occupied_cells
from nav.core.planner import Costmap, plan as astar

from mission.core.config import MissionConfig
from mission.core.exploration import plan_viewpoints, prepare_map


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("map_yaml")
    parser.add_argument("--nav-config", help="YAML de nav: se usa su grupo planner (default: el de NavConfig)")
    parser.add_argument("--config", help="YAML de mission (default: valores de MissionConfig)")
    parser.add_argument("--start", type=float, nargs=2, metavar=("X", "Y"))
    parser.add_argument("--output", default="viewpoints.png")
    parser.add_argument("--paths", action="store_true",
                        help="dibujar el camino del A* de nav entre puntos, sobre el mapa original "
                             "(el que usa el navegador, sin los bloques de mesas de la misión)")
    args = parser.parse_args()

    config = MissionConfig.load_yaml(args.config) if args.config else MissionConfig()
    nav_config = NavConfig.load_yaml(args.nav_config) if args.nav_config else NavConfig()
    original = load_map(args.map_yaml)
    grid, costmap = prepare_map(original, nav_config.planner, config.exploration)
    plan = plan_viewpoints(grid, costmap, config.exploration, start=args.start)
    start = tuple(args.start) if args.start else plan.viewpoints[0]
    summary = plan.summary()
    print(json.dumps(summary))

    # Colores: desconocido gris, ocupado del mapa negro, mesas y bancos
    # detectados gris oscuro, libre transitable blanco, libre no transitable
    # gris claro; objetivo visto en verde y no visto en rojo.
    image = np.full((grid.height, grid.width, 3), 0.80)
    image[free_cells(grid)] = 0.93
    image[costmap.traversable] = 1.0
    image[plan.targets & plan.covered] = (0.75, 0.92, 0.75)
    image[plan.targets & ~plan.covered] = (0.98, 0.70, 0.70)
    # Lo que prepare_map agregó como ocupado: bloques de mesas y bancos.
    image[occupied_cells(grid) & ~occupied_cells(original)] = 0.35
    image[occupied_cells(original)] = 0.0
    c = grid.config
    extent = (c.origin_x, c.origin_x + grid.width * c.resolution,
              c.origin_y, c.origin_y + grid.height * c.resolution)
    rows, cols = np.nonzero(plan.targets)
    margin = 0.5
    xlim = (c.origin_x + cols.min() * c.resolution - margin, c.origin_x + (cols.max() + 1) * c.resolution + margin)
    ylim = (c.origin_y + rows.min() * c.resolution - margin, c.origin_y + (rows.max() + 1) * c.resolution + margin)

    figure, axes = plt.subplots(figsize=(9, 9))
    axes.imshow(image, origin="lower", extent=extent, interpolation="nearest")
    path = np.array([start] + list(plan.viewpoints))
    if args.paths:
        # El camino que calcularía el navegador de la etapa B en el robot: su A*
        # sobre el mapa original, que no conoce los bloques de la misión.
        nav_costmap = Costmap(original, nav_config.planner)
        crossing = 0
        zones = occupied_cells(grid) & ~occupied_cells(original)
        for number, (a, b) in enumerate(zip(path[:-1], path[1:])):
            segment = np.array(astar(nav_costmap, tuple(a), tuple(b)))
            cols, rows = original.world_to_grid(segment[:, 0], segment[:, 1])
            crossing += int(zones[rows, cols].sum())
            axes.plot(segment[:, 0], segment[:, 1], "-", color="tab:blue", linewidth=1.5, alpha=0.8,
                      label="camino del A* de nav" if number == 0 else None)
        print(json.dumps({"astar_points_inside_furniture_blocks": crossing}))
    else:
        # Solo el orden de visita: el camino real lo calcula el A* del navegador.
        axes.plot(path[:, 0], path[:, 1], ":", color="tab:blue", linewidth=1.2, alpha=0.7,
                  label="orden de visita (no es el camino)")
    axes.plot(*start, "s", color="tab:orange", markersize=9, label="inicio")
    for number, (x, y) in enumerate(plan.viewpoints, start=1):
        axes.plot(x, y, "o", color="tab:blue", markersize=11)
        axes.text(x, y, str(number), color="white", ha="center", va="center", fontsize=7, weight="bold")
    axes.set_xlim(*xlim)
    axes.set_ylim(*ylim)
    axes.set_aspect("equal")
    axes.set_xlabel("x [m]")
    axes.set_ylabel("y [m]")
    axes.set_title(f"{summary['viewpoints']} puntos, cobertura {100 * summary['coverage']:.1f} %, "
                   f"recorrido {summary['tour_m']:.1f} m (sin cruzar mesas)\n"
                   "verde: visto, rojo: no visto, gris oscuro: mesas y bancos detectados")
    axes.legend(loc="upper right")
    figure.tight_layout()
    figure.savefig(args.output, dpi=130)
    print(f"Saved {Path(args.output).resolve()}")


if __name__ == "__main__":
    main()
