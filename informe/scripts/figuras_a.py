#!/usr/bin/env python3
"""Genera las figuras de la parte A a partir de las corridas guardadas.

Uso, desde la raíz del repo, después de `experimentos_a.py` y `analisis_a.py`:

    PYTHONPATH=slam:nav python informe/scripts/figuras_a.py
"""
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / 'informe/datos'
RUNS = DATA / 'experimentos_a'
FIGURES = REPO / 'informe/figuras'
FINAL = RUNS / 'base/seed_7'  # idéntico a sesiones/20261116/mapa/intento3_corregido

# paleta Okabe-Ito
BLUE, ORANGE, GREEN, VERMILION, PURPLE, GREY = (
    '#0072B2', '#E69F00', '#009E73', '#D55E00', '#CC79A7', '#999999')

plt.rcParams.update({'font.size': 13, 'axes.titlesize': 14, 'axes.labelsize': 13,
                     'legend.fontsize': 12, 'savefig.dpi': 200, 'savefig.bbox': 'tight'})


def load_map(run):
    meta = yaml.safe_load((run / 'map.yaml').read_text())
    image = np.array(Image.open(run / meta['image']))
    return image, meta['resolution'], meta['origin'][0], meta['origin'][1]


def show_map(ax, run, window, pad=0.0):
    """Dibuja el mapa de una corrida recortado a `window` = (x0, x1, y0, y1) en metros."""
    image, res, ox, oy = load_map(run)
    height, width = image.shape
    extent = [ox, ox + width * res, oy, oy + height * res]
    ax.imshow(image, cmap='gray', extent=extent, vmin=0, vmax=255, interpolation='nearest')
    ax.set_xlim(window[0] - pad, window[1] + pad)
    ax.set_ylim(window[2] - pad, window[3] + pad)
    ax.set_aspect('equal')


def odometry_path(bag):
    """Trayectoria de la odometría sola, relativa a su primera pose."""
    odom = np.load(DATA / f'{bag}.npz')['odometry']
    odom = odom[np.isfinite(odom).all(axis=1)]
    x0, y0, yaw0 = odom[0]
    dx, dy = odom[:, 0] - x0, odom[:, 1] - y0
    return np.c_[dx * np.cos(yaw0) + dy * np.sin(yaw0), -dx * np.sin(yaw0) + dy * np.cos(yaw0)]


ROOM = (-12.1, 1.5, -4.1, 12.1)  # ventana que contiene el aula en el mapa de bag_mapa2


def figure_map():
    """Mapa del aula con las trayectorias, y el mismo mapa visto por el planificador."""
    from nav.core.config import NavConfig
    from nav.core.map import load_map as load_nav_map
    from nav.core.planner import Costmap, plan

    fig, (left, right) = plt.subplots(1, 2, figsize=(15, 8.6))
    show_map(left, FINAL, ROOM)
    slam = np.load(FINAL / 'trajectory.npy')
    odom = odometry_path('bag_mapa2')
    left.plot(odom[:, 0], odom[:, 1], color=VERMILION, lw=2.0, label='odometría sola')
    left.plot(slam[:, 1], slam[:, 2], color=BLUE, lw=2.0, label='SLAM')
    left.plot(odom[-1, 0], odom[-1, 1], 'o', color=VERMILION, ms=9, mec='black')
    left.plot(slam[-1, 1], slam[-1, 2], 'o', color=BLUE, ms=9, mec='black')
    left.plot(0, 0, '*', color=ORANGE, ms=22, mec='black', label='partida', zorder=5)
    left.legend(loc='upper left', framealpha=1)
    left.set_title('Mapa del aula y recorrido')
    left.set_xlabel('x [m]')
    left.set_ylabel('y [m]')

    config = NavConfig.load_yaml(REPO / 'nav/config/real.yaml')
    grid = load_nav_map(FINAL / 'map.yaml', config.sensor)
    costmap = Costmap(grid, config.planner)
    show_map(right, FINAL, ROOM)
    c = grid.config
    extent = [c.origin_x, c.origin_x + grid.width * c.resolution,
              c.origin_y, c.origin_y + grid.height * c.resolution]
    free = np.array(Image.open(FINAL / 'map.pgm'))[::-1] == 254
    inflated = np.ma.masked_where(~(free & ~costmap.traversable), np.ones(free.shape))
    right.imshow(inflated, origin='lower', extent=extent, cmap=matplotlib.colors.ListedColormap([ORANGE]),
                 alpha=0.55, interpolation='nearest')
    routes = [((-1.0, 0.0), (-10.5, 3.5), BLUE), ((-10.3, -2.4), (-5.0, 7.0), VERMILION),
              ((-0.8, 5.6), (-5.3, -0.7), GREEN)]
    for start, goal, color in routes:
        path = np.array(plan(costmap, start, goal))
        right.plot(path[:, 0], path[:, 1], color=color, lw=2.4)
        right.plot(*start, 'o', color=color, ms=9, mec='black')
        right.plot(*goal, 's', color=color, ms=9, mec='black')
    right.plot([], [], color=ORANGE, lw=8, alpha=0.55, label='zona inflada (0,22 m)')
    right.plot([], [], 'o', color='white', mec='black', ms=9, label='inicio')
    right.plot([], [], 's', color='white', mec='black', ms=9, label='objetivo')
    right.legend(loc='upper left', framealpha=1)
    right.set_title('Rutas de A* sobre el mapa')
    right.set_xlabel('x [m]')
    fig.savefig(FIGURES / 'a_mapa_aula.png')
    plt.close(fig)


FIGURES_BY_NAME = {'mapa': figure_map}


def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--only', nargs='+', choices=sorted(FIGURES_BY_NAME))
    args = parser.parse_args()
    for name in args.only or FIGURES_BY_NAME:
        FIGURES_BY_NAME[name]()
        print('figura', name)


if __name__ == '__main__':
    main()
