#!/usr/bin/env python3
"""Genera las figuras de la parte C a partir de los datos guardados.

Usa el mapa del aula, el plan de exploración que calcula `mission`, las muestras
y detecciones de `bag_conos_2` y los logs de las dos corridas en el robot
(`sesiones/20261125/`). No usa ROS. Desde la raíz del repo, con `main` (que
tiene `mission/` y las sesiones) mergeado en esta rama:

    PYTHONPATH=slam:nav:mission python informe/scripts/figuras_c.py
"""
import csv
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from nav.core.config import NavConfig
from nav.core.map import free_cells, load_map, occupied_cells
from nav.core.planner import Costmap, plan as astar
from mission.core.config import MissionConfig
from mission.core.exploration import plan_viewpoints, prepare_map

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / 'informe/datos'
FIGURES = REPO / 'informe/figuras'
MAP = REPO / 'sesiones/20261116/mapa/intento3_corregido/map.yaml'
RUNS = REPO / 'sesiones/20261125'
BAG = REPO / 'mission/artifacts/bag_conos_2'
GAZEBO = REPO / 'mission/artifacts/gazebo_20261124'

# paleta Okabe-Ito, como en figuras_a.py
BLUE, ORANGE, GREEN, VERMILION, PURPLE, GREY = (
    '#0072B2', '#E69F00', '#009E73', '#D55E00', '#CC79A7', '#999999')

plt.rcParams.update({'font.size': 13, 'axes.titlesize': 14, 'axes.labelsize': 13,
                     'legend.fontsize': 11, 'savefig.dpi': 200, 'savefig.bbox': 'tight'})

ROOM = (-12.0, 1.2, -3.9, 7.9)  # ventana del aula en el mapa (x0, x1, y0, y1)
CONE_HEIGHT = 0.28              # medido con cinta: parte roja del cono


def exploration_plan():
    """Mapa original, mapa con mesas y bancos, mapa de costos de nav y el plan desde (0, 0)."""
    config = MissionConfig.load_yaml(REPO / 'mission/config/real.yaml')
    nav_config = NavConfig.load_yaml(REPO / 'nav/config/real.yaml')
    original = load_map(MAP, nav_config.sensor)
    grid, costmap = prepare_map(original, nav_config.planner, config.exploration)
    plan = plan_viewpoints(grid, costmap, config.exploration, start=(0.0, 0.0))
    return original, grid, Costmap(original, nav_config.planner), plan


def background(original, grid, plan=None):
    """Imagen RGB del aula: libre blanco, paredes negras, mesas y bancos gris oscuro.

    Con `plan`, el piso que la cámara ve desde los puntos en verde y el que no
    ve en rojo.
    """
    image = np.full((grid.height, grid.width, 3), 0.82)
    image[free_cells(original)] = 1.0
    if plan is not None:
        image[plan.targets & plan.covered] = (0.78, 0.92, 0.80)
        image[plan.targets & ~plan.covered] = (0.98, 0.72, 0.68)
    image[occupied_cells(grid) & ~occupied_cells(original)] = 0.45
    image[occupied_cells(original)] = 0.0
    c = grid.config
    extent = (c.origin_x, c.origin_x + grid.width * c.resolution,
              c.origin_y, c.origin_y + grid.height * c.resolution)
    return image, extent


def show_room(ax, image, extent):
    ax.imshow(image, origin='lower', extent=extent, interpolation='nearest')
    ax.set_xlim(ROOM[0], ROOM[1])
    ax.set_ylim(ROOM[2], ROOM[3])
    ax.set_aspect('equal')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')


def viewpoint_marks(ax, viewpoints, color=BLUE, alpha=1.0, size=12):
    for number, (x, y) in enumerate(viewpoints, start=1):
        ax.plot(x, y, 'o', color=color, markersize=size, alpha=alpha, zorder=4)
        ax.text(x, y, str(number), color='white', ha='center', va='center', fontsize=8,
                weight='bold', zorder=5)


def figure_plan(original, grid, nav_costmap, plan):
    image, extent = background(original, grid, plan)
    fig, ax = plt.subplots(figsize=(7.5, 7))
    show_room(ax, image, extent)
    stops = [(0.0, 0.0)] + list(plan.viewpoints)
    for number, (a, b) in enumerate(zip(stops[:-1], stops[1:])):
        segment = np.array(astar(nav_costmap, tuple(a), tuple(b)))
        ax.plot(segment[:, 0], segment[:, 1], '-', color=BLUE, lw=1.6, alpha=0.8,
                label='camino del A* de la parte B' if number == 0 else None)
    ax.plot(0, 0, 's', color=ORANGE, markersize=11, label='inicio', zorder=5)
    viewpoint_marks(ax, plan.viewpoints)
    ax.legend(loc='upper right', framealpha=0.95)
    fig.savefig(FIGURES / 'c_plan.png')
    plt.close(fig)
    return plan.summary()


def parse_run(log_path):
    """Paradas del robot, punto salteado, cono y objetivo de aproximación según el log de la misión."""
    goal = re.compile(r'\[navigator\]: goal \((-?[\d.]+), (-?[\d.]+), (-?[\d.]+) deg\)')
    events = {'stops': [], 'dropped': [], 'cone': None, 'approach': None, 'candidate': None}
    last_goal = None
    for line in log_path.read_text(errors='replace').splitlines():
        m = goal.search(line)
        if m:
            last_goal = (float(m.group(1)), float(m.group(2)))
        elif '[navigator]: goal reached' in line and last_goal is not None:
            events['stops'].append(last_goal)
        elif 'goal dropped' in line and last_goal is not None:
            events['dropped'].append(last_goal)
        elif 'red candidate at' in line:
            x, y = re.search(r'at \((-?[\d.]+), (-?[\d.]+)\)', line).groups()
            events['candidate'] = (float(x), float(y))
        elif 'cone confirmed at' in line:
            x, y = re.search(r'at \((-?[\d.]+), (-?[\d.]+)\)', line).groups()
            events['cone'] = (float(x), float(y))
        elif 'reached the cone' in line:
            events['approach'] = last_goal
            break
    return events


def figure_runs(original, grid, nav_costmap, plan):
    image, extent = background(original, grid)
    fig, axes = plt.subplots(1, 2, figsize=(15, 7.4))
    summary = {}
    for ax, (name, title) in zip(axes, [('corrida_corta', 'Corrida corta'),
                                       ('corrida_larga', 'Corrida larga')]):
        run = parse_run(RUNS / name / 'mision.log')
        summary[name] = run
        show_room(ax, image, extent)
        viewpoint_marks(ax, plan.viewpoints, color=GREY, alpha=0.8, size=10)
        # Sin trayectoria guardada: entre parada y parada, el camino que planifica
        # el A* de la parte B. Muestra por dónde fue, no la pose medida.
        stops = run['stops']
        for a, b in zip(stops[:-1], stops[1:]):
            if np.hypot(b[0] - a[0], b[1] - a[1]) < 0.1:
                continue
            segment = np.array(astar(nav_costmap, a, b))
            ax.plot(segment[:, 0], segment[:, 1], '-', color=BLUE, lw=1.6, alpha=0.8)
        stops = np.array(stops)
        ax.plot(stops[:, 0], stops[:, 1], 'o', color=BLUE, markersize=5)
        for x, y in run['dropped']:
            ax.plot(x, y, 'X', color=VERMILION, markersize=14, markeredgecolor='black', zorder=6)
        ax.plot(*run['cone'], '^', color=VERMILION, markersize=16, markeredgecolor='black', zorder=6)
        ax.plot(*run['approach'], 's', color=GREEN, markersize=11, markeredgecolor='black', zorder=6)
        ax.set_title(title)
    handles = [plt.Line2D([], [], color=GREY, marker='o', ls='', markersize=10, label='punto de observación'),
               plt.Line2D([], [], color=BLUE, marker='o', lw=1.6, markersize=5,
                          label='paradas y camino del A* entre ellas'),
               plt.Line2D([], [], color=VERMILION, marker='X', ls='', markersize=12, markeredgecolor='black',
                          label='objetivo abandonado (BLOCKED 15 s)'),
               plt.Line2D([], [], color=VERMILION, marker='^', ls='', markersize=13, markeredgecolor='black',
                          label='cono confirmado (posición estimada)'),
               plt.Line2D([], [], color=GREEN, marker='s', ls='', markersize=10, markeredgecolor='black',
                          label='objetivo de aproximación')]
    fig.legend(handles=handles, loc='lower center', ncol=3, frameon=False, bbox_to_anchor=(0.5, -0.06))
    fig.savefig(FIGURES / 'c_corridas.png')
    plt.close(fig)
    return summary


def figure_detection():
    """Cuatro imágenes de bag_conos_2 con la salida del detector y de la profundidad."""
    frames = [('frame_0100.png', '(a) conos azules: ignorados'),
              ('frame_0148.png', '(b) a 0,81 m'),
              ('frame_0452.png', '(c) entre sillas y ruedas, a 1,22 m'),
              ('frame_0587.png', '(d) contraluz, a menos de 0,6 m')]
    fig, axes = plt.subplots(2, 2, figsize=(9, 7))
    for ax, (name, label) in zip(axes.flat, frames):
        ax.imshow(Image.open(BAG / 'samples' / name))
        ax.set_title(label, fontsize=12)
        ax.axis('off')
    fig.tight_layout()
    fig.savefig(FIGURES / 'c_deteccion.png', dpi=130)
    plt.close(fig)


def figure_bag():
    """Posiciones medidas del cono en el frame de la odometría y alto medido según la distancia."""
    rows = list(csv.DictReader((DATA / 'c_bag_detecciones.csv').open()))
    summary = json.loads((BAG / 'summary.json').read_text())
    located = [r for r in rows if r['status'] == 'located']
    fig, (left, right) = plt.subplots(1, 2, figsize=(12, 5), gridspec_kw={'width_ratios': [1.15, 1]})

    left.scatter([float(r['x']) for r in located], [float(r['y']) for r in located], s=10,
                 color=VERMILION, alpha=0.5, label='cono medido en una imagen')
    for track in summary['tracks']:
        if track['measurements'] >= 50:
            left.plot(track['x'], track['y'], 'x', color='black', markersize=12, mew=2)
            left.annotate(f"{track['measurements']} mediciones\ndispersión {100 * track['spread_median_m']:.0f} cm",
                          (track['x'], track['y']), xytext=(0, -38), textcoords='offset points',
                          fontsize=10, ha='center')
    left.set_xlim(-2.0, 2.6)
    left.set_ylim(-2.6, -0.3)
    left.set_aspect('equal')
    left.set_xlabel('x odometría [m]')
    left.set_ylabel('y odometría [m]')
    left.set_title('Posición medida del cono')
    left.legend(loc='upper left', fontsize=10)
    left.grid(alpha=0.3)

    # Solo las detecciones ubicadas: las descartadas por tamaño tienen la
    # profundidad de la pared de atrás, no la del cono.
    distance = np.array([float(r['distance']) for r in located])
    height = np.array([float(r['height']) for r in located])
    right.scatter(distance, height, s=10, color=BLUE, alpha=0.5, label='alto medido en una imagen')
    edges = [0.6, 0.9, 1.2, 1.5, 2.1, 3.0]
    centers, medians = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        inside = (distance >= a) & (distance < b)
        if inside.sum() >= 8:
            centers.append(np.median(distance[inside]))
            medians.append(np.median(height[inside]))
    right.plot(centers, medians, '-o', color='black', lw=2, label='mediana por tramo de distancia')
    right.axhline(CONE_HEIGHT, color=GREEN, ls='--', lw=2, label='alto real (cinta)')
    right.set_ylim(0.15, 0.45)
    right.set_xlabel('distancia [m]')
    right.set_ylabel('alto medido [m]')
    right.set_title('Tamaño medido según la distancia')
    right.legend(loc='upper right', fontsize=10)
    right.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIGURES / 'c_bag.png')
    plt.close(fig)
    return list(zip(centers, medians, [int(((distance >= a) & (distance < b)).sum()) for a, b in zip(edges[:-1], edges[1:])]))


def figure_hole():
    """Captura de RViz del escenario aula_hueco en Gazebo, recortada al aula."""
    image = Image.open(GAZEBO / 'aula_hueco/rviz.png')
    image.save(FIGURES / 'c_hueco.png')


def main():
    original, grid, nav_costmap, plan = exploration_plan()
    print('plan', json.dumps(figure_plan(original, grid, nav_costmap, plan)))
    runs = figure_runs(original, grid, nav_costmap, plan)
    for name, run in runs.items():
        print(name, {k: v for k, v in run.items() if k != 'stops'}, 'stops', len(run['stops']))
    figure_detection()
    print('bag alto por tramo', figure_bag())
    figure_hole()


if __name__ == '__main__':
    main()
