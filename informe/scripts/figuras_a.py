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


ROOM = (-12.1, 1.5, -4.1, 12.1)  # ventana que contiene el laboratorio en el mapa de bag_mapa2


def figure_map():
    """Mapa del laboratorio con las trayectorias, y el mismo mapa visto por el planificador."""
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
    left.set_title('Mapa del laboratorio y recorrido')
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


def figure_scan_matching():
    """El mismo recorrido con la propuesta de odometría y con scan matching, 8 partículas."""
    fig, axes = plt.subplots(1, 2, figsize=(15, 8.6))
    cases = [(RUNS / 'odometria_8/seed_3', 'Propuesta de odometría, 8 partículas'),
             (RUNS / 'base/seed_3', 'Propuesta con scan matching, 8 partículas')]
    for ax, (run, title) in zip(axes, cases):
        show_map(ax, run, ROOM)
        path = np.load(run / 'trajectory.npy')
        ax.plot(path[:, 1], path[:, 2], color=BLUE, lw=1.6, label='trayectoria estimada')
        ax.plot(0, 0, '*', color=ORANGE, ms=22, mec='black', label='partida', zorder=5)
        ax.plot(path[-1, 1], path[-1, 2], 'o', color=VERMILION, ms=10, mec='black',
                label='llegada estimada', zorder=6)
        ax.set_title(title)
        ax.set_xlabel('x [m]')
    axes[0].set_ylabel('y [m]')
    axes[0].legend(loc='upper left', framealpha=1)
    fig.savefig(FIGURES / 'a_scan_matching.png')
    plt.close(fig)


# experimento, etiqueta; None separa grupos
SENSITIVITY_ROWS = [
    ('base', 'base: 8 partículas, celda 5 cm'),
    None,
    ('particulas_4', '4 partículas'),
    ('particulas_16', '16 partículas'),
    ('particulas_32', '32 partículas'),
    None,
    ('odometria_8', 'sin scan matching, 8 partículas'),
    ('odometria_128', 'sin scan matching, 128 partículas'),
    None,
    ('match_sigma_003', 'sigma del matching 0,03 m'),
    ('match_sigma_010', 'sigma del matching 0,10 m'),
    ('match_yaw_003', 'ventana de giro ±0,03 rad'),
    ('match_yaw_012', 'ventana de giro ±0,12 rad'),
    None,
    ('sigma_hit_005', 'sigma del sensor 0,05 m'),
    ('sigma_hit_030', 'sigma del sensor 0,30 m'),
    ('grosor_003', 'grosor de impacto 3 cm'),
    ('sin_retorno_libre', 'rayo sin retorno = libre'),
    None,
    ('paso_004', 'integrar cada 4 cm'),
    ('paso_016', 'integrar cada 16 cm'),
    None,
    ('celda_025', 'celda 2,5 cm'),
    ('celda_100', 'celda 10 cm'),
]


def per_seed(experiment):
    """Cierre [cm] de cada semilla de un experimento."""
    import json
    return np.array([100 * json.loads(p.read_text())['end_to_start_m']
                     for p in sorted((RUNS / experiment).glob('seed_*/run.json'))])


def summary():
    with (DATA / 'resumen_a.csv').open() as handle:
        return {row['experimento']: row for row in csv.DictReader(handle)}


def figure_sensitivity():
    """Error de cierre, nitidez del mapa y costo para cada cambio de un parámetro."""
    table = summary()
    with (DATA / 'tiempos_a.csv').open() as handle:
        timing = {row['experimento']: float(row['update_mediana_ms']) for row in csv.DictReader(handle)}
    fig, (left, right, cost) = plt.subplots(1, 3, figsize=(17, 9.3), sharey=True)
    y, ticks, labels = 0, [], []
    for row in SENSITIVITY_ROWS:
        if row is None:
            y -= 0.6
            continue
        name, label = row
        color = VERMILION if name == 'base' else BLUE
        closure = per_seed(name)
        median = float(np.median(closure))
        left.barh(y, median, color=color, height=0.62)
        # la línea negra va de la mejor a la peor semilla
        left.plot([closure.min(), closure.max()], [y, y], color='black', lw=2.2)
        left.plot([closure.min(), closure.max()], [y, y], '|', color='black', ms=11, mew=2.2)
        left.text(closure.max() + 1.2, y, f'{median:.0f}', va='center', fontsize=15)
        residual = float(table[name]['residuo_cm'])
        right.barh(y, min(residual, 5.2), color=color, height=0.62)
        right.text(min(residual, 5.2) + 0.08, y, f'{residual:.1f}'.replace('.', ','),
                   va='center', fontsize=15, bbox=dict(fc='white', ec='none', pad=1.5))
        cost.barh(y, timing[name], color=color, height=0.62)
        cost.text(timing[name] + 2.5, y, f'{timing[name]:.0f}', va='center', fontsize=15,
                  bbox=dict(fc='white', ec='none', pad=1.5))
        ticks.append(y)
        labels.append(label)
        y -= 1
    cost.axvline(137, color='black', lw=1.6, ls=':', zorder=0, label='llega un scan nuevo')
    cost.legend(loc='lower right', fontsize=15, framealpha=1)
    cost.set_xlim(0, 165)
    cost.set_title('Tiempo por actualización [ms]', fontsize=18)
    cost.set_xlabel('en una laptop, una corrida sola\ntiene que ser menor que 137 ms', fontsize=16)
    cost.axvline(timing['base'], color=VERMILION, lw=1.6, ls='--', zorder=0)
    cost.grid(axis='x', alpha=0.35)
    cost.set_axisbelow(True)
    cost.tick_params(labelsize=16)
    for ax, base in ((left, float(np.median(per_seed('base')))),
                     (right, float(table['base']['residuo_cm']))):
        ax.axvline(base, color=VERMILION, lw=1.6, ls='--', zorder=0, label='valor de la base')
        ax.grid(axis='x', alpha=0.35)
        ax.set_axisbelow(True)
        ax.tick_params(labelsize=16)
    left.legend(loc='lower right', fontsize=15, framealpha=1)
    left.set_yticks(ticks)
    left.set_yticklabels(labels)
    left.set_xlim(0, 70)
    left.set_ylim(y + 0.3, ticks[0] + 0.8)
    left.set_title('Error al volver a la partida [cm]', fontsize=18)
    left.set_xlabel('barra: mediana de las semillas\nlínea negra: de la mejor a la peor semilla', fontsize=16)
    right.set_xlim(0, 6)
    right.set_title('Distancia scan a pared [cm]', fontsize=18)
    right.set_xlabel('promedio de los scans integrados\ncuanto menor, más nítido el mapa', fontsize=16)
    fig.tight_layout()
    fig.savefig(FIGURES / 'a_sensibilidad.png')
    plt.close(fig)


def figure_cell():
    """El mismo rincón del laboratorio mapeado con tres tamaños de celda."""
    window = (-9.7, -5.0, 3.6, 8.1)
    cases = [('celda_025', 'Celda de 2,5 cm'), ('base', 'Celda de 5 cm'), ('celda_100', 'Celda de 10 cm')]
    fig, axes = plt.subplots(1, 3, figsize=(15, 5.3))
    for ax, (name, title) in zip(axes, cases):
        show_map(ax, RUNS / name / 'seed_1', window)
        ax.set_title(title)
        ax.set_xlabel('x [m]')
    axes[0].set_ylabel('y [m]')
    fig.savefig(FIGURES / 'a_celda.png')
    plt.close(fig)


def figure_speed():
    """Residuo de cada scan integrado según la velocidad de giro del robot en ese momento."""
    import sys
    sys.path.insert(0, str(Path(__file__).parent))
    from analisis_a import scan_residuals

    residual = 100 * scan_residuals(FINAL, 'bag_mapa2', 1)
    stamps = np.load(FINAL / 'trajectory.npy')[:, 0]
    data = np.load(DATA / 'bag_mapa2.npz')
    t, odom = data['metadata'][:, 0], data['odometry']
    k = 2  # velocidad promediada sobre 4 scans, unos 0,5 s
    dt = t[2 * k:] - t[:-2 * k]
    turn = np.abs(np.angle(np.exp(1j * (odom[2 * k:, 2] - odom[:-2 * k, 2])))) / dt
    speed = np.hypot(odom[2 * k:, 0] - odom[:-2 * k, 0], odom[2 * k:, 1] - odom[:-2 * k, 1]) / dt
    index = np.clip(np.searchsorted(t, stamps) - k, 0, len(dt) - 1)
    turn, speed = turn[index], speed[index]

    fig, (left, right) = plt.subplots(2, 1, figsize=(7, 8.2))
    panels = [(left, turn, [0, 0.1, 0.3, 0.5, 0.8, 2.0], 'velocidad de giro [rad/s]', ['< 0,1', '0,1–0,3', '0,3–0,5', '0,5–0,8', '> 0,8']),
              (right, speed, [0, 0.15, 0.3, 0.4, 1.0], 'velocidad de avance [m/s]', ['< 0,15', '0,15–0,3', '0,3–0,4', '> 0,4'])]
    for ax, value, edges, label, names in panels:
        means, counts = [], []
        for lo, hi in zip(edges[:-1], edges[1:]):
            inside = (value >= lo) & (value < hi)
            means.append(residual[inside].mean())
            counts.append(int(inside.sum()))
        bars = ax.bar(names, means, color=BLUE, width=0.7)
        for bar, count in zip(bars, counts):
            ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.05, f'{count} scans',
                    ha='center', fontsize=14)
        ax.set_xlabel(label, fontsize=17)
        ax.set_ylabel('residuo medio [cm]', fontsize=17)
        ax.set_ylim(0, 3.0)
        ax.tick_params(labelsize=15)
        ax.grid(axis='y', alpha=0.35)
        ax.set_axisbelow(True)
    fig.tight_layout()
    fig.savefig(FIGURES / 'a_velocidad.png')
    plt.close(fig)


FIGURES_BY_NAME = {'mapa': figure_map, 'scan_matching': figure_scan_matching,
                   'sensibilidad': figure_sensitivity, 'celda': figure_cell,
                   'velocidad': figure_speed}


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
