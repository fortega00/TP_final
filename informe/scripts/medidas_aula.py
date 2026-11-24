#!/usr/bin/env python3
"""Dibuja sobre el mapa del aula las paredes a medir con cinta y lo que mide el mapa.

Cada medida es el largo de un tramo de pared, de esquina a esquina, con la cinta
apoyada en el piso contra la pared. Sobre el mapa, la posición de cada pared se
obtiene caminando desde un punto libre hasta la primera pared. Uso, desde la raíz
del repo:

    python informe/scripts/medidas_aula.py
"""
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image

REPO = Path(__file__).resolve().parents[2]
MAP_DIR = REPO / 'sesiones/20261116/mapa/intento3_corregido'
OUTPUT = REPO / 'informe/figuras/medidas_aula.png'
FREE = 254
WALL_CELLS = 3
BLUE, VERMILION, PURPLE, ORANGE = '#0072B2', '#D55E00', '#CC79A7', '#E69F00'

# Cara interna de cada pared: nombre, eje de la recta de búsqueda ('x' horizontal,
# 'y' vertical), coordenada fija de la recta, punto libre de partida, extremo ('lo' o 'hi').
WALLS = {
    'x_izquierda': ('x', 0.0, -9.5, 'lo'),
    'x_derecha': ('x', -0.8, -5.0, 'hi'),
    'y_abajo': ('y', -8.7, 0.0, 'lo'),
    'y_escalon': ('y', -10.2, 0.0, 'hi'),
    'x_escalon': ('x', 6.9, -5.0, 'lo'),
    'y_arriba': ('y', -8.7, 0.0, 'hi'),
    'x_entrante': ('x', 6.9, -5.0, 'hi'),
    'y_entrante': ('y', -1.5, 0.0, 'hi'),
    'x_fondo': ('x', 5.8, 0.0, 'hi'),
    'y_fondo': ('y', 0.6, 5.8, 'lo'),
}

# Tramos: letra, esquina inicial y final como nombres de pared (x, y), lado hacia el que
# se dibuja la etiqueta (dx, dy) y de qué esquina a qué esquina va, para la tabla.
SEGMENTS = [
    ('A', ('x_izquierda', 'y_abajo'), ('x_derecha', 'y_abajo'), (0, 1)),
    ('B', ('x_izquierda', 'y_abajo'), ('x_izquierda', 'y_escalon'), (1, 0)),
    ('C', ('x_izquierda', 'y_escalon'), ('x_escalon', 'y_escalon'), (0, -1)),
    ('D', ('x_escalon', 'y_escalon'), ('x_escalon', 'y_arriba'), (1, 0)),
    ('E', ('x_escalon', 'y_arriba'), ('x_entrante', 'y_arriba'), (0, -1)),
    ('F', ('x_entrante', 'y_arriba'), ('x_entrante', 'y_entrante'), (-1, 0)),
    ('G', ('x_entrante', 'y_entrante'), ('x_fondo', 'y_entrante'), (0, -1)),
    ('H', ('x_derecha', 'y_abajo'), ('x_derecha', 'y_fondo'), (-1, 0)),
]

# posición de la etiqueta a lo largo del tramo cuando el medio tapa otra cosa
LABEL_AT = {'H': 0.72, 'E': 0.3}

# fotos: nombre, dónde pararse, hacia dónde apuntar
PHOTOS = [('F1', (-10.6, -2.3), (-8.4, -0.4)), ('F2', (-3.7, 6.75), (-5.9, 5.75))]


def load_map():
    meta = yaml.safe_load((MAP_DIR / 'map.yaml').read_text())
    image = np.array(Image.open(MAP_DIR / meta['image']))
    return image, meta['resolution'], meta['origin'][0], meta['origin'][1]


def free_run(image, res, ox, oy, axis, fixed, start):
    """Extremos (en metros) del tramo libre que contiene a `start` sobre la recta."""
    height = image.shape[0]
    if axis == 'x':
        row = height - 1 - int((fixed - oy) / res)
        line, origin = image[row, :], ox
    else:
        col = int((fixed - ox) / res)
        line, origin = image[::-1, col], oy
    i = int((start - origin) / res)
    if line[i] != FREE:
        raise ValueError(f'el punto de partida de la recta {axis}={fixed} no es libre')
    # Una pared son al menos WALL_CELLS celdas seguidas no libres; menos que eso es ruido suelto.
    lo = i
    while (line[lo - WALL_CELLS:lo] == FREE).any():
        lo -= 1
    hi = i
    while (line[hi + 1:hi + 1 + WALL_CELLS] == FREE).any():
        hi += 1
    return origin + lo * res, origin + (hi + 1) * res


def main():
    image, res, ox, oy = load_map()
    wall = {}
    for name, (axis, fixed, start, end) in WALLS.items():
        lo, hi = free_run(image, res, ox, oy, axis, fixed, start)
        wall[name] = lo if end == 'lo' else hi

    occupied = np.argwhere(image == 0)
    r0, c0 = occupied.min(0) - 8
    r1, c1 = occupied.max(0) + 8
    height = image.shape[0]
    extent = [ox + c0 * res, ox + (c1 + 1) * res, oy + (height - 1 - r1) * res, oy + (height - r0) * res]

    fig, ax = plt.subplots(figsize=(11, 12.5))
    ax.imshow(image[r0:r1 + 1, c0:c1 + 1], cmap='gray', extent=extent, vmin=0, vmax=255,
              interpolation='nearest')
    print('tramo  mapa [m]')
    for k, (name, start, end, side) in enumerate(SEGMENTS):
        color = BLUE if k % 2 == 0 else VERMILION
        p = np.array([wall[start[0]], wall[start[1]]])
        q = np.array([wall[end[0]], wall[end[1]]])
        length = float(np.linalg.norm(q - p))
        # el tramo se dibuja corrido 0,18 m hacia adentro para que no lo tape la pared
        inward = 0.18 * np.array(side)
        ax.plot(*zip(p + inward, q + inward), color=color, lw=6, solid_capstyle='butt')
        for corner in (p, q):
            ax.plot(*corner, 'o', color='white', mec='black', ms=11, mew=2, zorder=5)
        along = LABEL_AT.get(name, 0.5)
        label = p + along * (q - p) + 0.75 * np.array(side)
        ax.text(*label, f'{name}  ≈ {length:.2f} m', color='white', fontsize=15, fontweight='bold',
                ha='center', va='center', rotation=0 if side[0] == 0 else 90,
                bbox=dict(boxstyle='round,pad=0.3', fc=color, ec='none'), zorder=6)
        print(f'{name:5s}  {length:.2f}')

    for name, spot, target in PHOTOS:
        ax.annotate('', xy=target, xytext=spot,
                    arrowprops=dict(arrowstyle='-|>', color=PURPLE, lw=3, mutation_scale=28))
        ax.text(*spot, name, color='white', fontsize=15, fontweight='bold', ha='center', va='center',
                bbox=dict(boxstyle='circle,pad=0.35', fc=PURPLE, ec='black'))
    ax.plot(0, 0, marker='*', ms=22, color=ORANGE, mec='black')
    ax.text(-0.3, 0.0, 'partida', fontsize=13, ha='right', va='center',
            bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='black'))
    ax.set_xlabel('x [m]', fontsize=14)
    ax.set_ylabel('y [m]', fontsize=14)
    ax.tick_params(labelsize=12)
    ax.set_title('Paredes a medir con cinta, de esquina a esquina (círculos blancos)\n'
                 'El valor es lo que mide el mapa. F1 y F2: desde dónde sacar las fotos',
                 fontsize=15)
    fig.savefig(OUTPUT, dpi=130, bbox_inches='tight')


if __name__ == '__main__':
    main()
