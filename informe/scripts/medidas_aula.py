#!/usr/bin/env python3
"""Dibuja sobre el mapa del aula las distancias a medir con cinta y lo que mide el mapa.

Cada medida es la distancia entre las caras internas de dos paredes a lo largo de
una recta. Sobre el mapa se obtiene caminando desde un punto libre hasta la primera
pared, en cada sentido. Uso, desde la raíz del repo:

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
# fotos: nombre, dónde pararse, hacia dónde apuntar
PHOTOS = [('F1', (-10.9, -2.8), (-8.9, -0.9)), ('F2', (-2.3, 7.1), (-4.3, 5.2 + 1.0))]
# posición de cada etiqueta a lo largo de su flecha, para que no se tapen
LABEL_AT = {'W1': 0.6, 'W2': 0.6, 'W3': 0.5, 'H1': 0.55, 'H2': 0.83, 'H3': 0.8, 'C': 0.5}

# nombre, eje de la recta ('x' horizontal, 'y' vertical), coordenada fija, punto libre de partida, color
MEASURES = [
    ('W1', 'x', -0.8, -5.0, '#0072B2'),
    ('W2', 'x', 3.0, -5.0, '#0072B2'),
    ('W3', 'x', 6.9, -5.0, '#0072B2'),
    ('H1', 'y', -10.2, 0.0, '#D55E00'),
    ('H2', 'y', -8.7, 0.0, '#D55E00'),
    ('H3', 'y', -1.5, 0.0, '#D55E00'),
    ('C', 'x', 6.9, -5.0, '#009E73'),
]


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
    occupied = np.argwhere(image == 0)
    r0, c0 = occupied.min(0) - 8
    r1, c1 = occupied.max(0) + 8
    height = image.shape[0]
    extent = [ox + c0 * res, ox + (c1 + 1) * res, oy + (height - 1 - r1) * res, oy + (height - r0) * res]

    fig, ax = plt.subplots(figsize=(11, 12.5))
    ax.imshow(image[r0:r1 + 1, c0:c1 + 1], cmap='gray', extent=extent, vmin=0, vmax=255,
              interpolation='nearest')
    ends = {}
    for name, axis, fixed, start, color in MEASURES:
        ends[name] = free_run(image, res, ox, oy, axis, fixed, start)
    # C es el ancho del escalón: de la pared izquierda (extremo de W2) a la pared del escalón.
    ends['C'] = (ends['W2'][0], ends['W3'][0])
    fixed_of = {m[0]: m[2] for m in MEASURES}
    fixed_of['C'] = 4.75
    print('medida  mapa [m]')
    for name, axis, _, _, color in MEASURES:
        a, b = ends[name]
        fixed = fixed_of[name]
        p, q = ((a, fixed), (b, fixed)) if axis == 'x' else ((fixed, a), (fixed, b))
        ax.annotate('', xy=q, xytext=p, arrowprops=dict(arrowstyle='<->', color=color, lw=2.6,
                                                        shrinkA=0, shrinkB=0))
        along = LABEL_AT[name]
        mid = (p[0] + along * (q[0] - p[0]), p[1] + along * (q[1] - p[1]))
        ax.text(*mid, f'{name}  ≈ {b - a:.2f} m', color='white', fontsize=15, fontweight='bold',
                ha='center', va='center', rotation=0 if axis == 'x' else 90,
                bbox=dict(boxstyle='round,pad=0.3', fc=color, ec='none'))
        print(f'{name:6s}  {b - a:.2f}')
    ax.plot(0, 0, marker='*', ms=22, color='#E69F00', mec='black')
    ax.text(-0.25, 0.0, 'partida', fontsize=13, ha='right', va='center',
            bbox=dict(boxstyle='round,pad=0.2', fc='white', ec='black'))
    # desde dónde sacar las fotos del aula: posición y hacia dónde apuntar
    for name, spot, target in PHOTOS:
        ax.annotate('', xy=target, xytext=spot,
                    arrowprops=dict(arrowstyle='-|>', color='#CC79A7', lw=3, mutation_scale=28))
        ax.text(*spot, name, color='white', fontsize=15, fontweight='bold', ha='center', va='center',
                bbox=dict(boxstyle='circle,pad=0.35', fc='#CC79A7', ec='black'))
    ax.set_xlabel('x [m]', fontsize=14)
    ax.set_ylabel('y [m]', fontsize=14)
    ax.tick_params(labelsize=12)
    ax.set_title('Distancias a medir con cinta, entre caras internas de pared\n'
                 '(el valor es lo que mide el mapa). F1 y F2: fotos', fontsize=15)
    fig.savefig(OUTPUT, dpi=130, bbox_inches='tight')


if __name__ == '__main__':
    main()
