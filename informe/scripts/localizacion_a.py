#!/usr/bin/env python3
"""Localiza al robot sobre el mapa final con la localización de la parte B y la compara con el SLAM.

Repite los dos recorridos del laboratorio, sin ROS, con la pose inicial correcta y
varias semillas. No hay pose verdadera: se mide cuánto se aparta la localización
de la trayectoria que estimó el SLAM con el mismo bag.

- bag_mapa2 construyó el mapa, así que localización y SLAM están en la misma terna.
- bag_mapa es un recorrido independiente: su trayectoria de SLAM vive en otra terna
  y se alinea con el mejor ajuste rígido antes de comparar.

Uso, desde la raíz del repo, después de `experimentos_a.py`:

    PYTHONPATH=slam:nav python informe/scripts/localizacion_a.py
"""
import json
import math
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / 'informe/datos'
MAP = DATA / 'experimentos_a/base/seed_7/map.yaml'  # idéntico al mapa final
OUTPUT = DATA / 'localizacion_a'
SEEDS = (1, 2, 3, 4, 5, 6)
# bag: (pose inicial en el mapa, corrida de SLAM de referencia, alinear con ajuste rígido)
RUNS = {
    'bag_mapa2': ((0.0, 0.0, 0.0), 'base/seed_7', False),
    'bag_mapa': ((0.02, 0.08, math.pi), 'recorrido_1/seed_1', True),
}


def localize(job):
    bag, seed = job
    out = OUTPUT / f'{bag}_seed_{seed}.json'
    if not out.exists():
        subprocess.run([sys.executable, str(REPO / 'nav/scripts/replay_localization.py'),
                        '--input', str(DATA / f'{bag}.npz'), '--map', str(MAP),
                        '--config', str(REPO / 'nav/config/real.yaml'), '--output', str(out),
                        '--seed', str(seed), '--initial-pose', *map(str, RUNS[bag][0])],
                       check=True, capture_output=True)
    return bag, seed, json.loads(out.read_text())


def rigid_fit(points, reference):
    """Rotación y traslación que mejor llevan `points` sobre `reference` (Kabsch)."""
    pc, rc = points.mean(0), reference.mean(0)
    u, _, vt = np.linalg.svd((points - pc).T @ (reference - rc))
    rotation = vt.T @ np.diag([1.0, np.sign(np.linalg.det(vt.T @ u.T))]) @ u.T
    return (points - pc) @ rotation.T + rc


def score(bag, result):
    _, reference_run, fit = RUNS[bag]
    slam = np.load(DATA / 'experimentos_a' / reference_run / 'trajectory.npy')
    samples = result['samples']
    t = np.array([s['stamp'] for s in samples])
    inside = (t >= slam[0, 0]) & (t <= slam[-1, 0])
    xy = np.array([[s['x'], s['y']] for s in samples])[inside]
    reference = np.c_[np.interp(t[inside], slam[:, 0], slam[:, 1]),
                      np.interp(t[inside], slam[:, 0], slam[:, 2])]
    if fit:
        reference = rigid_fit(reference, xy)
    error = np.hypot(*(xy - reference).T)
    return dict(rmse_cm=100 * float(np.sqrt((error ** 2).mean())), max_cm=100 * float(error.max()),
                std_xy_max_cm=100 * max(s['std_xy'] for s in samples),
                update_ms=1000 * result['summary']['seconds_per_update'],
                burst_converged=result['summary']['burst_converged'])


def map_agreement():
    """Fracción de celdas ocupadas del mapa del otro recorrido que caen cerca de una del mapa final.

    El mapa de bag_mapa está en otra terna. Se lleva a la del mapa final con el ajuste
    rígido entre la trayectoria de su SLAM y la de la localización del mismo bag.
    """
    from PIL import Image
    from scipy import ndimage
    res, origin = 0.05, np.array([-16.0, -20.0])
    slam = np.load(DATA / 'experimentos_a/recorrido_1/seed_1/trajectory.npy')
    samples = json.loads((OUTPUT / 'bag_mapa_seed_1.json').read_text())['samples']
    t = np.array([s['stamp'] for s in samples])
    inside = (t >= slam[0, 0]) & (t <= slam[-1, 0])
    located = np.array([[s['x'], s['y']] for s in samples])[inside]
    mapped = np.c_[np.interp(t[inside], slam[:, 0], slam[:, 1]), np.interp(t[inside], slam[:, 0], slam[:, 2])]
    other = np.array(Image.open(DATA / 'experimentos_a/recorrido_1/seed_1/map.pgm'))
    final = np.array(Image.open(MAP.with_name('map.pgm')))
    rows, cols = np.nonzero(other == 0)
    cells = np.c_[origin[0] + (cols + 0.5) * res, origin[1] + (other.shape[0] - rows - 0.5) * res]
    # el ajuste rígido que lleva la trayectoria del SLAM sobre la de la localización lleva también las celdas
    pc, rc = mapped.mean(0), located.mean(0)
    u, _, vt = np.linalg.svd((mapped - pc).T @ (located - rc))
    rotation = vt.T @ np.diag([1.0, np.sign(np.linalg.det(vt.T @ u.T))]) @ u.T
    moved = (cells - pc) @ rotation.T + rc
    distance = ndimage.distance_transform_edt(final != 0) * res
    col = ((moved[:, 0] - origin[0]) / res).astype(int)
    row = final.shape[0] - 1 - ((moved[:, 1] - origin[1]) / res).astype(int)
    d = distance[row, col]
    return float((d <= 0.10).mean()), float((d <= 0.20).mean())


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    jobs = [(bag, seed) for bag in RUNS for seed in SEEDS]
    with ThreadPoolExecutor(6) as pool:
        results = list(pool.map(localize, jobs))
    rows = [dict(bag=bag, seed=seed, **score(bag, result)) for bag, seed, result in results]
    (DATA / 'localizacion_a.json').write_text(json.dumps(rows, indent=1) + '\n')
    for bag in RUNS:
        sel = [r for r in rows if r['bag'] == bag]
        print(bag, 'RMSE [cm] por semilla:', ' '.join(f"{r['rmse_cm']:.1f}" for r in sel),
              '| máximo [cm]:', ' '.join(f"{r['max_cm']:.0f}" for r in sel),
              '| ms por update:', f"{np.median([r['update_ms'] for r in sel]):.1f}",
              '| burst ok:', all(r['burst_converged'] for r in sel))
    near, far = map_agreement()
    print(f'celdas ocupadas del mapa de bag_mapa a 10 cm o menos del mapa final: {100 * near:.1f} %, a 20 cm o menos: {100 * far:.1f} %')


if __name__ == '__main__':
    main()
