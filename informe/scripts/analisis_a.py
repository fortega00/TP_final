#!/usr/bin/env python3
"""Resume las corridas de `experimentos_a.py` en una tabla por experimento.

No hay pose verdadera del aula, entonces todas las métricas miden consistencia:

- cierre: distancia entre la pose final y la inicial. El robot terminó cada
  recorrido en el lugar de partida, así que debería dar cerca de cero.
- dispersión: cuánto se apartan entre sí las trayectorias de las distintas
  semillas (RMS respecto de la trayectoria promedio).
- grosor: grosor mediano de las paredes del contorno del aula en el mapa.
- residuo: distancia media de los puntos de cada scan integrado a la celda
  ocupada más cercana del mapa final.

Uso, desde la raíz del repo:

    python informe/scripts/analisis_a.py

Escribe `informe/datos/resumen_a.csv` y muestra la tabla.
"""
import csv
import json
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / 'informe/datos'
RUNS = DATA / 'experimentos_a'
OUTPUT = DATA / 'resumen_a.csv'
FAILED_CLOSURE_M = 0.5


def load_map(run):
    meta = yaml.safe_load((run / 'map.yaml').read_text())
    image = np.array(Image.open(run / meta['image']))
    return image, meta['resolution'], np.array(meta['origin'][:2])


def run_lengths(mask):
    """Para cada celda verdadera, largo del tramo horizontal de celdas verdaderas que la contiene."""
    out = np.zeros(mask.shape, dtype=int)
    for r, row in enumerate(mask):
        edges = np.flatnonzero(np.diff(np.r_[0, row.astype(int), 0]))
        for start, stop in zip(edges[::2], edges[1::2]):
            out[r, start:stop] = stop - start
    return out


def wall_thickness(image, res):
    """Grosor mediano del contorno: la componente conexa más grande de celdas ocupadas."""
    occupied = image == 0
    labels, count = ndimage.label(occupied, structure=np.ones((3, 3)))
    sizes = ndimage.sum(occupied, labels, range(1, count + 1))
    outline = labels == 1 + int(np.argmax(sizes))
    # en una pared alineada con los ejes, el grosor es el tramo más corto de los dos
    thickness = np.minimum(run_lengths(outline), run_lengths(outline.T).T)
    return float(np.median(thickness[outline])) * res


def scan_residuals(run, bag, every):
    """Distancia de cada punto de los scans integrados a la pared más cercana del mapa."""
    image, res, origin = load_map(run)
    distance = ndimage.distance_transform_edt(image != 0) * res
    trajectory = np.load(run / 'trajectory.npy')
    data = np.load(DATA / f'{bag}.npz')
    meta, ranges = data['metadata'][::every], data['ranges'][::every]
    lx, ly, lyaw = data['laser']
    index = {round(float(stamp), 6): i for i, stamp in enumerate(meta[:, 0])}
    height = image.shape[0]
    per_scan = []
    for stamp, x, y, yaw in trajectory:
        i = index[round(float(stamp), 6)]
        r = ranges[i]
        ok = np.isfinite(r) & (r > meta[i, 3]) & (r < min(meta[i, 4], 8.0))
        angle = meta[i, 1] + meta[i, 2] * np.flatnonzero(ok) + lyaw
        bx = lx + r[ok] * np.cos(angle)
        by = ly + r[ok] * np.sin(angle)
        mx = x + bx * np.cos(yaw) - by * np.sin(yaw)
        my = y + bx * np.sin(yaw) + by * np.cos(yaw)
        col = ((mx - origin[0]) / res).astype(int)
        row = height - 1 - ((my - origin[1]) / res).astype(int)
        per_scan.append(distance[row, col].mean())
    return np.array(per_scan)


def spread_cm(trajectories):
    """RMS de la distancia de cada trayectoria a la trayectoria promedio, en tiempos comunes."""
    start = max(t[0, 0] for t in trajectories)
    stop = min(t[-1, 0] for t in trajectories)
    times = np.linspace(start, stop, 400)
    xy = np.array([[np.interp(times, t[:, 0], t[:, 1]), np.interp(times, t[:, 0], t[:, 2])]
                   for t in trajectories])
    deviation = np.linalg.norm(xy - xy.mean(axis=0), axis=1)
    return 100 * float(np.sqrt((deviation ** 2).mean()))


def summarize(folder):
    runs = sorted(p.parent for p in folder.glob('seed_*/run.json'))
    info = [json.loads((run / 'run.json').read_text()) for run in runs]
    closure = np.array([i['end_to_start_m'] for i in info])
    good = [run for run, c in zip(runs, closure) if c < FAILED_CLOSURE_M]
    maps = [load_map(run) for run in good]
    residual = [np.median(scan_residuals(run, info[0]['bag'], info[0]['every'])) for run in good]
    return dict(
        experimento=folder.name,
        corridas=len(runs),
        fallidas=len(runs) - len(good),
        cierre_mediana_cm=round(100 * float(np.median(closure)), 1),
        cierre_max_cm=round(100 * float(closure.max()), 1),
        dispersion_cm=round(spread_cm([np.load(run / 'trajectory.npy') for run in good]), 1)
        if len(good) > 1 else '',
        grosor_cm=round(100 * float(np.median([wall_thickness(m[0], m[1]) for m in maps])), 1)
        if good else '',
        residuo_cm=round(100 * float(np.median(residual)), 2) if good else '',
        integrados=int(np.median([i['diagnostics']['processed_scans'] for i in info])),
        remuestreos=int(np.median([i['diagnostics']['resamples'] for i in info])),
        update_mediana_ms=round(1000 * float(np.median([i['update_median_s'] for i in info])), 1),
    )


def main():
    rows = [summarize(folder) for folder in sorted(RUNS.iterdir()) if any(folder.glob('seed_*/run.json'))]
    with OUTPUT.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    widths = {key: max(len(key), *(len(str(row[key])) for row in rows)) for key in rows[0]}
    print('  '.join(key.ljust(widths[key]) for key in rows[0]))
    for row in rows:
        print('  '.join(str(row[key]).ljust(widths[key]) for key in row))


if __name__ == '__main__':
    main()
