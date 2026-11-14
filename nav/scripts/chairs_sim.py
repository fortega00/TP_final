#!/usr/bin/env python3
"""Prueba sintética de sillas movidas: agrega discos a los scans y al mapa.

Uso: chairs_sim.py <grabacion.npz> <mapa.yaml> <carpeta_salida> <cantidad> <radio_m> <semilla>

La grabación tiene que traer la pose real (`truth`), o sea que es de simulación.
Escribe en la carpeta de salida:

  scans.npz  la grabación con `cantidad` discos agregados a cada scan (intersección
             rayo-disco desde la pose real): sillas que están en la sala y no en el mapa
  map.yaml   el mapa con otros `cantidad` discos dibujados como celdas ocupadas:
             sillas que están en el mapa y ya no en la sala

Los discos van en celdas libres a más de 60 cm del recorrido. Después se corre
`replay_localization.py` con cada combinación. Resultados en docs/decisiones/011.
"""
import sys
from pathlib import Path
import numpy as np
import yaml

inputs, map_yaml, out, count, radius, seed = sys.argv[1:]
count, radius, out = int(count), float(radius), Path(out)
out.mkdir(parents=True, exist_ok=True)
rng = np.random.default_rng(int(seed))
data = dict(np.load(inputs))
meta = yaml.safe_load(Path(map_yaml).read_text())
raw = (Path(map_yaml).parent / meta["image"]).read_bytes()
lines = raw.split(b"\n", 3)
width, height = map(int, lines[1].split())
image = np.frombuffer(lines[3], np.uint8).reshape(height, width).copy()
res, (ox, oy, _) = meta["resolution"], meta["origin"]
free = np.flipud(image) == 254
truth = data["truth"]


def place(n):
    """n centros de disco en celdas libres, lejos del recorrido y dentro de la zona recorrida."""
    rows, cols = np.nonzero(free)
    x, y = ox + (cols + 0.5) * res, oy + (rows + 0.5) * res
    near = np.min(np.hypot(x[:, None] - truth[::5, 0], y[:, None] - truth[::5, 1]), axis=1)
    ok = (near > 0.6) & (near < 4.0)
    pick = rng.choice(np.flatnonzero(ok), n, replace=False)
    return np.c_[x[pick], y[pick]]


in_room, in_map = place(count), place(count)

# Sillas en la sala: se acorta cada rayo que choca con un disco.
laser = data["laser"]
ranges = data["ranges"].astype(np.float64)
for i, (m, (tx, ty, tyaw)) in enumerate(zip(data["metadata"], truth)):
    sx, sy = tx + laser[0] * np.cos(tyaw), ty + laser[0] * np.sin(tyaw)
    angles = tyaw + laser[2] + m[1] + np.arange(ranges.shape[1]) * m[2]
    dx, dy = np.cos(angles), np.sin(angles)
    for cx, cy in in_room:
        along = (cx - sx) * dx + (cy - sy) * dy
        across2 = (cx - sx) ** 2 + (cy - sy) ** 2 - along ** 2
        hit = (along > 0) & (across2 < radius ** 2)
        distance = along[hit] - np.sqrt(radius ** 2 - across2[hit])
        current = ranges[i, hit]
        ranges[i, hit] = np.where(distance < np.where(np.isfinite(current), current, np.inf),
                                  distance, current)
data["ranges"] = ranges.astype(np.float32)
np.savez(out / "scans.npz", **data)

# Sillas en el mapa: el otro grupo de discos se dibuja como celdas ocupadas.
rows, cols = np.mgrid[0:height, 0:width]
x, y = ox + (cols + 0.5) * res, oy + (height - 1 - rows + 0.5) * res
for cx, cy in in_map:
    image[np.hypot(x - cx, y - cy) < radius] = 0
(out / "map.pgm").write_bytes(b"\n".join(lines[:3]) + b"\n" + image.tobytes())
meta["image"] = "map.pgm"
(out / "map.yaml").write_text(yaml.safe_dump(meta))
print(f"{count} discs of {radius} m in scans.npz, {count} others in map.yaml")
