#!/usr/bin/env python3
"""Repetibilidad de la localización sobre marcas del piso.

En el robot real no hay pose verdadera. El procedimiento es estacionar el robot
varias veces sobre las mismas cruces de cinta. Este script:

1. busca en la odometría de la grabación las paradas (robot quieto al menos
   `--min-stop` segundos);
2. toma la pose que estimó la localización en cada parada;
3. agrupa las paradas que caen a menos de `--mark-radius` entre sí: cada grupo
   es una marca;
4. informa, por marca, cuánto se separan las estimaciones de su promedio.

Mide precisión, no exactitud: un corrimiento fijo del mapa no aparece
(docs/decisiones/010). Si el robot no se estacionó con cuidado sobre la cruz,
ese error de manejo queda sumado.
"""
import argparse
import json
import math
from pathlib import Path
import numpy as np


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("recording", type=Path, help=".npz de bag_to_npz.py")
    p.add_argument("localization", type=Path, help="JSON de replay_localization.py sobre esa grabación")
    p.add_argument("--min-stop", type=float, default=3.0, help="duración mínima de una parada, s")
    p.add_argument("--mark-radius", type=float, default=0.5, help="paradas más cerca que esto son la misma marca, m")
    args = p.parse_args()

    data = np.load(args.recording)
    t, odom = data["metadata"][:, 0], data["odometry"]
    samples = json.loads(args.localization.read_text())["samples"]
    estimate_t = np.array([r["stamp"] for r in samples])
    estimate = np.array([[r["x"], r["y"], r["yaw"]] for r in samples])

    # Parada: tramo donde la odometría casi no cambia entre scans consecutivos.
    step = np.hypot(*np.diff(odom[:, :2], axis=0).T)
    turn = np.abs(np.arctan2(np.sin(np.diff(odom[:, 2])), np.cos(np.diff(odom[:, 2]))))
    still = np.r_[(step < 0.003) & (turn < 0.005), False]
    stops, start = [], None
    for i, flag in enumerate(still):
        if flag and start is None:
            start = i
        elif not flag and start is not None:
            if t[i] - t[start] >= args.min_stop:
                stops.append((t[start], t[i]))
            start = None
    if not stops:
        raise SystemExit("No stops found in the recording")

    # Pose estimada en cada parada: la última actualización antes de que termine.
    visits = []
    for begin, end in stops:
        index = np.searchsorted(estimate_t, end, side="right") - 1
        if index < 0:
            continue
        visits.append((begin, *estimate[index]))
    visits = np.array(visits)

    # Agrupar paradas cercanas: cada grupo es una marca.
    marks = []
    for visit in visits:
        for mark in marks:
            centre = np.mean([v[1:3] for v in mark], axis=0)
            if math.hypot(*(visit[1:3] - centre)) < args.mark_radius:
                mark.append(visit)
                break
        else:
            marks.append([visit])

    report = []
    for mark in marks:
        mark = np.array(mark)
        centre = mark[:, 1:3].mean(axis=0)
        distance = np.hypot(*(mark[:, 1:3] - centre).T)
        yaw = mark[:, 3]
        mean_yaw = math.atan2(np.sin(yaw).mean(), np.cos(yaw).mean())
        yaw_error = np.abs(np.arctan2(np.sin(yaw - mean_yaw), np.cos(yaw - mean_yaw)))
        report.append(dict(x=round(float(centre[0]), 2), y=round(float(centre[1]), 2),
                           visits=int(len(mark)),
                           spread_cm=round(100 * float(distance.max()), 1),
                           yaw_spread_deg=round(float(np.degrees(yaw_error.max())), 1),
                           at_seconds=[round(float(v - t[0]), 1) for v in mark[:, 0]]))
    print(json.dumps(dict(stops=len(visits), marks=report), indent=1))


if __name__ == "__main__":
    main()
