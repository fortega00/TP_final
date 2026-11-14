#!/usr/bin/env python3
"""Compara dos trayectorias estimadas sobre la misma grabación.

Sirve para medir la localización contra el SLAM cuando no hay pose real. Acepta
la salida de `replay_localization.py` y el `samples.json` del SLAM de la etapa A.

Informa dos números:
- directo: distancia entre las dos trayectorias tal como están. Vale cuando las
  dos usan el mismo frame, por ejemplo localización sobre el mapa de ese SLAM;
- con ajuste rígido: después de alinear con la mejor rotación más traslación
  (docs/decisiones/003). Vale cuando los frames son distintos.

Que coincidan no prueba que estén bien: los dos métodos usan el mismo modelo de
sensor y pueden equivocarse igual (docs/decisiones/010).
"""
import argparse
import json
from pathlib import Path
import numpy as np


def load(path: Path) -> np.ndarray:
    """Devuelve filas (stamp, x, y)."""
    data = json.loads(path.read_text())
    if "samples" in data:      # replay_localization.py
        return np.array([[r["stamp"], r["x"], r["y"]] for r in data["samples"]])
    if "paths" in data:        # samples.json del SLAM
        return np.array(data["paths"]["slam"])[:, :3]
    raise SystemExit(f"{path}: unknown trajectory format")


def rigid_fit(points: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Mejor rotación más traslación de `points` hacia `reference` (Kabsch)."""
    pc, rc = points.mean(axis=0), reference.mean(axis=0)
    u, _, vt = np.linalg.svd((points - pc).T @ (reference - rc))
    sign = np.sign(np.linalg.det(vt.T @ u.T))
    rotation = vt.T @ np.diag([1.0, sign]) @ u.T
    return (points - pc) @ rotation.T + rc


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("trajectory", type=Path)
    p.add_argument("reference", type=Path)
    args = p.parse_args()
    a, b = load(args.trajectory), load(args.reference)
    # Se compara en los instantes de la primera trayectoria que caen dentro de la segunda.
    inside = (a[:, 0] >= b[0, 0]) & (a[:, 0] <= b[-1, 0])
    if inside.sum() < 3:
        raise SystemExit("The two trajectories do not overlap in time")
    a = a[inside]
    reference = np.column_stack([np.interp(a[:, 0], b[:, 0], b[:, i]) for i in (1, 2)])

    def stats(points):
        e = np.hypot(*(points - reference).T)
        return dict(rmse_cm=round(100 * float(np.sqrt(np.mean(e ** 2))), 1),
                    max_cm=round(100 * float(e.max()), 1))

    print(json.dumps(dict(samples=int(len(a)), direct=stats(a[:, 1:]),
                          rigid_fit=stats(rigid_fit(a[:, 1:], reference)))))


if __name__ == "__main__":
    main()
