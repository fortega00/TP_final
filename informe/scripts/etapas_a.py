#!/usr/bin/env python3
"""Error del SLAM en las dos etapas que tienen una referencia independiente.

- Café en Gazebo (`cafe_sim.npz`): la referencia es la pose del simulador, grabada
  junto con los scans. SLAM y referencia se expresan respecto de su primera pose.
- Bag del TP Parcial (`bag_parcial.npz`): la referencia es la localización del TP3
  sobre el mapa de la cátedra (`referencia_parcial.npz`). El SLAM vive en otra terna,
  así que se alinea con el mejor ajuste rígido de toda la trayectoria.

Uso, desde la raíz del repo, después de
`experimentos_a.py --only etapa_cafe etapa_parcial`:

    python informe/scripts/etapas_a.py
"""
import json
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / 'informe/datos'
RUNS = DATA / 'experimentos_a'


def relative(poses):
    """Poses (x, y) expresadas en la terna de la primera pose (x, y, yaw)."""
    c, s = np.cos(-poses[0, 2]), np.sin(-poses[0, 2])
    dx, dy = poses[:, 0] - poses[0, 0], poses[:, 1] - poses[0, 1]
    return np.c_[c * dx - s * dy, s * dx + c * dy]


def rigid_fit(points, reference):
    """Lleva `points` sobre `reference` con la mejor rotación y traslación (Kabsch)."""
    pc, rc = points.mean(0), reference.mean(0)
    u, _, vt = np.linalg.svd((points - pc).T @ (reference - rc))
    rotation = vt.T @ np.diag([1.0, np.sign(np.linalg.det(vt.T @ u.T))]) @ u.T
    return (points - pc) @ rotation.T + rc


def rmse_cm(error):
    return 100 * float(np.sqrt((error ** 2).mean()))


def score_cafe(run):
    data = np.load(DATA / 'cafe_sim.npz')
    t = data['metadata'][:, 0]
    slam = np.load(run / 'trajectory.npy')

    def at(series):
        return np.c_[[np.interp(slam[:, 0], t, np.unwrap(series[:, i]) if i == 2 else series[:, i])
                      for i in range(3)]].T

    truth = relative(at(data['truth']))
    odometry = relative(at(data['odometry']))
    estimate = relative(slam[:, 1:4])
    return dict(slam_cm=rmse_cm(np.hypot(*(estimate - truth).T)),
                odometria_cm=rmse_cm(np.hypot(*(odometry - truth).T)))


def score_parcial(run):
    reference = np.load(DATA / 'referencia_parcial.npz')
    t, pf = reference['pf_t'], reference['pf']
    slam = np.load(run / 'trajectory.npy')
    slam = slam[(slam[:, 0] >= t[0]) & (slam[:, 0] <= t[-1])]
    ref_xy = np.c_[np.interp(slam[:, 0], t, pf[:, 0]), np.interp(slam[:, 0], t, pf[:, 1])]
    error = np.hypot(*(rigid_fit(slam[:, 1:3], ref_xy) - ref_xy).T)
    return dict(slam_cm=rmse_cm(error), maximo_cm=100 * float(error.max()))


def main():
    rows = []
    for name, score in (('etapa_cafe', score_cafe), ('etapa_parcial', score_parcial)):
        for run in sorted((RUNS / name).glob('seed_*')):
            if not (run / 'run.json').exists():
                continue
            info = json.loads((run / 'run.json').read_text())
            rows.append(dict(etapa=name, semilla=info['seed'], **score(run),
                             integrados=info['diagnostics']['processed_scans']))
    if not rows:
        raise SystemExit('no hay corridas: correr antes experimentos_a.py --only etapa_cafe etapa_parcial')
    (DATA / 'etapas_a.json').write_text(json.dumps(rows, indent=1) + '\n')
    for row in rows:
        print(' '.join(f'{k}={v:.1f}' if isinstance(v, float) else f'{k}={v}' for k, v in row.items()))


if __name__ == '__main__':
    main()
