#!/usr/bin/env python3
"""Corre el núcleo de SLAM sobre los bags del laboratorio con distintas configuraciones.

Cada experimento cambia una sola cosa respecto de la configuración base (la que
generó el mapa elegido, `sesiones/20261116/mapa/intento3_corregido`) y se repite con
varias semillas. No usa ROS. Uso, desde la raíz del repo:

    PYTHONPATH=slam python informe/scripts/experimentos_a.py [--workers 6] [--only nombre ...]

Deja en `informe/datos/experimentos_a/<experimento>/seed_<n>/` el mapa, la
trayectoria y un `run.json` con el diagnóstico. Una corrida que ya existe no se repite.
"""
import argparse
import copy
import json
import math
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / 'informe/datos'
OUTPUT = DATA / 'experimentos_a'
BASE_CONFIG = REPO / 'sesiones/20261116/mapa/intento3_corregido/config.yaml'
SEEDS = (1, 2, 3, 4, 5, 6)

# nombre: (bag, un scan de cada cuántos, cambios sobre la configuración base)
EXPERIMENTS = {
    'base': ('bag_mapa2', 1, {}),
    # partículas
    'particulas_4': ('bag_mapa2', 1, {'filter.particles': 4}),
    'particulas_16': ('bag_mapa2', 1, {'filter.particles': 16}),
    'particulas_32': ('bag_mapa2', 1, {'filter.particles': 32}),
    'e16': ('bag_mapa2', 1, {'filter.particles': 16, 'scan_match.sample_xy': 0.005}),
    # propuesta: sin scan matching, con el ruido de odometría bajo y con el alto
    'odometria_8': ('bag_mapa2', 1, {'scan_match.enabled': False}),
    'odometria_128': ('bag_mapa2', 1, {'scan_match.enabled': False, 'filter.particles': 128,
                                       'motion.alpha1': 0.1, 'motion.alpha2': 0.01,
                                       'motion.alpha3': 0.1, 'motion.alpha4': 0.1}),
    # scan matching
    'match_sigma_003': ('bag_mapa2', 1, {'scan_match.sigma': 0.03}),
    'match_sigma_010': ('bag_mapa2', 1, {'scan_match.sigma': 0.10}),
    'match_yaw_003': ('bag_mapa2', 1, {'scan_match.window_yaw': 0.03}),
    'match_yaw_012': ('bag_mapa2', 1, {'scan_match.window_yaw': 0.12}),
    # sensor y mapa
    'sigma_hit_005': ('bag_mapa2', 1, {'sensor.sigma_hit': 0.05}),
    'sigma_hit_030': ('bag_mapa2', 1, {'sensor.sigma_hit': 0.30}),
    'grosor_003': ('bag_mapa2', 1, {'sensor.hit_thickness': 0.03}),
    'sin_retorno_libre': ('bag_mapa2', 1, {'sensor.no_return_is_free': True}),
    # compuerta de movimiento
    'paso_004': ('bag_mapa2', 1, {'update.min_translation': 0.04, 'update.min_rotation': 0.03}),
    'paso_016': ('bag_mapa2', 1, {'update.min_translation': 0.16, 'update.min_rotation': 0.12}),
    # tamaño de celda
    'celda_025': ('bag_mapa2', 1, {'map.resolution': 0.025}),
    'celda_100': ('bag_mapa2', 1, {'map.resolution': 0.10}),
    # menos scans por metro recorrido, como si el robot fuera más rápido
    'scans_1_de_2': ('bag_mapa2', 2, {}),
    'scans_1_de_3': ('bag_mapa2', 3, {}),
    'scans_1_de_4': ('bag_mapa2', 4, {}),
    # el otro recorrido
    'recorrido_1': ('bag_mapa', 1, {}),
    'recorrido_1_e16': ('bag_mapa', 1, {'filter.particles': 16, 'scan_match.sample_xy': 0.005}),
    # las otras dos etapas del workflow, con la misma configuración (las puntúa etapas_a.py)
    'etapa_cafe': ('cafe_sim', 1, {}),
    'etapa_parcial': ('bag_parcial', 1, {}),
}


def build_config(changes, seed):
    raw = yaml.safe_load(BASE_CONFIG.read_text())
    raw['filter']['seed'] = seed
    for key, value in changes.items():
        group, name = key.split('.')
        if name not in raw[group]:
            raise KeyError(f'{key} no existe en la configuración base')
        raw[group][name] = value
    return raw


def run(job):
    # imports acá para que cada proceso cargue el núcleo por su cuenta
    from slam.core.config import SlamConfig
    from slam.core.filter import FastSLAM
    from slam.core.geometry import Pose2D
    from slam.core.types import Scan

    name, seed, output = job
    bag, every, changes = EXPERIMENTS[name]
    out = output / name / f'seed_{seed}'
    if (out / 'run.json').exists():
        return name, seed, 'ya estaba'
    out.mkdir(parents=True, exist_ok=True)
    raw = build_config(changes, seed)
    model = FastSLAM(SlamConfig.from_dict(raw))
    data = np.load(DATA / f'{bag}.npz')
    laser = Pose2D(*data['laser'])
    update_seconds = []
    started = time.monotonic()
    for meta, ranges, odom in zip(data['metadata'][::every], data['ranges'][::every],
                                  data['odometry'][::every]):
        if not np.isfinite(odom).all():
            continue
        scan = Scan(ranges, meta[1], meta[2], meta[3], meta[4], meta[0], laser)
        before = time.monotonic()
        if model.process(scan, Pose2D(*odom)):
            update_seconds.append(time.monotonic() - before)
    wall = time.monotonic() - started
    model.save_map(out / 'map')
    trajectory = np.array([[t, p.x, p.y, p.yaw] for t, p in model.best_particle.trajectory])
    np.save(out / 'trajectory.npy', trajectory)
    (out / 'config.yaml').write_text(yaml.safe_dump(raw, sort_keys=False))
    end = trajectory[-1]
    summary = dict(experiment=name, seed=seed, bag=bag, every=every, changes=changes,
                   wall_seconds=wall, update_median_s=float(np.median(update_seconds)),
                   update_p95_s=float(np.percentile(update_seconds, 95)),
                   end_to_start_m=math.hypot(end[1], end[2]), end_yaw_rad=float(end[3]),
                   diagnostics=copy.deepcopy(model.diagnostics))
    (out / 'run.json').write_text(json.dumps(summary, indent=2, default=float) + '\n')
    return name, seed, f'{wall:.0f} s'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers', type=int, default=6)
    parser.add_argument('--only', nargs='+', choices=sorted(EXPERIMENTS))
    parser.add_argument('--seeds', nargs='+', type=int, default=list(SEEDS))
    parser.add_argument('--output', type=Path, default=OUTPUT,
                        help='otra carpeta de salida, por ejemplo para medir tiempos con --workers 1')
    args = parser.parse_args()
    jobs = [(name, seed, args.output.resolve()) for name in (args.only or EXPERIMENTS)
            for seed in args.seeds]
    with ProcessPoolExecutor(args.workers) as pool:
        for name, seed, note in pool.map(run, jobs):
            print(f'{name} seed {seed}: {note}', flush=True)


if __name__ == '__main__':
    main()
