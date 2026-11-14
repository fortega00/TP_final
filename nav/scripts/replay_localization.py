#!/usr/bin/env python3
"""Corre la localización, sin ROS, sobre scans y odometría grabados.

Lee un .npz con `metadata` (stamp, angle_min, angle_increment, range_min,
range_max), `ranges`, `odometry`, `laser` (extrínseca) y, si la grabación es de
simulación, `truth` (pose real en el frame del mapa). Sirve para comparar
configuraciones sobre exactamente las mismas lecturas. No prueba el nodo ROS.
"""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import yaml

from slam.core.geometry import Pose2D
from slam.core.types import Scan
from nav.core.config import NavConfig
from nav.core.localization import MonteCarloLocalization
from nav.core.map import load_map


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--map", type=Path, required=True)
    p.add_argument("--config", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True, help="JSON de resultados")
    p.add_argument("--seed", type=int)
    p.add_argument("--first", type=int, default=0, help="índice del primer scan a usar")
    p.add_argument("--last", type=int, help="índice siguiente al último scan a usar")
    p.add_argument("--set", action="append", default=[], metavar="GRUPO.CAMPO=VALOR",
                   help="pisa un valor de la configuración, por ejemplo scan_match.enabled=true")
    p.add_argument("--initial-pose", type=float, nargs=3, metavar=("X", "Y", "YAW"),
                   help="pose del click inicial; por defecto la pose real del primer scan")
    p.add_argument("--initial-offset", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                   metavar=("DX", "DY", "DYAW"), help="error del click inicial respecto de esa pose")
    # Desvíos que manda la herramienta 2D Pose Estimate de RViz: 0,5 m y 15 grados.
    p.add_argument("--initial-std-xy", type=float, default=0.5)
    p.add_argument("--initial-std-yaw", type=float, default=math.pi / 12)
    args = p.parse_args()

    raw = yaml.safe_load(args.config.read_text())
    for item in args.set:
        key, value = item.split("=", 1)
        group, name = key.split(".")
        raw.setdefault(group, {})[name] = yaml.safe_load(value)
    if args.seed is not None:
        raw.setdefault("localization", {})["seed"] = args.seed
    config = NavConfig.from_dict(raw)
    grid = load_map(args.map, config.sensor)
    data = np.load(args.input)
    has_truth = "truth" in data.files
    mount = Pose2D(*data["laser"])

    def scan_at(i):
        m = data["metadata"][i]
        return Scan(data["ranges"][i], m[1], m[2], m[3], m[4], m[0], mount)

    if args.initial_pose is None and not has_truth:
        raise SystemExit("--initial-pose is required when the recording has no true pose")
    last = len(data["ranges"]) if args.last is None else args.last
    base = args.initial_pose if args.initial_pose is not None else data["truth"][args.first]
    click = Pose2D(*(np.asarray(base) + args.initial_offset))

    mcl = MonteCarloLocalization(grid, config)
    started = time.perf_counter()
    mcl.initialize(click, args.initial_std_xy, args.initial_std_yaw)
    burst_converged = mcl.converge(scan_at(args.first))
    burst_seconds = time.perf_counter() - started
    rows, update_seconds = [], []

    def record(i):
        pose, _ = mcl.estimate()
        row = dict(stamp=float(data["metadata"][i][0]), x=pose.x, y=pose.y, yaw=pose.yaw,
                   std_xy=mcl.std_xy, std_yaw=mcl.std_yaw, residual=mcl.scan_residual(scan_at(i)))
        if has_truth:
            tx, ty, tyaw = data["truth"][i]
            row.update(error_xy=math.hypot(pose.x - tx, pose.y - ty),
                       error_yaw=abs(math.atan2(math.sin(pose.yaw - tyaw), math.cos(pose.yaw - tyaw))))
        rows.append(row)

    record(args.first)
    for i in range(args.first, last):
        if mcl.process(scan_at(i), Pose2D(*data["odometry"][i])):
            update_seconds.append(mcl.diagnostics["update_seconds"])
            record(i)

    summary = dict(burst_converged=burst_converged,
                   burst_iterations=mcl.diagnostics["burst_iterations"],
                   burst_seconds=round(burst_seconds, 4),
                   updates=len(update_seconds),
                   seconds_per_update=round(float(np.median(update_seconds)), 4),
                   residual_mean_cm=round(100 * float(np.mean([r["residual"] for r in rows])), 2),
                   resamples=mcl.diagnostics["resamples"],
                   injected_particles=mcl.diagnostics["injected_particles"])
    if has_truth:
        e = np.array([r["error_xy"] for r in rows])
        yaw = np.degrees([r["error_yaw"] for r in rows])
        summary.update(click_error_cm=round(100 * math.hypot(*args.initial_offset[:2]), 1),
                       after_burst_cm=round(100 * float(e[0]), 1),
                       rmse_cm=round(100 * float(np.sqrt(np.mean(e ** 2))), 1),
                       max_cm=round(100 * float(e.max()), 1),
                       final_cm=round(100 * float(e[-1]), 1),
                       yaw_rmse_deg=round(float(np.sqrt(np.mean(yaw ** 2))), 2))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(dict(summary=summary, configuration=raw, samples=rows)) + "\n")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
