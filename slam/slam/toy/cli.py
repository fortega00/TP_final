"""Standalone reproducible validation commands; ROS is deliberately optional."""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path

from ..core.config import SlamConfig
from .world import ToyConfig, run_benchmark


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Entrega A: FastSLAM con grillas de ocupación")
    subcommands = parser.add_subparsers(dest="command", required=True)
    toy = subcommands.add_parser("toy", help="Ejecutar oficina sintética y exportar mapa/métricas")
    toy.add_argument("--output", type=Path, default=Path("artifacts/toy"))
    toy.add_argument("--config", type=Path)
    toy.add_argument("--seed", type=int, default=7)
    toy.add_argument("--particles", type=int)
    toy.add_argument("--resolution", type=float)
    toy.add_argument("--sensor-noise", type=float, default=.015, help="Desvío estándar LiDAR [m]")
    toy.add_argument("--odom-translation-noise", type=float, default=.004, help="Ruido odom [m/sqrt(m)]")
    toy.add_argument("--odom-rotation-noise", type=float, default=.003, help="Ruido odom [rad/sqrt(m+rad)]")
    toy.add_argument("--odom-scale", type=float, default=1.002)
    toy.add_argument("--dropout", type=float, default=.005)
    toy.add_argument("--beams", type=int, default=180)
    toy.add_argument("--scan-step", type=float, default=.28, help="Traslación entre scans [m]")
    toy.add_argument("--max-scans", type=int, help="Limitar ejecución; no equivale a validar recorrido completo")
    toy.add_argument("--check", action="store_true", help="Salir con código 2 si fallan los umbrales del toy")
    args = parser.parse_args(argv)
    config = SlamConfig.load_yaml(args.config) if args.config else SlamConfig()
    config = replace(config, filter=replace(config.filter, seed=args.seed))
    if args.particles is not None:
        config = replace(config, filter=replace(config.filter, particles=args.particles))
    if args.resolution is not None:
        config = replace(config, map=replace(config.map, resolution=args.resolution))
    simulator = ToyConfig(sensor_noise=args.sensor_noise,
                          odom_translation_noise=args.odom_translation_noise,
                          odom_rotation_noise=args.odom_rotation_noise,
                          odom_scale=args.odom_scale, dropout=args.dropout,
                          beams=args.beams, translation_step=args.scan_step)
    metrics = run_benchmark(config, args.output, simulator, args.seed, args.max_scans)
    summary = {key: metrics[key] for key in (
        "toy_acceptance_passed", "loop_completed", "scans", "processed_scans", "particles", "resolution_m",
        "occupied_precision_1cell", "occupied_recall_1cell", "occupied_iou_exact", "position_rmse_m",
        "odometry_position_rmse_m", "final_position_error_m", "route_cells_reachable_fraction",
        "runtime_seconds")}
    summary["output"] = str(args.output.resolve())
    print(json.dumps(summary, indent=2))
    return 2 if args.check and not metrics["toy_acceptance_passed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
