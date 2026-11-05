#!/usr/bin/env python3
"""Run a small explicit parameter/measurement ablation and retain every result."""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from tp_slam.config import SlamConfig
from tp_slam.toy import ToyConfig, run_benchmark


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/tuning"))
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--config", type=Path)
    args = parser.parse_args()
    base = SlamConfig.load_yaml(args.config) if args.config else SlamConfig()
    base = replace(base, filter=replace(base.filter, seed=args.seed))
    profiles = [
        ("baseline", base, ToyConfig()),
        ("coarse_10cm", replace(base, map=replace(base.map, resolution=.10)), ToyConfig()),
        ("noisy_lidar", base, ToyConfig(sensor_noise=.06)),
        ("noisy_odometry", base, ToyConfig(odom_translation_noise=.02, odom_rotation_noise=.015, odom_scale=1.01)),
    ]
    results = []
    for name, slam, simulator in profiles:
        print(f"Running {name}", flush=True)
        result = run_benchmark(slam, args.output / name, simulator, args.seed)
        result["profile"] = name
        results.append(result)
        print(json.dumps({key: result[key] for key in (
            "profile", "occupied_precision_1cell", "occupied_recall_1cell",
            "position_rmse_m", "route_cells_reachable_fraction", "runtime_seconds")}), flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "comparison.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
    fields = ["profile", "resolution_m", "occupied_precision_1cell", "occupied_recall_1cell", "occupied_iou_exact",
              "wall_distance_p95_m", "position_rmse_m", "route_cells_reachable_fraction", "runtime_seconds"]
    lines = ["| " + " | ".join(fields) + " |", "| " + " | ".join("---" for _ in fields) + " |"]
    for result in results:
        lines.append("| " + " | ".join(f"{result[key]:.4f}" if isinstance(result[key], float) else str(result[key]) for key in fields) + " |")
    (args.output / "comparison.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
