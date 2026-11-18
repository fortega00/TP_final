#!/usr/bin/env python3
"""Perfila el núcleo sobre entradas fijas y conserva mapa, trayectoria y tiempos.

El mismo NPZ puede evaluarse con dos revisiones cambiando PYTHONPATH. La pose
real, si existe, se usa únicamente para métricas posteriores y jamás en SLAM.
"""
import argparse
import cProfile
import hashlib
import json
import math
from pathlib import Path
import resource
import time

import numpy as np
import yaml

from slam.core.config import SlamConfig
from slam.core.filter import FastSLAM
from slam.core.geometry import Pose2D
from slam.core.types import Scan


def statistics(values):
    a = np.asarray(values, dtype=float)
    return {"count": len(a), "total": float(a.sum()), "median": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())} if len(a) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int)
    parser.add_argument("--cprofile", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = SlamConfig.load_yaml(args.config)
    args.output.joinpath("config.yaml").write_text(yaml.safe_dump(config.to_dict(), sort_keys=False))
    # NpzFile descomprime de nuevo cada acceso; materializar antes de medir.
    with np.load(args.input) as archive:
        data = {key: archive[key] for key in archive.files}
    for key in ("metadata", "ranges", "odometry", "laser"):
        if key not in data:
            raise ValueError("Missing input array: " + key)
    initialize_started = time.perf_counter()
    model = FastSLAM(config)
    initialize_seconds = time.perf_counter() - initialize_started
    stage_times = {}
    for name in ("_predict", "_weigh", "_update_maps", "_update_estimate", "_resample_if_needed"):
        original = getattr(model, name)
        values = stage_times[name] = []
        def measured(*a, _original=original, _values=values, **kw):
            start = time.perf_counter()
            result = _original(*a, **kw)
            _values.append(time.perf_counter() - start)
            return result
        setattr(model, name, measured)
    mount = Pose2D(*data["laser"])
    odom_path, truth_path, processing_times = [], [], []
    anchor = truth_anchor = None
    cfg = config.filter
    initial = Pose2D(cfg.initial_x, cfg.initial_y, cfg.initial_yaw)
    profiler = cProfile.Profile() if args.cprofile else None
    cpu0 = time.process_time()
    started = time.perf_counter()
    if profiler:
        profiler.enable()
    count = len(data["metadata"]) if args.limit is None else min(args.limit, len(data["metadata"]))
    for index in range(count):
        meta, ranges, odom = data["metadata"][index], data["ranges"][index], data["odometry"][index]
        if not np.isfinite(odom).all():
            raise ValueError("Input contains invalid odometry")
        scan = Scan(ranges, meta[1], meta[2], meta[3], meta[4], meta[0], mount)
        pose = Pose2D(*odom)
        start = time.perf_counter()
        if model.process(scan, pose):
            processing_times.append(time.perf_counter() - start)
            anchor = initial.compose(pose.inverse()) if anchor is None else anchor
            relative = anchor.compose(pose)
            odom_path.append([meta[0], relative.x, relative.y, relative.yaw])
            if "truth" in data:
                truth = Pose2D(*data["truth"][index])
                truth_anchor = initial.compose(truth.inverse()) if truth_anchor is None else truth_anchor
                aligned = truth_anchor.compose(truth)
                truth_path.append([meta[0], aligned.x, aligned.y, aligned.yaw])
        if index % 200 == 0:
            print(json.dumps({"input_scan": index, "processed": model.diagnostics["processed_scans"],
                              "wall_s": round(time.perf_counter() - started, 2)}), flush=True)
    wall = time.perf_counter() - started
    cpu = time.process_time() - cpu0
    if profiler:
        profiler.disable()
        profiler.dump_stats(str(args.output / "profile.pstats"))
    rows = [[stamp, pose.x, pose.y, pose.yaw] for stamp, pose in model.best_particle.trajectory]
    paths = {"slam": rows, "odom": odom_path}
    if truth_path:
        paths["truth"] = truth_path
    model.save_map(args.output / "map")
    np.savez_compressed(args.output / "grid.npz", observed=model.best_particle.grid.observed,
                        log_odds=model.best_particle.grid.log_odds)
    metadata = {"scan_stamp": model.last_stamp, "diagnostics": model.diagnostics}
    (args.output / "map.json").write_text(json.dumps(metadata, indent=2) + "\n")
    samples = {"mode": "simulation" if truth_path else "bag", "success": True,
               "configuration": config.to_dict(), "paths": paths,
               "final_status": model.diagnostics, "wall_seconds": wall, "status_history": [],
               "evaluation_kind": "Fixed-input core replay; not live ROS throughput"}
    (args.output / "samples.json").write_text(json.dumps(samples, indent=2) + "\n")
    # Copiar para que la evidencia se pueda mover sin enlaces a carpetas temporales.
    (args.output / "scans.npz").write_bytes(args.input.read_bytes())
    report = {"input": str(args.input), "input_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
              "initialize_and_jit_warmup_seconds": initialize_seconds,
              "proposal_backend": getattr(model.proposal, "acceleration_backend", "baseline_numpy"),
              "input_scans": count, "wall_seconds": wall, "cpu_seconds": cpu,
              "cpu_fraction": cpu / wall, "max_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
              "integrated_update_seconds": statistics(processing_times),
              "stages_seconds": {k: statistics(v) for k, v in stage_times.items()},
              "diagnostics": model.diagnostics, "cprofile_enabled": bool(profiler),
              "map_sha256": hashlib.sha256((args.output / "map.pgm").read_bytes()).hexdigest()}
    (args.output / "timing.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report), flush=True)


if __name__ == "__main__":
    main()
