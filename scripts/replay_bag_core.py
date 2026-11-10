#!/usr/bin/env python3
"""Screen the unchanged FastSLAM core on aligned real-bag measurements.

This is parameter screening, not ROS integration or an independent accuracy test.
"""
import argparse
import json
import math
import os
from pathlib import Path
import time
import numpy as np
import yaml
from tp_slam.config import SlamConfig
from tp_slam.filter import FastSLAM
from tp_slam.geometry import Pose2D
from tp_slam.types import Scan


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input', type=Path, required=True)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--particles', type=int)
    p.add_argument('--alpha', type=float)
    p.add_argument('--sigma', type=float)
    args = p.parse_args()
    raw = yaml.safe_load(args.config.read_text())
    if args.particles is not None:
        raw['filter']['particles'] = args.particles
    if args.alpha is not None:
        raw['motion'] = dict(alpha1=args.alpha, alpha2=args.alpha/10,
                             alpha3=args.alpha, alpha4=args.alpha)
    if args.sigma is not None:
        raw['sensor']['sigma_hit'] = args.sigma
    config = SlamConfig.from_dict(raw)
    model = FastSLAM(config)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / 'config.yaml').write_text(yaml.safe_dump(raw, sort_keys=False))
    data = np.load(args.input)
    odom_path = []
    anchor = None
    started = time.monotonic()
    for i, (meta, ranges, odom) in enumerate(zip(data['metadata'], data['ranges'], data['odometry'])):
        if not np.isfinite(odom).all():
            continue
        scan = Scan(ranges, meta[1], meta[2], meta[3], meta[4], meta[0],
                    Pose2D(.0435, 5.26e-5, math.pi))
        pose = Pose2D(*odom)
        if model.process(scan, pose):
            anchor = pose.inverse() if anchor is None else anchor
            relative = anchor.compose(pose)
            odom_path.append([meta[0], relative.x, relative.y, relative.yaw])
        if i % 100 == 0:
            best = model.best_particle.pose
            print(json.dumps({'scan': i, 'wall_s': round(time.monotonic()-started, 1),
                              'pose': [best.x, best.y, best.yaw]}), flush=True)
    model.save_map(args.output / 'map')
    rows = [[t, pose.x, pose.y, pose.yaw] for t, pose in model.best_particle.trajectory]
    samples = dict(mode='bag', success=True, configuration=raw,
                   paths=dict(slam=rows, odom=odom_path), final_status=model.diagnostics,
                   wall_seconds=time.monotonic()-started, status_history=[],
                   evaluation_kind='offline parameter screening; ROS validation still required')
    (args.output / 'samples.json').write_text(json.dumps(samples, indent=2)+'\n')
    (args.output / 'scans.npz').symlink_to(os.path.relpath(args.input.resolve(), args.output.resolve()))
    print('FINAL', json.dumps({'seconds': samples['wall_seconds'], 'end_to_start_m':
                              math.hypot(rows[-1][1], rows[-1][2]),
                              'diagnostics': model.diagnostics}), flush=True)


if __name__ == '__main__':
    main()
