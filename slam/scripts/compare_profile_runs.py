#!/usr/bin/env python3
"""Compara evidencia conservada, sin volver a ejecutar los estimadores."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def sample_truth(data, stamps):
    return np.column_stack([np.interp(stamps, data['metadata'][:, 0],
                                    np.unwrap(data['truth'][:, i]) if i == 2 else data['truth'][:, i])
                            for i in range(3)])


def world_estimates(samples, data, common):
    path = np.asarray(samples['paths']['slam'])
    initial = samples['configuration']['filter']
    anchor = sample_truth(data, [path[0, 0]])[0]
    target = sample_truth(data, common)
    pose = np.column_stack([np.interp(common, path[:, 0], np.unwrap(path[:, i]) if i == 3 else path[:, i])
                            for i in (1, 2, 3)])
    angle = anchor[2]-initial['initial_yaw']
    c, s = np.cos(angle), np.sin(angle)
    relative = pose[:, :2]-[initial['initial_x'], initial['initial_y']]
    pose[:, 0] = anchor[0]+c*relative[:, 0]-s*relative[:, 1]
    pose[:, 1] = anchor[1]+s*relative[:, 0]+c*relative[:, 1]
    pose[:, 2] += angle
    distance = np.linalg.norm(pose[:, :2]-target[:, :2], axis=1)
    yaw = np.arctan2(np.sin(pose[:, 2]-target[:, 2]), np.cos(pose[:, 2]-target[:, 2]))
    return {'position_rmse_m': float(np.sqrt(np.mean(distance**2))),
            'position_p95_m': float(np.percentile(distance, 95)),
            'yaw_rmse_deg': float(np.degrees(np.sqrt(np.mean(yaw**2)))),
            'initial_truth_anchor': anchor.tolist(),
            'alignment': 'Known initial GT pose only; no fitted rigid transform'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--candidate', type=Path, required=True)
    parser.add_argument('--input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    runs = {key: json.loads((path/'samples.json').read_text())
            for key, path in [('baseline', args.baseline), ('candidate', args.candidate)]}
    report = {'baseline': str(args.baseline), 'candidate': str(args.candidate),
              'identical_configuration': runs['baseline']['configuration'] == runs['candidate']['configuration'],
              'map_pgm_equal': (args.baseline/'map.pgm').read_bytes() == (args.candidate/'map.pgm').read_bytes(),
              'map_sha256': {key: hashlib.sha256((path/'map.pgm').read_bytes()).hexdigest()
                             for key, path in [('baseline', args.baseline), ('candidate', args.candidate)]},
              'diagnostics': {key: samples['final_status'] for key, samples in runs.items()}}
    if all((path/'grid.npz').exists() for path in [args.baseline, args.candidate]):
        with np.load(args.baseline/'grid.npz') as left, np.load(args.candidate/'grid.npz') as right:
            report['raw_grid_equal'] = {key: bool(np.array_equal(left[key], right[key]))
                                        for key in ('observed', 'log_odds')}
            report['raw_grid_bytes_equal'] = {key: bool(left[key].dtype == right[key].dtype and
                                                       left[key].tobytes() == right[key].tobytes())
                                              for key in ('observed', 'log_odds')}
        report['ancestral_trajectory_equal'] = runs['baseline']['paths']['slam'] == runs['candidate']['paths']['slam']
        report['ancestral_trajectory_bytes_equal'] = (
            np.asarray(runs['baseline']['paths']['slam'], dtype=np.float64).tobytes() ==
            np.asarray(runs['candidate']['paths']['slam'], dtype=np.float64).tobytes())
    report['timing'] = {}
    for key, path in [('baseline', args.baseline), ('candidate', args.candidate)]:
        if (path/'timing.json').exists():
            report['timing'][key] = json.loads((path/'timing.json').read_text())
    if args.input:
        with np.load(args.input) as archive:
            data = {key: archive[key] for key in archive.files}
        report['input_sha256'] = hashlib.sha256(args.input.read_bytes()).hexdigest()
        if 'truth' in data:
            paths = [np.asarray(samples['paths']['slam']) for samples in runs.values()]
            common = data['metadata'][:, 0]
            common = common[(common >= max(p[0, 0] for p in paths)) &
                            (common <= min(p[-1, 0] for p in paths))]
            report['common_truth_samples'] = len(common)
            report['quality_on_same_truth_timestamps'] = {
                key: world_estimates(samples, data, common) for key, samples in runs.items()}
    for key, path in [('baseline', args.baseline), ('candidate', args.candidate)]:
        if (path/'metrics.json').exists():
            measured = json.loads((path/'metrics.json').read_text())
            report.setdefault('map_quality_approximate', {})[key] = {
                'geometry': measured.get('approximate_sdf_reference'),
                'connectivity': measured.get('navigation_check')}
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
