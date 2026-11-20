"""Métricas de evaluación independientes del núcleo del estimador y de ROS."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.ndimage import binary_dilation
from scipy.spatial import cKDTree


def metric_disk(radius, resolution):
    """Disco en metros: el ceil limita el arreglo, no aumenta el radio solicitado."""
    if radius < 0 or resolution <= 0:
        raise ValueError('Invalid inflation radius or resolution')
    n = int(math.ceil(radius / resolution))
    yy, xx = np.mgrid[-n:n+1, -n:n+1]
    return (xx*resolution)**2 + (yy*resolution)**2 <= radius**2 + 1e-12


def navigable_cells(free, resolution, radius):
    """Infla ocupados, desconocido y exterior; solo permite huella en espacio libre."""
    return ~binary_dilation(~free, structure=metric_disk(radius, resolution), border_value=1)


def laser_endpoints(pose, mount, ranges, angles):
    """Transforma extremos usando la extrínseca base-láser de la entrada."""
    c, s = np.cos(pose[2]), np.sin(pose[2])
    x = pose[0] + c*mount[0] - s*mount[1]
    y = pose[1] + s*mount[0] + c*mount[1]
    a = pose[2] + mount[2] + angles
    return np.column_stack((x + ranges*np.cos(a), y + ranges*np.sin(a)))


def angular_error(angle, target):
    """Diferencia angular mínima, incluida la discontinuidad de ±pi."""
    return np.arctan2(np.sin(angle-target), np.cos(angle-target))


def endpoint_consistency(samples, occupied, metadata, source):
    """Residuo sin truncar a centros de obstáculos; usa 30 haces fijos por scan."""
    with np.load(source) as archive:
        scan_meta, readings, mount = archive['metadata'], archive['ranges'], archive['laser']
    path = np.asarray(samples['paths']['slam'])
    points = []
    sensor = samples['configuration']['sensor']
    for row in path:
        index = int(np.searchsorted(scan_meta[:, 0], row[0]))
        if index == len(scan_meta) or abs(scan_meta[index, 0]-row[0]) > 1e-6:
            raise ValueError('Trajectory timestamp is missing from sensor input')
        meta, ranges = scan_meta[index], readings[index]
        valid = np.flatnonzero(np.isfinite(ranges) & (ranges >= max(meta[3], sensor['min_range'])) &
                               (ranges < min(meta[4], sensor['max_range'])))
        if not len(valid):
            continue
        inds = valid[np.linspace(0, len(valid)-1, min(30, len(valid)), dtype=int)]
        points.append(laser_endpoints(row[1:], mount, ranges[inds], meta[1]+inds*meta[2]))
    rows, cols = np.nonzero(occupied)
    if not len(cols) or not points:
        raise ValueError('Endpoint evaluation requires obstacles and valid hits')
    centers = np.column_stack((cols+.5, rows+.5))*metadata['resolution'] + metadata['origin'][:2]
    distances = cKDTree(centers).query(np.concatenate(points))[0]
    return {'metric_version': 2, 'label': 'in_sample_endpoint_to_occupied_cell_center',
            'samples': len(distances), 'beams_per_scan_max': 30,
            'distance_cap_m': None, 'laser_extrinsic': mount.tolist(),
            'distance_m': {'median': float(np.median(distances)),
                           'p95': float(np.percentile(distances, 95)),
                           'mean': float(distances.mean()), 'max': float(distances.max())},
            'fraction_within_10cm': float(np.mean(distances <= .1)),
            'warning': 'In-sample consistency, not independent map or pose accuracy.'}


def aligned_truth(samples, data, stamps):
    """Expresa GT en el frame inicial configurado, sin ajuste a posteriori."""
    times = data['metadata'][:, 0]
    first = samples['paths']['slam'][0][0]
    anchor = np.array([np.interp(first, times, np.unwrap(data['truth'][:, k]) if k == 2
                                 else data['truth'][:, k]) for k in range(3)])
    truth = np.column_stack([np.interp(stamps, times, np.unwrap(data['truth'][:, k]) if k == 2
                                       else data['truth'][:, k]) for k in range(3)])
    cfg = samples['configuration']['filter']
    angle = cfg['initial_yaw']-anchor[2]
    c, s = np.cos(angle), np.sin(angle)
    delta = truth[:, :2]-anchor[:2]
    truth[:, 0] = cfg['initial_x']+c*delta[:, 0]-s*delta[:, 1]
    truth[:, 1] = cfg['initial_y']+s*delta[:, 0]+c*delta[:, 1]
    truth[:, 2] += angle
    return truth


def compare_cycle(args):
    """Analiza un ciclo ya ejecutado y prepara evidencia para revisión humana."""
    import csv
    import os
    import subprocess
    import sys
    from PIL import Image
    import yaml
    from compare_profile_runs import world_estimates
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/slam_hitl_mpl')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    folders = [args.output, *args.compare]
    report_dir = args.report_dir
    report_dir.mkdir(parents=True, exist_ok=False)
    with np.load(args.input_npz) as archive:
        data = {key: archive[key] for key in archive.files}
    if 'truth' not in data:
        raise ValueError('This Cafe comparison requires ground truth')
    inputs_sha = hashlib.sha256(args.input_npz.read_bytes()).hexdigest()
    runs = {p.name: json.loads((p/'samples.json').read_text()) for p in folders}
    paths = [np.asarray(run['paths']['slam']) for run in runs.values()]
    common = data['metadata'][:, 0]
    common = common[(common >= max(p[0, 0] for p in paths)) &
                    (common <= min(p[-1, 0] for p in paths))]
    if not len(common):
        raise ValueError('Trajectories do not share ground truth timestamps')
    if not all(np.array_equal(paths[0][:, 0], path[:, 0]) for path in paths):
        raise ValueError('Endpoint comparison needs the same integrated scan timestamps')

    table, plot_data, records = [], [], []
    for folder in folders:
        command = [sys.executable, str(Path(__file__).with_name('analyze_a_experiment.py')),
                   str(folder), '--input-npz', str(args.input_npz),
                   '--inflation-radius', str(args.inflation_radius)]
        if args.cafe_package:
            command += ['--cafe-package', str(args.cafe_package)]
        result = subprocess.run(command, capture_output=True, text=True)
        (report_dir/(folder.name+'_analysis.log')).write_text(result.stdout+result.stderr)
        if result.returncode:
            raise RuntimeError('Map analysis failed for '+str(folder))
        measured = json.loads((folder/'metrics.json').read_text())
        timing = json.loads((folder/'timing.json').read_text())
        if timing['input_sha256'] != inputs_sha:
            raise ValueError('Experiment input hash mismatch')
        with np.load(folder/'updates.npz') as trace:
            values = trace['values']
        samples = runs[folder.name]
        config = samples['configuration']
        pose = world_estimates(samples, data, common)
        endpoints = measured['in_sample_consistency']
        navigation = measured['navigation_check']
        geometry = measured.get('approximate_sdf_reference', {})
        times = values[:, 1]
        row = {'case': folder.name, 'particles': config['filter']['particles'],
               'prior_xy': config['scan_match']['prior_xy'],
               'sample_xy': config['scan_match']['sample_xy'],
               'backend': timing['proposal_backend'],
               'rmse_cm': 100*pose['position_rmse_m'], 'yaw_rmse_deg': pose['yaw_rmse_deg'],
               'endpoints_10cm_percent': 100*endpoints['fraction_within_10cm'],
               'endpoint_median_cm': 100*endpoints['distance_m']['median'],
               'endpoint_p95_cm': 100*endpoints['distance_m']['p95'],
               'connected_percent': 100*navigation['connected_visited_fraction'],
               'occupied_precision_percent': 100*geometry.get('occupied_precision', float('nan')),
               'wall_recall_percent': 100*geometry.get('reachable_wall_surface_recall', float('nan')),
               'observed_coverage_percent': 100*geometry.get('reachable_observed_coverage', float('nan')),
               'median_ms': 1000*float(np.median(times)),
               'p95_ms': 1000*float(np.percentile(times, 95)),
               'max_ms': 1000*float(times.max()),
               'over_90ms_percent': 100*float(np.mean(times > .09)),
               'over_137ms_percent': 100*float(np.mean(times > .137)),
               'ess_ratio_mean': float(values[:, 2].mean()),
               'ess_ratio_min': float(values[:, 2].min()),
               'resamples': int(samples['final_status']['resamples']),
               'integrations': len(times), 'replay_seconds': timing['wall_seconds'],
               'jit_and_init_seconds': timing['initialize_and_jit_warmup_seconds'],
               'peak_rss_mib': timing['max_rss_kib']/1024}
        table.append(row)
        records.append({'id': folder.name, 'configuration': config, 'input_sha256': inputs_sha,
                        'seed': config['filter']['seed'], 'metrics': row,
                        'human_observations': [], 'decision': 'investigar',
                        'decision_reason': 'Pending human visual evaluation',
                        'ancestral_trajectory_evaluation': pose,
                        'timing_scope': 'Full FastSLAM.process; excludes ROS and JIT initialization'})
        meta = yaml.safe_load((folder/'map.yaml').read_text())
        pixels = np.flipud(np.asarray(Image.open(folder/meta['image'])))
        extent = [meta['origin'][0], meta['origin'][0]+pixels.shape[1]*meta['resolution'],
                  meta['origin'][1], meta['origin'][1]+pixels.shape[0]*meta['resolution']]
        plot_data.append((folder.name, pixels, extent, samples))

    equality = {}
    for name in ('observed', 'log_odds'):
        with np.load(folders[0]/'grid.npz') as a, np.load(folders[1]/'grid.npz') as b:
            equality[name] = bool(a[name].dtype == b[name].dtype and a[name].tobytes() == b[name].tobytes())
    equality['ancestral_trajectory'] = runs[folders[0].name]['paths']['slam'] == runs[folders[1].name]['paths']['slam']
    equality['pgm'] = (folders[0]/'map.pgm').read_bytes() == (folders[1]/'map.pgm').read_bytes()
    report = {'schema_version': 2, 'input_sha256': inputs_sha,
              'common_truth_samples': len(common), 'same_integrated_timestamps': True,
              'inflation_radius_m': args.inflation_radius,
              'baseline_vs_optimized8_equal': equality, 'experiments': records,
              'limits': ['Single seed 7; not a robustness study',
                         'Core replay only; ROS queue, drops and full callback not measured',
                         'SDF boxes approximate visual meshes; endpoints are in-sample',
                         'Ancestral trajectory selected at end, interpolated on common timestamps'],
              'next_step': 'STOP: human map review; no next cycle or merge authorized automatically'}
    (report_dir/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    with (report_dir/'comparison.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(table[0]))
        writer.writeheader()
        writer.writerows(table)

    def draw(ax, item, show_path=False):
        name, pixels, extent, samples = item
        ax.imshow(pixels, origin='lower', extent=extent, cmap='gray', vmin=0, vmax=255,
                  interpolation='nearest')
        if show_path:
            path = np.asarray(samples['paths']['slam'])
            truth = aligned_truth(samples, data, common)
            ax.plot(truth[:, 0], truth[:, 1], color='#168b30', linewidth=.6, label='GT')
            ax.plot(path[:, 1], path[:, 2], color='#007b9c', linewidth=.6, label='SLAM ancestral')
        c = samples['configuration']
        ax.set_title(f"{name}: N={c['filter']['particles']}\nprior={c['scan_match']['prior_xy']}, sample={c['scan_match']['sample_xy']}", fontsize=10)
        ax.set(xlabel='x [m]', ylabel='y [m]', aspect='equal')

    for show_paths, filename in [(False, 'maps.png'), (True, 'maps_and_paths.png')]:
        fig, axes = plt.subplots(1, len(plot_data), figsize=(17, 10), sharex=True, sharey=True)
        for ax, item in zip(axes, plot_data):
            draw(ax, item, show_paths)
            ax.set(xlim=(-6, 5), ylim=(-14, 13))
        if show_paths:
            axes[0].legend(loc='lower right', fontsize=7)
        fig.tight_layout()
        fig.savefig(report_dir/filename, dpi=160)
        plt.close(fig)
    regions = [('Pared derecha', (3.2, 5.), (-10, 0)),
               ('Sector superior', (-5.8, 4.8), (7, 13)),
               ('Puerta inferior', (-5, -2), (-12, -10))]
    fig, axes = plt.subplots(3, len(plot_data), figsize=(18, 12))
    for j, (region, xlim, ylim) in enumerate(regions):
        for i, item in enumerate(plot_data):
            draw(axes[j, i], item)
            axes[j, i].set(xlim=xlim, ylim=ylim, aspect='equal')
            if i == 0:
                axes[j, i].set_ylabel(region+'\ny [m]')
    fig.tight_layout()
    fig.savefig(report_dir/'details.png', dpi=160)
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for folder in folders:
        with np.load(folder/'updates.npz') as trace:
            values = trace['values']
        elapsed = values[:, 0]-values[0, 0]
        axes[0].plot(elapsed, 1000*values[:, 1], linewidth=.6, alpha=.7, label=folder.name)
        axes[1].plot(elapsed, values[:, 2], linewidth=.6, alpha=.6, label=folder.name)
    for limit in (90, 137):
        axes[0].axhline(limit, color='black', linestyle='--', linewidth=.7)
    axes[1].axhline(.5, color='black', linestyle='--', linewidth=.7)
    axes[0].set(ylabel='FastSLAM.process [ms]', title='Costo del nucleo y ESS; sin ROS ni calentamiento inicial')
    axes[1].set(ylabel='ESS / N', xlabel='Tiempo del sensor [s]')
    for ax in axes:
        ax.legend(ncol=len(folders))
        ax.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(report_dir/'timing_and_ess.png', dpi=160)
    plt.close(fig)
    lines = ['# Café HITL 01 — revisión humana pendiente', '',
             '| Caso | N | RMSE cm | Endpoints ≤10 cm % | P95 extremo cm | Conectados % | Núcleo med/P95 ms | >137 ms % | RSS MiB |',
             '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in table:
        lines.append(f"| {r['case']} | {r['particles']} | {r['rmse_cm']:.2f} | {r['endpoints_10cm_percent']:.2f} | {r['endpoint_p95_cm']:.2f} | {r['connected_percent']:.2f} | {r['median_ms']:.1f}/{r['p95_ms']:.1f} | {r['over_137ms_percent']:.2f} | {r['peak_rss_mib']:.1f} |")
    lines += ['', 'La comparación completa está en comparison.json y comparison.csv.',
              'Mapas: maps.png; trayectorias: maps_and_paths.png; ampliaciones: details.png.',
              'Los tres paneles de detalle usan la misma escala entre casos para cada región.',
              'Métrica de endpoints v2: extrínseca real del NPZ, centros de celdas y sin truncamiento.',
              'Inflación 22 cm de ocupado, desconocido y exterior; puntos GT comunes.',
              'El costo medido corresponde al núcleo, no al callback completo ROS.',
              'A/B compara datos completos; si son iguales la rapidez no cuenta como mejora intrínseca.',
              'Los umbrales son orientativos. No se ha elegido finalista ni autorizado fusión.',
              '', '## Observaciones humanas', '',
              'Pendientes: paredes dobles, ensanchamiento, habitaciones, pasillos y puerta inferior.',
              'Estado de todos los casos: investigar hasta la revisión conjunta.', '']
    (report_dir/'README.md').write_text('\n'.join(lines))
    print(json.dumps({'experiments': table, 'baseline_equality': equality}, indent=2))
