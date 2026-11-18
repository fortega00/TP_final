#!/usr/bin/env python3
"""Compara un ciclo del núcleo sobre un bag sin ground truth, después del SLAM.

Reutiliza endpoints v2 e inflación métrica de Café. El mapa histórico se
inspecciona en su frame nativo; no se usa como entrada ni como verdad de terreno.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import label
import yaml

from experiment_metrics import angular_error, endpoint_consistency, navigable_cells


def read_map(folder, stem):
    """Lee mapas trinaríos exportados por nuestro núcleo en coordenadas métricas."""
    meta = yaml.safe_load((folder / (stem + '.yaml')).read_text())
    if meta['origin'][2] != 0 or meta['negate'] != 0:
        raise ValueError('Only unrotated non-negated maps are supported')
    pixels = np.flipud(np.asarray(Image.open(folder / meta['image'])))
    if not set(np.unique(pixels)).issubset({0, 128, 205, 254}):
        raise ValueError('Expected a trinary exported map')
    return meta, pixels


def interpolate_path(path, stamps):
    """Interpola SE(2), desenvolviendo yaw y rechazando extrapolaciones."""
    path, stamps = np.asarray(path), np.asarray(stamps)
    if np.any(np.diff(path[:, 0]) <= 0) or np.any(stamps < path[0, 0]) or np.any(stamps > path[-1, 0]):
        raise ValueError('Path interpolation needs increasing timestamps inside its domain')
    return np.column_stack([stamps, *[np.interp(stamps, path[:, 0],
                              np.unwrap(path[:, k]) if k == 3 else path[:, k])
                              for k in (1, 2, 3)]])


def connectivity(pixels, meta, points, start, radius=.22):
    """Cuenta puntos de prueba alcanzables; ningún punto es ground truth."""
    safe = navigable_cells(pixels == 254, meta['resolution'], radius)
    components, count = label(safe)
    origin = np.asarray(meta['origin'][:2])
    initial = np.floor((np.asarray(start) - origin) / meta['resolution']).astype(int)
    if not (0 <= initial[0] < pixels.shape[1] and 0 <= initial[1] < pixels.shape[0]):
        raise ValueError('Initial pose lies outside the map')
    component = components[initial[1], initial[0]]
    cells = np.floor((np.asarray(points) - origin) / meta['resolution']).astype(int)
    inside = ((cells[:, 0] >= 0) & (cells[:, 0] < pixels.shape[1]) &
              (cells[:, 1] >= 0) & (cells[:, 1] < pixels.shape[0]))
    passed = np.zeros(len(cells), dtype=bool)
    admissible = np.zeros(len(cells), dtype=bool)
    admissible[inside] = safe[cells[inside, 1], cells[inside, 0]]
    if component:
        passed[inside] = components[cells[inside, 1], cells[inside, 0]] == component
    return {'radius_m': radius, 'checkpoints': len(cells), 'connected': int(passed.sum()),
            'fraction': float(passed.mean()), 'failed_indices': np.flatnonzero(~passed).tolist(),
            'admissible_checkpoints': int(admissible.sum()),
            'connected_fraction_of_admissible': float(passed.sum()/admissible.sum()) if admissible.any() else None,
            'disconnected_admissible_indices': np.flatnonzero(admissible & ~passed).tolist(),
            'unsafe_footprint_indices': np.flatnonzero(~admissible).tolist(),
            'unknown_and_exterior_blocked_and_inflated': True,
            'start_cell_class': 'free' if pixels[initial[1], initial[0]] == 254 else
                                'occupied' if pixels[initial[1], initial[0]] == 0 else 'unknown',
            'start_is_navigable': bool(component), 'components': int(count),
            'navigable_area_m2': float(safe.sum() * meta['resolution']**2),
            'start_component_area_m2': float((components == component).sum() * meta['resolution']**2)
                                      if component else 0.}


def summarize(values):
    """Resume una serie completa, sin mezclar calentamiento inicial."""
    return {'median': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
            'max': float(np.max(values)), 'mean': float(np.mean(values))}


def common_interior_seed(maps, points):
    """Busca un único punto de referencia libre con huella completa en todos los mapas.

    No sustituye el resultado desde el origen: permite describir por separado
    la conectividad interior cuando el origen inicial no admite la huella.
    """
    accepted = np.ones(len(points), dtype=bool)
    for meta, pixels in maps.values():
        safe = navigable_cells(pixels == 254, meta['resolution'], .22)
        cells = np.floor((points - np.asarray(meta['origin'][:2])) / meta['resolution']).astype(int)
        inside = ((cells[:, 0] >= 0) & (cells[:, 0] < pixels.shape[1]) &
                  (cells[:, 1] >= 0) & (cells[:, 1] < pixels.shape[0]))
        valid = np.zeros(len(points), dtype=bool)
        valid[inside] = safe[cells[inside, 1], cells[inside, 0]]
        accepted &= valid
    indices = np.flatnonzero(accepted)
    return int(indices[0]) if len(indices) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--history', type=Path, required=True)
    parser.add_argument('--cases', nargs='+', default=list('ABCDE'))
    parser.add_argument('--plot-cases', nargs='+', default=['H', 'B', 'C', 'D', 'E'])
    parser.add_argument('--interior-seed-index', type=int)
    parser.add_argument('--reused-cases', nargs='*', default=[])
    parser.add_argument('--title', default='Bag 16/11 — ciclo 01, revisión humana pendiente')
    parser.add_argument('--repeat-check', nargs=2, metavar=('CASE', 'PREVIOUS_FOLDER'))
    args = parser.parse_args()
    if len(set(args.cases)) != len(args.cases) or not {'A', 'B'}.issubset(args.cases) or 'H' in args.cases:
        parser.error('Cases must be unique, include baseline A and parity control B, and exclude historical H')
    root, report = args.root, args.root / 'report'
    report.mkdir(exist_ok=False)
    source = root / 'input/bag_mapa2.npz'
    input_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    with np.load(source) as archive:
        data = {k: archive[k] for k in archive.files}
    if 'truth' in data:
        raise ValueError('This report is specifically for a bag without ground truth')
    runs = {case: json.loads((root / case / 'samples.json').read_text()) for case in args.cases}
    paths = {case: np.asarray(run['paths']['slam']) for case, run in runs.items()}
    stamps = paths['A'][:, 0]
    if not all(np.array_equal(stamps, path[:, 0]) for path in paths.values()):
        raise ValueError('All cases must integrate the same scan timestamps')
    indices = np.searchsorted(data['metadata'][:, 0], stamps)
    np.testing.assert_array_equal(data['metadata'][indices, 0], stamps)
    integrated = np.zeros(len(data['metadata']), bool)
    integrated[indices] = True
    bounded = (data['metadata'][:, 0] >= stamps[0]) & (data['metadata'][:, 0] <= stamps[-1])
    heldout_stamps = data['metadata'][~integrated & bounded, 0]
    probe_stamps = data['metadata'][::20, 0]
    probe_stamps = probe_stamps[(probe_stamps >= stamps[0]) & (probe_stamps <= stamps[-1])]
    baseline_points = interpolate_path(paths['A'], probe_stamps)[:, 1:3]
    np.savez_compressed(report / 'checkpoints.npz', stamps=probe_stamps, xy=baseline_points,
                        withheld_stamps=heldout_stamps)
    historical = json.loads((args.history / 'aula.json').read_text())
    if historical['config'] != runs['A']['configuration']:
        raise ValueError('Baseline must use the configuration saved on 16 November')
    maps, rows, records = {}, [], {}
    maps['H'] = read_map(args.history, 'aula')
    initial = runs['A']['configuration']['filter']
    start = [initial['initial_x'], initial['initial_y']]
    for case, samples in runs.items():
        config = samples['configuration']
        timing = json.loads((root / case / 'timing.json').read_text())
        if timing['input_sha256'] != input_sha:
            raise ValueError('Frozen input mismatch')
        meta, pixels = maps[case] = read_map(root / case, 'map')
        measured = endpoint_consistency(samples, pixels == 0, meta, source)
        heldout = dict(samples, paths={'slam': interpolate_path(paths[case], heldout_stamps).tolist()})
        validation = endpoint_consistency(heldout, pixels == 0, meta, source)
        validation['label'] = 'withheld_scan_endpoints_using_interpolated_ancestral_poses'
        validation['warning'] = ('Scans were not integrated or matched. Poses are interpolated from '
                                 'the fitted trajectory; correlated same-session data, not ground truth.')
        own_points = interpolate_path(paths[case], probe_stamps)[:, 1:3]
        common_nav = connectivity(pixels, meta, baseline_points, start)
        own_nav = connectivity(pixels, meta, own_points, start)
        with np.load(root / case / 'updates.npz') as trace:
            updates = trace['values']
        np.testing.assert_array_equal(updates[:, 0], stamps)
        t, ess = updates[:, 1], updates[:, 2]
        path = paths[case]
        trajectory = {'length_m': float(np.linalg.norm(np.diff(path[:, 1:3], axis=0), axis=1).sum()),
                      'end_to_start_distance_m': float(np.linalg.norm(path[-1, 1:3] - path[0, 1:3])),
                      'end_to_start_yaw_deg': float(np.degrees(angular_error(path[-1, 3], path[0, 3]))),
                      'warning': 'Relative start-end separation; not absolute error or verified loop-closure error.'}
        row = {'case': case, 'particles': config['filter']['particles'],
               'timing_origin': 'previous_cycle' if case in args.reused_cases else 'current_cycle',
               'prior_xy': config['scan_match']['prior_xy'], 'sample_xy': config['scan_match']['sample_xy'],
               'endpoints_10cm_percent': 100 * measured['fraction_within_10cm'],
               'endpoint_median_cm': 100 * measured['distance_m']['median'],
               'endpoint_p95_cm': 100 * measured['distance_m']['p95'],
               'withheld_10cm_percent': 100 * validation['fraction_within_10cm'],
               'withheld_median_cm': 100 * validation['distance_m']['median'],
               'withheld_p95_cm': 100 * validation['distance_m']['p95'],
               'common_connected_percent': 100 * common_nav['fraction'],
               'own_connected_percent': 100 * own_nav['fraction'],
               'end_separation_m': trajectory['end_to_start_distance_m'],
               'median_ms': 1000 * np.median(t), 'p95_ms': 1000 * np.percentile(t, 95),
               'max_ms': 1000 * t.max(), 'over_90ms_percent': 100 * np.mean(t > .09),
               'over_137ms_percent': 100 * np.mean(t > .137),
               'ess_mean_ratio': float(ess.mean()), 'ess_min_ratio': float(ess.min()),
               'resamples': samples['final_status']['resamples'],
               'resample_update_percent': 100 * samples['final_status']['resamples'] / len(t),
               'free_area_m2': float(np.sum(pixels == 254) * meta['resolution']**2),
               'occupied_area_m2': float(np.sum(pixels == 0) * meta['resolution']**2),
               'wall_seconds': timing['wall_seconds'], 'backend': timing['proposal_backend'],
               'init_seconds': timing['initialize_and_jit_warmup_seconds'],
               'peak_rss_mib': timing['max_rss_kib'] / 1024, 'integrations': len(t)}
        rows.append(row)
        records[case] = {'summary': row, 'integrated_scan_endpoints': measured,
                         'withheld_scan_endpoints': validation,
                         'navigation_common_baseline_checkpoints': common_nav,
                         'navigation_own_estimated_checkpoints': own_nav,
                         'trajectory': trajectory, 'timing': timing, 'ess_ratio': summarize(ess),
                         'export_snapshot': {'last_integrated_stamp': float(path[-1, 0]),
                                             'gap_to_historical_last_scan_s': float(path[-1, 0] - historical['scan_stamp'])}}
        (root / case / 'metrics.json').write_text(json.dumps(records[case], indent=2) + '\n')
    hmeta, hpixels = maps['H']
    historical_nav = connectivity(hpixels, hmeta, baseline_points, start)
    seed_index = args.interior_seed_index if args.interior_seed_index is not None else common_interior_seed(maps, baseline_points)
    if seed_index is not None and not 0 <= seed_index < len(baseline_points):
        raise ValueError('Interior seed index is outside the frozen checkpoints')
    interior = {'scope': 'Additional interior connectivity; does not replace failure at the initial origin.',
                'selection': ('Fixed checkpoint index supplied from the previous cycle; no relocation if a new map blocks it.'
                              if args.interior_seed_index is not None else
                              'First baseline checkpoint whose 22 cm footprint is free in every map, including H.'),
                'checkpoint_index': seed_index}
    if seed_index is not None:
        seed = baseline_points[seed_index]
        interior.update(xy=seed.tolist(), stamp=float(probe_stamps[seed_index]), maps={})
        for case, (meta, pixels) in maps.items():
            interior['maps'][case] = connectivity(pixels, meta, baseline_points, seed)
            if case in records:
                own = interpolate_path(paths[case], probe_stamps)[:, 1:3]
                records[case]['navigation_interior_common_checkpoints'] = interior['maps'][case]
                records[case]['navigation_interior_own_checkpoints'] = connectivity(pixels, meta, own, seed)
                records[case]['summary']['interior_connected_percent'] = 100 * interior['maps'][case]['fraction']
                (root / case / 'metrics.json').write_text(json.dumps(records[case], indent=2) + '\n')
    equality = {'config': runs['A']['configuration'] == runs['B']['configuration'],
                'pgm': (root / 'A/map.pgm').read_bytes() == (root / 'B/map.pgm').read_bytes(),
                'trajectory': runs['A']['paths']['slam'] == runs['B']['paths']['slam']}
    with np.load(root / 'A/grid.npz') as a, np.load(root / 'B/grid.npz') as b:
        equality['raw_grid'] = all(a[k].dtype == b[k].dtype and a[k].tobytes() == b[k].tobytes()
                                   for k in ('observed', 'log_odds'))
    result = {'schema_version': 2, 'has_ground_truth': False, 'input_sha256': input_sha,
              'evaluation_kind': 'Fixed-input core replays; not live ROS throughput',
              'common_integrated_scans': len(stamps), 'withheld_scans': len(heldout_stamps),
              'withheld_scope': 'Skipped by odometry movement/time gate; no scan matching or map writes.',
              'checkpoints_scope': 'Same timestamps every 20 input scans; A estimated positions are a proxy, not GT.',
              'timing_scope': 'One low-priority process at a time; host may also run user Gazebo; no isolated CPU claim.',
              'historical': {'source': str(args.history), 'map_sha256': hashlib.sha256((args.history / 'aula.pgm').read_bytes()).hexdigest(),
                             'metadata': historical, 'same_frame_assumption': 'Same stationary initial robot pose/heading; no fitted alignment.',
                             'navigation_baseline_checkpoint_proxy': historical_nav,
                             'free_area_m2': float(np.sum(hpixels == 254) * hmeta['resolution']**2),
                             'occupied_area_m2': float(np.sum(hpixels == 0) * hmeta['resolution']**2),
                             'missing': ['ancestral trajectory', 'scan selection', 'complete update trace', 'ground truth'],
                             'endpoint_comparison': 'Unavailable without original poses; no endpoints inferred from another run.',
                             'usage': 'Post-hoc qualitative reference only, never filter input or ground truth.'},
              'A_B_bitwise_equality': equality, 'experiments': rows, 'details': records,
              'interior_connectivity_diagnostic': interior,
              'reused_cases': args.reused_cases,
              'human_review': {'status': 'pending', 'decisions': {case: 'investigar' for case in args.cases}}}
    if args.repeat_check:
        case, previous_folder = args.repeat_check
        previous = Path(previous_folder)
        old_samples = json.loads((previous / 'samples.json').read_text())
        repeat = {'configuration': runs[case]['configuration'] == old_samples['configuration'],
                  'trajectory': runs[case]['paths']['slam'] == old_samples['paths']['slam'],
                  'pgm': (root / case / 'map.pgm').read_bytes() == (previous / 'map.pgm').read_bytes()}
        with np.load(root / case / 'grid.npz') as current, np.load(previous / 'grid.npz') as old:
            repeat['raw_grid'] = all(current[k].dtype == old[k].dtype and current[k].tobytes() == old[k].tobytes()
                                     for k in ('observed', 'log_odds'))
        result['repeat_control_equality'] = {'case': case, 'previous_folder': str(previous), 'equal': repeat}
    (report / 'comparison.json').write_text(json.dumps(result, indent=2) + '\n')
    with (report / 'comparison.csv').open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys(), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
    plot_report(report, root, maps, paths, args.plot_cases)
    lines = ['# ' + args.title, '',
             '| Caso | N | prior / sample m | Endpoints ≤10 cm % | Reservados ≤10 cm % | P95 extremos cm | Conectividad interior % | Med/P95 núcleo ms | >137 ms % |',
             '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for row in rows:
        lines.append(f"| {row['case']} | {row['particles']} | {row['prior_xy']:.3f} / {row['sample_xy']:.3f} | "
                     f"{row['endpoints_10cm_percent']:.2f} | {row['withheld_10cm_percent']:.2f} | "
                     f"{row['endpoint_p95_cm']:.2f} | {row.get('interior_connected_percent', 0.):.2f} | "
                     f"{row['median_ms']:.1f}/{row['p95_ms']:.1f} | {row['over_137ms_percent']:.2f} |")
    lines += ['', 'H es el mapa guardado intento3 el 16/11: no es ground truth.',
              'A y B usan la configuración histórica; A original, B optimizado. Las demás variantes son optimizadas.',
              'Endpoints v2: 30 haces por scan, extrínseca del bag, distancia a centros sin truncar.',
              f'{len(heldout_stamps)} scans reservados: no integrados; poses interpoladas y datos correlacionados.',
              'Conectividad: inflación 22 cm, ocupado/desconocido/exterior bloqueados.',
              'Puntos comunes del replay A, además del recorrido propio: ambos son estimaciones.',
              'El resultado estricto desde el origen se conserva en comparison.json; no se liberan celdas artificialmente.',
              'La columna interior usa el checkpoint documentado en comparison.json; se archivan índice, pose y ambos resultados.',
              'No hay RMSE ni error absoluto. Separación inicio-fin no certifica cierre.',
              'Núcleo con prioridad baja en host compartido; no mide TF, colas ni descartes ROS.',
              'Mapas en coordenadas nativas: misma escala, orientación, colores y sin ajuste al histórico.',
              'Archivos: maps.png, maps_and_paths.png, details.png, timing_and_ess.png, navigation.png.',
              'Detalles completos en comparison.json y comparison.csv.', '',
              '## Evaluación humana', '', 'Pendiente: paredes dobles, continuidad de paredes, mesas, pasillos y puerta.',
              'No se cambia el perfil físico de uso ni se fusiona este worktree.', '']
    (report / 'README.md').write_text('\n'.join(lines))
    print(json.dumps({'experiments': rows, 'A_B_equality': equality}, indent=2))


def plot_report(report, root, maps, paths, selected):
    """Conserva escala/orientación y regiones idénticas para todos los casos."""
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/hitl_mpl_cache')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    boxes, extents = [], {}
    for case, (meta, pixels) in maps.items():
        r, (x, y, _) = meta['resolution'], meta['origin']
        extents[case] = [x, x + pixels.shape[1]*r, y, y + pixels.shape[0]*r]
        yy, xx = np.nonzero((pixels == 0) | (pixels == 254))
        boxes.append([x + xx.min()*r, x + (xx.max()+1)*r, y + yy.min()*r, y + (yy.max()+1)*r])
    boxes = np.array(boxes)
    crop = [boxes[:, 0].min()-.4, boxes[:, 1].max()+.4,
            boxes[:, 2].min()-.4, boxes[:, 3].max()+.4]
    titles = {'H': 'H · guardado 16/11', 'A': 'A · original 8', 'B': 'B · optimizado 8',
              'C': 'C · optimizado 16', 'D': 'D · 16, prior .025', 'E': 'E · 16, sample .005'}
    for case in paths:
        if case not in titles:
            cfg = json.loads((root / case / 'samples.json').read_text())['configuration']
            titles[case] = (f"{case} · {cfg['filter']['particles']} particulas\n"
                            f"prior {cfg['scan_match']['prior_xy']:g}, sample {cfg['scan_match']['sample_xy']:g}")
    if not selected or any(case not in maps for case in selected):
        raise ValueError('Plot cases must refer to evaluated maps or historical H')
    columns = len(selected)
    def draw(ax, case, limits, path=False, navigation=False):
        meta, pixels = maps[case]
        rendered = np.where(pixels == 0, 0., np.where(pixels == 254, 1., .55))
        ax.imshow(rendered, origin='lower', extent=extents[case], cmap='gray', vmin=0, vmax=1,
                  interpolation='nearest')
        if navigation:
            safe = navigable_cells(pixels == 254, meta['resolution'], .22)
            overlay = np.zeros((*safe.shape, 4))
            overlay[safe] = [.0, .65, .4, .38]
            ax.imshow(overlay, origin='lower', extent=extents[case], interpolation='nearest')
        if path and case in paths:
            track = paths[case]
            ax.plot(track[:, 1], track[:, 2], color='#e67e22', linewidth=.65)
            ax.scatter(track[[0, -1], 1], track[[0, -1], 2], c=['#00a86b', '#d92626'], s=15, zorder=3)
        ax.set(xlim=limits[:2], ylim=limits[2:], aspect='equal', xlabel='x [m]', ylabel='y [m]', title=titles[case])
    for filename, with_paths, with_navigation in [('maps.png', False, False),
                                                   ('maps_and_paths.png', True, False),
                                                   ('navigation.png', False, True)]:
        fig, axes = plt.subplots(1, columns, figsize=(4.4*columns, 8), sharex=True, sharey=True, squeeze=False)
        for ax, case in zip(axes[0], selected):
            draw(ax, case, crop, with_paths, with_navigation)
        fig.tight_layout()
        fig.savefig(report / filename, dpi=170)
        plt.close(fig)
    # Regiones fijas de la sala: se archivan para poder reproducir la inspección.
    regions = [('Pared superior y puerta', [-10, 1, 5.4, 8.2]),
               ('Mesas y pasillos centrales', [-9, -1, -.5, 5.6]),
               ('Pared inferior y esquinas', [-12, 1, -4, -1.5])]
    (report / 'regions.json').write_text(json.dumps(regions, indent=2) + '\n')
    fig, axes = plt.subplots(3, columns, figsize=(4.4*columns, 15), squeeze=False)
    for row, (name, limits) in enumerate(regions):
        for col, case in enumerate(selected):
            draw(axes[row, col], case, limits)
            if col == 0:
                axes[row, col].set_ylabel(name + '\ny [m]')
    fig.tight_layout()
    fig.savefig(report / 'details.png', dpi=170)
    plt.close(fig)
    fig, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
    for case in paths:
        with np.load(root / case / 'updates.npz') as archive:
            u = archive['values']
        times = u[:, 0] - u[0, 0]
        axes[0].plot(times, u[:, 1]*1000, label=case, linewidth=.6, alpha=.75)
        axes[1].plot(times, u[:, 2], label=case, linewidth=.6, alpha=.75)
    for limit in (90, 137):
        axes[0].axhline(limit, color='black', linestyle='--', linewidth=.7)
    axes[1].axhline(.5, color='black', linestyle='--', linewidth=.7)
    axes[0].set(ylabel='Nucleo [ms]', title='Host compartido, prioridad baja; sin calentamiento ni ROS')
    axes[1].set(xlabel='Tiempo del sensor [s]', ylabel='ESS / N')
    for ax in axes:
        ax.legend(ncol=min(5, len(paths)))
        ax.grid(alpha=.25)
    fig.tight_layout()
    fig.savefig(report / 'timing_and_ess.png', dpi=170)
    plt.close(fig)


if __name__ == '__main__':
    main()
