#!/usr/bin/env python3
"""Analyze exported ROS maps and paths, with explicit limits on reference data."""
import argparse
from collections import deque
import json
import math
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image
import yaml


def wrap(x):
    return np.arctan2(np.sin(x), np.cos(x))


def stats(values):
    values = np.asarray(values, dtype=float)
    return {'mean': float(values.mean()), 'median': float(np.median(values)),
            'p95': float(np.percentile(values, 95)), 'max': float(values.max())} if len(values) else None


def dilate(mask, radius):
    result = np.zeros_like(mask)
    height, width = mask.shape
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx*dx + dy*dy > radius*radius:
                continue
            ys, ye = max(0, dy), min(height, height + dy)
            xs, xe = max(0, dx), min(width, width + dx)
            result[ys:ye, xs:xe] |= mask[ys-dy:ye-dy, xs-dx:xe-dx]
    return result


def connected(mask, start):
    result = np.zeros_like(mask)
    row, col = start
    if not (0 <= row < mask.shape[0] and 0 <= col < mask.shape[1] and mask[row, col]):
        return result
    todo = deque([(row, col)])
    result[row, col] = True
    while todo:
        y, x = todo.popleft()
        for yy, xx in ((y-1, x), (y+1, x), (y, x-1), (y, x+1)):
            if 0 <= yy < mask.shape[0] and 0 <= xx < mask.shape[1] and mask[yy, xx] and not result[yy, xx]:
                result[yy, xx] = True
                todo.append((yy, xx))
    return result


def pose_metrics(paths):
    result = {}
    for key, rows in paths.items():
        a = np.asarray(rows, dtype=float)
        if len(a):
            result[key] = {'samples': len(a), 'duration_s': float(a[-1, 0] - a[0, 0]),
                           'length_m': float(np.linalg.norm(np.diff(a[:, 1:3], axis=0), axis=1).sum()),
                           'end_to_start_distance_m': float(np.linalg.norm(a[-1, 1:3]-a[0, 1:3])),
                           'end_to_start_yaw_deg': float(np.degrees(wrap(a[-1, 3]-a[0, 3]))),
                           'last_pose': a[-1, 1:].tolist()}
    truth = np.asarray(paths.get('truth', []), dtype=float)
    if len(truth):
        for key in ('slam', 'odom'):
            a = np.asarray(paths.get(key, []), dtype=float)
            if not len(a):
                continue
            a = a[(a[:, 0] >= truth[0, 0]) & (a[:, 0] <= truth[-1, 0])]
            target = np.column_stack([np.interp(a[:, 0], truth[:, 0], truth[:, i]) for i in (1, 2)])
            yaw = np.interp(a[:, 0], truth[:, 0], np.unwrap(truth[:, 3]))
            position = np.linalg.norm(a[:, 1:3]-target, axis=1)
            angular = np.abs(wrap(a[:, 3]-yaw))
            result[key].update(position_error_m=stats(position),
                               position_rmse_m=float(np.sqrt(np.mean(position**2))),
                               yaw_rmse_deg=float(np.degrees(np.sqrt(np.mean(angular**2)))),
                               final_position_error_m=float(position[-1]))
    return result


def cafe_reference(package, config, height, width):
    """Rasterize local SDF collision boxes at the laser plane, not render meshes.

    This is an approximate structural reference; GPU LiDAR uses visual meshes.
    It must not be described as an exact sensor-ground-truth occupancy map.
    """
    resolution = config['resolution']
    x = config['origin_x'] + (np.arange(width) + .5) * resolution
    y = config['origin_y'] + (np.arange(height) + .5) * resolution
    xx, yy = np.meshgrid(x, y)
    occupied, floor = np.zeros((height, width), bool), np.zeros((height, width), bool)
    world = ET.parse(package / 'worlds/cafe.world').getroot()
    boxes = []
    for include in world.findall('./world/include'):
        name = include.findtext('uri').split('model://')[-1]
        parent = np.fromstring(include.findtext('pose', '0 0 0 0 0 0'), sep=' ')
        model = ET.parse(package / 'models' / name / 'model.sdf').getroot()
        for link in model.findall('./model/link'):
            lp = np.fromstring(link.findtext('pose', '0 0 0 0 0 0'), sep=' ')
            for collision in link.findall('collision'):
                size = collision.findtext('geometry/box/size')
                if size is None:
                    continue
                sx, sy, sz = np.fromstring(size, sep=' ')
                cp = np.fromstring(collision.findtext('pose', '0 0 0 0 0 0'), sep=' ')
                # All installed cafe box/link poses have zero roll and pitch.
                if any(abs(v) > 1e-8 for v in (*parent[3:5], *lp[3:5], *cp[3:5])):
                    raise ValueError('Reference needs general 3D box transforms')
                center = parent[:3] + lp[:3] + cp[:3]
                angle = parent[5] + lp[5] + cp[5]
                dx, dy = xx-center[0], yy-center[1]
                # Cell-box overlap preserves posts thinner than one cell.
                half_cell = resolution/2 * (abs(np.cos(angle))+abs(np.sin(angle)))
                inside = ((np.abs(np.cos(angle)*dx+np.sin(angle)*dy) <= sx/2+half_cell) &
                          (np.abs(-np.sin(angle)*dx+np.cos(angle)*dy) <= sy/2+half_cell))
                if collision.attrib['name'] == 'main_floor':
                    floor |= inside
                elif center[2]-sz/2 <= .19 <= center[2]+sz/2:
                    occupied |= inside
                    boxes.append({'model': name, 'collision': collision.attrib['name'],
                                  'center': center.tolist(), 'size': [sx, sy, sz]})
    return occupied, floor, boxes


def endpoint_consistency(output, samples, occupied, metadata):
    """In-sample residual only; this cannot establish absolute map accuracy."""
    source = output / 'scans.npz'
    path = np.asarray(samples['paths'].get('slam', []), dtype=float)
    if not source.exists() or not len(path):
        return None
    data = np.load(source)
    scan_meta, ranges = data['metadata'], data['ranges']
    points = []
    offset = .043 if samples['mode'] == 'simulation' else .0435
    offset_y = 0 if samples['mode'] == 'simulation' else 5.26e-5
    for row in path:
        index = int(np.argmin(np.abs(scan_meta[:, 0]-row[0])))
        if abs(scan_meta[index, 0]-row[0]) > 1e-5:
            continue
        reading, scan = ranges[index], scan_meta[index]
        inds = np.flatnonzero(np.isfinite(reading) & (reading > max(scan[3], samples['configuration']['sensor']['min_range']))
                             & (reading < min(scan[4], samples['configuration']['sensor']['max_range'])))
        inds = inds[np.linspace(0, len(inds)-1, min(30, len(inds)), dtype=int)] if len(inds) else inds
        angles = scan[1] + inds*scan[2] + math.pi
        bx, by = offset+reading[inds]*np.cos(angles), offset_y+reading[inds]*np.sin(angles)
        c, s = np.cos(row[3]), np.sin(row[3])
        points.extend(np.column_stack((row[1]+c*bx-s*by, row[2]+s*bx+c*by)))
    if not points:
        return None
    points = np.asarray(points)
    resolution = metadata['resolution']
    origin = np.asarray(metadata['origin'][:2])
    cells = np.floor((points-origin)/resolution).astype(int)
    distances = np.full(len(points), .6)
    height, width = occupied.shape
    radius = int(math.ceil(.6/resolution))
    for dy in range(-radius, radius+1):
        for dx in range(-radius, radius+1):
            if dx*dx+dy*dy > radius*radius:
                continue
            cx, cy = cells[:, 0]+dx, cells[:, 1]+dy
            valid = (cx >= 0) & (cx < width) & (cy >= 0) & (cy < height)
            inds = np.flatnonzero(valid)
            inds = inds[occupied[cy[inds], cx[inds]]]
            centers = origin + (cells[inds]+np.array([dx, dy])+.5)*resolution
            distances[inds] = np.minimum(distances[inds], np.linalg.norm(points[inds]-centers, axis=1))
    return {'label': 'in_sample_endpoint_to_exported_obstacle_distance',
            'samples': len(distances), 'distance_cap_m': .6, 'distance_m': stats(distances),
            'fraction_within_10cm': float(np.mean(distances <= .10)),
            'warning': 'Uses the same measurements that built the map; not independent accuracy.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    parser.add_argument('--cafe-package', type=Path)
    parser.add_argument('--reference-map', type=Path, help='Prior map for qualitative comparison, never ground truth')
    args = parser.parse_args()
    output = args.output
    samples = json.loads((output / 'samples.json').read_text())
    metadata = yaml.safe_load((output / 'map.yaml').read_text())
    image = np.flipud(np.asarray(Image.open(output / metadata['image'])))
    occupied, free = image == 0, image == 254
    known = occupied | free
    resolution = metadata['resolution']
    result = {'mode': samples['mode'], 'run_completed': samples['success'],
              'has_ground_truth': bool(samples['paths'].get('truth')),
              'poses': pose_metrics(samples['paths']), 'status': samples['final_status'],
              'map': {'resolution_m': resolution, 'width': image.shape[1], 'height': image.shape[0],
                      'occupied_cells': int(occupied.sum()), 'free_cells': int(free.sum()),
                      'unknown_cells': int((~known).sum()), 'known_area_m2': float(known.sum()*resolution**2)},
              'wall_seconds': samples.get('wall_seconds'),
              'in_sample_consistency': endpoint_consistency(output, samples, occupied, metadata)}
    export_info = output / 'map.json'
    if export_info.exists():
        export = json.loads(export_info.read_text())
        captured_stamp = samples['paths']['slam'][-1][0]
        result['export_snapshot'] = {
            'scan_stamp': export['scan_stamp'], 'captured_path_stamp': captured_stamp,
            'gap_s': export['scan_stamp']-captured_stamp,
            'diagnostics': export['diagnostics']}
    navigable = free & ~dilate(occupied, math.ceil(.27/resolution))
    origin = np.asarray(metadata['origin'][:2])
    start_xy = np.floor((np.array([0., 0.])-origin)/resolution).astype(int)
    component = connected(navigable, (start_xy[1], start_xy[0]))
    check_path = np.asarray(samples['paths'].get('truth') or samples['paths'].get('slam', []))
    if len(check_path):
        points = check_path[::20, 1:3]
        cells = np.floor((points-origin)/resolution).astype(int)
        inside = ((cells[:, 0] >= 0) & (cells[:, 0] < image.shape[1]) &
                  (cells[:, 1] >= 0) & (cells[:, 1] < image.shape[0]))
        success = np.zeros(len(cells), bool)
        success[inside] = component[cells[inside, 1], cells[inside, 0]]
        result['navigation_check'] = {
            'robot_radius_plus_margin_m': .27, 'sampled_visited_poses': len(cells),
            'connected_visited_fraction': float(success.mean()),
            'unknown_is_blocked': True,
            'scope': 'Connectivity of sampled visited poses; not safety for arbitrary goals.'}
    updates = [r['update_seconds'] for r in samples.get('status_history', []) if r.get('processed_scans', 0)]
    result['sampled_update_wall_seconds'] = stats(updates)
    # This distribution samples the latest update once per status message,
    # rather than pretending to include every filter invocation.
    result['timing_sampling'] = 'Latest filter update sampled by 1 Hz wall-time status messages.'
    if args.reference_map:
        prior = yaml.safe_load(args.reference_map.read_text())
        source_image = args.reference_map.parent / prior['image']
        prior['image'] = 'reference_map.pgm'
        (output / prior['image']).write_bytes(source_image.read_bytes())
        (output / 'reference_map.yaml').write_text(yaml.safe_dump(prior, sort_keys=False))
        result['prior_map_comparison'] = {
            'source': str(args.reference_map), 'image_sha256': hashlib.sha256(source_image.read_bytes()).hexdigest(),
            'usage': 'Qualitative comparison after SLAM; not used by the filter; not ground truth.',
            'alignment': 'Original map coordinates, without fitted rotation or translation.'}
    reference = None
    if args.cafe_package:
        config = samples['configuration']['map']
        reference, floor, boxes = cafe_reference(args.cafe_package, config, *image.shape)
        safe = floor & ~dilate(reference, math.ceil(.27/resolution))
        start = (int((0-config['origin_y'])/resolution), int((0-config['origin_x'])/resolution))
        reachable = connected(safe, start)
        tolerance = math.ceil(.15/resolution)
        surface = reference & dilate(reachable, math.ceil(.40/resolution))
        result['approximate_sdf_reference'] = {
            'method': 'Collision boxes at world z=0.19m; visual mesh LiDAR may disagree.',
            'assumption': 'Map origin coincides with Gazebo spawn (0,0,0); initial robot stationary.',
            'tolerance_m': .15, 'robot_radius_plus_margin_m': .27,
            'reachable_free_area_m2': float(reachable.sum()*resolution**2),
            'reachable_free_coverage': float((free & reachable).sum()/max(1, reachable.sum())),
            'reachable_observed_coverage': float((known & reachable).sum()/max(1, reachable.sum())),
            'occupied_precision': float((occupied & dilate(reference, tolerance)).sum()/max(1, occupied.sum())),
            'reachable_wall_surface_recall': float((surface & dilate(occupied, tolerance)).sum()/max(1, surface.sum())),
            'free_on_reference_obstacle_cells': int((free & reference).sum())}
        np.savez_compressed(output / 'reference.npz', occupied=reference, floor=floor, reachable=reachable)
        (output / 'reference_boxes.json').write_text(json.dumps(boxes, indent=2)+'\n')
    (output / 'metrics.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(9, 10))
    origin = metadata['origin']
    extent = [origin[0], origin[0]+image.shape[1]*resolution,
              origin[1], origin[1]+image.shape[0]*resolution]
    ax.imshow(image, origin='lower', extent=extent, cmap='gray', vmin=0, vmax=255)
    colors = {'slam': '#00a896', 'odom': '#f18f01', 'truth': '#19983b'}
    for key, rows in samples['paths'].items():
        a = np.asarray(rows)
        if len(a):
            ax.plot(a[:, 1], a[:, 2], color=colors[key], linewidth=1.2, label=key)
    ax.set(xlabel='x [m]', ylabel='y [m]', aspect='equal', title='FastSLAM — '+samples['mode'])
    ax.legend()
    fig.tight_layout()
    fig.savefig(output / 'map_and_paths.png', dpi=180)
    plt.close(fig)
    truth = np.asarray(samples['paths'].get('truth', []), dtype=float)
    if len(truth):
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
        for key in ('slam', 'odom'):
            a = np.asarray(samples['paths'][key], dtype=float)
            a = a[(a[:, 0] >= truth[0, 0]) & (a[:, 0] <= truth[-1, 0])]
            xy = np.column_stack([np.interp(a[:, 0], truth[:, 0], truth[:, i]) for i in (1, 2)])
            yaw = np.interp(a[:, 0], truth[:, 0], np.unwrap(truth[:, 3]))
            t = a[:, 0]-truth[0, 0]
            axes[0].plot(t, np.linalg.norm(a[:, 1:3]-xy, axis=1)*100,
                         color=colors[key], label=key)
            axes[1].plot(t, np.degrees(np.abs(wrap(a[:, 3]-yaw))),
                         color=colors[key], label=key)
        axes[0].set(ylabel='Position error [cm]', title='Captured ancestral trajectory vs ground truth')
        axes[1].set(xlabel='Sensor time [s]', ylabel='Yaw error [deg]')
        for ax in axes:
            ax.grid(alpha=.3)
            ax.legend()
        fig.tight_layout()
        fig.savefig(output / 'pose_errors.png', dpi=180)
        plt.close(fig)
    if reference is not None:
        fig, ax = plt.subplots(figsize=(8, 11))
        ax.imshow(np.where(reference, 0, 255), origin='lower', extent=extent,
                  cmap='gray', vmin=0, vmax=255)
        missing = np.ma.masked_where(~(reachable & ~known), np.ones_like(known, dtype=float))
        ax.imshow(missing, origin='lower', extent=extent, cmap='Reds', vmin=0, vmax=1, alpha=.7)
        a = np.asarray(samples['paths'].get('truth', []))
        if len(a):
            ax.plot(a[:, 1], a[:, 2], color=colors['truth'], linewidth=.8)
        ax.set(xlabel='x [m]', ylabel='y [m]', aspect='equal',
               title='Approximate SDF reference; red = reachable cells not observed')
        fig.tight_layout()
        fig.savefig(output / 'coverage.png', dpi=180)
        plt.close(fig)
    if args.reference_map:
        fig, axes = plt.subplots(1, 2, figsize=(12, 8))
        for ax, meta, pixels, title in (
                (axes[0], prior, np.flipud(np.asarray(Image.open(output / prior['image']))), 'Prior map (not ground truth)'),
                (axes[1], metadata, image, 'FastSLAM real-bag map')):
            x, y, _ = meta['origin']
            r = meta['resolution']
            ax.imshow(pixels, origin='lower', extent=[x, x+pixels.shape[1]*r, y, y+pixels.shape[0]*r],
                      cmap='gray', vmin=0, vmax=255)
            ax.set(xlim=(-8, 6), ylim=(-12, 4), aspect='equal', xlabel='x [m]', ylabel='y [m]', title=title)
        fig.tight_layout()
        fig.savefig(output / 'reference_comparison.png', dpi=180)
        plt.close(fig)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
