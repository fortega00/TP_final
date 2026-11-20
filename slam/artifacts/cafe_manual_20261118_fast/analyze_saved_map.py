"""Evalúa archivos guardados de la corrida rápida; no ejecuta SLAM ni ROS."""

from pathlib import Path
import hashlib
import importlib.util
import json
import math
import os
import xml.etree.ElementTree as ET

os.environ.setdefault("MPLCONFIGDIR", "/tmp/tp-final-map-evaluation-mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation, distance_transform_edt, label
import yaml

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "evaluation", HERE / "source/analyze_a_experiment.py")
evaluation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluation)
spec = importlib.util.spec_from_file_location(
    "metric_helpers", HERE / "source/experiment_metrics.py")
helpers = importlib.util.module_from_spec(spec)
spec.loader.exec_module(helpers)
PACKAGE = Path(os.environ.get("CAFE_PACKAGE", "/home/fran/PRA_00/pkg/custom_map"))
export = json.loads((HERE / "cafe.json").read_text())
metadata = yaml.safe_load((HERE / "cafe.yaml").read_text())
pixels = np.flipud(np.asarray(Image.open(HERE / metadata["image"])))
occupied, free = pixels == 0, pixels == 254
known = occupied | free
resolution = metadata["resolution"]
origin = np.array(metadata["origin"][:2])
reference, floor, boxes = evaluation.cafe_reference(
    PACKAGE, export["config"]["map"], *pixels.shape)
yy, xx = np.meshgrid(origin[1] + (np.arange(pixels.shape[0]) + .5) * resolution,
                     origin[0] + (np.arange(pixels.shape[1]) + .5) * resolution,
                     indexing="ij")
start_xy = np.floor(-origin / resolution).astype(int)
start = (start_xy[1], start_xy[0])
safe_reference = helpers.navigable_cells(floor & ~reference, resolution, .27)
reachable_reference = evaluation.connected(safe_reference, start)
surface = reference & binary_dilation(
    reachable_reference, structure=helpers.metric_disk(.40, resolution))
indoor = floor & ~reference & (yy > -10.9)
distance_to_reference = distance_transform_edt(~reference) * resolution
distance_to_map = distance_transform_edt(~occupied) * resolution

def fraction(numerator, denominator):
    """Devuelve una fracción sin inventar valor cuando el conjunto está vacío."""
    count = int(denominator.sum())
    return float((numerator & denominator).sum() / count) if count else None

results = {
    "evaluation_kind": "Static analysis of saved fast manual run; no estimator replay",
    "run_identification": "User confirmed latest run was fast, not the collision run",
    "map_sha256": hashlib.sha256((HERE / "cafe.pgm").read_bytes()).hexdigest(),
    "reference": {
        "kind": "Approximate SDF collision boxes at z=0.19 m; GPU LiDAR uses visual meshes",
        "alignment": "Original spawn/map coordinates, no fitted transform",
        "files_sha256": {str(p.relative_to(PACKAGE)): hashlib.sha256(p.read_bytes()).hexdigest()
                         for p in [PACKAGE / "worlds/cafe.world",
                                   PACKAGE / "models/custom_cafe/model.sdf",
                                   PACKAGE / "models/custom_cafe_table/model.sdf"]},
        "coverage_warning": "Reachable floor includes the outdoor strip below the open lower door",
    },
    "resolution_m": resolution,
    "areas_m2": {"free": float(free.sum() * resolution**2),
                  "occupied": float(occupied.sum() * resolution**2),
                  "known": float(known.sum() * resolution**2)},
    "coverage": {
        "approx_reachable_floor_observed": fraction(known, reachable_reference),
        "approx_reachable_floor_free": fraction(free, reachable_reference),
        "approx_indoor_non_obstacle_observed": fraction(known, indoor),
        "approx_indoor_non_obstacle_free": fraction(free, indoor),
        "approx_indoor_unknown_area_m2": float((~known & indoor).sum() * resolution**2),
    },
    "geometry": {}, "regions": {}, "long_wall_fits": {}, "connectivity": {},
    "diagnostics": export["diagnostics"],
    "dropped_scans": export["dropped_scans"],
    "adapter": export["adapter"],
    "last_scan_stamp": export["scan_stamp"],
    "unavailable": ["Slow run map", "Pose RMSE and trajectory", "Measured speed and angular velocity",
                    "Full timing/ESS/drop history", "Runtime Numba backend of the terminated process",
                    "Per-beam acquisition time"],
    "user_reported_teleop_limits": json.loads((HERE / "teleop_limits.json").read_text()),
}
for tolerance in (.05, .10, .15):
    results["geometry"][str(tolerance)] = {
        "tolerance_m": tolerance,
        "occupied_precision_approx": fraction(distance_to_reference <= tolerance+1e-12, occupied),
        "reachable_wall_surface_recall_approx": fraction(distance_to_map <= tolerance+1e-12, surface),
    }
results["geometry"]["occupied_distance_to_reference_m"] = evaluation.stats(distance_to_reference[occupied])
obstacle_points = np.column_stack((xx[occupied], yy[occupied]))
results["table_legs"] = []
for box in boxes:
    if box["model"] != "custom_cafe_table":
        continue
    center_xy = np.asarray(box["center"][:2])
    distances = np.linalg.norm(obstacle_points-center_xy, axis=1)
    closest = int(np.argmin(distances))
    col, row = np.floor((center_xy-origin)/resolution).astype(int)
    results["table_legs"].append({
        "reference_center_xy_m": center_xy.tolist(),
        "nearest_occupied_center_xy_m": obstacle_points[closest].tolist(),
        "nearest_occupied_distance_m": float(distances[closest]),
        "occupied_within_15cm": bool(distances[closest] <= .15),
        "reference_center_pixel": int(pixels[row, col]),
        "warning": "Map/reference comparison, not a measurement of robot pose error",
    })

# Verifica que las patas finas existen también en la malla que observa el GPU LiDAR.
mesh_path = PACKAGE / "models/custom_cafe_table/meshes/custom_cafe_table.dae"
tree = ET.parse(mesh_path).getroot()
ns = {"c": tree.tag.split("}")[0][1:]}
mesh = tree.find('.//c:geometry[@id="ID38"]/c:mesh', ns)
vertex_source = mesh.find('c:vertices/c:input', ns).get("source")[1:]
unit = float(tree.find('c:asset/c:unit', ns).get("meter"))
vertices = np.fromstring(mesh.find(f'c:source[@id="{vertex_source}"]/c:float_array', ns).text,
                         sep=" ").reshape(-1, 3)*unit
crossings = []
for triangles in mesh.findall('c:triangles', ns):
    inputs = triangles.findall('c:input', ns)
    stride = max(int(item.get("offset", "0")) for item in inputs)+1
    offset = int(next(item for item in inputs if item.get("semantic") == "VERTEX").get("offset", "0"))
    indices = np.fromstring(triangles.find('c:p', ns).text, sep=" ", dtype=int).reshape(-1, stride)[:, offset].reshape(-1, 3)
    for face in vertices[indices]:
        for i, j in [(0, 1), (1, 2), (2, 0)]:
            a, b = face[i], face[j]
            if (a[2]-.19)*(b[2]-.19) < 0:
                crossings.append((a+(.19-a[2])/(b[2]-a[2])*(b-a))[:2])
crossings = np.asarray(crossings)
results["table_visual_cross_section"] = {
    "mesh_sha256": hashlib.sha256(mesh_path.read_bytes()).hexdigest(),
    "height_m": .19, "bounds_min_xy_m": crossings.min(axis=0).tolist(),
    "bounds_max_xy_m": crossings.max(axis=0).tolist(),
    "width_xy_m": np.ptp(crossings, axis=0).tolist(),
    "scope": "Untransformed ID38 column instance in this specific Z_UP table asset; other table geometry is above the laser plane",
}
for name, y0, y1 in [("lower", -10.9, -4.), ("middle", -4., 4.), ("upper", 4., 12.1)]:
    region = indoor & (yy >= y0) & (yy < y1)
    results["regions"][name] = {
        "y_interval_m": [y0, y1], "approx_observed_fraction": fraction(known, region),
        "approx_free_fraction": fraction(free, region),
        "approx_unknown_area_m2": float((~known & region).sum()*resolution**2)}
for name, expected_x, y0, y1 in [("left", -5.03, -10.7, 6.8), ("right", 4.24, -10.7, 9.9)]:
    selected = occupied & (np.abs(xx - expected_x) < .35) & (yy >= y0) & (yy <= y1)
    x, y = xx[selected], yy[selected]
    if len(x) < 2:
        continue
    slope, intercept = np.polyfit(y, x, 1)
    results["long_wall_fits"][name] = {
        "cells": len(x), "angle_from_vertical_deg": float(np.degrees(np.arctan(slope))),
        "mean_offset_from_sdf_m": float(np.mean(x - expected_x)),
        "p95_absolute_residual_to_fitted_line_m": float(np.percentile(np.abs(x-(slope*y+intercept)), 95)),
    }

components = {}
for radius in (.22, .27):
    mask = helpers.navigable_cells(free, resolution, radius)
    component = evaluation.connected(mask, start)
    components[radius] = component
    labels, count = label(mask)
    sizes = np.bincount(labels.ravel())[1:]
    results["connectivity"][str(radius)] = {
        "inflation_radius_m": radius, "unknown_and_exterior_blocked": True,
        "free_area_connected_to_spawn_m2": float(component.sum()*resolution**2),
        "fraction_of_candidate_navigable_cells_connected": fraction(component, mask),
        "components": int(count),
        "component_areas_m2_descending": (np.sort(sizes)[::-1]*resolution**2).tolist(),
        "warning": "Static connectivity only; no controller, arbitrary-goal or footprint calibration validation",
    }

# Se distingue hueco sin ocupados de paso confirmado libre en toda la banda.
band = (yy[:, 0] >= -11.08) & (yy[:, 0] <= -10.88)
center = int((-3.61-origin[0])/resolution)
results["lower_door"] = {"approx_nominal_width_m": 1.09, "band_y_m": [-11.08, -10.88]}
for name, columns in [("no_occupied_gap_m", ~occupied[band].any(axis=0)),
                      ("fully_observed_free_gap_m", free[band].all(axis=0))]:
    left = right = center
    if not columns[center]:
        width = 0.
    else:
        while left > 0 and columns[left-1]: left -= 1
        while right+1 < len(columns) and columns[right+1]: right += 1
        width = float((right-left+1)*resolution)
    results["lower_door"][name] = width

(HERE / "metrics.json").write_text(json.dumps(results, indent=2)+"\n")
(HERE / "reference_boxes.json").write_text(json.dumps(boxes, indent=2)+"\n")
np.savez_compressed(HERE / "reference.npz", occupied=reference, floor=floor,
                    reachable=reachable_reference, safe_map=components[.27])
extent = [origin[0], origin[0]+pixels.shape[1]*resolution,
          origin[1], origin[1]+pixels.shape[0]*resolution]
fig, axes = plt.subplots(1, 3, figsize=(15, 12), sharex=True, sharey=True)
axes[0].imshow(pixels, origin="lower", extent=extent, cmap="gray", vmin=0, vmax=255)
axes[0].set_title("Corrida rápida: mapa guardado")
overlay = np.full((*pixels.shape, 3), .82)
overlay[free] = [1., 1., 1.]
overlay[reference] = [.98, .5, .08]
overlay[occupied] = [0., .42, .62]
overlay[occupied & reference] = [.1, .1, .1]
axes[1].imshow(overlay, origin="lower", extent=extent)
axes[1].set_title("Azul: mapa; naranja: SDF aproximado")
coverage = np.full((*pixels.shape, 3), .82)
coverage[free] = [1., 1., 1.]
coverage[components[.27]] = [.45, .80, .55]
coverage[~known & indoor] = [.95, .3, .3]
coverage[occupied] = [0., 0., 0.]
axes[2].imshow(coverage, origin="lower", extent=extent)
axes[2].set_title("Verde: conectado, radio 27 cm\nRojo: interior SDF sin observar")
for ax in axes:
    ax.plot(0, 0, "r+", markersize=8)
    ax.set(xlabel="x [m]", ylabel="y [m]", aspect="equal", xlim=(-6., 5.), ylim=(-14., 13.))
fig.tight_layout()
fig.savefig(HERE / "map_comparison.png", dpi=160)
plt.close(fig)

fig, axes = plt.subplots(1, 3, figsize=(15, 5))
for ax, limits, title in zip(axes, [(-5.5, -1.5, -12., -8.), (-5.5, 4.8, 6., 12.5), (-5.5, 4.8, -4., 4.)],
                              ["Puerta inferior", "Sector superior", "Sector central"]):
    ax.imshow(overlay, origin="lower", extent=extent)
    ax.set(xlim=limits[:2], ylim=limits[2:], title=title, xlabel="x [m]", ylabel="y [m]", aspect="equal")
fig.tight_layout()
fig.savefig(HERE / "details.png", dpi=160)
plt.close(fig)

fig, axes = plt.subplots(1, 5, figsize=(15, 4))
for ax, leg in zip(axes, results["table_legs"]):
    x, y = leg["reference_center_xy_m"]
    ax.imshow(pixels, origin="lower", extent=extent, cmap="gray", vmin=0, vmax=255)
    ax.add_patch(plt.Circle((x, y), .021, fill=False, color="darkorange", linewidth=2))
    ax.set(xlim=(x-.5, x+.5), ylim=(y-.5, y+.5), aspect="equal", xlabel="x [m]", ylabel="y [m]",
           title=f"Pata ({x:g}, {y:g})\nOcupado a {leg['nearest_occupied_distance_m']:.2f} m")
fig.suptitle("Posiciones nominales de patas: círculo naranja; negro: ocupado en el mapa")
fig.tight_layout()
fig.savefig(HERE / "table_legs.png", dpi=160)
plt.close(fig)
print(json.dumps({key: results[key] for key in ["geometry", "coverage", "regions", "connectivity", "lower_door", "long_wall_fits", "table_legs", "table_visual_cross_section"]}, indent=2))
