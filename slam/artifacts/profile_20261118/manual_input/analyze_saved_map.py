"""Evalúa la exportación existente; no ejecuta SLAM, ROS ni simulación."""

from pathlib import Path
import hashlib
import importlib.util
import json
import math
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/tp-final-map-evaluation-mpl")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import yaml


HERE = Path(__file__).resolve().parent
REPO = HERE.parents[2]
SOURCE = REPO / "slam/scripts/analyze_a_experiment.py"
spec = importlib.util.spec_from_file_location("evaluation", SOURCE)
evaluation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(evaluation)
PREFIX = "cafe_teleop_20261118_131436"
PACKAGE = Path("/home/fran/PRA_00/pkg/custom_map")
export = json.loads((HERE / (PREFIX + ".json")).read_text())
metadata = yaml.safe_load((HERE / (PREFIX + ".yaml")).read_text())
pixels = np.flipud(np.asarray(Image.open(HERE / metadata["image"])))
occupied, free = pixels == 0, pixels == 254
known = occupied | free
resolution = metadata["resolution"]
origin = np.array(metadata["origin"][:2])
reference, floor, boxes = evaluation.cafe_reference(
    PACKAGE, export["config"]["map"], *pixels.shape)
radius = math.ceil(.27 / resolution)
start_xy = np.floor((np.array([0., 0.]) - origin) / resolution).astype(int)
start = (start_xy[1], start_xy[0])
reachable = evaluation.connected(floor & ~evaluation.dilate(reference, radius), start)
surface = reference & evaluation.dilate(reachable, math.ceil(.4 / resolution))
interior = floor & ~reference
yy, xx = np.meshgrid(origin[1] + (np.arange(pixels.shape[0]) + .5) * resolution,
                     origin[0] + (np.arange(pixels.shape[1]) + .5) * resolution,
                     indexing="ij")
# El piso también se extiende al exterior de la puerta inferior.
indoor = interior & (yy > -10.9)
results = {
    "evaluation_kind": "Static analysis of an existing manual run; no estimator replay",
    "map_sha256": hashlib.sha256((HERE / metadata["image"]).read_bytes()).hexdigest(),
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
                  "known": float(known.sum() * resolution**2),
                  "approx_reachable_floor": float(reachable.sum() * resolution**2)},
    "coverage": {
        "approx_reachable_floor_observed": float((known & reachable).sum() / reachable.sum()),
        "approx_reachable_floor_free": float((free & reachable).sum() / reachable.sum()),
        "approx_indoor_non_obstacle_observed": float((known & indoor).sum() / indoor.sum()),
        "approx_indoor_non_obstacle_free": float((free & indoor).sum() / indoor.sum()),
    },
    "geometry": {},
    "diagnostics": export["diagnostics"],
    "dropped_scans": export["dropped_scans"],
    "last_scan_stamp": export["scan_stamp"],
    "unavailable": ["Pose RMSE and trajectory", "Actual speed and angular velocity",
                    "Latency distribution and final queue length", "Per-scan acquisition time"],
}
reported = HERE / "teleop_limits.json"
if reported.exists():
    results["user_reported_teleop_limits"] = json.loads(reported.read_text())
for tolerance in (.05, .10, .15):
    cells = int(round(tolerance / resolution))
    results["geometry"][str(tolerance)] = {
        "tolerance_m": tolerance,
        "occupied_precision_approx": float((occupied & evaluation.dilate(reference, cells)).sum()
                                           / occupied.sum()),
        "reachable_wall_surface_recall_approx": float((surface & evaluation.dilate(occupied, cells)).sum()
                                                      / surface.sum()),
    }

results["long_wall_fits"] = {}
for name, expected_x, y0, y1 in [("left", -5.03, -10.7, 6.8),
                                ("right", 4.24, -10.7, 9.9)]:
    selected = occupied & (np.abs(xx - expected_x) < .35) & (yy >= y0) & (yy <= y1)
    x, y = xx[selected], yy[selected]
    slope, intercept = np.polyfit(y, x, 1)
    residual = x - (slope*y + intercept)
    results["long_wall_fits"][name] = {
        "cells": len(x), "angle_from_vertical_deg": float(np.degrees(np.arctan(slope))),
        "mean_offset_from_sdf_m": float(np.mean(x - expected_x)),
        "p95_absolute_residual_to_fitted_line_m": float(np.percentile(np.abs(residual), 95)),
    }

results["connectivity"] = {}
for label, mask in [("occupied_inflation_unknown_blocked_at_center",
                     free & ~evaluation.dilate(occupied, radius)),
                    ("occupied_and_unknown_inflation",
                     free & ~evaluation.dilate(~free, radius))]:
    component = evaluation.connected(mask, start)
    results["connectivity"][label] = {
        "radius_plus_margin_m": .27,
        "free_area_connected_to_spawn_m2": float(component.sum() * resolution**2),
        "fraction_of_candidate_navigable_cells_connected": float(component.sum() / max(mask.sum(), 1)),
        "warning": "Static connectivity only; no controller or arbitrary-goal validation",
    }

# Abertura inferior: medir el intervalo libre continuo que contiene su centro nominal.
door_rows = (yy[:, 0] >= -11.08) & (yy[:, 0] <= -10.88)
solid = occupied[door_rows].any(axis=0)
center = int((-3.61 - origin[0]) / resolution)
left = right = center
while left > 0 and not solid[left-1]:
    left -= 1
while right + 1 < len(solid) and not solid[right+1]:
    right += 1
results["lower_door"] = {"approx_nominal_width_m": 1.09,
                          "mapped_obstacle_gap_m": float((right-left+1)*resolution),
                          "band_y_m": [-11.08, -10.88],
                          "nominal_center_x_m": -3.61}

(HERE / "metrics.json").write_text(json.dumps(results, indent=2) + "\n")
extent = [origin[0], origin[0] + pixels.shape[1]*resolution,
          origin[1], origin[1] + pixels.shape[0]*resolution]
fig, axes = plt.subplots(1, 2, figsize=(12, 12), sharex=True, sharey=True)
axes[0].imshow(pixels, origin="lower", extent=extent, cmap="gray", vmin=0, vmax=255)
axes[0].set_title("Mapa manual guardado, celda 5 cm")
overlay = np.full((*pixels.shape, 3), .82)
overlay[free] = [1., 1., 1.]
overlay[reference] = [.98, .5, .08]
overlay[occupied] = [0., .42, .62]
overlay[occupied & reference] = [.1, .1, .1]
axes[1].imshow(overlay, origin="lower", extent=extent)
axes[1].set_title("Azul: mapa; naranja: referencia SDF aproximada")
for ax in axes:
    ax.plot(0, 0, "r+", markersize=8)
    ax.set(xlabel="x [m]", ylabel="y [m]", aspect="equal", xlim=(-6., 5.), ylim=(-16., 13.))
fig.tight_layout()
fig.savefig(HERE / "map_comparison.png", dpi=150)
plt.close(fig)
print(json.dumps(results, indent=2))
