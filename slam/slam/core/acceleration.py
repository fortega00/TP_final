"""Bucles numéricos propios, con JIT opcional y alternativa NumPy equivalente."""
import math
import os

import numpy as np

try:
    if os.environ.get("SLAM_DISABLE_NUMBA") == "1":
        raise ImportError("Numba disabled by environment")
    from numba import njit
except ImportError:
    njit = None


def _lookup_loop(sensor_x, sensor_y, beam_x, beam_y, orientations,
                 origin_x, origin_y, resolution, col0, row0, field, outside):
    """Consulta las mismas celdas, sin matrices temporales de índices y máscaras."""
    result = np.empty((len(sensor_x), beam_x.shape[1]), dtype=np.float64)
    for candidate in range(len(sensor_x)):
        orientation = orientations[candidate]
        for beam in range(beam_x.shape[1]):
            col = math.floor((sensor_x[candidate] + beam_x[orientation, beam] - origin_x)
                             / resolution) - col0
            row = math.floor((sensor_y[candidate] + beam_y[orientation, beam] - origin_y)
                             / resolution) - row0
            result[candidate, beam] = (field[row, col] if 0 <= row < field.shape[0]
                                       and 0 <= col < field.shape[1] else outside)
    return result


def _lookup_numpy(sensor_x, sensor_y, beam_x, beam_y, orientations,
                  origin_x, origin_y, resolution, col0, row0, field, outside):
    cols = np.floor((sensor_x[:, None] + beam_x[orientations] - origin_x)
                    / resolution).astype(int) - col0
    rows = np.floor((sensor_y[:, None] + beam_y[orientations] - origin_y)
                    / resolution).astype(int) - row0
    inside = (cols >= 0) & (cols < field.shape[1]) & (rows >= 0) & (rows < field.shape[0])
    result = np.full(cols.shape, outside)
    result[inside] = field[rows[inside], cols[inside]]
    return result


BACKEND = "numba" if njit else "numpy"
# Sin fastmath: conservar el redondeo y la precedencia de las operaciones.
lookup_scores = njit(cache=True, nogil=True)(_lookup_loop) if njit else _lookup_numpy


def _integrate_loop(log_odds, observed, sensor_x, sensor_y, ranges, hits,
                    cosines, sines, steps, offsets, half, origin_x, origin_y,
                    resolution, width, height, free_increment, hit_increment,
                    minimum, maximum):
    """Muestrea los mismos puntos; deduplica con marcas y prevalece el impacto."""
    evidence = np.zeros(width * height, dtype=np.uint8)
    free_cells = np.empty(len(ranges) * len(steps), dtype=np.int64)
    hit_cells = np.empty(len(ranges) * len(offsets), dtype=np.int64)
    free_count = hit_count = clipped = 0
    for beam in range(len(ranges)):
        distance = ranges[beam]
        c, s = cosines[beam], sines[beam]
        end_col = math.floor((sensor_x + distance*c - origin_x) / resolution)
        end_row = math.floor((sensor_y + distance*s - origin_y) / resolution)
        endpoint_valid = 0 <= end_col < width and 0 <= end_row < height
        if not endpoint_valid:
            clipped += 1
        free_until = distance - (half if hits[beam] else 0.)
        for step in steps:
            if step >= free_until:
                break
            col = math.floor((sensor_x + step*c - origin_x) / resolution)
            row = math.floor((sensor_y + step*s - origin_y) / resolution)
            if 0 <= col < width and 0 <= row < height:
                index = row*width + col
                if evidence[index] == 0:
                    evidence[index] = 1
                    free_cells[free_count] = index
                    free_count += 1
        if hits[beam] and endpoint_valid:
            for offset in offsets:
                position = max(0., distance + offset)
                col = math.floor((sensor_x + position*c - origin_x) / resolution)
                row = math.floor((sensor_y + position*s - origin_y) / resolution)
                if 0 <= col < width and 0 <= row < height:
                    index = row*width + col
                    if evidence[index] != 2:
                        evidence[index] = 2
                        hit_cells[hit_count] = index
                        hit_count += 1
    # NumPy mantiene float32 al sumar un escalar representable a estos arrays.
    free_delta, hit_delta = np.float32(free_increment), np.float32(hit_increment)
    for n in range(free_count):
        index = free_cells[n]
        if evidence[index] == 1:
            value = np.float32(log_odds[index] + free_delta)
            log_odds[index] = min(max(value, minimum), maximum)
            observed[index] = True
    for n in range(hit_count):
        index = hit_cells[n]
        value = np.float32(log_odds[index] + hit_delta)
        log_odds[index] = min(max(value, minimum), maximum)
        observed[index] = True
    return clipped


integrate_scan = njit(cache=True, nogil=True)(_integrate_loop) if njit else None


def _nearest_loop(occupied, cols, rows, x, y, dx, dy, origin_x, origin_y,
                  resolution, maximum2):
    """Distancias exactas a centros de las mismas celdas del vecindario circular."""
    result = np.full(len(x), maximum2)
    height, width = occupied.shape
    for beam in range(len(x)):
        if not (0 <= cols[beam] < width and 0 <= rows[beam] < height):
            continue
        for offset in range(len(dx)):
            col, row = cols[beam] + dx[offset], rows[beam] + dy[offset]
            if 0 <= col < width and 0 <= row < height and occupied[row, col]:
                ex = origin_x + (col + .5)*resolution - x[beam]
                ey = origin_y + (row + .5)*resolution - y[beam]
                distance2 = ex*ex + ey*ey
                if distance2 < result[beam]:
                    result[beam] = distance2
    return result


nearest_squared = njit(cache=True, nogil=True)(_nearest_loop) if njit else None


def warmup():
    """Compila antes de recibir sensores, sin consumir RNG ni modificar mapas."""
    if njit:
        point = np.zeros(1, dtype=np.float64)
        beams = np.zeros((1, 1), dtype=np.float64)
        orientation = np.zeros(1, dtype=np.int64)
        lookup_scores(point, point, beams, beams, orientation, 0., 0., .05,
                      0, 0, beams, -1.)
        for dtype in (np.float32, np.float64):
            integrate_scan(np.zeros(1, dtype=np.float32), np.zeros(1, dtype=np.bool_),
                           0., 0., np.zeros(1, dtype=dtype), np.zeros(1, dtype=np.bool_),
                           point, point, point, point, .01, 0., 0., .05, 1, 1,
                           -.6, .8, -4., 4.)
        nearest_squared(np.zeros((1, 1), dtype=np.bool_), orientation, orientation,
                        point, point, orientation, orientation, 0., 0., .05, .36)
