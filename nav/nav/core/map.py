"""Carga del mapa estático (PGM + YAML, el formato de map_server). Sin ROS.

El mapa se devuelve como una `OccupancyGrid` de `slam.core`, la misma clase que
usa el SLAM de la etapa A. Así el modelo de medición y el scan matching de
`slam.core` funcionan sobre el mapa cargado sin cambios.
"""

from pathlib import Path
import numpy as np
import yaml

from slam.core.config import MapConfig, SensorConfig
from slam.core.grid import OccupancyGrid


def _read_pgm(path: Path) -> np.ndarray:
    """Lee un PGM binario (P5) de 8 bits. Devuelve la imagen con la fila 0 arriba."""
    data = path.read_bytes()
    tokens, position = [], 0
    # Encabezado: "P5", ancho, alto y valor máximo, separados por espacios o saltos
    # de línea, con comentarios opcionales que empiezan con '#'.
    while len(tokens) < 4:
        while data[position:position + 1].isspace():
            position += 1
        if data[position:position + 1] == b"#":
            position = data.index(b"\n", position) + 1
            continue
        end = position
        while not data[end:end + 1].isspace():
            end += 1
        tokens.append(data[position:end])
        position = end
    position += 1  # un solo separador entre el encabezado y los pixeles
    if tokens[0] != b"P5" or int(tokens[3]) != 255:
        raise ValueError(f"{path}: only binary 8-bit PGM (P5, maxval 255) is supported")
    width, height = int(tokens[1]), int(tokens[2])
    pixels = np.frombuffer(data, dtype=np.uint8, offset=position)
    if len(pixels) != width * height:
        raise ValueError(f"{path}: expected {width * height} pixels, found {len(pixels)}")
    return pixels.reshape(height, width)


def load_map(yaml_path, sensor: SensorConfig = SensorConfig()) -> OccupancyGrid:
    """Carga un mapa trinario y lo devuelve como grilla de ocupación.

    Regla de map_server: un pixel p tiene ocupación (255 - p) / 255. Por encima
    de `occupied_thresh` la celda es ocupada, por debajo de `free_thresh` es
    libre y en el medio es desconocida.

    En la grilla, una celda ocupada queda con el log-odds máximo, una libre con
    el mínimo y una desconocida sin observar (`observed` en False).
    """
    yaml_path = Path(yaml_path).expanduser()
    with yaml_path.open(encoding="utf8") as handle:
        meta = yaml.safe_load(handle)
    if meta.get("negate", 0) != 0:
        raise ValueError(f"{yaml_path}: negate must be 0")
    if meta.get("mode", "trinary") != "trinary":
        raise ValueError(f"{yaml_path}: only trinary maps are supported")
    origin_x, origin_y, origin_yaw = meta["origin"]
    if origin_yaw != 0:
        raise ValueError(f"{yaml_path}: a rotated map origin is not supported")
    if not 0 < meta["free_thresh"] < meta["occupied_thresh"] < 1:
        raise ValueError(f"{yaml_path}: require 0 < free_thresh < occupied_thresh < 1")
    image = _read_pgm(yaml_path.parent / meta["image"])
    # El PGM empieza por la fila de arriba y la grilla por la y mínima.
    occupancy = (255.0 - np.flipud(image)) / 255.0
    height, width = occupancy.shape
    resolution = float(meta["resolution"])
    # OccupancyGrid calcula las celdas como ceil(metros / resolución). Restar media
    # celda evita que el redondeo de punto flotante agregue una fila o columna.
    grid = OccupancyGrid(MapConfig(resolution=resolution,
                                   width=(width - 0.5) * resolution,
                                   height=(height - 0.5) * resolution,
                                   origin_x=float(origin_x), origin_y=float(origin_y)))
    if (grid.height, grid.width) != (height, width):
        raise RuntimeError("Grid size does not match the map image")
    occupied = occupancy > meta["occupied_thresh"]
    free = occupancy < meta["free_thresh"]
    grid.log_odds[occupied] = sensor.log_odds_max
    grid.log_odds[free] = sensor.log_odds_min
    grid.observed[:] = occupied | free
    return grid


def occupied_cells(grid: OccupancyGrid) -> np.ndarray:
    """Máscara de celdas ocupadas (paredes y obstáculos del mapa)."""
    return grid.observed & (grid.log_odds > grid.prior_log_odds)


def free_cells(grid: OccupancyGrid) -> np.ndarray:
    """Máscara de celdas libres. Las desconocidas no son libres."""
    return grid.observed & (grid.log_odds < grid.prior_log_odds)
