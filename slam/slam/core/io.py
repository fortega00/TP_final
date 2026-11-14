"""Exporta el mapa a PGM + YAML, el formato que lee el map_server de ROS. Sin ROS."""

from pathlib import Path
import numpy as np
import yaml


def save_map(grid, path, occupied_thresh=0.65, free_thresh=0.25):
    """Guarda un mapa de tres valores: libre (254), ocupado (0) y desconocido (128).

    Una celda nunca observada queda desconocida, no libre. La grilla tiene el
    origen abajo a la izquierda y el PGM empieza por la fila de arriba, por eso
    se invierte con flipud.
    """
    if not 0 < free_thresh < occupied_thresh < 1:
        raise ValueError("Require 0 < free_thresh < occupied_thresh < 1")
    path = Path(path).expanduser()
    if path.suffix.lower() in (".pgm", ".yaml", ".yml"):
        path = path.with_suffix("")
    path.parent.mkdir(parents=True, exist_ok=True)
    pgm = Path(str(path) + ".pgm")
    metadata = Path(str(path) + ".yaml")
    probabilities = 1.0 / (1.0 + np.exp(-grid.log_odds))
    pixels = np.full(grid.observed.shape, 128, dtype=np.uint8)
    pixels[grid.observed & (probabilities < free_thresh)] = 254
    pixels[grid.observed & (probabilities > occupied_thresh)] = 0
    with pgm.open("wb") as handle:
        handle.write(f"P5\n{grid.width} {grid.height}\n255\n".encode("ascii"))
        handle.write(np.flipud(pixels).tobytes())
    config = grid.config
    data = {"image": pgm.name, "mode": "trinary", "resolution": config.resolution,
            "origin": [config.origin_x, config.origin_y, 0.0], "negate": 0,
            "occupied_thresh": occupied_thresh, "free_thresh": free_thresh}
    with metadata.open("w", encoding="utf8") as handle:
        yaml.safe_dump(data, handle, sort_keys=False)
    return pgm, metadata
