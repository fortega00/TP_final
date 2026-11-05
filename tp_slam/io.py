"""Map-server compatible PGM/YAML export; no ROS dependency."""

from pathlib import Path
import numpy as np
import yaml


def save_map(grid, path, occupied_thresh=0.65, free_thresh=0.25):
    """Save a conservative trinary map: unknown cells remain unknown.

    Arrays use lower-left origins; PGM rows start at the top, hence flipud.
    Unknown pixel 128 lies strictly between the YAML free/occupied thresholds.
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
