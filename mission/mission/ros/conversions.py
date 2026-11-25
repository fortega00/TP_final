"""Conversiones entre mensajes de ROS y lo que usa `mission.core`, sin importar ROS.

Separadas del nodo para poder probarlas sin ROS instalado: reciben números o
arreglos, no mensajes.
"""

import numpy as np

from ..core.cone_locator import Intrinsics


def transform_matrix(translation, quaternion) -> np.ndarray:
    """Matriz 4 x 4 de una traslación (x, y, z) y un cuaternión (x, y, z, w) de TF."""
    x, y, z, w = quaternion
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4)
    matrix[:3, :3] = [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]
    matrix[:3, 3] = translation
    return matrix


def depth_in_meters(depth: np.ndarray, encoding: str) -> np.ndarray:
    """Imagen de profundidad en metros (float32). Cero o NaN quedan como lecturas inválidas.

    Las cámaras Astra suelen publicar 16UC1 en milímetros; Gazebo, 32FC1 en metros.
    """
    if encoding in ("16UC1", "mono16"):
        return depth.astype(np.float32) / 1000.0
    if encoding == "32FC1":
        return depth.astype(np.float32, copy=False)
    raise ValueError(f"Unsupported depth encoding {encoding!r}")


def intrinsics_from_k(k, width: int) -> Intrinsics:
    """Calibración a partir de la matriz K de `camera_info` (9 valores, por filas)."""
    return Intrinsics(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]), width=int(width))


def match_size(depth: np.ndarray, shape) -> np.ndarray:
    """Lleva la profundidad al tamaño de la imagen de color (vecino más cercano).

    Supone que las dos imágenes cubren el mismo campo de visión (profundidad
    registrada al color). Hay que confirmarlo con la cámara del robot.
    """
    if depth.shape[:2] == tuple(shape[:2]):
        return depth
    rows = (np.arange(shape[0]) * depth.shape[0] / shape[0]).astype(int)
    cols = (np.arange(shape[1]) * depth.shape[1] / shape[1]).astype(int)
    return depth[rows][:, cols]
