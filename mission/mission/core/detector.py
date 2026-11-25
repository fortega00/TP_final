"""Detector de conos rojos en una imagen de color. Sin ROS. Decisión 016.

Visión clásica, sin aprendizaje: OpenCV solo para operaciones genéricas de
imagen (pasar a HSV, umbral, morfología, contornos). Pasos:

1. Color: se pasa a HSV, que separa el tono del brillo y hace al umbral menos
   sensible a la luz. El rojo está en los dos extremos del tono (0 y 179 en
   OpenCV), así que se usan dos rangos. Saturación y brillo mínimos descartan
   grises, blancos y sombras.
2. Limpieza: apertura (saca puntitos sueltos) y cierre (rellena huecos chicos).
3. Forma, por cada mancha: área mínima, proporción alto/ancho, que sea llena
   (área sobre envolvente convexa) y que se angoste hacia arriba, como un cono.
   Esto descarta objetos rojos que no son conos, como una mochila rectangular.

El tamaño real en metros necesita la profundidad y la calibración de la
cámara: ese filtro está en la ubicación del cono (paso 5), no acá.

Los umbrales son supuestos hasta tener imágenes del aula (`bag_conos`).
"""

from dataclasses import dataclass

import cv2
import numpy as np

from .config import DetectorConfig


@dataclass(frozen=True)
class Detection:
    """Una mancha con forma de cono rojo, en píxeles de la imagen."""
    center: tuple          # (u, v) del centroide
    bottom: tuple          # (u, v) del medio del borde de abajo: donde el cono toca el piso
    box: tuple             # (x, y, ancho, alto)
    area: int              # píxeles de la mancha
    solidity: float
    taper: float


def red_mask(image_bgr: np.ndarray, config: DetectorConfig) -> np.ndarray:
    """Máscara de píxeles rojos (255) después de limpiarla con morfología."""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    low = cv2.inRange(hsv, (0, config.sat_min, config.val_min), (config.hue_low_max, 255, 255))
    high = cv2.inRange(hsv, (config.hue_high_min, config.sat_min, config.val_min), (179, 255, 255))
    mask = cv2.bitwise_or(low, high)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (config.kernel, config.kernel))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def taper(blob: np.ndarray) -> float:
    """Ancho del tercio de arriba sobre ancho del tercio de abajo de una mancha.

    `blob` es la máscara recortada a su rectángulo. El ancho de cada tercio es
    la mediana del ancho por fila, para que una fila suelta no lo decida.
    """
    widths = np.count_nonzero(blob, axis=1)
    third = max(1, len(widths) // 3)
    top, bottom = np.median(widths[:third]), np.median(widths[-third:])
    return float(top / bottom) if bottom > 0 else float("inf")


def detect_cones(image_bgr: np.ndarray, config: DetectorConfig) -> list:
    """Manchas con forma de cono rojo, de la más grande a la más chica."""
    mask = red_mask(image_bgr, config)
    # [-2] funciona con OpenCV 3, 4 y 5, que devuelven tuplas de distinto largo.
    contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    found = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < config.min_area:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if not config.aspect_min <= h / w <= config.aspect_max:
            continue
        hull = cv2.contourArea(cv2.convexHull(contour))
        solidity = area / hull if hull > 0 else 0.0
        if solidity < config.solidity_min:
            continue
        # Solo los píxeles de esta mancha, sin otras que caigan en el mismo rectángulo.
        blob = np.zeros((h, w), dtype=np.uint8)
        cv2.drawContours(blob, [contour - (x, y)], -1, 255, thickness=cv2.FILLED)
        narrowing = taper(blob)
        if narrowing > config.taper_max:
            continue
        moments = cv2.moments(contour)
        center = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])
        found.append(Detection(center=center, bottom=(x + w / 2, y + h), box=(x, y, w, h),
                               area=int(area), solidity=float(solidity), taper=narrowing))
    return sorted(found, key=lambda d: d.area, reverse=True)
