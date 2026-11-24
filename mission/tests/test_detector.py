"""Detector de conos rojos sobre imágenes sintéticas. Sin ROS.

Los colores se definen en HSV de OpenCV (tono 0-179). Son una primera
comprobación de la lógica; los umbrales se ajustan con imágenes del aula.
"""

import unittest

import cv2
import numpy as np

from mission.core.config import DetectorConfig, MissionConfig
from mission.core.detector import detect_cones, red_mask, taper

WIDTH, HEIGHT = 640, 480


def bgr(hue, sat=220, val=200):
    """Color BGR a partir de un HSV de OpenCV."""
    pixel = np.uint8([[[hue, sat, val]]])
    return tuple(int(c) for c in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])


def scene(seed=0):
    """Piso gris con algo de ruido, como una foto con poca textura."""
    rng = np.random.default_rng(seed)
    image = np.full((HEIGHT, WIDTH, 3), 120, dtype=np.int16)
    image += rng.integers(-12, 13, size=image.shape, dtype=np.int16)
    return np.clip(image, 0, 255).astype(np.uint8)


def draw_cone(image, base_x, base_y, width=60, height=90, color=None):
    """Cono visto de frente: trapecio con la base apoyada en (base_x, base_y)."""
    top = width * 0.2
    points = np.array([[base_x - width / 2, base_y], [base_x + width / 2, base_y],
                       [base_x + top / 2, base_y - height], [base_x - top / 2, base_y - height]], dtype=np.int32)
    cv2.fillPoly(image, [points], color or bgr(2))
    return image


class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.config = DetectorConfig()

    def test_finds_a_red_cone_and_where_it_touches_the_floor(self):
        image = draw_cone(scene(), 320, 400)
        found = detect_cones(image, self.config)
        self.assertEqual(len(found), 1)
        u, v = found[0].bottom
        self.assertAlmostEqual(u, 320, delta=3)
        self.assertAlmostEqual(v, 400, delta=3)
        x, y, w, h = found[0].box
        self.assertTrue(x <= found[0].center[0] <= x + w and y <= found[0].center[1] <= y + h)

    def test_red_on_the_other_end_of_the_hue_circle(self):
        # El rojo da la vuelta: tono 176 también es rojo.
        found = detect_cones(draw_cone(scene(), 320, 400, color=bgr(176)), self.config)
        self.assertEqual(len(found), 1)

    def test_orange_cone_is_ignored(self):
        image = draw_cone(scene(), 320, 400, color=bgr(15))
        self.assertEqual(detect_cones(image, self.config), [])
        # Lo descarta el tono: con un rango que incluye el naranja, sí lo detectaría.
        self.assertEqual(len(detect_cones(image, DetectorConfig(hue_low_max=20))), 1)

    def test_other_colors_are_ignored(self):
        image = scene()
        for x, hue in ((100, 30), (250, 60), (400, 110), (540, 150)):   # amarillo, verde, azul, violeta
            draw_cone(image, x, 400, color=bgr(hue))
        self.assertEqual(detect_cones(image, self.config), [])

    def test_red_rectangle_is_not_a_cone(self):
        # Una mochila o una caja roja: no se angosta hacia arriba.
        image = scene()
        cv2.rectangle(image, (280, 300), (360, 400), bgr(2), thickness=cv2.FILLED)
        self.assertEqual(detect_cones(image, self.config), [])
        self.assertEqual(len(detect_cones(image, DetectorConfig(taper_max=10.0))), 1)

    def test_too_thin_is_not_a_cone(self):
        # Algo rojo que se angosta pero es demasiado alto para su ancho (un
        # poste, una pata): lo descarta la proporción, no el angostamiento.
        image = draw_cone(scene(), 320, 420, width=14, height=260)
        self.assertEqual(detect_cones(image, self.config), [])
        self.assertEqual(len(detect_cones(image, DetectorConfig(aspect_max=100.0))), 1)

    def test_tiny_red_shapes_are_ignored(self):
        # Conitos de pocos píxeles: ruido o un cono demasiado lejos.
        image = scene()
        for x in range(60, 600, 80):
            draw_cone(image, x, 300, width=10, height=12)
        self.assertEqual(detect_cones(image, self.config), [])
        self.assertGreater(len(detect_cones(image, DetectorConfig(min_area=1, kernel=1))), 0)

    def test_washed_out_or_dark_red_is_ignored(self):
        # Poca saturación (rosado, madera) o muy oscuro: no alcanza los mínimos.
        image = draw_cone(scene(), 200, 400, color=bgr(2, sat=80))
        draw_cone(image, 440, 400, color=bgr(2, val=50))
        self.assertEqual(detect_cones(image, self.config), [])

    def test_dim_but_saturated_red_is_found(self):
        found = detect_cones(draw_cone(scene(), 320, 400, color=bgr(2, val=100)), self.config)
        self.assertEqual(len(found), 1)

    def test_several_cones_biggest_first(self):
        image = draw_cone(scene(), 150, 420, width=40, height=60)
        draw_cone(image, 450, 440, width=80, height=120)
        found = detect_cones(image, self.config)
        self.assertEqual(len(found), 2)
        self.assertGreater(found[0].area, found[1].area)
        self.assertAlmostEqual(found[0].bottom[0], 450, delta=3)

    def test_red_cone_next_to_an_orange_one(self):
        image = draw_cone(scene(), 220, 400, color=bgr(15))
        draw_cone(image, 420, 400)
        found = detect_cones(image, self.config)
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0].bottom[0], 420, delta=3)

    def test_mask_is_clean_without_red(self):
        self.assertEqual(np.count_nonzero(red_mask(scene(), self.config)), 0)


class TaperTests(unittest.TestCase):
    def test_triangle_narrows_and_rectangle_does_not(self):
        triangle = np.zeros((30, 30), dtype=np.uint8)
        cv2.fillPoly(triangle, [np.array([[0, 29], [29, 29], [15, 0]], dtype=np.int32)], 255)
        self.assertLess(taper(triangle), 0.5)
        self.assertAlmostEqual(taper(np.full((30, 20), 255, dtype=np.uint8)), 1.0)


class DetectorConfigTests(unittest.TestCase):
    def test_invalid_values_are_rejected(self):
        for bad in ({"hue_low_max": 175}, {"hue_high_min": 200}, {"sat_min": 300},
                    {"kernel": 4}, {"min_area": -1}, {"aspect_min": 5.0},
                    {"solidity_min": 1.5}, {"taper_max": 0}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                MissionConfig.from_dict({"detector": bad})


if __name__ == "__main__":
    unittest.main()
