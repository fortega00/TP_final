"""Scripts de medición: comparan trayectorias y miden repetibilidad sobre marcas."""

import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"


def run(script, *args):
    result = subprocess.run([sys.executable, str(SCRIPTS / script), *map(str, args)],
                            capture_output=True, text=True, check=True)
    return json.loads(result.stdout)


class ScriptTests(unittest.TestCase):
    def setUp(self):
        self.folder = tempfile.TemporaryDirectory()
        self.path = Path(self.folder.name)

    def tearDown(self):
        self.folder.cleanup()

    def write_localization(self, name, rows):
        (self.path / name).write_text(json.dumps(
            {"samples": [dict(stamp=t, x=x, y=y, yaw=yaw) for t, x, y, yaw in rows]}))
        return self.path / name

    def test_compare_separates_frame_offset_from_shape_error(self):
        t = np.linspace(0, 10, 50)
        reference = np.c_[t, 0.5 * t, 0.2 * t ** 2 / 10, np.zeros_like(t)]
        # La misma trayectoria girada 90 grados y corrida: otro frame, misma forma.
        moved = np.c_[t, -reference[:, 2] + 3.0, reference[:, 1] - 1.0, np.zeros_like(t)]
        a = self.write_localization("a.json", moved)
        b = self.write_localization("b.json", reference)
        result = run("compare_trajectories.py", a, b)
        self.assertGreater(result["direct"]["rmse_cm"], 100)
        self.assertLess(result["rigid_fit"]["rmse_cm"], 0.1)

    def test_marks_group_visits_and_report_their_spread(self):
        # El robot para 5 s en A, va a B, vuelve a A y vuelve a B. La odometría
        # deriva, la localización dice que la segunda visita a A cayó a 6 cm de la
        # primera y la segunda a B a 2 cm.
        stamps, odometry, estimates = [], [], []
        visits = [((0.0, 0.0), (0.00, 0.00)), ((3.0, 1.0), (3.00, 1.00)),
                  ((0.4, 0.3), (0.06, 0.00)), ((3.5, 1.2), (3.00, 1.02))]
        clock, drift = 0.0, 0.0
        for (odom_x, odom_y), (est_x, est_y) in visits:
            for _ in range(25):                       # quieto 5 s a 5 Hz
                stamps.append(clock); odometry.append([odom_x, odom_y, 0.0])
                clock += 0.2
            estimates.append([clock - 1.0, est_x, est_y, 0.0])   # última actualización, ya frenado
            for k in range(10):                       # traslado de 2 s
                stamps.append(clock); odometry.append([odom_x + 0.1 * (k + 1) + drift, odom_y, 0.0])
                clock += 0.2
        meta = np.c_[stamps, np.zeros((len(stamps), 4))]
        np.savez(self.path / "rec.npz", metadata=meta, odometry=np.array(odometry))
        loc = self.write_localization("loc.json", estimates)
        result = run("mark_repeatability.py", self.path / "rec.npz", loc)
        self.assertEqual(result["stops"], 4)
        self.assertEqual([m["visits"] for m in result["marks"]], [2, 2])
        # Dos visitas a 6 cm entre sí quedan a 3 cm de su promedio.
        self.assertAlmostEqual(result["marks"][0]["spread_cm"], 3.0, places=1)
        self.assertAlmostEqual(result["marks"][1]["spread_cm"], 1.0, places=1)


if __name__ == "__main__":
    unittest.main()
