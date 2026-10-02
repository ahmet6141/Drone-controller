"""tools/tof8x8.py — 8×8 ToF avuç analizi testleri (sentetik karelerle)."""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

import tof8x8  # noqa: E402


class TestGeometry(unittest.TestCase):
    def test_zone_tangents_symmetric(self):
        tan = tof8x8.zone_tangents(45.0)
        self.assertEqual(len(tan), 8)
        for i in range(4):
            self.assertAlmostEqual(tan[i], -tan[7 - i])

    def test_radial_and_perpendicular_modes_agree(self):
        for radial in (True, False):
            frame = tof8x8.synthetic_frame(palm_distance_m=0.30, radial=radial)
            est = tof8x8.analyze(frame, radial=radial)
            self.assertTrue(est.found)
            self.assertAlmostEqual(est.distance_m, 0.30, delta=0.005)

    def test_plane_fit_recovers_tilt(self):
        pts = [(x * 0.01, y * 0.01, 0.2 + 0.5 * x * 0.01) for x in range(-3, 4) for y in range(-3, 4)]
        a, b, c, rms = tof8x8.fit_plane(pts)
        self.assertAlmostEqual(a, 0.5)
        self.assertAlmostEqual(b, 0.0)
        self.assertAlmostEqual(c, 0.2)
        self.assertLess(rms, 1e-9)


class TestPalmDetection(unittest.TestCase):
    def test_centered_palm(self):
        est = tof8x8.analyze(tof8x8.synthetic_frame(palm_distance_m=0.40))
        self.assertTrue(est.found)
        self.assertGreaterEqual(est.zones, 2)
        self.assertAlmostEqual(est.distance_m, 0.40, delta=0.01)
        self.assertAlmostEqual(est.offset_x_m, 0.0, delta=0.01)
        self.assertAlmostEqual(est.offset_y_m, 0.0, delta=0.01)
        self.assertFalse(est.contact)

    def test_offset_palm_gives_lateral_error(self):
        est = tof8x8.analyze(tof8x8.synthetic_frame(palm_distance_m=0.40, palm_center_m=(0.06, -0.04)))
        self.assertTrue(est.found)
        self.assertGreater(est.offset_x_m, 0.03)
        self.assertLess(est.offset_y_m, -0.01)

    def test_tilted_palm(self):
        est = tof8x8.analyze(tof8x8.synthetic_frame(palm_distance_m=0.15, tilt_deg=20.0))
        self.assertTrue(est.found)
        self.assertAlmostEqual(est.tilt_deg, 20.0, delta=2.0)
        self.assertLess(est.flatness_rms_m, 0.005)

    def test_floor_only_is_not_a_palm(self):
        # zemin yakında (0.9 m) olsa bile tek yüzey → avuç değil
        frame = tof8x8.synthetic_frame(ground_m=0.9, with_palm=False)
        self.assertFalse(tof8x8.analyze(frame, ground_m=0.9).found)
        self.assertFalse(tof8x8.analyze(frame).found)          # zemin bilinmiyorsa da reddet

    def test_contact(self):
        est = tof8x8.analyze(tof8x8.synthetic_frame(palm_distance_m=0.03), ground_m=1.1)
        self.assertTrue(est.found)
        self.assertTrue(est.contact)
        self.assertEqual(est.zones, 64)

    def test_invalid_zones_are_ignored(self):
        frame = tof8x8.synthetic_frame(palm_distance_m=0.25)
        for r, c in ((0, 0), (0, 7), (7, 0), (2, 5)):
            frame[r][c] = None
        est = tof8x8.analyze(frame)
        self.assertTrue(est.found)
        self.assertAlmostEqual(est.distance_m, 0.25, delta=0.01)

    def test_too_few_valid_zones(self):
        frame = [[None] * 8 for _ in range(8)]
        frame[3][3] = 400
        self.assertFalse(tof8x8.analyze(frame).found)

    def test_wide_fov_sensor(self):
        fov = tof8x8.FOV_DEG["vl53l7cx"]
        est = tof8x8.analyze(tof8x8.synthetic_frame(palm_distance_m=0.30, fov_deg=fov), fov_deg=fov)
        self.assertTrue(est.found)
        self.assertAlmostEqual(est.distance_m, 0.30, delta=0.01)


if __name__ == "__main__":
    unittest.main()
