#!/usr/bin/env python3
"""
test_gps_rtk.py

Unit tests for RtkGpsErrorModel in athena_sim.sim_gps_rtk.
"""

import math
import unittest

from athena_sim.sim_gps_rtk import (
    RtkGpsErrorModel,
    STATUS_FIX,
    STATUS_GBAS_FIX,
    STATUS_NO_FIX,
    STATUS_SBAS_FIX,
)


class TestRtkGpsErrorModel(unittest.TestCase):

    def setUp(self):
        self.datum_lat = 12.8614
        self.datum_lon = 77.4380
        self.datum_alt = 860.0

    def test_invalid_mode_raises_error(self):
        """Verify initialization with invalid mode raises ValueError."""
        with self.assertRaises(ValueError):
            RtkGpsErrorModel(mode='non_existent_mode')

    def test_rtk_fixed_mode(self):
        """Verify RTK Fixed mode produces centimeter-level covariance and GBAS status."""
        model = RtkGpsErrorModel(mode='fixed', seed=42)
        noisy_lat, noisy_lon, noisy_alt, status, cov, h_acc, v_acc = model.apply_error(
            self.datum_lat, self.datum_lon, self.datum_alt
        )

        self.assertEqual(status, STATUS_GBAS_FIX)
        self.assertAlmostEqual(h_acc, 0.02, places=3)
        self.assertAlmostEqual(cov[0], 0.02 * 0.02, places=5)
        self.assertAlmostEqual(cov[4], 0.02 * 0.02, places=5)
        self.assertAlmostEqual(cov[8], 0.04 * 0.04, places=5)

        # Coordinate shift should be very small (< 0.15m)
        d_lat_m = math.radians(noisy_lat - self.datum_lat) * 6378137.0
        self.assertLess(abs(d_lat_m), 0.15)

    def test_rtk_float_mode(self):
        """Verify RTK Float mode produces sub-meter covariance and SBAS status."""
        model = RtkGpsErrorModel(mode='float', seed=42)
        _, _, _, status, cov, h_acc, _ = model.apply_error(
            self.datum_lat, self.datum_lon, self.datum_alt
        )

        self.assertEqual(status, STATUS_SBAS_FIX)
        self.assertAlmostEqual(h_acc, 0.35, places=2)
        self.assertAlmostEqual(cov[0], 0.35 * 0.35, places=3)

    def test_standalone_mode(self):
        """Verify Standalone mode produces ~1.8m standard GNSS accuracy and FIX status."""
        model = RtkGpsErrorModel(mode='standalone', seed=42)
        _, _, _, status, cov, h_acc, _ = model.apply_error(
            self.datum_lat, self.datum_lon, self.datum_alt
        )

        self.assertEqual(status, STATUS_FIX)
        self.assertAlmostEqual(h_acc, 1.80, places=2)
        self.assertAlmostEqual(cov[0], 1.80 * 1.80, places=2)

    def test_no_fix_mode(self):
        """Verify NO_FIX mode signals status -1 and large covariance diagonal."""
        model = RtkGpsErrorModel(mode='no_fix')
        noisy_lat, noisy_lon, _, status, cov, h_acc, _ = model.apply_error(
            self.datum_lat, self.datum_lon, self.datum_alt
        )

        self.assertEqual(status, STATUS_NO_FIX)
        self.assertEqual(noisy_lat, self.datum_lat)
        self.assertEqual(noisy_lon, self.datum_lon)
        self.assertGreaterEqual(cov[0], 1e4)

    def test_dynamic_mode_switch(self):
        """Verify dynamic mode switching at runtime."""
        model = RtkGpsErrorModel(mode='fixed')
        self.assertEqual(model.mode, 'fixed')

        model.set_mode('float')
        self.assertEqual(model.mode, 'float')

        model.set_mode('standalone')
        self.assertEqual(model.mode, 'standalone')

        model.set_mode('no_fix')
        self.assertEqual(model.mode, 'no_fix')


if __name__ == '__main__':
    unittest.main()
