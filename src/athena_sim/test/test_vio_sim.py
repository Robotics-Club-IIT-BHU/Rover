#!/usr/bin/env python3
"""
test_vio_sim.py

Unit tests for VioErrorModel in athena_sim.vio_sim.
"""

import math
import unittest

from athena_sim.vio_sim import VioErrorModel


class TestVioErrorModel(unittest.TestCase):

    def test_forward_integration(self):
        """Verify constant forward velocity integrates position forward along x-axis."""
        model = VioErrorModel(scale_error=1.0, vel_noise_std=0.0, yaw_rate_noise_std=0.0, yaw_drift_rate=0.0)
        dt = 0.1  # 100 ms

        # Step 10 times at 1.0 m/s -> should travel ~1.0 m
        for _ in range(10):
            x, y, yaw, vx, wz, qual, cov = model.step(true_vx=1.0, true_wz=0.0, dt=dt)

        self.assertAlmostEqual(x, 1.0, places=3)
        self.assertAlmostEqual(y, 0.0, places=3)
        self.assertAlmostEqual(yaw, 0.0, places=3)
        self.assertAlmostEqual(model.total_distance, 1.0, places=3)

    def test_scale_factor_bias(self):
        """Verify scale factor error inflates measured velocity."""
        scale = 1.05  # 5% scale error
        model = VioErrorModel(scale_error=scale, vel_noise_std=0.0, yaw_rate_noise_std=0.0, yaw_drift_rate=0.0)

        _, _, _, meas_vx, _, _, _ = model.step(true_vx=1.0, true_wz=0.0, dt=0.05)
        self.assertAlmostEqual(meas_vx, 1.05, places=3)

    def test_feature_quality_dropout(self):
        """Verify feature quality degradation damps velocity and expands covariance."""
        model = VioErrorModel(seed=42)
        model.set_feature_quality(1.0)
        _, _, _, _, _, _, cov_normal = model.step(1.0, 0.0, 0.05)

        # Drop feature quality to 0.1 (e.g. entering featureless sand dune)
        model.set_feature_quality(0.1)
        _, _, _, meas_vx, _, _, cov_degraded = model.step(1.0, 0.0, 0.05)

        self.assertLess(meas_vx, 0.3)
        self.assertGreater(cov_degraded[0], cov_normal[0] * 5.0)


if __name__ == '__main__':
    unittest.main()
