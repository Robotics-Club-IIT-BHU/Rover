#!/usr/bin/env python3
"""
test_mission_runner.py

Unit tests for MissionScorer in athena_sim.mission_runner.
"""

import unittest

from athena_sim.mission_runner import MissionScorer


class TestMissionScorer(unittest.TestCase):

    def setUp(self):
        self.markers = [
            {'id': 'DROP_ALPHA', 'name': 'Marker Alpha', 'x': 20.0, 'y': 20.0},
            {'id': 'DROP_BRAVO', 'name': 'Marker Bravo', 'x': 40.0, 'y': 40.0},
        ]
        self.red_zones = [
            {'id': 'RZ_1', 'name': 'Contaminated Red Zone', 'center_x': 50.0, 'center_y': 50.0, 'radius_m': 5.0}
        ]
        self.scorer = MissionScorer(
            delivery_markers=self.markers,
            red_zones=self.red_zones,
            max_possible_points=500.0,
        )

    def test_clock_increment_and_cap(self):
        """Verify clock runs up to 25 minutes (1500s) maximum."""
        self.scorer.start_mission(now_s=1000.0)
        t = self.scorer.update_clock(now_s=1100.0)
        self.assertEqual(t, 100.0)
        self.assertTrue(self.scorer.is_running)

        # Update past 25 minutes
        t_end = self.scorer.update_clock(now_s=2600.0)  # 1600s later
        self.assertEqual(t_end, 1500.0)
        self.assertFalse(self.scorer.is_running)
        self.assertTrue(self.scorer.is_completed)

    def test_seven_minute_abort_rule(self):
        """Verify mission automatically aborts if 0 points scored after 7 minutes (420s)."""
        self.scorer.start_mission(now_s=1000.0)
        self.scorer.update_clock(now_s=1425.0)  # 425s > 420s with 0 points

        self.assertTrue(self.scorer.is_aborted)
        self.assertFalse(self.scorer.is_running)
        self.assertIn('7-minute rule', self.scorer.abort_reason)

    def test_delivery_scoring_rings(self):
        """Verify proximity rings: <0.5m=100pts, <1.0m=70pts, <2.0m=40pts."""
        self.scorer.start_mission(now_s=0.0)

        # Ring 1 (<0.5m): rover at 20.2, 20.1 -> dist ≈ 0.22m
        res1 = self.scorer.record_delivery('DROP_ALPHA', 20.2, 20.1, is_autonomous=True)
        self.assertEqual(res1['status'], 'SUCCESS')
        self.assertEqual(res1['points'], 100.0)
        self.assertEqual(self.scorer.raw_score, 100.0)

        # Duplicate delivery must be rejected
        res_dup = self.scorer.record_delivery('DROP_ALPHA', 20.1, 20.1, is_autonomous=True)
        self.assertEqual(res_dup['status'], 'DUPLICATE')

        # Ring 2 (<1.0m): rover at 40.6, 40.3 -> dist ≈ 0.67m
        # Teleoperated delivery -> 50% multiplier (70 * 0.5 = 35)
        res2 = self.scorer.record_delivery('DROP_BRAVO', 40.6, 40.3, is_autonomous=False)
        self.assertEqual(res2['status'], 'SUCCESS')
        self.assertEqual(res2['points'], 35.0)
        self.assertEqual(self.scorer.raw_score, 135.0)

    def test_red_zone_violation_penalty(self):
        """Verify red zone penetration imposes 10% penalty of max score."""
        self.scorer.start_mission(now_s=0.0)
        # Target zone center is 50, 50, radius 5
        # Test outside
        in_zone, _ = self.scorer.check_position(40.0, 40.0)
        self.assertFalse(in_zone)
        self.assertEqual(self.scorer.red_zone_violations, 0)

        # Test entry into zone (50.5, 50.5)
        in_zone, zone_name = self.scorer.check_position(50.5, 50.5)
        self.assertTrue(in_zone)
        self.assertEqual(self.scorer.red_zone_violations, 1)
        # 10% of 500 = 50.0
        self.assertEqual(self.scorer.penalties, 50.0)

    def test_intervention_limits_and_penalties(self):
        """Verify max 2 interventions with 30% penalty each."""
        self.scorer.start_mission(now_s=0.0)

        ok1, msg1 = self.scorer.request_intervention()
        self.assertTrue(ok1)
        self.assertEqual(self.scorer.interventions_used, 1)
        self.assertEqual(self.scorer.penalties, 150.0)  # 30% of 500

        ok2, msg2 = self.scorer.request_intervention()
        self.assertTrue(ok2)
        self.assertEqual(self.scorer.interventions_used, 2)
        self.assertEqual(self.scorer.penalties, 300.0)

        # 3rd intervention should be rejected
        ok3, msg3 = self.scorer.request_intervention()
        self.assertFalse(ok3)
        self.assertEqual(self.scorer.interventions_used, 2)

    def test_report_generation(self):
        """Verify report string contains breakdown, scores, and markdown formatting."""
        self.scorer.start_mission(now_s=0.0)
        self.scorer.record_delivery('DROP_ALPHA', 20.1, 20.1, is_autonomous=True)
        report = self.scorer.generate_report()
        self.assertIn('# IRC 2027 Mission Execution Report', report)
        self.assertIn('DROP_ALPHA', report)
        self.assertIn('Raw Points Earned', report)


if __name__ == '__main__':
    unittest.main()
