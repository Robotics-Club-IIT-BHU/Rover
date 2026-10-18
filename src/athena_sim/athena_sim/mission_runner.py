#!/usr/bin/env python3
"""
mission_runner.py

IRC 2027 Mission Clock, Rules Engine & Scorer.

Enforces competition rules defined in the official IRC 2027 Rulebook:
  - 25-minute mission cap (1500 s).
  - 7-minute checkpoint: abort mission if no points scored in first 420 s.
  - 15-minute checkpoint: require >30% points to unlock remaining 10 minutes.
  - RADO autonomous delivery scoring with proximity rings (<0.5m, <1.0m, <2.0m).
  - Autonomous vs. teleoperated multiplier (1.0x vs 0.5x).
  - Red contaminated no-go zone violation tracking (10% deduction per breach).
  - Human intervention simulation (max 2 interventions, 30% penalty each, start-line reset).
  - Real-time scoreboard and post-mission report generation.
"""

import math
import os
import time
from typing import Dict, List, Optional, Tuple


class MissionScorer:
    """
    Pure Python IRC 2027 rules engine and scoring calculator.
    Independent of ROS 2 runtime for unit testing and CI verification.
    """

    MAX_MISSION_TIME_S = 1500.0  # 25 minutes
    ABORT_CHECKPOINT_S = 420.0   # 7 minutes
    GATE_CHECKPOINT_S = 900.0    # 15 minutes
    MAX_INTERVENTIONS = 2

    def __init__(
        self,
        delivery_markers: Optional[List[Dict]] = None,
        red_zones: Optional[List[Dict]] = None,
        max_possible_points: float = 500.0,
    ):
        self.delivery_markers = delivery_markers or []
        self.red_zones = red_zones or []
        self.max_possible_points = max_possible_points

        # Mission state
        self.start_time: Optional[float] = None
        self.elapsed_time_s = 0.0
        self.is_running = False
        self.is_aborted = False
        self.is_completed = False
        self.abort_reason: Optional[str] = None

        # Scores and Penalties
        self.raw_score = 0.0
        self.penalties = 0.0
        self.deliveries_completed: List[Dict] = []
        self.red_zone_violations = 0
        self.interventions_used = 0

        # Inside-zone tracking to prevent double-counting continuous violations
        self._inside_red_zones = set()

    def start_mission(self, now_s: Optional[float] = None):
        """Start or reset the 25-minute competition timer."""
        self.start_time = now_s if now_s is not None else time.time()
        self.elapsed_time_s = 0.0
        self.is_running = True
        self.is_aborted = False
        self.is_completed = False
        self.abort_reason = None
        self.raw_score = 0.0
        self.penalties = 0.0
        self.deliveries_completed.clear()
        self.red_zone_violations = 0
        self.interventions_used = 0
        self._inside_red_zones.clear()

    def update_clock(self, now_s: Optional[float] = None) -> float:
        """Update elapsed mission time and evaluate rule checkpoints."""
        if not self.is_running or self.start_time is None:
            return self.elapsed_time_s

        current_time = now_s if now_s is not None else time.time()
        raw_elapsed = current_time - self.start_time

        # Cap: 25 minutes maximum
        if raw_elapsed >= self.MAX_MISSION_TIME_S:
            self.elapsed_time_s = self.MAX_MISSION_TIME_S
            self.is_running = False
            self.is_completed = True
            return self.elapsed_time_s

        self.elapsed_time_s = raw_elapsed

        # Checkpoint 1: 7-Minute Abort Rule
        if self.elapsed_time_s >= self.ABORT_CHECKPOINT_S and self.raw_score <= 0.0:
            self.is_running = False
            self.is_aborted = True
            self.abort_reason = "7-minute rule: Aborted due to 0 points scored in first 7 minutes."
            return self.elapsed_time_s

        # Checkpoint 2: 15-Minute Rule (>30% points required to proceed)
        if self.elapsed_time_s >= self.GATE_CHECKPOINT_S:
            earned_ratio = self.raw_score / max(self.max_possible_points, 1.0)
            if earned_ratio < 0.30 and self.elapsed_time_s < self.GATE_CHECKPOINT_S + 5.0:
                self.is_running = False
                self.is_aborted = True
                self.abort_reason = f"15-minute gate: Aborted. Scored {earned_ratio * 100:.1f}% (required >30%)."
                return self.elapsed_time_s

        return self.elapsed_time_s

    def check_position(self, x: float, y: float) -> Tuple[bool, Optional[str]]:
        """
        Evaluate rover position against red contaminated no-go zones.
        Returns (is_in_red_zone, zone_name).
        """
        in_any_zone = False
        active_zone_name = None

        for idx, rz in enumerate(self.red_zones):
            cx = rz.get('center_x', rz.get('x', 0.0))
            cy = rz.get('center_y', rz.get('y', 0.0))
            radius = rz.get('radius_m', 5.0)

            dist = math.hypot(x - cx, y - cy)
            if dist <= radius:
                in_any_zone = True
                active_zone_name = rz.get('name', f"RedZone_{idx}")
                if idx not in self._inside_red_zones:
                    self._inside_red_zones.add(idx)
                    self.red_zone_violations += 1
                    # 10% penalty per violation of total possible points
                    self.penalties += 0.10 * self.max_possible_points
            else:
                self._inside_red_zones.discard(idx)

        return in_any_zone, active_zone_name

    def record_delivery(
        self, marker_id: str, rover_x: float, rover_y: float, is_autonomous: bool = True
    ) -> Dict:
        """
        Score a delivery attempt against proximity rings of the target marker.
        Ring 1: < 0.5 m -> 100 pts
        Ring 2: < 1.0 m -> 70 pts
        Ring 3: < 2.0 m -> 40 pts
        Beyond: 0 pts
        Autonomous multiplier: 1.0x; Teleoperated: 0.5x
        """
        target = next((m for m in self.delivery_markers if m.get('id') == marker_id), None)
        if not target:
            return {'status': 'FAILED', 'reason': f"Unknown marker '{marker_id}'", 'points': 0.0}

        # Check if already scored
        if any(d['marker_id'] == marker_id for d in self.deliveries_completed):
            return {'status': 'DUPLICATE', 'reason': f"Marker '{marker_id}' already scored", 'points': 0.0}

        dist = math.hypot(rover_x - target['x'], rover_y - target['y'])

        if dist <= 0.5:
            base_pts = 100.0
            ring = 'Ring 1 (<0.5m)'
        elif dist <= 1.0:
            base_pts = 70.0
            ring = 'Ring 2 (<1.0m)'
        elif dist <= 2.0:
            base_pts = 40.0
            ring = 'Ring 3 (<2.0m)'
        else:
            return {'status': 'OUT_OF_RANGE', 'distance_m': dist, 'points': 0.0}

        multiplier = 1.0 if is_autonomous else 0.5
        awarded_pts = base_pts * multiplier
        self.raw_score += awarded_pts

        record = {
            'marker_id': marker_id,
            'name': target.get('name', marker_id),
            'distance_m': round(dist, 3),
            'ring': ring,
            'autonomous': is_autonomous,
            'points': awarded_pts,
            'timestamp_s': round(self.elapsed_time_s, 1),
        }
        self.deliveries_completed.append(record)
        return {'status': 'SUCCESS', **record}

    def request_intervention(self) -> Tuple[bool, str]:
        """
        Record a human intervention (max 2 allowed).
        Imposes a 30% penalty and restarts rover at the start line.
        """
        if self.interventions_used >= self.MAX_INTERVENTIONS:
            return False, f"Maximum interventions ({self.MAX_INTERVENTIONS}) already exhausted."

        self.interventions_used += 1
        penalty_amount = 0.30 * self.max_possible_points
        self.penalties += penalty_amount
        return True, f"Intervention #{self.interventions_used} recorded. 30% penalty ({penalty_amount:.1f} pts) applied."

    def get_final_score(self) -> float:
        """Calculate net final score: max(0.0, raw_score - penalties)."""
        return max(0.0, self.raw_score - self.penalties)

    def generate_report(self) -> str:
        """Generate a complete Markdown post-mission evaluation report."""
        net_score = self.get_final_score()
        mins = int(self.elapsed_time_s // 60)
        secs = int(self.elapsed_time_s % 60)

        lines = [
            "# IRC 2027 Mission Execution Report",
            "",
            f"**Execution Status**: {'ABORTED' if self.is_aborted else ('COMPLETED' if self.is_completed else 'IN PROGRESS')}  ",
            f"**Elapsed Time**: {mins:02d}:{secs:02d} / 25:00  ",
        ]
        if self.abort_reason:
            lines.append(f"**Abort Trigger**: {self.abort_reason}  ")

        lines.extend([
            "",
            "## Score Breakdown",
            "",
            f"- **Raw Points Earned**: {self.raw_score:.1f} pts",
            f"- **Total Penalties**: -{self.penalties:.1f} pts",
            f"- **Net Final Score**: **{net_score:.1f}** / {self.max_possible_points:.1f} pts",
            "",
            "## Autonomous Deliveries",
            "",
            "| Marker ID | Name | Proximity | Ring | Mode | Points |",
            "|---|---|---|---|---|---|",
        ])

        if not self.deliveries_completed:
            lines.append("| — | No deliveries completed | — | — | — | 0.0 |")
        else:
            for d in self.deliveries_completed:
                mode_str = "Autonomous (1.0x)" if d['autonomous'] else "Teleop (0.5x)"
                lines.append(f"| {d['marker_id']} | {d['name']} | {d['distance_m']:.2f} m | {d['ring']} | {mode_str} | +{d['points']:.1f} |")

        lines.extend([
            "",
            "## Penalties & Infractions",
            "",
            f"- **Red Zone Breaches**: {self.red_zone_violations} violations (-{self.red_zone_violations * 0.10 * self.max_possible_points:.1f} pts)",
            f"- **Human Interventions**: {self.interventions_used} / {self.MAX_INTERVENTIONS} used (-{self.interventions_used * 0.30 * self.max_possible_points:.1f} pts)",
            "",
        ])

        return "\n".join(lines)


# ROS 2 Node integration
try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class MissionRunnerNode(Node):
        """ROS 2 Node monitoring rover pose and scoring mission progress live."""

        def __init__(self):
            super().__init__('mission_runner')

            self.declare_parameter('zones_file', '')

            zones_file = self.get_parameter('zones_file').value
            delivery_markers = []
            red_zones = []

            if zones_file and os.path.exists(zones_file):
                import yaml
                try:
                    with open(zones_file, 'r') as f:
                        data = yaml.safe_load(f)
                        delivery_markers = data.get('delivery_markers', [])
                        red_zones = data.get('red_zones', [])
                    self.get_logger().info(f"Loaded {len(delivery_markers)} markers, {len(red_zones)} red zones from {zones_file}")
                except Exception as e:
                    self.get_logger().warn(f"Failed to load zones_file {zones_file}: {e}")

            self.scorer = MissionScorer(
                delivery_markers=delivery_markers,
                red_zones=red_zones,
                max_possible_points=500.0,
            )
            self.scorer.start_mission()

            self.sub_odom = self.create_subscription(
                Odometry, '/odometry/global', self._odom_cb, 10
            )
            self.pub_status = self.create_publisher(String, '/sim/mission_status', 10)

            self.timer = self.create_timer(1.0, self._timer_cb)
            self.get_logger().info('Mission Runner and Scorer node initialized.')

        def _odom_cb(self, msg: Odometry):
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            in_red, zone_name = self.scorer.check_position(x, y)
            if in_red:
                self.get_logger().warn(f"CONTAMINATION ALERT: Rover penetrated {zone_name}!")

        def _timer_cb(self):
            elapsed = self.scorer.update_clock()
            msg = String()
            msg.data = f"T={elapsed:.1f}s | Score={self.scorer.get_final_score():.1f} | Active={self.scorer.is_running}"
            self.pub_status.publish(msg)

    def main(args=None):
        rclpy.init(args=args)
        node = MissionRunnerNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

except ImportError:
    def main(args=None):
        print("mission_runner ROS 2 node requires rclpy and nav_msgs.")
