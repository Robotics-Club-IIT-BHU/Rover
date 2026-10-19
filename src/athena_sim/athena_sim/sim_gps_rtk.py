#!/usr/bin/env python3
"""
sim_gps_rtk.py

Realistic RTK (Real-Time Kinematic) GNSS (Global Navigation Satellite System)
sensor simulator and error injector for the Athena rover.

Wraps raw Gazebo NavSat / ground-truth coordinates with the exact RTK error
characteristics, fix statuses, and covariance matrices used by pixhawk_bridge:
  - RTK_FIXED:      sigma ≈ 0.02 m, status GBAS (2), covariance ≈ 0.0004 m²
  - RTK_FLOAT:      sigma ≈ 0.35 m, status GBAS (2) / SBAS (1), covariance ≈ 0.12 m²
  - STANDALONE:     sigma ≈ 1.80 m, status FIX (0), HDOP-derived covariance ≈ 3.2 m²
  - NO_FIX:         status NO_FIX (-1), covariance diagonal > 1000 m²

Supports runtime degradation commands via topic/service to rehearse failover
behaviors (Fixed -> Float -> Standalone -> Outage).
"""

import math
import random
import time
from typing import Dict, List, Optional, Tuple

# WGS-84 equatorial radius in meters
R_EARTH = 6378137.0

# NavSatStatus constants matching sensor_msgs/msg/NavSatStatus
STATUS_NO_FIX = -1
STATUS_FIX = 0
STATUS_SBAS_FIX = 1
STATUS_GBAS_FIX = 2

# NavSatFix covariance types
COVARIANCE_TYPE_APPROXIMATED = 1


class RtkGpsErrorModel:
    """
    Pure Python model for RTK GNSS sensor errors and covariance matrices.
    Independent of ROS 2 runtime so it can be verified in unit tests.
    """

    MODES = ['fixed', 'float', 'standalone', 'no_fix']

    def __init__(self, mode: str = 'fixed', seed: Optional[int] = None):
        self.mode = mode.lower()
        if self.mode not in self.MODES:
            raise ValueError(f"Unknown GPS mode '{mode}'. Expected one of {self.MODES}")
        
        self._rng = random.Random(seed)
        
        # Base noise specifications (meters standard deviation)
        self.specs: Dict[str, Dict[str, float]] = {
            'fixed': {
                'h_sigma': 0.02,     # 2 cm horizontal RTK fixed floor
                'v_sigma': 0.04,     # 4 cm vertical
                'h_acc': 0.02,
                'v_acc': 0.04,
                'status': STATUS_GBAS_FIX,
            },
            'float': {
                'h_sigma': 0.35,     # 35 cm horizontal RTK float
                'v_sigma': 0.70,     # 70 cm vertical
                'h_acc': 0.35,
                'v_acc': 0.70,
                'status': STATUS_SBAS_FIX,
            },
            'standalone': {
                'h_sigma': 1.80,     # 1.8 m standard GNSS
                'v_sigma': 3.60,     # 3.6 m vertical
                'h_acc': 1.80,
                'v_acc': 3.60,
                'status': STATUS_FIX,
            },
            'no_fix': {
                'h_sigma': 25.0,     # Outage
                'v_sigma': 50.0,
                'h_acc': 99.9,
                'v_acc': 99.9,
                'status': STATUS_NO_FIX,
            },
        }

        # Multipath wander states (slow random walk)
        self._wander_east = 0.0
        self._wander_north = 0.0

    def set_mode(self, mode: str):
        mode_lower = mode.lower()
        if mode_lower not in self.MODES:
            raise ValueError(f"Unknown GPS mode '{mode}'. Expected {self.MODES}")
        self.mode = mode_lower

    def apply_error(
        self, lat: float, lon: float, alt: float
    ) -> Tuple[float, float, float, int, List[float], float, float]:
        """
        Apply GPS error model to true lat/lon/alt.
        
        Returns:
            noisy_lat (deg), noisy_lon (deg), noisy_alt (m),
            status (int), covariance (9 floats), h_acc (m), v_acc (m)
        """
        spec = self.specs[self.mode]

        # In NO_FIX mode, simulate lost satellite tracking
        if self.mode == 'no_fix':
            cov = [
                1e5, 0.0, 0.0,
                0.0, 1e5, 0.0,
                0.0, 0.0, 1e5
            ]
            return lat, lon, alt, spec['status'], cov, spec['h_acc'], spec['v_acc']

        # Horizontal error in local ENU (East-North-Up) tangent plane
        h_sig = spec['h_sigma']
        v_sig = spec['v_sigma']

        # Add Gaussian jitter + bounded multipath wander
        self._wander_east = 0.95 * self._wander_east + 0.05 * self._rng.gauss(0.0, h_sig * 0.3)
        self._wander_north = 0.95 * self._wander_north + 0.05 * self._rng.gauss(0.0, h_sig * 0.3)

        d_east = self._rng.gauss(0.0, h_sig) + self._wander_east
        d_north = self._rng.gauss(0.0, h_sig) + self._wander_north
        d_up = self._rng.gauss(0.0, v_sig)

        # Convert meters offset to latitude and longitude delta
        # d_lat = d_north / R_EARTH, d_lon = d_east / (R_EARTH * cos(lat))
        lat_rad = math.radians(lat)
        d_lat_deg = math.degrees(d_north / R_EARTH)
        d_lon_deg = math.degrees(d_east / (R_EARTH * max(math.cos(lat_rad), 1e-6)))

        noisy_lat = lat + d_lat_deg
        noisy_lon = lon + d_lon_deg
        noisy_alt = alt + d_up

        # Covariance matrix calculation matching pixhawk_bridge.py
        sh = spec['h_acc']
        sv = spec['v_acc']
        covariance = [
            sh * sh, 0.0, 0.0,
            0.0, sh * sh, 0.0,
            0.0, 0.0, sv * sv
        ]

        return noisy_lat, noisy_lon, noisy_alt, spec['status'], covariance, sh, sv


# ROS 2 Node integration (active when rclpy and messages are available)
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix, NavSatStatus
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    class SimGpsRtkNode(Node):
        """ROS 2 Node that wraps incoming Gazebo GPS fixes with RTK error models."""

        def __init__(self):
            super().__init__('sim_gps_rtk')

            self.declare_parameter('gps_mode', 'fixed')
            self.declare_parameter('datum_lat', 12.8614)
            self.declare_parameter('datum_lon', 77.4380)
            self.declare_parameter('datum_alt', 860.0)
            self.declare_parameter('frame_id', 'gps_link')

            mode = self.get_parameter('gps_mode').value
            self.frame_id = self.get_parameter('frame_id').value
            self.model = RtkGpsErrorModel(mode=mode)

            self.sub_raw = self.create_subscription(
                NavSatFix, '/gps/fix_raw', self._raw_cb, 10
            )
            self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', 10)
            self.sub_mode = self.create_subscription(
                String, '/sim/set_gps_mode', self._mode_cb, 10
            )

            self.get_logger().info(f"sim_gps_rtk node initialized in mode: '{self.model.mode}'")

        def _raw_cb(self, msg: NavSatFix):
            noisy_lat, noisy_lon, noisy_alt, status, cov, _, _ = self.model.apply_error(
                msg.latitude, msg.longitude, msg.altitude
            )

            out = NavSatFix()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.frame_id
            out.status.status = status
            out.status.service = NavSatStatus.SERVICE_GPS
            out.latitude = noisy_lat
            out.longitude = noisy_lon
            out.altitude = noisy_alt
            out.position_covariance = cov
            out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            self.pub_fix.publish(out)

        def _mode_cb(self, msg: String):
            try:
                self.model.set_mode(msg.data)
                self.get_logger().info(f"GPS mode changed to: {self.model.mode}")
            except ValueError as e:
                self.get_logger().warn(str(e))

    def main(args=None):
        rclpy.init(args=args)
        node = SimGpsRtkNode()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()

except ImportError:
    # When rclpy is not installed in the current environment
    def main(args=None):
        print("sim_gps_rtk ROS 2 node requires rclpy and sensor_msgs.")
