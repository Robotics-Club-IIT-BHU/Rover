#!/usr/bin/env python3
"""
stack_check.py

One-shot health check of the whole nav stack: measures every link in the
chain and prints PASS/FAIL per stage, so you can see at a glance whether
sensing, localization, and obstacle mapping are all live.

    ros2 run athena_gps_nav stack_check
    ros2 run athena_gps_nav stack_check --ros-args -p window:=10.0

Subscribes (counts messages for `window` seconds, publishes nothing):
    /camera/camera/depth/color/points[_downsampled]  sensor_msgs/PointCloud2
    /imu/data, /gps/fix                       sensor_msgs/Imu, NavSatFix
    /vio/odometry_raw, /vio/odometry          nav_msgs/Odometry
    /odometry/local, /odometry/global, /odometry/gps   nav_msgs/Odometry
    /local_costmap/costmap, /global_costmap/costmap    nav_msgs/OccupancyGrid
    /cmd_vel                                  geometry_msgs/Twist

Chain checked:
    camera  -> depth cloud -> downsampled cloud -> costmap obstacle cells
    pixhawk -> imu + gps fix
    VIO     -> vio_gate -> EKF local (odom->base_link) -> EKF global (map->odom)
    TF tree map -> odom -> base_link -> camera_link

It exits by calling rclpy.shutdown() once the window elapses, so it always
terminates on its own; there is nothing to Ctrl-C.
"""

import math
from array import array

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import Twist
from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2

import tf2_ros

BE = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=5)
REL = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                 history=HistoryPolicy.KEEP_LAST, depth=5)
LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

# Lowest rate (Hz) at which each stream is still considered healthy.  These
# are floors, not targets: they are set well below what the hardware actually
# achieves so that a PASS means "alive", not "fast".
MIN_IMU_HZ = 30
MIN_GPS_HZ = 1
MIN_DEPTH_HZ = 3
MIN_CLOUD_DS_HZ = 1
MIN_VIO_HZ = 2
# 15, not 20.  The filters are CONFIGURED for 30 Hz and never reach it on
# this 6-core Jetson under full stack load: measured 24-26 Hz on 2026-08-31
# and 19.5 Hz on 2026-09-04.  A threshold of 20 therefore reported a FAIL on
# a perfectly healthy stack, which is worse than no check at all - a line
# that is always red gets ignored, and then a real one gets ignored with it.
#
# 15 is chosen to separate the two states that actually differ.  The genuine
# failure this check exists to catch is the Pixhawk IMU dropping out, which
# takes the filters down to 7-8 Hz because they lose their fastest input and
# fall back to VIO alone.  Healthy is 19-26, broken is 7-8; nothing observed
# has ever landed between.  Raise this again if the Jetson's CPU budget
# improves, but measure the healthy band first.
MIN_EKF_HZ = 15
MIN_NAVSAT_HZ = 1

# nav2_costmap_2d's own scale: 100 is lethal, 1..98 is inflation around it.
LETHAL_COST = 99

GRAVITY = 9.81
MAX_GRAVITY_ERROR = 0.3     # m/s2, |g| this far off means wrong units/scaling
MAX_ACCEL_RESIDUAL = 0.5    # m/s2, accel-vs-attitude mismatch -> wrong frame
MIN_IMU_SAMPLES = 20        # below this the averages are too noisy to judge


class StackCheck(Node):
    def __init__(self):
        super().__init__('stack_check')
        self.declare_parameter('window', 10.0)
        self.window = float(self.get_parameter('window').value)

        self.counts = {}
        self.extra = {}

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        def sub(topic, mtype, qos, key, cb=None):
            self.counts[key] = 0

            def handler(msg, k=key, extra_cb=cb):
                self.counts[k] += 1
                if extra_cb:
                    extra_cb(msg)
            self.create_subscription(mtype, topic, handler, qos)

        sub('/camera/camera/depth/color/points', PointCloud2, BE, 'depth_cloud')
        sub('/camera/camera/depth/color/points_downsampled', PointCloud2, BE,
            'cloud_ds', self._cloud)
        self._grids = {}          # which -> (cells, width), patched by updates
        self._imu = {k: [] for k in ('ax', 'ay', 'az', 'roll', 'pitch')}
        sub('/imu/data', Imu, REL, 'imu', self._imu_sample)
        sub('/gps/fix', NavSatFix, REL, 'gps', self._gps)
        sub('/vio/odometry_raw', Odometry, REL, 'vio_raw')
        sub('/vio/odometry', Odometry, REL, 'vio_gated')
        sub('/odometry/local', Odometry, REL, 'ekf_local')
        sub('/odometry/global', Odometry, REL, 'ekf_global')
        sub('/odometry/gps', Odometry, REL, 'navsat')
        sub('/local_costmap/costmap', OccupancyGrid, LATCHED, 'local_costmap',
            lambda m: self._costmap('local', m))
        sub('/global_costmap/costmap', OccupancyGrid, LATCHED, 'global_costmap',
            lambda m: self._costmap('global', m))
        # With always_send_full_costmap: False (the global costmap) the full
        # grid is sent once, often before any obstacle is in it, and every
        # later change arrives only on *_updates.  Reading the full grid alone
        # reported "0 lethal" FAILs on a healthy costmap (2026-10-02).
        for which in ('local', 'global'):
            self.create_subscription(
                OccupancyGridUpdate, f'/{which}_costmap/costmap_updates',
                lambda u, w=which: self._costmap_update(w, u), REL)
        sub('/cmd_vel', Twist, REL, 'cmd_vel')

        self.create_timer(self.window, self._report)
        self._done = False

    def _imu_sample(self, msg):
        a = msg.linear_acceleration
        q = msg.orientation
        self._imu['ax'].append(a.x)
        self._imu['ay'].append(a.y)
        self._imu['az'].append(a.z)
        self._imu['roll'].append(math.atan2(
            2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        self._imu['pitch'].append(math.asin(
            max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))

    def _imu_physics(self):
        """Cross-check accelerometer against reported attitude.

        At rest the accelerometer must read exactly the gravity vector
        rotated into the body frame.  If |g| is off, the units or scaling
        are wrong; if the per-axis residual is large, the orientation (or
        the NED->ENU conversion) is wrong.  This catches a mis-wired or
        mis-converted IMU that a rate check alone would call healthy.
        """
        n = len(self._imu['ax'])
        if n < MIN_IMU_SAMPLES:
            return None
        mean = {k: sum(v) / len(v) for k, v in self._imu.items()}
        g = math.sqrt(mean['ax'] ** 2 + mean['ay'] ** 2 + mean['az'] ** 2)
        roll, pitch = mean['roll'], mean['pitch']
        pred = (-math.sin(pitch) * g,
                math.sin(roll) * math.cos(pitch) * g,
                math.cos(roll) * math.cos(pitch) * g)
        resid = max(abs(pred[0] - mean['ax']),
                    abs(pred[1] - mean['ay']),
                    abs(pred[2] - mean['az']))
        return g, resid, math.degrees(roll), math.degrees(pitch)

    def _cloud(self, msg):
        self.extra['cloud_pts'] = msg.width * msg.height

    def _gps(self, msg):
        self.extra['gps_status'] = msg.status.status
        self.extra['gps_sigma'] = math.sqrt(max(msg.position_covariance[0], 0.0))

    def _costmap(self, which, msg):
        self._grids[which] = (array('b', msg.data), msg.info.width)
        self.extra[f'{which}_frame'] = msg.header.frame_id

    def _costmap_update(self, which, u):
        if which not in self._grids:
            return                  # no full grid yet to patch
        data, width = self._grids[which]
        for row in range(u.height):
            start = (u.y + row) * width + u.x
            data[start:start + u.width] = array('b', u.data[row * u.width:(row + 1) * u.width])

    def _count_costmap(self, which):
        data = self._grids[which][0]
        self.extra[f'{which}_occ'] = sum(1 for v in data if v >= LETHAL_COST)
        self.extra[f'{which}_infl'] = sum(1 for v in data if 0 < v < LETHAL_COST)

    def _tf_ok(self, parent, child):
        try:
            self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            return True
        except Exception:
            return False

    # -------------------------------------------------------------- report
    # The sections below all append to self._out via _say/_check.  They are
    # split up only so each stage of the chain reads as one block; the order
    # they are called in is the order of the printed report.
    def _say(self, text):
        self._out.append(text)

    def _check(self, ok, label, detail):
        self._out.append(f'  [{"PASS" if ok else "FAIL"}]  {label:<26s} {detail}')

    def _hz(self, key):
        return self.counts[key] / self.window

    def _report(self):
        if self._done:
            return
        self._done = True
        self._out = []

        self._say('=' * 66)
        self._say(f'Athena stack check  ({self.window:.0f} s window)')
        self._say('=' * 66)

        self._report_sensors()
        self._report_perception()
        self._report_localization()
        self._report_tf()
        self._report_nav2()

        fails = sum(1 for x in self._out if '[FAIL]' in x)
        self._say('')
        self._say('=' * 66)
        self._say(f'  {fails} FAIL' if fails else '  ALL CHECKS PASS')
        self._say('=' * 66)
        print('\n'.join(self._out), flush=True)
        rclpy.shutdown()

    def _report_sensors(self):
        e = self.extra
        self._say('\nSENSORS')
        self._check(self._hz('imu') > MIN_IMU_HZ, 'pixhawk imu',
                    f'{self._hz("imu"):5.1f} Hz')
        phys = self._imu_physics()
        if phys:
            g, resid, roll, pitch = phys
            self._check(abs(g - GRAVITY) < MAX_GRAVITY_ERROR
                        and resid < MAX_ACCEL_RESIDUAL,
                        'imu physics (at rest)',
                        f'|g|={g:.2f} m/s2, attitude-vs-accel residual '
                        f'{resid:.3f}, tilt r={roll:+.0f} p={pitch:+.0f} deg')
        gs = e.get('gps_status', -99)
        self._check(self._hz('gps') > MIN_GPS_HZ, 'pixhawk gps',
                    f'{self._hz("gps"):5.1f} Hz, '
                    f'{"3D FIX" if gs == 0 else "NO FIX (indoors?)"}'
                    + (f', sigma {e["gps_sigma"]:.1f} m'
                       if 'gps_sigma' in e else ''))
        self._check(self._hz('depth_cloud') > MIN_DEPTH_HZ,
                    'depth cloud (camera)',
                    f'{self._hz("depth_cloud"):5.1f} Hz')

    def _report_perception(self):
        e = self.extra
        self._say('\nOBSTACLE PERCEPTION  (depth camera -> costmap)')
        self._check(self._hz('cloud_ds') > MIN_CLOUD_DS_HZ,
                    'downsampled cloud',
                    f'{self._hz("cloud_ds"):5.1f} Hz, '
                    f'{e.get("cloud_pts", 0)} pts/frame')
        for which in self._grids:
            self._count_costmap(which)          # once, here: the global grid is 1e6 cells
        for which in ('local', 'global'):
            occ, infl = e.get(f'{which}_occ'), e.get(f'{which}_infl')
            self._check(occ is not None and occ > 0,
                        f'{which} costmap marking',
                        f'{occ} lethal + {infl} inflated cells'
                        if occ is not None else 'no costmap received')

    def _report_localization(self):
        c = self.counts
        self._say('\nODOMETRY / LOCALIZATION')
        self._check(self._hz('vio_raw') > MIN_VIO_HZ, 'VIO raw (rtabmap)',
                    f'{self._hz("vio_raw"):5.1f} Hz')
        dropped = c['vio_raw'] - c['vio_gated']
        self._check(self._hz('vio_gated') > MIN_VIO_HZ, 'VIO gated',
                    f'{self._hz("vio_gated"):5.1f} Hz '
                    f'({dropped} implausible dropped)')
        self._check(self._hz('ekf_local') > MIN_EKF_HZ, 'EKF local (odom)',
                    f'{self._hz("ekf_local"):5.1f} Hz')
        self._check(self._hz('ekf_global') > MIN_EKF_HZ, 'EKF global (map)',
                    f'{self._hz("ekf_global"):5.1f} Hz')
        self._check(self._hz('navsat') > MIN_NAVSAT_HZ, 'navsat gps->map',
                    f'{self._hz("navsat"):5.1f} Hz'
                    + ('' if c['navsat'] else '  (needs a GPS fix)'))

    def _report_tf(self):
        self._say('\nTF TREE')
        for parent, child in (('map', 'odom'), ('odom', 'base_link'),
                              ('base_link', 'camera_link'),
                              ('base_link', 'camera_imu_optical_frame')):
            self._check(self._tf_ok(parent, child), f'{parent} -> {child}', '')

    def _report_nav2(self):
        self._say('\nNAV2')
        # Never a FAIL: an idle /cmd_vel is the correct state with no goal.
        self._check(True, 'cmd_vel', f'{self._hz("cmd_vel"):5.1f} Hz'
                    + (' (idle - no goal sent)'
                       if self.counts['cmd_vel'] == 0 else ''))


def main(args=None):
    rclpy.init(args=args)
    node = StackCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
