#!/usr/bin/env python3
"""
nav_status.py. Says, in one line, why the last Nav2 goal ended.

The problem it solves
    Nav2 Humble's NavigateToPose result carries no error code (those arrived in
    Iron), so a goal that fails just stops: the operator sees the rover sit
    still and then nothing.  Clicked goals are worse, because the Foxglove 3D
    panel publishes them straight to /goal_pose, bt_navigator turns them into
    an action goal internally, and nobody is holding the result at all.

    The reason does exist, it is just scattered: the planner and the
    controller each log why they gave up, the behaviour tree log says which
    stage failed and how many recovery rounds ran, and the action status topic
    says how the goal ended.  This node joins those up for EVERY goal,
    whoever sent it (Foxglove click, goal_manager, waypoint_manager, RViz,
    a script), and states the result in plain words.

Inputs (all cheap: events, or a few messages a second)
    /navigate_to_pose/_action/status  action_msgs/GoalStatusArray   how it ended
    /behavior_tree_log                nav2_msgs/BehaviorTreeLog     recovery rounds
    /rosout                           rcl_interfaces/Log            why (filtered
                                      by node name in Python: ~6 msg/s idle)
    /goal_pose                        geometry_msgs/PoseStamped     raw clicks
    /athena/goal_status               std_msgs/String               goal_manager
    /tf_static                        tf2_msgs/TFMessage            is map->odom static?

    It deliberately does NOT subscribe to /tf (49 msg/s here: a permanent
    Python TF listener costs about a tenth of a Jetson core) or to the action
    feedback (published every BT tick, ~100 Hz).  When a goal ends it opens a
    short-lived TF listener and takes ONE local costmap message to check what
    the rover is actually looking at, then closes both again.

Output
    /athena/nav_status   std_msgs/String, latched (transient local), e.g.
        FAILED 12:08:26 goal (4.01, 0.34) base_link, 31 s: goal expired ...
    The same line goes to the terminal through the node's logger (and so to
    /rosout and the Foxglove Log tab too).

Parameters
    publish_topic   default /athena/nav_status; '' = log only (observe mode)
    recovery_rounds default 6, number_of_retries of the BT's NavigateRecovery
    footprint_x / footprint_y   default 0.65 / 0.5, half-length / half-width
    costmap_topic   default /local_costmap/costmap
    xy_goal_tolerance default 0.5 (controller_server general_goal_checker)

Author: Jashan
"""

import math
import re
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_action_status_default)

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Log
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

try:                                    # present wherever Nav2 is installed
    from nav2_msgs.msg import BehaviorTreeLog
except ImportError:                     # pragma: no cover
    BehaviorTreeLog = None


# =========================================================================
# Pure logic below this line: no ROS calls, so it can be tested against real
# log lines without a running graph (see test/test_nav_status.py).
# =========================================================================

# (logger names, pattern, evidence key).  Logger names are matched exactly:
# '/rosout' carries the node logger name, e.g. 'planner_server' or
# 'local_costmap.local_costmap'.  Humble only forwards NODE loggers to
# /rosout, so lines from bare loggers such as 'transformPoseInTargetFrame'
# or 'BehaviorTreeEngine' never arrive here; every rule below uses a line
# that does.  All strings verified against the installed 1.1.20 libraries.
_PLANNER = ('planner_server',)
_CONTROLLER = ('controller_server',)
_BT_CLIENT = ('bt_navigator_navigate_to_pose_rclcpp_node',)
_COSTMAPS = ('local_costmap.local_costmap', 'global_costmap.global_costmap',
             'local_costmap', 'global_costmap')
_LIFECYCLE = ('lifecycle_manager_navigation', 'lifecycle_manager')

RULES = [
    (_PLANNER, r'Could not transform the start or goal pose', 'goal_tf'),
    (_PLANNER, r'goal sent to the planner is off the global costmap', 'goal_off_map'),
    (_PLANNER, r'start position is off the global costmap', 'start_off_map'),
    (_PLANNER, r'failed to create plan|failed to generate a valid path', 'no_path'),
    (_PLANNER, r'Planner loop missed its desired rate.*Current loop rate is ([\d.]+) Hz',
     'slow_planner'),
    (_CONTROLLER, r'detected collision ahead', 'collision'),
    (_CONTROLLER, r'Failed to make progress', 'no_progress'),
    (_CONTROLLER, r'Unable to transform robot pose|Exception in transformPose', 'robot_tf'),
    (_CONTROLLER, r'Resulting plan has 0 poses|Received plan with zero length|'
                  r'Invalid path, Path is empty', 'empty_plan'),
    (_CONTROLLER, r'Controller patience exceeded', 'patience'),
    (_CONTROLLER, r'Control loop missed its desired rate', 'slow_controller'),
    (_BT_CLIENT, r'Timed out while waiting for action server to acknowledge '
                 r'goal request for (\w+)', 'server_timeout'),
    (_BT_CLIENT, r'"?(\w+)"? action server not available', 'server_missing'),
    (_COSTMAPS, r'observation buffer has not been updated', 'camera_stale'),
    (('bt_navigator',), r'Begin navigating from current location '
                        r'\((-?[\d.]+), (-?[\d.]+)\) to \((-?[\d.]+), (-?[\d.]+)\)', 'begin'),
    (('bt_navigator',), r'Received goal preemption request', 'preempt'),
    (('bt_navigator',), r'Client requested to cancel the goal', 'cancel_req'),
    (('bt_navigator',), r'Error loading XML|BT file not found|Exception when loading BT',
     'bt_error'),
    (_LIFECYCLE, r'Failed to bring up all requested nodes', 'nav2_down'),
    (_LIFECYCLE, r'SERVER (\w+) IS DOWN', 'nav2_down'),
    (_LIFECYCLE, r'Managed nodes are active', 'nav2_up'),
    (('teleop_mux',), r'manual control taken', 'teleop'),
    (('goal_manager',), r'^goal cancelled', 'cancel_cmd'),
]
_COMPILED = [(frozenset(names), re.compile(p), key) for names, p, key in RULES]
WATCHED_LOGGERS = frozenset(n for names, _, _ in RULES for n in names)

# Evidence that can end a goal, most specific first.  When several were seen
# in the last few seconds before an abort, the earliest in this list wins.
FATAL = ['goal_tf', 'goal_off_map', 'start_off_map', 'bt_error', 'camera_stale',
         'robot_tf', 'no_path', 'empty_plan', 'collision', 'no_progress',
         'server_timeout', 'server_missing']

SHORT = {
    'goal_tf': 'goal expired (stale TF stamp)',
    'goal_off_map': 'goal outside the planning window',
    'start_off_map': 'rover outside the global costmap',
    'bt_error': 'behaviour tree failed to load',
    'camera_stale': 'depth camera silent',
    'robot_tf': 'robot pose transform too old',
    'no_path': 'no path to the goal',
    'empty_plan': 'rover off its planned path',
    'collision': 'obstacle in the path (collision ahead)',
    'no_progress': 'rover not moving',
    'server_timeout': 'Nav2 server too slow to answer',
    'server_missing': 'Nav2 server missing',
}

STATUS_NAME = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
               GoalStatus.STATUS_CANCELED: 'CANCELED',
               GoalStatus.STATUS_ABORTED: 'ABORTED'}
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
          GoalStatus.STATUS_CANCELING)

RECOVERY_NODE = 'RecoveryActions'            # the RoundRobin, stock and Athena trees
RECOVERY_ACTION_TEXT = {                     # BT node name -> what the operator sees
    'Wait': 'waiting 5 s', 'RecoveryWait': 'waiting 5 s',
    'ClearingActions': 'clearing both costmaps', 'RecoveryClearAll': 'clearing both costmaps',
    'RecoveryClearLocal': 'clearing the local costmap',
    'Spin': 'SPINNING (stock tree!)', 'BackUp': 'BACKING UP (stock tree!)',
}


def classify(name, msg):
    """Map one /rosout line to (key, regex groups), or None."""
    if name not in WATCHED_LOGGERS:
        return None
    for names, rx, key in _COMPILED:
        if name in names:
            m = rx.search(msg)
            if m:
                return key, m.groups()
    return None


class Episode:
    """Everything known about one NavigateToPose goal while it runs."""

    def __init__(self, goal_id, t0):
        self.goal_id = goal_id
        self.t0 = t0
        self.target = None          # (x, y) as bt_navigator logged it
        self.start = None           # (x, y) of the rover in map at the start
        self.frame = None           # frame of a raw /goal_pose, else 'map'
        self.stamp = None           # its header.stamp, seconds
        self.ev = {}                # key -> dict(n, first, last, arg)
        self.rounds = 0
        # Nav2 does not reset NavigateRecovery's retry count when a goal is
        # preempted (the tree keeps ticking with the new goal), so rounds the
        # replaced goal used come off this goal's budget too.  Seen live
        # 2026-10-02: the first goal used 3, its replacement failed after 3.
        self.inherited = 0
        self.recovery = None        # text of the recovery action now running
        self.cancel_source = None
        self.replaced = False
        self.announced = set()      # trouble keys already published live

    def add(self, key, t, arg=None):
        e = self.ev.setdefault(key, {'n': 0, 'first': t, 'last': t, 'arg': arg})
        e['n'] += 1
        e['last'] = t
        if arg:
            e['arg'] = arg

    def deciding(self, t_end, window=15.0):
        """The fatal reason behind an abort: recent first, then most specific."""
        seen = [k for k in FATAL if k in self.ev]
        if not seen:
            return None
        recent = [k for k in seen if self.ev[k]['last'] >= t_end - window]
        return (recent or seen)[0]

    def others(self, but):
        parts = [f'{SHORT[k]} x{self.ev[k]["n"]}' for k in FATAL
                 if k in self.ev and k != but]
        return ('; also ' + ', '.join(parts)) if parts else ''


def fmt_goal(ep):
    if ep.target is None:
        return 'goal'
    return f'goal ({ep.target[0]:.2f}, {ep.target[1]:.2f}) {ep.frame or "map"}'


def hhmmss(t):
    return time.strftime('%H:%M:%S', time.localtime(t))


def explain_trouble(ep, key):
    """Live line, published the first time a problem shows up mid-goal."""
    e = ep.ev[key]
    if key == 'goal_tf':
        return ('TROUBLE: Nav2 can no longer read this goal: it was stamped in '
                f'\'{ep.frame or "a moving frame"}\' and that stamp has left the 10 s TF '
                'buffer. It will fail; resend it via /athena/goal_click or with the 3D '
                'display frame set to \'map\'')
    if key == 'collision':
        return ('TROUBLE: controller stopped: lethal costmap cells in the path '
                '(collision ahead); clearing the local costmap and retrying')
    if key == 'no_path':
        return 'TROUBLE: planner found no path to the goal; retrying'
    if key == 'camera_stale':
        return ('STALLED: no depth cloud for >2 s, costmap not current: planner and '
                'controller are paused until the camera returns (check the RealSense)')
    if key == 'robot_tf':
        return 'TROUBLE: robot pose transform too old (EKF lagging or CPU overload)'
    if key == 'no_progress':
        return 'TROUBLE: rover has not moved 0.3 m in 30 s; recovering'
    if key == 'server_timeout':
        return (f'TROUBLE: {e["arg"] or "a Nav2 server"} did not answer the behaviour '
                'tree in time (CPU overload); retrying')
    return None


def explain_end(ep, outcome, t_end, insp=None, xy_tol=0.5):
    """Final one-liner.  Returns (severity, text); severity 'info'|'warn'|'error'."""
    dur = max(0.0, t_end - ep.t0)
    head = f'{hhmmss(t_end)} {fmt_goal(ep)}, {dur:.0f} s'
    insp = insp or {}

    if outcome == 'SUCCEEDED':
        d = insp.get('dist_to_goal')
        d0 = None                       # rover-to-goal distance when the goal began
        if ep.target and (ep.frame or 'map') == 'base_link':
            d0 = math.hypot(*ep.target)
        elif ep.target and ep.start and (ep.frame or 'map') == 'map':
            d0 = math.hypot(ep.target[0] - ep.start[0], ep.target[1] - ep.start[1])
        if d0 is not None and dur < 1.5:
            if d0 <= xy_tol + 0.05:
                return 'info', (f'SUCCEEDED {head}: goal was already within {xy_tol} m '
                                '(xy_goal_tolerance), nothing to drive')
            return 'warn', (f'SUCCEEDED? {head}: "reached" instantly although the goal is '
                            f'{d0:.1f} m away. The goal point is blocked, so the planner '
                            'aimed at the nearest reachable point (NavFn tolerance 1.0 m), '
                            'and that point is already next to the rover')
        if d is not None and d > xy_tol + 0.15:
            return 'warn', (f'SUCCEEDED? {head}: stopped {d:.1f} m from the requested point. '
                            'The goal itself is blocked or inside inflation, so the planner '
                            'aimed at the nearest reachable point (NavFn tolerance 1.0 m)')
        tail = f' ({d:.2f} m from target)' if d is not None else ''
        return 'info', f'SUCCEEDED {head}: reached{tail}'

    if outcome == 'CANCELED':
        who = {'teleop': 'manual control taken (teleop input)',
               'cancel_cmd': 'cancel command (/athena/goal_text or goal_cancel)'}.get(
                   ep.cancel_source, 'cancelled by a client')
        key = ep.deciding(t_end)
        before = (f'; before that: {SHORT[key]} x{ep.ev[key]["n"]}{ep.others(key)}'
                  if key else '')
        return 'info', f'CANCELED {head}: {who}{before}'

    if outcome == 'REPLACED':
        key = ep.deciding(t_end)
        before = f'; it was failing: {SHORT[key]}{ep.others(key)}' if key else ''
        return 'info', f'REPLACED {head}: a newer goal took over{before}'

    # ---- ABORTED ----------------------------------------------------------
    key = ep.deciding(t_end)
    rounds = f'{ep.rounds} recovery round(s)'
    if ep.inherited:
        rounds += f' (+{ep.inherited} used by the goal it replaced)'
    if key is None:
        return 'error', (f'FAILED {head}: after {rounds}, no specific reason was logged '
                         '(see the Log tab)')
    e = ep.ev[key]
    extra = ep.others(key)
    if key == 'goal_tf':
        age = (f' {e["first"] - ep.stamp:.0f} s after it was sent'
               if ep.stamp else ' about 10 s after it was sent')
        why = (f'goal expired{age}. It was stamped in \'{ep.frame or "a moving frame"}\', '
               'Nav2 Humble re-transforms it at that stamp on every replan and TF keeps '
               'only 10 s (planner: could not transform goal). Click via /athena/goal_click, '
               'or set the 3D display frame to \'map\'')
    elif key == 'goal_off_map':
        why = ('goal is outside the planner\'s 100 x 100 m rolling window (>50 m away): '
               'send a closer intermediate goal')
    elif key == 'start_off_map':
        why = 'the rover\'s own pose is off the global costmap: localization jumped'
    elif key == 'bt_error':
        why = 'the behaviour tree could not load: check default_nav_to_pose_bt_xml'
    elif key == 'camera_stale':
        why = ('the depth camera stopped (no cloud for >2 s) and the costmap went stale: '
               'check the RealSense')
    elif key == 'robot_tf':
        why = ('robot pose transform too old: EKF stalled or the Jetson is overloaded '
               '(controller: unable to transform robot pose)')
    elif key == 'no_path':
        why = ('no path: the goal or every route to it is blocked in the global costmap '
               '(planner: failed to create plan, tolerance 1.0 m).')
    elif key == 'empty_plan':
        why = ('the rover is off its planned path (controller: plan has 0 poses): '
               'localization jumped?')
    elif key == 'collision':
        why = (f'blocked: lethal costmap cells in the rover\'s path and no detour found '
               f'(controller: collision ahead x{e["n"]}, {rounds})')
    elif key == 'no_progress':
        why = ('rover did not move 0.3 m in 30 s (controller: failed to make progress): '
               'stuck, slipping, motors not driving, or odometry frozen')
    elif key == 'server_timeout':
        why = (f'Nav2 too slow: {e["arg"] or "a server"} did not acknowledge the behaviour '
               'tree in time (CPU overload, bt_navigator default_server_timeout)')
    else:
        why = SHORT.get(key, key)

    # What the local costmap shows right now, if it was inspected.
    blocked = key in ('collision', 'no_path', 'no_progress')
    g = insp.get('goal_cell')
    if g in ('lethal', 'inscribed') and key in ('collision', 'no_path'):
        why += (' | the goal point itself is INSIDE an obstacle' if g == 'lethal' else
                ' | the goal point is within 0.5 m of an obstacle (inscribed)')
    fp = insp.get('lethal_in_footprint', 0)
    if fp and blocked:
        why += (f' | {fp} lethal cells INSIDE the rover\'s own footprint: stale costmap '
                'memory (the camera cannot see under or beside the rover), not a real obstacle')
    sw = insp.get('lethal_in_sweep', 0)
    if sw and blocked:
        why += (f' | {sw} lethal cells within the rover\'s 0.82 m turning circle: it cannot '
                'rotate in place and never reverses on its own (no rear sensor). Back it '
                'out by teleop')
    ahead = insp.get('lethal_ahead_m')
    if ahead is not None and key in ('collision', 'no_path'):
        why += f' | nearest lethal cell {ahead:.2f} m ahead of the bumper'
    return 'error', f'FAILED {head}: {why}{extra}'


# =========================================================================
# ROS node
# =========================================================================

def _yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Probe:
    """One-shot look at TF and the local costmap, then it unsubscribes.

    A permanent Python TF listener on this Jetson costs ~10 % of a core
    (/tf runs at 49 msg/s), so this exists only for the second or two after
    a goal ends.
    """

    def __init__(self, node, want_grid, done_cb, timeout=2.0):
        import tf2_ros                           # local: only needed here
        from nav_msgs.msg import OccupancyGrid
        self.node, self.done_cb, self.want_grid = node, done_cb, want_grid
        self.buf = tf2_ros.Buffer()
        self.lis = tf2_ros.TransformListener(self.buf, node)
        self.grid = None
        self.sub = None
        if want_grid:
            qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.sub = node.create_subscription(
                OccupancyGrid, node.costmap_topic, self._grid_cb, qos)
        self.deadline = time.monotonic() + timeout
        self.timer = node.create_timer(0.1, self._poll)

    def _grid_cb(self, msg):
        self.grid = msg

    def _poll(self):
        ep = self.node.closing
        frames = {'odom'}
        if ep is not None and (ep.frame or 'map') in ('map', 'odom'):
            frames.add(ep.frame or 'map')
        if self.grid is not None:
            frames.add(self.grid.header.frame_id)
        ready = all(self.buf.can_transform(f, 'base_link', rclpy.time.Time())
                    for f in frames)
        if (ready and (self.grid is not None or not self.want_grid)) or \
                time.monotonic() > self.deadline:
            self.timer.cancel()
            self.node.destroy_timer(self.timer)
            try:
                result = self._measure()
            except Exception as e:                   # never take the node down
                result = {'error': repr(e)}
            self.lis.unregister()
            if self.sub is not None:
                self.node.destroy_subscription(self.sub)
            self.done_cb(result)

    def _lookup(self, target, source):
        try:
            return self.buf.lookup_transform(target, source, rclpy.time.Time())
        except Exception:
            return None

    def _measure(self):
        n = self.node
        out = {}
        ep = n.closing
        # distance from the rover to the requested goal, in the goal's frame
        if ep is not None and ep.target is not None and (ep.frame or 'map') in ('map', 'odom'):
            tr = self._lookup(ep.frame or 'map', 'base_link')
            if tr is not None:
                t = tr.transform.translation
                out['dist_to_goal'] = math.hypot(ep.target[0] - t.x, ep.target[1] - t.y)
        g = self.grid
        rob = self._lookup(g.header.frame_id if g else 'odom', 'base_link')
        if g is None or rob is None:
            return out
        import numpy as np
        w, h, res = g.info.width, g.info.height, g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        data = np.asarray(g.data, dtype=np.int8).reshape(h, w)
        rx, ry = rob.transform.translation.x, rob.transform.translation.y
        yaw = _yaw(rob.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        # cells within 2 m of the rover, expressed in base_link
        r = int(2.0 / res) + 1
        ci, cj = int((ry - oy) / res), int((rx - ox) / res)
        i0, i1 = max(0, ci - r), min(h, ci + r + 1)
        j0, j1 = max(0, cj - r), min(w, cj + r + 1)
        if i0 >= i1 or j0 >= j1:
            return out
        jj, ii = np.meshgrid(np.arange(j0, j1), np.arange(i0, i1))
        wx = ox + (jj + 0.5) * res - rx
        wy = oy + (ii + 0.5) * res - ry
        bx = c * wx + s * wy
        by = -s * wx + c * wy
        sub = data[i0:i1, j0:j1]
        lethal = sub >= 100                         # OccupancyGrid: 100 = LETHAL, 99 = inscribed
        fx, fy = n.fp_x, n.fp_y
        inside = (np.abs(bx) <= fx) & (np.abs(by) <= fy)
        out['lethal_in_footprint'] = int(np.count_nonzero(lethal & inside))
        # Inside the circle the corners sweep when turning on the spot, but
        # not under the rover: it cannot rotate in place, and it never backs up.
        sweep = (np.hypot(bx, by) <= math.hypot(fx, fy)) & ~inside
        out['lethal_in_sweep'] = int(np.count_nonzero(lethal & sweep))
        ahead = lethal & (bx > fx) & (bx <= fx + 1.0) & (np.abs(by) <= fy)
        if np.any(ahead):
            out['lethal_ahead_m'] = float(np.min(bx[ahead]) - fx)
        # the goal cell, if the goal is inside this 8 x 8 m window
        if ep is not None and ep.target is not None and (ep.frame or 'map') in ('map', 'odom'):
            gx, gy = ep.target
            if (ep.frame or 'map') != g.header.frame_id:
                tr = self._lookup(g.header.frame_id, ep.frame or 'map')
                if tr is not None:
                    q, t = tr.transform.rotation, tr.transform.translation
                    a = _yaw(q)
                    gx, gy = (t.x + math.cos(a) * gx - math.sin(a) * gy,
                              t.y + math.sin(a) * gx + math.cos(a) * gy)
                else:
                    gx = gy = None
            if gx is not None:
                gi, gj = int((gy - oy) / res), int((gx - ox) / res)
                if 0 <= gi < h and 0 <= gj < w:
                    v = int(data[gi, gj])
                    out['goal_cell'] = ('lethal' if v >= 100 else 'inscribed' if v == 99
                                        else 'unknown' if v < 0 else 'free')
        return out


class NavStatus(Node):

    def __init__(self):
        super().__init__('nav_status')
        self.declare_parameter('publish_topic', '/athena/nav_status')
        self.declare_parameter('recovery_rounds', 6)
        self.declare_parameter('footprint_x', 0.65)
        self.declare_parameter('footprint_y', 0.5)
        self.declare_parameter('costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('xy_goal_tolerance', 0.5)
        self.declare_parameter('inspect_costmap', True)

        topic = self.get_parameter('publish_topic').value
        self.max_rounds = int(self.get_parameter('recovery_rounds').value)
        self.fp_x = float(self.get_parameter('footprint_x').value)
        self.fp_y = float(self.get_parameter('footprint_y').value)
        self.costmap_topic = self.get_parameter('costmap_topic').value
        self.xy_tol = float(self.get_parameter('xy_goal_tolerance').value)
        self.inspect = bool(self.get_parameter('inspect_costmap').value)

        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(String, topic, latched) if topic else None

        self.status = {}            # goal_id -> last status code
        self.primed = False         # first status message seen
        self.t_start = self._now()
        self.ep = None              # Episode of the running goal
        self.closing = None         # Episode being finalised by a Probe
        self.pending_begin = None   # (t, start, target) logged before status arrived
        self.raw_goal = None        # (t_wall, PoseStamped) last /goal_pose
        self.last_text = None
        self.map_odom_static = False
        self.last_teleop = self.last_cancel_cmd = -1e9

        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self._status_cb, qos_profile_action_status_default)
        rosout_qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(Log, '/rosout', self._rosout_cb, rosout_qos)
        if BehaviorTreeLog is not None:
            self.create_subscription(BehaviorTreeLog, '/behavior_tree_log', self._bt_cb, 20)
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_pose_cb, 10)
        self.create_subscription(String, '/athena/goal_status', self._gm_cb, 10)
        static_qos = QoSProfile(depth=100, history=HistoryPolicy.KEEP_LAST,
                                reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, '/tf_static', self._static_cb, static_qos)

        self._say('info', 'IDLE: watching Nav2 goals (clicked, typed, waypoint, any client)')

    # ------------------------------------------------------------- output
    def _say(self, sev, text):
        if text == self.last_text:
            return
        self.last_text = text
        # One call site per severity: rclpy refuses a call site whose
        # severity changes between calls.
        if sev == 'error':
            self.get_logger().error(text)
        elif sev == 'warn':
            self.get_logger().warning(text)
        else:
            self.get_logger().info(text)
        if self.pub is not None:
            self.pub.publish(String(data=text))

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------- inputs
    def _static_cb(self, msg):
        for t in msg.transforms:
            if t.header.frame_id.lstrip('/') == 'map' and t.child_frame_id.lstrip('/') == 'odom':
                self.map_odom_static = True

    def _goal_pose_cb(self, msg):
        """A click (or waypoint_manager) going straight to bt_navigator."""
        now = self._now()
        self.raw_goal = (now, msg)
        frame = msg.header.frame_id.lstrip('/') or 'map'
        moving = frame not in ('map',) and not (frame == 'odom' and self.map_odom_static)
        if moving:
            self._say('warn', f'WARNING {hhmmss(now)}: a goal arrived on /goal_pose stamped in '
                      f'\'{frame}\'. Nav2 Humble re-reads it at that stamp on every replan, '
                      'so it fails ~10 s later unless reached by then. Use /athena/goal_click '
                      'or the 3D display frame \'map\'')
        self._once(3.0, lambda: self._check_accepted(now))

    def _once(self, delay, fn):
        """rclpy (Humble) has no one-shot timer: cancel it on its first call."""
        holder = {}

        def fire():
            holder['t'].cancel()
            self.destroy_timer(holder['t'])
            fn()
        holder['t'] = self.create_timer(delay, fire)

    def _check_accepted(self, t_click):
        if self.ep is not None and self.ep.t0 >= t_click - 0.5:
            return                                   # accepted, all good
        if self.count_publishers('/navigate_to_pose/_action/status') == 0:
            self._say('error', f'IGNORED {hhmmss(t_click)}: the clicked goal never reached Nav2: '
                      'the navigate_to_pose server is not running (Nav2 failed to start? '
                      'check lifecycle_manager in the Log tab)')
        else:
            # Seen but not started can be a real rejection OR a /goal_pose
            # message bt_navigator never received (a publisher that sent
            # before discovery finished); from here the two look identical.
            self._say('error', f'REJECTED {hhmmss(t_click)}: Nav2 did not start the clicked '
                      'goal within 3 s. Either bt_navigator is not active (still starting, '
                      'or a Nav2 server crashed and lifecycle_manager deactivated the '
                      'stack), or the goal message never reached it: send it again')

    def _gm_cb(self, msg):
        """goal_manager's own replies: mirror refusals so one tab tells all."""
        t = msg.data.strip()
        low = t.lower()
        if low.startswith('refused') or 'rejected' in low or 'unavailable' in low or \
                'not available' in low or low.startswith('could not'):
            sev = 'error' if 'rejected' in low else 'warn'
            label = 'REJECTED' if 'rejected' in low else 'REFUSED'
            self._say(sev, f'{label} {hhmmss(self._now())} (goal_manager): {t}')

    def _rosout_cb(self, msg):
        if msg.name not in WATCHED_LOGGERS:          # cheap reject for ~all traffic
            return
        hit = classify(msg.name, msg.msg)
        if hit is None:
            return
        key, groups = hit
        now = self._now()
        if key == 'begin':
            a, b, c, d = (float(x) for x in groups)
            self.pending_begin = (now, (a, b), (c, d))
            if self.ep is not None and self.ep.target is None and now - self.ep.t0 < 3.0:
                self._attach_begin(self.ep)
            return
        if key == 'nav2_down':
            what = f'{groups[0]} stopped responding (heartbeat lost; CPU overload?)' \
                if groups else 'bring-up aborted (a server failed to configure)'
            self._say('error', f'NAV2 DOWN {hhmmss(now)}: {what}. Nav2 is inactive: every goal '
                      'will be rejected until navigation is restarted')
            return
        if key == 'nav2_up':
            self._say('info', f'READY {hhmmss(now)}: Nav2 is active')
            return
        if key == 'teleop':
            self.last_teleop = now
            return
        if key == 'cancel_cmd':
            self.last_cancel_cmd = now
            return
        ep = self.ep
        if ep is None:
            if key == 'camera_stale':
                self._say('warn', 'CAMERA: no depth cloud for >2 s: Nav2 will not plan or drive '
                          'until it returns (check the RealSense)')
            return
        if key in ('preempt', 'cancel_req'):
            return
        ep.add(key, now, groups[0] if groups else None)
        if key in FATAL and key not in ep.announced:
            ep.announced.add(key)
            text = explain_trouble(ep, key)
            if text:
                self._say('warn', text)

    def _bt_cb(self, msg):
        ep = self.ep
        if ep is None:
            return
        # A parent's transition is logged after its child's, so count the
        # round first and describe the action afterwards.
        action = None
        for e in msg.event_log:
            name = e.node_name
            if name == RECOVERY_NODE and e.previous_status == 'IDLE' and \
                    e.current_status != 'IDLE':
                ep.rounds += 1
            elif name in RECOVERY_ACTION_TEXT and e.current_status == 'RUNNING':
                action = RECOVERY_ACTION_TEXT[name]
        if action:
            ep.recovery = action
            key = ep.deciding(self._now())
            cause = f' | cause: {SHORT[key]} x{ep.ev[key]["n"]}' if key else ''
            used = ep.inherited + max(ep.rounds, 1)
            shared = f' ({ep.inherited} used by the replaced goal)' if ep.inherited else ''
            self._say('warn', f'RECOVERING {used}/{self.max_rounds}{shared}: '
                      f'{action}{cause}')

    def _status_cb(self, msg):
        now = self._now()
        # Only the latched message delivered on connect is history.  If no
        # goal ever ran, the first message is a live one and must count.
        first = not self.primed and now - self.t_start < 2.0
        self.primed = True
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid).hex()
            prev = self.status.get(gid)
            self.status[gid] = st.status
            if prev == st.status:
                continue
            if first:                                # history from before we started
                if st.status in ACTIVE:
                    self._start(gid, now, True)
                continue
            if st.status in ACTIVE:
                if prev not in ACTIVE:
                    self._start(gid, now, False)
            elif st.status in STATUS_NAME:
                if prev is None:                     # whole life between two messages
                    self._start(gid, now, False)
                self._end(gid, STATUS_NAME[st.status], now)
        if len(self.status) > 200:                   # rcl_action keeps 15 min of goals
            live = {bytes(s.goal_info.goal_id.uuid).hex() for s in msg.status_list}
            self.status = {k: v for k, v in self.status.items() if k in live}

    # ------------------------------------------------------------- episodes
    def _attach_begin(self, ep):
        t, start, target = self.pending_begin
        ep.start, ep.target = start, target
        self.pending_begin = None

    def _start(self, gid, now, already_running):
        inherited = 0
        if self.ep is not None and self.ep.goal_id != gid:
            inherited = self.ep.inherited + self.ep.rounds
            self.ep.replaced = True                  # Humble aborts a preempted goal
            self._end(self.ep.goal_id, 'ABORTED', now)
        ep = Episode(gid, now)
        ep.inherited = inherited
        if self.pending_begin and now - self.pending_begin[0] < 3.0:
            self._attach_begin(ep)
        if self.raw_goal and now - self.raw_goal[0] < 3.0:
            msg = self.raw_goal[1]
            ep.frame = msg.header.frame_id.lstrip('/') or 'map'
            ep.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        else:
            ep.frame = 'map'                         # goal_manager, waypoints, relay
        self.ep = ep
        where = ''
        if ep.target:
            where = f' to ({ep.target[0]:.2f}, {ep.target[1]:.2f}) {ep.frame}'
        self._say('info', f'ACTIVE {hhmmss(now)}: navigating{where}'
                  + (' (already running when nav_status started)' if already_running else ''))

    def _end(self, gid, outcome, now):
        ep = self.ep
        if ep is None or ep.goal_id != gid:
            return
        self.ep = None
        if ep.replaced and outcome == 'ABORTED':
            outcome = 'REPLACED'
        if outcome == 'CANCELED':
            if now - self.last_teleop < 3.0:
                ep.cancel_source = 'teleop'
            elif now - self.last_cancel_cmd < 3.0:
                ep.cancel_source = 'cancel_cmd'
        sev, text = explain_end(ep, outcome, now, None, self.xy_tol)
        self._say(sev, text)
        # Then look at the world once and refine the line if that adds anything.
        if outcome in ('ABORTED', 'SUCCEEDED') and self.inspect:
            def done(insp, ep=ep, outcome=outcome, now=now):
                self.closing = None
                if insp.get('error'):
                    self.get_logger().debug(f'probe failed: {insp["error"]}')
                    return
                sev2, text2 = explain_end(ep, outcome, now, insp, self.xy_tol)
                if text2 != text and self.ep is None:      # a new goal outranks this
                    self._say(sev2, text2)
            self.closing = ep
            Probe(self, outcome == 'ABORTED', done)


def main(args=None):
    rclpy.init(args=args)
    node = NavStatus()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
