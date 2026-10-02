"""nav_status parsing and wording, tested against real Athena log lines.

Every line below was copied from ~/.ros/log on the rover (date noted), except
the camera_stale one, which is built from the format string in the installed
nav2_costmap_2d 1.1.20 library because that warning has never fired here yet.

    python3 -m pytest athena_remote/test/test_nav_status.py
"""
import pytest

from athena_remote.nav_status import Episode, classify, explain_end

REAL = [
    # (logger name, message, expected key, expected groups)
    ('planner_server', 'Could not transform the start or goal pose in the costmap frame',
     'goal_tf', ()),                                                            # 2026-10-02
    ('planner_server', 'GridBased: failed to create plan with tolerance 1.00.', 'no_path', ()),
    ('planner_server', 'Planning algorithm GridBased failed to generate a valid path to '
                       '(-2.73, 0.78)', 'no_path', ()),                         # 2026-08-31
    ('planner_server', 'The goal sent to the planner is off the global costmap. Planning '
                       'will always fail to this goal.', 'goal_off_map', ()),   # 2026-09-02
    ('planner_server', 'Planner loop missed its desired rate of 10.0000 Hz. Current loop '
                       'rate is 2.0886 Hz', 'slow_planner', ('2.0886',)),       # 2026-10-02
    ('controller_server', 'RegulatedPurePursuitController detected collision ahead!',
     'collision', ()),                                                          # 2026-10-02
    ('controller_server', 'Controller patience exceeded', 'patience', ()),
    ('controller_server', 'Failed to make progress', 'no_progress', ()),        # 2026-09-04
    ('controller_server', "Unable to transform robot pose into global plan's frame",
     'robot_tf', ()),                                                           # 2026-08-31
    ('controller_server', 'Resulting plan has 0 poses in it.', 'empty_plan', ()),
    ('bt_navigator_navigate_to_pose_rclcpp_node',
     'Timed out while waiting for action server to acknowledge goal request for '
     'compute_path_to_pose', 'server_timeout', ('compute_path_to_pose',)),       # 2026-10-02
    ('bt_navigator', 'Begin navigating from current location (0.68, 0.27) to (4.01, 0.34)',
     'begin', ('0.68', '0.27', '4.01', '0.34')),
    ('bt_navigator', 'Begin navigating from current location (2.40, 0.46) to (-2.63, -0.42)',
     'begin', ('2.40', '0.46', '-2.63', '-0.42')),
    ('bt_navigator', 'Received goal preemption request', 'preempt', ()),
    ('lifecycle_manager_navigation', '\x1b[34m\x1b[1mManaged nodes are active\x1b[0m\x1b[0m',
     'nav2_up', ()),
    ('lifecycle_manager_navigation', 'CRITICAL FAILURE: SERVER bt_navigator IS DOWN after not '
     'receiving a heartbeat for 4000 ms. Shutting down related nodes.', 'nav2_down',
     ('bt_navigator',)),                                                        # 2026-08-27
    ('lifecycle_manager_navigation', 'Failed to bring up all requested nodes. Aborting bringup.',
     'nav2_down', ()),                                                          # 2026-09-02
    ('local_costmap.local_costmap', 'The /camera/camera/depth/color/points_downsampled '
     'observation buffer has not been updated for 2.51 seconds, and it should be updated '
     'every 2.00 seconds.', 'camera_stale', ()),
    ('teleop_mux', 'manual control taken', 'teleop', ()),
]


@pytest.mark.parametrize('name,msg,key,groups', REAL)
def test_classify_real_lines(name, msg, key, groups):
    assert classify(name, msg) == (key, groups)


@pytest.mark.parametrize('name,msg', [
    # Humble forwards only NODE loggers to /rosout; these never arrive, and
    # must not match even if they did.
    ('transformPoseInTargetFrame', 'Extrapolation Error looking up target frame: Lookup would '
     'require extrapolation into the past.'),
    ('BehaviorTreeEngine', 'Behavior Tree tick rate 100.00 was exceeded!'),
    ('rgbd_odometry', 'Odom: quality=172, std dev=0.016281m|0.017689rad'),
    ('controller_server', 'Passing new path to controller.'),
    ('bt_navigator_navigate_to_pose_rclcpp_node', 'Failed to get result for follow_path in '
     'node halt!'),
])
def test_ignored_lines(name, msg):
    assert classify(name, msg) is None


def _episode(target, frame='map', start=(0.0, 0.0), t0=100.0):
    ep = Episode('g', t0)
    ep.start, ep.target, ep.frame = start, target, frame
    return ep


def test_stale_stamp_goal_2026_10_02():
    # 12:07:54 goal 4.01, 0.34 in base_link: planner TF failures from +10.7 s,
    # three waits, abort at +31.3 s
    ep = _episode((4.01, 0.34), 'base_link', (0.68, 0.27), t0=1790923074.86)
    ep.stamp = 1790923073.893
    for t in (1790923084.6, 1790923085.2, 1790923106.1):
        ep.add('goal_tf', t)
    ep.rounds = 6
    sev, text = explain_end(ep, 'ABORTED', 1790923106.18)
    assert sev == 'error'
    assert text.startswith('FAILED ')
    assert "goal expired 11 s after it was sent" in text
    assert "'base_link'" in text and '/athena/goal_click' in text


def test_preempted_goal_reads_replaced_not_failed():
    ep = _episode((2.07, -0.38), 'base_link', t0=10.0)
    ep.add('goal_tf', 25.0)
    sev, text = explain_end(ep, 'REPLACED', 29.0)
    assert text.startswith('REPLACED ') and 'goal expired' in text and sev == 'info'


def test_goal_inside_tolerance_is_called_out():
    sev, text = explain_end(_episode((0.30, 0.10)), 'SUCCEEDED', 100.1)
    assert 'already within 0.5 m' in text


def test_instant_success_far_away_is_suspicious_2026_08_31():
    # 02:49:53: "Reached the goal!" 1.5 ms after a 1.16 m goal was accepted
    sev, text = explain_end(_episode((1.13, 0.25)), 'SUCCEEDED', 100.08)
    assert sev == 'warn' and text.startswith('SUCCEEDED?') and '1.2 m away' in text


def test_success_short_of_goal_from_probe():
    sev, text = explain_end(_episode((5.0, 0.0)), 'SUCCEEDED', 130.0, {'dist_to_goal': 1.4})
    assert 'stopped 1.4 m from the requested point' in text


def test_collision_with_residue_under_rover():
    ep = _episode((3.0, 0.0))
    for t in (101, 102, 103):
        ep.add('collision', t)
    ep.add('patience', 103)
    ep.rounds = 6
    sev, text = explain_end(ep, 'ABORTED', 110, {'lethal_in_footprint': 37,
                                                  'lethal_ahead_m': 0.15})
    assert 'collision ahead x3' in text
    assert "37 lethal cells INSIDE the rover's own footprint" in text
    assert '0.15 m ahead of the bumper' in text


def test_goal_inside_obstacle():
    ep = _episode((2.0, 0.0))
    ep.add('no_path', 105)
    sev, text = explain_end(ep, 'ABORTED', 110, {'goal_cell': 'lethal'})
    assert 'no path' in text and 'goal point itself is INSIDE an obstacle' in text


def test_trapped_cannot_turn_bench_2026_10_02():
    # bench T2: rover parked beside the pillar, every goal "collision ahead"
    ep = _episode((-1.19, -1.69))
    for t in range(101, 120):
        ep.add('collision', t)
    sev, text = explain_end(ep, 'ABORTED', 121, {'lethal_in_footprint': 0,
                                                  'lethal_in_sweep': 12,
                                                  'lethal_ahead_m': 0.47})
    assert '0.82 m turning circle' in text and 'Back it out by teleop' in text


def test_recent_reason_beats_old_one():
    ep = _episode((3.0, 0.0))
    ep.add('collision', 101)            # early, recovered
    ep.add('no_progress', 160)          # what actually killed it
    sev, text = explain_end(ep, 'ABORTED', 170)
    assert 'did not move 0.3 m in 30 s' in text and 'also obstacle in the path' in text


def test_no_evidence():
    sev, text = explain_end(_episode((3.0, 0.0)), 'ABORTED', 140)
    assert 'no specific reason' in text


def test_rounds_used_by_a_replaced_goal_are_reported_2026_10_02():
    # Live, dry run: goal A used 3 recovery rounds, goal B preempted it and
    # failed after 3 more, because NavigateRecovery's count is not reset.
    ep = _episode((4.0, 0.0))
    ep.rounds, ep.inherited = 3, 3
    sev, text = explain_end(ep, 'ABORTED', 140)
    assert '3 recovery round(s) (+3 used by the goal it replaced)' in text


def test_cancel_by_teleop():
    ep = _episode((3.0, 0.0))
    ep.cancel_source = 'teleop'
    ep.add('collision', 102)
    sev, text = explain_end(ep, 'CANCELED', 105)
    assert 'manual control taken' in text and 'collision' in text
