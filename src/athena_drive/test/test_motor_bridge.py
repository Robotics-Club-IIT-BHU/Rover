"""Unit tests for athena_drive (twist_to_pwm, curvature preservation, deadband mapping, VelocityProfile)."""

import pytest
import rclpy

from athena_drive.motor_bridge import MotorBridge
from athena_drive.velocity_profile import JerkLimiter, VelocityProfile


class MockMotorBridge:
    """Mock object providing the kinematics and configuration parameters of MotorBridge."""

    def __init__(self, track=0.40, vmax=0.70, min_pwm=40, max_pwm=255, inv_l=1, inv_r=1):
        self.track = track
        self.vmax = vmax
        self.min_pwm = min_pwm
        self.max_pwm = max_pwm
        self.inv_l = inv_l
        self.inv_r = inv_r

    twist_to_pwm = MotorBridge.twist_to_pwm


# ---------------------------------------------------------------------------
# 1. Kinematics & twist_to_pwm
# ---------------------------------------------------------------------------

def test_twist_to_pwm_zero():
    bridge = MockMotorBridge()
    left, right = bridge.twist_to_pwm(0.0, 0.0)
    assert left == 0
    assert right == 0


def test_twist_to_pwm_straight_forward():
    bridge = MockMotorBridge(track=0.40, vmax=0.70, min_pwm=40, max_pwm=255)
    # v = 0.35 m/s (half of vmax), w = 0
    # span = 255 - 40 = 215. mag = 40 + (0.35 / 0.70) * 215 = 40 + 107.5 = 147.5 -> 148
    left, right = bridge.twist_to_pwm(0.35, 0.0)
    assert left == 148
    assert right == 148


def test_twist_to_pwm_straight_backward():
    bridge = MockMotorBridge(track=0.40, vmax=0.70, min_pwm=40, max_pwm=255)
    left, right = bridge.twist_to_pwm(-0.35, 0.0)
    assert left == -148
    assert right == -148


def test_twist_to_pwm_pure_rotation():
    bridge = MockMotorBridge(track=0.50, vmax=1.00, min_pwm=40, max_pwm=255)
    # v = 0, w = 1.0 rad/s
    # vl = -1.0 * 0.50 / 2 = -0.25 m/s
    # vr = +1.0 * 0.50 / 2 = +0.25 m/s
    # span = 215, mag = 40 + 0.25 * 215 = 40 + 53.75 = 93.75 -> 94
    left, right = bridge.twist_to_pwm(0.0, 1.0)
    assert left == -94
    assert right == 94

    # Opposite rotation
    left_opp, right_opp = bridge.twist_to_pwm(0.0, -1.0)
    assert left_opp == 94
    assert right_opp == -94


def test_twist_to_pwm_inversion():
    bridge = MockMotorBridge(track=0.40, vmax=0.70, min_pwm=40, max_pwm=255, inv_l=-1, inv_r=1)
    left, right = bridge.twist_to_pwm(0.35, 0.0)
    assert left == -148
    assert right == 148

    bridge_both_inv = MockMotorBridge(track=0.40, vmax=0.70, min_pwm=40, max_pwm=255, inv_l=-1, inv_r=-1)
    left_b, right_b = bridge_both_inv.twist_to_pwm(0.35, 0.0)
    assert left_b == -148
    assert right_b == -148


# ---------------------------------------------------------------------------
# 2. Deadband Mapping
# ---------------------------------------------------------------------------

def test_deadband_cutoff():
    bridge = MockMotorBridge(min_pwm=40, max_pwm=255)
    # Velocity below 1e-3 threshold should map to 0 PWM
    left, right = bridge.twist_to_pwm(0.0005, 0.0)
    assert left == 0
    assert right == 0

    left_neg, right_neg = bridge.twist_to_pwm(-0.0005, 0.0)
    assert left_neg == 0
    assert right_neg == 0


def test_deadband_jump():
    bridge = MockMotorBridge(track=0.40, vmax=0.70, min_pwm=40, max_pwm=255)
    # Any velocity >= 1e-3 must jump to at least min_pwm (40)
    left, right = bridge.twist_to_pwm(0.002, 0.0)
    assert left >= 40
    assert right >= 40

    # Negative velocity must jump to at most -min_pwm (-40)
    left_neg, right_neg = bridge.twist_to_pwm(-0.002, 0.0)
    assert left_neg <= -40
    assert right_neg <= -40


def test_deadband_max_saturation():
    bridge = MockMotorBridge(track=0.40, vmax=0.70, min_pwm=40, max_pwm=255)
    left, right = bridge.twist_to_pwm(0.70, 0.0)
    assert left == 255
    assert right == 255

    left_neg, right_neg = bridge.twist_to_pwm(-0.70, 0.0)
    assert left_neg == -255
    assert right_neg == -255


# ---------------------------------------------------------------------------
# 3. Curvature Preservation Under Saturation
# ---------------------------------------------------------------------------

def test_curvature_preservation_under_saturation():
    """Verify that when wheel speeds saturate past vmax, both sides scale down together.

    If clipped independently, the turn radius would change.
    """
    track = 0.40
    vmax = 0.70
    min_pwm = 40
    max_pwm = 255
    bridge = MockMotorBridge(track=track, vmax=vmax, min_pwm=min_pwm, max_pwm=max_pwm)

    v = 0.60
    w = 1.00
    # Kinematics without saturation:
    # vl = 0.60 - 1.00 * 0.40 / 2 = 0.40
    # vr = 0.60 + 1.00 * 0.40 / 2 = 0.80
    # Curvature ratio vl / vr = 0.40 / 0.80 = 0.50
    # peak = 0.80 > vmax (0.70)
    # Scaled wheel speeds:
    # vl_scaled = 0.40 * (0.70 / 0.80) = 0.35
    # vr_scaled = 0.80 * (0.70 / 0.80) = 0.70
    # Scaled ratio vl_scaled / vr_scaled = 0.35 / 0.70 = 0.50 (curvature strictly preserved!)
    # Expected PWM:
    # pwm_l = 40 + (0.35 / 0.70) * 215 = 40 + 107.5 = 148
    # pwm_r = 40 + (0.70 / 0.70) * 215 = 255
    left, right = bridge.twist_to_pwm(v, w)
    assert left == 148
    assert right == 255

    # If it had clipped independently:
    # vr would clip to 0.70 (pwm 255), but vl would stay 0.40:
    # pwm_l_unscaled = 40 + (0.40 / 0.70) * 215 = 163
    # Check that left is 148, NOT 163
    assert left != 163


def test_curvature_preservation_negative_turn():
    track = 0.40
    vmax = 0.70
    bridge = MockMotorBridge(track=track, vmax=vmax, min_pwm=40, max_pwm=255)

    v = 0.60
    w = -1.00
    # vl = 0.80, vr = 0.40
    # Scaled vl = 0.70, vr = 0.35
    left, right = bridge.twist_to_pwm(v, w)
    assert left == 255
    assert right == 148


# ---------------------------------------------------------------------------
# 4. VelocityProfile and JerkLimiter
# ---------------------------------------------------------------------------

def test_jerk_limiter_limits():
    max_accel = 0.5
    max_jerk = 1.0
    limiter = JerkLimiter(max_accel=max_accel, max_jerk=max_jerk)

    dt = 0.02
    prev_a = 0.0
    worst_a = 0.0
    worst_j = 0.0

    # Accelerate to 0.5 m/s, hold, then decelerate to 0.0
    for i in range(100):
        target = 0.5 if i < 50 else 0.0
        limiter.step(target, dt)
        a = limiter.a
        j = abs(a - prev_a) / dt
        worst_a = max(worst_a, abs(a))
        worst_j = max(worst_j, j)
        prev_a = a

    assert worst_a <= max_accel + 1e-5
    assert worst_j <= max_jerk + 1e-5


def test_jerk_limiter_anti_overshoot():
    limiter = JerkLimiter(max_accel=0.5, max_jerk=1.0)
    dt = 0.05
    target = 0.4

    velocities = []
    for _ in range(50):
        v = limiter.step(target, dt)
        velocities.append(v)

    # Velocity must never exceed target
    assert all(v <= target + 1e-6 for v in velocities)
    # Velocity must settle to target
    assert pytest.approx(velocities[-1], abs=1e-3) == target

    # Now step back down to 0
    decel_velocities = []
    for _ in range(50):
        v = limiter.step(0.0, dt)
        decel_velocities.append(v)

    assert all(v >= -1e-6 for v in decel_velocities)
    assert pytest.approx(decel_velocities[-1], abs=1e-3) == 0.0


def test_jerk_limiter_dt_zero_or_negative():
    limiter = JerkLimiter(max_accel=0.5, max_jerk=1.0)
    v_orig = limiter.v
    assert limiter.step(1.0, 0.0) == v_orig
    assert limiter.step(1.0, -0.01) == v_orig


def test_jerk_limiter_reset():
    limiter = JerkLimiter(max_accel=0.5, max_jerk=1.0)
    limiter.step(0.5, 0.1)
    assert limiter.v > 0
    limiter.reset(0.2)
    assert limiter.v == 0.2
    assert limiter.a == 0.0


def test_velocity_profile_lin_and_ang():
    vp = VelocityProfile(
        max_accel=0.5, max_jerk=1.0,
        max_ang_accel=1.5, max_ang_jerk=3.0
    )
    dt = 0.05
    v, w = vp.step(0.4, 0.8, dt)
    assert v > 0.0
    assert w > 0.0
    assert vp.accel == (vp.lin.a, vp.ang.a)

    vp.reset()
    assert vp.lin.v == 0.0
    assert vp.ang.v == 0.0
    assert vp.accel == (0.0, 0.0)


# ---------------------------------------------------------------------------
# 5. MotorBridge Node Instantiation (Dry Run)
# ---------------------------------------------------------------------------

def test_motor_bridge_node():
    rclpy.init(args=['--ros-args', '-p', 'dry_run:=true', '-p', 'track_width:=0.5', '-p', 'max_wheel_speed:=1.0'])
    try:
        node = MotorBridge()
        assert node.track == 0.5
        assert node.vmax == 1.0
        assert node.dry_run is True
        left, right = node.twist_to_pwm(0.5, 0.0)
        assert left > 0
        assert right > 0
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
