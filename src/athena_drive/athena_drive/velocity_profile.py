#!/usr/bin/env python3
"""
velocity_profile.py. Jerk-limited velocity shaping.

A step in commanded velocity is a step in torque, and a step in torque is
a jerk: the wheels break traction, the chassis rocks, and the camera the
visual odometry depends on gets shaken exactly when the rover is trying to
work out how it moved.  Limiting acceleration alone is not enough - it
still lets acceleration itself change instantly, which is what you feel as
the lurch at the start and the nod at the stop.

So this limits BOTH:

    acceleration   |dv/dt|  <= max_accel      how hard it pulls
    jerk           |da/dt|  <= max_jerk       how fast that pull changes

The result is an S-curve: acceleration eases in, holds, then eases out.

Anti-overshoot
--------------
A naive jerk limiter overshoots, because by the time it notices it has
arrived it is still accelerating and needs distance to unwind that.  Each
step therefore checks the velocity change still available while ramping
acceleration down to zero at max_jerk:

    v_stop = a^2 / (2*jerk)

If the remaining error is smaller than that, the profile is already
committed past the target, so acceleration is driven toward zero instead
of chasing.  Velocity is then clamped so it can never cross the target.

Deliberately separate from the ROS node so it can be unit-tested without
a running graph - see `test_profile()` at the bottom.
"""


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


class JerkLimiter:
    """One axis of jerk-limited velocity shaping."""

    def __init__(self, max_accel, max_jerk):
        self.max_accel = float(max_accel)
        self.max_jerk = float(max_jerk)
        self.v = 0.0     # current shaped velocity
        self.a = 0.0     # current acceleration

    def reset(self, v=0.0):
        self.v = v
        self.a = 0.0

    def step(self, target, dt):
        if dt <= 0.0:
            return self.v
        a_start = self.a          # jerk is measured against THIS, once
        err = target - self.v

        # Velocity that will still be gained while easing acceleration to
        # zero at max_jerk.  Past this point we are committed and must
        # start unwinding, or we overshoot.
        v_committed = (self.a * self.a) / (2.0 * self.max_jerk) \
            if self.max_jerk > 0 else 0.0

        if abs(err) <= v_committed and abs(self.a) > 1e-9:
            a_target = 0.0                      # unwind, do not chase
        else:
            a_target = _clamp(err / dt, -self.max_accel, self.max_accel)

        # jerk-limit the change in acceleration
        da = _clamp(a_target - self.a, -self.max_jerk * dt, self.max_jerk * dt)
        self.a = _clamp(self.a + da, -self.max_accel, self.max_accel)

        v_new = self.v + self.a * dt

        # Never cross the target: crossing means the next step fights back,
        # which reads as a hunt around the setpoint.  Ease acceleration
        # toward zero at the jerk limit rather than zeroing it outright -
        # snapping it to 0 is itself an infinite jerk, which is the exact
        # thing this class exists to prevent.
        if (err > 0 and v_new > target) or (err < 0 and v_new < target):
            v_new = target
            # Relative to a_start, NOT to the value just written above:
            # clamping against the updated acceleration would allow two
            # jerk-limited changes inside a single step, i.e. twice the
            # jerk limit at exactly the moment the profile settles.
            self.a = _clamp(0.0, a_start - self.max_jerk * dt,
                            a_start + self.max_jerk * dt)

        self.v = v_new
        return self.v


class VelocityProfile:
    """Jerk-limited shaping of a (linear, angular) command pair."""

    def __init__(self, max_accel, max_jerk, max_ang_accel, max_ang_jerk):
        self.lin = JerkLimiter(max_accel, max_jerk)
        self.ang = JerkLimiter(max_ang_accel, max_ang_jerk)

    def reset(self):
        self.lin.reset()
        self.ang.reset()

    def step(self, v_target, w_target, dt):
        return (self.lin.step(v_target, dt),
                self.ang.step(w_target, dt))

    @property
    def accel(self):
        return self.lin.a, self.ang.a


def test_profile():
    """Smoke test: step input, then stop. Prints the shape and checks limits."""
    dt = 0.05
    p = VelocityProfile(max_accel=0.5, max_jerk=1.0,
                        max_ang_accel=1.5, max_ang_jerk=3.0)
    prev_a = 0.0
    worst_j = 0.0
    worst_a = 0.0
    print(f'{"t":>5} {"v":>7} {"a":>7} {"jerk":>7}')
    t = 0.0
    for i in range(80):
        target = 0.4 if i < 40 else 0.0
        v, _ = p.step(target, 0.0, dt)
        a = p.lin.a
        j = abs(a - prev_a) / dt
        worst_j = max(worst_j, j)
        worst_a = max(worst_a, abs(a))
        prev_a = a
        if i % 5 == 0 or i == 39:
            print(f'{t:5.2f} {v:7.3f} {a:7.3f} {j:7.3f}')
        t += dt
    print(f'\npeak |accel| = {worst_a:.3f} (limit 0.5)')
    print(f'peak |jerk|  = {worst_j:.3f} (limit 1.0)')
    ok = worst_a <= 0.5 + 1e-6 and worst_j <= 1.0 + 1e-6
    print('LIMITS RESPECTED' if ok else 'LIMIT VIOLATED')
    return ok


if __name__ == '__main__':
    raise SystemExit(0 if test_profile() else 1)
