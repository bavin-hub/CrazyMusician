"""
Velocity plus acceleration-feedforward ESO from crazymusician's adrc-vel branch.

PX4 closes the velocity loop. The observer models its proportional velocity
response, and the estimated disturbance is cancelled through acceleration
feedforward. This module has no ROS dependencies.
"""

import math

import numpy as np


G = 9.81


def min_jerk(t, duration):
    """Return a smooth transition and its first two time derivatives."""
    if t <= 0 or duration <= 0:
        return (0.0, 0.0, 0.0) if t <= 0 else (1.0, 0.0, 0.0)
    if t >= duration:
        return 1.0, 0.0, 0.0
    x = t / duration
    return (
        10 * x**3 - 15 * x**4 + 6 * x**5,
        (30 * x**2 - 60 * x**3 + 30 * x**4) / duration,
        (60 * x - 180 * x**2 + 120 * x**3) / duration**2,
    )


def limit_accel(acceleration, tilt_max_rad):
    """Preserve upstream's 0.5g vertical and tilt-based horizontal limits."""
    result = np.array(acceleration, dtype=float)
    result[2] = min(max(result[2], -0.5 * G), 0.5 * G)
    xy_max = (G - result[2]) * math.tan(tilt_max_rad)
    norm = math.hypot(result[0], result[1])
    if norm > xy_max:
        result[:2] *= xy_max / norm
    return result


class Reference:
    """Hold a NED point or generate a minimum-jerk move to the next point."""

    def __init__(self):
        self.start = np.zeros(3)
        self.goal = np.zeros(3)
        self.t0 = self.duration = 0.0

    def hold(self, position, now):
        """Initialize a stationary reference."""
        self.start = self.goal = np.array(position, dtype=float)
        self.t0, self.duration = now, 0.0

    def move_to(self, goal, now, duration):
        """Start a smooth move from the current reference."""
        self.start, _, _ = self.sample(now)
        self.goal = np.array(goal, dtype=float)
        self.t0, self.duration = now, max(duration, 1e-3)

    def sample(self, now):
        """Return reference position, velocity, and acceleration."""
        s, sd, sdd = min_jerk(now - self.t0, self.duration)
        delta = self.goal - self.start
        return self.start + s * delta, sd * delta, sdd * delta


class VelAccADRC:
    """Three-axis velocity references with ESO acceleration feedforward."""

    def __init__(self, k_pos=(1.0, 1.0, 1.0), wo=(8.0, 8.0, 8.0),
                 k_vel=(1.8, 1.8, 4.0), v_max=(1.0, 1.0, 0.8),
                 tilt_max_deg=20.0, use_feedforward=True, use_r_ddot=True):
        self.k_pos, self.wo, self.k_vel = (
            np.array(value, dtype=float) for value in (k_pos, wo, k_vel))
        self.v_max = np.array(v_max, dtype=float)
        self.tilt = math.radians(tilt_max_deg)
        self.use_ff, self.use_rdd = use_feedforward, use_r_ddot
        self.z1 = np.zeros(3)
        self.z2 = np.zeros(3)
        self.u_model = np.zeros(3)
        self.v_ref = np.zeros(3)
        self.a_ff = np.zeros(3)

    def reset(self, velocity):
        """Initialize velocity estimate and clear disturbance/model history."""
        self.z1 = np.array(velocity, dtype=float)
        self.z2[:] = 0.0
        self.u_model[:] = 0.0

    def observe(self, velocity, dt, engaged):
        """Integrate the ESO using the previous modeled PX4 acceleration."""
        u = self.u_model if engaged else np.zeros(3)
        error = self.z1 - np.asarray(velocity, dtype=float)
        self.z1 = self.z1 + dt * (self.z2 - 2 * self.wo * error + u)
        self.z2 = self.z2 - dt * self.wo**2 * error

    def control(self, position, reference, r_dot, r_ddot, ff_scale=1.0):
        """Return velocity setpoints and limited acceleration feedforward."""
        velocity = self.k_pos * (np.asarray(reference) - np.asarray(position)) + r_dot
        self.v_ref = np.clip(velocity, -self.v_max, self.v_max)
        feedforward = np.zeros(3)
        if self.use_ff:
            feedforward -= self.z2
        if self.use_rdd:
            feedforward += np.asarray(r_ddot)
        self.a_ff = limit_accel(feedforward, self.tilt) * min(max(ff_scale, 0.0), 1.0)
        self.u_model = limit_accel(
            self.k_vel * (self.v_ref - self.z1) + self.a_ff, self.tilt)
        return self.v_ref.copy(), self.a_ff.copy()
