"""Simulated robot hardware: X-shaped omni drive, gyro, cameras, TCRT line sensors.

Angle convention (everything the robot API returns):
    degrees in (-180, 180], 0 = straight ahead, positive = clockwise (to the right).
"""
import math

from .constants import (
    BALL, BLUE_GOAL, GOAL_X, OBJECTS, ROBOT_RADIUS, TCRT_COUNT, TCRT_RADIUS,
    WHEEL_BASE_RADIUS, YELLOW_GOAL,
)
from .field import goal_mouth, is_on_line

# Omni wheels in an X: mounting angles measured CCW from the robot's right (+x_r) axis.
# 0 = front-right, 1 = front-left, 2 = back-left, 3 = back-right.
WHEEL_ANGLES = [math.radians(a) for a in (45, 135, 225, 315)]
WHEEL_NAMES = ("front-right", "front-left", "back-left", "back-right")


def wrap180(a):
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def inverse_kinematics(vx, vy, omega):
    """Robot-frame command -> 4 wheel surface speeds (cm/s).

    vx: right (cm/s), vy: forward (cm/s), omega: deg/s, positive = clockwise."""
    w_ccw = -math.radians(omega)
    return [-math.sin(p) * vx + math.cos(p) * vy + WHEEL_BASE_RADIUS * w_ccw for p in WHEEL_ANGLES]


def forward_kinematics(wheels):
    """4 wheel speeds -> (vx, vy, omega) in the same convention as inverse_kinematics."""
    vx = -sum(math.sin(p) * w for p, w in zip(WHEEL_ANGLES, wheels)) / 2
    vy = sum(math.cos(p) * w for p, w in zip(WHEEL_ANGLES, wheels)) / 2
    w_ccw = sum(wheels) / 4 / WHEEL_BASE_RADIUS
    return vx, vy, -math.degrees(w_ccw)


def saturate(wheels, wmax):
    """Scale all wheels together so none exceeds wmax (keeps the motion direction)."""
    peak = max(abs(w) for w in wheels)
    if peak <= wmax or peak == 0:
        return list(wheels)
    k = wmax / peak
    return [w * k for w in wheels]


def relative_angle(rx, ry, heading, tx, ty):
    """Angle of target (tx, ty) seen from a robot at (rx, ry) with heading (rad, CCW from +x)."""
    a = math.atan2(ty - ry, tx - rx)
    return wrap180(math.degrees(heading - a))


def _seg_point_dist(ax, ay, bx, by, px, py):
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(ax + t * dx - px, ay + t * dy - py)


class RobotHardware:
    """All sensors/actuators of one robot. Pure state access; timing lives in runner.py."""

    def __init__(self, sim, body, rng):
        self.sim = sim
        self.body = body
        self.cfg = sim.cfg.sim
        self.rng = rng
        self.wheels = [0.0, 0.0, 0.0, 0.0]
        self.gyro_zero = 0.0
        self.gyro_drift = 0.0
        self.line_state = [False] * TCRT_COUNT
        self.pending_interrupts = []
        self.reset_gyro_zero()

    # ------------------------------------------------------------ motors
    def set_speed(self, vx, vy, omega):
        w = saturate(inverse_kinematics(float(vx), float(vy), float(omega)), self.cfg.max_wheel_speed)
        self.set_wheels(w)

    def set_wheels(self, wheels):
        self.wheels = list(wheels)
        vx, vy, om = forward_kinematics(self.wheels)
        self.body.cmd = (vx, vy, -math.radians(om))

    def stop(self):
        self.set_wheels([0.0] * 4)

    # ------------------------------------------------------------ gyro
    def reset_gyro_zero(self):
        self.gyro_zero = self.body.angle
        self.gyro_drift = 0.0
        if self.cfg.noise:
            self.gyro_drift_rate = self.rng.gauss(0, self.cfg.gyro_drift_deg_per_min / 60.0)
        else:
            self.gyro_drift_rate = 0.0
        self.gyro_t0 = self.sim.time

    def gyro(self):
        a = math.degrees(self.gyro_zero - self.body.angle)
        if self.cfg.noise:
            a += self.gyro_drift_rate * (self.sim.time - self.gyro_t0)
            a += self.rng.gauss(0, self.cfg.gyro_noise_deg)
        return wrap180(a)

    # ------------------------------------------------------------ cameras
    def _object_point(self, obj):
        if obj == BALL:
            p = self.sim.world.ball.position
            return p.x, p.y
        return GOAL_X[obj], 0.0

    def _occluded(self, tx, ty):
        if not self.cfg.occlusion:
            return False
        rx, ry = self.body.x, self.body.y
        dist = math.hypot(tx - rx, ty - ry)
        for other in self.sim.world.robots.values():
            if other is self.body or not other.on_field:
                continue
            if math.hypot(other.x - rx, other.y - ry) >= dist:
                continue
            if _seg_point_dist(rx, ry, tx, ty, other.x, other.y) < ROBOT_RADIUS:
                return True
        return False

    def _noisy_angle(self, a):
        if self.cfg.noise:
            a = wrap180(a + self.rng.gauss(0, self.cfg.angle_noise_deg))
        return a

    def _noisy_dist(self, d):
        if self.cfg.noise:
            d *= 1 + self.rng.gauss(0, self.cfg.distance_noise_pct / 100)
        return max(0.0, d)

    @staticmethod
    def _check_obj(obj):
        if obj not in OBJECTS:
            raise ValueError(f"unknown object {obj!r}; use BALL, BLUE_GOAL or YELLOW_GOAL")

    def object_angle(self, obj):
        """360-degree camera: angle to the ball / goal centre from anywhere."""
        self._check_obj(obj)
        if not self.body.on_field:
            return None
        tx, ty = self._object_point(obj)
        if self._occluded(tx, ty):
            return None
        return self._noisy_angle(relative_angle(self.body.x, self.body.y, self.body.angle, tx, ty))

    def _in_front_view(self, tx, ty):
        b = self.body
        d = math.hypot(tx - b.x, ty - b.y)
        a = relative_angle(b.x, b.y, b.angle, tx, ty)
        return d <= self.cfg.front_range_cm and abs(a) <= self.cfg.front_fov_deg / 2, d, a

    def object_location(self, obj):
        """Front camera: (distance_cm, angle_deg) if the object is in the field of view."""
        self._check_obj(obj)
        b = self.body
        if not b.on_field:
            return None
        if obj == BALL:
            tx, ty = self._object_point(obj)
            ok, d, a = self._in_front_view(tx, ty)
            if not ok or self._occluded(tx, ty):
                return None
            return self._noisy_dist(d), self._noisy_angle(a)
        # goals: visible if any part of the mouth is in view (and we are in front of it)
        (x0, y0), (x1, y1) = goal_mouth(obj)
        if (obj == YELLOW_GOAL and b.x >= x0) or (obj == BLUE_GOAL and b.x <= x0):
            return None
        visible = []
        n = 30
        for i in range(n + 1):
            px, py = x0, y0 + (y1 - y0) * i / n
            ok, _, _ = self._in_front_view(px, py)
            if ok and not self._occluded(px, py):
                visible.append(py)
        if not visible:
            return None
        cy = (visible[0] + visible[-1]) / 2
        _, d, a = self._in_front_view(x0, cy)
        return self._noisy_dist(d), self._noisy_angle(a)

    # ------------------------------------------------------------ line sensors
    def tcrt_positions(self):
        """World positions of the 16 sensors; index 0 = front, increasing clockwise."""
        b = self.body
        out = []
        for i in range(TCRT_COUNT):
            a = b.angle - math.radians(i * 360.0 / TCRT_COUNT)
            out.append((b.x + TCRT_RADIUS * math.cos(a), b.y + TCRT_RADIUS * math.sin(a)))
        return out

    def update_line_sensors(self):
        """Called by the simulator after every physics step; queues rising edges."""
        if not self.body.on_field:
            self.line_state = [False] * TCRT_COUNT
            return
        new = [is_on_line(x, y) for (x, y) in self.tcrt_positions()]
        for i, (old, cur) in enumerate(zip(self.line_state, new)):
            if cur and not old and i not in self.pending_interrupts:
                self.pending_interrupts.append(i)
        self.line_state = new

    # ------------------------------------------------------------ dribbler / kicker
    def set_dribbler(self, on):
        self.body.set_dribbler(on)

    def kick(self):
        return self.body.kick(self.sim.time)

    def has_ball(self):
        return self.body.ball_in_zone()
