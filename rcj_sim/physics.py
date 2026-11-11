"""2D rigid-body physics (pymunk): walls, goals, ball, robots, dribbler, kicker."""
import math

import pymunk

from .constants import (
    BALL_MASS, BALL_RADIUS, HELD_BALL_X, NOTCH_BACK_X, NOTCH_HALF_WIDTH,
    ROBOT_MASS, ROBOT_RADIUS, WALL_X, WALL_Y,
)
from .field import wall_polygons

# capture zone for the dribbler / kicker, in robot-local coordinates of the ball centre
CAPTURE_X_MIN = HELD_BALL_X - 0.8
CAPTURE_X_MAX = HELD_BALL_X + 1.8
CAPTURE_Y_MAX = 2.5
DRIBBLER_RELEASE_DIST = 1.5

CAT_WALL, CAT_BALL, CAT_ROBOT = 1, 2, 4


def _arc(r, a0, a1, n):
    return [(r * math.cos(a0 + (a1 - a0) * i / n), r * math.sin(a0 + (a1 - a0) * i / n))
            for i in range(n + 1)]


def robot_polygons():
    """Convex pieces of an 18 cm round robot with a 1.5 cm deep front notch.

    The robot faces local +x."""
    r = ROBOT_RADIUS
    chord = math.acos(NOTCH_BACK_X / r)              # where the notch plate meets the shell
    prong = math.asin(NOTCH_HALF_WIDTH / r)
    body = _arc(r, chord, 2 * math.pi - chord, 32)  # everything behind the notch plate
    top = [(NOTCH_BACK_X, NOTCH_HALF_WIDTH)] + _arc(r, prong, chord, 4)
    bottom = [(x, -y) for (x, y) in top]
    return [body, top, bottom]


class RobotBody:
    def __init__(self, world, name):
        self.world = world
        self.name = name
        self.body = pymunk.Body(ROBOT_MASS, pymunk.moment_for_circle(ROBOT_MASS, 0, ROBOT_RADIUS))
        self.shapes = []
        for verts in robot_polygons():
            s = pymunk.Poly(self.body, verts)
            s.elasticity = 0.3
            s.friction = 0.5
            s.filter = pymunk.ShapeFilter(categories=CAT_ROBOT)
            s.robot_name = name
            self.shapes.append(s)
        # commanded body velocity in the robot frame: (right cm/s, forward cm/s, CCW rad/s)
        self.cmd = (0.0, 0.0, 0.0)
        self.on_field = False
        self.frozen = False              # referee hold: motors have no effect
        self.dribbler_on = False
        self.hold_joint = None
        self.last_kick_time = -1e9

    # ------------------------------------------------------------ placement
    def place(self, x, y, angle):
        b = self.body
        b.position = (x, y)
        b.angle = angle
        b.velocity = (0, 0)
        b.angular_velocity = 0
        if not self.on_field:
            self.world.space.add(b, *self.shapes)
            self.on_field = True
        self.world.space.reindex_shapes_for_body(b)

    def remove(self):
        self.release_ball()
        if self.on_field:
            self.world.space.remove(self.body, *self.shapes)
            self.on_field = False

    # ------------------------------------------------------------ state
    @property
    def x(self):
        return self.body.position.x

    @property
    def y(self):
        return self.body.position.y

    @property
    def angle(self):
        """Heading, radians CCW from +x."""
        return self.body.angle

    def ball_local(self):
        return self.body.world_to_local(self.world.ball.position)

    def ball_in_zone(self):
        if not self.on_field:
            return False
        lx, ly = self.ball_local()
        return CAPTURE_X_MIN <= lx <= CAPTURE_X_MAX and abs(ly) <= CAPTURE_Y_MAX

    @property
    def holding(self):
        return self.hold_joint is not None

    # ------------------------------------------------------------ actuators
    def set_dribbler(self, on):
        self.dribbler_on = bool(on)
        if not on:
            self.release_ball()

    def release_ball(self):
        if self.hold_joint is not None:
            self.world.space.remove(self.hold_joint)
            self.hold_joint = None
            if self.world.holder is self:
                self.world.holder = None

    def _capture(self):
        w = self.world
        if w.holder is not None and w.holder is not self:
            return  # one robot at a time
        j = pymunk.PivotJoint(self.body, w.ball, (HELD_BALL_X, 0), (0, 0))
        j.max_force = BALL_MASS * w.cfg.dribbler_grip
        j.collide_bodies = True
        w.space.add(j)
        self.hold_joint = j
        w.holder = self

    def kick(self, now):
        cfg = self.world.cfg
        if now - self.last_kick_time < cfg.kick_cooldown:
            return False
        self.last_kick_time = now  # the solenoid fires even with no ball
        if not self.ball_in_zone():
            return False
        self.release_ball()
        ball = self.world.ball
        fwd = self.body.rotation_vector
        base = self.body.velocity_at_world_point(ball.position)
        ball.velocity = (base.x + fwd.x * cfg.kick_speed, base.y + fwd.y * cfg.kick_speed)
        self.world.last_touch = self.name
        return True

    # ------------------------------------------------------------ per-step
    def drive(self, dt):
        cfg = self.world.cfg
        b = self.body
        if self.frozen:
            tv, tw = (0.0, 0.0), 0.0
        else:
            vr, vf, tw = self.cmd
            c, s = math.cos(b.angle), math.sin(b.angle)
            # forward = (c, s), right = (s, -c)
            tv = (vf * c + vr * s, vf * s - vr * c)
        if cfg.accel_limit:
            dvx, dvy = tv[0] - b.velocity.x, tv[1] - b.velocity.y
            dv = math.hypot(dvx, dvy)
            lim = cfg.max_accel * dt
            if dv > lim:
                dvx, dvy = dvx * lim / dv, dvy * lim / dv
            b.velocity = (b.velocity.x + dvx, b.velocity.y + dvy)
            wlim = cfg.max_accel / ROBOT_RADIUS * dt
            dw = max(-wlim, min(wlim, tw - b.angular_velocity))
            b.angular_velocity += dw
        else:
            b.velocity = tv
            b.angular_velocity = tw

    def post_step(self):
        if self.hold_joint is not None:
            anchor = self.body.local_to_world((HELD_BALL_X, 0))
            if (self.world.ball.position - anchor).length > DRIBBLER_RELEASE_DIST:
                self.release_ball()     # knocked out of the dribbler
        elif self.dribbler_on and self.ball_in_zone():
            self._capture()


class World:
    def __init__(self, sim_cfg):
        self.cfg = sim_cfg
        self.space = pymunk.Space()
        self.space.gravity = (0, 0)
        self.space.iterations = 20
        for verts in wall_polygons():
            s = pymunk.Poly(self.space.static_body, verts)
            s.elasticity = 0.5
            s.friction = 0.4
            s.filter = pymunk.ShapeFilter(categories=CAT_WALL)
            self.space.add(s)
        self.ball = pymunk.Body(BALL_MASS, pymunk.moment_for_circle(BALL_MASS, 0, BALL_RADIUS))
        self.ball_shape = pymunk.Circle(self.ball, BALL_RADIUS)
        self.ball_shape.elasticity = 0.8
        self.ball_shape.friction = 0.3
        self.ball_shape.filter = pymunk.ShapeFilter(categories=CAT_BALL)
        self.space.add(self.ball, self.ball_shape)
        self.robots = {}
        self.holder = None
        self.last_touch = None

    def add_robot(self, name):
        rb = RobotBody(self, name)
        self.robots[name] = rb
        return rb

    def place_ball(self, x, y, vx=0.0, vy=0.0):
        if self.holder is not None:
            self.holder.release_ball()
        self.ball.position = (x, y)
        self.ball.velocity = (vx, vy)
        self.ball.angular_velocity = 0
        self.space.reindex_shapes_for_body(self.ball)

    def step(self, dt):
        for rb in self.robots.values():
            if rb.on_field:
                rb.drive(dt)
        if self.holder is None:
            v = self.ball.velocity
            sp = v.length
            dec = self.cfg.ball_decel * dt
            self.ball.velocity = (0, 0) if sp <= dec else v * ((sp - dec) / sp)
        self.space.step(dt)
        for rb in self.robots.values():
            if rb.on_field:
                _keep_inside(rb.body, ROBOT_RADIUS)
                rb.post_step()
        _keep_inside(self.ball, BALL_RADIUS)


def _keep_inside(body, r):
    """Safety net: never let a body end up outside the walls."""
    x, y = body.position
    vx, vy = body.velocity
    lx, ly = WALL_X - r, WALL_Y - r
    if abs(x) > lx:
        x, vx = math.copysign(lx, x), 0.0
    if abs(y) > ly:
        y, vy = math.copysign(ly, y), 0.0
    if (x, y) != tuple(body.position):
        body.position = (x, y)
        body.velocity = (vx, vy)
