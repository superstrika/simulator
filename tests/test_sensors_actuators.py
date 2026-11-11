import math

import pytest

from conftest import IDLE, logs
from rcj_sim import constants as C
from rcj_sim.config import SimConfig
from rcj_sim.physics import World


def one_robot(make_sim, code=IDLE, **kw):
    sim = make_sim([("R", "blue", code)], **kw)
    e = sim.entity("R")
    return sim, e


def test_omni_camera_angles(make_sim):
    sim, e = one_robot(make_sim)
    sim.move_robot(e, 0, 0, 0.0)          # facing +x
    hw = e.hw
    for (bx, by), expected in [((50, 0), 0), ((0, -50), 90), ((0, 50), -90), ((-50, 0), 180),
                               ((50, -50), 45)]:
        sim.move_ball(bx, by)
        assert hw.object_angle(C.BALL) == pytest.approx(expected)
    assert hw.object_angle(C.YELLOW_GOAL) == pytest.approx(0)
    assert abs(hw.object_angle(C.BLUE_GOAL)) == pytest.approx(180)


def test_front_camera_fov_and_range(make_sim):
    sim, e = one_robot(make_sim, sim={"front_range_cm": 60})
    sim.move_robot(e, 0, 0, 0.0)
    hw = e.hw
    sim.move_ball(50, 0)
    d, a = hw.object_location(C.BALL)
    assert d == pytest.approx(50) and a == pytest.approx(0)
    sim.move_ball(40, -20)                # 26.6 deg to the right: inside +-30
    d, a = hw.object_location(C.BALL)
    assert a == pytest.approx(math.degrees(math.atan2(20, 40)))
    sim.move_ball(30, 30)                 # 45 deg left: outside the FOV
    assert hw.object_location(C.BALL) is None
    sim.move_ball(70, 0)                  # beyond the 60 cm range
    assert hw.object_location(C.BALL) is None


def test_front_camera_goals(make_sim):
    sim, e = one_robot(make_sim)
    hw = e.hw
    sim.move_robot(e, 0, 0, 0.0)
    d, a = hw.object_location(C.YELLOW_GOAL)
    assert d == pytest.approx(C.GOAL_MOUTH_X) and a == pytest.approx(0, abs=1e-6)
    assert hw.object_location(C.BLUE_GOAL) is None
    sim.move_robot(e, 0, 0, math.pi)
    assert hw.object_location(C.BLUE_GOAL) is not None
    # partially visible goal: only the part inside the FOV counts
    sim.move_robot(e, 80, -60, math.radians(30))
    d, a = hw.object_location(C.YELLOW_GOAL)
    assert -30 <= a <= -15
    full = hw.object_angle(C.YELLOW_GOAL)
    assert full < a                       # centre of the whole goal is further left


def test_gyro_follows_rotation(make_sim):
    code = """
def main(robot):
    robot.set_speed(0, 0, 90)     # 90 deg/s clockwise
    robot.delay(1000)
    robot.motor_stop()
    print("gyro", round(robot.get_angle()))
    while True:
        robot.delay(1000)
"""
    sim, e = one_robot(make_sim, code)
    sim.run_for(1.2)
    assert logs(sim, "R") == ["gyro 90"]


def test_line_interrupt_reports_side(make_sim):
    code = """
hits = []
def on_line(sensor):
    hits.append(sensor)
def main(robot):
    robot.set_interrupt(on_line)
    robot.set_speed(60, 0, 0)       # drive to the right
    while not hits:
        robot.delay(5)
    robot.motor_stop()
    print("first", hits[0])
    while True:
        robot.delay(1000)
"""
    sim, e = one_robot(make_sim, code)
    sim.move_robot(e, 0, -55, 0.0)    # facing +x, so "right" is towards the -y boundary line
    sim.run_for(1.0)
    assert logs(sim, "R") == ["first 4"]
    assert e.body.y > -C.HALF_W       # stopped on the line, did not run out


def test_interrupt_preempts_delay(make_sim):
    code = """
def stop(robot, sensor):
    robot.motor_stop()
def main(robot):
    robot.set_interrupt(stop)
    robot.set_speed(0, 80, 0)
    robot.delay(5000)             # long sleep: the interrupt must still run
    while True:
        robot.delay(1000)
"""
    sim, e = one_robot(make_sim, code)
    sim.move_robot(e, 0, 50, math.pi / 2)   # facing +y towards the side line
    sim.run_for(2.0)
    assert e.body.y < C.HALF_W - 3


def test_dribbler_holds_ball_while_turning(make_sim):
    code = """
from rcj_sim import BALL
def main(robot):
    robot.set_dribbler(True)
    robot.set_speed(0, 40, 0)
    while not robot.has_ball():
        robot.delay(5)
    robot.set_speed(0, 0, 360)
    robot.delay(1000)
    robot.motor_stop()
    print("held", robot.has_ball())
    while True:
        robot.delay(1000)
"""
    sim, e = one_robot(make_sim, code)
    sim.move_robot(e, 0, 0, 0.0)
    sim.move_ball(25, 0)
    sim.run_for(2.0)
    assert logs(sim, "R") == ["held True"]
    assert e.body.holding


def test_kick_needs_ball_and_cooldown(make_sim):
    sim, e = one_robot(make_sim)
    sim.move_robot(e, 0, 0, 0.0)
    sim.move_ball(50, 0)
    assert e.hw.kick() is False                         # no ball in front
    sim.move_ball(C.HELD_BALL_X + 0.2, 0)
    assert e.hw.kick() is False                         # still recharging
    sim.run_for(0.6)
    sim.move_ball(C.HELD_BALL_X + 0.2, 0)
    assert e.hw.kick() is True
    assert sim.world.ball.velocity.x == pytest.approx(SimConfig().kick_speed, rel=0.01)


def test_kick_power_check():
    """Appendix A: kicking from the own goal reaches the opposite goal, but the
    rebound must not reach the back wall of the kicker's own goal."""
    cfg = SimConfig()
    w = World(cfg)
    x0 = -C.GOAL_MOUTH_X + 9 + C.HELD_BALL_X - C.ROBOT_RADIUS
    w.place_ball(x0, 0, cfg.kick_speed, 0)
    reached = False
    back = C.GOAL_BACK_X - C.BALL_RADIUS - 0.35
    for _ in range(int(15 * cfg.physics_hz)):
        w.step(1 / cfg.physics_hz)
        x = w.ball.position.x
        if x >= back:
            reached = True
        if reached:
            assert x > -back
    assert reached
