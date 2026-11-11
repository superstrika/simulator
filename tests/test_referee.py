import math

import pytest

from conftest import IDLE
from rcj_sim import constants as C
from rcj_sim.field import goal_side


def shoot_into(sim, goal):
    s = goal_side(goal)
    sim.move_ball(s * 100, 0)
    sim.world.ball.velocity = (s * 200, 0)


def test_free_mode_ends_at_first_goal(make_sim):
    sim = make_sim([("B1", "blue", IDLE), ("Y1", "yellow", IDLE)], mode="free")
    shoot_into(sim, C.YELLOW_GOAL)
    sim.run_for(1.0)
    assert sim.finished
    assert sim.referee.score == {"blue": 1, "yellow": 0}


def test_basic_goal_then_kickoff(make_sim):
    sim = make_sim([("B1", "blue", IDLE), ("Y1", "yellow", IDLE)], mode="basic")
    shoot_into(sim, C.BLUE_GOAL)
    sim.run_for(0.3)
    ref = sim.referee
    assert ref.score == {"blue": 0, "yellow": 1}
    assert not sim.finished
    assert ref.kickoff_team == "blue"            # team scored against kicks off
    assert tuple(sim.world.ball.position) == (0, 0)
    assert ref.state == "hold"
    sim.run_for(1.0)
    assert ref.state == "playing"


def test_kickoff_positions_respect_clearance(make_sim):
    robots = [("B1", "blue", IDLE), ("B2", "blue", IDLE), ("Y1", "yellow", IDLE), ("Y2", "yellow", IDLE)]
    sim = make_sim(robots, mode="basic")
    for e in sim.entities:
        b = e.body
        assert (b.x < 0) == (e.cfg.team == "blue")              # own half
        if e.cfg.team == "yellow":                              # not kicking off
            d = math.hypot(b.x, b.y) - C.ROBOT_RADIUS
            assert d >= C.KICKOFF_CLEARANCE


def test_half_time_swaps_sides_and_full_time(make_sim):
    code = """
def main(robot):
    print(robot.opponent_goal)
    while True:
        robot.delay(100)
"""
    sim = make_sim([("B1", "blue", code)], mode="basic", referee={"half_duration_s": 2})
    sim.run_for(2.2)
    assert sim.referee.half == 2
    assert sim.sides_swapped
    assert sim.defended_goal("blue") == C.YELLOW_GOAL
    sim.run_for(4)
    assert sim.finished
    lines = [l for _, l in sim.entity("B1").ctrl.logs]
    assert lines == [C.YELLOW_GOAL, C.BLUE_GOAL]   # program restarted with new goal


def test_out_of_bounds_wall(make_sim):
    sim = make_sim([("B1", "blue", IDLE)], mode="full",
                   referee={"out_of_bounds_penalty_s": 1.0, "lack_of_progress": False})
    e = sim.entity("B1")
    sim.move_robot(e, 0, C.WALL_Y - C.ROBOT_RADIUS, 0.0)
    sim.step()
    assert not e.body.on_field and e.penalty_until is not None
    sim.run_for(1.1)
    assert e.body.on_field
    # back on the unoccupied neutral spot furthest from the ball (ball is at the centre)
    assert (round(e.body.x, 3), round(abs(e.body.y), 3)) in [(sx, abs(sy)) for sx, sy in [
        (-(C.HALF_L - 45), 40), ((C.HALF_L - 45), 40)]]
    # ...facing its own goal (blue goal is at -x)
    assert math.cos(e.body.angle) == pytest.approx(-1)


def test_out_of_bounds_inside_penalty_area(make_sim):
    sim = make_sim([("B1", "blue", IDLE)], mode="full")
    e = sim.entity("B1")
    sim.move_robot(e, -100, 0, 0.0)
    sim.step()
    assert e.status == "penalty"


def test_partly_in_penalty_area_is_fine(make_sim):
    sim = make_sim([("B1", "blue", IDLE)], mode="full")
    e = sim.entity("B1")
    sim.move_robot(e, -88, 0, 0.0)
    sim.run_for(0.2)
    assert e.body.on_field


def test_kickoff_ends_penalty(make_sim):
    sim = make_sim([("B1", "blue", IDLE), ("Y1", "yellow", IDLE)], mode="full")
    e = sim.entity("B1")
    sim.move_robot(e, 0, C.WALL_Y - C.ROBOT_RADIUS, 0.0)
    sim.step()
    assert e.status == "penalty"
    shoot_into(sim, C.YELLOW_GOAL)
    sim.run_for(0.3)
    assert e.body.on_field and e.penalty_until is None


def test_multiple_defense(make_sim):
    sim = make_sim([("B1", "blue", IDLE), ("B2", "blue", IDLE)], mode="full")
    b1, b2 = sim.entity("B1"), sim.entity("B2")
    sim.move_ball(0, 30)
    sim.move_robot(b1, -80, 20, 0.0)
    sim.move_robot(b2, -80, -20, 0.0)
    sim.step()
    assert (round(b1.body.x, 1), round(b1.body.y, 1)) == (-80, 20)   # closer to the ball: stays
    assert (round(b2.body.x, 3), round(b2.body.y, 3)) in [tuple(map(float, p)) for p in C.NEUTRAL_SPOTS]


def test_lack_of_progress(make_sim):
    sim = make_sim([("B1", "blue", IDLE)], mode="basic", referee={"lack_of_progress_s": 2.0})
    sim.move_robot(sim.entity("B1"), 30, 25)          # near the ball, not on a spot
    sim.move_ball(10, 10)
    sim.run_for(2.5)
    assert tuple(sim.world.ball.position) == (0, 0)   # nearest unoccupied neutral spot


def test_lack_of_progress_skips_occupied_spot(make_sim):
    sim = make_sim([("B1", "blue", IDLE)], mode="basic", referee={"lack_of_progress_s": 2.0})
    sim.move_robot(sim.entity("B1"), -5, 0)          # standing on the centre spot
    sim.move_ball(10, 10)
    sim.run_for(2.5)
    assert tuple(sim.world.ball.position) == (C.HALF_L - 45, 40)


def test_no_lack_of_progress_when_nobody_near_ball(make_sim):
    sim = make_sim([("B1", "blue", IDLE)], mode="basic", referee={"lack_of_progress_s": 2.0})
    sim.move_robot(sim.entity("B1"), -80, -60)
    sim.move_ball(10, 10)
    sim.run_for(2.5)
    assert tuple(sim.world.ball.position) == (10, 10)


def test_full_mode_limits_team_size(make_sim):
    robots = [(f"B{i}", "blue", IDLE) for i in range(3)]
    sim = make_sim(robots, mode="full")
    assert len(sim.entities) == 2
    assert sim.warnings
