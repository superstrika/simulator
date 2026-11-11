import math

import pytest

from rcj_sim import constants as C
from rcj_sim.field import ball_in_goal, in_penalty_area, is_on_line, neutral_spots
from rcj_sim.hardware import forward_kinematics, inverse_kinematics, saturate


# ---------------------------------------------------------------- rule dimensions
def test_official_dimensions():
    assert (C.FIELD_LENGTH, C.FIELD_WIDTH) == (219.0, 158.0)
    assert (2 * C.WALL_X, 2 * C.WALL_Y) == (243.0, 182.0)
    assert (C.GOAL_WIDTH, C.GOAL_DEPTH, C.GOAL_HEIGHT) == (60.0, 7.4, 10.0)
    assert (C.PENALTY_WIDTH, C.PENALTY_DEPTH, C.PENALTY_CORNER_RADIUS) == (80.0, 25.0, 15.0)
    assert C.ROBOT_DIAMETER == 18.0 and C.ROBOT_HEIGHT == 18.0
    assert C.BALL_DIAMETER == pytest.approx(4.27)
    assert C.CAPTURE_DEPTH == 1.5


def test_neutral_spots():
    spots = neutral_spots()
    assert len(spots) == 5
    assert (0.0, 0.0) in spots
    assert (C.HALF_L - 45, 40.0) in spots and (-(C.HALF_L - 45), -40.0) in spots


@pytest.mark.parametrize("pt,expected", [
    ((0, 0), False),
    ((109, 0), True),          # boundary line under the goal mouth
    ((50, 78.5), True),        # side boundary line
    ((50, 76), False),
    ((115, 50), False),        # outer area is plain carpet
    ((85, 0), True),           # front line of the penalty area
    ((-85, 10), True),
    ((90, 0), False),          # inside the penalty area
    ((107, 0), False),
    ((91.5, 36), True),        # rounded corner line
    ((92, 35), False),         # inside the rounded corner
    ((90, 37.5), False),       # outside the rounded corner
])
def test_white_lines(pt, expected):
    assert is_on_line(*pt) is expected


def test_penalty_area():
    assert in_penalty_area(100, 0, "yellow_goal")
    assert not in_penalty_area(100, 0, "blue_goal")
    assert in_penalty_area(-100, 30)
    assert not in_penalty_area(-86, 39)   # cut off by the rounded corner


def test_ball_in_goal():
    r = C.BALL_RADIUS
    assert ball_in_goal(C.GOAL_BACK_X - r, 0) == "yellow_goal"
    assert ball_in_goal(-(C.GOAL_BACK_X - r), 10) == "blue_goal"
    assert ball_in_goal(C.GOAL_MOUTH_X, 0) is None    # in the mouth, not at the back wall
    assert ball_in_goal(C.GOAL_BACK_X, 40) is None


# ---------------------------------------------------------------- omni kinematics
@pytest.mark.parametrize("cmd", [(0, 100, 0), (50, 0, 0), (30, -40, 0), (0, 0, 90), (20, 60, -45)])
def test_kinematics_round_trip(cmd):
    out = forward_kinematics(inverse_kinematics(*cmd))
    assert out == pytest.approx(cmd, abs=1e-9)


def test_x_wheel_layout_forward():
    w = inverse_kinematics(0, 100, 0)
    s = 100 / math.sqrt(2)
    assert w == pytest.approx([s, -s, -s, s])


def test_saturation_keeps_direction():
    w = saturate(inverse_kinematics(0, 1000, 0), 120)
    assert max(abs(x) for x in w) == pytest.approx(120)
    vx, vy, om = forward_kinematics(w)
    assert vx == pytest.approx(0, abs=1e-9) and om == pytest.approx(0, abs=1e-9)
    assert vy == pytest.approx(120 * math.sqrt(2))
