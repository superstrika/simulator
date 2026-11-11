"""Field geometry: white lines, penalty areas, goals and neutral spots."""
import math

from .constants import (
    BALL_RADIUS, GOAL_BACK_X, GOAL_MOUTH_X, GOAL_WIDTH,
    HALF_L, HALF_W, LINE_WIDTH, NEUTRAL_SPOTS, PENALTY_CORNER_RADIUS,
    PENALTY_DEPTH, PENALTY_WIDTH, ROBOT_RADIUS, WALL_X, WALL_Y, BLUE_GOAL,
    YELLOW_GOAL,
)

PENALTY_FRONT_X = HALF_L - PENALTY_DEPTH          # 84.5
_CORNER_CX = PENALTY_FRONT_X + PENALTY_CORNER_RADIUS  # 99.5
_CORNER_CY = PENALTY_WIDTH / 2 - PENALTY_CORNER_RADIUS  # 25


def goal_side(goal):
    """-1 for the blue goal (at -x), +1 for the yellow goal (at +x)."""
    return -1 if goal == BLUE_GOAL else 1


def _in_rounded_area(ax, ay, inset):
    """Is (ax, |y|) inside the +x penalty area shrunk by `inset`? ax, ay >= 0 mirrored."""
    front = PENALTY_FRONT_X + inset
    half_w = PENALTY_WIDTH / 2 - inset
    if ax < front or ax > HALF_L - inset or ay > half_w:
        return False
    if ax < _CORNER_CX and ay > _CORNER_CY:
        r = PENALTY_CORNER_RADIUS - inset
        return math.hypot(ax - _CORNER_CX, ay - _CORNER_CY) <= r
    return True


def in_penalty_area(x, y, goal=None):
    """Is the point inside a penalty area (the white line is part of the area)?

    goal: BLUE_GOAL / YELLOW_GOAL to test one area, or None for either."""
    if goal is not None and (x > 0) != (goal_side(goal) > 0):
        return False
    return _in_rounded_area(abs(x), abs(y), 0.0)


def penalty_area_distance(x, y, goal):
    """Approximate distance from a point to the penalty area of `goal` (0 inside)."""
    s = goal_side(goal)
    ax, ay = x * s, abs(y)
    if _in_rounded_area(ax, ay, 0.0):
        return 0.0
    # distance to the rounded rectangle [front, HALF_L] x [-w/2, w/2]
    if ax < _CORNER_CX and ay > _CORNER_CY:
        return max(0.0, math.hypot(ax - _CORNER_CX, ay - _CORNER_CY) - PENALTY_CORNER_RADIUS)
    dx = max(PENALTY_FRONT_X - ax, 0.0, ax - HALF_L)
    dy = max(ay - PENALTY_WIDTH / 2, 0.0)
    return math.hypot(dx, dy)


def is_on_line(x, y):
    """True if the point is on a white line (boundary line or penalty-area line)."""
    ax, ay = abs(x), abs(y)
    if ax > HALF_L or ay > HALF_W:
        return False  # outer area is plain green carpet
    if ax >= HALF_L - LINE_WIDTH or ay >= HALF_W - LINE_WIDTH:
        return True   # boundary line
    return _in_rounded_area(ax, ay, 0.0) and not _in_rounded_area(ax, ay, LINE_WIDTH)


def in_field(x, y):
    """Inside the playing field (white line included)."""
    return abs(x) <= HALF_L and abs(y) <= HALF_W


def ball_in_goal(x, y):
    """Return the goal whose back wall the ball touches, else None."""
    if abs(y) < GOAL_WIDTH / 2 and abs(x) >= GOAL_BACK_X - BALL_RADIUS - 0.35:
        return YELLOW_GOAL if x > 0 else BLUE_GOAL
    return None


def robot_touches_wall(x, y, margin=0.4):
    return (abs(x) >= WALL_X - ROBOT_RADIUS - margin or
            abs(y) >= WALL_Y - ROBOT_RADIUS - margin)


def robot_fully_in_penalty(x, y, goal=None):
    """The whole robot disc is inside a penalty area."""
    for k in range(16):
        a = k * math.pi / 8
        if not in_penalty_area(x + ROBOT_RADIUS * math.cos(a), y + ROBOT_RADIUS * math.sin(a), goal):
            return False
    return in_penalty_area(x, y, goal)


def robot_partly_in_penalty(x, y, goal):
    return penalty_area_distance(x, y, goal) < ROBOT_RADIUS


GOAL_SIDE_WALL = 2.0   # thickness of a goal's side walls (they stand in the outer area)
_WALL_DEPTH = 30.0      # outer walls are modelled as thick blocks so nothing can tunnel through


def _box(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def wall_polygons():
    """Static collision polygons: outer walls and the goal boxes.

    The space between a goal's back wall and the outer wall is solid, like a real goal box."""
    D = _WALL_DEPTH
    polys = [
        _box(-WALL_X - D, -WALL_Y - D, WALL_X + D, -WALL_Y),
        _box(-WALL_X - D, WALL_Y, WALL_X + D, WALL_Y + D),
        _box(-WALL_X - D, -WALL_Y, -WALL_X, WALL_Y),
        _box(WALL_X, -WALL_Y, WALL_X + D, WALL_Y),
    ]
    hw = GOAL_WIDTH / 2
    for s in (-1, 1):
        mx, bx, wx = s * GOAL_MOUTH_X, s * GOAL_BACK_X, s * WALL_X
        polys.append(_box(min(bx, wx), -hw, max(bx, wx), hw))                       # behind the back wall
        polys.append(_box(min(mx, wx), hw, max(mx, wx), hw + GOAL_SIDE_WALL))       # side walls
        polys.append(_box(min(mx, wx), -hw - GOAL_SIDE_WALL, max(mx, wx), -hw))
    return polys


def goal_mouth(goal):
    """The two post points of a goal's mouth."""
    x = goal_side(goal) * GOAL_MOUTH_X
    return (x, -GOAL_WIDTH / 2), (x, GOAL_WIDTH / 2)


def neutral_spots():
    return list(NEUTRAL_SPOTS)
