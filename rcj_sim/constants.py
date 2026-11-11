"""Official RoboCupJunior Soccer dimensions and simulator defaults.

All lengths are in centimetres, masses in kilograms, times in seconds.

Sources:
  * Rules:               https://robocup-junior.github.io/soccer-rules/master/rules.html
  * Field specification: https://robocup-junior.github.io/soccer-rules/master/field_specification.html
  * Ball specification:  https://robocup-junior.github.io/soccer-rules/master/ball_specification.pdf

The robot size requested (diameter 18 cm, height 18 cm) is the *Soccer Vision*
(formerly "Open") league limit, so this simulator uses that league's ball
(an orange golf ball).

World coordinate system: origin at the centre of the field, x along the long
axis, y along the short axis (y up on screen).  The BLUE goal is at -x and the
YELLOW goal at +x.
"""

# ---------------------------------------------------------------- field
FIELD_LENGTH = 219.0        # playing field incl. white line (x extent)
FIELD_WIDTH = 158.0         # playing field incl. white line (y extent)
OUTER_AREA = 12.0           # green area between white line and wall
LINE_WIDTH = 2.0            # white lines are 20 mm wide
WALL_HEIGHT = 22.0          # matte black walls (drawn only)

HALF_L = FIELD_LENGTH / 2   # 109.5  outer edge of the white boundary line
HALF_W = FIELD_WIDTH / 2    # 79.0
WALL_X = HALF_L + OUTER_AREA  # 121.5
WALL_Y = HALF_W + OUTER_AREA  # 91.0

# ---------------------------------------------------------------- goals
GOAL_WIDTH = 60.0           # inner width
GOAL_DEPTH = 7.4            # inner depth (74 mm)
GOAL_HEIGHT = 10.0          # inner height (drawn only)
GOAL_WALL_THICKNESS = 1.0
GOAL_MOUTH_X = HALF_L       # posts stand over the white line
GOAL_BACK_X = HALF_L + GOAL_DEPTH  # 116.9

# ---------------------------------------------------------------- markings
PENALTY_WIDTH = 80.0        # along y
PENALTY_DEPTH = 25.0        # along x, measured from the outer edge of the boundary line
PENALTY_CORNER_RADIUS = 15.0
CENTER_CIRCLE_DIAMETER = 60.0
NEUTRAL_SPOT_FROM_EDGE = 45.0

NEUTRAL_SPOTS = [
    (0.0, 0.0),
    (-(HALF_L - NEUTRAL_SPOT_FROM_EDGE), PENALTY_WIDTH / 2),
    (-(HALF_L - NEUTRAL_SPOT_FROM_EDGE), -PENALTY_WIDTH / 2),
    ((HALF_L - NEUTRAL_SPOT_FROM_EDGE), PENALTY_WIDTH / 2),
    ((HALF_L - NEUTRAL_SPOT_FROM_EDGE), -PENALTY_WIDTH / 2),
]

# ---------------------------------------------------------------- ball (Soccer Vision: golf ball)
BALL_DIAMETER = 4.27
BALL_RADIUS = BALL_DIAMETER / 2
BALL_MASS = 0.046

# ---------------------------------------------------------------- robot
ROBOT_DIAMETER = 18.0
ROBOT_RADIUS = ROBOT_DIAMETER / 2
ROBOT_HEIGHT = 18.0
ROBOT_MASS = 1.2
CAPTURE_DEPTH = 1.5          # ball may enter the convex hull by at most 1.5 cm
NOTCH_HALF_WIDTH = 2.3       # half-width of the ball-capturing notch
NOTCH_BACK_X = ROBOT_RADIUS - CAPTURE_DEPTH   # 7.5, local x of the notch back plate
HELD_BALL_X = NOTCH_BACK_X + BALL_RADIUS      # local x of a ball resting in the notch
WHEEL_BASE_RADIUS = 7.0      # distance from centre to each omni wheel
TCRT_COUNT = 16
TCRT_RADIUS = 7.0            # radius of the light-sensor ring

# ---------------------------------------------------------------- match
TEAMS = ("blue", "yellow")   # a team is named after the goal it defends in the 1st half
MAX_ROBOTS_PER_TEAM = 2
HALF_DURATION = 600.0        # 2 halves of 10 minutes
KICKOFF_CLEARANCE = 30.0     # non-kicking robots must be >= 30 cm from the ball

# object identifiers used by the robot API
BALL = "ball"
BLUE_GOAL = "blue_goal"
YELLOW_GOAL = "yellow_goal"
OBJECTS = (BALL, BLUE_GOAL, YELLOW_GOAL)

GOAL_X = {BLUE_GOAL: -GOAL_MOUTH_X, YELLOW_GOAL: GOAL_MOUTH_X}
