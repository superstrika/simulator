"""Example goalie: rides the front line of its own penalty area and tracks the ball sideways.

Uses polling of the line sensors (no interrupt) to stay on the line.
"""
from rcj_sim import BALL

FRONT = (15, 0, 1)
BACK = (7, 8, 9)
SIDES = (3, 4, 5, 11, 12, 13)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def main(robot):
    robot.set_dribbler(False)
    print("goalie ready, defending", robot.my_goal)
    while True:
        goal = robot.get_object_angle(robot.opponent_goal)   # None if hidden (occlusion option)
        omega = clamp(goal * 5, -300, 300) if goal is not None else 0
        lines = robot.get_line_sensors()
        front_on = any(lines[i] for i in FRONT)
        back_on = any(lines[i] for i in BACK)
        side_on = any(lines[i] for i in SIDES)

        # forward / backward: keep the back sensors on the penalty-area line
        if front_on:
            vy = 50          # we slipped over the line: go forward
        elif side_on and not back_on:
            vy = 30
        elif back_on:
            vy = 0
        else:
            vy = -50         # find the line behind us

        # sideways: stay between the ball and the goal
        ball = robot.get_object_angle(BALL)
        vx = 0.0
        if ball is not None and abs(ball) < 100:
            vx = clamp(ball * 3, -100, 100)
        own = robot.get_object_angle(robot.my_goal)
        if own is not None and abs(own) < 145:      # drifted too far to one side
            vx = 60 if own > 0 else -60

        if robot.has_ball():
            robot.kick()
        robot.set_speed(vx, vy, omega)
        robot.delay(10)
