"""Example striker: turns to the ball, catches it with the dribbler,
turns towards the opponent goal and kicks.

Works for either team because it uses robot.opponent_goal.
"""
import math

from rcj_sim import BALL

SPEED = 130          # cm/s
robot = None
escape_until = 0     # ms: while millis() < escape_until we drive away from a line
escape_dir = 0.0     # degrees, robot frame (0 = forward, + = clockwise)


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def angle_diff(a, b):
    return (a - b + 180) % 360 - 180


def drive(direction, speed, omega=0.0):
    """Move in `direction` (degrees clockwise from forward) at `speed` cm/s."""
    rad = math.radians(direction)
    robot.set_speed(speed * math.sin(rad), speed * math.cos(rad), omega)


def on_line(sensor):
    """Interrupt: a line sensor touched a white line. Back off in the opposite direction,
    unless the ball is just beyond that line (then go and get it)."""
    global escape_until, escape_dir
    sensor_angle = robot.line_sensor_angle(sensor)
    if robot.has_ball():
        return
    ball = robot.get_object_angle(BALL)
    if ball is not None and abs(angle_diff(ball, sensor_angle)) < 90:
        return
    escape_dir = sensor_angle + 180
    escape_until = robot.millis() + 200
    drive(escape_dir, SPEED)


def main(r):
    global robot
    robot = r
    robot.set_interrupt(on_line)
    robot.set_dribbler(True)
    print("striker ready, attacking", robot.opponent_goal)

    while True:
        if robot.millis() < escape_until:
            drive(escape_dir, SPEED)

        elif robot.has_ball():
            # turn with the ball towards the goal, then shoot
            goal_angle = robot.get_object_angle(robot.opponent_goal)
            if goal_angle is None:                       # hidden behind a robot: use the gyro
                goal_angle = -robot.get_angle()
            goal = robot.get_object_location(robot.opponent_goal)
            if goal is not None and abs(goal[1]) < 10 and goal[0] < 130:
                if robot.kick():
                    print(f"kick! goal {goal[0]:.0f} cm away")
            omega = clamp(goal_angle * 4, -240, 240)
            if abs(goal_angle) < 20:
                drive(0, 90, omega)
            else:
                drive(180, 20, omega)    # turn while pulling the ball back (away from walls)

        else:
            ball_angle = robot.get_object_angle(BALL)
            if ball_angle is None:
                robot.motor_stop()
            else:
                front = robot.get_object_location(BALL)
                dist = front[0] if front else 100.0
                omega = clamp(ball_angle * 6, -400, 400)        # face the ball
                speed = clamp(dist * 2.0, 40, SPEED)
                if abs(ball_angle) > 60:
                    speed = 30                                   # mostly turn first
                if any(robot.get_line_sensors()):
                    speed = min(speed, 35)                       # careful near the walls
                drive(ball_angle * 0.8, speed, omega)

        robot.delay(10)
