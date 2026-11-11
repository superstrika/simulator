"""Line-sensor demo: drive forward until a line interrupt fires, then turn around.

Shows set_interrupt(), gyro and the front camera.
"""
from rcj_sim import BALL

hits = []


def on_line(robot, sensor):          # two-argument form also receives the robot
    hits.append(sensor)
    print(f"line! sensor {sensor} ({robot.line_sensor_angle(sensor):.1f} deg)  gyro={robot.get_angle():.1f}")
    robot.motor_stop()


def main(robot):
    robot.set_interrupt(on_line)
    direction = 1
    while True:
        hits.clear()
        robot.set_speed(0, 60 * direction, 0)
        while not hits:
            robot.delay(5)
        robot.set_speed(0, -60 * direction, 0)   # back off the line
        robot.delay(400)
        direction = -direction
        print("ball seen by front camera:", robot.get_object_location(BALL))
