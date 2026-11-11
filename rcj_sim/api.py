"""The robot programming interface.

Write a Python file with a ``main(robot)`` function::

    from rcj_sim import BALL

    def on_line(sensor):          # called when line sensor 0..15 touches a white line
        print("line on sensor", sensor)

    def main(robot):
        robot.set_interrupt(on_line)
        while True:
            angle = robot.get_object_angle(BALL)
            robot.set_speed(0, 80, 0)
            robot.delay(10)

Conventions
-----------
* Distances in cm, speeds in cm/s, angles in degrees.
* Angles are in (-180, 180]: 0 = straight ahead, positive = clockwise (to the right).
* ``set_speed(vx, vy, omega)``: vx = to the right, vy = forward, omega = deg/s clockwise.
* Line sensors: 16 TCRTs on a ring; index 0 is at the front and indices increase
  clockwise in 22.5 degree steps (4 = right, 8 = back, 12 = left).
"""
from .constants import BALL, BLUE_GOAL, TCRT_COUNT, YELLOW_GOAL


class Robot:
    """Handle to one simulated robot. Passed to your ``main(robot)``."""

    def __init__(self, controller):
        self._c = controller
        self._gen = controller.generation
        self._hw = controller.hw
        self._entity = controller.entity

    def _call(self):
        self._c.api_call(self._gen)

    # ------------------------------------------------------------ identity
    @property
    def name(self):
        """Robot name from the match configuration."""
        return self._entity.cfg.name

    @property
    def team(self):
        """'blue' or 'yellow' (the goal this team defends in the 1st half)."""
        return self._entity.cfg.team

    @property
    def my_goal(self):
        """The goal this robot is defending right now (changes at half time)."""
        return self._c.sim.defended_goal(self.team)

    @property
    def opponent_goal(self):
        """The goal this robot should attack right now."""
        return YELLOW_GOAL if self.my_goal == BLUE_GOAL else BLUE_GOAL

    # ------------------------------------------------------------ motion
    def set_speed(self, vx, vy, omega):
        """Drive the X-shaped omni wheels.

        vx: cm/s to the right, vy: cm/s forward, omega: deg/s clockwise.
        If a wheel would exceed its max speed all wheels are scaled down together."""
        self._call()
        self._hw.set_speed(vx, vy, omega)

    def motor_stop(self):
        """Stop all four drive motors."""
        self._call()
        self._hw.stop()

    def set_wheel_speeds(self, front_right, front_left, back_left, back_right):
        """Low-level: set each omni wheel surface speed in cm/s."""
        self._call()
        from .hardware import saturate
        self._hw.set_wheels(saturate([front_right, front_left, back_left, back_right],
                                     self._hw.cfg.max_wheel_speed))

    def get_wheel_speeds(self):
        """Current wheel speeds [front_right, front_left, back_left, back_right] in cm/s."""
        self._call()
        return list(self._hw.wheels)

    # ------------------------------------------------------------ gyro
    def get_angle(self):
        """Gyro heading in degrees, relative to the heading when the program started
        (or the last reset_angle()). Positive = turned clockwise."""
        self._call()
        return self._hw.gyro()

    def reset_angle(self):
        """Make the current heading the new 0 of the gyro."""
        self._call()
        self._hw.reset_gyro_zero()

    # ------------------------------------------------------------ cameras
    def get_object_angle(self, obj):
        """360-degree camera: angle to BALL, BLUE_GOAL or YELLOW_GOAL from anywhere
        on the field. Returns None only if the object is hidden (occlusion option)."""
        self._call()
        return self._hw.object_angle(obj)

    def get_object_location(self, obj):
        """Front camera: (distance_cm, angle_deg) to BALL, BLUE_GOAL or YELLOW_GOAL,
        or None if the object is not inside the forward field of view."""
        self._call()
        return self._hw.object_location(obj)

    # ------------------------------------------------------------ line sensors
    def set_interrupt(self, func):
        """Register func(sensor_index) - or func(robot, sensor_index) - to run when
        any of the 16 line sensors moves onto a white line. Pass None to remove."""
        self._call()
        self._c.set_isr(func, self)

    def get_line_sensors(self):
        """Current state of the 16 line sensors (True = on a white line)."""
        self._call()
        return list(self._hw.line_state)

    LINE_SENSOR_COUNT = TCRT_COUNT

    @staticmethod
    def line_sensor_angle(index):
        """Mounting angle of a line sensor, degrees clockwise from the front."""
        return (index % TCRT_COUNT) * 360.0 / TCRT_COUNT

    # ------------------------------------------------------------ ball handling
    def set_dribbler(self, on):
        """Turn the dribbler on (True) or off (False)."""
        self._call()
        self._hw.set_dribbler(on)

    def dribbler_on(self):
        self.set_dribbler(True)

    def dribbler_off(self):
        self.set_dribbler(False)

    def kick(self):
        """Fire the kicker forward. Returns True if the ball was kicked.
        The kicker needs kick_cooldown seconds (default 0.5) to recharge."""
        self._call()
        return self._hw.kick()

    def has_ball(self):
        """True if the ball is in the capture zone at the front (light-gate sensor)."""
        self._call()
        return self._hw.has_ball()

    # ------------------------------------------------------------ time / misc
    def delay(self, ms):
        """Sleep for ms milliseconds of simulation time."""
        self._c.delay(self._gen, ms)

    def millis(self):
        """Milliseconds of simulation time since the simulator started."""
        self._call()
        return int(self._c.local_time * 1000)

    def micros(self):
        """Microseconds of simulation time (finer resolution than millis())."""
        self._call()
        return int(round(self._c.local_time * 1_000_000))

    def log(self, *args):
        """Print a message tagged with this robot's name (same as print)."""
        self._c.log_write(" ".join(str(a) for a in args) + "\n")


__all__ = ["Robot", "BALL", "BLUE_GOAL", "YELLOW_GOAL"]
