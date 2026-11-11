"""Run the real Superstrika robot code inside rcj_sim.

The robot's own Python files (hunter.py, pidCalc.py, gyroMovement.py,
multipleMotors.py, servo.py, dribbler.py, kicker.py, ICamera.py, data.py, ...)
are imported *unchanged* from the Superstrika repository.  Only the hardware
layer is replaced:

    gpiozero, smbus2              -> fake modules (PWMLED / Button / DigitalOutputDevice / SMBus)
    robot.components.motor        -> SimMotor7046: speed -> simulated X omni wheels
    robot.components.gyro         -> SimMPU6050:   z angle from the simulated robot
    robot.components.webcamera    -> SimWebCamera: tilting front camera on the servo
    time.time / time.sleep        -> simulation time (only inside robot threads)

Everything the real code computes (PID, wheel mixing, statuses, timing) runs as-is.

The numbers in SETTINGS are physical assumptions about the real robot that the
code alone does not tell us - calibrate them against the real robot.
"""
import __future__
import importlib
import importlib.abc
import importlib.machinery
import math
import os
import sys
import threading
import time
import types

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_ROOT = os.path.dirname(HERE)

SUPERSTRIKA_PATH = os.environ.get(
    "SUPERSTRIKA_PATH", os.path.join(os.path.dirname(SIM_ROOT), "Superstrika-new"))

SETTINGS = {
    # move_forward_cm() drives 0.05 s per cm at setSpeed(0, 30) -> 20 cm/s, i.e. a wheel at
    # 30*cos(45deg) = 21.2 % turns at 14.1 cm/s  ->  0.667 cm/s per % of motor speed.
    "wheel_cm_s_per_percent": 0.667,
    "motor_deadband_percent": 0.0,     # motors below this |speed| % do not turn (motor7046.MIN_SPEED = 30?)
    # camera on the servo
    "camera_offset_cm": 9.0,           # camera distance forward of the robot centre (front edge)
    "camera_hfov_deg": 60.0,           # horizontal field of view
    "camera_vfov_deg": 45.0,           # vertical field of view
    "camera_range_cm": 300.0,
    "servo_horizon_angle": 180.0,      # servo angle where the camera looks straight ahead
    "servo_pitch_per_deg": 0.5,        # degrees of camera pitch-down per servo degree below that
    "camera_frame_ms": 100.0,          # time for one frame + YOLO inference on the Pi
    "goal_height_cm": 10.0,
    "ball_height_cm": 4.27,
}

_bridges = {}                # thread ident -> Bridge
_installed = False
_install_lock = threading.Lock()
_real_time = time.time
_real_sleep = time.sleep


def current_bridge():
    return _bridges.get(threading.get_ident())


# ====================================================================== simulation time
def _sim_time():
    b = current_bridge()
    if b is None:
        return _real_time()
    return b.robot.micros() / 1e6


def _sim_sleep(seconds):
    b = current_bridge()
    if b is None:
        return _real_sleep(seconds)
    b.robot.delay(max(0.0, float(seconds)) * 1000.0)


# ====================================================================== import hook
class _PostponedAnnotationsLoader(importlib.machinery.SourceFileLoader):
    """Compiles the robot code with `from __future__ import annotations` so its
    Python 3.10+ annotations (`bool | None`) also load on older interpreters.
    Never reads or writes .pyc files in the robot repository."""

    def get_code(self, fullname):
        source = self.get_data(self.path)
        return compile(source, self.path, "exec", dont_inherit=True,
                       flags=__future__.annotations.compiler_flag)


class _RobotPackageFinder(importlib.abc.MetaPathFinder):
    def __init__(self, root):
        self.root = root

    def find_spec(self, fullname, path=None, target=None):
        if fullname != "robot" and not fullname.startswith("robot."):
            return None
        search = [self.root] if fullname == "robot" else path
        spec = importlib.machinery.PathFinder.find_spec(fullname, search)
        if spec is None or spec.origin is None or not spec.origin.endswith(".py"):
            return spec
        loader = _PostponedAnnotationsLoader(fullname, spec.origin)
        spec.loader = loader
        return spec


# ====================================================================== fake gpiozero / smbus2
def _make_gpiozero():
    m = types.ModuleType("gpiozero")

    class _Device:
        def __init__(self, pin=None, *args, **kwargs):
            self.pin = pin
            self.closed = False
            self._bridge = current_bridge()

        def close(self):
            self.closed = True

    class PWMLED(_Device):
        def __init__(self, pin=None, *args, frequency=100, **kwargs):
            super().__init__(pin)
            self.frequency = frequency
            self._value = 0.0

        @property
        def value(self):
            return self._value

        @value.setter
        def value(self, v):
            self._value = float(v)
            if self._bridge is not None:
                self._bridge.pwm_written(self.pin, self._value)

        def on(self):
            self.value = 1.0

        def off(self):
            self.value = 0.0

    class DigitalOutputDevice(_Device):
        def __init__(self, pin=None, *args, **kwargs):
            super().__init__(pin)
            self.value = 0

        def on(self):
            self.value = 1
            if self._bridge is not None:
                self._bridge.digital_on(self.pin)

        def off(self):
            self.value = 0

    class Button(_Device):
        """Never pressed in the simulator (the start switch keeps its initial state)."""

        def __init__(self, pin=None, *args, **kwargs):
            super().__init__(pin)
            self.when_activated = self.when_deactivated = None
            self.when_pressed = self.when_released = None
            self.is_pressed = self.is_active = False

    class GPIODeviceClosed(Exception):
        pass

    m.PWMLED, m.DigitalOutputDevice, m.Button = PWMLED, DigitalOutputDevice, Button
    m.LED = m.OutputDevice = DigitalOutputDevice
    m.exc = types.SimpleNamespace(GPIODeviceClosed=GPIODeviceClosed)
    return m


def _make_smbus2():
    m = types.ModuleType("smbus2")

    class SMBus:
        def __init__(self, bus=None, *a, **k):
            self.bus = bus

        def write_byte_data(self, *a, **k):
            pass

        def read_byte_data(self, *a, **k):
            return 0

        def read_i2c_block_data(self, addr, reg, length, *a, **k):
            return [0] * length

        def close(self):
            pass

    m.SMBus = SMBus
    return m


# ====================================================================== fake components
def _make_motor_module():
    m = types.ModuleType("robot.components.motor")

    class motor7046:
        """Simulated DC motor. `speed` is -100..100 (%), exactly like the real class."""
        _h = 0
        MIN_SPEED = 30

        def __init__(self, pin1, pin2, switch: bool = False):
            if switch:
                pin1, pin2 = pin2, pin1
            self._pin1, self._pin2 = pin1, pin2
            self._speed = 0.0
            self._bridge = current_bridge()

        @property
        def speed(self):
            return self._speed

        @speed.setter
        def speed(self, speed: float):
            self._speed = speed / 100
            self._notify()

        def stophard(self):
            self._speed = 0
            self._notify()

        def stop(self):
            self._speed = 0
            self._notify()

        def _notify(self):
            if self._bridge is not None:
                self._bridge.motor_changed(self)

    m.motor7046 = motor7046
    return m


def _make_gyro_module():
    m = types.ModuleType("robot.components.gyro")

    class MPU6050:
        """Simulated MPU6050: integrated z angle, degrees, counter-clockwise positive."""

        def __init__(self, i2c=None, addr=0x68):
            self.i2c, self.addr = i2c, addr
            self._bridge = current_bridge()
            self.omega_offset = {"x": 0.0, "y": 0.0, "z": 0.0}
            self._zero = self._bridge.heading_ccw()
            self.theta = {"x": 0, "y": 0, "z": 0}

        def get_theta(self):
            z = self._bridge.heading_ccw() - self._zero
            self.theta = {"x": 0, "y": 0, "z": int(z)}
            return {"x": 0.0, "y": 0.0, "z": z}

        def get_z_angle(self):
            return self.get_theta()["z"]

        def reset_theta(self):
            self._zero = self._bridge.heading_ccw()
            self.theta = {"x": 0, "y": 0, "z": 0}

        def update(self):
            self.get_theta()

        def get_omega(self):
            return {"x": 0.0, "y": 0.0, "z": 0.0}

        def get_accel(self):
            return {"x": 0.0, "y": 0.0, "z": 1.0}

        def get_temp(self):
            return 25.0

        def reset_gyro_offset(self):
            pass

        def set_gyro_offset(self, offset):
            self.omega_offset = offset

        def set_gyro_sensitivity(self, factor):
            pass

        def set_accel_sensitivity(self, factor):
            pass

    m.MPU6050 = MPU6050
    return m


def _make_webcamera_module():
    from robot.abstracts.ICamera import ICamera, DisplacementVector
    from robot.consts.enum import Object

    m = types.ModuleType("robot.components.webcamera")

    class SimWebCamera(ICamera):
        """Simulated YOLO webcam on the tilt servo. Same interface as WebCamera."""

        def __init__(self, modelPath=None, objectNames=None, focalLength=0.0, *args, **kwargs):
            self._bridge = current_bridge()

        def getObjects(self, mirror: bool = False):
            self._bridge.camera_frame()
            return {obj: self._bridge.camera_view(obj, DisplacementVector)
                    for obj in (Object.Ball, Object.BlueGoal, Object.YellowGoal)}

        def isObjectDetected(self, obj):
            self._bridge.camera_frame()
            return self._bridge.camera_view(obj, DisplacementVector) is not None

    m.WebCamera = SimWebCamera
    m.SimWebCamera = SimWebCamera
    return m


# ====================================================================== installation
def install():
    """Install the import hook, fake hardware modules and the sim-time patch (once per process)."""
    global _installed
    with _install_lock:
        if _installed:
            return
        if not os.path.isdir(os.path.join(SUPERSTRIKA_PATH, "robot")):
            raise FileNotFoundError(
                f"Superstrika code not found at {SUPERSTRIKA_PATH!r}. "
                "Set the SUPERSTRIKA_PATH environment variable to the repository folder.")
        sys.meta_path.insert(0, _RobotPackageFinder(SUPERSTRIKA_PATH))
        sys.modules["gpiozero"] = _make_gpiozero()
        sys.modules["smbus2"] = _make_smbus2()

        robot_pkg = importlib.import_module("robot")
        components = importlib.import_module("robot.components")
        for name, mod in (("motor", _make_motor_module()), ("gyro", _make_gyro_module())):
            sys.modules[f"robot.components.{name}"] = mod
            setattr(components, name, mod)

        data = importlib.import_module("robot.consts.data")
        log_dir = os.path.join(SIM_ROOT, "logs")
        os.makedirs(log_dir, exist_ok=True)
        data.LOG_PATH = os.path.join(log_dir, "superstrika.log")   # hunter.py logs here

        cam = _make_webcamera_module()
        sys.modules["robot.components.webcamera"] = cam
        components.webcamera = cam

        time.time = _sim_time
        time.sleep = _sim_sleep
        _installed = True
        assert robot_pkg is not None


# ====================================================================== per-robot bridge
def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class Bridge:
    def __init__(self, sim_robot):
        from rcj_sim import BALL, BLUE_GOAL, YELLOW_GOAL
        from robot.consts.enum import Object
        import robot.consts.data as data

        self.robot = sim_robot
        self.thread = threading.get_ident()
        self.data = data
        self.S = dict(SETTINGS)
        self.wheel_pct = [0.0, 0.0, 0.0, 0.0]          # multipleMotors order: FR, RR, RL, FL
        self.servo_angle = float(data.GOOD_ANGLE)
        self._last_gyro = None
        self._heading = 0.0
        pins = data.MOTOR_PINS
        self.wheel_of_pins = {frozenset(pins[2 * i:2 * i + 2]): i for i in range(4)}
        self.dribbler_pins = frozenset(data.DRIBBLER_PIN)
        self.sim_object = {Object.Ball: BALL, Object.BlueGoal: BLUE_GOAL, Object.YellowGoal: YELLOW_GOAL}
        self.Object = Object

    def _here(self):
        return threading.get_ident() == self.thread

    # ------------------------------------------------------------ motors
    def motor_changed(self, motor):
        if not self._here():
            return   # e.g. Hunt.__del__ during garbage collection
        pins = frozenset((motor._pin1, motor._pin2))
        if pins == self.dribbler_pins:
            self.robot.set_dribbler(motor._speed != 0)
            return
        idx = self.wheel_of_pins.get(pins)
        if idx is None:
            return
        self.wheel_pct[idx] = motor._speed * 100
        k = self.S["wheel_cm_s_per_percent"]
        dead = self.S["motor_deadband_percent"]
        cm = [0.0 if abs(p) < dead else p * k for p in self.wheel_pct]
        fr, rr, rl, fl = cm
        self.robot.set_wheel_speeds(fr, fl, rl, rr)

    # ------------------------------------------------------------ gpio
    def pwm_written(self, pin, value):
        if pin == self.data.SERVO_PIN and value > 0:
            # Servo.calculateDuty: duty% = 2.5 + angle/180*10
            self.servo_angle = (value * 100 - 2.5) / 10 * 180

    def digital_on(self, pin):
        if pin == getattr(self.data, "KICKER_PIN", None) and self._here():
            self.robot.kick()

    # ------------------------------------------------------------ gyro
    def heading_ccw(self):
        """Unwrapped heading, degrees counter-clockwise (MPU6050 z axis pointing up)."""
        g = -self.robot.get_angle()
        if self._last_gyro is not None:
            self._heading += _wrap180(g - self._last_gyro)
        self._last_gyro = g
        return self._heading

    # ------------------------------------------------------------ camera
    def camera_frame(self):
        self.robot.delay(self.S["camera_frame_ms"])

    def camera_view(self, obj, DisplacementVector):
        """What the tilted front camera sees: DisplacementVector(distance_cm, angle_deg) or None.
        Angle is positive to the right, distance is along the floor from the camera."""
        S = self.S
        loc = self.robot.get_object_location(self.sim_object[obj])
        if loc is None:
            return None
        d, a = loc
        fx = d * math.cos(math.radians(a)) - S["camera_offset_cm"]   # forward of the camera
        fy = d * math.sin(math.radians(a))                          # to the right
        if fx <= 0:
            return None
        dist = math.hypot(fx, fy)
        ang = math.degrees(math.atan2(fy, fx))
        if abs(ang) > S["camera_hfov_deg"] / 2 or dist > S["camera_range_cm"]:
            return None
        # vertical field of view of the tilted camera
        h = float(self.data.CAMERA_HEIGHT_CM)
        pitch = max(0.0, (S["servo_horizon_angle"] - self.servo_angle) * S["servo_pitch_per_deg"])
        half_v = S["camera_vfov_deg"] / 2
        top_h = S["ball_height_cm"] if obj == self.Object.Ball else S["goal_height_cm"]
        dep_bottom = math.degrees(math.atan2(h, dist))          # looking down at the object's base
        dep_top = math.degrees(math.atan2(h - top_h, dist))     # ... and at its top
        if dep_bottom < pitch - half_v or dep_top > pitch + half_v:
            return None
        return DisplacementVector(dist, ang)


# ====================================================================== entry point
def run_hunter(sim_robot):
    """Create the real Hunt robot on simulated hardware and run hunt()."""
    install()
    bridge = Bridge(sim_robot)
    _bridges[bridge.thread] = bridge
    try:
        hunter = importlib.import_module("robot.hunter")
        hunt = hunter.Hunt(debug=False)
        if not hasattr(hunt, "camera"):
            # hunter.py has the camera line commented out; attach the simulated one
            hunt.camera = sys.modules["robot.components.webcamera"].WebCamera(None)
        hunt.hunt()
    finally:
        _bridges.pop(bridge.thread, None)
