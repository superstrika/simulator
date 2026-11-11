"""Simulation core: ties physics, robot hardware, robot programs and the referee together."""
import random
from collections import deque

from .constants import BLUE_GOAL, TEAMS, YELLOW_GOAL
from .hardware import RobotHardware
from .physics import World
from .referee import Referee
from .runner import RobotController, install_stdout_router


class RobotEntity:
    def __init__(self, sim, cfg, index):
        self.cfg = cfg
        self.index = index
        self.body = sim.world.add_robot(cfg.name)
        self.hw = RobotHardware(sim, self.body, random.Random(sim.cfg.sim.seed * 1000 + index))
        self.ctrl = RobotController(sim, self)
        self.penalty_until = None   # sim time when an out-of-bounds penalty ends
        self.damaged = False        # removed by hand (UI right-click)

    @property
    def available(self):
        return self.penalty_until is None and not self.damaged

    @property
    def status(self):
        if self.damaged:
            return "damaged"
        if self.penalty_until is not None:
            return "penalty"
        return self.ctrl.state


class Simulation:
    def __init__(self, cfg, echo_logs=True):
        cfg.validate()
        self.cfg = cfg
        self.echo_logs = echo_logs
        self.dt = 1.0 / cfg.sim.physics_hz
        self.step_count = 0
        self.time = 0.0
        self.world = World(cfg.sim)
        self.sides_swapped = False
        self.finished = False
        self.events = deque(maxlen=100)
        self.goal_log = []
        self.warnings = []

        robots = [r for r in cfg.robots if r.enabled]
        if cfg.referee.mode == "full":
            kept = []
            for t in TEAMS:
                team = [r for r in robots if r.team == t]
                if len(team) > cfg.referee.max_robots_per_team:
                    self.warnings.append(
                        f"full mode: team {t} has {len(team)} robots, only the first "
                        f"{cfg.referee.max_robots_per_team} play")
                kept += team[:cfg.referee.max_robots_per_team]
            robots = [r for r in robots if r in kept]
        self.entities = [RobotEntity(self, r, i) for i, r in enumerate(robots)]
        self.referee = Referee(self)
        install_stdout_router()
        for w in self.warnings:
            self.event(w)

    # ------------------------------------------------------------ queries
    def defended_goal(self, team):
        g = BLUE_GOAL if team == "blue" else YELLOW_GOAL
        if self.sides_swapped:
            g = YELLOW_GOAL if g == BLUE_GOAL else BLUE_GOAL
        return g

    def entity(self, name):
        for e in self.entities:
            if e.cfg.name == name:
                return e
        raise KeyError(name)

    def event(self, text):
        self.events.append((self.time, text))
        if self.echo_logs:
            from .runner import _real_stdout
            _real_stdout.write(f"[{self.time:7.2f}s REF] {text}\n")

    # ------------------------------------------------------------ control
    def start(self):
        self.referee.start_match()
        self.restart_programs()

    def restart_programs(self):
        for e in self.entities:
            e.ctrl.start()

    def set_frozen(self, frozen):
        for e in self.entities:
            e.body.frozen = frozen

    def reset(self):
        """Stop everything and start a new match."""
        self.shutdown()
        self.time = 0.0
        self.step_count = 0
        self.finished = False
        self.sides_swapped = False
        self.goal_log = []
        for e in self.entities:
            e.penalty_until = None
            e.damaged = False
        self.referee = Referee(self)
        self.start()

    def shutdown(self):
        for e in self.entities:
            e.ctrl.stop()

    def step(self):
        if self.finished:
            return
        for e in self.entities:
            e.ctrl.service()
        self.world.step(self.dt)
        self.step_count += 1
        self.time = self.step_count * self.dt
        for e in self.entities:
            e.hw.update_line_sensors()
        self.referee.update()

    def run_for(self, seconds):
        end = self.step_count + int(round(seconds / self.dt))
        while self.step_count < end and not self.finished:
            self.step()

    # ------------------------------------------------------------ manual referee (UI)
    def toggle_damaged(self, e):
        if e.damaged:
            e.damaged = False
            if e.penalty_until is None:
                self.referee.place_on_neutral_spot(e)
                e.body.frozen = self.referee.state != "playing"
            self.event(f"{e.cfg.name} back on the field")
        else:
            e.damaged = True
            e.body.remove()
            e.hw.line_state = [False] * len(e.hw.line_state)
            self.event(f"{e.cfg.name} removed (damaged)")

    def move_ball(self, x, y):
        self.world.place_ball(x, y)

    def move_robot(self, e, x, y, angle=None):
        if not e.body.on_field:
            return
        e.body.place(x, y, e.body.angle if angle is None else angle)
