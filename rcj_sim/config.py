"""Match configuration (YAML file or command-line)."""
import os
from dataclasses import dataclass, field, fields
from typing import List, Optional

from .constants import HALF_DURATION, MAX_ROBOTS_PER_TEAM, TEAMS

MODES = ("free", "basic", "full")


@dataclass
class RobotConfig:
    name: str
    team: str                      # "blue" / "yellow": the goal it defends in the 1st half
    code: str                      # path to the robot's Python file
    enabled: bool = True
    start: Optional[dict] = None   # {x, y, heading}; heading 0 = facing the opponent goal, + clockwise


@dataclass
class SimConfig:
    physics_hz: int = 200
    seed: int = 0
    # motors
    max_wheel_speed: float = 120.0        # cm/s per omni wheel (robot max ~170 cm/s)
    accel_limit: bool = False             # realistic: limit acceleration (and pushing force)
    max_accel: float = 400.0              # cm/s^2 when accel_limit is on
    # kicker / dribbler
    kick_speed: float = 300.0             # cm/s ball speed after a kick
    kick_cooldown: float = 0.5            # s for the capacitor to recharge
    dribbler_grip: float = 3000.0         # max holding acceleration on the ball, cm/s^2
    ball_decel: float = 80.0              # rolling friction deceleration, cm/s^2
    # cameras
    front_fov_deg: float = 60.0
    front_range_cm: float = 250.0
    occlusion: bool = False               # other robots can hide the ball / goals
    # noise
    noise: bool = False
    angle_noise_deg: float = 2.0
    distance_noise_pct: float = 3.0
    gyro_noise_deg: float = 0.3
    gyro_drift_deg_per_min: float = 1.0
    # scheduler
    api_call_cost_ms: float = 0.1         # simulated CPU time used by every robot API call
    watchdog_s: float = 2.0               # real seconds without an API call -> robot marked hung
    echo_prints: bool = True              # echo robot print() output to the console


@dataclass
class RefereeConfig:
    mode: str = "basic"                   # free | basic | full
    half_duration_s: float = HALF_DURATION
    halves: int = 2
    swap_sides: bool = True
    first_kickoff: str = "blue"
    goal_hold_s: float = 1.0              # robots frozen after a goal / kick-off placement
    # full mode rules
    out_of_bounds: bool = True
    out_of_bounds_penalty_s: float = 60.0
    multiple_defense: bool = True
    lack_of_progress: bool = True
    lack_of_progress_s: float = 5.0
    lack_of_progress_cm: float = 3.0
    max_robots_per_team: int = MAX_ROBOTS_PER_TEAM


@dataclass
class MatchConfig:
    robots: List[RobotConfig] = field(default_factory=list)
    sim: SimConfig = field(default_factory=SimConfig)
    referee: RefereeConfig = field(default_factory=RefereeConfig)

    def validate(self):
        if self.referee.mode not in MODES:
            raise ValueError(f"mode must be one of {MODES}, got {self.referee.mode!r}")
        if self.referee.first_kickoff not in TEAMS:
            raise ValueError("first_kickoff must be 'blue' or 'yellow'")
        names = set()
        for r in self.robots:
            if r.team not in TEAMS:
                raise ValueError(f"robot {r.name}: team must be 'blue' or 'yellow'")
            if r.name in names:
                raise ValueError(f"duplicate robot name {r.name!r}")
            names.add(r.name)
            if not os.path.isfile(r.code):
                raise ValueError(f"robot {r.name}: code file not found: {r.code}")
        return self


def _apply(dc, data, label):
    known = {f.name for f in fields(dc)}
    for k, v in (data or {}).items():
        if k not in known:
            raise ValueError(f"unknown {label} option {k!r}")
        setattr(dc, k, v)


def load_yaml(path):
    import yaml
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    base = os.path.dirname(os.path.abspath(path))
    cfg = MatchConfig()
    _apply(cfg.sim, data.get("sim"), "sim")
    _apply(cfg.referee, data.get("referee"), "referee")
    if "mode" in data:
        cfg.referee.mode = data["mode"]
    for i, r in enumerate(data.get("robots") or []):
        code = r["code"]
        if not os.path.isabs(code):
            code = os.path.join(base, code)
        cfg.robots.append(RobotConfig(
            name=str(r.get("name", f"R{i + 1}")), team=r["team"], code=code,
            enabled=bool(r.get("enabled", True)), start=r.get("start")))
    return cfg.validate()


def from_lists(blue, yellow, mode="basic"):
    """Quick config: lists of code paths for each team."""
    cfg = MatchConfig()
    cfg.referee.mode = mode
    for team, paths in (("blue", blue or []), ("yellow", yellow or [])):
        for i, p in enumerate(paths):
            cfg.robots.append(RobotConfig(name=f"{team[0].upper()}{i + 1}", team=team, code=p))
    return cfg.validate()
