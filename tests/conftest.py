import os
import sys
import textwrap

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from rcj_sim.config import MatchConfig, RobotConfig  # noqa: E402
from rcj_sim.sim import Simulation  # noqa: E402

IDLE = """
def main(robot):
    while True:
        robot.delay(1000)
"""


@pytest.fixture
def make_sim(tmp_path):
    """make_sim([(name, team, code_str), ...], mode='free', sim={...}, referee={...})"""
    sims = []

    def _make(robots, mode="free", sim=None, referee=None, start=True):
        cfg = MatchConfig()
        cfg.referee.mode = mode
        for k, v in (sim or {}).items():
            setattr(cfg.sim, k, v)
        for k, v in (referee or {}).items():
            setattr(cfg.referee, k, v)
        for name, team, code in robots:
            p = tmp_path / f"{name}.py"
            p.write_text(textwrap.dedent(code))
            cfg.robots.append(RobotConfig(name=name, team=team, code=str(p)))
        s = Simulation(cfg, echo_logs=False)
        sims.append(s)
        if start:
            s.start()
        return s

    yield _make
    for s in sims:
        s.shutdown()


def logs(sim, name):
    return [line for _, line in sim.entity(name).ctrl.logs]
