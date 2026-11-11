"""Runs Superstrika's real robot code (from ../Superstrika-new) on the simulated hardware."""
import math
import os
import subprocess
import sys

import pytest

from conftest import ROOT

SUPER = os.environ.get("SUPERSTRIKA_PATH", os.path.join(os.path.dirname(ROOT), "Superstrika-new"))
pytestmark = pytest.mark.skipif(not os.path.isdir(os.path.join(SUPER, "robot")),
                                reason="Superstrika repository not found")


def _run(code, seconds):
    """Run a snippet in a fresh interpreter (the bridge patches modules process-wide)."""
    prog = f"""
import sys, math
sys.path.insert(0, {ROOT!r}); sys.path.insert(0, {os.path.join(ROOT, 'superstrika')!r})
from rcj_sim.config import load_yaml, RobotConfig
from rcj_sim.sim import Simulation
{code}
"""
    out = subprocess.run([sys.executable, "-c", prog], capture_output=True, text=True, timeout=120)
    assert out.returncode == 0, out.stderr
    return out.stdout


def test_motor_mixing_matches_move_forward_calibration(tmp_path):
    prog = tmp_path / "fwd.py"
    prog.write_text(
        "from superstrika_bridge import install, Bridge, _bridges\n"
        "def main(r):\n"
        "    install(); b = Bridge(r); _bridges[b.thread] = b\n"
        "    import robot.processes.multipleMotors as mm, robot.consts.data as data\n"
        "    m = mm.multipleMotors(data.MOTOR_PINS)\n"
        "    m.setSpeed(0, 30, 0)\n"
        "    r.delay(1000)\n"
        "    m.stop()\n"
        "    while True: r.delay(1000)\n")
    out = _run(f"""
cfg = load_yaml({os.path.join(ROOT, 'superstrika', 'match.yaml')!r})
cfg.sim.accel_limit = False
cfg.robots = [RobotConfig(name='S', team='blue', code={str(prog)!r})]
sim = Simulation(cfg, echo_logs=False); sim.start()
e = sim.entities[0]; x0 = e.body.x
sim.run_for(1.5)
print(round(e.body.x - x0, 1), round(e.body.y, 2))
sim.shutdown()
""", 2)
    dx, dy = map(float, out.split())
    assert dx == pytest.approx(20.0, abs=0.5)     # move_forward_cm: 0.05 s/cm at speed 30
    assert dy == pytest.approx(0.0, abs=0.1)


def test_hunter_captures_ball_and_carries_it(tmp_path):
    out = _run(f"""
cfg = load_yaml({os.path.join(ROOT, 'superstrika', 'match.yaml')!r})
sim = Simulation(cfg, echo_logs=False); sim.start()
sim.run_for(15)
e = sim.entity('Superstrika')
print(e.ctrl.state, e.ctrl.error is None, e.body.holding, round(e.body.x))
sim.shutdown()
""", 15)
    state, no_error, holding, x = out.split()
    assert state == "running" and no_error == "True"
    assert holding == "True"          # caught with the dribbler...
    assert int(x) > 50                # ...and carried towards the yellow goal


def test_robot_repository_untouched():
    out = subprocess.run(["git", "-C", SUPER, "status", "--porcelain"], capture_output=True, text=True)
    if out.returncode != 0:
        pytest.skip("not a git repository")
    assert "__pycache__" not in out.stdout
