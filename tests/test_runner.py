import os

from conftest import IDLE, ROOT, logs
from rcj_sim.config import load_yaml
from rcj_sim.sim import Simulation


def test_same_script_separate_globals(make_sim):
    code = """
count = 0
def main(robot):
    global count
    for _ in range(5):
        count += 1
        robot.delay(10)
    print("count", count)
    while True:
        robot.delay(1000)
"""
    sim = make_sim([("A", "blue", code), ("B", "yellow", code)])
    # make both robots share one file
    sim.shutdown()
    sim.entity("B").ctrl.code_path = sim.entity("A").ctrl.code_path
    sim.reset()
    sim.run_for(0.5)
    assert logs(sim, "A") == ["count 5"]
    assert logs(sim, "B") == ["count 5"]


def test_delay_and_millis(make_sim):
    code = """
def main(robot):
    t0 = robot.millis()
    robot.delay(250)
    t1 = robot.millis()
    print(t1 - t0)
    while True:
        robot.delay(1000)
"""
    sim = make_sim([("R", "blue", code)])
    sim.run_for(0.5)
    assert int(logs(sim, "R")[0]) in (250, 251)


def test_busy_loop_does_not_freeze(make_sim):
    code = """
def main(robot):
    while True:
        robot.get_angle()          # no delay(), but API calls cost sim time
"""
    sim = make_sim([("R", "blue", code)])
    sim.run_for(0.5)
    assert sim.time >= 0.5
    assert sim.entity("R").ctrl.state == "running"


def test_crash_is_reported(make_sim):
    code = """
def main(robot):
    robot.set_speed(0, 50, 0)
    robot.delay(50)
    1 / 0
"""
    sim = make_sim([("R", "blue", code)])
    sim.run_for(0.3)
    e = sim.entity("R")
    assert e.ctrl.state == "crashed"
    assert "ZeroDivisionError" in e.ctrl.error
    assert e.hw.wheels == [0, 0, 0, 0]


def test_watchdog_marks_hung_then_recovers(make_sim):
    code = """
import time
def main(robot):
    t = time.time()
    while time.time() - t < 0.6:
        pass                       # stuck without touching the robot API
    while True:
        robot.delay(10)
"""
    sim = make_sim([("R", "blue", code)], sim={"watchdog_s": 0.2})
    sim.step()
    e = sim.entity("R")
    assert e.ctrl.state == "hung"
    import time
    time.sleep(0.7)
    sim.run_for(0.1)
    assert e.ctrl.state == "running"


def test_reset_restarts_program(make_sim):
    code = """
def main(robot):
    print("start")
    while True:
        robot.delay(100)
"""
    sim = make_sim([("R", "blue", code)])
    sim.run_for(0.2)
    sim.reset()
    sim.run_for(0.2)
    assert logs(sim, "R") == ["start", "start"]


def test_deterministic_matches():
    def run():
        cfg = load_yaml(os.path.join(ROOT, "examples", "match.yaml"))
        sim = Simulation(cfg, echo_logs=False)
        sim.start()
        sim.run_for(15)
        out = (tuple(sim.world.ball.position), tuple((e.body.x, e.body.y) for e in sim.entities))
        sim.shutdown()
        return out
    assert run() == run()


def test_example_match_runs_clean():
    cfg = load_yaml(os.path.join(ROOT, "examples", "match.yaml"))
    cfg.referee.half_duration_s = 30
    sim = Simulation(cfg, echo_logs=False)
    sim.start()
    sim.run_for(65)
    try:
        assert sim.finished
        assert sim.referee.half == 2
        for e in sim.entities:
            assert e.ctrl.error is None, e.ctrl.error
    finally:
        sim.shutdown()


def test_idle_constant_used():
    assert "def main" in IDLE
