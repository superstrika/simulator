import os

from conftest import IDLE, ROOT
from rcj_sim import constants as C
from rcj_sim.config import load_yaml, save_yaml


def test_hide_and_show_restores_position(make_sim):
    sim = make_sim([("B1", "blue", IDLE)], mode="basic")
    e = sim.entity("B1")
    sim.move_robot(e, -40, 30, 0.5)
    sim.hide(e)
    assert e.hidden and not e.body.on_field and e.status == "hidden"
    sim.run_for(0.5)
    assert e.ctrl.state == "running"            # its program keeps running while hidden
    sim.show(e)
    assert not e.hidden and e.body.on_field
    assert (round(e.body.x, 3), round(e.body.y, 3), round(e.body.angle, 3)) == (-40, 30, 0.5)


def test_show_on_occupied_place_uses_neutral_spot(make_sim):
    sim = make_sim([("B1", "blue", IDLE), ("Y1", "yellow", IDLE)], mode="basic")
    b, y = sim.entity("B1"), sim.entity("Y1")
    sim.move_robot(b, -40, 30)
    sim.hide(b)
    sim.move_robot(y, -42, 30)                   # someone took the spot
    sim.show(b)
    assert (round(b.body.x, 3), round(b.body.y, 3)) in [tuple(map(float, p)) for p in C.NEUTRAL_SPOTS]


def test_hidden_robot_skipped_at_kickoff(make_sim):
    sim = make_sim([("B1", "blue", IDLE)], mode="basic")
    e = sim.entity("B1")
    sim.hide(e)
    sim.referee.kickoff("blue")
    assert not e.body.on_field


def test_yaml_round_trip(tmp_path):
    cfg = load_yaml(os.path.join(ROOT, "examples", "match.yaml"))
    cfg.referee.mode = "full"
    cfg.sim.noise = True
    cfg.robots[1].enabled = False
    path = tmp_path / "m.yaml"
    save_yaml(cfg, str(path))
    back = load_yaml(str(path))
    assert back.referee.mode == "full" and back.sim.noise
    assert [(r.name, r.team, os.path.abspath(r.code), r.enabled) for r in back.robots] == \
           [(r.name, r.team, os.path.abspath(r.code), r.enabled) for r in cfg.robots]


def test_setup_default_config_is_valid():
    from rcj_sim.setup_gui import default_config
    default_config().validate()
