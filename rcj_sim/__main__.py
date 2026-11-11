"""Command line entry point.

  python -m rcj_sim examples/match.yaml
  python -m rcj_sim --mode free --blue examples/striker.py examples/goalie.py --yellow examples/striker.py
  python -m rcj_sim examples/match.yaml --headless --duration 120
  python -m rcj_sim examples/match.yaml --screenshot shot.png --duration 3
"""
import argparse
import os
import sys
import time

from .config import MODES, from_lists, load_yaml


def build_parser():
    p = argparse.ArgumentParser(prog="python -m rcj_sim", description="RoboCupJunior Soccer simulator")
    p.add_argument("config", nargs="?", help="match YAML file")
    p.add_argument("--blue", nargs="*", default=None, metavar="CODE.py",
                   help="robot programs for the blue team (defends the blue goal in the 1st half)")
    p.add_argument("--yellow", nargs="*", default=None, metavar="CODE.py",
                   help="robot programs for the yellow team")
    p.add_argument("--mode", choices=MODES, help="referee mode (overrides the config file)")
    p.add_argument("--half", type=float, help="half duration in seconds")
    p.add_argument("--seed", type=int, help="random seed (noise)")
    p.add_argument("--noise", action="store_true", help="enable sensor noise and gyro drift")
    p.add_argument("--headless", action="store_true", help="no window, run as fast as possible")
    p.add_argument("--duration", type=float, help="stop after this many simulated seconds")
    p.add_argument("--screenshot", metavar="PNG", help="run --duration seconds, save a frame and exit")
    p.add_argument("--scale", type=float, default=None, help="pixels per cm (default: fit the screen, max 4)")
    p.add_argument("--quiet", action="store_true", help="do not echo robot prints to the console")
    return p


def make_config(args):
    if args.config:
        cfg = load_yaml(args.config)
        if args.mode:
            cfg.referee.mode = args.mode
    elif args.blue is not None or args.yellow is not None:
        cfg = from_lists(args.blue, args.yellow, args.mode or "basic")
    else:
        raise SystemExit("give a match YAML file or --blue/--yellow robot programs (see --help)")
    if args.half:
        cfg.referee.half_duration_s = args.half
    if args.seed is not None:
        cfg.sim.seed = args.seed
    if args.noise:
        cfg.sim.noise = True
    return cfg.validate()


def run_headless(sim, duration):
    t0 = time.time()
    sim.start()
    end = duration if duration else sim.cfg.referee.half_duration_s * sim.cfg.referee.halves + 1
    if sim.cfg.referee.mode == "free" and not duration:
        end = 600.0
    sim.run_for(end)
    ref = sim.referee
    print(f"\n=== {sim.time:.1f}s simulated in {time.time() - t0:.1f}s real ===")
    print(f"score  BLUE {ref.score['blue']} : {ref.score['yellow']} YELLOW")
    for t, team, half in sim.goal_log:
        print(f"  goal {team:6s} at {t:7.2f}s (half {half})")
    for e in sim.entities:
        line = f"  {e.cfg.name:12s} {e.status}"
        if e.ctrl.error:
            line += "\n" + e.ctrl.error
        print(line)
    sim.shutdown()


def main(argv=None):
    args = build_parser().parse_args(argv)
    cfg = make_config(args)
    sys.path.insert(0, os.getcwd())
    from .sim import Simulation
    sim = Simulation(cfg, echo_logs=not args.quiet)
    if args.headless:
        run_headless(sim, args.duration)
        return
    from .render import run_window, save_screenshot
    if args.screenshot:
        save_screenshot(sim, args.screenshot, args.duration or 2.0, scale=args.scale)
        return
    run_window(sim, scale=args.scale, duration=args.duration)


if __name__ == "__main__":
    main()
