"""rcj_sim robot program that runs Superstrika's real robot/hunter.py (Hunt.hunt()).

The Superstrika repository is found at ../Superstrika-new next to this simulator,
or wherever the SUPERSTRIKA_PATH environment variable points.
"""
from superstrika_bridge import run_hunter


def main(sim_robot):
    run_hunter(sim_robot)
