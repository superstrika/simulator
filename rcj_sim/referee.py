"""Automatic referee with three modes.

free  : no clock, no penalties. The match ends at the first goal.
basic : score, 2 halves with a clock, kick-off after every goal, sides swap at half time,
        lack of progress (a stuck ball is moved to a neutral spot). No penalties.
full  : basic + out of bounds (1 min penalty), multiple defense, max 2 robots per team.
"""
import math

from .constants import BALL_RADIUS, BLUE_GOAL, ROBOT_RADIUS, TEAMS, YELLOW_GOAL
from .field import (
    ball_in_goal, goal_side, neutral_spots, robot_fully_in_penalty,
    robot_partly_in_penalty, robot_touches_wall,
)

KICKER_SLOTS = [(15, 0), (75, 0), (50, 35), (50, -35), (60, 60), (60, -60)]
RECEIVER_SLOTS = [(45, 0), (75, 0), (50, 35), (50, -35), (60, 60), (60, -60)]


def other(team):
    return "yellow" if team == "blue" else "blue"


class Referee:
    def __init__(self, sim):
        self.sim = sim
        self.cfg = sim.cfg.referee
        self.mode = self.cfg.mode
        self.score = {t: 0 for t in TEAMS}
        self.half = 1
        self.clock = 0.0
        self.state = "playing"      # playing | hold | finished
        self.hold_until = 0.0
        self.kickoff_team = self.cfg.first_kickoff
        self.first_kickoff = self.cfg.first_kickoff
        self.banner = None          # (text, until_time)
        self._lop_anchor = (0.0, 0.0)
        self._lop_time = 0.0

    # ------------------------------------------------------------ helpers
    @property
    def full(self):
        return self.mode == "full"

    @property
    def timed(self):
        return self.mode in ("basic", "full")

    def time_left(self):
        return max(0.0, self.cfg.half_duration_s - self.clock)

    def show(self, text, secs=2.0):
        self.banner = (text, self.sim.time + secs)
        self.sim.event(text)

    def heading_towards_opponent(self, team):
        # facing +x when defending the goal at -x
        return 0.0 if goal_side(self.sim.defended_goal(team)) < 0 else math.pi

    def _free_spots(self, exclude=None):
        """Neutral spots with no robot on them."""
        sim = self.sim
        out = []
        for (sx, sy) in neutral_spots():
            busy = False
            for e in sim.entities:
                if e is exclude or not e.body.on_field:
                    continue
                if math.hypot(e.body.x - sx, e.body.y - sy) < ROBOT_RADIUS * 2 + 1:
                    busy = True
                    break
            if not busy:
                out.append((sx, sy))
        return out or neutral_spots()

    def _ball_pos(self):
        p = self.sim.world.ball.position
        return p.x, p.y

    def place_on_neutral_spot(self, e, facing_own_goal=True):
        """Rule 2.8/2.9: unoccupied neutral spot furthest from the ball, facing its own goal."""
        bx, by = self._ball_pos()
        spots = self._free_spots(exclude=e)
        spots.sort(key=lambda p: math.hypot(p[0] - bx, p[1] - by), reverse=True)
        sx, sy = spots[0]
        ang = self.heading_towards_opponent(e.cfg.team)
        if facing_own_goal:
            ang += math.pi
        elif e.body.on_field:
            ang = e.body.angle
        e.body.place(sx, sy, ang)

    # ------------------------------------------------------------ match flow
    def start_match(self):
        self.score = {t: 0 for t in TEAMS}
        self.half = 1
        self.clock = 0.0
        self.state = "playing"
        self.kickoff(self.first_kickoff, hold=0.0)

    def kickoff(self, team, hold=None):
        """Place the ball in the centre and the robots in kick-off positions."""
        sim = self.sim
        self.kickoff_team = team
        sim.world.place_ball(0.0, 0.0)
        for e in sim.entities:
            if e.penalty_until is not None:   # a kick-off ends out-of-bounds penalties
                e.penalty_until = None
        for t in TEAMS:
            side = goal_side(sim.defended_goal(t))
            slots = KICKER_SLOTS if t == team else RECEIVER_SLOTS
            players = [e for e in sim.entities if e.cfg.team == t and e.available]
            for i, e in enumerate(players):
                if e.cfg.start:
                    st = e.cfg.start
                    x, y = float(st.get("x", 0)), float(st.get("y", 0))
                    if sim.sides_swapped:
                        x, y = -x, -y
                    ang = self.heading_towards_opponent(t) - math.radians(float(st.get("heading", 0)))
                else:
                    sx, sy = slots[i % len(slots)]
                    x, y = side * sx, sy * (-side)
                    ang = self.heading_towards_opponent(t)
                e.body.place(x, y, ang)
                e.body.release_ball()
        self._lop_anchor = (0.0, 0.0)
        self._lop_time = sim.time
        hold = self.cfg.goal_hold_s if hold is None else hold
        if hold > 0:
            self.state = "hold"
            self.hold_until = sim.time + hold
            sim.set_frozen(True)
        else:
            self.state = "playing"
            sim.set_frozen(False)

    def finish(self, text):
        self.state = "finished"
        self.sim.set_frozen(True)
        self.show(text, secs=1e9)
        self.sim.finished = True

    # ------------------------------------------------------------ per step
    def update(self):
        sim = self.sim
        if self.state == "finished":
            return
        self._return_penalized()
        if self.state == "hold":
            if sim.time < self.hold_until:
                return
            self.state = "playing"
            sim.set_frozen(False)

        bx, by = self._ball_pos()
        goal = ball_in_goal(bx, by)
        if goal is not None:
            self._goal(goal)
            return

        if self.full:
            self._full_rules(bx, by)
        if self.timed and self.cfg.lack_of_progress:
            self._lack_of_progress(bx, by)

        if self.timed:
            self.clock += sim.dt
            if self.clock >= self.cfg.half_duration_s:
                self._end_of_half()

    def _goal(self, goal):
        sim = self.sim
        scorer = "yellow" if sim.defended_goal("blue") == goal else "blue"
        self.score[scorer] += 1
        sim.goal_log.append((sim.time, scorer, self.half))
        if self.mode == "free":
            self.finish(f"GOAL for {scorer.upper()}!  (press R to reset)")
            return
        self.show(f"GOAL for {scorer.upper()}!  {self.score['blue']} : {self.score['yellow']}")
        self.kickoff(other(scorer))      # team scored against kicks off

    def _end_of_half(self):
        sim = self.sim
        if self.half >= self.cfg.halves:
            b, y = self.score["blue"], self.score["yellow"]
            res = "DRAW" if b == y else f"{'BLUE' if b > y else 'YELLOW'} WINS"
            self.finish(f"FULL TIME  {b} : {y}  {res}")
            return
        self.half += 1
        self.clock = 0.0
        if self.cfg.swap_sides:
            sim.sides_swapped = not sim.sides_swapped
        self.kickoff(other(self.first_kickoff))
        sim.restart_programs()            # robots are rebooted at half time
        self.show(f"HALF TIME - half {self.half} starts")

    # ------------------------------------------------------------ full-mode rules
    def _return_penalized(self):
        sim = self.sim
        for e in sim.entities:
            if e.penalty_until is not None and sim.time >= e.penalty_until:
                e.penalty_until = None
                if not e.damaged:
                    self.place_on_neutral_spot(e)
                    e.body.frozen = self.state != "playing"
                    sim.event(f"{e.cfg.name} returns to the field")

    def penalize(self, e, reason):
        e.body.remove()
        e.hw.line_state = [False] * len(e.hw.line_state)
        e.penalty_until = self.sim.time + self.cfg.out_of_bounds_penalty_s
        self.show(f"{e.cfg.name}: {reason} - out for {self.cfg.out_of_bounds_penalty_s:.0f}s")

    def _full_rules(self, bx, by):
        sim = self.sim
        cfg = self.cfg
        if cfg.out_of_bounds:
            for e in sim.entities:
                b = e.body
                if not b.on_field:
                    continue
                if robot_touches_wall(b.x, b.y):
                    self.penalize(e, "out of bounds (touched wall)")
                elif robot_fully_in_penalty(b.x, b.y):
                    self.penalize(e, "out of bounds (inside penalty area)")

        if cfg.multiple_defense:
            for t in TEAMS:
                goal = sim.defended_goal(t)
                inside = [e for e in sim.entities if e.cfg.team == t and e.body.on_field
                          and robot_partly_in_penalty(e.body.x, e.body.y, goal)]
                if len(inside) >= 2:
                    inside.sort(key=lambda e: math.hypot(e.body.x - bx, e.body.y - by))
                    far = inside[-1]
                    self.place_on_neutral_spot(far, facing_own_goal=False)
                    self.show(f"{far.cfg.name}: multiple defense")

    def _lack_of_progress(self, bx, by):
        sim, cfg = self.sim, self.cfg
        ax, ay = self._lop_anchor
        if math.hypot(bx - ax, by - ay) > cfg.lack_of_progress_cm:
            self._lop_anchor = (bx, by)
            self._lop_time = sim.time
        elif sim.time - self._lop_time >= cfg.lack_of_progress_s:
            spots = [p for p in self._free_spots()
                     if math.hypot(p[0] - bx, p[1] - by) > 5]
            spots = spots or neutral_spots()
            spots.sort(key=lambda p: math.hypot(p[0] - bx, p[1] - by))
            sx, sy = spots[0]
            sim.world.place_ball(sx, sy)
            self._lop_anchor = (sx, sy)
            self._lop_time = sim.time
            self.show("Lack of progress - ball moved to neutral spot")


__all__ = ["Referee", "BLUE_GOAL", "YELLOW_GOAL", "BALL_RADIUS"]
