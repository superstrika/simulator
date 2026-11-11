"""pygame top-down view and controls.

Keys:   Space pause/resume   N single step   +/- speed   R reset match
        K kick-off           F camera cones  D debug (trails, line sensors)   H help   Esc quit
Mouse:  drag the ball or a robot; scroll over a robot to rotate it;
        right-click a robot (or click its panel card) to remove / restore it (damaged).
"""
import math
import os
from collections import deque

from .constants import (
    BALL_RADIUS, CENTER_CIRCLE_DIAMETER, GOAL_BACK_X, GOAL_MOUTH_X, GOAL_WIDTH,
    HALF_L, HALF_W, LINE_WIDTH, PENALTY_CORNER_RADIUS, PENALTY_DEPTH,
    PENALTY_WIDTH, ROBOT_RADIUS, TCRT_COUNT, WALL_X, WALL_Y,
)
from .field import neutral_spots, wall_polygons
from .hardware import relative_angle, wrap180
from .physics import robot_polygons

# ------------------------------------------------------------------ colours
BG = (24, 26, 30)
PANEL_BG = (34, 37, 43)
CARD_BG = (44, 48, 56)
CARPET = (22, 110, 52)
WHITE = (245, 245, 245)
BLACK = (10, 10, 10)
WALL = (15, 15, 15)
GOAL_COL = {"blue_goal": (40, 110, 255), "yellow_goal": (250, 205, 0)}
TEAM_COL = {"blue": (70, 140, 255), "yellow": (250, 205, 0)}
BALL_COL = (255, 120, 0)
ROBOT_BODY = (58, 60, 66)
TEXT = (230, 232, 236)
DIM = (140, 145, 155)
RED = (240, 70, 70)
GREEN = (90, 220, 120)

SPEEDS = [0.25, 0.5, 1.0, 2.0, 4.0, 8.0]
TOP_H = 52
BOTTOM_H = 26
MARGIN = 14
PANEL_W = 350


def auto_scale(pg):
    try:
        info = pg.display.Info()
        sw, sh = info.current_w, info.current_h
    except Exception:
        return 4.0
    if sw <= 0 or sh <= 0:
        return 4.0
    s = min((sw - PANEL_W - 2 * MARGIN - 40) / (2 * WALL_X),
            (sh - TOP_H - BOTTOM_H - 2 * MARGIN - 90) / (2 * WALL_Y))
    return max(2.0, min(4.0, s))


class Renderer:
    def __init__(self, pg, sim, scale):
        self.pg = pg
        self.sim = sim
        self.S = scale
        self.field_w = int(2 * WALL_X * scale)
        self.field_h = int(2 * WALL_Y * scale)
        self.width = self.field_w + 2 * MARGIN + PANEL_W
        self.height = max(TOP_H + self.field_h + 2 * MARGIN + BOTTOM_H, 560)
        self.font = pg.font.Font(None, 20)
        self.font_small = pg.font.Font(None, 17)
        self.font_big = pg.font.Font(None, 40)
        self.font_banner = pg.font.Font(None, 56)
        self.show_fov = False
        self.debug = False
        self.show_help = False
        self.selected = None
        self.trail = deque(maxlen=240)
        self.card_rects = []
        self.static = self._draw_static()
        self._robot_polys = robot_polygons()

    # ------------------------------------------------------------ coordinates
    def w2s(self, x, y):
        return (MARGIN + (x + WALL_X) * self.S, TOP_H + MARGIN + (WALL_Y - y) * self.S)

    def s2w(self, sx, sy):
        return ((sx - MARGIN) / self.S - WALL_X, WALL_Y - (sy - TOP_H - MARGIN) / self.S)

    def _rect(self, x0, y0, x1, y1):
        """Screen rect for world box [x0,x1] x [y0,y1]."""
        ax, ay = self.w2s(x0, y1)
        bx, by = self.w2s(x1, y0)
        return self.pg.Rect(round(ax), round(ay), round(bx - ax), round(by - ay))

    # ------------------------------------------------------------ static field
    def _draw_static(self):
        pg, S = self.pg, self.S
        surf = pg.Surface((self.width, self.height))
        surf.fill(BG)
        # walls + carpet
        outer = self._rect(-WALL_X, -WALL_Y, WALL_X, WALL_Y)
        pg.draw.rect(surf, WALL, outer.inflate(12, 12))
        pg.draw.rect(surf, CARPET, outer)
        # boundary line: white field, green inside
        pg.draw.rect(surf, WHITE, self._rect(-HALF_L, -HALF_W, HALF_L, HALF_W))
        pg.draw.rect(surf, CARPET, self._rect(-HALF_L + LINE_WIDTH, -HALF_W + LINE_WIDTH,
                                              HALF_L - LINE_WIDTH, HALF_W - LINE_WIDTH))
        # penalty areas (rounded front corners)
        r_out = round(PENALTY_CORNER_RADIUS * S)
        r_in = round((PENALTY_CORNER_RADIUS - LINE_WIDTH) * S)
        for s in (-1, 1):
            x_front, x_back = s * (HALF_L - PENALTY_DEPTH), s * (HALF_L - LINE_WIDTH)
            hw = PENALTY_WIDTH / 2
            outer_r = self._rect(min(x_front, s * HALF_L), -hw, max(x_front, s * HALF_L), hw)
            xi = x_front + s * LINE_WIDTH
            inner_r = self._rect(min(xi, x_back), -hw + LINE_WIDTH, max(xi, x_back), hw - LINE_WIDTH)
            if s > 0:
                kw = dict(border_top_left_radius=r_out, border_bottom_left_radius=r_out)
                kwi = dict(border_top_left_radius=r_in, border_bottom_left_radius=r_in)
            else:
                kw = dict(border_top_right_radius=r_out, border_bottom_right_radius=r_out)
                kwi = dict(border_top_right_radius=r_in, border_bottom_right_radius=r_in)
            pg.draw.rect(surf, WHITE, outer_r, **kw)
            pg.draw.rect(surf, CARPET, inner_r, **kwi)
        # centre circle + neutral spots (thin black marker)
        c = self.w2s(0, 0)
        pg.draw.circle(surf, BLACK, (round(c[0]), round(c[1])), round(CENTER_CIRCLE_DIAMETER / 2 * S), 1)
        for (x, y) in neutral_spots():
            p = self.w2s(x, y)
            pg.draw.circle(surf, BLACK, (round(p[0]), round(p[1])), max(2, round(0.6 * S)))
        # goal structure
        for poly in wall_polygons()[4:]:
            pts = [self.w2s(x, y) for (x, y) in poly]
            pg.draw.polygon(surf, WALL, pts)
        for goal, s in (("blue_goal", -1), ("yellow_goal", 1)):
            r = self._rect(min(s * GOAL_MOUTH_X, s * GOAL_BACK_X), -GOAL_WIDTH / 2,
                           max(s * GOAL_MOUTH_X, s * GOAL_BACK_X), GOAL_WIDTH / 2)
            pg.draw.rect(surf, GOAL_COL[goal], r)
        # panel background
        pg.draw.rect(surf, PANEL_BG, pg.Rect(self.width - PANEL_W, 0, PANEL_W, self.height))
        return surf

    # ------------------------------------------------------------ drawing
    def draw(self, surf, speed, paused):
        pg, sim = self.pg, self.sim
        surf.blit(self.static, (0, 0))
        ball = sim.world.ball.position
        self.trail.append((ball.x, ball.y))

        overlay = pg.Surface((self.width, self.height), pg.SRCALPHA)
        overlay.set_clip(self._rect(-WALL_X, -WALL_Y, WALL_X, WALL_Y))
        if self.show_fov:
            for e in sim.entities:
                if e.body.on_field:
                    self._draw_fov(overlay, e)
        if self.debug and len(self.trail) > 2:
            pts = [self.w2s(x, y) for x, y in self.trail]
            pg.draw.lines(overlay, (255, 160, 60, 120), False, pts, 2)
        surf.blit(overlay, (0, 0))

        for e in sim.entities:
            if e.body.on_field:
                self._draw_robot(surf, e)
        bp = self.w2s(ball.x, ball.y)
        pg.draw.circle(surf, BALL_COL, (round(bp[0]), round(bp[1])), max(3, round(BALL_RADIUS * self.S)))
        pg.draw.circle(surf, (120, 50, 0), (round(bp[0]), round(bp[1])), max(3, round(BALL_RADIUS * self.S)), 1)

        self._draw_top(surf, speed, paused)
        self._draw_panel(surf)
        self._draw_bottom(surf)
        self._draw_banner(surf)
        if self.show_help:
            self._draw_help(surf)

    def _draw_fov(self, overlay, e):
        cfg = self.sim.cfg.sim
        b = e.body
        half = math.radians(cfg.front_fov_deg / 2)
        rng = cfg.front_range_cm
        pts = [self.w2s(b.x, b.y)]
        for k in range(13):
            a = b.angle - half + 2 * half * k / 12
            pts.append(self.w2s(b.x + rng * math.cos(a), b.y + rng * math.sin(a)))
        col = TEAM_COL[e.cfg.team]
        self.pg.draw.polygon(overlay, (*col, 28), pts)
        self.pg.draw.lines(overlay, (*col, 90), True, pts, 1)

    def _draw_robot(self, surf, e):
        pg, S = self.pg, self.S
        b = e.body
        c, s = math.cos(b.angle), math.sin(b.angle)

        def tf(lx, ly):
            return self.w2s(b.x + lx * c - ly * s, b.y + lx * s + ly * c)

        team_col = TEAM_COL[e.cfg.team]
        center = self.w2s(b.x, b.y)
        ci = (round(center[0]), round(center[1]))
        for poly in self._robot_polys:
            pg.draw.polygon(surf, ROBOT_BODY, [tf(x, y) for x, y in poly])
        # team-coloured bumper ring (drawn as an arc that leaves the notch open)
        pts = []
        for k in range(41):
            a = math.radians(40 + 280 * k / 40)
            pts.append(tf(ROBOT_RADIUS * math.cos(a), ROBOT_RADIUS * math.sin(a)))
        pg.draw.lines(surf, team_col, False, pts, max(2, round(0.7 * S)))
        if e is self.selected:
            pg.draw.circle(surf, WHITE, ci, round((ROBOT_RADIUS + 2) * S), 1)
        # dribbler bar
        if b.dribbler_on:
            col = GREEN if b.holding else (170, 170, 170)
            pg.draw.line(surf, col, tf(7.3, -2.4), tf(7.3, 2.4), max(2, round(0.6 * S)))
        # heading
        pg.draw.line(surf, WHITE, center, tf(6.5, 0), 2)
        # white top marker with the robot number (rule 3.5)
        pg.draw.circle(surf, WHITE, ci, round(3.0 * S))
        label = self.font_small.render(str(e.index + 1), True, BLACK)
        surf.blit(label, label.get_rect(center=ci))
        # line sensors
        if self.debug or any(e.hw.line_state):
            for i, (x, y) in enumerate(e.hw.tcrt_positions()):
                p = self.w2s(x, y)
                on = e.hw.line_state[i]
                if on or self.debug:
                    pg.draw.circle(surf, RED if on else (90, 90, 90), (round(p[0]), round(p[1])), 2 if not on else 3)
        # error marker
        if e.ctrl.state in ("crashed", "hung"):
            t = self.font.render("!", True, RED)
            surf.blit(t, (ci[0] + ROBOT_RADIUS * S * 0.7, ci[1] - ROBOT_RADIUS * S))

    def _txt(self, surf, text, pos, col=TEXT, font=None, anchor="topleft"):
        img = (font or self.font).render(text, True, col)
        r = img.get_rect(**{anchor: pos})
        surf.blit(img, r)
        return r

    def _draw_top(self, surf, speed, paused):
        sim, ref = self.sim, self.sim.referee
        fw = self.field_w + 2 * MARGIN
        mid = fw // 2
        self._txt(surf, "BLUE", (mid - 70, TOP_H // 2), TEAM_COL["blue"], self.font_big, "midright")
        self._txt(surf, f"{ref.score['blue']} : {ref.score['yellow']}", (mid, TOP_H // 2), TEXT,
                  self.font_big, "center")
        self._txt(surf, "YELLOW", (mid + 70, TOP_H // 2), TEAM_COL["yellow"], self.font_big, "midleft")
        if ref.timed:
            tl = ref.time_left()
            clock = f"{['1st', '2nd', '3rd', '4th'][min(ref.half, 4) - 1]} half  {int(tl // 60):02d}:{int(tl % 60):02d}"
        else:
            clock = "free play - first goal wins"
        self._txt(surf, clock, (MARGIN, 10), TEXT)
        self._txt(surf, f"mode: {ref.mode}   t = {sim.time:7.2f}s", (MARGIN, 30), DIM, self.font_small)
        state = "PAUSED" if paused else f"x{speed:g}"
        self._txt(surf, state, (fw - MARGIN, 10), RED if paused else GREEN, anchor="topright")
        sides = "sides swapped" if sim.sides_swapped else ""
        self._txt(surf, sides, (fw - MARGIN, 30), DIM, self.font_small, "topright")

    def _draw_bottom(self, surf):
        y = self.height - BOTTOM_H + 6
        self._txt(surf, "Space pause  N step  +/- speed  R reset  K kick-off  F cameras  D debug  H help",
                  (MARGIN, y), DIM, self.font_small)

    def _draw_banner(self, surf):
        ref = self.sim.referee
        if ref.banner and self.sim.time <= ref.banner[1]:
            img = self.font_banner.render(ref.banner[0], True, WHITE)
            cx = MARGIN + self.field_w // 2
            cy = TOP_H + MARGIN + self.field_h // 2
            r = img.get_rect(center=(cx, cy))
            bg = self.pg.Surface((r.width + 30, r.height + 18), self.pg.SRCALPHA)
            bg.fill((0, 0, 0, 170))
            surf.blit(bg, (r.x - 15, r.y - 9))
            surf.blit(img, r)

    def _draw_help(self, surf):
        lines = [l.strip() for l in __doc__.strip().splitlines()[2:] if l.strip()]
        w, h = 640, 30 + 22 * len(lines)
        x = MARGIN + (self.field_w - w) // 2
        y = TOP_H + 40
        bg = self.pg.Surface((w, h), self.pg.SRCALPHA)
        bg.fill((0, 0, 0, 210))
        surf.blit(bg, (x, y))
        for i, l in enumerate(lines):
            self._txt(surf, l, (x + 16, y + 14 + 22 * i), TEXT)

    def _draw_panel(self, surf):
        pg, sim = self.pg, self.sim
        x0 = self.width - PANEL_W + 10
        y = 10
        self.card_rects = []
        for e in sim.entities:
            h = 128
            card = pg.Rect(x0, y, PANEL_W - 20, h)
            self.card_rects.append((card, e))
            pg.draw.rect(surf, CARD_BG, card, border_radius=6)
            if e is self.selected:
                pg.draw.rect(surf, WHITE, card, 1, border_radius=6)
            pg.draw.rect(surf, TEAM_COL[e.cfg.team], pg.Rect(x0, y, 5, h),
                         border_top_left_radius=6, border_bottom_left_radius=6)
            status = e.status
            if e.penalty_until is not None:
                status += f" {max(0, e.penalty_until - sim.time):.0f}s"
            scol = {"running": GREEN, "crashed": RED, "hung": RED}.get(e.status, DIM)
            self._txt(surf, f"{e.index + 1}  {e.cfg.name}", (x0 + 12, y + 6), TEXT)
            self._txt(surf, status, (card.right - 8, y + 6), scol, anchor="topright")
            self._txt(surf, os.path.basename(e.cfg.code), (x0 + 12, y + 24), DIM, self.font_small)

            b = e.body
            ball = sim.world.ball.position
            hw = e.hw
            if b.on_field:
                gyro = wrap180(math.degrees(hw.gyro_zero - b.angle))
                ba = relative_angle(b.x, b.y, b.angle, ball.x, ball.y)
                dist = math.hypot(ball.x - b.x, ball.y - b.y)
                in_view = abs(ba) <= sim.cfg.sim.front_fov_deg / 2 and dist <= sim.cfg.sim.front_range_cm
                l1 = f"gyro {gyro:6.1f}   ball {ba:6.1f}"
                l2 = f"front cam: {'%.0f cm' % dist if in_view else '-'}   " \
                     f"{'HOLD' if b.holding else ('drib' if b.dribbler_on else '')}"
            else:
                l1, l2 = "off the field", ""
            self._txt(surf, l1, (x0 + 12, y + 42), TEXT, self.font_small)
            self._txt(surf, l2, (x0 + 12, y + 58), TEXT, self.font_small)
            # wheels
            wl = " ".join(f"{w:5.0f}" for w in hw.wheels)
            self._txt(surf, f"wheels {wl}", (x0 + 12, y + 74), DIM, self.font_small)
            # line sensors
            for i in range(TCRT_COUNT):
                col = RED if hw.line_state[i] else (80, 84, 92)
                pg.draw.rect(surf, col, pg.Rect(x0 + 12 + i * 12, y + 92, 9, 7))
            # last log / error
            if e.ctrl.error:
                msg = e.ctrl.error.strip().splitlines()[-1]
                col = RED
            elif e.ctrl.logs:
                msg = e.ctrl.logs[-1][1]
                col = DIM
            else:
                msg, col = "", DIM
            self._txt(surf, msg[:52], (x0 + 12, y + 106), col, self.font_small)
            y += h + 8
        # events
        y = max(y, self.height - 150)
        self._txt(surf, "Referee", (x0, y), TEXT)
        y += 18
        for t, text in list(sim.events)[-6:]:
            self._txt(surf, f"{t:6.1f}s {text}"[:54], (x0, y), DIM, self.font_small)
            y += 16

    # ------------------------------------------------------------ picking
    def robot_at(self, wx, wy):
        for e in self.sim.entities:
            if e.body.on_field and math.hypot(e.body.x - wx, e.body.y - wy) <= ROBOT_RADIUS:
                return e
        return None

    def card_at(self, sx, sy):
        for rect, e in self.card_rects:
            if rect.collidepoint(sx, sy):
                return e
        return None


def _make_renderer(pg, sim, scale):
    pg.font.init()
    return Renderer(pg, sim, scale or 4.0)


def save_screenshot(sim, path, duration, scale=None):
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame as pg
    pg.init()
    r = _make_renderer(pg, sim, scale or 4.0)
    r.show_fov = True
    sim.start()
    sim.run_for(duration)
    surf = pg.Surface((r.width, r.height))
    r.draw(surf, 1.0, False)
    pg.image.save(surf, path)
    sim.shutdown()
    pg.quit()
    print(f"saved {path}")


def run_window(sim, scale=None, duration=None):
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame as pg
    pg.init()
    if scale is None:
        scale = auto_scale(pg)
    r = _make_renderer(pg, sim, scale)
    screen = pg.display.set_mode((r.width, r.height))
    pg.display.set_caption("RoboCupJunior Soccer Simulator")
    clock = pg.time.Clock()

    speed_i = SPEEDS.index(1.0)
    paused = False
    acc = 0.0
    dragging = None   # "ball" or an entity
    sim.start()
    running = True
    while running:
        for ev in pg.event.get():
            if ev.type == pg.QUIT:
                running = False
            elif ev.type == pg.KEYDOWN:
                k = ev.key
                if k in (pg.K_ESCAPE, pg.K_q):
                    running = False
                elif k == pg.K_SPACE:
                    paused = not paused
                elif k == pg.K_n:
                    paused = True
                    for _ in range(max(1, sim.cfg.sim.physics_hz // 20)):
                        sim.step()
                elif k in (pg.K_PLUS, pg.K_EQUALS, pg.K_KP_PLUS):
                    speed_i = min(len(SPEEDS) - 1, speed_i + 1)
                elif k in (pg.K_MINUS, pg.K_KP_MINUS):
                    speed_i = max(0, speed_i - 1)
                elif k == pg.K_r:
                    sim.reset()
                    r.trail.clear()
                elif k == pg.K_k:
                    if not sim.finished:
                        sim.referee.kickoff(sim.referee.kickoff_team)
                elif k == pg.K_f:
                    r.show_fov = not r.show_fov
                elif k == pg.K_d:
                    r.debug = not r.debug
                elif k == pg.K_h:
                    r.show_help = not r.show_help
            elif ev.type == pg.MOUSEBUTTONDOWN:
                wx, wy = r.s2w(*ev.pos)
                if ev.button == 1:
                    card = r.card_at(*ev.pos)
                    if card is not None:
                        r.selected = card
                        continue
                    bp = sim.world.ball.position
                    if math.hypot(bp.x - wx, bp.y - wy) <= BALL_RADIUS + 3:
                        dragging = "ball"
                    else:
                        e = r.robot_at(wx, wy)
                        if e is not None:
                            dragging = e
                            r.selected = e
                elif ev.button == 3:
                    e = r.robot_at(wx, wy) or r.card_at(*ev.pos)
                    if e is not None:
                        sim.toggle_damaged(e)
            elif ev.type == pg.MOUSEBUTTONUP and ev.button == 1:
                dragging = None
            elif ev.type == pg.MOUSEWHEEL:
                wx, wy = r.s2w(*pg.mouse.get_pos())
                e = r.robot_at(wx, wy)
                if e is not None:
                    sim.move_robot(e, e.body.x, e.body.y, e.body.angle - math.radians(10 * ev.y))

        if dragging is not None:
            wx, wy = r.s2w(*pg.mouse.get_pos())
            if dragging == "ball":
                sim.move_ball(max(-WALL_X + 2.2, min(WALL_X - 2.2, wx)),
                              max(-WALL_Y + 2.2, min(WALL_Y - 2.2, wy)))
            else:
                sim.move_robot(dragging, max(-WALL_X + 9.2, min(WALL_X - 9.2, wx)),
                               max(-WALL_Y + 9.2, min(WALL_Y - 9.2, wy)))

        if not paused and not sim.finished:
            acc += SPEEDS[speed_i] * sim.cfg.sim.physics_hz / 60.0
            n = int(acc)
            acc -= n
            n = min(n, sim.cfg.sim.physics_hz)   # never more than 1 s of sim per frame
            for _ in range(n):
                sim.step()
            if duration and sim.time >= duration:
                running = False

        r.draw(screen, SPEEDS[speed_i], paused)
        pg.display.flip()
        clock.tick(60)

    sim.shutdown()
    pg.quit()
