"""Match setup screen, shown when the simulator starts without a configuration.

Pick the referee mode, match and simulation options and the robots (name, team,
program file, on/off), then press Start.  Esc in the match returns here.
The last setup is remembered in ~/RCJ-Sim/last_match.yaml.
"""
import copy
import os
import time

from .config import MODES, MatchConfig, RobotConfig, load_yaml, save_yaml
from .resources import LAST_MATCH, USER_DIR, examples_dir

W, H = 1180, 760

BG = (24, 26, 30)
PANEL = (34, 37, 43)
FIELD_BG = (48, 52, 60)
FIELD_FOCUS = (60, 66, 78)
TEXT = (230, 232, 236)
DIM = (140, 145, 155)
ACCENT = (70, 140, 255)
GREEN = (60, 160, 95)
RED = (220, 75, 75)
TEAM_COL = {"blue": (70, 140, 255), "yellow": (240, 195, 0)}

MODE_LABELS = [("free", "Free play"), ("basic", "Basic"), ("full", "Full rules")]
MODE_HELP = {
    "free": "No clock, no penalties - the match ends at the first goal.",
    "basic": "Score, halves, kick-offs after goals, sides swap, lack of progress.",
    "full": "Basic + out of bounds (1 min), multiple defense, max 2 robots per team.",
}


# ====================================================================== tiny immediate-mode UI
class UI:
    def __init__(self, pg):
        self.pg = pg
        self.font = pg.font.Font(None, 22)
        self.small = pg.font.Font(None, 18)
        self.big = pg.font.Font(None, 34)
        self.hits = []
        self.wheel = []
        self.focus = None
        self.buf = ""
        self.commit_fn = None
        self.surf = None

    def begin(self, surf):
        self.surf = surf
        self.hits = []
        self.wheel = []

    # ------------------------------------------------------------ drawing helpers
    def fit(self, text, width, font=None):
        font = font or self.font
        if font.size(text)[0] <= width:
            return text
        while text and font.size(text + "...")[0] > width:
            text = text[:-1]
        return text + "..."

    def label(self, text, pos, col=TEXT, font=None, anchor="topleft"):
        img = (font or self.font).render(str(text), True, col)
        r = img.get_rect(**{anchor: pos})
        self.surf.blit(img, r)
        return r

    def button(self, rect, text, fn, kind="normal", enabled=True, font=None):
        pg = self.pg
        rect = pg.Rect(rect)
        hover = rect.collidepoint(pg.mouse.get_pos()) and enabled
        col = {"normal": (64, 69, 80), "primary": GREEN, "danger": (90, 50, 55),
               "accent": ACCENT}[kind]
        if hover:
            col = tuple(min(255, c + 22) for c in col)
        if not enabled:
            col = (45, 48, 55)
        pg.draw.rect(self.surf, col, rect, border_radius=5)
        f = font or self.font
        self.label(self.fit(text, rect.width - 8, f), rect.center, TEXT if enabled else DIM, f, "center")
        if enabled:
            self.hits.append((rect, fn))
        return rect

    def checkbox(self, pos, text, value, fn):
        pg = self.pg
        box = pg.Rect(pos[0], pos[1], 20, 20)
        pg.draw.rect(self.surf, FIELD_BG, box, border_radius=4)
        if value:
            pg.draw.rect(self.surf, ACCENT, box.inflate(-6, -6), border_radius=3)
        r = self.label(text, (box.right + 8, box.centery), TEXT, anchor="midleft") if text else box
        self.hits.append((box.union(r), lambda: fn(not value)))

    def segmented(self, rect, options, current, fn, colors=None, font=None):
        pg = self.pg
        rect = pg.Rect(rect)
        n = len(options)
        w = rect.width / n
        for i, (val, text) in enumerate(options):
            r = pg.Rect(round(rect.x + i * w), rect.y, round(w) - 2, rect.height)
            sel = val == current
            col = (colors or {}).get(val, ACCENT) if sel else FIELD_BG
            pg.draw.rect(self.surf, col, r, border_radius=5)
            txt_col = (20, 20, 20) if sel and colors and val == "yellow" else (TEXT if sel else DIM)
            self.label(text, r.center, txt_col, font or self.font, "center")
            self.hits.append((r, (lambda v=val: fn(v))))

    def textfield(self, rect, key, value, fn, font=None):
        pg = self.pg
        rect = pg.Rect(rect)
        focused = self.focus == key
        pg.draw.rect(self.surf, FIELD_FOCUS if focused else FIELD_BG, rect, border_radius=4)
        if focused:
            pg.draw.rect(self.surf, ACCENT, rect, 1, border_radius=4)
        text = self.buf if focused else str(value)
        if focused and int(time.time() * 2) % 2 == 0:
            text += "|"
        self.label(self.fit(text, rect.width - 10, font), (rect.x + 6, rect.centery), TEXT, font, "midleft")

        def focus():
            self.commit()
            self.focus, self.buf, self.commit_fn = key, str(value), fn
        self.hits.append((rect, focus))

    def number(self, pos, key, value, fn, step=1, lo=None, hi=None, width=90, integer=True):
        x, y = pos

        def clamp(v):
            if lo is not None:
                v = max(lo, v)
            if hi is not None:
                v = min(hi, v)
            return int(round(v)) if integer else v

        def commit(text):
            try:
                fn(clamp(float(text)))
            except ValueError:
                pass
        self.button((x, y, 26, 26), "-", lambda: fn(clamp(value - step)))
        shown = int(value) if integer else round(value, 2)
        self.textfield((x + 30, y, width, 26), key, shown, commit)
        self.button((x + 34 + width, y, 26, 26), "+", lambda: fn(clamp(value + step)))

    # ------------------------------------------------------------ events
    def commit(self):
        if self.focus is not None and self.commit_fn is not None:
            fn, buf = self.commit_fn, self.buf
            self.focus, self.commit_fn = None, None
            fn(buf)
        self.focus = None

    def handle(self, ev):
        """Returns True if the event was used."""
        pg = self.pg
        if ev.type == pg.MOUSEBUTTONDOWN and ev.button == 1:
            for rect, fn in reversed(self.hits):
                if rect.collidepoint(ev.pos):
                    fn()
                    return True
            self.commit()
            return False
        if ev.type == pg.MOUSEWHEEL:
            mp = pg.mouse.get_pos()
            for rect, fn in self.wheel:
                if rect.collidepoint(mp):
                    fn(ev.y)
                    return True
        if self.focus is not None:
            if ev.type == pg.TEXTINPUT:
                self.buf += ev.text
                return True
            if ev.type == pg.KEYDOWN:
                if ev.key == pg.K_BACKSPACE:
                    self.buf = self.buf[:-1]
                elif ev.key in (pg.K_RETURN, pg.K_KP_ENTER, pg.K_TAB):
                    self.commit()
                elif ev.key == pg.K_ESCAPE:
                    self.focus = self.commit_fn = None
                return True
        return False


# ====================================================================== file picker
class FilePicker:
    def __init__(self, ui, title, exts, start_dir, on_done, save_name=None):
        self.ui = ui
        self.title = title
        self.exts = tuple(exts)
        self.cwd = start_dir if os.path.isdir(start_dir or "") else os.path.expanduser("~")
        self.on_done = on_done
        self.save_name = save_name          # None = open mode
        self.selected = None
        self.scroll = 0
        self.error = ""
        self._last_click = (None, 0.0)
        self._list()

    def _list(self):
        try:
            items = [e for e in os.scandir(self.cwd) if not e.name.startswith(".")]
        except OSError as ex:
            self.error = str(ex)
            items = []
        dirs = sorted((e for e in items if e.is_dir()), key=lambda e: e.name.lower())
        files = sorted((e for e in items if e.is_file() and e.name.lower().endswith(self.exts)),
                       key=lambda e: e.name.lower())
        self.entries = [(e.name, True) for e in dirs] + [(e.name, False) for e in files]
        self.scroll = 0
        self.selected = None

    def go(self, path):
        if os.path.isdir(path):
            self.cwd = os.path.abspath(path)
            self.error = ""
            self._list()

    def finish(self, path):
        self.ui.picker = None
        self.on_done(path)

    def accept(self):
        if self.save_name is not None:
            name = self.save_name.strip()
            if not name:
                self.error = "Type a file name"
                return
            if not name.lower().endswith((".yaml", ".yml")):
                name += ".yaml"
            self.finish(os.path.join(self.cwd, name))
        elif self.selected:
            self.finish(os.path.join(self.cwd, self.selected))

    def click(self, name, is_dir):
        now = time.time()
        double = self._last_click[0] == name and now - self._last_click[1] < 0.4
        self._last_click = (name, now)
        if is_dir:
            self.go(os.path.join(self.cwd, name))
        else:
            self.selected = name
            if self.save_name is not None:
                self.save_name = name
            if double:
                self.accept()

    def draw(self):
        ui, pg = self.ui, self.ui.pg
        s = ui.surf
        shade = pg.Surface((W, H), pg.SRCALPHA)
        shade.fill((0, 0, 0, 170))
        s.blit(shade, (0, 0))
        box = pg.Rect(0, 0, 780, 580)
        box.center = (W // 2, H // 2)
        pg.draw.rect(s, PANEL, box, border_radius=8)
        ui.hits.append((box, lambda: None))        # swallow clicks on the dialog background
        ui.label(self.title, (box.x + 20, box.y + 16), TEXT, ui.big)
        ui.label(ui.fit(self.cwd, 740, ui.small), (box.x + 20, box.y + 54), DIM, ui.small)
        bx = box.x + 20
        for text, path in (("Up", os.path.dirname(self.cwd)), ("Examples", examples_dir()),
                           ("Home", os.path.expanduser("~")), ("Current folder", os.getcwd())):
            w = ui.font.size(text)[0] + 24
            ui.button((bx, box.y + 76, w, 28), text, (lambda p=path: self.go(p)))
            bx += w + 8
        lst = pg.Rect(box.x + 20, box.y + 116, box.width - 40, 360)
        pg.draw.rect(s, (28, 30, 35), lst, border_radius=5)
        row_h = 26
        visible = lst.height // row_h
        self.scroll = max(0, min(self.scroll, max(0, len(self.entries) - visible)))
        ui.wheel.append((lst, lambda dy: setattr(self, "scroll", self.scroll - dy * 3)))
        for i, (name, is_dir) in enumerate(self.entries[self.scroll:self.scroll + visible]):
            r = pg.Rect(lst.x + 4, lst.y + 4 + i * row_h, lst.width - 8, row_h - 2)
            if name == self.selected:
                pg.draw.rect(s, (50, 80, 130), r, border_radius=4)
            elif r.collidepoint(pg.mouse.get_pos()):
                pg.draw.rect(s, (40, 44, 52), r, border_radius=4)
            ui.label(("[ ] " if is_dir else "    ") + name + ("/" if is_dir else ""),
                     (r.x + 8, r.centery), ACCENT if is_dir else TEXT, ui.font, "midleft")
            ui.hits.append((r, (lambda n=name, d=is_dir: self.click(n, d))))
        if not self.entries:
            ui.label(f"No folders or {'/'.join(self.exts)} files here", lst.center, DIM, anchor="center")
        y = lst.bottom + 14
        if self.save_name is not None:
            ui.label("File name:", (box.x + 20, y + 13), TEXT, anchor="midleft")
            ui.textfield((box.x + 110, y, 300, 26), "picker-name", self.save_name,
                         lambda t: setattr(self, "save_name", t))
        if self.error:
            ui.label(self.error, (box.x + 20, box.bottom - 30), RED, ui.small, "midleft")
        ui.button((box.right - 250, box.bottom - 48, 110, 34), "Cancel", lambda: self.finish(None))
        ok = "Save" if self.save_name is not None else "Open"
        ui.button((box.right - 130, box.bottom - 48, 110, 34), ok, self.accept, "primary",
                  enabled=self.save_name is not None or bool(self.selected))


# ====================================================================== setup screen
def default_config():
    ex = examples_dir()
    cfg = MatchConfig()
    cfg.referee.mode = "basic"
    for name, team, prog in (("B-Striker", "blue", "striker.py"), ("B-Goalie", "blue", "goalie.py"),
                             ("Y-Striker", "yellow", "striker.py"), ("Y-Goalie", "yellow", "goalie.py")):
        cfg.robots.append(RobotConfig(name=name, team=team, code=os.path.join(ex, prog)))
    return cfg


def initial_config():
    if os.path.isfile(LAST_MATCH):
        try:
            return load_yaml(LAST_MATCH, validate=False)
        except Exception:
            pass
    return default_config()


class SetupScreen:
    def __init__(self, pg, cfg, message=""):
        self.pg = pg
        self.ui = UI(pg)
        self.ui.picker = None
        self.cfg = cfg
        self.message = message
        self.message_col = RED if message else DIM
        self.result = None
        self.done = False
        self.robot_scroll = 0
        self.yaml_dir = USER_DIR if os.path.isdir(USER_DIR) else os.getcwd()

    # ------------------------------------------------------------ actions
    def say(self, text, ok=False):
        self.message = text
        self.message_col = GREEN if ok else RED

    def add_robot(self, template=None):
        names = {r.name for r in self.cfg.robots}
        if template is not None:
            team, code = template.team, template.code
        else:
            blue = sum(r.team == "blue" for r in self.cfg.robots)
            team = "blue" if blue <= len(self.cfg.robots) - blue else "yellow"
            code = self.cfg.robots[-1].code if self.cfg.robots else os.path.join(examples_dir(), "striker.py")
        i = 1
        while f"{team[0].upper()}{i}" in names:
            i += 1
        self.cfg.robots.append(RobotConfig(name=f"{team[0].upper()}{i}", team=team, code=code))

    def pick_code(self, r):
        start = os.path.dirname(r.code) if r.code and os.path.isdir(os.path.dirname(r.code)) else examples_dir()

        def done(path):
            if path:
                r.code = path
        self.ui.picker = FilePicker(self.ui, f"Program for {r.name}", (".py",), start, done)

    def load(self):
        def done(path):
            if not path:
                return
            try:
                self.cfg = load_yaml(path, validate=False)
                self.yaml_dir = os.path.dirname(path)
                self.say(f"Loaded {os.path.basename(path)}", ok=True)
            except Exception as ex:
                self.say(f"Could not load: {ex}")
        self.ui.picker = FilePicker(self.ui, "Load match configuration", (".yaml", ".yml"),
                                    self.yaml_dir, done)

    def save(self):
        def done(path):
            if not path:
                return
            try:
                save_yaml(self.cfg, path)
                self.yaml_dir = os.path.dirname(path)
                self.say(f"Saved {path}", ok=True)
            except Exception as ex:
                self.say(f"Could not save: {ex}")
        start = self.yaml_dir if os.path.isdir(self.yaml_dir) else os.getcwd()
        self.ui.picker = FilePicker(self.ui, "Save match configuration", (".yaml", ".yml"),
                                    start, done, save_name="match.yaml")

    def start(self):
        self.ui.commit()
        cfg = copy.deepcopy(self.cfg)
        try:
            cfg.validate()
        except ValueError as ex:
            self.say(str(ex))
            return
        if not any(r.enabled for r in cfg.robots):
            self.say("Turn on at least one robot")
            return
        try:
            os.makedirs(USER_DIR, exist_ok=True)
            save_yaml(cfg, LAST_MATCH)
        except OSError:
            pass
        self.result = cfg
        self.done = True

    # ------------------------------------------------------------ drawing
    def draw(self, surf):
        pg, ui, cfg = self.pg, self.ui, self.cfg
        surf.fill(BG)
        ui.begin(surf)
        ui.label("RoboCupJunior Soccer Simulator", (30, 22), TEXT, ui.big)
        ui.label("Match setup", (32, 52), DIM)

        # ---------------- left: match + simulation
        left = pg.Rect(20, 80, 440, 590)
        pg.draw.rect(surf, PANEL, left, border_radius=8)
        x, y = 36, 94
        ref, sim = cfg.referee, cfg.sim
        ui.label("Match", (x, y), TEXT, ui.big)
        y += 36
        ui.label("Referee mode", (x, y), DIM, ui.small)
        ui.segmented((x, y + 16, 408, 30), MODE_LABELS, ref.mode, lambda v: setattr(ref, "mode", v))
        ui.label(MODE_HELP.get(ref.mode, ""), (x, y + 52), DIM, ui.small)
        y += 76
        timed = ref.mode != "free"
        if timed:
            ui.label("Half duration (s)", (x, y + 13), TEXT, anchor="midleft")
            ui.number((x + 190, y), "half", ref.half_duration_s, lambda v: setattr(ref, "half_duration_s", v),
                      step=30, lo=10, hi=3600)
            y += 34
            ui.label("Halves", (x, y + 13), TEXT, anchor="midleft")
            ui.number((x + 190, y), "halves", ref.halves, lambda v: setattr(ref, "halves", v), lo=1, hi=4)
            y += 34
        ui.label("First kick-off", (x, y + 13), TEXT, anchor="midleft")
        ui.segmented((x + 190, y, 150, 26), [("blue", "Blue"), ("yellow", "Yellow")], ref.first_kickoff,
                     lambda v: setattr(ref, "first_kickoff", v), colors=TEAM_COL)
        y += 38
        if timed:
            ui.checkbox((x, y), "Lack of progress", ref.lack_of_progress,
                        lambda v: setattr(ref, "lack_of_progress", v))
            y += 28
        if ref.mode == "full":
            ui.checkbox((x, y), "Out of bounds (1 min penalty)", ref.out_of_bounds,
                        lambda v: setattr(ref, "out_of_bounds", v))
            y += 28
            ui.checkbox((x, y), "Multiple defense", ref.multiple_defense,
                        lambda v: setattr(ref, "multiple_defense", v))
            y += 28

        y = max(y + 10, 392)
        ui.label("Simulation", (x, y), TEXT, ui.big)
        y += 36
        ui.checkbox((x, y), "Sensor noise + gyro drift", sim.noise, lambda v: setattr(sim, "noise", v))
        y += 28
        ui.checkbox((x, y), "Robots hide the ball from cameras", sim.occlusion,
                    lambda v: setattr(sim, "occlusion", v))
        y += 28
        ui.checkbox((x, y), "Motor acceleration limit", sim.accel_limit, lambda v: setattr(sim, "accel_limit", v))
        y += 34
        for label, key, attr, step, lo, hi in (
                ("Front camera FOV (deg)", "fov", "front_fov_deg", 5, 10, 179),
                ("Front camera range (cm)", "range", "front_range_cm", 10, 20, 400),
                ("Kick speed (cm/s)", "kick", "kick_speed", 10, 50, 600),
                ("Max wheel speed (cm/s)", "wheel", "max_wheel_speed", 10, 20, 300)):
            ui.label(label, (x, y + 13), TEXT, anchor="midleft")
            ui.number((x + 230, y), key, getattr(sim, attr), (lambda v, a=attr: setattr(sim, a, v)),
                      step=step, lo=lo, hi=hi, width=70)
            y += 32

        # ---------------- right: robots
        right = pg.Rect(480, 80, 680, 590)
        pg.draw.rect(surf, PANEL, right, border_radius=8)
        rx = 496
        ui.label("Robots", (rx, 94), TEXT, ui.big)
        blue = sum(r.team == "blue" and r.enabled for r in cfg.robots)
        yellow = sum(r.team == "yellow" and r.enabled for r in cfg.robots)
        cnt = ui.label(f"blue {blue}", (rx + 120, 104), TEAM_COL["blue"])
        ui.label(f"yellow {yellow}", (cnt.right + 14, 104), TEAM_COL["yellow"])
        if ref.mode == "full" and (blue > 2 or yellow > 2):
            ui.label("full rules: only the first 2 per team play", (rx + 300, 106), RED, ui.small)
        hy = 136
        for text, hx in (("On", rx), ("Name", rx + 34), ("Team", rx + 194), ("Program", rx + 300)):
            ui.label(text, (hx, hy), DIM, ui.small)
        rows = pg.Rect(rx - 6, 154, 660, 456)
        row_h = 38
        visible = rows.height // row_h
        self.robot_scroll = max(0, min(self.robot_scroll, max(0, len(cfg.robots) - visible)))
        ui.wheel.append((rows, lambda dy: setattr(self, "robot_scroll", self.robot_scroll - dy)))
        for i, r in enumerate(cfg.robots[self.robot_scroll:self.robot_scroll + visible]):
            yy = rows.y + i * row_h
            ui.checkbox((rx, yy + 6), "", r.enabled, (lambda v, rr=r: setattr(rr, "enabled", v)))
            ui.textfield((rx + 34, yy + 3, 150, 28), f"name-{id(r)}", r.name,
                         (lambda t, rr=r: setattr(rr, "name", t.strip() or rr.name)))
            ui.segmented((rx + 194, yy + 3, 96, 28), [("blue", "B"), ("yellow", "Y")], r.team,
                         (lambda v, rr=r: setattr(rr, "team", v)), colors=TEAM_COL)
            exists = os.path.isfile(r.code)
            prog = os.path.basename(r.code) if r.code else "choose..."
            folder = os.path.basename(os.path.dirname(r.code)) if r.code else ""
            ui.button((rx + 300, yy + 3, 230, 28), f"{prog}   ({folder})" if folder else prog,
                      (lambda rr=r: self.pick_code(rr)), "normal" if exists else "danger", font=ui.small)
            ui.button((rx + 538, yy + 3, 52, 28), "Copy", (lambda rr=r: self._set(dup=rr)), font=ui.small)
            ui.button((rx + 596, yy + 3, 52, 28), "Del", (lambda rr=r: self._set(remove=rr)), "danger",
                      font=ui.small)
        if len(cfg.robots) > visible:
            ui.label(f"scroll for more ({len(cfg.robots)} robots)", (rx, rows.bottom - 2), DIM, ui.small)
        if not cfg.robots:
            ui.label("No robots yet - add one below", (rx, rows.y + 10), DIM)
        ui.button((rx, 622, 150, 34), "+ Add robot", self.add_robot, "accent")
        ui.label("Same program for several robots: use Copy, or pick the same file.",
                 (rx + 164, 639), DIM, ui.small, "midleft")

        # ---------------- bottom bar
        ui.button((20, 690, 120, 44), "Load...", self.load)
        ui.button((150, 690, 120, 44), "Save as...", self.save)
        if self.message:
            ui.label(ui.fit(self.message, 680, ui.small), (290, 712), self.message_col, ui.small, "midleft")
        ui.button((990, 690, 170, 44), "Start match", self.start, "primary")

        if ui.picker is not None:
            ui.hits = []          # the dialog is modal
            ui.wheel = []
            ui.picker.draw()

    def _set(self, remove=None, dup=None):
        if remove is not None:
            self.cfg.robots.remove(remove)
        if dup is not None:
            self.add_robot(template=dup)

    # ------------------------------------------------------------ loop
    def run(self, screen):
        pg = self.pg
        clock = pg.time.Clock()
        pg.key.start_text_input()
        while not self.done:
            self.draw(screen)
            pg.display.flip()
            for ev in pg.event.get():
                if ev.type == pg.QUIT:
                    return None
                if self.ui.handle(ev):
                    continue
                if ev.type == pg.KEYDOWN and self.ui.picker is None:
                    if ev.key in (pg.K_RETURN, pg.K_KP_ENTER):
                        self.start()
                    elif ev.key == pg.K_ESCAPE:
                        return None
                elif ev.type == pg.KEYDOWN and ev.key == pg.K_ESCAPE:
                    self.ui.picker = None
            clock.tick(60)
        return self.result


def run_app(scale=None, quiet=False):
    """Setup screen -> match -> (Esc) back to setup, until the window is closed."""
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    import pygame as pg
    from .render import run_window
    from .sim import Simulation

    pg.init()
    cfg = initial_config()
    message = ""
    while True:
        screen = pg.display.set_mode((W, H))
        pg.display.set_caption("RoboCupJunior Soccer Simulator - setup")
        screen_obj = SetupScreen(pg, cfg, message)
        result = screen_obj.run(screen)
        cfg = screen_obj.cfg
        if result is None:
            break
        try:
            sim = Simulation(result, echo_logs=not quiet)
        except Exception as ex:
            message = f"Could not start: {ex}"
            continue
        message = ""
        if run_window(sim, scale=scale, from_setup=True) == "quit":
            return
    pg.quit()


__all__ = ["run_app", "SetupScreen", "MODES"]
