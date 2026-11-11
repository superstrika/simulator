"""Runs user robot code in threads, locked to simulation time.

Each robot thread owns a *local clock*.  `robot.delay(ms)` advances it by `ms`,
and every other API call costs a little simulated CPU time
(`sim.api_call_cost_ms`).  When a thread's clock gets ahead of the simulation
it blocks until the simulation catches up.  Only one thread runs at a time and
always in the same order, so matches are reproducible.

Line-sensor interrupts are delivered inside the robot thread at its next API
call (including while it sleeps in `delay()`), before control returns to the
user code - like a hardware interrupt pre-empting the main loop.
"""
import importlib.util
import inspect
import os
import sys
import threading
import time
import traceback
from collections import deque

EPS = 1e-9

_thread_owner = {}          # thread ident -> RobotController
_real_stdout = sys.stdout


class RobotReset(BaseException):
    """Raised inside robot code to stop it (reset / half time). Do not catch it."""


class _Slot:
    """Handshake events for one generation of a robot thread."""

    def __init__(self):
        self.go = threading.Event()
        self.done = threading.Event()


class _StdoutRouter:
    def write(self, s):
        ctrl = _thread_owner.get(threading.get_ident())
        if ctrl is None:
            return _real_stdout.write(s)
        ctrl.log_write(s)
        return len(s)

    def flush(self):
        _real_stdout.flush()

    def __getattr__(self, name):
        return getattr(_real_stdout, name)


def install_stdout_router():
    """Route print() from robot threads into each robot's log (re-wraps if stdout was replaced)."""
    global _real_stdout
    if not isinstance(sys.stdout, _StdoutRouter):
        _real_stdout = sys.stdout
        sys.stdout = _StdoutRouter()


class RobotController:
    def __init__(self, sim, entity):
        self.sim = sim
        self.entity = entity
        self.hw = entity.hw
        self.name = entity.cfg.name
        self.code_path = os.path.abspath(entity.cfg.code)
        self.generation = 0
        self.slot = _Slot()
        self.thread = None
        self.state = "idle"   # idle | running | finished | crashed | hung | stopped
        self.error = None
        self.logs = deque(maxlen=200)
        self._partial = ""
        self.local_time = 0.0
        self.reset_requested = False
        self.isr = None
        self.isr_with_robot = False
        self.in_isr = False

    # ================================================================ main-thread side
    def start(self):
        """Start (or restart) the robot program from its first line."""
        self.stop()
        self.generation += 1
        self.slot = _Slot()
        self.local_time = self.sim.time
        self.reset_requested = False
        self.isr = None
        self.in_isr = False
        self.error = None
        self.hw.pending_interrupts.clear()
        self.hw.stop()
        self.hw.set_dribbler(False)
        self.hw.reset_gyro_zero()
        self.state = "running"
        install_stdout_router()
        self.thread = threading.Thread(target=self._thread_main, args=(self.generation, self.slot),
                                       name=f"robot-{self.name}", daemon=True)
        self.thread.start()

    def stop(self):
        """Ask the running program to stop (raises RobotReset inside it)."""
        if self.thread is None or self.state not in ("running", "hung"):
            return
        if self.state == "running":
            self.reset_requested = True
            self._resume(timeout=self.sim.cfg.sim.watchdog_s)
        # a hung thread is abandoned; it raises RobotReset at its next API call
        self.generation += 1
        self.state = "stopped"
        self.hw.stop()

    def service(self):
        """Called by the simulator once per physics step, before the step."""
        if self.state == "hung":
            if self.slot.done.is_set():  # it finally reached an API call
                self.state = "running"
                self.error = None
            else:
                return
        if self.state != "running":
            return
        if (self.local_time <= self.sim.time + EPS or self.reset_requested
                or (self.hw.pending_interrupts and not self.in_isr)):
            if not self._resume(timeout=self.sim.cfg.sim.watchdog_s):
                self.state = "hung"
                self.error = ("Robot code has not called the robot API for "
                              f"{self.sim.cfg.sim.watchdog_s:.0f}s of real time "
                              "(infinite loop without robot.delay()?)")
                self.hw.stop()

    def _resume(self, timeout):
        slot = self.slot
        slot.done.clear()
        slot.go.set()
        return slot.done.wait(timeout)

    # ================================================================ robot-thread side
    def _thread_main(self, gen, slot):
        _thread_owner[threading.get_ident()] = self
        slot.go.wait()
        slot.go.clear()
        try:
            module = self._load_module(gen)
            main = getattr(module, "main", None)
            if not callable(main):
                raise AttributeError(f"{os.path.basename(self.code_path)} must define main(robot)")
            from .api import Robot
            main(Robot(self))
            if gen == self.generation:
                self.state = "finished"
        except RobotReset:
            if gen == self.generation:
                self.state = "stopped"
        except Exception:
            if gen == self.generation:
                self.error = traceback.format_exc()
                self.state = "crashed"
                _real_stdout.write(f"\n[{self.name}] robot code crashed:\n{self.error}\n")
        finally:
            if gen == self.generation:
                self.hw.stop()
                self.flush_log()
            _thread_owner.pop(threading.get_ident(), None)
            slot.done.set()

    def _load_module(self, gen):
        d = os.path.dirname(self.code_path)
        if d not in sys.path:
            sys.path.insert(0, d)
        mod_name = f"rcj_robot_{self.name}_{gen}".replace("-", "_").replace(" ", "_")
        spec = importlib.util.spec_from_file_location(mod_name, self.code_path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def _check_alive(self, gen):
        if gen != self.generation or self.reset_requested:
            raise RobotReset()

    def _handoff(self):
        slot = self.slot
        slot.go.clear()
        slot.done.set()
        slot.go.wait()
        slot.go.clear()

    def sync(self, gen):
        """Block until simulation time reaches our local clock; run pending interrupts."""
        while True:
            self._check_alive(gen)
            if self.hw.pending_interrupts and not self.in_isr:
                if self.isr is None:
                    self.hw.pending_interrupts.clear()
                else:
                    self._run_isr(gen)
            if self.local_time <= self.sim.time + EPS:
                return
            self._handoff()

    def _run_isr(self, gen):
        pending = list(self.hw.pending_interrupts)
        self.hw.pending_interrupts.clear()
        saved = self.local_time
        self.local_time = self.sim.time   # the interrupt runs "now", even mid-delay
        self.in_isr = True
        try:
            for idx in pending:
                if self.isr_with_robot:
                    self.isr(self.api_obj, idx)
                else:
                    self.isr(idx)
        finally:
            self.in_isr = False
            self.local_time = max(saved, self.local_time)

    def api_call(self, gen, cost=True):
        self._check_alive(gen)
        if cost:
            self.local_time += self.sim.cfg.sim.api_call_cost_ms / 1000.0
        self.sync(gen)

    def delay(self, gen, ms):
        self._check_alive(gen)
        # never let the local clock lag more than one physics step behind
        base = max(self.local_time, self.sim.time - self.sim.dt)
        self.local_time = base + max(0.0, float(ms)) / 1000.0
        self.sync(gen)

    def set_isr(self, func, api_obj):
        if func is None:
            self.isr = None
            return
        if not callable(func):
            raise TypeError("set_interrupt needs a function")
        try:
            n = len([p for p in inspect.signature(func).parameters.values()
                     if p.default is p.empty and p.kind in (p.POSITIONAL_ONLY, p.POSITIONAL_OR_KEYWORD)])
        except (TypeError, ValueError):
            n = 1
        self.isr_with_robot = n >= 2
        self.api_obj = api_obj
        self.isr = func

    # ================================================================ logging
    def log_write(self, s):
        self._partial += s
        while "\n" in self._partial:
            line, self._partial = self._partial.split("\n", 1)
            self._emit(line)

    def flush_log(self):
        if self._partial:
            self._emit(self._partial)
            self._partial = ""

    def _emit(self, line):
        t = self.sim.time
        self.logs.append((t, line))
        if self.sim.echo_logs and self.sim.cfg.sim.echo_prints:
            _real_stdout.write(f"[{t:7.2f}s {self.name}] {line}\n")
