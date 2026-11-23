"""Scorer for the cooler-twist task (pure Python, no simulator): take an AMD Wraith Prism cooler off an AM4 processor
whose thermal paste has dried, and put it on the anti-static mat, with the processor left in its socket.

Inputs each control step (20 Hz), all read from the simulator by the task kit (``isaac/coolerkit.py``):
* ``cpu_lift_mm``: the processor's height above its seated position in the socket (articulation joint ``pz``);
* ``parted``: the cooler's base has come away from the processor's lid (the paste film no longer holds it; the
  cooler is free of the processor);
* ``cooler_on_mat``: the cooler rests upright on the mat (its centre over the mat, its base on the mat surface, tilt
  under 15 deg, at rest);
* ``contact``: dict of robot-contact flags for 'cooler', 'cpu', 'bench' (PhysX contact forces between the robot's
  links and those bodies: collider identity, not positions).

Rules (the trial's score is the level at its end; the veto latches 0):
* veto ``cpu_yanked``: the processor came YANK_MM or more out of its socket at any time (it came off with the cooler:
  its pins leave the closed ZIF socket and bend; the real-world failure this task is about);
* 1.0 (``success``; ends the trial): the cooler rests on the mat, the processor is in its socket and was never pulled
  out, and the robot has not touched the cooler, processor or bench for CLEAR_S;
* 0.5: the cooler is off the processor (parted) and the processor never came out, but the cooler is not resting on
  the mat at the end (still held, or put down somewhere else);
* 0 otherwise (cooler still on the processor; any veto).
"""

from __future__ import annotations

YANK_MM = 1.0
CLEAR_S = 3.0


class Scorer:
    def __init__(self):
        self.t = 0.0
        self.veto = None
        self.veto_t = None
        self.parted_t = None
        self.clear_s = 0.0
        self.max_cpu_lift = 0.0
        self.last = {}

    def _veto(self, why):
        if self.veto is None:
            self.veto, self.veto_t = why, round(self.t, 3)

    def step(self, dt, cpu_lift_mm, parted, cooler_on_mat, contact):
        self.t += dt
        self.max_cpu_lift = max(self.max_cpu_lift, cpu_lift_mm)
        if cpu_lift_mm >= YANK_MM:
            self._veto("cpu_yanked")
        if parted and self.parted_t is None:
            self.parted_t = round(self.t, 3)
        any_contact = any(contact.get(k, False) for k in ("cooler", "cpu", "bench"))
        self.clear_s = self.clear_s + dt if parted and cooler_on_mat and not any_contact else 0.0
        level = self.level(parted, cooler_on_mat)
        self.last = dict(
            level=level,
            veto=self.veto,
            veto_t=self.veto_t,
            parted=bool(parted),
            parted_t=self.parted_t,
            cooler_on_mat=bool(cooler_on_mat),
            cpu_lift_mm=round(cpu_lift_mm, 3),
            max_cpu_lift_mm=round(self.max_cpu_lift, 3),
            contact={k: bool(v) for k, v in contact.items()},
            clear_s=round(self.clear_s, 3),
        )
        return level

    def level(self, parted, cooler_on_mat):
        if self.veto is not None:
            return 0.0
        if self.parted_t is None:
            return 0.0
        if cooler_on_mat and self.clear_s >= CLEAR_S - 1e-9:
            return 1.0
        return 0.5
