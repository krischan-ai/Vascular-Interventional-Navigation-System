"""Closed-loop steering controller for the *physical* CathSim guidewire.

Open-loop pushing does not navigate: the pre-bent J-tip points a fixed azimuth,
digs into the vessel wall, and the wire just coils as it is inserted (measured:
~30mm of a 584mm segment_part route before stalling). This controller closes the
loop -- each step it looks at where the physical tip is versus the planned
centerline and outputs ``(delta_push, delta_rotate)`` that:

  * rotate the J-tip's bend plane to aim the tip along the path (the only real
    steering DOF a guidewire has), using sign-search hill-climbing on the tip
    heading error since the bend azimuth is not directly observable;
  * gate the insertion speed on alignment (push hard when aimed down the lumen,
    ease off when misaligned) and on contact force (back off when jamming);
  * detect stalls (no arc-length gain) and sweep rotation to find a way through,
    briefly retracting when wedged.

It deliberately uses only quantities the engine already exposes (tip position /
direction, contact force) plus the planned path, so it can drive either the
NavigationEngine physics mode or the offline measurement harness.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class AutopilotConfig:
    """Tunable gains for :class:`PhysicsAutopilot`.

    Distances are meters, angles radians, control outputs in [-1, 1].
    """

    lookahead_m: float = 0.025          # how far ahead on the path to aim
    base_push: float = 0.35             # insertion command when well aligned
    min_push: float = 0.05              # floor so we keep gently probing
    align_full_deg: float = 25.0        # <= this heading error -> full push
    align_zero_deg: float = 110.0       # >= this -> push at min_push only
    rotate_gain: float = 1.4            # P-gain mapping heading error -> rotate
    max_rotate: float = 1.0
    force_soft: float = 1.5             # contact force where push starts easing
    force_hard: float = 6.0            # contact force where push is fully cut
    stall_window: int = 12              # steps of no progress -> declare stall
    stall_advance_m: float = 0.002      # min arc-length gain counted as progress
    sweep_rotate: float = 1.0           # rotation magnitude during a stall sweep
    sweep_push: float = 0.12            # gentle push kept during a sweep
    retract_push: float = -0.25         # brief pull-back when deeply wedged
    retract_after: int = 30             # stall steps before trying a retract


class PhysicsAutopilot:
    """Drives the physical guidewire along a planned centerline.

    Usage::

        ap = PhysicsAutopilot(path_points)
        ap.reset()
        push, rotate = ap.compute(tip_pos, tip_dir, contact_force)

    ``path_points`` is an (N, 3) array of centerline points in MuJoCo meters
    (the same planned path the engine tracks). The controller is stateful across
    a run; call :meth:`reset` before each episode.
    """

    def __init__(self, path_points, config: AutopilotConfig | None = None):
        pts = np.asarray(path_points, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[0] < 2 or pts.shape[1] != 3:
            raise ValueError("path_points must be an (N>=2, 3) array")
        self._pts = pts
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self._cumlen = np.concatenate([[0.0], np.cumsum(seg)])
        self._total = float(self._cumlen[-1])
        self.config = config or AutopilotConfig()
        self.reset()

    def reset(self) -> None:
        """Clear per-episode controller state."""
        self._prev_heading: float | None = None
        self._rotate_sign: float = 1.0
        self._best_arclen: float = 0.0
        self._stall_steps: int = 0
        self._sweep_phase: float = 0.0

    # -- path geometry helpers (independent of the engine) -----------------
    def _arclen_at(self, point: np.ndarray) -> tuple[float, int]:
        """Arc length (m) of the nearest path vertex to ``point`` and its index."""
        diffs = self._pts - point
        idx = int(np.argmin(np.einsum("ij,ij->i", diffs, diffs)))
        return float(self._cumlen[idx]), idx

    def _point_at(self, s: float) -> np.ndarray:
        s = float(np.clip(s, 0.0, self._total))
        idx = int(np.searchsorted(self._cumlen, s))
        if idx <= 0:
            return self._pts[0]
        if idx >= len(self._pts):
            return self._pts[-1]
        s0, s1 = self._cumlen[idx - 1], self._cumlen[idx]
        t = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
        return self._pts[idx - 1] + t * (self._pts[idx] - self._pts[idx - 1])

    # -- control law -------------------------------------------------------
    def compute(
        self, tip_pos, tip_dir, contact_force: float = 0.0
    ) -> tuple[float, float]:
        """Return ``(delta_push, delta_rotate)`` for the current physical state."""
        cfg = self.config
        tip = np.asarray(tip_pos, dtype=np.float64)
        d = np.asarray(tip_dir, dtype=np.float64)
        nd = float(np.linalg.norm(d))
        d = d / nd if nd > 1e-9 else np.array([0.0, 0.0, 1.0])

        s_now, _ = self._arclen_at(tip)
        aim = self._point_at(s_now + cfg.lookahead_m)
        to_aim = aim - tip
        n_aim = float(np.linalg.norm(to_aim))
        desired = to_aim / n_aim if n_aim > 1e-9 else d

        cos_err = float(np.clip(np.dot(d, desired), -1.0, 1.0))
        heading = float(np.arccos(cos_err))  # 0 = aimed down the lumen

        # Stall bookkeeping: progress = deepest arc length reached so far.
        if s_now > self._best_arclen + cfg.stall_advance_m:
            self._best_arclen = s_now
            self._stall_steps = 0
        else:
            self._stall_steps += 1

        if self._stall_steps >= cfg.stall_window:
            return self._stall_action()

        # Steering: rotation spins the J-tip bend azimuth; we cannot observe that
        # azimuth, so hill-climb -- keep rotating one way while heading error
        # falls, flip when it rises.
        if self._prev_heading is not None and heading > self._prev_heading + 1e-3:
            self._rotate_sign = -self._rotate_sign
        self._prev_heading = heading
        rotate = self._rotate_sign * min(cfg.rotate_gain * heading, cfg.max_rotate)

        # Push gating on alignment, then attenuate on contact force.
        push = self._push_from_alignment(heading)
        push *= self._force_attenuation(contact_force)
        push = max(push, cfg.min_push)

        return float(np.clip(push, -1.0, 1.0)), float(np.clip(rotate, -1.0, 1.0))

    def _push_from_alignment(self, heading: float) -> float:
        cfg = self.config
        full = np.radians(cfg.align_full_deg)
        zero = np.radians(cfg.align_zero_deg)
        if heading <= full:
            frac = 1.0
        elif heading >= zero:
            frac = 0.0
        else:
            frac = 1.0 - (heading - full) / (zero - full)
        return cfg.min_push + (cfg.base_push - cfg.min_push) * frac

    def _force_attenuation(self, force: float) -> float:
        cfg = self.config
        f = abs(float(force))
        if f <= cfg.force_soft:
            return 1.0
        if f >= cfg.force_hard:
            return 0.0
        return 1.0 - (f - cfg.force_soft) / (cfg.force_hard - cfg.force_soft)

    def _stall_action(self) -> tuple[float, float]:
        """Wedged: sweep rotation to find an opening, retract if deeply stuck."""
        cfg = self.config
        # Continuous rotation sweep so the J-tip cycles through azimuths.
        self._sweep_phase += 1.0
        rotate = cfg.sweep_rotate * (1.0 if (int(self._sweep_phase) // 8) % 2 == 0 else -1.0)
        if self._stall_steps >= cfg.stall_window + cfg.retract_after:
            # Reset the stall clock after a retract attempt so we try again.
            self._stall_steps = cfg.stall_window
            return float(cfg.retract_push), float(rotate)
        return float(cfg.sweep_push), float(rotate)

    @property
    def deepest_arclen(self) -> float:
        """Deepest arc length (m) reached so far this episode."""
        return self._best_arclen
