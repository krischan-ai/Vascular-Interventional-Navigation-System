"""PhysicsEngine seam: the engine-agnostic contract and shared path geometry.

This module defines the minimal stable interface every backend (kinematic /
MuJoCo / future Warp) must satisfy, plus the value types they exchange with
``NavigationEngine``:

- :class:`RawPose` -- one step's *raw* read. It carries only quantities every
  backend can produce directly (tip pose, contact force, wall gap, joints,
  reward/done). Derived quantities (path_progress/deviation, curvature,
  safety_status, risk_score) are intentionally *absent*: ``NavigationEngine``
  computes them from a ``RawPose`` plus its planned-path knowledge, so they never
  have to be reimplemented per engine.
- :class:`PlannedPath` -- arc-length geometry over the planned centerline, shared
  by the kinematic engine (drives motion), the MuJoCo engine (prethread IK) and
  ``NavigationEngine`` (progress/deviation).

The one concession to engine-specific progress semantics is ``RawPose.arclen``:
engines that track an along-path coordinate report it so ``NavigationEngine``
can derive exact, continuous progress; engines that leave it ``None`` fall back
to nearest-vertex progress. Deviation is always derived independently from the
real tip position, so an along-path coordinate cannot hide lateral displacement.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

import numpy as np

# Distance reported when the guidewire is not in contact with any wall (m).
# Shared sentinel so the kinematic engine (no contacts) and the MuJoCo engine
# (no active contacts this step) agree on the free-space value.
MAX_WALL_DISTANCE = 0.05


@dataclass
class RawPose:
    """Engine-agnostic raw read of a single simulation step.

    Coordinates and distances are in MuJoCo units (meters). The quaternion uses
    [x, y, z, w] order to match the Godot/WebSocket protocol convention.
    """

    tip_position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    tip_direction: list[float] = field(default_factory=lambda: [0.0, 0.0, 1.0])
    tip_quaternion: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 1.0])
    # Resultant physical contact force at the guidewire tip (N).
    contact_force: float = 0.0
    contact_count: int = 0
    wall_distance: float = MAX_WALL_DISTANCE
    # Positive geometric penetration beyond the lumen boundary (m).  Kept
    # separate from contact_force so safety does not infer breach from force.
    wall_penetration: float = 0.0
    target_position: list[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    joint_positions: list[float] = field(default_factory=list)
    joint_velocities: list[float] = field(default_factory=list)
    reward: float = 0.0
    done: bool = False
    # Along-path arc length (meters), or None when the engine does not track it.
    # NavigationEngine uses this only for continuous path_progress; lateral
    # deviation is derived independently from tip_position.
    arclen: float | None = None


@runtime_checkable
class PhysicsEngine(Protocol):
    """Minimal stable interface between NavigationEngine and a physics backend.

    Implementations own everything backend-specific (model construction, joint
    layout, contacts, IK). They expose only these cross-engine concepts; derived
    navigation quantities stay in NavigationEngine.
    """

    def reset(self) -> RawPose:
        """Reset to the initial pose and return the first raw read."""
        ...

    def step(self, push: float, rotate: float) -> RawPose:
        """Advance one control step from clamped inputs in [-1, 1]."""
        ...

    def render_bodies(self) -> list[dict[str, list[float]]]:
        """Per-segment guidewire render data: ``{"pos": [x,y,z], "quat": [x,y,z,w]}``."""
        ...

    def close(self) -> None:
        """Release any held resources."""
        ...

    @property
    def control_timestep(self) -> float:
        """Seconds of simulated time per control step (for velocity estimation)."""
        ...


def quat_from_direction(direction) -> list[float]:
    """Quaternion [x, y, z, w] rotating +z onto ``direction`` (protocol order)."""
    a = np.array([0.0, 0.0, 1.0])
    b = np.asarray(direction, dtype=np.float64)
    nb = float(np.linalg.norm(b))
    if nb < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    b = b / nb
    d = float(np.dot(a, b))
    if d > 1.0 - 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    if d < -1.0 + 1e-9:
        return [1.0, 0.0, 0.0, 0.0]  # 180 deg about x
    axis = np.cross(a, b)
    w = 1.0 + d
    q = np.array([axis[0], axis[1], axis[2], w])
    q = q / np.linalg.norm(q)
    return [float(v) for v in q]


class PlannedPath:
    """Arc-length geometry over a planned centerline (meters).

    Wraps the polyline points with cumulative arc length and an optional KD-tree
    so callers can sample a point/tangent at a given arc length and project a tip
    onto the path for progress/deviation. Shared by both engines and the
    orchestrator so the geometry lives in exactly one place.
    """

    def __init__(self, points, radii=None) -> None:
        pts = np.asarray(points, dtype=np.float64)
        segment_len = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.points = pts
        self.cumlen = np.concatenate([[0.0], np.cumsum(segment_len)])
        self.total_len = float(self.cumlen[-1])
        # Optional per-point lumen radius (meters), e.g. real VMTK inscribed radii
        # carried by multi-branch routes. Consumed by NewtonEngine to build a
        # variable-radius vessel wall; ignored by other engines. None when absent.
        if radii is None:
            self.radii = None
        else:
            r = np.asarray(radii, dtype=np.float64).reshape(-1)
            self.radii = r if len(r) == len(pts) else None

        try:
            from scipy.spatial import cKDTree

            self._kdtree = cKDTree(pts)
        except Exception:
            self._kdtree = None

    def point_at_arclen(self, s: float) -> np.ndarray:
        """Interpolate a point on the path at arc length ``s`` (meters)."""
        cum = self.cumlen
        pts = self.points
        s = float(np.clip(s, 0.0, self.total_len))
        idx = int(np.searchsorted(cum, s))
        if idx <= 0:
            return pts[0].copy()
        if idx >= len(pts):
            return pts[-1].copy()
        s0, s1 = cum[idx - 1], cum[idx]
        t = 0.0 if s1 <= s0 else (s - s0) / (s1 - s0)
        return pts[idx - 1] + t * (pts[idx] - pts[idx - 1])

    def radius_at_arclen(self, s: float) -> float | None:
        """Interpolate the lumen radius at arc length ``s`` (meters), or None."""
        if self.radii is None:
            return None
        return float(np.interp(float(np.clip(s, 0.0, self.total_len)), self.cumlen, self.radii))

    def tangent_at_arclen(self, s: float) -> np.ndarray:
        """Unit tangent of the path at arc length ``s`` (meters)."""
        eps = max(self.total_len * 1e-3, 1e-4)
        p0 = self.point_at_arclen(s - eps)
        p1 = self.point_at_arclen(s + eps)
        d = p1 - p0
        n = float(np.linalg.norm(d))
        return d / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])

    def progress_deviation(self, tip) -> tuple[float, float]:
        """Progress in [0, 1] and deviation (m) for a tip via nearest vertex.

        This is the projection used in physics mode, where only the tip position
        is known. Returns (0.0, 0.0) for a degenerate (zero-length) path.
        """
        if self.total_len <= 0.0:
            return 0.0, 0.0

        tip = np.asarray(tip, dtype=np.float64)
        if self._kdtree is not None:
            deviation, idx = self._kdtree.query(tip)
            idx = int(idx)
        else:
            diffs = self.points - tip
            sq = np.einsum("ij,ij->i", diffs, diffs)
            idx = int(np.argmin(sq))
            deviation = float(np.sqrt(sq[idx]))

        progress = float(self.cumlen[idx] / self.total_len)
        return progress, float(deviation)

    def deviation(self, tip) -> float:
        """Shortest distance (m) from ``tip`` to the planned polyline.

        Unlike :meth:`progress_deviation`, this projects onto every line segment
        instead of measuring to the nearest sampled vertex. It is used when an
        engine already supplies a continuous along-path coordinate but the real
        tip may still move laterally, as in Newton force-drive mode.

        Returns 0.0 for a degenerate zero-length path, matching the established
        missing/degenerate path behavior of :meth:`progress_deviation`.
        """
        if self.total_len <= 0.0 or len(self.points) < 2:
            return 0.0

        tip = np.asarray(tip, dtype=np.float64)
        start = self.points[:-1]
        segment = self.points[1:] - start
        tip_offset = tip - start
        segment_sq = np.einsum("ij,ij->i", segment, segment)
        projection = np.zeros_like(segment_sq)
        valid = segment_sq > 1e-18
        projection[valid] = (
            np.einsum("ij,ij->i", tip_offset[valid], segment[valid])
            / segment_sq[valid]
        )
        projection = np.clip(projection, 0.0, 1.0)
        closest = start + projection[:, None] * segment
        distance_sq = np.einsum("ij,ij->i", tip - closest, tip - closest)
        return float(np.sqrt(np.min(distance_sq)))

    def inner_wall_offset(self, s: float, wall_lean: float, gain: float) -> np.ndarray:
        """Offset toward the inner (concave) side of the local curve at ``s``.

        Zero on straight sections; grows with curvature, capped at ``wall_lean``.
        Used by the kinematic render so the wire hugs the inner wall at bends.
        """
        delta = 0.01
        p_prev = self.point_at_arclen(s - delta)
        p = self.point_at_arclen(s)
        p_next = self.point_at_arclen(s + delta)
        inward = 0.5 * (p_prev + p_next) - p  # toward the center of curvature
        # Keep only the lateral component; a one-sided difference at the path
        # ends (where p_prev/p_next clamp) is purely tangential and must not
        # shift the wire along its own direction.
        tangent = self.tangent_at_arclen(s)
        inward = inward - tangent * float(np.dot(inward, tangent))
        n = float(np.linalg.norm(inward))
        if n < 1e-9:
            return np.zeros(3)
        mag = min(wall_lean, n * gain)
        return (inward / n) * mag

    def as_list(self) -> list[list[float]]:
        """The path points as a plain list of [x, y, z]."""
        return self.points.tolist()
