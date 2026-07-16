"""Segmented guidewire profile definitions.

The profile layer is intentionally backend-neutral: it describes guidewire
geometry, material parameters, and distal tip shape in millimeters. Physics
engines can map these values into their own solver units without owning the
clinical vocabulary.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Literal


TipShapeType = Literal["straight", "angled", "j_tip", "c_shape", "s_shape", "hook"]


@dataclass(frozen=True)
class TipShapeProfile:
    """Distal tip geometry and local material scaling."""

    shape_type: TipShapeType = "j_tip"
    tip_length_mm: float = 12.0
    precurve_angle_deg: float = 35.0
    precurve_radius_mm: float | None = 8.0
    precurve_plane_deg: float = 0.0
    soft_tip_length_mm: float = 10.0
    cap_length_mm: float = 2.0
    bend_stiffness_scale: float = 1.0
    torsion_stiffness_scale: float = 1.0
    max_push_speed_scale: float = 1.0
    wall_poking_risk_scale: float = 1.0


@dataclass(frozen=True)
class GuidewireSegmentParam:
    """Per-segment material/contact parameters.

    Ratios are normalized over the active simulated guidewire length, from
    distal tip (0.0) to proximal root (1.0).
    """

    name: str
    start_ratio: float
    end_ratio: float
    radius_mm: float
    mass_scale: float
    stretch_stiffness: float
    bend_stiffness: float
    torsion_stiffness: float
    stretch_damping: float
    bend_damping: float
    torsion_damping: float
    contact_radius_mm: float
    contact_ke: float
    friction_static: float
    friction_dynamic: float
    preferred_spacing_mm: float
    precurve_angle_deg: float = 0.0
    precurve_radius_mm: float | None = None
    precurve_plane_deg: float = 0.0
    max_contact_force: float | None = None
    max_breach_mm: float | None = None

    def length_mm(self, total_length_mm: float) -> float:
        return max(0.0, (self.end_ratio - self.start_ratio) * float(total_length_mm))


@dataclass(frozen=True)
class WireNode:
    """Discretized node carrying segment parameters at an arc length."""

    arclen_mm: float
    material_segment: str
    radius_mm: float
    bend_stiffness: float
    torsion_stiffness: float
    stretch_stiffness: float
    contact_radius_mm: float
    contact_ke: float
    friction_dynamic: float
    support_state: str = "free"


@dataclass(frozen=True)
class GuidewireProfile:
    """Full guidewire material profile plus distal tip preset."""

    name: str
    total_length_mm: float
    segments: tuple[GuidewireSegmentParam, ...]
    tip_shape: TipShapeProfile

    def __post_init__(self) -> None:
        if self.total_length_mm <= 0:
            raise ValueError("total_length_mm must be positive")
        if not self.segments:
            raise ValueError("GuidewireProfile requires at least one segment")

        previous_end = 0.0
        for segment in self.segments:
            if segment.start_ratio < -1e-9 or segment.end_ratio > 1.0 + 1e-9:
                raise ValueError(f"segment ratio out of [0, 1]: {segment.name}")
            if segment.end_ratio <= segment.start_ratio:
                raise ValueError(f"segment end must be greater than start: {segment.name}")
            if abs(segment.start_ratio - previous_end) > 1e-6:
                raise ValueError(f"segments must be contiguous at {segment.name}")
            previous_end = segment.end_ratio
        if abs(previous_end - 1.0) > 1e-6:
            raise ValueError("segments must cover the full [0, 1] guidewire")

    def segment_at(self, arclen_mm: float) -> GuidewireSegmentParam:
        ratio = float(arclen_mm) / self.total_length_mm
        ratio = min(max(ratio, 0.0), 1.0)
        for segment in self.segments:
            if ratio < segment.end_ratio or segment is self.segments[-1]:
                return segment
        return self.segments[-1]

    def segment_by_name(self, name: str) -> GuidewireSegmentParam:
        for segment in self.segments:
            if segment.name == name:
                return segment
        raise KeyError(name)

    @property
    def distal_segments(self) -> tuple[GuidewireSegmentParam, GuidewireSegmentParam]:
        return (
            self.segment_by_name("atraumatic_cap"),
            self.segment_by_name("pre_shaped_soft_tip"),
        )


def _tip_shape(shape_type: TipShapeType) -> TipShapeProfile:
    presets: dict[TipShapeType, TipShapeProfile] = {
        "straight": TipShapeProfile(
            shape_type="straight",
            precurve_angle_deg=0.0,
            precurve_radius_mm=None,
            bend_stiffness_scale=1.15,
            torsion_stiffness_scale=1.0,
            wall_poking_risk_scale=0.9,
        ),
        "angled": TipShapeProfile(
            shape_type="angled",
            precurve_angle_deg=30.0,
            precurve_radius_mm=10.0,
            bend_stiffness_scale=1.0,
            torsion_stiffness_scale=1.0,
            wall_poking_risk_scale=1.0,
        ),
        "j_tip": TipShapeProfile(),
        "c_shape": TipShapeProfile(
            shape_type="c_shape",
            tip_length_mm=18.0,
            precurve_angle_deg=110.0,
            precurve_radius_mm=7.5,
            bend_stiffness_scale=0.85,
            torsion_stiffness_scale=0.9,
            max_push_speed_scale=0.8,
            wall_poking_risk_scale=0.8,
        ),
        "s_shape": TipShapeProfile(
            shape_type="s_shape",
            tip_length_mm=20.0,
            precurve_angle_deg=70.0,
            precurve_radius_mm=6.0,
            precurve_plane_deg=35.0,
            bend_stiffness_scale=0.8,
            torsion_stiffness_scale=0.85,
            max_push_speed_scale=0.75,
            wall_poking_risk_scale=0.95,
        ),
        "hook": TipShapeProfile(
            shape_type="hook",
            precurve_angle_deg=75.0,
            precurve_radius_mm=4.5,
            bend_stiffness_scale=0.9,
            torsion_stiffness_scale=0.9,
            max_push_speed_scale=0.7,
            wall_poking_risk_scale=1.35,
        ),
    }
    return presets[shape_type]


def default_guidewire_profile(
    *,
    total_length_mm: float = 200.0,
    tip_shape: TipShapeType = "j_tip",
    name: str | None = None,
) -> GuidewireProfile:
    """Create the default six-segment guidewire from the backend design doc."""

    tip = _tip_shape(tip_shape)
    segments = (
        GuidewireSegmentParam(
            "atraumatic_cap", 0.00, 0.01, 0.18, 0.7, 0.6, 0.08, 0.05,
            0.0, 0.02, 0.02, 0.19, 1.0e5, 0.18, 0.15, 1.0,
        ),
        GuidewireSegmentParam(
            "pre_shaped_soft_tip", 0.01, 0.10, 0.20, 0.8, 0.7, 0.16, 0.16,
            0.0, 0.04, 0.04, 0.21, 1.5e5, 0.28, 0.25, 1.2,
            tip.precurve_angle_deg,
            tip.precurve_radius_mm,
            tip.precurve_plane_deg,
        ),
        GuidewireSegmentParam(
            "flexible_transition", 0.10, 0.25, 0.23, 0.9, 0.85, 0.32, 0.4,
            0.0, 0.05, 0.05, 0.24, 2.0e5, 0.32, 0.30, 2.0,
        ),
        GuidewireSegmentParam(
            "torque_response", 0.25, 0.45, 0.25, 1.0, 0.95, 0.65, 1.0,
            0.0, 0.06, 0.06, 0.26, 2.5e5, 0.34, 0.30, 2.5,
        ),
        GuidewireSegmentParam(
            "main_support_shaft", 0.45, 0.85, 0.27, 1.0, 1.0, 1.0, 1.0,
            0.0, 0.07, 0.07, 0.28, 3.0e5, 0.38, 0.35, 3.0,
        ),
        GuidewireSegmentParam(
            "proximal_control", 0.85, 1.00, 0.27, 1.05, 1.0, 1.1, 1.25,
            0.0, 0.07, 0.07, 0.28, 3.0e5, 0.38, 0.35, 4.0,
        ),
    )

    if tip.bend_stiffness_scale != 1.0 or tip.torsion_stiffness_scale != 1.0:
        soft = segments[1]
        segments = (
            segments[0],
            replace(
                soft,
                bend_stiffness=soft.bend_stiffness * tip.bend_stiffness_scale,
                torsion_stiffness=soft.torsion_stiffness * tip.torsion_stiffness_scale,
            ),
            *segments[2:],
        )

    profile_name = name or f"soft_{tip_shape}_training_wire"
    return GuidewireProfile(profile_name, float(total_length_mm), segments, tip)


def discretize_guidewire(profile: GuidewireProfile) -> list[WireNode]:
    """Discretize a profile using each segment's preferred spacing."""

    nodes: list[WireNode] = []
    for segment in profile.segments:
        start_mm = segment.start_ratio * profile.total_length_mm
        length_mm = segment.length_mm(profile.total_length_mm)
        count = max(1, int(round(length_mm / max(segment.preferred_spacing_mm, 1e-6))))
        for index in range(count):
            frac = index / count
            arclen_mm = start_mm + frac * length_mm
            nodes.append(_node_from_segment(arclen_mm, segment))

    end_segment = profile.segments[-1]
    nodes.append(_node_from_segment(profile.total_length_mm, end_segment))
    return nodes


def _node_from_segment(arclen_mm: float, segment: GuidewireSegmentParam) -> WireNode:
    return WireNode(
        arclen_mm=float(arclen_mm),
        material_segment=segment.name,
        radius_mm=segment.radius_mm,
        bend_stiffness=segment.bend_stiffness,
        torsion_stiffness=segment.torsion_stiffness,
        stretch_stiffness=segment.stretch_stiffness,
        contact_radius_mm=segment.contact_radius_mm,
        contact_ke=segment.contact_ke,
        friction_dynamic=segment.friction_dynamic,
    )
