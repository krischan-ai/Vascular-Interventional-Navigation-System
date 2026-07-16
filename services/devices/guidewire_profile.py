"""Segmented guidewire and tip-shape parameter models.

The profile is expressed from distal tip (ratio 0.0) to proximal root
(ratio 1.0), matching the clinical segmentation in doc/3.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Literal


TipShape = Literal["straight", "angled", "j_tip", "c_shape", "s_shape", "hook"]


@dataclass(frozen=True)
class GuidewireSegmentParam:
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

    def contains_ratio(self, ratio: float) -> bool:
        return self.start_ratio <= ratio < self.end_ratio


@dataclass(frozen=True)
class TipShapeProfile:
    shape_type: TipShape
    tip_length_mm: float
    precurve_angle_deg: float
    precurve_radius_mm: float | None
    precurve_plane_deg: float
    soft_tip_length_mm: float
    cap_length_mm: float
    bend_stiffness_scale: float
    torsion_stiffness_scale: float
    max_push_speed_scale: float
    wall_poking_risk_scale: float

    @classmethod
    def preset(cls, shape_type: str) -> "TipShapeProfile":
        shape = "j_tip" if shape_type == "j" else shape_type
        presets = {
            "straight": cls("straight", 8.0, 0.0, None, 0.0, 8.0, 2.0, 0.9, 0.9, 1.0, 0.8),
            "angled": cls("angled", 10.0, 30.0, 18.0, 0.0, 10.0, 2.0, 0.8, 0.75, 0.9, 1.0),
            "j_tip": cls("j_tip", 12.0, 60.0, 12.0, 0.0, 12.0, 2.0, 0.7, 0.65, 0.8, 0.9),
            "c_shape": cls("c_shape", 16.0, 100.0, 9.0, 0.0, 16.0, 2.0, 0.6, 0.55, 0.7, 0.9),
            "s_shape": cls("s_shape", 18.0, 75.0, 10.0, 0.0, 18.0, 2.0, 0.65, 0.6, 0.75, 1.0),
            "hook": cls("hook", 12.0, 90.0, 7.0, 0.0, 12.0, 2.0, 0.65, 0.55, 0.65, 1.35),
        }
        if shape not in presets:
            raise ValueError(f"unknown tip shape: {shape_type!r}")
        return presets[shape]


@dataclass(frozen=True)
class WireNode:
    arclen_mm: float
    material_segment: str
    radius_mm: float
    bend_stiffness: float
    torsion_stiffness: float
    support_state: str = "free"


@dataclass(frozen=True)
class GuidewireProfile:
    name: str
    total_length_mm: float
    tip_shape: TipShapeProfile
    segments: tuple[GuidewireSegmentParam, ...]

    @classmethod
    def default(
        cls,
        *,
        total_length_mm: float,
        tip_shape: str = "j_tip",
        radius_mm: float = 0.4,
        contact_ke: float = 3_000_000.0,
    ) -> "GuidewireProfile":
        tip = TipShapeProfile.preset(tip_shape)
        segments = (
            GuidewireSegmentParam(
                "atraumatic_cap", 0.00, 0.02, radius_mm, 0.7, 0.6, 0.08, 0.05,
                0.0, 0.05, 0.02, radius_mm, contact_ke, 0.12, 0.15, 1.0,
            ),
            GuidewireSegmentParam(
                "pre_shaped_soft_tip", 0.02, 0.10, radius_mm, 0.8, 0.7, 0.16, 0.16,
                0.0, 0.05, 0.03, radius_mm, contact_ke, 0.20, 0.25, 1.5,
                tip.precurve_angle_deg, tip.precurve_radius_mm, tip.precurve_plane_deg,
            ),
            GuidewireSegmentParam(
                "flexible_transition", 0.10, 0.24, radius_mm, 0.9, 0.85, 0.32, 0.45,
                0.0, 0.08, 0.04, radius_mm, contact_ke, 0.25, 0.30, 2.0,
            ),
            GuidewireSegmentParam(
                "torque_response", 0.24, 0.45, radius_mm, 1.0, 0.95, 0.70, 1.00,
                0.0, 0.10, 0.05, radius_mm, contact_ke, 0.25, 0.30, 3.0,
            ),
            GuidewireSegmentParam(
                "main_support_shaft", 0.45, 0.85, radius_mm, 1.0, 1.0, 1.0, 1.0,
                0.0, 0.10, 0.05, radius_mm, contact_ke, 0.30, 0.35, 4.0,
            ),
            GuidewireSegmentParam(
                "proximal_control", 0.85, 1.00, radius_mm, 1.05, 1.0, 1.15, 1.25,
                0.0, 0.10, 0.05, radius_mm, contact_ke, 0.30, 0.35, 5.0,
            ),
        )
        if tip.shape_type != "straight":
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
        return cls(f"segmented_{tip.shape_type}", float(total_length_mm), tip, segments)

    def segment_at_ratio(self, distal_ratio: float) -> GuidewireSegmentParam:
        ratio = min(max(float(distal_ratio), 0.0), 1.0)
        for segment in self.segments:
            if segment.contains_ratio(ratio):
                return segment
        return self.segments[-1]

    def segment_at_arclen_mm(self, arclen_from_distal_mm: float) -> GuidewireSegmentParam:
        if self.total_length_mm <= 0.0:
            return self.segments[0]
        return self.segment_at_ratio(arclen_from_distal_mm / self.total_length_mm)

    def body_segment_names(self, body_count: int) -> list[str]:
        if body_count <= 0:
            return []
        names: list[str] = []
        for i in range(body_count):
            proximal_ratio = i / max(body_count - 1, 1)
            distal_ratio = 1.0 - proximal_ratio
            names.append(self.segment_at_ratio(distal_ratio).name)
        return names

    def joint_bend_profile(self, joint_count: int, base_bend: float) -> list[float]:
        profile: list[float] = []
        for j in range(max(0, joint_count)):
            proximal_ratio = (j + 0.5) / max(joint_count, 1)
            distal_ratio = 1.0 - proximal_ratio
            segment = self.segment_at_ratio(distal_ratio)
            profile.append(float(base_bend) * segment.bend_stiffness)
        return profile


def discretize_guidewire(profile: GuidewireProfile) -> list[WireNode]:
    nodes: list[WireNode] = []
    total = profile.total_length_mm
    for segment in profile.segments:
        start = segment.start_ratio * total
        end = segment.end_ratio * total
        length = max(0.0, end - start)
        n = max(1, int(round(length / max(segment.preferred_spacing_mm, 1e-6))))
        for k in range(n):
            arclen = start + (k / n) * length
            nodes.append(WireNode(
                arclen_mm=arclen,
                material_segment=segment.name,
                radius_mm=segment.radius_mm,
                bend_stiffness=segment.bend_stiffness,
                torsion_stiffness=segment.torsion_stiffness,
            ))
    nodes.append(WireNode(
        arclen_mm=total,
        material_segment=profile.segments[-1].name,
        radius_mm=profile.segments[-1].radius_mm,
        bend_stiffness=profile.segments[-1].bend_stiffness,
        torsion_stiffness=profile.segments[-1].torsion_stiffness,
    ))
    return nodes
