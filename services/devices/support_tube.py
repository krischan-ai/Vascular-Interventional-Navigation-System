"""Coaxial support tube model for guidewire simulations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


SupportTubeType = Literal[
    "introducer_sheath",
    "guiding_catheter",
    "intermediate_catheter",
    "microcatheter",
]


@dataclass
class SupportTube:
    name: str
    tube_type: SupportTubeType
    tip_arclen_mm: float
    inner_radius_mm: float
    outer_radius_mm: float
    bend_stiffness: float
    axial_friction: float
    lateral_constraint_ke: float
    can_follow_wire: bool
    can_deliver_device: bool
    is_active: bool = True


@dataclass
class CoaxialSupportSystem:
    sheath: SupportTube | None = None
    guiding_catheter: SupportTube | None = None
    intermediate_catheter: SupportTube | None = None
    microcatheter: SupportTube | None = None

    @classmethod
    def default(cls, *, total_length_mm: float, free_wire_length_mm: float) -> "CoaxialSupportSystem":
        support_tip = max(0.0, float(total_length_mm) - max(0.0, float(free_wire_length_mm)))
        return cls(
            sheath=SupportTube(
                "Introducer Sheath", "introducer_sheath", min(20.0, support_tip),
                0.55, 0.80, 1.4, 0.20, 2.0e5, False, True,
            ),
            guiding_catheter=SupportTube(
                "Guiding Catheter", "guiding_catheter", min(support_tip, total_length_mm * 0.45),
                0.50, 0.70, 1.2, 0.25, 2.5e5, False, True,
            ),
            intermediate_catheter=SupportTube(
                "Intermediate Catheter", "intermediate_catheter", min(support_tip, total_length_mm * 0.70),
                0.45, 0.60, 0.9, 0.28, 3.0e5, True, True,
            ),
            microcatheter=SupportTube(
                "Microcatheter", "microcatheter", support_tip,
                0.42, 0.55, 0.6, 0.30, 3.5e5, True, True,
            ),
        )

    def active_tubes(self) -> list[SupportTube]:
        tubes = [self.sheath, self.guiding_catheter, self.intermediate_catheter, self.microcatheter]
        return [tube for tube in tubes if tube is not None and tube.is_active]

    def effective_support_tube(self) -> SupportTube | None:
        tubes = self.active_tubes()
        if not tubes:
            return None
        priority = {
            "introducer_sheath": 0,
            "guiding_catheter": 1,
            "intermediate_catheter": 2,
            "microcatheter": 3,
        }
        return max(tubes, key=lambda tube: (tube.tip_arclen_mm, priority[tube.tube_type]))

    def effective_support_tip(self) -> float:
        tube = self.effective_support_tube()
        return 0.0 if tube is None else float(tube.tip_arclen_mm)

    def free_wire_length_mm(self, guidewire_tip_arclen_mm: float) -> float:
        return max(0.0, float(guidewire_tip_arclen_mm) - self.effective_support_tip())

    def support_ratio(self, total_length_mm: float) -> float:
        if total_length_mm <= 0.0:
            return 0.0
        return min(max(self.effective_support_tip() / total_length_mm, 0.0), 1.0)
