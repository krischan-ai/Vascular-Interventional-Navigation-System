"""Coaxial support system definitions for guidewire simulations."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Literal


TubeType = Literal[
    "introducer_sheath",
    "guiding_catheter",
    "intermediate_catheter",
    "microcatheter",
]


@dataclass(frozen=True)
class SupportTube:
    """One active tube in the coaxial support stack."""

    name: str
    tube_type: TubeType
    tip_arclen_mm: float
    inner_radius_mm: float
    outer_radius_mm: float
    bend_stiffness: float
    axial_friction: float
    lateral_constraint_ke: float
    can_follow_wire: bool
    can_deliver_device: bool
    is_active: bool = True

    def __post_init__(self) -> None:
        if self.tip_arclen_mm < 0:
            raise ValueError("tip_arclen_mm must be non-negative")
        if self.inner_radius_mm <= 0 or self.outer_radius_mm <= 0:
            raise ValueError("tube radii must be positive")
        if self.outer_radius_mm < self.inner_radius_mm:
            raise ValueError("outer_radius_mm must be >= inner_radius_mm")


@dataclass(frozen=True)
class CoaxialSupportSystem:
    """External support tubes that constrain guidewire state dynamically."""

    sheath: SupportTube | None = None
    guiding_catheter: SupportTube | None = None
    intermediate_catheter: SupportTube | None = None
    microcatheter: SupportTube | None = None

    def active_tubes(self) -> tuple[SupportTube, ...]:
        return tuple(
            tube
            for tube in (
                self.sheath,
                self.guiding_catheter,
                self.intermediate_catheter,
                self.microcatheter,
            )
            if tube is not None and tube.is_active
        )

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

    def effective_support_type(self) -> str | None:
        tube = self.effective_support_tube()
        return None if tube is None else tube.tube_type

    def free_wire_length_mm(self, guidewire_tip_arclen_mm: float) -> float:
        return max(0.0, float(guidewire_tip_arclen_mm) - self.effective_support_tip())

    def support_ratio(self, guidewire_tip_arclen_mm: float) -> float:
        tip = max(0.0, float(guidewire_tip_arclen_mm))
        if tip <= 0.0:
            return 0.0
        return min(1.0, self.effective_support_tip() / tip)

    def support_state_at(self, arclen_mm: float) -> str:
        if float(arclen_mm) <= self.effective_support_tip():
            return "inside_support_tube"
        return "distal_free_span"


def default_support_system(
    *,
    guidewire_tip_arclen_mm: float = 150.0,
    free_wire_length_mm: float = 30.0,
) -> CoaxialSupportSystem:
    """Create a conservative default stack with a microcatheter as support tip."""

    support_tip = max(0.0, float(guidewire_tip_arclen_mm) - float(free_wire_length_mm))
    return CoaxialSupportSystem(
        sheath=SupportTube(
            "introducer_sheath",
            "introducer_sheath",
            25.0,
            1.0,
            1.4,
            1.0,
            0.4,
            1.0e5,
            False,
            True,
        ),
        guiding_catheter=SupportTube(
            "guiding_catheter",
            "guiding_catheter",
            min(80.0, support_tip),
            0.75,
            1.0,
            0.8,
            0.35,
            1.5e5,
            False,
            True,
        ),
        intermediate_catheter=SupportTube(
            "intermediate_catheter",
            "intermediate_catheter",
            min(105.0, support_tip),
            0.45,
            0.65,
            0.5,
            0.3,
            2.0e5,
            True,
            True,
        ),
        microcatheter=SupportTube(
            "microcatheter",
            "microcatheter",
            support_tip,
            0.28,
            0.42,
            0.25,
            0.25,
            2.5e5,
            True,
            False,
        ),
    )
