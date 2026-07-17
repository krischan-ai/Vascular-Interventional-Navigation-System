"""Clinical guidewire design presets built on top of material profiles."""

from __future__ import annotations

from dataclasses import dataclass

from services.devices.guidewire_profile import GuidewireProfile, default_guidewire_profile


DEFAULT_GUIDEWIRE_LENGTH_MM = 1800.0
DEFAULT_EXCHANGE_GUIDEWIRE_LENGTH_MM = 3000.0
DEFAULT_ACTIVE_SIM_LENGTH_MM = 200.0
DEFAULT_014_RADIUS_MM = 0.1778
DEFAULT_035_RADIUS_MM = 0.4445


@dataclass(frozen=True)
class GuidewireDesign:
    """Clinical device specification wrapping the simulated material profile."""

    name: str
    display_name_zh: str
    clinical_total_length_mm: float
    active_sim_length_mm: float
    diameter_inch: float
    radius_mm: float
    exchange_length: bool
    intended_use: tuple[str, ...]
    compatible_support: tuple[str, ...]
    profile: GuidewireProfile

    def summary_zh(self) -> str:
        length_cm = self.clinical_total_length_mm / 10.0
        return f"{self.display_name_zh} / {length_cm:.0f} cm"

    def diagnostics(self) -> dict[str, object]:
        return {
            "design_name": self.name,
            "display_name_zh": self.display_name_zh,
            "clinical_total_length_mm": self.clinical_total_length_mm,
            "active_sim_length_mm": self.active_sim_length_mm,
            "diameter_inch": self.diameter_inch,
            "radius_mm": self.radius_mm,
            "exchange_length": self.exchange_length,
            "intended_use": self.intended_use,
            "compatible_support": self.compatible_support,
            "summary_zh": self.summary_zh(),
        }


def standard_014_jtip(active_sim_length_mm: float = DEFAULT_ACTIVE_SIM_LENGTH_MM) -> GuidewireDesign:
    profile = default_guidewire_profile(
        total_length_mm=active_sim_length_mm,
        tip_shape="j_tip",
        name="soft_j_tip_training_wire",
    )
    return GuidewireDesign(
        name="standard_014_jtip",
        display_name_zh="0.014 J-tip 标准训练导丝",
        clinical_total_length_mm=DEFAULT_GUIDEWIRE_LENGTH_MM,
        active_sim_length_mm=float(active_sim_length_mm),
        diameter_inch=0.014,
        radius_mm=DEFAULT_014_RADIUS_MM,
        exchange_length=False,
        intended_use=("small_vessel", "branch_selection", "large_curvature_training"),
        compatible_support=("microcatheter", "guiding_catheter"),
        profile=profile,
    )


def exchange_014_jtip(active_sim_length_mm: float = DEFAULT_ACTIVE_SIM_LENGTH_MM) -> GuidewireDesign:
    profile = default_guidewire_profile(
        total_length_mm=active_sim_length_mm,
        tip_shape="j_tip",
        name="exchange_j_tip_wire",
    )
    return GuidewireDesign(
        name="exchange_014_jtip",
        display_name_zh="0.014 J-tip 交换导丝",
        clinical_total_length_mm=DEFAULT_EXCHANGE_GUIDEWIRE_LENGTH_MM,
        active_sim_length_mm=float(active_sim_length_mm),
        diameter_inch=0.014,
        radius_mm=DEFAULT_014_RADIUS_MM,
        exchange_length=True,
        intended_use=("device_exchange", "maintain_distal_access"),
        compatible_support=("microcatheter", "guiding_catheter"),
        profile=profile,
    )


def standard_035_straight_support(
    active_sim_length_mm: float = DEFAULT_ACTIVE_SIM_LENGTH_MM,
) -> GuidewireDesign:
    profile = default_guidewire_profile(
        total_length_mm=active_sim_length_mm,
        tip_shape="straight",
        name="straight_support_wire",
    )
    return GuidewireDesign(
        name="standard_035_straight_support",
        display_name_zh="0.035 直头支撑导丝",
        clinical_total_length_mm=DEFAULT_GUIDEWIRE_LENGTH_MM,
        active_sim_length_mm=float(active_sim_length_mm),
        diameter_inch=0.035,
        radius_mm=DEFAULT_035_RADIUS_MM,
        exchange_length=False,
        intended_use=("large_peripheral_vessel", "catheter_delivery", "sheath_support"),
        compatible_support=("sheath", "guiding_catheter"),
        profile=profile,
    )


def default_guidewire_design(
    *,
    name: str = "standard_014_jtip",
    active_sim_length_mm: float = DEFAULT_ACTIVE_SIM_LENGTH_MM,
) -> GuidewireDesign:
    presets = {
        "standard_014_jtip": standard_014_jtip,
        "exchange_014_jtip": exchange_014_jtip,
        "standard_035_straight_support": standard_035_straight_support,
    }
    try:
        return presets[name](active_sim_length_mm)
    except KeyError as exc:
        raise KeyError(f"unknown guidewire design preset: {name}") from exc


def design_from_profile(profile: GuidewireProfile) -> GuidewireDesign:
    return GuidewireDesign(
        name="standard_014_jtip",
        display_name_zh="0.014 J-tip 标准训练导丝",
        clinical_total_length_mm=DEFAULT_GUIDEWIRE_LENGTH_MM,
        active_sim_length_mm=profile.total_length_mm,
        diameter_inch=0.014,
        radius_mm=DEFAULT_014_RADIUS_MM,
        exchange_length=False,
        intended_use=("small_vessel", "branch_selection", "large_curvature_training"),
        compatible_support=("microcatheter", "guiding_catheter"),
        profile=profile,
    )


__all__ = [
    "DEFAULT_ACTIVE_SIM_LENGTH_MM",
    "DEFAULT_EXCHANGE_GUIDEWIRE_LENGTH_MM",
    "DEFAULT_GUIDEWIRE_LENGTH_MM",
    "GuidewireDesign",
    "default_guidewire_design",
    "design_from_profile",
    "exchange_014_jtip",
    "standard_014_jtip",
    "standard_035_straight_support",
]
