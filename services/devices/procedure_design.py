"""Procedure and vascular access presets for navigation sessions."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AccessProfile:
    name: str
    display_name_zh: str
    vessel: str
    vessel_label_zh: str
    entry_zone: str
    entry_zone_label_zh: str
    constraints: tuple[str, ...]
    needle_entry_offset_mm: float | None = None
    needle_entry_range_mm: tuple[float, float] | None = None

    def needle_entry_label_zh(self) -> str:
        if self.name == "femoral_access":
            return f"{self.entry_zone_label_zh}，腹股沟韧带以下、股动脉分叉以上"
        if self.needle_entry_offset_mm is not None:
            return f"{self.entry_zone_label_zh}约 {self.needle_entry_offset_mm:.0f} mm"
        return self.entry_zone_label_zh


@dataclass(frozen=True)
class ProcedureDesign:
    name: str
    display_name_zh: str
    procedure_type: str
    access: AccessProfile
    guidewire_design_name: str
    support_stack: tuple[str, ...]
    target_strategy: str
    requires_large_curvature_guidance: bool = True

    def diagnostics(self, guidewire_summary: str | None = None) -> dict[str, object]:
        return {
            "name": self.name,
            "display_name_zh": self.display_name_zh,
            "procedure_type": self.procedure_type,
            "access_site": self.access.vessel,
            "access_site_label": self.access.vessel_label_zh,
            "access_route": self.access.name,
            "access_route_label": self.access.display_name_zh,
            "entry_zone": self.access.entry_zone,
            "entry_zone_label": self.access.entry_zone_label_zh,
            "needle_entry_label": self.access.needle_entry_label_zh(),
            "guidewire_design_name": self.guidewire_design_name,
            "guidewire_summary": guidewire_summary,
            "support_stack": self.support_stack,
            "support_stack_label": support_stack_label(self.support_stack),
            "target_strategy": self.target_strategy,
            "requires_large_curvature_guidance": self.requires_large_curvature_guidance,
        }


def support_stack_label(stack: tuple[str, ...]) -> str:
    labels = {
        "sheath": "鞘管",
        "guiding_catheter": "导引导管",
        "intermediate_catheter": "中间导管",
        "microcatheter": "微导管",
    }
    return " / ".join(labels.get(item, item) for item in stack)


def femoral_access() -> AccessProfile:
    return AccessProfile(
        name="femoral_access",
        display_name_zh="股动脉入路",
        vessel="common_femoral_artery",
        vessel_label_zh="股总动脉",
        entry_zone="femoral_head_projection",
        entry_zone_label_zh="股骨头投影区",
        constraints=("below_inguinal_ligament", "above_cfa_bifurcation"),
    )


def radial_access() -> AccessProfile:
    return AccessProfile(
        name="radial_access",
        display_name_zh="桡动脉入路",
        vessel="radial_artery",
        vessel_label_zh="桡动脉",
        entry_zone="proximal_to_radial_styloid",
        entry_zone_label_zh="桡骨茎突近端",
        constraints=("avoid_too_distal_puncture",),
        needle_entry_offset_mm=20.0,
        needle_entry_range_mm=(10.0, 30.0),
    )


def femoral_aorta_branch_navigation() -> ProcedureDesign:
    return ProcedureDesign(
        name="femoral_aorta_branch_navigation",
        display_name_zh="股动脉入路主动脉分支导航",
        procedure_type="aorta_branch_navigation",
        access=femoral_access(),
        guidewire_design_name="standard_014_jtip",
        support_stack=("guiding_catheter", "microcatheter"),
        target_strategy="aorta_tree_branch_target",
    )


def radial_coronary_like_navigation() -> ProcedureDesign:
    return ProcedureDesign(
        name="radial_coronary_like_navigation",
        display_name_zh="桡动脉入路冠脉样导航",
        procedure_type="coronary_like_navigation",
        access=radial_access(),
        guidewire_design_name="standard_014_jtip",
        support_stack=("guiding_catheter", "microcatheter"),
        target_strategy="coronary_like_high_curvature_branch",
    )


def default_procedure_design(name: str = "femoral_aorta_branch_navigation") -> ProcedureDesign:
    presets = {
        "femoral_aorta_branch_navigation": femoral_aorta_branch_navigation,
        "radial_coronary_like_navigation": radial_coronary_like_navigation,
    }
    try:
        return presets[name]()
    except KeyError as exc:
        raise KeyError(f"unknown procedure design preset: {name}") from exc


__all__ = [
    "AccessProfile",
    "ProcedureDesign",
    "default_procedure_design",
    "femoral_access",
    "femoral_aorta_branch_navigation",
    "radial_access",
    "radial_coronary_like_navigation",
]
