import pytest

from services.devices.guidewire_profile import (
    GuidewireProfile,
    GuidewireSegmentParam,
    default_guidewire_profile,
    discretize_guidewire,
)
from services.devices.support_tube import (
    CoaxialSupportSystem,
    SupportTube,
    default_support_system,
)


def test_default_profile_has_six_contiguous_segments():
    profile = default_guidewire_profile(total_length_mm=200.0)

    assert [segment.name for segment in profile.segments] == [
        "atraumatic_cap",
        "pre_shaped_soft_tip",
        "flexible_transition",
        "torque_response",
        "main_support_shaft",
        "proximal_control",
    ]
    assert profile.segments[0].start_ratio == 0.0
    assert profile.segments[-1].end_ratio == 1.0
    assert [segment.start_ratio for segment in profile.segments[1:]] == pytest.approx(
        [segment.end_ratio for segment in profile.segments[:-1]]
    )


def test_segment_at_uses_distal_to_proximal_arclength():
    profile = default_guidewire_profile(total_length_mm=200.0)

    assert profile.segment_at(0.5).name == "atraumatic_cap"
    assert profile.segment_at(10.0).name == "pre_shaped_soft_tip"
    assert profile.segment_at(60.0).name == "torque_response"
    assert profile.segment_at(180.0).name == "proximal_control"
    assert profile.segment_at(999.0).name == "proximal_control"


def test_tip_shape_switch_changes_precurve_and_soft_tip_stiffness():
    straight = default_guidewire_profile(tip_shape="straight")
    hook = default_guidewire_profile(tip_shape="hook")

    straight_tip = straight.segment_by_name("pre_shaped_soft_tip")
    hook_tip = hook.segment_by_name("pre_shaped_soft_tip")

    assert straight.tip_shape.shape_type == "straight"
    assert straight_tip.precurve_angle_deg == 0.0
    assert hook.tip_shape.shape_type == "hook"
    assert hook_tip.precurve_angle_deg == pytest.approx(75.0)
    assert hook_tip.bend_stiffness < straight_tip.bend_stiffness
    assert hook.tip_shape.wall_poking_risk_scale > straight.tip_shape.wall_poking_risk_scale


def test_discretization_is_non_uniform_and_carries_material_params():
    profile = default_guidewire_profile(total_length_mm=200.0)
    nodes = discretize_guidewire(profile)

    distal_nodes = [node for node in nodes if node.material_segment == "pre_shaped_soft_tip"]
    shaft_nodes = [node for node in nodes if node.material_segment == "main_support_shaft"]

    assert nodes[0].arclen_mm == pytest.approx(0.0)
    assert nodes[-1].arclen_mm == pytest.approx(200.0)
    assert len(distal_nodes) > 0
    assert len(shaft_nodes) > 0
    assert distal_nodes[0].bend_stiffness < shaft_nodes[0].bend_stiffness
    assert distal_nodes[0].torsion_stiffness < shaft_nodes[0].torsion_stiffness
    assert len(distal_nodes) / profile.segment_by_name("pre_shaped_soft_tip").length_mm(200.0) > (
        len(shaft_nodes) / profile.segment_by_name("main_support_shaft").length_mm(200.0)
    )


def test_profile_requires_full_contiguous_coverage():
    segment = GuidewireSegmentParam(
        "only_tip", 0.0, 0.5, 0.2, 1.0, 1.0, 1.0, 1.0,
        0.0, 0.0, 0.0, 0.2, 1.0, 0.2, 0.2, 1.0,
    )

    with pytest.raises(ValueError, match="cover"):
        GuidewireProfile("bad", 100.0, (segment,), default_guidewire_profile().tip_shape)


def test_effective_support_tip_uses_most_distal_active_tube():
    system = CoaxialSupportSystem(
        sheath=SupportTube("sheath", "introducer_sheath", 20.0, 1.0, 1.2, 1.0, 0.3, 1.0, False, True),
        guiding_catheter=SupportTube("guide", "guiding_catheter", 80.0, 0.8, 1.0, 0.8, 0.3, 1.0, False, True),
        intermediate_catheter=SupportTube("intermediate", "intermediate_catheter", 110.0, 0.5, 0.7, 0.5, 0.3, 1.0, True, True),
        microcatheter=SupportTube("micro", "microcatheter", 125.0, 0.3, 0.4, 0.3, 0.2, 1.0, True, False),
    )

    assert system.effective_support_tip() == pytest.approx(125.0)
    assert system.effective_support_type() == "microcatheter"
    assert system.free_wire_length_mm(150.0) == pytest.approx(25.0)
    assert system.support_ratio(150.0) == pytest.approx(125.0 / 150.0)
    assert system.support_state_at(100.0) == "inside_support_tube"
    assert system.support_state_at(130.0) == "distal_free_span"


def test_microcatheter_advance_shortens_free_wire_length():
    before = default_support_system(guidewire_tip_arclen_mm=150.0, free_wire_length_mm=35.0)
    after = default_support_system(guidewire_tip_arclen_mm=150.0, free_wire_length_mm=20.0)

    assert before.effective_support_type() == "microcatheter"
    assert after.effective_support_tip() > before.effective_support_tip()
    assert after.free_wire_length_mm(150.0) < before.free_wire_length_mm(150.0)


def test_default_guidewire_design_presets_keep_clinical_and_sim_lengths_separate():
    from services.devices import (
        default_guidewire_design,
        exchange_014_jtip,
        standard_035_straight_support,
    )

    standard = default_guidewire_design(active_sim_length_mm=60.0)
    exchange = exchange_014_jtip(active_sim_length_mm=60.0)
    support = standard_035_straight_support(active_sim_length_mm=60.0)

    assert standard.name == "standard_014_jtip"
    assert standard.clinical_total_length_mm == pytest.approx(1800.0)
    assert standard.active_sim_length_mm == pytest.approx(60.0)
    assert standard.profile.total_length_mm == pytest.approx(60.0)
    assert standard.diameter_inch == pytest.approx(0.014)
    assert standard.radius_mm == pytest.approx(0.1778)
    assert standard.exchange_length is False
    assert standard.profile.tip_shape.shape_type == "j_tip"
    assert standard.summary_zh() == "0.014 J-tip 标准训练导丝 / 180 cm"

    assert exchange.clinical_total_length_mm == pytest.approx(3000.0)
    assert exchange.exchange_length is True
    assert exchange.profile.tip_shape.shape_type == "j_tip"

    assert support.diameter_inch == pytest.approx(0.035)
    assert support.radius_mm == pytest.approx(0.4445)
    assert support.profile.tip_shape.shape_type == "straight"


def test_default_access_and_procedure_presets_expose_clinical_context():
    from services.devices import (
        default_procedure_design,
        femoral_access,
        radial_access,
        radial_coronary_like_navigation,
    )

    femoral = femoral_access()
    radial = radial_access()
    procedure = default_procedure_design()
    radial_procedure = radial_coronary_like_navigation()

    assert femoral.vessel == "common_femoral_artery"
    assert femoral.vessel_label_zh == "股总动脉"
    assert femoral.entry_zone == "femoral_head_projection"
    assert femoral.needle_entry_label_zh() == "股骨头投影区，腹股沟韧带以下、股动脉分叉以上"

    assert radial.vessel == "radial_artery"
    assert radial.needle_entry_offset_mm == pytest.approx(20.0)
    assert radial.needle_entry_range_mm == pytest.approx((10.0, 30.0))

    assert procedure.name == "femoral_aorta_branch_navigation"
    assert procedure.guidewire_design_name == "standard_014_jtip"
    assert procedure.support_stack == ("guiding_catheter", "microcatheter")
    diag = procedure.diagnostics("0.014 J-tip 标准训练导丝 / 180 cm")
    assert diag["access_site"] == "common_femoral_artery"
    assert diag["access_route_label"] == "股动脉入路"
    assert diag["support_stack_label"] == "导引导管 / 微导管"
    assert diag["guidewire_summary"] == "0.014 J-tip 标准训练导丝 / 180 cm"

    assert radial_procedure.access.name == "radial_access"
