"""Anti-buckling (抗屈曲) unit tests for the Newton force-drive engine.

Covers the two D4 debts attacked in this pass (doc/08 §9.1 sheath constraint,
§28.9 抗屈曲):

1. Prolapse guard: ``_feed_budget`` stops the forward feed once the fed-vs-tip
   slack exceeds ``max_slack`` so holding W against a blocked tip no longer
   piles wire into loops.
2. Sheath auto mode: ``sheath_bodies=-1`` glues all but the distal ``free_len``
   of wire, shortening the unsupported column that buckles.

All tests run without the ``newton`` package: it is imported lazily in
``_ensure_initialized``, and everything here exercises ``__init__``-level
numpy logic only.
"""

import numpy as np
import pytest

from services.physics.base import PlannedPath
from services.physics.newton_engine import (
    NewtonEngine,
    _count_static_dynamic_contacts,
    _feed_budget,
    _insertion_complete,
    _project_points_to_centerline,
    _static_dynamic_force_stats,
)


def _straight_path(total_len: float = 0.3, n: int = 61) -> PlannedPath:
    pts = np.zeros((n, 3))
    pts[:, 2] = np.linspace(0.0, total_len, n)
    return PlannedPath(pts, radii=np.full(n, 0.003))


@pytest.fixture
def engine(monkeypatch) -> NewtonEngine:
    # Pin the params the tests depend on so ambient env cannot skew them.
    monkeypatch.setenv("CATHSIM_NEWTON_DRIVE", "force")
    monkeypatch.setenv("CATHSIM_NEWTON_SEG_LEN", "0.003")
    monkeypatch.setenv("CATHSIM_NEWTON_FREE_LEN", "0.03")
    monkeypatch.delenv("CATHSIM_NEWTON_SHEATH_BODIES", raising=False)
    monkeypatch.delenv("CATHSIM_NEWTON_MAX_SLACK", raising=False)
    return NewtonEngine(path=_straight_path())


def test_zero_insertion_budget_is_not_completion():
    assert _insertion_complete(0.0, 0.0) is False
    assert _insertion_complete(0.5, 1.0) is False
    assert _insertion_complete(1.0, 1.0) is True


def test_static_dynamic_contact_counter_excludes_self_contacts():
    shape_body = np.asarray([-1, 0, 1, 2], dtype=np.int32)
    shape0 = np.asarray([0, 0, 1, 2], dtype=np.int32)
    shape1 = np.asarray([1, 3, 2, 3], dtype=np.int32)

    assert _count_static_dynamic_contacts(shape0, shape1, shape_body) == 2


def test_centerline_projection_does_not_treat_axial_spacing_as_wall_offset():
    centerline = np.asarray([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.002],
        [0.0, 0.0, 0.004],
    ])
    radii = np.asarray([0.002, 0.003, 0.004])
    xyz = np.asarray([
        [0.0, 0.0, 0.003],
        [0.0035, 0.0, 0.003],
    ])

    projected, distances, local_radii, arclens, segments = (
        _project_points_to_centerline(xyz, centerline, radii)
    )

    np.testing.assert_allclose(projected, [
        [0.0, 0.0, 0.003],
        [0.0, 0.0, 0.003],
    ])
    assert distances == pytest.approx([0.0, 0.0035])
    assert local_radii == pytest.approx([0.0035, 0.0035])
    assert arclens == pytest.approx([0.003, 0.003])
    assert segments.tolist() == [1, 1]


def test_static_dynamic_force_stats_orients_force_onto_dynamic_bodies():
    body0 = np.asarray([-1, 2, 1, -1])
    body1 = np.asarray([1, -1, 2, 3])
    force_on_body1 = np.asarray([
        [3.0, 0.0, 0.0],
        [-4.0, 0.0, 0.0],
        [99.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
    ])

    count, peak, force_sum, resultant = _static_dynamic_force_stats(
        body0, body1, force_on_body1
    )

    assert count == 3
    assert peak == 4.0
    assert force_sum == 8.0
    # Forces on the three dynamic bodies are +3, +4, and -1 along x.
    assert resultant == 6.0


def test_breach_stats_converts_penetration_to_force_proxy(engine):
    center = engine._centerline[10]
    radius = float(engine._radii[10])
    touching_offset = radius - engine._rod_radius

    touching = np.asarray([center + [touching_offset, 0.0, 0.0]])
    assert engine._breach_stats(touching)[2] == 0.0
    worst, wall_distance, contact_force = engine._breach_stats(
        np.asarray([center + [touching_offset + 0.0001, 0.0, 0.0]])
    )

    assert worst == pytest.approx(0.0001)
    assert wall_distance == 0.0
    assert contact_force == pytest.approx(0.0001 * engine.contact_ke)


class TestFeedBudget:
    """Pure prolapse-guard budget function."""

    def test_no_slack_passes_full_advance(self):
        assert _feed_budget(0.002, 0.0, 0.012) == 0.002

    def test_slack_at_limit_blocks_feed(self):
        assert _feed_budget(0.002, 0.012, 0.012) == 0.0

    def test_slack_beyond_limit_blocks_feed(self):
        # A buckled tip can project *behind* the fed arclen -- still blocked.
        assert _feed_budget(0.002, 0.05, 0.012) == 0.0

    def test_partial_budget_near_limit(self):
        assert _feed_budget(0.002, 0.011, 0.012) == pytest.approx(0.001)

    def test_backward_feed_never_limited(self):
        # Pull-back is the recovery move; the guard must not block it.
        assert _feed_budget(-0.002, 0.05, 0.012) == -0.002

    def test_guard_disabled_by_nonpositive_max_slack(self):
        assert _feed_budget(0.002, 0.05, 0.0) == 0.002


class TestSheathAutoMode:
    """sheath_bodies=-1 resolves to 'all but the distal free_len'."""

    def test_auto_default_is_sentinel(self, engine):
        assert engine._sheath_bodies == -1

    def test_auto_leaves_free_len_free(self, engine):
        # 20 bodies x 3mm, free_len=30mm -> 10 free, 10 glued.
        alpha = engine._glue_alpha(20)
        assert engine._sheath_count(20) == 10
        assert (alpha[:10] == 1.0).all()
        assert (alpha[10:] == 0.0).all()

    def test_explicit_sheath_bodies_wins(self, engine):
        # sheath_bodies=1 reproduces the original root-only D4 glue.
        engine._sheath_bodies = 1
        alpha = engine._glue_alpha(20)
        assert alpha[0] == 1.0
        assert (alpha[1:] == 0.0).all()

    def test_sheath_count_clamped_to_at_least_one(self, engine):
        engine._free_len = 10.0  # absurdly long free span
        assert engine._sheath_count(20) == 1

    def test_sheath_count_clamped_to_body_count(self, engine):
        engine._sheath_bodies = 999
        assert engine._sheath_count(20) == 20

    def test_anchor_mode_ramp_unchanged(self, monkeypatch):
        # D3 anchor glue mask must be byte-for-byte the pre-refactor inline loop
        # (legacy fallback -- preserved as-is, quirks included).
        monkeypatch.setenv("CATHSIM_NEWTON_DRIVE", "anchor")
        anchor = NewtonEngine(path=_straight_path())
        nb = 20
        expected = np.ones(nb)
        for i in range(anchor._free_span):
            j = nb - 1 - i
            if j >= 0:
                frac = (i + 1) / anchor._free_span
                expected[j] = max(
                    anchor._tip_alpha, 1.0 - frac * (1.0 - anchor._tip_alpha)
                )
        assert anchor._glue_alpha(nb) == pytest.approx(expected)


class TestLiveTuning:
    """New params flow through the engine_params channel."""

    def test_deform_params_exposes_antibuckle_params(self, engine):
        params = engine.deform_params
        assert params["tip_shape"] == "j_tip"
        assert params["sheath_bodies"] == -1
        assert params["free_len"] == pytest.approx(0.03)
        assert params["max_slack"] == pytest.approx(0.012)

    def test_diagnostics_available_before_init(self, engine):
        diag = engine.diagnostics()

        assert diag["initialized"] is False
        assert diag["drive"] == "force"
        assert diag["free_len_m"] == pytest.approx(0.03)
        assert diag["max_slack_m"] == pytest.approx(0.012)
        assert diag["sheath_bodies"] == -1
        assert diag["wall_contact_count"] == 0
        assert diag["max_penetration_m"] == 0.0
        assert diag["contact_impulse"] == 0.0
        assert "slack_m" not in diag

    def test_mechanics_state_exposes_clinical_contract_before_init(self, engine):
        state = engine.mechanics_state()

        assert state["source"] == "newton_engine.mechanics_state"
        assert state["guidewire"]["shape_type"] == "j_tip"
        assert state["guidewire"]["curve_angle_deg"] == pytest.approx(35.0)
        assert state["support"]["support_state"] == "modeled"
        assert state["support"]["effective_support_type"] == "proximal_sheath"
        assert state["support"]["free_wire_length_m"] == pytest.approx(0.03, abs=0.003)
        assert state["support"]["support_ratio"] > 0.0
        assert state["risk"]["buckling_risk"] == "unknown"
        assert state["risk"]["wall_contact_count"] == 0
        assert state["risk"]["max_penetration_m"] is None
        assert state["risk"]["contact_impulse"] == 0.0
        assert state["risk"]["normal_poking_score"] is None
        assert state["risk"]["tangential_slide_score"] is None

    def test_wall_slide_metrics_detect_tangential_slide(self, engine):
        xyz = np.array([
            [0.00255, 0.0, 0.090],
            [0.00255, 0.0, 0.100],
            [0.00255, 0.0, 0.110],
        ])

        metrics = engine._wall_slide_metrics(xyz)

        assert metrics["tangential_slide_score"] == pytest.approx(1.0)
        assert metrics["normal_poking_score"] == pytest.approx(0.0)
        assert metrics["wall_contact_body_index"] == 0
        assert metrics["wall_normal"][0] == pytest.approx(1.0)

    def test_wall_slide_metrics_detect_normal_poking(self, engine):
        xyz = np.array([
            [0.0018, 0.0, 0.100],
            [0.0024, 0.0, 0.100],
            [0.0030, 0.0, 0.100],
        ])

        metrics = engine._wall_slide_metrics(xyz)

        assert metrics["normal_poking_score"] > 0.95
        assert metrics["tangential_slide_score"] < 0.1

    def test_tip_orientation_metrics_reports_distal_angle_and_lag(self, engine):
        engine._twist = np.deg2rad(45.0)
        xyz = np.array([
            [0.0010, 0.0, 0.090],
            [0.0020, 0.0, 0.100],
            [0.0030, 0.0, 0.100],
        ])

        metrics = engine._tip_orientation_metrics(xyz)

        assert metrics["distal_tip_rotation_deg"] == pytest.approx(0.0, abs=1e-3)
        assert metrics["torsion_lag_deg"] == pytest.approx(45.0, abs=1e-3)
        assert metrics["tip_deflection_score"] > 0.5

    def test_support_control_advances_and_retracts_free_span(self, engine):
        start = engine.mechanics_state()["support"]["free_wire_length_m"]

        advanced = engine.apply_support_control(1.0)["support"]["free_wire_length_m"]
        retracted = engine.apply_support_control(-1.0)["support"]["free_wire_length_m"]

        assert advanced < start
        assert retracted > advanced

    def test_set_deform_params_updates_before_init(self, engine):
        # Not initialized (no newton locally): values update, no rebuild/crash.
        effective = engine.set_deform_params(
            sheath_bodies=5, free_len=0.05, max_slack=0.02, jtip_deg=0.0, jtip_bodies=0
        )
        assert effective["sheath_bodies"] == 5
        assert effective["free_len"] == pytest.approx(0.05)
        assert effective["max_slack"] == pytest.approx(0.02)
        assert effective["tip_shape"] == "straight"
        assert engine._sheath_count(20) == 5


class TestSegmentedStiffness:
    """D4-R soft-tip / hard-shaft bend stiffness profile."""

    def test_bend_profile_keeps_proximal_hard_and_distal_soft(self, engine):
        engine._bend = 20.0
        engine._tip_bend = 2.0
        engine._soft_tip = 4

        profile = engine._bend_profile(10)

        assert profile[:6].tolist() == pytest.approx([20.0] * 6)
        assert profile[6:].tolist() == pytest.approx([15.5, 11.0, 6.5, 2.0])

    def test_bend_profile_short_rod_still_reaches_tip_bend(self, engine):
        engine._bend = 20.0
        engine._tip_bend = 2.0
        engine._soft_tip = 10

        profile = engine._bend_profile(3)

        assert profile.tolist() == pytest.approx([14.0, 8.0, 2.0])

    def test_deform_params_exposes_bend_profile_preview(self, engine):
        params = engine.deform_params

        assert "bend_profile" in params
        assert params["bend_profile"][-1] == pytest.approx(engine._tip_bend)

    def test_protocol_parameters_exposes_effective_solver_contract(self, engine):
        params = engine.protocol_parameters

        assert params["rod_length"] == pytest.approx(engine._rod_length)
        assert params["rod_segment_length"] == pytest.approx(engine._rod_seg_len)
        assert params["control_timestep"] == pytest.approx(engine._control_dt)
        assert params["substeps"] == engine._substeps
        assert params["solver_iterations"] == engine._iterations

class TestDeviceDiagnostics:
    """Segmented guidewire and support metadata exposed by Newton diagnostics."""

    def test_diagnostics_exposes_guidewire_profile_and_support(self, engine):
        diag = engine.diagnostics()

        assert diag["guidewire"]["profile_name"] == "soft_j_tip_training_wire"
        assert diag["guidewire"]["tip_shape"] == "j_tip"
        assert diag["guidewire"]["current_tip_segment"] == "pre_shaped_soft_tip"
        assert diag["guidewire"]["segment_count"] == 6
        assert diag["support"]["effective_support_type"] == "microcatheter"
        assert diag["support"]["free_wire_length_mm"] == pytest.approx(30.0)

    def test_profile_supplies_newton_jtip_default(self, engine):
        assert engine._jtip_deg == pytest.approx(engine.guidewire_profile.tip_shape.precurve_angle_deg)
        assert engine._jtip_bodies >= 1

    def test_body_metadata_marks_segments_and_support_state(self, engine):
        engine._alpha = np.asarray([1.0, 1.0, 0.0, 0.0])

        root = engine._body_segment_metadata(0, 4)
        tip = engine._body_segment_metadata(3, 4)

        assert root["material_segment"] == "proximal_control"
        assert root["support_state"] == "inside_support_tube"
        assert tip["material_segment"] == "atraumatic_cap"
        assert tip["support_state"] == "distal_free_span"


class TestGuidewireRiskMetrics:
    """Geometry-derived guidewire risk metrics without importing Newton."""

    def test_tip_wall_metrics_detects_tangential_wall_slide(self, engine):
        xyz = np.asarray([
            [0.0022, 0.0, 0.045],
            [0.0022, 0.0, 0.050],
        ])

        metrics = engine._tip_wall_metrics(xyz, 0.05)

        assert metrics["normal_poking_score"] < 0.2
        assert metrics["tangential_slide_score"] > 0.35
        assert metrics["wall_slide_state"] == "WALL_SLIDE_OK"

    def test_tip_wall_metrics_detects_normal_poking(self, engine):
        xyz = np.asarray([
            [0.0015, 0.0, 0.050],
            [0.0026, 0.0, 0.050],
        ])

        metrics = engine._tip_wall_metrics(xyz, 0.05)

        assert metrics["normal_poking_score"] >= 0.7
        assert metrics["wall_slide_state"] == "TIP_POKING_WARNING"

    def test_torsion_lag_tracks_jtip_plane_response(self, engine):
        engine._twist = np.deg2rad(90.0)
        engine._jtip_bodies = 2
        xyz = np.asarray([
            [0.0, 0.0, 0.042],
            [0.0, 0.0, 0.045],
            [-0.002, 0.0, 0.048],
        ])

        assert engine._torsion_lag_deg(xyz, 0.048) == pytest.approx(0.0, abs=1e-6)
