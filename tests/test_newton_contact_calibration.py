from scripts.run_newton_contact_calibration import _validate


def _case(
    penetration_mm: float,
    *,
    contacts: int,
    proxy_force: float,
    solver_force: float,
) -> dict:
    return {
        "requested_penetration_mm": penetration_mm,
        "measured_penetration_mm": max(0.0, penetration_mm),
        "wall_contact_count": contacts,
        "penetration_force_proxy_n": proxy_force,
        "solver_contact_force_sum_n": solver_force,
    }


def test_calibration_accepts_geometric_deep_breach_with_explicit_warning():
    cases = [
        _case(-0.5, contacts=0, proxy_force=0.0, solver_force=0.0),
        _case(0.0, contacts=2, proxy_force=0.0, solver_force=1.0),
        _case(0.05, contacts=2, proxy_force=150.0, solver_force=2.0),
        _case(0.1, contacts=2, proxy_force=300.0, solver_force=3.0),
        _case(0.3, contacts=0, proxy_force=900.0, solver_force=0.0),
    ]

    checks = _validate(cases, rod_radius_mm=0.27)

    assert checks["passed"] is True
    assert checks["deep_penetration_detected_geometrically"] is True
    assert checks["deep_penetration_manifold_is_continuous"] is False


def test_calibration_rejects_missing_shallow_contact_force():
    cases = [
        _case(-0.5, contacts=0, proxy_force=0.0, solver_force=0.0),
        _case(0.0, contacts=2, proxy_force=0.0, solver_force=1.0),
        _case(0.05, contacts=0, proxy_force=150.0, solver_force=0.0),
        _case(0.3, contacts=0, proxy_force=900.0, solver_force=0.0),
    ]

    checks = _validate(cases, rod_radius_mm=0.27)

    assert checks["passed"] is False
    assert checks["shallow_penetration_generates_contact"] is False
    assert checks["solver_force_is_available_for_shallow_contact"] is False
