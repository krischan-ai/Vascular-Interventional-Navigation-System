from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from typing import Any

import numpy as np

from services.physics.base import PlannedPath
from services.physics.newton_engine import (
    NewtonEngine,
    _count_static_dynamic_contacts,
    _point_at_s,
    _static_dynamic_force_stats,
)


DEFAULT_PENETRATIONS_MM = (-0.5, 0.0, 0.05, 0.1, 0.3)


def _penetration_values(value: str) -> list[float]:
    try:
        values = [float(item.strip()) for item in value.split(",")]
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            "penetrations must be comma-separated millimeter values"
        ) from exc
    if not values:
        raise argparse.ArgumentTypeError("at least one penetration is required")
    return values


def _straight_path(length_m: float, radius_m: float) -> PlannedPath:
    points = np.zeros((51, 3), dtype=np.float64)
    points[:, 2] = np.linspace(0.0, length_m, len(points))
    return PlannedPath(points, radii=np.full(len(points), radius_m))


def _configure_reproducible_probe(contact_ke: float, rod_radius_m: float) -> None:
    os.environ["CATHSIM_NEWTON_DRIVE"] = "force"
    os.environ["CATHSIM_NEWTON_SETTLE_STEPS"] = "0"
    os.environ["CATHSIM_NEWTON_JTIP_DEG"] = "0"
    os.environ["CATHSIM_NEWTON_JTIP_BODIES"] = "0"
    os.environ["CATHSIM_NEWTON_CONTACT_KE"] = str(contact_ke)
    os.environ["CATHSIM_NEWTON_ROD_RADIUS"] = str(rod_radius_m)


def _run_case(
    penetration_m: float,
    *,
    lumen_radius_m: float,
    rod_radius_m: float,
    contact_ke: float,
) -> dict[str, Any]:
    import warp as wp

    engine = NewtonEngine(
        path=_straight_path(0.1, lumen_radius_m),
        n_substeps=6,
        rod_length=0.012,
        rod_seg_len=0.003,
        free_len=0.006,
        max_slack=0.006,
        insertion_margin=0.0,
    )
    try:
        engine.reset()
        body = engine._rod_bodies[-1]
        arclen = float(engine._base_arc[-1])
        center = _point_at_s(engine._centerline, arclen)
        local_radius = float(np.interp(arclen, engine._cl_cum, engine._radii))

        body_q = engine._s0.body_q.numpy()
        body_q[body, :3] = center + np.asarray([
            local_radius - rod_radius_m + penetration_m,
            0.0,
            0.0,
        ])
        engine._s0.body_q.assign(body_q)
        engine._s0.body_qd.zero_()
        # The probe teleports a body. Synchronize VBD's previous pose so the
        # measured force is contact response, not an artificial inertial spike.
        engine._solver.body_q_prev.assign(engine._s0.body_q)

        engine._s0.clear_forces()
        engine._model.collide(engine._s0, engine._contacts)
        contact_count = int(engine._contacts.rigid_contact_count.numpy()[0])
        shape0 = engine._contacts.rigid_contact_shape0.numpy()[:contact_count]
        shape1 = engine._contacts.rigid_contact_shape1.numpy()[:contact_count]
        wall_contact_count = _count_static_dynamic_contacts(
            shape0,
            shape1,
            engine._shape_body_np,
        )
        pre_breach, pre_clearance, proxy_force = engine._breach_stats(
            body_q[engine._rod_bodies, :3]
        )

        previous_pose = wp.clone(engine._solver.body_q_prev)
        engine._solver.step(
            engine._s0,
            engine._s1,
            engine._control,
            engine._contacts,
            engine._sim_dt,
        )
        if not hasattr(engine._solver, "collect_rigid_contact_forces"):
            raise RuntimeError("installed SolverVBD does not expose contact-force collection")
        body0, body1, _, _, forces, force_count = (
            engine._solver.collect_rigid_contact_forces(
                engine._s1.body_q,
                previous_pose,
                engine._contacts,
                engine._sim_dt,
            )
        )
        active_force_count = int(force_count.numpy()[0])
        solver_contact_count, peak_force, force_sum, resultant_force = (
            _static_dynamic_force_stats(
                body0.numpy()[:active_force_count],
                body1.numpy()[:active_force_count],
                forces.numpy()[:active_force_count],
            )
        )
        post_xyz = engine._s1.body_q.numpy()[engine._rod_bodies, :3]
        post_breach, post_clearance, _ = engine._breach_stats(post_xyz)

        return {
            "requested_penetration_mm": penetration_m * 1000.0,
            "measured_penetration_mm": max(0.0, pre_breach) * 1000.0,
            "pre_solver_clearance_mm": pre_clearance * 1000.0,
            "post_solver_penetration_mm": max(0.0, post_breach) * 1000.0,
            "post_solver_clearance_mm": post_clearance * 1000.0,
            "wall_contact_count": wall_contact_count,
            "solver_contact_count": solver_contact_count,
            "penetration_force_proxy_n": proxy_force,
            "solver_peak_contact_force_n": peak_force,
            "solver_contact_force_sum_n": force_sum,
            "solver_contact_resultant_n": resultant_force,
            "solver_contact_impulse_n_s": force_sum * engine._sim_dt,
            "solver_timestep_ms": engine._sim_dt * 1000.0,
        }
    finally:
        engine.close()


def _nondecreasing(values: list[float], tolerance: float = 1e-8) -> bool:
    return all(
        current + tolerance >= previous
        for previous, current in zip(values, values[1:])
    )


def _validate(
    cases: list[dict[str, Any]],
    *,
    rod_radius_mm: float,
) -> dict[str, bool]:
    clear = [case for case in cases if case["requested_penetration_mm"] < 0.0]
    touching = [
        case
        for case in cases
        if abs(case["requested_penetration_mm"]) <= 1e-12
    ]
    shallow_penetrating = [
        case
        for case in cases
        if 0.0 < case["requested_penetration_mm"] < rod_radius_mm
    ]
    deep_penetrating = [
        case
        for case in cases
        if case["requested_penetration_mm"] >= rod_radius_mm
    ]
    ordered = sorted(cases, key=lambda case: case["requested_penetration_mm"])
    proxy_values = [
        float(case["penetration_force_proxy_n"]) for case in ordered
    ]
    solver_values = [
        float(case["solver_contact_force_sum_n"]) for case in ordered
    ]
    checks = {
        "requested_geometry_matches": all(
            abs(
                float(case["measured_penetration_mm"])
                - max(0.0, float(case["requested_penetration_mm"]))
            )
            <= 1e-4
            for case in cases
        ),
        "clearance_has_no_wall_contacts": bool(clear)
        and all(int(case["wall_contact_count"]) == 0 for case in clear),
        "touching_generates_contact": bool(touching)
        and all(int(case["wall_contact_count"]) > 0 for case in touching),
        "shallow_penetration_generates_contact": bool(shallow_penetrating)
        and all(
            int(case["wall_contact_count"]) > 0
            for case in shallow_penetrating
        ),
        "deep_penetration_detected_geometrically": bool(deep_penetrating)
        and all(
            float(case["measured_penetration_mm"]) > 0.0
            and float(case["penetration_force_proxy_n"]) > 0.0
            for case in deep_penetrating
        ),
        "proxy_force_is_monotonic": _nondecreasing(proxy_values),
        "solver_force_is_available_for_shallow_contact": bool(shallow_penetrating)
        and all(
            float(case["solver_contact_force_sum_n"]) > 0.0
            for case in shallow_penetrating
        ),
        # Characterization only: hard-contact forces can depend on constraints
        # and manifold changes, so this is reported but not yet a release gate.
        "solver_force_is_monotonic": _nondecreasing(solver_values),
        "deep_penetration_manifold_is_continuous": bool(deep_penetrating)
        and all(
            int(case["wall_contact_count"]) > 0
            for case in deep_penetrating
        ),
        "critical_breach_case_present": any(
            float(case["measured_penetration_mm"]) >= 0.3 - 1e-4
            for case in cases
        ),
    }
    release_gates = {
        key: value
        for key, value in checks.items()
        if key not in {
            "solver_force_is_monotonic",
            "deep_penetration_manifold_is_continuous",
        }
    }
    checks["passed"] = all(release_gates.values())
    return checks


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Run a controlled Newton/VBD wall-contact calibration probe."
    )
    parser.add_argument(
        "--penetrations-mm",
        type=_penetration_values,
        default=list(DEFAULT_PENETRATIONS_MM),
        help="Comma-separated signed wall penetrations in millimeters",
    )
    parser.add_argument("--lumen-radius-mm", type=float, default=3.0)
    parser.add_argument("--rod-radius-mm", type=float, default=0.27)
    parser.add_argument("--contact-ke", type=float, default=3_000_000.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)

    if args.lumen_radius_mm <= args.rod_radius_mm or args.rod_radius_mm <= 0.0:
        parser.error("lumen radius must exceed a positive rod radius")
    if args.contact_ke <= 0.0:
        parser.error("contact-ke must be positive")

    rod_radius_m = args.rod_radius_mm / 1000.0
    lumen_radius_m = args.lumen_radius_mm / 1000.0
    _configure_reproducible_probe(args.contact_ke, rod_radius_m)
    cases = [
        _run_case(
            penetration_mm / 1000.0,
            lumen_radius_m=lumen_radius_m,
            rod_radius_m=rod_radius_m,
            contact_ke=args.contact_ke,
        )
        for penetration_mm in args.penetrations_mm
    ]
    result = {
        "status": "passed",
        "probe": "newton_vbd_controlled_wall_contact_v1",
        "parameters": {
            "penetrations_mm": args.penetrations_mm,
            "lumen_radius_mm": args.lumen_radius_mm,
            "rod_radius_mm": args.rod_radius_mm,
            "contact_ke_n_per_m": args.contact_ke,
        },
        "metric_semantics": {
            "wall_contact_count": "Newton collision manifold pairs before VBD resolution",
            "penetration_force_proxy_n": "geometric penetration times contact_ke",
            "solver_contact_force_sum_n": "sum of VBD per-contact force magnitudes",
            "solver_contact_resultant_n": "resultant VBD force on dynamic guidewire bodies",
        },
        "limitations": [
            "VBD contact force includes guidewire constraint response and is not a clinically calibrated wall force.",
            "VBD force is not required to be monotonic across teleported penetration cases.",
            "Once penetration exceeds the rod radius, the capsule can pass fully beyond the lumen surface and lose its collision manifold; geometric breach remains the safety fallback.",
        ],
        "cases": cases,
        "checks": _validate(cases, rod_radius_mm=args.rod_radius_mm),
    }
    warning_checks = (
        "solver_force_is_monotonic",
        "deep_penetration_manifold_is_continuous",
    )
    has_warning = not all(result["checks"][name] for name in warning_checks)
    if result["checks"]["passed"]:
        result["status"] = "passed_with_warnings" if has_warning else "passed"
    else:
        result["status"] = "failed"
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0 if result["checks"]["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
