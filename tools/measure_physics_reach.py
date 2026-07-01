"""Measure how far the *physical* (non-guided) guidewire advances in a phantom.

This is a diagnostic harness for the physics-navigation work (steps 1-2 of the
"real guidewire physics" plan): it builds a NavigationEngine in physical mode,
feeds a constant forward push, and reports how far the tip travels along the
phantom's shipped centerline (path_progress) plus tip/target distance.

Usage:
    python -m tools.measure_physics_reach --phantom segment_part --target root \
        --insertion-max 0.7 --n-bodies 160 --steps 400
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

# dm_control imports a renderer even with pixels off; glfw is the Windows backend.
os.environ.setdefault("MUJOCO_GL", "glfw")

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phantom", default="segment_part")
    parser.add_argument("--target", default="root")
    parser.add_argument("--insertion-max", type=float, default=0.2)
    parser.add_argument("--stiffness-scale", type=float, default=1.0)
    parser.add_argument("--n-bodies", type=int, default=80)
    parser.add_argument("--n-substeps", type=int, default=None)
    parser.add_argument("--steps", type=int, default=400)
    parser.add_argument("--push", type=float, default=1.0)
    parser.add_argument(
        "--autopilot",
        action="store_true",
        help="Use the closed-loop PhysicsAutopilot instead of constant push.",
    )
    parser.add_argument("--base-push", type=float, default=0.35)
    parser.add_argument("--lookahead", type=float, default=0.025)
    parser.add_argument(
        "--prethread",
        action="store_true",
        help="Bend the physical wire onto the centerline at spawn.",
    )
    parser.add_argument(
        "--settle",
        type=int,
        default=0,
        help="Physics steps with zero action after reset (let the wire settle).",
    )
    parser.add_argument("--report-every", type=int, default=50)
    args = parser.parse_args()

    from services.navigation_engine import NavigationEngine

    print(
        f"[build] phantom={args.phantom} target={args.target} "
        f"insertion_max={args.insertion_max} stiffness_scale={args.stiffness_scale} "
        f"n_bodies={args.n_bodies} n_substeps={args.n_substeps}"
    )
    engine = NavigationEngine(
        phantom=args.phantom,
        target=args.target,
        guided=False,
        n_bodies=args.n_bodies,
        n_substeps=args.n_substeps,
        insertion_max=args.insertion_max,
        stiffness_scale=args.stiffness_scale,
        prethread=args.prethread,
    )

    t0 = time.perf_counter()
    state = engine.reset()
    t_reset = time.perf_counter() - t0
    path_len = engine._path.total_len if engine._path is not None else 0.0
    print(f"[reset] ok in {t_reset:.2f}s | planned_path_len={path_len:.3f}m")
    print(
        f"[reset] tip={_fmt(state.tip_position)} target={_fmt(state.target_position)} "
        f"progress={state.path_progress:.3f} dev={state.path_deviation*1000:.1f}mm"
    )

    for j in range(args.settle):
        state = engine.step(delta_push=0.0, delta_rotate=0.0)
    if args.settle:
        print(
            f"[settle {args.settle}] progress={state.path_progress:.3f} "
            f"dev={state.path_deviation*1000:.1f}mm "
            f"tip->tgt={_dist(state.tip_position, state.target_position)*1000:.1f}mm "
            f"force={state.contact_force:.3f}"
        )

    autopilot = None
    if args.autopilot:
        from services.physics_autopilot import AutopilotConfig, PhysicsAutopilot

        cfg = AutopilotConfig(base_push=args.base_push, lookahead_m=args.lookahead)
        autopilot = PhysicsAutopilot(engine.planned_path, config=cfg)
        autopilot.reset()
        print(f"[autopilot] base_push={cfg.base_push} lookahead={cfg.lookahead_m}m")

    max_progress = state.path_progress
    min_tip_target = _dist(state.tip_position, state.target_position)
    t1 = time.perf_counter()
    for i in range(1, args.steps + 1):
        if autopilot is not None:
            push, rotate = autopilot.compute(
                state.tip_position, state.tip_direction, state.contact_force
            )
        else:
            push, rotate = args.push, 0.0
        state = engine.step(delta_push=push, delta_rotate=rotate)
        max_progress = max(max_progress, state.path_progress)
        min_tip_target = min(min_tip_target, _dist(state.tip_position, state.target_position))
        if i % args.report_every == 0 or i == args.steps:
            print(
                f"[step {i:4d}] progress={state.path_progress:.3f} "
                f"(max {max_progress:.3f}) dev={state.path_deviation*1000:5.1f}mm "
                f"tip->tgt={_dist(state.tip_position, state.target_position)*1000:6.1f}mm "
                f"force={state.contact_force:7.3f} safety={state.safety_status}"
            )
    dt = time.perf_counter() - t1

    print("-" * 60)
    print(
        f"[result] max_progress={max_progress:.3f} "
        f"({max_progress*path_len*1000:.0f}mm of {path_len*1000:.0f}mm) | "
        f"min_tip->target={min_tip_target*1000:.1f}mm | "
        f"{args.steps} steps in {dt:.1f}s ({args.steps/dt:.0f} step/s)"
    )
    engine.close()
    return 0


def _dist(a, b) -> float:
    return float(np.linalg.norm(np.asarray(a) - np.asarray(b)))


def _fmt(v) -> str:
    return "[" + ", ".join(f"{x:.3f}" for x in v) + "]"


if __name__ == "__main__":
    raise SystemExit(main())
