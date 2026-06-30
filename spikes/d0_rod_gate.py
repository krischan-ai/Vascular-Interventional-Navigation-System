"""D0 gate: can Newton model a sub-mm elastic guidewire rod at 60 Hz?

doc/08 §3.3 gate — answer two questions with the smallest possible script:

  1. Does Newton have a thin elastic-rod primitive with a *bending* stiffness
     knob, and is that knob well-behaved (monotonic, stable)?
  2. Can a single cathsim-scale rod (~0.25 m, sub-mm radius) step in real time
     (>= 60 control-Hz) on the A6000?

Newton 1.3.0 answers #1 structurally: ``ModelBuilder.add_rod`` builds a chain of
capsule bodies joined by *cable joints* (1 stretch DOF + 1 bend/twist DOF), i.e.
a discrete Cosserat rod. ``bend_stiffness`` is the per-joint torque-per-radian
[N*m/rad] — exactly the calibration target for cathsim's joint ``stiffness_scale``
(doc/08 §1.3). This script exercises that primitive headless and prints a verdict.

Run (server, cathsim-newton env):
    python d0_rod_gate.py
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np
import warp as wp

import newton

# --- cathsim-scale rod defaults -------------------------------------------------
# aorta_trunk guidewire ~0.256 m; segment_part ~0.58 m. Radius is sub-mm.
L_DEFAULT = 0.25          # rod length [m]
RADIUS_DEFAULT = 4.0e-4   # capsule radius [m] = 0.4 mm (sub-mm, the *hard* case)
N_SEG_DEFAULT = 64        # segments (~4 mm each, near cathsim node density)
DENSITY = 1.8e3           # ~ nitinol/polymer guidewire, kg/m^3


def build_model(
    *,
    solver_name: str,
    bend_stiffness: float,
    n_seg: int = N_SEG_DEFAULT,
    length: float = L_DEFAULT,
    radius: float = RADIUS_DEFAULT,
    anchor_root: bool = False,
    start_z: float = 0.30,
    horizontal: bool = True,
    add_ground: bool = True,
    bend_damping: float = 1.0e-1,
    stretch_stiffness: float = 1.0e5,
    stretch_damping: float = 0.0,  # MUST stay 0; stretch_damping>0 unravels the rod (see d0 notes)
):
    """Build a single thin rod and return (model, solver, states, control, contacts, rod_bodies)."""
    builder = newton.ModelBuilder()
    builder.rigid_gap = 0.0
    builder.default_shape_cfg.density = DENSITY
    builder.default_shape_cfg.mu = 0.3

    if add_ground:
        ground_cfg = newton.ModelBuilder.ShapeConfig(mu=1.0)
        builder.add_ground_plane(cfg=ground_cfg)

    direction = wp.vec3(1.0, 0.0, 0.0) if horizontal else wp.vec3(0.0, 0.0, -1.0)
    start = wp.vec3(0.0, 0.0, start_z)
    pts = newton.utils.create_straight_cable_points(
        start=start, direction=direction, length=length, num_segments=n_seg
    )
    quats = newton.utils.create_parallel_transport_cable_quaternions(pts, twist_total=0.0)

    rod_cfg = newton.ModelBuilder.ShapeConfig(density=DENSITY, mu=0.3)
    rod_bodies, _rod_joints = builder.add_rod(
        positions=pts,
        quaternions=quats,
        radius=radius,
        cfg=rod_cfg,
        stretch_stiffness=stretch_stiffness,
        stretch_damping=stretch_damping,
        bend_stiffness=bend_stiffness,
        bend_damping=bend_damping,
        label="guidewire",
    )

    if anchor_root:
        # Pin the proximal end. A body is truly held kinematic only when mass,
        # inverse mass, inertia and inverse inertia are all zeroed (matches the
        # newton cable Y-junction example); zeroing mass alone lets it fly off.
        rb0 = rod_bodies[0]
        builder.body_mass[rb0] = 0.0
        builder.body_inv_mass[rb0] = 0.0
        builder.body_inertia[rb0] = wp.mat33(0.0)
        builder.body_inv_inertia[rb0] = wp.mat33(0.0)

    builder.color(balance_colors=False)
    model = builder.finalize()
    model.set_gravity((0.0, 0.0, -9.81))

    if solver_name == "xpbd":
        solver = newton.solvers.SolverXPBD(model, iterations=5)
    elif solver_name == "vbd":
        solver = newton.solvers.SolverVBD(model, iterations=5)
    else:
        raise ValueError(solver_name)

    state_0 = model.state()
    state_1 = model.state()
    control = model.control()
    contacts = model.contacts()
    return model, solver, state_0, state_1, control, contacts, rod_bodies


def make_stepper(model, solver, state_0, state_1, control, contacts, sim_dt, substeps):
    """Return a `frame()` callable advancing one control frame, CUDA-graph captured if possible."""
    states = [state_0, state_1]

    def simulate():
        a, b = states[0], states[1]
        for _ in range(substeps):
            a.clear_forces()
            model.collide(a, contacts)
            solver.step(a, b, control, contacts, sim_dt)
            a, b = b, a
        states[0], states[1] = a, b

    graph = None
    if wp.get_device().is_cuda:
        try:
            with wp.ScopedCapture() as cap:
                simulate()
            graph = cap.graph
        except Exception as e:  # graph capture can reject dynamic shapes
            print(f"  [graph capture unavailable: {e}]")
            graph = None

    def frame():
        if graph is not None:
            wp.capture_launch(graph)
        else:
            simulate()

    return frame, states


def tip_state(state, rod_bodies):
    """Return (root_xyz, tip_xyz) of the rod from body transforms."""
    q = state.body_q.numpy()
    root = q[rod_bodies[0], :3]
    tip = q[rod_bodies[-1], :3]
    return root, tip


# --- gate sub-tests -------------------------------------------------------------

def test_throughput(solver_name, fps=60, substeps=10, frames=600):
    """Measure wall-clock control-frames/sec for a free-dropping rod."""
    sim_dt = (1.0 / fps) / substeps
    model, solver, s0, s1, control, contacts, rod_bodies = build_model(
        solver_name=solver_name, bend_stiffness=1.0e1, start_z=0.30
    )
    frame, states = make_stepper(model, solver, s0, s1, control, contacts, sim_dt, substeps)

    # warmup
    for _ in range(10):
        frame()
    wp.synchronize()

    t0 = time.perf_counter()
    for _ in range(frames):
        frame()
    wp.synchronize()
    dt = time.perf_counter() - t0

    fps_actual = frames / dt
    phys_sps = fps_actual * substeps
    q = states[0].body_q.numpy()
    qd = states[0].body_qd.numpy()
    finite = bool(np.isfinite(q).all() and np.isfinite(qd).all())
    print(
        f"  [{solver_name}] {frames} frames x {substeps} substeps in {dt:.3f}s -> "
        f"{fps_actual:8.1f} control-fps  ({phys_sps:8.0f} phys-steps/s)  finite={finite}"
    )
    return fps_actual, finite


def test_cantilever(solver_name, bend_values, fps=60, substeps=10, settle_frames=600):
    """Anchor the root, let the horizontal rod settle, report tip droop per bend stiffness."""
    sim_dt = (1.0 / fps) / substeps
    print(f"  [{solver_name}] cantilever tip droop vs bend_stiffness (rod L={L_DEFAULT} m):")
    droops = []
    for bs in bend_values:
        model, solver, s0, s1, control, contacts, rod_bodies = build_model(
            solver_name=solver_name, bend_stiffness=bs, anchor_root=True,
            start_z=0.30, add_ground=False,  # free space: isolate bending from contact
        )
        frame, states = make_stepper(model, solver, s0, s1, control, contacts, sim_dt, substeps)
        root0, tip0 = tip_state(states[0], rod_bodies)
        for _ in range(settle_frames):
            frame()
        wp.synchronize()
        root1, tip1 = tip_state(states[0], rod_bodies)
        droop = float(root1[2] - tip1[2])          # how far the free tip fell below the root [m]
        horiz = float(np.linalg.norm((tip1 - root1)[:2]))  # retained horizontal reach [m]
        q = states[0].body_q.numpy()[:, :3]
        seg = np.array([np.linalg.norm(q[rod_bodies[i + 1]] - q[rod_bodies[i]])
                        for i in range(len(rod_bodies) - 1)])
        stretch_pct = (seg.mean() / (L_DEFAULT / N_SEG_DEFAULT) - 1.0) * 100.0
        finite = bool(np.isfinite(states[0].body_q.numpy()).all())
        droops.append(droop)
        print(
            f"      bend={bs:9.1e} N*m/rad -> tip droop={droop*1e3:8.2f} mm   "
            f"horiz reach={horiz*1e3:7.1f} mm   stretch={stretch_pct:+5.1f}%  finite={finite}"
        )
    # Two things must hold for the bend knob to be *usable* as a calibration target:
    #  (1) monotonic: a stiffer rod droops no more than a softer one (within noise);
    #  (2) responsive: the knob actually moves the rod across a meaningful range,
    #      i.e. it is not saturated floppy/rigid for the whole sweep.
    monotonic = all(droops[i] >= droops[i + 1] - 5e-3 for i in range(len(droops) - 1))
    span_mm = (max(droops) - min(droops)) * 1e3
    responsive = span_mm > 20.0
    print(f"      -> monotonic={monotonic}  droop span={span_mm:.1f} mm  responsive(>20mm)={responsive}")
    return droops, (monotonic and responsive)


def test_drop_stability(solver_name, fps=60, substeps=10, frames=600):
    """Drop the rod onto the ground; verify it settles finite and does not tunnel through."""
    sim_dt = (1.0 / fps) / substeps
    model, solver, s0, s1, control, contacts, rod_bodies = build_model(
        solver_name=solver_name, bend_stiffness=5.0e-3, start_z=0.05,  # start low, drop onto z=0
    )
    frame, states = make_stepper(model, solver, s0, s1, control, contacts, sim_dt, substeps)
    for _ in range(frames):
        frame()
    wp.synchronize()
    q = states[0].body_q.numpy()
    qd = states[0].body_qd.numpy()
    finite = bool(np.isfinite(q).all() and np.isfinite(qd).all())
    min_z = float(q[rod_bodies, 2].min())
    max_speed = float(np.abs(qd).max())
    # tunneling tolerance ~ 2x radius below ground
    no_tunnel = min_z > -2.0 * RADIUS_DEFAULT - 1e-3
    settled = max_speed < 5.0
    print(
        f"  [{solver_name}] drop-settle: min_z={min_z*1e3:7.3f} mm  max|qd|={max_speed:7.3f}  "
        f"finite={finite} no_tunnel={no_tunnel} settled={settled}"
    )
    return finite and no_tunnel and settled


def main():
    ap = argparse.ArgumentParser()
    # VBD is Newton's validated rod/cable solver (all shipped cable examples use it);
    # XPBD is kept selectable to confirm it does *not* drive cable-joint bend stiffness.
    ap.add_argument("--solvers", nargs="+", default=["vbd"])
    ap.add_argument("--substeps", type=int, default=10)
    args = ap.parse_args()

    wp.init()
    dev = wp.get_device()
    print(f"=== D0 rod gate on {dev} (warp {wp.__version__}, newton {newton.__version__}) ===")
    print(f"rod: L={L_DEFAULT}m radius={RADIUS_DEFAULT*1e3:.2f}mm n_seg={N_SEG_DEFAULT} "
          f"substeps={args.substeps}\n")

    # Responsive zone for a sub-mm rod: floppy (~1e-1) -> near-rigid (~1e3).
    bend_values = [1.0e-1, 1.0e0, 1.0e1, 1.0e2, 1.0e3]
    summary = {}
    for sv in args.solvers:
        print(f"--- solver: {sv} ---")
        try:
            fps_actual, finite_t = test_throughput(sv, substeps=args.substeps)
            droops, monotonic = test_cantilever(sv, bend_values, substeps=args.substeps)
            stable = test_drop_stability(sv, substeps=args.substeps)
            realtime = fps_actual >= 60.0 and finite_t
            summary[sv] = dict(fps=fps_actual, realtime=realtime, monotonic=monotonic, stable=stable)
        except Exception as e:
            import traceback
            traceback.print_exc()
            summary[sv] = dict(error=str(e))
        print()

    print("=== GATE VERDICT ===")
    any_pass = False
    for sv, r in summary.items():
        if "error" in r:
            print(f"  {sv}: ERROR {r['error']}")
            continue
        ok = r["realtime"] and r["monotonic"] and r["stable"]
        any_pass = any_pass or ok
        print(
            f"  {sv}: realtime>=60Hz={r['realtime']} ({r['fps']:.0f}fps)  "
            f"bend-monotonic={r['monotonic']}  drop-stable={r['stable']}  -> "
            f"{'PASS' if ok else 'FAIL'}"
        )
    print(f"\nD0 GATE: {'PASS -> proceed on Newton (option B)' if any_pass else 'FAIL -> consider hand-written Warp (A)'}")


if __name__ == "__main__":
    main()
