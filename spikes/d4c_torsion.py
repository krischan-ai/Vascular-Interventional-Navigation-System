"""D4c probe: does proximal torsion steer a pre-bent J-tip?

Question: Newton's cable joint exposes only [stretch, bend] DOFs (the probe showed
`joint_twist_lower/upper = None`), so twist may be a *rest-frame* property baked by
`create_parallel_transport_cable_quaternions(twist_total=...)` rather than a
runtime-drivable DOF. J-tip steering needs the opposite: rotating the proximal
shaft at runtime must rotate the plane of the distal pre-bend, so `rotate` can aim
the tip into a chosen branch.

This spike builds a straight shaft with a pre-bent J-tip (free space, no vessel),
anchors the root, and rotates the root frame about the shaft axis over time. It
measures whether the tip's azimuth (the direction the J points, around the shaft)
tracks the applied root angle.

  tip_azimuth = atan2(tip_y, tip_x) relative to the shaft axis at the tip base.
  gain = d(tip_azimuth) / d(root_angle).   ~1 => torsion propagates (steerable).
                                            ~0 => no twist coupling (need another
                                                  steering mechanism).

Run on the A6000 (from ~/cathsim-warp):
    python spikes/d4c_torsion.py
    python spikes/d4c_torsion.py --mode restframe   # baked twist_total instead
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import warp as wp

import newton


ROD_RADIUS = 4.0e-4
SEG = 3.0e-3


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def rot_z(theta):
    return np.array([0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2)])


def build_jtip_rod(n_shaft, n_tip, bend_deg):
    """Straight shaft along +z, then a J-tip curving in +x over n_tip segments."""
    pts = [np.array([0.0, 0.0, z * SEG]) for z in range(n_shaft)]
    p = pts[-1].copy()
    d = np.array([0.0, 0.0, 1.0])
    dphi = math.radians(bend_deg) / max(n_tip, 1)
    for _ in range(n_tip):
        # rotate the heading toward +x in the x-z plane
        c, s = math.cos(dphi), math.sin(dphi)
        d = np.array([c * d[0] + s * d[2], 0.0, -s * d[0] + c * d[2]])
        p = p + SEG * d
        pts.append(p.copy())
    return np.asarray(pts)


def tip_azimuth(tip_xyz, base_xyz):
    """Azimuth (rad) of the tip offset around the shaft axis (+z)."""
    off = tip_xyz - base_xyz
    return math.atan2(off[1], off[0])


def run(args):
    wp.init()
    print(f"=== D4c torsion probe mode={args.mode} on {wp.get_device()} (newton {newton.__version__}) ===")

    results = []
    twists = [0.0] if args.mode == "drive" else [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]
    for twist_total in twists:
        pts = build_jtip_rod(args.shaft, args.tip, args.bend_deg)
        b = newton.ModelBuilder()
        b.rigid_gap = 0.0
        b.default_shape_cfg.density = 1.8e3
        rod_q = newton.utils.create_parallel_transport_cable_quaternions(
            [wp.vec3(*p) for p in pts], twist_total=twist_total)
        rod_bodies, _ = b.add_rod(positions=[wp.vec3(*p) for p in pts], quaternions=rod_q, radius=ROD_RADIUS,
                                  stretch_stiffness=3e7, stretch_damping=0.0, bend_stiffness=args.bend,
                                  bend_damping=1e-1, label="gw")
        root = rod_bodies[0]
        b.body_mass[root] = 0.0; b.body_inv_mass[root] = 0.0
        b.body_inertia[root] = wp.mat33(0.0); b.body_inv_inertia[root] = wp.mat33(0.0)
        b.color(balance_colors=False)
        m = b.finalize(); m.set_gravity((0.0, 0.0, 0.0))
        solver = newton.solvers.SolverVBD(m, iterations=6)
        s0, s1 = m.state(), m.state()
        ctrl, contacts = m.control(), m.contacts()
        rb = np.asarray(rod_bodies)
        base_body = rb[args.shaft - 1]   # last shaft node = base of the J
        tip_body = rb[-1]
        root_base_quat = s0.body_q.numpy()[root, 3:7].copy()
        sim_dt = (1.0 / 60) / args.substeps

        def settle(steps):
            nonlocal s0, s1
            for _ in range(steps):
                s0.clear_forces(); m.collide(s0, contacts); solver.step(s0, s1, ctrl, contacts, sim_dt); s0, s1 = s1, s0

        settle(args.settle)
        q = s0.body_q.numpy()
        az0 = tip_azimuth(q[tip_body, :3], q[base_body, :3])

        if args.mode == "restframe":
            print(f"  twist_total={math.degrees(twist_total):6.1f}deg -> tip_azimuth={math.degrees(az0):+7.1f}deg")
            results.append((twist_total, az0))
            continue

        # drive mode: rotate the root frame about z and track the tip azimuth
        print(f"  settled tip_azimuth={math.degrees(az0):+.1f}deg; now rotating root...")
        applied, measured = [], []
        for k in range(1, args.turns + 1):
            theta = k * args.dtheta
            for _ in range(args.steps_per_turn):
                s0.clear_forces(); m.collide(s0, contacts)
                solver.step(s0, s1, ctrl, contacts, sim_dt); s0, s1 = s1, s0
                bq = s0.body_q.numpy()
                bq[root, 3:7] = quat_mul(rot_z(theta), root_base_quat)
                s0.body_q.assign(bq)
            q = s0.body_q.numpy()
            az = tip_azimuth(q[tip_body, :3], q[base_body, :3])
            applied.append(math.degrees(theta))
            measured.append(math.degrees(az - az0))
            print(f"    root={math.degrees(theta):+7.1f}deg -> d_tip_azimuth={math.degrees(az-az0):+7.1f}deg")
        applied = np.asarray(applied); measured = np.unwrap(np.radians(measured))
        measured = np.degrees(measured)
        gain = float(np.polyfit(applied, measured, 1)[0]) if len(applied) > 1 else 0.0
        print(f"  torsion gain d(tip_az)/d(root) = {gain:+.3f}  "
              f"({'STEERABLE' if abs(gain) > 0.3 else 'NO twist coupling'})")
        return abs(gain) > 0.3

    if args.mode == "restframe":
        azs = [math.degrees(a) for _, a in results]
        spread = max(azs) - min(azs)
        print(f"  rest-frame azimuth spread over twist_total = {spread:.1f}deg "
              f"({'twist sets tip plane' if spread > 30 else 'no effect'})")
        return spread > 30


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--mode", choices=["drive", "restframe"], default="drive")
    p.add_argument("--shaft", type=int, default=14)
    p.add_argument("--tip", type=int, default=6)
    p.add_argument("--bend-deg", type=float, default=90.0)
    p.add_argument("--bend", type=float, default=50.0)
    p.add_argument("--substeps", type=int, default=8)
    p.add_argument("--settle", type=int, default=200)
    p.add_argument("--turns", type=int, default=8)
    p.add_argument("--dtheta", type=float, default=math.pi / 4)
    p.add_argument("--steps-per-turn", type=int, default=60)
    args = p.parse_args()
    ok = run(args)
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
