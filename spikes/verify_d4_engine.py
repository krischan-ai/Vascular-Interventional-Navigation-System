"""In-process D4 validation of NewtonEngine through NavigationEngine.

Mirrors how D3 was validated but drives the engine directly (no WebSocket): build
NavigationEngine(aorta_tree, route) with the Newton backend, reset, push forward,
and report the pipeline-level signals (contact_force / wall_distance / progress /
control-fps) plus a rotate-deflection check that the D4c torsion actually steers
the tip.

Run on the A6000 (from ~/cathsim-warp):
    CATHSIM_PHYSICS_ENGINE=newton python spikes/verify_d4_engine.py --route endpoint_6
"""

from __future__ import annotations

import argparse
import os
import time

import numpy as np

os.environ.setdefault("CATHSIM_PHYSICS_ENGINE", "newton")

from services.navigation_engine import NavigationEngine


def run(args):
    print(f"=== D4 engine verify [{args.route}] drive={os.environ.get('CATHSIM_NEWTON_DRIVE','force')} ===")
    nav = NavigationEngine(phantom=args.phantom, target=args.route, route_target=args.route, guided=False)
    st = nav.reset()
    tip0 = np.asarray(st.tip_position, dtype=float)

    # --- push forward, blind constant speed (worst case, no force gating) ------
    cf, wd, prog = [], [], []
    t0 = time.perf_counter()
    for _ in range(args.frames):
        st = nav.step(delta_push=1.0, delta_rotate=0.0)
        cf.append(st.contact_force); wd.append(st.wall_distance); prog.append(st.path_progress)
    elapsed = time.perf_counter() - t0
    tip1 = np.asarray(st.tip_position, dtype=float)
    fps = args.frames / elapsed
    cf = np.asarray(cf); wd = np.asarray(wd); prog = np.asarray(prog)
    # contact_force is a geometric proxy = breach(m) * contact_ke, so implied
    # wall breach (mm) = contact_force / contact_ke * 1e3. Containment = <~1.5mm.
    breach_mm = cf / args.contact_ke * 1e3
    print(f"[push] progress {prog[0]*100:.1f}% -> {prog[-1]*100:.1f}%  tip_moved={np.linalg.norm(tip1-tip0)*1e3:.1f}mm")
    print(f"[push] implied_breach max={breach_mm.max():.2f}mm mean={breach_mm.mean():.2f}mm  "
          f"contact_force max={cf.max():.0f}  control-fps={fps:.1f}")
    contained = bool(breach_mm.max() < args.breach_budget and np.isfinite(tip1).all())

    # --- rotate-deflection check: torsion should move the tip laterally --------
    tip_pre = np.asarray(nav.step(0.0, 0.0).tip_position, dtype=float)
    for _ in range(args.rotate_frames):
        st = nav.step(delta_push=0.0, delta_rotate=1.0)
    tip_rot = np.asarray(st.tip_position, dtype=float)
    deflect = float(np.linalg.norm(tip_rot - tip_pre)) * 1e3
    steers = deflect > args.deflect_budget
    print(f"[rotate] tip deflected {deflect:.2f}mm over {args.rotate_frames} rotate-steps  "
          f"({'STEERS' if steers else 'no deflection'})")

    nav.close()
    ok = contained and steers
    print(f"D4 ENGINE VERIFY [{args.route}]: {'PASS' if ok else 'CHECK'}  "
          f"(contained={contained} steers={steers})")
    return ok


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--phantom", default="aorta_tree")
    p.add_argument("--route", default="endpoint_6")
    p.add_argument("--frames", type=int, default=150)
    p.add_argument("--rotate-frames", type=int, default=40)
    p.add_argument("--contact-ke", type=float, default=2.0e6)
    p.add_argument("--breach-budget", type=float, default=1.5)
    p.add_argument("--deflect-budget", type=float, default=0.5)
    args = p.parse_args()
    raise SystemExit(0 if run(args) else 1)


if __name__ == "__main__":
    main()
