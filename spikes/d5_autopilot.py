"""D5: closed-loop PhysicsAutopilot on the D4 real-force Newton engine.

D4 force drive transmits push + steers via torsion, but a passive constant push
jams at sharp bends (endpoint_3 stalls ~52%). D5 closes the loop: the existing
`PhysicsAutopilot` (look-ahead heading error + sign-search hill-climb on rotate +
force-gated push + stall sweep/retract) drives `NavigationEngine`'s Newton backend
to the target. This is the D4+D5 coupling the plan calls for (force drive needs
active steering to advance).

Key adaptation: Newton's `contact_force` is a geometric proxy (`breach·contact_ke`,
range 0..~4000), NOT MuJoCo's Newtons. The force gates are rescaled accordingly
(`--force-soft/--force-hard`, default 300/1500 => breach 0.1mm/0.5mm).

Run on the A6000 (from ~/cathsim-warp):
    PYTHONPATH=. CATHSIM_PHYSICS_ENGINE=newton python spikes/d5_autopilot.py --route endpoint_3
"""

from __future__ import annotations

import argparse
import os

import numpy as np

os.environ.setdefault("CATHSIM_PHYSICS_ENGINE", "newton")

from services.navigation_engine import NavigationEngine
from services.physics_autopilot import AutopilotConfig, PhysicsAutopilot


def run(args):
    print(f"=== D5 autopilot [{args.route}] on Newton force drive ===")
    nav = NavigationEngine(phantom=args.phantom, target=args.route, route_target=args.route, guided=False)
    st = nav.reset()
    path_pts = np.asarray(nav.planned_path, dtype=np.float64)

    cfg = AutopilotConfig(
        base_push=args.base_push,
        lookahead_m=args.lookahead,
        force_soft=args.force_soft,
        force_hard=args.force_hard,
        force_emergency=args.force_emergency,
    )
    ap = PhysicsAutopilot(path_pts, config=cfg)
    ap.reset()

    contact_ke = args.contact_ke
    prog, cf, pushes, rotates = [], [], [], []
    for _ in range(args.frames):
        push, rotate = ap.compute(st.tip_position, st.tip_direction, st.contact_force)
        st = nav.step(delta_push=push, delta_rotate=rotate)
        prog.append(st.path_progress); cf.append(st.contact_force)
        pushes.append(push); rotates.append(rotate)

    prog = np.asarray(prog); cf = np.asarray(cf)
    breach_mm = cf / contact_ke * 1e3
    reached = prog.max() >= args.reach_frac
    print(f"[autopilot] progress max={prog.max()*100:.1f}% final={prog[-1]*100:.1f}%  "
          f"deepest_arclen={ap.deepest_arclen*1e3:.1f}mm")
    print(f"[autopilot] implied_breach max={breach_mm.max():.2f}mm mean={breach_mm.mean():.2f}mm  "
          f"|rotate|mean={np.mean(np.abs(rotates)):.2f} push_mean={np.mean(pushes):.2f}")
    contained = bool(breach_mm.max() < args.breach_budget and np.isfinite(prog).all())
    nav.close()
    ok = reached and contained
    print(f"D5 AUTOPILOT [{args.route}]: {'PASS' if ok else 'CHECK'}  "
          f"(reached>={args.reach_frac*100:.0f}%={reached} contained={contained})")
    return ok


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--phantom", default="aorta_tree")
    p.add_argument("--route", default="endpoint_3")
    p.add_argument("--frames", type=int, default=600)
    p.add_argument("--base-push", type=float, default=0.35)
    p.add_argument("--lookahead", type=float, default=0.025)
    p.add_argument("--force-soft", type=float, default=300.0)
    p.add_argument("--force-hard", type=float, default=1500.0)
    p.add_argument("--force-emergency", type=float, default=2500.0)
    p.add_argument("--contact-ke", type=float, default=3.0e6)
    p.add_argument("--reach-frac", type=float, default=0.9)
    p.add_argument("--breach-budget", type=float, default=1.5)
    args = p.parse_args()
    raise SystemExit(0 if run(args) else 1)


if __name__ == "__main__":
    main()
