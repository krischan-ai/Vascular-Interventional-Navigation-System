"""Blocked-tip verification of the anti-buckling pass (doc/08 §9.1 sheath / §28.9).

The A6000 smoke (spikes/smoke2.py) only exercised a SUCCESS route (autopilot
endpoint_9), where the tip advances so the prolapse guard's feed budget is never
exhausted -- it proved "the guard doesn't break a working route", NOT that it
actually prevents buckling. This spike closes that gap by driving a CONSTANT
push=1.0 into a known jam (endpoint_3 stalls a blind push at its sharp bend,
per spikes/d5_autopilot.py) and comparing:

    OLD  sheath_bodies=1, max_slack=0   (pre-session: root-only glue, no guard)
    NEW  defaults                       (auto sheath + prolapse guard)

Buckling metrics (higher = more coiled / piled up):
  slack_mm    fed arclen - actual tip arclen. Guard caps NEW near max_slack;
              OLD grows unbounded as wire piles into the jam.
  breach_mm   max wall penetration (contact_force/contact_ke). Buckled loops
              press into the wall -> spikes for OLD.
  pile_ratio  rod rest length / centerline span it covers. ~1 = laid along the
              lumen; >>1 = folded into a short span (buckled).
  offaxis_mm  max body distance to the centerline. Loops wander off-axis.

Run on the A6000 (from ~/cathsim-warp):
    CATHSIM_PHYSICS_ENGINE=newton python spikes/verify_antibuckle.py --route endpoint_3
"""

from __future__ import annotations

import argparse
import os

import numpy as np

os.environ.setdefault("CATHSIM_PHYSICS_ENGINE", "newton")

from services.navigation_engine import NavigationEngine
from services.physics.newton_engine import _nearest_arclen


def _metrics(nav: NavigationEngine) -> dict:
    eng = nav._engine
    cl, cum = eng._centerline, eng._cl_cum
    bodies = np.asarray([b["pos"] for b in nav.get_render_bodies()], dtype=np.float64)
    nb = len(bodies)
    seg_len = float(eng._rod_seg_len)
    rest_len = (nb - 1) * seg_len

    root_arc = _nearest_arclen(cl, cum, bodies[0])
    tip_arc = _nearest_arclen(cl, cum, bodies[-1])
    span = max(tip_arc - root_arc, 1e-6)
    fed_arc = float(eng._insert_s + eng._base_arc[-1])

    # Max distance of any body to the nearest centerline node (off-axis excursion).
    d2 = np.sum((bodies[:, None, :] - cl[None, :, :]) ** 2, axis=2)
    offaxis = float(np.sqrt(d2.min(axis=1)).max())

    return {
        "slack_mm": (fed_arc - tip_arc) * 1e3,
        "pile_ratio": rest_len / span,
        "offaxis_mm": offaxis * 1e3,
        "tip_arc_mm": tip_arc * 1e3,
    }


def _drive(nav: NavigationEngine, cfg: dict, frames: int, contact_ke: float) -> dict:
    nav.reset()
    nav.set_engine_params(cfg)  # sheath_bodies (reglues) + max_slack (live)
    breach_mm = 0.0
    prog = 0.0
    for _ in range(frames):
        st = nav.step(delta_push=1.0, delta_rotate=0.0)
        breach_mm = max(breach_mm, st.contact_force / contact_ke * 1e3)
        prog = st.path_progress
    m = _metrics(nav)
    m["breach_mm"] = breach_mm
    m["reach_pct"] = prog * 100.0
    return m


def run(args):
    print(f"=== anti-buckle verify [{args.route}] constant push=1.0, {args.frames} frames ===")
    nav = NavigationEngine(
        phantom=args.phantom, target=args.route, route_target=args.route, guided=False
    )
    ke = float(getattr(nav._engine, "contact_ke", 3.0e6))

    configs = {
        "OLD (sheath=1, guard off)": {"sheath_bodies": 1, "max_slack": 0.0},
        "NEW (auto sheath + guard)": {"sheath_bodies": -1, "max_slack": 0.012},
    }
    rows = {name: _drive(nav, cfg, args.frames, ke) for name, cfg in configs.items()}

    cols = ["reach_pct", "slack_mm", "breach_mm", "pile_ratio", "offaxis_mm"]
    print(f"\n{'config':<28}" + "".join(f"{c:>12}" for c in cols))
    for name, m in rows.items():
        print(f"{name:<28}" + "".join(f"{m[c]:>12.2f}" for c in cols))

    old, new = rows["OLD (sheath=1, guard off)"], rows["NEW (auto sheath + guard)"]
    print("\nverdict:")
    print(f"  slack   {old['slack_mm']:.1f} -> {new['slack_mm']:.1f} mm "
          f"({'capped' if new['slack_mm'] <= 15.0 else 'NOT capped'})")
    print(f"  pile    {old['pile_ratio']:.2f} -> {new['pile_ratio']:.2f} "
          f"({'less folded' if new['pile_ratio'] < old['pile_ratio'] else 'NOT better'})")
    print(f"  breach  {old['breach_mm']:.2f} -> {new['breach_mm']:.2f} mm "
          f"({'contained' if new['breach_mm'] <= old['breach_mm'] + 0.05 else 'NOT better'})")
    guard_ok = new["slack_mm"] <= 15.0 and new["pile_ratio"] <= old["pile_ratio"] + 0.05
    print(f"\n{'PASS' if guard_ok else 'FAIL'}: anti-buckling "
          f"{'bounds the jam (guard + auto-sheath work)' if guard_ok else 'did not help -- retune'}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--phantom", default="aorta_tree")
    p.add_argument("--route", default="endpoint_3")
    p.add_argument("--frames", type=int, default=400)
    run(p.parse_args())


if __name__ == "__main__":
    main()
