"""Introspect the compiled guidewire joint / qpos layout.

Needed to design the centerline pre-threading: to bend the physical wire onto
the planned centerline at reset we must know, for each guidewire body, which
qpos addresses its two steering hinges occupy and how the body frames chain.

Usage:
    python -m tools.inspect_guidewire_joints --phantom segment_part --n-bodies 40
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "glfw")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phantom", default="segment_part")
    parser.add_argument("--target", default="root")
    parser.add_argument("--n-bodies", type=int, default=40)
    args = parser.parse_args()

    from services.navigation_engine import NavigationEngine

    engine = NavigationEngine(
        phantom=args.phantom, target=args.target, guided=False,
        n_bodies=args.n_bodies, insertion_max=0.7,
    )
    engine.reset()
    physics = engine._env.physics
    model = physics.model

    print(f"nq={model.nq} nv={model.nv} nbody={model.nbody} njnt={model.njnt}")
    print("-- joints (name | type | qposadr | bodyname) --")
    for j in range(model.njnt):
        jname = model.id2name(j, "joint") or ""
        btype = int(model.jnt_type[j])
        qadr = int(model.jnt_qposadr[j])
        bid = int(model.jnt_bodyid[j])
        bname = model.id2name(bid, "body") or ""
        print(f"  {jname:22s} type={btype} qposadr={qadr:3d} body={bname}")

    print("-- guidewire body chain (id | name | parentid | pos) --")
    for b in range(model.nbody):
        bname = model.id2name(b, "body") or ""
        if "guidewire" in bname or "tip" in bname:
            pid = int(model.body_parentid[b])
            pos = model.body_pos[b]
            print(f"  {b:3d} {bname:22s} parent={pid:3d} pos={pos}")

    # After a normal reset, compare each chain body's world +z (R@z) with the
    # actual world vector to the next body. This reveals the true segment-
    # direction axis and validates the pre-threading frame convention.
    import numpy as np

    data = physics.data
    chain_ids = [
        b for b in range(model.nbody)
        if ("guidewire_body_" in (model.id2name(b, "body") or ""))
        or ("tip_body_" in (model.id2name(b, "body") or ""))
    ]
    print("-- per-body: world +z (R@z) vs actual dir to next body --")
    for k, b in enumerate(chain_ids[:8]):
        R = np.asarray(data.xmat[b]).reshape(3, 3)
        zaxis = R @ np.array([0.0, 0.0, 1.0])
        xpos = np.asarray(data.xpos[b])
        if k + 1 < len(chain_ids):
            nxt = np.asarray(data.xpos[chain_ids[k + 1]])
            actual = nxt - xpos
            n = np.linalg.norm(actual)
            actual = actual / n if n > 1e-9 else actual
        else:
            actual = np.zeros(3)
        name = model.id2name(b, "body") or ""
        print(
            f"  {name:24s} xpos={_v(xpos)} R@z={_v(zaxis)} dir_to_next={_v(actual)}"
        )

    engine.close()
    return 0


def _v(a) -> str:
    return "[" + ", ".join(f"{x:+.3f}" for x in a) + "]"


if __name__ == "__main__":
    raise SystemExit(main())
