"""Drive NewtonEngine directly and sweep params to find a stable demo:
tip advances coherently (no tail-first buckle) and rod stays in its tube."""
import os, json, numpy as np

ROOT = "src/cathsim/dm/components/phantom_assets/meshes/segment_part"
cl = json.load(open(ROOT + "/centerline.json"))
PTS = np.array(cl["waypoints"], dtype=float)


def proj_arclen(centerline, p):
    a = centerline[:-1]; b = centerline[1:]
    ab = b - a; ap = p - a
    denom = np.einsum("ij,ij->i", ab, ab) + 1e-18
    t = np.clip(np.einsum("ij,ij->i", ap, ab) / denom, 0, 1)
    proj = a + t[:, None] * ab
    d2 = np.sum((p - proj) ** 2, axis=1)
    i = int(np.argmin(d2))
    seg = np.linalg.norm(np.diff(centerline, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(seg)])
    return cum[i] + t[i] * seg[i]


def run(bend, substeps, lumen, push, rod_len, steps=120):
    os.environ.update({
        "CATHSIM_NEWTON_BEND": str(bend), "CATHSIM_NEWTON_SUBSTEPS": str(substeps),
        "CATHSIM_NEWTON_LUMEN_RADIUS": str(lumen), "CATHSIM_NEWTON_PUSH_SPEED": str(push),
        "CATHSIM_NEWTON_ROD_LENGTH": str(rod_len),
    })
    from services.physics.base import PlannedPath
    import importlib, services.physics.newton_engine as ne
    importlib.reload(ne)
    eng = ne.NewtonEngine(path=PlannedPath(PTS))
    eng.reset()
    cl_rs = eng._centerline
    rod_len_actual = eng._rod_length
    tip_lag = []
    breach_max = -1e9
    for s in range(steps):
        eng.step(push=1.0, rotate=0.0)
        q = eng._s0.body_q.numpy()[eng._rod_bodies, :3]
        tip_s = proj_arclen(cl_rs, q[-1]); root_s = proj_arclen(cl_rs, q[0])
        tip_lag.append((tip_s - root_s))  # should stay ~rod_len if rigid; shrinks if buckling
        radial = max(np.sqrt(((cl_rs - p) ** 2).sum(1)).min() for p in q)
        breach_max = max(breach_max, radial + eng._rod_radius - lumen)
    q = eng._s0.body_q.numpy()[eng._rod_bodies, :3]
    tip_adv = proj_arclen(cl_rs, q[-1]) - proj_arclen(eng._centerline, eng._s1.body_q.numpy()[eng._rod_bodies[-1], :3])
    final_lag = tip_lag[-1]
    finite = bool(np.isfinite(q).all())
    eng.close()
    return dict(rodlen=rod_len_actual, final_span=final_lag, span0=tip_lag[0],
                breach_mm=breach_max * 1e3, finite=finite)


print(f"rod follows centerline if tip-root span stays ~rod_len; buckle if span shrinks.")
print(f"{'bend':>6} {'ss':>3} {'lumen_mm':>8} {'push':>5} | {'span0_mm':>9} {'spanF_mm':>9} {'breach_mm':>9} {'finite':>6}")
for bend in (1.0, 20.0, 100.0, 400.0):
    for ss in (20,):
        r = run(bend, ss, 0.0025, 0.03, 0.06)
        print(f"{bend:>6.0f} {ss:>3} {0.0025*1e3:>8.1f} {0.03:>5.2f} | "
              f"{r['span0']*1e3:>9.1f} {r['final_span']*1e3:>9.1f} {r['breach_mm']:>9.3f} {str(r['finite']):>6}")
