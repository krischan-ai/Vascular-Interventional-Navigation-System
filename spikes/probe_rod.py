"""Probe Newton add_rod: per-joint bend stiffness arrays + twist/torsion API."""
import numpy as np, warp as wp, newton
wp.init()
b = newton.ModelBuilder()
pts = [wp.vec3(0,0,z) for z in np.linspace(0,0.06,21)]
q = newton.utils.create_parallel_transport_cable_quaternions(pts, twist_total=0.0)
rod_bodies, rod_joints = b.add_rod(positions=pts, quaternions=q, radius=4e-4,
    stretch_stiffness=1e5, stretch_damping=0.0, bend_stiffness=50.0, bend_damping=1e-1, label="gw")
print("newton", newton.__version__)
print("n_bodies", len(rod_bodies), "n_joints(returned)", len(rod_joints) if hasattr(rod_joints,'__len__') else rod_joints)
print("rod_joints=", rod_joints)
b.color(balance_colors=False)
m = b.finalize()
print("\n=== builder attrs with 'joint' or 'target' or 'ke' or 'bend' or 'twist' ===")
for name in sorted(dir(b)):
    if any(k in name.lower() for k in ('joint','target','bend','twist','stiff')) and not name.startswith('_'):
        v = getattr(b, name)
        if isinstance(v, (list, np.ndarray)):
            print(f"  builder.{name}: len={len(v)}", (v[:4] if len(v) else ''))
print("\n=== model attrs (arrays) with joint/target/bend/twist/stiff ===")
for name in sorted(dir(m)):
    if any(k in name.lower() for k in ('joint','target','bend','twist','stiff')) and not name.startswith('_'):
        v = getattr(m, name)
        try:
            arr = v.numpy()
            print(f"  model.{name}: shape={arr.shape} dtype={arr.dtype} sample={arr.flatten()[:6]}")
        except Exception:
            print(f"  model.{name}: {type(v).__name__} = {v}")
print("\n=== add_rod signature ===")
import inspect
print(inspect.signature(b.add_rod))
