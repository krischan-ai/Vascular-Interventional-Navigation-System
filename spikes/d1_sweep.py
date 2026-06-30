"""D1 operational envelope: wall breach vs push-speed x substeps x collision mode,
through the 90deg curved tube at a fixed ~0.10 m advance. Tells the D3 engine how to
clamp push rate / choose substeps so the rod never tunnels. Each printed line ends in
worst_breach[mm] and PASS/FAIL (PASS if breach < 0.5mm)."""
import warp as wp
import d1_tube_collision as d1

wp.init()
ADV = 0.10  # fixed advance distance [m]
for name, use_sdf in (("mesh", False), ("SDF", True)):
    print(f"================= collision = {name} =================")
    for ss in (10, 40):
        for sp in (0.05, 0.2, 0.5, 1.0):
            frames = max(1, int(round(ADV * 60 / sp)))
            d1.push_through(True, sp, frames, ss, use_sdf, f"{name} ss={ss:<2d} v={sp:.2f}")
    print()
