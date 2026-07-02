# D4 — Real force-driven guidewire (results)

Spike: [d4_force_drive.py](d4_force_drive.py). Run on the A6000 (`cathsim-newton`),
aorta_tree routes, Newton 1.3.0 / warp 1.14.0. Continues doc/08 §9.1 D4.

## D4a — force transmission (replaces D3 graded soft-anchor) ✅

**What changed vs D3.** D3 drove kinematically: every inserted body was glued to
the centered route each substep (`pos = (1-alpha)*phys + alpha*target`), so the
whole rod teleported forward — no tip lag, no buckling, no wall-hug. D4 glues only
the proximal `--sheath-bodies` (the introducer/port + already-traversed wire) and
advances *that* at the push speed; the entire distal working length + tip is FREE
physics. The push is transmitted down the rod by the cable **stretch** constraint,
and the tip pose emerges from **bend** stiffness + **wall contact**.

**The decisive finding: stretch stiffness must be ~3e7, not 1e5.** With the D0/D3
`stretch_stiffness=1e5`, teleporting the sheath front shocks the stretch
constraint and the rod stretches 80–240 % (`strain` 0.8–2.4) — a solver failure,
not realism (a real guidewire is near-inextensible). Raising it makes the rod
inextensible and the push transmits cleanly:

| stretch | tip_lag steady | strain steady | tip_reach (ep9 131mm) | fps |
|---|---|---|---|---|
| 1e5 | ~40 mm (unstable) | 0.26–0.87 | 87–94 mm | 32 |
| **3e7** | **1.0 mm** | **0.017** | **129 mm** | **32** |
| 1e8 | 0.9 mm | 0.008 | 129 mm | 32 |

3e7 is the knee; 1e8 is a safety margin. `stretch_damping=0.0` still mandatory
(D0 finding unchanged). More VBD iterations/substeps do NOT fix the 1e5 strain —
stiffness does.

**The stiffness tradeoff is real (why no single passive bend passes all routes).**

- `bend=50` (stiff shaft): pushable, but on a sharp bend the stiff shaft **chords
  across the curve and presses through the outer wall** — ep3 breach +6.3 mm (穿墙).
- `bend=20` (conforming shaft): follows vessel curvature, **contained on every
  route**, but on the tightest bends the softer wire **prolapses / the tip jams**
  (ep9 tip stops at 83 mm under blind constant push).

This is textbook guidewire mechanics: too stiff → perforate, too soft →
prolapse/no-push. No passive stiffness threads a 340° tortuous vessel — that is
exactly why interventionalists **torque a soft J-tip** (→ D4c torsion + D5
steering). The blind constant-speed push in the spike is also worst-case: D5's
force-gated push halts on the resistance spike, so the jam never accumulates the
46 mm lag seen here.

**D4a chosen recipe (contained on all routes @ ~32fps):**
`--drive force --sheath-bodies 1 --stretch 3e7 --bend 20 --soft-tip 10
--tip-bend 2 --contact-ke 2e6` (8 substeps × 4 VBD iters).

Containment sweep (breach worst, lower is better; <0 = fully inside lumen):

| route | len | rmin | turn | breach worst | tip_lag | progressing |
|---|---|---|---|---|---|---|
| endpoint_6 | 259mm | 3.67 | 301° | +0.02mm | 7.2mm | yes |
| endpoint_9 | 131mm | 2.93 | 189° | +0.32mm | (jams, D5) | no |
| endpoint_3 | 210mm | 3.23 | 340° | +0.95mm | (jams, D5) | no |
| endpoint_15 | 325mm | 2.60 | 326° | −0.00mm | 13.9mm | yes |
| endpoint_26 | 520mm | 2.82 | 399° | +0.03mm | 14.9mm | yes |
| endpoint_24 | 538mm | 3.95 | 321° | +0.02mm | 14.8mm | yes |

**Acceptance split (important):** D4a guarantees **containment even under blind
over-push** (prolapse inside, never perforate) + **real force transmission**
(lag emerges: 7–15 mm on long routes, strain ~0.02). *Passing sharp bends where
the passive tip jams is delegated to D4c (steering) + D5 (force-gated autopilot),
not a D4a failure.*

## D4b — segmented stiffness (软头硬身)

Per-joint bend stiffness is directly settable: cable-joint DOF layout is
`[stretch, bend]` per joint, so bend ke lives at **odd indices** of
`model.joint_target_ke` (`ke[2*j+1]`). The spike ramps the distal `--soft-tip`
joints from `--bend` down to `--tip-bend`. Working; folded into the D4a recipe
(soft-tip 10, tip-bend 2). Shaft `bend` governs wall-conformance (containment);
tip softness governs atraumatic bend entry. Further calibration to a real
guidewire's soft-tip length pending.

## D4c — Cosserat torsion → J-tip steering ✅

Spike: [d4c_torsion.py](d4c_torsion.py). Straight shaft + pre-bent J-tip, free
space, anchor root, measure tip azimuth (the direction the J points around the
shaft) vs applied root angle.

- **Rest-frame twist does NOT steer.** Baking `twist_total` into
  `create_parallel_transport_cable_quaternions` produced 0° tip-azimuth spread —
  useless for runtime steering.
- **Runtime root-frame rotation DOES steer, gain ≈ 1.** Rotating the anchored
  root's quaternion about the shaft axis each substep propagates twist down the
  cable and rotates the J-tip's bend plane nearly 1:1:
  `d(tip_azimuth)/d(root_angle) = 0.96` over a full 360° turn. **STEERABLE.**

So even though the cable joint exposes no explicit twist DOF (`joint_twist_*` =
None), its bend constraint references the child body's full orientation, so
twisting the proximal anchor carries torsion to the tip. **This is the steering
primitive for `rotate`:** `rotate` → rotate the root anchor frame about the local
tangent → J-tip reorients. Closed-loop aiming (which way, how much) is D5.

## D4d — sheath constraint — sheath-bodies=1 sufficient so far; revisit for long-route proximal buckling

## Port into newton_engine.py ✅ (validated through NavigationEngine)

Ported (D3-style, zero upper-layer change): `services/physics/newton_engine.py`
now defaults to `CATHSIM_NEWTON_DRIVE=force` (D3 `anchor` kept as fallback).
Force mode glues only `sheath_bodies=1`, frees the distal length, feeds the root
along the route, integrates `rotate` into a root-frame twist about the local
tangent (D4c), soft-tip ramp on `joint_target_ke`, `stretch=3e7`, `bend=20`,
`contact_ke=3e6`, plus a 20-step settle on init. Verify:
[verify_d4_engine.py](verify_d4_engine.py) drives it in-process through the real
`NavigationEngine` and reads pipeline signals.

| route | len/rmin/turn | progress | implied breach max | rotate deflect | fps | verdict |
|---|---|---|---|---|---|---|
| endpoint_9 | 131/2.9/189° | 44→99% | 0.37mm | 5.9mm | 54 | PASS |
| endpoint_6 | 259/3.7/301° | 22→99% | 0.26mm | 7.6mm | 54 | PASS |
| endpoint_3 | 210/3.2/340° | 27→52%* | 1.34mm | 5.2mm | 54 | PASS |
| endpoint_15 | 325/2.6/326° | 18→87% | 0.28mm | 2.9mm | 54 | PASS |
| endpoint_26 | 520/2.8/399° | 11→59% | 0.05mm | 11.8mm | 54 | PASS |
| endpoint_9 (anchor) | — | 45→99% | 0.00mm | 0 (n/a) | 46 | fallback OK |

*endpoint_3 tip jams at a sharp bend (passive wire, no steering) but stays
contained — passing it is D5's force-gated + torsion-steer job.

**Pipeline-level wins vs D3:** real `contact_force`/`wall_distance` now vary with
true wall contact (D3 anchor glued to centerline → force ≡ 0); `rotate` steers
(D3 ignored it); force mode is *faster* (54 vs 46 fps, only 1 body glued/substep).
`contact_force` is a geometric proxy `= breach(m) * contact_ke`, so implied
breach(mm) `= contact_force / 3e6 * 1e3`.

## D5 — closed-loop PhysicsAutopilot on the force drive (partial)

Spike: [d5_autopilot.py](d5_autopilot.py). Reconnects the existing
`services/physics_autopilot.py` (never wired into the live pipeline before — only
`tools/measure_physics_reach.py` used it) onto NavigationEngine's Newton force
backend. Adaptation: Newton `contact_force` is the geometric proxy
(`breach·contact_ke`), so the force gates are rescaled (`force_soft=300`,
`force_hard=1500`, new `force_emergency=2500` ≈ breach 0.1/0.5/0.8mm).

Two autopilot safety fixes (both pass the existing 6 unit tests):
- force-gate the **stall sweep** forward push (was un-gated → rammed);
- **emergency retract** (no sweep) above `force_emergency` — spinning a wedged tip
  hard against the wall augers it through (endpoint_3 perforated to 10mm).

| route | progress (autopilot) | breach max/mean | push_mean | verdict |
|---|---|---|---|---|
| endpoint_9 | **99.6%** | 0.24 / 0.06 mm | 0.16 | PASS |
| endpoint_6 | **99.5%** | 0.24 / 0.01 mm | 0.24 | PASS |
| endpoint_3 | 37% (jams) | 5.84 / 0.83 mm | 0.03 | CHECK |

**Main-route acceptance MET:** endpoint_9/6 auto-reach the target (~99%),
contained (0.24mm), force-gated push (0.16–0.24 vs blind 1.0), no whipping.

### Pre-bent J-tip cracks the sharp branches ✅

The rod was built straight along the centerline, so torsion only had
*incidental-curvature* authority — too weak to point into a sharp branch
(endpoint_3 jammed at 37%). Fix: `_apply_jtip` bends the distal `jtip_bodies=3`
segments into a pre-bent J rest shape (`CATHSIM_NEWTON_JTIP_DEG`, default **35°**
in force mode). The baked curvature is held by bend stiffness, so `rotate`
reorients the J to select branches. **35° is the knee** (20° too weak → 54%; 50°
over-bent → jams/perforates at 37%).

Autopilot with J-tip=35° across the route set (frames scaled to route length):

| route | len | progress (autopilot) | breach max/mean | verdict |
|---|---|---|---|---|
| endpoint_9 | 131mm | 99.1% | 0.46 / 0.09 mm | PASS |
| endpoint_6 | 259mm | 99.5% | 0.15 / 0.01 mm | PASS |
| endpoint_3 | 210mm | **98.9%** | 1.08 / 0.04 mm | **PASS** (was 37%) |
| endpoint_15 | 325mm | 99.9% | 0.51 / 0.00 mm | PASS |
| endpoint_24 | 538mm | 99.5% | 0.19 / 0.00 mm | PASS |
| endpoint_26 | 520mm | 49% (stalls) | 0.64 / 0.33 mm | CHECK (contained) |

**5/6 routes auto-reach the target (~99%), all contained.** Only endpoint_26 (the
single 399°-turn 520mm route) has a genuine mid-route stall at ~257mm — the
§28.9 tortuous-route long-tail. Under blind push (no steering) the J-tip catches
walls earlier and advances less (ep9 47%) — correct behaviour: a J-tip wire must
be torqued to pass bends; that is what the autopilot/manual steering does.

## Verdict
D4 resolved and shipped: **(a)** force transmission (`stretch=3e7`); **(b)**
soft-head/stiff-body (`joint_target_ke[2*j+1]`, shaft `bend=20`); **(c)** torsion
steering via runtime root-frame rotation + **pre-bent J-tip (35°)** for real
branch selection. D5 autopilot reconnected and force-gated: **5/6 routes
auto-reach target, all contained @54fps**. Remaining long-tail: the single most
tortuous route (endpoint_26) mid-route stall — wide-lumen/stall convergence
tuning (§28.9). Ready to deploy to the running backend + Godot smoke test.
