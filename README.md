# Vascular Interventional Navigation System

Research-oriented digital-twin platform for vascular guidewire/catheter navigation, combining vessel path planning, physics simulation, real-time services, 3D interaction, safety state, and reinforcement-learning interfaces.

> **Portfolio focus:** simulation systems, medical navigation, physics/AI integration, real-time backend engineering, Godot visualization, and safety-aware control architecture.

> This project is for research, simulation, teaching, and human-computer-interaction experiments. It is **not** a clinical control system or medical-device claim.

## What this project solves

Vascular-intervention research needs multiple systems to agree on the same geometry, path semantics, control commands, safety state, and simulation fidelity. This project integrates those layers into one reusable navigation platform rather than treating path planning, physics, visualization, and RL as separate demos.

## System at a glance

```text
Vessel Imaging / VTK / Centerlines / Radius
                    ↓
          Asset & Path Planning
          A* · Routes · B-spline
                    ↓
       Shared Navigation Core
   Path · ShapeIntent · Safety · Risk
          ↙                 ↘
 Godot / XR HCI        Gymnasium / RL
          ↘                 ↙
        Physics Abstraction
  Newton · MuJoCo · Kinematic
                    ↓
 Real-time State · Recording · Evaluation
```

## Engineering highlights

| Area | Implementation |
| --- | --- |
| Geometry & planning | Vessel assets, centerline graph, A* routing, route switching, B-spline smoothing, radius-aware paths |
| Physics | Shared abstraction for Newton GPU simulation, MuJoCo comparison, and kinematic/guided validation |
| Control | `ShapeIntent` layer maps human/RL intent into controlled push/rotate actions |
| Navigation | Progress, deviation, curvature, mechanics, path state, risk, and flow guidance in a shared engine |
| Real-time service | FastAPI REST + WebSocket sessions, navigation-state streaming, path requests, emergency-stop state |
| Visualization | Godot medical-navigation workstation with 3D vessel/guidewire/path views and status feedback |
| RL interface | Gymnasium environment with PPO/SAC training and evaluation entry points |
| XR direction | Godot/OpenXR safety-oriented client work for PICO 4 Ultra experimentation |

## Fidelity matters

The platform explicitly distinguishes simulation modes instead of presenting every output as equivalent physics:

- **Newton** — GPU-oriented high-fidelity development path with guidewire and SDF collision work.
- **MuJoCo** — compatibility and physics-comparison path.
- **Kinematic / guided** — reachability, demonstration, and frontend validation; not treated as equivalent to physical contact simulation.
- **RL** — policy layer that still declares and obeys the underlying physics and safety semantics.

## Tech stack

**Python · FastAPI · WebSocket · Newton / Warp · MuJoCo · Godot · GDScript · Gymnasium · Stable-Baselines3 · PPO / SAC · OpenXR · VTK / medical geometry workflows**

## Architecture principles

1. **One shared core:** human interaction and RL reuse the same path, control, navigation, and physics abstractions.
2. **Intent over direct state mutation:** clients/policies submit high-level intent or push/rotate commands rather than directly rewriting guidewire geometry.
3. **Explicit fidelity:** guided demonstrations cannot masquerade as validated physical simulation.
4. **Backend-owned safety:** risk, safety, force, and device state must come from traceable backend fields; missing data stays unknown rather than being invented in the UI.
5. **Visual/physics separation:** rendering assets are separated from collision/SDF assets so visual changes do not silently change simulation semantics.

## Quick start

Create the Python environment and install the project:

```bash
python -m venv .venv
# activate the environment for your OS
python -m pip install --upgrade pip
python -m pip install -e .
```

Start the backend:

```bash
# default integration port used by the Godot client
CATHSIM_PORT=9000 python -m services.main
```

Then open `godot_client/project.godot` with the repository's supported Godot version for the desktop navigation interface.

## Core verification

Representative regression entry points include service APIs, navigation environment/training, frontend contracts, and XR safety contracts:

```bash
python -m pytest \
  tests/test_services_api.py \
  tests/test_navigation_gym_env.py \
  tests/test_navigation_train.py \
  tests/test_frontend_contract.py \
  tests/test_xr_sprint0_contract.py -q
```

Some physics, asset, Godot, GPU, or XR tests require optional runtime dependencies or hardware.

## Repository map

```text
services/             FastAPI, sessions, navigation, path, safety and physics services
src/cathsim/          Gymnasium / RL integration and reusable simulation package
godot_client/         Desktop and XR interaction clients
data/                 Vessel, path and simulation assets
tools/                Asset conversion and engineering utilities
tests/                Backend, navigation, training, frontend and XR contracts
docs/                 Architecture, images and detailed engineering documentation
doc/                  Extended design documents
```

## Documentation

- [Full original README](./docs/FULL_README.md) — complete subsystem status, architecture, runtime modes, validation boundaries, and development detail.
- [`docs/`](./docs) — architecture assets and focused documentation.
- [`doc/`](./doc) — extended design documents.
- [`TERMS.md`](./TERMS.md) — terminology and project boundaries.

---

**The central engineering goal is a shared, safety-aware navigation core that can support visualization, simulation, and learning without mixing their fidelity or control semantics.**
