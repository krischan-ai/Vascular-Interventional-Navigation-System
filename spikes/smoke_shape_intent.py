"""Smoke test: ShapeIntent (autopilot) control end-to-end over the live WS chain.

Exercises the new pipeline against the running Newton backend:
  session_start -> shape_intent(engage) -> control(0,0) ticks driven by the
  ShapeIntentController -> tip advances along the route (path.progress climbs),
  then waypoint-mode + disengage. Prints a pass/fail summary.
"""

import asyncio
import json
import time

import websockets

URI = "ws://localhost:9000/ws/session"


def _msg(type_name, data):
    return json.dumps({
        "type": type_name,
        "session_id": "",
        "timestamp": int(time.time() * 1000),
        "data": data,
    })


async def recv_until(ws, want, timeout=15.0):
    """Return the data of the next message of type ``want``; answer pings; skip
    other types. Raises on timeout."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        raw = await asyncio.wait_for(ws.recv(), timeout=deadline - time.time())
        m = json.loads(raw)
        t = m.get("type", "")
        if t == "ping":
            await ws.send(_msg("pong", {}))
            continue
        if t == "error":
            print("  [server error]", m.get("data"))
            continue
        if t == want:
            return m.get("data", {})
    raise TimeoutError(f"no '{want}' within {timeout}s")


async def main():
    async with websockets.connect(URI, max_size=8 << 20) as ws:
        await ws.send(_msg("session_start", {
            "phantom": "aorta_tree", "target": "root", "batch_mode": True,
            "n_bodies": 40, "n_substeps": 2,
        }))
        started = await recv_until(ws, "session_started", timeout=60.0)
        print("session_started; routes:", list((started.get("routes") or {}).keys())[:6],
              "..." if started.get("routes") else "")

        # 1) Engage plain centerline autopilot (intent=None) via the new message.
        await ws.send(_msg("shape_intent", {"active": True}))
        ack = await recv_until(ws, "shape_intent")
        print("engage centerline ack:", ack)
        assert ack.get("active") is True and ack.get("mode") == "centerline", ack

        # 2) Drive with neutral control ticks; the controller supplies push/rotate.
        progress = []
        wall_min = 1e9
        waypoints = None
        for i in range(300):
            await ws.send(_msg("control", {"delta_push": 0.0, "delta_rotate": 0.0}))
            batch = await recv_until(ws, "state_batch")
            path = batch.get("path", {})
            if path.get("waypoints"):
                waypoints = path["waypoints"]
            progress.append(float(path.get("progress", 0.0)))
            wall_min = min(wall_min, float(batch.get("safety", {}).get("wall_distance", 1.0)))
            await asyncio.sleep(0.035)  # respect the 30 Hz control rate limit
        p0, pmax, pend = progress[0], max(progress), progress[-1]
        print(f"centerline drive: progress {p0:.3f} -> max {pmax:.3f} (end {pend:.3f}), "
              f"min wall_distance {wall_min*1000:.2f}mm over {len(progress)} ticks")

        # 3) Waypoint mode: aim at a mid-route waypoint, confirm the ack switches.
        wp_ack = None
        if waypoints and len(waypoints) > 4:
            wp = waypoints[int(0.6 * len(waypoints))]
            await ws.send(_msg("shape_intent", {"active": True, "target_waypoint": wp}))
            wp_ack = await recv_until(ws, "shape_intent")
            print("engage waypoint ack:", wp_ack)

        # 4) Disengage -> manual control.
        await ws.send(_msg("shape_intent", {"active": False}))
        off = await recv_until(ws, "shape_intent")
        print("disengage ack:", off)

        ok = (pmax > p0 + 0.05) and (off.get("active") is False) and \
             (wp_ack is None or wp_ack.get("mode") == "waypoint")
        print("\nSMOKE RESULT:", "PASS" if ok else "FAIL",
              f"(advanced {pmax - p0:+.3f} progress)")
        return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
