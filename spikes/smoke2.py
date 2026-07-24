"""Smoke v2: longer autopilot run on a known-good route (endpoint_9), through the
new ShapeIntent pipeline, with containment (safety status) breakdown."""

import asyncio
import json
import time
from collections import Counter

import websockets

URI = "ws://localhost:9000/ws/session"


def _msg(t, d):
    return json.dumps({"type": t, "session_id": "", "timestamp": int(time.time() * 1000), "data": d})


async def recv_until(ws, want, timeout=30.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        m = json.loads(await asyncio.wait_for(ws.recv(), timeout=deadline - time.time()))
        t = m.get("type", "")
        if t == "ping":
            await ws.send(_msg("pong", {})); continue
        if t == "error":
            print("  [err]", m.get("data")); continue
        if t == want:
            return m.get("data", {})
    raise TimeoutError(want)


async def main():
    async with websockets.connect(URI, max_size=8 << 20) as ws:
        await ws.send(_msg("session_start", {"phantom": "aorta_tree", "target": "root",
                                             "batch_mode": True, "n_bodies": 40, "n_substeps": 2}))
        await recv_until(ws, "session_started", timeout=60.0)
        # Route to a D5-PASS branch.
        await ws.send(_msg("select_route", {"target": "endpoint_9"}))
        b = await recv_until(ws, "state_batch")
        wps = b.get("path", {}).get("waypoints", [])
        print("selected endpoint_9; route waypoints:", len(wps))

        await ws.send(_msg("shape_intent", {"active": True}))
        print("engage:", await recv_until(ws, "shape_intent"))

        prog, status = [], Counter()
        for i in range(700):
            await ws.send(_msg("control", {"delta_push": 0.0, "delta_rotate": 0.0}))
            batch = await recv_until(ws, "state_batch")
            prog.append(float(batch.get("path", {}).get("progress", 0.0)))
            status[batch.get("safety", {}).get("status", "?")] += 1
            await asyncio.sleep(0.035)
        print(f"progress {prog[0]:.3f} -> max {max(prog):.3f} (end {prog[-1]:.3f}) over {len(prog)} ticks")
        print("safety status counts:", dict(status))

        # waypoint-mode ack check
        if len(wps) > 4:
            await ws.send(_msg("shape_intent", {"active": True, "target_waypoint": wps[int(0.7 * len(wps))]}))
            print("waypoint ack:", await recv_until(ws, "shape_intent"))
        await ws.send(_msg("shape_intent", {"active": False}))
        print("disengage:", await recv_until(ws, "shape_intent"))


if __name__ == "__main__":
    asyncio.run(main())
