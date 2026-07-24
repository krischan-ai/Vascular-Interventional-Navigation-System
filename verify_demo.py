"""End-to-end verification of the Newton minimal demo over the real WebSocket link."""
import asyncio, json, time
import numpy as np
import websockets

URL = "ws://localhost:9000/ws/session"


async def main():
    async with websockets.connect(URL, max_size=None) as ws:
        await ws.send(json.dumps({"type": "session_start", "data": {
            "phantom": "segment_part", "target": "root", "batch_mode": True}}))

        n_bodies = n_wp = 0
        first_tip = last_tip = None
        first_batch_t = None
        frames = 0
        tip_path = []

        # phase 1: collect a few frames at rest to confirm bodies+path
        t_end = time.time() + 2.0
        while time.time() < t_end:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=2.0)
            except asyncio.TimeoutError:
                break
            m = json.loads(raw)
            if m.get("type") == "state_batch":
                d = m["data"]
                bodies = d.get("bodies") or []
                if bodies:
                    n_bodies = len(bodies)
                    tip = np.array(bodies[-1]["pos"], dtype=float)
                    if first_tip is None:
                        first_tip = tip
                        first_batch_t = time.time()
                    last_tip = tip
                    tip_path.append(tip)
                    frames += 1
                wp = (d.get("path") or {}).get("waypoints") or []
                if wp:
                    n_wp = len(wp)

        rest_tip = last_tip.copy() if last_tip is not None else None
        print(f"[rest] n_bodies={n_bodies} n_waypoints={n_wp} frames={frames}")
        print(f"[rest] tip0={np.round(first_tip,4) if first_tip is not None else None}")

        # phase 2: push for ~4s, read frames concurrently
        async def pusher():
            t_end = time.time() + 4.0
            while time.time() < t_end:
                await ws.send(json.dumps({"type": "control", "data": {
                    "delta_push": 1.0, "delta_rotate": 0.0}}))
                await asyncio.sleep(0.04)

        push_task = asyncio.create_task(pusher())
        moved_frames = 0
        t_end = time.time() + 4.5
        finite = True
        while time.time() < t_end:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=2.0)
            except asyncio.TimeoutError:
                break
            m = json.loads(raw)
            if m.get("type") == "state_batch":
                bodies = m["data"].get("bodies") or []
                if bodies:
                    tip = np.array(bodies[-1]["pos"], dtype=float)
                    last_tip = tip
                    tip_path.append(tip)
                    moved_frames += 1
                    arr = np.array([b["pos"] for b in bodies], dtype=float)
                    if not np.isfinite(arr).all():
                        finite = False
        await push_task

        disp = float(np.linalg.norm(last_tip - rest_tip)) if rest_tip is not None else 0.0
        total_path = float(np.sum(np.linalg.norm(np.diff(np.array(tip_path), axis=0), axis=1))) if len(tip_path) > 1 else 0.0
        print(f"[push] frames={moved_frames} finite={finite}")
        print(f"[push] tip after push={np.round(last_tip,4)}")
        print(f"[push] tip displacement under push = {disp*1000:.1f} mm")
        print(f"[push] tip total traversed path     = {total_path*1000:.1f} mm")
        verdict = (n_bodies > 0 and n_wp > 0 and finite and disp > 2e-3)
        print(f"\nVERDICT: {'PASS - Newton physics rod renders and moves under push' if verdict else 'FAIL'}")


asyncio.run(main())
