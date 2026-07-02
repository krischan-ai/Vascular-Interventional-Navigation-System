"""Quick WebSocket probe: dump state_batch structure + check bodies advance."""
import asyncio, json, time
import numpy as np
import websockets

URL = "ws://localhost:9000/ws/session"


async def main():
    async with websockets.connect(URL, max_size=None) as ws:
        await ws.send(json.dumps({"type": "session_start", "data": {
            "phantom": "aorta_tree", "target": "endpoint_9",
            "route_target": "endpoint_9", "batch_mode": True}}))

        async def push():
            te = time.time() + 3.0
            while time.time() < te:
                await ws.send(json.dumps({"type": "control", "data": {
                    "delta_push": 1.0, "delta_rotate": 0.0}}))
                await asyncio.sleep(0.04)

        pt = asyncio.create_task(push())
        tips = []
        dumped = 0
        te = time.time() + 3.5
        while time.time() < te:
            try:
                m = json.loads(await asyncio.wait_for(ws.recv(), timeout=2.0))
            except asyncio.TimeoutError:
                break
            if m.get("type") != "state_batch":
                continue
            d = m["data"]
            if dumped < 1:
                print("state_batch data keys:", list(d.keys()))
                for k, v in d.items():
                    if isinstance(v, list):
                        head = str(v[0])[:80] if v else ""
                        print("  ", k, "list[", len(v), "] sample=", head)
                    elif isinstance(v, dict):
                        print("  ", k, "dict keys=", list(v.keys())[:8])
                    else:
                        print("  ", k, "=", str(v)[:60])
                dumped += 1
            bodies = d.get("bodies") or []
            if bodies:
                tips.append(np.array(bodies[-1]["pos"], dtype=float))
        await pt
        if tips:
            adv = float(np.linalg.norm(tips[-1] - tips[0]) * 1e3)
            print("BODIES frames=", len(tips), "n_bodies=", len(bodies), "tip_adv_mm=", round(adv, 1))
        else:
            print("NO bodies field in state_batch")


asyncio.run(main())
