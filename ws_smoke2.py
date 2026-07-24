import asyncio, json, numpy as np, websockets
URL="ws://localhost:9000/ws/session"
async def main():
    async with websockets.connect(URL, max_size=None) as ws:
        await ws.send(json.dumps({"type":"session_start","data":{"phantom":"aorta_tree","target":"endpoint_6","batch_mode":True}}))
        async def latest(timeout):
            m=json.loads(await asyncio.wait_for(ws.recv(),timeout))
            return m
        # drain until first state_batch (Newton init may take ~10-20s)
        tip0=None; nb=nwp=0
        for _ in range(200):
            m=await latest(40);
            if m.get("type")=="state_batch":
                d=m["data"]; nb=len(d.get("bodies") or []); nwp=len((d.get("path") or {}).get("waypoints") or [])
                tip0=np.array(d["bodies"][-1]["pos"]); break
        print(f"[start] bodies={nb} waypoints={nwp} tip0={np.round(tip0,3)}")
        def cf_of(d):
            return float((d.get("safety") or {}).get("contact_force", d.get("contact_force",0)) or 0)
        def prog_of(d):
            return (d.get("nav") or {}).get("path_progress", d.get("path_progress",0)) or 0
        # push: interleave send+read
        maxcf=0.0; lastd=None
        for i in range(120):
            await ws.send(json.dumps({"type":"control","data":{"delta_push":1.0,"delta_rotate":0.0}}))
            m=await latest(5)
            if m.get("type")=="state_batch": lastd=m["data"]; maxcf=max(maxcf,cf_of(lastd))
            await asyncio.sleep(0.03)
        tip1=np.array(lastd["bodies"][-1]["pos"])
        print(f"[push] tip moved={np.linalg.norm(tip1-tip0)*1e3:.1f}mm  maxcf={maxcf:.1f}  progress={prog_of(lastd):.3f}")
        # rotate
        tipa=tip1
        for i in range(60):
            await ws.send(json.dumps({"type":"control","data":{"delta_push":0.0,"delta_rotate":1.0}}))
            m=await latest(5)
            if m.get("type")=="state_batch": lastd=m["data"]
            await asyncio.sleep(0.03)
        tipb=np.array(lastd["bodies"][-1]["pos"])
        print(f"[rotate] tip deflected={np.linalg.norm(tipb-tipa)*1e3:.2f}mm")
        print("SMOKE OK")
asyncio.run(main())
