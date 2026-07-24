import asyncio, json, numpy as np, websockets
URL="ws://localhost:9000/ws/session"
async def main():
    async with websockets.connect(URL, max_size=None) as ws:
        await ws.send(json.dumps({"type":"session_start","data":{"phantom":"aorta_tree","target":"endpoint_6","batch_mode":True}}))
        async def read_one(timeout=3.0):
            while True:
                m=json.loads(await asyncio.wait_for(ws.recv(),timeout))
                if m.get("type")=="state_batch": return m["data"]
        d=await read_one(40.0); nb=len(d.get("bodies") or []); nwp=len((d.get("path") or {}).get("waypoints") or [])
        tip0=np.array(d["bodies"][-1]["pos"]); print(f"[start] bodies={nb} waypoints={nwp} tip0={np.round(tip0,3)}")
        # push forward ~3s
        maxcf=0.0; prog=0.0
        async def drive(pu,ro,secs):
            t=asyncio.get_event_loop().time()
            while asyncio.get_event_loop().time()-t<secs:
                await ws.send(json.dumps({"type":"control","data":{"delta_push":pu,"delta_rotate":ro}}))
                await asyncio.sleep(0.04)
        task=asyncio.create_task(drive(1.0,0.0,3.0))
        t=asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time()-t<3.2:
            d=await read_one()
            cf=d.get("safety",{}).get("contact_force", d.get("contact_force",0)) or 0
            maxcf=max(maxcf,float(cf)); prog=d.get("nav",{}).get("path_progress", d.get("path_progress",prog)) or prog
        await task
        tip1=np.array(d["bodies"][-1]["pos"])
        print(f"[push] tip moved={np.linalg.norm(tip1-tip0)*1e3:.1f}mm  maxcf={maxcf:.1f}  progress={prog}")
        # rotate test
        tipa=np.array((await read_one())["bodies"][-1]["pos"])
        task=asyncio.create_task(drive(0.0,1.0,1.5));
        t=asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time()-t<1.6: d=await read_one()
        await task
        tipb=np.array(d["bodies"][-1]["pos"])
        print(f"[rotate] tip deflected={np.linalg.norm(tipb-tipa)*1e3:.2f}mm")
        print("SMOKE OK")
asyncio.run(main())
