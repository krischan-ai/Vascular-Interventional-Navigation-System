import asyncio, json, numpy as np, websockets
URL="ws://localhost:9000/ws/session"
async def main():
    async with websockets.connect(URL, max_size=None) as ws:
        await ws.send(json.dumps({"type":"session_start","data":{"phantom":"aorta_tree","target":"endpoint_6","batch_mode":True}}))
        async def recv_type(t, timeout):
            for _ in range(400):
                m=json.loads(await asyncio.wait_for(ws.recv(),timeout))
                if m.get("type")==t: return m
        m=await recv_type("state_batch",40); print("[start] bodies=",len(m["data"].get("bodies") or []))
        # send engine_params (live tune) and read the echo
        await ws.send(json.dumps({"type":"engine_params","data":{"bend":6.0,"tip_bend":1.0,"rotate_speed":6.0,"push_speed":0.08}}))
        m=await recv_type("engine_params",10)
        print("[engine_params echo]", {k:round(v,3) for k,v in m["data"]["effective"].items() if k in ("bend","tip_bend","push_speed","rotate_speed")})
        # push a bit to confirm still stepping after live tune
        last=None
        for _ in range(40):
            await ws.send(json.dumps({"type":"control","data":{"delta_push":1.0,"delta_rotate":0.0}}))
            m=await recv_type("state_batch",5); last=m["data"]; await asyncio.sleep(0.03)
        print("[after tune] tip=",np.round(last["bodies"][-1]["pos"],3))
        # rebuild param
        await ws.send(json.dumps({"type":"engine_params","data":{"jtip_deg":45.0}}))
        m=await recv_type("engine_params",10); print("[rebuild echo] jtip_deg=",m["data"]["effective"]["jtip_deg"])
        await ws.send(json.dumps({"type":"control","data":{"delta_push":1.0,"delta_rotate":0.0}}))
        m=await recv_type("state_batch",40); print("[after rebuild] bodies=",len(m["data"].get("bodies") or []))
        print("PARAMS SMOKE OK")
asyncio.run(main())
