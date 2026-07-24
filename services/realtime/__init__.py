"""Real-time control link: autonomous physics stepping + state streaming.

Phase B of doc/07 §七: decouple the physics step rate from the client frame rate.
A :class:`~services.realtime.physics_worker.PhysicsWorker` steps the engine
continuously in its own thread (reading a latest-input buffer), and the WebSocket
handler pushes the most recent frame to the client at a fixed rate; the client
interpolates between physics frames for smooth 60fps rendering.

This buys *visual* smoothness, not control *fidelity*: the physics is still low
frequency (~3Hz for aorta_trunk), so the control-latency floor remains.
"""

from services.realtime.physics_worker import Frame, PhysicsWorker

__all__ = ["Frame", "PhysicsWorker"]
