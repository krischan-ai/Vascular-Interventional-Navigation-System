extends Node
## WebSocket client for the CathSim VPP backend.
##
## Implements the protocol from doc/03-API与通信协议.md against the FastAPI
## service in services/websocket_handler.py:
##   - sends session_start (with batch_mode) on connect
##   - sends control / reset / path_request
##   - receives session_started / state_update / state_batch / path_response / error
##   - answers ping heartbeats with pong

signal connected
signal disconnected
signal session_started(session_id: String, state: Dictionary)
signal state_received(state: Dictionary)        ## state_update payload
signal batch_received(batch: Dictionary)        ## state_batch payload
signal path_received(path: Dictionary)          ## path_response payload
signal error_received(error: Dictionary)        ## error payload

@export var server_url: String = "ws://localhost:8000/ws/session"
@export var phantom: String = "low_tort"
@export var target: String = "bca"
@export var batch_mode: bool = true
# VPP navigation: when start/end positions (LPS millimeters) are set, the backend
# plans the route, spawns the guidewire at the vessel entry, and streams the path.
@export var case_id: String = "case_001"
@export var start_position: Array = []
@export var end_position: Array = []
@export var smooth: bool = true
# Kinematic centerline-follow: drive the guidewire along the planned path so it
# reliably reaches the target on full-length VPP vessels. Only takes effect when
# a path is planned (low_tort sessions stay physics-driven regardless).
@export var guided: bool = true
# Interactive performance profile: a lighter guidewire and fewer physics
# substeps cut per-step cost ~10-15x for responsive control (full fidelity is
# n_bodies=80, n_substeps=3).
@export var n_bodies: int = 40
@export var n_substeps: int = 2

@export var session_retry_interval: float = 3.0  ## resend session_start until acknowledged
@export var ack_timeout: float = 2.0  ## clear in-flight control if no response arrives

var _socket := WebSocketPeer.new()
var _was_open := false
var session_id: String = ""
var _session_accum := 0.0    ## time since last session_start attempt
var _session_attempts := 0
var _seen_types := {}        ## debug: first-occurrence logging
var _control_sent := false   ## debug: log the first control we send
# Lock-step control: keep at most one control command in flight so the client
# paces itself to the backend's step throughput instead of flooding it.
var _awaiting := false
var _awaiting_since := 0.0


func _ready() -> void:
	# Allow project setting override of the endpoint.
	if ProjectSettings.has_setting("network/config/server_url"):
		server_url = str(ProjectSettings.get_setting("network/config/server_url"))
	# VPP state_batch carries the full planned path (thousands of points) on the
	# first frame, which far exceeds the 64KB default inbound buffer and would
	# otherwise drop the connection. Size up before connecting.
	_socket.inbound_buffer_size = 1 << 22  # 4 MB
	_socket.outbound_buffer_size = 1 << 20  # 1 MB
	_socket.max_queued_packets = 2048
	var err := _socket.connect_to_url(server_url)
	if err != OK:
		push_error("WebSocket connect_to_url failed: %d" % err)


func _process(delta: float) -> void:
	# Safety: if a control response never arrives, release the lock so input is
	# not stuck blocked forever.
	if _awaiting and (Time.get_ticks_msec() / 1000.0 - _awaiting_since) > ack_timeout:
		_awaiting = false

	_socket.poll()
	var state := _socket.get_ready_state()

	match state:
		WebSocketPeer.STATE_OPEN:
			if not _was_open:
				_was_open = true
				connected.emit()
			# (Re)send session_start until the server acknowledges with a
			# session_id. The first packet right after the handshake can be
			# dropped, so we retry on an interval.
			if session_id == "":
				_session_accum += delta
				if _session_attempts == 0 or _session_accum >= session_retry_interval:
					_session_attempts += 1
					_session_accum = 0.0
					print("[WS] sending session_start (attempt %d)" % _session_attempts)
					_send("session_start", _build_session_start())
			while _socket.get_available_packet_count() > 0:
				var packet := _socket.get_packet().get_string_from_utf8()
				_handle_packet(packet)
		WebSocketPeer.STATE_CLOSED:
			if _was_open:
				_was_open = false
				print("[WS] closed code=%d reason='%s'" % [
					_socket.get_close_code(), _socket.get_close_reason()])
				disconnected.emit()


func _build_session_start() -> Dictionary:
	var data := {
		"phantom": phantom,
		"target": target,
		"batch_mode": batch_mode,
		"n_bodies": n_bodies,
		"n_substeps": n_substeps,
		"case_id": case_id,
	}
	# Request server-side path planning + entry alignment only when a VPP route
	# is configured; low_tort sessions omit these and spawn near the origin.
	if start_position.size() == 3 and end_position.size() == 3:
		data["start_position"] = start_position
		data["end_position"] = end_position
		data["smooth"] = smooth
		data["guided"] = guided
	return data


func _handle_packet(packet: String) -> void:
	var parsed: Variant = JSON.parse_string(packet)
	if typeof(parsed) != TYPE_DICTIONARY:
		push_warning("Discarding malformed message: %s" % packet)
		return
	var msg: Dictionary = parsed
	var msg_type := str(msg.get("type", ""))
	var data: Dictionary = msg.get("data", {})

	if not _seen_types.has(msg_type):
		_seen_types[msg_type] = true
		print("[WS] first '%s' message received" % msg_type)

	match msg_type:
		"ping":
			_send("pong", {})
		"session_started":
			session_id = str(msg.get("session_id", ""))
			session_started.emit(session_id, data.get("state", {}))
		"state_update":
			_awaiting = false
			state_received.emit(data)
		"state_batch":
			_awaiting = false
			batch_received.emit(data)
		"path_response":
			path_received.emit(data)
		"error":
			# SESSION_EXISTS can occur from a benign session_start retry after the
			# session was already created; ignore it.
			if str(data.get("code", "")) == "SESSION_EXISTS":
				print("[WS] ignoring benign SESSION_EXISTS")
			else:
				print("[WS] SERVER ERROR: %s" % str(data))
				error_received.emit(data)
				push_warning("Server error: %s" % str(data))
		_:
			pass


func send_control(delta_push: float, delta_rotate: float) -> void:
	# Do not send control until the server has created a session, otherwise the
	# backend replies NO_SESSION for every command (and that spam hides the real
	# session_start result).
	if not _was_open or session_id == "":
		return
	# Lock-step: skip if a control is still awaiting its state response, so we
	# never flood the backend faster than it can step (which causes latency and
	# heartbeat starvation).
	if _awaiting:
		return
	if not _control_sent:
		_control_sent = true
		print("[WS] first control sent (push=%.2f rot=%.2f)" % [delta_push, delta_rotate])
	_awaiting = true
	_awaiting_since = Time.get_ticks_msec() / 1000.0
	_send("control", {
		"delta_push": clampf(delta_push, -1.0, 1.0),
		"delta_rotate": clampf(delta_rotate, -1.0, 1.0),
	})


func send_reset() -> void:
	if _was_open:
		_send("reset", {})


func send_path_request(start_position: Array, end_position: Array,
		case_id: String = "case_001", smooth: bool = true) -> void:
	if _was_open:
		_send("path_request", {
			"case_id": case_id,
			"start_position": start_position,
			"end_position": end_position,
			"smooth": smooth,
		})


func _send(type_name: String, data: Dictionary) -> void:
	var message := {
		"type": type_name,
		"session_id": session_id,
		"timestamp": int(Time.get_unix_time_from_system() * 1000.0),
		"data": data,
	}
	_socket.send_text(JSON.stringify(message))


func _exit_tree() -> void:
	_socket.close()
