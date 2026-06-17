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

var _socket := WebSocketPeer.new()
var _was_open := false
var session_id: String = ""


func _ready() -> void:
	# Allow project setting override of the endpoint.
	if ProjectSettings.has_setting("network/config/server_url"):
		server_url = str(ProjectSettings.get_setting("network/config/server_url"))
	var err := _socket.connect_to_url(server_url)
	if err != OK:
		push_error("WebSocket connect_to_url failed: %d" % err)


func _process(_delta: float) -> void:
	_socket.poll()
	var state := _socket.get_ready_state()

	match state:
		WebSocketPeer.STATE_OPEN:
			if not _was_open:
				_was_open = true
				connected.emit()
				_send("session_start", {
					"phantom": phantom,
					"target": target,
					"batch_mode": batch_mode,
				})
			while _socket.get_available_packet_count() > 0:
				var packet := _socket.get_packet().get_string_from_utf8()
				_handle_packet(packet)
		WebSocketPeer.STATE_CLOSED:
			if _was_open:
				_was_open = false
				disconnected.emit()


func _handle_packet(packet: String) -> void:
	var parsed: Variant = JSON.parse_string(packet)
	if typeof(parsed) != TYPE_DICTIONARY:
		push_warning("Discarding malformed message: %s" % packet)
		return
	var msg: Dictionary = parsed
	var msg_type := str(msg.get("type", ""))
	var data: Dictionary = msg.get("data", {})

	match msg_type:
		"ping":
			_send("pong", {})
		"session_started":
			session_id = str(msg.get("session_id", ""))
			session_started.emit(session_id, data.get("state", {}))
		"state_update":
			state_received.emit(data)
		"state_batch":
			batch_received.emit(data)
		"path_response":
			path_received.emit(data)
		"error":
			error_received.emit(data)
			push_warning("Server error: %s" % str(data))
		_:
			pass


func send_control(delta_push: float, delta_rotate: float) -> void:
	if not _was_open:
		return
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
