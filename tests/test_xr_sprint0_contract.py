from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_godot_471_entry_and_desktop_xr_split():
    wrapper = read("scripts/godot.ps1")
    validator = read("scripts/validate_godot.ps1")
    project = read("godot_client/project.godot")
    launcher = read("start_godot.bat")

    assert "4.7.1.stable.official.a13da4feb" in wrapper
    assert "CATHSIM_GODOT_EXE" in wrapper
    assert "4.6.3" not in wrapper
    assert "4.7.1.stable.official.a13da4feb" in validator
    assert "Godot_v4.7.1-stable" in validator
    assert "4.6.3" not in validator
    assert "--xr-mode off" in validator
    assert 'run/main_scene="res://scenes/main.tscn"' in project
    assert "openxr/enabled=true" in project
    assert "shaders/enabled=true" in project
    assert "192.168.1.107" not in project
    assert "192.168.1.107" not in launcher
    assert "ws://127.0.0.1:9000/ws/session" in project


def test_xr_scene_is_tracking_only_and_zero_control():
    scene = read("godot_client/scenes/xr/MainXR.tscn")
    bootstrap = read("godot_client/scripts/xr/xr_bootstrap.gd")

    assert 'name="XRMain"' in scene
    for node_type in ("XROrigin3D.new()", "XRCamera3D.new()", "XRController3D.new()"):
        assert node_type in bootstrap
    for tracker in ("left_hand", "right_hand"):
        assert f'&"{tracker}"' in bootstrap
    for pose in ("aim", "grip"):
        assert f'&"{pose}"' in bootstrap
    assert 'XRServer.find_interface(INTERFACE_NAME)' in bootstrap
    assert "_xr_interface.is_initialized()" in bootstrap
    assert "get_viewport().use_xr = true" in bootstrap
    assert "DisplayServer.VSYNC_DISABLED" in bootstrap
    assert '"push": 0.0' in bootstrap
    assert '"rotate": 0.0' in bootstrap
    assert '"deadman_active": false' in bootstrap
    assert '"controls_blocked": true' in bootstrap
    assert "Time.get_unix_time_from_system()" in bootstrap
    assert "[XRBootstrap] status=" in bootstrap
    assert "send_control(" not in bootstrap
    assert "WebSocketClient" not in bootstrap


def test_xr_m0_documents_freeze_safety_and_truthfulness():
    matrix = read("docs/xr/version-matrix.md")
    audit = read("docs/xr/current-capability-audit.md")
    control = read("docs/xr/control-data-contract.md")
    visual = read("docs/xr/visual-baseline.md")
    imaging = read("docs/xr/imaging-boundary.md")
    status = read("docs/xr/sprint0-status.md")

    assert "4.7.1.stable.official.a13da4feb" in matrix
    for blocker in ("OpenJDK", "Android SDK", "Vendors Plugin", "PICO OS"):
        assert blocker in matrix
    assert "不调用 `send_control()`" in audit
    for field in (
        "control_source",
        "input_sequence",
        "client_timestamp_ms",
        "deadman_active",
    ):
        assert field in control
    assert "Fresh | < 200 ms" in control
    assert "Delayed | 200–499 ms" in control
    assert "Stale | ≥ 500 ms" in control
    assert "后端控制看门狗 | 150 ms" in control
    for color in ("#030911", "#4FA6FF", "#65E56F", "#FFD65A", "#FF4B4B"):
        assert color in visual
    assert "急停在所有影像布局中常驻" in visual
    assert "实时状态：未接入" in imaging
    assert "禁止显示 `LIVE`" in imaging
    assert "source=local_3d_camera" in imaging
    assert "动态 SubViewport | 1 个" in imaging
    assert "S0-09 | 完成" in status
    assert "S0-10 | 阻断" in status
