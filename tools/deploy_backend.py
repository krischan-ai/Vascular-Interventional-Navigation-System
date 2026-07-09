"""Deploy the backend to the remote simulation server and restart uvicorn.

Usage (PowerShell):

    python tools/deploy_backend.py

Credentials can be kept in the ignored local file ``.deploy_backend.local.json``
or provided through environment variables. The script packages runtime backend
files, uploads them over SFTP, creates a timestamped remote backup, restarts the
server, then checks /api/v1/health.
"""

from __future__ import annotations

import argparse
import getpass
import json
import os
import posixpath
import shlex
import sys
import tempfile
import textwrap
import time
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile

try:
    import paramiko
except ImportError as exc:  # pragma: no cover - depends on operator machine
    raise SystemExit(
        "paramiko is required for SSH deploy. Install it with: python -m pip install paramiko"
    ) from exc


ROOT = Path(__file__).resolve().parents[1]
LOCAL_CONFIG = ROOT / ".deploy_backend.local.json"
DEFAULT_HOST = "192.168.1.107"
DEFAULT_USER = "ps"
DEFAULT_SU_USER = "root"
DEFAULT_REMOTE_PROJECT = "/home/ps/cathsim-warp"
DEFAULT_REMOTE_PYTHON = "/home/ps/anaconda3/envs/cathsim-newton/bin/python"
DEFAULT_PORT = 9000


def load_local_config() -> dict:
    if not LOCAL_CONFIG.is_file():
        return {}
    with LOCAL_CONFIG.open("r", encoding="utf-8") as file_obj:
        data = json.load(file_obj)
    if not isinstance(data, dict):
        raise ValueError(f"Expected JSON object in {LOCAL_CONFIG}")
    return data


def _config_default(config: dict, key: str, env_name: str, fallback):
    if env_name in os.environ:
        return os.environ[env_name]
    return config.get(key, fallback)


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _iter_files(base: Path) -> list[Path]:
    if not base.exists():
        return []
    ignored_parts = {"__pycache__", ".pytest_cache", ".mypy_cache"}
    ignored_suffixes = {".pyc", ".pyo"}
    return [
        path
        for path in base.rglob("*")
        if path.is_file()
        and not any(part in ignored_parts for part in path.parts)
        and path.suffix not in ignored_suffixes
    ]


def build_package(output_dir: Path) -> Path:
    """Create a deploy zip containing backend runtime files and VPP assets."""
    package = output_dir / f"cathsim_backend_{time.strftime('%Y%m%d_%H%M%S')}.zip"
    files: list[Path] = []
    files.extend(_iter_files(ROOT / "services"))
    files.extend(_iter_files(ROOT / "src" / "cathsim"))

    case_dir = ROOT / "data" / "vpp_assets" / "case_001"
    for rel in [
        "manifest.json",
        "graph/graph.json",
        "graph/node_radii.json",
        "derived/routes.json",
        "derived/targets.json",
        "mujoco/case_001_vpp.xml",
    ]:
        path = case_dir / rel
        if path.is_file():
            files.append(path)

    mesh_dir = case_dir / "mujoco" / "meshes" / "case_001"
    files.extend(sorted(mesh_dir.glob("hull_*.stl")))

    with ZipFile(package, "w", ZIP_DEFLATED, compresslevel=6) as zip_file:
        for path in files:
            zip_file.write(path, path.relative_to(ROOT).as_posix())
    return package


def connect(host: str, user: str, password: str, timeout: float) -> paramiko.SSHClient:
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(
        host,
        username=user,
        password=password,
        timeout=timeout,
        look_for_keys=False,
        allow_agent=False,
    )
    return client


def run(
    client: paramiko.SSHClient,
    command: str,
    timeout: int = 60,
    *,
    su_user: str | None = None,
    su_password: str | None = None,
) -> str:
    display_command = command
    if su_user:
        command = f"su {shlex.quote(su_user)} -c {shlex.quote(command)}"
        display_command = f"su {shlex.quote(su_user)} -c <deploy command>"
        if not su_password:
            raise ValueError("su_password is required when su_user is set")

    channel = client.get_transport().open_session()
    channel.settimeout(1.0)
    if su_user:
        channel.get_pty()
    channel.exec_command(command)
    if su_user:
        time.sleep(0.3)
        channel.send(su_password + "\n")

    stdout_chunks: list[bytes] = []
    stderr_chunks: list[bytes] = []
    deadline = time.time() + timeout
    while True:
        if channel.recv_ready():
            stdout_chunks.append(channel.recv(65536))
        if channel.recv_stderr_ready():
            stderr_chunks.append(channel.recv_stderr(65536))
        if channel.exit_status_ready():
            while channel.recv_ready():
                stdout_chunks.append(channel.recv(65536))
            while channel.recv_stderr_ready():
                stderr_chunks.append(channel.recv_stderr(65536))
            break
        if time.time() > deadline:
            channel.close()
            raise TimeoutError(f"remote command timed out after {timeout}s: {display_command}")
        time.sleep(0.1)

    status = channel.recv_exit_status()
    out = b"".join(stdout_chunks).decode("utf-8", "replace")
    err = b"".join(stderr_chunks).decode("utf-8", "replace")
    if status != 0:
        raise RuntimeError(
            f"remote command failed ({status}):\n{display_command}\nSTDOUT:\n{out}\nSTDERR:\n{err}"
        )
    return out + err


def upload(client: paramiko.SSHClient, package: Path, remote_tmp: str) -> None:
    sftp = client.open_sftp()
    try:
        sftp.put(str(package), remote_tmp)
    finally:
        sftp.close()


def remote_install_command(
    *,
    remote_project: str,
    remote_zip: str,
    remote_python: str,
    port: int,
    substeps: int,
    push_speed: float,
) -> str:
    payload = {
        "remote_project": remote_project,
        "remote_zip": remote_zip,
        "remote_python": remote_python,
        "port": port,
        "substeps": substeps,
        "push_speed": push_speed,
    }
    payload_json = json.dumps(payload)
    return (
        f"CATHSIM_DEPLOY_CFG={shlex.quote(payload_json)} "
        f"{shlex.quote(remote_python)} - <<'PY'\n"
        + textwrap.dedent(
            """
            import json
            import os
            import shutil
            import signal
            import subprocess
            import time
            import urllib.request
            from pathlib import Path
            from zipfile import ZipFile

            cfg = json.loads(os.environ["CATHSIM_DEPLOY_CFG"])
            project = Path(cfg["remote_project"])
            remote_zip = Path(cfg["remote_zip"])
            remote_python = cfg["remote_python"]
            port = int(cfg["port"])
            ts = time.strftime("%Y%m%d_%H%M%S")

            if not project.is_dir():
                raise SystemExit(f"remote project not found: {project}")

            backup = project.parent / f"{project.name}_predeploy_{ts}"
            backup.mkdir(parents=True, exist_ok=True)
            for rel in ["services", "src/cathsim", "data/vpp_assets/case_001"]:
                src = project / rel
                if src.exists():
                    dst = backup / rel
                    dst.parent.mkdir(parents=True, exist_ok=True)
                    if src.is_dir():
                        shutil.copytree(src, dst)
                    else:
                        shutil.copy2(src, dst)

            with ZipFile(remote_zip) as zip_file:
                zip_file.extractall(project)

            def cmdline(pid: int) -> str:
                try:
                    raw = Path(f"/proc/{pid}/cmdline").read_bytes()
                except OSError:
                    return ""
                return raw.replace(b"\\x00", b" ").decode("utf-8", "replace")

            my_pid = os.getpid()
            targets = []
            for name in os.listdir("/proc"):
                if not name.isdigit():
                    continue
                pid = int(name)
                if pid == my_pid:
                    continue
                line = cmdline(pid)
                if (
                    "uvicorn" in line
                    and "services.main:app" in line
                    and f"--port {port}" in line
                ):
                    targets.append(pid)

            for pid in targets:
                try:
                    os.kill(pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            time.sleep(2.0)
            for pid in targets:
                if Path(f"/proc/{pid}").exists():
                    try:
                        os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass

            log_path = project / f"backend_{port}.log"
            env = os.environ.copy()
            env.update({
                "CATHSIM_PHYSICS_ENGINE": "newton_demo",
                "CATHSIM_NEWTON_SUBSTEPS": str(cfg["substeps"]),
                "CATHSIM_NEWTON_PUSH_SPEED": str(cfg["push_speed"]),
                "CATHSIM_PORT": str(port),
                "CATHSIM_VPP_DATA_ROOT": str(project / "data" / "vpp_assets"),
            })
            log = log_path.open("w", encoding="utf-8")
            proc = subprocess.Popen(
                [
                    remote_python,
                    "-m",
                    "uvicorn",
                    "services.main:app",
                    "--host",
                    "0.0.0.0",
                    "--port",
                    str(port),
                ],
                cwd=project,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            log.close()

            deadline = time.time() + 30.0
            health = None
            last_error = None
            while time.time() < deadline:
                try:
                    with urllib.request.urlopen(
                        f"http://127.0.0.1:{port}/api/v1/health", timeout=3
                    ) as response:
                        health = json.loads(response.read().decode("utf-8"))
                    break
                except Exception as exc:
                    last_error = exc
                    time.sleep(1.0)

            if health is None:
                tail = ""
                if log_path.exists():
                    tail = "\\n".join(log_path.read_text(errors="replace").splitlines()[-80:])
                raise SystemExit(f"backend failed to start: {last_error}\\n{tail}")

            print(json.dumps({
                "pid": proc.pid,
                "backup": str(backup),
                "health": health,
                "log": str(log_path),
            }, ensure_ascii=False))
            PY
            """
        )
        + "\n"
    )


def parse_args() -> argparse.Namespace:
    config = load_local_config()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--host",
        default=_config_default(config, "host", "CATHSIM_DEPLOY_HOST", DEFAULT_HOST),
    )
    parser.add_argument(
        "--user",
        default=_config_default(config, "user", "CATHSIM_DEPLOY_USER", DEFAULT_USER),
    )
    parser.add_argument(
        "--remote-project",
        default=_config_default(
            config,
            "remote_project",
            "CATHSIM_REMOTE_PROJECT",
            DEFAULT_REMOTE_PROJECT,
        ),
    )
    parser.add_argument(
        "--remote-python",
        default=_config_default(
            config,
            "remote_python",
            "CATHSIM_REMOTE_PYTHON",
            DEFAULT_REMOTE_PYTHON,
        ),
    )
    parser.add_argument(
        "--su-user",
        default=_config_default(config, "su_user", "CATHSIM_DEPLOY_SU_USER", DEFAULT_SU_USER),
    )
    parser.add_argument(
        "--use-su-root",
        action=argparse.BooleanOptionalAction,
        default=_as_bool(
            _config_default(config, "use_su_root", "CATHSIM_DEPLOY_USE_SU_ROOT", False)
        ),
    )
    parser.add_argument(
        "--port",
        type=int,
        default=int(_config_default(config, "port", "CATHSIM_PORT", DEFAULT_PORT)),
    )
    parser.add_argument(
        "--substeps",
        type=int,
        default=int(_config_default(config, "substeps", "CATHSIM_NEWTON_SUBSTEPS", "6")),
    )
    parser.add_argument(
        "--push-speed",
        type=float,
        default=float(
            _config_default(config, "push_speed", "CATHSIM_NEWTON_PUSH_SPEED", "0.05")
        ),
    )
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--keep-package", action="store_true")
    args = parser.parse_args()
    args.deploy_password = os.environ.get(
        "CATHSIM_DEPLOY_PASSWORD", config.get("password")
    )
    args.su_password = os.environ.get(
        "CATHSIM_DEPLOY_SU_PASSWORD", config.get("su_password")
    )
    return args


def main() -> int:
    args = parse_args()
    password = args.deploy_password
    if not password:
        password = getpass.getpass(f"SSH password for {args.user}@{args.host}: ")
    su_password = args.su_password
    if args.use_su_root and not su_password:
        su_password = getpass.getpass(f"su password for {args.su_user}: ")

    with tempfile.TemporaryDirectory(prefix="cathsim_deploy_") as tmp:
        package = build_package(Path(tmp))
        print(f"package: {package.name} ({package.stat().st_size / 1024 / 1024:.2f} MiB)")
        client = connect(args.host, args.user, password, args.timeout)
        try:
            remote_zip = posixpath.join("/home", args.user, package.name)
            print(f"upload: {args.host}:{remote_zip}")
            upload(client, package, remote_zip)
            print("install/restart...")
            command = remote_install_command(
                remote_project=args.remote_project,
                remote_zip=remote_zip,
                remote_python=args.remote_python,
                port=args.port,
                substeps=args.substeps,
                push_speed=args.push_speed,
            )
            output = run(
                client,
                command,
                timeout=300,
                su_user=args.su_user if args.use_su_root else None,
                su_password=su_password if args.use_su_root else None,
            )
            result = json.loads(output.strip().splitlines()[-1])
            print(f"started pid: {result['pid']}")
            print(f"backup: {result['backup']}")
            print(f"log: {result['log']}")
            print(f"health: {json.dumps(result['health'], ensure_ascii=False)}")
            run(client, f"rm -f {remote_zip}", timeout=20)
        finally:
            client.close()

        if args.keep_package:
            saved = ROOT / package.name
            package.replace(saved)
            print(f"kept local package: {saved}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
