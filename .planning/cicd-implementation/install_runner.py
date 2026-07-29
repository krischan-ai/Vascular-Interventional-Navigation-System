from __future__ import annotations

import json
import shlex
import subprocess
import uuid
from pathlib import Path

from tools.deploy_backend import connect, run


ROOT = Path(__file__).resolve().parents[2]
WORKTREE = Path(r"D:\tmp\cathsim-cicd-worktree")
REPOSITORY = "krischan-ai/cathsim-centerline"
RUNNER_NAME = "cathsim-production-192-168-1-107"


def gh_json(*args: str):
    return json.loads(
        subprocess.check_output(
            ["gh", "api", *args],
            cwd=WORKTREE,
            text=True,
            encoding="utf-8",
        )
    )


config = json.loads(
    (ROOT / ".deploy_backend.local.json").read_text(encoding="utf-8")
)
downloads = gh_json(f"repos/{REPOSITORY}/actions/runners/downloads")
download = next(
    item
    for item in downloads
    if item["os"] == "linux" and item["architecture"] == "x64"
)
token = gh_json(
    "--method",
    "POST",
    f"repos/{REPOSITORY}/actions/runners/registration-token",
)["token"]

client = connect(config["host"], config["user"], config["password"], 15)
remote_stage = f"/home/{config['user']}/.cathsim-ci-{uuid.uuid4().hex}"
try:
    run(client, f"mkdir -p {shlex.quote(remote_stage)}", timeout=20)
    sftp = client.open_sftp()
    try:
        for source in sorted((WORKTREE / "server" / "ci").iterdir()):
            if source.is_file():
                sftp.put(str(source), f"{remote_stage}/{source.name}")
    finally:
        sftp.close()

    payload = {
        "download_url": download["download_url"],
        "checksum": download["sha256_checksum"],
        "token": token,
        "repository_url": f"https://github.com/{REPOSITORY}",
        "runner_name": RUNNER_NAME,
        "stage": remote_stage,
    }
    command = (
        f"CATHSIM_RUNNER_CFG={shlex.quote(json.dumps(payload))} python3 - <<'PY'\n"
        + r'''
import json
import os
import shutil
import subprocess
from pathlib import Path

cfg = json.loads(os.environ["CATHSIM_RUNNER_CFG"])
user = "cathsim-runner"
runner_dir = Path("/opt/actions-runner-cathsim")
archive = Path("/tmp/actions-runner-cathsim.tar.gz")


def call(*args):
    subprocess.run(args, check=True)


if subprocess.run(
    ["id", user], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
).returncode != 0:
    call("useradd", "--create-home", "--shell", "/bin/bash", user)

call("usermod", "--append", "--groups", "docker", user)
runner_dir.mkdir(parents=True, exist_ok=True)
call("chown", f"{user}:{user}", str(runner_dir))

if not (runner_dir / "config.sh").is_file():
    call("curl", "--fail", "--location", "--output", str(archive), cfg["download_url"])
    actual = subprocess.check_output(["sha256sum", str(archive)], text=True).split()[0]
    if actual != cfg["checksum"]:
        raise SystemExit("GitHub runner checksum mismatch")
    call(
        "tar", "--extract", "--gzip", "--file", str(archive),
        "--directory", str(runner_dir)
    )
    call("chown", "-R", f"{user}:{user}", str(runner_dir))
    archive.unlink(missing_ok=True)

if not (runner_dir / ".runner").is_file():
    call(
        "runuser", "-u", user, "--", str(runner_dir / "config.sh"),
        "--unattended", "--replace",
        "--url", cfg["repository_url"],
        "--token", cfg["token"],
        "--name", cfg["runner_name"],
        "--labels", "cathsim-production",
        "--work", "_work",
    )

if not (runner_dir / ".service").is_file():
    call(str(runner_dir / "svc.sh"), "install", user)
call(str(runner_dir / "svc.sh"), "start")
call("bash", f"{cfg['stage']}/install-cd-host.sh")
shutil.rmtree(cfg["stage"], ignore_errors=True)
print(json.dumps({"runner": cfg["runner_name"], "status": "installed"}))
'''
        + "\nPY\n"
    )
    print(
        run(
            client,
            command,
            timeout=240,
            su_user=config.get("su_user") if config.get("use_su_root") else None,
            su_password=config.get("su_password") if config.get("use_su_root") else None,
        )
    )
finally:
    client.close()
