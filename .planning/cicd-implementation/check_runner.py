from __future__ import annotations

import json
from pathlib import Path

from tools.deploy_backend import connect, run


root = Path(__file__).resolve().parents[2]
config = json.loads(
    (root / ".deploy_backend.local.json").read_text(encoding="utf-8")
)
client = connect(config["host"], config["user"], config["password"], 15)
try:
    command = r"""
set +e
echo '=== user ==='
id cathsim-runner
echo '=== files ==='
ls -la /opt/actions-runner-cathsim 2>/dev/null | head -40
echo '=== service ==='
systemctl list-units --type=service --all | grep -i actions.runner
systemctl status 'actions.runner.krischan-ai-cathsim-centerline.*' --no-pager 2>/dev/null | head -60
echo '=== host integration ==='
ls -l /usr/local/sbin/cathsim-deploy* /etc/sudoers.d/cathsim-runner /etc/cathsim/backend.env /opt/cathsim/compose/* 2>/dev/null
echo '=== stage/download ==='
ls -lh /tmp/actions-runner-cathsim.tar.gz /home/ps/.cathsim-ci-* 2>/dev/null
echo '=== production unchanged ==='
ss -ltnp | grep ':9000 '
curl -fsS --max-time 5 http://127.0.0.1:9000/api/v1/health
echo
"""
    print(
        run(
            client,
            command,
            timeout=30,
            su_user=config.get("su_user") if config.get("use_su_root") else None,
            su_password=config.get("su_password") if config.get("use_su_root") else None,
        )
    )
finally:
    client.close()
