#!/usr/bin/env bash
set -Eeuo pipefail

[[ "$(id -u)" -eq 0 ]] || {
  echo "run this installer as root" >&2
  exit 1
}

readonly SOURCE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

install -d -o root -g root -m 0755 /opt/cathsim/compose
install -d -o root -g root -m 0755 /var/lib/cathsim-deploy
install -d -o root -g root -m 0755 /etc/cathsim

install -o root -g root -m 0644 \
  "$SOURCE_DIR/compose.candidate.yaml" \
  /opt/cathsim/compose/compose.candidate.yaml
install -o root -g root -m 0644 \
  "$SOURCE_DIR/compose.production.yaml" \
  /opt/cathsim/compose/compose.production.yaml
install -o root -g root -m 0755 \
  "$SOURCE_DIR/cathsim-deploy" \
  /usr/local/sbin/cathsim-deploy
install -o root -g root -m 0755 \
  "$SOURCE_DIR/cathsim-deploy-diagnostics" \
  /usr/local/sbin/cathsim-deploy-diagnostics

if [[ ! -f /etc/cathsim/backend.env ]]; then
  install -o root -g root -m 0600 \
    "$SOURCE_DIR/backend.env.example" \
    /etc/cathsim/backend.env
fi

cat >/etc/sudoers.d/cathsim-runner <<'EOF'
Defaults!/usr/local/sbin/cathsim-deploy env_keep += "CATHSIM_IMAGE CATHSIM_IMAGE_DIGEST CATHSIM_EXPECTED_SHA CATHSIM_RUN_ID"
cathsim-runner ALL=(root) NOPASSWD: /usr/local/sbin/cathsim-deploy
cathsim-runner ALL=(root) NOPASSWD: /usr/local/sbin/cathsim-deploy-diagnostics
EOF
chmod 0440 /etc/sudoers.d/cathsim-runner
visudo -cf /etc/sudoers.d/cathsim-runner

echo "CD host files installed."
echo "Review /etc/cathsim/backend.env before registering or enabling the runner."
