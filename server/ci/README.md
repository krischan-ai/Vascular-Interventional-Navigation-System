# Backend CI/CD host integration

## Image publication channels

The production runner builds the simulation image once and publishes the same
content-addressed image through both channels:

- LAN: `192.168.1.107:5001/siat/cathsim-simulation`
- Remote fallback: `swr.cn-east-3.myhuaweicloud.com/siat/cathsim-simulation`

Every commit receives an immutable `sha-<40-character-git-sha>` tag. The
release aliases start at `v1.1` and `latest`. Deployment prefers the LAN
registry, falls back to SWR, and rejects the artifact unless both registry
pushes produced the same `sha256:` digest.

Configure `SWR_USERNAME` and `SWR_PASSWORD` as GitHub Actions secrets. Do not
store registry credentials in this repository or in `/etc/cathsim/backend.env`.

The repository workflow builds an immutable simulation image tagged with the
full Git commit SHA. A repository-scoped self-hosted runner invokes the
root-owned `/usr/local/sbin/cathsim-deploy` entrypoint.

## Safety model

- Pull requests run tests only on GitHub-hosted runners.
- Production deploys run only after a push reaches the protected upstream
  `feature/backend-rl-training-framework` branch.
- The candidate starts on loopback port 19001 and must pass HTTP health and
  `verify_simulation.py` before production changes.
- The current and previous image references are stored under
  `/var/lib/cathsim-deploy`.
- A failed production health or simulation check restores the previous image.
- The first migration from the legacy root/Conda process is deliberately not
  automatic. In GitHub Actions, manually run `Backend CI/CD` with `deploy` and
  `confirm_first_cutover` checked. Only that explicit workflow dispatch may
  create the one-time production marker after candidate verification.

## One-time host setup

1. Create the unprivileged `cathsim-runner` account.
2. Install a repository-scoped GitHub Actions runner with labels:
   `self-hosted,linux,x64,cathsim-production`.
3. Run `server/ci/install-cd-host.sh` as root.
4. Review `/etc/cathsim/backend.env`.
5. Configure GitHub's `production` environment and protect the deployment
   branch.
6. Perform the legacy-to-container bootstrap during a maintenance window: validate the SHA image on port 19001, stop only the known legacy Uvicorn process bound to port 9000, start `compose.production.yaml`, verify HTTP and WebSocket traffic, record the image in `/var/lib/cathsim-deploy/current-image`, then create `/var/lib/cathsim-deploy/container-production-enabled`.

The bootstrap is intentionally operator-reviewed because the current port 9000 service is a root-owned Conda/Uvicorn process. After that one-time cutover, deployments and rollbacks are automatic.

Do not store SSH, root, SWR, or runner registration credentials in this
directory.

The VPP case assets are not tracked on this branch. Both Compose files mount `CATHSIM_VPP_ASSETS_HOST` read-only at `/app/data/vpp_assets`; deployment still requires `vpp_ready=true`.
