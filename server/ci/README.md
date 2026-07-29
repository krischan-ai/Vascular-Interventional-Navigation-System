# Backend CI/CD host integration

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
  automatic. `cathsim-deploy-bootstrap` must be performed once after candidate
  verification.

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

Do not store SSH, root, GHCR, or runner registration credentials in this
directory.

The VPP case assets are not tracked on this branch. Both Compose files mount `CATHSIM_VPP_ASSETS_HOST` read-only at `/app/data/vpp_assets`; deployment still requires `vpp_ready=true`.
