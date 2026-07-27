# syntax=docker/dockerfile:1.7

# Torch 2.4.1 and the validated Newton/Warp environment use CUDA 12.1.
# Keep Python at 3.10 because dm-control/MuJoCo and the RL workspace are
# currently accepted against Python 3.10.20.
ARG CUDA_IMAGE=nvidia/cuda:12.1.1-runtime-ubuntu22.04
FROM ${CUDA_IMAGE}

ARG DEBIAN_FRONTEND=noninteractive
ARG CATHSIM_UID=1000
ARG CATHSIM_GID=1000
ARG APT_MIRROR=https://repo.huaweicloud.com/ubuntu
ARG PIP_INDEX_URL=https://mirrors.huaweicloud.com/repository/pypi/simple

LABEL org.opencontainers.image.title="CathSim Server" \
      org.opencontainers.image.description="FastAPI + Newton/Warp simulation + reinforcement learning" \
      org.opencontainers.image.version="0.1.0"

ENV VIRTUAL_ENV=/opt/cathsim-venv \
    PATH=/opt/cathsim-venv/bin:$PATH \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONPATH=/app:/app/src \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_DEFAULT_TIMEOUT=120 \
    PIP_NO_CACHE_DIR=1 \
    XDG_CACHE_HOME=/home/cathsim/.cache \
    MUJOCO_GL=egl \
    MPLCONFIGDIR=/home/cathsim/.cache/matplotlib \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
    CATHSIM_PHYSICS_ENGINE=newton \
    CATHSIM_VPP_DATA_ROOT=/app/data/vpp_assets \
    CATHSIM_PORT=9000 \
    CATHSIM_NEWTON_SUBSTEPS=6 \
    CATHSIM_NEWTON_ITERS=2 \
    CATHSIM_NEWTON_PUSH_SPEED=0.05

RUN sed -i \
        -e "s|http://archive.ubuntu.com/ubuntu/|${APT_MIRROR}/|g" \
        -e "s|http://security.ubuntu.com/ubuntu/|${APT_MIRROR}/|g" \
        /etc/apt/sources.list \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates \
        libegl1 \
        libgl1 \
        libglib2.0-0 \
        libglfw3 \
        libosmesa6 \
        libspatialindex6 \
        libx11-6 \
        libxext6 \
        libxrender1 \
        passwd \
        python3 \
        python3-pip \
        python3-venv \
        tini \
    && python3 -m venv "$VIRTUAL_ENV" \
    && groupadd --gid "$CATHSIM_GID" cathsim \
    && useradd --uid "$CATHSIM_UID" --gid "$CATHSIM_GID" \
        --create-home --shell /usr/sbin/nologin cathsim \
    && rm -rf /var/lib/apt/lists/*

ENV PIP_INDEX_URL=${PIP_INDEX_URL}

RUN python -m pip install --upgrade "pip<25" setuptools wheel

WORKDIR /app

# Dependency layers remain cached while application code changes.
COPY reinforcement_learning/requirements-backend.txt /tmp/requirements-backend.txt
COPY reinforcement_learning/requirements-train.txt /tmp/requirements-train.txt
RUN python -m pip install \
      -r /tmp/requirements-backend.txt \
      -r /tmp/requirements-train.txt \
    && rm -f /tmp/requirements-backend.txt /tmp/requirements-train.txt

COPY pyproject.toml README.md LICENSE ./
COPY src ./src
COPY services ./services
COPY data ./data
COPY reinforcement_learning ./reinforcement_learning
COPY server ./server

RUN python -m pip install --no-deps --no-build-isolation . \
    && mkdir -p \
        /home/cathsim/.cache/matplotlib \
        /home/cathsim/.cache/warp \
        /app/reinforcement_learning/logs \
        /app/reinforcement_learning/models \
        /app/reinforcement_learning/runs \
    && chown -R cathsim:cathsim \
        /home/cathsim/.cache \
        /app/reinforcement_learning/logs \
        /app/reinforcement_learning/models \
        /app/reinforcement_learning/runs

USER cathsim

EXPOSE 9000

VOLUME ["/app/reinforcement_learning/logs", "/app/reinforcement_learning/models", "/app/reinforcement_learning/runs"]

HEALTHCHECK --interval=30s --timeout=5s --start-period=60s --retries=3 \
  CMD ["python", "-c", "import urllib.request; urllib.request.urlopen('http://127.0.0.1:9000/api/v1/health', timeout=3).read()"]

ENTRYPOINT ["/usr/bin/tini", "--"]
CMD ["python", "-m", "uvicorn", "services.main:app", "--host", "0.0.0.0", "--port", "9000", "--no-access-log"]
