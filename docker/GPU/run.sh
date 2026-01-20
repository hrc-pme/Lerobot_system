#!/bin/bash
# ===================================================================
# Docker Run Script for lerobot_system with GPU + GUI + UID fix
# ===================================================================

set -e

# -----------------------------
# Configuration
# -----------------------------
IMAGE_NAME="lerobot_system/gpu"
TAG="cu128-humble"
CONTAINER_NAME="lerobot_system-gpu-container"

# Repo root: run.sh 位於 docker/scripts 或類似位置時，往上兩層到專案根目錄
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
MOUNT_PATH_IN_CONTAINER="/home/hrc/Lerobot_system"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}Running lerobot_system Container with GPU${NC}"
echo -e "${BLUE}======================================${NC}"

# -----------------------------
# Check image exists
# -----------------------------
if ! docker image ls --format '{{.Repository}}:{{.Tag}}' | grep -q "^${IMAGE_NAME}:${TAG}$"; then
  echo -e "${RED}Error: Docker image ${IMAGE_NAME}:${TAG} not found${NC}"
  echo -e "${YELLOW}Please build it first (e.g., ./build.sh)${NC}"
  exit 1
fi

# -----------------------------
# Remove existing container
# -----------------------------
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  echo -e "${YELLOW}Stopping and removing existing container...${NC}"
  docker stop "${CONTAINER_NAME}" >/dev/null 2>&1 || true
  docker rm "${CONTAINER_NAME}" >/dev/null 2>&1 || true
fi

# -----------------------------
# GPU args (choose ONE method)
# -----------------------------
GPU_ARGS=""
if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
  echo -e "${GREEN}✓ NVIDIA GPU detected${NC}"
  GPU_ARGS="--gpus all"
elif docker info 2>/dev/null | grep -qi "nvidia"; then
  echo -e "${GREEN}✓ NVIDIA Docker runtime detected${NC}"
  GPU_ARGS="--runtime=nvidia"
else
  echo -e "${YELLOW}Warning: No GPU support detected${NC}"
fi

# -----------------------------
# X11 forwarding (GUI)
# -----------------------------
XSOCK=/tmp/.X11-unix
XAUTH_LIST=$(xauth nlist :0 2>/dev/null | sed -e 's/^..../ffff/' || true)

XAUTH="/tmp/.docker.xauth.$(id -u)"
rm -f "$XAUTH" 2>/dev/null || true

if [ -n "$XAUTH_LIST" ]; then
  touch "$XAUTH" 2>/dev/null || {
    echo -e "${YELLOW}Warning: Cannot create X11 auth file, GUI apps may not work${NC}"
    XAUTH=""
  }
  if [ -n "$XAUTH" ]; then
    chmod 644 "$XAUTH" || true
    echo "$XAUTH_LIST" | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
  fi
else
  echo -e "${YELLOW}Warning: No X11 authentication found, GUI apps may not work${NC}"
  XAUTH=""
fi

XAUTH_VOLUME=""
if [ -n "$XAUTH" ] && [ -f "$XAUTH" ]; then
  XAUTH_VOLUME="--volume=$XAUTH:$XAUTH:rw"
fi

# -----------------------------
# UID/GID from host
# -----------------------------
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"

echo -e "${BLUE}Starting container...${NC}"
echo -e "${YELLOW}Container name: ${GREEN}${CONTAINER_NAME}${NC}"
echo -e "${YELLOW}Image: ${GREEN}${IMAGE_NAME}:${TAG}${NC}"
echo -e "${YELLOW}Host UID/GID: ${GREEN}${HOST_UID}:${HOST_GID}${NC}"
echo -e "${YELLOW}Mount: ${GREEN}${REPO_ROOT} -> ${MOUNT_PATH_IN_CONTAINER}${NC}"

# -----------------------------
# Run container
# -----------------------------
docker run -it \
  --name "${CONTAINER_NAME}" \
  ${GPU_ARGS} \
  --network host \
  --ipc=host \
  --privileged \
  --env "DISPLAY=$DISPLAY" \
  --env "XAUTHORITY=$XAUTH" \
  --env "QT_X11_NO_MITSHM=1" \
  --env "RMW_IMPLEMENTATION=rmw_cyclonedx_cpp" \
  --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-1}" \
  --env "HOST_UID=${HOST_UID}" \
  --env "HOST_GID=${HOST_GID}" \
  --volume "$XSOCK:$XSOCK:rw" \
  ${XAUTH_VOLUME} \
  --volume "/dev:/dev:rw" \
  --volume "${REPO_ROOT}:${MOUNT_PATH_IN_CONTAINER}:rw" \
  --volume "/etc/localtime:/etc/localtime:ro" \
  --user root \
  --workdir "${MOUNT_PATH_IN_CONTAINER}" \
  "${IMAGE_NAME}:${TAG}" \
  /bin/bash -lc "
    set -e

    # 1) 對齊容器內 hrc 的 UID/GID = host UID/GID（解 bind mount 寫入權限）
    if [ \"\$(id -u hrc)\" != \"\$HOST_UID\" ] || [ \"\$(id -g hrc)\" != \"\$HOST_GID\" ]; then
      groupmod -g \$HOST_GID hrc 2>/dev/null || true
      usermod  -u \$HOST_UID -g \$HOST_GID hrc 2>/dev/null || true
    fi

    # 2) 把 mount 進來的 Lerobot_system 目錄 owner 修成 host UID/GID（避免 ros2_ws/src 變 1001）
    chown -R \$HOST_UID:\$HOST_GID ${MOUNT_PATH_IN_CONTAINER} 2>/dev/null || true

    echo '========================================'
    echo 'lerobot_system Development Container'
    echo '========================================'
    echo 'Workspace: ${MOUNT_PATH_IN_CONTAINER}'
    echo 'UID/GID (container hrc): ' \$(id -u hrc):\$(id -g hrc)
    echo 'Listing project root:'
    ls -la ${MOUNT_PATH_IN_CONTAINER} | head -n 50 || true
    echo '========================================'

    # 3) 進入後預設就在 /home/hrc/Lerobot_system
    cd ${MOUNT_PATH_IN_CONTAINER}

    # 4) 以 hrc 身分進入互動 shell
    exec su - hrc

    # FIX: 確保 hrc 用戶的 .bashrc 會自動啟動虛擬環境 (解決 su - 環境重置問題)
    if ! grep -q "/opt/venv/bin/activate" /home/hrc/.bashrc; then
       echo "source /opt/venv/bin/activate" >> /home/hrc/.bashrc
       # 新增：自動 source ROS 和 workspace
       echo "source /opt/ros/humble/setup.bash" >> /home/hrc/.bashrc
       echo "source /home/hrc/Lerobot_system/ros2_ws/install/setup.bash" >> /home/hrc/.bashrc
    fi
  "

echo -e "${GREEN}Container exited${NC}"
