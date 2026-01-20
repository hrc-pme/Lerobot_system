#!/usr/bin/env bash
set -e

IMAGE_NAME=lerobot_system/gpu
TAG=cu128-humble

docker build \
  -t ${IMAGE_NAME}:${TAG} \
  --build-arg LEROBOT_REF=lerobot-system \
  .
