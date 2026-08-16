#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE_NAME="${IMAGE_NAME:-mars-navigation:noetic}"

docker build -f "$REPO_DIR/docker/Dockerfile.noetic" -t "$IMAGE_NAME" "$REPO_DIR"

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

if [ -n "${DISPLAY:-}" ]; then
    touch "$XAUTH"
    xauth nlist "$DISPLAY" 2>/dev/null | sed -e 's/^..../ffff/' | xauth -f "$XAUTH" nmerge - 2>/dev/null || true
fi

docker run --rm -it \
    --net=host \
    --privileged \
    -e DISPLAY="${DISPLAY:-}" \
    -e XAUTHORITY="$XAUTH" \
    -e OPENAI_API_KEY="${OPENAI_API_KEY:-}" \
    -v "$XSOCK:$XSOCK:rw" \
    -v "$XAUTH:$XAUTH:rw" \
    "$IMAGE_NAME"
