#!/usr/bin/env bash
set -Eeuo pipefail

IMAGE_NAME=${IMAGE_NAME:-ros2-irs-rascl}
CONTAINER_NAME=${CONTAINER_NAME:-$IMAGE_NAME}

if [[ "${NO_ATTACH:-false}" == "true" ]] || ! docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
  if [[ "${REBUILD:-false}" == "true" ]]; then
    echo "Rebuilding container image without cache"
    docker build --network host --no-cache -t "$IMAGE_NAME" .
  elif [[ "${SOFT_REBUILD:-false}" == "true" ]] || ! docker image inspect "$IMAGE_NAME" >/dev/null 2>&1; then
    echo "Building container image"
    docker build --network host -t "$IMAGE_NAME" .
  else
    echo "Using existing image $IMAGE_NAME (set REBUILD=true after Dockerfile changes)"
  fi

  mkdir -p .devcontainer
  touch .devcontainer/.bash_history

  run_args=(
    run -it --rm
    --name "$CONTAINER_NAME"
    --privileged
    --network=host
    --cap-add CAP_NET_RAW
    --cap-add CAP_NET_ADMIN
    --cap-add CAP_IPC_LOCK
    --cap-add CAP_SYS_NICE
    -v "$PWD:/root/ws"
    -v "$PWD/.devcontainer/.bash_history:/root/.bash_history"
    -e TERM=xterm-256color
    -e QT_X11_NO_MITSHM=1
    -e LIBGL_ALWAYS_SOFTWARE=0
    -e MESA_GL_VERSION_OVERRIDE=3.3
    --log-driver=none
  )

  if grep -qiE 'microsoft|wsl' /proc/version && [[ -d /mnt/wslg/.X11-unix ]]; then
    run_args+=( -v /mnt/wslg/.X11-unix:/tmp/.X11-unix )
  elif [[ -d /tmp/.X11-unix ]]; then
    run_args+=( -v /tmp/.X11-unix:/tmp/.X11-unix )
  fi

  if [[ -n "${DISPLAY:-}" ]]; then
    run_args+=( -e DISPLAY )
  fi
  if [[ -f "${HOME}/.Xauthority" ]]; then
    run_args+=(
      -v "${HOME}/.Xauthority:/root/.Xauthority:ro"
      -e XAUTHORITY=/root/.Xauthority
    )
  fi

  if [[ -n "${XDG_RUNTIME_DIR:-}" && -n "${WAYLAND_DISPLAY:-}" \
        && -S "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}" ]]; then
    run_args+=(
      -v "${XDG_RUNTIME_DIR}/${WAYLAND_DISPLAY}:/run/user/0/${WAYLAND_DISPLAY}"
      -e WAYLAND_DISPLAY
      -e XDG_RUNTIME_DIR=/run/user/0
    )
  fi

  if [[ -e /dev/cpu_dma_latency ]]; then
    run_args+=( --device /dev/cpu_dma_latency )
  fi

  echo "Starting container $CONTAINER_NAME"
  docker "${run_args[@]}" "$IMAGE_NAME"
else
  echo "Attaching to running container $CONTAINER_NAME"
  docker exec -it "$CONTAINER_NAME" bash
fi
