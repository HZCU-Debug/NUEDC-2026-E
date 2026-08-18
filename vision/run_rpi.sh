#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
PYTHON_BIN="python3"
if [[ -x ".venv/bin/python" ]]; then
    PYTHON_BIN=".venv/bin/python"
fi

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-wayland}"

if [[ "${1:-}" == "--list" ]]; then
    exec "$PYTHON_BIN" rpi_device_launcher.py --list
fi

DEVICE_NAME="${CAMERA_PROFILE:-old_usb}"
if [[ $# -gt 0 && "${1:0:2}" != "--" ]]; then
    DEVICE_NAME="$1"
    shift
fi

exec "$PYTHON_BIN" rpi_device_launcher.py \
    --device "$DEVICE_NAME" \
    -- "$@"
