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

DEVICE_NAME="${1:-${CAMERA_PROFILE:-old_usb}}"
case "$DEVICE_NAME" in
    old_usb)
        CONTROLS_FILE="camera_profiles/old_usb_camera.json"
        ;;
    raspberrypi_a)
        CONTROLS_FILE="camera_profiles/raspberrypi_a_camera.json"
        ;;
    *)
        echo "Unknown camera '$DEVICE_NAME'. Use old_usb or raspberrypi_a." >&2
        exit 2
        ;;
esac

exec "$PYTHON_BIN" rpi_focus_preview.py \
    --camera 0 \
    --width 1280 \
    --height 720 \
    --fps 30 \
    --camera-controls-file "$CONTROLS_FILE"
