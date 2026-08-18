#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"
PYTHON_BIN="python3"
if [[ -x ".venv/bin/python" ]]; then
    PYTHON_BIN=".venv/bin/python"
fi

"$PYTHON_BIN" -m compileall -q .
"$PYTHON_BIN" -c "import cv2, numpy, serial; print('Runtime imports: OK')"
if "$PYTHON_BIN" -c "import vision_fast; print('Native acceleration: enabled', vision_fast.__file__)"; then
    :
else
    echo "Native acceleration: fallback (runtime is valid but slower)" >&2
fi
"$PYTHON_BIN" rpi_device_launcher.py --list
"$PYTHON_BIN" rpi_device_launcher.py --device old_usb --dry-run
"$PYTHON_BIN" rpi_device_launcher.py --device raspberrypi_a --dry-run
echo "Bundle verification: OK"
