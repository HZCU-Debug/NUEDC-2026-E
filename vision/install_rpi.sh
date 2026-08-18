#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

sudo apt update
sudo apt install -y \
    build-essential \
    python3-dev \
    python3-numpy \
    python3-opencv \
    python3-pip \
    python3-serial \
    python3-setuptools \
    python3-venv \
    v4l-utils

python3 -m venv --system-site-packages .venv
.venv/bin/python -c "import cv2, numpy, serial, setuptools; print('Python dependencies: OK')"

if .venv/bin/python setup_fast.py build_ext --inplace --force; then
    .venv/bin/python -c "import vision_fast; print('Native extension:', vision_fast.__file__)"
else
    echo "WARNING: vision_fast build failed; Python/OpenCV fallback remains available." >&2
fi

chmod +x install_rpi.sh run_rpi.sh focus_camera.sh verify_rpi.sh
./verify_rpi.sh

echo
echo "Install complete."
echo "List cameras: ./run_rpi.sh --list"
echo "Old USB camera: ./run_rpi.sh old_usb"
echo "RaspberryPi-A camera: ./run_rpi.sh raspberrypi_a"
echo "Use ESP32 Mode0 twice for each camera/mount before Mode1-4."
