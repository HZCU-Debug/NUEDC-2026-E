"""Launch the Raspberry Pi runtime with a named camera/device profile."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shlex
import sys
from typing import Optional


def load_profiles(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as profile_file:
        data = json.load(profile_file)
    if data.get("schema_version") != 1:
        raise RuntimeError("unsupported device profile schema")
    if not isinstance(data.get("devices"), dict) or not data["devices"]:
        raise RuntimeError("device profile file contains no devices")
    return data


def build_runtime_command(
    profile_path: Path,
    device_name: Optional[str] = None,
    python_executable: Optional[str] = None,
    extra_arguments=(),
):
    profile_path = profile_path.resolve()
    data = load_profiles(profile_path)
    name = device_name or data.get("default_device")
    if name not in data["devices"]:
        raise RuntimeError(
            "unknown device {!r}; available: {}".format(
                name,
                ", ".join(sorted(data["devices"])),
            )
        )
    root = profile_path.parent
    runtime = data.get("runtime", {})
    device = data["devices"][name]
    controls_path = root / device["camera_controls_file"]
    calibration_path = root / device["calibration_file"]
    if not controls_path.is_file():
        raise RuntimeError(
            "camera controls file is missing: {}".format(controls_path)
        )
    calibration_path.parent.mkdir(parents=True, exist_ok=True)
    command = [
        python_executable or sys.executable,
        str(root / "rpi_realtime_detection.py"),
        "--camera",
        str(runtime.get("camera", 0)),
        "--width",
        str(runtime.get("width", 1280)),
        "--height",
        str(runtime.get("height", 720)),
        "--fps",
        str(runtime.get("fps", 30)),
        "--rotate",
        str(runtime.get("rotate", 180)),
        "--detect-every",
        str(runtime.get("detect_every", 3)),
        "--solver-workers",
        str(runtime.get("solver_workers", 3)),
        "--best-effort-after-seconds",
        str(runtime.get("best_effort_after_seconds", 90)),
        "--camera-controls-file",
        str(controls_path),
        "--calibration-file",
        str(calibration_path),
        "--serial-device",
        str(runtime.get("serial_device", "/dev/serial0")),
        "--serial-baud",
        str(runtime.get("serial_baud", 115200)),
    ]
    if runtime.get("web", True):
        command.extend(("--web", "--port", str(runtime.get("port", 8080))))
    command.extend(str(argument) for argument in extra_arguments)
    return name, device, calibration_path, command


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        default=str(Path(__file__).with_name("rpi_device_profiles.json")),
    )
    parser.add_argument("--device", help="named device profile")
    parser.add_argument("--list", action="store_true", help="list devices")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("arguments", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    profile_path = Path(args.config)
    data = load_profiles(profile_path)
    if args.list:
        for name, device in data["devices"].items():
            default = " (default)" if name == data.get("default_device") else ""
            print("{}{}: {}".format(name, default, device.get("description", "")))
        return 0
    extra_arguments = list(args.arguments)
    if extra_arguments[:1] == ["--"]:
        extra_arguments.pop(0)
    name, device, calibration_path, command = build_runtime_command(
        profile_path,
        device_name=args.device,
        extra_arguments=extra_arguments,
    )
    print("Device profile:", name)
    print("Camera controls:", device["camera_controls_file"])
    print("Calibration:", calibration_path)
    if not calibration_path.is_file():
        print("Calibration is not present; use ESP32 Mode0 twice to create it.")
    print(
        "Command:",
        " ".join(shlex.quote(str(argument)) for argument in command),
    )
    if args.dry_run:
        return 0
    os.execv(command[0], command)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
