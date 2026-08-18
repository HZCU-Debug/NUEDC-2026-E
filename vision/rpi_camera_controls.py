"""Apply the validated UVC settings used by the Raspberry Pi vision system."""

from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import re
from typing import Dict, Optional, Tuple


# These values were validated with:
# USB CAMERA: USB CAMERA, serial EP.20CC54K01
#
# Keep the automatic controls first. Setting manual exposure or white-balance
# values before disabling their automatic modes can cause the driver to
# overwrite the requested values.
CAMERA_CONTROLS: Dict[str, int] = {
    "white_balance_automatic": 0,
    "auto_exposure": 1,
    "exposure_dynamic_framerate": 0,
    "brightness": 0,
    "contrast": 42,
    "saturation": 64,
    "hue": 0,
    "gamma": 300,
    "gain": 55,
    "power_line_frequency": 1,
    "white_balance_temperature": 4600,
    "sharpness": 70,
    "backlight_compensation": 0,
    "exposure_time_absolute": 320,
    "zoom_absolute": 0,
}

# The built-in/default profile is the original validated camera.  Keep its
# parameters and strictness unchanged when adding a different camera model.
OPTIONAL_CAMERA_CONTROLS = frozenset({"exposure_dynamic_framerate"})


def load_camera_control_profile(
    profile_path: Optional[str] = None,
) -> Tuple[str, Dict[str, int], frozenset]:
    """Load one camera's controls without changing the legacy defaults."""
    if not profile_path:
        return (
            "built_in_old_usb_camera",
            dict(CAMERA_CONTROLS),
            OPTIONAL_CAMERA_CONTROLS,
        )

    path = Path(profile_path)
    try:
        with path.open("r", encoding="utf-8") as profile_file:
            payload = json.load(profile_file)
    except FileNotFoundError as error:
        raise RuntimeError(
            "Camera control profile is missing: {}".format(path)
        ) from error
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(
            "Cannot read camera control profile {}: {}".format(path, error)
        ) from error

    if not isinstance(payload, dict):
        raise RuntimeError("Camera control profile must be a JSON object")
    raw_controls = payload.get("controls")
    if not isinstance(raw_controls, dict) or not raw_controls:
        raise RuntimeError(
            "Camera control profile {} has no controls".format(path)
        )
    controls = {}
    for name, value in raw_controls.items():
        if not isinstance(name, str) or not name.strip():
            raise RuntimeError("Camera control names must be non-empty strings")
        if isinstance(value, bool) or not isinstance(value, int):
            raise RuntimeError(
                "Camera control {} must have an integer value".format(name)
            )
        controls[name] = int(value)

    raw_optional = payload.get("optional_controls", [])
    if not isinstance(raw_optional, list) or not all(
        isinstance(name, str) for name in raw_optional
    ):
        raise RuntimeError("optional_controls must be a list of strings")
    optional = frozenset(raw_optional)
    unknown_optional = optional.difference(controls)
    if unknown_optional:
        raise RuntimeError(
            "Optional controls are absent from controls: {}".format(
                ", ".join(sorted(unknown_optional))
            )
        )
    profile_name = str(payload.get("name") or path.stem)
    return profile_name, controls, optional


def camera_device_path(camera_index: int) -> str:
    return "/dev/video{}".format(int(camera_index))


def _is_unknown_control_error(detail: str) -> bool:
    normalized = str(detail).strip().lower()
    return (
        "unknown control" in normalized
        or "unknown ctrl" in normalized
        or "invalid control" in normalized
    )


def apply_camera_controls(
    camera_index: int,
    *,
    strict: bool = True,
    profile_path: Optional[str] = None,
) -> Dict[str, int]:
    """Apply and verify the fixed UVC controls through ``v4l2-ctl``."""
    profile_name, controls, optional_controls = load_camera_control_profile(
        profile_path
    )
    executable = shutil.which("v4l2-ctl")
    if executable is None:
        message = (
            "v4l2-ctl is required to apply fixed camera controls; "
            "install it with: sudo apt install v4l-utils"
        )
        if strict:
            raise RuntimeError(message)
        print("WARNING:", message)
        return {}

    device = camera_device_path(camera_index)
    applied: Dict[str, int] = {}
    unsupported = set()
    for name, value in controls.items():
        result = subprocess.run(
            [
                executable,
                "-d",
                device,
                "--set-ctrl={}={}".format(name, value),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            message = "Cannot set camera control {}={}: {}".format(
                name,
                value,
                detail or "v4l2-ctl exited {}".format(result.returncode),
            )
            if (
                name in optional_controls
                and _is_unknown_control_error(detail)
            ):
                unsupported.add(name)
                print("WARNING:", message, "(skipped as unsupported)")
                continue
            if strict:
                raise RuntimeError(message)
            print("WARNING:", message)
            continue
    for name, expected in controls.items():
        if name in unsupported:
            continue
        result = subprocess.run(
            [
                executable,
                "-d",
                device,
                "--get-ctrl={}".format(name),
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        match = re.search(
            r":\s*(-?\d+)(?:\s|$)",
            result.stdout.strip(),
        )
        actual = int(match.group(1)) if match else None
        if result.returncode != 0 or actual != expected:
            detail = (result.stderr or result.stdout).strip()
            message = (
                "Camera control verification failed for {}: "
                "expected {}, got {}{}"
            ).format(
                name,
                expected,
                actual,
                " ({})".format(detail) if detail else "",
            )
            if strict:
                raise RuntimeError(message)
            print("WARNING:", message)
            continue
        applied[name] = actual

    expected_count = len(controls) - len(unsupported)
    if strict and len(applied) != expected_count:
        raise RuntimeError(
            "Only {}/{} supported camera controls were applied".format(
                len(applied),
                expected_count,
            )
        )
    print(
        "Fixed camera controls applied: {} profile={} "
        "({} values, {} unsupported)".format(
            device,
            profile_name,
            len(applied),
            len(unsupported),
        )
    )
    return applied
