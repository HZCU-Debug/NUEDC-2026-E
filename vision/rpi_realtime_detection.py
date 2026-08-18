"""Raspberry Pi USB-camera live A4 and puzzle-piece detector."""

import argparse
import json
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

import fragment_vision as vision
from rpi_camera_controls import apply_camera_controls, camera_device_path
from rpi_calibrate_fixed import (
    make_preview as make_calibration_preview,
    save_result as save_calibration_result,
)
from esp32_puzzle_link import (
    Link,
    PIECE_COUNT_MESSAGE,
    PIECE_DATA_MESSAGE,
    PIECE_FORMAT,
    ReliablePuzzleSender,
    build_result_messages,
    is_cancel_request,
    parse_mode_request,
)
from vision_state_machine import VisionStateMachine
from puzzle_solver import native_acceleration_available


class InlineBlueCalibration:
    """Track a stable blue-card calibration inside the realtime loop."""

    required_stable_frames = 2
    maximum_corner_drift_px = 5.0
    maximum_hue_drift = 3

    def __init__(self):
        self.reset()

    def reset(self):
        self.stable_frames = 0
        self.previous_points = None
        self.previous_hue = None
        self.latest = None

    @property
    def ready(self):
        return (
            self.latest is not None
            and self.stable_frames >= self.required_stable_frames
        )

    def invalidate(self):
        self.stable_frames = 0
        self.previous_points = None
        self.previous_hue = None
        self.latest = None

    def observe(self, frame):
        mask, rectified, calibration = (
            vision.create_upper_half_fixed_calibration(
                frame,
                vision.DetectConfig(),
            )
        )
        points = np.asarray(
            calibration["calibration_points_px"],
            dtype=np.float32,
        )
        hue = int(calibration["blue_hue"])
        stable = False
        if self.previous_points is not None:
            drift = np.linalg.norm(
                points - self.previous_points,
                axis=1,
            )
            hue_distance = abs(hue - self.previous_hue)
            hue_distance = min(hue_distance, 180 - hue_distance)
            stable = (
                float(np.max(drift))
                <= self.maximum_corner_drift_px
                and hue_distance <= self.maximum_hue_drift
            )
        self.stable_frames = self.stable_frames + 1 if stable else 1
        self.previous_points = points
        self.previous_hue = hue
        self.latest = (
            frame.copy(),
            mask.copy(),
            rectified.copy(),
            dict(calibration),
        )
        preview = make_calibration_preview(frame, calibration)
        cv2.putText(
            preview,
            "INLINE MODE 0 stable={}/{}".format(
                min(
                    self.stable_frames,
                    self.required_stable_frames,
                ),
                self.required_stable_frames,
            ),
            (18, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return preview

    def save(self, output_path):
        if not self.ready:
            raise RuntimeError(
                "No stable blue-card calibration is ready to save"
            )
        frame, mask, rectified, calibration = self.latest
        return save_calibration_result(
            output_path,
            frame,
            mask,
            rectified,
            calibration,
            next_step_message=False,
        )


class AutomaticSolveRecovery:
    """Keep one ESP32 task active while vision retries fresh stable frames."""

    def __init__(self, retry_delay_seconds=0.75):
        self.retry_delay_seconds = max(0.0, float(retry_delay_seconds))
        self.reset()

    def reset(self):
        self.failure_count = 0
        self.retry_not_before = 0.0
        self.task_started_at = None
        self.best_candidate = None

    def begin_task(self, now=None):
        self.failure_count = 0
        self.retry_not_before = 0.0
        self.best_candidate = None
        self.task_started_at = (
            time.monotonic() if now is None else float(now)
        )

    def elapsed(self, now=None):
        if self.task_started_at is None:
            return 0.0
        timestamp = time.monotonic() if now is None else float(now)
        return max(0.0, timestamp - self.task_started_at)

    def deadline_reached(self, timeout_seconds, now=None):
        timeout_seconds = max(0.0, float(timeout_seconds))
        return (
            self.task_started_at is not None
            and self.elapsed(now=now) >= timeout_seconds
        )

    def consider_candidate(self, candidate):
        if candidate is None:
            return False
        if (
            self.best_candidate is None
            or tuple(candidate["quality_key"])
            < tuple(self.best_candidate["quality_key"])
        ):
            self.best_candidate = candidate
            return True
        return False

    def record_failure(self, now=None):
        self.failure_count += 1
        timestamp = time.monotonic() if now is None else float(now)
        self.retry_not_before = timestamp + self.retry_delay_seconds
        return self.failure_count

    def ready(self, now=None):
        timestamp = time.monotonic() if now is None else float(now)
        return timestamp >= self.retry_not_before


def mode3_beam_levels_for_retry(
    failure_count,
    high_beam_after_failures=2,
):
    """Use fresh low-beam frames before escalating one difficult frame."""
    failure_count = max(0, int(failure_count))
    high_beam_after_failures = max(0, int(high_beam_after_failures))
    if failure_count >= high_beam_after_failures:
        return (40, 400, 1600)
    return (40, 400)


def piece_geometry_is_stable(
    previous_payload,
    current_payload,
    maximum_centroid_drift_mm=1.0,
    maximum_vertex_drift_mm=1.5,
    maximum_edge_drift_mm=1.5,
):
    """Require stable measured geometry, not only a stable piece count."""
    if previous_payload is None or current_payload is None:
        return False
    previous_pieces = previous_payload.get("pieces", [])
    current_pieces = current_payload.get("pieces", [])
    if len(previous_pieces) != len(current_pieces):
        return False

    for previous, current in zip(previous_pieces, current_pieces):
        previous_vertices = np.asarray(
            previous.get("vertices_mm", []),
            dtype=np.float32,
        ).reshape(-1, 2)
        current_vertices = np.asarray(
            current.get("vertices_mm", []),
            dtype=np.float32,
        ).reshape(-1, 2)
        if (
            len(previous_vertices) < 3
            or previous_vertices.shape != current_vertices.shape
        ):
            return False
        previous_centroid = np.asarray(
            previous.get("centroid_mm", []),
            dtype=np.float32,
        )
        current_centroid = np.asarray(
            current.get("centroid_mm", []),
            dtype=np.float32,
        )
        if previous_centroid.shape != (2,) or current_centroid.shape != (2,):
            return False
        if float(np.linalg.norm(current_centroid - previous_centroid)) > float(
            maximum_centroid_drift_mm
        ):
            return False

        # Compare coordinates relative to each centroid so a tiny global
        # calibration translation does not look like a shape change.
        previous_relative = previous_vertices - previous_centroid
        current_relative = current_vertices - current_centroid
        direct_drift = np.linalg.norm(
            current_relative - previous_relative,
            axis=1,
        )
        reverse_drift = np.linalg.norm(
            current_relative - previous_relative[::-1],
            axis=1,
        )
        if min(float(np.max(direct_drift)), float(np.max(reverse_drift))) > float(
            maximum_vertex_drift_mm
        ):
            return False

        previous_edges = np.sort(
            np.linalg.norm(
                np.roll(previous_vertices, -1, axis=0) - previous_vertices,
                axis=1,
            )
        )
        current_edges = np.sort(
            np.linalg.norm(
                np.roll(current_vertices, -1, axis=0) - current_vertices,
                axis=1,
            )
        )
        if float(np.max(np.abs(current_edges - previous_edges))) > float(
            maximum_edge_drift_mm
        ):
            return False
    return True


class TaskCancelled(RuntimeError):
    """Raised when ESP32 cancels the active vision/motion task."""


class BestEffortDeadlineReached(RuntimeError):
    """Raised to stop full solving and switch to safe best effort."""


def save_failure_diagnostics(
    frame,
    config,
    error,
    phase,
    mode,
    attempt=None,
):
    """Overwrite a compact diagnostic bundle for the latest failure."""
    paths = {
        "raw": "rpi_failure_raw.png",
        "background_mask": "rpi_failure_background_mask.png",
        "divider_mask": "rpi_failure_divider_mask.png",
        "overlay": "rpi_failure_overlay.jpg",
        "json": "rpi_failure.json",
    }
    metadata = {
        "captured_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "phase": str(phase),
        "mode": int(mode),
        "error": str(error),
        "frame_shape": list(frame.shape),
        "config": dict(vars(config)),
        "files": paths,
    }
    if attempt is not None:
        metadata["attempt"] = int(attempt)
    cv2.imwrite(paths["raw"], frame)
    overlay = frame.copy()
    try:
        background, background_hue = (
            vision.build_blue_background_mask(frame, config)
        )
        metadata["background_hue"] = int(background_hue)
        cv2.imwrite(paths["background_mask"], background)
        contours, _ = cv2.findContours(
            background,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE,
        )
        frame_area = float(frame.shape[0] * frame.shape[1])
        contour_records = []
        for index, contour in enumerate(
            sorted(contours, key=cv2.contourArea, reverse=True)[:8]
        ):
            area = float(cv2.contourArea(contour))
            x, y, width, height = cv2.boundingRect(contour)
            contour_records.append(
                {
                    "index": index,
                    "area_px2": round(area, 1),
                    "area_ratio": round(area / frame_area, 5),
                    "bounding_rect": [x, y, width, height],
                }
            )
            cv2.drawContours(
                overlay,
                [contour],
                -1,
                (0, 0, 255) if index < 2 else (0, 255, 255),
                3,
            )
            cv2.putText(
                overlay,
                "G{}".format(index),
                (x + 5, y + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )
        metadata["green_contours"] = contour_records

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        value = hsv[:, :, 2]
        paper_values = value[background > 0]
        if paper_values.size:
            paper_value = float(np.median(paper_values))
            dark_limit = int(
                np.clip(
                    paper_value * float(config.divider_value_ratio),
                    35.0,
                    155.0,
                )
            )
            divider_mask = (
                (value <= dark_limit)
                & (background == 0)
            ).astype(np.uint8) * 255
            cv2.imwrite(paths["divider_mask"], divider_mask)
            metadata["paper_value_median"] = round(paper_value, 2)
            metadata["divider_dark_limit"] = dark_limit
            metadata["divider_dark_pixels"] = int(
                np.count_nonzero(divider_mask)
            )
    except Exception as diagnostic_error:
        metadata["diagnostic_error"] = str(diagnostic_error)

    cv2.putText(
        overlay,
        "{} MODE {}: {}".format(phase, mode, str(error))[:110],
        (12, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.52,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(paths["overlay"], overlay)
    with open(paths["json"], "w", encoding="utf-8") as output_file:
        json.dump(
            metadata,
            output_file,
            ensure_ascii=False,
            indent=2,
        )
    history_path = "rpi_failure_history.jsonl"
    with open(history_path, "a", encoding="utf-8") as history_file:
        history_file.write(
            json.dumps(metadata, ensure_ascii=False, separators=(",", ":"))
            + "\n"
        )
    print(
        "\nSaved failure diagnostics: {}; history={}".format(
            ", ".join(paths.values()),
            history_path,
        )
    )


class PreviewState:
    def __init__(self):
        self.condition = threading.Condition()
        self.jpeg = None
        self.frame_id = 0

    def update(self, frame):
        success, encoded = cv2.imencode(
            ".jpg",
            frame,
            [cv2.IMWRITE_JPEG_QUALITY, 82],
        )
        if not success:
            return
        with self.condition:
            self.jpeg = encoded.tobytes()
            self.frame_id += 1
            self.condition.notify_all()

    def wait_next(self, previous_id):
        with self.condition:
            self.condition.wait_for(
                lambda: self.frame_id != previous_id,
                timeout=1.0,
            )
            return self.frame_id, self.jpeg


def make_preview_handler(preview_state):
    class PreviewHandler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path in ("/", "/index.html"):
                page = (
                    "<!doctype html><html><head>"
                    "<meta charset='utf-8'>"
                    "<meta name='viewport' "
                    "content='width=device-width,initial-scale=1'>"
                    "<title>Raspberry Pi Vision</title>"
                    "<style>body{margin:0;background:#111;color:#eee;"
                    "font-family:sans-serif;text-align:center}"
                    "img{max-width:100vw;max-height:92vh}"
                    "</style></head><body>"
                    "<h3>Raspberry Pi puzzle vision</h3>"
                    "<img src='/stream.mjpg'></body></html>"
                ).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
                return

            if self.path == "/stream.mjpg":
                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame",
                )
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                frame_id = -1
                try:
                    while True:
                        frame_id, jpeg = preview_state.wait_next(
                            frame_id
                        )
                        if jpeg is None:
                            continue
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(
                            b"Content-Type: image/jpeg\r\n"
                        )
                        self.wfile.write(
                            "Content-Length: {}\r\n\r\n".format(
                                len(jpeg)
                            ).encode("ascii")
                        )
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
                return

            self.send_error(404)

        def log_message(self, *_):
            return

    return PreviewHandler


def start_preview_server(preview_state, port):
    server = ThreadingHTTPServer(
        ("0.0.0.0", port),
        make_preview_handler(preview_state),
    )
    thread = threading.Thread(
        target=server.serve_forever,
        daemon=True,
    )
    thread.start()
    return server


def open_camera(
    index,
    width,
    height,
    fps,
    apply_fixed_controls=True,
    camera_controls_file=None,
):
    device = camera_device_path(index)
    camera = cv2.VideoCapture(device, cv2.CAP_V4L2)
    camera.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG"),
    )
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_FPS, fps)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not camera.isOpened():
        raise RuntimeError(
            "Cannot open {}".format(device)
        )
    if apply_fixed_controls:
        apply_camera_controls(
            index,
            profile_path=camera_controls_file,
        )
    return camera


def rotate_frame(frame, angle):
    if angle == 90:
        return cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    if angle == 180:
        return cv2.rotate(frame, cv2.ROTATE_180)
    if angle == 270:
        return cv2.rotate(
            frame,
            cv2.ROTATE_90_COUNTERCLOCKWISE,
        )
    return frame


def fit_window(frame, maximum_width, maximum_height):
    height, width = frame.shape[:2]
    scale = min(
        1.0,
        maximum_width / float(width),
        maximum_height / float(height),
    )
    if scale >= 1.0:
        return frame
    return cv2.resize(
        frame,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_AREA,
    )


def paste_fit(canvas, frame, left, top, width, height):
    if frame is None:
        return
    source_height, source_width = frame.shape[:2]
    scale = min(
        width / float(source_width),
        height / float(source_height),
    )
    resized = cv2.resize(
        frame,
        (
            max(1, int(round(source_width * scale))),
            max(1, int(round(source_height * scale))),
        ),
        interpolation=(
            cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
        ),
    )
    y = top + (height - resized.shape[0]) // 2
    x = left + (width - resized.shape[1]) // 2
    canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized


def make_landscape_dashboard(
    raw_frame,
    detection,
    solution,
    mode,
    stable_frames,
    required_frames,
    solve_status,
):
    canvas = np.full((720, 1280, 3), 20, dtype=np.uint8)
    right_frame = solution if solution is not None else detection
    paste_fit(canvas, raw_frame, 10, 62, 620, 648)
    paste_fit(canvas, right_frame, 650, 62, 620, 648)
    cv2.putText(
        canvas,
        "MODE {}  AUTO {}/{}  {}".format(
            mode,
            min(stable_frames, required_frames),
            required_frames,
            solve_status,
        ),
        (18, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 220, 0),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "CAMERA",
        (18, 88),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
    )
    cv2.putText(
        canvas,
        "SOLUTION" if solution is not None else "DETECTION",
        (658, 88),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 255, 255),
        2,
    )
    return canvas


def make_terminal_progress(mode, cancel_check=None):
    started = time.perf_counter()
    width = 30

    def update(progress, message):
        if cancel_check is not None and cancel_check():
            raise TaskCancelled("ESP32 requested task cancellation")
        progress = max(0.0, min(1.0, float(progress)))
        filled = min(width, int(round(progress * width)))
        bar = "#" * filled + "-" * (width - filled)
        elapsed = time.perf_counter() - started
        print(
            "\rSOLVING mode={} [{}] {:3d}%  {:<42} {:6.1f}s".format(
                mode,
                bar,
                int(round(progress * 100.0)),
                str(message)[:42],
                elapsed,
            ),
            end=("\n" if progress >= 1.0 else ""),
            flush=True,
        )

    return update


def send_plan_reliably(link, sender, actions):
    """Freeze vision and deliver count/pieces with strict stop-and-wait."""
    messages = build_result_messages(actions)

    def describe(message):
        message_type, payload = message
        if message_type == PIECE_COUNT_MESSAGE:
            return "COUNT={}".format(payload[0])
        if message_type == PIECE_DATA_MESSAGE:
            values = struct.unpack(PIECE_FORMAT, payload)
            return (
                "PIECE id={} S=({},{})mm T_PICK=({},{})mm "
                "R={:+.2f}deg"
            ).format(
                values[0],
                values[1],
                values[2],
                values[3],
                values[4],
                values[5] / 100.0,
            )
        return "TYPE=0x{:02X} length={}".format(
            message_type,
            len(payload),
        )

    sender.start(actions)
    last_delivered = 0
    ignored_modes = 0
    packet_started = time.perf_counter()
    retry_started = link.retransmission_count
    print("TX 1/{} {}".format(len(messages), describe(messages[0])))
    while not sender.complete:
        event = link.poll()
        if is_cancel_request(event):
            sender.clear()
            raise TaskCancelled(
                "ESP32 cancelled while result packets were being sent"
            )
        if parse_mode_request(event) is not None:
            ignored_modes += 1
        sender.step(event)
        if sender.delivered_count != last_delivered:
            now = time.perf_counter()
            ack_ms = (now - packet_started) * 1000.0
            retries = link.retransmission_count - retry_started
            last_delivered = sender.delivered_count
            print(
                "ACK {}/{} {:.1f}ms retries={}".format(
                    sender.delivered_count,
                    sender.total_count,
                    ack_ms,
                    retries,
                ),
                flush=True,
            )
            if sender.delivered_count < len(messages):
                print(
                    "TX {}/{} {}".format(
                        sender.delivered_count + 1,
                        len(messages),
                        describe(messages[sender.delivered_count]),
                    )
                )
                packet_started = now
                retry_started = link.retransmission_count
        # poll() must run much faster than the 50 ms retry interval.
        time.sleep(0.001)
    print(
        "ESP32 TX [{}/{} ACK] complete".format(
            sender.delivered_count,
            sender.total_count,
        )
    )
    sender.clear()
    return ignored_modes


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--rotate",
        type=int,
        choices=(0, 90, 180, 270),
        default=180,
        help="clockwise image rotation before detection",
    )
    parser.add_argument("--detect-every", type=int, default=3)
    parser.add_argument(
        "--solver-workers",
        type=int,
        default=3,
        help=(
            "Mode 2/3 overlap and Mode 4 scoring workers "
            "(default: 3 on Raspberry Pi)"
        ),
    )
    parser.add_argument(
        "--skip-camera-controls",
        action="store_true",
        help="do not apply the saved fixed UVC camera controls",
    )
    parser.add_argument(
        "--camera-controls-file",
        default=None,
        help=(
            "camera-specific UVC control profile JSON; by default use "
            "the original validated camera settings"
        ),
    )
    parser.add_argument(
        "--mode",
        type=int,
        choices=(1, 2, 3, 4),
        default=1,
        help="puzzle mode",
    )
    parser.add_argument(
        "--manual",
        action="store_true",
        help="disable automatic solve and wait for P",
    )
    parser.add_argument(
        "--solve-retry-delay",
        type=float,
        default=0.75,
        help=(
            "seconds before automatically capturing fresh stable frames "
            "after a solve failure"
        ),
    )
    parser.add_argument(
        "--best-effort-after-seconds",
        type=float,
        default=90.0,
        help=(
            "after this many seconds of automatic retries, send a safe "
            "compact transport plan instead of waiting forever "
            "(default: 90)"
        ),
    )
    parser.add_argument(
        "--mode3-high-beam-after-failures",
        type=int,
        default=2,
        help=(
            "number of failed fresh Mode 3 frames before enabling "
            "beam=1600 (default: 2)"
        ),
    )
    parser.add_argument(
        "--web",
        action="store_true",
        help="show preview through a browser instead of cv2.imshow",
    )
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument(
        "--serial-device",
        help=(
            "ESP32 serial device, normally /dev/serial0; "
            "when set, modes are selected by reliable type 0x03 messages"
        ),
    )
    parser.add_argument(
        "--serial-baud",
        type=int,
        default=115200,
    )
    parser.add_argument(
        "--calibration-file",
        default="rpi_camera_calibration.json",
        help=(
            "saved fixed-camera calibration created by "
            "rpi_calibrate_fixed.py"
        ),
    )
    parser.add_argument(
        "--legacy-live-calibration",
        action="store_true",
        help=(
            "ignore the saved calibration and use legacy green A4 "
            "three-corner calibration"
        ),
    )
    args = parser.parse_args()

    serial_connection = None
    esp_link = None
    result_sender = None
    pending_mode = None
    ignored_busy_modes = 0
    mode_input_armed = True
    mode_quiet_started = time.monotonic()
    # Prevent stale retransmissions from the completed task from being
    # interpreted as a new user command.
    mode_rearm_quiet_seconds = 1.0
    current_mode = args.mode
    command_active = args.serial_device is None
    if args.serial_device:
        try:
            import serial
        except ImportError as error:
            raise RuntimeError(
                "pyserial is required: pip install pyserial"
            ) from error
        serial_connection = serial.Serial(
            args.serial_device,
            args.serial_baud,
            timeout=0,
        )
        esp_link = Link(serial_connection)
        result_sender = ReliablePuzzleSender(esp_link)

    fixed_calibration = None
    if not args.legacy_live_calibration:
        try:
            with open(
                args.calibration_file,
                "r",
                encoding="utf-8",
            ) as calibration_file:
                fixed_calibration = json.load(calibration_file)
        except FileNotFoundError as error:
            if args.serial_device:
                print(
                    "No fixed calibration file yet: {}. "
                    "Waiting for ESP32 MODE 0 calibration.".format(
                        args.calibration_file
                    )
                )
            else:
                raise RuntimeError(
                    "Fixed camera calibration is missing: {}. "
                    "Use ESP32 MODE 0 or run "
                    "rpi_calibrate_fixed.py first.".format(
                        args.calibration_file
                    )
                ) from error
        if fixed_calibration is not None and (
            fixed_calibration.get("calibration_method")
            != "fixed_upper_half_blue"
        ):
            raise RuntimeError(
                "Unsupported calibration file: {}".format(
                    args.calibration_file
                )
            )
    config = vision.DetectConfig(
        require_divider_calibration=False,
        calibration_strategy="three_corners",
        fixed_calibration=fixed_calibration,
    )
    inline_calibration = InlineBlueCalibration()
    calibration_active = False
    camera = open_camera(
        args.camera,
        args.width,
        args.height,
        args.fps,
        apply_fixed_controls=not args.skip_camera_controls,
        camera_controls_file=args.camera_controls_file,
    )

    # No lens-undistortion step is used for this USB camera.
    last_detection = None
    last_payload = None
    frozen_frame = None
    frozen_detection = None
    frozen_payload = None
    last_error = ""
    solution_preview = None
    stable_piece_count = 0
    stable_frames = 0
    solve_attempted = False
    solve_recovery = AutomaticSolveRecovery(args.solve_retry_delay)
    solve_status = (
        "WAIT_CMD" if args.serial_device else "WAIT"
    )
    last_failure_save_at = 0.0
    frame_id = 0
    preview_state = PreviewState() if args.web else None
    preview_server = None
    if args.web:
        preview_server = start_preview_server(
            preview_state,
            args.port,
        )
        print(
            "Web preview: http://<raspberry-pi-ip>:{}".format(
                args.port
            )
        )

    for _ in range(30):
        camera.read()

    print("Realtime detection started")
    print(
        "Native acceleration: {}".format(
            "enabled" if native_acceleration_available() else "fallback"
        )
    )
    print(
        "Solver workers: {} (Mode 2/3 overlap + Mode 4 scoring)".format(
            args.solver_workers
        )
    )
    if fixed_calibration is not None:
        print(
            "Fixed calibration loaded: {} "
            "(blue upper half X=105..210mm, Y=0..297mm)".format(
                args.calibration_file
            )
        )
    elif args.legacy_live_calibration:
        print("WARNING: using legacy live green A4 calibration")
    else:
        print(
            "WAITING FOR ESP32 MODE 0 TO CREATE FIXED CALIBRATION"
        )
    print(
        "Mode {}: auto_solve={}, P=solve, R=restart, "
        "S=save, Q/ESC=quit".format(current_mode, not args.manual)
    )
    print(
        "Workspace direction: source=physical right "
        "(canonical lower half), target=physical left "
        "(canonical upper half)"
    )
    if args.serial_device:
        print(
            "ESP32 control: {} at {} baud; waiting for "
            "reliable TYPE=0x03 PAYLOAD=0|1|2|3|4|255(CANCEL)".format(
                args.serial_device,
                args.serial_baud,
            )
        )
    if not args.web:
        cv2.namedWindow("Puzzle Vision", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Puzzle Vision", 1280, 720)

    try:
        while True:
            auto_solve_requested = False
            if esp_link is not None and result_sender is not None:
                link_event = esp_link.poll()
                if is_cancel_request(link_event):
                    result_sender.clear()
                    calibration_active = False
                    inline_calibration.invalidate()
                    pending_mode = None
                    command_active = False
                    mode_input_armed = False
                    mode_quiet_started = time.monotonic()
                    solution_preview = None
                    last_detection = None
                    last_payload = None
                    frozen_frame = None
                    frozen_detection = None
                    frozen_payload = None
                    stable_piece_count = 0
                    stable_frames = 0
                    solve_attempted = False
                    solve_recovery.reset()
                    solve_status = "WAIT_CMD"
                    last_error = ""
                    ignored_busy_modes = 0
                    print(
                        "\nESP32 CANCEL received: active task aborted; "
                        "no motion plan sent; returning to WAIT_CMD"
                    )
                    continue
                requested_mode = parse_mode_request(link_event)
                if requested_mode is not None:
                    mode_quiet_started = time.monotonic()
                    if requested_mode == 0 and calibration_active:
                        if not inline_calibration.ready:
                            print(
                                "\nMODE 0 finish requested, but blue-card "
                                "calibration is not stable yet "
                                "({}/{}); staying in calibration".format(
                                    inline_calibration.stable_frames,
                                    inline_calibration.required_stable_frames,
                                )
                            )
                        else:
                            fixed_calibration = (
                                inline_calibration.save(
                                    args.calibration_file
                                )
                            )
                            config.fixed_calibration = fixed_calibration
                            calibration_active = False
                            command_active = False
                            current_mode = args.mode
                            solve_status = "WAIT_CMD"
                            mode_input_armed = False
                            mode_quiet_started = time.monotonic()
                            stable_piece_count = 0
                            stable_frames = 0
                            last_error = ""
                            print(
                                "\nMODE 0 calibration saved and activated: "
                                "{}".format(args.calibration_file)
                            )
                            print(
                                "Remove the blue card; MODE 1/2/3/4 are ready."
                            )
                    elif requested_mode == 0:
                        if command_active or (
                            result_sender.active
                            and not result_sender.complete
                        ) or not mode_input_armed:
                            ignored_busy_modes += 1
                            if ignored_busy_modes == 1:
                                print(
                                    "\nESP32 MODE 0 ignored while "
                                    "busy/not armed"
                                )
                        else:
                            calibration_active = True
                            command_active = True
                            current_mode = 0
                            mode_input_armed = False
                            inline_calibration.reset()
                            solution_preview = None
                            last_detection = None
                            last_payload = None
                            stable_piece_count = 0
                            stable_frames = 0
                            solve_attempted = False
                            solve_recovery.reset()
                            solve_status = "CALIBRATING"
                            last_error = ""
                            print(
                                "\nESP32 requested MODE 0: "
                                "blue-card calibration started"
                            )
                    elif command_active or (
                        result_sender.active
                        and not result_sender.complete
                    ) or not mode_input_armed:
                        ignored_busy_modes += 1
                        if ignored_busy_modes == 1:
                            print(
                                "\nESP32 mode {} ignored while "
                                "busy/not armed".format(
                                    requested_mode
                                )
                            )
                    else:
                        pending_mode = requested_mode
                        mode_input_armed = False

                result_sender.step(link_event)
                if result_sender.active and not result_sender.complete:
                    solve_status = "SENDING {}/{}".format(
                        result_sender.delivered_count,
                        result_sender.total_count,
                    )
                if result_sender.complete:
                    print(
                        "\nESP32 delivered {}/{} result packets".format(
                            result_sender.delivered_count,
                            result_sender.total_count,
                        )
                    )
                    if ignored_busy_modes:
                        print(
                            "Ignored {} repeated mode request(s) "
                            "during this task".format(
                                ignored_busy_modes
                            )
                        )
                        ignored_busy_modes = 0
                    result_sender.clear()
                    command_active = False
                    solve_attempted = False
                    solve_recovery.reset()
                    stable_piece_count = 0
                    stable_frames = 0
                    frozen_frame = None
                    frozen_detection = None
                    frozen_payload = None
                    solve_status = "WAIT_CMD"
                    mode_input_armed = False
                    mode_quiet_started = time.monotonic()
                    print(
                        "WAITING FOR MODE LINE TO BECOME QUIET"
                    )

                if (
                    not command_active
                    and not result_sender.active
                    and not mode_input_armed
                    and time.monotonic() - mode_quiet_started
                    >= mode_rearm_quiet_seconds
                ):
                    mode_input_armed = True
                    ignored_busy_modes = 0
                    print("\nREADY FOR NEXT ESP32 MODE")

                if (
                    pending_mode is not None
                    and not command_active
                    and not result_sender.active
                ):
                    current_mode = pending_mode
                    pending_mode = None
                    if (
                        config.fixed_calibration is None
                        and not args.legacy_live_calibration
                    ):
                        print(
                            "\nMODE {} rejected: no fixed calibration. "
                            "Send MODE 0 twice to calibrate first.".format(
                                current_mode
                            )
                        )
                        command_active = False
                        solve_status = "WAIT_CAL"
                        mode_input_armed = False
                        mode_quiet_started = time.monotonic()
                        continue
                    command_active = True
                    solution_preview = None
                    last_detection = None
                    last_payload = None
                    frozen_frame = None
                    frozen_detection = None
                    frozen_payload = None
                    stable_piece_count = 0
                    stable_frames = 0
                    solve_attempted = False
                    solve_recovery.begin_task()
                    solve_status = "DETECTING"
                    last_error = ""
                    print(
                        "\nESP32 requested MODE {}".format(
                            current_mode
                        )
                    )

            success, frame = camera.read()
            if not success:
                print("Camera frame read failed")
                continue

            frame = rotate_frame(frame, args.rotate)
            frame_id += 1
            raw_preview = frame.copy()

            if calibration_active:
                if frame_id % max(1, args.detect_every) == 0:
                    start = time.perf_counter()
                    try:
                        last_detection = inline_calibration.observe(
                            frame
                        )
                        elapsed_ms = (
                            time.perf_counter() - start
                        ) * 1000.0
                        last_error = ""
                        solve_status = "CALIBRATING {}/{}".format(
                            min(
                                inline_calibration.stable_frames,
                                inline_calibration.required_stable_frames,
                            ),
                            inline_calibration.required_stable_frames,
                        )
                        print(
                            "\rMODE 0 blue calibration stable={}/{} "
                            "vision={:.1f}ms          ".format(
                                min(
                                    inline_calibration.stable_frames,
                                    inline_calibration.required_stable_frames,
                                ),
                                inline_calibration.required_stable_frames,
                                elapsed_ms,
                            ),
                            end="",
                            flush=True,
                        )
                    except Exception as error:
                        inline_calibration.invalidate()
                        last_error = str(error)
                        solve_status = "CALIBRATING WAIT"
                        print(
                            "\rMODE 0 calibration_error: {}          ".format(
                                last_error
                            ),
                            end="",
                            flush=True,
                        )
                if last_error:
                    cv2.putText(
                        raw_preview,
                        last_error[:85],
                        (15, 35),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                dashboard = make_landscape_dashboard(
                    raw_preview,
                    last_detection,
                    None,
                    0,
                    inline_calibration.stable_frames,
                    inline_calibration.required_stable_frames,
                    solve_status,
                )
                if args.web:
                    preview_state.update(dashboard)
                else:
                    cv2.imshow("Puzzle Vision", dashboard)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q"), ord("Q")):
                        break
                continue

            should_detect = (
                args.serial_device is None or command_active
            )
            if (
                should_detect
                and frame_id % max(1, args.detect_every) == 0
            ):
                start = time.perf_counter()
                try:
                    _, annotated, payload = vision.process_frame(
                        frame,
                        config,
                    )
                    elapsed_ms = (
                        time.perf_counter() - start
                    ) * 1000.0
                    piece_count = len(payload["pieces"])
                    cv2.putText(
                        annotated,
                        "Pi USB P={} vision={:.0f}ms".format(
                            piece_count,
                            elapsed_ms,
                        ),
                        (16, 62),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    calibration_method = payload.get(
                        "calibration",
                        {},
                    ).get(
                        "calibration_method",
                        "unknown",
                    )
                    calibration_region = payload.get(
                        "calibration",
                        {},
                    ).get(
                        "calibration_region",
                        "unknown",
                    )
                    cv2.putText(
                        annotated,
                        "CAL={} REG={}".format(
                            calibration_method,
                            calibration_region,
                        ),
                        (16, 86),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    previous_payload = last_payload
                    last_detection = annotated
                    last_payload = payload
                    last_error = ""
                    valid_count = (
                        piece_count == 4
                        if current_mode == 1
                        else 2 <= piece_count <= 4
                    )
                    if not command_active:
                        stable_piece_count = 0
                        stable_frames = 0
                    elif (
                        valid_count
                        and piece_count == stable_piece_count
                        and piece_geometry_is_stable(
                            previous_payload,
                            payload,
                        )
                    ):
                        stable_frames += 1
                    elif valid_count:
                        stable_piece_count = piece_count
                        stable_frames = 1
                    else:
                        stable_piece_count = 0
                        stable_frames = 0
                    best_effort_due = (
                        not args.manual
                        and command_active
                        and valid_count
                        and solve_recovery.deadline_reached(
                            args.best_effort_after_seconds
                        )
                    )
                    if (
                        not args.manual
                        and command_active
                        and not solve_attempted
                        and (
                            best_effort_due
                            or (
                                solve_recovery.ready()
                                and stable_frames >= 2
                            )
                        )
                    ):
                        # Freeze the exact frame which passed detection.
                        # Do not read another camera frame for solving.
                        frozen_frame = frame.copy()
                        frozen_detection = annotated.copy()
                        frozen_payload = payload
                        auto_solve_requested = True
                        if best_effort_due:
                            solve_status = "BEST_EFFORT_PENDING"
                    print(
                        "\rpieces={} stable={}/2 "
                        "vision={:.1f}ms cal={} reg={}          ".format(
                            piece_count,
                            min(stable_frames, 2),
                            elapsed_ms,
                            calibration_method,
                            calibration_region,
                        ),
                        end="",
                        flush=True,
                    )
                except Exception as error:
                    stable_piece_count = 0
                    stable_frames = 0
                    last_error = str(error)
                    now = time.monotonic()
                    if now - last_failure_save_at >= 2.0:
                        save_failure_diagnostics(
                            frame,
                            config,
                            error,
                            "detection",
                            current_mode,
                        )
                        last_failure_save_at = now
                    print(
                        "\rvision_error: {}          ".format(
                            last_error
                        ),
                        end="",
                        flush=True,
                    )

            if last_error:
                cv2.putText(
                    raw_preview,
                    last_error[:85],
                    (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
            dashboard = make_landscape_dashboard(
                raw_preview,
                last_detection,
                solution_preview,
                current_mode,
                stable_frames,
                2,
                solve_status,
            )

            if args.web:
                preview_state.update(dashboard)
                key = 255
            else:
                cv2.imshow("Puzzle Vision", dashboard)
                key = cv2.waitKey(1) & 0xFF
            if auto_solve_requested and key not in (
                27,
                ord("q"),
                ord("Q"),
            ):
                key = ord("p")
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("r"), ord("R")):
                solution_preview = None
                frozen_frame = None
                frozen_detection = None
                frozen_payload = None
                stable_piece_count = 0
                stable_frames = 0
                solve_attempted = False
                solve_recovery.reset()
                solve_status = "WAIT"
                if args.serial_device and not command_active:
                    solve_status = "WAIT_CMD"
                last_error = ""
            if key in (ord("p"), ord("P")):
                solve_attempted = True
                solve_status = "SOLVING"
                solve_input = "rpi_solve_input.png"
                solve_detection = "rpi_solve_detection.jpg"
                solve_json = "rpi_solve_detection.json"
                solve_frame = (
                    frozen_frame
                    if frozen_frame is not None
                    else frame
                )
                solve_detection_frame = (
                    frozen_detection
                    if frozen_detection is not None
                    else last_detection
                )
                solve_payload = (
                    frozen_payload
                    if frozen_payload is not None
                    else last_payload
                )
                cv2.imwrite(solve_input, solve_frame)
                if solve_detection_frame is not None:
                    cv2.imwrite(
                        solve_detection,
                        solve_detection_frame,
                    )
                if solve_payload is not None:
                    with open(
                        solve_json,
                        "w",
                        encoding="utf-8",
                    ) as output_file:
                        json.dump(
                            solve_payload,
                            output_file,
                            ensure_ascii=False,
                            indent=2,
                        )
                solving_screen = solve_frame.copy()
                cv2.putText(
                    solving_screen,
                    "SOLVING MODE {} ...".format(current_mode),
                    (30, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    3,
                    cv2.LINE_AA,
                )
                if not args.web:
                    cv2.imshow(
                        "Puzzle Vision",
                        make_landscape_dashboard(
                            solving_screen,
                            last_detection,
                            None,
                            current_mode,
                            stable_frames,
                            2,
                            "SOLVING",
                        ),
                    )
                    cv2.waitKey(1)
                else:
                    preview_state.update(solving_screen)

                print(
                    "\nSOLVING mode={} input={} detection={} json={}".format(
                        current_mode,
                        solve_input,
                        solve_detection,
                        solve_json,
                    )
                )
                try:
                    def cancel_requested_during_solve():
                        nonlocal ignored_busy_modes
                        if esp_link is None:
                            control_event = None
                        else:
                            control_event = esp_link.poll()
                            if is_cancel_request(control_event):
                                return True
                            if parse_mode_request(control_event) is not None:
                                ignored_busy_modes += 1
                        if (
                            not args.manual
                            and solve_recovery.deadline_reached(
                                args.best_effort_after_seconds
                            )
                        ):
                            raise BestEffortDeadlineReached(
                                "automatic solve deadline reached"
                            )
                        return False

                    progress_callback = make_terminal_progress(
                        current_mode,
                        cancel_check=cancel_requested_during_solve,
                    )
                    mode3_beam_levels = mode3_beam_levels_for_retry(
                        solve_recovery.failure_count,
                        args.mode3_high_beam_after_failures,
                    )
                    if current_mode == 3:
                        print(
                            "Mode 3 beam levels={} previous_failures={}".format(
                                mode3_beam_levels,
                                solve_recovery.failure_count,
                            )
                        )
                    state_machine = VisionStateMachine(
                        output_directory="rpi_tmp",
                        detect_config=config,
                        progress_callback=progress_callback,
                        solver_workers=args.solver_workers,
                        mode3_beam_levels=mode3_beam_levels,
                    )
                    state_machine.select_mode(current_mode)
                    try:
                        if (
                            not args.manual
                            and solve_recovery.deadline_reached(
                                args.best_effort_after_seconds
                            )
                        ):
                            raise BestEffortDeadlineReached(
                                "automatic solve deadline reached"
                            )
                        solve_result = state_machine.start(solve_input)
                        if cancel_requested_during_solve():
                            raise TaskCancelled(
                                "ESP32 cancelled before motion-plan transmission"
                            )
                    except BestEffortDeadlineReached:
                        elapsed_seconds = solve_recovery.elapsed()
                        solve_status = "BEST_EFFORT"
                        solve_recovery.consider_candidate(
                            state_machine.best_effort_candidate
                        )
                        print(
                            "\nSOLVE DEADLINE {:.1f}s reached after "
                            "{:.1f}s; selecting the best scored candidate"
                            .format(
                                args.best_effort_after_seconds,
                                elapsed_seconds,
                            )
                        )
                        state_machine = VisionStateMachine(
                            output_directory="rpi_tmp",
                            detect_config=config,
                            solver_workers=args.solver_workers,
                        )
                        state_machine.select_mode(current_mode)
                        if solve_recovery.best_candidate is not None:
                            try:
                                solve_result = state_machine.start_best_candidate(
                                    solve_recovery.best_candidate,
                                    elapsed_seconds=elapsed_seconds,
                                )
                                print(
                                    "BEST-EFFORT source={} quality={}".format(
                                        solve_recovery.best_candidate["source"],
                                        solve_recovery.best_candidate["quality_key"],
                                    )
                                )
                            except RuntimeError as candidate_error:
                                print(
                                    "Best scored candidate was not mechanically "
                                    "safe: {}; using compact transport fallback"
                                    .format(candidate_error)
                                )
                                state_machine = VisionStateMachine(
                                    output_directory="rpi_tmp",
                                    detect_config=config,
                                    solver_workers=args.solver_workers,
                                )
                                state_machine.select_mode(current_mode)
                                solve_result = state_machine.start_best_effort(
                                    solve_input,
                                    reason="no_safe_scored_candidate",
                                    elapsed_seconds=elapsed_seconds,
                                )
                        else:
                            solve_result = state_machine.start_best_effort(
                                solve_input,
                                reason="no_scored_candidate",
                                elapsed_seconds=elapsed_seconds,
                            )
                    except Exception:
                        solve_recovery.consider_candidate(
                            state_machine.best_effort_candidate
                        )
                        raise
                    solve_recovery.reset()
                    solution_preview = cv2.imread(
                        solve_result["solution_image"]
                    )
                    print(
                        json.dumps(
                            solve_result,
                            ensure_ascii=False,
                            indent=2,
                        )
                    )
                    plan = state_machine.get_plan()
                    plan_path = "rpi_solve_plan.json"
                    with open(
                        plan_path,
                        "w",
                        encoding="utf-8",
                    ) as output_file:
                        json.dump(
                            plan,
                            output_file,
                            ensure_ascii=False,
                            indent=2,
                        )
                    print(
                        json.dumps(
                            plan,
                            ensure_ascii=False,
                            indent=2,
                        )
                    )
                    print("Saved motion plan: {}".format(plan_path))
                    if args.web and solution_preview is not None:
                        preview_state.update(solution_preview)
                    if result_sender is not None:
                        packet_count = len(plan["actions"]) + 1
                        solve_status = "SENDING 0/{}".format(packet_count)
                        print(
                            "ESP32 sending {} packets "
                            "(count + {} pieces)".format(
                                packet_count,
                                len(plan["actions"]),
                            )
                        )
                        ignored_busy_modes += send_plan_reliably(
                            esp_link,
                            result_sender,
                            plan["actions"],
                        )
                        discard_received = getattr(
                            esp_link,
                            "discard_received",
                            None,
                        )
                        if discard_received is not None:
                            discard_received()
                        if ignored_busy_modes:
                            print(
                                "Ignored {} repeated mode request(s) "
                                "during this task".format(
                                    ignored_busy_modes
                                )
                            )
                            ignored_busy_modes = 0
                        command_active = False
                        solve_attempted = False
                        stable_piece_count = 0
                        stable_frames = 0
                        frozen_frame = None
                        frozen_detection = None
                        frozen_payload = None
                        solve_status = "WAIT_CMD"
                        mode_input_armed = False
                        mode_quiet_started = time.monotonic()
                        print(
                            "WAITING FOR MODE LINE TO BECOME QUIET"
                        )
                    else:
                        solve_status = "DONE"
                except TaskCancelled as error:
                    if result_sender is not None:
                        result_sender.clear()
                    calibration_active = False
                    pending_mode = None
                    command_active = False
                    mode_input_armed = False
                    mode_quiet_started = time.monotonic()
                    solution_preview = None
                    last_detection = None
                    last_payload = None
                    frozen_frame = None
                    frozen_detection = None
                    frozen_payload = None
                    stable_piece_count = 0
                    stable_frames = 0
                    solve_attempted = False
                    solve_recovery.reset()
                    solve_status = "WAIT_CMD"
                    last_error = ""
                    ignored_busy_modes = 0
                    print(
                        "\nESP32 CANCEL received during active task: {}; "
                        "returning to WAIT_CMD".format(error)
                    )
                except Exception as error:
                    automatic_recovery = not args.manual
                    attempt = solve_recovery.record_failure()
                    solve_status = (
                        "RETRY_WAIT #{}".format(attempt)
                        if automatic_recovery
                        else "ERROR"
                    )
                    last_error = "solve failed attempt {}: {}".format(
                        attempt,
                        error,
                    )
                    save_failure_diagnostics(
                        solve_frame,
                        config,
                        error,
                        "solve",
                        current_mode,
                        attempt=attempt,
                    )
                    print("\n" + last_error)
                    if automatic_recovery:
                        # Keep this ESP32 task active. No motion packet is
                        # sent until a fresh frame yields a validated plan.
                        command_active = True
                        solve_attempted = False
                        stable_piece_count = 0
                        stable_frames = 0
                        frozen_frame = None
                        frozen_detection = None
                        frozen_payload = None
                        solution_preview = None
                        print(
                            "AUTO-RETRY mode={} next_attempt={} after "
                            "{:.2f}s; waiting for 2 fresh stable frames"
                            .format(
                                current_mode,
                                attempt + 1,
                                solve_recovery.retry_delay_seconds,
                            )
                        )
            if key in (ord("s"), ord("S")):
                stamp = time.strftime("%Y%m%d_%H%M%S")
                raw_path = "rpi_raw_{}.jpg".format(stamp)
                detection_path = "rpi_detection_{}.jpg".format(
                    stamp
                )
                json_path = "rpi_detection_{}.json".format(stamp)
                cv2.imwrite(raw_path, frame)
                if last_detection is not None:
                    cv2.imwrite(detection_path, last_detection)
                if last_payload is not None:
                    with open(
                        json_path,
                        "w",
                        encoding="utf-8",
                    ) as output_file:
                        json.dump(
                            last_payload,
                            output_file,
                            ensure_ascii=False,
                            indent=2,
                        )
                print(
                    "\nSaved: {}, {}, {}".format(
                        raw_path,
                        detection_path,
                        json_path,
                    )
                )
    except KeyboardInterrupt:
        pass
    finally:
        if preview_server is not None:
            preview_server.shutdown()
            preview_server.server_close()
        camera.release()
        if result_sender is not None:
            result_sender.clear()
        if serial_connection is not None:
            serial_connection.close()
        cv2.destroyAllWindows()
        print("\nStopped")


if __name__ == "__main__":
    main()
