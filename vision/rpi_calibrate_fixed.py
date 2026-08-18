"""One-time global calibration from a blue 105 x 297 mm upper-half card."""

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np

import fragment_vision as vision
from rpi_camera_controls import apply_camera_controls, camera_device_path


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


def make_preview(frame, calibration):
    preview = frame.copy()
    points = np.round(
        np.asarray(
            calibration["calibration_points_px"],
            dtype=np.float32,
        )
    ).astype(np.int32)
    cv2.polylines(
        preview,
        [points.reshape(-1, 1, 2)],
        True,
        (255, 255, 0),
        4,
        cv2.LINE_AA,
    )
    labels = (
        "TL (210,0)",
        "TR (210,297)",
        "BR (105,297)",
        "BL (105,0)",
    )
    for point, label in zip(points, labels):
        cv2.circle(
            preview,
            tuple(point),
            9,
            (0, 0, 255),
            cv2.FILLED,
            cv2.LINE_AA,
        )
        cv2.putText(
            preview,
            label,
            (int(point[0]) + 10, int(point[1]) - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    cv2.putText(
        preview,
        "BLUE HALF FIXED CAL hue={} score={:.3f}".format(
            calibration["blue_hue"],
            calibration["calibration_score"],
        ),
        (18, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    return preview


def save_result(
    output_path,
    frame,
    mask,
    rectified,
    calibration,
    next_step_message=True,
):
    calibration = dict(calibration)
    calibration["created_at"] = time.strftime(
        "%Y-%m-%d %H:%M:%S"
    )
    output = Path(output_path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as output_file:
        json.dump(
            calibration,
            output_file,
            ensure_ascii=False,
            indent=2,
        )
    stem = output.with_suffix("")
    preview_path = Path(str(stem) + "_preview.jpg")
    mask_path = Path(str(stem) + "_mask.png")
    rectified_path = Path(str(stem) + "_rectified.jpg")
    cv2.imwrite(
        str(preview_path),
        make_preview(frame, calibration),
    )
    cv2.imwrite(str(mask_path), mask)
    cv2.imwrite(str(rectified_path), rectified)
    print("Saved calibration:", output)
    print("Saved preview:", preview_path)
    print("Saved mask:", mask_path)
    print("Saved rectified image:", rectified_path)
    if next_step_message:
        print(
            "Remove the blue card, then start real-time detection "
            "with this calibration file."
        )
    return calibration


def calibrate_frame(frame, target_hue):
    config = vision.DetectConfig()
    return vision.create_upper_half_fixed_calibration(
        frame,
        config,
        target_hue=target_hue,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        default="",
        help="calibrate from an existing rotated image instead of camera",
    )
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--skip-camera-controls",
        action="store_true",
        help="do not apply the saved fixed UVC camera controls",
    )
    parser.add_argument(
        "--camera-controls-file",
        default=None,
        help="camera-specific UVC control profile JSON",
    )
    parser.add_argument(
        "--rotate",
        type=int,
        choices=(0, 90, 180, 270),
        default=180,
    )
    parser.add_argument(
        "--blue-hue",
        type=int,
        default=None,
        help="optional OpenCV HSV hue (0-179); automatic by default",
    )
    parser.add_argument(
        "--output",
        default="rpi_camera_calibration.json",
    )
    args = parser.parse_args()

    if args.source:
        frame = cv2.imread(args.source)
        if frame is None:
            raise RuntimeError(
                "Cannot read calibration image: {}".format(
                    args.source
                )
            )
        mask, rectified, calibration = calibrate_frame(
            frame,
            args.blue_hue,
        )
        save_result(
            args.output,
            frame,
            mask,
            rectified,
            calibration,
        )
        return

    camera = open_camera(
        args.camera,
        args.width,
        args.height,
        args.fps,
        apply_fixed_controls=not args.skip_camera_controls,
        camera_controls_file=args.camera_controls_file,
    )
    for _ in range(20):
        camera.read()

    latest = None
    last_frame = None
    last_error = ""
    print(
        "Place the 105x297 mm blue card in the camera upper half. "
        "S=save calibration, Q/ESC=quit."
    )
    try:
        while True:
            success, frame = camera.read()
            if not success:
                print("Camera frame read failed")
                continue
            frame = rotate_frame(frame, args.rotate)
            last_frame = frame.copy()
            try:
                mask, rectified, calibration = calibrate_frame(
                    frame,
                    args.blue_hue,
                )
                preview = make_preview(frame, calibration)
                latest = (
                    frame.copy(),
                    mask.copy(),
                    rectified.copy(),
                    calibration,
                )
                last_error = ""
            except RuntimeError as error:
                preview = frame.copy()
                last_error = str(error)
                cv2.putText(
                    preview,
                    last_error,
                    (18, 38),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.62,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                latest = None

            cv2.imshow("Fixed global calibration", preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                if latest is None:
                    failure_path = Path(args.output).with_name(
                        Path(args.output).stem
                        + "_failure_raw.png"
                    )
                    if last_frame is not None:
                        cv2.imwrite(
                            str(failure_path),
                            last_frame,
                        )
                    print(
                        "No reliable blue-card calibration to save: "
                        + (last_error or "unknown error")
                    )
                    print(
                        "Saved failure frame:",
                        failure_path,
                    )
                    continue
                save_result(args.output, *latest)
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
