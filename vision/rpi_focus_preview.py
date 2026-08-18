"""USB camera live preview and manual-focus assistant for Raspberry Pi."""

import argparse
import time

import cv2

from rpi_camera_controls import apply_camera_controls, camera_device_path


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


def focus_score(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def main():
    parser = argparse.ArgumentParser()
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
    args = parser.parse_args()

    camera = open_camera(
        args.camera,
        args.width,
        args.height,
        args.fps,
        apply_fixed_controls=not args.skip_camera_controls,
        camera_controls_file=args.camera_controls_file,
    )

    # Allow the fixed manual exposure and white balance to settle.
    for _ in range(20):
        camera.read()

    frame_counter = 0
    fps_value = 0.0
    fps_start = time.perf_counter()

    print("Q/ESC: quit, S: save current frame")

    try:
        while True:
            success, frame = camera.read()
            if not success:
                print("Camera frame read failed")
                continue

            height, width = frame.shape[:2]
            roi_width = max(80, width // 3)
            roi_height = max(80, height // 3)
            left = (width - roi_width) // 2
            top = (height - roi_height) // 2
            right = left + roi_width
            bottom = top + roi_height

            focus_roi = frame[top:bottom, left:right]
            sharpness = focus_score(focus_roi)

            frame_counter += 1
            now = time.perf_counter()
            fps_elapsed = now - fps_start
            if fps_elapsed >= 1.0:
                fps_value = frame_counter / fps_elapsed
                frame_counter = 0
                fps_start = now

            preview = frame.copy()
            cv2.rectangle(
                preview,
                (left, top),
                (right, bottom),
                (0, 255, 0),
                2,
            )
            cv2.putText(
                preview,
                "FOCUS {:.0f}  FPS {:.1f}".format(
                    sharpness,
                    fps_value,
                ),
                (20, 38),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                preview,
                "Adjust lens; maximize FOCUS in green box",
                (20, 72),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            # Show a 2x enlargement of the central focus region.
            zoom = cv2.resize(
                focus_roi,
                None,
                fx=2.0,
                fy=2.0,
                interpolation=cv2.INTER_NEAREST,
            )
            cv2.imshow("USB camera focus", preview)
            cv2.imshow("Center 2x zoom", zoom)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("s"), ord("S")):
                output_path = "rpi_focus_{}.jpg".format(
                    time.strftime("%Y%m%d_%H%M%S")
                )
                cv2.imwrite(output_path, frame)
                print("Saved:", output_path)
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
