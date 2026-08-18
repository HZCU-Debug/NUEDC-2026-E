"""从完整湖蓝A4照片识别碎片并求解100×60 mm目标矩形。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from fragment_vision import (
    DetectConfig,
    PIXELS_PER_MM,
    process_frame,
    resize_for_debug,
)
from puzzle_solver import normalize_solution, solve_polygons
from puzzle_solver import solve_with_template


def polygon_centroid(polygon: np.ndarray) -> np.ndarray:
    contour = polygon.astype(np.float32).reshape(-1, 1, 2)
    moments = cv2.moments(contour)
    if abs(moments["m00"]) < 1e-8:
        return np.mean(polygon, axis=0)
    return np.asarray(
        [
            moments["m10"] / moments["m00"],
            moments["m01"] / moments["m00"],
        ],
        dtype=np.float32,
    )


def render_solution(
    targets,
    rotations,
    metrics,
    output_path: str,
) -> None:
    width_px = int(round(210 * PIXELS_PER_MM))
    height_px = int(round(297 * PIXELS_PER_MM))
    canvas = np.full((height_px, width_px, 3), (190, 150, 60), np.uint8)
    divider_y = int(round(148.5 * PIXELS_PER_MM))
    cv2.line(canvas, (0, divider_y), (width_px - 1, divider_y), (0, 255, 255), 2)

    colors = [
        (80, 220, 255),
        (255, 180, 80),
        (180, 255, 120),
        (220, 120, 255),
    ]
    for piece_id, target in enumerate(targets):
        polygon_px = np.round(target * PIXELS_PER_MM).astype(np.int32)
        cv2.fillPoly(canvas, [polygon_px], colors[piece_id % len(colors)])
        cv2.polylines(canvas, [polygon_px], True, (20, 20, 20), 2)
        center = np.round(
            polygon_centroid(target) * PIXELS_PER_MM
        ).astype(int)
        target_center_mm = polygon_centroid(target)
        label_x = min(
            max(4, int(center[0]) + 5),
            width_px - 175,
        )
        label_y = min(
            max(18, int(center[1]) - 5),
            height_px - 24,
        )
        cv2.putText(
            canvas,
            "P{} {:.1f}deg".format(piece_id, rotations[piece_id]),
            (label_x, label_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            "T({:.1f},{:.1f})mm".format(
                target_center_mm[0],
                target_center_mm[1],
            ),
            (label_x, label_y + 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (0, 0, 0),
            2,
            cv2.LINE_AA,
        )

    summary = "{:.1f}x{:.1f}mm fill={:.3f}".format(
        metrics["long_side_mm"],
        metrics["short_side_mm"],
        metrics["fill_ratio"],
    )
    cv2.putText(
        canvas,
        summary,
        (18, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(output_path, canvas)


def normalize_angle(angle: float) -> float:
    return (angle + 180.0) % 360.0 - 180.0


def default_detection_path(solution_path: str) -> str:
    path = Path(solution_path)
    if "solution" in path.stem:
        stem = path.stem.replace("solution", "detection", 1)
    else:
        stem = path.stem + "_detection"
    return str(path.with_name(stem + path.suffix))


def main() -> int:
    total_start = time.perf_counter()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", default="puzzle_solution.jpg")
    parser.add_argument(
        "--detection-output",
        default="",
        help="检测图路径；默认根据 --output 自动生成",
    )
    parser.add_argument("--beam-width", type=int, default=2000)
    parser.add_argument(
        "--template",
        default=str(Path(__file__).with_name("fixed_puzzle_template.json")),
        help="固定碎片模板；不存在或不匹配时使用通用求解",
    )
    args = parser.parse_args()

    load_start = time.perf_counter()
    frame = cv2.imread(args.source)
    if frame is None:
        raise RuntimeError("无法读取图片：{}".format(args.source))
    frame = resize_for_debug(frame)
    load_ms = (time.perf_counter() - load_start) * 1000.0

    vision_start = time.perf_counter()
    _, detection_annotated, detection = process_frame(frame, DetectConfig())
    vision_ms = (time.perf_counter() - vision_start) * 1000.0
    if len(detection["pieces"]) != 4:
        raise RuntimeError(
            "需要识别到4块碎片，当前为{}块".format(len(detection["pieces"]))
        )

    polygons = [
        np.asarray(piece["vertices_mm"], dtype=np.float32)
        for piece in detection["pieces"]
    ]
    solve_start = time.perf_counter()
    used_template = False
    template_path = Path(args.template)
    if template_path.is_file():
        template_data = json.loads(template_path.read_text(encoding="utf-8"))
        template_polygons = [
            np.asarray(piece["target_vertices_mm"], dtype=np.float32)
            for piece in template_data["pieces"]
        ]
        try:
            state, metrics = solve_with_template(
                polygons,
                template_polygons,
            )
            used_template = True
        except RuntimeError:
            used_template = False
    if not used_template:
        state, metrics = solve_polygons(
            polygons,
            target_size_mm=(100.0, 60.0),
            beam_width=args.beam_width,
        )

    if used_template:
        assignment = [
            int(template_id)
            for template_id in metrics["template_assignment"]
        ]
        targets = [
            template_polygons[assignment[piece_id]].copy()
            for piece_id in range(len(polygons))
        ]
        rotations = [
            np.degrees(
                np.arctan2(
                    state.transforms[piece_id][0][1, 0],
                    state.transforms[piece_id][0][0, 0],
                )
            )
            for piece_id in range(len(polygons))
        ]
    else:
        targets, rotations = normalize_solution(
            polygons,
            state,
            target_origin_mm=(55.0, 30.0),
        )
    solve_ms = (time.perf_counter() - solve_start) * 1000.0
    rotations = [normalize_angle(angle) for angle in rotations]

    render_start = time.perf_counter()
    detection_output = (
        args.detection_output
        if args.detection_output
        else default_detection_path(args.output)
    )
    cv2.imwrite(detection_output, detection_annotated)
    render_solution(targets, rotations, metrics, args.output)
    render_ms = (time.perf_counter() - render_start) * 1000.0

    result_pieces = []
    for piece_id, target in enumerate(targets):
        result_pieces.append(
            {
                "id": piece_id,
                "source_centroid_mm": detection["pieces"][piece_id][
                    "centroid_mm"
                ],
                "source_pickup_mm": detection["pieces"][piece_id][
                    "pickup_mm"
                ],
                "target_centroid_mm": np.round(
                    polygon_centroid(target),
                    2,
                ).tolist(),
                "target_rotation_deg": round(rotations[piece_id], 2),
                "target_vertices_mm": np.round(target, 2).tolist(),
            }
        )

    total_ms = (time.perf_counter() - total_start) * 1000.0
    result = {
        "calibration": detection["calibration"],
        "metrics": metrics,
        "matches": [
            {
                "fixed_piece": match[0],
                "fixed_edge": match[1],
                "moving_piece": match[2],
                "moving_edge": match[3],
                "length_error": round(match[4], 4),
            }
            for match in state.matches
        ],
        "pieces": result_pieces,
        "detection_image": detection_output,
        "solution_image": args.output,
        "timing_ms": {
            "image_load": round(load_ms, 2),
            "vision": round(vision_ms, 2),
            "puzzle_solve": round(solve_ms, 2),
            "render": round(render_ms, 2),
            "total": round(total_ms, 2),
        },
    }
    print(
        (
            "耗时：读取 {:.2f} ms，视觉 {:.2f} ms，拼图 {:.2f} ms，"
            "绘制 {:.2f} ms，总计 {:.2f} ms"
        ).format(load_ms, vision_ms, solve_ms, render_ms, total_ms),
        file=sys.stderr,
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
