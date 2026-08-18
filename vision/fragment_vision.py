"""湖蓝色卡纸上的拼图碎片检测与吸取点调试工具。

示例：
    python fragment_vision.py --source 0
    python fragment_vision.py --source test2.jpg
    python fragment_vision.py --source test2.jpg --no-gui --output result.jpg
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, replace
from itertools import combinations
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np


Point = Tuple[int, int]
A4_SHORT_MM = 210.0
A4_LONG_MM = 297.0
PIXELS_PER_MM = 4.0
# 自备图2中存在约 1 cm 的真实短边，只合并明显小于它的毛刺。
MIN_DETECTED_EDGE_MM = 5.0


@dataclass
class DetectConfig:
    hue_tolerance: int = 14
    saturation_min: int = 55
    value_min: int = 35
    open_size: int = 5
    close_size: int = 11
    paper_erode_size: int = 9
    min_area_ratio: float = 0.005
    max_area_ratio: float = 0.15
    min_paper_area_ratio: float = 0.08
    polygon_epsilon_percent: float = 1.5
    safe_radius_px: int = 12
    divider_value_ratio: float = 0.68
    divider_exclusion_mm: float = 3.5
    require_divider_calibration: bool = False
    calibration_strategy: str = "auto"
    fixed_calibration: Optional[Dict[str, object]] = None
    # Mode 3 may treat dark printed card artwork as part of the physical
    # fragment so edge-touching pips cannot carve false contour notches.
    include_dark_artwork_in_piece_mask: bool = False
    artwork_dark_value_ratio: float = 0.58
    # Canonical lower half corresponds to the physical right-hand source
    # area with the current fixed camera orientation.
    source_half: str = "lower"


def odd_kernel(value: int) -> int:
    value = max(1, int(value))
    return value if value % 2 == 1 else value + 1


def learn_blue_hue(hsv: np.ndarray) -> int:
    """从高饱和度区域自动估计湖蓝卡纸的主色相。"""
    hue, saturation, value = cv2.split(hsv)
    colorful = (saturation > 70) & (value > 40)
    if not np.any(colorful):
        raise RuntimeError("画面中没有检测到足够的湖蓝色区域")
    histogram = np.bincount(hue[colorful].ravel(), minlength=180)
    return int(np.argmax(histogram))


def build_blue_background_mask(
    frame: np.ndarray,
    config: DetectConfig,
) -> Tuple[np.ndarray, int]:
    """学习湖蓝背景并返回背景掩膜。"""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    background_hue = learn_blue_hue(hsv)
    background = build_mask_for_hue(
        hue,
        saturation,
        value,
        background_hue,
        config,
    )
    return background, background_hue


def build_mask_for_hue(
    hue: np.ndarray,
    saturation: np.ndarray,
    value: np.ndarray,
    target_hue: int,
    config: DetectConfig,
) -> np.ndarray:
    """Build a cleaned HSV mask around an explicitly selected hue."""
    background_hue = int(target_hue) % 180
    hue_delta = np.abs(hue.astype(np.int16) - background_hue)
    hue_delta = np.minimum(hue_delta, 180 - hue_delta)
    background = (
        (hue_delta <= config.hue_tolerance)
        & (saturation >= config.saturation_min)
        & (value >= config.value_min)
    ).astype(np.uint8) * 255

    background = cv2.morphologyEx(
        background,
        cv2.MORPH_CLOSE,
        np.ones(
            (odd_kernel(config.paper_erode_size),) * 2,
            dtype=np.uint8,
        ),
    )
    return background


def order_corners(points: np.ndarray) -> np.ndarray:
    """将四角排序为左上、右上、右下、左下。"""
    points = points.reshape(4, 2).astype(np.float32)
    sums = points.sum(axis=1)
    differences = np.diff(points, axis=1).reshape(-1)
    return np.asarray(
        [
            points[np.argmin(sums)],
            points[np.argmin(differences)],
            points[np.argmax(sums)],
            points[np.argmax(differences)],
        ],
        dtype=np.float32,
    )


def select_a4_quadrilateral(
    candidate_points: np.ndarray,
    paper_contour: np.ndarray,
) -> np.ndarray:
    """从受背景干扰的候选角点中选择最符合A4比例的四边形。"""
    points = candidate_points.reshape(-1, 2)
    if len(points) < 4 or len(points) > 8:
        raise RuntimeError(
            "A4边界候选角数量异常：{}".format(len(points))
        )

    target_aspect_ratio = A4_LONG_MM / A4_SHORT_MM
    contour_area = max(1.0, float(cv2.contourArea(paper_contour)))
    best_score = float("inf")
    best_corners = None

    for selected in combinations(points, 4):
        corners = order_corners(
            np.asarray(selected, dtype=np.float32)
        )
        if len(np.unique(corners, axis=0)) != 4:
            continue
        polygon = corners.reshape(-1, 1, 2)
        if not cv2.isContourConvex(polygon):
            continue

        top = np.linalg.norm(corners[1] - corners[0])
        bottom = np.linalg.norm(corners[2] - corners[3])
        left = np.linalg.norm(corners[3] - corners[0])
        right = np.linalg.norm(corners[2] - corners[1])
        mean_width = (top + bottom) * 0.5
        mean_height = (left + right) * 0.5
        if min(mean_width, mean_height) < 1.0:
            continue

        aspect_ratio = max(mean_width, mean_height) / min(
            mean_width,
            mean_height,
        )
        quadrilateral_area = abs(float(cv2.contourArea(polygon)))
        area_coverage = quadrilateral_area / contour_area
        if area_coverage < 0.70 or area_coverage > 1.15:
            continue

        aspect_error = abs(
            np.log(aspect_ratio / target_aspect_ratio)
        )
        coverage_error = abs(1.0 - area_coverage)
        score = 3.0 * aspect_error + coverage_error
        if score < best_score:
            best_score = score
            best_corners = corners

    if best_corners is None or best_score > 0.80:
        raise RuntimeError("没有找到比例合理的A4四角组合")
    return best_corners


def line_intersection(
    first_point: np.ndarray,
    first_direction: np.ndarray,
    second_point: np.ndarray,
    second_direction: np.ndarray,
) -> np.ndarray:
    matrix = np.column_stack(
        (first_direction, -second_direction)
    ).astype(np.float64)
    determinant = float(np.linalg.det(matrix))
    if abs(determinant) < 1e-6:
        raise RuntimeError("A4相邻边接近平行，无法精确计算角点")
    parameters = np.linalg.solve(
        matrix,
        (second_point - first_point).astype(np.float64),
    )
    return (
        first_point + parameters[0] * first_direction
    ).astype(np.float32)


def fit_contour_side(
    contour_points: np.ndarray,
    start: np.ndarray,
    end: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    direction = end - start
    length = float(np.linalg.norm(direction))
    if length < 2.0:
        raise RuntimeError("A4候选边过短")

    relative = contour_points - start
    projection = np.sum(relative * direction, axis=1) / (length * length)
    perpendicular = np.abs(
        relative[:, 0] * direction[1]
        - relative[:, 1] * direction[0]
    ) / length
    distance_limit = max(4.0, length * 0.035)
    selected = contour_points[
        (projection >= 0.06)
        & (projection <= 0.94)
        & (perpendicular <= distance_limit)
    ]
    if len(selected) < 8:
        return start.astype(np.float32), (
            direction / length
        ).astype(np.float32)

    fitted = cv2.fitLine(
        selected.astype(np.float32),
        cv2.DIST_HUBER,
        0,
        0.01,
        0.01,
    ).reshape(-1)
    fitted_direction = np.asarray(
        [fitted[0], fitted[1]],
        dtype=np.float32,
    )
    fitted_point = np.asarray(
        [fitted[2], fitted[3]],
        dtype=np.float32,
    )
    return fitted_point, fitted_direction


def refine_a4_corners(
    paper_contour: np.ndarray,
    initial_corners: np.ndarray,
) -> np.ndarray:
    """用纸张四条边的轮廓点拟合直线，再求交点以稳定四角。"""
    initial = order_corners(initial_corners)
    contour_points = paper_contour.reshape(-1, 2).astype(np.float32)
    fitted_lines = []
    for edge_index in range(4):
        fitted_lines.append(
            fit_contour_side(
                contour_points,
                initial[edge_index],
                initial[(edge_index + 1) % 4],
            )
        )

    refined = np.asarray(
        [
            line_intersection(
                fitted_lines[3][0],
                fitted_lines[3][1],
                fitted_lines[0][0],
                fitted_lines[0][1],
            ),
            line_intersection(
                fitted_lines[0][0],
                fitted_lines[0][1],
                fitted_lines[1][0],
                fitted_lines[1][1],
            ),
            line_intersection(
                fitted_lines[1][0],
                fitted_lines[1][1],
                fitted_lines[2][0],
                fitted_lines[2][1],
            ),
            line_intersection(
                fitted_lines[2][0],
                fitted_lines[2][1],
                fitted_lines[3][0],
                fitted_lines[3][1],
            ),
        ],
        dtype=np.float32,
    )

    diagonal = max(
        1.0,
        float(np.linalg.norm(initial[2] - initial[0])),
    )
    maximum_shift = float(
        np.max(np.linalg.norm(refined - initial, axis=1))
    )
    polygon = refined.reshape(-1, 1, 2)
    if (
        maximum_shift > 0.10 * diagonal
        or not cv2.isContourConvex(polygon)
    ):
        return initial
    return refined


def validate_a4_source(
    background: np.ndarray,
    source: np.ndarray,
) -> None:
    """验证候选四边形内部确实主要由湖蓝A4覆盖。"""
    validation_width = 210
    validation_height = 297
    destination = np.asarray(
        [
            [0, 0],
            [validation_width - 1, 0],
            [validation_width - 1, validation_height - 1],
            [0, validation_height - 1],
        ],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(
        source.astype(np.float32),
        destination,
    )
    warped = cv2.warpPerspective(
        background,
        homography,
        (validation_width, validation_height),
        flags=cv2.INTER_NEAREST,
    )
    overall_ratio = float(np.mean(warped > 0))
    band = 12
    inset = 3
    edge_ratios = [
        float(np.mean(warped[inset:band, :] > 0)),
        float(np.mean(warped[-band:-inset, :] > 0)),
        float(np.mean(warped[:, inset:band] > 0)),
        float(np.mean(warped[:, -band:-inset] > 0)),
    ]
    mean_edge_ratio = float(np.mean(edge_ratios))
    # 最外圈会受到透视插值和角点亚像素误差影响，因此边带只用于
    # 排除明显错误的四边形，主体覆盖率仍是主要依据。
    if (
        overall_ratio < 0.72
        or mean_edge_ratio < 0.35
        or min(edge_ratios) < 0.08
    ):
        raise RuntimeError(
            "A4四角不可靠：矫正后纸面覆盖不足 "
            "overall={:.2f}, edges={}".format(
                overall_ratio,
                [round(value, 2) for value in edge_ratios],
            )
        )


def validate_a4_not_clipped(
    paper_contour: np.ndarray,
    frame_shape: Tuple[int, ...],
    margin_ratio: float = 0.006,
) -> None:
    """Reject a paper contour clipped by any camera-frame boundary."""
    frame_height, frame_width = frame_shape[:2]
    margin = max(
        3,
        int(round(min(frame_width, frame_height) * margin_ratio)),
    )
    points = paper_contour.reshape(-1, 2)
    minimum = np.min(points, axis=0)
    maximum = np.max(points, axis=0)
    clipped_edges = []
    if minimum[0] <= margin:
        clipped_edges.append("left")
    if maximum[0] >= frame_width - 1 - margin:
        clipped_edges.append("right")
    if minimum[1] <= margin:
        clipped_edges.append("top")
    if maximum[1] >= frame_height - 1 - margin:
        clipped_edges.append("bottom")
    if clipped_edges:
        raise RuntimeError(
            "A4 boundary is outside camera view: "
            + ",".join(clipped_edges)
        )


def _quadrilateral_from_contour(contour: np.ndarray) -> np.ndarray:
    hull = cv2.convexHull(contour)
    perimeter = cv2.arcLength(hull, True)
    corners = cv2.approxPolyDP(hull, perimeter * 0.015, True)
    if len(corners) != 4:
        corners = cv2.approxPolyDP(
            hull,
            perimeter * 0.010,
            True,
        )
    source = select_a4_quadrilateral(corners, contour)
    return refine_a4_corners(contour, source)


def _fit_divider_center_line(
    frame: np.ndarray,
    background: np.ndarray,
    approximate_start: np.ndarray,
    approximate_end: np.ndarray,
    value_ratio: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Fit the dark divider centre inside a narrow edge band."""
    value = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2]
    edge = approximate_end - approximate_start
    length = float(np.linalg.norm(edge))
    if length < 20.0:
        raise RuntimeError("A4 divider candidate is too short")

    unit = edge / length
    yy, xx = np.indices(value.shape)
    relative_x = xx.astype(np.float32) - float(approximate_start[0])
    relative_y = yy.astype(np.float32) - float(approximate_start[1])
    projection = (
        relative_x * float(unit[0])
        + relative_y * float(unit[1])
    )
    perpendicular = np.abs(
        relative_x * float(unit[1])
        - relative_y * float(unit[0])
    )
    band_width = max(8.0, length * 0.035)
    sample_band = (
        (projection >= -0.02 * length)
        & (projection <= 1.02 * length)
        & (perpendicular <= band_width)
    )
    surrounding = (
        (projection >= 0.05 * length)
        & (projection <= 0.95 * length)
        & (perpendicular >= band_width * 0.55)
        & (perpendicular <= band_width)
    )
    surrounding_values = value[surrounding]
    if surrounding_values.size < 100:
        raise RuntimeError("not enough pixels around A4 divider")
    paper_value = float(np.median(surrounding_values))
    dark_limit = int(
        np.clip(
            paper_value * float(value_ratio),
            35.0,
            155.0,
        )
    )
    dark = sample_band & (value <= dark_limit)
    dark_y, dark_x = np.nonzero(dark)
    if len(dark_x) < max(80, int(length * 0.20)):
        raise RuntimeError("A4 divider line was not detected reliably")

    fitted = cv2.fitLine(
        np.column_stack((dark_x, dark_y)).astype(np.float32),
        cv2.DIST_HUBER,
        0,
        0.01,
        0.01,
    ).reshape(-1)
    point = np.asarray([fitted[2], fitted[3]], dtype=np.float32)
    direction = np.asarray([fitted[0], fitted[1]], dtype=np.float32)
    direction /= max(float(np.linalg.norm(direction)), 1e-6)
    normal = np.asarray(
        [-direction[1], direction[0]],
        dtype=np.float32,
    )
    support_offset = max(5.0, length * 0.006)
    dark_points = np.column_stack((dark_x, dark_y)).astype(np.float32)
    first_support = np.round(
        dark_points + normal * support_offset
    ).astype(np.int32)
    second_support = np.round(
        dark_points - normal * support_offset
    ).astype(np.int32)
    frame_height, frame_width = background.shape
    valid = (
        (first_support[:, 0] >= 0)
        & (first_support[:, 0] < frame_width)
        & (first_support[:, 1] >= 0)
        & (first_support[:, 1] < frame_height)
        & (second_support[:, 0] >= 0)
        & (second_support[:, 0] < frame_width)
        & (second_support[:, 1] >= 0)
        & (second_support[:, 1] < frame_height)
    )
    supported_points = dark_points[valid]
    first_support = first_support[valid]
    second_support = second_support[valid]
    if len(supported_points):
        supported = (
            background[
                first_support[:, 1],
                first_support[:, 0],
            ] > 0
        ) & (
            background[
                second_support[:, 1],
                second_support[:, 0],
            ] > 0
        )
        supported_points = supported_points[supported]
    if len(supported_points) < max(40, int(length * 0.10)):
        raise RuntimeError(
            "A4 divider endpoints do not have green support"
        )

    positions = (supported_points - point) @ direction
    minimum = float(np.percentile(positions, 0.5))
    maximum = float(np.percentile(positions, 99.5))
    first_endpoint = point + direction * minimum
    second_endpoint = point + direction * maximum
    return point, direction, first_endpoint, second_endpoint


def _detect_required_divider_line(
    frame: np.ndarray,
    background: np.ndarray,
    config: DetectConfig,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    """Detect the required long black divider inside the green paper."""
    contours, _ = cv2.findContours(
        background,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    frame_area = float(frame.shape[0] * frame.shape[1])
    paper_regions = [
        contour
        for contour in contours
        if cv2.contourArea(contour)
        >= frame_area * config.min_paper_area_ratio * 0.20
    ]
    if not paper_regions:
        raise RuntimeError("no sufficiently large green paper region")

    paper_hull = cv2.convexHull(
        np.concatenate(paper_regions, axis=0)
    )
    paper_mask = np.zeros(background.shape, dtype=np.uint8)
    cv2.drawContours(paper_mask, [paper_hull], -1, 255, -1)
    paper_mask = cv2.erode(
        paper_mask,
        np.ones(
            (odd_kernel(config.paper_erode_size),) * 2,
            dtype=np.uint8,
        ),
    )

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    paper_values = value[background > 0]
    if paper_values.size < 200:
        raise RuntimeError("not enough green pixels around divider")
    paper_value = float(np.median(paper_values))
    dark_limit = int(
        np.clip(
            paper_value * float(config.divider_value_ratio),
            35.0,
            155.0,
        )
    )
    dark = (
        (paper_mask > 0)
        & (value <= dark_limit)
        & (background == 0)
    ).astype(np.uint8) * 255
    dark = cv2.morphologyEx(
        dark,
        cv2.MORPH_OPEN,
        np.ones((3, 3), dtype=np.uint8),
    )
    dark_contours, _ = cv2.findContours(
        dark,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )

    _, _, paper_width, paper_height = cv2.boundingRect(paper_hull)
    required_length = 0.42 * min(paper_width, paper_height)
    paper_center = np.mean(
        paper_hull.reshape(-1, 2).astype(np.float32),
        axis=0,
    )
    paper_long_span = float(max(paper_width, paper_height))
    best = None
    best_score = -1.0
    for contour in dark_contours:
        if len(contour) < 20:
            continue
        (_, _), (rect_width, rect_height), _ = cv2.minAreaRect(
            contour
        )
        major = float(max(rect_width, rect_height))
        minor = float(max(1.0, min(rect_width, rect_height)))
        elongation = major / minor
        if major < required_length or elongation < 4.0:
            continue
        centroid = np.mean(
            contour.reshape(-1, 2).astype(np.float32),
            axis=0,
        )
        center_offset = float(
            np.linalg.norm(centroid - paper_center)
        ) / max(paper_long_span, 1.0)
        if center_offset > 0.38:
            continue
        score = major * min(elongation, 30.0) / (
            1.0 + 3.0 * center_offset
        )
        if score > best_score:
            best_score = score
            best = (contour, minor)

    if best is None:
        raise RuntimeError(
            "required black center divider was not detected"
        )

    divider_contour, divider_thickness = best
    points = divider_contour.reshape(-1, 2).astype(np.float32)
    fitted = cv2.fitLine(
        points,
        cv2.DIST_HUBER,
        0,
        0.01,
        0.01,
    ).reshape(-1)
    point = np.asarray([fitted[2], fitted[3]], dtype=np.float32)
    direction = np.asarray(
        [fitted[0], fitted[1]],
        dtype=np.float32,
    )
    direction /= max(float(np.linalg.norm(direction)), 1e-6)
    positions = (points - point) @ direction
    first_endpoint = point + direction * float(
        np.percentile(positions, 0.5)
    )
    second_endpoint = point + direction * float(
        np.percentile(positions, 99.5)
    )
    return (
        point,
        direction,
        first_endpoint,
        second_endpoint,
        divider_thickness,
    )


def rectify_a4_from_upper_half(
    frame: np.ndarray,
    background: np.ndarray,
    background_hue: int,
    config: DetectConfig,
) -> Tuple[np.ndarray, Dict[str, object]]:
    """Calibrate from the top corners and required middle divider."""
    frame_height, frame_width = frame.shape[:2]
    contours, _ = cv2.findContours(
        background,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    frame_area = float(frame_height * frame_width)
    candidates = [
        contour
        for contour in contours
        if cv2.contourArea(contour)
        >= frame_area * config.min_paper_area_ratio * 0.30
    ]
    candidates.sort(key=cv2.contourArea, reverse=True)
    if len(candidates) < 2:
        # The printed/drawn divider may stop just short of the paper
        # edge, or morphology may bridge that small gap. Detect the
        # actual black line and explicitly cut the green mask along it.
        detected_divider = _detect_required_divider_line(
            frame,
            background,
            config,
        )
        split_background = background.copy()
        point, direction = detected_divider[:2]
        diagonal = float(np.hypot(*frame.shape[:2]))
        split_start = np.round(
            point - direction * diagonal
        ).astype(np.int32)
        split_end = np.round(
            point + direction * diagonal
        ).astype(np.int32)
        split_thickness = max(
            config.paper_erode_size * 2 + 3,
            int(round(detected_divider[4] * 1.6)),
        )
        cv2.line(
            split_background,
            tuple(split_start),
            tuple(split_end),
            0,
            split_thickness,
            cv2.LINE_8,
        )
        contours, _ = cv2.findContours(
            split_background,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE,
        )
        candidates = [
            contour
            for contour in contours
            if cv2.contourArea(contour)
            >= frame_area * config.min_paper_area_ratio * 0.30
        ]
        candidates.sort(key=cv2.contourArea, reverse=True)
        if len(candidates) < 2:
            raise RuntimeError(
                "black divider could not separate the A4 halves"
            )

    halves = candidates[:2]
    combined_hull = cv2.convexHull(
        np.concatenate(halves, axis=0)
    )
    _, _, combined_width, combined_height = cv2.boundingRect(
        combined_hull
    )
    portrait = combined_height >= combined_width
    upper_axis = 1 if portrait else 0
    upper_contour = min(
        halves,
        key=lambda contour: contour_centroid(contour)[upper_axis],
    )
    validate_a4_not_clipped(upper_contour, frame.shape)

    source = _quadrilateral_from_contour(upper_contour)
    if not portrait:
        source = source[[3, 0, 1, 2]]

    contour_points = upper_contour.reshape(-1, 2).astype(np.float32)
    top_line = fit_contour_side(
        contour_points,
        source[0],
        source[1],
    )
    right_line = fit_contour_side(
        contour_points,
        source[1],
        source[2],
    )
    left_line = fit_contour_side(
        contour_points,
        source[3],
        source[0],
    )
    divider_line = _fit_divider_center_line(
        frame,
        background,
        source[3],
        source[2],
        config.divider_value_ratio,
    )
    divider_endpoints = [divider_line[2], divider_line[3]]
    divider_left = min(
        divider_endpoints,
        key=lambda point: float(np.linalg.norm(point - source[3])),
    )
    divider_right = min(
        divider_endpoints,
        key=lambda point: float(np.linalg.norm(point - source[2])),
    )
    calibration_source = np.asarray(
        [
            line_intersection(
                left_line[0],
                left_line[1],
                top_line[0],
                top_line[1],
            ),
            line_intersection(
                top_line[0],
                top_line[1],
                right_line[0],
                right_line[1],
            ),
            divider_right,
            divider_left,
        ],
        dtype=np.float32,
    )
    if not cv2.isContourConvex(
        calibration_source.reshape(-1, 1, 2)
    ):
        raise RuntimeError(
            "upper-half A4 calibration points are not convex"
        )

    top_width = float(
        np.linalg.norm(calibration_source[1] - calibration_source[0])
    )
    divider_width = float(
        np.linalg.norm(calibration_source[2] - calibration_source[3])
    )
    left_height = float(
        np.linalg.norm(calibration_source[3] - calibration_source[0])
    )
    right_height = float(
        np.linalg.norm(calibration_source[2] - calibration_source[1])
    )
    observed_aspect = (
        0.5 * (top_width + divider_width)
        / max(0.5 * (left_height + right_height), 1e-6)
    )
    expected_aspect = A4_SHORT_MM / (A4_LONG_MM * 0.5)
    if abs(np.log(observed_aspect / expected_aspect)) > 0.35:
        raise RuntimeError(
            "upper-half A4 aspect ratio is unreliable: {:.3f}".format(
                observed_aspect
            )
        )

    width_px = int(round(A4_SHORT_MM * PIXELS_PER_MM))
    height_px = int(round(A4_LONG_MM * PIXELS_PER_MM))
    divider_y_px = (height_px - 1) * 0.5
    destination_half = np.asarray(
        [
            [0, 0],
            [width_px - 1, 0],
            [width_px - 1, divider_y_px],
            [0, divider_y_px],
        ],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(
        calibration_source,
        destination_half,
    )
    rectified = cv2.warpPerspective(
        frame,
        homography,
        (width_px, height_px),
    )

    full_destination = np.asarray(
        [
            [
                [0, 0],
                [width_px - 1, 0],
                [width_px - 1, height_px - 1],
                [0, height_px - 1],
            ]
        ],
        dtype=np.float32,
    )
    predicted_full_corners = cv2.perspectiveTransform(
        full_destination,
        np.linalg.inv(homography),
    )[0]
    full_margin_x = frame_width * 0.08
    full_margin_y = frame_height * 0.08
    if np.any(
        (predicted_full_corners[:, 0] < -full_margin_x)
        | (
            predicted_full_corners[:, 0]
            > frame_width - 1 + full_margin_x
        )
        | (predicted_full_corners[:, 1] < -full_margin_y)
        | (
            predicted_full_corners[:, 1]
            > frame_height - 1 + full_margin_y
        )
    ):
        raise RuntimeError(
            "blue-card extrapolation places the full A4 outside "
            "the camera frame"
        )
    calibration = {
        "paper_corners_px": np.round(
            predicted_full_corners,
            1,
        ).tolist(),
        "calibration_points_px": np.round(
            calibration_source,
            1,
        ).tolist(),
        "divider_points_px": np.round(
            calibration_source[[3, 2]],
            1,
        ).tolist(),
        "paper_size_mm": [A4_SHORT_MM, A4_LONG_MM],
        "pixels_per_mm": PIXELS_PER_MM,
        "homography": np.round(homography, 8).tolist(),
        "background_hue_before_rectify": background_hue,
        "input_orientation": (
            "portrait" if portrait else "landscape"
        ),
        "calibration_region": (
            "top_half" if portrait else "left_half"
        ),
        "canonical_orientation": "portrait",
        "calibration_method": "upper_half_divider",
        "coordinate_system": {
            "origin": "canonical_top_left",
            "x_axis": "A4_short_edge_0_to_210_mm",
            "y_axis": "A4_long_edge_0_to_297_mm",
        },
    }
    return rectified, calibration


def rectify_a4_from_three_corners(
    frame: np.ndarray,
    background: np.ndarray,
    background_hue: int,
    config: DetectConfig,
) -> Tuple[np.ndarray, Dict[str, object]]:
    """Estimate one hidden A4 corner from three visible outer corners."""
    contours, _ = cv2.findContours(
        background,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    frame_area = float(frame.shape[0] * frame.shape[1])
    regions = [
        contour
        for contour in contours
        if cv2.contourArea(contour)
        >= frame_area * config.min_paper_area_ratio * 0.18
    ]
    if not regions:
        raise RuntimeError("no green A4 region for three-corner estimate")

    paper_hull = cv2.convexHull(np.concatenate(regions, axis=0))
    hull_area = max(1.0, float(cv2.contourArea(paper_hull)))
    perimeter = cv2.arcLength(paper_hull, True)
    candidate_points = None
    for epsilon_ratio in (0.012, 0.009, 0.006, 0.018, 0.025):
        approximated = cv2.approxPolyDP(
            paper_hull,
            perimeter * epsilon_ratio,
            True,
        ).reshape(-1, 2)
        if 4 <= len(approximated) <= 9:
            candidate_points = approximated.astype(np.float32)
            break
    if candidate_points is None:
        raise RuntimeError(
            "three-corner A4 candidate count is unreliable"
        )

    frame_height, frame_width = frame.shape[:2]
    destination_portrait = np.asarray(
        [
            [0, 0],
            [A4_SHORT_MM * PIXELS_PER_MM - 1, 0],
            [
                A4_SHORT_MM * PIXELS_PER_MM - 1,
                A4_LONG_MM * PIXELS_PER_MM - 1,
            ],
            [0, A4_LONG_MM * PIXELS_PER_MM - 1],
        ],
        dtype=np.float32,
    )
    expected_aspect = A4_LONG_MM / A4_SHORT_MM
    best = None
    best_score = float("inf")

    for selected in combinations(candidate_points, 3):
        observed = np.asarray(selected, dtype=np.float32)
        pair_distances = [
            (
                float(np.linalg.norm(observed[first] - observed[second])),
                first,
                second,
            )
            for first, second in ((0, 1), (0, 2), (1, 2))
        ]
        _, first, second = max(pair_distances)
        corner_index = 3 - first - second
        inferred = (
            observed[first]
            + observed[second]
            - observed[corner_index]
        )
        quad = order_corners(
            np.vstack((observed, inferred)).astype(np.float32)
        )
        inferred_index = int(
            np.argmin(
                np.linalg.norm(quad - inferred, axis=1)
            )
        )
        # The camera fixture always exposes the top-left, top-right, and
        # bottom-left A4 corners.  The bottom-right corner is obstructed and
        # must therefore always be reconstructed rather than treated as an
        # observed hull point.
        if inferred_index != 2:
            continue
        polygon = quad.reshape(-1, 1, 2)
        if not cv2.isContourConvex(polygon):
            continue

        quad_area = abs(float(cv2.contourArea(polygon)))
        area_ratio = quad_area / hull_area
        if not 0.82 <= area_ratio <= 1.65:
            continue

        top = float(np.linalg.norm(quad[1] - quad[0]))
        bottom = float(np.linalg.norm(quad[2] - quad[3]))
        left = float(np.linalg.norm(quad[3] - quad[0]))
        right = float(np.linalg.norm(quad[2] - quad[1]))
        mean_width = 0.5 * (top + bottom)
        mean_height = 0.5 * (left + right)
        if min(mean_width, mean_height) < 40.0:
            continue
        observed_aspect = max(mean_width, mean_height) / min(
            mean_width,
            mean_height,
        )
        aspect_error = abs(
            np.log(observed_aspect / expected_aspect)
        )
        if aspect_error > 0.30:
            continue

        portrait = mean_height >= mean_width
        source = quad if portrait else quad[[3, 0, 1, 2]]
        homography = cv2.getPerspectiveTransform(
            source.astype(np.float32),
            destination_portrait,
        )
        validation = cv2.warpPerspective(
            background,
            homography,
            (
                int(round(A4_SHORT_MM)),
                int(round(A4_LONG_MM)),
            ),
            flags=cv2.INTER_NEAREST,
        )
        coverage = float(np.mean(validation > 0))
        if coverage < 0.52:
            continue

        outside_x = max(
            0.0,
            -float(inferred[0]),
            float(inferred[0]) - (frame_width - 1),
        ) / max(frame_width, 1)
        outside_y = max(
            0.0,
            -float(inferred[1]),
            float(inferred[1]) - (frame_height - 1),
        ) / max(frame_height, 1)
        if max(outside_x, outside_y) > 0.22:
            continue

        score = (
            4.0 * aspect_error
            + 0.8 * abs(1.0 - area_ratio)
            + 1.5 * (1.0 - coverage)
            + 2.0 * (outside_x + outside_y)
        )
        if score < best_score:
            best_score = score
            best = (
                source,
                quad,
                inferred,
                portrait,
                homography,
                coverage,
            )

    if best is None or best_score > 1.35:
        raise RuntimeError(
            "no reliable three-corner A4 geometry was found"
        )

    source, image_quad, inferred, portrait, homography, coverage = best
    contour_points = np.concatenate(regions, axis=0).reshape(
        -1,
        2,
    ).astype(np.float32)
    try:
        top_point, top_direction = fit_contour_side(
            contour_points,
            image_quad[0],
            image_quad[1],
        )
        right_point, right_direction = fit_contour_side(
            contour_points,
            image_quad[1],
            image_quad[2],
        )
        measured_top_right = line_intersection(
            top_point,
            top_direction,
            right_point,
            right_direction,
        )
        refinement_distance = float(
            np.linalg.norm(
                measured_top_right - image_quad[1]
            )
        )
        if refinement_distance <= max(
            50.0,
            0.06 * np.hypot(frame_width, frame_height),
        ):
            image_quad = image_quad.copy()
            image_quad[1] = measured_top_right
            # Only TL, TR, and BL are physically visible in this fixture.
            # BR must always be reconstructed from those three observations.
            image_quad[2] = (
                image_quad[1]
                + image_quad[3]
                - image_quad[0]
            )
            inferred = image_quad[2].copy()
            top = float(
                np.linalg.norm(image_quad[1] - image_quad[0])
            )
            bottom = float(
                np.linalg.norm(image_quad[2] - image_quad[3])
            )
            left = float(
                np.linalg.norm(image_quad[3] - image_quad[0])
            )
            right = float(
                np.linalg.norm(image_quad[2] - image_quad[1])
            )
            portrait = 0.5 * (left + right) >= 0.5 * (
                top + bottom
            )
            source = (
                image_quad
                if portrait
                else image_quad[[3, 0, 1, 2]]
            )
            homography = cv2.getPerspectiveTransform(
                source.astype(np.float32),
                destination_portrait,
            )
            validation = cv2.warpPerspective(
                background,
                homography,
                (
                    int(round(A4_SHORT_MM)),
                    int(round(A4_LONG_MM)),
                ),
                flags=cv2.INTER_NEAREST,
            )
            coverage = float(np.mean(validation > 0))
    except RuntimeError:
        pass

    width_px = int(round(A4_SHORT_MM * PIXELS_PER_MM))
    height_px = int(round(A4_LONG_MM * PIXELS_PER_MM))
    rectified = cv2.warpPerspective(
        frame,
        homography,
        (width_px, height_px),
    )
    calibration = {
        "paper_corners_px": np.round(source, 1).tolist(),
        "image_ordered_corners_px": np.round(
            image_quad,
            1,
        ).tolist(),
        "inferred_corner_px": np.round(inferred, 1).tolist(),
        "inferred_corner_role": "bottom_right",
        "paper_size_mm": [A4_SHORT_MM, A4_LONG_MM],
        "pixels_per_mm": PIXELS_PER_MM,
        "homography": np.round(homography, 8).tolist(),
        "background_hue_before_rectify": background_hue,
        "input_orientation": (
            "portrait" if portrait else "landscape"
        ),
        "calibration_region": "three_outer_corners",
        "canonical_orientation": "portrait",
        "calibration_method": "three_a4_corners",
        "three_corner_score": round(float(best_score), 4),
        "green_coverage": round(float(coverage), 4),
        "coordinate_system": {
            "origin": "canonical_top_left",
            "x_axis": "A4_short_edge_0_to_210_mm",
            "y_axis": "A4_long_edge_0_to_297_mm",
        },
    }
    return rectified, calibration


def create_upper_half_fixed_calibration(
    frame: np.ndarray,
    config: DetectConfig,
    target_hue: Optional[int] = None,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
    """Calibrate global coordinates from a 105 x 297 mm blue upper half."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    frame_height, frame_width = frame.shape[:2]
    frame_area = float(frame_height * frame_width)

    if target_hue is None:
        top = max(0, int(round(frame_height * 0.04)))
        bottom = min(frame_height, int(round(frame_height * 0.58)))
        left = max(0, int(round(frame_width * 0.08)))
        right = min(frame_width, int(round(frame_width * 0.92)))
        roi_hue = hue[top:bottom, left:right]
        roi_saturation = saturation[top:bottom, left:right]
        roi_value = value[top:bottom, left:right]
        colorful = (
            (roi_saturation >= config.saturation_min)
            & (roi_value >= config.value_min)
        )
        if not np.any(colorful):
            raise RuntimeError(
                "no saturated blue calibration card in upper image"
            )
        histogram = np.bincount(
            roi_hue[colorful].ravel(),
            minlength=180,
        ).astype(np.float64)
        hue_candidates = []
        minimum_separation = max(5, config.hue_tolerance)
        for candidate in np.argsort(histogram)[::-1]:
            candidate = int(candidate)
            if all(
                min(
                    abs(candidate - previous),
                    180 - abs(candidate - previous),
                )
                >= minimum_separation
                for previous in hue_candidates
            ):
                hue_candidates.append(candidate)
            if len(hue_candidates) >= 6:
                break
    else:
        hue_candidates = [int(target_hue) % 180]

    expected_aspect = A4_LONG_MM / (A4_SHORT_MM * 0.5)
    calibration_mask_config = replace(
        config,
        hue_tolerance=min(config.hue_tolerance, 8),
        saturation_min=max(config.saturation_min, 200),
    )
    best = None
    best_score = float("inf")
    for candidate_hue in hue_candidates:
        mask = build_mask_for_hue(
            hue,
            saturation,
            value,
            candidate_hue,
            calibration_mask_config,
        )
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            np.ones(
                (odd_kernel(config.open_size),) * 2,
                dtype=np.uint8,
            ),
        )
        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE,
        )
        for contour in contours:
            contour_area = float(cv2.contourArea(contour))
            area_ratio = contour_area / frame_area
            if not 0.08 <= area_ratio <= 0.75:
                continue
            center_x, center_y = contour_centroid(contour)
            if center_y >= frame_height * 0.65:
                continue

            hull = cv2.convexHull(contour)
            perimeter = cv2.arcLength(hull, True)
            corners = None
            for epsilon_ratio in (
                0.012,
                0.009,
                0.018,
                0.006,
                0.025,
                0.035,
            ):
                approximated = cv2.approxPolyDP(
                    hull,
                    perimeter * epsilon_ratio,
                    True,
                )
                if len(approximated) == 4:
                    corners = approximated.reshape(-1, 2)
                    break
            if corners is None:
                continue

            source = order_corners(corners)
            source = refine_a4_corners(contour, source)
            if not cv2.isContourConvex(
                source.reshape(-1, 1, 2)
            ):
                continue
            top_width = float(
                np.linalg.norm(source[1] - source[0])
            )
            bottom_width = float(
                np.linalg.norm(source[2] - source[3])
            )
            left_height = float(
                np.linalg.norm(source[3] - source[0])
            )
            right_height = float(
                np.linalg.norm(source[2] - source[1])
            )
            mean_width = 0.5 * (top_width + bottom_width)
            mean_height = 0.5 * (left_height + right_height)
            if min(mean_width, mean_height) < 60.0:
                continue
            observed_aspect = mean_width / mean_height
            aspect_error = abs(
                np.log(observed_aspect / expected_aspect)
            )
            if aspect_error > 0.25:
                continue
            quad_area = abs(
                float(
                    cv2.contourArea(
                        source.reshape(-1, 1, 2)
                    )
                )
            )
            coverage_error = abs(
                1.0 - contour_area / max(quad_area, 1.0)
            )
            upper_position_penalty = max(
                0.0,
                center_y / frame_height - 0.38,
            )
            score = (
                4.0 * aspect_error
                + coverage_error
                + 2.0 * upper_position_penalty
                - 0.15 * area_ratio
            )
            if score < best_score:
                best_score = score
                best = (
                    source,
                    mask,
                    candidate_hue,
                    observed_aspect,
                )

    if best is None:
        raise RuntimeError(
            "no reliable 105x297 mm blue card found in upper image"
        )

    source, blue_mask, blue_hue, observed_aspect = best
    width_px = int(round(A4_SHORT_MM * PIXELS_PER_MM))
    height_px = int(round(A4_LONG_MM * PIXELS_PER_MM))
    half_x_px = (width_px - 1) * 0.5
    # Source order is camera TL, TR, BR, BL. In global coordinates these
    # points are (210,0), (210,297), (105,297), (105,0).
    destination = np.asarray(
        [
            [width_px - 1, 0],
            [width_px - 1, height_px - 1],
            [half_x_px, height_px - 1],
            [half_x_px, 0],
        ],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(
        source.astype(np.float32),
        destination,
    )
    rectified = cv2.warpPerspective(
        frame,
        homography,
        (width_px, height_px),
    )
    full_destination = np.asarray(
        [
            [
                [0, 0],
                [width_px - 1, 0],
                [width_px - 1, height_px - 1],
                [0, height_px - 1],
            ]
        ],
        dtype=np.float32,
    )
    predicted_full_corners = cv2.perspectiveTransform(
        full_destination,
        np.linalg.inv(homography),
    )[0]
    calibration = {
        "schema_version": 1,
        "calibration_method": "fixed_upper_half_blue",
        "calibration_region": "camera_upper_half",
        "input_frame_size_px": [frame_width, frame_height],
        "calibration_points_px": np.round(source, 3).tolist(),
        "calibration_points_mm": [
            [210.0, 0.0],
            [210.0, 297.0],
            [105.0, 297.0],
            [105.0, 0.0],
        ],
        "paper_corners_px": np.round(
            predicted_full_corners,
            3,
        ).tolist(),
        "paper_size_mm": [A4_SHORT_MM, A4_LONG_MM],
        "pixels_per_mm": PIXELS_PER_MM,
        "homography": np.round(homography, 10).tolist(),
        "blue_hue": int(blue_hue),
        "observed_half_aspect": round(
            float(observed_aspect),
            5,
        ),
        "calibration_score": round(float(best_score), 5),
        "global_x_range_mm": [105.0, 210.0],
        "global_y_range_mm": [0.0, 297.0],
        "canonical_orientation": "portrait",
        "coordinate_system": {
            "origin": "canonical_top_left",
            "x_axis": "A4_short_edge_0_to_210_mm",
            "y_axis": "A4_long_edge_0_to_297_mm",
        },
    }
    return blue_mask, rectified, calibration


def rectify_a4_from_fixed_calibration(
    frame: np.ndarray,
    calibration: Dict[str, object],
) -> Tuple[np.ndarray, Dict[str, object]]:
    """Warp a frame with a previously saved, camera-fixed homography."""
    expected_size = calibration.get("input_frame_size_px")
    actual_size = [frame.shape[1], frame.shape[0]]
    if expected_size is None or list(expected_size) != actual_size:
        raise RuntimeError(
            "fixed calibration frame size {} does not match {}".format(
                expected_size,
                actual_size,
            )
        )
    homography = np.asarray(
        calibration.get("homography"),
        dtype=np.float64,
    )
    if homography.shape != (3, 3) or not np.all(
        np.isfinite(homography)
    ):
        raise RuntimeError("fixed calibration homography is invalid")
    width_px = int(round(A4_SHORT_MM * PIXELS_PER_MM))
    height_px = int(round(A4_LONG_MM * PIXELS_PER_MM))
    rectified = cv2.warpPerspective(
        frame,
        homography,
        (width_px, height_px),
    )
    runtime_calibration = dict(calibration)
    runtime_calibration["calibration_method"] = (
        "fixed_upper_half_blue"
    )
    runtime_calibration["calibration_region"] = "saved_global"
    runtime_calibration["fixed_calibration_loaded"] = True
    return rectified, runtime_calibration


def rectify_a4(
    frame: np.ndarray,
    config: DetectConfig,
) -> Tuple[np.ndarray, Dict[str, object]]:
    """检测湖蓝 A4 四角并矫正为固定比例的俯视图。"""
    if config.fixed_calibration is not None:
        return rectify_a4_from_fixed_calibration(
            frame,
            config.fixed_calibration,
        )
    background, background_hue = build_blue_background_mask(frame, config)
    if config.calibration_strategy == "three_corners":
        return rectify_a4_from_three_corners(
            frame,
            background,
            background_hue,
            config,
        )
    if config.calibration_strategy not in ("auto", "divider"):
        raise ValueError(
            "unknown calibration strategy: {}".format(
                config.calibration_strategy
            )
        )
    divider_error = None
    try:
        return rectify_a4_from_upper_half(
            frame,
            background,
            background_hue,
            config,
        )
    except RuntimeError as error:
        divider_error = str(error)
        if (
            config.require_divider_calibration
            or config.calibration_strategy == "divider"
        ):
            try:
                return rectify_a4_from_three_corners(
                    frame,
                    background,
                    background_hue,
                    config,
                )
            except RuntimeError as three_corner_error:
                raise RuntimeError(
                    "divider calibration failed: {}; "
                    "three-corner calibration failed: {}".format(
                        divider_error,
                        three_corner_error,
                    )
                ) from three_corner_error

    contours, _ = cv2.findContours(
        background,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    if not contours:
        raise RuntimeError("没有找到湖蓝 A4 卡纸")

    paper_contour = max(contours, key=cv2.contourArea)
    try:
        validate_a4_not_clipped(paper_contour, frame.shape)
    except RuntimeError as full_error:
        raise RuntimeError(
            "divider calibration failed: {}; "
            "full-A4 fallback failed: {}".format(
                divider_error,
                full_error,
            )
        ) from full_error
    frame_area = frame.shape[0] * frame.shape[1]
    paper_area_ratio = cv2.contourArea(paper_contour) / frame_area
    if paper_area_ratio < config.min_paper_area_ratio:
        raise RuntimeError(
            "湖蓝 A4 卡纸在画面中占比过小：{:.1%}".format(
                paper_area_ratio
            )
        )

    hull = cv2.convexHull(paper_contour)
    perimeter = cv2.arcLength(hull, True)
    corners = cv2.approxPolyDP(hull, perimeter * 0.015, True)
    if len(corners) == 4:
        source = select_a4_quadrilateral(
            corners,
            paper_contour,
        )
    else:
        candidate_corners = cv2.approxPolyDP(
            hull,
            perimeter * 0.010,
            True,
        )
        source = select_a4_quadrilateral(
            candidate_corners,
            paper_contour,
        )
    source = refine_a4_corners(paper_contour, source)
    validate_a4_source(background, source)

    # 固定纸面坐标系：
    # X始终沿A4短边（0~210 mm），Y始终沿A4长边（0~297 mm）。
    # 横向画面统一顺时针旋转为规范竖向画面。
    image_ordered_corners = source.copy()
    top_width = np.linalg.norm(source[1] - source[0])
    bottom_width = np.linalg.norm(source[2] - source[3])
    left_height = np.linalg.norm(source[3] - source[0])
    right_height = np.linalg.norm(source[2] - source[1])
    portrait = (left_height + right_height) >= (top_width + bottom_width)
    input_orientation = "portrait" if portrait else "landscape"
    if not portrait:
        # 原图左下、左上、右上、右下分别映射到规范图
        # 左上、右上、右下、左下。
        source = source[[3, 0, 1, 2]]

    width_mm = A4_SHORT_MM
    height_mm = A4_LONG_MM
    width_px = int(round(width_mm * PIXELS_PER_MM))
    height_px = int(round(height_mm * PIXELS_PER_MM))
    destination = np.asarray(
        [
            [0, 0],
            [width_px - 1, 0],
            [width_px - 1, height_px - 1],
            [0, height_px - 1],
        ],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(source, destination)
    rectified = cv2.warpPerspective(
        frame,
        homography,
        (width_px, height_px),
    )
    calibration = {
        "paper_corners_px": np.round(source, 1).tolist(),
        "image_ordered_corners_px": np.round(
            image_ordered_corners,
            1,
        ).tolist(),
        "paper_size_mm": [width_mm, height_mm],
        "pixels_per_mm": PIXELS_PER_MM,
        "homography": np.round(homography, 8).tolist(),
        "background_hue_before_rectify": background_hue,
        "input_orientation": input_orientation,
        "canonical_orientation": "portrait",
        "calibration_method": "full_a4_corners",
        "divider_fallback_error": divider_error,
        "coordinate_system": {
            "origin": "canonical_top_left",
            "x_axis": "A4_short_edge_0_to_210_mm",
            "y_axis": "A4_long_edge_0_to_297_mm",
        },
    }
    return rectified, calibration


def build_piece_mask(
    frame: np.ndarray,
    config: DetectConfig,
) -> Tuple[np.ndarray, int]:
    """提取矫正后湖蓝卡纸范围内的非湖蓝碎片。"""
    if config.include_dark_artwork_in_piece_mask:
        # Mode 3 uses direct inverse-green segmentation. Learn the current
        # camera's green hue instead of hard-coding [35, 85], because the
        # observed background can move above 85 between cameras/exposures.
        # Black artwork is therefore foreground card material even when it
        # touches a cut edge. Do not close the green mask here: a large close
        # would fill the non-green artwork hole and recreate a false inward
        # notch after inversion.
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        background_hue = learn_blue_hue(hsv)
        hue, saturation, value = cv2.split(hsv)
        hue_delta = np.abs(hue.astype(np.int16) - background_hue)
        hue_delta = np.minimum(hue_delta, 180 - hue_delta)
        background = np.where(
            (hue_delta <= int(config.hue_tolerance))
            & (saturation >= int(config.saturation_min))
            & (value >= int(config.value_min)),
            255,
            0,
        ).astype(np.uint8)
    else:
        background, background_hue = build_blue_background_mask(
            frame,
            config,
        )
    contours, _ = cv2.findContours(
        background,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    if not contours:
        raise RuntimeError("没有找到湖蓝卡纸区域")

    frame_area = frame.shape[0] * frame.shape[1]
    paper_regions = [
        contour
        for contour in contours
        if cv2.contourArea(contour) >= frame_area * 0.03
    ]
    if not paper_regions:
        paper_regions = [max(contours, key=cv2.contourArea)]
    paper_regions.sort(key=cv2.contourArea, reverse=True)
    paper_contour = cv2.convexHull(
        np.concatenate(paper_regions[:2], axis=0)
    )
    paper_support = np.zeros(frame.shape[:2], dtype=np.uint8)
    cv2.drawContours(
        paper_support,
        [paper_contour],
        -1,
        255,
        thickness=cv2.FILLED,
    )
    frame_height, frame_width = frame.shape[:2]
    filled_ratio = float(np.mean(paper_support > 0))
    horizontal_inset = max(3, int(round(frame_width * 0.008)))
    vertical_inset = max(3, int(round(frame_height * 0.008)))
    horizontal_band = max(7, int(round(frame_width * 0.025)))
    vertical_band = max(7, int(round(frame_height * 0.025)))
    edge_ratios = [
        float(
            np.mean(
                paper_support[
                    vertical_inset:
                    vertical_inset + vertical_band,
                    :,
                ] > 0
            )
        ),
        float(
            np.mean(
                paper_support[
                    -vertical_inset - vertical_band:
                    -vertical_inset,
                    :,
                ] > 0
            )
        ),
        float(
            np.mean(
                paper_support[
                    :,
                    horizontal_inset:
                    horizontal_inset + horizontal_band,
                ] > 0
            )
        ),
        float(
            np.mean(
                paper_support[
                    :,
                    -horizontal_inset - horizontal_band:
                    -horizontal_inset,
                ] > 0
            )
        ),
    ]
    mean_edge_ratio = float(np.mean(edge_ratios))
    extremely_low_edges = sum(
        value < 0.10 for value in edge_ratios
    )
    if (
        filled_ratio < 0.75
        or mean_edge_ratio < 0.35
        or extremely_low_edges >= 2
    ):
        raise RuntimeError(
            "矫正后的A4边界不完整，拒绝输出坐标 "
            "fill={:.2f}, edges={}".format(
                filled_ratio,
                [round(value, 2) for value in edge_ratios],
            )
        )
    paper_support = cv2.erode(
        paper_support,
        np.ones(
            (odd_kernel(config.paper_erode_size),) * 2,
            dtype=np.uint8,
        ),
    )

    mask = cv2.bitwise_and(
        cv2.bitwise_not(background),
        paper_support,
    )
    # Dark card artwork can share the green paper's camera hue and be
    # swallowed by the background mask.  Add only clearly dark pixels inside
    # the verified paper support back to the piece foreground.  They become
    # part of a fragment only when the following morphology connects them to
    # its white stock; isolated dark paper marks remain too small to survive
    # contour filtering.  The divider is removed explicitly below.
    if config.include_dark_artwork_in_piece_mask:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        value = hsv[:, :, 2]
        background_values = value[background > 0]
    else:
        background_values = np.asarray([], dtype=np.uint8)
    if background_values.size:
        background_value = float(np.median(background_values))
        dark_ink_limit = int(
            round(
                max(
                    45.0,
                    min(
                        105.0,
                        background_value
                        * float(config.artwork_dark_value_ratio),
                    ),
                )
            )
        )
        dark_ink = np.where(
            (value <= dark_ink_limit) & (paper_support > 0),
            255,
            0,
        ).astype(np.uint8)
        mask = cv2.bitwise_or(mask, dark_ink)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        np.ones((odd_kernel(config.open_size),) * 2, dtype=np.uint8),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        np.ones((odd_kernel(config.close_size),) * 2, dtype=np.uint8),
    )
    divider_y = frame_height // 2
    divider_half_band = max(
        2,
        int(
            round(
                config.divider_exclusion_mm * PIXELS_PER_MM
            )
        ),
    )
    mask[
        max(0, divider_y - divider_half_band):
        min(frame_height, divider_y + divider_half_band + 1),
        :,
    ] = 0
    if config.source_half == "lower":
        mask[
            :min(
                frame_height,
                divider_y + divider_half_band + 1,
            ),
            :,
        ] = 0
    elif config.source_half == "upper":
        mask[
            max(0, divider_y - divider_half_band):,
            :,
        ] = 0
    elif config.source_half not in ("both", ""):
        raise ValueError(
            "source_half must be 'upper', 'lower', or 'both'"
        )
    return mask, background_hue


def contour_centroid(contour: np.ndarray) -> Tuple[float, float]:
    """将碎片视为厚度和密度均匀的薄片，计算其面积重心。"""
    moments = cv2.moments(contour)
    if abs(moments["m00"]) < 1e-8:
        x, y, width, height = cv2.boundingRect(contour)
        return x + width / 2.0, y + height / 2.0
    return (
        moments["m10"] / moments["m00"],
        moments["m01"] / moments["m00"],
    )


def remove_false_short_edges(
    polygon: np.ndarray,
    reference_contour: np.ndarray,
    minimum_length_px: float,
) -> np.ndarray:
    """合并明显短于题目下限的毛刺边，并尽量保持原轮廓面积。"""
    points = polygon.reshape(-1, 2)
    reference_area = abs(float(cv2.contourArea(reference_contour)))
    while len(points) > 3:
        lengths = np.linalg.norm(
            np.roll(points, -1, axis=0) - points,
            axis=1,
        )
        edge_index = int(np.argmin(lengths))
        if float(lengths[edge_index]) >= minimum_length_px:
            break

        next_index = (edge_index + 1) % len(points)
        candidates = [
            np.delete(points, edge_index, axis=0),
            np.delete(points, next_index, axis=0),
        ]
        points = min(
            candidates,
            key=lambda candidate: abs(
                abs(
                    float(
                        cv2.contourArea(
                            candidate.astype(np.float32).reshape(-1, 1, 2)
                        )
                    )
                )
                - reference_area
            ),
        )
    return points.astype(np.int32).reshape(-1, 1, 2)


def remove_nearly_collinear_vertices(
    polygon: np.ndarray,
    minimum_interior_angle_deg: float = 165.0,
) -> np.ndarray:
    """Merge vertices that only split one almost-straight physical edge."""
    points = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
    while len(points) > 3:
        angles = []
        for vertex_id, vertex in enumerate(points):
            previous_vector = points[vertex_id - 1] - vertex
            next_vector = points[
                (vertex_id + 1) % len(points)
            ] - vertex
            denominator = max(
                1e-6,
                float(
                    np.linalg.norm(previous_vector)
                    * np.linalg.norm(next_vector)
                ),
            )
            cosine_value = np.clip(
                float(
                    np.dot(previous_vector, next_vector)
                )
                / denominator,
                -1.0,
                1.0,
            )
            angles.append(
                float(np.degrees(np.arccos(cosine_value)))
            )
        vertex_id = int(np.argmax(angles))
        if angles[vertex_id] < minimum_interior_angle_deg:
            break
        points = np.delete(points, vertex_id, axis=0)
    return np.round(points).astype(np.int32).reshape(-1, 1, 2)


def collapse_artwork_split_edge_vertices(
    polygon: np.ndarray,
    minimum_interior_angle_deg: float = 155.0,
    maximum_line_offset_px: float = 3.5 * PIXELS_PER_MM,
) -> np.ndarray:
    """Merge a printed-artwork split point that still lies near one edge.

    This is intentionally stricter than generic collinear cleanup: the point
    must both form a very obtuse angle and remain within a few millimetres of
    the line through its neighbours.  A genuine obtuse puzzle corner normally
    has a much larger line offset and is therefore preserved.
    """
    points = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
    while len(points) > 3:
        candidates = []
        for vertex_id, vertex in enumerate(points):
            previous = points[vertex_id - 1]
            following = points[(vertex_id + 1) % len(points)]
            previous_vector = previous - vertex
            following_vector = following - vertex
            denominator = max(
                1e-6,
                float(
                    np.linalg.norm(previous_vector)
                    * np.linalg.norm(following_vector)
                ),
            )
            cosine_value = np.clip(
                float(np.dot(previous_vector, following_vector))
                / denominator,
                -1.0,
                1.0,
            )
            interior_angle = float(
                np.degrees(np.arccos(cosine_value))
            )
            neighbour_line = following - previous
            neighbour_distance = max(
                1e-6,
                float(np.linalg.norm(neighbour_line)),
            )
            line_offset = abs(
                float(np.cross(neighbour_line, vertex - previous))
            ) / neighbour_distance
            if (
                interior_angle >= minimum_interior_angle_deg
                and line_offset <= maximum_line_offset_px
            ):
                candidates.append(
                    (line_offset, -interior_angle, vertex_id)
                )
        if not candidates:
            break
        _, _, vertex_id = min(candidates)
        points = np.delete(points, vertex_id, axis=0)
    return np.round(points).astype(np.int32).reshape(-1, 1, 2)


def reconstruct_split_corner(
    polygon: np.ndarray,
    reference_contour: np.ndarray,
) -> np.ndarray:
    """Replace a short artificial split edge by the adjacent-line crossing."""
    points = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
    if len(points) not in (4, 5):
        return polygon

    lengths = np.linalg.norm(
        np.roll(points, -1, axis=0) - points,
        axis=1,
    )
    reference_area = max(
        1.0,
        abs(float(cv2.contourArea(reference_contour))),
    )
    candidates = []
    for edge_id, short_length in enumerate(lengths):
        previous_length = float(lengths[edge_id - 1])
        following_length = float(
            lengths[(edge_id + 1) % len(points)]
        )
        adjacent_minimum = min(previous_length, following_length)
        if (
            adjacent_minimum < 1e-6
            or float(short_length) / adjacent_minimum > 0.35
        ):
            continue

        first_start = points[edge_id - 1]
        first_end = points[edge_id]
        second_start = points[(edge_id + 1) % len(points)]
        second_end = points[(edge_id + 2) % len(points)]
        try:
            crossing = line_intersection(
                first_start,
                first_end - first_start,
                second_start,
                second_end - second_start,
            )
        except RuntimeError:
            continue
        endpoint_shift = max(
            float(np.linalg.norm(crossing - first_end)),
            float(np.linalg.norm(crossing - second_start)),
        )
        if endpoint_shift > adjacent_minimum * 0.55:
            continue

        rebuilt = []
        for point_id, point in enumerate(points):
            if point_id == edge_id:
                rebuilt.append(crossing)
            elif point_id == (edge_id + 1) % len(points):
                continue
            else:
                rebuilt.append(point)
        rebuilt = np.asarray(rebuilt, dtype=np.float32)
        if (
            len(rebuilt) != len(points) - 1
            or not cv2.isContourConvex(
                rebuilt.reshape(-1, 1, 2)
            )
        ):
            continue
        rebuilt_area = abs(float(cv2.contourArea(rebuilt)))
        area_ratio = rebuilt_area / reference_area
        if not 0.82 <= area_ratio <= 1.18:
            continue
        candidates.append(
            (
                float(short_length) / adjacent_minimum,
                endpoint_shift / adjacent_minimum,
                rebuilt,
            )
        )

    if not candidates:
        return polygon
    _, _, rebuilt = min(
        candidates,
        key=lambda item: (item[0], item[1]),
    )
    return np.round(rebuilt).astype(np.int32).reshape(-1, 1, 2)


def bridge_artwork_notch_chain(
    polygon: np.ndarray,
    reference_contour: np.ndarray,
    maximum_vertices: int = 5,
) -> np.ndarray:
    """Bridge consecutive short false edges by intersecting their side lines."""
    points = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
    point_count = len(points)
    if point_count <= maximum_vertices:
        return polygon

    reference_area = max(
        1.0,
        abs(float(cv2.contourArea(reference_contour))),
    )
    candidates = []
    for start_id in range(point_count):
        rolled = np.roll(points, -start_id, axis=0)
        # First physical side is rolled[0] -> rolled[1].  The second starts
        # at rolled[second_id]; vertices in between trace the printed notch.
        for second_id in range(3, point_count - 1):
            rebuilt_count = point_count - second_id + 1
            if not 3 <= rebuilt_count <= maximum_vertices:
                continue
            first_length = float(np.linalg.norm(rolled[1] - rolled[0]))
            second_length = float(
                np.linalg.norm(
                    rolled[second_id + 1] - rolled[second_id]
                )
            )
            adjacent_minimum = min(first_length, second_length)
            if adjacent_minimum < MIN_DETECTED_EDGE_MM * PIXELS_PER_MM:
                continue
            internal_lengths = np.linalg.norm(
                rolled[2:second_id + 1] - rolled[1:second_id],
                axis=1,
            )
            if (
                not len(internal_lengths)
                or float(np.max(internal_lengths)) > adjacent_minimum * 0.50
            ):
                continue
            try:
                crossing = line_intersection(
                    rolled[0],
                    rolled[1] - rolled[0],
                    rolled[second_id],
                    rolled[second_id + 1] - rolled[second_id],
                )
            except RuntimeError:
                continue
            endpoint_shift = max(
                float(np.linalg.norm(crossing - rolled[1])),
                float(np.linalg.norm(crossing - rolled[second_id])),
            )
            if endpoint_shift > adjacent_minimum * 0.65:
                continue
            rebuilt = np.vstack(
                [
                    rolled[0],
                    crossing,
                    rolled[second_id + 1:],
                ]
            ).astype(np.float32)
            rebuilt_contour = rebuilt.reshape(-1, 1, 2)
            if not cv2.isContourConvex(rebuilt_contour):
                continue
            area_ratio = abs(float(cv2.contourArea(rebuilt_contour))) / reference_area
            if not 0.82 <= area_ratio <= 1.18:
                continue
            fit_score = polygon_contour_fit_score(
                rebuilt_contour,
                reference_contour,
            )
            candidates.append(
                (
                    fit_score,
                    endpoint_shift / adjacent_minimum,
                    rebuilt_contour,
                )
            )

    if not candidates:
        return polygon
    _, _, rebuilt = min(candidates, key=lambda item: (item[0], item[1]))
    return np.round(rebuilt).astype(np.int32).reshape(-1, 1, 2)


def polygon_contour_fit_score(
    polygon: np.ndarray,
    reference_contour: np.ndarray,
) -> float:
    """Score straight-edge fit while penalizing area distortion."""
    vertices = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
    contour_points = np.asarray(
        reference_contour,
        dtype=np.float32,
    ).reshape(-1, 2)
    if len(vertices) < 3 or len(contour_points) < 3:
        return float("inf")

    distances = []
    for edge_id in range(len(vertices)):
        start = vertices[edge_id]
        vector = vertices[(edge_id + 1) % len(vertices)] - start
        length_squared = max(float(np.dot(vector, vector)), 1e-6)
        projection = np.clip(
            ((contour_points - start) @ vector) / length_squared,
            0.0,
            1.0,
        )
        nearest = start + projection[:, None] * vector
        distances.append(
            np.linalg.norm(contour_points - nearest, axis=1)
        )
    mean_distance = float(
        np.mean(np.min(np.stack(distances, axis=1), axis=1))
    )
    reference_area = max(
        1.0,
        abs(float(cv2.contourArea(reference_contour))),
    )
    polygon_area = abs(float(cv2.contourArea(polygon)))
    area_error = abs(polygon_area / reference_area - 1.0)
    return mean_distance + area_error * 20.0


def generate_polygon_candidates(
    contour: np.ndarray,
    config: DetectConfig,
    primary_polygon: np.ndarray,
    maximum_candidates: int = 2,
) -> List[Tuple[np.ndarray, float]]:
    """Keep the primary polygon and one strong alternate vertex model."""
    perimeter = cv2.arcLength(contour, True)
    minimum_length_px = MIN_DETECTED_EDGE_MM * PIXELS_PER_MM
    primary = np.asarray(primary_polygon, dtype=np.int32).reshape(-1, 1, 2)
    primary_score = polygon_contour_fit_score(primary, contour)
    best_by_vertex_count: Dict[int, Tuple[np.ndarray, float]] = {}

    bridged_notch = bridge_artwork_notch_chain(primary, contour)
    if len(bridged_notch) < len(primary):
        bridged_score = polygon_contour_fit_score(
            bridged_notch,
            contour,
        )
        best_by_vertex_count[len(bridged_notch)] = (
            bridged_notch.astype(np.int32),
            bridged_score,
        )

    # A short bevel can be the visible bottom of a notch whose two adjacent
    # physical sides should meet at one corner.  Preserve the measured model
    # as primary and offer the adjacent-line intersection as an alternate.
    rebuilt_corner = reconstruct_split_corner(primary, contour)
    if len(rebuilt_corner) < len(primary):
        rebuilt_score = polygon_contour_fit_score(
            rebuilt_corner,
            contour,
        )
        best_by_vertex_count[len(rebuilt_corner)] = (
            rebuilt_corner.astype(np.int32),
            rebuilt_score,
        )

    for epsilon_percent in np.arange(0.6, 3.01, 0.2):
        candidate = cv2.approxPolyDP(
            contour,
            perimeter * float(epsilon_percent) / 100.0,
            True,
        )
        candidate = remove_false_short_edges(
            candidate,
            contour,
            minimum_length_px,
        )
        candidate = remove_nearly_collinear_vertices(candidate)
        vertex_count = len(candidate)
        if not 3 <= vertex_count <= 6:
            continue
        if not cv2.isContourConvex(candidate):
            continue
        lengths = np.linalg.norm(
            np.roll(
                candidate.reshape(-1, 2),
                -1,
                axis=0,
            )
            - candidate.reshape(-1, 2),
            axis=1,
        )
        if float(np.min(lengths)) < minimum_length_px:
            continue
        score = polygon_contour_fit_score(candidate, contour)
        previous = best_by_vertex_count.get(vertex_count)
        if previous is None or score < previous[1]:
            best_by_vertex_count[vertex_count] = (
                candidate.astype(np.int32),
                score,
            )

    # Printed artwork that reaches a cut edge can carve narrow notches out
    # of the white-region contour.  When that creates more vertices than the
    # task permits, recover an additional straight-edged model from the
    # contour hull.  This is deliberately only an alternate for an already
    # over-complex primary contour; genuine <=5-edge concave pieces retain
    # their measured geometry unchanged.
    if len(primary) > 5:
        reference_area = max(
            1.0,
            abs(float(cv2.contourArea(contour))),
        )
        hull = cv2.convexHull(contour)
        hull_area_ratio = abs(float(cv2.contourArea(hull))) / reference_area
        if hull_area_ratio <= 1.18:
            hull_perimeter = cv2.arcLength(hull, True)
            for epsilon_percent in np.arange(1.0, 3.01, 0.25):
                candidate = cv2.approxPolyDP(
                    hull,
                    hull_perimeter * float(epsilon_percent) / 100.0,
                    True,
                )
                candidate = remove_nearly_collinear_vertices(candidate)
                if not 3 <= len(candidate) <= 5:
                    continue
                candidate = sharpen_polygon_from_contour(
                    candidate,
                    contour,
                )
                candidate = remove_false_short_edges(
                    candidate,
                    contour,
                    minimum_length_px,
                )
                candidate = remove_nearly_collinear_vertices(candidate)
                vertex_count = len(candidate)
                if (
                    not 3 <= vertex_count <= 5
                    or not cv2.isContourConvex(candidate)
                ):
                    continue
                polygon_area_ratio = abs(
                    float(cv2.contourArea(candidate))
                ) / reference_area
                if not 0.82 <= polygon_area_ratio <= 1.18:
                    continue
                lengths = np.linalg.norm(
                    np.roll(candidate.reshape(-1, 2), -1, axis=0)
                    - candidate.reshape(-1, 2),
                    axis=1,
                )
                if float(np.min(lengths)) < minimum_length_px:
                    continue
                score = polygon_contour_fit_score(candidate, contour)
                previous = best_by_vertex_count.get(vertex_count)
                if previous is None or score < previous[1]:
                    best_by_vertex_count[vertex_count] = (
                        candidate.astype(np.int32),
                        score,
                    )

    candidates = [(primary, primary_score)]
    alternatives = sorted(
        (
            value
            for vertex_count, value in best_by_vertex_count.items()
            if vertex_count != len(primary)
        ),
        key=lambda item: item[1],
    )
    candidates.extend(
        alternatives[: max(0, maximum_candidates - 1)]
    )
    return candidates


def sharpen_polygon_from_contour(
    polygon: np.ndarray,
    reference_contour: np.ndarray,
    maximum_corner_shift_px: float = 16.0,
) -> np.ndarray:
    """Fit each contour side separately and intersect lines for sharp corners."""
    vertices = np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
    contour_points = np.asarray(
        reference_contour,
        dtype=np.float32,
    ).reshape(-1, 2)
    edge_count = len(vertices)
    if edge_count < 3 or len(contour_points) < edge_count * 4:
        return polygon

    # Assign every raw contour point to its nearest initial polygon side.
    distances = []
    for edge_id in range(edge_count):
        start = vertices[edge_id]
        vector = vertices[(edge_id + 1) % edge_count] - start
        length_squared = max(float(np.dot(vector, vector)), 1e-6)
        projection = np.clip(
            ((contour_points - start) @ vector) / length_squared,
            0.0,
            1.0,
        )
        nearest = start + projection[:, None] * vector
        distances.append(
            np.sum((contour_points - nearest) ** 2, axis=1)
        )
    assignments = np.argmin(np.stack(distances, axis=1), axis=1)

    fitted_lines = []
    for edge_id in range(edge_count):
        start = vertices[edge_id]
        vector = vertices[(edge_id + 1) % edge_count] - start
        length = float(np.linalg.norm(vector))
        if length < 2.0:
            return polygon
        direction = vector / length
        side_points = contour_points[assignments == edge_id]
        if len(side_points) < 4:
            return polygon

        # Corner pixels are rounded and less reliable; fit the central side.
        along = (side_points - start) @ direction
        central = side_points[
            (along >= length * 0.10) & (along <= length * 0.90)
        ]
        if len(central) >= 4:
            side_points = central
        vx, vy, x0, y0 = cv2.fitLine(
            side_points.reshape(-1, 1, 2),
            cv2.DIST_L2,
            0,
            0.01,
            0.01,
        ).reshape(-1)
        fitted_direction = np.asarray([vx, vy], dtype=np.float32)
        if float(np.dot(fitted_direction, direction)) < 0:
            fitted_direction *= -1.0
        fitted_lines.append(
            (
                np.asarray([x0, y0], dtype=np.float32),
                fitted_direction,
            )
        )

    sharp_vertices = []
    for vertex_id in range(edge_count):
        previous_point, previous_direction = fitted_lines[
            (vertex_id - 1) % edge_count
        ]
        current_point, current_direction = fitted_lines[vertex_id]
        denominator = (
            previous_direction[0] * current_direction[1]
            - previous_direction[1] * current_direction[0]
        )
        if abs(float(denominator)) < 0.08:
            return polygon
        delta = current_point - previous_point
        distance = (
            delta[0] * current_direction[1]
            - delta[1] * current_direction[0]
        ) / denominator
        corner = previous_point + distance * previous_direction
        if (
            not np.all(np.isfinite(corner))
            or float(np.linalg.norm(corner - vertices[vertex_id]))
            > maximum_corner_shift_px
        ):
            return polygon
        sharp_vertices.append(corner)

    sharp = np.asarray(sharp_vertices, dtype=np.float32).reshape(-1, 1, 2)
    original_area = abs(float(cv2.contourArea(vertices)))
    sharp_area = abs(float(cv2.contourArea(sharp)))
    if (
        original_area < 1.0
        or not 0.85 <= sharp_area / original_area <= 1.15
        or not cv2.isContourConvex(sharp)
    ):
        return polygon
    return np.round(sharp).astype(np.int32)


def choose_pickup_point(
    contour: np.ndarray,
    centroid: Tuple[float, float],
    safe_radius_px: int,
) -> Tuple[Point, float, str]:
    """优先吸取面积重心；重心不安全时选择最近的安全位置。"""
    x, y, width, height = cv2.boundingRect(contour)
    padding = max(2, safe_radius_px + 2)
    x0 = x - padding
    y0 = y - padding

    local_contour = contour.copy()
    local_contour[:, 0, 0] -= x0
    local_contour[:, 0, 1] -= y0
    local_mask = np.zeros(
        (height + 2 * padding, width + 2 * padding),
        dtype=np.uint8,
    )
    cv2.drawContours(
        local_mask,
        [local_contour],
        -1,
        255,
        thickness=cv2.FILLED,
    )
    distance = cv2.distanceTransform(local_mask, cv2.DIST_L2, 5)

    local_cx = int(round(centroid[0] - x0))
    local_cy = int(round(centroid[1] - y0))
    centroid_clearance = float(distance[local_cy, local_cx])
    if centroid_clearance >= safe_radius_px:
        return (
            (int(round(centroid[0])), int(round(centroid[1]))),
            centroid_clearance,
            "centroid",
        )

    safe_y, safe_x = np.where(distance >= safe_radius_px)
    if len(safe_x) > 0:
        squared_distance = (
            (safe_x - (centroid[0] - x0)) ** 2
            + (safe_y - (centroid[1] - y0)) ** 2
        )
        best = int(np.argmin(squared_distance))
        px = int(safe_x[best])
        py = int(safe_y[best])
        return (
            (px + x0, py + y0),
            float(distance[py, px]),
            "nearest_safe",
        )

    _, maximum, _, location = cv2.minMaxLoc(distance)
    return (
        (location[0] + x0, location[1] + y0),
        float(maximum),
        "max_clearance",
    )


def split_touching_contour(
    contour: np.ndarray,
    config: DetectConfig,
    maximum_vertices: int = 5,
) -> List[np.ndarray]:
    """Split two convex fragments joined through one narrow contact.

    Erosion is used only to discover stable interior seeds. Every pixel from
    the original contour is then assigned to its nearest seed, so the output
    fragments retain the full measured area instead of remaining eroded.
    """
    original_area = abs(float(cv2.contourArea(contour)))
    if original_area < 1.0:
        return []

    x, y, width, height = cv2.boundingRect(contour)
    maximum_erosion_px = int(round(6.0 * PIXELS_PER_MM))
    padding = maximum_erosion_px + 3
    local_mask = np.zeros(
        (height + padding * 2, width + padding * 2),
        dtype=np.uint8,
    )
    local_contour = contour.copy()
    local_contour[:, 0, 0] -= x - padding
    local_contour[:, 0, 1] -= y - padding
    cv2.drawContours(
        local_mask,
        [local_contour],
        -1,
        255,
        thickness=cv2.FILLED,
    )

    seed_labels = None
    seed_components = None
    kernel = np.ones((3, 3), dtype=np.uint8)
    minimum_seed_area = original_area * 0.08
    for erosion_px in range(2, maximum_erosion_px + 1):
        eroded = cv2.erode(
            local_mask,
            kernel,
            iterations=erosion_px,
        )
        count, labels, stats, _ = cv2.connectedComponentsWithStats(
            eroded
        )
        components = [
            component
            for component in range(1, count)
            if float(
                stats[component, cv2.CC_STAT_AREA]
            ) >= minimum_seed_area
        ]
        if len(components) == 2:
            seed_labels = labels
            seed_components = components
            break
        if len(components) > 2:
            return []

    if seed_labels is None or seed_components is None:
        return []

    seed_map = np.full(local_mask.shape, 255, dtype=np.uint8)
    for component in seed_components:
        seed_map[seed_labels == component] = 0
    _, nearest_labels = cv2.distanceTransformWithLabels(
        seed_map,
        cv2.DIST_L2,
        5,
        labelType=cv2.DIST_LABEL_CCOMP,
    )
    region_label_values = []
    for component in seed_components:
        values = nearest_labels[seed_labels == component]
        if not values.size:
            return []
        region_label_values.append(int(values[0]))

    parts = []
    for label_value in region_label_values:
        region = np.where(
            (nearest_labels == label_value) & (local_mask > 0),
            255,
            0,
        ).astype(np.uint8)
        region_contours, _ = cv2.findContours(
            region,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_NONE,
        )
        if not region_contours:
            return []
        part = max(region_contours, key=cv2.contourArea)
        part_area = abs(float(cv2.contourArea(part)))
        if part_area < original_area * 0.15:
            return []

        perimeter = cv2.arcLength(part, True)
        polygon = cv2.approxPolyDP(
            part,
            perimeter * config.polygon_epsilon_percent / 100.0,
            True,
        )
        polygon = remove_false_short_edges(
            polygon,
            part,
            MIN_DETECTED_EDGE_MM * PIXELS_PER_MM,
        )
        polygon = sharpen_polygon_from_contour(polygon, part)
        polygon = remove_false_short_edges(
            polygon,
            part,
            MIN_DETECTED_EDGE_MM * PIXELS_PER_MM,
        )
        polygon = remove_nearly_collinear_vertices(polygon)
        if (
            not 3 <= len(polygon) <= maximum_vertices
            or not cv2.isContourConvex(polygon)
        ):
            return []

        part = part.copy()
        part[:, 0, 0] += x - padding
        part[:, 0, 1] += y - padding
        parts.append(part)

    if len(parts) != 2:
        return []
    return parts


def detect_pieces(
    frame: np.ndarray,
    config: DetectConfig,
) -> Tuple[np.ndarray, List[Dict[str, object]], int]:
    mask, background_hue = build_piece_mask(frame, config)
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    frame_height, frame_width = frame.shape[:2]
    frame_area = frame_height * frame_width
    min_area = frame_area * config.min_area_ratio
    max_area = frame_area * config.max_area_ratio

    expanded_contours = []
    split_contour_ids = set()
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not min_area <= area <= max_area:
            expanded_contours.append(contour)
            continue
        perimeter = cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(
            contour,
            perimeter * config.polygon_epsilon_percent / 100.0,
            True,
        )
        polygon = remove_false_short_edges(
            polygon,
            contour,
            MIN_DETECTED_EDGE_MM * PIXELS_PER_MM,
        )
        polygon = remove_nearly_collinear_vertices(polygon)
        if len(polygon) > 5:
            split_parts = split_touching_contour(
                contour,
                config,
                maximum_vertices=5,
            )
            if split_parts:
                expanded_contours.extend(split_parts)
                split_contour_ids.update(
                    id(part) for part in split_parts
                )
                continue
        expanded_contours.append(contour)
    contours = expanded_contours

    pieces = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if not min_area <= area <= max_area:
            continue

        x, y, width, height = cv2.boundingRect(contour)
        boundary_margin = max(
            2,
            int(round(config.paper_erode_size / 3.0)),
        )
        boundary_artifact = (
            height > frame_height * 0.75
            or width > frame_width * 0.85
            or width / float(height) < 0.18
            or x <= boundary_margin
            or y <= boundary_margin
            or x + width >= frame_width - boundary_margin
            or y + height >= frame_height - boundary_margin
        )
        if boundary_artifact:
            continue

        perimeter = cv2.arcLength(contour, True)
        epsilon = perimeter * config.polygon_epsilon_percent / 100.0
        polygon = cv2.approxPolyDP(contour, epsilon, True)
        polygon = remove_false_short_edges(
            polygon,
            contour,
            MIN_DETECTED_EDGE_MM * PIXELS_PER_MM,
        )
        polygon = sharpen_polygon_from_contour(
            polygon,
            contour,
        )
        # Corner sharpening can create a tiny extra segment near an
        # otherwise valid tip. Remove those post-sharpen artifacts too.
        polygon = remove_false_short_edges(
            polygon,
            contour,
            MIN_DETECTED_EDGE_MM * PIXELS_PER_MM,
        )
        polygon = remove_nearly_collinear_vertices(polygon)
        if config.include_dark_artwork_in_piece_mask:
            # Mode 3 artwork can touch a cut edge and leave a shallow split
            # point even after the dark pixels are restored to the piece.
            # Join the two measured side segments only when that point is
            # demonstrably close to their common straight line.
            polygon = collapse_artwork_split_edge_vertices(polygon)
        if id(contour) in split_contour_ids:
            polygon = reconstruct_split_corner(
                polygon,
                contour,
            )
        polygon_candidates = generate_polygon_candidates(
            contour,
            config,
            polygon,
        )
        centroid = contour_centroid(contour)
        pickup, clearance, pickup_mode = choose_pickup_point(
            contour,
            centroid,
            config.safe_radius_px,
        )
        pieces.append(
            {
                "_contour": contour,
                "_polygon": polygon,
                "_polygon_candidates": polygon_candidates,
                "area_px2": area,
                "centroid_px": centroid,
                "pickup_px": pickup,
                "clearance_px": clearance,
                "pickup_mode": pickup_mode,
            }
        )

    pieces.sort(
        key=lambda piece: (
            piece["centroid_px"][1],
            piece["centroid_px"][0],
        )
    )

    filled_mask = np.zeros(mask.shape, dtype=np.uint8)
    results = []
    for piece_id, piece in enumerate(pieces):
        cv2.drawContours(
            filled_mask,
            [piece["_contour"]],
            -1,
            255,
            thickness=cv2.FILLED,
        )
        results.append(
            {
                "id": piece_id,
                "area_px2": round(piece["area_px2"], 1),
                "centroid_px": [
                    round(piece["centroid_px"][0], 1),
                    round(piece["centroid_px"][1], 1),
                ],
                "pickup_px": list(piece["pickup_px"]),
                "clearance_px": round(piece["clearance_px"], 1),
                "pickup_mode": piece["pickup_mode"],
                "vertices_px": piece["_polygon"].reshape(-1, 2).tolist(),
                "polygon_candidates_px": [
                    candidate.reshape(-1, 2).tolist()
                    for candidate, _ in piece[
                        "_polygon_candidates"
                    ]
                ],
                "polygon_candidate_scores": [
                    round(float(score), 4)
                    for _, score in piece["_polygon_candidates"]
                ],
                "_contour": piece["_contour"],
                "_polygon": piece["_polygon"],
            }
        )
    return filled_mask, results, background_hue


def draw_results(
    frame: np.ndarray,
    pieces: List[Dict[str, object]],
    background_hue: int,
) -> np.ndarray:
    output = frame.copy()
    paper_width_mm = frame.shape[1] / PIXELS_PER_MM
    paper_height_mm = frame.shape[0] / PIXELS_PER_MM
    cv2.putText(
        output,
        "A4 {:.0f}x{:.0f} X=short Y=long O=TL hue={} P={}".format(
            paper_width_mm,
            paper_height_mm,
            background_hue,
            len(pieces),
        ),
        (16, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.58,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )

    for piece in pieces:
        pickup = tuple(piece["pickup_px"])
        cv2.drawContours(output, [piece["_contour"]], -1, (0, 200, 0), 2)
        cv2.drawContours(output, [piece["_polygon"]], -1, (255, 180, 0), 2)
        cv2.circle(output, pickup, 4, (0, 0, 255), -1)
        cv2.circle(
            output,
            pickup,
            max(1, int(round(piece["clearance_px"]))),
            (0, 0, 255),
            1,
        )
        pickup_mm = (
            piece["pickup_px"][0] / PIXELS_PER_MM,
            piece["pickup_px"][1] / PIXELS_PER_MM,
        )
        label = "P{} S({:.1f},{:.1f})mm".format(
            piece["id"],
            pickup_mm[0],
            pickup_mm[1],
        )
        font_scale = 0.42
        thickness = 1
        label_width = cv2.getTextSize(
            label,
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            thickness,
        )[0][0]
        label_x = min(
            max(4, pickup[0] + 8),
            max(4, output.shape[1] - label_width - 5),
        )
        label_y = min(
            max(18, pickup[1] - 8),
            output.shape[0] - 5,
        )
        position = (label_x, label_y)
        cv2.putText(
            output,
            label,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (20, 20, 20),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            output,
            label,
            position,
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )
    return output


def serializable_pieces(
    pieces: List[Dict[str, object]],
) -> List[Dict[str, object]]:
    return [
        {key: value for key, value in piece.items() if not key.startswith("_")}
        for piece in pieces
    ]


def create_controls(config: DetectConfig) -> None:
    cv2.namedWindow("controls", cv2.WINDOW_NORMAL)
    noop = lambda _value: None
    cv2.createTrackbar(
        "hue tolerance",
        "controls",
        config.hue_tolerance,
        40,
        noop,
    )
    cv2.createTrackbar(
        "saturation min",
        "controls",
        config.saturation_min,
        255,
        noop,
    )
    cv2.createTrackbar(
        "value min",
        "controls",
        config.value_min,
        255,
        noop,
    )
    cv2.createTrackbar("open", "controls", config.open_size, 31, noop)
    cv2.createTrackbar("close", "controls", config.close_size, 51, noop)
    cv2.createTrackbar(
        "polygon eps x10",
        "controls",
        int(round(config.polygon_epsilon_percent * 10)),
        80,
        noop,
    )
    cv2.createTrackbar(
        "safe radius",
        "controls",
        config.safe_radius_px,
        100,
        noop,
    )
    cv2.createTrackbar(
        "min area permille",
        "controls",
        int(round(config.min_area_ratio * 1000)),
        50,
        noop,
    )
    cv2.createTrackbar(
        "max area percent",
        "controls",
        int(round(config.max_area_ratio * 100)),
        50,
        noop,
    )


def read_controls() -> DetectConfig:
    return DetectConfig(
        hue_tolerance=max(
            1,
            cv2.getTrackbarPos("hue tolerance", "controls"),
        ),
        saturation_min=cv2.getTrackbarPos("saturation min", "controls"),
        value_min=cv2.getTrackbarPos("value min", "controls"),
        open_size=max(1, cv2.getTrackbarPos("open", "controls")),
        close_size=max(1, cv2.getTrackbarPos("close", "controls")),
        polygon_epsilon_percent=max(
            0.1,
            cv2.getTrackbarPos("polygon eps x10", "controls") / 10.0,
        ),
        safe_radius_px=max(
            1,
            cv2.getTrackbarPos("safe radius", "controls"),
        ),
        min_area_ratio=max(
            0.001,
            cv2.getTrackbarPos("min area permille", "controls") / 1000.0,
        ),
        max_area_ratio=max(
            0.01,
            cv2.getTrackbarPos("max area percent", "controls") / 100.0,
        ),
    )


def make_payload(
    frame: np.ndarray,
    pieces: List[Dict[str, object]],
    background_hue: int,
    calibration: Dict[str, object],
) -> Dict[str, object]:
    output_pieces = serializable_pieces(pieces)
    for piece in output_pieces:
        piece["centroid_mm"] = [
            round(value / PIXELS_PER_MM, 2)
            for value in piece["centroid_px"]
        ]
        piece["pickup_mm"] = [
            round(value / PIXELS_PER_MM, 2)
            for value in piece["pickup_px"]
        ]
        piece["clearance_mm"] = round(
            piece["clearance_px"] / PIXELS_PER_MM,
            2,
        )
        piece["vertices_mm"] = [
            [
                round(vertex[0] / PIXELS_PER_MM, 2),
                round(vertex[1] / PIXELS_PER_MM, 2),
            ]
            for vertex in piece["vertices_px"]
        ]
        piece["polygon_candidates_mm"] = [
            [
                [
                    round(vertex[0] / PIXELS_PER_MM, 2),
                    round(vertex[1] / PIXELS_PER_MM, 2),
                ]
                for vertex in candidate
            ]
            for candidate in piece.get(
                "polygon_candidates_px",
                [piece["vertices_px"]],
            )
        ]
        vertices_mm = np.asarray(piece["vertices_mm"], dtype=np.float32)
        piece["edge_lengths_mm"] = np.round(
            np.linalg.norm(
                np.roll(vertices_mm, -1, axis=0) - vertices_mm,
                axis=1,
            ),
            2,
        ).tolist()
    return {
        "rectified_size_px": [frame.shape[1], frame.shape[0]],
        "background_hue": background_hue,
        "calibration": calibration,
        "pieces": output_pieces,
    }


def process_frame(
    frame: np.ndarray,
    config: DetectConfig,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
    rectified, calibration = rectify_a4(frame, config)
    mask, pieces, background_hue = detect_pieces(rectified, config)
    annotated = draw_results(rectified, pieces, background_hue)
    payload = make_payload(
        rectified,
        pieces,
        background_hue,
        calibration,
    )
    return mask, annotated, payload


def resize_for_debug(
    frame: np.ndarray,
    maximum_side: int = 1500,
) -> np.ndarray:
    if max(frame.shape[:2]) <= maximum_side:
        return frame
    scale = maximum_side / float(max(frame.shape[:2]))
    return cv2.resize(
        frame,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_AREA,
    )


def run_image(
    image_path: str,
    no_gui: bool,
    output_path: str,
) -> int:
    total_start = time.perf_counter()
    load_start = time.perf_counter()
    frame = cv2.imread(image_path)
    if frame is None:
        raise RuntimeError("无法读取图片：{}".format(image_path))
    frame = resize_for_debug(frame)
    load_ms = (time.perf_counter() - load_start) * 1000.0
    config = DetectConfig()

    if no_gui:
        vision_start = time.perf_counter()
        mask, annotated, payload = process_frame(frame, config)
        vision_ms = (time.perf_counter() - vision_start) * 1000.0
        if output_path:
            cv2.imwrite(output_path, annotated)
        total_ms = (time.perf_counter() - total_start) * 1000.0
        payload["timing_ms"] = {
            "image_load": round(load_ms, 2),
            "vision": round(vision_ms, 2),
            "total": round(total_ms, 2),
        }
        print(
            "耗时：读取 {:.2f} ms，视觉 {:.2f} ms，总计 {:.2f} ms".format(
                load_ms,
                vision_ms,
                total_ms,
            ),
            file=sys.stderr,
        )
        print(json.dumps(payload, ensure_ascii=False, indent=2))
        return 0

    create_controls(config)
    while True:
        vision_start = time.perf_counter()
        mask, annotated, payload = process_frame(frame, read_controls())
        vision_ms = (time.perf_counter() - vision_start) * 1000.0
        payload["timing_ms"] = {"vision": round(vision_ms, 2)}
        cv2.putText(
            annotated,
            "vision {:.1f} ms".format(vision_ms),
            (16, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("frame", annotated)
        cv2.imshow("filled mask", mask)
        key = cv2.waitKey(20) & 0xFF
        if key in (27, ord("q")):
            break
        if key == ord("s"):
            save_path = output_path or "fragment_debug_result.jpg"
            cv2.imwrite(save_path, annotated)
            cv2.imwrite("fragment_debug_mask.png", mask)
            print("已保存：{}".format(save_path))
            print(json.dumps(payload, ensure_ascii=False, indent=2))
    cv2.destroyAllWindows()
    return 0


def run_camera(camera_index: int, width: int, height: int) -> int:
    capture = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not capture.isOpened():
        raise RuntimeError("无法打开摄像头索引 {}".format(camera_index))

    create_controls(DetectConfig())
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError("摄像头读取画面失败")
            vision_start = time.perf_counter()
            mask, annotated, payload = process_frame(frame, read_controls())
            vision_ms = (time.perf_counter() - vision_start) * 1000.0
            payload["timing_ms"] = {"vision": round(vision_ms, 2)}
            cv2.putText(
                annotated,
                "vision {:.1f} ms".format(vision_ms),
                (16, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("frame", annotated)
            cv2.imshow("filled mask", mask)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key == ord("s"):
                cv2.imwrite("fragment_debug_result.jpg", annotated)
                cv2.imwrite("fragment_debug_mask.png", mask)
                print(json.dumps(payload, ensure_ascii=False, indent=2))
    finally:
        capture.release()
        cv2.destroyAllWindows()
    return 0


def parse_source(value: str) -> Union[int, str]:
    return int(value) if value.isdigit() else value


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        default="0",
        help="摄像头索引或图片路径，默认 0",
    )
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="图片模式下只输出 JSON，不打开窗口",
    )
    parser.add_argument(
        "--output",
        default="",
        help="保存标注图的路径",
    )
    return parser


def main() -> int:
    args = build_argument_parser().parse_args()
    source = parse_source(args.source)
    if isinstance(source, int):
        if args.no_gui:
            raise ValueError("摄像头模式不能使用 --no-gui")
        return run_camera(source, args.width, args.height)
    return run_image(source, args.no_gui, args.output)


if __name__ == "__main__":
    raise SystemExit(main())
