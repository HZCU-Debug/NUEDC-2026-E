"""最多四块多边形碎片的几何拼接求解器。"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations, permutations
from math import atan2, cos, degrees, radians, sin, sqrt
import os
import time
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    if os.environ.get("VISION_DISABLE_NATIVE", "").strip() == "1":
        raise ImportError("native acceleration disabled")
    import vision_fast as _vision_fast
except ImportError:
    _vision_fast = None


def native_acceleration_available() -> bool:
    return _vision_fast is not None


@dataclass
class SearchState:
    transforms: Dict[int, Tuple[np.ndarray, np.ndarray]]
    used_edges: frozenset
    match_error: float
    matches: Tuple[Tuple[int, int, int, int, float], ...]


def polygon_area(polygon: np.ndarray) -> float:
    array = np.asarray(polygon, dtype=np.float32)
    if not array.flags.c_contiguous:
        array = np.ascontiguousarray(array)
    return abs(float(cv2.contourArea(array)))


def apply_transform(
    polygon: np.ndarray,
    transform: Tuple[np.ndarray, np.ndarray],
) -> np.ndarray:
    rotation, translation = transform
    return polygon @ rotation.T + translation


def edge_lengths(polygon: np.ndarray) -> np.ndarray:
    return np.linalg.norm(np.roll(polygon, -1, axis=0) - polygon, axis=1)


def edge_alignment_transforms(
    moving_polygon: np.ndarray,
    moving_edge: int,
    fixed_polygon_world: np.ndarray,
    fixed_edge: int,
) -> List[Tuple[np.ndarray, np.ndarray]]:
    """生成反向共线的中点对齐和两种端点对齐。"""
    moving_start = moving_polygon[moving_edge]
    moving_end = moving_polygon[(moving_edge + 1) % len(moving_polygon)]
    fixed_start = fixed_polygon_world[fixed_edge]
    fixed_end = fixed_polygon_world[(fixed_edge + 1) % len(fixed_polygon_world)]

    moving_vector = moving_end - moving_start
    target_vector = fixed_start - fixed_end
    angle = atan2(target_vector[1], target_vector[0]) - atan2(
        moving_vector[1],
        moving_vector[0],
    )
    rotation = np.asarray(
        [[cos(angle), -sin(angle)], [sin(angle), cos(angle)]],
        dtype=np.float32,
    )
    moving_midpoint = (moving_start + moving_end) * 0.5
    fixed_midpoint = (fixed_start + fixed_end) * 0.5
    translations = [
        fixed_midpoint - rotation @ moving_midpoint,
        fixed_start - rotation @ moving_end,
        fixed_end - rotation @ moving_start,
    ]
    unique = []
    for translation in translations:
        if not any(
            float(np.dot(translation - old[1], translation - old[1]))
            < 0.25
            for old in unique
        ):
            unique.append((rotation, translation))
    return unique


def partial_edge_alignment_transforms(
    moving_polygon: np.ndarray,
    moving_edge: int,
    fixed_polygon_world: np.ndarray,
    fixed_edge: int,
    offsets_on_long_edge: Sequence[float],
) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Align a short edge to discrete partition offsets on a longer edge."""
    moving_start = moving_polygon[moving_edge]
    moving_end = moving_polygon[(moving_edge + 1) % len(moving_polygon)]
    fixed_start = fixed_polygon_world[fixed_edge]
    fixed_end = fixed_polygon_world[(fixed_edge + 1) % len(fixed_polygon_world)]
    moving_vector = moving_end - moving_start
    fixed_vector = fixed_start - fixed_end
    moving_length = float(
        (moving_vector[0] ** 2 + moving_vector[1] ** 2) ** 0.5
    )
    fixed_length = float(
        (fixed_vector[0] ** 2 + fixed_vector[1] ** 2) ** 0.5
    )
    if moving_length < 1e-6 or fixed_length < 1e-6:
        return []

    angle = atan2(fixed_vector[1], fixed_vector[0]) - atan2(
        moving_vector[1],
        moving_vector[0],
    )
    rotation = np.asarray(
        [[cos(angle), -sin(angle)], [sin(angle), cos(angle)]],
        dtype=np.float32,
    )
    transforms = []
    if fixed_length >= moving_length:
        fixed_direction = fixed_vector / fixed_length
        for offset in offsets_on_long_edge:
            target_start = fixed_end + fixed_direction * float(offset)
            translation = target_start - rotation @ moving_start
            transforms.append((rotation, translation))
    else:
        moving_direction = moving_vector / moving_length
        for offset in offsets_on_long_edge:
            moving_sub_start = (
                moving_start + moving_direction * float(offset)
            )
            translation = fixed_end - rotation @ moving_sub_start
            transforms.append((rotation, translation))

    unique = []
    for transform in transforms:
        if not any(
            float(
                np.dot(
                    transform[1] - old[1],
                    transform[1] - old[1],
                )
            )
            < 0.25
            for old in unique
        ):
            unique.append(transform)
    return unique


def convex_overlap_area(a: np.ndarray, b: np.ndarray) -> float:
    """当前题目自备碎片为凸多边形；返回两凸多边形相交面积。"""
    try:
        a = np.asarray(a, dtype=np.float32)
        b = np.asarray(b, dtype=np.float32)
        if not a.flags.c_contiguous:
            a = np.ascontiguousarray(a)
        if not b.flags.c_contiguous:
            b = np.ascontiguousarray(b)
        area, _ = cv2.intersectConvexConvex(
            a,
            b,
        )
        return float(area)
    except cv2.error:
        return float("inf")


def batch_convex_overlap_areas(
    first_polygons: Sequence[np.ndarray],
    second_polygons: Sequence[np.ndarray],
    workers: int = 1,
) -> np.ndarray:
    """Compute pairwise convex overlap, using native OpenMP when available."""
    if len(first_polygons) != len(second_polygons):
        raise ValueError("polygon pair batches must have equal length")
    pair_count = len(first_polygons)
    if pair_count == 0:
        return np.empty(0, dtype=np.float64)
    if _vision_fast is None or pair_count < 8:
        return np.asarray(
            [
                convex_overlap_area(first, second)
                for first, second in zip(first_polygons, second_polygons)
            ],
            dtype=np.float64,
        )

    first_arrays = [
        np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
        for polygon in first_polygons
    ]
    second_arrays = [
        np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
        for polygon in second_polygons
    ]
    first_counts = np.asarray(
        [len(polygon) for polygon in first_arrays],
        dtype=np.int32,
    )
    second_counts = np.asarray(
        [len(polygon) for polygon in second_arrays],
        dtype=np.int32,
    )
    first_batch = np.zeros(
        (pair_count, int(np.max(first_counts)), 2),
        dtype=np.float32,
    )
    second_batch = np.zeros(
        (pair_count, int(np.max(second_counts)), 2),
        dtype=np.float32,
    )
    for index, (first, second) in enumerate(
        zip(first_arrays, second_arrays)
    ):
        first_batch[index, : len(first)] = first
        second_batch[index, : len(second)] = second
    return np.asarray(
        _vision_fast.batch_convex_overlap_areas(
            first_batch,
            first_counts,
            second_batch,
            second_counts,
            workers=max(1, int(workers)),
        ),
        dtype=np.float64,
    )


def batch_edge_alignment_world(
    polygons: Sequence[np.ndarray],
    jobs: Sequence[Tuple[int, int, np.ndarray, int, int, float]],
    workers: int = 1,
) -> List[Tuple[Tuple[np.ndarray, np.ndarray], np.ndarray]]:
    """Build edge transforms and transformed polygons in one native batch.

    ``kind`` is 0/1/2 for midpoint/two endpoint full-edge alignments and 3
    for a partition offset.  The Python branch is deliberately kept as the
    reference implementation and as the fallback on non-native platforms.
    """
    if not jobs:
        return []
    polygon_arrays = [
        np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
        for polygon in polygons
    ]
    if _vision_fast is None or len(jobs) < 8:
        results = []
        for (
            moving_id,
            moving_edge,
            fixed_polygon,
            fixed_edge,
            kind,
            offset,
        ) in jobs:
            if kind == 3:
                transforms = partial_edge_alignment_transforms(
                    polygon_arrays[moving_id],
                    moving_edge,
                    fixed_polygon,
                    fixed_edge,
                    [offset],
                )
                if not transforms:
                    raise ValueError("invalid partial edge alignment job")
                transform = transforms[0]
            else:
                transforms = edge_alignment_transforms(
                    polygon_arrays[moving_id],
                    moving_edge,
                    fixed_polygon,
                    fixed_edge,
                )
                if kind >= len(transforms):
                    raise ValueError("invalid full edge alignment variant")
                transform = transforms[kind]
            results.append(
                (
                    transform,
                    apply_transform(polygon_arrays[moving_id], transform),
                )
            )
        return results

    polygon_counts = np.asarray(
        [len(polygon) for polygon in polygon_arrays],
        dtype=np.int32,
    )
    polygon_batch = np.zeros(
        (len(polygon_arrays), int(np.max(polygon_counts)), 2),
        dtype=np.float32,
    )
    for piece_id, polygon in enumerate(polygon_arrays):
        polygon_batch[piece_id, : len(polygon)] = polygon
    fixed_arrays = [
        np.asarray(job[2], dtype=np.float32).reshape(-1, 2)
        for job in jobs
    ]
    fixed_counts = np.asarray(
        [len(polygon) for polygon in fixed_arrays],
        dtype=np.int32,
    )
    fixed_batch = np.zeros(
        (len(jobs), int(np.max(fixed_counts)), 2),
        dtype=np.float32,
    )
    for job_id, polygon in enumerate(fixed_arrays):
        fixed_batch[job_id, : len(polygon)] = polygon
    rotations, translations, worlds = _vision_fast.batch_edge_alignment_world(
        polygon_batch,
        polygon_counts,
        np.asarray([job[0] for job in jobs], dtype=np.int32),
        np.asarray([job[1] for job in jobs], dtype=np.int32),
        fixed_batch,
        fixed_counts,
        np.asarray([job[3] for job in jobs], dtype=np.int32),
        np.asarray([job[4] for job in jobs], dtype=np.int32),
        np.asarray([job[5] for job in jobs], dtype=np.float64),
        workers=max(1, int(workers)),
    )
    rotations = np.asarray(rotations, dtype=np.float32)
    translations = np.asarray(translations, dtype=np.float32)
    worlds = np.asarray(worlds, dtype=np.float32)
    return [
        (
            (rotations[job_id], translations[job_id]),
            worlds[job_id, : polygon_counts[job[0]]],
        )
        for job_id, job in enumerate(jobs)
    ]


def full_alignment_kinds(
    moving_length: float,
    fixed_length: float,
) -> Tuple[int, ...]:
    """Match the 0.5 mm translation de-duplication in the Python reference."""
    if abs(float(moving_length) - float(fixed_length)) < 1.0:
        return (0,)
    return (0, 1, 2)


def corner_alignment_error_deg(
    fixed_angles: np.ndarray,
    fixed_edge: int,
    moving_angles: np.ndarray,
    moving_edge: int,
    kind: int,
    partition_offset: float = 0.0,
    fixed_length: float = 0.0,
    moving_length: float = 0.0,
    endpoint_tolerance_mm: float = 0.75,
) -> Optional[float]:
    """Return the two-ray compatibility error for an edge placement.

    A seam endpoint is useful only when the two piece angles make either a
    straight outer side (sum 180 degrees) or a rectangular outer corner (sum
    90 degrees).  Unlike a length-only edge proposal, this lets one vertex
    constrain both the seam direction and its neighbouring boundary ray.

    ``kind`` follows :func:`batch_edge_alignment_world`.  Midpoint alignment
    is accepted only for almost equal full edges, where both endpoint pairs
    coincide.  A partition proposal is angle-driven only when it touches an
    endpoint of the longer edge; true edge-interior T-junctions intentionally
    remain the responsibility of the normal edge-search fallback.
    """
    fixed_count = len(fixed_angles)
    moving_count = len(moving_angles)
    endpoint_pairs = []
    if kind == 0:
        if abs(float(fixed_length) - float(moving_length)) >= 1.0:
            return None
        endpoint_pairs = [
            (fixed_edge, (moving_edge + 1) % moving_count),
            ((fixed_edge + 1) % fixed_count, moving_edge),
        ]
    elif kind == 1:
        endpoint_pairs = [
            (fixed_edge, (moving_edge + 1) % moving_count),
        ]
    elif kind == 2:
        endpoint_pairs = [
            ((fixed_edge + 1) % fixed_count, moving_edge),
        ]
    elif kind == 3:
        long_length = max(float(fixed_length), float(moving_length))
        short_length = min(float(fixed_length), float(moving_length))
        end_offset = max(0.0, long_length - short_length)
        if abs(float(partition_offset)) <= endpoint_tolerance_mm:
            endpoint_pairs = [
                ((fixed_edge + 1) % fixed_count, moving_edge),
            ]
        elif abs(float(partition_offset) - end_offset) <= endpoint_tolerance_mm:
            endpoint_pairs = [
                (fixed_edge, (moving_edge + 1) % moving_count),
            ]
        else:
            return None
    else:
        raise ValueError("unknown edge alignment kind: {}".format(kind))

    errors = []
    for fixed_vertex, moving_vertex in endpoint_pairs:
        angle_sum = float(
            fixed_angles[fixed_vertex] + moving_angles[moving_vertex]
        )
        errors.append(min(abs(angle_sum - 90.0), abs(angle_sum - 180.0)))
    return max(errors) if errors else None


def unique_partition_offsets(offsets: Sequence[float]) -> List[float]:
    unique = []
    for offset in offsets:
        value = float(offset)
        if not any(abs(value - old) < 0.5 for old in unique):
            unique.append(value)
    return unique


def state_polygons(
    polygons: Sequence[np.ndarray],
    state: SearchState,
) -> Dict[int, np.ndarray]:
    return {
        piece_id: apply_transform(polygons[piece_id], transform)
        for piece_id, transform in state.transforms.items()
    }


def partial_score(
    polygons: Sequence[np.ndarray],
    state: SearchState,
    polygon_areas: Optional[Sequence[float]] = None,
) -> float:
    placed = state_polygons(polygons, state)
    all_points = np.concatenate(list(placed.values()), axis=0).astype(np.float32)
    (_, _), (width, height), _ = cv2.minAreaRect(all_points)
    rectangle_area = max(1e-6, float(width * height))
    if polygon_areas is None:
        pieces_area = sum(
            polygon_area(polygons[piece_id])
            for piece_id in placed
        )
    else:
        pieces_area = sum(
            float(polygon_areas[piece_id])
            for piece_id in placed
        )
    compactness = max(0.0, rectangle_area / max(pieces_area, 1e-6) - 1.0)
    return state.match_error + 0.08 * compactness


def raster_union_metrics(
    polygons: Sequence[np.ndarray],
    pixels_per_mm: float = 4.0,
) -> Tuple[int, float]:
    outer_vertices, connected_ratio, _ = raster_union_outline(
        polygons,
        pixels_per_mm=pixels_per_mm,
    )
    return outer_vertices, connected_ratio


def raster_union_outline(
    polygons: Sequence[np.ndarray],
    pixels_per_mm: float = 4.0,
) -> Tuple[int, float, np.ndarray]:
    """Return simplified vertex count, connectivity and dense outer outline."""
    all_points = np.concatenate(polygons, axis=0)
    minimum = np.floor(np.min(all_points, axis=0) - 3.0)
    maximum = np.ceil(np.max(all_points, axis=0) + 3.0)
    size = np.maximum(1, np.ceil((maximum - minimum) * pixels_per_mm)).astype(int)
    mask = np.zeros((int(size[1]), int(size[0])), dtype=np.uint8)
    for polygon in polygons:
        shifted = np.round((polygon - minimum) * pixels_per_mm).astype(np.int32)
        cv2.fillPoly(mask, [shifted], 255)
    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_NONE,
    )
    if not contours:
        return 99, 0.0, np.empty((0, 2), dtype=np.float32)
    largest = max(contours, key=cv2.contourArea)
    perimeter = cv2.arcLength(largest, True)
    outer = cv2.approxPolyDP(largest, perimeter * 0.015, True)
    connected_ratio = cv2.contourArea(largest) / max(
        1.0,
        sum(cv2.contourArea(contour) for contour in contours),
    )
    outline = (
        largest.reshape(-1, 2).astype(np.float32) / pixels_per_mm
        + minimum
    )
    return len(outer), float(connected_ratio), outline


def outer_rectangle_corner_metrics(
    polygons: Sequence[np.ndarray],
    pixels_per_mm: float = 4.0,
    probe_distance_mm: float = 6.0,
    outline: Optional[np.ndarray] = None,
) -> Tuple[List[float], float, float]:
    """Measure the real union angle at each minimum-area rectangle corner.

    Unlike ``minAreaRect``, this follows the assembled outline on both sides
    of every corner. A trapezoid or chamfer therefore cannot pass merely
    because its bounding box is rectangular.
    """
    if outline is None:
        _, _, outline = raster_union_outline(
            polygons,
            pixels_per_mm=pixels_per_mm,
        )
    if len(outline) < 4:
        return [], float("inf"), float("inf")

    all_points = np.concatenate(polygons, axis=0).astype(np.float32)
    box = cv2.boxPoints(cv2.minAreaRect(all_points)).astype(np.float32)
    if _vision_fast is not None:
        angles, maximum_error, maximum_offset = (
            _vision_fast.outer_corner_metrics(
                outline,
                box,
                probe_distance_mm=float(probe_distance_mm),
            )
        )
        return (
            [float(angle) for angle in angles],
            float(maximum_error),
            float(maximum_offset),
        )
    angles = []
    offsets = []
    outline_count = len(outline)

    def probe(index, direction):
        anchor = outline[index]
        minimum_distance_squared = probe_distance_mm * probe_distance_mm
        for step in range(1, outline_count):
            point = outline[(index + direction * step) % outline_count]
            delta = point - anchor
            distance_squared = float(
                delta[0] * delta[0] + delta[1] * delta[1]
            )
            if distance_squared >= minimum_distance_squared:
                return point
        return None

    for box_corner in box:
        corner_deltas = outline - box_corner
        distances_squared = np.einsum(
            "ij,ij->i",
            corner_deltas,
            corner_deltas,
            optimize=False,
        )
        corner_index = int(np.argmin(distances_squared))
        anchor = outline[corner_index]
        previous = probe(corner_index, -1)
        following = probe(corner_index, 1)
        offsets.append(sqrt(float(distances_squared[corner_index])))
        if previous is None or following is None:
            return [], float("inf"), float("inf")
        first = previous - anchor
        second = following - anchor
        first_length_squared = float(np.dot(first, first))
        second_length_squared = float(np.dot(second, second))
        denominator = max(
            1e-8,
            sqrt(first_length_squared * second_length_squared),
        )
        cosine_value = np.clip(
            float(np.dot(first, second)) / denominator,
            -1.0,
            1.0,
        )
        angles.append(float(degrees(np.arccos(cosine_value))))

    maximum_error = max(abs(angle - 90.0) for angle in angles)
    return angles, float(maximum_error), float(max(offsets))


def rectangular_piece_diagnostics(
    polygons: Sequence[np.ndarray],
    maximum_angle_error_deg: float = 8.0,
    minimum_fill_ratio: float = 0.90,
    minimum_side_mm: float = 8.0,
) -> Tuple[bool, List[np.ndarray], List[Dict[str, float]]]:
    """Validate source pieces as high-confidence physical rectangles."""
    boxes = []
    diagnostics = []
    all_rectangular = bool(polygons)
    for piece_id, polygon in enumerate(polygons):
        contour = np.asarray(polygon, dtype=np.float32).reshape(-1, 1, 2)
        hull = cv2.convexHull(contour)
        perimeter = max(1e-6, float(cv2.arcLength(hull, True)))
        quad = cv2.approxPolyDP(hull, 0.015 * perimeter, True).reshape(-1, 2)
        box = cv2.boxPoints(cv2.minAreaRect(hull)).astype(np.float32)
        box_area = max(1e-6, abs(float(cv2.contourArea(box))))
        fill_ratio = abs(float(cv2.contourArea(hull))) / box_area
        side_lengths = np.linalg.norm(
            np.roll(box, -1, axis=0) - box,
            axis=1,
        )
        minimum_measured_side = float(np.min(side_lengths))
        if len(quad) == 4:
            angles = interior_angles(quad)
            maximum_angle_error = float(np.max(np.abs(angles - 90.0)))
        else:
            maximum_angle_error = float("inf")
        accepted = bool(
            len(quad) == 4
            and maximum_angle_error <= float(maximum_angle_error_deg)
            and fill_ratio >= float(minimum_fill_ratio)
            and minimum_measured_side >= float(minimum_side_mm)
        )
        all_rectangular = all_rectangular and accepted
        boxes.append(box)
        diagnostics.append(
            {
                "piece_id": int(piece_id),
                "convex_vertices": int(len(quad)),
                "maximum_angle_error_deg": (
                    round(maximum_angle_error, 3)
                    if np.isfinite(maximum_angle_error)
                    else None
                ),
                "rectangle_fill_ratio": round(float(fill_ratio), 4),
                "minimum_side_mm": round(minimum_measured_side, 3),
                "accepted": bool(accepted),
            }
        )
    return bool(all_rectangular), boxes, diagnostics


def enumerate_rectangular_piece_layouts(
    source_rectangles: Sequence[np.ndarray],
    long_side_range_mm: Tuple[float, float] = (90.0, 120.0),
    short_side_range_mm: Tuple[float, float] = (50.0, 90.0),
    maximum_join_error_mm: float = 3.0,
    minimum_fill_ratio: float = 0.94,
) -> List[Dict[str, object]]:
    """Enumerate row, column, grid and T-junction rectangle layouts."""
    rectangles = [
        np.asarray(rectangle, dtype=np.float32).reshape(4, 2)
        for rectangle in source_rectangles
    ]
    if not rectangles:
        return []
    dimensions = []
    for rectangle in rectangles:
        lengths = np.linalg.norm(
            np.roll(rectangle, -1, axis=0) - rectangle,
            axis=1,
        )
        first = float(np.mean(lengths[0::2]))
        second = float(np.mean(lengths[1::2]))
        dimensions.append((max(first, second), min(first, second)))

    cache = {}

    def shifted_slots(slots, offset_x, offset_y):
        return {
            piece_id: (
                x + float(offset_x),
                y + float(offset_y),
                width,
                height,
            )
            for piece_id, (x, y, width, height) in slots.items()
        }

    def layout_key(layout):
        values = []
        for piece_id in sorted(layout["slots"]):
            values.extend(
                round(float(value), 2)
                for value in layout["slots"][piece_id]
            )
        return (
            round(float(layout["width"]), 2),
            round(float(layout["height"]), 2),
            tuple(values),
        )

    def layouts_for(mask):
        if mask in cache:
            return cache[mask]
        if mask & (mask - 1) == 0:
            piece_id = int(mask.bit_length() - 1)
            long_side, short_side = dimensions[piece_id]
            cache[mask] = [
                {
                    "width": width,
                    "height": height,
                    "slots": {piece_id: (0.0, 0.0, width, height)},
                    "join_error_mm": 0.0,
                }
                for width, height in (
                    (long_side, short_side),
                    (short_side, long_side),
                )
            ]
            return cache[mask]

        candidates = {}
        lowest_bit = mask & -mask
        first_mask = (mask - 1) & mask
        while first_mask:
            second_mask = mask ^ first_mask
            if second_mask and first_mask & lowest_bit:
                for first in layouts_for(first_mask):
                    for second in layouts_for(second_mask):
                        height_error = abs(
                            float(first["height"])
                            - float(second["height"])
                        )
                        if height_error <= float(maximum_join_error_mm):
                            slots = dict(first["slots"])
                            slots.update(
                                shifted_slots(
                                    second["slots"], first["width"], 0.0
                                )
                            )
                            layout = {
                                "width": float(first["width"])
                                + float(second["width"]),
                                "height": max(
                                    float(first["height"]),
                                    float(second["height"]),
                                ),
                                "slots": slots,
                                "join_error_mm": (
                                    float(first["join_error_mm"])
                                    + float(second["join_error_mm"])
                                    + height_error
                                ),
                            }
                            key = layout_key(layout)
                            if (
                                key not in candidates
                                or layout["join_error_mm"]
                                < candidates[key]["join_error_mm"]
                            ):
                                candidates[key] = layout

                        width_error = abs(
                            float(first["width"])
                            - float(second["width"])
                        )
                        if width_error <= float(maximum_join_error_mm):
                            slots = dict(first["slots"])
                            slots.update(
                                shifted_slots(
                                    second["slots"], 0.0, first["height"]
                                )
                            )
                            layout = {
                                "width": max(
                                    float(first["width"]),
                                    float(second["width"]),
                                ),
                                "height": float(first["height"])
                                + float(second["height"]),
                                "slots": slots,
                                "join_error_mm": (
                                    float(first["join_error_mm"])
                                    + float(second["join_error_mm"])
                                    + width_error
                                ),
                            }
                            key = layout_key(layout)
                            if (
                                key not in candidates
                                or layout["join_error_mm"]
                                < candidates[key]["join_error_mm"]
                            ):
                                candidates[key] = layout
            first_mask = (first_mask - 1) & mask
        cache[mask] = list(candidates.values())
        return cache[mask]

    all_mask = (1 << len(rectangles)) - 1
    total_area = sum(width * height for width, height in dimensions)
    results = []
    for layout in layouts_for(all_mask):
        short_side, long_side = sorted(
            (float(layout["width"]), float(layout["height"]))
        )
        fill_ratio = total_area / max(1e-6, short_side * long_side)
        if not (
            float(long_side_range_mm[0]) <= long_side
            <= float(long_side_range_mm[1])
            and float(short_side_range_mm[0]) <= short_side
            <= float(short_side_range_mm[1])
            and fill_ratio >= float(minimum_fill_ratio)
        ):
            continue
        targets = []
        rotations = []
        fit_error = 0.0
        for piece_id, source in enumerate(rectangles):
            x, y, width, height = layout["slots"][piece_id]
            target = np.asarray(
                [
                    [x, y],
                    [x + width, y],
                    [x + width, y + height],
                    [x, y + height],
                ],
                dtype=np.float32,
            )
            transform, error = best_rigid_fit(source, target)
            rotation, _ = transform
            targets.append(target)
            rotations.append(
                float(
                    degrees(
                        atan2(
                            float(rotation[1, 0]),
                            float(rotation[0, 0]),
                        )
                    )
                )
            )
            fit_error += float(error)
        results.append(
            {
                "targets": targets,
                "rotations": rotations,
                "long_side_mm": long_side,
                "short_side_mm": short_side,
                "fill_ratio": float(fill_ratio),
                "join_error_mm": float(layout["join_error_mm"]),
                "fit_error_mm": float(fit_error),
                "slot_signature": tuple(
                    tuple(
                        round(float(value), 3)
                        for value in layout["slots"][piece_id]
                    )
                    for piece_id in range(len(rectangles))
                ),
            }
        )
    results.sort(
        key=lambda item: (
            round(1.0 - float(item["fill_ratio"]), 6),
            round(float(item["join_error_mm"]), 6),
            round(float(item["fit_error_mm"]), 6),
            item["slot_signature"],
        )
    )
    return results


def gapped_rectangle_edge_metrics(
    polygons: Sequence[np.ndarray],
    maximum_side_distance_mm: float = 6.0,
    maximum_parallel_error_deg: float = 10.0,
) -> Dict[str, object]:
    """Measure four rectangle side lines without requiring closed corners.

    Hand-placed pieces intentionally leave gaps.  The raster union therefore
    gains artificial diagonal connectors and extra corners.  This metric
    instead assigns real fragment edges to the four sides of the assembly's
    minimum-area rectangle, unions their projected coverage, and measures the
    angles between the fitted directions of adjacent supported sides.
    """
    placed = [np.asarray(polygon, dtype=np.float32) for polygon in polygons]
    all_points = np.concatenate(placed, axis=0).astype(np.float32)
    box = cv2.boxPoints(cv2.minAreaRect(all_points)).astype(np.float64)
    side_edges = [[] for _ in range(4)]
    side_lengths = []
    for side_id in range(4):
        start = box[side_id]
        delta = box[(side_id + 1) % 4] - start
        side_lengths.append(float(np.linalg.norm(delta)))

    distance_limit = max(1e-6, float(maximum_side_distance_mm))
    angle_limit = max(1e-6, float(maximum_parallel_error_deg))
    for polygon in placed:
        for edge_id in range(len(polygon)):
            first = polygon[edge_id].astype(np.float64)
            second = polygon[(edge_id + 1) % len(polygon)].astype(np.float64)
            edge = second - first
            edge_length = float(np.linalg.norm(edge))
            if edge_length < 1.0:
                continue
            edge_unit = edge / edge_length
            best = None
            for side_id in range(4):
                start = box[side_id]
                side = box[(side_id + 1) % 4] - start
                side_length = side_lengths[side_id]
                if side_length < 1e-6:
                    continue
                side_unit = side / side_length
                cosine = np.clip(abs(float(np.dot(edge_unit, side_unit))), 0.0, 1.0)
                parallel_error = float(degrees(np.arccos(cosine)))
                if parallel_error > angle_limit:
                    continue
                normal = np.asarray([-side_unit[1], side_unit[0]])
                maximum_distance = max(
                    abs(float(np.dot(first - start, normal))),
                    abs(float(np.dot(second - start, normal))),
                )
                if maximum_distance > distance_limit:
                    continue
                projections = sorted(
                    (
                        float(np.dot(first - start, side_unit)),
                        float(np.dot(second - start, side_unit)),
                    )
                )
                interval_start = max(0.0, projections[0])
                interval_end = min(side_length, projections[1])
                overlap = interval_end - interval_start
                if overlap < 1.0:
                    continue
                score = parallel_error / angle_limit + maximum_distance / distance_limit
                if best is None or score < best[0]:
                    aligned = edge_unit if np.dot(edge_unit, side_unit) >= 0 else -edge_unit
                    best = (
                        score,
                        side_id,
                        interval_start,
                        interval_end,
                        aligned,
                        overlap,
                    )
            if best is not None:
                side_edges[best[1]].append(best[2:])

    coverage_ratios = []
    fitted_directions = []
    for side_id, records in enumerate(side_edges):
        intervals = sorted((record[0], record[1]) for record in records)
        covered = 0.0
        if intervals:
            current_start, current_end = intervals[0]
            for interval_start, interval_end in intervals[1:]:
                if interval_start <= current_end:
                    current_end = max(current_end, interval_end)
                else:
                    covered += current_end - current_start
                    current_start, current_end = interval_start, interval_end
            covered += current_end - current_start
        coverage_ratios.append(covered / max(1e-6, side_lengths[side_id]))
        if not records:
            fitted_directions.append(None)
            continue
        vector = sum(
            (record[2] * record[3] for record in records),
            np.zeros(2, dtype=np.float64),
        )
        vector_length = float(np.linalg.norm(vector))
        fitted_directions.append(
            vector / vector_length if vector_length > 1e-6 else None
        )

    corner_angles = []
    for side_id in range(4):
        first = fitted_directions[side_id]
        second = fitted_directions[(side_id + 1) % 4]
        if first is None or second is None:
            continue
        cosine = np.clip(abs(float(np.dot(first, second))), 0.0, 1.0)
        corner_angles.append(float(degrees(np.arccos(cosine))))
    return {
        "side_coverage_ratios": [float(value) for value in coverage_ratios],
        "minimum_side_coverage_ratio": float(min(coverage_ratios)),
        "mean_side_coverage_ratio": float(np.mean(coverage_ratios)),
        "supported_side_count": int(sum(value > 0.0 for value in coverage_ratios)),
        "corner_angles_deg": corner_angles,
        "minimum_corner_angle_deg": (
            float(min(corner_angles)) if corner_angles else 0.0
        ),
    }


def final_score(
    polygons: Sequence[np.ndarray],
    state: SearchState,
    target_size_mm: Tuple[float, float],
) -> Tuple[float, Dict[str, float]]:
    placed_map = state_polygons(polygons, state)
    placed = list(placed_map.values())
    all_points = np.concatenate(placed, axis=0).astype(np.float32)
    (_, _), (width, height), _ = cv2.minAreaRect(all_points)
    short_side, long_side = sorted((float(width), float(height)))
    target_short, target_long = sorted(target_size_mm)

    rectangle_area = max(1e-6, short_side * long_side)
    pieces_area = sum(polygon_area(polygon) for polygon in placed)
    fill_error = abs(rectangle_area - pieces_area) / rectangle_area
    size_error = (
        abs(short_side - target_short) / target_short
        + abs(long_side - target_long) / target_long
    )
    outer_vertices, connected_ratio = raster_union_metrics(placed)
    outer_error = abs(outer_vertices - 4) * 0.20
    connected_error = (1.0 - connected_ratio) * 2.0
    score = (
        state.match_error * 0.8
        + fill_error * 3.0
        + size_error * 1.8
        + outer_error
        + connected_error
    )
    metrics = {
        "score": score,
        "short_side_mm": short_side,
        "long_side_mm": long_side,
        "fill_ratio": pieces_area / rectangle_area,
        "outer_vertices": float(outer_vertices),
        "connected_ratio": connected_ratio,
        "match_error": state.match_error,
    }
    return score, metrics


def unknown_rectangle_score(
    polygons: Sequence[np.ndarray],
    state: SearchState,
    long_side_range_mm: Tuple[float, float] = (90.0, 120.0),
    short_side_range_mm: Tuple[float, float] = (50.0, 90.0),
) -> Tuple[float, Dict[str, float]]:
    """不预设宽高，只按矩形完整度和赛题尺寸范围评价拼法。"""
    placed_map = state_polygons(polygons, state)
    placed = list(placed_map.values())
    all_points = np.concatenate(placed, axis=0).astype(np.float32)
    (_, _), (width, height), _ = cv2.minAreaRect(all_points)
    short_side, long_side = sorted((float(width), float(height)))
    rectangle_area = max(1e-6, short_side * long_side)
    pieces_area = sum(polygon_area(polygon) for polygon in placed)
    fill_ratio = pieces_area / rectangle_area
    fill_error = abs(1.0 - fill_ratio)
    outer_vertices, connected_ratio, outer_outline = raster_union_outline(
        placed
    )
    (
        outer_corner_angles,
        outer_corner_max_error,
        outer_corner_max_offset,
    ) = outer_rectangle_corner_metrics(placed, outline=outer_outline)
    gapped_edge_metrics = gapped_rectangle_edge_metrics(placed)

    def range_error(value, limits):
        lower, upper = limits
        if value < lower:
            return (lower - value) / max(lower, 1e-6)
        if value > upper:
            return (value - upper) / max(upper, 1e-6)
        return 0.0

    dimension_error = (
        range_error(long_side, long_side_range_mm)
        + range_error(short_side, short_side_range_mm)
    )
    outer_error = abs(outer_vertices - 4) * 0.24
    connected_error = (1.0 - connected_ratio) * 2.5
    score = (
        state.match_error * 0.8
        + fill_error * 4.0
        + outer_error
        + connected_error
        + dimension_error * 4.0
    )
    metrics = {
        "score": float(score),
        "short_side_mm": short_side,
        "long_side_mm": long_side,
        "fill_ratio": fill_ratio,
        "outer_vertices": float(outer_vertices),
        "connected_ratio": connected_ratio,
        "match_error": state.match_error,
        "dimension_error": dimension_error,
        "outer_corner_angles_deg": [
            round(float(angle), 3) for angle in outer_corner_angles
        ],
        "outer_corner_max_error_deg": float(outer_corner_max_error),
        "outer_corner_max_offset_mm": float(outer_corner_max_offset),
        "gapped_side_coverage_ratios": gapped_edge_metrics[
            "side_coverage_ratios"
        ],
        "gapped_minimum_side_coverage_ratio": float(
            gapped_edge_metrics["minimum_side_coverage_ratio"]
        ),
        "gapped_mean_side_coverage_ratio": float(
            gapped_edge_metrics["mean_side_coverage_ratio"]
        ),
        "gapped_supported_side_count": float(
            gapped_edge_metrics["supported_side_count"]
        ),
        "gapped_corner_angles_deg": gapped_edge_metrics[
            "corner_angles_deg"
        ],
        "gapped_minimum_corner_angle_deg": float(
            gapped_edge_metrics["minimum_corner_angle_deg"]
        ),
        "strategy": "unknown_rectangle_edge_search",
    }
    return float(score), metrics


def solve_edge_search(
    polygons: Sequence[np.ndarray],
    target_size_mm: Optional[Tuple[float, float]] = (100.0, 60.0),
    edge_tolerance: float = 0.85,
    beam_width: int = 2000,
    progress_callback=None,
) -> Tuple[SearchState, Dict[str, float]]:
    """返回最接近目标矩形的拼接状态。"""
    if not 2 <= len(polygons) <= 4:
        raise ValueError("求解器只支持2至4块碎片")
    polygons = [np.asarray(polygon, dtype=np.float32) for polygon in polygons]
    lengths = [edge_lengths(polygon) for polygon in polygons]
    polygon_areas = [polygon_area(polygon) for polygon in polygons]
    anchor = int(np.argmax(polygon_areas))
    identity = (
        np.eye(2, dtype=np.float32),
        np.zeros(2, dtype=np.float32),
    )
    states = [
        SearchState(
            transforms={anchor: identity},
            used_edges=frozenset(),
            match_error=0.0,
            matches=(),
        )
    ]

    total_stages = max(1, len(polygons) - 1)
    while len(states[0].transforms) < len(polygons):
        stage_index = len(states[0].transforms) - 1
        expanded = []
        for state_index, state in enumerate(states):
            if progress_callback is not None:
                progress_callback(
                    min(
                        0.94,
                        (
                            stage_index
                            + state_index / max(1, len(states))
                        )
                        / total_stages,
                    ),
                    "edge stage {}/{}".format(
                        stage_index + 1,
                        total_stages,
                    ),
                )
            placed_world = state_polygons(polygons, state)
            unplaced = [
                piece_id
                for piece_id in range(len(polygons))
                if piece_id not in state.transforms
            ]
            for moving_id in unplaced:
                for fixed_id, fixed_polygon in placed_world.items():
                    for fixed_edge, fixed_length in enumerate(lengths[fixed_id]):
                        for moving_edge, moving_length in enumerate(
                            lengths[moving_id]
                        ):
                            relative_error = abs(
                                float(fixed_length - moving_length)
                            ) / max(float(fixed_length), float(moving_length))
                            if relative_error > edge_tolerance:
                                continue

                            transforms_for_edge = edge_alignment_transforms(
                                polygons[moving_id],
                                moving_edge,
                                fixed_polygon,
                                fixed_edge,
                            )
                            for transform in transforms_for_edge:
                                moving_world = apply_transform(
                                    polygons[moving_id],
                                    transform,
                                )
                                moving_area = polygon_areas[moving_id]
                                overlaps = [
                                    convex_overlap_area(moving_world, other)
                                    for other in placed_world.values()
                                ]
                                if any(
                                    overlap > moving_area * 0.025
                                    for overlap in overlaps
                                ):
                                    continue

                                transforms = dict(state.transforms)
                                transforms[moving_id] = transform
                                used_edges = set(state.used_edges)
                                used_edges.add((moving_id, moving_edge))
                                expanded.append(
                                    SearchState(
                                        transforms=transforms,
                                        used_edges=frozenset(used_edges),
                                        match_error=(
                                            state.match_error
                                            + relative_error * 0.15
                                        ),
                                        matches=state.matches
                                        + (
                                            (
                                                fixed_id,
                                                fixed_edge,
                                                moving_id,
                                                moving_edge,
                                                relative_error,
                                            ),
                                        ),
                                    )
                                )

        if not expanded:
            raise RuntimeError("没有找到可继续拼接的候选边")
        expanded.sort(
            key=lambda state: partial_score(
                polygons,
                state,
                polygon_areas=polygon_areas,
            )
        )
        states = expanded[:beam_width]

    if target_size_mm is None:
        scored = [
            (unknown_rectangle_score(polygons, state), state)
            for state in states
        ]
    else:
        scored = [
            (final_score(polygons, state, target_size_mm), state)
            for state in states
        ]
    (score, metrics), best_state = min(scored, key=lambda item: item[0][0])
    metrics["score"] = score
    if progress_callback is not None:
        progress_callback(1.0, "edge search complete")
    return best_state, metrics


def generate_edge_search_states(
    polygons: Sequence[np.ndarray],
    edge_tolerance: float = 0.35,
    beam_width: int = 5000,
    allow_partitioned_edges: bool = False,
    deduplicate_states: bool = False,
    progress_callback=None,
    overlap_workers: int = 1,
) -> List[SearchState]:
    """Generate complete edge-alignment candidates for texture-aware solvers.

    Unlike ``solve_edge_search`` this function intentionally does not choose a
    winner.  Mode 3 can therefore re-rank the geometric candidates with the
    playing-card artwork along each seam.
    """
    if not 2 <= len(polygons) <= 4:
        raise ValueError("candidate generation supports 2 to 4 pieces")
    polygons = [np.asarray(polygon, dtype=np.float32) for polygon in polygons]
    lengths = [edge_lengths(polygon) for polygon in polygons]
    polygon_areas = [polygon_area(polygon) for polygon in polygons]
    partition_offsets = {}
    if allow_partitioned_edges:
        edge_records = [
            (piece_id, edge_id, float(length))
            for piece_id, piece_lengths in enumerate(lengths)
            for edge_id, length in enumerate(piece_lengths)
        ]
        for long_piece, long_edge, long_length in edge_records:
            shorter = [
                record
                for record in edge_records
                if record[0] != long_piece
                and record[2] < long_length * (1.0 - edge_tolerance)
                and record[2] >= long_length * 0.12
            ]
            for count in range(2, min(3, len(shorter)) + 1):
                for group in combinations(shorter, count):
                    if len({record[0] for record in group}) != count:
                        continue
                    group_length = sum(record[2] for record in group)
                    relative_error = abs(
                        group_length - long_length
                    ) / max(long_length, 1e-6)
                    if relative_error > max(edge_tolerance, 0.20):
                        continue
                    long_key = (long_piece, long_edge)
                    for ordering in permutations(group):
                        offset = 0.0
                        for short_piece, short_edge, short_length in ordering:
                            short_key = (short_piece, short_edge)
                            partition_offsets.setdefault(
                                (long_key, short_key),
                                set(),
                            ).add(round(offset, 3))
                            offset += short_length
    anchor = int(np.argmax(polygon_areas))
    identity = (
        np.eye(2, dtype=np.float32),
        np.zeros(2, dtype=np.float32),
    )
    states = [
        SearchState(
            transforms={anchor: identity},
            used_edges=frozenset(),
            match_error=0.0,
            matches=(),
        )
    ]

    total_stages = max(1, len(polygons) - 1)
    while len(states[0].transforms) < len(polygons):
        stage_index = len(states[0].transforms) - 1
        state_batches = []
        alignment_jobs = []
        alignment_owners = []
        for state_index, state in enumerate(states):
            if progress_callback is not None:
                progress_callback(
                    min(
                        0.96,
                        (
                            stage_index
                            + state_index / max(1, len(states))
                        )
                        / total_stages,
                    ),
                    "candidate stage {}/{}".format(
                        stage_index + 1,
                        total_stages,
                    ),
                )
            placed_world = state_polygons(polygons, state)
            unplaced = [
                piece_id
                for piece_id in range(len(polygons))
                if piece_id not in state.transforms
            ]
            pending = []
            for moving_id in unplaced:
                for fixed_id, fixed_polygon in placed_world.items():
                    for fixed_edge, fixed_length in enumerate(lengths[fixed_id]):
                        for moving_edge, moving_length in enumerate(
                            lengths[moving_id]
                        ):
                            relative_error = abs(
                                float(fixed_length - moving_length)
                            ) / max(float(fixed_length), float(moving_length))
                            fixed_key = (fixed_id, fixed_edge)
                            moving_key = (moving_id, moving_edge)
                            if fixed_length >= moving_length:
                                partition_key = (fixed_key, moving_key)
                            else:
                                partition_key = (moving_key, fixed_key)
                            is_full_match = relative_error <= edge_tolerance
                            is_partition_match = (
                                allow_partitioned_edges
                                and partition_key in partition_offsets
                            )
                            if not is_full_match and not is_partition_match:
                                continue
                            if is_full_match:
                                # Partition fallback already has many offset
                                # hypotheses. Keep only the most stable
                                # midpoint alignment for ordinary full seams.
                                kinds = (
                                    (0,)
                                    if allow_partitioned_edges
                                    else full_alignment_kinds(
                                        moving_length,
                                        fixed_length,
                                    )
                                )
                                for kind in kinds:
                                    alignment_jobs.append(
                                        (
                                            moving_id,
                                            moving_edge,
                                            fixed_polygon,
                                            fixed_edge,
                                            kind,
                                            0.0,
                                        )
                                    )
                                    alignment_owners.append(
                                        (
                                            len(state_batches),
                                            moving_id,
                                            moving_edge,
                                            fixed_id,
                                            fixed_edge,
                                            relative_error,
                                            is_full_match,
                                        )
                                    )
                            if is_partition_match:
                                for offset in unique_partition_offsets(
                                    sorted(partition_offsets[partition_key])
                                ):
                                    alignment_jobs.append(
                                        (
                                            moving_id,
                                            moving_edge,
                                            fixed_polygon,
                                            fixed_edge,
                                            3,
                                            offset,
                                        )
                                    )
                                    alignment_owners.append(
                                        (
                                            len(state_batches),
                                            moving_id,
                                            moving_edge,
                                            fixed_id,
                                            fixed_edge,
                                            relative_error,
                                            is_full_match,
                                        )
                                    )
            state_batches.append(
                (state, tuple(placed_world.values()), pending)
            )

        alignment_results = batch_edge_alignment_world(
            polygons,
            alignment_jobs,
            workers=overlap_workers,
        )
        for owner, (transform, moving_world) in zip(
            alignment_owners,
            alignment_results,
        ):
            (
                batch_index,
                moving_id,
                moving_edge,
                fixed_id,
                fixed_edge,
                relative_error,
                is_full_match,
            ) = owner
            state_batches[batch_index][2].append(
                (
                    moving_id,
                    moving_edge,
                    fixed_id,
                    fixed_edge,
                    relative_error,
                    is_full_match,
                    transform,
                    moving_world,
                )
            )

        overlap_first = []
        overlap_second = []
        overlap_owners = []
        for batch_index, (_, placed_values, pending) in enumerate(
            state_batches
        ):
            for pending_index, item in enumerate(pending):
                moving_world = item[7]
                for other in placed_values:
                    overlap_first.append(moving_world)
                    overlap_second.append(other)
                    overlap_owners.append((batch_index, pending_index))
        overlap_areas = batch_convex_overlap_areas(
            overlap_first,
            overlap_second,
            workers=overlap_workers,
        )
        overlaps = [
            np.zeros(len(pending), dtype=np.bool_)
            for _, _, pending in state_batches
        ]
        for area, (batch_index, pending_index) in zip(
            overlap_areas,
            overlap_owners,
        ):
            moving_id = state_batches[batch_index][2][pending_index][0]
            if area > polygon_areas[moving_id] * 0.025:
                overlaps[batch_index][pending_index] = True

        expanded = []
        for batch_index, (state, _, pending) in enumerate(state_batches):
            for pending_index, item in enumerate(pending):
                if overlaps[batch_index][pending_index]:
                    continue
                (
                    moving_id,
                    moving_edge,
                    fixed_id,
                    fixed_edge,
                    relative_error,
                    is_full_match,
                    transform,
                    _,
                ) = item
                transforms = dict(state.transforms)
                transforms[moving_id] = transform
                expanded.append(
                    SearchState(
                        transforms=transforms,
                        used_edges=state.used_edges
                        | {(moving_id, moving_edge)},
                        match_error=state.match_error
                        + (
                            relative_error * 0.15
                            if is_full_match
                            else 0.02
                        ),
                        matches=state.matches
                        + (
                            (
                                fixed_id,
                                fixed_edge,
                                moving_id,
                                moving_edge,
                                relative_error,
                            ),
                        ),
                    )
                )
        if not expanded:
            return []
        partial_scores, _, _, signatures = batch_beam_state_metrics(
            polygons,
            expanded,
            polygon_areas=polygon_areas,
            angle_step_deg=0.5,
            translation_step_mm=0.25,
            workers=overlap_workers,
        )
        if deduplicate_states:
            # The same placement is often reached through several attachment
            # orders.  Merge near-identical transforms before sorting and
            # beam pruning so later stages do not repeat the same geometry.
            deduplicated = {}
            for candidate_id, candidate in enumerate(expanded):
                candidate_score = float(partial_scores[candidate_id])
                signature = signatures[candidate_id]
                previous = deduplicated.get(signature)
                if (
                    previous is None
                    or candidate_score < previous[0]
                ):
                    deduplicated[signature] = (
                        candidate_score,
                        candidate,
                    )
            scored_expanded = list(deduplicated.values())
        else:
            # The native minimum-area rectangle is deliberately lightweight
            # and can differ from cv2.minAreaRect by a few 1e-4.  That is too
            # small to matter geometrically, but a Beam cutoff may turn it
            # into a different Mode-4 artwork candidate.  Let native scores
            # discard candidates far from the cutoff, then restore the
            # original Python/OpenCV ordering for every state that can still
            # enter this Beam.  The margin is comfortably above the measured
            # native-vs-OpenCV error while keeping expensive Python scoring to
            # a small boundary set.
            native_order = np.argsort(partial_scores, kind="stable")
            cutoff_index = min(beam_width, len(expanded)) - 1
            native_cutoff = float(partial_scores[native_order[cutoff_index]])
            refinement_ids = [
                candidate_id
                for candidate_id, score in enumerate(partial_scores)
                if float(score) <= native_cutoff + 0.002
            ]
            scored_expanded = [
                (
                    partial_score(
                        polygons,
                        expanded[candidate_id],
                        polygon_areas=polygon_areas,
                    ),
                    expanded[candidate_id],
                )
                for candidate_id in refinement_ids
            ]
        scored_expanded.sort(key=lambda item: item[0])
        states = [
            candidate for _, candidate in scored_expanded[:beam_width]
        ]
    if progress_callback is not None:
        progress_callback(1.0, "candidate generation complete")
    return states


def transform_signature(
    state: SearchState,
    angle_step_deg: float = 1.0,
    translation_step_mm: float = 0.75,
) -> Tuple[int, ...]:
    """Quantize placements so repeated construction paths can be merged."""
    signature = []
    for piece_id in sorted(state.transforms):
        rotation, translation = state.transforms[piece_id]
        angle = degrees(atan2(rotation[1, 0], rotation[0, 0]))
        signature.extend(
            (
                int(piece_id),
                int(round(angle / angle_step_deg)),
                int(round(float(translation[0]) / translation_step_mm)),
                int(round(float(translation[1]) / translation_step_mm)),
            )
        )
    return tuple(signature)


def unknown_partial_is_feasible(
    polygons: Sequence[np.ndarray],
    state: SearchState,
    maximum_long_mm: float = 130.0,
    maximum_short_mm: float = 105.0,
    placed_world: Optional[Sequence[np.ndarray]] = None,
) -> bool:
    """Reject a partial layout whose bounding box cannot become legal."""
    placed = (
        list(placed_world)
        if placed_world is not None
        else list(state_polygons(polygons, state).values())
    )
    points = np.concatenate(placed, axis=0).astype(np.float32)
    (_, _), (width, height), _ = cv2.minAreaRect(points)
    short_side, long_side = sorted((float(width), float(height)))
    if long_side > maximum_long_mm or short_side > maximum_short_mm:
        return False

    return True


def cheap_unknown_score(
    polygons: Sequence[np.ndarray],
    state: SearchState,
    polygon_areas: Optional[Sequence[float]] = None,
) -> float:
    """Rank candidates without the expensive raster-union operation."""
    placed = list(state_polygons(polygons, state).values())
    points = np.concatenate(placed, axis=0).astype(np.float32)
    (_, _), (width, height), _ = cv2.minAreaRect(points)
    short_side, long_side = sorted((float(width), float(height)))
    rectangle_area = max(1e-6, short_side * long_side)
    if polygon_areas is None:
        pieces_area = sum(
            polygon_area(polygons[piece_id])
            for piece_id in state.transforms
        )
    else:
        pieces_area = sum(
            float(polygon_areas[piece_id])
            for piece_id in state.transforms
        )
    fill_error = abs(1.0 - pieces_area / rectangle_area)

    dimension_error = 0.0
    if long_side < 90.0:
        dimension_error += (90.0 - long_side) / 90.0
    elif long_side > 120.0:
        dimension_error += (long_side - 120.0) / 120.0
    if short_side < 50.0:
        dimension_error += (50.0 - short_side) / 50.0
    elif short_side > 90.0:
        dimension_error += (short_side - 90.0) / 90.0
    return (
        float(state.match_error)
        + fill_error * 3.5
        + dimension_error * 8.0
    )


def batch_beam_state_metrics(
    polygons: Sequence[np.ndarray],
    states: Sequence[SearchState],
    polygon_areas: Optional[Sequence[float]] = None,
    angle_step_deg: float = 1.0,
    translation_step_mm: float = 0.75,
    workers: int = 1,
):
    """Score and quantize Beam states in one native/OpenMP batch.

    The returned order is identical to ``states``.  Python remains the
    authoritative fallback so the optional extension never becomes a runtime
    requirement.
    """
    states = list(states)
    if polygon_areas is None:
        polygon_areas = [polygon_area(polygon) for polygon in polygons]
    polygon_areas = np.asarray(polygon_areas, dtype=np.float64)
    if not states:
        return (
            np.empty(0, dtype=np.float64),
            np.empty(0, dtype=np.float64),
            np.empty(0, dtype=np.bool_),
            [],
        )
    if _vision_fast is None or len(states) < 8:
        return (
            np.asarray(
                [
                    partial_score(
                        polygons,
                        state,
                        polygon_areas=polygon_areas,
                    )
                    for state in states
                ],
                dtype=np.float64,
            ),
            np.asarray(
                [
                    cheap_unknown_score(
                        polygons,
                        state,
                        polygon_areas=polygon_areas,
                    )
                    for state in states
                ],
                dtype=np.float64,
            ),
            np.asarray(
                [unknown_partial_is_feasible(polygons, state) for state in states],
                dtype=np.bool_,
            ),
            [
                transform_signature(
                    state,
                    angle_step_deg=angle_step_deg,
                    translation_step_mm=translation_step_mm,
                )
                for state in states
            ],
        )

    polygon_arrays = [
        np.asarray(polygon, dtype=np.float32).reshape(-1, 2)
        for polygon in polygons
    ]
    counts = np.asarray([len(polygon) for polygon in polygon_arrays], dtype=np.int32)
    padded = np.zeros(
        (len(polygons), int(np.max(counts)), 2),
        dtype=np.float32,
    )
    for piece_id, polygon in enumerate(polygon_arrays):
        padded[piece_id, : len(polygon)] = polygon

    state_count = len(states)
    piece_count = len(polygons)
    rotations = np.zeros(
        (state_count, piece_count, 2, 2),
        dtype=np.float32,
    )
    translations = np.zeros(
        (state_count, piece_count, 2),
        dtype=np.float32,
    )
    placed = np.zeros((state_count, piece_count), dtype=np.bool_)
    match_errors = np.empty(state_count, dtype=np.float64)
    for state_id, state in enumerate(states):
        match_errors[state_id] = float(state.match_error)
        for piece_id, (rotation, translation) in state.transforms.items():
            placed[state_id, piece_id] = True
            rotations[state_id, piece_id] = rotation
            translations[state_id, piece_id] = translation

    partial, cheap, feasible, quantized = _vision_fast.batch_beam_state_metrics(
        padded,
        counts,
        rotations,
        translations,
        placed,
        match_errors,
        polygon_areas,
        angle_step_deg=float(angle_step_deg),
        translation_step_mm=float(translation_step_mm),
        workers=max(1, int(workers)),
    )
    quantized = np.asarray(quantized, dtype=np.int64)
    signatures = []
    for state_id in range(state_count):
        signature = []
        for piece_id in range(piece_count):
            if not placed[state_id, piece_id]:
                continue
            signature.extend(
                (
                    int(piece_id),
                    int(quantized[state_id, piece_id, 0]),
                    int(quantized[state_id, piece_id, 1]),
                    int(quantized[state_id, piece_id, 2]),
                )
            )
        signatures.append(tuple(signature))
    return (
        np.asarray(partial, dtype=np.float64),
        np.asarray(cheap, dtype=np.float64),
        np.asarray(feasible, dtype=np.bool_),
        signatures,
    )


def build_partition_edge_offsets(
    lengths: Sequence[np.ndarray],
    edge_tolerance: float,
) -> Dict[
    Tuple[Tuple[int, int], Tuple[int, int]],
    Tuple[float, ...],
]:
    """Find offsets where two or three short edges can cover one long edge."""
    records = [
        (piece_id, edge_id, float(length))
        for piece_id, piece_lengths in enumerate(lengths)
        for edge_id, length in enumerate(piece_lengths)
    ]
    offsets = {}
    for long_piece, long_edge, long_length in records:
        shorter = [
            record
            for record in records
            if record[0] != long_piece
            and record[2] < long_length * (1.0 - edge_tolerance)
            and record[2] >= long_length * 0.12
        ]
        for count in range(2, min(3, len(shorter)) + 1):
            for group in combinations(shorter, count):
                if len({record[0] for record in group}) != count:
                    continue
                combined = sum(record[2] for record in group)
                if (
                    abs(combined - long_length)
                    / max(long_length, 1e-6)
                    > max(edge_tolerance, 0.16)
                ):
                    continue
                for ordering in permutations(group):
                    offset = 0.0
                    for short_piece, short_edge, short_length in ordering:
                        key = (
                            (long_piece, long_edge),
                            (short_piece, short_edge),
                        )
                        offsets.setdefault(key, set()).add(
                            round(offset, 3)
                        )
                        offset += short_length
    return {
        key: tuple(sorted(values))
        for key, values in offsets.items()
    }


def connection_topology_signature(
    state: SearchState,
) -> Tuple[Tuple[int, int], ...]:
    """Represent only which pieces touch, independent of placement order."""
    return tuple(
        sorted(
            {
                tuple(sorted((int(match[0]), int(match[2]))))
                for match in state.matches
            }
        )
    )


def select_diverse_unknown_states(
    polygons: Sequence[np.ndarray],
    states: Sequence[SearchState],
    beam_width: int,
    polygon_areas: Optional[Sequence[float]] = None,
    precomputed_scores: Optional[Sequence[float]] = None,
) -> List[SearchState]:
    """Keep several candidates from each connection topology."""
    if precomputed_scores is None:
        ordered = sorted(
            states,
            key=lambda state: cheap_unknown_score(
                polygons,
                state,
                polygon_areas=polygon_areas,
            ),
        )
    else:
        if len(precomputed_scores) != len(states):
            raise ValueError("precomputed score count must match states")
        ordered = [
            state
            for _, state in sorted(
                zip(precomputed_scores, states),
                key=lambda item: item[0],
            )
        ]
    beam_width = max(20, int(beam_width))
    per_topology = max(3, min(8, beam_width // 8))
    counts = {}
    selected = []
    selected_ids = set()
    for state in ordered:
        topology = connection_topology_signature(state)
        if counts.get(topology, 0) >= per_topology:
            continue
        selected.append(state)
        selected_ids.add(id(state))
        counts[topology] = counts.get(topology, 0) + 1
        if len(selected) >= beam_width:
            return selected
    for state in ordered:
        if id(state) in selected_ids:
            continue
        selected.append(state)
        if len(selected) >= beam_width:
            break
    return selected


def strong_rectangle_angle_fallback_allowed(
    metrics: Dict[str, float],
    relaxed_error_deg: Optional[float],
    maximum_outer_corner_offset_mm: float,
) -> bool:
    """Allow a noisy hand-cut corner only when all other rectangle cues agree."""
    return bool(
        relaxed_error_deg is not None
        and metrics["outer_vertices"] == 4.0
        and metrics["dimension_error"] <= 0.04
        and 0.92 <= metrics["fill_ratio"] <= 1.08
        and metrics["connected_ratio"] >= 0.999
        and metrics["match_error"] <= 0.08
        and metrics["outer_corner_max_offset_mm"]
        <= maximum_outer_corner_offset_mm
        and metrics["outer_corner_max_error_deg"]
        <= float(relaxed_error_deg)
    )


def gapped_rectangle_edge_fallback_allowed(
    metrics: Dict[str, float],
    minimum_corner_angle_deg: float = 80.0,
    minimum_side_coverage_ratio: float = 0.25,
    minimum_mean_side_coverage_ratio: float = 0.60,
) -> bool:
    """Accept a gapped Mode-2 rectangle from its four supported side lines."""
    return bool(
        metrics.get("gapped_supported_side_count", 0.0) == 4.0
        and metrics.get("gapped_minimum_corner_angle_deg", 0.0)
        >= float(minimum_corner_angle_deg)
        and metrics.get("gapped_minimum_side_coverage_ratio", 0.0)
        >= float(minimum_side_coverage_ratio)
        and metrics.get("gapped_mean_side_coverage_ratio", 0.0)
        >= float(minimum_mean_side_coverage_ratio)
        and metrics["outer_vertices"] <= 8.0
        and metrics["dimension_error"] <= 0.04
        and 0.86 <= metrics["fill_ratio"] <= 1.08
        and metrics["connected_ratio"] >= 0.999
        and metrics["match_error"] <= 0.06
    )


def rank_unknown_rectangle_candidates(
    polygons: Sequence[np.ndarray],
    edge_tolerance: float = 0.18,
    beam_width: int = 40,
    time_limit_seconds: float = 20.0,
    maximum_results: int = 24,
    allow_partitioned_edges: bool = False,
    maximum_outer_corner_error_deg: float = 5.0,
    strong_rectangle_outer_corner_error_deg: Optional[float] = None,
    maximum_outer_corner_offset_mm: float = 3.0,
    maximum_outer_vertices: int = 8,
    minimum_measured_fill_ratio: float = 0.92,
    long_side_range_mm: Tuple[float, float] = (90.0, 120.0),
    short_side_range_mm: Tuple[float, float] = (50.0, 90.0),
    progress_callback=None,
    overlap_workers: int = 1,
    corner_driven: bool = False,
    corner_tolerance_deg: float = 18.0,
    allow_gapped_rectangle_edges: bool = False,
) -> List[Tuple[SearchState, Dict[str, float]]]:
    """Generate a small ranked set of strictly legal Mode-2 candidates.

    Each newly attached seam edge can be used only once. A longer edge on the
    placed assembly may still accept multiple shorter seam segments.
    Equivalent placements from different construction orders are merged.
    """
    if not 2 <= len(polygons) <= 4:
        raise ValueError("Mode 2 supports 2 to 4 pieces")
    polygons = [
        np.asarray(polygon, dtype=np.float32)
        for polygon in polygons
    ]
    lengths = [edge_lengths(polygon) for polygon in polygons]
    angles = [interior_angles(polygon) for polygon in polygons]
    polygon_areas = [polygon_area(polygon) for polygon in polygons]
    partition_offsets = (
        build_partition_edge_offsets(lengths, edge_tolerance)
        if allow_partitioned_edges
        else {}
    )
    anchor = int(np.argmax(polygon_areas))
    identity = (
        np.eye(2, dtype=np.float32),
        np.zeros(2, dtype=np.float32),
    )
    states = [
        SearchState(
            transforms={anchor: identity},
            used_edges=frozenset(),
            match_error=0.0,
            matches=(),
        )
    ]
    started = time.time()
    total_stages = len(polygons) - 1

    for stage_index in range(total_stages):
        deduplicated = {}
        stage_candidates = []
        rejection_counts = {
            "used": 0,
            "length": 0,
            "overlap": 0,
            "bounds": 0,
        }
        state_count = max(1, len(states))
        state_batches = []
        alignment_jobs = []
        alignment_owners = []
        for state_index, state in enumerate(states):
            elapsed = time.time() - started
            if elapsed > time_limit_seconds:
                raise RuntimeError(
                    "Mode 2 search timeout after {:.1f}s".format(elapsed)
                )
            if progress_callback is not None:
                progress_callback(
                    min(
                        0.82,
                        (
                            stage_index
                            + state_index / state_count
                        )
                        / max(1, total_stages)
                        * 0.82,
                    ),
                    "fast edge stage {}/{}".format(
                        stage_index + 1,
                        total_stages,
                    ),
                )

            placed_world = state_polygons(polygons, state)
            unplaced = [
                piece_id
                for piece_id in range(len(polygons))
                if piece_id not in state.transforms
            ]
            pending = []
            for moving_id in unplaced:
                for fixed_id, fixed_polygon in placed_world.items():
                    for fixed_edge, fixed_length in enumerate(
                        lengths[fixed_id]
                    ):
                        fixed_key = (fixed_id, fixed_edge)
                        for moving_edge, moving_length in enumerate(
                            lengths[moving_id]
                        ):
                            moving_key = (moving_id, moving_edge)
                            if moving_key in state.used_edges:
                                rejection_counts["used"] += 1
                                continue
                            relative_error = abs(
                                float(fixed_length - moving_length)
                            ) / max(
                                float(fixed_length),
                                float(moving_length),
                                1e-6,
                            )
                            if fixed_length >= moving_length:
                                partition_key = (
                                    fixed_key,
                                    moving_key,
                                )
                            else:
                                partition_key = (
                                    moving_key,
                                    fixed_key,
                                )
                            is_full_match = (
                                relative_error <= edge_tolerance
                            )
                            is_partition_match = (
                                partition_key in partition_offsets
                            )
                            if not is_full_match and not is_partition_match:
                                rejection_counts["length"] += 1
                                continue

                            if is_full_match:
                                for kind in full_alignment_kinds(
                                    moving_length,
                                    fixed_length,
                                ):
                                    corner_error = None
                                    if corner_driven:
                                        corner_error = corner_alignment_error_deg(
                                            angles[fixed_id],
                                            fixed_edge,
                                            angles[moving_id],
                                            moving_edge,
                                            kind,
                                            fixed_length=fixed_length,
                                            moving_length=moving_length,
                                        )
                                        if (
                                            corner_error is None
                                            or corner_error
                                            > float(corner_tolerance_deg)
                                        ):
                                            continue
                                    alignment_jobs.append(
                                        (
                                            moving_id,
                                            moving_edge,
                                            fixed_polygon,
                                            fixed_edge,
                                            kind,
                                            0.0,
                                        )
                                    )
                                    alignment_owners.append(
                                        (
                                            len(state_batches),
                                            moving_id,
                                            fixed_id,
                                            fixed_edge,
                                            moving_edge,
                                            moving_key,
                                            moving_length,
                                            fixed_length,
                                            relative_error,
                                            is_full_match,
                                            corner_error,
                                        )
                                    )
                            if is_partition_match:
                                for offset in unique_partition_offsets(
                                    partition_offsets[partition_key]
                                ):
                                    corner_error = None
                                    if corner_driven:
                                        corner_error = corner_alignment_error_deg(
                                            angles[fixed_id],
                                            fixed_edge,
                                            angles[moving_id],
                                            moving_edge,
                                            3,
                                            partition_offset=offset,
                                            fixed_length=fixed_length,
                                            moving_length=moving_length,
                                        )
                                        if (
                                            corner_error is None
                                            or corner_error
                                            > float(corner_tolerance_deg)
                                        ):
                                            continue
                                    alignment_jobs.append(
                                        (
                                            moving_id,
                                            moving_edge,
                                            fixed_polygon,
                                            fixed_edge,
                                            3,
                                            offset,
                                        )
                                    )
                                    alignment_owners.append(
                                        (
                                            len(state_batches),
                                            moving_id,
                                            fixed_id,
                                            fixed_edge,
                                            moving_edge,
                                            moving_key,
                                            moving_length,
                                            fixed_length,
                                            relative_error,
                                            is_full_match,
                                            corner_error,
                                        )
                                    )

            state_batches.append(
                (state, tuple(placed_world.values()), pending)
            )

        alignment_results = batch_edge_alignment_world(
            polygons,
            alignment_jobs,
            workers=overlap_workers,
        )
        for owner, (transform, moving_world) in zip(
            alignment_owners,
            alignment_results,
        ):
            (
                batch_index,
                moving_id,
                fixed_id,
                fixed_edge,
                moving_edge,
                moving_key,
                moving_length,
                fixed_length,
                relative_error,
                is_full_match,
                corner_error,
            ) = owner
            state_batches[batch_index][2].append(
                (
                    moving_id,
                    fixed_id,
                    fixed_edge,
                    moving_edge,
                    moving_key,
                    moving_length,
                    fixed_length,
                    relative_error,
                    is_full_match,
                    corner_error,
                    transform,
                    moving_world,
                )
            )

        overlap_first = []
        overlap_second = []
        overlap_owners = []
        for batch_index, (_, placed_values, pending) in enumerate(
            state_batches
        ):
            for pending_index, item in enumerate(pending):
                moving_world = item[11]
                for other in placed_values:
                    overlap_first.append(moving_world)
                    overlap_second.append(other)
                    overlap_owners.append((batch_index, pending_index))
        overlap_areas = batch_convex_overlap_areas(
            overlap_first,
            overlap_second,
            workers=overlap_workers,
        )
        overlaps = [
            np.zeros(len(pending), dtype=np.bool_)
            for _, _, pending in state_batches
        ]
        for area, (batch_index, pending_index) in zip(
            overlap_areas,
            overlap_owners,
        ):
            moving_id = state_batches[batch_index][2][pending_index][0]
            if area > polygon_areas[moving_id] * 0.025:
                overlaps[batch_index][pending_index] = True

        for batch_index, batch in enumerate(state_batches):
            state, placed_values, pending = batch
            for pending_index, item in enumerate(pending):
                if overlaps[batch_index][pending_index]:
                    rejection_counts["overlap"] += 1
                    continue
                (
                    moving_id,
                    fixed_id,
                    fixed_edge,
                    moving_edge,
                    moving_key,
                    moving_length,
                    fixed_length,
                    relative_error,
                    is_full_match,
                    corner_error,
                    transform,
                    moving_world,
                ) = item
                transforms = dict(state.transforms)
                transforms[moving_id] = transform
                used_edges = set(state.used_edges)
                if is_full_match or moving_length <= fixed_length:
                    used_edges.add(moving_key)
                candidate = SearchState(
                    transforms=transforms,
                    used_edges=frozenset(used_edges),
                    match_error=(
                        state.match_error
                        + (
                            relative_error * 0.18
                            if is_full_match
                            else 0.025
                        )
                    ),
                    matches=state.matches
                    + (
                        (
                            fixed_id,
                            fixed_edge,
                            moving_id,
                            moving_edge,
                            relative_error,
                        ),
                    ),
                )
                stage_candidates.append(candidate)

        if stage_candidates:
            _, _, feasible, signatures = batch_beam_state_metrics(
                polygons,
                stage_candidates,
                polygon_areas=polygon_areas,
                angle_step_deg=1.0,
                translation_step_mm=0.75,
                workers=overlap_workers,
            )
            for candidate_id, candidate in enumerate(stage_candidates):
                if not feasible[candidate_id]:
                    rejection_counts["bounds"] += 1
                    continue
                signature = signatures[candidate_id]
                previous = deduplicated.get(signature)
                if (
                    previous is None
                    or candidate.match_error < previous.match_error
                ):
                    deduplicated[signature] = candidate

        if not deduplicated:
            raise RuntimeError(
                "Mode 2 has no feasible edge candidate at stage {}; "
                "rejected={}".format(
                    stage_index + 1,
                    rejection_counts,
                )
            )
        expanded = list(deduplicated.values())
        # Native feasibility/signature generation removes the large batch.
        # Keep the authoritative OpenCV score for the much smaller deduped
        # set so Mode 2/3 Beam ordering remains byte-for-byte compatible with
        # the pre-acceleration implementation.
        expanded_scores = [
            cheap_unknown_score(
                polygons,
                candidate,
                polygon_areas=polygon_areas,
            )
            for candidate in expanded
        ]
        states = select_diverse_unknown_states(
            polygons,
            expanded,
            beam_width,
            polygon_areas=polygon_areas,
            precomputed_scores=expanded_scores,
        )

    if progress_callback is not None:
        progress_callback(0.86, "validate legal rectangles")

    final_cheap_scores = [
        cheap_unknown_score(
            polygons,
            state,
            polygon_areas=polygon_areas,
        )
        for state in states
    ]
    states = [
        state
        for _, state in sorted(
            zip(final_cheap_scores, states),
            key=lambda item: item[0],
        )
    ]
    validated = []
    validation_rejections = {
        "size": 0,
        "outer": 0,
        "corner": 0,
        "connected": 0,
        "fill": 0,
    }
    closest_metrics = None
    validation_limit = min(len(states), max(40, maximum_results * 3))
    for state_id, state in enumerate(states[:validation_limit]):
        if time.time() - started > time_limit_seconds:
            break
        if progress_callback is not None:
            progress_callback(
                0.86 + 0.13 * state_id / max(1, validation_limit),
                "validate rectangle {}/{}".format(
                    state_id + 1,
                    validation_limit,
                ),
            )
        score, metrics = unknown_rectangle_score(
            polygons,
            state,
            long_side_range_mm=long_side_range_mm,
            short_side_range_mm=short_side_range_mm,
        )
        if (
            closest_metrics is None
            or float(score) < float(closest_metrics["score"])
        ):
            closest_metrics = dict(metrics)
            closest_metrics["score"] = float(score)
        rejected = False
        if metrics["dimension_error"] > 0.04:
            validation_rejections["size"] += 1
            rejected = True
        gapped_rectangle_fallback = bool(
            allow_gapped_rectangle_edges
            and gapped_rectangle_edge_fallback_allowed(metrics)
        )
        strict_rectangle = metrics["outer_vertices"] == 4.0
        measured_near_rectangle = (
            metrics["outer_vertices"] <= float(maximum_outer_vertices)
            and metrics["dimension_error"] <= 0.04
            and metrics["fill_ratio"] >= float(minimum_measured_fill_ratio)
            and metrics["connected_ratio"] >= 0.98
            and metrics["match_error"] <= 0.08
        )
        if (
            not strict_rectangle
            and not measured_near_rectangle
            and not gapped_rectangle_fallback
        ):
            validation_rejections["outer"] += 1
            rejected = True
        strong_rectangle_angle_fallback = (
            strict_rectangle
            and strong_rectangle_angle_fallback_allowed(
                metrics,
                strong_rectangle_outer_corner_error_deg,
                maximum_outer_corner_offset_mm,
            )
        )
        strict_corner_angle = (
            metrics["outer_corner_max_error_deg"]
            <= maximum_outer_corner_error_deg
        )
        if (
            not strict_corner_angle
            and not strong_rectangle_angle_fallback
            and not gapped_rectangle_fallback
        ) or (
            metrics["outer_corner_max_offset_mm"]
            > maximum_outer_corner_offset_mm
            and not gapped_rectangle_fallback
        ):
            validation_rejections["corner"] += 1
            rejected = True
        if metrics["connected_ratio"] < 0.98:
            validation_rejections["connected"] += 1
            rejected = True
        if not 0.86 <= metrics["fill_ratio"] <= 1.08:
            validation_rejections["fill"] += 1
            rejected = True
        if rejected:
            continue
        metrics["score"] = float(score)
        metrics["strategy"] = (
            "fast_gapped_rectangle_edge_search"
            if gapped_rectangle_fallback
            else "fast_unique_edge_search"
            if strict_rectangle
            else "fast_measured_rectangle_search"
        )
        metrics["outer_vertices_relaxed"] = float(
            not strict_rectangle
        )
        metrics["outer_corner_angle_tolerance_deg"] = float(
            strong_rectangle_outer_corner_error_deg
            if strong_rectangle_angle_fallback
            else maximum_outer_corner_error_deg
        )
        metrics["outer_corner_strict_tolerance_deg"] = float(
            maximum_outer_corner_error_deg
        )
        metrics["strong_rectangle_angle_fallback"] = bool(
            strong_rectangle_angle_fallback
        )
        metrics["gapped_rectangle_edge_fallback"] = bool(
            gapped_rectangle_fallback
        )
        metrics["outer_corner_offset_tolerance_mm"] = float(
            maximum_outer_corner_offset_mm
        )
        metrics["partitioned_edges"] = float(allow_partitioned_edges)
        metrics["corner_driven"] = float(corner_driven)
        metrics["corner_tolerance_deg"] = float(corner_tolerance_deg)
        metrics["candidate_count"] = float(len(states))
        validated.append((state, metrics))

    if not validated:
        if closest_metrics is None:
            closest_text = "none"
        else:
            closest_text = (
                "{:.1f}x{:.1f}mm fill={:.3f} outer={} "
                "connected={:.3f} dim_error={:.3f} "
                "corner_error={:.1f}deg corner_offset={:.1f}mm"
            ).format(
                closest_metrics["long_side_mm"],
                closest_metrics["short_side_mm"],
                closest_metrics["fill_ratio"],
                int(closest_metrics["outer_vertices"]),
                closest_metrics["connected_ratio"],
                closest_metrics["dimension_error"],
                closest_metrics["outer_corner_max_error_deg"],
                closest_metrics["outer_corner_max_offset_mm"],
            )
        raise RuntimeError(
            "Mode 2 found no legal rectangular candidate; "
            "checked={} rejected={} closest={}".format(
                validation_limit,
                validation_rejections,
                closest_text,
            )
        )
    validated.sort(key=lambda item: item[1]["score"])
    if progress_callback is not None:
        progress_callback(1.0, "fast edge search complete")
    return validated[:maximum_results]


def interior_angles(polygon: np.ndarray) -> np.ndarray:
    angles = []
    for index, vertex in enumerate(polygon):
        previous_vector = polygon[index - 1] - vertex
        next_vector = polygon[(index + 1) % len(polygon)] - vertex
        denominator = max(
            1e-8,
            float(np.linalg.norm(previous_vector) * np.linalg.norm(next_vector)),
        )
        cosine_value = np.clip(
            float(np.dot(previous_vector, next_vector)) / denominator,
            -1.0,
            1.0,
        )
        angles.append(degrees(np.arccos(cosine_value)))
    return np.asarray(angles, dtype=np.float32)


def rectangle_corner_definitions(
    target_size_mm: Tuple[float, float],
):
    width, height = target_size_mm
    return [
        (
            np.asarray([0.0, 0.0], dtype=np.float32),
            np.asarray([1.0, 0.0], dtype=np.float32),
            np.asarray([0.0, 1.0], dtype=np.float32),
        ),
        (
            np.asarray([width, 0.0], dtype=np.float32),
            np.asarray([-1.0, 0.0], dtype=np.float32),
            np.asarray([0.0, 1.0], dtype=np.float32),
        ),
        (
            np.asarray([width, height], dtype=np.float32),
            np.asarray([-1.0, 0.0], dtype=np.float32),
            np.asarray([0.0, -1.0], dtype=np.float32),
        ),
        (
            np.asarray([0.0, height], dtype=np.float32),
            np.asarray([1.0, 0.0], dtype=np.float32),
            np.asarray([0.0, -1.0], dtype=np.float32),
        ),
    ]


def rotation_fitting_two_directions(
    source_a: np.ndarray,
    source_b: np.ndarray,
    target_a: np.ndarray,
    target_b: np.ndarray,
) -> Tuple[np.ndarray, float]:
    source_a = source_a / max(1e-8, float(np.linalg.norm(source_a)))
    source_b = source_b / max(1e-8, float(np.linalg.norm(source_b)))
    target_a = target_a / max(1e-8, float(np.linalg.norm(target_a)))
    target_b = target_b / max(1e-8, float(np.linalg.norm(target_b)))
    cross_sum = (
        source_a[0] * target_a[1] - source_a[1] * target_a[0]
        + source_b[0] * target_b[1]
        - source_b[1] * target_b[0]
    )
    dot_sum = float(np.dot(source_a, target_a) + np.dot(source_b, target_b))
    angle = atan2(float(cross_sum), dot_sum)
    rotation = np.asarray(
        [[cos(angle), -sin(angle)], [sin(angle), cos(angle)]],
        dtype=np.float32,
    )
    fitted_a = rotation @ source_a
    fitted_b = rotation @ source_b
    residual = (
        degrees(np.arccos(np.clip(np.dot(fitted_a, target_a), -1.0, 1.0)))
        + degrees(np.arccos(np.clip(np.dot(fitted_b, target_b), -1.0, 1.0)))
    ) * 0.5
    return rotation, residual


def corner_options_for_piece(
    polygon: np.ndarray,
    piece_id: int,
    target_size_mm: Tuple[float, float],
    angle_tolerance_deg: float = 18.0,
    outside_tolerance_mm: float = 6.0,
):
    options = []
    angles = interior_angles(polygon)
    target_width, target_height = target_size_mm
    for vertex_id, angle in enumerate(angles):
        if abs(float(angle) - 90.0) > angle_tolerance_deg:
            continue
        vertex = polygon[vertex_id]
        previous_vector = polygon[vertex_id - 1] - vertex
        next_vector = polygon[(vertex_id + 1) % len(polygon)] - vertex
        for corner_id, (corner, axis_a, axis_b) in enumerate(
            rectangle_corner_definitions(target_size_mm)
        ):
            for first_axis, second_axis in (
                (axis_a, axis_b),
                (axis_b, axis_a),
            ):
                rotation, residual = rotation_fitting_two_directions(
                    previous_vector,
                    next_vector,
                    first_axis,
                    second_axis,
                )
                if residual > angle_tolerance_deg:
                    continue
                translation = corner - rotation @ vertex
                transformed = apply_transform(
                    polygon,
                    (rotation, translation),
                )
                minimum = np.min(transformed, axis=0)
                maximum = np.max(transformed, axis=0)
                inside = (
                    minimum[0] >= -outside_tolerance_mm
                    and minimum[1] >= -outside_tolerance_mm
                    and maximum[0] <= target_width + outside_tolerance_mm
                    and maximum[1] <= target_height + outside_tolerance_mm
                )
                if not inside:
                    continue
                options.append(
                    {
                        "piece_id": piece_id,
                        "vertex_id": vertex_id,
                        "corner_id": corner_id,
                        "transform": (rotation, translation),
                        "angle_error": abs(float(angle) - 90.0) / 90.0,
                    }
                )
    return options


def transforms_have_overlap(
    polygons: Sequence[np.ndarray],
    transforms: Dict[int, Tuple[np.ndarray, np.ndarray]],
) -> bool:
    placed = {
        piece_id: apply_transform(polygons[piece_id], transform)
        for piece_id, transform in transforms.items()
    }
    for first_id, second_id in combinations(placed.keys(), 2):
        overlap = convex_overlap_area(placed[first_id], placed[second_id])
        smaller_area = min(
            polygon_area(placed[first_id]),
            polygon_area(placed[second_id]),
        )
        if overlap > smaller_area * 0.025:
            return True
    return False


def solve_corner_first(
    polygons: Sequence[np.ndarray],
    target_size_mm: Tuple[float, float],
) -> Tuple[SearchState, Dict[str, float]]:
    polygons = [np.asarray(polygon, dtype=np.float32) for polygon in polygons]
    options_by_piece = {
        piece_id: corner_options_for_piece(
            polygon,
            piece_id,
            target_size_mm,
        )
        for piece_id, polygon in enumerate(polygons)
    }
    smallest_piece = int(
        np.argmin([polygon_area(polygon) for polygon in polygons])
    )
    anchor_options = options_by_piece[smallest_piece]
    if not anchor_options:
        raise RuntimeError("最小碎片没有可靠的直角候选")

    states = []
    for option in anchor_options:
        states.append(
            (
                {smallest_piece: option["transform"]},
                {option["corner_id"]},
                option["angle_error"],
            )
        )

    while len(states[0][0]) < min(3, len(polygons)):
        expanded = []
        for transforms, occupied_corners, angle_error in states:
            for piece_id in range(len(polygons)):
                if piece_id in transforms:
                    continue
                for option in options_by_piece[piece_id]:
                    if option["corner_id"] in occupied_corners:
                        continue
                    next_transforms = dict(transforms)
                    next_transforms[piece_id] = option["transform"]
                    if transforms_have_overlap(polygons, next_transforms):
                        continue
                    expanded.append(
                        (
                            next_transforms,
                            occupied_corners | {option["corner_id"]},
                            angle_error + option["angle_error"],
                        )
                    )
        if not expanded:
            raise RuntimeError("无法由三个直角建立矩形框架")
        expanded.sort(
            key=lambda item: item[2]
            + partial_score(
                polygons,
                SearchState(
                    transforms=item[0],
                    used_edges=frozenset(),
                    match_error=item[2],
                    matches=(),
                ),
            )
        )
        states = expanded[:500]

    full_states = []
    target_width, target_height = target_size_mm
    for transforms, occupied_corners, angle_error in states:
        remaining = [
            piece_id
            for piece_id in range(len(polygons))
            if piece_id not in transforms
        ]
        if not remaining:
            full_states.append(
                SearchState(
                    transforms=transforms,
                    used_edges=frozenset(),
                    match_error=angle_error,
                    matches=(),
                )
            )
            continue

        moving_id = remaining[0]
        candidate_transforms = []
        for option in options_by_piece[moving_id]:
            if option["corner_id"] not in occupied_corners:
                candidate_transforms.append(
                    (option["transform"], option["angle_error"], ())
                )

        placed_world = {
            piece_id: apply_transform(polygons[piece_id], transform)
            for piece_id, transform in transforms.items()
        }
        moving_lengths = edge_lengths(polygons[moving_id])
        for fixed_id, fixed_polygon in placed_world.items():
            fixed_lengths = edge_lengths(polygons[fixed_id])
            for fixed_edge, fixed_length in enumerate(fixed_lengths):
                for moving_edge, moving_length in enumerate(moving_lengths):
                    relative_error = abs(
                        float(fixed_length - moving_length)
                    ) / max(float(fixed_length), float(moving_length))
                    for transform in edge_alignment_transforms(
                        polygons[moving_id],
                        moving_edge,
                        fixed_polygon,
                        fixed_edge,
                    ):
                        candidate_transforms.append(
                            (
                                transform,
                                relative_error * 0.15,
                                (
                                    (
                                        fixed_id,
                                        fixed_edge,
                                        moving_id,
                                        moving_edge,
                                        relative_error,
                                    ),
                                ),
                            )
                        )

        for transform, extra_error, matches in candidate_transforms:
            next_transforms = dict(transforms)
            next_transforms[moving_id] = transform
            if transforms_have_overlap(polygons, next_transforms):
                continue
            moving_world = apply_transform(polygons[moving_id], transform)
            minimum = np.min(moving_world, axis=0)
            maximum = np.max(moving_world, axis=0)
            if (
                minimum[0] < -6.0
                or minimum[1] < -6.0
                or maximum[0] > target_width + 6.0
                or maximum[1] > target_height + 6.0
            ):
                continue
            full_states.append(
                SearchState(
                    transforms=next_transforms,
                    used_edges=frozenset(),
                    match_error=angle_error + extra_error,
                    matches=matches,
                )
            )

    if not full_states:
        raise RuntimeError("三个直角确定后无法放入剩余碎片")
    scored = [
        (final_score(polygons, state, target_size_mm), state)
        for state in full_states
    ]
    (score, metrics), best_state = min(scored, key=lambda item: item[0][0])
    metrics["score"] = score
    metrics["strategy"] = "corner_first"
    return best_state, metrics


def solve_polygons(
    polygons: Sequence[np.ndarray],
    target_size_mm: Tuple[float, float] = (100.0, 60.0),
    edge_tolerance: float = 0.85,
    beam_width: int = 2000,
) -> Tuple[SearchState, Dict[str, float]]:
    """优先使用三个直角求解，评分不合格时回退通用边搜索。"""
    try:
        corner_state, corner_metrics = solve_corner_first(
            polygons,
            target_size_mm,
        )
        if (
            corner_metrics["outer_vertices"] == 4.0
            and corner_metrics["fill_ratio"] >= 0.85
            and corner_metrics["score"] <= 0.9
        ):
            return corner_state, corner_metrics
    except RuntimeError:
        corner_state = None
        corner_metrics = None

    edge_state, edge_metrics = solve_edge_search(
        polygons,
        target_size_mm,
        edge_tolerance,
        beam_width,
    )
    edge_metrics["strategy"] = "edge_fallback"
    if (
        corner_state is not None
        and corner_metrics is not None
        and corner_metrics["score"] < edge_metrics["score"]
    ):
        return corner_state, corner_metrics
    return edge_state, edge_metrics


def best_rigid_fit(
    source: np.ndarray,
    target: np.ndarray,
) -> Tuple[Tuple[np.ndarray, np.ndarray], float]:
    """尝试循环顶点编号和方向，返回无镜像刚体拟合及RMS误差。"""
    if len(source) != len(target):
        raise ValueError("源多边形与模板顶点数不同")
    source = np.asarray(source, dtype=np.float32)
    target = np.asarray(target, dtype=np.float32)
    best_transform = None
    best_error = float("inf")
    for candidate in (source, source[::-1]):
        for shift in range(len(source)):
            shifted = np.roll(candidate, shift, axis=0)
            source_center = np.mean(shifted, axis=0)
            target_center = np.mean(target, axis=0)
            source_zero = shifted - source_center
            target_zero = target - target_center
            covariance = source_zero.T @ target_zero
            left, _, right = np.linalg.svd(covariance)
            rotation = right.T @ left.T
            if np.linalg.det(rotation) < 0:
                right[-1, :] *= -1
                rotation = right.T @ left.T
            translation = target_center - rotation @ source_center
            fitted = apply_transform(shifted, (rotation, translation))
            error = float(
                np.sqrt(np.mean(np.sum((fitted - target) ** 2, axis=1)))
            )
            if error < best_error:
                best_error = error
                best_transform = (rotation.astype(np.float32), translation)
    return best_transform, best_error


def sample_polygon_boundary(
    polygon: np.ndarray,
    sample_count: int = 64,
) -> np.ndarray:
    """Uniformly sample a closed polygon boundary.

    This makes template fitting tolerant of an extra or missing collinear
    corner produced by contour approximation.
    """
    polygon = np.asarray(polygon, dtype=np.float32)
    following = np.roll(polygon, -1, axis=0)
    vectors = following - polygon
    lengths = np.linalg.norm(vectors, axis=1)
    perimeter = float(np.sum(lengths))
    if len(polygon) < 3 or perimeter < 1e-6:
        raise ValueError("polygon boundary is invalid")

    cumulative = np.concatenate(
        (
            np.asarray([0.0], dtype=np.float32),
            np.cumsum(lengths, dtype=np.float32),
        )
    )
    distances = (
        np.arange(sample_count, dtype=np.float32)
        * (perimeter / float(sample_count))
    )
    sampled = []
    edge_id = 0
    for distance in distances:
        while (
            edge_id + 1 < len(polygon)
            and distance >= cumulative[edge_id + 1]
        ):
            edge_id += 1
        edge_length = max(float(lengths[edge_id]), 1e-6)
        ratio = (
            float(distance) - float(cumulative[edge_id])
        ) / edge_length
        sampled.append(
            polygon[edge_id] + vectors[edge_id] * ratio
        )
    return np.asarray(sampled, dtype=np.float32)


def best_template_fit(
    source: np.ndarray,
    target: np.ndarray,
) -> Tuple[Tuple[np.ndarray, np.ndarray], float]:
    """Fit exact corners when possible, otherwise fit sampled boundaries."""
    if len(source) == len(target):
        return best_rigid_fit(source, target)
    return best_rigid_fit(
        sample_polygon_boundary(source),
        sample_polygon_boundary(target),
    )


def solve_with_template(
    polygons: Sequence[np.ndarray],
    template_polygons: Sequence[np.ndarray],
    maximum_rms_mm: float = 6.0,
) -> Tuple[SearchState, Dict[str, float]]:
    """将同一套固定碎片按形状匹配到已验证的目标模板。"""
    polygons = [np.asarray(polygon, dtype=np.float32) for polygon in polygons]
    templates = [
        np.asarray(polygon, dtype=np.float32)
        for polygon in template_polygons
    ]
    if len(polygons) != len(templates):
        raise RuntimeError("当前碎片数量与固定模板不同")

    pair_fits = {}
    for piece_id, polygon in enumerate(polygons):
        for template_id, template in enumerate(templates):
            if abs(len(polygon) - len(template)) > 1:
                continue
            pair_fits[(piece_id, template_id)] = best_template_fit(
                polygon,
                template,
            )

    best_assignment = None
    best_total = float("inf")
    for assignment in permutations(range(len(templates))):
        if any(
            (piece_id, template_id) not in pair_fits
            for piece_id, template_id in enumerate(assignment)
        ):
            continue
        errors = [
            pair_fits[(piece_id, template_id)][1]
            for piece_id, template_id in enumerate(assignment)
        ]
        total = sum(errors)
        if total < best_total:
            best_total = total
            best_assignment = (assignment, errors)
    if best_assignment is None:
        raise RuntimeError("没有找到与固定模板顶点数一致的分配")

    assignment, errors = best_assignment
    if max(errors) > maximum_rms_mm:
        raise RuntimeError(
            "固定模板匹配误差过大：最大RMS {:.2f} mm".format(max(errors))
        )
    transforms = {
        piece_id: pair_fits[(piece_id, template_id)][0]
        for piece_id, template_id in enumerate(assignment)
    }
    state = SearchState(
        transforms=transforms,
        used_edges=frozenset(),
        match_error=float(np.mean(errors) / 100.0),
        matches=(),
    )
    target_size = (100.0, 60.0)
    assigned_templates = [
        templates[template_id] for template_id in assignment
    ]
    identity = (
        np.eye(2, dtype=np.float32),
        np.zeros(2, dtype=np.float32),
    )
    template_state = SearchState(
        transforms={
            piece_id: identity for piece_id in range(len(assigned_templates))
        },
        used_edges=frozenset(),
        match_error=0.0,
        matches=(),
    )
    _, metrics = final_score(
        assigned_templates,
        template_state,
        target_size,
    )
    metrics["strategy"] = "fixed_template"
    metrics["template_mean_rms_mm"] = float(np.mean(errors))
    metrics["template_max_rms_mm"] = float(max(errors))
    metrics["template_assignment"] = [
        float(template_id) for template_id in assignment
    ]
    return state, metrics


def normalize_solution(
    polygons: Sequence[np.ndarray],
    state: SearchState,
    target_origin_mm: Tuple[float, float] = (55.0, 30.0),
) -> Tuple[List[np.ndarray], List[float]]:
    """将求解结果旋转为横向矩形并移动到A4下半区。"""
    placed_map = state_polygons(polygons, state)
    all_points = np.concatenate(list(placed_map.values()), axis=0).astype(np.float32)
    rectangle = cv2.minAreaRect(all_points)
    box = cv2.boxPoints(rectangle)
    vectors = np.roll(box, -1, axis=0) - box
    lengths = np.linalg.norm(vectors, axis=1)
    long_vector = vectors[int(np.argmax(lengths))]
    correction_angle = -atan2(long_vector[1], long_vector[0])
    correction = np.asarray(
        [
            [cos(correction_angle), -sin(correction_angle)],
            [sin(correction_angle), cos(correction_angle)],
        ],
        dtype=np.float32,
    )

    corrected = {
        piece_id: polygon @ correction.T
        for piece_id, polygon in placed_map.items()
    }
    corrected_all = np.concatenate(list(corrected.values()), axis=0)
    minimum = np.min(corrected_all, axis=0)
    offset = np.asarray(target_origin_mm, dtype=np.float32) - minimum

    targets = []
    rotations = []
    for piece_id in range(len(polygons)):
        targets.append(corrected[piece_id] + offset)
        original_rotation = state.transforms[piece_id][0]
        total_rotation = correction @ original_rotation
        rotations.append(
            degrees(atan2(total_rotation[1, 0], total_rotation[0, 0]))
        )
    return targets, rotations
