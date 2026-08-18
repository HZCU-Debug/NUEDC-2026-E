"""Mode 4 rectangular playing-card reconstruction and pixel validation."""

from __future__ import annotations

import os
from itertools import permutations
from math import degrees
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

try:
    if os.environ.get("VISION_DISABLE_NATIVE", "").strip() == "1":
        raise ImportError("native acceleration disabled")
    import vision_fast as _vision_fast
except ImportError:
    _vision_fast = None


MODE4_RECTANGLE_ANGLE_TOLERANCE_DEG = 12.0
MODE4_RECTANGLE_MIN_FILL_RATIO = 0.88
MODE4_RENDER_PIXELS_PER_MM = 4.0


def _edge_lengths(quad: np.ndarray) -> np.ndarray:
    quad = np.asarray(quad, dtype=np.float32).reshape(4, 2)
    return np.linalg.norm(np.roll(quad, -1, axis=0) - quad, axis=1)


def _rotation_between(first: np.ndarray, second: np.ndarray) -> float:
    first_vector = np.asarray(first[1] - first[0], dtype=np.float32)
    second_vector = np.asarray(second[1] - second[0], dtype=np.float32)
    first_angle = np.degrees(np.arctan2(first_vector[1], first_vector[0]))
    second_angle = np.degrees(np.arctan2(second_vector[1], second_vector[0]))
    return float((second_angle - first_angle + 180.0) % 360.0 - 180.0)


def enumerate_rectangular_slot_assignments(
    source_rectangles: Sequence[np.ndarray],
    target_slots: Sequence[np.ndarray],
    relative_size_tolerance: float = 0.14,
):
    """Enumerate every size-compatible piece-to-slot assignment.

    Geometry search is deliberately label-agnostic here.  For each
    permutation, target vertices are cyclically aligned with the source
    rectangle.  The remaining equivalent 180-degree choice is handled by
    ``half_turn_variants`` during artwork scoring.
    """
    sources = [
        np.asarray(rectangle, dtype=np.float32).reshape(4, 2)
        for rectangle in source_rectangles
    ]
    slots = [
        np.asarray(rectangle, dtype=np.float32).reshape(4, 2)
        for rectangle in target_slots
    ]
    if len(sources) != len(slots):
        return
    source_lengths = [_edge_lengths(source) for source in sources]
    for slot_permutation in permutations(range(len(slots))):
        assigned_targets = []
        assigned_rotations = []
        compatible = True
        for piece_id, slot_id in enumerate(slot_permutation):
            source = sources[piece_id]
            slot = slots[slot_id]
            lengths = source_lengths[piece_id]
            best = None
            # Shifts 0/1 cover the two distinct rectangle orientations;
            # shifts 2/3 are their later 180-degree variants.
            for shift in (0, 1):
                aligned = np.roll(slot, -shift, axis=0)
                target_lengths = _edge_lengths(aligned)
                relative_error = float(
                    np.max(
                        np.abs(target_lengths - lengths)
                        / np.maximum(1e-6, lengths)
                    )
                )
                if best is None or relative_error < best[0]:
                    best = (relative_error, aligned)
            if best is None or best[0] > relative_size_tolerance:
                compatible = False
                break
            aligned = best[1].astype(np.float32)
            assigned_targets.append(aligned)
            assigned_rotations.append(_rotation_between(source, aligned))
        if compatible:
            yield (
                assigned_targets,
                assigned_rotations,
                tuple(int(value) for value in slot_permutation),
            )


def polygon_inner_angles(points: np.ndarray) -> List[float]:
    points = np.asarray(points, dtype=np.float32).reshape(-1, 2)
    angles = []
    for vertex_id, vertex in enumerate(points):
        previous = points[vertex_id - 1] - vertex
        following = points[(vertex_id + 1) % len(points)] - vertex
        denominator = max(
            1e-8,
            float(np.linalg.norm(previous) * np.linalg.norm(following)),
        )
        cosine_value = np.clip(
            float(np.dot(previous, following)) / denominator,
            -1.0,
            1.0,
        )
        angles.append(float(degrees(np.arccos(cosine_value))))
    return angles


def regularize_rectangular_pieces(
    pieces: List[Dict[str, object]],
    pixels_per_mm: float = MODE4_RENDER_PIXELS_PER_MM,
    angle_tolerance_deg: float = MODE4_RECTANGLE_ANGLE_TOLERANCE_DEG,
) -> Tuple[List[np.ndarray], List[Dict[str, object]]]:
    """Recover ink-notched card outlines, validate them, then fit boxes.

    Dark artwork can touch a cut edge and split the white-card mask, leaving
    artificial inward notches in ``_polygon``.  The physical competition
    pieces are rectangular, so validate the convex outer envelope instead of
    treating those ink notches as real corners.
    """
    rectangles_mm = []
    diagnostics = []
    for piece in pieces:
        measured = np.asarray(piece["_polygon"], dtype=np.float32).reshape(-1, 2)
        hull = cv2.convexHull(
            np.asarray(piece["_contour"], dtype=np.float32)
        )
        hull_perimeter = max(1.0, float(cv2.arcLength(hull, True)))
        if len(measured) == 4:
            validation_quad = measured
        else:
            validation_quad = cv2.approxPolyDP(
                hull,
                0.01 * hull_perimeter,
                True,
            ).astype(np.float32).reshape(-1, 2)
        if len(validation_quad) != 4:
            raise RuntimeError(
                "Mode 4 convex outline for P{} is not quadrilateral: "
                "measured={} convex={}"
                .format(piece["id"], len(measured), len(validation_quad))
            )
        angles = polygon_inner_angles(validation_quad)
        maximum_error = max(abs(angle - 90.0) for angle in angles)
        if maximum_error > angle_tolerance_deg:
            raise RuntimeError(
                "Mode 4 P{} is not rectangular: angles={} max_error="
                "{:.2f}deg > {:.2f}deg".format(
                    piece["id"],
                    [round(angle, 2) for angle in angles],
                    maximum_error,
                    angle_tolerance_deg,
                )
            )

        rectangle = cv2.minAreaRect(hull)
        source_box = cv2.boxPoints(rectangle).astype(np.float32)
        box_area = max(1.0, abs(float(cv2.contourArea(source_box))))
        contour_area = abs(float(cv2.contourArea(hull)))
        fill_ratio = contour_area / box_area
        if fill_ratio < MODE4_RECTANGLE_MIN_FILL_RATIO:
            raise RuntimeError(
                "Mode 4 P{} rectangle fill {:.3f} < {:.3f}".format(
                    piece["id"],
                    fill_ratio,
                    MODE4_RECTANGLE_MIN_FILL_RATIO,
                )
            )

        box_int = np.round(source_box).astype(np.int32).reshape(-1, 1, 2)
        piece["_mode4_source_quad_px"] = source_box
        piece["_polygon"] = box_int
        piece["vertices_px"] = box_int.reshape(-1, 2).tolist()
        piece["polygon_candidates_px"] = [piece["vertices_px"]]
        rectangle_mm = source_box / float(pixels_per_mm)
        rectangles_mm.append(rectangle_mm.astype(np.float32))
        diagnostics.append(
            {
                "piece_id": int(piece["id"]),
                "measured_vertices": int(len(measured)),
                "validation_vertices": int(len(validation_quad)),
                "ink_notch_recovered": bool(len(measured) != 4),
                "measured_angles_deg": [round(angle, 3) for angle in angles],
                "maximum_angle_error_deg": round(maximum_error, 3),
                "rectangle_fill_ratio": round(fill_ratio, 4),
            }
        )
    return rectangles_mm, diagnostics


def half_turn_variants(piece_count: int):
    """Yield every independent 180-degree orientation of rectangular pieces."""
    for variant in range(1 << piece_count):
        yield tuple(bool(variant & (1 << piece_id)) for piece_id in range(piece_count))


def reconstruct_card_pixels(
    rectified: np.ndarray,
    pieces: Sequence[Dict[str, object]],
    targets: Sequence[np.ndarray],
    half_turns: Sequence[bool],
    pixels_per_mm: float = MODE4_RENDER_PIXELS_PER_MM,
    return_piece_masks: bool = False,
    warp_cache: Optional[dict] = None,
):
    """Warp source rectangles into one gap-free card canvas."""
    target_arrays = [
        np.asarray(target, dtype=np.float32).reshape(-1, 2)
        for target in targets
    ]
    all_points = np.concatenate(target_arrays, axis=0)
    minimum = np.min(all_points, axis=0)
    maximum = np.max(all_points, axis=0)
    width = max(2, int(np.ceil((maximum[0] - minimum[0]) * pixels_per_mm)) + 1)
    height = max(2, int(np.ceil((maximum[1] - minimum[1]) * pixels_per_mm)) + 1)
    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    valid = np.zeros((height, width), dtype=np.uint8)
    piece_masks = []

    for piece_id, (piece, target) in enumerate(zip(pieces, target_arrays)):
        source_quad = np.asarray(
            piece["_mode4_source_quad_px"],
            dtype=np.float32,
        ).reshape(4, 2)
        destination = (target - minimum) * float(pixels_per_mm)
        if half_turns[piece_id]:
            destination = np.roll(destination, 2, axis=0)
        cache_key = None
        cached = None
        if warp_cache is not None:
            cache_key = (
                int(piece_id),
                int(width),
                int(height),
                tuple(
                    np.round(destination * 1000.0)
                    .astype(np.int32)
                    .reshape(-1)
                    .tolist()
                ),
            )
            cached = warp_cache.get(cache_key)
        if cached is None:
            homography = cv2.getPerspectiveTransform(
                source_quad,
                destination,
            )
            source_mask_key = (
                "source_mask",
                int(piece_id),
                int(rectified.shape[0]),
                int(rectified.shape[1]),
                tuple(
                    np.round(source_quad * 1000.0)
                    .astype(np.int32)
                    .reshape(-1)
                    .tolist()
                ),
            )
            source_mask = (
                warp_cache.get(source_mask_key)
                if warp_cache is not None
                else None
            )
            if source_mask is None:
                source_mask = np.zeros(rectified.shape[:2], dtype=np.uint8)
                cv2.fillConvexPoly(
                    source_mask,
                    np.round(source_quad).astype(np.int32),
                    255,
                )
                source_mask = cv2.erode(
                    source_mask,
                    np.ones((3, 3), dtype=np.uint8),
                )
                if warp_cache is not None:
                    warp_cache[source_mask_key] = source_mask
            warped_image = cv2.warpPerspective(
                rectified,
                homography,
                (width, height),
                flags=cv2.INTER_LINEAR,
                borderValue=(255, 255, 255),
            )
            warped_mask = cv2.warpPerspective(
                source_mask,
                homography,
                (width, height),
                flags=cv2.INTER_NEAREST,
            )
            if warp_cache is not None and cache_key is not None:
                warp_cache[cache_key] = (warped_image, warped_mask)
        else:
            warped_image, warped_mask = cached
        use = warped_mask > 0
        canvas[use] = warped_image[use]
        valid[use] = 255
        piece_masks.append((warped_mask > 0).astype(np.uint8))

    if return_piece_masks:
        return canvas, valid, piece_masks
    return canvas, valid


def printed_ink_mask(image: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
    """Extract black/red playing-card printing while rejecting white paper."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    hue, saturation, value = cv2.split(hsv)
    # Green cardboard and its dark cut-edge fringe must not become "black
    # ink".  True black print is either very dark or dark and weakly
    # saturated; the green background is strongly saturated.
    red = (
        ((hue <= 12) | (hue >= 168))
        & (saturation >= 65)
        & (value >= 55)
    )
    # Specular lighting can lift black ink into a medium grey on individual
    # pieces.  Apply that compensation only to black-suit cards.  On red
    # cards, a globally raised grey threshold also captures dim paper and
    # red-edge shadows, so retain the original conservative threshold there.
    reliable_red = int(np.count_nonzero(red & (valid_mask > 0))) >= 30
    grey_ink_limit = 145 if reliable_red else 195
    black = (value < 100) | (
        (grey < grey_ink_limit) & (saturation < 100)
    )
    mask = ((black | red) & (valid_mask > 0)).astype(np.uint8)
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        np.ones((2, 2), dtype=np.uint8),
    )


def red_print_mask(image: np.ndarray, valid_mask: np.ndarray) -> np.ndarray:
    """Extract saturated red print independently from dark cut-edge shadow."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    red = (
        ((hue <= 12) | (hue >= 168))
        & (saturation >= 65)
        & (value >= 55)
        & (valid_mask > 0)
    )
    return cv2.morphologyEx(
        red.astype(np.uint8),
        cv2.MORPH_OPEN,
        np.ones((2, 2), dtype=np.uint8),
    )


def _dice_overlap(first: np.ndarray, second: np.ndarray) -> float:
    first = first.astype(bool)
    second = second.astype(bool)
    denominator = int(np.count_nonzero(first)) + int(np.count_nonzero(second))
    if denominator == 0:
        return 0.0
    return 2.0 * float(np.count_nonzero(first & second)) / denominator


def _best_shifted_overlap(
    first: np.ndarray,
    second: np.ndarray,
    maximum_shift_px: int = 4,
) -> float:
    """Return the best zero-padded shift overlap without rolling images.

    ``np.roll`` copied the complete mask for every tested displacement.  This
    implementation compares only the intersecting views and accounts for the
    cropped foreground in the Dice denominator, which is exactly equivalent
    to the old zero-padded result.
    """
    first = np.asarray(first, dtype=bool)
    second = np.asarray(second, dtype=bool)
    if first.shape != second.shape or first.ndim != 2:
        raise ValueError("shifted overlap expects equally sized 2-D masks")
    if (
        _vision_fast is not None
        and os.environ.get("VISION_USE_NATIVE_OVERLAP", "").strip() == "1"
    ):
        return float(
            _vision_fast.best_shifted_overlap(
                first,
                second,
                maximum_shift_px=int(maximum_shift_px),
            )
        )

    height, width = first.shape
    first_count = int(np.count_nonzero(first))
    best = 0.0
    for shift_y in range(-maximum_shift_px, maximum_shift_px + 1):
        if shift_y >= 0:
            first_y = slice(shift_y, height)
            second_y = slice(0, height - shift_y)
        else:
            first_y = slice(0, height + shift_y)
            second_y = slice(-shift_y, height)
        for shift_x in range(-maximum_shift_px, maximum_shift_px + 1):
            if shift_x >= 0:
                first_x = slice(shift_x, width)
                second_x = slice(0, width - shift_x)
            else:
                first_x = slice(0, width + shift_x)
                second_x = slice(-shift_x, width)

            second_view = second[second_y, second_x]
            denominator = first_count + int(np.count_nonzero(second_view))
            if denominator == 0:
                continue
            intersection = int(
                np.count_nonzero(first[first_y, first_x] & second_view)
            )
            best = max(best, 2.0 * float(intersection) / denominator)
    return best


def _small_index_component_count(
    corner: np.ndarray,
    minimum_area: int = 25,
    maximum_area: int = 800,
) -> int:
    """Count digit/small-suit sized blobs, excluding a large central pip."""
    corner = cv2.morphologyEx(
        corner.astype(np.uint8),
        cv2.MORPH_CLOSE,
        np.ones((3, 3), dtype=np.uint8),
    )
    component_count, _, statistics, _ = cv2.connectedComponentsWithStats(
        corner,
        connectivity=8,
    )
    areas = statistics[1:, cv2.CC_STAT_AREA]
    return int(
        np.count_nonzero(
            (areas >= int(minimum_area))
            & (areas <= int(maximum_area))
        )
    )


def score_rectangular_card_prefilter(
    rectified: np.ndarray,
    pieces: Sequence[Dict[str, object]],
    targets: Sequence[np.ndarray],
    half_turns: Sequence[bool],
    warp_cache: Optional[dict] = None,
) -> Dict[str, object]:
    """Cheap half-resolution rank/index and long-axis symmetry prefilter.

    This stage deliberately omits colour, rim and seam morphology.  It only
    ranks candidates; the survivors still pass through the full-resolution
    validator below before they can produce a motion plan.
    """
    card, valid = reconstruct_card_pixels(
        rectified,
        pieces,
        targets,
        half_turns,
        pixels_per_mm=MODE4_RENDER_PIXELS_PER_MM * 0.5,
        warp_cache=warp_cache,
    )
    ink = printed_ink_mask(card, valid)
    height, width = ink.shape
    total_ink = int(np.count_nonzero(ink))
    corner_width = max(6, int(round(width * 0.18)))
    corner_height = max(6, int(round(height * 0.20)))
    corners = (
        ink[:corner_height, :corner_width],
        ink[:corner_height, width - corner_width:],
        ink[height - corner_height:, :corner_width],
        ink[height - corner_height:, width - corner_width:],
    )
    corner_counts = [int(np.count_nonzero(corner)) for corner in corners]
    component_counts = [
        _small_index_component_count(
            corner,
            minimum_area=6,
            maximum_area=250,
        )
        for corner in corners
    ]
    first_id, second_id = (1, 2)
    other_ids = (0, 3)
    corner_overlap = _best_shifted_overlap(
        corners[first_id],
        np.flip(corners[second_id], axis=(0, 1)),
        maximum_shift_px=2,
    )
    chosen_ink = corner_counts[first_id] + corner_counts[second_id]
    other_ink = corner_counts[other_ids[0]] + corner_counts[other_ids[1]]
    chosen_components = (
        component_counts[first_id] + component_counts[second_id]
    )
    other_components = (
        component_counts[other_ids[0]] + component_counts[other_ids[1]]
    )
    ink_dominance = float(chosen_ink + 1) / float(other_ink + 1)
    component_dominance = float(chosen_components + 1) / float(
        other_components + 1
    )
    diagonal_dominance = float(
        np.sqrt(ink_dominance * component_dominance)
    )

    pattern_ink = ink.copy()
    pattern_ink[:corner_height, :corner_width] = 0
    pattern_ink[:corner_height, width - corner_width:] = 0
    pattern_ink[height - corner_height:, :corner_width] = 0
    pattern_ink[height - corner_height:, width - corner_width:] = 0
    long_axis_parallel_flip = 0 if width >= height else 1
    pattern_axis_overlap = _best_shifted_overlap(
        pattern_ink,
        np.flip(pattern_ink, axis=long_axis_parallel_flip),
        maximum_shift_px=2,
    )
    short_axis_parallel_flip = 1 - long_axis_parallel_flip
    pattern_short_axis_overlap = _best_shifted_overlap(
        pattern_ink,
        np.flip(pattern_ink, axis=short_axis_parallel_flip),
        maximum_shift_px=2,
    )
    pattern_biaxial_overlap = min(
        pattern_axis_overlap,
        pattern_short_axis_overlap,
    )
    global_overlap = _dice_overlap(ink, np.flip(ink, axis=(0, 1)))
    minimum_corner_ink = min(
        corner_counts[first_id],
        corner_counts[second_id],
    )
    expected_corner_ink = max(20.0, float(total_ink) * 0.010)
    quick_score = (
        (1.0 - corner_overlap) * 90.0
        + (1.0 - pattern_axis_overlap) * 100.0
        + (1.0 - pattern_short_axis_overlap) * 100.0
        + (1.0 - global_overlap) * 25.0
        + max(0.0, 1.2 - diagonal_dominance) * 45.0
        + max(0.0, expected_corner_ink - minimum_corner_ink) * 0.8
    )
    return {
        "score": float(quick_score),
        "corner_180_overlap": float(corner_overlap),
        "pattern_axis_overlap": float(pattern_axis_overlap),
        "pattern_long_axis_overlap": float(pattern_axis_overlap),
        "pattern_short_axis_overlap": float(pattern_short_axis_overlap),
        "pattern_biaxial_overlap": float(pattern_biaxial_overlap),
        "global_180_overlap": float(global_overlap),
        "diagonal_dominance": float(diagonal_dominance),
        "corner_counts": corner_counts,
        "index_component_counts": component_counts,
        "minimum_corner_ink": float(minimum_corner_ink),
        "total_ink_pixels": float(total_ink),
    }


def score_rectangular_card_corner_anchor(
    rectified: np.ndarray,
    pieces: Sequence[Dict[str, object]],
    targets: Sequence[np.ndarray],
    half_turns: Sequence[bool],
    warp_cache: Optional[dict] = None,
) -> Dict[str, object]:
    """Rank a layout using only the two legal index corners.

    This deliberately runs before the body-axis prefilter.  It renders at a
    lower resolution and does not inspect the central artwork, global card
    symmetry, colour, borders, or seams.  The result is therefore only a
    search constraint; every survivor is still checked by the existing axis
    prefilter and full-resolution validator.
    """
    card, valid = reconstruct_card_pixels(
        rectified,
        pieces,
        targets,
        half_turns,
        pixels_per_mm=MODE4_RENDER_PIXELS_PER_MM * 0.375,
        warp_cache=warp_cache,
    )
    ink = printed_ink_mask(card, valid)
    height, width = ink.shape
    total_ink = int(np.count_nonzero(ink))
    corner_width = max(5, int(round(width * 0.18)))
    corner_height = max(5, int(round(height * 0.20)))
    corners = (
        ink[:corner_height, :corner_width],
        ink[:corner_height, width - corner_width:],
        ink[height - corner_height:, :corner_width],
        ink[height - corner_height:, width - corner_width:],
    )
    corner_counts = [int(np.count_nonzero(corner)) for corner in corners]
    component_counts = [
        _small_index_component_count(
            corner,
            minimum_area=3,
            maximum_area=160,
        )
        for corner in corners
    ]
    first_id, second_id = (1, 2)
    other_ids = (0, 3)
    corner_overlap = _best_shifted_overlap(
        corners[first_id],
        np.flip(corners[second_id], axis=(0, 1)),
        maximum_shift_px=1,
    )
    chosen_ink = corner_counts[first_id] + corner_counts[second_id]
    other_ink = corner_counts[other_ids[0]] + corner_counts[other_ids[1]]
    chosen_components = component_counts[first_id] + component_counts[second_id]
    other_components = component_counts[other_ids[0]] + component_counts[other_ids[1]]
    ink_dominance = float(chosen_ink + 1) / float(other_ink + 1)
    component_dominance = float(chosen_components + 1) / float(
        other_components + 1
    )
    diagonal_dominance = float(
        np.sqrt(ink_dominance * component_dominance)
    )
    minimum_corner_ink = min(
        corner_counts[first_id],
        corner_counts[second_id],
    )
    expected_corner_ink = max(10.0, float(total_ink) * 0.008)
    score = (
        (1.0 - corner_overlap) * 100.0
        + max(0.0, 1.2 - diagonal_dominance) * 50.0
        + max(0.0, expected_corner_ink - minimum_corner_ink) * 1.2
    )
    return {
        "score": float(score),
        "corner_180_overlap": float(corner_overlap),
        "diagonal_dominance": float(diagonal_dominance),
        "corner_counts": corner_counts,
        "index_component_counts": component_counts,
        "minimum_corner_ink": float(minimum_corner_ink),
        "total_ink_pixels": float(total_ink),
    }


def score_rectangular_card_artwork(
    rectified: np.ndarray,
    pieces: Sequence[Dict[str, object]],
    targets: Sequence[np.ndarray],
    half_turns: Sequence[bool],
    warp_cache: Optional[dict] = None,
) -> Dict[str, object]:
    """Score 180 symmetry, blank borders and one diagonal index pair."""
    card, valid, piece_masks = reconstruct_card_pixels(
        rectified,
        pieces,
        targets,
        half_turns,
        return_piece_masks=True,
        warp_cache=warp_cache,
    )
    ink = printed_ink_mask(card, valid)
    red_ink = red_print_mask(card, valid)
    height, width = ink.shape
    total_ink = int(np.count_nonzero(ink))
    rotated_ink = np.flip(ink, axis=(0, 1))
    global_overlap = _dice_overlap(ink, rotated_ink)
    center_margin_x = max(1, int(round(width * 0.16)))
    center_margin_y = max(1, int(round(height * 0.16)))
    center_ink = ink[
        center_margin_y:height - center_margin_y,
        center_margin_x:width - center_margin_x,
    ]
    center_overlap = _best_shifted_overlap(
        center_ink,
        np.flip(center_ink, axis=(0, 1)),
    )

    # A card index is compact and touches one real outer corner.  Keep these
    # windows deliberately smaller than a quarter-card so central pips cannot
    # impersonate the rank/small-suit pair.
    corner_width = max(8, int(round(width * 0.18)))
    corner_height = max(8, int(round(height * 0.20)))
    corners = (
        ink[:corner_height, :corner_width],
        ink[:corner_height, width - corner_width:],
        ink[height - corner_height:, :corner_width],
        ink[height - corner_height:, width - corner_width:],
    )
    corner_counts = [int(np.count_nonzero(corner)) for corner in corners]
    index_component_counts = [
        _small_index_component_count(corner) for corner in corners
    ]
    # In the required final card orientation the rank/suit indices are only
    # legal at the top-right and bottom-left corners.  Allowing both
    # diagonals made a geometrically plausible but 90-degree-wrong card look
    # valid, so keep this pair fixed even when the other diagonal happens to
    # contain more central-pip pixels.
    diagonal_id = 1
    first_id, second_id = (1, 2)
    other_ids = (0, 3)
    first_corner = corners[first_id]
    second_corner = np.flip(corners[second_id], axis=(0, 1))
    corner_overlap = _best_shifted_overlap(first_corner, second_corner)
    chosen_ink = corner_counts[first_id] + corner_counts[second_id]
    other_ink = corner_counts[other_ids[0]] + corner_counts[other_ids[1]]
    chosen_index_components = (
        index_component_counts[first_id]
        + index_component_counts[second_id]
    )
    other_index_components = (
        index_component_counts[other_ids[0]]
        + index_component_counts[other_ids[1]]
    )
    diagonal_component_dominance = float(chosen_index_components + 1) / float(
        other_index_components + 1
    )
    # A blurred pip grazing a corner window can create one tiny component in
    # every corner.  Component counts alone then report a false 1:1 tie even
    # though the two real index corners contain far more ink.  Combine the
    # component and ink-area evidence geometrically: both support normal
    # cases, while strong area evidence rescues a true index pair from small
    # nuisance components without accepting the opposite diagonal.
    diagonal_ink_dominance = float(chosen_ink + 1) / float(other_ink + 1)
    diagonal_dominance = float(
        np.sqrt(diagonal_component_dominance * diagonal_ink_dominance)
    )
    corner_balance_error = abs(
        corner_counts[first_id] - corner_counts[second_id]
    ) / max(1.0, float(chosen_ink))

    # Corner indices are related by a 180-degree turn, while the body pips
    # of a standard playing card are mirrored about the center axis parallel
    # to the card's long edges.  Remove all four index windows before testing
    # that independent artwork symmetry.
    pattern_ink = ink.copy()
    pattern_ink[:corner_height, :corner_width] = 0
    pattern_ink[:corner_height, width - corner_width:] = 0
    pattern_ink[height - corner_height:, :corner_width] = 0
    pattern_ink[
        height - corner_height:,
        width - corner_width:,
    ] = 0
    long_axis_parallel_flip = 0 if width >= height else 1
    pattern_axis_overlap = _best_shifted_overlap(
        pattern_ink,
        np.flip(pattern_ink, axis=long_axis_parallel_flip),
    )
    short_axis_parallel_flip = 1 - long_axis_parallel_flip
    pattern_short_axis_overlap = _best_shifted_overlap(
        pattern_ink,
        np.flip(pattern_ink, axis=short_axis_parallel_flip),
    )
    pattern_biaxial_overlap = min(
        pattern_axis_overlap,
        pattern_short_axis_overlap,
    )

    border_thickness = max(3, int(round(min(width, height) * 0.055)))
    border = np.zeros_like(ink, dtype=bool)
    border[:border_thickness, :] = True
    border[-border_thickness:, :] = True
    border[:, :border_thickness] = True
    border[:, -border_thickness:] = True
    allowed = np.zeros_like(ink, dtype=bool)
    if diagonal_id == 0:
        allowed[:corner_height, :corner_width] = True
        allowed[-corner_height:, -corner_width:] = True
        diagonal_name = "top_left+bottom_right"
    else:
        allowed[:corner_height, -corner_width:] = True
        allowed[-corner_height:, :corner_width] = True
        diagonal_name = "top_right+bottom_left"
    forbidden_border_ink = int(np.count_nonzero(ink.astype(bool) & border & ~allowed))
    forbidden_border_ratio = forbidden_border_ink / max(1.0, float(total_ink))


    # Even the rank/small-suit index has a white margin to both card edges.
    # A cut pip placed on the outside boundary touches this rim and is not a
    # legitimate index, so this check is intentionally applied everywhere.
    rim_thickness = max(2, int(round(min(width, height) * 0.018)))
    outer_rim = np.zeros_like(ink, dtype=bool)
    outer_rim[:rim_thickness, :] = True
    outer_rim[-rim_thickness:, :] = True
    outer_rim[:, :rim_thickness] = True
    outer_rim[:, -rim_thickness:] = True
    outer_rim_ink = int(np.count_nonzero(ink.astype(bool) & outer_rim))
    outer_rim_ink_ratio = outer_rim_ink / max(1.0, float(total_ink))
    total_red_ink = int(np.count_nonzero(red_ink))
    forbidden_red_border_ink = int(
        np.count_nonzero(red_ink.astype(bool) & border & ~allowed)
    )
    forbidden_red_border_ratio = forbidden_red_border_ink / max(
        1.0, float(total_red_ink)
    )
    outer_red_rim_ink = int(
        np.count_nonzero(red_ink.astype(bool) & outer_rim)
    )
    outer_red_rim_ratio = outer_red_rim_ink / max(
        1.0, float(total_red_ink)
    )

    # Seam continuity was removed from both ranking and validation because
    # repeated pips can make unrelated neighbouring artwork look continuous.
    # Do not compute the former morphology even as a diagnostic: it adds work
    # and its value is not reliable enough to guide the solver.

    minimum_corner_ink = min(corner_counts[first_id], corner_counts[second_id])
    # The reconstructed canvas has a stable physical scale, but blur and
    # print colour still change how many index pixels survive thresholding.
    # A fixed 200-pixel floor rejected a visually clear low-contrast index.
    # Scale the requirement with the card's total detected ink while keeping
    # conservative lower and upper bounds; the component checks below still
    # require a real index group in both legal corners.
    minimum_required_corner_ink = max(
        80.0,
        min(200.0, float(total_ink) * 0.012),
    )

    score = (
        (1.0 - global_overlap) * 30.0
        + (1.0 - corner_overlap) * 80.0
        + max(0.0, 1.8 - diagonal_dominance) * 30.0
        + max(0.0, 250.0 - minimum_corner_ink) * 1.00
        + corner_balance_error * 8.0
        + forbidden_border_ratio * 35.0
        + outer_rim_ink_ratio * 100.0
        + forbidden_red_border_ratio * 300.0
        + outer_red_rim_ratio * 500.0
    )
    # A two-strip cut has a much longer exposed cut edge relative to its ink
    # area, so camera shadow contributes a larger rim ratio.  Keep the
    # original strict limits for three/four-piece cards.
    has_reliable_red_print = total_red_ink >= 30
    maximum_forbidden_border_ratio = (
        0.22 if len(pieces) == 2
        else 0.15 if has_reliable_red_print
        else 0.125
    )
    maximum_outer_rim_ratio = (
        0.15 if len(pieces) == 2 or has_reliable_red_print else 0.10
    )
    minimum_corner_overlap = 0.35 if len(pieces) == 2 else 0.48
    valid_artwork = bool(
        total_ink >= 30
        and minimum_corner_ink >= minimum_required_corner_ink
        # The two rank/small-suit groups must occupy opposite corners of the
        # final large rectangle.  Central-seam coordinates never enter these
        # corner windows.
        and index_component_counts[first_id] >= 1
        and index_component_counts[second_id] >= 1
        and global_overlap >= 0.32
        and corner_overlap >= minimum_corner_overlap
        and (len(pieces) == 2 or pattern_biaxial_overlap >= 0.60)
        and diagonal_dominance >= 1.2
        and forbidden_border_ratio <= maximum_forbidden_border_ratio
        and outer_rim_ink_ratio <= maximum_outer_rim_ratio
        and forbidden_red_border_ratio <= 0.003
        and outer_red_rim_ratio <= 0.01
    )
    return {
        "score": float(score),
        "valid": valid_artwork,
        "global_180_overlap": float(global_overlap),
        "center_180_overlap": float(center_overlap),
        "corner_180_overlap": float(corner_overlap),
        "corner_diagonal": diagonal_name,
        "corner_counts": corner_counts,
        "minimum_corner_ink": float(minimum_corner_ink),
        "minimum_required_corner_ink": float(
            minimum_required_corner_ink
        ),
        "index_component_counts": index_component_counts,
        "diagonal_component_dominance": float(
            diagonal_component_dominance
        ),
        "diagonal_ink_dominance": float(diagonal_ink_dominance),
        "diagonal_dominance": float(diagonal_dominance),
        "corner_balance_error": float(corner_balance_error),
        "pattern_axis_overlap": float(pattern_axis_overlap),
        "pattern_long_axis_overlap": float(pattern_axis_overlap),
        "pattern_short_axis_overlap": float(pattern_short_axis_overlap),
        "pattern_biaxial_overlap": float(pattern_biaxial_overlap),
        "pattern_axis": (
            "horizontal" if width >= height else "vertical"
        ),
        "forbidden_border_ink_ratio": float(forbidden_border_ratio),
        "forbidden_border_ink_pixels": float(forbidden_border_ink),
        "outer_rim_ink_ratio": float(outer_rim_ink_ratio),
        "outer_rim_ink_pixels": float(outer_rim_ink),
        "forbidden_red_border_ratio": float(forbidden_red_border_ratio),
        "forbidden_red_border_pixels": float(forbidden_red_border_ink),
        "outer_red_rim_ratio": float(outer_red_rim_ratio),
        "outer_red_rim_pixels": float(outer_red_rim_ink),
        "seam_diagnostics_computed": False,
        "maximum_outer_rim_ratio": float(maximum_outer_rim_ratio),
        "maximum_forbidden_border_ratio": float(
            maximum_forbidden_border_ratio
        ),
        "minimum_corner_overlap": float(minimum_corner_overlap),
        "total_ink_pixels": float(total_ink),
        "half_turns": [bool(value) for value in half_turns],
        "_card_image": card,
        "_ink_mask": ink,
    }


def render_rectangular_card_solution(
    rectified: np.ndarray,
    pieces: Sequence[Dict[str, object]],
    targets: Sequence[np.ndarray],
    half_turns: Sequence[bool],
    details: Dict[str, object],
    output_path: str,
) -> None:
    """Save the reconstructed card with the new Mode 4 validation metrics."""
    card, _ = reconstruct_card_pixels(
        rectified,
        pieces,
        targets,
        half_turns,
    )
    scale = min(3.0, 900.0 / max(card.shape[:2]))
    enlarged = cv2.resize(
        card,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_NEAREST,
    )
    header = np.full((96, enlarged.shape[1], 3), 35, dtype=np.uint8)
    lines = [
        "Mode4 rectangle + 180 symmetry",
        "global={:.3f} axis-L/S={:.3f}/{:.3f} corner180={:.3f} diag={} border={:.3f} rim={:.3f}".format(
            details["global_180_overlap"],
            details["pattern_long_axis_overlap"],
            details["pattern_short_axis_overlap"],
            details["corner_180_overlap"],
            details["corner_diagonal"],
            details["forbidden_border_ink_ratio"],
            details["outer_rim_ink_ratio"],
        ),
    ]
    for line_id, line in enumerate(lines):
        cv2.putText(
            header,
            line,
            (12, 34 + line_id * 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    cv2.imwrite(output_path, np.vstack((header, enlarged)))
