r"""三模式拼图视觉状态机及控制联调接口。

标准输入协议示例：
    MODE,1
    START,C:\Users\77019\Desktop\test4.jpg
    STATUS
    NEXT
    ACK,0,OK
    RESET
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import sys
import time
import uuid
from dataclasses import asdict, dataclass, replace
from enum import Enum, IntEnum
from itertools import combinations, permutations, product
from pathlib import Path
from typing import Callable, Dict, List, Optional

import cv2
import numpy as np

from fragment_vision import (
    DetectConfig,
    PIXELS_PER_MM,
    choose_pickup_point,
    contour_centroid,
    detect_pieces,
    draw_results,
    make_payload,
    process_frame,
    remove_false_short_edges,
    rectify_a4,
    resize_for_debug,
    sharpen_polygon_from_contour,
)
from card_pattern_solver import (
    enumerate_rectangular_slot_assignments,
    half_turn_variants,
    regularize_rectangular_pieces,
    render_rectangular_card_solution,
    score_rectangular_card_artwork,
    score_rectangular_card_corner_anchor,
    score_rectangular_card_prefilter,
)
from puzzle_solver import (
    enumerate_rectangular_piece_layouts,
    generate_edge_search_states,
    interior_angles,
    native_acceleration_available,
    normalize_solution,
    rank_unknown_rectangle_candidates,
    rectangular_piece_diagnostics,
    solve_with_template,
    unknown_rectangle_score,
)
from solve_puzzle_image import polygon_centroid, render_solution


TARGET_CLEARANCE_MM = 8.0
TARGET_DIVIDER_Y_MM = 148.5
TARGET_DIVIDER_GAP_MM = 10.0
# Canonical Y=0..148.5 mm is the physical left-hand destination half.
# Geometry is initially normalized here; after clearance, the whole assembly
# is translated so its nearest edge stays 10 mm above the middle divider.
TARGET_ORIGIN_MM = (55.0, 30.0)
# This is only a pre-clearance contour-fit tolerance. Perspective and
# polygon quantization can produce a thin provisional overlap even for the
# correct assembly. The selected layout is still separated and then checked
# against the strict final zero-overlap tolerance before a plan is emitted.
MODE4_PROVISIONAL_OVERLAP_MAX_MM2 = 15.0
MODE4_MAX_GEOMETRY_CANDIDATES = 60
MODE4_MAX_ASSIGNMENT_CANDIDATES = 48
MODE4_PREFILTER_BATCH_SIZE = 16
# A correct reconstruction should remain close to the best body-pattern
# axis symmetry, but camera exposure and cut-edge warping make a strict
# absolute threshold brittle.  Keep this relative window, then let the full
# corner, dual-axis and 180-degree artwork scores decide among the survivors.
MODE4_PATTERN_AXIS_SELECTION_TOLERANCE = 0.04
# Keep enough candidates for blurred/partially cut rank glyphs.  Clear
# indices still rank first, while 128 is substantially below the 384/768
# orientation combinations produced by typical four-piece inputs.
MODE4_CORNER_ANCHOR_LIMIT = 128
MODE3_FAST_BEAM_MIN_PARTITION_EVIDENCE = 8
MODE3_FAST_BEAM_MAX_SCORE = 0.15
MODE23_CORNER_BEAM_WIDTH = 120
MODE2_CORNER_TOLERANCE_DEG = 18.0
MODE3_CORNER_TOLERANCE_DEG = 35.0
MODE23_CORNER_MAX_SCORE = 0.147
MODE3_RECTANGULAR_PIECE_ANGLE_TOLERANCE_DEG = 8.0
MODE2_MAX_OUTER_CORNER_OFFSET_MM = 5.0
MODE3_MAX_OUTER_CORNER_OFFSET_MM = 5.0
CORNER_FAST_MAX_OUTER_CORNER_OFFSET_MM = 5.0
MODE23_STRONG_RECTANGLE_MAX_OUTER_CORNER_ERROR_DEG = 25.0


class TaskMode(IntEnum):
    FIXED_COLOR = 1
    UNKNOWN_WHITE = 2
    CARD_GEOMETRY = 3
    CARD_ARTWORK = 4


class VisionState(str, Enum):
    IDLE = "idle"
    MODE_SELECTED = "mode_selected"
    CAPTURING = "capturing"
    CALIBRATING = "calibrating"
    DETECTING = "detecting"
    SOLVING = "solving"
    PLAN_READY = "plan_ready"
    WAITING_ACK = "waiting_ack"
    VERIFYING = "verifying"
    COMPLETED = "completed"
    ERROR = "error"


@dataclass
class MoveAction:
    sequence: int
    piece_id: int
    source_pickup_mm: List[float]
    source_centroid_mm: List[float]
    target_pickup_mm: List[float]
    target_centroid_mm: List[float]
    rotation_delta_deg: float
    status: str = "pending"
    retries: int = 0


TaskHandler = Callable[[str], Dict[str, object]]


class VisionStateMachine:
    def __init__(
        self,
        template_path: Optional[str] = None,
        output_directory: Optional[str] = None,
        maximum_retries: int = 2,
        detect_config: Optional[DetectConfig] = None,
        progress_callback: Optional[
            Callable[[float, str], None]
        ] = None,
        solver_workers: int = 1,
        mode3_beam_levels=None,
    ) -> None:
        workspace = Path(__file__).resolve().parent
        self.template_path = Path(
            template_path
            or workspace / "fixed_puzzle_template.json"
        )
        self.output_directory = Path(
            output_directory or workspace / "tmp"
        )
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self.maximum_retries = maximum_retries
        self.detect_config = detect_config or DetectConfig()
        self.progress_callback = progress_callback
        self.solver_workers = max(1, int(solver_workers))
        if mode3_beam_levels is None:
            mode3_beam_levels = (400, 1600)
        self.mode3_beam_levels = tuple(
            int(value) for value in mode3_beam_levels
        )
        if (
            not self.mode3_beam_levels
            or any(value <= 0 for value in self.mode3_beam_levels)
        ):
            raise ValueError("Mode 3 beam levels must be positive")
        self.handlers: Dict[TaskMode, TaskHandler] = {
            TaskMode.FIXED_COLOR: self._run_fixed_color,
            TaskMode.UNKNOWN_WHITE: self._run_unknown_white,
            TaskMode.CARD_GEOMETRY: self._run_card_geometry,
            TaskMode.CARD_ARTWORK: self._run_card_artwork,
        }
        self.reset()

    def _map_solver_jobs(self, function, jobs):
        """Evaluate independent scoring jobs in stable input order."""
        jobs = list(jobs)
        if self.solver_workers <= 1 or len(jobs) <= 1:
            return [function(job) for job in jobs]
        worker_count = min(self.solver_workers, len(jobs))
        chunk_size = (len(jobs) + worker_count - 1) // worker_count
        chunks = [
            jobs[start : start + chunk_size]
            for start in range(0, len(jobs), chunk_size)
        ]

        def evaluate_chunk(chunk):
            return [function(job) for job in chunk]

        with ThreadPoolExecutor(
            max_workers=worker_count,
            thread_name_prefix="vision-score",
        ) as executor:
            chunk_results = list(executor.map(evaluate_chunk, chunks))
        return [
            result
            for chunk_result in chunk_results
            for result in chunk_result
        ]

    def _report_progress(
        self,
        progress: float,
        message: str,
    ) -> None:
        if self.progress_callback is not None:
            self.progress_callback(
                max(0.0, min(1.0, float(progress))),
                str(message),
            )

    def reset(self) -> Dict[str, object]:
        self.state = VisionState.IDLE
        self.mode: Optional[TaskMode] = None
        self.plan_id: Optional[str] = None
        self.actions: List[MoveAction] = []
        self.active_action: Optional[MoveAction] = None
        self.metrics: Dict[str, object] = {}
        self.timing_ms: Dict[str, float] = {}
        self.last_error: Optional[str] = None
        self.detection_image: Optional[str] = None
        self.solution_image: Optional[str] = None
        self.best_effort_candidate = None
        return self.status()

    def register_handler(
        self,
        mode: TaskMode,
        handler: TaskHandler,
    ) -> None:
        """后续为模式2、模式3注册对应视觉处理函数。"""
        self.handlers[mode] = handler

    def select_mode(self, mode: int) -> Dict[str, object]:
        selected = TaskMode(mode)
        self.mode = selected
        self.plan_id = None
        self.actions = []
        self.active_action = None
        self.metrics = {}
        self.best_effort_candidate = None
        self.last_error = None
        self.state = VisionState.MODE_SELECTED
        return self.status()

    def start(self, image_path: str) -> Dict[str, object]:
        if self.mode is None:
            raise RuntimeError("请先通过 MODE 选择赛题模式")
        handler = self.handlers.get(self.mode)
        if handler is None:
            raise NotImplementedError(
                "模式{}尚未接入视觉处理器".format(int(self.mode))
            )

        self.plan_id = uuid.uuid4().hex[:12]
        self.actions = []
        self.active_action = None
        self.last_error = None
        try:
            result = handler(image_path)
            return self._accept_plan_result(result)
        except Exception as error:
            self.state = VisionState.ERROR
            self.last_error = str(error)
            raise

    def start_best_effort(
        self,
        image_path: str,
        reason: str = "solve_timeout",
        elapsed_seconds: Optional[float] = None,
    ) -> Dict[str, object]:
        """Build a mechanically safe transport plan when solving times out."""
        if self.mode is None:
            raise RuntimeError("select a mode before best-effort planning")
        self.plan_id = uuid.uuid4().hex[:12]
        self.actions = []
        self.active_action = None
        self.last_error = None
        try:
            result = self._run_best_effort_transport(
                image_path,
                reason=reason,
                elapsed_seconds=elapsed_seconds,
            )
            return self._accept_plan_result(result)
        except Exception as error:
            self.state = VisionState.ERROR
            self.last_error = str(error)
            raise

    def start_best_candidate(
        self,
        candidate: Dict[str, object],
        elapsed_seconds: Optional[float] = None,
    ) -> Dict[str, object]:
        """Publish the best safe approximate puzzle found during retries."""
        if self.mode is None:
            raise RuntimeError("select a mode before best-candidate planning")
        self.plan_id = uuid.uuid4().hex[:12]
        self.actions = []
        self.active_action = None
        self.last_error = None
        try:
            result = self._build_best_candidate_result(
                candidate,
                elapsed_seconds=elapsed_seconds,
            )
            return self._accept_plan_result(result)
        except Exception as error:
            self.state = VisionState.ERROR
            self.last_error = str(error)
            raise

    def _remember_best_candidate(
        self,
        detection,
        detection_annotated,
        targets,
        rotations,
        metrics,
        quality_key,
        source,
        matches=None,
    ) -> None:
        candidate = {
            "quality_key": tuple(float(value) for value in quality_key),
            "detection": detection,
            "detection_annotated": np.asarray(detection_annotated).copy(),
            "targets": [
                np.asarray(target, dtype=np.float32).copy()
                for target in targets
            ],
            "rotations": [float(value) for value in rotations],
            "metrics": dict(metrics),
            "source": str(source),
            "matches": tuple(matches or ()),
        }
        if (
            self.best_effort_candidate is None
            or candidate["quality_key"]
            < self.best_effort_candidate["quality_key"]
        ):
            self.best_effort_candidate = candidate

    def _accept_plan_result(self, result: Dict[str, object]):
        self.actions = result["actions"]
        self.metrics = result["metrics"]
        self.timing_ms = result["timing_ms"]
        self.detection_image = result["detection_image"]
        self.solution_image = result["solution_image"]
        self.state = VisionState.PLAN_READY
        return {
            "plan_id": self.plan_id,
            "state": self.state.value,
            "action_count": len(self.actions),
            "metrics": self.metrics,
            "timing_ms": self.timing_ms,
            "detection_image": self.detection_image,
            "solution_image": self.solution_image,
        }

    def next_action(self) -> Dict[str, object]:
        if self.state == VisionState.WAITING_ACK and self.active_action:
            return asdict(self.active_action)
        if self.state != VisionState.PLAN_READY:
            raise RuntimeError(
                "当前状态 {} 不能获取下一动作".format(self.state.value)
            )

        pending = [
            action
            for action in self.actions
            if action.status in ("pending", "retry")
        ]
        if not pending:
            self.state = VisionState.COMPLETED
            return {"completed": True}

        self.active_action = pending[0]
        self.active_action.status = "sent"
        self.state = VisionState.WAITING_ACK
        return asdict(self.active_action)

    def acknowledge(
        self,
        piece_id: int,
        success: bool,
    ) -> Dict[str, object]:
        if self.state != VisionState.WAITING_ACK or self.active_action is None:
            raise RuntimeError("当前没有等待确认的动作")
        if self.active_action.piece_id != piece_id:
            raise RuntimeError(
                "确认碎片{}与当前碎片{}不一致".format(
                    piece_id,
                    self.active_action.piece_id,
                )
            )

        if success:
            self.active_action.status = "done"
            self.active_action = None
            if all(action.status == "done" for action in self.actions):
                self.state = VisionState.COMPLETED
            else:
                self.state = VisionState.PLAN_READY
        else:
            self.active_action.retries += 1
            if self.active_action.retries > self.maximum_retries:
                self.active_action.status = "failed"
                self.state = VisionState.ERROR
                self.last_error = "碎片{}移动连续失败".format(piece_id)
            else:
                self.active_action.status = "retry"
                self.active_action = None
                self.state = VisionState.PLAN_READY
        return self.status()

    def get_plan(self) -> Dict[str, object]:
        return {
            "plan_id": self.plan_id,
            "mode": int(self.mode) if self.mode is not None else None,
            "state": self.state.value,
            "actions": [asdict(action) for action in self.actions],
            "metrics": self.metrics,
            "timing_ms": self.timing_ms,
        }

    def status(self) -> Dict[str, object]:
        completed = sum(
            action.status == "done" for action in self.actions
        )
        return {
            "state": self.state.value,
            "mode": int(self.mode) if self.mode is not None else None,
            "plan_id": self.plan_id,
            "completed_actions": completed,
            "total_actions": len(self.actions),
            "active_piece_id": (
                self.active_action.piece_id
                if self.active_action is not None
                else None
            ),
            "last_error": self.last_error,
        }

    def handle_command(self, command: str) -> Dict[str, object]:
        """处理一行控制命令，适合后续串口或TCP转发。"""
        command = command.strip()
        if not command:
            raise ValueError("空命令")
        operation, separator, arguments = command.partition(",")
        operation = operation.strip().upper()

        if operation == "PING":
            return {"pong": True, "state": self.state.value}
        if operation == "RESET":
            return self.reset()
        if operation == "STATUS":
            return self.status()
        if operation == "PLAN":
            return self.get_plan()
        if operation == "MODE":
            if not separator:
                raise ValueError("MODE 命令缺少模式编号")
            return self.select_mode(int(arguments.strip()))
        if operation == "START":
            if not separator:
                raise ValueError("START 命令缺少图片路径")
            return self.start(arguments.strip())
        if operation == "NEXT":
            return self.next_action()
        if operation == "ACK":
            parts = [part.strip() for part in arguments.split(",")]
            if len(parts) != 2:
                raise ValueError("ACK 格式应为 ACK,piece_id,OK|FAIL")
            success = parts[1].upper() == "OK"
            if parts[1].upper() not in ("OK", "FAIL"):
                raise ValueError("ACK 结果只能是 OK 或 FAIL")
            return self.acknowledge(int(parts[0]), success)
        raise ValueError("未知命令：{}".format(operation))

    def _build_best_candidate_result(
        self,
        candidate: Dict[str, object],
        elapsed_seconds: Optional[float],
    ) -> Dict[str, object]:
        """Turn a rejected solver candidate into a safe approximate plan."""
        started = time.perf_counter()
        detection = candidate["detection"]
        pieces = detection["pieces"]
        targets = [
            np.asarray(target, dtype=np.float32).copy()
            for target in candidate["targets"]
        ]
        rotations = [float(value) for value in candidate["rotations"]]
        if len(targets) != len(pieces) or len(rotations) != len(pieces):
            raise RuntimeError("best candidate piece count is inconsistent")

        matches = self._infer_adjacency_matches(
            targets,
            existing_matches=candidate.get("matches", ()),
        )
        targets, adjacency_gaps = self._add_assembly_clearance(
            targets,
            matches=matches,
            target_origin=np.asarray(TARGET_ORIGIN_MM, dtype=np.float32),
            clearance_mm=TARGET_CLEARANCE_MM,
        )
        targets = self._separate_small_overlaps(
            targets,
            target_origin=np.asarray(TARGET_ORIGIN_MM, dtype=np.float32),
        )
        points = np.concatenate(targets, axis=0)
        width = float(np.max(points[:, 0]) - np.min(points[:, 0]))
        if width > 210.0:
            raise RuntimeError("best approximate candidate is wider than A4")
        shift_x = 105.0 - 0.5 * float(
            np.min(points[:, 0]) + np.max(points[:, 0])
        )
        targets = [
            target + np.asarray([shift_x, 0.0], dtype=np.float32)
            for target in targets
        ]
        targets, placement_metrics = self._place_above_divider(targets)
        overlap = self._validate_zero_overlap(targets)
        points = np.concatenate(targets, axis=0)
        minimum = np.min(points, axis=0)
        maximum = np.max(points, axis=0)
        if minimum[0] < -0.001 or maximum[0] > 210.001:
            raise RuntimeError("best approximate candidate is outside A4 width")

        width = float(maximum[0] - minimum[0])
        height = float(maximum[1] - minimum[1])
        area = sum(
            abs(float(cv2.contourArea(target.astype(np.float32))))
            for target in targets
        )
        metrics = dict(candidate.get("metrics", {}))
        metrics.update(
            {
                "best_effort": True,
                "puzzle_solved": False,
                "best_effort_reason": "automatic_solve_timeout",
                "best_effort_strategy": "best_scored_partial_puzzle",
                "best_effort_candidate_source": candidate["source"],
                "best_effort_quality_key": list(candidate["quality_key"]),
                "best_effort_elapsed_seconds": (
                    None
                    if elapsed_seconds is None
                    else round(float(elapsed_seconds), 3)
                ),
                "piece_count": len(pieces),
                "long_side_mm": round(max(width, height), 3),
                "short_side_mm": round(min(width, height), 3),
                "fill_ratio": round(area / max(width * height, 1e-6), 6),
                "overlap_area_mm2": overlap,
                "target_clearance_mm": TARGET_CLEARANCE_MM,
                "adjacency_gap_min_mm": round(
                    min(adjacency_gaps) if adjacency_gaps else 0.0,
                    3,
                ),
                "adjacency_gap_max_mm": round(
                    max(adjacency_gaps) if adjacency_gaps else 0.0,
                    3,
                ),
                "workspace_source": "physical_right",
                "workspace_target": "physical_left",
            }
        )
        metrics.update(placement_metrics)

        detection_image = self.output_directory / "fsm_best_candidate_detection.jpg"
        solution_image = self.output_directory / "fsm_best_candidate_solution.jpg"
        cv2.imwrite(
            str(detection_image),
            np.asarray(candidate["detection_annotated"]),
        )
        render_solution(targets, rotations, metrics, str(solution_image))

        order = sorted(
            range(len(pieces)),
            key=lambda piece_id: pieces[piece_id]["area_px2"],
            reverse=True,
        )
        actions = []
        for sequence, piece_id in enumerate(order):
            piece = pieces[piece_id]
            target_center = polygon_centroid(targets[piece_id])
            rotation_delta = self._normalize_angle(rotations[piece_id])
            target_pickup = self._target_pickup_point(
                piece["pickup_mm"],
                piece["centroid_mm"],
                target_center,
                rotation_delta,
            )
            actions.append(
                MoveAction(
                    sequence=sequence,
                    piece_id=piece_id,
                    source_pickup_mm=[round(float(v), 2) for v in piece["pickup_mm"]],
                    source_centroid_mm=[round(float(v), 2) for v in piece["centroid_mm"]],
                    target_pickup_mm=[round(float(v), 2) for v in target_pickup],
                    target_centroid_mm=[round(float(v), 2) for v in target_center],
                    rotation_delta_deg=round(rotation_delta, 2),
                )
            )
        total_ms = (time.perf_counter() - started) * 1000.0
        return {
            "actions": actions,
            "metrics": metrics,
            "timing_ms": {
                "best_candidate_finalize": round(total_ms, 2),
                "total": round(total_ms, 2),
            },
            "detection_image": str(detection_image),
            "solution_image": str(solution_image),
        }

    def _run_best_effort_transport(
        self,
        image_path: str,
        reason: str,
        elapsed_seconds: Optional[float],
    ) -> Dict[str, object]:
        """Move all detected pieces into a safe compact target layout."""
        total_start = time.perf_counter()
        self.state = VisionState.CAPTURING
        frame = cv2.imread(image_path)
        if frame is None:
            raise RuntimeError("cannot read best-effort image: {}".format(image_path))
        frame = resize_for_debug(frame)
        load_ms = (time.perf_counter() - total_start) * 1000.0

        self.state = VisionState.DETECTING
        vision_start = time.perf_counter()
        fallback_config = replace(
            self.detect_config,
            include_dark_artwork_in_piece_mask=(
                self.mode == TaskMode.CARD_GEOMETRY
            ),
        )
        _, detection_annotated, detection = process_frame(
            frame,
            fallback_config,
        )
        pieces = detection["pieces"]
        if not 2 <= len(pieces) <= 4:
            raise RuntimeError(
                "best-effort transport requires 2 to 4 detected pieces; "
                "detected {}".format(len(pieces))
            )
        source_polygons = [
            np.asarray(piece["vertices_mm"], dtype=np.float32)
            for piece in pieces
        ]
        vision_ms = (time.perf_counter() - vision_start) * 1000.0

        self.state = VisionState.SOLVING
        layout_start = time.perf_counter()
        targets, rotations, layout_metrics = self._pack_best_effort_targets(
            source_polygons,
            gap_mm=TARGET_CLEARANCE_MM,
        )
        overlap = self._validate_zero_overlap(targets)
        all_points = np.concatenate(targets, axis=0)
        minimum = np.min(all_points, axis=0)
        maximum = np.max(all_points, axis=0)
        if (
            minimum[0] < -0.001
            or minimum[1] < -0.001
            or maximum[0] > 210.001
            or maximum[1]
            > TARGET_DIVIDER_Y_MM - TARGET_DIVIDER_GAP_MM + 0.001
        ):
            raise RuntimeError("best-effort target layout is outside workspace")
        layout_ms = (time.perf_counter() - layout_start) * 1000.0

        polygon_area = sum(
            abs(float(cv2.contourArea(polygon.astype(np.float32))))
            for polygon in targets
        )
        width = float(maximum[0] - minimum[0])
        height = float(maximum[1] - minimum[1])
        metrics = {
            "best_effort": True,
            "puzzle_solved": False,
            "best_effort_reason": str(reason),
            "best_effort_elapsed_seconds": (
                None
                if elapsed_seconds is None
                else round(float(elapsed_seconds), 3)
            ),
            "best_effort_strategy": "safe_compact_transport",
            "piece_count": len(pieces),
            "long_side_mm": round(max(width, height), 3),
            "short_side_mm": round(min(width, height), 3),
            "fill_ratio": round(
                polygon_area / max(width * height, 1e-6),
                6,
            ),
            "overlap_area_mm2": overlap,
            "target_clearance_mm": TARGET_CLEARANCE_MM,
            "target_origin_mm": [
                round(float(minimum[0]), 3),
                round(float(minimum[1]), 3),
            ],
            "target_max_y_mm": round(float(maximum[1]), 3),
            "target_divider_y_mm": TARGET_DIVIDER_Y_MM,
            "target_divider_gap_mm": round(
                TARGET_DIVIDER_Y_MM - float(maximum[1]),
                3,
            ),
            "workspace_source": "physical_right",
            "workspace_target": "physical_left",
        }
        metrics.update(layout_metrics)

        detection_image = self.output_directory / "fsm_best_effort_detection.jpg"
        solution_image = self.output_directory / "fsm_best_effort_solution.jpg"
        cv2.imwrite(str(detection_image), detection_annotated)
        render_solution(targets, rotations, metrics, str(solution_image))

        order = sorted(
            range(len(pieces)),
            key=lambda piece_id: pieces[piece_id]["area_px2"],
            reverse=True,
        )
        actions = []
        for sequence, piece_id in enumerate(order):
            piece = pieces[piece_id]
            target_center = polygon_centroid(targets[piece_id])
            rotation_delta = self._normalize_angle(rotations[piece_id])
            target_pickup = self._target_pickup_point(
                piece["pickup_mm"],
                piece["centroid_mm"],
                target_center,
                rotation_delta,
            )
            actions.append(
                MoveAction(
                    sequence=sequence,
                    piece_id=piece_id,
                    source_pickup_mm=[
                        round(float(value), 2)
                        for value in piece["pickup_mm"]
                    ],
                    source_centroid_mm=[
                        round(float(value), 2)
                        for value in piece["centroid_mm"]
                    ],
                    target_pickup_mm=[
                        round(float(value), 2)
                        for value in target_pickup
                    ],
                    target_centroid_mm=[
                        round(float(value), 2)
                        for value in target_center
                    ],
                    rotation_delta_deg=round(rotation_delta, 2),
                )
            )

        total_ms = (time.perf_counter() - total_start) * 1000.0
        return {
            "actions": actions,
            "metrics": metrics,
            "timing_ms": {
                "image_load": round(load_ms, 2),
                "vision": round(vision_ms, 2),
                "best_effort_layout": round(layout_ms, 2),
                "total": round(total_ms, 2),
            },
            "detection_image": str(detection_image),
            "solution_image": str(solution_image),
        }

    @staticmethod
    def _pack_best_effort_targets(polygons, gap_mm=TARGET_CLEARANCE_MM):
        """Choose the most compact safe shelf layout over 0/90 rotations."""
        count = len(polygons)
        usable_width = 200.0
        usable_height = TARGET_DIVIDER_Y_MM - TARGET_DIVIDER_GAP_MM - 10.0
        normalized = []
        for polygon in polygons:
            center = polygon_centroid(np.asarray(polygon, dtype=np.float32))
            normalized.append(np.asarray(polygon, dtype=np.float32) - center)

        best = None
        for order in permutations(range(count)):
            for quarter_turns in product((0, 1), repeat=count):
                variants = []
                for piece_id, polygon in enumerate(normalized):
                    angle = 90.0 * quarter_turns[piece_id]
                    radians = np.deg2rad(angle)
                    rotation = np.asarray(
                        [
                            [np.cos(radians), -np.sin(radians)],
                            [np.sin(radians), np.cos(radians)],
                        ],
                        dtype=np.float32,
                    )
                    rotated = polygon @ rotation.T
                    variants.append(
                        (rotated, np.min(rotated, axis=0), np.max(rotated, axis=0))
                    )
                for columns in range(1, count + 1):
                    rows = [
                        order[start : start + columns]
                        for start in range(0, count, columns)
                    ]
                    row_widths = []
                    row_heights = []
                    for row in rows:
                        row_widths.append(
                            sum(
                                float(variants[piece_id][2][0]
                                      - variants[piece_id][1][0])
                                for piece_id in row
                            )
                            + gap_mm * max(0, len(row) - 1)
                        )
                        row_heights.append(
                            max(
                                float(variants[piece_id][2][1]
                                      - variants[piece_id][1][1])
                                for piece_id in row
                            )
                        )
                    width = max(row_widths)
                    height = sum(row_heights) + gap_mm * (len(rows) - 1)
                    if width > usable_width or height > usable_height:
                        continue
                    long_side = max(width, height)
                    short_side = min(width, height)
                    range_penalty = (
                        max(0.0, 90.0 - long_side)
                        + max(0.0, long_side - 120.0)
                        + max(0.0, 50.0 - short_side)
                        + max(0.0, short_side - 90.0)
                    )
                    score = (
                        range_penalty * 1000.0
                        + width * height
                        + abs(width - height) * 0.01
                    )
                    candidate = (
                        score, order, quarter_turns, rows, row_widths,
                        row_heights, variants, width, height,
                    )
                    if best is None or candidate[0] < best[0]:
                        best = candidate
        if best is None:
            raise RuntimeError("pieces do not fit a safe best-effort target layout")

        (_, order, quarter_turns, rows, row_widths, row_heights,
         variants, width, height) = best
        targets = [None] * count
        x_base = (210.0 - width) * 0.5
        y_cursor = TARGET_DIVIDER_Y_MM - TARGET_DIVIDER_GAP_MM - height
        for row_id, row in enumerate(rows):
            x_cursor = x_base + (width - row_widths[row_id]) * 0.5
            for piece_id in row:
                rotated, minimum, maximum = variants[piece_id]
                piece_width = float(maximum[0] - minimum[0])
                piece_height = float(maximum[1] - minimum[1])
                row_offset = (row_heights[row_id] - piece_height) * 0.5
                translation = np.asarray(
                    [x_cursor - minimum[0], y_cursor + row_offset - minimum[1]],
                    dtype=np.float32,
                )
                targets[piece_id] = rotated + translation
                x_cursor += piece_width + gap_mm
            y_cursor += row_heights[row_id] + gap_mm
        return targets, [90.0 * value for value in quarter_turns], {
            "best_effort_layout_score": round(float(best[0]), 3),
            "best_effort_rows": len(rows),
            "best_effort_columns": max(len(row) for row in rows),
        }

    def _run_fixed_color(self, image_path: str) -> Dict[str, object]:
        total_start = time.perf_counter()
        self.state = VisionState.CAPTURING
        load_start = time.perf_counter()
        frame = cv2.imread(image_path)
        if frame is None:
            raise RuntimeError("无法读取图片：{}".format(image_path))
        frame = resize_for_debug(frame)
        load_ms = (time.perf_counter() - load_start) * 1000.0

        self.state = VisionState.CALIBRATING
        vision_start = time.perf_counter()
        _, detection_annotated, detection = process_frame(
            frame,
            self.detect_config,
        )
        self.state = VisionState.DETECTING
        if len(detection["pieces"]) != 4:
            raise RuntimeError(
                "模式1要求4块碎片，当前识别到{}块".format(
                    len(detection["pieces"])
                )
            )
        vision_ms = (time.perf_counter() - vision_start) * 1000.0

        self.state = VisionState.SOLVING
        solve_start = time.perf_counter()
        self._report_progress(0.05, "match fixed template")
        polygons = [
            np.asarray(piece["vertices_mm"], dtype=np.float32)
            for piece in detection["pieces"]
        ]
        template_started = time.perf_counter()
        try:
            targets, rotations, metrics, state = self._solve_fixed_polygons(
                polygons
            )
        except RuntimeError:
            try:
                (
                    approximate_targets,
                    approximate_rotations,
                    approximate_metrics,
                    _,
                ) = self._solve_fixed_polygons(
                    polygons,
                    maximum_rms_mm=float("inf"),
                )
                self._remember_best_candidate(
                    detection,
                    detection_annotated,
                    approximate_targets,
                    approximate_rotations,
                    dict(
                        approximate_metrics,
                        partial_matched_piece_count=len(polygons),
                        partial_matched_seam_count=0,
                        task_mode=1.0,
                    ),
                    (
                        -len(polygons),
                        0.0,
                        float(
                            approximate_metrics.get(
                                "template_mean_rms_mm",
                                float("inf"),
                            )
                        ),
                    ),
                    "mode1_best_template_assignment",
                )
            except RuntimeError:
                pass
            raise
        template_match_ms = (
            time.perf_counter() - template_started
        ) * 1000.0
        self._report_progress(0.75, "use fixed assembly clearance")
        clearance_started = time.perf_counter()
        if metrics.get("template_target_mode") != "fixed_8mm_clearance":
            matches = self._infer_adjacency_matches(
                targets,
                existing_matches=state.matches,
            )
            metrics["fill_ratio_before_clearance"] = float(
                metrics["fill_ratio"]
            )
            targets, clearance_metrics = self._apply_target_clearance(
                targets,
                matches,
                target_origin=np.asarray(TARGET_ORIGIN_MM, dtype=np.float32),
                clearance_mm=TARGET_CLEARANCE_MM,
                size_tolerance_mm=0.0,
            )
            metrics.update(clearance_metrics)
        targets, placement_metrics = self._place_above_divider(targets)
        metrics.update(placement_metrics)
        metrics["workspace_source"] = "physical_right"
        metrics["workspace_target"] = "physical_left"
        target_clearance_ms = (
            time.perf_counter() - clearance_started
        ) * 1000.0
        solve_ms = (time.perf_counter() - solve_start) * 1000.0

        detection_image = self.output_directory / "fsm_detection.jpg"
        solution_image = self.output_directory / "fsm_solution.jpg"
        cv2.imwrite(str(detection_image), detection_annotated)
        render_solution(
            targets,
            rotations,
            metrics,
            str(solution_image),
        )
        self._report_progress(1.0, "Mode 1 solution ready")

        # 大片优先放置，最后放最小片，减少已放碎片被碰动的风险。
        order = sorted(
            range(len(polygons)),
            key=lambda piece_id: detection["pieces"][piece_id]["area_px2"],
            reverse=True,
        )
        actions = []
        for sequence, piece_id in enumerate(order):
            piece = detection["pieces"][piece_id]
            target_center = polygon_centroid(targets[piece_id])
            rotation_delta = self._normalize_angle(
                rotations[piece_id]
            )
            target_pickup = self._target_pickup_point(
                piece["pickup_mm"],
                piece["centroid_mm"],
                target_center,
                rotation_delta,
            )
            actions.append(
                MoveAction(
                    sequence=sequence,
                    piece_id=piece_id,
                    source_pickup_mm=[
                        round(float(value), 2)
                        for value in piece["pickup_mm"]
                    ],
                    source_centroid_mm=[
                        round(float(value), 2)
                        for value in piece["centroid_mm"]
                    ],
                    target_pickup_mm=[
                        round(float(value), 2)
                        for value in target_pickup
                    ],
                    target_centroid_mm=[
                        round(float(value), 2)
                        for value in target_center
                    ],
                    rotation_delta_deg=round(
                        rotation_delta,
                        2,
                    ),
                )
            )

        total_ms = (time.perf_counter() - total_start) * 1000.0
        return {
            "actions": actions,
            "metrics": metrics,
            "timing_ms": {
                "image_load": round(load_ms, 2),
                "vision": round(vision_ms, 2),
                "template_match": round(template_match_ms, 2),
                "target_clearance": round(target_clearance_ms, 2),
                "puzzle_solve": round(solve_ms, 2),
                "total": round(total_ms, 2),
            },
            "detection_image": str(detection_image),
            "solution_image": str(solution_image),
        }

    def _solve_fixed_polygons(self, polygons, maximum_rms_mm=8.0):
        if not self.template_path.is_file():
            raise RuntimeError(
                "missing Mode 1 template: {}".format(
                    self.template_path
                )
            )
        with self.template_path.open(
            "r",
            encoding="utf-8",
        ) as template_file:
            template_data = json.load(template_file)
        template_polygons = [
            np.asarray(
                piece["target_vertices_mm"],
                dtype=np.float32,
            )
            for piece in template_data.get("pieces", [])
        ]
        if len(template_polygons) != 4:
            raise RuntimeError(
                "Mode 1 template must contain four pieces"
            )
        state, metrics = solve_with_template(
            polygons,
            template_polygons,
            maximum_rms_mm=maximum_rms_mm,
        )
        assignment = [
            int(template_id)
            for template_id in metrics["template_assignment"]
        ]
        targets = [
            template_polygons[assignment[piece_id]].copy()
            for piece_id in range(len(polygons))
        ]
        rotations = [
            float(
                np.degrees(
                    np.arctan2(
                        state.transforms[piece_id][0][1, 0],
                        state.transforms[piece_id][0][0, 0],
                    )
                )
            )
            for piece_id in range(len(polygons))
        ]
        metrics["template_match_mode"] = "fixed_shape"
        # Legacy templates may contain precomputed clearance coordinates.
        # Always start from the gap-free template and apply the common
        # edge-to-edge clearance pass used by every mode.
        metrics["template_target_mode"] = "raw_template"
        return targets, rotations, metrics, state

    def _fixed_target_metrics(self, targets):
        targets = [np.asarray(target, dtype=np.float32) for target in targets]
        overlap_area = self._total_overlap_area(targets)
        matches = self._infer_adjacency_matches(targets, existing_matches=[])
        gaps = []
        for match in matches:
            first_id, _, second_id, _ = match[:4]
            first = targets[int(first_id)]
            second = targets[int(second_id)]
            gaps.append(self._polygon_distance(first, second))
        return {
            "overlap_area_mm2": round(float(overlap_area), 4),
            "target_clearance_mm": TARGET_CLEARANCE_MM,
            "clearance_basis": "adjacent_edge_normal_distance",
            "adjacency_gap_min_mm": round(min(gaps) if gaps else 0.0, 3),
            "adjacency_gap_max_mm": round(max(gaps) if gaps else 0.0, 3),
        }

    def _run_unknown_white(self, image_path: str) -> Dict[str, object]:
        """Mode 2: general geometry-only puzzle solving."""
        return self._run_geometry_puzzle(image_path, mode_number=2)

    def _run_card_geometry(self, image_path: str) -> Dict[str, object]:
        """Mode 3: solve from polygon geometry without card artwork."""
        return self._run_geometry_puzzle(image_path, mode_number=3)

    def _run_geometry_puzzle(
        self,
        image_path: str,
        mode_number: int,
    ) -> Dict[str, object]:
        """模式2：现场未知白色碎片，自动推断目标矩形宽高。"""
        total_start = time.perf_counter()
        self.state = VisionState.CAPTURING
        load_start = time.perf_counter()
        frame = cv2.imread(image_path)
        if frame is None:
            raise RuntimeError("无法读取图片：{}".format(image_path))
        frame = resize_for_debug(frame)
        load_ms = (time.perf_counter() - load_start) * 1000.0

        self.state = VisionState.CALIBRATING
        vision_start = time.perf_counter()
        geometry_detect_config = replace(
            self.detect_config,
            include_dark_artwork_in_piece_mask=(mode_number == 3),
        )
        _, detection_annotated, detection = process_frame(
            frame,
            geometry_detect_config,
        )
        self.state = VisionState.DETECTING
        piece_count = len(detection["pieces"])
        if not 2 <= piece_count <= 4:
            raise RuntimeError(
                "模式2要求2至4块碎片，当前识别到{}块".format(
                    piece_count
                )
            )
        for piece in detection["pieces"]:
            vertex_count = len(piece["vertices_mm"])
            if vertex_count > 5:
                candidates_mm = piece.get("polygon_candidates_mm", [])
                candidate_scores = piece.get("polygon_candidate_scores", [])
                legal_candidates = [
                    (candidate_id, candidate)
                    for candidate_id, candidate in enumerate(candidates_mm)
                    if 3 <= len(candidate) <= 5
                ]
                if not legal_candidates:
                    raise RuntimeError(
                        "Mode {} P{} detected {} edges and found no "
                        "reliable <=5-edge recovery".format(
                            mode_number,
                            piece["id"],
                            vertex_count,
                        )
                    )
                selected_id, selected = min(
                    legal_candidates,
                    key=lambda item: (
                        candidate_scores[item[0]]
                        if item[0] < len(candidate_scores)
                        else float("inf"),
                        len(item[1]),
                    ),
                )
                piece["original_vertex_count"] = int(vertex_count)
                piece["edge_notch_recovered"] = True
                piece["vertices_mm"] = selected
                candidates_px = piece.get("polygon_candidates_px", [])
                if selected_id < len(candidates_px):
                    piece["vertices_px"] = candidates_px[selected_id]
                selected_array = np.asarray(selected, dtype=np.float32)
                piece["edge_lengths_mm"] = np.linalg.norm(
                    np.roll(selected_array, -1, axis=0) - selected_array,
                    axis=1,
                ).tolist()
        vision_ms = (time.perf_counter() - vision_start) * 1000.0

        self.state = VisionState.SOLVING
        solve_start = time.perf_counter()
        self._report_progress(
            0.02,
            "prepare Mode {} geometry search".format(mode_number),
        )
        primary_polygons = [
            np.asarray(piece["vertices_mm"], dtype=np.float32)
            for piece in detection["pieces"]
        ]
        (
            all_pieces_high_confidence_rectangular,
            rectangular_source_boxes,
            rectangular_piece_metrics,
        ) = rectangular_piece_diagnostics(primary_polygons)
        polygon_models = [(-1, primary_polygons)]
        if mode_number == 3:
            # The rectified image is sampled at 0.25 mm/pixel.  Two frames
            # of the same stationary puzzle can therefore move individual
            # vertices by one or two pixels and reorder a symmetric beam
            # search.  Quantise shape coordinates around each centroid to a
            # stable 0.5 mm grid as a strict (same-vertex-count) fallback.
            quantized_polygons = []
            for polygon in primary_polygons:
                center = np.mean(polygon, axis=0)
                quantized = (
                    np.round((polygon - center) / 0.5) * 0.5 + center
                )
                quantized_polygons.append(
                    quantized.astype(np.float32)
                )
            polygon_models.append((-2, quantized_polygons))
        for piece_id, piece in enumerate(detection["pieces"]):
            alternatives = piece.get("polygon_candidates_mm", [])
            if len(alternatives) < 2:
                continue
            if (
                mode_number == 3
                and len(alternatives[1])
                != len(primary_polygons[piece_id])
            ):
                # A rounded corner or residual print notch may create a
                # different-vertex alternate.  Mode 3 must not turn a real
                # triangle into a quadrilateral merely to obtain an easier
                # rectangular search state.
                continue
            alternate_polygons = list(primary_polygons)
            alternate_polygons[piece_id] = np.asarray(
                alternatives[1],
                dtype=np.float32,
            )
            polygon_models.append((piece_id, alternate_polygons))

        polygons = primary_polygons
        search_errors = []
        search_attempts = []
        model_count = len(polygon_models)
        # Mode 3 playing-card geometry contains repeated isosceles edges.
        # Keep more symmetric branches and accept the measured 84+ mm card
        # length. Modes 2 and 3 share the same final rectangle validation;
        # their candidate-search schedules remain intentionally different.
        beam_levels = (
            self.mode3_beam_levels
            if mode_number == 3
            else (40, 80)
        )
        maximum_corner_error = 5.0
        corner_search_ms = 0.0
        full_edge_search_ms = 0.0
        partition_search_ms = 0.0
        clearance_validation_ms = 0.0
        long_side_range = (
            (84.0, 120.0) if mode_number == 3 else (90.0, 120.0)
        )
        selected = None
        if (
            mode_number == 2
            and piece_count == 4
            and all_pieces_high_confidence_rectangular
        ):
            rectangle_search_started = time.perf_counter()
            rectangle_layouts = enumerate_rectangular_piece_layouts(
                rectangular_source_boxes,
                long_side_range_mm=long_side_range,
                short_side_range_mm=(50.0, 90.0),
            )
            rectangle_search_ms = (
                time.perf_counter() - rectangle_search_started
            ) * 1000.0
            full_edge_search_ms += rectangle_search_ms
            search_attempts.append(
                {
                    "model": 0,
                    "route": "all_rectangular_layout",
                    "beam": 0,
                    "time_ms": round(rectangle_search_ms, 2),
                    "result": (
                        "geometry_candidates"
                        if rectangle_layouts
                        else "no_geometry"
                    ),
                }
            )
            validation_started = time.perf_counter()
            all_piece_pairs = tuple(
                (first_id, 0, second_id, 0)
                for first_id in range(piece_count)
                for second_id in range(first_id + 1, piece_count)
            )
            for candidate_id, layout in enumerate(rectangle_layouts):
                candidate_targets = [
                    np.asarray(target, dtype=np.float32)
                    for target in layout["targets"]
                ]
                matches = self._infer_adjacency_matches(
                    candidate_targets,
                    existing_matches=all_piece_pairs,
                )
                matched_piece_ids = {
                    int(piece_id)
                    for match in matches
                    for piece_id in (match[0], match[2])
                }
                self._remember_best_candidate(
                    detection,
                    detection_annotated,
                    candidate_targets,
                    layout["rotations"],
                    {
                        "score": float(layout["join_error_mm"]),
                        "partial_matched_piece_count": len(matched_piece_ids),
                        "partial_matched_seam_count": len(matches),
                        "task_mode": 2.0,
                    },
                    (
                        -len(matched_piece_ids),
                        -len(matches),
                        float(layout["join_error_mm"]),
                        candidate_id,
                    ),
                    "mode2_all_rectangular_layout",
                    matches=matches,
                )
                try:
                    candidate_targets, clearance_metrics = (
                        self._apply_target_clearance(
                            candidate_targets,
                            matches,
                            target_origin=np.asarray(
                                TARGET_ORIGIN_MM,
                                dtype=np.float32,
                            ),
                            clearance_mm=TARGET_CLEARANCE_MM,
                            size_tolerance_mm=0.0,
                            long_side_range_mm=long_side_range,
                            short_side_range_mm=(50.0, 90.0),
                        )
                    )
                except RuntimeError as error:
                    search_errors.append(
                        "all-rectangular layout {}: {}".format(
                            candidate_id,
                            error,
                        )
                    )
                    continue
                metrics = {
                    "score": float(layout["join_error_mm"]),
                    "strategy": "mode2_all_rectangular_layout",
                    "search_route": "all_rectangular_layout",
                    "task_mode": 2.0,
                    "beam_width": 0.0,
                    "partition_beam_width": 0.0,
                    "corner_beam_width": 0.0,
                    "corner_driven": False,
                    "all_pieces_near_rectangular": True,
                    "all_pieces_high_confidence_rectangular": True,
                    "rectangular_piece_diagnostics": (
                        rectangular_piece_metrics
                    ),
                    "rectangular_layout_candidate_count": float(
                        len(rectangle_layouts)
                    ),
                    "rectangular_layout_candidate_rank": float(candidate_id),
                    "rectangular_layout_join_error_mm": float(
                        layout["join_error_mm"]
                    ),
                    "rectangular_layout_fit_error_mm": float(
                        layout["fit_error_mm"]
                    ),
                    "search_attempt_count": float(len(search_attempts)),
                    "search_attempts": list(search_attempts),
                    "solver_workers": float(self.solver_workers),
                    "native_acceleration": bool(
                        native_acceleration_available()
                    ),
                }
                metrics.update(clearance_metrics)
                selected = (
                    None,
                    metrics,
                    candidate_targets,
                    list(layout["rotations"]),
                )
                polygons = rectangular_source_boxes
                break
            clearance_validation_ms += (
                time.perf_counter() - validation_started
            ) * 1000.0

        def partition_evidence(candidate_polygons):
            edges = []
            for piece_id, polygon in enumerate(candidate_polygons):
                lengths = np.linalg.norm(
                    np.roll(polygon, -1, axis=0) - polygon,
                    axis=1,
                )
                edges.extend(
                    (piece_id, float(length)) for length in lengths
                )
            evidence = []
            for long_id, (long_piece, long_length) in enumerate(edges):
                available = [
                    edge
                    for edge_id, edge in enumerate(edges)
                    if edge_id != long_id and edge[0] != long_piece
                ]
                for short_edges in combinations(available, 2):
                    if short_edges[0][0] == short_edges[1][0]:
                        continue
                    short_lengths = [edge[1] for edge in short_edges]
                    if long_length < max(short_lengths) * 1.35:
                        continue
                    relative_error = abs(
                        sum(short_lengths) - long_length
                    ) / max(1e-6, long_length)
                    if relative_error <= 0.08:
                        evidence.append(relative_error)
            return len(evidence), min(evidence) if evidence else 1.0

        model_specs = []
        for model_id, (alternate_piece_id, candidate_polygons) in enumerate(
            polygon_models
        ):
            evidence_count, evidence_error = partition_evidence(
                candidate_polygons
            )
            # The primary contour follows the measured shape and can benefit
            # from explicit long-edge partitions.  Alternate contours are
            # already repaired/simplified models, so try their direct edge
            # matches first instead of spending a high beam on partitions.
            route_order = (
                (True, False)
                if model_id == 0 and evidence_count
                else (False, True)
            )
            model_specs.append(
                (
                    model_id,
                    alternate_piece_id,
                    candidate_polygons,
                    evidence_count,
                    evidence_error,
                    route_order,
                )
            )

        search_schedule = []
        # A corner endpoint constrains both the seam ray and its neighbouring
        # boundary ray. Try this smaller proposal set before the normal edge
        # Beam. Edge-interior T-junctions remain on the established fallback.
        all_pieces_near_rectangular = bool(
            primary_polygons
            and all(
                len(polygon) == 4
                and float(
                    np.max(np.abs(interior_angles(polygon) - 90.0))
                )
                <= MODE3_RECTANGULAR_PIECE_ANGLE_TOLERANCE_DEG
                for polygon in primary_polygons
            )
        )
        corner_fast_path_allowed = not (
            mode_number == 3 and all_pieces_near_rectangular
        )
        corner_model_specs = (
            (model_specs[:1] if mode_number == 3 else model_specs)
            if corner_fast_path_allowed
            else []
        )
        for model_spec in corner_model_specs:
            search_schedule.append(
                model_spec[:5]
                + (
                    model_spec[5][0],
                    MODE23_CORNER_BEAM_WIDTH,
                    False,
                    True,
                )
            )
        # Exhaust every contour model at the cheap beam before any model is
        # allowed to escalate.  This prevents one difficult primary contour
        # from consuming several seconds while a repaired alternate has a
        # direct low-beam solution.
        mode3_fast_beam_eligible = bool(
            mode_number == 3
            and model_specs
            and model_specs[0][3]
            >= MODE3_FAST_BEAM_MIN_PARTITION_EVIDENCE
        )
        for beam_width in beam_levels:
            if mode_number == 3 and beam_width < 400:
                # Small beams are a single cheap fast path, not a complete
                # traversal of every contour model and route.  Strong
                # partition evidence identifies the cases where this path
                # historically preserves the beam=400 result.  Otherwise
                # skip directly to the established Mode 3 schedule.
                if not mode3_fast_beam_eligible:
                    continue
                model_spec = model_specs[0]
                search_schedule.append(
                    model_spec[:5]
                    + (model_spec[5][0], beam_width, False, False)
                )
                continue
            for model_spec in model_specs:
                for allow_partitioned in model_spec[5]:
                    search_schedule.append(
                        model_spec[:5]
                        + (allow_partitioned, beam_width, False, False)
                    )
        for schedule_id, (
            model_id,
            alternate_piece_id,
            candidate_polygons,
            evidence_count,
            evidence_error,
            allow_partitioned,
            beam_width,
            rounded_outer_fallback,
            corner_driven,
        ) in enumerate(search_schedule):
            if selected is not None:
                break
            def model_progress(local_value, message):
                self._report_progress(
                    0.03
                    + 0.88
                    * (schedule_id + local_value)
                    / max(1, len(search_schedule)),
                    "model {}/{}: {}".format(
                        model_id + 1,
                        model_count,
                        message,
                    ),
                )

            if corner_driven:
                route_name = (
                    "corner_pairs_partitioned"
                    if allow_partitioned
                    else "corner_pairs_full"
                )
            else:
                route_name = (
                    "partitioned_edges"
                    if allow_partitioned
                    else "full_edges"
                )
            if rounded_outer_fallback:
                route_name += "_rounded_outer"
            model_progress(
                0.05,
                "{} beam={}".format(route_name, beam_width),
            )
            search_started = time.perf_counter()
            try:
                current_candidates = rank_unknown_rectangle_candidates(
                    candidate_polygons,
                    edge_tolerance=0.18,
                    beam_width=beam_width,
                    time_limit_seconds=30.0,
                    maximum_results=(48 if mode_number == 3 else 24),
                    allow_partitioned_edges=allow_partitioned,
                    maximum_outer_corner_error_deg=maximum_corner_error,
                    strong_rectangle_outer_corner_error_deg=(
                        MODE23_STRONG_RECTANGLE_MAX_OUTER_CORNER_ERROR_DEG
                        if mode_number in (2, 3)
                        else None
                    ),
                    maximum_outer_corner_offset_mm=(
                        CORNER_FAST_MAX_OUTER_CORNER_OFFSET_MM
                        if corner_driven
                        else MODE3_MAX_OUTER_CORNER_OFFSET_MM
                        if mode_number == 3
                        else MODE2_MAX_OUTER_CORNER_OFFSET_MM
                    ),
                    maximum_outer_vertices=8,
                    minimum_measured_fill_ratio=0.92,
                    long_side_range_mm=long_side_range,
                    overlap_workers=self.solver_workers,
                    corner_driven=corner_driven,
                    corner_tolerance_deg=(
                        MODE3_CORNER_TOLERANCE_DEG
                        if mode_number == 3
                        else MODE2_CORNER_TOLERANCE_DEG
                    ),
                    allow_gapped_rectangle_edges=(mode_number in (2, 3)),
                    progress_callback=lambda value, message: model_progress(
                        0.08 + 0.84 * value,
                        "{} beam={}: {}".format(
                            route_name,
                            beam_width,
                            message,
                        ),
                    ),
                )
                search_ms = (
                    time.perf_counter() - search_started
                ) * 1000.0
                if corner_driven:
                    corner_search_ms += search_ms
                elif allow_partitioned:
                    partition_search_ms += search_ms
                else:
                    full_edge_search_ms += search_ms
                search_attempts.append(
                    {
                        "model": model_id + 1,
                        "route": route_name,
                        "beam": beam_width,
                        "time_ms": round(search_ms, 2),
                        "result": "geometry_candidates",
                    }
                )
            except RuntimeError as error:
                search_ms = (
                    time.perf_counter() - search_started
                ) * 1000.0
                if corner_driven:
                    corner_search_ms += search_ms
                elif allow_partitioned:
                    partition_search_ms += search_ms
                else:
                    full_edge_search_ms += search_ms
                detail = str(error)
                search_errors.append(
                    "model {} {} beam={}: {}".format(
                        model_id + 1,
                        route_name,
                        beam_width,
                        detail,
                    )
                )
                search_attempts.append(
                    {
                        "model": model_id + 1,
                        "route": route_name,
                        "beam": beam_width,
                        "time_ms": round(search_ms, 2),
                        "result": "no_geometry",
                    }
                )
                continue

            validation_started = time.perf_counter()
            rejected = []
            for candidate_id, (state, metrics) in enumerate(
                current_candidates
            ):
                candidate_targets, candidate_rotations = normalize_solution(
                    candidate_polygons,
                    state,
                    target_origin_mm=TARGET_ORIGIN_MM,
                )
                matches = self._infer_adjacency_matches(
                    candidate_targets,
                    existing_matches=state.matches,
                )
                matched_piece_ids = {
                    int(piece_id)
                    for match in matches
                    for piece_id in (match[0], match[2])
                }
                self._remember_best_candidate(
                    detection,
                    detection_annotated,
                    candidate_targets,
                    candidate_rotations,
                    dict(
                        metrics,
                        partial_matched_piece_count=len(matched_piece_ids),
                        partial_matched_seam_count=len(matches),
                        task_mode=float(mode_number),
                    ),
                    (
                        -len(matched_piece_ids),
                        -len(matches),
                        float(metrics.get("score", float("inf"))),
                        candidate_id,
                    ),
                    "mode{}_ranked_geometry".format(mode_number),
                    matches=matches,
                )
                try:
                    candidate_targets, clearance_metrics = self._apply_target_clearance(
                        candidate_targets,
                        matches,
                        target_origin=np.asarray(
                            TARGET_ORIGIN_MM,
                            dtype=np.float32,
                        ),
                        clearance_mm=TARGET_CLEARANCE_MM,
                        size_tolerance_mm=0.0,
                        long_side_range_mm=long_side_range,
                        short_side_range_mm=(50.0, 90.0),
                    )
                except RuntimeError as error:
                    rejected.append(str(error))
                    continue
                metrics.update(clearance_metrics)
                if (
                    corner_driven
                    and float(metrics["score"])
                    > MODE23_CORNER_MAX_SCORE
                ):
                    rejected.append(
                        "corner score {:.4f} exceeds {:.4f}".format(
                            float(metrics["score"]),
                            MODE23_CORNER_MAX_SCORE,
                        )
                    )
                    continue
                if (
                    mode_number == 3
                    and beam_width < 400
                    and not corner_driven
                    and float(metrics["score"])
                    > MODE3_FAST_BEAM_MAX_SCORE
                ):
                    rejected.append(
                        "fast-beam score {:.4f} exceeds {:.4f}".format(
                            float(metrics["score"]),
                            MODE3_FAST_BEAM_MAX_SCORE,
                        )
                    )
                    continue
                metrics["beam_width"] = float(beam_width)
                metrics["partition_beam_width"] = float(
                    beam_width
                    if allow_partitioned and not corner_driven
                    else 0
                )
                metrics["search_route"] = route_name
                metrics["corner_driven"] = bool(corner_driven)
                metrics["corner_beam_width"] = float(
                    beam_width if corner_driven else 0
                )
                metrics["corner_max_score"] = float(
                    MODE23_CORNER_MAX_SCORE
                )
                metrics["corner_fast_path_allowed"] = bool(
                    corner_fast_path_allowed
                )
                metrics["all_pieces_near_rectangular"] = bool(
                    all_pieces_near_rectangular
                )
                metrics["rectangular_piece_angle_tolerance_deg"] = float(
                    MODE3_RECTANGULAR_PIECE_ANGLE_TOLERANCE_DEG
                )
                metrics["rounded_outer_fallback"] = bool(
                    rounded_outer_fallback
                )
                metrics["search_attempt_count"] = float(
                    len(search_attempts)
                )
                metrics["search_attempts"] = list(search_attempts)
                metrics["partition_evidence_count"] = float(
                    evidence_count
                )
                metrics["partition_evidence_best_error"] = float(
                    evidence_error
                )
                metrics["contour_model_count"] = float(model_count)
                metrics["selected_contour_model"] = float(model_id)
                metrics["alternate_contour_piece_id"] = float(
                    alternate_piece_id
                )
                metrics["selected_contour_model_kind"] = (
                    "primary"
                    if alternate_piece_id == -1
                    else "quantized_0.5mm"
                    if alternate_piece_id == -2
                    else "piece_alternate"
                )
                metrics["clearance_candidate_rank"] = float(candidate_id)
                metrics["task_mode"] = float(mode_number)
                metrics["solver_workers"] = float(self.solver_workers)
                metrics["beam_levels_enabled"] = list(beam_levels)
                metrics["mode3_high_beam_enabled"] = bool(
                    mode_number == 3 and max(beam_levels) > 400
                )
                metrics["mode3_fast_beam_eligible"] = bool(
                    mode3_fast_beam_eligible
                )
                metrics["mode3_fast_beam_min_partition_evidence"] = float(
                    MODE3_FAST_BEAM_MIN_PARTITION_EVIDENCE
                )
                metrics["mode3_fast_beam_max_score"] = float(
                    MODE3_FAST_BEAM_MAX_SCORE
                )
                metrics["native_acceleration"] = bool(
                    native_acceleration_available()
                )
                if mode_number == 3:
                    metrics["strategy"] = "mode3_geometry_only"
                selected = (
                    state,
                    metrics,
                    candidate_targets,
                    candidate_rotations,
                )
                polygons = candidate_polygons
                break
            clearance_validation_ms += (
                time.perf_counter() - validation_started
            ) * 1000.0
            if selected is not None:
                break
            search_errors.append(
                "model {} {} beam={} clearance: {}".format(
                    model_id + 1,
                    route_name,
                    beam_width,
                    rejected[0] if rejected else "no candidate",
                )
            )

        if selected is None:
            detail = search_errors[-1] if search_errors else "no search attempt"
            raise RuntimeError(
                "Mode {} found no final-safe solution after {} attempts; {}"
                .format(mode_number, len(search_attempts), detail)
            )
        state, metrics, targets, rotations = selected
        targets, placement_metrics = self._place_above_divider(targets)
        metrics.update(placement_metrics)
        metrics["workspace_source"] = "physical_right"
        metrics["workspace_target"] = "physical_left"
        solve_ms = (time.perf_counter() - solve_start) * 1000.0

        detection_image = (
            self.output_directory
            / "fsm_mode{}_detection.jpg".format(mode_number)
        )
        solution_image = (
            self.output_directory
            / "fsm_mode{}_solution.jpg".format(mode_number)
        )
        cv2.imwrite(str(detection_image), detection_annotated)
        render_solution(
            targets,
            rotations,
            metrics,
            str(solution_image),
        )
        self._report_progress(
            1.0,
            "Mode {} geometry solution ready".format(mode_number),
        )

        order = sorted(
            range(len(polygons)),
            key=lambda piece_id: detection["pieces"][piece_id]["area_px2"],
            reverse=True,
        )
        actions = []
        for sequence, piece_id in enumerate(order):
            piece = detection["pieces"][piece_id]
            target_center = polygon_centroid(targets[piece_id])
            rotation_delta = self._normalize_angle(
                rotations[piece_id]
            )
            target_pickup = self._target_pickup_point(
                piece["pickup_mm"],
                piece["centroid_mm"],
                target_center,
                rotation_delta,
            )
            actions.append(
                MoveAction(
                    sequence=sequence,
                    piece_id=piece_id,
                    source_pickup_mm=[
                        round(float(value), 2)
                        for value in piece["pickup_mm"]
                    ],
                    source_centroid_mm=[
                        round(float(value), 2)
                        for value in piece["centroid_mm"]
                    ],
                    target_pickup_mm=[
                        round(float(value), 2)
                        for value in target_pickup
                    ],
                    target_centroid_mm=[
                        round(float(value), 2)
                        for value in target_center
                    ],
                    rotation_delta_deg=round(
                        rotation_delta,
                        2,
                    ),
                )
            )

        total_ms = (time.perf_counter() - total_start) * 1000.0
        return {
            "actions": actions,
            "metrics": metrics,
            "timing_ms": {
                "image_load": round(load_ms, 2),
                "vision": round(vision_ms, 2),
                "corner_search": round(corner_search_ms, 2),
                "full_edge_search": round(full_edge_search_ms, 2),
                "partition_search": round(partition_search_ms, 2),
                "clearance_validation": round(clearance_validation_ms, 2),
                "puzzle_solve": round(solve_ms, 2),
                "total": round(total_ms, 2),
            },
            "detection_image": str(detection_image),
            "solution_image": str(solution_image),
        }

    def _run_card_artwork(self, image_path: str) -> Dict[str, object]:
        """Mode 4: rectangular pieces selected by card-pixel constraints."""
        total_start = time.perf_counter()
        self.state = VisionState.CAPTURING
        load_start = time.perf_counter()
        frame = cv2.imread(image_path)
        if frame is None:
            raise RuntimeError("cannot read image: {}".format(image_path))
        frame = resize_for_debug(frame)
        load_ms = (time.perf_counter() - load_start) * 1000.0

        self.state = VisionState.CALIBRATING
        vision_start = time.perf_counter()
        rectified, calibration = rectify_a4(frame, self.detect_config)
        _, pieces, background_hue = detect_pieces(
            rectified,
            self.detect_config,
        )
        piece_count = len(pieces)
        if not 2 <= piece_count <= 4:
            raise RuntimeError(
                "Mode 4 requires 2 to 4 pieces; detected {}".format(
                    piece_count
                )
            )
        polygons, rectangle_diagnostics = regularize_rectangular_pieces(
            pieces
        )
        detection_annotated = draw_results(
            rectified,
            pieces,
            background_hue,
        )
        detection = make_payload(
            rectified,
            pieces,
            background_hue,
            calibration,
        )
        vision_ms = (time.perf_counter() - vision_start) * 1000.0

        self.state = VisionState.SOLVING
        solve_start = time.perf_counter()
        target_origin = np.asarray(TARGET_ORIGIN_MM, dtype=np.float32)
        self._report_progress(0.03, "Mode 4 rectangle geometry search")
        search_start = time.perf_counter()
        raw_states = generate_edge_search_states(
            polygons,
            edge_tolerance=0.18,
            # Piece labels must remain distinct.  Geometry-only deduplication
            # would collapse permutations of four equal rectangles, even
            # though their printed artwork is different.
            beam_width=1200,
            allow_partitioned_edges=True,
            deduplicate_states=False,
            overlap_workers=self.solver_workers,
            progress_callback=lambda value, message: self._report_progress(
                0.04 + 0.43 * value,
                "Mode 4 geometry: " + message,
            ),
        )
        geometry_search_ms = (time.perf_counter() - search_start) * 1000.0

        filter_start = time.perf_counter()
        geometry_candidates = []
        signatures = set()
        for state in raw_states:
            geometry_score, metrics = unknown_rectangle_score(polygons, state)
            if (
                metrics["connected_ratio"] < 0.98
                or not 0.90 <= metrics["fill_ratio"] <= 1.06
                or metrics["dimension_error"] > 0.05
                or metrics["outer_vertices"] > 8.0
                or metrics["outer_corner_max_error_deg"] > 5.0
                # Mode 4 first verifies every source piece as a rectangle;
                # allow a little more accumulated perspective/box-fit error
                # at the assembled outer corners.
                or metrics["outer_corner_max_offset_mm"] > 3.5
            ):
                continue
            targets, rotations = normalize_solution(
                polygons,
                state,
                target_origin_mm=TARGET_ORIGIN_MM,
            )
            provisional_overlap = self._total_overlap_area(targets)
            if provisional_overlap > MODE4_PROVISIONAL_OVERLAP_MAX_MM2:
                continue
            centroids = np.asarray(
                [polygon_centroid(target) for target in targets],
                dtype=np.float32,
            )
            position_signature = tuple(
                np.round((centroids - target_origin) * 2.0)
                .astype(np.int32)
                .reshape(-1)
                .tolist()
            )
            # A rectangle at the same centroid can still be horizontal or
            # vertical.  The artwork pass only enumerates additional 180°
            # turns, so preserve distinct rotations modulo 180° here.
            orientation_signature = tuple(
                int(round((float(rotation) % 180.0) / 2.0))
                for rotation in rotations
            )
            signature = position_signature + orientation_signature
            if signature in signatures:
                continue
            signatures.add(signature)
            geometry_candidates.append(
                (
                    float(geometry_score),
                    state,
                    dict(metrics),
                    targets,
                    rotations,
                    float(provisional_overlap),
                )
            )
        geometry_candidates.sort(key=lambda item: item[0])
        # Geometry states are only unlabeled rectangle-slot layouts.  Expand
        # every compatible piece permutation explicitly so artwork-correct
        # arrangements cannot be lost through geometry beam pruning.
        unique_slot_layouts = []
        slot_layout_signatures = set()
        for candidate in geometry_candidates:
            slot_descriptors = []
            all_slot_points = np.concatenate(candidate[3], axis=0)
            slot_origin = np.min(all_slot_points, axis=0)
            for slot in candidate[3]:
                center = polygon_centroid(slot) - slot_origin
                lengths = np.linalg.norm(
                    np.roll(slot, -1, axis=0) - slot,
                    axis=1,
                )
                short_side, long_side = sorted(
                    (float(np.min(lengths)), float(np.max(lengths)))
                )
                slot_descriptors.append(
                    (
                        int(round(float(center[0]) * 2.0)),
                        int(round(float(center[1]) * 2.0)),
                        int(round(short_side * 2.0)),
                        int(round(long_side * 2.0)),
                    )
                )
            layout_signature = tuple(sorted(slot_descriptors))
            if layout_signature in slot_layout_signatures:
                continue
            slot_layout_signatures.add(layout_signature)
            unique_slot_layouts.append(candidate)
            if len(unique_slot_layouts) >= MODE4_MAX_GEOMETRY_CANDIDATES:
                break

        expanded_candidates = []
        expanded_signatures = set()
        rectangle_sizes = []
        for polygon in polygons:
            edge_lengths = np.linalg.norm(
                np.roll(polygon, -1, axis=0) - polygon,
                axis=1,
            )
            rectangle_sizes.append(
                (float(np.min(edge_lengths)), float(np.max(edge_lengths)))
            )
        equal_quarters = bool(
            piece_count == 4
            and max(size[0] for size in rectangle_sizes)
            <= min(size[0] for size in rectangle_sizes) * 1.12
            and max(size[1] for size in rectangle_sizes)
            <= min(size[1] for size in rectangle_sizes) * 1.12
        )
        assignment_limit = 24 if equal_quarters else MODE4_MAX_ASSIGNMENT_CANDIDATES
        for candidate in unique_slot_layouts:
            candidate_slots = candidate[3]
            for assigned_targets, assigned_rotations, assignment in (
                enumerate_rectangular_slot_assignments(
                    polygons,
                    candidate_slots,
                )
            ):
                centers = tuple(
                    tuple(
                        np.round(polygon_centroid(target) * 2.0)
                        .astype(np.int32)
                        .tolist()
                    )
                    for target in assigned_targets
                )
                assignment_signature = centers + tuple(
                    int(round((rotation % 180.0) / 2.0))
                    for rotation in assigned_rotations
                )
                if assignment_signature in expanded_signatures:
                    continue
                expanded_signatures.add(assignment_signature)
                expanded_candidates.append(
                    (
                        candidate[0],
                        candidate[1],
                        candidate[2],
                        assigned_targets,
                        assigned_rotations,
                        candidate[5],
                        assignment,
                    )
                )
                if (
                    len(expanded_candidates)
                    >= assignment_limit
                ):
                    break
            if len(expanded_candidates) >= assignment_limit:
                break
        geometry_candidates = expanded_candidates
        geometry_filter_ms = (time.perf_counter() - filter_start) * 1000.0
        if not geometry_candidates:
            raise RuntimeError(
                "Mode 4 found no rectangular geometry satisfying real "
                "outer-corner constraints"
            )

        # First anchor the two legal rank/suit corners (top-right and
        # bottom-left).  This low-resolution pass intentionally omits the
        # body artwork so only a small set reaches the more expensive axis
        # symmetry prefilter.
        corner_anchor_start = time.perf_counter()
        corner_anchored = []
        corner_warp_cache = {}
        variants = tuple(half_turn_variants(piece_count))
        total_scores = len(geometry_candidates) * len(variants)
        corner_jobs = [
            (geometry_candidate, half_turns)
            for geometry_candidate in geometry_candidates
            for half_turns in variants
        ]

        def score_corner_anchor(job):
            geometry_candidate, half_turns = job
            corner_details = score_rectangular_card_corner_anchor(
                rectified,
                pieces,
                geometry_candidate[3],
                half_turns,
                warp_cache=corner_warp_cache,
            )
            return (
                float(corner_details["score"]),
                geometry_candidate,
                half_turns,
                corner_details,
            )

        corner_results = self._map_solver_jobs(
            score_corner_anchor,
            corner_jobs,
        )
        for completed_scores, result in enumerate(corner_results, start=1):
            if completed_scores % max(1, total_scores // 30) == 0:
                self._report_progress(
                    0.48 + 0.25 * completed_scores / total_scores,
                    "Mode 4 corner anchor {}/{}".format(
                        completed_scores,
                        total_scores,
                    ),
                )
            corner_anchored.append(result)
        corner_anchored.sort(key=lambda item: item[0])
        corner_anchored = [
            (
                score,
                geometry_candidate,
                half_turns,
                dict(corner_details, anchor_rank=rank),
            )
            for rank, (
                score,
                geometry_candidate,
                half_turns,
                corner_details,
            ) in enumerate(corner_anchored)
        ]
        corner_anchor_ms = (
            time.perf_counter() - corner_anchor_start
        ) * 1000.0

        def axis_prefilter(anchor_candidates):
            axis_warp_cache = {}

            def score_axis(candidate):
                (
                    anchor_score,
                    geometry_candidate,
                    half_turns,
                    corner_details,
                ) = candidate
                quick_details = score_rectangular_card_prefilter(
                    rectified,
                    pieces,
                    geometry_candidate[3],
                    half_turns,
                    warp_cache=axis_warp_cache,
                )
                quick_details["corner_anchor_score"] = float(anchor_score)
                quick_details["corner_anchor_overlap"] = float(
                    corner_details["corner_180_overlap"]
                )
                quick_details["corner_anchor_rank"] = int(
                    corner_details["anchor_rank"]
                )
                return (
                    float(quick_details["score"]),
                    geometry_candidate,
                    half_turns,
                    quick_details,
                )

            ranked = self._map_solver_jobs(score_axis, anchor_candidates)
            # Keep two complementary shortlists in the same 16-item batch:
            # the full two-axis prefilter catches balanced artwork, while the
            # long-axis ordering preserves the strongest playing-card body
            # cue when lighting weakens the perpendicular reflection.  This
            # avoids stopping on the first valid-looking but wrong diagonal.
            score_ranked = sorted(ranked, key=lambda item: item[0])
            long_axis_ranked = sorted(
                ranked,
                key=lambda item: (
                    -float(item[3].get("pattern_long_axis_overlap", 0.0)),
                    item[0],
                ),
            )
            merged_ranked = []
            seen = set()
            for rank in range(len(ranked)):
                for item in (score_ranked[rank], long_axis_ranked[rank]):
                    item_id = id(item)
                    if item_id in seen:
                        continue
                    seen.add(item_id)
                    merged_ranked.append(item)
            return merged_ranked

        primary_anchor_count = min(
            len(corner_anchored),
            MODE4_CORNER_ANCHOR_LIMIT,
        )
        prefilter_start = time.perf_counter()
        prefiltered = axis_prefilter(
            corner_anchored[:primary_anchor_count]
        )
        artwork_prefilter_ms = (
            time.perf_counter() - prefilter_start
        ) * 1000.0

        def invalid_candidate_key(candidate):
            if equal_quarters:
                return (
                    -float(candidate[8].get("pattern_biaxial_overlap", 0.0)),
                    float(candidate[0]),
                )
            return (float(candidate[0]),)

        def full_artwork_candidates(ranked_candidates):
            valid_candidates = []
            local_best_invalid = None
            completed = 0
            evaluated_until = 0
            started = time.perf_counter()
            full_warp_cache = {}

            def score_full_job(job):
                prefilter_rank, ranked_candidate = job
                (
                    prefilter_score,
                    geometry_candidate,
                    half_turns,
                    quick_details,
                ) = ranked_candidate
                (
                    geometry_score,
                    state,
                    metrics,
                    targets,
                    rotations,
                    provisional_overlap,
                    assignment,
                ) = geometry_candidate
                details = score_rectangular_card_artwork(
                    rectified,
                    pieces,
                    targets,
                    half_turns,
                    warp_cache=full_warp_cache,
                )
                details["prefilter_score"] = float(prefilter_score)
                details["prefilter_rank"] = float(prefilter_rank)
                details["prefilter_corner_180_overlap"] = float(
                    quick_details["corner_180_overlap"]
                )
                details["prefilter_pattern_axis_overlap"] = float(
                    quick_details["pattern_axis_overlap"]
                )
                details["prefilter_pattern_biaxial_overlap"] = float(
                    quick_details["pattern_biaxial_overlap"]
                )
                details["corner_anchor_score"] = float(
                    quick_details["corner_anchor_score"]
                )
                details["corner_anchor_rank"] = int(
                    quick_details["corner_anchor_rank"]
                )
                combined_score = (
                    float(details["score"])
                    + 0.04 * geometry_score
                    + 0.08 * provisional_overlap
                )
                return (
                    combined_score,
                    geometry_score,
                    state,
                    metrics,
                    targets,
                    rotations,
                    provisional_overlap,
                    half_turns,
                    details,
                    assignment,
                )

            # Normally the first corner/axis-ranked batch contains the
            # correct arrangement.  Continue batch by batch when needed.
            while (
                evaluated_until < len(ranked_candidates)
                and not valid_candidates
            ):
                batch_end = min(
                    len(ranked_candidates),
                    evaluated_until + MODE4_PREFILTER_BATCH_SIZE,
                )
                batch_jobs = [
                    (prefilter_rank, ranked_candidates[prefilter_rank])
                    for prefilter_rank in range(
                        evaluated_until,
                        batch_end,
                    )
                ]
                batch_candidates = self._map_solver_jobs(
                    score_full_job,
                    batch_jobs,
                )
                completed += len(batch_candidates)
                for candidate in batch_candidates:
                    details = candidate[8]
                    if (
                        local_best_invalid is None
                        or invalid_candidate_key(candidate)
                        < invalid_candidate_key(local_best_invalid)
                    ):
                        local_best_invalid = candidate
                    if details["valid"]:
                        valid_candidates.append(candidate)
                evaluated_until = batch_end
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            return valid_candidates, local_best_invalid, completed, elapsed_ms

        (
            artwork_candidates,
            best_invalid,
            completed_full_scores,
            artwork_score_ms,
        ) = full_artwork_candidates(prefiltered)
        corner_anchor_fallback = False
        # Ambiguous or damaged indices must not turn the optimisation into a
        # false rejection.  Only if the anchored shortlist fails, rank and
        # validate every remaining candidate with the original rules.
        if (
            not artwork_candidates
            and primary_anchor_count < len(corner_anchored)
        ):
            corner_anchor_fallback = True
            fallback_prefilter_start = time.perf_counter()
            fallback_prefiltered = axis_prefilter(
                corner_anchored[primary_anchor_count:]
            )
            artwork_prefilter_ms += (
                time.perf_counter() - fallback_prefilter_start
            ) * 1000.0
            (
                artwork_candidates,
                fallback_best_invalid,
                fallback_completed,
                fallback_score_ms,
            ) = full_artwork_candidates(fallback_prefiltered)
            completed_full_scores += fallback_completed
            artwork_score_ms += fallback_score_ms
            if (
                fallback_best_invalid is not None
                and (
                    best_invalid is None
                    or invalid_candidate_key(fallback_best_invalid)
                    < invalid_candidate_key(best_invalid)
                )
            ):
                best_invalid = fallback_best_invalid
        if not artwork_candidates:
            details = best_invalid[8] if best_invalid is not None else {}
            if best_invalid is not None:
                approximate_targets = best_invalid[4]
                approximate_rotations = [
                    self._normalize_angle(
                        float(rotation)
                        + (180.0 if best_invalid[7][piece_id] else 0.0)
                    )
                    for piece_id, rotation in enumerate(best_invalid[5])
                ]
                approximate_matches = self._infer_adjacency_matches(
                    approximate_targets,
                    existing_matches=[],
                )
                approximate_piece_ids = {
                    int(piece_id)
                    for match in approximate_matches
                    for piece_id in (match[0], match[2])
                }
                self._remember_best_candidate(
                    detection,
                    detection_annotated,
                    approximate_targets,
                    approximate_rotations,
                    {
                        "score": float(best_invalid[0]),
                        "geometry_score": float(best_invalid[1]),
                        "partial_matched_piece_count": len(
                            approximate_piece_ids
                        ),
                        "partial_matched_seam_count": len(
                            approximate_matches
                        ),
                        "pattern_biaxial_overlap": float(
                            details.get("pattern_biaxial_overlap", 0.0)
                        ),
                        "task_mode": 4.0,
                    },
                    (
                        -len(approximate_piece_ids),
                        -len(approximate_matches),
                        -float(
                            details.get("pattern_biaxial_overlap", 0.0)
                        ),
                        float(best_invalid[0]),
                    ),
                    "mode4_best_rejected_artwork",
                )
                rejected_detection = (
                    self.output_directory / "fsm_mode4_rejected_detection.jpg"
                )
                rejected_solution = (
                    self.output_directory / "fsm_mode4_best_rejected.jpg"
                )
                cv2.imwrite(str(rejected_detection), detection_annotated)
                render_rectangular_card_solution(
                    rectified,
                    pieces,
                    best_invalid[4],
                    best_invalid[7],
                    details,
                    str(rejected_solution),
                )
                rejected_preview = cv2.imread(str(rejected_solution))
                if rejected_preview is not None:
                    cv2.putText(
                        rejected_preview,
                        "REJECTED - DIAGNOSTIC ONLY",
                        (12, rejected_preview.shape[0] - 18),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.72,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imwrite(str(rejected_solution), rejected_preview)
            raise RuntimeError(
                "Mode 4 no card passed pixel validation: global={:.3f} "
                "corner={:.3f} diag={:.3f} diag_comp={:.3f} "
                "diag_ink={:.3f} border={:.3f} rim={:.3f} "
                "red_border={:.3f} "
                "red_rim={:.3f} ink={:.0f} corner_min={:.0f}/{:.0f} "
                "corner_counts={} index_components={}"
                .format(
                    details.get("global_180_overlap", 0.0),
                    details.get("corner_180_overlap", 0.0),
                    details.get("diagonal_dominance", 0.0),
                    details.get("diagonal_component_dominance", 0.0),
                    details.get("diagonal_ink_dominance", 0.0),
                    details.get("forbidden_border_ink_ratio", 1.0),
                    details.get("outer_rim_ink_ratio", 1.0),
                    details.get("forbidden_red_border_ratio", 1.0),
                    details.get("outer_red_rim_ratio", 1.0),
                    details.get("total_ink_pixels", 0.0),
                    details.get("minimum_corner_ink", 0.0),
                    details.get("minimum_required_corner_ink", 0.0),
                    details.get("corner_counts", []),
                    details.get("index_component_counts", []),
                )
            )
        if piece_count == 4 and equal_quarters:
            best_pattern_axis_overlap = max(
                float(item[8].get("pattern_long_axis_overlap", 0.0))
                for item in artwork_candidates
            )
            minimum_pattern_axis_overlap = (
                best_pattern_axis_overlap
                - MODE4_PATTERN_AXIS_SELECTION_TOLERANCE
            )
            axis_consistent_candidates = [
                item
                for item in artwork_candidates
                if float(item[8].get("pattern_long_axis_overlap", 0.0))
                >= minimum_pattern_axis_overlap
            ]
            axis_consistent_candidates.sort(
                key=lambda item: (
                    item[0],
                    -item[8].get("pattern_long_axis_overlap", 0.0),
                )
            )
            artwork_candidates = axis_consistent_candidates
        else:
            best_pattern_axis_overlap = None
            minimum_pattern_axis_overlap = None
            artwork_candidates.sort(key=lambda item: item[0])
        (
            combined_score,
            geometry_score,
            state,
            metrics,
            artwork_targets,
            base_rotations,
            overlap_before_adjust,
            selected_half_turns,
            artwork_details,
            selected_assignment,
        ) = artwork_candidates[0]
        if best_pattern_axis_overlap is not None:
            artwork_details["best_valid_pattern_axis_overlap"] = float(
                best_pattern_axis_overlap
            )
            artwork_details["minimum_selected_pattern_axis_overlap"] = float(
                minimum_pattern_axis_overlap
            )
        rotations = [
            self._normalize_angle(
                float(rotation) + (180.0 if selected_half_turns[piece_id] else 0.0)
            )
            for piece_id, rotation in enumerate(base_rotations)
        ]
        approximate_matches = self._infer_adjacency_matches(
            artwork_targets,
            existing_matches=[],
        )
        approximate_piece_ids = {
            int(piece_id)
            for match in approximate_matches
            for piece_id in (match[0], match[2])
        }
        self._remember_best_candidate(
            detection,
            detection_annotated,
            artwork_targets,
            rotations,
            {
                "score": float(combined_score),
                "geometry_score": float(geometry_score),
                "partial_matched_piece_count": len(approximate_piece_ids),
                "partial_matched_seam_count": len(approximate_matches),
                "pattern_biaxial_overlap": float(
                    artwork_details.get("pattern_biaxial_overlap", 0.0)
                ),
                "task_mode": 4.0,
            },
            (
                -len(approximate_piece_ids),
                -len(approximate_matches),
                -float(
                    artwork_details.get("pattern_biaxial_overlap", 0.0)
                ),
                float(combined_score),
            ),
            "mode4_best_valid_artwork_before_final_safety",
        )

        targets = self._separate_small_overlaps(
            artwork_targets,
            target_origin=target_origin,
        )
        pre_clearance_points = np.concatenate(targets, axis=0).astype(
            np.float32
        )
        (_, _), pre_clearance_size, _ = cv2.minAreaRect(
            pre_clearance_points
        )
        pre_clearance_short, pre_clearance_long = sorted(
            map(float, pre_clearance_size)
        )
        if not (
            90.0 <= pre_clearance_long <= 120.0
            and 50.0 <= pre_clearance_short <= 90.0
        ):
            raise RuntimeError(
                "Mode 4 pre-clearance rectangle has illegal size: "
                "{:.2f}x{:.2f}mm".format(
                    pre_clearance_long,
                    pre_clearance_short,
                )
            )
        targets, adjacency_gaps = self._add_assembly_clearance(
            targets,
            matches=self._infer_adjacency_matches(
                targets,
                # The geometry state was generated before the explicit
                # piece-to-slot permutation, so its labeled edge matches no
                # longer belong to these piece ids.  Rebuild adjacency from
                # the selected target layout itself.
                existing_matches=[],
            ),
            target_origin=target_origin,
            clearance_mm=TARGET_CLEARANCE_MM,
        )
        targets = self._separate_small_overlaps(
            targets,
            target_origin=target_origin,
        )
        overlap_area = self._validate_zero_overlap(
            targets,
            tolerance_mm2=0.001,
        )
        targets, placement_metrics = self._place_above_divider(targets)
        if adjacency_gaps and (
            min(adjacency_gaps) < TARGET_CLEARANCE_MM - 0.05
            or max(adjacency_gaps) > TARGET_CLEARANCE_MM + 0.05
        ):
            raise RuntimeError(
                "Mode 4 adjacent-edge gap is not {:.2f}mm: "
                "range {:.2f}..{:.2f}mm"
                .format(
                    TARGET_CLEARANCE_MM,
                    min(adjacency_gaps),
                    max(adjacency_gaps)
                )
            )

        all_points = np.concatenate(targets, axis=0).astype(np.float32)
        (_, _), size, _ = cv2.minAreaRect(all_points)
        target_short, target_long = sorted(map(float, size))

        metrics = dict(metrics)
        metrics.update(
            {
                "strategy": "mode4_rectangles_180_symmetry_border_corners",
                "task_mode": 4.0,
                "solver_workers": float(self.solver_workers),
                "score": float(combined_score),
                "geometry_score": float(geometry_score),
                "global_180_overlap": float(artwork_details["global_180_overlap"]),
                "center_180_overlap": float(artwork_details["center_180_overlap"]),
                "corner_180_overlap": float(artwork_details["corner_180_overlap"]),
                "pattern_axis_overlap": float(
                    artwork_details["pattern_axis_overlap"]
                ),
                "pattern_long_axis_overlap": float(
                    artwork_details["pattern_long_axis_overlap"]
                ),
                "pattern_short_axis_overlap": float(
                    artwork_details["pattern_short_axis_overlap"]
                ),
                "pattern_biaxial_overlap": float(
                    artwork_details["pattern_biaxial_overlap"]
                ),
                "best_valid_pattern_axis_overlap": (
                    None
                    if best_pattern_axis_overlap is None
                    else float(best_pattern_axis_overlap)
                ),
                "minimum_selected_pattern_axis_overlap": (
                    None
                    if minimum_pattern_axis_overlap is None
                    else float(minimum_pattern_axis_overlap)
                ),
                "pattern_axis_selection_tolerance": float(
                    MODE4_PATTERN_AXIS_SELECTION_TOLERANCE
                ),
                "pattern_axis": artwork_details["pattern_axis"],
                "corner_diagonal": artwork_details["corner_diagonal"],
                "corner_counts": artwork_details["corner_counts"],
                "minimum_corner_ink": float(
                    artwork_details["minimum_corner_ink"]
                ),
                "minimum_required_corner_ink": float(
                    artwork_details["minimum_required_corner_ink"]
                ),
                "index_component_counts": artwork_details[
                    "index_component_counts"
                ],
                "diagonal_dominance": float(artwork_details["diagonal_dominance"]),
                "diagonal_component_dominance": float(
                    artwork_details["diagonal_component_dominance"]
                ),
                "diagonal_ink_dominance": float(
                    artwork_details["diagonal_ink_dominance"]
                ),
                "forbidden_border_ink_ratio": float(
                    artwork_details["forbidden_border_ink_ratio"]
                ),
                "outer_rim_ink_ratio": float(
                    artwork_details["outer_rim_ink_ratio"]
                ),
                "forbidden_red_border_ratio": float(
                    artwork_details["forbidden_red_border_ratio"]
                ),
                "outer_red_rim_ratio": float(
                    artwork_details["outer_red_rim_ratio"]
                ),
                "seam_diagnostics_computed": False,
                "total_ink_pixels": float(artwork_details["total_ink_pixels"]),
                "selected_half_turns": list(selected_half_turns),
                "slot_assignment": list(selected_assignment),
                "rectangle_diagnostics": rectangle_diagnostics,
                "raw_geometry_candidates": float(len(raw_states)),
                "unique_slot_layouts": float(len(unique_slot_layouts)),
                "assignment_candidates": float(len(geometry_candidates)),
                "assignment_candidate_limit": float(assignment_limit),
                "equal_quarter_rectangles": equal_quarters,
                "valid_geometry_candidates": float(len(geometry_candidates)),
                "corner_anchor_candidates": float(total_scores),
                "corner_anchor_survivors": float(primary_anchor_count),
                "corner_anchor_fallback": corner_anchor_fallback,
                "artwork_prefilter_candidates": float(
                    total_scores
                    if corner_anchor_fallback
                    else primary_anchor_count
                ),
                "artwork_candidates_scored": float(completed_full_scores),
                "artwork_candidates_valid": float(len(artwork_candidates)),
                "selected_prefilter_rank": float(
                    artwork_details.get("prefilter_rank", -1.0)
                ),
                "selected_prefilter_score": float(
                    artwork_details.get("prefilter_score", 0.0)
                ),
                "selected_corner_anchor_rank": float(
                    artwork_details.get("corner_anchor_rank", -1)
                ),
                "overlap_before_adjust_mm2": round(overlap_before_adjust, 4),
                "overlap_area_mm2": overlap_area,
                "target_clearance_mm": TARGET_CLEARANCE_MM,
                "clearance_basis": "adjacent_edge_normal_distance",
                "pre_clearance_short_side_mm": pre_clearance_short,
                "pre_clearance_long_side_mm": pre_clearance_long,
                "short_side_mm": target_short,
                "long_side_mm": target_long,
                "workspace_source": "physical_right",
                "workspace_target": "physical_left",
                "adjacency_gap_min_mm": round(
                    min(adjacency_gaps) if adjacency_gaps else 0.0,
                    3,
                ),
                "adjacency_gap_max_mm": round(
                    max(adjacency_gaps) if adjacency_gaps else 0.0,
                    3,
                ),
            }
        )
        metrics.update(placement_metrics)
        solve_ms = (time.perf_counter() - solve_start) * 1000.0

        detection_image = self.output_directory / "fsm_mode4_detection.jpg"
        solution_image = self.output_directory / "fsm_mode4_solution.jpg"
        cv2.imwrite(str(detection_image), detection_annotated)
        render_rectangular_card_solution(
            rectified,
            pieces,
            artwork_targets,
            selected_half_turns,
            artwork_details,
            str(solution_image),
        )
        self._report_progress(1.0, "Mode 4 rectangle artwork solution ready")

        order = sorted(
            range(piece_count),
            key=lambda piece_id: detection["pieces"][piece_id]["area_px2"],
            reverse=True,
        )
        actions = []
        for sequence, piece_id in enumerate(order):
            piece = detection["pieces"][piece_id]
            target_center = polygon_centroid(targets[piece_id])
            rotation_delta = rotations[piece_id]
            target_pickup = self._target_pickup_point(
                piece["pickup_mm"],
                piece["centroid_mm"],
                target_center,
                rotation_delta,
            )
            actions.append(
                MoveAction(
                    sequence=sequence,
                    piece_id=piece_id,
                    source_pickup_mm=[round(float(v), 2) for v in piece["pickup_mm"]],
                    source_centroid_mm=[round(float(v), 2) for v in piece["centroid_mm"]],
                    target_pickup_mm=[round(float(v), 2) for v in target_pickup],
                    target_centroid_mm=[round(float(v), 2) for v in target_center],
                    rotation_delta_deg=round(rotation_delta, 2),
                )
            )

        total_ms = (time.perf_counter() - total_start) * 1000.0
        return {
            "actions": actions,
            "metrics": metrics,
            "timing_ms": {
                "image_load": round(load_ms, 2),
                "vision": round(vision_ms, 2),
                "geometry_search": round(geometry_search_ms, 2),
                "geometry_filter": round(geometry_filter_ms, 2),
                "corner_anchor": round(corner_anchor_ms, 2),
                "artwork_prefilter": round(artwork_prefilter_ms, 2),
                "artwork_score": round(artwork_score_ms, 2),
                "puzzle_solve": round(solve_ms, 2),
                "total": round(total_ms, 2),
            },
            "detection_image": str(detection_image),
            "solution_image": str(solution_image),
        }

    @staticmethod
    def _validate_zero_overlap(
        polygons: List[np.ndarray],
        tolerance_mm2: float = 0.01,
    ) -> float:
        """Reject a motion plan if any two target pieces overlap."""
        total_overlap = 0.0
        for first_id in range(len(polygons)):
            first = cv2.convexHull(
                np.asarray(polygons[first_id], dtype=np.float32)
            )
            for second_id in range(first_id + 1, len(polygons)):
                second = cv2.convexHull(
                    np.asarray(polygons[second_id], dtype=np.float32)
                )
                overlap, _ = cv2.intersectConvexConvex(first, second)
                overlap = max(0.0, float(overlap))
                if overlap > tolerance_mm2:
                    raise RuntimeError(
                        "target overlap: P{} and P{} = "
                        "{:.3f} mm^2".format(
                            first_id,
                            second_id,
                            overlap,
                        )
                    )
                # Values below tolerance are floating-point contact noise,
                # not a physical overlap, and are reported as zero.
        return round(total_overlap, 4)

    @staticmethod
    def _place_above_divider(
        polygons: List[np.ndarray],
        divider_y_mm: float = TARGET_DIVIDER_Y_MM,
        gap_mm: float = TARGET_DIVIDER_GAP_MM,
    ):
        """Translate the finished assembly to a fixed gap above the divider."""
        adjusted = [
            np.asarray(polygon, dtype=np.float32).copy()
            for polygon in polygons
        ]
        all_points = np.concatenate(adjusted, axis=0)
        desired_max_y = float(divider_y_mm) - float(gap_mm)
        shift_y = desired_max_y - float(np.max(all_points[:, 1]))
        shift = np.asarray([0.0, shift_y], dtype=np.float32)
        adjusted = [polygon + shift for polygon in adjusted]
        placed_points = np.concatenate(adjusted, axis=0)
        minimum = np.min(placed_points, axis=0)
        maximum = np.max(placed_points, axis=0)
        if minimum[1] < -0.001:
            raise RuntimeError(
                "assembly is too tall to keep {:.1f}mm above divider"
                .format(gap_mm)
            )
        actual_gap = float(divider_y_mm) - float(maximum[1])
        return adjusted, {
            "target_origin_mm": [
                round(float(minimum[0]), 3),
                round(float(minimum[1]), 3),
            ],
            "target_max_y_mm": round(float(maximum[1]), 3),
            "target_divider_y_mm": float(divider_y_mm),
            "target_divider_gap_mm": round(actual_gap, 3),
        }

    @staticmethod
    def _total_overlap_area(polygons: List[np.ndarray]) -> float:
        total_overlap = 0.0
        for first_id in range(len(polygons)):
            first = cv2.convexHull(
                np.asarray(polygons[first_id], dtype=np.float32)
            )
            for second_id in range(first_id + 1, len(polygons)):
                second = cv2.convexHull(
                    np.asarray(polygons[second_id], dtype=np.float32)
                )
                overlap, _ = cv2.intersectConvexConvex(first, second)
                total_overlap += max(0.0, float(overlap))
        return total_overlap

    @classmethod
    def _separate_small_overlaps(
        cls,
        polygons: List[np.ndarray],
        target_origin: np.ndarray,
        clearance_mm: float = 0.02,
    ) -> List[np.ndarray]:
        """Resolve small fitting overlaps using convex minimum translations."""
        adjusted = [
            np.asarray(polygon, dtype=np.float32).copy()
            for polygon in polygons
        ]
        for _ in range(12):
            changed = False
            for first_id in range(len(adjusted)):
                for second_id in range(first_id + 1, len(adjusted)):
                    first = adjusted[first_id]
                    second = adjusted[second_id]
                    overlap, _ = cv2.intersectConvexConvex(first, second)
                    if float(overlap) <= 1e-5:
                        continue

                    axes = []
                    for polygon in (first, second):
                        edges = np.roll(polygon, -1, axis=0) - polygon
                        for edge in edges:
                            length = float(np.linalg.norm(edge))
                            if length < 1e-6:
                                continue
                            axes.append(
                                np.asarray(
                                    [-edge[1], edge[0]],
                                    dtype=np.float32,
                                )
                                / length
                            )
                    best_axis = None
                    best_depth = float("inf")
                    for axis in axes:
                        first_projection = first @ axis
                        second_projection = second @ axis
                        depth = min(
                            float(np.max(first_projection)),
                            float(np.max(second_projection)),
                        ) - max(
                            float(np.min(first_projection)),
                            float(np.min(second_projection)),
                        )
                        if depth < best_depth:
                            best_depth = depth
                            best_axis = axis
                    if best_axis is None or best_depth <= 0:
                        continue
                    first_center = np.mean(first, axis=0)
                    second_center = np.mean(second, axis=0)
                    if (
                        float(
                            np.dot(
                                second_center - first_center,
                                best_axis,
                            )
                        )
                        < 0
                    ):
                        best_axis = -best_axis
                    shift = (
                        best_axis
                        * (best_depth + clearance_mm)
                        * 0.5
                    )
                    adjusted[first_id] -= shift
                    adjusted[second_id] += shift
                    changed = True
            if not changed:
                break

        remaining = cls._total_overlap_area(adjusted)
        if remaining > 0.001:
            raise RuntimeError(
                "cannot remove residual overlap: "
                "{:.4f} mm^2".format(remaining)
            )
        all_points = np.concatenate(adjusted, axis=0)
        offset = (
            np.asarray(target_origin, dtype=np.float32)
            - np.min(all_points, axis=0)
        )
        return [polygon + offset for polygon in adjusted]

    @staticmethod
    def _polygon_distance(
        first: np.ndarray,
        second: np.ndarray,
    ) -> float:
        """Return the minimum point-to-edge distance between two polygons."""
        def points_to_edges(points, polygon):
            best = float("inf")
            for edge_id in range(len(polygon)):
                start = polygon[edge_id]
                end = polygon[(edge_id + 1) % len(polygon)]
                vector = end - start
                length_squared = max(
                    float(np.dot(vector, vector)),
                    1e-8,
                )
                projection = np.clip(
                    ((points - start) @ vector) / length_squared,
                    0.0,
                    1.0,
                )
                nearest = start + projection[:, None] * vector
                best = min(
                    best,
                    float(
                        np.min(
                            np.linalg.norm(points - nearest, axis=1)
                        )
                    ),
                )
            return best

        return min(
            points_to_edges(first, second),
            points_to_edges(second, first),
        )

    @staticmethod
    def _polygon_vertex_distance(
        first: np.ndarray,
        second: np.ndarray,
    ) -> float:
        """Return the closest vertex-to-vertex distance in millimetres."""
        first = np.asarray(first, dtype=np.float32)
        second = np.asarray(second, dtype=np.float32)
        distances = np.linalg.norm(
            first[:, None, :] - second[None, :, :],
            axis=2,
        )
        return float(np.min(distances))

    @classmethod
    def _add_assembly_clearance(
        cls,
        polygons: List[np.ndarray],
        matches,
        target_origin: np.ndarray,
        clearance_mm: float,
    ):
        """Separate matched seam edges by ``clearance_mm`` along their normals."""
        adjusted = [
            np.asarray(polygon, dtype=np.float32).copy()
            for polygon in polygons
        ]
        seams = []
        seen_pairs = set()
        centers = np.asarray(
            [np.mean(polygon, axis=0) for polygon in adjusted],
            dtype=np.float64,
        )
        for match in matches:
            first_id, first_edge, second_id, second_edge = map(
                int, match[:4]
            )
            if first_id == second_id:
                continue
            if first_id > second_id:
                first_id, second_id = second_id, first_id
                first_edge, second_edge = second_edge, first_edge
            pair = (first_id, second_id)
            if pair in seen_pairs:
                continue
            seen_pairs.add(pair)
            first = adjusted[first_id]
            second = adjusted[second_id]
            first_start = first[first_edge % len(first)]
            first_end = first[(first_edge + 1) % len(first)]
            second_start = second[second_edge % len(second)]
            second_end = second[(second_edge + 1) % len(second)]
            first_vector = first_end - first_start
            second_vector = second_end - second_start
            first_length = float(np.linalg.norm(first_vector))
            second_length = float(np.linalg.norm(second_vector))
            if min(first_length, second_length) <= 1e-6:
                continue
            first_tangent = first_vector / first_length
            second_tangent = second_vector / second_length
            if float(np.dot(first_tangent, second_tangent)) < 0.0:
                second_tangent = -second_tangent
            tangent = first_tangent + second_tangent
            tangent_length = float(np.linalg.norm(tangent))
            if tangent_length <= 1e-6:
                tangent = first_tangent
            else:
                tangent = tangent / tangent_length
            normal = np.asarray([-tangent[1], tangent[0]], dtype=np.float64)
            if float(np.dot(centers[second_id] - centers[first_id], normal)) < 0.0:
                normal = -normal
            first_midpoint = 0.5 * (first_start + first_end)
            second_midpoint = 0.5 * (second_start + second_end)
            current_gap = float(
                np.dot(second_midpoint - first_midpoint, normal)
            )
            seams.append(
                (
                    first_id,
                    first_edge,
                    second_id,
                    second_edge,
                    normal,
                    current_gap,
                )
            )

        if seams:
            # Each row constrains only the normal component between a pair of
            # rigid pieces.  The minimum-norm least-squares solution preserves
            # the solved shape while making every real seam 8 mm wide.  Two
            # gauge rows keep the assembly centre fixed.
            row_count = len(seams) + 2
            matrix = np.zeros((row_count, 2 * len(adjusted)), dtype=np.float64)
            right_hand_side = np.zeros(row_count, dtype=np.float64)
            for row, seam in enumerate(seams):
                first_id, _, second_id, _, normal, current_gap = seam
                matrix[row, 2 * first_id:2 * first_id + 2] = -normal
                matrix[row, 2 * second_id:2 * second_id + 2] = normal
                right_hand_side[row] = float(clearance_mm) - current_gap
            matrix[-2, 0::2] = 1.0
            matrix[-1, 1::2] = 1.0
            translations, _, _, _ = np.linalg.lstsq(
                matrix,
                right_hand_side,
                rcond=None,
            )
            translations = translations.reshape((-1, 2))
            adjusted = [
                polygon + translations[piece_id].astype(np.float32)
                for piece_id, polygon in enumerate(adjusted)
            ]

        all_points = np.concatenate(adjusted, axis=0)
        offset = (
            np.asarray(target_origin, dtype=np.float32)
            - np.min(all_points, axis=0)
        )
        adjusted = [polygon + offset for polygon in adjusted]
        adjusted = cls._separate_small_overlaps(
            adjusted,
            target_origin=np.asarray(target_origin, dtype=np.float32),
        )
        gaps = []
        for first_id, first_edge, second_id, second_edge, normal, _ in seams:
            first = adjusted[first_id]
            second = adjusted[second_id]
            first_midpoint = 0.5 * (
                first[first_edge % len(first)]
                + first[(first_edge + 1) % len(first)]
            )
            second_midpoint = 0.5 * (
                second[second_edge % len(second)]
                + second[(second_edge + 1) % len(second)]
            )
            gaps.append(
                abs(float(np.dot(second_midpoint - first_midpoint, normal)))
            )
        return adjusted, gaps

    @classmethod
    def _infer_adjacency_matches(
        cls,
        polygons: List[np.ndarray],
        existing_matches=(),
        contact_threshold_mm: float = 5.0,
    ):
        """Return real nearby, overlapping and near-parallel seam edges."""
        polygons = [
            np.asarray(polygon, dtype=np.float32)
            for polygon in polygons
        ]

        def best_seam(first_id, second_id):
            first = polygons[first_id]
            second = polygons[second_id]
            best = None
            for first_edge in range(len(first)):
                first_start = first[first_edge]
                first_end = first[(first_edge + 1) % len(first)]
                first_vector = first_end - first_start
                first_length = float(np.linalg.norm(first_vector))
                if first_length <= 1e-6:
                    continue
                first_tangent = first_vector / first_length
                for second_edge in range(len(second)):
                    second_start = second[second_edge]
                    second_end = second[(second_edge + 1) % len(second)]
                    second_vector = second_end - second_start
                    second_length = float(np.linalg.norm(second_vector))
                    if second_length <= 1e-6:
                        continue
                    second_tangent = second_vector / second_length
                    parallel_cosine = abs(
                        float(np.dot(first_tangent, second_tangent))
                    )
                    parallel_cosine = float(
                        np.clip(parallel_cosine, -1.0, 1.0)
                    )
                    parallel_error = float(
                        np.degrees(np.arccos(parallel_cosine))
                    )
                    if parallel_error > 20.0:
                        continue
                    if float(np.dot(first_tangent, second_tangent)) < 0.0:
                        second_tangent = -second_tangent
                    tangent = first_tangent + second_tangent
                    tangent_norm = float(np.linalg.norm(tangent))
                    if tangent_norm <= 1e-6:
                        tangent = first_tangent
                    else:
                        tangent = tangent / tangent_norm
                    normal = np.asarray(
                        [-tangent[1], tangent[0]],
                        dtype=np.float32,
                    )
                    first_midpoint = 0.5 * (first_start + first_end)
                    second_midpoint = 0.5 * (second_start + second_end)
                    line_gap = abs(
                        float(
                            np.dot(
                                second_midpoint - first_midpoint,
                                normal,
                            )
                        )
                    )
                    if line_gap > float(contact_threshold_mm):
                        continue
                    first_projection = sorted(
                        (
                            float(np.dot(first_start, tangent)),
                            float(np.dot(first_end, tangent)),
                        )
                    )
                    second_projection = sorted(
                        (
                            float(np.dot(second_start, tangent)),
                            float(np.dot(second_end, tangent)),
                        )
                    )
                    overlap = max(
                        0.0,
                        min(first_projection[1], second_projection[1])
                        - max(first_projection[0], second_projection[0]),
                    )
                    minimum_overlap = min(
                        3.0,
                        0.15 * min(first_length, second_length),
                    )
                    if overlap < minimum_overlap:
                        continue
                    score = (
                        line_gap
                        + 0.20 * parallel_error
                        - 0.01 * overlap
                    )
                    candidate = (
                        score,
                        first_edge,
                        second_edge,
                        line_gap,
                    )
                    if best is None or candidate < best:
                        best = candidate
            return best

        edges = []
        for first_id in range(len(polygons)):
            for second_id in range(first_id + 1, len(polygons)):
                seam = best_seam(first_id, second_id)
                edges.append(
                    (
                        cls._polygon_distance(
                            polygons[first_id],
                            polygons[second_id],
                        ),
                        first_id,
                        second_id,
                        seam,
                    )
                )
        edges.sort(key=lambda item: item[0])
        requested_pairs = {
            tuple(sorted((int(match[0]), int(match[2]))))
            for match in existing_matches
            if int(match[0]) != int(match[2])
        }
        matches = []
        for distance, first_id, second_id, seam in edges:
            pair = (first_id, second_id)
            if seam is None:
                continue
            if (
                distance > float(contact_threshold_mm)
                and pair not in requested_pairs
            ):
                continue
            _, first_edge, second_edge, line_gap = seam
            matches.append(
                (
                    first_id,
                    int(first_edge),
                    second_id,
                    int(second_edge),
                    float(line_gap),
                )
            )
        return tuple(matches)

    @classmethod
    def _apply_target_clearance(
        cls,
        polygons: List[np.ndarray],
        matches,
        target_origin: np.ndarray,
        clearance_mm: float,
        size_tolerance_mm: float = 0.0,
        long_side_range_mm=None,
        short_side_range_mm=None,
    ):
        adjusted = cls._separate_small_overlaps(
            polygons,
            target_origin=target_origin,
        )
        pre_clearance_points = np.concatenate(adjusted, axis=0).astype(
            np.float32
        )
        (_, _), (pre_width, pre_height), _ = cv2.minAreaRect(
            pre_clearance_points
        )
        pre_clearance_short, pre_clearance_long = sorted(
            (float(pre_width), float(pre_height))
        )
        size_tolerance_mm = max(0.0, float(size_tolerance_mm))
        if long_side_range_mm is None:
            long_side_range_mm = (
                90.0 - size_tolerance_mm,
                120.0 + size_tolerance_mm,
            )
        if short_side_range_mm is None:
            short_side_range_mm = (
                50.0 - size_tolerance_mm,
                90.0 + size_tolerance_mm,
            )
        long_side_min, long_side_max = map(float, long_side_range_mm)
        short_side_min, short_side_max = map(float, short_side_range_mm)
        if not (
            long_side_min <= pre_clearance_long <= long_side_max
            and short_side_min <= pre_clearance_short <= short_side_max
        ):
            raise RuntimeError(
                "pre-clearance rectangle has illegal size: "
                "{:.2f}x{:.2f}mm".format(
                    pre_clearance_long,
                    pre_clearance_short,
                )
            )
        adjusted, gaps = cls._add_assembly_clearance(
            adjusted,
            matches=matches,
            target_origin=target_origin,
            clearance_mm=clearance_mm,
        )
        adjusted = cls._separate_small_overlaps(
            adjusted,
            target_origin=target_origin,
        )
        overlap = cls._validate_zero_overlap(
            adjusted,
            tolerance_mm2=0.001,
        )
        if gaps and (
            min(gaps) < float(clearance_mm) - 0.05
            or max(gaps) > float(clearance_mm) + 0.05
        ):
            raise RuntimeError(
                "adjacent-edge gap is not {:.2f}mm: "
                "range {:.2f}..{:.2f}mm".format(
                    float(clearance_mm),
                    min(gaps),
                    max(gaps),
                )
            )
        all_points = np.concatenate(adjusted, axis=0).astype(np.float32)
        (_, _), (width, height), _ = cv2.minAreaRect(all_points)
        short_side, long_side = sorted((float(width), float(height)))
        area = sum(
            abs(float(cv2.contourArea(polygon)))
            for polygon in adjusted
        )
        return adjusted, {
            "short_side_mm": short_side,
            "long_side_mm": long_side,
            "fill_ratio": area / max(1e-6, short_side * long_side),
            "target_clearance_mm": float(clearance_mm),
            "clearance_basis": "adjacent_edge_normal_distance",
            "rectangle_validation_stage": "before_target_clearance",
            "pre_clearance_size_tolerance_mm": size_tolerance_mm,
            "pre_clearance_long_side_range_mm": [
                long_side_min,
                long_side_max,
            ],
            "pre_clearance_short_side_range_mm": [
                short_side_min,
                short_side_max,
            ],
            "pre_clearance_short_side_mm": pre_clearance_short,
            "pre_clearance_long_side_mm": pre_clearance_long,
            "adjacency_gap_min_mm": round(
                min(gaps) if gaps else 0.0,
                3,
            ),
            "adjacency_gap_max_mm": round(
                max(gaps) if gaps else 0.0,
                3,
            ),
            "overlap_area_mm2": overlap,
        }

    @staticmethod
    def _target_pickup_point(
        source_pickup,
        source_centroid,
        target_centroid,
        rotation_deg,
    ) -> np.ndarray:
        """Map the held source point to its compensated target position."""
        angle = np.deg2rad(float(rotation_deg))
        rotation = np.asarray(
            [
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle), np.cos(angle)],
            ],
            dtype=np.float32,
        )
        source_offset = (
            np.asarray(source_pickup, dtype=np.float32)
            - np.asarray(source_centroid, dtype=np.float32)
        )
        return (
            np.asarray(target_centroid, dtype=np.float32)
            + rotation @ source_offset
        )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return (angle + 180.0) % 360.0 - 180.0


def run_stdio(machine: VisionStateMachine) -> int:
    """每读入一行命令，输出一行JSON，便于控制端联调。"""
    for line in sys.stdin:
        try:
            payload = {
                "ok": True,
                "data": machine.handle_command(line),
            }
        except Exception as error:
            payload = {
                "ok": False,
                "error": str(error),
                "state": machine.state.value,
            }
        print(json.dumps(payload, ensure_ascii=False), flush=True)
    return 0


def demo(
    machine: VisionStateMachine,
    image_path: str,
    mode: int = 1,
) -> int:
    commands = [
        "MODE,{}".format(mode),
        "START,{}".format(image_path),
        "PLAN",
    ]
    for command in commands:
        response = machine.handle_command(command)
        print(command)
        print(json.dumps(response, ensure_ascii=False, indent=2))
    while machine.state != VisionState.COMPLETED:
        action = machine.handle_command("NEXT")
        print("NEXT")
        print(json.dumps(action, ensure_ascii=False, indent=2))
        machine.handle_command(
            "ACK,{},OK".format(action["piece_id"])
        )
    print(json.dumps(machine.status(), ensure_ascii=False, indent=2))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", default="", help="使用图片执行完整联调演示")
    parser.add_argument(
        "--demo-mode",
        type=int,
        choices=(1, 2, 3, 4),
        default=1,
        help="演示模式编号，默认1",
    )
    parser.add_argument(
        "--output-directory",
        default="",
        help="状态机调试图片输出目录",
    )
    parser.add_argument(
        "--solver-workers",
        type=int,
        default=3,
        help="Mode 2/3 overlap and Mode 4 scoring workers",
    )
    args = parser.parse_args()
    machine = VisionStateMachine(
        output_directory=args.output_directory or None,
        solver_workers=args.solver_workers,
    )
    if args.demo:
        return demo(machine, args.demo, args.demo_mode)
    return run_stdio(machine)


if __name__ == "__main__":
    raise SystemExit(main())
