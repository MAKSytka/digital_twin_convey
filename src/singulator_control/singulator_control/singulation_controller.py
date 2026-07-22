"""V7 immutable-global-queue controller for the 14x4 singulator.

V7 removes cluster re-creation and absolute exit-slot scheduling.  Products are
captured once at the matrix entry, appended to one immutable global queue, and
controlled only through the measured clearances between adjacent queue items.

The controller also maintains logical product identities above the raw vision
track IDs.  When one large detection covers several predicted products, the
observation is treated as a merged contour: the original logical products coast
as separate ghost tracks and retain their queue order until they are visible
again.

Shared conveyor cells use an urgency-weighted average of all contacting product
requests.  No cell is assigned to one arbitrary owner, which removes the command
chatter seen in V6.  All commanded speeds remain positive and bounded.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import rclpy
from rclpy.node import Node

from singulator_interfaces.msg import (
    BoxObservation,
    BoxObservationArray,
    MatrixCommand,
)

from .global_queue_logic import (
    GapState,
    allocate_cell_speeds,
    build_pairwise_speed_profile,
    clamp,
    cosine_similarity,
)


def normalise_half_turn(angle: float) -> float:
    return ((angle + math.pi / 2.0) % math.pi) - math.pi / 2.0


def angle_error(angle: float, reference: float) -> float:
    return normalise_half_turn(angle - reference)


@dataclass(frozen=True, slots=True)
class ObservedBox:
    raw_track_id: int
    x: float
    y: float
    length: float
    width: float
    yaw: float
    confidence: float

    @property
    def projected_half_length(self) -> float:
        return 0.5 * (
            abs(self.length * math.cos(self.yaw))
            + abs(self.width * math.sin(self.yaw))
        )

    @property
    def projected_half_width(self) -> float:
        return 0.5 * (
            abs(self.length * math.sin(self.yaw))
            + abs(self.width * math.cos(self.yaw))
        )

    @property
    def footprint_area(self) -> float:
        return max(1.0e-6, self.length * self.width)


@dataclass(slots=True)
class LogicalBox:
    uid: int
    sequence: int
    raw_track_id: int
    x: float
    y: float
    length: float
    width: float
    yaw: float
    confidence: float
    first_seen_s: float
    stamp_s: float
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0
    last_command_speed: float = 2.20
    occluded_until_s: float = -math.inf
    last_merge_evidence_s: float = -math.inf
    wave_id: int | None = None
    queued: bool = False
    observation_count: int = 1

    def update(self, observation: ObservedBox, stamp_s: float) -> None:
        dt = stamp_s - self.stamp_s
        if dt > 1.0e-4:
            measured_vx = (observation.x - self.x) / dt
            measured_vy = (observation.y - self.y) / dt
            measured_yaw_rate = angle_error(observation.yaw, self.yaw) / dt
            alpha = 0.45
            self.vx = alpha * measured_vx + (1.0 - alpha) * self.vx
            self.vy = alpha * measured_vy + (1.0 - alpha) * self.vy
            self.yaw_rate = (
                alpha * measured_yaw_rate
                + (1.0 - alpha) * self.yaw_rate
            )

        self.raw_track_id = observation.raw_track_id
        self.x = observation.x
        self.y = observation.y
        self.length = max(0.02, observation.length)
        self.width = max(0.01, observation.width)
        self.yaw = normalise_half_turn(observation.yaw)
        self.confidence = observation.confidence
        self.stamp_s = stamp_s
        self.observation_count += 1

    @property
    def projected_half_length(self) -> float:
        return 0.5 * (
            abs(self.length * math.cos(self.yaw))
            + abs(self.width * math.sin(self.yaw))
        )

    @property
    def projected_half_width(self) -> float:
        return 0.5 * (
            abs(self.length * math.sin(self.yaw))
            + abs(self.width * math.cos(self.yaw))
        )

    @property
    def footprint_area(self) -> float:
        return max(1.0e-6, self.length * self.width)

    def prediction_speed(self, maximum_speed: float) -> float:
        measured = self.vx if self.vx > 0.05 else self.last_command_speed
        blended = 0.60 * measured + 0.40 * self.last_command_speed
        return clamp(blended, 0.0, maximum_speed)

    def predicted_x(
        self,
        now_s: float,
        maximum_speed: float,
        maximum_horizon_s: float,
    ) -> float:
        age = clamp(now_s - self.stamp_s, 0.0, maximum_horizon_s)
        return self.x + self.prediction_speed(maximum_speed) * age

    def predicted_y(self, now_s: float, maximum_horizon_s: float) -> float:
        age = clamp(now_s - self.stamp_s, 0.0, maximum_horizon_s)
        return self.y + clamp(self.vy, -0.8, 0.8) * age

    def is_ghost(self, now_s: float, frame_grace_s: float) -> bool:
        return now_s - self.stamp_s > frame_grace_s


@dataclass(slots=True)
class CellProposal:
    uid: int
    speed: float
    overlap: float
    urgency: float


class SingulationController(Node):
    """Direct adjacent-gap controller with immutable global product order."""

    def __init__(self) -> None:
        super().__init__("singulation_controller")

        self.declare_parameter("boxes_topic", "/singulator/boxes")
        self.declare_parameter("command_topic", "/singulator/matrix/command")
        self.declare_parameter("rows", 14)
        self.declare_parameter("cols", 4)
        self.declare_parameter("cell_length_m", 0.360)
        self.declare_parameter("cell_width_m", 0.175)
        self.declare_parameter("gap_x_m", 0.020)
        self.declare_parameter("gap_y_m", 0.020)
        self.declare_parameter("matrix_center_x_m", 0.0)

        self.declare_parameter("minimum_speed_mps", 1.00)
        self.declare_parameter("maximum_speed_mps", 3.00)
        self.declare_parameter("idle_speed_mps", 2.20)
        self.declare_parameter("transport_speed_mps", 2.50)
        self.declare_parameter("maximum_acceleration_mps2", 6.0)

        self.declare_parameter("target_gap_m", 0.18)
        self.declare_parameter("inter_wave_target_gap_m", 0.28)
        self.declare_parameter("gap_gain", 3.00)
        self.declare_parameter("relative_velocity_gain", 0.50)
        self.declare_parameter("maximum_relative_speed_mps", 2.00)
        self.declare_parameter("order_inversion_margin_m", 0.03)
        self.declare_parameter("exit_gap_check_margin_m", 0.80)
        self.declare_parameter("deadline_separation_distance_m", 1.80)
        self.declare_parameter("deadline_gap_margin_m", 0.04)
        self.declare_parameter("deadline_recovery_gain", 1.35)
        self.declare_parameter("deadline_min_time_s", 0.10)

        self.declare_parameter("entry_gate_offset_m", 0.30)
        self.declare_parameter("entry_capture_window_s", 0.18)
        self.declare_parameter("entry_wave_dx_m", 0.36)
        self.declare_parameter("entry_wave_max_size", 6)
        self.declare_parameter("entry_capture_back_margin_m", 1.35)
        self.declare_parameter("entry_capture_front_margin_m", 0.18)

        self.declare_parameter("normal_track_timeout_s", 0.60)
        self.declare_parameter("merged_track_timeout_s", 2.80)
        self.declare_parameter("logical_item_expiration_s", 3.50)
        self.declare_parameter("prediction_max_horizon_s", 3.00)
        self.declare_parameter("reid_max_distance_m", 0.70)
        self.declare_parameter("reid_max_lateral_distance_m", 0.30)
        self.declare_parameter("reid_max_backward_distance_m", 0.22)
        self.declare_parameter("merge_area_ratio", 1.35)
        self.declare_parameter("merge_padding_x_m", 0.08)
        self.declare_parameter("merge_padding_y_m", 0.05)
        self.declare_parameter("exit_remove_margin_m", 0.65)

        self.declare_parameter("prediction_horizon_s", 0.12)
        self.declare_parameter("longitudinal_control_margin_m", 0.10)
        self.declare_parameter("yaw_gain", 0.65)
        self.declare_parameter("yaw_rate_gain", 0.05)
        self.declare_parameter("maximum_yaw_delta_mps", 0.25)
        self.declare_parameter("dense_pair_yaw_gap_m", 0.10)
        self.declare_parameter("yaw_control_exit_margin_m", 0.65)

        self.declare_parameter("allocation_urgency_gain", 1.50)
        self.declare_parameter("allocation_idle_regularization", 0.03)
        self.declare_parameter("allocation_iterations", 12)
        self.declare_parameter("uncontrollable_similarity", 0.97)
        self.declare_parameter("minimum_confidence", 0.12)
        self.declare_parameter("observation_timeout_s", 0.70)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("diagnostic_period_s", 1.0)

        self.rows = int(self.get_parameter("rows").value)
        self.cols = int(self.get_parameter("cols").value)
        self.cell_length = float(self.get_parameter("cell_length_m").value)
        self.cell_width = float(self.get_parameter("cell_width_m").value)
        self.gap_x = float(self.get_parameter("gap_x_m").value)
        self.gap_y = float(self.get_parameter("gap_y_m").value)
        self.matrix_center_x = float(
            self.get_parameter("matrix_center_x_m").value
        )
        self.pitch_x = self.cell_length + self.gap_x
        self.pitch_y = self.cell_width + self.gap_y
        self.matrix_length = (
            self.rows * self.cell_length
            + (self.rows - 1) * self.gap_x
        )
        self.matrix_width = (
            self.cols * self.cell_width
            + (self.cols - 1) * self.gap_y
        )
        self.matrix_min_x = self.matrix_center_x - self.matrix_length / 2.0
        self.matrix_max_x = self.matrix_center_x + self.matrix_length / 2.0

        self.minimum_speed = float(
            self.get_parameter("minimum_speed_mps").value
        )
        self.maximum_speed = float(
            self.get_parameter("maximum_speed_mps").value
        )
        self.idle_speed = float(self.get_parameter("idle_speed_mps").value)
        self.transport_speed = float(
            self.get_parameter("transport_speed_mps").value
        )
        self.maximum_acceleration = float(
            self.get_parameter("maximum_acceleration_mps2").value
        )

        self.target_gap = float(self.get_parameter("target_gap_m").value)
        self.inter_wave_target_gap = float(
            self.get_parameter("inter_wave_target_gap_m").value
        )
        self.gap_gain = float(self.get_parameter("gap_gain").value)
        self.relative_velocity_gain = float(
            self.get_parameter("relative_velocity_gain").value
        )
        self.maximum_relative_speed = float(
            self.get_parameter("maximum_relative_speed_mps").value
        )
        self.order_inversion_margin = float(
            self.get_parameter("order_inversion_margin_m").value
        )
        self.exit_gap_check_margin = float(
            self.get_parameter("exit_gap_check_margin_m").value
        )
        self.deadline_separation_distance = float(
            self.get_parameter("deadline_separation_distance_m").value
        )
        self.deadline_gap_margin = float(
            self.get_parameter("deadline_gap_margin_m").value
        )
        self.deadline_recovery_gain = float(
            self.get_parameter("deadline_recovery_gain").value
        )
        self.deadline_min_time = float(
            self.get_parameter("deadline_min_time_s").value
        )

        self.entry_gate_offset = float(
            self.get_parameter("entry_gate_offset_m").value
        )
        self.entry_capture_window = float(
            self.get_parameter("entry_capture_window_s").value
        )
        self.entry_wave_dx = float(
            self.get_parameter("entry_wave_dx_m").value
        )
        self.entry_wave_max_size = int(
            self.get_parameter("entry_wave_max_size").value
        )
        self.entry_capture_back_margin = float(
            self.get_parameter("entry_capture_back_margin_m").value
        )
        self.entry_capture_front_margin = float(
            self.get_parameter("entry_capture_front_margin_m").value
        )
        self.entry_gate_x = self.matrix_min_x - self.entry_gate_offset

        self.normal_track_timeout = float(
            self.get_parameter("normal_track_timeout_s").value
        )
        self.merged_track_timeout = float(
            self.get_parameter("merged_track_timeout_s").value
        )
        self.logical_item_expiration = float(
            self.get_parameter("logical_item_expiration_s").value
        )
        self.prediction_max_horizon = float(
            self.get_parameter("prediction_max_horizon_s").value
        )
        self.reid_max_distance = float(
            self.get_parameter("reid_max_distance_m").value
        )
        self.reid_max_lateral_distance = float(
            self.get_parameter("reid_max_lateral_distance_m").value
        )
        self.reid_max_backward_distance = float(
            self.get_parameter("reid_max_backward_distance_m").value
        )
        self.merge_area_ratio = float(
            self.get_parameter("merge_area_ratio").value
        )
        self.merge_padding_x = float(
            self.get_parameter("merge_padding_x_m").value
        )
        self.merge_padding_y = float(
            self.get_parameter("merge_padding_y_m").value
        )
        self.exit_remove_margin = float(
            self.get_parameter("exit_remove_margin_m").value
        )

        self.prediction_horizon = float(
            self.get_parameter("prediction_horizon_s").value
        )
        self.longitudinal_control_margin = float(
            self.get_parameter("longitudinal_control_margin_m").value
        )
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.yaw_rate_gain = float(
            self.get_parameter("yaw_rate_gain").value
        )
        self.maximum_yaw_delta = float(
            self.get_parameter("maximum_yaw_delta_mps").value
        )
        self.dense_pair_yaw_gap = float(
            self.get_parameter("dense_pair_yaw_gap_m").value
        )
        self.yaw_control_exit_margin = float(
            self.get_parameter("yaw_control_exit_margin_m").value
        )

        self.allocation_urgency_gain = float(
            self.get_parameter("allocation_urgency_gain").value
        )
        self.allocation_idle_regularization = float(
            self.get_parameter("allocation_idle_regularization").value
        )
        self.allocation_iterations = int(
            self.get_parameter("allocation_iterations").value
        )
        self.uncontrollable_similarity = float(
            self.get_parameter("uncontrollable_similarity").value
        )
        self.minimum_confidence = float(
            self.get_parameter("minimum_confidence").value
        )
        self.observation_timeout = float(
            self.get_parameter("observation_timeout_s").value
        )
        self.publish_rate = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.diagnostic_period = float(
            self.get_parameter("diagnostic_period_s").value
        )

        self._validate_parameters()

        self.items: dict[int, LogicalBox] = {}
        self.track_to_uid: dict[int, int] = {}
        self.global_order: list[int] = []
        self.next_uid = 1
        self.next_sequence = 1
        self.next_wave_id = 1
        self.pending_wave_uids: list[int] = []
        self.pending_wave_opened_s: float | None = None

        self.last_observation_stamp_s: float | None = None
        self.last_command = [self.idle_speed] * (self.rows * self.cols)
        self.last_publish_s: float | None = None
        self.last_diagnostic_s = -math.inf

        self.last_active_boxes = 0
        self.last_queue_size = 0
        self.last_ghost_tracks = 0
        self.last_merged_observations = 0
        self.last_reidentified_tracks = 0
        self.last_recovered_orphans = 0
        self.last_order_inversions = 0
        self.last_unresolved_at_exit = 0
        self.last_deadline_boost_pairs = 0
        self.last_min_adjacent_gap = math.inf
        self.last_shared_cells = 0
        self.last_uncontrollable_pairs = 0
        self.last_allocation_error = 0.0

        boxes_topic = str(self.get_parameter("boxes_topic").value)
        command_topic = str(self.get_parameter("command_topic").value)
        self.observation_subscription = self.create_subscription(
            BoxObservationArray,
            boxes_topic,
            self._on_observations,
            10,
        )
        self.command_publisher = self.create_publisher(
            MatrixCommand,
            command_topic,
            10,
        )
        self.timer = self.create_timer(
            1.0 / max(1.0, self.publish_rate),
            self._on_control_timer,
        )

        self.get_logger().info(
            "V7 immutable global queue started: "
            f"speed=[{self.minimum_speed:.2f}, {self.maximum_speed:.2f}] m/s, "
            f"transport={self.transport_speed:.2f} m/s, "
            f"entry_gate={self.entry_gate_x:.2f} m, "
            "slot_scheduler=false, shared_cell_owner=false"
        )

    def _validate_parameters(self) -> None:
        if self.rows <= 0 or self.cols <= 0:
            raise ValueError("rows and cols must be positive")
        if not (
            0.0 < self.minimum_speed
            <= self.idle_speed
            <= self.maximum_speed
        ):
            raise ValueError(
                "Expected 0 < minimum_speed <= idle_speed <= maximum_speed"
            )
        if not (
            self.minimum_speed
            <= self.transport_speed
            <= self.maximum_speed
        ):
            raise ValueError("transport_speed_mps must be inside speed limits")
        if self.maximum_relative_speed <= 0.0:
            raise ValueError("maximum_relative_speed_mps must be positive")
        if self.maximum_acceleration <= 0.0:
            raise ValueError("maximum_acceleration_mps2 must be positive")
        if self.target_gap <= 0.0 or self.inter_wave_target_gap <= 0.0:
            raise ValueError("target gaps must be positive")
        if self.deadline_separation_distance <= 0.0:
            raise ValueError("deadline_separation_distance_m must be positive")
        if self.deadline_gap_margin < 0.0:
            raise ValueError("deadline_gap_margin_m must be >= 0")
        if self.deadline_recovery_gain <= 0.0:
            raise ValueError("deadline_recovery_gain must be positive")
        if self.deadline_min_time <= 0.0:
            raise ValueError("deadline_min_time_s must be positive")
        if self.entry_capture_window <= 0.0:
            raise ValueError("entry_capture_window_s must be positive")
        if self.entry_wave_max_size <= 0:
            raise ValueError("entry_wave_max_size must be positive")
        if self.normal_track_timeout <= 0.0:
            raise ValueError("normal_track_timeout_s must be positive")
        if self.merged_track_timeout < self.normal_track_timeout:
            raise ValueError(
                "merged_track_timeout_s must be >= normal_track_timeout_s"
            )
        if self.publish_rate <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        if self.allocation_idle_regularization < 0.0:
            raise ValueError("allocation_idle_regularization must be >= 0")
        if self.allocation_iterations <= 0:
            raise ValueError("allocation_iterations must be positive")

    def _on_observations(self, message: BoxObservationArray) -> None:
        stamp_s = self._stamp_to_seconds(message)
        if stamp_s <= 0.0:
            stamp_s = self._now_seconds()
        self.last_observation_stamp_s = stamp_s

        observations = [
            ObservedBox(
                raw_track_id=int(observation.id),
                x=float(observation.center.x),
                y=float(observation.center.y),
                length=max(0.02, float(observation.length_m)),
                width=max(0.01, float(observation.width_m)),
                yaw=normalise_half_turn(float(observation.yaw_rad)),
                confidence=float(observation.confidence),
            )
            for observation in message.boxes
            if float(observation.confidence) >= self.minimum_confidence
        ]

        self._reconcile_observations(observations, stamp_s)
        self._update_entry_queue(stamp_s)
        self._cleanup_items(stamp_s)

    def _reconcile_observations(
        self,
        observations: list[ObservedBox],
        stamp_s: float,
    ) -> None:
        """Bind raw vision detections to persistent logical products."""

        merge_indices, merge_members = self._detect_merged_observations(
            observations,
            stamp_s,
        )
        self.last_merged_observations = len(merge_indices)

        for members in merge_members.values():
            for uid in members:
                item = self.items.get(uid)
                if item is None:
                    continue
                item.occluded_until_s = max(
                    item.occluded_until_s,
                    stamp_s + self.merged_track_timeout,
                )
                item.last_merge_evidence_s = stamp_s

        normal_indices = [
            index for index in range(len(observations))
            if index not in merge_indices
        ]
        assigned_observations: set[int] = set()
        assigned_uids: set[int] = set()
        assignments: dict[int, int] = {}
        reidentified = 0

        # First retain a raw ID binding when it remains geometrically plausible.
        for index in normal_indices:
            observation = observations[index]
            uid = self.track_to_uid.get(observation.raw_track_id)
            item = self.items.get(uid) if uid is not None else None
            if item is None or uid in assigned_uids:
                continue
            if not self._observation_matches_item(
                observation,
                item,
                stamp_s,
                relaxed=False,
            ):
                continue
            assignments[index] = uid
            assigned_observations.add(index)
            assigned_uids.add(uid)

        # Then re-identify changed raw IDs by motion, lateral position and size.
        candidates: list[tuple[float, int, int]] = []
        for index in normal_indices:
            if index in assigned_observations:
                continue
            observation = observations[index]
            for uid, item in self.items.items():
                if uid in assigned_uids:
                    continue
                cost = self._reidentification_cost(
                    observation,
                    item,
                    stamp_s,
                )
                if cost is not None:
                    candidates.append((cost, index, uid))

        for _, index, uid in sorted(candidates):
            if index in assigned_observations or uid in assigned_uids:
                continue
            assignments[index] = uid
            assigned_observations.add(index)
            assigned_uids.add(uid)
            if observations[index].raw_track_id != self.items[uid].raw_track_id:
                reidentified += 1

        for index, uid in assignments.items():
            observation = observations[index]
            item = self.items[uid]
            old_raw_id = item.raw_track_id
            if self.track_to_uid.get(old_raw_id) == uid:
                self.track_to_uid.pop(old_raw_id, None)
            item.update(observation, stamp_s)
            item.occluded_until_s = max(item.occluded_until_s, stamp_s)
            self.track_to_uid[observation.raw_track_id] = uid

        for index in normal_indices:
            if index in assigned_observations:
                continue
            observation = observations[index]
            uid = self.next_uid
            self.next_uid += 1
            item = LogicalBox(
                uid=uid,
                sequence=self.next_sequence,
                raw_track_id=observation.raw_track_id,
                x=observation.x,
                y=observation.y,
                length=observation.length,
                width=observation.width,
                yaw=observation.yaw,
                confidence=observation.confidence,
                first_seen_s=stamp_s,
                stamp_s=stamp_s,
                last_command_speed=self.idle_speed,
            )
            self.next_sequence += 1
            self.items[uid] = item
            self.track_to_uid[observation.raw_track_id] = uid

        self.last_reidentified_tracks = reidentified

    def _detect_merged_observations(
        self,
        observations: list[ObservedBox],
        stamp_s: float,
    ) -> tuple[set[int], dict[int, list[int]]]:
        merge_indices: set[int] = set()
        merge_members: dict[int, list[int]] = {}

        recent_items = [
            item for item in self.items.values()
            if stamp_s - item.stamp_s <= self.logical_item_expiration
        ]
        for index, observation in enumerate(observations):
            covered: list[LogicalBox] = []
            for item in recent_items:
                predicted_x = item.predicted_x(
                    stamp_s,
                    self.maximum_speed,
                    self.prediction_max_horizon,
                )
                predicted_y = item.predicted_y(
                    stamp_s,
                    self.prediction_max_horizon,
                )
                if (
                    abs(predicted_x - observation.x)
                    <= observation.projected_half_length
                    + self.merge_padding_x
                    and abs(predicted_y - observation.y)
                    <= observation.projected_half_width
                    + self.merge_padding_y
                ):
                    covered.append(item)

            if len(covered) < 2:
                continue
            largest_member = max(item.footprint_area for item in covered)
            if (
                observation.footprint_area
                < self.merge_area_ratio * largest_member
            ):
                continue

            # At least two logical products lie inside one enlarged contour.
            merge_indices.add(index)
            merge_members[index] = [item.uid for item in covered]

        return merge_indices, merge_members

    def _observation_matches_item(
        self,
        observation: ObservedBox,
        item: LogicalBox,
        stamp_s: float,
        *,
        relaxed: bool,
    ) -> bool:
        predicted_x = item.predicted_x(
            stamp_s,
            self.maximum_speed,
            self.prediction_max_horizon,
        )
        predicted_y = item.predicted_y(
            stamp_s,
            self.prediction_max_horizon,
        )
        dx = observation.x - predicted_x
        dy = observation.y - predicted_y
        scale = 1.25 if relaxed else 1.0
        if dx < -scale * self.reid_max_backward_distance:
            return False
        if abs(dy) > scale * self.reid_max_lateral_distance:
            return False
        return math.hypot(dx, dy) <= scale * self.reid_max_distance

    def _reidentification_cost(
        self,
        observation: ObservedBox,
        item: LogicalBox,
        stamp_s: float,
    ) -> float | None:
        if not self._observation_matches_item(
            observation,
            item,
            stamp_s,
            relaxed=True,
        ):
            return None

        predicted_x = item.predicted_x(
            stamp_s,
            self.maximum_speed,
            self.prediction_max_horizon,
        )
        predicted_y = item.predicted_y(
            stamp_s,
            self.prediction_max_horizon,
        )
        dx = observation.x - predicted_x
        dy = observation.y - predicted_y
        position_cost = (
            (dx / max(self.reid_max_distance, 1.0e-6)) ** 2
            + 2.5
            * (
                dy / max(self.reid_max_lateral_distance, 1.0e-6)
            ) ** 2
        )
        length_error = abs(
            math.log(observation.length / max(item.length, 1.0e-6))
        )
        width_error = abs(
            math.log(observation.width / max(item.width, 1.0e-6))
        )
        return position_cost + 0.20 * (length_error + width_error)

    def _update_entry_queue(self, now_s: float) -> None:
        self.global_order = [
            uid for uid in self.global_order if uid in self.items
        ]
        self.pending_wave_uids = [
            uid
            for uid in self.pending_wave_uids
            if uid in self.items and not self.items[uid].queued
        ]
        if not self.pending_wave_uids:
            self.pending_wave_opened_s = None

        def near_entry(item: LogicalBox) -> bool:
            x = item.predicted_x(
                now_s,
                self.maximum_speed,
                self.prediction_max_horizon,
            )
            return (
                self.matrix_min_x - self.entry_capture_back_margin
                <= x
                <= self.entry_gate_x + self.entry_capture_front_margin
            )

        unqueued = [
            item
            for item in self.items.values()
            if not item.queued and item.uid not in self.pending_wave_uids
        ]
        entry_candidates = sorted(
            (item for item in unqueued if near_entry(item)),
            key=lambda item: (item.first_seen_s, item.sequence),
        )

        if not self.pending_wave_uids and entry_candidates:
            first = entry_candidates.pop(0)
            self.pending_wave_uids = [first.uid]
            self.pending_wave_opened_s = first.first_seen_s

        if self.pending_wave_uids:
            assert self.pending_wave_opened_s is not None
            pending_items = [self.items[uid] for uid in self.pending_wave_uids]
            pending_x = [
                item.predicted_x(
                    now_s,
                    self.maximum_speed,
                    self.prediction_max_horizon,
                )
                for item in pending_items
            ]
            for item in list(entry_candidates):
                if len(self.pending_wave_uids) >= self.entry_wave_max_size:
                    break
                item_x = item.predicted_x(
                    now_s,
                    self.maximum_speed,
                    self.prediction_max_horizon,
                )
                combined = pending_x + [item_x]
                within_time = (
                    item.first_seen_s - self.pending_wave_opened_s
                    <= self.entry_capture_window
                )
                within_x = max(combined) - min(combined) <= self.entry_wave_dx
                if within_time and within_x:
                    self.pending_wave_uids.append(item.uid)
                    pending_x.append(item_x)
                    entry_candidates.remove(item)

            pending_items = [self.items[uid] for uid in self.pending_wave_uids]
            front_x = max(
                item.predicted_x(
                    now_s,
                    self.maximum_speed,
                    self.prediction_max_horizon,
                )
                for item in pending_items
            )
            capture_expired = (
                now_s - self.pending_wave_opened_s
                >= self.entry_capture_window
            )
            crossed_gate = front_x >= self.entry_gate_x
            full = len(pending_items) >= self.entry_wave_max_size
            if capture_expired or crossed_gate or full:
                self._finalise_pending_wave(now_s)

        # A genuinely new track appearing beyond the gate is most likely a raw
        # ID change that escaped re-identification.  Insert it without changing
        # the relative order of any already queued products.
        recovered = 0
        for item in sorted(
            (item for item in self.items.values() if not item.queued),
            key=lambda candidate: candidate.sequence,
        ):
            if item.uid in self.pending_wave_uids:
                continue
            x = item.predicted_x(
                now_s,
                self.maximum_speed,
                self.prediction_max_horizon,
            )
            if x <= self.entry_gate_x + self.entry_capture_front_margin:
                continue
            self._insert_recovered_item(item, now_s)
            recovered += 1
        self.last_recovered_orphans = recovered

    def _finalise_pending_wave(self, now_s: float) -> None:
        del now_s
        if not self.pending_wave_uids:
            return
        members = [
            self.items[uid]
            for uid in self.pending_wave_uids
            if uid in self.items and not self.items[uid].queued
        ]
        members.sort(key=lambda item: (-item.y, item.sequence))
        wave_id = self.next_wave_id
        self.next_wave_id += 1
        for item in members:
            item.wave_id = wave_id
            item.queued = True
            self.global_order.append(item.uid)
        self.pending_wave_uids = []
        self.pending_wave_opened_s = None

    def _insert_recovered_item(
        self,
        item: LogicalBox,
        now_s: float,
    ) -> None:
        item.wave_id = self.next_wave_id
        self.next_wave_id += 1
        item.queued = True
        item_x = item.predicted_x(
            now_s,
            self.maximum_speed,
            self.prediction_max_horizon,
        )

        insert_at = len(self.global_order)
        for index, uid in enumerate(self.global_order):
            other = self.items.get(uid)
            if other is None:
                continue
            other_x = other.predicted_x(
                now_s,
                self.maximum_speed,
                self.prediction_max_horizon,
            )
            if item_x > other_x:
                insert_at = index
                break
        self.global_order.insert(insert_at, item.uid)

    def _cleanup_items(self, now_s: float) -> None:
        remove_uids: list[int] = []
        for uid, item in self.items.items():
            predicted_x = item.predicted_x(
                now_s,
                self.maximum_speed,
                self.prediction_max_horizon,
            )
            exited = predicted_x > self.matrix_max_x + self.exit_remove_margin
            stale = (
                now_s - item.stamp_s > self.logical_item_expiration
                and now_s > item.occluded_until_s
            )
            if exited or stale:
                remove_uids.append(uid)

        for uid in remove_uids:
            item = self.items.pop(uid, None)
            if item is None:
                continue
            if self.track_to_uid.get(item.raw_track_id) == uid:
                self.track_to_uid.pop(item.raw_track_id, None)
            self.global_order = [value for value in self.global_order if value != uid]
            self.pending_wave_uids = [
                value for value in self.pending_wave_uids if value != uid
            ]

    def _on_control_timer(self) -> None:
        now_s = self._now_seconds()
        if self.last_publish_s is None:
            dt = 1.0 / max(1.0, self.publish_rate)
        else:
            dt = clamp(now_s - self.last_publish_s, 0.001, 0.25)
        self.last_publish_s = now_s

        observation_is_fresh = (
            self.last_observation_stamp_s is not None
            and now_s - self.last_observation_stamp_s
            <= self.observation_timeout
        )

        self._reset_cycle_diagnostics()
        if observation_is_fresh:
            desired = self._build_cell_speeds(now_s)
        else:
            desired = [self.idle_speed] * (self.rows * self.cols)

        limited = self._limit_command_rate(desired, dt)
        self.last_command = limited
        self._publish_command(limited)
        self._publish_diagnostics(now_s, observation_is_fresh)

    def _active_items(self, now_s: float) -> list[LogicalBox]:
        margin = 0.50
        result: list[LogicalBox] = []
        for item in self.items.values():
            age = now_s - item.stamp_s
            active_by_observation = age <= self.normal_track_timeout
            active_by_merge = now_s <= item.occluded_until_s
            if not (active_by_observation or active_by_merge):
                continue
            x = item.predicted_x(
                now_s,
                self.maximum_speed,
                self.prediction_max_horizon,
            )
            y = item.predicted_y(now_s, self.prediction_max_horizon)
            if not (
                self.matrix_min_x - margin
                <= x
                <= self.matrix_max_x + margin
            ):
                continue
            if abs(y) > self.matrix_width / 2.0 + 0.15:
                continue
            result.append(item)
        return result

    def _build_cell_speeds(self, now_s: float) -> list[float]:
        active = self._active_items(now_s)
        self.last_active_boxes = len(active)
        self.last_queue_size = len(self.global_order)
        self.last_ghost_tracks = sum(
            item.is_ghost(now_s, 0.12) for item in active
        )
        if not active:
            return [self.idle_speed] * (self.rows * self.cols)

        active_by_uid = {item.uid: item for item in active}
        ordered = [
            active_by_uid[uid]
            for uid in self.global_order
            if uid in active_by_uid
        ]
        unqueued = [item for item in active if not item.queued]

        control_x: dict[int, float] = {}
        control_y: dict[int, float] = {}
        for item in active:
            predicted_x = item.predicted_x(
                now_s,
                self.maximum_speed,
                self.prediction_max_horizon,
            )
            predicted_y = item.predicted_y(
                now_s,
                self.prediction_max_horizon,
            )
            control_x[item.uid] = (
                predicted_x
                + item.prediction_speed(self.maximum_speed)
                * self.prediction_horizon
            )
            control_y[item.uid] = predicted_y

        states: list[GapState] = []
        for index, item in enumerate(ordered):
            target_gap = self.target_gap
            if index + 1 < len(ordered):
                follower = ordered[index + 1]
                if item.wave_id != follower.wave_id:
                    target_gap = self.inter_wave_target_gap
            states.append(
                GapState(
                    uid=item.uid,
                    x=control_x[item.uid],
                    half_length=item.projected_half_length,
                    vx=item.vx,
                    target_gap_to_follower=target_gap,
                )
            )

        # Near the throat a proportional controller may detect a small deficit
        # too late.  Add the minimum relative speed that can create a safe
        # clearance before the leading edge reaches the matrix exit.
        deadline_deltas: list[float] = []
        deadline_boost_pairs = 0
        for index, (leader, follower) in enumerate(zip(ordered, ordered[1:])):
            clearance = (
                states[index].x
                - states[index].half_length
                - states[index + 1].x
                - states[index + 1].half_length
            )
            remaining_distance = self.matrix_max_x - max(
                control_x[leader.uid], control_x[follower.uid]
            )
            deadline_delta = 0.0
            if remaining_distance < self.deadline_separation_distance:
                time_to_exit = max(
                    self.deadline_min_time,
                    max(0.0, remaining_distance)
                    / max(self.transport_speed, 1.0e-6),
                )
                exit_deficit = max(
                    0.0,
                    states[index].target_gap_to_follower
                    + self.deadline_gap_margin
                    - clearance,
                )
                deadline_delta = (
                    self.deadline_recovery_gain * exit_deficit / time_to_exit
                )
                nominal_delta = (
                    self.gap_gain
                    * max(0.0, states[index].target_gap_to_follower - clearance)
                    + self.relative_velocity_gain
                    * max(0.0, states[index + 1].vx - states[index].vx)
                )
                if deadline_delta > nominal_delta + 1.0e-6:
                    deadline_boost_pairs += 1
            deadline_deltas.append(deadline_delta)

        profile = build_pairwise_speed_profile(
            states,
            transport_speed=self.transport_speed,
            minimum_speed=self.minimum_speed,
            maximum_speed=self.maximum_speed,
            gap_gain=self.gap_gain,
            relative_velocity_gain=self.relative_velocity_gain,
            maximum_relative_speed=self.maximum_relative_speed,
            inversion_margin=self.order_inversion_margin,
            minimum_delta_by_pair=deadline_deltas,
        )
        self.last_deadline_boost_pairs = deadline_boost_pairs
        self.last_order_inversions = profile.inversion_count
        self.last_min_adjacent_gap = min(
            profile.clearance_by_pair,
            default=math.inf,
        )

        target_speed = dict(profile.speed_by_uid)
        urgency = dict(profile.urgency_by_uid)
        for item in unqueued:
            target_speed[item.uid] = self.idle_speed
            urgency[item.uid] = 0.05

        dense_uids: set[int] = set()
        unresolved_at_exit = 0
        for index, clearance in enumerate(profile.clearance_by_pair):
            leader = ordered[index]
            follower = ordered[index + 1]
            target = states[index].target_gap_to_follower
            if clearance < self.dense_pair_yaw_gap:
                dense_uids.add(leader.uid)
                dense_uids.add(follower.uid)
            near_exit = (
                max(control_x[leader.uid], control_x[follower.uid])
                >= self.matrix_max_x - self.exit_gap_check_margin
            )
            if near_exit and clearance < 0.85 * target:
                unresolved_at_exit += 1
        self.last_unresolved_at_exit = unresolved_at_exit

        proposals: list[list[CellProposal]] = [
            [] for _ in range(self.rows * self.cols)
        ]
        contacts: dict[int, list[tuple[int, float]]] = {}
        for item in active:
            uid = item.uid
            contacts[uid] = []
            ghost = item.is_ghost(now_s, 0.12)
            for row, col, overlap in self._overlapped_cells(
                item,
                control_x[uid],
                control_y[uid],
            ):
                speed = target_speed.get(uid, self.idle_speed)
                if (
                    uid not in dense_uids
                    and not ghost
                    and control_x[uid]
                    < self.matrix_max_x - self.yaw_control_exit_margin
                ):
                    speed += self._yaw_correction(
                        item,
                        col,
                        control_y[uid],
                    )
                speed = clamp(
                    speed,
                    self.minimum_speed,
                    self.maximum_speed,
                )
                index = row * self.cols + col
                proposals[index].append(
                    CellProposal(
                        uid=uid,
                        speed=speed,
                        overlap=overlap,
                        urgency=urgency.get(uid, 0.0),
                    )
                )
                contacts[uid].append((index, overlap))

        # Solve against the effective (overlap-weighted) speed of each box,
        # instead of independently averaging every shared cell.  This lets a
        # partially shared contact patch still create a useful speed gradient.
        target_speed_for_allocation: dict[int, float] = {}
        for uid, item_contacts in contacts.items():
            total_overlap = sum(overlap for _, overlap in item_contacts)
            if total_overlap <= 1.0e-9:
                continue
            proposal_by_cell = {
                index: proposal.speed
                for index, cell_proposals in enumerate(proposals)
                for proposal in cell_proposals
                if proposal.uid == uid
            }
            target_speed_for_allocation[uid] = sum(
                proposal_by_cell.get(index, target_speed.get(uid, self.idle_speed))
                * overlap
                for index, overlap in item_contacts
            ) / total_overlap

        result = allocate_cell_speeds(
            contacts,
            target_speed_for_allocation,
            urgency,
            cell_count=self.rows * self.cols,
            idle_speed=self.idle_speed,
            minimum_speed=self.minimum_speed,
            maximum_speed=self.maximum_speed,
            urgency_gain=self.allocation_urgency_gain,
            idle_regularization=self.allocation_idle_regularization,
            iterations=self.allocation_iterations,
        )
        shared_cells = 0
        for cell_proposals in proposals:
            unique_uids = {proposal.uid for proposal in cell_proposals}
            if len(unique_uids) > 1:
                shared_cells += 1
        self.last_shared_cells = shared_cells

        allocation_errors: list[float] = []
        contact_vectors: dict[int, list[float]] = {}
        for item in active:
            vector = [0.0] * (self.rows * self.cols)
            total_overlap = sum(weight for _, weight in contacts[item.uid])
            effective_speed = self.idle_speed
            if total_overlap > 1.0e-9:
                effective_speed = sum(
                    result[index] * weight
                    for index, weight in contacts[item.uid]
                ) / total_overlap
                for index, weight in contacts[item.uid]:
                    vector[index] = weight / total_overlap
            item.last_command_speed = effective_speed
            contact_vectors[item.uid] = vector
            allocation_errors.append(
                abs(
                    effective_speed
                    - target_speed_for_allocation.get(
                        item.uid,
                        self.idle_speed,
                    )
                )
            )
        self.last_allocation_error = max(allocation_errors, default=0.0)

        uncontrollable = 0
        for index, clearance in enumerate(profile.clearance_by_pair):
            if clearance >= states[index].target_gap_to_follower:
                continue
            leader_uid = ordered[index].uid
            follower_uid = ordered[index + 1].uid
            similarity = cosine_similarity(
                contact_vectors.get(leader_uid, []),
                contact_vectors.get(follower_uid, []),
            )
            if similarity >= self.uncontrollable_similarity:
                uncontrollable += 1
        self.last_uncontrollable_pairs = uncontrollable

        return result

    def _overlapped_cells(
        self,
        item: LogicalBox,
        predicted_x: float,
        predicted_y: float,
    ) -> Iterable[tuple[int, int, float]]:
        half_x = item.projected_half_length + self.longitudinal_control_margin
        half_y = item.projected_half_width
        box_min_x = predicted_x - half_x
        box_max_x = predicted_x + half_x
        box_min_y = predicted_y - half_y
        box_max_y = predicted_y + half_y

        for row in range(self.rows):
            centre_x = self.matrix_center_x + (
                row - (self.rows - 1) / 2.0
            ) * self.pitch_x
            cell_min_x = centre_x - self.cell_length / 2.0
            cell_max_x = centre_x + self.cell_length / 2.0
            overlap_x = min(box_max_x, cell_max_x) - max(
                box_min_x,
                cell_min_x,
            )
            if overlap_x <= 0.0:
                continue

            for col in range(self.cols):
                centre_y = (
                    col - (self.cols - 1) / 2.0
                ) * self.pitch_y
                cell_min_y = centre_y - self.cell_width / 2.0
                cell_max_y = centre_y + self.cell_width / 2.0
                overlap_y = min(box_max_y, cell_max_y) - max(
                    box_min_y,
                    cell_min_y,
                )
                if overlap_y <= 0.0:
                    continue
                yield row, col, overlap_x * overlap_y

    def _yaw_correction(
        self,
        item: LogicalBox,
        col: int,
        predicted_y: float,
    ) -> float:
        yaw_error = angle_error(item.yaw, 0.0)
        centre_y = (
            col - (self.cols - 1) / 2.0
        ) * self.pitch_y
        lever_scale = max(item.projected_half_width, self.pitch_y)
        side = clamp(
            (centre_y - predicted_y) / lever_scale,
            -1.0,
            1.0,
        )
        correction = (
            self.yaw_gain * yaw_error
            + self.yaw_rate_gain * item.yaw_rate
        ) * side
        return clamp(
            correction,
            -self.maximum_yaw_delta,
            self.maximum_yaw_delta,
        )

    def _limit_command_rate(
        self,
        desired: list[float],
        dt: float,
    ) -> list[float]:
        maximum_step = self.maximum_acceleration * dt
        result: list[float] = []
        for previous, target in zip(self.last_command, desired):
            target = clamp(
                target,
                self.minimum_speed,
                self.maximum_speed,
            )
            result.append(
                previous
                + clamp(
                    target - previous,
                    -maximum_step,
                    maximum_step,
                )
            )
        return result

    def _publish_command(self, speeds: list[float]) -> None:
        message = MatrixCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "singulator_matrix"
        message.rows = self.rows
        message.cols = self.cols
        message.target_speed_mps = [float(value) for value in speeds]
        self.command_publisher.publish(message)

    def _reset_cycle_diagnostics(self) -> None:
        self.last_active_boxes = 0
        self.last_queue_size = len(self.global_order)
        self.last_ghost_tracks = 0
        self.last_order_inversions = 0
        self.last_unresolved_at_exit = 0
        self.last_deadline_boost_pairs = 0
        self.last_min_adjacent_gap = math.inf
        self.last_shared_cells = 0
        self.last_uncontrollable_pairs = 0
        self.last_allocation_error = 0.0

    def _publish_diagnostics(
        self,
        now_s: float,
        observation_is_fresh: bool,
    ) -> None:
        if now_s - self.last_diagnostic_s < self.diagnostic_period:
            return
        self.last_diagnostic_s = now_s
        min_gap = (
            "n/a"
            if not math.isfinite(self.last_min_adjacent_gap)
            else f"{self.last_min_adjacent_gap:.3f} m"
        )
        queue_preview = ",".join(
            str(uid) for uid in self.global_order[:12]
        )
        self.get_logger().info(
            "control_v7: "
            f"vision={'fresh' if observation_is_fresh else 'stale'}, "
            f"boxes={self.last_active_boxes}, "
            f"queue={self.last_queue_size}[{queue_preview}], "
            f"ghosts={self.last_ghost_tracks}, "
            f"merged={self.last_merged_observations}, "
            f"reidentified={self.last_reidentified_tracks}, "
            f"orphans={self.last_recovered_orphans}, "
            f"inversions={self.last_order_inversions}, "
            f"unresolved_exit={self.last_unresolved_at_exit}, "
            f"deadline_boost={self.last_deadline_boost_pairs}, "
            f"min_gap={min_gap}, "
            f"shared_cells={self.last_shared_cells}, "
            f"uncontrollable={self.last_uncontrollable_pairs}, "
            f"allocation_error={self.last_allocation_error:.3f} m/s, "
            f"speed=[{min(self.last_command):.2f}, "
            f"{max(self.last_command):.2f}] m/s"
        )

    @staticmethod
    def _stamp_to_seconds(message: BoxObservationArray) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SingulationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
