"""Closed-loop baseline controller for the 14x4 conveyor singulator.

The node consumes camera observations and publishes one longitudinal velocity
for every matrix cell.  It creates a single longitudinal queue by combining:

* pairwise gap control between ordered boxes;
* differential left/right cell speeds for yaw correction;
* overlap-based mapping from each box footprint to matrix cells;
* temporal speed limiting and conservative conflict resolution.

This is intentionally a deterministic, inspectable baseline rather than a
black-box optimiser.  It is suitable for simulation tuning and for collecting
data before a later MPC/optimisation controller is introduced.
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


@dataclass(slots=True)
class TrackedBox:
    track_id: int
    sequence: int
    x: float
    y: float
    length: float
    width: float
    yaw: float
    confidence: float
    stamp_s: float
    vx: float = 0.0
    yaw_rate: float = 0.0
    lag_reference_x: float | None = None
    lag_reference_stamp_s: float | None = None
    longitudinal_lag_m: float = 0.0

    def update(self, observation: BoxObservation, stamp_s: float) -> None:
        dt = stamp_s - self.stamp_s
        if dt > 1.0e-4:
            measured_vx = (float(observation.center.x) - self.x) / dt
            measured_yaw_rate = angle_error(
                float(observation.yaw_rad),
                self.yaw,
            ) / dt
            alpha = 0.45
            self.vx = alpha * measured_vx + (1.0 - alpha) * self.vx
            self.yaw_rate = (
                alpha * measured_yaw_rate
                + (1.0 - alpha) * self.yaw_rate
            )

        self.x = float(observation.center.x)
        self.y = float(observation.center.y)
        self.length = max(0.02, float(observation.length_m))
        self.width = max(0.01, float(observation.width_m))
        self.yaw = normalise_half_turn(float(observation.yaw_rad))
        self.confidence = float(observation.confidence)
        self.stamp_s = stamp_s

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


@dataclass(slots=True)
class CellProposal:
    speed: float
    weight: float
    track_id: int
    track_x: float
    queue_rank: int


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalise_half_turn(angle: float) -> float:
    """Normalise rectangle yaw to [-pi/2, pi/2)."""
    return ((angle + math.pi / 2.0) % math.pi) - math.pi / 2.0


def angle_error(angle: float, reference: float) -> float:
    return normalise_half_turn(angle - reference)


class SingulationController(Node):
    """Vision-feedback controller that forms a longitudinal single-file queue."""

    def __init__(self) -> None:
        super().__init__("singulation_controller")

        self.declare_parameter("boxes_topic", "/singulator/boxes")
        self.declare_parameter(
            "command_topic",
            "/singulator/matrix/command",
        )
        self.declare_parameter("rows", 14)
        self.declare_parameter("cols", 4)
        self.declare_parameter("cell_length_m", 0.360)
        self.declare_parameter("cell_width_m", 0.175)
        self.declare_parameter("gap_x_m", 0.020)
        self.declare_parameter("gap_y_m", 0.020)
        self.declare_parameter("base_speed_mps", 2.00)
        self.declare_parameter("minimum_speed_mps", 0.35)
        self.declare_parameter("maximum_speed_mps", 3.00)
        self.declare_parameter("leader_speed_mps", 2.80)
        self.declare_parameter("target_gap_m", 0.18)
        self.declare_parameter("hard_gap_m", 0.035)
        self.declare_parameter("gap_gain", 2.20)
        self.declare_parameter("relative_velocity_gain", 0.45)
        self.declare_parameter("leader_boost_gain", 1.20)
        self.declare_parameter("nominal_transport_speed_mps", 2.00)
        self.declare_parameter("maximum_longitudinal_lag_m", 0.30)
        self.declare_parameter("lag_guard_horizon_s", 0.20)
        self.declare_parameter("lag_recovery_gain", 2.00)
        self.declare_parameter("prediction_horizon_s", 0.18)
        self.declare_parameter("longitudinal_control_margin_m", 0.16)
        self.declare_parameter("yaw_gain", 1.35)
        self.declare_parameter("yaw_rate_gain", 0.10)
        self.declare_parameter("maximum_yaw_delta_mps", 0.90)
        self.declare_parameter("enable_lateral_steering", False)
        self.declare_parameter("lateral_to_yaw_gain", 0.70)
        self.declare_parameter("maximum_target_yaw_rad", 0.30)
        self.declare_parameter("minimum_confidence", 0.20)
        self.declare_parameter("observation_timeout_s", 0.65)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("maximum_acceleration_mps2", 3.0)
        self.declare_parameter("diagnostic_period_s", 1.0)

        self.rows = int(self.get_parameter("rows").value)
        self.cols = int(self.get_parameter("cols").value)
        if self.rows <= 0 or self.cols <= 0:
            raise ValueError("rows and cols must be positive")

        self.cell_length = float(
            self.get_parameter("cell_length_m").value
        )
        self.cell_width = float(
            self.get_parameter("cell_width_m").value
        )
        self.gap_x = float(self.get_parameter("gap_x_m").value)
        self.gap_y = float(self.get_parameter("gap_y_m").value)
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
        self.matrix_min_x = -self.matrix_length / 2.0
        self.matrix_max_x = self.matrix_length / 2.0

        self.base_speed = float(
            self.get_parameter("base_speed_mps").value
        )
        self.minimum_speed = float(
            self.get_parameter("minimum_speed_mps").value
        )
        self.maximum_speed = float(
            self.get_parameter("maximum_speed_mps").value
        )
        self.leader_speed = float(
            self.get_parameter("leader_speed_mps").value
        )
        self.target_gap = float(self.get_parameter("target_gap_m").value)
        self.hard_gap = float(self.get_parameter("hard_gap_m").value)
        self.gap_gain = float(self.get_parameter("gap_gain").value)
        self.relative_velocity_gain = float(
            self.get_parameter("relative_velocity_gain").value
        )
        self.leader_boost_gain = float(
            self.get_parameter("leader_boost_gain").value
        )
        self.nominal_transport_speed = float(
            self.get_parameter("nominal_transport_speed_mps").value
        )
        self.maximum_longitudinal_lag = float(
            self.get_parameter("maximum_longitudinal_lag_m").value
        )
        self.lag_guard_horizon = float(
            self.get_parameter("lag_guard_horizon_s").value
        )
        self.lag_recovery_gain = float(
            self.get_parameter("lag_recovery_gain").value
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
        self.enable_lateral_steering = bool(
            self.get_parameter("enable_lateral_steering").value
        )
        self.lateral_to_yaw_gain = float(
            self.get_parameter("lateral_to_yaw_gain").value
        )
        self.maximum_target_yaw = float(
            self.get_parameter("maximum_target_yaw_rad").value
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
        self.maximum_acceleration = float(
            self.get_parameter("maximum_acceleration_mps2").value
        )
        self.diagnostic_period = float(
            self.get_parameter("diagnostic_period_s").value
        )

        if self.minimum_speed <= 0.0:
            raise ValueError(
                "minimum_speed_mps must be positive; reverse motion is disabled"
            )
        if self.nominal_transport_speed <= 0.0:
            raise ValueError("nominal_transport_speed_mps must be positive")
        if self.maximum_longitudinal_lag <= 0.0:
            raise ValueError("maximum_longitudinal_lag_m must be positive")
        if self.lag_guard_horizon <= 0.0:
            raise ValueError("lag_guard_horizon_s must be positive")
        if not (
            self.minimum_speed
            <= self.base_speed
            <= self.maximum_speed
        ):
            raise ValueError(
                "Expected minimum_speed <= base_speed <= maximum_speed"
            )

        self.tracks: dict[int, TrackedBox] = {}
        self.next_sequence = 1
        self.last_observation_stamp_s: float | None = None
        self.last_command = [self.base_speed] * (self.rows * self.cols)
        self.last_publish_s: float | None = None
        self.last_diagnostic_s = -math.inf
        self.last_min_gap = math.inf
        self.last_conflict_cells = 0
        self.last_active_boxes = 0
        self.last_max_lag = 0.0

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
            "Closed-loop singulation controller started: "
            f"{self.rows}x{self.cols}, base={self.base_speed:.2f} m/s, "
            f"gap={self.target_gap:.2f} m, "
            f"forward_only>={self.minimum_speed:.2f} m/s, "
            f"lag_limit={self.maximum_longitudinal_lag:.2f} m"
        )

    def _on_observations(self, message: BoxObservationArray) -> None:
        stamp_s = self._stamp_to_seconds(message)
        if stamp_s <= 0.0:
            stamp_s = self._now_seconds()
        self.last_observation_stamp_s = stamp_s

        observed_ids: set[int] = set()
        for observation in message.boxes:
            if float(observation.confidence) < self.minimum_confidence:
                continue
            track_id = int(observation.id)
            observed_ids.add(track_id)
            track = self.tracks.get(track_id)
            if track is None:
                self.tracks[track_id] = TrackedBox(
                    track_id=track_id,
                    sequence=self.next_sequence,
                    x=float(observation.center.x),
                    y=float(observation.center.y),
                    length=max(0.02, float(observation.length_m)),
                    width=max(0.01, float(observation.width_m)),
                    yaw=normalise_half_turn(float(observation.yaw_rad)),
                    confidence=float(observation.confidence),
                    stamp_s=stamp_s,
                )
                self.next_sequence += 1
            else:
                track.update(observation, stamp_s)

        # A track is retained briefly through one or two missed camera frames.
        expiration = max(1.0, 2.5 * self.observation_timeout)
        for track_id, track in list(self.tracks.items()):
            if stamp_s - track.stamp_s > expiration:
                del self.tracks[track_id]

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

        if observation_is_fresh:
            active = self._active_tracks(now_s)
            desired = self._build_cell_speeds(active)
        else:
            active = []
            desired = [self.base_speed] * (self.rows * self.cols)
            self.last_min_gap = math.inf
            self.last_conflict_cells = 0
            self.last_max_lag = 0.0

        limited = self._limit_command_rate(desired, dt)
        self.last_command = limited
        self.last_active_boxes = len(active)
        self._publish_command(limited)
        self._publish_diagnostics(now_s, observation_is_fresh)

    def _active_tracks(self, now_s: float) -> list[TrackedBox]:
        margin = 0.45
        return [
            track
            for track in self.tracks.values()
            if now_s - track.stamp_s <= self.observation_timeout
            and self.matrix_min_x - margin
            <= track.x
            <= self.matrix_max_x + margin
            and abs(track.y)
            <= self.matrix_width / 2.0 + 0.15
        ]

    def _build_cell_speeds(
        self,
        active_tracks: list[TrackedBox],
    ) -> list[float]:
        if not active_tracks:
            self.last_min_gap = math.inf
            self.last_conflict_cells = 0
            self.last_max_lag = 0.0
            return [self.base_speed] * (self.rows * self.cols)

        # Current longitudinal position is the primary ordering signal.  The
        # persistent sequence is only a deterministic tie-breaker for a wave of
        # side-by-side boxes.  This avoids keeping a wrong order after one box
        # has already moved ahead of another.
        queue = sorted(
            active_tracks,
            key=lambda item: (-item.x, item.sequence),
        )
        speed_floors = {
            track.track_id: self._lag_limited_speed_floor(track)
            for track in queue
        }
        self.last_max_lag = max(
            (track.longitudinal_lag_m for track in queue),
            default=0.0,
        )

        box_speeds: dict[int, float] = {}
        box_speeds[queue[0].track_id] = clamp(
            max(self.leader_speed, speed_floors[queue[0].track_id]),
            self.minimum_speed,
            self.maximum_speed,
        )

        minimum_clearance = math.inf
        predecessor = queue[0]
        for follower in queue[1:]:
            centre_distance = predecessor.x - follower.x
            body_distance = (
                predecessor.projected_half_length
                + follower.projected_half_length
            )
            clearance = centre_distance - body_distance
            minimum_clearance = min(minimum_clearance, clearance)

            closing_speed = max(0.0, follower.vx - predecessor.vx)
            braking_distance = (
                closing_speed * closing_speed
                / max(2.0 * self.maximum_acceleration, 1.0e-6)
            )
            dynamic_target_gap = self.target_gap + braking_distance
            spacing_error = clearance - dynamic_target_gap
            predecessor_speed = box_speeds[predecessor.track_id]
            desired_speed = (
                predecessor_speed
                + self.gap_gain * spacing_error
                - self.relative_velocity_gain * closing_speed
            )

            if clearance < self.hard_gap:
                # Forward-only emergency separation: accelerate the front box
                # and slow the rear box, but never reverse it.  The rear speed is
                # additionally bounded by its remaining longitudinal lag budget.
                boost = self.leader_boost_gain * (
                    self.hard_gap - clearance
                )
                box_speeds[predecessor.track_id] = clamp(
                    max(
                        predecessor_speed,
                        self.leader_speed + boost,
                        speed_floors[predecessor.track_id],
                    ),
                    self.minimum_speed,
                    self.maximum_speed,
                )
                desired_speed = self.minimum_speed

            box_speeds[follower.track_id] = clamp(
                max(desired_speed, speed_floors[follower.track_id]),
                self.minimum_speed,
                self.maximum_speed,
            )
            predecessor = follower

        self.last_min_gap = minimum_clearance

        proposals: list[list[CellProposal]] = [
            [] for _ in range(self.rows * self.cols)
        ]
        for queue_rank, track in enumerate(queue):
            base_box_speed = box_speeds[track.track_id]
            predicted_x = track.x + clamp(
                track.vx,
                0.0,
                self.maximum_speed,
            ) * self.prediction_horizon
            for row, col, weight in self._overlapped_cells(
                track,
                predicted_x,
            ):
                speed = max(
                    speed_floors[track.track_id],
                    base_box_speed + self._yaw_correction(
                        track,
                        col,
                    ),
                )
                proposals[row * self.cols + col].append(
                    CellProposal(
                        speed=clamp(
                            speed,
                            self.minimum_speed,
                            self.maximum_speed,
                        ),
                        weight=weight,
                        track_id=track.track_id,
                        track_x=predicted_x,
                        queue_rank=queue_rank,
                    )
                )

        result = [self.base_speed] * (self.rows * self.cols)
        conflict_cells = 0
        for index, cell_proposals in enumerate(proposals):
            if not cell_proposals:
                continue
            unique_tracks = {item.track_id for item in cell_proposals}
            if len(unique_tracks) > 1:
                conflict_cells += 1
                row = index // self.cols
                cell_centre_x = (
                    row - (self.rows - 1) / 2.0
                ) * self.pitch_x
                # A shared actuator is assigned to the nearest predicted box.
                # The small queue-rank term gives the front box priority only
                # when distances are practically equal.  Adjacent rows can
                # therefore accelerate the leader and brake the follower rather
                # than applying one minimum speed to both.
                owner = min(
                    cell_proposals,
                    key=lambda item: (
                        abs(item.track_x - cell_centre_x),
                        item.queue_rank,
                    ),
                )
                result[index] = owner.speed
                continue
            total_weight = sum(item.weight for item in cell_proposals)
            result[index] = sum(
                item.speed * item.weight for item in cell_proposals
            ) / max(total_weight, 1.0e-9)

        self.last_conflict_cells = conflict_cells
        return result

    def _lag_limited_speed_floor(self, track: TrackedBox) -> float:
        """Return a positive speed floor that respects the lag budget.

        The reference trajectory is a virtual conveyor moving at
        ``nominal_transport_speed_mps`` from the moment the box first becomes
        controllable on the matrix.  The measured longitudinal lag is
        ``x_reference - x_measured``.  The command floor is chosen so that, over
        the guard horizon, additional lag cannot exceed the remaining budget.
        Once the budget is exhausted the floor rises above nominal speed and the
        box is gently recovered instead of being held or reversed.
        """
        if track.lag_reference_x is None:
            track.lag_reference_x = track.x
            track.lag_reference_stamp_s = track.stamp_s
            track.longitudinal_lag_m = 0.0

        assert track.lag_reference_stamp_s is not None
        elapsed = max(0.0, track.stamp_s - track.lag_reference_stamp_s)
        expected_x = (
            track.lag_reference_x
            + self.nominal_transport_speed * elapsed
        )
        lag = max(0.0, expected_x - track.x)
        track.longitudinal_lag_m = lag

        remaining_lag = max(
            0.0,
            self.maximum_longitudinal_lag - lag,
        )
        budget_floor = (
            self.nominal_transport_speed
            - remaining_lag / self.lag_guard_horizon
        )

        if lag > self.maximum_longitudinal_lag:
            budget_floor = max(
                budget_floor,
                self.nominal_transport_speed
                + self.lag_recovery_gain
                * (lag - self.maximum_longitudinal_lag),
            )

        return clamp(
            max(self.minimum_speed, budget_floor),
            self.minimum_speed,
            self.maximum_speed,
        )

    def _overlapped_cells(
        self,
        track: TrackedBox,
        predicted_x: float,
    ) -> Iterable[tuple[int, int, float]]:
        half_x = (
            track.projected_half_length
            + self.longitudinal_control_margin
        )
        half_y = track.projected_half_width
        box_min_x = predicted_x - half_x
        box_max_x = predicted_x + half_x
        box_min_y = track.y - half_y
        box_max_y = track.y + half_y

        for row in range(self.rows):
            centre_x = (
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

    def _yaw_correction(self, track: TrackedBox, col: int) -> float:
        target_yaw = 0.0
        if self.enable_lateral_steering:
            target_yaw = clamp(
                -self.lateral_to_yaw_gain * track.y,
                -self.maximum_target_yaw,
                self.maximum_target_yaw,
            )

        yaw_error = angle_error(track.yaw, target_yaw)
        centre_y = (
            col - (self.cols - 1) / 2.0
        ) * self.pitch_y
        lever_scale = max(
            track.projected_half_width,
            self.pitch_y,
        )
        side = clamp((centre_y - track.y) / lever_scale, -1.0, 1.0)

        # For positive yaw, the +Y side is accelerated.  Its +X traction
        # produces a negative Z torque and rotates the box clockwise to zero.
        correction = (
            self.yaw_gain * yaw_error
            + self.yaw_rate_gain * track.yaw_rate
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
        message.target_speed_mps = [float(item) for item in speeds]
        self.command_publisher.publish(message)

    def _publish_diagnostics(
        self,
        now_s: float,
        observation_is_fresh: bool,
    ) -> None:
        if now_s - self.last_diagnostic_s < self.diagnostic_period:
            return
        self.last_diagnostic_s = now_s
        min_gap_text = (
            "n/a"
            if not math.isfinite(self.last_min_gap)
            else f"{self.last_min_gap:.3f} m"
        )
        self.get_logger().info(
            "control: "
            f"vision={'fresh' if observation_is_fresh else 'stale'}, "
            f"boxes={self.last_active_boxes}, "
            f"min_gap={min_gap_text}, "
            f"conflict_cells={self.last_conflict_cells}, "
            f"max_lag={self.last_max_lag:.3f} m, "
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
