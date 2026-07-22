"""ROS 2 adapter: camera frames -> tracked BoxObservationArray messages."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from singulator_interfaces.msg import BoxObservation, BoxObservationArray

from .detector_core import Detection, DetectorConfig, StreamingBoxDetector


@dataclass(slots=True)
class Track:
    track_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    last_stamp_s: float = 0.0
    misses: int = 0

    def prediction(self, stamp_s: float) -> tuple[float, float]:
        dt = max(0.0, min(0.5, stamp_s - self.last_stamp_s))
        return self.x + self.vx * dt, self.y + self.vy * dt


class NearestNeighbourTracker:
    """Deterministic conveyor tracker with anisotropic motion gating."""

    def __init__(
        self,
        maximum_distance_m: float,
        maximum_lateral_distance_m: float,
        maximum_backward_distance_m: float,
        maximum_misses: int,
    ) -> None:
        self.maximum_distance_m = maximum_distance_m
        self.maximum_lateral_distance_m = maximum_lateral_distance_m
        self.maximum_backward_distance_m = maximum_backward_distance_m
        self.maximum_misses = maximum_misses
        self.next_id = 1
        self.tracks: dict[int, Track] = {}

    def update(
        self,
        positions: Iterable[tuple[float, float]],
        stamp_s: float,
    ) -> list[int]:
        points = list(positions)
        assignments: dict[int, int] = {}
        candidates: list[tuple[float, int, int]] = []

        for track_id, track in self.tracks.items():
            predicted_x, predicted_y = track.prediction(stamp_s)
            for detection_index, (x, y) in enumerate(points):
                dx = x - predicted_x
                dy = y - predicted_y
                if dx < -self.maximum_backward_distance_m:
                    continue
                if abs(dy) > self.maximum_lateral_distance_m:
                    continue
                distance = math.hypot(dx, dy)
                if distance > self.maximum_distance_m:
                    continue
                # Lateral swaps are more damaging than a small longitudinal error.
                cost = (
                    (dx / max(self.maximum_distance_m, 1.0e-6)) ** 2
                    + 2.5
                    * (dy / max(self.maximum_lateral_distance_m, 1.0e-6)) ** 2
                )
                candidates.append((cost, track_id, detection_index))

        used_tracks: set[int] = set()
        used_detections: set[int] = set()
        for _, track_id, detection_index in sorted(candidates):
            if track_id in used_tracks or detection_index in used_detections:
                continue
            assignments[detection_index] = track_id
            used_tracks.add(track_id)
            used_detections.add(detection_index)

        for detection_index, (x, y) in enumerate(points):
            track_id = assignments.get(detection_index)
            if track_id is None:
                track_id = self.next_id
                self.next_id += 1
                self.tracks[track_id] = Track(
                    track_id=track_id,
                    x=x,
                    y=y,
                    last_stamp_s=stamp_s,
                )
                assignments[detection_index] = track_id
                used_tracks.add(track_id)
                continue

            track = self.tracks[track_id]
            dt = stamp_s - track.last_stamp_s
            if dt > 1.0e-4:
                measured_vx = (x - track.x) / dt
                measured_vy = (y - track.y) / dt
                alpha = 0.50
                track.vx = alpha * measured_vx + (1.0 - alpha) * track.vx
                track.vy = alpha * measured_vy + (1.0 - alpha) * track.vy
            track.x = x
            track.y = y
            track.last_stamp_s = stamp_s
            track.misses = 0

        for track_id, track in list(self.tracks.items()):
            if track_id in used_tracks:
                continue
            track.misses += 1
            if track.misses > self.maximum_misses:
                del self.tracks[track_id]

        return [assignments[index] for index in range(len(points))]


class VisionStreamNode(Node):
    def __init__(self) -> None:
        super().__init__("vision_stream_node")

        self.declare_parameter("image_topic", "/singulator/camera/image_raw")
        self.declare_parameter("boxes_topic", "/singulator/boxes")
        self.declare_parameter(
            "debug_image_topic",
            "/singulator/perception/debug_image",
        )
        self.declare_parameter("frame_id", "singulator_matrix")
        self.declare_parameter("dataset_folder", "")
        self.declare_parameter("field_length_m", 7.70)
        self.declare_parameter("field_width_m", 0.90)
        self.declare_parameter("field_min_x_m", -3.95)
        self.declare_parameter("field_max_y_m", 0.45)
        self.declare_parameter("belt_top_z_m", 0.08)
        self.declare_parameter("default_box_height_m", 0.12)
        self.declare_parameter("calibration_frames", 15)
        self.declare_parameter("processing_stride", 1)

        self.declare_parameter("background_threshold", 18)
        self.declare_parameter("use_background_subtraction", True)
        self.declare_parameter("inner_erode_px", 2)
        self.declare_parameter("minimum_contour_area_px", 18.0)
        self.declare_parameter("morphology_open_px", 3)
        self.declare_parameter("morphology_close_px", 3)
        self.declare_parameter("dilate_iterations", 0)
        self.declare_parameter("enable_touching_split", True)
        self.declare_parameter("split_min_contour_area_px", 180.0)
        self.declare_parameter("split_peak_ratio", 0.42)
        self.declare_parameter("split_min_area_fraction", 0.12)

        self.declare_parameter("track_max_distance_m", 0.55)
        self.declare_parameter("track_max_lateral_distance_m", 0.28)
        self.declare_parameter("track_max_backward_distance_m", 0.16)
        self.declare_parameter("track_max_misses", 12)
        self.declare_parameter("publish_debug_image", True)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.boxes_topic = str(self.get_parameter("boxes_topic").value)
        self.debug_image_topic = str(
            self.get_parameter("debug_image_topic").value
        )
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.field_min_x_m = float(self.get_parameter("field_min_x_m").value)
        self.field_max_y_m = float(self.get_parameter("field_max_y_m").value)
        self.belt_top_z_m = float(self.get_parameter("belt_top_z_m").value)
        self.default_box_height_m = float(
            self.get_parameter("default_box_height_m").value
        )
        self.calibration_frames = max(
            3,
            int(self.get_parameter("calibration_frames").value),
        )
        self.processing_stride = max(
            1,
            int(self.get_parameter("processing_stride").value),
        )
        self.publish_debug_image = bool(
            self.get_parameter("publish_debug_image").value
        )

        config = DetectorConfig(
            field_length_m=float(self.get_parameter("field_length_m").value),
            field_width_m=float(self.get_parameter("field_width_m").value),
            background_threshold=int(
                self.get_parameter("background_threshold").value
            ),
            use_background_subtraction=bool(
                self.get_parameter("use_background_subtraction").value
            ),
            inner_erode_px=int(self.get_parameter("inner_erode_px").value),
            minimum_contour_area_px=float(
                self.get_parameter("minimum_contour_area_px").value
            ),
            morphology_open_px=int(
                self.get_parameter("morphology_open_px").value
            ),
            morphology_close_px=int(
                self.get_parameter("morphology_close_px").value
            ),
            dilate_iterations=int(
                self.get_parameter("dilate_iterations").value
            ),
            enable_touching_split=bool(
                self.get_parameter("enable_touching_split").value
            ),
            split_min_contour_area_px=float(
                self.get_parameter("split_min_contour_area_px").value
            ),
            split_peak_ratio=float(
                self.get_parameter("split_peak_ratio").value
            ),
            split_min_area_fraction=float(
                self.get_parameter("split_min_area_fraction").value
            ),
        )
        dataset_folder = str(self.get_parameter("dataset_folder").value)
        self.detector = StreamingBoxDetector(config, dataset_folder)
        self.tracker = NearestNeighbourTracker(
            maximum_distance_m=float(
                self.get_parameter("track_max_distance_m").value
            ),
            maximum_lateral_distance_m=float(
                self.get_parameter("track_max_lateral_distance_m").value
            ),
            maximum_backward_distance_m=float(
                self.get_parameter("track_max_backward_distance_m").value
            ),
            maximum_misses=int(self.get_parameter("track_max_misses").value),
        )

        self.bridge = CvBridge()
        self.empty_frames: list[np.ndarray] = []
        self.received_frames = 0
        self.last_error_text = ""

        self.boxes_publisher = self.create_publisher(
            BoxObservationArray,
            self.boxes_topic,
            10,
        )
        self.debug_publisher = self.create_publisher(
            Image,
            self.debug_image_topic,
            qos_profile_sensor_data,
        )
        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Waiting for {self.calibration_frames} empty frames on "
            f"{self.image_topic}"
        )

    def _on_image(self, message: Image) -> None:
        self.received_frames += 1
        if self.received_frames % self.processing_stride != 0:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self._log_error_once(f"Could not decode camera image: {error}")
            return

        if not self.detector.is_calibrated:
            self._collect_calibration_frame(frame)
            return

        try:
            detections, candidate_mask = self.detector.detect(frame)
        except (RuntimeError, ValueError, cv2.error) as error:
            self._log_error_once(f"Vision frame rejected: {error}")
            return

        stamp_s = self._stamp_to_seconds(message)
        world_positions = [self._to_world(item)[:2] for item in detections]
        track_ids = self.tracker.update(world_positions, stamp_s)
        self._publish_observations(message, detections, track_ids)
        self._publish_debug(message, frame, candidate_mask, detections, track_ids)
        self.last_error_text = ""

    def _collect_calibration_frame(self, frame: np.ndarray) -> None:
        self.empty_frames.append(frame.copy())
        if len(self.empty_frames) < self.calibration_frames:
            return

        median_frame = np.median(
            np.stack(self.empty_frames, axis=0),
            axis=0,
        ).astype(np.uint8)
        try:
            corners = self.detector.calibrate(median_frame)
        except (RuntimeError, ValueError, cv2.error) as error:
            self._log_error_once(f"Camera calibration failed: {error}")
            self.empty_frames.clear()
            return

        self.empty_frames.clear()
        self.get_logger().info(
            "Vision calibrated. Field corners px="
            + np.array2string(corners, precision=1)
        )

    def _publish_observations(
        self,
        image_message: Image,
        detections: list[Detection],
        track_ids: list[int],
    ) -> None:
        array = BoxObservationArray()
        array.header.stamp = image_message.header.stamp
        array.header.frame_id = self.frame_id

        for detection, track_id in zip(detections, track_ids):
            world_x, world_y, world_yaw = self._to_world(detection)
            observation = BoxObservation()
            observation.id = int(track_id)
            observation.model_name = (
                detection.box_type
                if detection.box_type != "unknown"
                else f"vision_box_{track_id}"
            )
            observation.center.x = world_x
            observation.center.y = world_y
            observation.center.z = (
                self.belt_top_z_m + self.default_box_height_m / 2.0
            )
            observation.length_m = float(detection.length_m)
            observation.width_m = float(detection.width_m)
            observation.height_m = self.default_box_height_m
            observation.yaw_rad = world_yaw
            observation.confidence = float(detection.confidence)
            array.boxes.append(observation)

        self.boxes_publisher.publish(array)

    def _publish_debug(
        self,
        image_message: Image,
        frame: np.ndarray,
        candidate_mask: np.ndarray,
        detections: list[Detection],
        track_ids: list[int],
    ) -> None:
        if not self.publish_debug_image:
            return
        debug = self.detector.draw_debug(frame, detections, track_ids)
        mask_bgr = cv2.cvtColor(candidate_mask, cv2.COLOR_GRAY2BGR)
        mask_bgr = cv2.resize(mask_bgr, (debug.shape[1], debug.shape[0]))
        combined = np.vstack((debug, mask_bgr))
        debug_message = self.bridge.cv2_to_imgmsg(combined, encoding="bgr8")
        debug_message.header = image_message.header
        self.debug_publisher.publish(debug_message)

    def _to_world(self, detection: Detection) -> tuple[float, float, float]:
        local_x, local_y = detection.center_local_m
        world_x = self.field_min_x_m + local_x
        world_y = self.field_max_y_m - local_y
        world_yaw = -detection.angle_image_rad
        world_yaw = ((world_yaw + math.pi / 2.0) % math.pi) - math.pi / 2.0
        return world_x, world_y, world_yaw

    @staticmethod
    def _stamp_to_seconds(message: Image) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9

    def _log_error_once(self, text: str) -> None:
        if text != self.last_error_text:
            self.get_logger().error(text)
            self.last_error_text = text


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VisionStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
