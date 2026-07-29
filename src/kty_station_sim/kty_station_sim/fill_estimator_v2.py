"""Robust depth-volume estimator for the runtime-v7 KTY cycle."""

from __future__ import annotations

import json
import math
import time

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class KtyFillEstimatorV2(Node):
    """Estimate occupied volume while excluding the four KTY walls.

    The stage-4 estimator integrated almost the full projected rectangle.  At
    the container rim this included the 400 mm walls and could report a nearly
    full KTY before products occupied the floor.  Runtime v7 evaluates a
    calibrated inner core, rejects wall-height samples and extrapolates the
    core volume to the full 600 x 400 mm floor area.
    """

    def __init__(self) -> None:
        super().__init__("kty_fill_estimator_v2")
        defaults = {
            "depth_topic": "/kty/vision/depth_image",
            "camera_info_topic": "/kty/vision/camera_info",
            "output_topic": "/kty/fill/state",
            "camera_to_bottom_m": 1.25,
            "internal_length_m": 0.60,
            "internal_width_m": 0.40,
            "internal_height_m": 0.40,
            "wall_exclusion_margin_m": 0.040,
            "minimum_height_m": 0.006,
            "maximum_product_height_m": 0.360,
            "processing_hz": 4.0,
            "minimum_valid_fraction": 0.68,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.depth_topic = str(value("depth_topic"))
        self.info_topic = str(value("camera_info_topic"))
        self.output_topic = str(value("output_topic"))
        self.camera_to_bottom = float(value("camera_to_bottom_m"))
        self.length = float(value("internal_length_m"))
        self.width = float(value("internal_width_m"))
        self.height = float(value("internal_height_m"))
        self.margin = float(value("wall_exclusion_margin_m"))
        self.minimum_height = float(value("minimum_height_m"))
        self.maximum_height = float(value("maximum_product_height_m"))
        self.processing_hz = float(value("processing_hz"))
        self.minimum_valid_fraction = float(value("minimum_valid_fraction"))
        if self.length <= 2.0 * self.margin or self.width <= 2.0 * self.margin:
            raise ValueError("wall_exclusion_margin_m leaves no measurable KTY core")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(String, self.output_topic, qos)
        self.bridge = CvBridge()
        self.camera_info: CameraInfo | None = None
        self.last_processed = 0.0
        self.sequence = 0
        self.create_subscription(
            CameraInfo,
            self.info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _intrinsics(self, cols: int, rows: int) -> tuple[float, float, float, float]:
        if self.camera_info is not None and self.camera_info.k[0] > 0.0:
            return (
                float(self.camera_info.k[0]),
                float(self.camera_info.k[4]),
                float(self.camera_info.k[2]),
                float(self.camera_info.k[5]),
            )
        fx = cols / (2.0 * math.tan(1.05 / 2.0))
        return fx, fx, cols / 2.0, rows / 2.0

    def _on_depth(self, message: Image) -> None:
        now = time.monotonic()
        if now - self.last_processed < 1.0 / max(self.processing_hz, 0.1):
            return
        self.last_processed = now
        try:
            raw = np.asarray(
                self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            )
            scale = 0.001 if raw.dtype == np.uint16 else 1.0
            depth = raw.astype(np.float32) * scale
            if depth.ndim != 2:
                return
            self._process(message, depth)
        except Exception as error:  # runtime sensor guard
            self.get_logger().error(f"Fill estimator frame failed: {error!r}")

    def _process(self, message: Image, depth: np.ndarray) -> None:
        rows, cols = depth.shape
        fx, fy, cx, cy = self._intrinsics(cols, rows)
        core_length = self.length - 2.0 * self.margin
        core_width = self.width - 2.0 * self.margin
        half_u = core_length * fx / (2.0 * self.camera_to_bottom)
        half_v = core_width * fy / (2.0 * self.camera_to_bottom)
        u0, u1 = max(0, int(cx - half_u)), min(cols, int(cx + half_u))
        v0, v1 = max(0, int(cy - half_v)), min(rows, int(cy + half_v))
        roi = np.zeros(depth.shape, np.uint8)
        roi[v0:v1, u0:u1] = 255
        roi = cv2.erode(roi, np.ones((5, 5), np.uint8), iterations=1)
        roi_pixels = roi > 0

        finite = np.isfinite(depth) & (depth > 0.20) & (depth < 3.0)
        valid = finite & roi_pixels
        valid_fraction = float(
            np.count_nonzero(valid) / max(1, np.count_nonzero(roi_pixels))
        )

        repaired = depth.astype(np.float32, copy=True)
        fallback = (
            float(np.nanmedian(repaired[valid])) if np.any(valid) else self.camera_to_bottom
        )
        repaired[~finite] = fallback
        repaired = cv2.medianBlur(repaired, 5)

        height_map = np.clip(self.camera_to_bottom - repaired, 0.0, self.height)
        # Wall-top samples cluster near 0.40 m.  They are never product volume
        # in this test because the accepted product-height criterion is 0.28 m.
        product = (
            valid
            & (height_map >= self.minimum_height)
            & (height_map <= self.maximum_height)
        )
        clean = np.zeros_like(height_map)
        clean[product] = height_map[product]
        clean = cv2.morphologyEx(clean, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

        # Perspective-correct projected pixel area at the observed depth.
        pixel_area = np.square(repaired) / max(fx * fy, 1.0e-9)
        core_volume = float(np.sum(clean[roi_pixels] * pixel_area[roi_pixels]))
        area_scale = (self.length * self.width) / (core_length * core_width)
        estimated_volume = core_volume * area_scale
        capacity = self.length * self.width * self.height
        fill_ratio = float(np.clip(estimated_volume / capacity, 0.0, 1.0))
        positive = clean[(clean > 0.0) & roi_pixels]
        maximum_height = (
            float(np.percentile(positive, 99.5)) if positive.size else 0.0
        )
        occupied_ratio = float(
            np.count_nonzero(positive) / max(1, np.count_nonzero(roi_pixels))
        )

        self.sequence += 1
        payload = {
            "schema": "kty_fill_state/v2",
            "sequence": self.sequence,
            "camera_ok": valid_fraction >= self.minimum_valid_fraction,
            "valid_depth_fraction": valid_fraction,
            "estimated_volume_m3": estimated_volume,
            "measured_core_volume_m3": core_volume,
            "capacity_m3": capacity,
            "fill_ratio": fill_ratio,
            "maximum_height_m": maximum_height,
            "occupied_floor_ratio": occupied_ratio,
            "wall_exclusion_margin_m": self.margin,
        }
        output = String()
        output.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyFillEstimatorV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
