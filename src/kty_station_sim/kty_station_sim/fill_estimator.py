"""Depth-only KTY fill-volume estimator.

The estimator integrates the measured height map over the calibrated inner floor
area. It is independent from instance segmentation so gate control remains
available even when touching cartons are not separated yet.
"""

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


class KtyFillEstimator(Node):
    def __init__(self) -> None:
        super().__init__("kty_fill_estimator")
        defaults = {
            "depth_topic": "/kty/vision/depth_image",
            "camera_info_topic": "/kty/vision/camera_info",
            "output_topic": "/kty/fill/state",
            "camera_to_bottom_m": 1.25,
            "internal_length_m": 0.60,
            "internal_width_m": 0.40,
            "internal_height_m": 0.40,
            "minimum_height_m": 0.005,
            "processing_hz": 10.0,
            "roi_erode_px": 8,
            "minimum_valid_fraction": 0.70,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.camera_to_bottom = float(
            self.get_parameter("camera_to_bottom_m").value
        )
        self.internal_length = float(
            self.get_parameter("internal_length_m").value
        )
        self.internal_width = float(
            self.get_parameter("internal_width_m").value
        )
        self.internal_height = float(
            self.get_parameter("internal_height_m").value
        )
        self.minimum_height = float(
            self.get_parameter("minimum_height_m").value
        )
        self.processing_hz = float(self.get_parameter("processing_hz").value)
        self.roi_erode_px = int(self.get_parameter("roi_erode_px").value)
        self.minimum_valid_fraction = float(
            self.get_parameter("minimum_valid_fraction").value
        )
        if self.processing_hz <= 0.0:
            raise ValueError("processing_hz must be positive")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(String, self.output_topic, qos)
        self.bridge = CvBridge()
        self.camera_info: CameraInfo | None = None
        self.sequence = 0
        self.last_processed = 0.0

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
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

    def _intrinsics(
        self,
        width: int,
        height: int,
    ) -> tuple[float, float, float, float]:
        if self.camera_info is not None and self.camera_info.k[0] > 0.0:
            return (
                float(self.camera_info.k[0]),
                float(self.camera_info.k[4]),
                float(self.camera_info.k[2]),
                float(self.camera_info.k[5]),
            )
        fx = width / (2.0 * math.tan(1.05 / 2.0))
        return fx, fx, width / 2.0, height / 2.0

    def _on_depth(self, message: Image) -> None:
        now = time.monotonic()
        if now - self.last_processed < 1.0 / self.processing_hz:
            return
        self.last_processed = now

        try:
            depth = np.asarray(
                self.bridge.imgmsg_to_cv2(
                    message,
                    desired_encoding="passthrough",
                )
            )
        except Exception as error:  # pragma: no cover
            self.get_logger().warning(f"Depth conversion failed: {error}")
            return

        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)
        if depth.ndim != 2:
            return

        height_px, width_px = depth.shape
        fx, fy, cx, cy = self._intrinsics(width_px, height_px)
        half_u = self.internal_length * fx / (2.0 * self.camera_to_bottom)
        half_v = self.internal_width * fy / (2.0 * self.camera_to_bottom)
        u0 = max(0, int(math.floor(cx - half_u)))
        u1 = min(width_px, int(math.ceil(cx + half_u)))
        v0 = max(0, int(math.floor(cy - half_v)))
        v1 = min(height_px, int(math.ceil(cy + half_v)))

        roi = np.zeros(depth.shape, dtype=np.uint8)
        roi[v0:v1, u0:u1] = 255
        if self.roi_erode_px > 0:
            kernel_size = 2 * self.roi_erode_px + 1
            roi = cv2.erode(
                roi,
                np.ones((kernel_size, kernel_size), np.uint8),
                iterations=1,
            )

        roi_pixels = roi > 0
        finite = np.isfinite(depth) & (depth > 0.15) & (depth < 3.0)
        valid = finite & roi_pixels
        valid_fraction = float(
            np.count_nonzero(valid) / max(1, np.count_nonzero(roi_pixels))
        )

        height_map = np.zeros(depth.shape, dtype=np.float32)
        height_map[valid] = np.clip(
            self.camera_to_bottom - depth[valid],
            0.0,
            self.internal_height,
        )
        height_map[height_map < self.minimum_height] = 0.0

        pixel_area = self.camera_to_bottom**2 / max(fx * fy, 1.0e-9)
        estimated_volume = float(np.sum(height_map[roi_pixels]) * pixel_area)
        capacity = (
            self.internal_length
            * self.internal_width
            * self.internal_height
        )
        fill_ratio = min(1.0, max(0.0, estimated_volume / capacity))

        positive = height_map[(height_map > 0.0) & roi_pixels]
        maximum_height = (
            float(np.percentile(positive, 99.0))
            if positive.size
            else 0.0
        )
        occupied_area_ratio = float(
            np.count_nonzero(positive)
            / max(1, np.count_nonzero(roi_pixels))
        )

        self.sequence += 1
        payload = {
            "schema": "kty_fill_state/v1",
            "sequence": self.sequence,
            "camera_ok": valid_fraction >= self.minimum_valid_fraction,
            "valid_depth_fraction": valid_fraction,
            "estimated_volume_m3": estimated_volume,
            "capacity_m3": capacity,
            "fill_ratio": fill_ratio,
            "maximum_height_m": maximum_height,
            "occupied_floor_ratio": occupied_area_ratio,
        }
        output = String()
        output.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyFillEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
