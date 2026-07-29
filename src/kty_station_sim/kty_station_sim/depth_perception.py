"""RGB-D contour extraction and persistent product IDs inside a KTY."""

from __future__ import annotations

from dataclasses import dataclass
import math

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from singulator_interfaces.msg import KtyProductContour, KtyProductContourArray


@dataclass(slots=True)
class Detection:
    mask: np.ndarray
    contour_px: np.ndarray
    centroid_x: float
    centroid_y: float
    top_height: float
    visible_area: float
    confidence: float
    polygon_xy: list[tuple[float, float]]
    extent: tuple[float, float, float, float]
    track_id: int = 0


@dataclass(slots=True)
class Track:
    track_id: int
    x: float
    y: float
    height: float
    misses: int = 0


class GreedyTracker:
    def __init__(self, maximum_distance: float, maximum_misses: int) -> None:
        self.maximum_distance = maximum_distance
        self.maximum_misses = maximum_misses
        self.next_id = 1
        self.tracks: dict[int, Track] = {}

    def update(self, detections: list[Detection]) -> None:
        candidates: list[tuple[float, int, int]] = []
        for track_id, track in self.tracks.items():
            for index, detection in enumerate(detections):
                distance = math.hypot(
                    detection.centroid_x - track.x,
                    detection.centroid_y - track.y,
                )
                height_delta = abs(detection.top_height - track.height)
                if distance <= self.maximum_distance and height_delta <= 0.18:
                    candidates.append((distance + 0.25 * height_delta, track_id, index))

        assigned_tracks: set[int] = set()
        assigned_detections: set[int] = set()
        for _, track_id, detection_index in sorted(candidates):
            if track_id in assigned_tracks or detection_index in assigned_detections:
                continue
            detections[detection_index].track_id = track_id
            assigned_tracks.add(track_id)
            assigned_detections.add(detection_index)

        for index, detection in enumerate(detections):
            if index in assigned_detections:
                track = self.tracks[detection.track_id]
                track.x = detection.centroid_x
                track.y = detection.centroid_y
                track.height = detection.top_height
                track.misses = 0
                continue
            track_id = self.next_id
            self.next_id += 1
            detection.track_id = track_id
            self.tracks[track_id] = Track(
                track_id=track_id,
                x=detection.centroid_x,
                y=detection.centroid_y,
                height=detection.top_height,
            )
            assigned_tracks.add(track_id)

        for track_id, track in list(self.tracks.items()):
            if track_id in assigned_tracks:
                continue
            track.misses += 1
            if track.misses > self.maximum_misses:
                del self.tracks[track_id]


class KtyDepthPerception(Node):
    def __init__(self) -> None:
        super().__init__("kty_depth_perception")
        defaults = {
            "rgb_topic": "/kty/camera/image",
            "depth_topic": "/kty/camera/depth_image",
            "camera_info_topic": "/kty/camera/camera_info",
            "output_topic": "/kty/perception/contours",
            "debug_topic": "/kty/perception/debug_image",
            "camera_to_kty_bottom_m": 1.25,
            "internal_length_m": 0.60,
            "internal_width_m": 0.40,
            "internal_height_m": 0.40,
            "minimum_product_height_m": 0.008,
            "minimum_contour_area_px": 40.0,
            "split_peak_ratio": 0.42,
            "track_max_distance_m": 0.12,
            "track_max_misses": 6,
            "side_clearance_m": 0.025,
            "simulated_depth_noise_std_m": 0.002,
            "simulated_dropout_probability": 0.01,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(
            self.get_parameter("camera_info_topic").value
        )
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.debug_topic = str(self.get_parameter("debug_topic").value)
        self.camera_to_bottom = float(
            self.get_parameter("camera_to_kty_bottom_m").value
        )
        self.internal_length = float(
            self.get_parameter("internal_length_m").value
        )
        self.internal_width = float(self.get_parameter("internal_width_m").value)
        self.internal_height = float(
            self.get_parameter("internal_height_m").value
        )
        self.minimum_height = float(
            self.get_parameter("minimum_product_height_m").value
        )
        self.minimum_area = float(
            self.get_parameter("minimum_contour_area_px").value
        )
        self.split_peak_ratio = float(
            self.get_parameter("split_peak_ratio").value
        )
        self.side_clearance = float(
            self.get_parameter("side_clearance_m").value
        )
        self.noise_std = float(
            self.get_parameter("simulated_depth_noise_std_m").value
        )
        self.dropout_probability = float(
            self.get_parameter("simulated_dropout_probability").value
        )

        self.bridge = CvBridge()
        self.latest_rgb: np.ndarray | None = None
        self.camera_info: CameraInfo | None = None
        self.frame_sequence = 0
        self.rng = np.random.default_rng(42)
        self.tracker = GreedyTracker(
            maximum_distance=float(
                self.get_parameter("track_max_distance_m").value
            ),
            maximum_misses=int(self.get_parameter("track_max_misses").value),
        )

        self.output_pub = self.create_publisher(
            KtyProductContourArray,
            self.output_topic,
            10,
        )
        self.debug_pub = self.create_publisher(
            Image,
            self.debug_topic,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.rgb_topic,
            self._on_rgb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._on_camera_info,
            qos_profile_sensor_data,
        )

    def _on_rgb(self, message: Image) -> None:
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:  # pragma: no cover - runtime bridge guard
            self.get_logger().error(f"RGB conversion failed: {error}")
            return
        # Mild blur and image noise emulate a non-ideal industrial camera.
        image = cv2.GaussianBlur(image, (3, 3), 0.5)
        noise = self.rng.normal(0.0, 2.0, image.shape).astype(np.int16)
        self.latest_rgb = np.clip(image.astype(np.int16) + noise, 0, 255).astype(
            np.uint8
        )

    def _on_camera_info(self, message: CameraInfo) -> None:
        self.camera_info = message

    def _decode_depth(self, message: Image) -> np.ndarray:
        depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        depth = np.asarray(depth)
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)
        if self.noise_std > 0.0:
            depth += self.rng.normal(0.0, self.noise_std, depth.shape).astype(
                np.float32
            )
        if self.dropout_probability > 0.0:
            drop = self.rng.random(depth.shape) < self.dropout_probability
            depth[drop] = np.nan
        return depth

    def _intrinsics(self, width: int, height: int) -> tuple[float, float, float, float]:
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
        if self.latest_rgb is None:
            return
        try:
            depth = self._decode_depth(message)
        except Exception as error:  # pragma: no cover - runtime bridge guard
            self.get_logger().error(f"Depth conversion failed: {error}")
            return
        if depth.ndim != 2:
            self.get_logger().error(f"Unexpected depth shape: {depth.shape}")
            return

        rgb = self.latest_rgb
        if rgb.shape[:2] != depth.shape:
            rgb = cv2.resize(rgb, (depth.shape[1], depth.shape[0]))

        detections, roi_mask, height_map, valid_fraction = self._detect(rgb, depth)
        self.tracker.update(detections)
        clearances = self._compute_side_clearances(detections)

        self.frame_sequence += 1
        output = KtyProductContourArray()
        output.header = message.header
        output.header.frame_id = "kty_inner"
        output.frame_sequence = self.frame_sequence
        output.camera_ok = valid_fraction >= 0.70
        output.valid_depth_fraction = float(valid_fraction)
        output.maximum_height_m = (
            max((item.top_height for item in detections), default=0.0)
        )
        output.top_fill_ratio = min(
            1.0,
            sum(item.visible_area for item in detections)
            / (self.internal_length * self.internal_width),
        )

        for detection, clearance in zip(detections, clearances):
            contour = KtyProductContour()
            contour.track_id = detection.track_id
            contour.centroid.x = detection.centroid_x
            contour.centroid.y = detection.centroid_y
            contour.centroid.z = detection.top_height
            contour.top_height_m = detection.top_height
            contour.visible_area_m2 = detection.visible_area
            contour.confidence = detection.confidence
            for x, y in detection.polygon_xy:
                point = Point32()
                point.x = float(x)
                point.y = float(y)
                point.z = float(detection.top_height)
                contour.polygon.points.append(point)
            (
                contour.clearance_neg_x_m,
                contour.clearance_pos_x_m,
                contour.clearance_neg_y_m,
                contour.clearance_pos_y_m,
            ) = clearance
            contour.side_neg_x_accessible = clearance[0] >= self.side_clearance
            contour.side_pos_x_accessible = clearance[1] >= self.side_clearance
            contour.side_neg_y_accessible = clearance[2] >= self.side_clearance
            contour.side_pos_y_accessible = clearance[3] >= self.side_clearance
            output.products.append(contour)

        self.output_pub.publish(output)
        self._publish_debug(message, rgb, roi_mask, height_map, detections)

    def _detect(
        self,
        rgb: np.ndarray,
        depth: np.ndarray,
    ) -> tuple[list[Detection], np.ndarray, np.ndarray, float]:
        height, width = depth.shape
        fx, fy, cx, cy = self._intrinsics(width, height)

        half_u = self.internal_length * fx / (2.0 * self.camera_to_bottom)
        half_v = self.internal_width * fy / (2.0 * self.camera_to_bottom)
        u0 = max(0, int(math.floor(cx - half_u)))
        u1 = min(width, int(math.ceil(cx + half_u)))
        v0 = max(0, int(math.floor(cy - half_v)))
        v1 = min(height, int(math.ceil(cy + half_v)))

        roi_mask = np.zeros_like(depth, dtype=np.uint8)
        roi_mask[v0:v1, u0:u1] = 255
        roi_mask = cv2.erode(roi_mask, np.ones((5, 5), np.uint8), iterations=1)

        finite = np.isfinite(depth) & (depth > 0.15) & (depth < 3.0)
        roi_pixels = roi_mask > 0
        valid_fraction = float(
            np.count_nonzero(finite & roi_pixels)
            / max(1, np.count_nonzero(roi_pixels))
        )

        height_map = np.zeros_like(depth, dtype=np.float32)
        height_map[finite] = self.camera_to_bottom - depth[finite]
        product_mask = (
            finite
            & roi_pixels
            & (height_map >= self.minimum_height)
            & (height_map <= self.internal_height + 0.08)
        ).astype(np.uint8) * 255

        product_mask = cv2.morphologyEx(
            product_mask,
            cv2.MORPH_OPEN,
            np.ones((3, 3), np.uint8),
        )
        product_mask = cv2.morphologyEx(
            product_mask,
            cv2.MORPH_CLOSE,
            np.ones((5, 5), np.uint8),
        )

        region_masks = self._split_regions(product_mask, rgb)
        detections: list[Detection] = []
        for region in region_masks:
            area_px = int(np.count_nonzero(region))
            if area_px < self.minimum_area:
                continue
            contours, _ = cv2.findContours(
                region,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE,
            )
            if not contours:
                continue
            contour_px = max(contours, key=cv2.contourArea)
            if cv2.contourArea(contour_px) < self.minimum_area:
                continue
            moments = cv2.moments(contour_px)
            if abs(moments["m00"]) < 1.0e-6:
                continue
            centroid_u = moments["m10"] / moments["m00"]
            centroid_v = moments["m01"] / moments["m00"]
            region_depth = depth[region > 0]
            region_depth = region_depth[np.isfinite(region_depth)]
            if region_depth.size == 0:
                continue
            median_depth = float(np.median(region_depth))
            centroid_x = (centroid_u - cx) * median_depth / fx
            centroid_y = -(centroid_v - cy) * median_depth / fy
            top_height = float(np.percentile(height_map[region > 0], 95.0))
            visible_area = float(area_px * median_depth**2 / (fx * fy))

            epsilon = 0.015 * cv2.arcLength(contour_px, True)
            polygon_px = cv2.approxPolyDP(contour_px, epsilon, True)
            polygon_xy: list[tuple[float, float]] = []
            for point in polygon_px.reshape(-1, 2):
                u, v = float(point[0]), float(point[1])
                polygon_xy.append(
                    (
                        (u - cx) * median_depth / fx,
                        -(v - cy) * median_depth / fy,
                    )
                )
            if len(polygon_xy) < 3:
                continue

            xs = [point[0] for point in polygon_xy]
            ys = [point[1] for point in polygon_xy]
            compactness = min(
                1.0,
                4.0 * math.pi * cv2.contourArea(contour_px)
                / max(cv2.arcLength(contour_px, True) ** 2, 1.0),
            )
            confidence = min(1.0, 0.55 + 0.45 * compactness)
            detections.append(
                Detection(
                    mask=region,
                    contour_px=contour_px,
                    centroid_x=centroid_x,
                    centroid_y=centroid_y,
                    top_height=top_height,
                    visible_area=visible_area,
                    confidence=confidence,
                    polygon_xy=polygon_xy,
                    extent=(min(xs), max(xs), min(ys), max(ys)),
                )
            )

        detections.sort(key=lambda item: (item.centroid_x, item.centroid_y))
        return detections, roi_mask, height_map, valid_fraction

    def _split_regions(self, mask: np.ndarray, rgb: np.ndarray) -> list[np.ndarray]:
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        regions: list[np.ndarray] = []
        for label in range(1, count):
            if stats[label, cv2.CC_STAT_AREA] < self.minimum_area:
                continue
            component = (labels == label).astype(np.uint8) * 255
            distance = cv2.distanceTransform(component, cv2.DIST_L2, 5)
            maximum = float(distance.max())
            if maximum <= 1.0:
                regions.append(component)
                continue
            peaks = (distance >= self.split_peak_ratio * maximum).astype(np.uint8)
            peaks = cv2.morphologyEx(
                peaks,
                cv2.MORPH_OPEN,
                np.ones((3, 3), np.uint8),
            )
            seed_count, seed_labels = cv2.connectedComponents(peaks)
            if seed_count <= 2:
                regions.append(component)
                continue

            markers = np.zeros(mask.shape, dtype=np.int32)
            markers[component == 0] = 1
            for seed in range(1, seed_count):
                markers[seed_labels == seed] = seed + 1
            guide = rgb.copy()
            depth_edges = cv2.normalize(distance, None, 0, 255, cv2.NORM_MINMAX)
            depth_edges = cv2.cvtColor(depth_edges.astype(np.uint8), cv2.COLOR_GRAY2BGR)
            guide = cv2.addWeighted(guide, 0.65, depth_edges, 0.35, 0.0)
            cv2.watershed(guide, markers)

            split_any = False
            for marker in range(2, seed_count + 1):
                region = ((markers == marker) & (component > 0)).astype(np.uint8) * 255
                if np.count_nonzero(region) >= self.minimum_area:
                    regions.append(region)
                    split_any = True
            if not split_any:
                regions.append(component)
        return regions

    def _compute_side_clearances(
        self,
        detections: list[Detection],
    ) -> list[tuple[float, float, float, float]]:
        half_x = self.internal_length / 2.0
        half_y = self.internal_width / 2.0
        results: list[tuple[float, float, float, float]] = []
        for index, detection in enumerate(detections):
            xmin, xmax, ymin, ymax = detection.extent
            neg_x = max(0.0, xmin + half_x)
            pos_x = max(0.0, half_x - xmax)
            neg_y = max(0.0, ymin + half_y)
            pos_y = max(0.0, half_y - ymax)
            for other_index, other in enumerate(detections):
                if index == other_index:
                    continue
                oxmin, oxmax, oymin, oymax = other.extent
                y_overlap = min(ymax, oymax) - max(ymin, oymin) > 0.0
                x_overlap = min(xmax, oxmax) - max(xmin, oxmin) > 0.0
                if y_overlap:
                    if oxmax <= xmin:
                        neg_x = min(neg_x, max(0.0, xmin - oxmax))
                    elif oxmin >= xmax:
                        pos_x = min(pos_x, max(0.0, oxmin - xmax))
                if x_overlap:
                    if oymax <= ymin:
                        neg_y = min(neg_y, max(0.0, ymin - oymax))
                    elif oymin >= ymax:
                        pos_y = min(pos_y, max(0.0, oymin - ymax))
            results.append((neg_x, pos_x, neg_y, pos_y))
        return results

    def _publish_debug(
        self,
        source: Image,
        rgb: np.ndarray,
        roi_mask: np.ndarray,
        height_map: np.ndarray,
        detections: list[Detection],
    ) -> None:
        debug = rgb.copy()
        roi_contours, _ = cv2.findContours(
            roi_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        cv2.drawContours(debug, roi_contours, -1, (255, 255, 255), 1)
        for detection in detections:
            cv2.drawContours(debug, [detection.contour_px], -1, (0, 255, 0), 2)
            moments = cv2.moments(detection.contour_px)
            if abs(moments["m00"]) > 1.0e-6:
                u = int(moments["m10"] / moments["m00"])
                v = int(moments["m01"] / moments["m00"])
                cv2.putText(
                    debug,
                    f"ID {detection.track_id} h={detection.top_height:.3f}",
                    (u - 35, v),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (0, 0, 0),
                    3,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    debug,
                    f"ID {detection.track_id} h={detection.top_height:.3f}",
                    (u - 35, v),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (255, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
        # Report the same filtered detections as KtyProductContourArray.  The
        # raw height map also contains station structure outside the KTY ROI;
        # displaying its global maximum produced misleading values (for
        # example 0.567 m for an empty 0.400 m-high KTY).
        maximum = max((item.top_height for item in detections), default=0.0)
        cv2.putText(
            debug,
            f"max height={maximum:.3f} m",
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (20, 20, 20),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            debug,
            f"max height={maximum:.3f} m",
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.60,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        message.header = source.header
        self.debug_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyDepthPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
