"""ROS wrapper for classical RGB-D carton instance segmentation."""

from __future__ import annotations

import math

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Point32
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from singulator_interfaces.msg import KtyGraspCandidate, KtyProductContour, KtyProductContourArray
from .classical_3d_core import Classical3DSegmenter, Tracker


class KtyClassical3DPerception(Node):
    def __init__(self):
        super().__init__("kty_classical_3d_perception")
        defaults = {
            "rgb_topic": "/kty/vision/image", "depth_topic": "/kty/vision/depth_image",
            "camera_info_topic": "/kty/vision/camera_info", "output_topic": "/kty/perception/contours",
            "debug_topic": "/kty/perception/debug_image", "camera_to_kty_bottom_m": 1.25,
            "internal_length_m": 0.60, "internal_width_m": 0.40, "internal_height_m": 0.40,
            "minimum_product_height_m": 0.008, "minimum_contour_area_px": 55.0,
            "depth_edge_threshold_m": 0.010, "normal_edge_threshold": 0.12,
            "seed_min_distance_px": 18, "seed_height_prominence_m": 0.012,
            "track_max_distance_m": 0.14, "track_max_height_delta_m": 0.18,
            "track_max_misses": 12, "side_clearance_m": 0.025,
            "top_normal_min_z": 0.82, "top_occlusion_max": 0.42,
            "simulated_depth_noise_std_m": 0.001, "simulated_dropout_probability": 0.002,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = lambda name: self.get_parameter(name).value
        self.rgb_topic, self.depth_topic = str(p("rgb_topic")), str(p("depth_topic"))
        self.info_topic, self.output_topic, self.debug_topic = str(p("camera_info_topic")), str(p("output_topic")), str(p("debug_topic"))
        self.length, self.width = float(p("internal_length_m")), float(p("internal_width_m"))
        self.side_clearance = float(p("side_clearance_m"))
        self.noise_std, self.dropout = float(p("simulated_depth_noise_std_m")), float(p("simulated_dropout_probability"))
        self.bridge, self.latest_rgb, self.camera_info = CvBridge(), None, None
        self.sequence, self.rng = 0, np.random.default_rng(84)
        self.segmenter = Classical3DSegmenter(
            camera_to_bottom=float(p("camera_to_kty_bottom_m")), internal_length=self.length,
            internal_width=self.width, internal_height=float(p("internal_height_m")),
            minimum_height=float(p("minimum_product_height_m")), minimum_area_px=float(p("minimum_contour_area_px")),
            depth_edge_threshold=float(p("depth_edge_threshold_m")), normal_edge_threshold=float(p("normal_edge_threshold")),
            seed_distance_px=int(p("seed_min_distance_px")), seed_height_prominence=float(p("seed_height_prominence_m")),
            top_normal_min_z=float(p("top_normal_min_z")), top_occlusion_max=float(p("top_occlusion_max")),
        )
        self.tracker = Tracker(float(p("track_max_distance_m")), float(p("track_max_height_delta_m")), int(p("track_max_misses")))
        self.publisher = self.create_publisher(KtyProductContourArray, self.output_topic, 10)
        self.debug_publisher = self.create_publisher(Image, self.debug_topic, qos_profile_sensor_data)
        self.create_subscription(Image, self.rgb_topic, self._on_rgb, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic, self._on_info, qos_profile_sensor_data)

    def _on_rgb(self, message):
        try:
            self.latest_rgb = cv2.GaussianBlur(self.bridge.imgmsg_to_cv2(message, "bgr8"), (3, 3), 0.45)
        except Exception as error:
            self.get_logger().warning(f"RGB conversion failed: {error}")

    def _on_info(self, message):
        self.camera_info = message

    def _intrinsics(self, cols, rows):
        if self.camera_info is not None and self.camera_info.k[0] > 0:
            return float(self.camera_info.k[0]), float(self.camera_info.k[4]), float(self.camera_info.k[2]), float(self.camera_info.k[5])
        fx = cols / (2.0 * math.tan(1.05 / 2.0))
        return fx, fx, cols / 2.0, rows / 2.0

    def _on_depth(self, message):
        if self.latest_rgb is None:
            return
        try:
            depth = np.asarray(self.bridge.imgmsg_to_cv2(message, "passthrough"))
        except Exception as error:
            self.get_logger().warning(f"Depth conversion failed: {error}")
            return
        depth = depth.astype(np.float32) * (0.001 if depth.dtype == np.uint16 else 1.0)
        if self.noise_std > 0:
            depth += self.rng.normal(0, self.noise_std, depth.shape).astype(np.float32)
        if self.dropout > 0:
            depth[self.rng.random(depth.shape) < self.dropout] = np.nan
        rgb = cv2.resize(self.latest_rgb, (depth.shape[1], depth.shape[0])) if self.latest_rgb.shape[:2] != depth.shape else self.latest_rgb
        intrinsics = self._intrinsics(depth.shape[1], depth.shape[0])
        visible, roi, height, boundary, valid_fraction = self.segmenter.segment(rgb, depth, intrinsics)
        occluded = self.tracker.update(visible)
        self._clearances_and_grasps(visible)
        tracks = sorted([*visible, *occluded], key=lambda item: item.track_id)
        self.sequence += 1
        output = KtyProductContourArray()
        output.header, output.header.frame_id = message.header, "kty_inner"
        output.frame_sequence, output.camera_ok, output.valid_depth_fraction = self.sequence, valid_fraction >= 0.70, float(valid_fraction)
        output.maximum_height_m = max((item.centroid[2] for item in visible), default=0.0)
        output.top_fill_ratio = min(1.0, sum(item.area_m2 for item in visible) / (self.length * self.width))
        output.products = [self._message(item) for item in tracks]
        self.publisher.publish(output)
        self._debug(message, rgb, roi, height, boundary, visible, occluded, intrinsics)

    def _clearances_and_grasps(self, items):
        half_x, half_y = self.length / 2.0, self.width / 2.0
        for index, item in enumerate(items):
            xmin, xmax, ymin, ymax = item.extent
            gaps = [xmin + half_x, half_x - xmax, ymin + half_y, half_y - ymax]
            for other_index, other in enumerate(items):
                if index == other_index:
                    continue
                oxmin, oxmax, oymin, oymax = other.extent
                if min(ymax, oymax) > max(ymin, oymin):
                    if oxmax <= xmin:
                        gaps[0] = min(gaps[0], xmin - oxmax)
                    elif oxmin >= xmax:
                        gaps[1] = min(gaps[1], oxmin - xmax)
                if min(xmax, oxmax) > max(xmin, oxmin):
                    if oymax <= ymin:
                        gaps[2] = min(gaps[2], ymin - oymax)
                    elif oymin >= ymax:
                        gaps[3] = min(gaps[3], oymin - ymax)
            item.clearances = tuple(max(0.0, value) for value in gaps)
            if item.state == "VISIBLE" and item.top_accessible:
                minimum = min(item.clearances)
                score = float(np.clip(item.confidence * (1.0 - item.occlusion) * min(1.0, 0.6 + minimum / 0.08), 0, 1))
                x, y, z = item.centroid
                item.grasps = [{"position": (x, y, z + 0.004), "approach": (0.0, 0.0, -1.0), "score": score, "clearance": 0.025, "strategy": "TOP_CENTER"}]
                for gap, approach, strategy in zip(item.clearances, ((-1,0,0),(1,0,0),(0,-1,0),(0,1,0)), ("SIDE_NEG_X","SIDE_POS_X","SIDE_NEG_Y","SIDE_POS_Y")):
                    if gap >= self.side_clearance:
                        item.grasps.append({"position": (x, y, max(0.02, z * 0.65)), "approach": approach, "score": score * min(0.88, 0.55 + gap / 0.12), "clearance": self.side_clearance, "strategy": strategy})

    def _message(self, item):
        msg = KtyProductContour()
        msg.track_id, msg.tracking_state = item.track_id, item.state
        msg.centroid = Point32(x=float(item.centroid[0]), y=float(item.centroid[1]), z=float(item.centroid[2]))
        msg.top_height_m, msg.visible_area_m2, msg.confidence = float(item.centroid[2]), float(item.area_m2), float(item.confidence)
        msg.yaw_rad, msg.occlusion_score, msg.top_accessible = float(item.yaw), float(item.occlusion), bool(item.top_accessible)
        msg.surface_normal.x, msg.surface_normal.y, msg.surface_normal.z = item.normal
        msg.estimated_size.x, msg.estimated_size.y, msg.estimated_size.z = item.size_xyz
        msg.polygon.points = [Point32(x=float(x), y=float(y), z=float(item.centroid[2])) for x, y in item.polygon_xy]
        msg.oriented_rectangle.points = [Point32(x=float(x), y=float(y), z=float(item.centroid[2])) for x, y in item.rectangle_xy]
        msg.clearance_neg_x_m, msg.clearance_pos_x_m, msg.clearance_neg_y_m, msg.clearance_pos_y_m = item.clearances
        msg.side_neg_x_accessible, msg.side_pos_x_accessible = item.clearances[0] >= self.side_clearance, item.clearances[1] >= self.side_clearance
        msg.side_neg_y_accessible, msg.side_pos_y_accessible = item.clearances[2] >= self.side_clearance, item.clearances[3] >= self.side_clearance
        for source in item.grasps:
            grasp = KtyGraspCandidate()
            grasp.strategy, grasp.score, grasp.required_clearance_m = source["strategy"], float(source["score"]), float(source["clearance"])
            grasp.pose.position.x, grasp.pose.position.y, grasp.pose.position.z = source["position"]
            grasp.pose.orientation.z, grasp.pose.orientation.w = math.sin(0.5 * item.yaw), math.cos(0.5 * item.yaw)
            grasp.approach_vector.x, grasp.approach_vector.y, grasp.approach_vector.z = source["approach"]
            msg.grasp_candidates.append(grasp)
        return msg

    def _debug(self, source, rgb, roi, height, boundary, visible, occluded, intrinsics):
        image = rgb.copy()
        cv2.drawContours(image, cv2.findContours(roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0], -1, (255,255,255), 1)
        fx, fy, cx, cy = intrinsics
        for item in visible:
            cv2.drawContours(image, [item.contour_px], -1, (0,255,0), 2)
            d = max(0.15, self.segmenter.camera_to_bottom - item.centroid[2])
            rectangle = np.array([(int(cx + x * fx / d), int(cy - y * fy / d)) for x, y in item.rectangle_xy], np.int32)
            cv2.polylines(image, [rectangle], True, (255,180,0), 2, cv2.LINE_AA)
            u, v = int(cx + item.centroid[0] * fx / d), int(cy - item.centroid[1] * fy / d)
            label = f"ID {item.track_id} VISIBLE occ={item.occlusion:.2f} g={len(item.grasps)}"
            cv2.putText(image, label, (u-45,v), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0,0,0), 3, cv2.LINE_AA)
            cv2.putText(image, label, (u-45,v), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0,255,255), 1, cv2.LINE_AA)
        status = f"visible={len(visible)} occluded={len(occluded)} max_h={float(np.max(height)):.3f}"
        cv2.putText(image, status, (12,24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (15,15,15), 3, cv2.LINE_AA)
        cv2.putText(image, status, (12,24), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255,240,80), 1, cv2.LINE_AA)
        thumb = cv2.applyColorMap(np.uint8(np.clip(boundary * 255, 0, 255)), cv2.COLORMAP_MAGMA)
        width = min(220, image.shape[1] // 3)
        thumb = cv2.resize(thumb, (width, int(thumb.shape[0] * width / thumb.shape[1])))
        image[-thumb.shape[0]:, -thumb.shape[1]:] = thumb
        output = self.bridge.cv2_to_imgmsg(image, "bgr8")
        output.header = source.header
        self.debug_publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = KtyClassical3DPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
