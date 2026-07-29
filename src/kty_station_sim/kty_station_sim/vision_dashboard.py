"""Live operator dashboard for the KTY RGB-D perception pipeline."""

from __future__ import annotations

import json
import math

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from singulator_interfaces.msg import KtyProductContourArray


class KtyVisionDashboard(Node):
    """Compose RGB, depth, top-view polygons and diagnostics into one UI."""

    CANVAS_WIDTH = 1440
    CANVAS_HEIGHT = 900
    IMAGE_WIDTH = 800
    IMAGE_HEIGHT = 600
    PANEL_X = 1020

    def __init__(self) -> None:
        super().__init__("kty_vision_dashboard")
        defaults = {
            "rgb_topic": "/kty/vision/image",
            "depth_topic": "/kty/vision/depth_image",
            "debug_topic": "/kty/perception/debug_image",
            "contours_topic": "/kty/perception/contours",
            "flow_state_topic": "/kty/flow/state",
            "dashboard_topic": "/kty/vision/dashboard",
            "show_window": True,
            "window_title": "KTY RGB-D Vision Dashboard",
            "refresh_hz": 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.show_window = bool(self.get_parameter("show_window").value)
        self.window_title = str(self.get_parameter("window_title").value)
        refresh_hz = float(self.get_parameter("refresh_hz").value)
        if refresh_hz <= 0.0:
            raise ValueError("refresh_hz must be positive")

        self.bridge = CvBridge()
        self.latest_rgb: np.ndarray | None = None
        self.latest_depth: np.ndarray | None = None
        self.latest_debug: np.ndarray | None = None
        self.latest_contours: KtyProductContourArray | None = None
        self.latest_flow_state: dict = {}
        self.last_header = None
        self.window_failed = False
        self.rendered_frames = 0

        self.dashboard_pub = self.create_publisher(
            Image,
            str(self.get_parameter("dashboard_topic").value),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("rgb_topic").value),
            self._on_rgb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("depth_topic").value),
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("debug_topic").value),
            self._on_debug,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            KtyProductContourArray,
            str(self.get_parameter("contours_topic").value),
            self._on_contours,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter("flow_state_topic").value),
            self._on_flow_state,
            10,
        )
        self.timer = self.create_timer(1.0 / refresh_hz, self._render)

        if self.show_window:
            try:
                cv2.namedWindow(self.window_title, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_title, 1280, 800)
            except cv2.error as error:
                self.window_failed = True
                self.get_logger().warning(f"Dashboard window unavailable: {error}")

    def _on_rgb(self, message: Image) -> None:
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
            self.last_header = message.header
        except Exception as error:  # pragma: no cover - runtime conversion guard
            self.get_logger().warning(f"RGB dashboard conversion failed: {error}")

    def _on_depth(self, message: Image) -> None:
        try:
            depth = np.asarray(
                self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
            )
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) * 0.001
            else:
                depth = depth.astype(np.float32)
            self.latest_depth = depth
            self.last_header = message.header
        except Exception as error:  # pragma: no cover - runtime conversion guard
            self.get_logger().warning(f"Depth dashboard conversion failed: {error}")

    def _on_debug(self, message: Image) -> None:
        try:
            self.latest_debug = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
            self.last_header = message.header
        except Exception as error:  # pragma: no cover - runtime conversion guard
            self.get_logger().warning(f"Debug image conversion failed: {error}")

    def _on_contours(self, message: KtyProductContourArray) -> None:
        self.latest_contours = message

    def _on_flow_state(self, message: String) -> None:
        try:
            self.latest_flow_state = json.loads(message.data)
        except json.JSONDecodeError:
            self.latest_flow_state = {"state": "INVALID_JSON"}

    @staticmethod
    def _put_text(
        image: np.ndarray,
        text: str,
        origin: tuple[int, int],
        scale: float = 0.58,
        color: tuple[int, int, int] = (225, 235, 245),
        thickness: int = 1,
    ) -> None:
        cv2.putText(
            image,
            text,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )

    @classmethod
    def _title(
        cls,
        image: np.ndarray,
        text: str,
        origin: tuple[int, int],
    ) -> None:
        cls._put_text(image, text, origin, 0.72, (255, 230, 120), 2)

    def _depth_view(self, width: int, height: int) -> np.ndarray:
        if self.latest_depth is None:
            view = np.zeros((height, width, 3), dtype=np.uint8)
            self._put_text(view, "Waiting for depth image...", (24, 45))
            return view

        depth = self.latest_depth
        valid = np.isfinite(depth) & (depth > 0.20) & (depth < 3.0)
        normalized = np.zeros(depth.shape, dtype=np.uint8)
        if np.any(valid):
            clipped = np.clip(depth, 0.35, 1.80)
            normalized[valid] = np.uint8(
                255.0 * (1.80 - clipped[valid]) / (1.80 - 0.35)
            )
        colored = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        colored[~valid] = (18, 20, 24)
        return cv2.resize(colored, (width, height), interpolation=cv2.INTER_NEAREST)

    @staticmethod
    def _map_point(
        x: float,
        y: float,
        width: int,
        height: int,
        margin: int = 34,
    ) -> tuple[int, int]:
        u = margin + (x + 0.30) / 0.60 * (width - 2 * margin)
        v = margin + (0.20 - y) / 0.40 * (height - 2 * margin)
        return int(round(u)), int(round(v))

    def _top_view(self, width: int, height: int) -> np.ndarray:
        view = np.full((height, width, 3), (24, 28, 34), dtype=np.uint8)
        margin = 34
        cv2.rectangle(
            view,
            (margin, margin),
            (width - margin, height - margin),
            (105, 125, 145),
            2,
        )
        cv2.line(
            view,
            (width // 2, margin),
            (width // 2, height - margin),
            (55, 65, 76),
            1,
        )
        cv2.line(
            view,
            (margin, height // 2),
            (width - margin, height // 2),
            (55, 65, 76),
            1,
        )

        message = self.latest_contours
        if message is None:
            self._put_text(view, "Waiting for contour polygons...", (24, 42))
            return view

        palette = (
            (70, 220, 255),
            (95, 255, 140),
            (255, 165, 80),
            (220, 120, 255),
            (90, 190, 255),
            (255, 220, 90),
        )
        for index, item in enumerate(message.products):
            points = [
                self._map_point(point.x, point.y, width, height, margin)
                for point in item.polygon.points
            ]
            if len(points) < 3:
                continue
            color = palette[index % len(palette)]
            polygon = np.asarray(points, dtype=np.int32).reshape((-1, 1, 2))
            overlay = view.copy()
            cv2.fillPoly(overlay, [polygon], color)
            cv2.addWeighted(overlay, 0.24, view, 0.76, 0.0, view)
            cv2.polylines(view, [polygon], True, color, 2, cv2.LINE_AA)
            center = self._map_point(
                item.centroid.x,
                item.centroid.y,
                width,
                height,
                margin,
            )
            cv2.circle(view, center, 4, color, -1, cv2.LINE_AA)
            self._put_text(
                view,
                f"ID {item.track_id}",
                (center[0] + 7, center[1] - 7),
                0.48,
                color,
                1,
            )

        self._put_text(view, "-X", (margin - 4, height - 10), 0.46)
        self._put_text(view, "+X", (width - margin - 24, height - 10), 0.46)
        self._put_text(view, "+Y", (7, margin + 5), 0.46)
        return view

    @staticmethod
    def _accessibility(item) -> str:
        labels = []
        if item.side_neg_x_accessible:
            labels.append("-X")
        if item.side_pos_x_accessible:
            labels.append("+X")
        if item.side_neg_y_accessible:
            labels.append("-Y")
        if item.side_pos_y_accessible:
            labels.append("+Y")
        return " ".join(labels) if labels else "none"

    def _stats_panel(self, width: int, height: int) -> np.ndarray:
        panel = np.full((height, width, 3), (19, 23, 29), dtype=np.uint8)
        self._title(panel, "KTY VISION", (24, 42))
        self._put_text(panel, "RGB-D contour tracking", (24, 70), 0.50, (145, 175, 205))
        cv2.line(panel, (24, 86), (width - 24, 86), (60, 75, 90), 1)

        state = str(self.latest_flow_state.get("state", "WAITING"))
        cycle = int(self.latest_flow_state.get("cycle_id", 0) or 0)
        self._put_text(panel, f"Cycle: {cycle}", (24, 120), 0.60)
        self._put_text(panel, f"State: {state}", (24, 151), 0.60, (90, 235, 180), 2)

        contours = self.latest_contours
        camera_ok = bool(contours.camera_ok) if contours is not None else False
        valid = float(contours.valid_depth_fraction) if contours is not None else 0.0
        maximum = float(contours.maximum_height_m) if contours is not None else 0.0
        fill = float(contours.top_fill_ratio) if contours is not None else 0.0
        frame = int(contours.frame_sequence) if contours is not None else 0
        count = len(contours.products) if contours is not None else 0

        status_color = (95, 235, 150) if camera_ok else (90, 120, 255)
        self._put_text(panel, f"Camera: {'OK' if camera_ok else 'WAIT'}", (24, 197), 0.62, status_color, 2)
        self._put_text(panel, f"Frame: {frame}", (24, 228))
        self._put_text(panel, f"Tracked objects: {count}", (24, 257))
        self._put_text(panel, f"Valid depth: {valid * 100.0:5.1f}%", (24, 286))
        self._put_text(panel, f"Max height: {maximum * 1000.0:5.0f} mm", (24, 315))
        self._put_text(panel, f"Top fill: {fill * 100.0:5.1f}%", (24, 344))

        bar_x, bar_y, bar_w, bar_h = 24, 365, width - 48, 14
        cv2.rectangle(panel, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (50, 60, 72), -1)
        cv2.rectangle(
            panel,
            (bar_x, bar_y),
            (bar_x + int(bar_w * min(1.0, max(0.0, fill))), bar_y + bar_h),
            (85, 205, 165),
            -1,
        )

        cv2.line(panel, (24, 405), (width - 24, 405), (60, 75, 90), 1)
        self._put_text(panel, "Detected products", (24, 434), 0.58, (255, 230, 120), 2)
        y = 466
        if contours is None or not contours.products:
            self._put_text(panel, "No product polygons", (24, y), 0.52, (145, 160, 175))
        else:
            for item in contours.products[:8]:
                self._put_text(
                    panel,
                    f"ID {item.track_id:02d}  h={item.top_height_m * 1000:4.0f}mm  "
                    f"p={len(item.polygon.points):02d}",
                    (24, y),
                    0.49,
                    (220, 230, 240),
                )
                y += 25
                self._put_text(
                    panel,
                    f"access: {self._accessibility(item)}",
                    (40, y),
                    0.43,
                    (130, 205, 245),
                )
                y += 29
                if y > height - 65:
                    break

        self._put_text(
            panel,
            "polygons: ~/.ros/kty_vision/",
            (24, height - 34),
            0.43,
            (125, 145, 165),
        )
        return panel

    def _render(self) -> None:
        canvas = np.full(
            (self.CANVAS_HEIGHT, self.CANVAS_WIDTH, 3),
            (14, 17, 22),
            dtype=np.uint8,
        )

        source = self.latest_debug if self.latest_debug is not None else self.latest_rgb
        if source is None:
            rgb_view = np.zeros((self.IMAGE_HEIGHT, self.IMAGE_WIDTH, 3), dtype=np.uint8)
            self._put_text(rgb_view, "Waiting for RGB-D camera...", (36, 62), 0.82)
        else:
            rgb_view = cv2.resize(
                source,
                (self.IMAGE_WIDTH, self.IMAGE_HEIGHT),
                interpolation=cv2.INTER_AREA,
            )

        canvas[60:660, 30:830] = rgb_view
        self._title(canvas, "RGB + detected contours", (30, 40))

        depth_view = self._depth_view(390, 200)
        top_view = self._top_view(390, 200)
        canvas[690:890, 30:420] = depth_view
        canvas[690:890, 440:830] = top_view
        self._put_text(canvas, "Depth heatmap", (30, 682), 0.54, (255, 230, 120), 2)
        self._put_text(canvas, "KTY top-view polygons", (440, 682), 0.54, (255, 230, 120), 2)

        panel = self._stats_panel(390, self.CANVAS_HEIGHT - 20)
        canvas[10:890, self.PANEL_X:1410] = panel
        cv2.rectangle(canvas, (20, 50), (840, 895), (48, 58, 70), 1)
        cv2.rectangle(canvas, (1010, 5), (1420, 895), (48, 58, 70), 1)

        output = self.bridge.cv2_to_imgmsg(canvas, encoding="bgr8")
        if self.last_header is not None:
            output.header = self.last_header
        output.header.frame_id = "kty_vision_dashboard"
        self.dashboard_pub.publish(output)
        self.rendered_frames += 1

        if self.show_window and not self.window_failed:
            try:
                cv2.imshow(self.window_title, canvas)
                cv2.waitKey(1)
            except cv2.error as error:
                self.window_failed = True
                self.get_logger().warning(f"Dashboard window disabled: {error}")

    def destroy_node(self):
        if self.show_window and not self.window_failed:
            try:
                cv2.destroyWindow(self.window_title)
            except cv2.error:
                pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyVisionDashboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
