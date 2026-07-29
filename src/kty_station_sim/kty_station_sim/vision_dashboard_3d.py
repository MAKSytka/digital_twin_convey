"""Stage-6 dashboard overlay for OBBs, occlusions and grasp candidates."""

from __future__ import annotations

import cv2
import numpy as np
import rclpy

from .vision_dashboard import KtyVisionDashboard


class KtyVisionDashboard3D(KtyVisionDashboard):
    def __init__(self) -> None:
        super().__init__()
        self.get_logger().info("Classical 3-D instance dashboard enabled")

    def _top_view(self, width: int, height: int) -> np.ndarray:
        view = super()._top_view(width, height)
        message = self.latest_contours
        if message is None:
            return view

        for item in message.products:
            state = str(item.tracking_state or "VISIBLE")
            rectangle = [
                self._map_point(point.x, point.y, width, height)
                for point in item.oriented_rectangle.points
            ]
            if len(rectangle) == 4:
                color = (80, 220, 255) if state == "VISIBLE" else (125, 125, 150)
                cv2.polylines(
                    view,
                    [np.asarray(rectangle, dtype=np.int32)],
                    True,
                    color,
                    2,
                    cv2.LINE_AA,
                )
            for candidate in item.grasp_candidates[:3]:
                start = self._map_point(
                    candidate.pose.position.x,
                    candidate.pose.position.y,
                    width,
                    height,
                )
                end = (
                    int(start[0] + 22 * candidate.approach_vector.x),
                    int(start[1] - 22 * candidate.approach_vector.y),
                )
                cv2.circle(view, start, 4, (0, 255, 255), -1, cv2.LINE_AA)
                cv2.arrowedLine(
                    view,
                    start,
                    end,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                    tipLength=0.35,
                )
        return view

    def _stats_panel(self, width: int, height: int) -> np.ndarray:
        panel = super()._stats_panel(width, height)
        message = self.latest_contours

        # Runtime-v10 vibration telemetry is carried in /kty/flow/state.  This
        # makes the physical profile visible even when the five-millimetre deck
        # stroke is difficult to judge from the overview camera.
        flow = self.latest_flow_state
        mode = str(flow.get("vibration_mode", "off"))
        command_mm = 1000.0 * float(flow.get("vibration_command_m", 0.0) or 0.0)
        frequency_hz = float(flow.get("vibration_frequency_hz", 0.0) or 0.0)
        accel_g = float(flow.get("vibration_peak_accel_g", 0.0) or 0.0)
        compaction = flow.get("last_compaction", {})
        if not isinstance(compaction, dict):
            compaction = {}
        height_drop_mm = 1000.0 * float(compaction.get("height_drop_m", 0.0) or 0.0)

        vibration_y = 610
        cv2.rectangle(
            panel,
            (18, vibration_y - 24),
            (width - 18, vibration_y + 124),
            (28, 34, 42),
            -1,
        )
        color = (95, 235, 170) if mode == "strong" else (130, 205, 245)
        self._put_text(panel, "Vibration / compaction", (24, vibration_y), 0.52, (255, 225, 115), 2)
        self._put_text(
            panel,
            f"Mode: {mode}   command: {command_mm:+4.1f} mm",
            (24, vibration_y + 28),
            0.46,
            color,
        )
        self._put_text(
            panel,
            f"Frequency: {frequency_hz:4.1f} Hz   peak: {accel_g:3.1f} g",
            (24, vibration_y + 54),
            0.46,
        )
        self._put_text(
            panel,
            f"Last max-height drop: {height_drop_mm:+5.1f} mm",
            (24, vibration_y + 80),
            0.46,
            (95, 235, 170) if height_drop_mm > 0.0 else (185, 195, 205),
        )
        bar_x, bar_y, bar_w, bar_h = 24, vibration_y + 94, width - 48, 12
        cv2.rectangle(panel, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (50, 60, 72), -1)
        normalized = min(1.0, abs(command_mm) / 5.0)
        cv2.rectangle(
            panel,
            (bar_x, bar_y),
            (bar_x + int(bar_w * normalized), bar_y + bar_h),
            color,
            -1,
        )

        if message is None:
            return panel
        visible = sum(
            1 for item in message.products if str(item.tracking_state) != "OCCLUDED"
        )
        occluded = sum(
            1 for item in message.products if str(item.tracking_state) == "OCCLUDED"
        )
        grasps = sum(len(item.grasp_candidates) for item in message.products)
        top_ready = sum(bool(item.top_accessible) for item in message.products)
        y = max(768, height - 132)
        cv2.rectangle(panel, (18, y - 24), (width - 18, height - 12), (28, 34, 42), -1)
        self._put_text(panel, "3-D instance planning", (24, y), 0.52, (255, 225, 115), 2)
        self._put_text(panel, f"VISIBLE / OCCLUDED: {visible} / {occluded}", (24, y + 28), 0.48)
        self._put_text(panel, f"Top accessible: {top_ready}", (24, y + 54), 0.48)
        self._put_text(panel, f"Grasp candidates: {grasps}", (24, y + 80), 0.48, (95, 235, 170))
        return panel


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyVisionDashboard3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
