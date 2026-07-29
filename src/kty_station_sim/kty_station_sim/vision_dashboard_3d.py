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
        y = max(610, height - 132)
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
