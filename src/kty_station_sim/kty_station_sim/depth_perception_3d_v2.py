"""Fault-contained runtime wrapper for classical 3-D perception."""

from __future__ import annotations

import math
import time
import traceback

from geometry_msgs.msg import Point32
import rclpy
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String

from singulator_interfaces.msg import KtyGraspCandidate, KtyProductContour

from .depth_perception_3d import KtyClassical3DPerception


class KtyClassical3DPerceptionV2(KtyClassical3DPerception):
    """Throttle heavy processing and contain callback exceptions.

    In the first 3-D runtime an exception from a single image callback escaped
    the executor, so the node disappeared while recorder and dashboard stayed
    alive.  Runtime v7 keeps the process present, publishes a fault description
    and continues with the next frame.
    """

    def __init__(self) -> None:
        super().__init__()
        self._last_processed = 0.0
        self._processing_period = 0.25  # 4 Hz at the balanced 8 Hz sensor rate.
        self._consecutive_failures = 0
        self._fault_pub = self.create_publisher(
            String,
            "/kty/perception/fault",
            10,
        )
        self.get_logger().info("Classical 3-D perception runtime v2 started at 4 Hz")

    def _on_depth(self, message) -> None:
        now = time.monotonic()
        if now - self._last_processed < self._processing_period:
            return
        self._last_processed = now
        try:
            super()._on_depth(message)
            self._consecutive_failures = 0
        except Exception as error:  # keep executor alive on malformed frames
            self._consecutive_failures += 1
            details = traceback.format_exc(limit=8)
            self.get_logger().error(
                f"3-D frame failed ({self._consecutive_failures} consecutive): "
                f"{error!r}\n{details}"
            )
            fault = String()
            fault.data = (
                f"frame_failure count={self._consecutive_failures} "
                f"type={type(error).__name__} detail={error}"
            )
            self._fault_pub.publish(fault)

    @staticmethod
    def _point(x: float, y: float, z: float) -> Point32:
        point = Point32()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    def _message(self, item) -> KtyProductContour:
        # Use explicit field assignment.  This works across Jazzy generated
        # message implementations and avoids keyword-constructor / tuple
        # assignment differences seen between local overlays.
        message = KtyProductContour()
        message.track_id = int(item.track_id)
        message.tracking_state = str(item.state)
        message.centroid = self._point(*item.centroid)
        message.top_height_m = float(item.centroid[2])
        message.visible_area_m2 = float(item.area_m2)
        message.confidence = float(item.confidence)
        message.yaw_rad = float(item.yaw)
        message.occlusion_score = float(item.occlusion)
        message.top_accessible = bool(item.top_accessible)
        message.surface_normal.x = float(item.normal[0])
        message.surface_normal.y = float(item.normal[1])
        message.surface_normal.z = float(item.normal[2])
        message.estimated_size.x = float(item.size_xyz[0])
        message.estimated_size.y = float(item.size_xyz[1])
        message.estimated_size.z = float(item.size_xyz[2])

        for x, y in item.polygon_xy:
            message.polygon.points.append(
                self._point(x, y, item.centroid[2])
            )
        for x, y in item.rectangle_xy:
            message.oriented_rectangle.points.append(
                self._point(x, y, item.centroid[2])
            )

        clearances = tuple(float(value) for value in item.clearances)
        message.clearance_neg_x_m = clearances[0]
        message.clearance_pos_x_m = clearances[1]
        message.clearance_neg_y_m = clearances[2]
        message.clearance_pos_y_m = clearances[3]
        message.side_neg_x_accessible = clearances[0] >= self.side_clearance
        message.side_pos_x_accessible = clearances[1] >= self.side_clearance
        message.side_neg_y_accessible = clearances[2] >= self.side_clearance
        message.side_pos_y_accessible = clearances[3] >= self.side_clearance

        if item.state == "OCCLUDED":
            # Hidden tracks remain in memory but are never actionable.
            return message

        for source in item.grasps:
            grasp = KtyGraspCandidate()
            grasp.strategy = str(source["strategy"])
            grasp.score = float(source["score"])
            grasp.required_clearance_m = float(source["clearance"])
            grasp.pose.position.x = float(source["position"][0])
            grasp.pose.position.y = float(source["position"][1])
            grasp.pose.position.z = float(source["position"][2])
            grasp.pose.orientation.z = math.sin(0.5 * float(item.yaw))
            grasp.pose.orientation.w = math.cos(0.5 * float(item.yaw))
            grasp.approach_vector.x = float(source["approach"][0])
            grasp.approach_vector.y = float(source["approach"][1])
            grasp.approach_vector.z = float(source["approach"][2])
            message.grasp_candidates.append(grasp)
        return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyClassical3DPerceptionV2()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
