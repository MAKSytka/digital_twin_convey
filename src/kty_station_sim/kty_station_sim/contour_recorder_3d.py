"""Extended polygon / OBB / grasp-candidate persistence for stage 6."""

from __future__ import annotations

import rclpy

from .contour_recorder import KtyContourRecorder


class KtyContourRecorder3D(KtyContourRecorder):
    def __init__(self) -> None:
        super().__init__()
        self.get_logger().info("Extended 3-D carton schema enabled")

    def _serialize(self, message):
        document = super()._serialize(message)
        document["schema"] = "kty_carton_instances_3d/v2"
        visible_count = 0
        occluded_count = 0
        grasp_count = 0
        for source, target in zip(message.products, document["products"]):
            state = str(source.tracking_state or "VISIBLE")
            if state == "OCCLUDED":
                occluded_count += 1
            else:
                visible_count += 1
            target.update(
                {
                    "tracking_state": state,
                    "oriented_rectangle_m": [
                        {"x": float(point.x), "y": float(point.y), "z": float(point.z)}
                        for point in source.oriented_rectangle.points
                    ],
                    "surface_normal": {
                        "x": float(source.surface_normal.x),
                        "y": float(source.surface_normal.y),
                        "z": float(source.surface_normal.z),
                    },
                    "estimated_size_m": {
                        "x": float(source.estimated_size.x),
                        "y": float(source.estimated_size.y),
                        "z": float(source.estimated_size.z),
                    },
                    "yaw_rad": float(source.yaw_rad),
                    "occlusion_score": float(source.occlusion_score),
                    "top_accessible": bool(source.top_accessible),
                    "grasp_candidates": [
                        {
                            "strategy": str(candidate.strategy),
                            "score": float(candidate.score),
                            "required_clearance_m": float(candidate.required_clearance_m),
                            "pose": {
                                "position": {
                                    "x": float(candidate.pose.position.x),
                                    "y": float(candidate.pose.position.y),
                                    "z": float(candidate.pose.position.z),
                                },
                                "orientation": {
                                    "x": float(candidate.pose.orientation.x),
                                    "y": float(candidate.pose.orientation.y),
                                    "z": float(candidate.pose.orientation.z),
                                    "w": float(candidate.pose.orientation.w),
                                },
                            },
                            "approach_vector": {
                                "x": float(candidate.approach_vector.x),
                                "y": float(candidate.approach_vector.y),
                                "z": float(candidate.approach_vector.z),
                            },
                        }
                        for candidate in source.grasp_candidates
                    ],
                }
            )
            grasp_count += len(source.grasp_candidates)
        document["visible_count"] = visible_count
        document["occluded_count"] = occluded_count
        document["grasp_candidate_count"] = grasp_count
        return document


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyContourRecorder3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
