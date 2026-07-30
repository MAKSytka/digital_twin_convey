from __future__ import annotations

from .contour_recorder import KtyContourRecorder

import rclpy


class KtyContourRecorder3D(KtyContourRecorder):
    """Recorder with extended 3-D planning fields in the JSON schema."""

    def __init__(self) -> None:
        super().__init__()
        self.get_logger().info("Extended 3-D carton schema enabled")

    def _document_from_message(self, message):
        document = super()._document_from_message(message)
        document["schema"] = "kty_carton_instances_3d/v2"
        products = document.get("products", [])
        visible_count = 0
        occluded_count = 0
        grasp_count = 0
        for target, source in zip(products, message.products):
            state = str(source.tracking_state)
            if state == "OCCLUDED":
                occluded_count += 1
            else:
                visible_count += 1
            target.update(
                {
                    "tracking_state": state,
                    "oriented_rectangle_m": [
                        {
                            "x": float(point.x),
                            "y": float(point.y),
                            "z": float(point.z),
                        }
                        for point in source.oriented_rectangle
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
                            "type": str(candidate.type),
                            "position": {
                                "x": float(candidate.position.x),
                                "y": float(candidate.position.y),
                                "z": float(candidate.position.z),
                            },
                            "approach": {
                                "x": float(candidate.approach.x),
                                "y": float(candidate.approach.y),
                                "z": float(candidate.approach.z),
                            },
                            "normal": {
                                "x": float(candidate.normal.x),
                                "y": float(candidate.normal.y),
                                "z": float(candidate.normal.z),
                            },
                            "score": float(candidate.score),
                            "clearance_m": float(candidate.clearance_m),
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
