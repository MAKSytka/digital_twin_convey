"""Persist RGB-D product contours for downstream robot planning."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from singulator_interfaces.msg import KtyProductContourArray


class KtyContourRecorder(Node):
    """Write every contour frame to JSONL and publish a JSON mirror."""

    def __init__(self) -> None:
        super().__init__("kty_contour_recorder")
        self.declare_parameter("input_topic", "/kty/perception/contours")
        self.declare_parameter("json_topic", "/kty/vision/polygons_json")
        self.declare_parameter("output_directory", "~/.ros/kty_vision")
        self.declare_parameter("save_empty_frames", False)

        input_topic = str(self.get_parameter("input_topic").value)
        json_topic = str(self.get_parameter("json_topic").value)
        output_directory = os.path.expandvars(
            os.path.expanduser(str(self.get_parameter("output_directory").value))
        )
        self.save_empty_frames = bool(
            self.get_parameter("save_empty_frames").value
        )

        self.output_directory = Path(output_directory)
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self.history_path = self.output_directory / "polygons.jsonl"
        self.latest_path = self.output_directory / "polygons_latest.json"

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.json_pub = self.create_publisher(String, json_topic, qos)
        self.create_subscription(
            KtyProductContourArray,
            input_topic,
            self._on_contours,
            10,
        )

        self.frames_written = 0
        self.get_logger().info(
            f"Polygon recorder output: {self.output_directory}"
        )

    @staticmethod
    def _stamp_to_dict(message: KtyProductContourArray) -> dict[str, int]:
        return {
            "sec": int(message.header.stamp.sec),
            "nanosec": int(message.header.stamp.nanosec),
        }

    def _serialize(self, message: KtyProductContourArray) -> dict:
        products = []
        for item in message.products:
            products.append(
                {
                    "track_id": int(item.track_id),
                    "centroid_m": {
                        "x": float(item.centroid.x),
                        "y": float(item.centroid.y),
                        "z": float(item.centroid.z),
                    },
                    "top_height_m": float(item.top_height_m),
                    "visible_area_m2": float(item.visible_area_m2),
                    "confidence": float(item.confidence),
                    "polygon_m": [
                        {
                            "x": float(point.x),
                            "y": float(point.y),
                            "z": float(point.z),
                        }
                        for point in item.polygon.points
                    ],
                    "accessible_sides": {
                        "neg_x": bool(item.side_neg_x_accessible),
                        "pos_x": bool(item.side_pos_x_accessible),
                        "neg_y": bool(item.side_neg_y_accessible),
                        "pos_y": bool(item.side_pos_y_accessible),
                    },
                    "clearance_m": {
                        "neg_x": float(item.clearance_neg_x_m),
                        "pos_x": float(item.clearance_pos_x_m),
                        "neg_y": float(item.clearance_neg_y_m),
                        "pos_y": float(item.clearance_pos_y_m),
                    },
                }
            )

        return {
            "schema": "kty_product_polygons/v1",
            "frame_id": message.header.frame_id,
            "stamp": self._stamp_to_dict(message),
            "frame_sequence": int(message.frame_sequence),
            "camera_ok": bool(message.camera_ok),
            "valid_depth_fraction": float(message.valid_depth_fraction),
            "maximum_height_m": float(message.maximum_height_m),
            "top_fill_ratio": float(message.top_fill_ratio),
            "product_count": len(products),
            "products": products,
        }

    def _write_latest_atomically(self, payload: str) -> None:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix="polygons_latest_",
            suffix=".json",
            dir=self.output_directory,
            text=True,
        )
        temporary_path = Path(temporary_name)
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
                stream.write(payload)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            temporary_path.replace(self.latest_path)
        finally:
            if temporary_path.exists():
                temporary_path.unlink(missing_ok=True)

    def _on_contours(self, message: KtyProductContourArray) -> None:
        document = self._serialize(message)
        compact = json.dumps(document, ensure_ascii=False, separators=(",", ":"))

        mirror = String()
        mirror.data = compact
        self.json_pub.publish(mirror)

        if not document["products"] and not self.save_empty_frames:
            return

        with self.history_path.open("a", encoding="utf-8") as stream:
            stream.write(compact)
            stream.write("\n")
        self._write_latest_atomically(
            json.dumps(document, ensure_ascii=False, indent=2)
        )
        self.frames_written += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyContourRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
