"""Runtime-v15: reliable model poses through a Gazebo JSON registry.

The Jazzy Pose_V -> TFMessage bridge can expose an empty / incomplete transform
stream for SceneBroadcaster dynamic poses.  Runtime v15 receives model names and
world positions from the already loaded KtyConveyorSurfaceSystem plugin through a
plain gz.msgs.StringMsg -> std_msgs/String bridge.  No `gz topic` subprocesses are
used in the control loop.
"""

from __future__ import annotations

import json
import threading
import time

import rclpy
from std_msgs.msg import String

from .mechatronics_cycle import Pose
from .mechatronics_cycle_v14 import KtyMechatronicsCycleV14


class KtyMechatronicsCycleV15(KtyMechatronicsCycleV14):
    """V14 mechanics with an explicit, named JSON model-pose registry."""

    REGISTRY_TOPIC = "/kty/mech/model_pose_registry_json"

    def __init__(self) -> None:
        self._v15_ready = threading.Event()
        self._registry_sequence = 0
        self._registry_parse_errors = 0
        self._registry_model_count = 0
        super().__init__()

        self.create_subscription(
            String,
            self.REGISTRY_TOPIC,
            self._on_pose_registry,
            10,
        )
        self._v15_ready.set()
        self.get_logger().info(
            "Runtime v15: Gazebo plugin JSON pose registry; TF Pose_V feedback disabled"
        )

    def _worker_main(self) -> None:
        self._v15_ready.wait()
        super()._worker_main()

    def _on_model_poses(self, message) -> None:
        """Ignore the legacy TF bridge if an old launch still starts it."""
        del message

    def _on_pose_registry(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            if payload.get("schema") != "kty_model_pose_registry/v1":
                raise ValueError("unexpected pose-registry schema")
            raw_models = payload.get("models")
            if not isinstance(raw_models, list):
                raise ValueError("models is not a list")

            poses: dict[str, Pose] = {}
            for item in raw_models:
                if not isinstance(item, dict):
                    continue
                name = str(item.get("name", "")).strip()
                if not name:
                    continue
                poses[name] = Pose(
                    x=float(item["x"]),
                    y=float(item["y"]),
                    z=float(item["z"]),
                )
            sequence = int(payload.get("sequence", 0))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self._registry_parse_errors += 1
            if self._registry_parse_errors <= 3:
                self.get_logger().warning(f"Invalid pose registry frame: {error}")
            return

        with self._pose_cache_lock:
            self._pose_cache = poses
            self._pose_cache_received_at = time.monotonic()
            self._registry_sequence = sequence
            self._registry_model_count = len(poses)

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        payload.update(
            {
                "runtime_profile": "kty_mechatronics_v15",
                "pose_feedback": "gazebo_plugin_json_registry",
                "pose_registry_topic": self.REGISTRY_TOPIC,
                "pose_registry_sequence": self._registry_sequence,
                "pose_registry_models": self._registry_model_count,
                "pose_registry_parse_errors": self._registry_parse_errors,
            }
        )
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV15()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
