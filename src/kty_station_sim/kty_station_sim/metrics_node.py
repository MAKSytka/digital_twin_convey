"""Experiment metrics for vibration compaction and RGB-D perception."""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import csv
import json
import math
from pathlib import Path
import re

import numpy as np
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from singulator_interfaces.msg import (
    KtyGroundTruthArray,
    KtyProductContourArray,
    KtyStationState,
)


PRODUCT_PATTERN = re.compile(r"(kty_product_c\d+_p\d+)")
KTY_PATTERN = re.compile(r"(kty_\d{6})")


@dataclass(slots=True)
class PoseSample:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    stamp_s: float
    speed: float = 0.0


@dataclass(slots=True)
class Snapshot:
    label: str
    stamp_s: float
    product_count: int
    fill_ratio: float
    maximum_height_m: float
    void_count: int
    void_volume_m3: float


class KtyMetrics(Node):
    def __init__(self) -> None:
        super().__init__("kty_metrics")
        defaults = {
            "output_directory": "/tmp/kty_station_metrics",
            "voxel_size_m": 0.02,
            "internal_length_m": 0.60,
            "internal_width_m": 0.40,
            "internal_height_m": 0.40,
            "settled_speed_mps": 0.03,
            "settled_hold_s": 0.20,
            "vision_match_distance_m": 0.08,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.output_directory = Path(
            str(self.get_parameter("output_directory").value)
        )
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self.voxel = float(self.get_parameter("voxel_size_m").value)
        self.length = float(self.get_parameter("internal_length_m").value)
        self.width = float(self.get_parameter("internal_width_m").value)
        self.height = float(self.get_parameter("internal_height_m").value)
        self.settled_speed = float(
            self.get_parameter("settled_speed_mps").value
        )
        self.settled_hold = float(self.get_parameter("settled_hold_s").value)
        self.vision_match_distance = float(
            self.get_parameter("vision_match_distance_m").value
        )

        self.state: KtyStationState | None = None
        self.previous_state = -1
        self.registry = {}
        self.poses: dict[str, PoseSample] = {}
        self.initial_poses: dict[str, PoseSample] = {}
        self.latest_perception: KtyProductContourArray | None = None
        self.snapshots: list[Snapshot] = []
        self.vibration_pairs: list[dict] = []
        self.pending_before: Snapshot | None = None
        self.timeseries: list[dict] = []
        self.vision_totals = {"tp": 0, "fp": 0, "fn": 0, "errors": []}
        self.settle_started_s: float | None = None
        self.settled_since_s: float | None = None
        self.settle_times: list[float] = []
        self.finalized_cycles: set[int] = set()

        self.create_subscription(
            TFMessage, "/kty/world/poses", self._on_poses, 20
        )
        self.create_subscription(
            KtyStationState, "/kty/station/state", self._on_state, 10
        )
        self.create_subscription(
            KtyGroundTruthArray,
            "/kty/ground_truth/registry",
            self._on_registry,
            10,
        )
        self.create_subscription(
            KtyProductContourArray,
            "/kty/perception/contours",
            self._on_perception,
            10,
        )
        self.timer = self.create_timer(0.05, self._sample)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _normalise_name(self, frame: str) -> str | None:
        match = PRODUCT_PATTERN.search(frame)
        if match:
            return match.group(1)
        match = KTY_PATTERN.search(frame)
        if match:
            return match.group(1)
        return None

    def _on_poses(self, message: TFMessage) -> None:
        now = self._now_s()
        for transform in message.transforms:
            name = self._normalise_name(transform.child_frame_id)
            if name is None:
                continue
            t = transform.transform.translation
            q = transform.transform.rotation
            previous = self.poses.get(name)
            speed = 0.0
            if previous is not None:
                dt = now - previous.stamp_s
                if dt > 1.0e-4:
                    speed = math.sqrt(
                        (t.x - previous.x) ** 2
                        + (t.y - previous.y) ** 2
                        + (t.z - previous.z) ** 2
                    ) / dt
            sample = PoseSample(
                x=float(t.x), y=float(t.y), z=float(t.z),
                qx=float(q.x), qy=float(q.y), qz=float(q.z), qw=float(q.w),
                stamp_s=now, speed=speed,
            )
            self.poses[name] = sample
            if name.startswith("kty_product_"):
                self.initial_poses.setdefault(name, sample)

    def _on_registry(self, message: KtyGroundTruthArray) -> None:
        if self.state is not None and message.cycle_id != self.state.cycle_id:
            return
        self.registry = {item.model_name: item for item in message.products}

    def _on_perception(self, message: KtyProductContourArray) -> None:
        self.latest_perception = message
        self._evaluate_vision(message)

    def _reset_cycle_accumulators(self) -> None:
        self.registry = {}
        self.initial_poses = {}
        self.latest_perception = None
        self.snapshots = []
        self.vibration_pairs = []
        self.pending_before = None
        self.timeseries = []
        self.vision_totals = {"tp": 0, "fp": 0, "fn": 0, "errors": []}
        self.settle_started_s = None
        self.settled_since_s = None
        self.settle_times = []

    def _on_state(self, message: KtyStationState) -> None:
        old_cycle = self.state.cycle_id if self.state is not None else 0
        self.state = message
        if message.cycle_id != old_cycle and message.state == KtyStationState.WAIT_EMPTY_KTY:
            self._reset_cycle_accumulators()

        if message.state == self.previous_state:
            return
        self.previous_state = message.state

        if message.state == KtyStationState.VIBRATE:
            self.pending_before = self._capture_snapshot("before_vibration")
            self.snapshots.append(self.pending_before)
        elif message.state == KtyStationState.SETTLE:
            after = self._capture_snapshot("after_vibration")
            self.snapshots.append(after)
            if self.pending_before is not None:
                self.vibration_pairs.append(
                    {
                        "before": asdict(self.pending_before),
                        "after": asdict(after),
                        "delta_fill_ratio": after.fill_ratio - self.pending_before.fill_ratio,
                        "delta_maximum_height_m": (
                            after.maximum_height_m
                            - self.pending_before.maximum_height_m
                        ),
                        "delta_void_count": after.void_count - self.pending_before.void_count,
                    }
                )
            self.settle_started_s = self._now_s()
            self.settled_since_s = None
        elif message.state == KtyStationState.EJECT_PREP:
            final = self._capture_snapshot("final_before_eject")
            self.snapshots.append(final)
            self._finalize_cycle(message.cycle_id)

    def _kty_pose(self) -> PoseSample | None:
        if self.state is None:
            return None
        return self.poses.get(f"kty_{self.state.cycle_id:06d}")

    def _products_inside(self) -> dict[str, tuple[object, PoseSample]]:
        kty = self._kty_pose()
        if kty is None:
            return {}
        result = {}
        for name, truth in self.registry.items():
            pose = self.poses.get(name)
            if pose is None:
                continue
            if (
                abs(pose.x - kty.x) <= self.length / 2.0 + 0.10
                and abs(pose.y - kty.y) <= self.width / 2.0 + 0.10
                and 0.40 <= pose.z <= 1.00
            ):
                result[name] = (truth, pose)
        return result

    def _rotation_matrix(self, pose: PoseSample) -> np.ndarray:
        x, y, z, w = pose.qx, pose.qy, pose.qz, pose.qw
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1.0e-9:
            return np.eye(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def _voxel_grid(self) -> tuple[np.ndarray, float, float, int, float]:
        nx = max(1, int(math.ceil(self.length / self.voxel)))
        ny = max(1, int(math.ceil(self.width / self.voxel)))
        nz = max(1, int(math.ceil(self.height / self.voxel)))
        xs = (np.arange(nx) + 0.5) * self.length / nx - self.length / 2.0
        ys = (np.arange(ny) + 0.5) * self.width / ny - self.width / 2.0
        zs = (np.arange(nz) + 0.5) * self.height / nz
        xx, yy, zz = np.meshgrid(xs, ys, zs, indexing="ij")
        local_points = np.stack((xx, yy, zz), axis=-1)
        occupied = np.zeros((nx, ny, nz), dtype=bool)

        kty = self._kty_pose()
        if kty is None:
            return occupied, 0.0, 0.0, 0, 0.0
        world_points = local_points.copy()
        world_points[..., 0] += kty.x
        world_points[..., 1] += kty.y
        world_points[..., 2] += 0.50

        for truth, pose in self._products_inside().values():
            rotation = self._rotation_matrix(pose)
            delta = world_points - np.array([pose.x, pose.y, pose.z])
            product_local = np.einsum("...j,ji->...i", delta, rotation)
            half = np.array(
                [truth.size_m.x, truth.size_m.y, truth.size_m.z],
                dtype=np.float64,
            ) / 2.0
            inside = np.all(np.abs(product_local) <= half + self.voxel * 0.35, axis=-1)
            occupied |= inside

        occupied_count = int(np.count_nonzero(occupied))
        fill_ratio = occupied_count / occupied.size
        occupied_z = np.where(np.any(occupied, axis=(0, 1)))[0]
        maximum_height = (
            (int(occupied_z.max()) + 1) * self.height / nz
            if occupied_z.size
            else 0.0
        )
        void_count, void_voxels = self._count_voids(occupied, maximum_height, nz)
        voxel_volume = (self.length / nx) * (self.width / ny) * (self.height / nz)
        return occupied, fill_ratio, maximum_height, void_count, void_voxels * voxel_volume

    def _count_voids(
        self,
        occupied: np.ndarray,
        maximum_height: float,
        nz: int,
    ) -> tuple[int, int]:
        if maximum_height <= 0.0:
            return 0, 0
        top_index = min(nz - 1, max(0, int(math.ceil(maximum_height / self.height * nz)) - 1))
        empty = ~occupied[:, :, : top_index + 1]
        reachable = np.zeros_like(empty, dtype=bool)
        queue: deque[tuple[int, int, int]] = deque()
        # Free space connected to the top of the current pile is not a trapped void.
        for x in range(empty.shape[0]):
            for y in range(empty.shape[1]):
                if empty[x, y, top_index]:
                    reachable[x, y, top_index] = True
                    queue.append((x, y, top_index))
        neighbours = ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))
        while queue:
            x, y, z = queue.popleft()
            for dx, dy, dz in neighbours:
                nx_, ny_, nz_ = x + dx, y + dy, z + dz
                if not (
                    0 <= nx_ < empty.shape[0]
                    and 0 <= ny_ < empty.shape[1]
                    and 0 <= nz_ < empty.shape[2]
                ):
                    continue
                if empty[nx_, ny_, nz_] and not reachable[nx_, ny_, nz_]:
                    reachable[nx_, ny_, nz_] = True
                    queue.append((nx_, ny_, nz_))

        trapped = empty & ~reachable
        visited = np.zeros_like(trapped, dtype=bool)
        component_count = 0
        voxel_count = int(np.count_nonzero(trapped))
        for start in zip(*np.where(trapped & ~visited)):
            component_count += 1
            visited[start] = True
            queue.append(start)
            while queue:
                x, y, z = queue.popleft()
                for dx, dy, dz in neighbours:
                    nx_, ny_, nz_ = x + dx, y + dy, z + dz
                    if not (
                        0 <= nx_ < trapped.shape[0]
                        and 0 <= ny_ < trapped.shape[1]
                        and 0 <= nz_ < trapped.shape[2]
                    ):
                        continue
                    if trapped[nx_, ny_, nz_] and not visited[nx_, ny_, nz_]:
                        visited[nx_, ny_, nz_] = True
                        queue.append((nx_, ny_, nz_))
        return component_count, voxel_count

    def _capture_snapshot(self, label: str) -> Snapshot:
        _, fill_ratio, maximum_height, void_count, void_volume = self._voxel_grid()
        products = self._products_inside()
        return Snapshot(
            label=label,
            stamp_s=self._now_s(),
            product_count=len(products),
            fill_ratio=float(fill_ratio),
            maximum_height_m=max(0.0, maximum_height),
            void_count=int(void_count),
            void_volume_m3=float(void_volume),
        )

    def _sample(self) -> None:
        if self.state is None or self.state.cycle_id <= 0:
            return
        products = self._products_inside()
        speeds = [pose.speed for _, pose in products.values()]
        maximum_speed = max(speeds, default=0.0)
        self.timeseries.append(
            {
                "stamp_s": self._now_s(),
                "cycle_id": self.state.cycle_id,
                "state": self.state.state_name,
                "product_count": len(products),
                "maximum_speed_mps": maximum_speed,
                "perception_maximum_height_m": (
                    self.latest_perception.maximum_height_m
                    if self.latest_perception is not None
                    else 0.0
                ),
                "perception_top_fill_ratio": (
                    self.latest_perception.top_fill_ratio
                    if self.latest_perception is not None
                    else 0.0
                ),
            }
        )

        if self.state.state == KtyStationState.SETTLE and self.settle_started_s is not None:
            now = self._now_s()
            if maximum_speed <= self.settled_speed:
                if self.settled_since_s is None:
                    self.settled_since_s = now
                elif now - self.settled_since_s >= self.settled_hold:
                    self.settle_times.append(now - self.settle_started_s)
                    self.settle_started_s = None
                    self.settled_since_s = None
            else:
                self.settled_since_s = None

    def _evaluate_vision(self, message: KtyProductContourArray) -> None:
        if not message.camera_ok:
            return
        ground_truth = self._products_inside()
        kty = self._kty_pose()
        if kty is None:
            return
        gt_points = {
            name: (pose.x - kty.x, pose.y - kty.y)
            for name, (_, pose) in ground_truth.items()
        }
        detections = [
            (float(item.centroid.x), float(item.centroid.y))
            for item in message.products
        ]
        candidates: list[tuple[float, str, int]] = []
        for name, (x, y) in gt_points.items():
            for index, (dx, dy) in enumerate(detections):
                distance = math.hypot(dx - x, dy - y)
                if distance <= self.vision_match_distance:
                    candidates.append((distance, name, index))
        used_gt: set[str] = set()
        used_det: set[int] = set()
        errors: list[float] = []
        for distance, name, index in sorted(candidates):
            if name in used_gt or index in used_det:
                continue
            used_gt.add(name)
            used_det.add(index)
            errors.append(distance)
        self.vision_totals["tp"] += len(used_gt)
        self.vision_totals["fp"] += len(detections) - len(used_det)
        self.vision_totals["fn"] += len(gt_points) - len(used_gt)
        self.vision_totals["errors"].extend(errors)

    def _finalize_cycle(self, cycle_id: int) -> None:
        if cycle_id in self.finalized_cycles:
            return
        self.finalized_cycles.add(cycle_id)
        cycle_dir = self.output_directory / f"cycle_{cycle_id:06d}"
        cycle_dir.mkdir(parents=True, exist_ok=True)

        with (cycle_dir / "timeseries.csv").open("w", newline="", encoding="utf-8") as stream:
            fields = list(self.timeseries[0].keys()) if self.timeseries else ["stamp_s"]
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.timeseries)

        displacements = []
        for name, initial in self.initial_poses.items():
            final = self.poses.get(name)
            if final is None:
                continue
            displacements.append(
                {
                    "model_name": name,
                    "displacement_m": math.sqrt(
                        (final.x - initial.x) ** 2
                        + (final.y - initial.y) ** 2
                        + (final.z - initial.z) ** 2
                    ),
                    "dx_m": final.x - initial.x,
                    "dy_m": final.y - initial.y,
                    "dz_m": final.z - initial.z,
                }
            )
        with (cycle_dir / "product_displacements.csv").open(
            "w", newline="", encoding="utf-8"
        ) as stream:
            fields = ["model_name", "displacement_m", "dx_m", "dy_m", "dz_m"]
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(displacements)

        tp = int(self.vision_totals["tp"])
        fp = int(self.vision_totals["fp"])
        fn = int(self.vision_totals["fn"])
        precision = tp / max(1, tp + fp)
        recall = tp / max(1, tp + fn)
        f1 = 2.0 * precision * recall / max(1.0e-9, precision + recall)
        errors = self.vision_totals["errors"]
        summary = {
            "cycle_id": cycle_id,
            "snapshots": [asdict(item) for item in self.snapshots],
            "vibration_pairs": self.vibration_pairs,
            "settle_times_s": self.settle_times,
            "mean_settle_time_s": (
                float(np.mean(self.settle_times)) if self.settle_times else None
            ),
            "vision": {
                "true_positive": tp,
                "false_positive": fp,
                "false_negative": fn,
                "precision": precision,
                "recall": recall,
                "f1": f1,
                "mean_position_error_m": float(np.mean(errors)) if errors else None,
            },
            "product_displacement_count": len(displacements),
        }
        (cycle_dir / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.get_logger().info(f"Metrics written to {cycle_dir}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMetrics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
