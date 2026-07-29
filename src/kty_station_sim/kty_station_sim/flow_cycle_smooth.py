"""Smooth stage-2 KTY flow cycle with an explicit vibration phase.

The original deterministic cycle remains available as a small, readable base
implementation.  This subclass keeps the same Gazebo UserCommands transport,
but improves the visible motion profile and adds a five-second vertical
vibration phase after loading.
"""

from __future__ import annotations

import math
import time

import rclpy

from .flow_cycle import (
    KtyFlowCycle,
    PRODUCT_PROFILES,
    Pose,
    make_flow_product_sdf,
)
from .model_factory import make_kty_sdf


class SmoothKtyFlowCycle(KtyFlowCycle):
    """Deterministic cycle with smoother KTY motion and visible vibration."""

    KTY_NAME = "kty_flow_container"
    PRODUCT_UPDATE_HZ = 8.0
    VIBRATION_DURATION_S = 5.0
    VIBRATION_FREQUENCY_HZ = 5.0
    VIBRATION_AMPLITUDE_M = 0.0020
    VIBRATION_UPDATE_HZ = 30.0

    @staticmethod
    def _smootherstep(ratio: float) -> float:
        """Fifth-order easing with zero velocity and acceleration at both ends."""
        ratio = min(1.0, max(0.0, ratio))
        return 6.0 * ratio**5 - 15.0 * ratio**4 + 10.0 * ratio**3

    def _run_cycle(self) -> None:
        self._cycle_id += 1
        self._spawned_products = 0
        self._inside_products = 0
        self._removed_models = 0

        self._publish_state("WAIT_SERVICES", "waiting for Gazebo UserCommands")
        self._wait_for_services(timeout_s=20.0)
        self._cleanup_stale_names()

        kty_name = self.KTY_NAME
        product_names = [
            f"kty_flow_product_{index:02d}"
            for index in range(1, self.product_count + 1)
        ]

        kty_start = Pose(
            x=self.kty_spawn_x,
            y=0.0,
            z=self.support_top_z,
        )
        self._publish_state("SPAWN_KTY", "creating empty dynamic KTY")
        if not self._create_model(
            kty_name,
            make_kty_sdf(kty_name),
            kty_start,
        ):
            raise RuntimeError("Gazebo rejected KTY create request")
        self._known_models.add(kty_name)

        self._publish_state(
            "APPROACH",
            "moving empty KTY with a jerk-limited pose profile",
            pose_update_hz=self.pose_update_hz,
        )
        self._move_pose_group(
            {kty_name: kty_start},
            distance_x=self.kty_active_x - self.kty_spawn_x,
            duration_s=self.approach_duration,
        )
        self._interruptible_sleep(self.empty_settle)

        self._publish_state("LOAD", "spawning products on the chute")
        for index, (name, profile) in enumerate(
            zip(product_names, PRODUCT_PROFILES, strict=False),
            start=1,
        ):
            yaw_half = 0.5 * profile.yaw
            pose = Pose(
                x=self.product_spawn_x,
                y=profile.spawn_y,
                z=self.product_spawn_z + 0.012 * ((index - 1) % 2),
                qz=math.sin(yaw_half),
                qw=math.cos(yaw_half),
            )
            if not self._create_model(
                name,
                make_flow_product_sdf(name, profile),
                pose,
            ):
                raise RuntimeError(f"Gazebo rejected create request for {name}")
            self._known_models.add(name)
            self._spawned_products = index
            self._publish_state(
                "LOAD",
                f"spawned product {index}/{self.product_count}",
                active_model=name,
            )
            self._interruptible_sleep(self.spawn_interval)

        self._publish_state(
            "SETTLE",
            "waiting for all products to slide and fall inside the KTY",
        )
        captured_poses = self._wait_until_loaded(
            kty_name,
            product_names,
            timeout_s=self.loaded_settle_timeout,
        )

        self._publish_state(
            "VIBRATE",
            "vertically vibrating loaded KTY before outfeed",
            vibration_duration_s=self.VIBRATION_DURATION_S,
            vibration_frequency_hz=self.VIBRATION_FREQUENCY_HZ,
            vibration_amplitude_mm=self.VIBRATION_AMPLITUDE_M * 1000.0,
        )
        self._vibrate_kty(kty_name, captured_poses[kty_name])

        # Read the real post-vibration poses.  This preserves the actual product
        # arrangement for the deterministic group transport to the outfeed.
        captured_poses = self._wait_until_loaded(
            kty_name,
            product_names,
            timeout_s=max(3.0, self.loaded_settle_timeout),
        )

        self._publish_state(
            "OUTFEED",
            "moving loaded KTY and captured products to outfeed",
            captured_models=len(captured_poses),
            kty_update_hz=self.pose_update_hz,
            product_update_hz=self.PRODUCT_UPDATE_HZ,
        )
        kty_pose = captured_poses[kty_name]
        self._move_pose_group(
            captured_poses,
            distance_x=self.kty_outfeed_x - kty_pose.x,
            duration_s=self.outfeed_duration,
        )

        self._publish_state(
            "OUTFEED_HOLD",
            "loaded KTY reached outfeed; holding for visual inspection",
        )
        self._interruptible_sleep(self.outfeed_hold)

        self._publish_state("DESPAWN", "removing products and KTY")
        remove_names = [*product_names, kty_name]
        removed = self._remove_models(remove_names, strict=True)
        self._removed_models = removed
        self._known_models.difference_update(remove_names)

        self._completed_cycles += 1
        self._publish_state(
            "COMPLETE",
            "cycle completed and all dynamic models removed",
            spawned_total=self._spawned_products,
            inside_total=self._inside_products,
            removed_total=self._removed_models,
        )

    def _move_pose_group(
        self,
        base_poses: dict[str, Pose],
        distance_x: float,
        duration_s: float,
    ) -> None:
        """Move the KTY smoothly and correct product poses at a lower rate.

        The KTY is updated at the full configured rate.  Products are allowed to
        follow through contact physics between periodic pose corrections.  This
        avoids launching seven Gazebo service processes on every 50 ms frame.
        """
        if duration_s <= 0.0:
            raise ValueError("motion duration must be positive")

        update_hz = max(10.0, float(self.pose_update_hz))
        period = 1.0 / update_hz
        steps = max(1, math.ceil(duration_s * update_hz))
        product_stride = max(1, round(update_hz / self.PRODUCT_UPDATE_HZ))
        started = time.monotonic()

        for step in range(1, steps + 1):
            self._check_interrupt()
            ratio = step / steps
            eased = self._smootherstep(ratio)

            targets: dict[str, Pose] = {}
            for name, pose in base_poses.items():
                update_now = (
                    name == self.KTY_NAME
                    or step == steps
                    or step % product_stride == 0
                )
                if update_now:
                    targets[name] = pose.translated_x(distance_x * eased)

            futures = {
                name: self._service_pool.submit(self._set_pose, name, pose)
                for name, pose in targets.items()
            }
            failures = [
                name
                for name, future in futures.items()
                if not future.result(
                    timeout=self.service_timeout_ms / 1000.0 + 3.0
                )
            ]
            if failures:
                raise RuntimeError(
                    f"set_pose failed for models: {', '.join(failures)}"
                )

            target_time = started + step * duration_s / steps
            remaining = target_time - time.monotonic()
            if remaining > 0.0:
                self._interruptible_sleep(min(period, remaining))

    def _vibrate_kty(self, name: str, base_pose: Pose) -> None:
        """Apply a visible, softly enveloped vertical vibration to the KTY."""
        steps = max(
            1,
            math.ceil(self.VIBRATION_DURATION_S * self.VIBRATION_UPDATE_HZ),
        )
        started = time.monotonic()

        for step in range(1, steps + 1):
            self._check_interrupt()
            elapsed = step / self.VIBRATION_UPDATE_HZ
            progress = min(1.0, elapsed / self.VIBRATION_DURATION_S)
            envelope = math.sin(math.pi * progress) ** 2
            offset = (
                self.VIBRATION_AMPLITUDE_M
                * envelope
                * math.sin(2.0 * math.pi * self.VIBRATION_FREQUENCY_HZ * elapsed)
            )
            target = Pose(
                x=base_pose.x,
                y=base_pose.y,
                z=base_pose.z + offset,
                qx=base_pose.qx,
                qy=base_pose.qy,
                qz=base_pose.qz,
                qw=base_pose.qw,
            )
            if not self._set_pose(name, target):
                raise RuntimeError("set_pose failed during KTY vibration")

            target_time = started + step / self.VIBRATION_UPDATE_HZ
            remaining = target_time - time.monotonic()
            if remaining > 0.0:
                self._interruptible_sleep(remaining)

        if not self._set_pose(name, base_pose):
            raise RuntimeError("failed to return KTY to neutral vibration pose")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SmoothKtyFlowCycle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
