"""Effective vibration profile for the accepted roller-free KTY runtime.

The previous 18 Hz / 3 mm command was theoretically energetic, but the
position-controlled deck received too few useful samples per cycle and did not
reproduce the requested displacement under the loaded KTY.  This runtime keeps
all v9 transport and lifecycle-locator behaviour and replaces only vibration:

* loading: 6 Hz, +/-1.2 mm;
* compaction: frequency-modulated 8..12 Hz, +/-5 mm, 12 s;
* cosine-smoothed 1.5 s ramp in and out.

At full amplitude the strong profile has a theoretical vertical acceleration
range of roughly 1.3..2.9 g, enough to unload products from the KTY bottom and
allow repeated settling contacts without changing horizontal transport.
"""

from __future__ import annotations

import math
import threading
import time

import rclpy
from std_msgs.msg import Float64

from .mechatronics_cycle_v3 import KtyMechatronicsCycleV3


GRAVITY_MPS2 = 9.81


class KtyMechatronicsCycleV10(KtyMechatronicsCycleV3):
    """V9 transport with an effective, measurable compaction vibration sweep."""

    def __init__(self) -> None:
        # The inherited constructor starts the worker thread.  Hold it until all
        # v10 parameters and telemetry fields are ready.
        self._v10_ready = threading.Event()
        super().__init__()

        self.declare_parameter("strong_vibration_sweep_hz", 2.0)
        self.declare_parameter("strong_vibration_modulation_hz", 0.35)
        self.declare_parameter("vibration_settle_s", 1.0)

        self.strong_sweep_hz = float(
            self.get_parameter("strong_vibration_sweep_hz").value
        )
        self.strong_modulation_hz = float(
            self.get_parameter("strong_vibration_modulation_hz").value
        )
        self.vibration_settle_s = float(
            self.get_parameter("vibration_settle_s").value
        )

        if self.strong_frequency <= self.strong_sweep_hz:
            raise ValueError(
                "strong_vibration_frequency_hz must exceed strong_vibration_sweep_hz"
            )
        if self.strong_modulation_hz <= 0.0:
            raise ValueError("strong_vibration_modulation_hz must be positive")

        self._vibration_command_m = 0.0
        self._vibration_frequency_hz = 0.0
        self._vibration_peak_accel_g = 0.0
        self._last_compaction = {
            "fill_before": 0.0,
            "fill_after": 0.0,
            "fill_delta": 0.0,
            "height_before_m": 0.0,
            "height_after_m": 0.0,
            "height_drop_m": 0.0,
        }

        self._v10_ready.set()
        self.get_logger().info(
            "Runtime v10 vibration: weak 6 Hz +/-1.2 mm; "
            "strong 8..12 Hz sweep +/-5 mm for 12 s"
        )

    def _worker_main(self) -> None:
        self._v10_ready.wait()
        super()._worker_main()

    @staticmethod
    def _smooth_ramp(value: float) -> float:
        value = max(0.0, min(1.0, value))
        # Smoothstep gives zero slope at both ends and avoids an impact when the
        # high-amplitude profile starts or stops.
        return value * value * (3.0 - 2.0 * value)

    def _publish_commands(self) -> None:
        with self._lock:
            commands = dict(self._commands)
            mode = self._vibration_mode
            started = self._vibration_started

        elapsed = time.monotonic() - started
        vibration = 0.0
        frequency = 0.0
        envelope = 0.0

        if mode == "weak":
            frequency = self.weak_frequency
            envelope = 1.0
            vibration = self.weak_amplitude * math.sin(
                2.0 * math.pi * frequency * elapsed
            )
        elif mode == "strong":
            ramp = max(0.10, self.strong_ramp)
            ramp_up = self._smooth_ramp(elapsed / ramp)
            remaining = max(0.0, self.strong_duration - elapsed)
            ramp_down = self._smooth_ramp(remaining / ramp)
            envelope = min(ramp_up, ramp_down)

            modulation = self.strong_modulation_hz
            sweep = self.strong_sweep_hz
            frequency = self.strong_frequency + sweep * math.sin(
                2.0 * math.pi * modulation * elapsed
            )
            # Integral of f(t)=f0+sweep*sin(2*pi*fm*t).  Using the integrated
            # phase avoids discontinuities that would occur if frequency*t were
            # changed directly on every callback.
            phase = (
                2.0 * math.pi * self.strong_frequency * elapsed
                + (sweep / modulation)
                * (1.0 - math.cos(2.0 * math.pi * modulation * elapsed))
            )
            vibration = self.strong_amplitude * envelope * math.sin(phase)

        peak_accel_g = (
            abs(self.strong_amplitude if mode == "strong" else self.weak_amplitude)
            * (2.0 * math.pi * frequency) ** 2
            * envelope
            / GRAVITY_MPS2
            if frequency > 0.0
            else 0.0
        )
        with self._lock:
            self._vibration_command_m = vibration
            self._vibration_frequency_hz = frequency
            self._vibration_peak_accel_g = peak_accel_g

        for key, value in commands.items():
            message = Float64()
            message.data = value
            self.command_publishers[key].publish(message)
        vibration_message = Float64()
        vibration_message.data = vibration
        self.command_publishers["vibration"].publish(vibration_message)

    def _close_gate_and_compact(self) -> None:
        self._transition(
            "CLOSE_GATE",
            "closing static slide gate before high-energy compaction",
        )
        self._set_commands(gate=0.0)
        self._interruptible_sleep(0.45)

        with self._lock:
            before = dict(self._latest_fill)

        low_hz = self.strong_frequency - self.strong_sweep_hz
        high_hz = self.strong_frequency + self.strong_sweep_hz
        self._transition(
            "COMPACT",
            (
                f"vertical sweep {low_hz:.1f}-{high_hz:.1f} Hz, "
                f"+/-{1000.0 * self.strong_amplitude:.1f} mm, "
                f"{self.strong_duration:.1f} s"
            ),
        )
        self._set_vibration("strong")
        next_spawn = time.monotonic()
        deadline = time.monotonic() + self.strong_duration
        while time.monotonic() < deadline:
            self._check_interrupt()
            now = time.monotonic()
            if now >= next_spawn:
                # The chute is closed, so these products accumulate above the
                # gate while the current KTY is being compacted.
                self._spawn_product()
                next_spawn = now + self.spawn_interval
            self._interruptible_sleep(0.02)

        self._set_vibration("off")
        self._interruptible_sleep(self.vibration_settle_s)

        with self._lock:
            after = dict(self._latest_fill)
        fill_before = float(before.get("fill_ratio", 0.0) or 0.0)
        fill_after = float(after.get("fill_ratio", 0.0) or 0.0)
        height_before = float(before.get("maximum_height_m", 0.0) or 0.0)
        height_after = float(after.get("maximum_height_m", 0.0) or 0.0)
        self._last_compaction = {
            "fill_before": fill_before,
            "fill_after": fill_after,
            "fill_delta": fill_after - fill_before,
            "height_before_m": height_before,
            "height_after_m": height_after,
            "height_drop_m": height_before - height_after,
        }
        self.get_logger().info(
            "Compaction measurement: "
            f"height {height_before:.3f}->{height_after:.3f} m "
            f"(drop {height_before - height_after:+.3f} m), "
            f"envelope fill {fill_before:.3f}->{fill_after:.3f}"
        )

    def _state_payload(self) -> dict:
        payload = super()._state_payload()
        with self._lock:
            payload.update(
                {
                    "vibration_profile": "vertical_frequency_sweep_v10",
                    "vibration_command_m": self._vibration_command_m,
                    "vibration_frequency_hz": self._vibration_frequency_hz,
                    "vibration_peak_accel_g": self._vibration_peak_accel_g,
                    "strong_vibration_min_hz": (
                        self.strong_frequency - self.strong_sweep_hz
                    ),
                    "strong_vibration_max_hz": (
                        self.strong_frequency + self.strong_sweep_hz
                    ),
                    "strong_vibration_amplitude_m": self.strong_amplitude,
                    "strong_vibration_duration_s": self.strong_duration,
                    "last_compaction": dict(self._last_compaction),
                }
            )
        return payload


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KtyMechatronicsCycleV10()
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
