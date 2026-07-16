#!/usr/bin/env python3

from __future__ import annotations

from concurrent.futures import Future, ThreadPoolExecutor
import json
import random
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node

from singulator_sim.box_model import BoxSpec


class SingulationRowSpawner(Node):
    """
    Спавнер поперечных рядов для демонстрации сингуляризации.

    Особенности:
    - работает по времени симуляции;
    - создаёт одну поперечную волну за один вызов;
    - использует Gazebo create_multiple;
    - формирует ряды из 1–4 коробок;
    - добавляет небольшой индивидуальный разброс X;
    - не запускает следующую операцию спавна,
      пока предыдущий запрос ещё обрабатывается.
    """

    BOX_COUNTS = (4, 3, 2, 1)

    BOX_COUNT_WEIGHTS = (
        0.40,  # четыре коробки
        0.30,  # три коробки
        0.15,  # две коробки
        0.15,  # одна коробка
    )

    EXPECTED_BOXES_PER_WAVE = (
        4 * 0.40
        + 3 * 0.30
        + 2 * 0.15
        + 1 * 0.15
    )

    ADJACENT_PAIRS = (
        (0, 1),
        (1, 2),
        (2, 3),
    )

    def __init__(self) -> None:
        super().__init__("singulation_row_spawner")

        self.declare_parameter(
            "world_name",
            "matrix_14x4_stream",
        )

        self.declare_parameter(
            "spawn_x",
            -3.60,
        )

        self.declare_parameter(
            "x_jitter_m",
            0.080,
        )

        self.declare_parameter(
            "belt_top_z",
            0.080,
        )

        self.declare_parameter(
            "infeed_speed_mps",
            2.0,
        )

        self.declare_parameter(
            "target_rate_boxes_per_sec",
            4.0,
        )

        self.declare_parameter(
            "maximum_box_length_m",
            0.400,
        )

        self.declare_parameter(
            "safety_gap_m",
            0.100,
        )

        self.declare_parameter(
            "seed",
            42,
        )

        self.declare_parameter(
            "service_timeout_ms",
            5000,
        )

        self.world_name = str(
            self.get_parameter("world_name").value
        )

        self.spawn_x = float(
            self.get_parameter("spawn_x").value
        )

        self.x_jitter_m = float(
            self.get_parameter("x_jitter_m").value
        )

        self.belt_top_z = float(
            self.get_parameter("belt_top_z").value
        )

        self.infeed_speed_mps = float(
            self.get_parameter("infeed_speed_mps").value
        )

        self.target_rate_boxes_per_sec = float(
            self.get_parameter(
                "target_rate_boxes_per_sec"
            ).value
        )

        self.maximum_box_length_m = float(
            self.get_parameter(
                "maximum_box_length_m"
            ).value
        )

        self.safety_gap_m = float(
            self.get_parameter("safety_gap_m").value
        )

        self.service_timeout_ms = int(
            self.get_parameter(
                "service_timeout_ms"
            ).value
        )

        seed = int(
            self.get_parameter("seed").value
        )

        self._validate_parameters()

        self.rng = random.Random(seed)

        # Требуемый период для заданной средней интенсивности.
        requested_period = (
            self.EXPECTED_BOXES_PER_WAVE
            / self.target_rate_boxes_per_sec
        )

        # Минимальное время для освобождения области спавна.
        clearance_distance = (
            self.maximum_box_length_m
            + 2.0 * self.x_jitter_m
            + self.safety_gap_m
        )

        minimum_clearance_period = (
            clearance_distance
            / self.infeed_speed_mps
        )

        # Не допускаем период, при котором новая волна
        # теоретически появляется внутри предыдущей.
        self.wave_period_sec = max(
            requested_period,
            minimum_clearance_period,
        )

        self.session_id = int(time.time())
        self.wave_id = 0

        self._state_lock = threading.Lock()
        self._spawn_in_progress = False

        # Не называем поле self.executor:
        # это зарезервированное свойство rclpy.Node.
        self.spawn_pool = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="row_spawn",
        )

        # При use_sim_time=true таймер использует /clock.
        self.timer = self.create_timer(
            self.wave_period_sec,
            self._on_timer,
        )

        effective_rate = (
            self.EXPECTED_BOXES_PER_WAVE
            / self.wave_period_sec
        )

        self.get_logger().info(
            "Спавнер поперечных рядов запущен"
        )

        self.get_logger().info(
            "Вероятности: "
            "4 коробки — 40%; "
            "3 — 30%; "
            "2 — 15%; "
            "1 — 15%"
        )

        self.get_logger().info(
            f"Период волн: "
            f"{self.wave_period_sec:.4f} с "
            "симуляционного времени"
        )

        self.get_logger().info(
            f"Ожидаемая интенсивность: "
            f"{effective_rate:.3f} коробки/с"
        )

        self.get_logger().info(
            f"Скорость входного конвейера: "
            f"{self.infeed_speed_mps:.3f} м/с"
        )

        self.get_logger().info(
            f"Разброс X: ±{self.x_jitter_m:.3f} м"
        )

    def _validate_parameters(self) -> None:
        if self.infeed_speed_mps <= 0.0:
            raise ValueError(
                "infeed_speed_mps должен быть больше нуля"
            )

        if self.target_rate_boxes_per_sec <= 0.0:
            raise ValueError(
                "target_rate_boxes_per_sec "
                "должен быть больше нуля"
            )

        if self.x_jitter_m < 0.0:
            raise ValueError(
                "x_jitter_m не может быть отрицательным"
            )

        if self.maximum_box_length_m <= 0.0:
            raise ValueError(
                "maximum_box_length_m "
                "должен быть больше нуля"
            )

        if self.safety_gap_m < 0.0:
            raise ValueError(
                "safety_gap_m не может быть отрицательным"
            )

    def _choose_box_count(self) -> int:
        return self.rng.choices(
            self.BOX_COUNTS,
            weights=self.BOX_COUNT_WEIGHTS,
            k=1,
        )[0]

    def _choose_lanes(
        self,
        box_count: int,
    ) -> list[int]:
        """
        Выбирает поперечные полосы.

        Для двух коробок используется соседняя пара,
        чтобы группа выглядела как плотный ряд.
        """

        if box_count == 4:
            return [0, 1, 2, 3]

        if box_count == 3:
            lanes = self.rng.sample(
                range(4),
                k=3,
            )

            return sorted(lanes)

        if box_count == 2:
            return list(
                self.rng.choice(
                    self.ADJACENT_PAIRS
                )
            )

        return [
            self.rng.randrange(4)
        ]

    def _make_wave_boxes(
        self,
        box_count: int,
    ) -> tuple[str, list[tuple[int, BoxSpec]]]:
        """
        Создаёт одну поперечную волну.

        Возвращает:
        - имя выбранного паттерна;
        - список пар (условная позиция, коробка).
        """

        if box_count == 4:
            boxes = [
                (
                    lane,
                    BoxSpec.random_grouped(
                        self.rng,
                        lane,
                    ),
                )
                for lane in (0, 1, 2, 3)
            ]

            return "four_standard", boxes

        if box_count == 3:
            lanes = self._choose_lanes(3)

            boxes = [
                (
                    lane,
                    BoxSpec.random_grouped(
                        self.rng,
                        lane,
                    ),
                )
                for lane in lanes
            ]

            return "three_standard", boxes

        if box_count == 2:
            subpattern = self.rng.choices(
                (
                    "asymmetric_pair",
                    "large_small",
                ),
                weights=(
                    0.50,
                    0.50,
                ),
                k=1,
            )[0]

            if subpattern == "asymmetric_pair":
                # Выбираются два различных профиля.
                # Поэтому габариты коробок пары
                # заведомо отличаются.
                left_profile, right_profile = (
                    self.rng.sample(
                        (
                            "long",
                            "compact",
                            "wide",
                        ),
                        k=2,
                    )
                )

                boxes = [
                    (
                        0,
                        BoxSpec.random_pair_variant(
                            self.rng,
                            side=-1,
                            profile=left_profile,
                        ),
                    ),
                    (
                        3,
                        BoxSpec.random_pair_variant(
                            self.rng,
                            side=1,
                            profile=right_profile,
                        ),
                    ),
                ]

                pattern_name = (
                    "two_asymmetric_"
                    f"{left_profile}_"
                    f"{right_profile}"
                )

                return pattern_name, boxes

            # Паттерн: одна крупная и одна маленькая.
            large_side = self.rng.choice((-1, 1))
            small_side = -large_side

            large_profile = self.rng.choice(
                (
                    "long",
                    "wide",
                    "skewed",
                )
            )

            large_position = (
                0 if large_side < 0 else 3
            )

            small_position = (
                0 if small_side < 0 else 3
            )

            boxes = [
                (
                    large_position,
                    BoxSpec.random_large_for_mixed_pair(
                        self.rng,
                        side=large_side,
                        profile=large_profile,
                    ),
                ),
                (
                    small_position,
                    BoxSpec.random_small_for_mixed_pair(
                        self.rng,
                        side=small_side,
                    ),
                ),
            ]

            pattern_name = (
                "large_small_"
                f"{large_profile}_"
                f"large_side_{large_side}"
            )

            return pattern_name, boxes

        if box_count == 1:
            profile = self.rng.choice(
                (
                    "long_diagonal",
                    "wide_oblique",
                    "medium_skewed",
                )
            )

            boxes = [
                (
                    1,
                    BoxSpec.random_single_large_variant(
                        self.rng,
                        profile=profile,
                    ),
                )
            ]

            return f"one_large_{profile}", boxes

        raise ValueError(
            f"Недопустимое число коробок: {box_count}"
        )

    def _next_model_name(
        self,
        wave_id: int,
        lane: int,
    ) -> str:
        return (
            f"box_{self.session_id}_"
            f"w{wave_id:06d}_"
            f"l{lane}"
        )

    def _on_timer(self) -> None:
        with self._state_lock:
            if self._spawn_in_progress:
                self.get_logger().warning(
                    "Предыдущая волна ещё создаётся. "
                    "Текущая волна пропущена, "
                    "чтобы не накапливать запросы."
                )
                return

            self._spawn_in_progress = True

        box_count = self._choose_box_count()

        pattern_name, wave_boxes = (
            self._make_wave_boxes(
                box_count
            )
        )

        positions = [
            position
            for position, _ in wave_boxes
        ]

        current_wave_id = self.wave_id
        self.wave_id += 1

        row_items = []

        # Небольшой общий сдвиг всей поперечной волны.
        row_center_x = (
            self.spawn_x
            + self.rng.uniform(-0.03, 0.03)
        )

        for position, box in wave_boxes:
            lane = position

            # У каждой коробки немного отличается X.
            box_x = (
                row_center_x
                + self.rng.uniform(
                    -self.x_jitter_m,
                    self.x_jitter_m,
                )
            )

            model_name = self._next_model_name(
                wave_id=current_wave_id,
                lane=lane,
            )

            color = (
                self.rng.uniform(0.25, 0.90),
                self.rng.uniform(0.20, 0.75),
                self.rng.uniform(0.10, 0.55),
            )

            row_items.append(
                (
                    model_name,
                    box,
                    box_x,
                    color,
                )
            )

        x_values = [
            item[2]
            for item in row_items
        ]

        self.get_logger().info(
            f"Волна {current_wave_id:06d}: "
            f"{box_count} коробок; "
            f"паттерн={pattern_name}; "
            f"позиции={positions}; "
            f"X=[{min(x_values):.3f}, "
            f"{max(x_values):.3f}]"
        )

        future = self.spawn_pool.submit(
            self._spawn_wave,
            current_wave_id,
            row_items,
        )

        future.add_done_callback(
            self._on_spawn_finished
        )

    def _build_factory_request(
        self,
        row_items,
    ) -> str:
        """
        Формирует protobuf-текст gz.msgs.EntityFactory_V.
        """

        request_parts = []

        for (
            model_name,
            box,
            box_x,
            color,
        ) in row_items:
            sdf = box.to_sdf(
                model_name=model_name,
                color=color,
            )

            spawn_z = (
                self.belt_top_z
                + box.size_z / 2.0
                + 0.008
            )

            # json.dumps корректно экранирует
            # кавычки и переводы строк SDF.
            escaped_sdf = json.dumps(sdf)

            request_parts.append(
                "\n".join(
                    (
                        "data {",
                        f"  sdf: {escaped_sdf}",
                        f'  name: "{model_name}"',
                        "  allow_renaming: false",
                        "  pose {",
                        "    position {",
                        f"      x: {box_x:.9f}",
                        f"      y: {box.y:.9f}",
                        f"      z: {spawn_z:.9f}",
                        "    }",
                        "  }",
                        "}",
                    )
                )
            )

        return "\n".join(request_parts)

    def _spawn_wave(
        self,
        wave_id: int,
        row_items,
    ) -> bool:
        request = self._build_factory_request(
            row_items
        )

        service_name = (
            f"/world/{self.world_name}/"
            "create_multiple"
        )

        command = [
            "gz",
            "service",
            "-s",
            service_name,
            "--reqtype",
            "gz.msgs.EntityFactory_V",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(self.service_timeout_ms),
            "--req",
            request,
        ]

        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=(
                    self.service_timeout_ms
                    / 1000.0
                    + 2.0
                ),
                check=False,
            )

        except subprocess.TimeoutExpired:
            self.get_logger().error(
                f"Тайм-аут создания волны "
                f"{wave_id:06d}"
            )
            return False

        except OSError as error:
            self.get_logger().error(
                f"Не удалось выполнить gz service: "
                f"{error}"
            )
            return False

        success = (
            result.returncode == 0
            and "data: true"
            in result.stdout.lower()
        )

        if success:
            self.get_logger().info(
                f"Волна {wave_id:06d} "
                "передана в Gazebo"
            )
            return True

        self.get_logger().error(
            f"Не удалось создать волну "
            f"{wave_id:06d}"
        )

        if result.stdout.strip():
            self.get_logger().error(
                f"stdout: {result.stdout.strip()}"
            )

        if result.stderr.strip():
            self.get_logger().error(
                f"stderr: {result.stderr.strip()}"
            )

        return False

    def _on_spawn_finished(
        self,
        future: Future,
    ) -> None:
        try:
            future.result()

        except Exception as error:
            self.get_logger().error(
                f"Необработанная ошибка спавна: "
                f"{error}"
            )

        finally:
            with self._state_lock:
                self._spawn_in_progress = False

    def close(self) -> None:
        self.spawn_pool.shutdown(
            wait=True,
            cancel_futures=False,
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = SingulationRowSpawner()

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
