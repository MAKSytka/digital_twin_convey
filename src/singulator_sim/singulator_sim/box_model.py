from __future__ import annotations

from dataclasses import dataclass
import math
import random


@dataclass(frozen=True)
class BoxSpec:
    """Описание одной коробки до её создания в Gazebo."""

    size_x: float
    size_y: float
    size_z: float

    mass: float
    y: float
    yaw: float

    lane: int | None = None

    @staticmethod
    def _clamp(
        value: float,
        minimum: float,
        maximum: float,
    ) -> float:
        return max(minimum, min(maximum, value))

    @classmethod
    def random_grouped(
        cls,
        rng: random.Random,
        lane: int,
    ) -> "BoxSpec":
        """
        Коробка для поперечной группы.

        Такие коробки ограничены по ширине и углу,
        чтобы несколько объектов могли появиться
        рядом при одинаковой координате X.
        """

        lane_centers = (
            -0.2925,
            -0.0975,
             0.0975,
             0.2925,
        )

        size_x = rng.uniform(0.080, 0.220)
        size_y = rng.uniform(0.050, 0.105)
        size_z = rng.uniform(0.030, 0.180)

        yaw = rng.uniform(
            math.radians(-10.0),
            math.radians(10.0),
        )

        y = (
            lane_centers[lane]
            + rng.uniform(-0.010, 0.010)
        )

        density = rng.uniform(180.0, 650.0)
        volume = size_x * size_y * size_z

        mass = cls._clamp(
            density * volume,
            0.05,
            5.0,
        )

        return cls(
            size_x=size_x,
            size_y=size_y,
            size_z=size_z,
            mass=mass,
            y=y,
            yaw=yaw,
            lane=lane,
        )

    @classmethod
    def random_single(
        cls,
        rng: random.Random,
        conveyor_width: float = 0.760,
    ) -> "BoxSpec":
        """Одиночная коробка: может быть крупнее и сильнее повёрнута."""

        for _ in range(200):
            size_x = rng.uniform(0.080, 0.380)
            size_y = rng.uniform(0.050, 0.300)
            size_z = rng.uniform(0.030, 0.220)

            yaw = rng.uniform(
                math.radians(-55.0),
                math.radians(55.0),
            )

            projected_half_width = 0.5 * (
                abs(size_x * math.sin(yaw))
                + abs(size_y * math.cos(yaw))
            )

            max_y = (
                conveyor_width / 2.0
                - projected_half_width
                - 0.015
            )

            if max_y <= 0.0:
                continue

            y = rng.uniform(-max_y, max_y)

            density = rng.uniform(180.0, 650.0)
            volume = size_x * size_y * size_z

            mass = cls._clamp(
                density * volume,
                0.05,
                5.0,
            )

            return cls(
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                mass=mass,
                y=y,
                yaw=yaw,
                lane=None,
            )

        raise RuntimeError(
            "Не удалось подобрать коробку, помещающуюся на конвейере"
        )

    @classmethod
    def random_wide_two_cell(
        cls,
        rng: random.Random,
        side: int,
    ) -> "BoxSpec":
        """
        Увеличенная коробка для сценария из двух товаров.

        Каждая коробка перекрывает примерно две
        поперечные ячейки матрицы.

        side:
            -1 — левая часть конвейера;
             1 — правая часть конвейера.
        """

        if side not in (-1, 1):
            raise ValueError(
                "side должен быть равен -1 или 1"
            )

        for _ in range(200):
            # Габариты находятся в пределах ТЗ.
            size_x = rng.uniform(0.250, 0.320)
            size_y = rng.uniform(0.140, 0.190)
            size_z = rng.uniform(0.060, 0.200)

            yaw = rng.uniform(
                math.radians(20.0),
                math.radians(30.0),
            )

            if rng.random() < 0.5:
                yaw = -yaw

            # Поперечная проекция повёрнутой коробки.
            projected_width = (
                abs(size_x * math.sin(yaw))
                + abs(size_y * math.cos(yaw))
            )

            # Шаг между центрами ячеек равен 195 мм.
            # Проекция 230–310 мм явно задевает
            # две соседние поперечные ячейки.
            if not 0.230 <= projected_width <= 0.310:
                continue

            y = (
                side * 0.205
                + rng.uniform(-0.012, 0.012)
            )

            # Проверяем, что коробка остаётся
            # внутри матрицы шириной 0,76 м.
            half_width = projected_width / 2.0

            if abs(y) + half_width > 0.370:
                continue

            density = rng.uniform(160.0, 420.0)
            volume = size_x * size_y * size_z

            mass = cls._clamp(
                density * volume,
                0.05,
                5.0,
            )

            return cls(
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                mass=mass,
                y=y,
                yaw=yaw,
                lane=None,
            )

        raise RuntimeError(
            "Не удалось подобрать широкую "
            "двухъячеечную коробку"
        )

    @classmethod
    def random_wide_three_cell(
        cls,
        rng: random.Random,
    ) -> "BoxSpec":
        """
        Одна крупная коробка для перекрытия
        примерно трёх поперечных ячеек.

        Используются максимальные допустимые
        размеры и диагональная ориентация.
        """

        for _ in range(200):
            # Максимальные размеры остаются
            # в пределах 400×320×280 мм.
            size_x = rng.uniform(0.360, 0.400)
            size_y = rng.uniform(0.250, 0.320)
            size_z = rng.uniform(0.080, 0.220)

            yaw = rng.uniform(
                math.radians(35.0),
                math.radians(50.0),
            )

            if rng.random() < 0.5:
                yaw = -yaw

            projected_width = (
                abs(size_x * math.sin(yaw))
                + abs(size_y * math.cos(yaw))
            )

            # Для покрытия трёх поперечных колонок
            # требуется проекция примерно 0,40–0,52 м.
            if not 0.400 <= projected_width <= 0.520:
                continue

            y = rng.uniform(-0.018, 0.018)

            half_width = projected_width / 2.0

            if abs(y) + half_width > 0.370:
                continue

            density = rng.uniform(140.0, 340.0)
            volume = size_x * size_y * size_z

            mass = cls._clamp(
                density * volume,
                0.05,
                5.0,
            )

            return cls(
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                mass=mass,
                y=y,
                yaw=yaw,
                lane=None,
            )

        raise RuntimeError(
            "Не удалось подобрать широкую "
            "трёхъячеечную коробку"
        )

    @classmethod
    def random_single_large_variant(
        cls,
        rng: random.Random,
        profile: str,
    ) -> "BoxSpec":
        """
        Одна крупная коробка с различными
        габаритами и углом поворота.

        Возможные профили:
        - long_diagonal;
        - wide_oblique;
        - medium_skewed.
        """

        profiles = {
            "long_diagonal": {
                "size_x": (0.340, 0.400),
                "size_y": (0.160, 0.230),
                "size_z": (0.080, 0.220),
                "yaw_deg": (45.0, 68.0),
                "projection": (0.380, 0.540),
            },
            "wide_oblique": {
                "size_x": (0.250, 0.340),
                "size_y": (0.260, 0.320),
                "size_z": (0.080, 0.220),
                "yaw_deg": (18.0, 40.0),
                "projection": (0.340, 0.500),
            },
            "medium_skewed": {
                "size_x": (0.300, 0.380),
                "size_y": (0.200, 0.290),
                "size_z": (0.070, 0.220),
                "yaw_deg": (30.0, 58.0),
                "projection": (0.360, 0.530),
            },
        }

        if profile not in profiles:
            raise ValueError(
                f"Неизвестный профиль большой коробки: {profile}"
            )

        config = profiles[profile]

        for _ in range(500):
            size_x = rng.uniform(*config["size_x"])
            size_y = rng.uniform(*config["size_y"])
            size_z = rng.uniform(*config["size_z"])

            yaw_abs = math.radians(
                rng.uniform(*config["yaw_deg"])
            )

            yaw = (
                yaw_abs
                if rng.random() < 0.5
                else -yaw_abs
            )

            projected_width = (
                abs(size_x * math.sin(yaw))
                + abs(size_y * math.cos(yaw))
            )

            minimum_projection, maximum_projection = (
                config["projection"]
            )

            if not (
                minimum_projection
                <= projected_width
                <= maximum_projection
            ):
                continue

            y = rng.uniform(-0.025, 0.025)

            if (
                abs(y)
                + projected_width / 2.0
                > 0.370
            ):
                continue

            density = rng.uniform(140.0, 360.0)
            volume = size_x * size_y * size_z

            mass = cls._clamp(
                density * volume,
                0.05,
                5.0,
            )

            return cls(
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                mass=mass,
                y=y,
                yaw=yaw,
                lane=None,
            )

        raise RuntimeError(
            "Не удалось сформировать большую коробку "
            f"профиля {profile}"
        )

    @classmethod
    def random_pair_variant(
        cls,
        rng: random.Random,
        side: int,
        profile: str,
    ) -> "BoxSpec":
        """
        Коробка для неодинаковой пары.

        side:
            -1 — левая сторона;
             1 — правая сторона.

        Профили имеют заведомо разные диапазоны
        размеров, поэтому две коробки пары
        визуально отличаются.
        """

        if side not in (-1, 1):
            raise ValueError(
                "side должен быть равен -1 или 1"
            )

        profiles = {
            "long": {
                "size_x": (0.270, 0.360),
                "size_y": (0.100, 0.160),
                "size_z": (0.050, 0.190),
                "yaw_deg": (18.0, 36.0),
                "projection": (0.190, 0.310),
            },
            "compact": {
                "size_x": (0.150, 0.220),
                "size_y": (0.090, 0.145),
                "size_z": (0.040, 0.150),
                "yaw_deg": (4.0, 22.0),
                "projection": (0.105, 0.215),
            },
            "wide": {
                "size_x": (0.200, 0.290),
                "size_y": (0.160, 0.220),
                "size_z": (0.050, 0.190),
                "yaw_deg": (10.0, 29.0),
                "projection": (0.180, 0.300),
            },
        }

        if profile not in profiles:
            raise ValueError(
                f"Неизвестный профиль коробки пары: {profile}"
            )

        config = profiles[profile]

        for _ in range(500):
            size_x = rng.uniform(*config["size_x"])
            size_y = rng.uniform(*config["size_y"])
            size_z = rng.uniform(*config["size_z"])

            yaw_abs = math.radians(
                rng.uniform(*config["yaw_deg"])
            )

            yaw = (
                yaw_abs
                if rng.random() < 0.5
                else -yaw_abs
            )

            projected_width = (
                abs(size_x * math.sin(yaw))
                + abs(size_y * math.cos(yaw))
            )

            minimum_projection, maximum_projection = (
                config["projection"]
            )

            if not (
                minimum_projection
                <= projected_width
                <= maximum_projection
            ):
                continue

            y = (
                side * 0.205
                + rng.uniform(-0.008, 0.008)
            )

            if (
                abs(y)
                + projected_width / 2.0
                > 0.372
            ):
                continue

            density = rng.uniform(160.0, 470.0)
            volume = size_x * size_y * size_z

            mass = cls._clamp(
                density * volume,
                0.05,
                5.0,
            )

            return cls(
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                mass=mass,
                y=y,
                yaw=yaw,
                lane=None,
            )

        raise RuntimeError(
            "Не удалось сформировать коробку пары "
            f"профиля {profile}"
        )

    @classmethod
    def random_large_for_mixed_pair(
        cls,
        rng: random.Random,
        side: int,
        profile: str,
    ) -> "BoxSpec":
        """
        Большая коробка для паттерна
        «большая + маленькая».

        Большой товар занимает одну сторону потока,
        оставляя место маленькому на противоположной.
        """

        if side not in (-1, 1):
            raise ValueError(
                "side должен быть равен -1 или 1"
            )

        profiles = {
            "long": {
                "size_x": (0.310, 0.400),
                "size_y": (0.145, 0.220),
                "size_z": (0.070, 0.220),
                "yaw_deg": (30.0, 56.0),
                "projection": (0.290, 0.430),
            },
            "wide": {
                "size_x": (0.250, 0.340),
                "size_y": (0.230, 0.310),
                "size_z": (0.070, 0.220),
                "yaw_deg": (15.0, 36.0),
                "projection": (0.300, 0.440),
            },
            "skewed": {
                "size_x": (0.280, 0.380),
                "size_y": (0.180, 0.275),
                "size_z": (0.060, 0.220),
                "yaw_deg": (34.0, 61.0),
                "projection": (0.300, 0.450),
            },
        }

        if profile not in profiles:
            raise ValueError(
                f"Неизвестный профиль смешанной пары: {profile}"
            )

        config = profiles[profile]

        for _ in range(500):
            size_x = rng.uniform(*config["size_x"])
            size_y = rng.uniform(*config["size_y"])
            size_z = rng.uniform(*config["size_z"])

            yaw_abs = math.radians(
                rng.uniform(*config["yaw_deg"])
            )

            yaw = (
                yaw_abs
                if rng.random() < 0.5
                else -yaw_abs
            )

            projected_width = (
                abs(size_x * math.sin(yaw))
                + abs(size_y * math.cos(yaw))
            )

            minimum_projection, maximum_projection = (
                config["projection"]
            )

            if not (
                minimum_projection
                <= projected_width
                <= maximum_projection
            ):
                continue

            y = (
                side * 0.110
                + rng.uniform(-0.010, 0.010)
            )

            if (
                abs(y)
                + projected_width / 2.0
                > 0.365
            ):
                continue

            density = rng.uniform(140.0, 370.0)
            volume = size_x * size_y * size_z

            mass = cls._clamp(
                density * volume,
                0.05,
                5.0,
            )

            return cls(
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                mass=mass,
                y=y,
                yaw=yaw,
                lane=None,
            )

        raise RuntimeError(
            "Не удалось сформировать большую коробку "
            "для смешанной пары"
        )

    @classmethod
    def random_small_for_mixed_pair(
        cls,
        rng: random.Random,
        side: int,
    ) -> "BoxSpec":
        """
        Маленькая коробка для паттерна
        «большая + маленькая».
        """

        if side not in (-1, 1):
            raise ValueError(
                "side должен быть равен -1 или 1"
            )

        for _ in range(300):
            size_x = rng.uniform(0.080, 0.160)
            size_y = rng.uniform(0.045, 0.090)
            size_z = rng.uniform(0.025, 0.120)

            yaw = math.radians(
                rng.uniform(-18.0, 18.0)
            )

            projected_width = (
                abs(size_x * math.sin(yaw))
                + abs(size_y * math.cos(yaw))
            )

            y = (
                side * 0.300
                + rng.uniform(-0.008, 0.008)
            )

            if (
                abs(y)
                + projected_width / 2.0
                > 0.372
            ):
                continue

            density = rng.uniform(180.0, 650.0)
            volume = size_x * size_y * size_z

            mass = cls._clamp(
                density * volume,
                0.02,
                2.0,
            )

            return cls(
                size_x=size_x,
                size_y=size_y,
                size_z=size_z,
                mass=mass,
                y=y,
                yaw=yaw,
                lane=None,
            )

        raise RuntimeError(
            "Не удалось сформировать маленькую коробку "
            "для смешанной пары"
        )

    def inertia(self) -> tuple[float, float, float]:
        """Моменты инерции прямоугольного параллелепипеда."""

        ixx = self.mass * (
            self.size_y**2 + self.size_z**2
        ) / 12.0

        iyy = self.mass * (
            self.size_x**2 + self.size_z**2
        ) / 12.0

        izz = self.mass * (
            self.size_x**2 + self.size_y**2
        ) / 12.0

        return ixx, iyy, izz

    def to_sdf(
        self,
        model_name: str,
        color: tuple[float, float, float],
    ) -> str:
        """Формирует SDF одной коробки."""

        ixx, iyy, izz = self.inertia()
        red, green, blue = color

        return f"""<?xml version="1.0"?>
<sdf version="1.10">
  <model name="{model_name}">

    <!-- Разрешаем физическому движку усыплять покоящуюся коробку -->
    <allow_auto_disable>false</allow_auto_disable>

    <link name="base_link">

      <!-- Поворот геометрии относительно модели -->
      <pose>0 0 0 0 0 {self.yaw:.9f}</pose>

      <inertial>
        <mass>{self.mass:.9f}</mass>

        <inertia>
          <ixx>{ixx:.12f}</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>

          <iyy>{iyy:.12f}</iyy>
          <iyz>0</iyz>

          <izz>{izz:.12f}</izz>
        </inertia>
      </inertial>

      <collision name="box_collision">

        <!-- Ограничиваем число контактных точек -->
        <max_contacts>8</max_contacts>

        <geometry>
          <box>
            <size>
              {self.size_x:.9f}
              {self.size_y:.9f}
              {self.size_z:.9f}
            </size>
          </box>
        </geometry>

        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
        </surface>

      </collision>

      <visual name="box_visual">
        <geometry>
          <box>
            <size>
              {self.size_x:.9f}
              {self.size_y:.9f}
              {self.size_z:.9f}
            </size>
          </box>
        </geometry>

        <material>
          <ambient>
            {red:.3f} {green:.3f} {blue:.3f} 1
          </ambient>

          <diffuse>
            {red:.3f} {green:.3f} {blue:.3f} 1
          </diffuse>
        </material>
      </visual>

    </link>
  </model>
</sdf>
"""
