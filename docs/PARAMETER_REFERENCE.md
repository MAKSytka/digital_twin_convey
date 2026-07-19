# Справочник параметров

## 1. Геометрия матрицы

| Параметр | Значение | Где менять |
|---|---:|---|
| Продольные ряды | 14 | генератор мира / launch |
| Поперечные колонки | 4 | генератор мира / launch |
| Активная длина ячейки | 0.360 м | `CELL_X` |
| Активная ширина ячейки | 0.175 м | `CELL_Y` |
| Продольный зазор | 0.020 м | `GAP_X` |
| Поперечный зазор | 0.020 м | `GAP_Y` |
| Шаг по X | 0.380 м | `CELL_X + GAP_X` |
| Шаг по Y | 0.195 м | `CELL_Y + GAP_Y` |
| Высота поверхности лент | 0.080 м | `BELT_TOP_Z` |
| Общая длина матрицы | 5.30 м | вычисляется |
| Общая ширина матрицы | 0.76 м | вычисляется |

Основные файлы:

```text
src/singulator_gazebo/scripts/generate_matrix_14x4_stream_v2.py
src/singulator_gazebo/worlds/matrix_14x4_stream_v2.sdf
src/singulator_description/models/roller_throat/model.sdf
```

После изменения генератора мира нужно повторно сгенерировать SDF и пересобрать пакет.

## 2. Физика лент

| Параметр | Текущее значение |
|---|---:|
| `mu` | 0.8 |
| `mu2` | 0.8 |
| Минимальная физическая скорость | -3.0 м/с |
| Максимальная физическая скорость | +3.0 м/с |
| Минимальное ускорение | -3.0 м/с² |
| Максимальное ускорение | +3.0 м/с² |
| Ограничение рывка | ±12 м/с³ |
| Максимальный возраст команды | 2.0 с |

Физический диапазон допускает реверс для испытаний, но рабочий алгоритм должен оставаться forward-only.

## 3. Входной конвейер и ролики

| Параметр | Текущее значение |
|---|---:|
| Скорость входного конвейера | 2.0 м/с |
| Скорость левого банка роликов | 2.0 м/с |
| Скорость правого банка роликов | 2.0 м/с |
| Длина горлышка | 0.80 м |
| Входная ширина | 0.760 м |
| Выходная ширина | 0.600 м |
| Ось левого банка | +100° к +X, эквивалент -80° |
| Ось правого банка | +80° к +X |
| Направление контактной скорости | ±10° к +X, внутрь к центру |
| Верх рабочей поверхности | 0.080 м |

Точки настройки:

```text
scripts/run_roller_demo.sh
src/singulator_bringup/launch/matrix_stream_roller.launch.py
src/singulator_control/singulator_control/roller_throat_controller.py
src/singulator_description/models/roller_throat/model.sdf
```

## 4. Параметры алгоритма сингуляризации

| Параметр | Значение | Назначение |
|---|---:|---|
| `base_speed_mps` | 2.00 | скорость свободных ячеек |
| `minimum_speed_mps` | 0.35 | минимальная положительная скорость |
| `maximum_speed_mps` | 3.00 | верхняя граница команды |
| `leader_speed_mps` | 2.80 | скорость лидера очереди |
| `target_gap_m` | 0.18 | целевой чистый зазор |
| `hard_gap_m` | 0.035 | критический зазор |
| `gap_gain` | 2.20 | усиление ошибки зазора |
| `relative_velocity_gain` | 0.45 | учёт скорости сближения |
| `leader_boost_gain` | 1.20 | ускорение лидера при конфликте |
| `nominal_transport_speed_mps` | 2.00 | эталонная скорость траектории |
| `maximum_longitudinal_lag_m` | 0.30 | максимальное допустимое отставание |
| `lag_guard_horizon_s` | 0.20 | горизонт ограничения дополнительного отставания |
| `lag_recovery_gain` | 2.00 | возврат коробки к эталонной траектории |
| `prediction_horizon_s` | 0.18 | прогноз положения товара |
| `longitudinal_control_margin_m` | 0.16 | упреждающее расширение зоны управления |
| `yaw_gain` | 1.35 | усиление коррекции угла |
| `publish_rate_hz` | 20–50 Гц | частота команд |

Основной файл:

```text
src/singulator_control/singulator_control/singulation_controller.py
```

Предпочтительно менять параметры через launch, а не редактировать алгоритм. Так проще сравнивать сценарии и сохранять воспроизводимость.

## 5. Параметры машинного зрения

| Параметр | Значение |
|---|---:|
| `field_length_m` | 7.70 |
| `field_width_m` | 0.90 |
| `field_min_x_m` | -3.95 |
| `field_max_y_m` | 0.45 |
| `belt_top_z_m` | 0.08 |
| `default_box_height_m` | 0.12 |
| `calibration_frames` | 15 |
| `processing_stride` | 1 |
| `background_threshold` | 25 |
| `track_max_distance_m` | 0.45 |
| `track_max_misses` | 5 |

Основные файлы:

```text
src/singulator_perception/singulator_perception/vision_stream_node.py
src/singulator_perception/singulator_perception/detector_core.py
```

Внутренние пороги детектора:

| Параметр | Значение |
|---|---:|
| `field_min_area_ratio` | 0.05 |
| `box_min_area_ratio` | 0.00015 |
| `box_max_area_ratio` | 0.50 |
| `min_good_matches` | 8 |
| морфологическое ядро | 5×5 |

## 6. Параметры потока коробок

Основные launch-параметры:

```text
target_rate_boxes_per_sec
seed
x_jitter_m
safety_gap_m
maximum_box_length_m
```

Для первого теста алгоритма:

```text
target_rate_boxes_per_sec = 2.0
seed = 42
```

После устойчивой работы увеличивать интенсивность до 3 и 4 коробок/с.
