# Handoff: стабильный runtime станции КТЯ v18

## Назначение

Документ фиксирует состояние рабочей симуляции станции операций с КТЯ перед публикацией изменений в GitHub.

Рабочая ветка:

```text
fix/kty-mechatronics-runtime-v7
```

Рабочий PR:

```text
#47 — fix(kty): continuous multi-cycle transport and medium carton mix
```

## Подтверждённое поведение

Локально подтверждён непрерывный цикл:

```text
LOAD
→ CLOSE_GATE
→ COMPACT
→ EJECT_ACTIVE
→ DESPAWN_ACTIVE
→ POSITION_NEXT
→ VERIFY_READY
→ OPEN_GATE
→ LOAD следующего КТЯ
```

Стабильно работают:

- два КТЯ одновременно: активный и ожидающий;
- параллельный отвод заполненного и предварительная подача следующего;
- плоские контактные транспортные зоны без роликов;
- статическая задвижка лотка с подтверждением через JSON-реестр Gazebo;
- статический позиционирующий упор с подтверждением жизненного цикла;
- удаление заполненного КТЯ и находившихся в нём товаров;
- восстановление пустого ожидающего КТЯ при редком заклинивании;
- RGB-D оценка заполнения;
- классическая 3D-сегментация товаров;
- OBB, полигоны, нормали и кандидаты захвата;
- виброуплотнение;
- dashboard машинного зрения.

## Зафиксированные runtime-параметры

Механические параметры v18 не изменять без отдельного калибровочного теста.

| Параметр | Значение |
|---|---:|
| Скорость транспортных зон | 0,80 м/с |
| Скорость точного позиционирования | 0,22 м/с |
| Слабая вибрация | 5 Гц, ±1,8 мм |
| Сильная вибрация | свип 6,5–9 Гц, ±8 мм |
| Длительность сильной вибрации | 15 с |
| Интервал спавна при открытой задвижке | 1,90 с |
| Интервал спавна при закрытой задвижке | 3,0 с |
| Максимум товаров при закрытой задвижке | 5 |
| Размеры товаров | 35 × 15 × 10 … 280 × 190 × 145 мм |
| Порог заполнения | 82% |
| Порог максимальной высоты | 340 мм |
| Высота камеры над дном КТЯ | 1,10 м |
| Внутренний размер КТЯ | 600 × 400 × 400 мм |

Последняя настройка заполнения сознательно затрагивает только три согласованных величины:

```text
fill_ratio_threshold      = 0.82
max_height_threshold_m    = 0.340
camera_to_bottom_m        = 1.10
```

Та же геометрия `1.10 м` используется узлом 3D-perception.

## Архитектура обратной связи

Gazebo-плагин `kty_conveyor_surface` публикует компактный реестр моделей:

```text
/kty/mech/model_pose_registry_json
```

Формат:

```text
Gazebo gz.msgs.StringMsg
→ ros_gz_bridge
→ ROS std_msgs/msg/String
```

Контроллер использует этот поток для:

- координат КТЯ;
- подтверждения создания и удаления моделей;
- подтверждения состояния задвижки;
- подтверждения состояния позиционирующего упора;
- контроля количества товаров.

Не возвращать частые вызовы `gz topic -e` в рабочий цикл: они ранее приводили к падению `gz-transport-topic` внутри `zmq_poll()`.

## Основные файлы

```text
src/kty_station_sim/kty_station_sim/mechatronics_cycle_v18.py
src/kty_station_sim/kty_station_sim/mechatronics_cycle_v17.py
src/kty_station_sim/kty_station_sim/mechatronics_cycle_v16.py
src/kty_station_sim/kty_station_sim/world_patch_v4.py
src/kty_station_sim/launch/kty_perception_3d.launch.py
src/kty_station_sim/launch/kty_mechatronics_v15.launch.py
src/kty_station_sim/launch/kty_mechatronics_v13.launch.py
src/kty_conveyor_surface/src/KtyConveyorSurfaceSystem.cc
scripts/build_kty_perception_3d.sh
scripts/run_kty_perception_3d.sh
scripts/check_kty_runtime_v18.sh
```

## Первый запуск на новой машине

```bash
cd ~/singulator_digital_twin
chmod +x scripts/*.sh tools/*.py
bash ./scripts/setup_dependencies.sh

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

bash ./scripts/build_kty_perception_3d.sh
source install/setup.bash
bash ./scripts/run_kty_perception_3d.sh
```

## Приёмочный тест

Во втором терминале:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash

chmod +x scripts/check_kty_runtime_v18.sh
bash ./scripts/check_kty_runtime_v18.sh
```

Приёмка успешна, когда:

1. наблюдаются четыре разных цикла `LOAD`;
2. нет состояния `ERROR`;
3. заполненный КТЯ удаляется до окончательного позиционирования следующего;
4. задвижка открывается в каждом новом цикле;
5. товары снова начинают поступать после смены КТЯ;
6. `position_recovery_failures = 0`;
7. допускается `position_recovery_respawns > 0`, если после восстановления цикл продолжается;
8. КТЯ заполняется до новых порогов без переполнения лотка.

## Проверка машинного зрения

```bash
ros2 topic echo /kty/fill/state --once
ros2 topic echo /kty/perception/contours --once
ros2 topic echo /kty/flow/state --once
```

Dashboard:

```bash
ros2 run kty_station_sim vision_dashboard_3d \
  --ros-args \
  -r __node:=kty_vision_dashboard_window \
  -p show_window:=true \
  -p refresh_hz:=3.0
```

## Известные особенности

- предупреждения Qt `Binding loop detected` не влияют на физику;
- сообщения Gazebo `Entity ... not found, so not removed` допустимы при идемпотентной очистке старых статических моделей;
- RTF зависит от открытого dashboard и производительности GPU/CPU;
- при Ctrl+C отдельные старые ROS-узлы могут завершаться позже Gazebo, но рабочие recorder/dashboard уже используют безопасный `try_shutdown`;
- автоматическое восстановление пустого КТЯ является частью нормальной отказоустойчивости v18.

## Подготовка PR к merge

До merge:

```bash
python3 tools/validate_kty_runtime_v18.py
bash ./scripts/build_kty_perception_3d.sh
bash ./scripts/check_kty_runtime_v18.sh
```

Зафиксировать в комментарии PR:

- commit SHA;
- результат четырёхциклового теста;
- число `position_recovery_respawns`;
- отсутствие `ERROR`;
- фактический RTF;
- подтверждение новых порогов `82% / 340 мм / 1,10 м`.

PR пока направлен в stacked-ветку `feat/kty-classical-3d-perception-v6`. Перед публикацией в `main` нужно проверить порядок слияния родительских PR и затем изменить base PR #47 на актуальную целевую ветку.
