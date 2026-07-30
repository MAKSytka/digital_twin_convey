# Полезные команды для симуляции КТЯ

## 1. Обновление рабочей ветки

```bash
cd ~/singulator_digital_twin

git fetch origin
git switch fix/kty-mechatronics-runtime-v7
git pull --ff-only origin fix/kty-mechatronics-runtime-v7

git rev-parse --short HEAD
git status --short
```

## 2. Остановка старого runtime

```bash
cd ~/singulator_digital_twin

bash ./scripts/stop_kty_mechatronics.sh 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f kty_station_sim 2>/dev/null || true
pkill -f ros_gz_bridge 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
```

## 3. Чистая сборка

Не подключать старый `install/setup.bash` перед сборкой.

```bash
cd ~/singulator_digital_twin

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

bash ./scripts/build_kty_perception_3d.sh
```

После успешной сборки:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

Проверка executable:

```bash
ros2 pkg executables kty_station_sim \
  | grep -E 'mechatronics_cycle_v18|vision_dashboard_3d|depth_perception_3d_v2'
```

Проверка Gazebo-плагина:

```bash
plugin_prefix="$(ros2 pkg prefix kty_conveyor_surface)"

test -f "$plugin_prefix/lib/libKtyConveyorSurfaceSystem.so" \
  && echo "OK: contact-surface plugin installed"
```

## 4. Запуск симуляции

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash

bash ./scripts/run_kty_perception_3d.sh
```

Запуск с окном dashboard через launch:

```bash
bash ./scripts/run_kty_perception_3d.sh show_dashboard:=true
```

## 5. Отдельное окно машинного зрения

Основная симуляция уже должна работать.

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run kty_station_sim vision_dashboard_3d \
  --ros-args \
  -r __node:=kty_vision_dashboard_window \
  -p show_window:=true \
  -p refresh_hz:=3.0
```

## 6. Проверка четырёх непрерывных циклов

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash

chmod +x scripts/check_kty_runtime_v18.sh
bash ./scripts/check_kty_runtime_v18.sh
```

Успешный результат:

```text
KTY runtime-v18 four-cycle continuity: OK
```

## 7. Проверка текущего состояния

```bash
ros2 topic echo /kty/flow/state
```

Одно сообщение:

```bash
ros2 topic echo /kty/flow/state --once
```

Ключевые поля:

```text
runtime_profile
cycle_id
state
active_kty
queue_kty
gate_registry_present
position_recovery_respawns
position_recovery_failures
product_size_max_m
```

## 8. Проверка заполнения

```bash
ros2 topic echo /kty/fill/state
```

Ключевые значения:

```text
fill_ratio
maximum_height_m
occupied_floor_ratio
camera_ok
valid_depth_fraction
```

Принятые пороги:

```text
fill_ratio_threshold   = 0.82
max_height_threshold_m = 0.340
camera_to_bottom_m     = 1.10
```

## 9. Проверка RGB-D и perception

```bash
ros2 topic hz /kty/vision/image
ros2 topic hz /kty/vision/depth_image
ros2 topic hz /kty/perception/contours
```

Получить один результат сегментации:

```bash
ros2 topic echo /kty/perception/contours --once
```

Проверить сохранённые полигоны:

```bash
ls -lh ~/.ros/kty_vision/
python3 -m json.tool ~/.ros/kty_vision/polygons_latest.json | less
```

## 10. Проверка реестра Gazebo

```bash
ros2 topic echo /kty/mech/model_pose_registry_json --once
```

Проверка частоты:

```bash
ros2 topic hz /kty/mech/model_pose_registry_json
```

В реестре должны присутствовать имена вида:

```text
kty_mech_container_0001
kty_mech_container_0002
kty_mech_product_000001
kty_mech_runtime_locator
kty_mech_chute_gate
```

Створка присутствует только в закрытых состояниях.

## 11. Проверка команд транспортных зон

```bash
ros2 topic echo /kty/mech/infeed_surface/cmd_vel
ros2 topic echo /kty/mech/active_surface/cmd_vel
ros2 topic echo /kty/mech/outfeed_surface/cmd_vel
```

Ожидаемая рабочая скорость:

```text
0.80 м/с
```

В `LOAD` и `COMPACT` active/outfeed обычно равны `0.0`.

## 12. Проверка вибрации

```bash
bash ./scripts/check_kty_vibration.sh
```

Команда joint:

```bash
ros2 topic echo /kty/mech/vibration/cmd_pos
```

Принятый сильный режим:

```text
6,5–9,0 Гц
±8 мм
15 секунд
```

## 13. Статические валидаторы

```bash
python3 tools/validate_kty_classical_3d.py
python3 tools/validate_kty_contact_surface.py
python3 tools/validate_kty_runtime_v17.py
python3 tools/validate_kty_runtime_v18.py
```

## 14. Проверка RTF

Предпочтительно использовать панель `World stats` в Gazebo GUI.

Нормальная оценка:

```text
real_time_factor
```

Dashboard лучше открывать после проверки механики, поскольку отдельное окно увеличивает нагрузку.

## 15. Перезапуск автомата без закрытия Gazebo

```bash
ros2 service call /kty/mech/restart std_srvs/srv/Trigger '{}'
```

## 16. Диагностический снимок перед отчётом об ошибке

```bash
{
  echo '=== git ==='
  git branch --show-current
  git rev-parse --short HEAD
  git status --short

  echo '=== nodes ==='
  ros2 node list

  echo '=== flow ==='
  ros2 topic echo /kty/flow/state --once

  echo '=== fill ==='
  ros2 topic echo /kty/fill/state --once

  echo '=== registry ==='
  ros2 topic echo /kty/mech/model_pose_registry_json --once
} | tee /tmp/kty_diagnostic_snapshot.txt
```

## 17. Подготовка к публикации

```bash
git status
git log --oneline --decorate -10
```

После успешного четырёхциклового теста сохранить:

```text
commit SHA
результат check_kty_runtime_v18.sh
RTF
position_recovery_respawns
position_recovery_failures
скриншот dashboard
```
