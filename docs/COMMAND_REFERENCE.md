# Справочник команд

## Подготовка окружения

```bash
cd ~/singulator_digital_twin
chmod +x scripts/*.sh tools/*.py
./scripts/setup_dependencies.sh
```

## Полная сборка

```bash
./scripts/build.sh
source install/setup.bash
```

## Чистая сборка затронутых пакетов

```bash
rm -rf build/<package> install/<package>
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select <package>
source install/setup.bash
```

## Основная рабочая демонстрация

```bash
./scripts/stop_roller_demo.sh
./scripts/run_roller_demo.sh
```

Параметризованный запуск:

```bash
INFEED_SPEED_MPS=2.0 \
TARGET_RATE_BOXES_PER_SEC=2.0 \
SEED=42 \
./scripts/run_roller_demo.sh
```

## Ручной launch

```bash
ros2 launch singulator_bringup matrix_stream_roller.launch.py \
  start_perception:=true \
  start_spawner:=true \
  start_demo_controller:=false \
  start_singulation_controller:=false \
  start_cleanup:=true \
  infeed_speed_mps:=2.0 \
  outfeed_speed_mps:=0.0 \
  target_rate_boxes_per_sec:=2.0 \
  seed:=42
```

`start_singulation_controller:=false` здесь не означает отсутствие алгоритма: roller-launch запускает рабочий контроллер самостоятельно. Не включать второй экземпляр.

## Диагностика ROS

```bash
ros2 node list
ros2 topic list
ros2 topic info /singulator/matrix/command --verbose
ros2 topic hz /singulator/boxes
ros2 topic hz /singulator/matrix/command
ros2 topic echo /singulator/boxes --once
ros2 topic echo /singulator/matrix/command --once
```

## Диагностика зрения

```bash
./scripts/check_vision.sh
ros2 run rqt_image_view rqt_image_view /singulator/perception/debug_image
```

## Диагностика алгоритма

```bash
./scripts/check_singulation.sh
./scripts/check_positive_flow.sh
```

Ожидается:

```text
Publisher count: 1
Node name: singulation_controller
```

И ни одного отрицательного значения в `target_speed_mps`.

## Диагностика роликов

```bash
./scripts/check_roller_upgrade.sh
ros2 topic echo /singulator/throat/left/cmd_vel --once
ros2 topic echo /singulator/throat/right/cmd_vel --once
gz topic -l | grep /singulator/throat/
```

## Камера наблюдателя

```bash
./scripts/view_throat.sh
```

## Полная остановка процессов

```bash
./scripts/stop_roller_demo.sh
pkill -f singulation_controller 2>/dev/null || true
pkill -f vision_stream_node 2>/dev/null || true
pkill -f image_bridge 2>/dev/null || true
pkill -f 'gz sim' 2>/dev/null || true
ros2 daemon stop
ros2 daemon start
```

## Проверка перед коммитом

```bash
python3 tools/validate_project.py
git diff --check
git status --short
```

## Публикация текущего состояния

```bash
./scripts/publish_handoff.sh
```

Либо вручную:

```bash
git add -A
git commit -m "Integrate vision, forward-only singulation and roller throat"
git push -u origin "$(git branch --show-current)"
```
