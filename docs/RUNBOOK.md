# Инструкция установки и запуска

## Чистая установка

```bash
git clone https://github.com/MAKSytka/digital_twin_convey.git ~/singulator_digital_twin
cd ~/singulator_digital_twin
chmod +x scripts/*.sh tools/*.py
./scripts/check_environment.sh
./scripts/setup_dependencies.sh
./scripts/build.sh
source install/setup.bash
```

## Статическая проверка

```bash
python3 tools/validate_project.py
python3 tools/validate_kty_station.py
```

Первая команда проверяет основной контур, вторая — структуру, сообщения, Python-синтаксис и SDF станции КТЯ.

## Матрица сингуляризации

### Рабочий сценарий

```bash
./scripts/run_roller_demo.sh
```

### Сценарий для внешнего контроллера

Терминал 1:

```bash
./scripts/run_scenario.sh
```

Терминал 2:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run <algorithm_package> <algorithm_executable>
```

Алгоритм публикует `/singulator/matrix/command`.

### Диагностика

```bash
./scripts/check_running.sh
ros2 topic hz /singulator/matrix/command
ros2 topic echo /singulator/matrix/state --once
ros2 topic hz /singulator/boxes
```

## Станция операций с КТЯ

### Стандартный запуск

```bash
./scripts/run_kty_station.sh
```

### Запуск с параметрами

```bash
./scripts/run_kty_station.sh \
  vibration_frequency_hz:=25.0 \
  vibration_amplitude_m:=0.001 \
  product_rate_products_per_s:=1.0 \
  seed:=42
```

Допустимые параметры первой версии:

- частота: `20...50 Гц`;
- амплитуда: `(0...0,003] м`, как отклонение от среднего положения;
- поток: положительное значение в товарах в секунду;
- `seed`: целое число для воспроизводимости.

Для стартовой проверки использовать `25 Гц` и `1 мм`. Не начинать runtime-калибровку с сочетания `50 Гц / 3 мм`, поскольку оно задаёт пиковое ускорение порядка `30 g`.

### Наблюдение автомата

```bash
ros2 topic echo /kty/station/state
ros2 topic echo /kty/fault
ros2 topic hz /kty/perception/contours
```

Отладочное изображение:

```bash
ros2 run rqt_image_view rqt_image_view \
  /kty/perception/debug_image
```

Исходные RGB-D топики:

```bash
ros2 topic hz /kty/camera/image
ros2 topic hz /kty/camera/depth_image
ros2 topic echo /kty/camera/camera_info --once
```

Ручной сброс после аварии:

```bash
ros2 service call /kty/station/reset std_srvs/srv/Trigger '{}'
```

Остановка:

```bash
./scripts/stop_kty_station.sh
```

### Метрики

```bash
find /tmp/kty_station_metrics -maxdepth 2 -type f -print
```

Для воспроизводимого эксперимента сохранять:

- параметры launch;
- `seed`;
- `summary.json`;
- `timeseries.csv`;
- `product_displacements.csv`;
- журнал запуска Gazebo и ROS-узлов.

## Пересборка после изменений

Для Python-файлов используется `--symlink-install`. После изменения `setup.py`, launch-файлов, YAML или сообщений выполнить:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

После изменения KTY-сообщений:

```bash
colcon build --symlink-install \
  --packages-select singulator_interfaces kty_station_sim
source install/setup.bash
```

Для отдельной пересборки станции:

```bash
colcon build --symlink-install --packages-select kty_station_sim
```

## Изменение геометрии

Матрица генерируется скриптом:

```text
src/singulator_gazebo/scripts/generate_matrix_14x4_stream.py
```

После изменения:

```bash
python3 src/singulator_gazebo/scripts/generate_matrix_14x4_stream.py
python3 tools/validate_project.py
colcon build --symlink-install --packages-select singulator_gazebo
```

Станция КТЯ пока редактируется непосредственно в:

```text
src/kty_station_sim/worlds/kty_station.sdf
src/kty_station_sim/config/station.yaml
```

После изменения:

```bash
python3 tools/validate_kty_station.py
colcon build --symlink-install --packages-select kty_station_sim
```

## Проверка перед коммитом

```bash
python3 tools/validate_project.py
python3 tools/validate_kty_station.py
git diff --check
git status
```

В `git status` не должны появляться `build/`, `install/`, `log/`, `__pycache__/`, метрики из `/tmp` или резервные копии.
