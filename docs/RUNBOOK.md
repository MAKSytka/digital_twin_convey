# Инструкция установки и запуска

## Чистая установка

```bash
git clone <repository-url> ~/singulator_digital_twin
cd ~/singulator_digital_twin
chmod +x scripts/*.sh tools/*.py
```

Проверить Ubuntu и установленные компоненты:

```bash
./scripts/check_environment.sh
```

Установить зависимости:

```bash
./scripts/setup_dependencies.sh
```

Собрать workspace:

```bash
./scripts/build.sh
source install/setup.bash
```

## Рекомендуемый рабочий процесс второго программиста

### Терминал 1 — симулятор и поток коробок

```bash
cd ~/singulator_digital_twin
./scripts/run_scenario.sh
```

### Терминал 2 — алгоритм управления

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run <algorithm_package> <algorithm_executable>
```

Алгоритм должен публиковать `/singulator/matrix/command`.

### Терминал 3 — диагностика

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./scripts/check_running.sh
```

Дополнительно:

```bash
ros2 topic hz /singulator/matrix/command
ros2 topic echo /singulator/matrix/state --once
```

## Демонстрация без внешнего алгоритма

```bash
./scripts/run_demo.sh
```

В демо одна переменная задаёт одинаковую скорость входа, матрицы и выхода:

```bash
CONVEYOR_SPEED_MPS=2.0 \
TARGET_RATE_BOXES_PER_SEC=4.0 \
SEED=123 \
./scripts/run_demo.sh
```

## Запуск без коробок

```bash
./scripts/run_integration.sh
```

После запуска можно проверить одну команду:

```bash
python3 examples/matrix_command_publisher.py --mode uniform --speed 2.0
```

Остановка:

```bash
python3 examples/matrix_command_publisher.py --mode stop
```

## Пересборка после изменений

Для Python-пакетов используется `--symlink-install`, поэтому изменения `.py` обычно подхватываются без полной пересборки. После изменения `setup.py`, сообщений, launch-файлов или CMake-пакетов выполнить:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Для отдельного пакета:

```bash
colcon build --symlink-install --packages-select singulator_sim
```

После изменения `.msg` сначала пересобрать интерфейсы и зависимые пакеты:

```bash
colcon build --symlink-install \
  --packages-select singulator_interfaces singulator_sim singulator_control
```

## Изменение геометрии мира

Основной генератор:

```text
src/singulator_gazebo/scripts/generate_matrix_14x4_stream.py
```

После изменения констант запустить из корня проекта:

```bash
python3 src/singulator_gazebo/scripts/generate_matrix_14x4_stream.py
python3 tools/validate_project.py
colcon build --symlink-install --packages-select singulator_gazebo
```

Генератор изначально записывает файл в `~/singulator_digital_twin`. Поэтому репозиторий рекомендуется хранить именно по этому пути либо предварительно адаптировать путь в `main()`.

## Проверка перед коммитом

```bash
python3 tools/validate_project.py
git status
git diff --check
```

В `git status` не должны появляться `build/`, `install/`, `log/`, `__pycache__/` и резервные копии.
