# Диагностика проблем

## `AMENT_TRACE_SETUP_FILES: unbound variable`

Причина: ROS 2 setup-скрипты используют переменные, которые могут быть не определены, а Bash-режим `set -u` запрещает обращение к таким переменным. Если ошибка возникает внутри проектного скрипта, временно отключите `nounset` на время `source`:

```bash
set +u
source /opt/ros/jazzy/setup.bash
set -u
```

Папки `build/`, `install/` и `log/` не хранятся в Git и появляются только после успешной команды:

```bash
colcon build --symlink-install
```

Если `install/setup.bash` отсутствует, сначала исправьте ошибку сборки и повторите `./scripts/build.sh`.

## `colcon` находит одинаковые пакеты

Причина: внутри workspace находится резервная копия `src` с пакетами тех же имён.

Проверка:

```bash
find . -name package.xml -print
```

В рабочем репозитории не должно быть `src_before_*`. Резервные версии хранятся в Git, а не внутри workspace.

## `Node not found` или executable не найден

```bash
source /opt/ros/jazzy/setup.bash
source ~/singulator_digital_twin/install/setup.bash
ros2 pkg executables singulator_sim
ros2 pkg executables singulator_control
```

После изменения `setup.py` требуется пересборка соответствующего пакета.

## `invalid message type` для `MatrixCommand`

```bash
source /opt/ros/jazzy/setup.bash
cd ~/singulator_digital_twin
colcon build --symlink-install --packages-select singulator_interfaces
source install/setup.bash
ros2 interface show singulator_interfaces/msg/MatrixCommand
```

Каждый новый терминал должен source-ить workspace.

## Коробки не создаются

Проверить мир и сервис:

```bash
gz service -l | grep matrix_14x4_stream
```

Нужен сервис:

```text
/world/matrix_14x4_stream/create_multiple
```

Проверить, что launch запущен с `start_spawner:=true`, а `/clock` приходит в ROS:

```bash
ros2 topic hz /clock
```

## Коробки стоят на входе

Проверить команды:

```bash
ros2 topic echo /singulator/infeed/cmd_vel --once
ros2 topic echo /singulator/matrix/command --once
```

Для теста:

```bash
python3 examples/matrix_command_publisher.py --mode uniform --speed 2.0
```

TrackController прекращает использовать устаревшую команду через 2 секунды, поэтому контроллер должен публиковать её периодически.

## Последние строки не двигаются

Основной мир намеренно создаёт `r12` и `r13` раньше остальных строк. Не менять порядок моделей в сгенерированном SDF без повторного теста.

Проверить Gazebo-одометрию:

```bash
gz topic -e -t /singulator/cell/r12_c00/odometry
gz topic -e -t /singulator/cell/r13_c03/odometry
```

## ROS-команда есть, а Gazebo её не получает

Проверить bridge-процессы:

```bash
ros2 node list | grep singulator_bridge
```

Должно быть пять bridge-узлов. Проверить крайние топики:

```bash
ros2 topic info /singulator/cell/r00_c00/cmd_vel
ros2 topic info /singulator/cell/r13_c03/cmd_vel
```

## Команды равны 2 м/с, но движение визуально медленнее

Скорости TrackController задаются в метрах за секунду **симуляционного времени**.
Если фактический Real Time Factor Gazebo ниже 1.0, движение будет выглядеть
медленнее относительно обычных часов. Например, при RTF=0.5 команда 2 м/с
остаётся физически равной 2 м/с в модели, но за одну реальную секунду симуляция
пройдёт только половину симуляционной секунды.

Сначала проверьте команды:

```bash
./scripts/check_speeds.sh
```

Затем посмотрите `Real Time Factor` в панели `World stats` Gazebo. Не нужно
компенсировать низкий RTF повышением команды: это исказит физическую модель.

## Real Time Factor падает

Основные источники нагрузки:

- большое число динамических коробок;
- высокая частота контактов;
- 56 TrackController;
- GUI Gazebo;
- частые операции создания и удаления моделей.

Практические меры:

- уменьшить `TARGET_RATE_BOXES_PER_SEC`;
- снизить число одновременно находящихся коробок;
- не мостить 58 odometry-топиков без необходимости;
- закрыть лишние визуальные панели Gazebo;
- использовать очистку прошедших коробок;
- запускать повторяемые тесты с одинаковым seed.

## `ThreadPoolExecutor` конфликтует с `rclpy.Node.executor`

В спавнере пул называется `spawn_pool`. Не переименовывать поле в `self.executor`, потому что у `rclpy.Node` уже есть одноимённое свойство.

## Wi-Fi, видеодрайвер или GUI Gazebo нестабильны

Проверить рендеринг:

```bash
gz sim -v 4 src/singulator_gazebo/worlds/matrix_14x4_stream.sdf
```

Для серверной проверки имеет смысл подготовить отдельный headless launch, но текущий основной launch открывает GUI.
