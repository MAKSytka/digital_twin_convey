# Digital Twin Convey

Цифровой двойник узлов автоматизированной сортировочной линии на базе **ROS 2 Jazzy** и **Gazebo Harmonic**.

## Что находится в репозитории

Проект содержит три самостоятельных контура:

1. матрицу сингуляризации `14 × 4` с 56 независимо управляемыми ячейками;
2. полноширинный инфид-сепаратор мелких и узких товаров;
3. станцию операций с КТЯ: непрерывную подачу тары, загрузку, RGB-D контроль, виброуплотнение, отвод и удаление заполненного КТЯ.

## Поддерживаемая среда

- Ubuntu 24.04 LTS;
- ROS 2 Jazzy;
- Gazebo Harmonic / Gazebo Sim 8;
- Python 3.12;
- `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`;
- OpenCV и `cv_bridge`.

## Быстрый старт стабильной симуляции КТЯ

```bash
cd ~/singulator_digital_twin

git fetch origin
git switch fix/kty-mechatronics-runtime-v7
git pull --ff-only origin fix/kty-mechatronics-runtime-v7

unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

bash ./scripts/build_kty_perception_3d.sh
source install/setup.bash
bash ./scripts/run_kty_perception_3d.sh
```

Текущий рабочий контроллер — `kty_mechatronics_v18`.

### Принятые параметры КТЯ

| Параметр | Значение |
|---|---:|
| Внутренний размер КТЯ | 600 × 400 × 400 мм |
| Размеры товаров | 35 × 15 × 10 … 280 × 190 × 145 мм |
| Порог расчётного заполнения | 82% |
| Порог максимальной высоты | 340 мм |
| Расстояние RGB-D камеры до дна КТЯ | 1,10 м |
| Слабая вибрация при загрузке | 5 Гц, ±1,8 мм |
| Уплотнение | 6,5–9 Гц, ±8 мм, 15 с |
| Скорость контактных зон | 0,80 м/с |
| Интервал спавна при открытом лотке | 1,90 с |

### Рабочий цикл

```text
LOAD
→ CLOSE_GATE
→ COMPACT
→ EJECT_ACTIVE
→ DESPAWN_ACTIVE
→ POSITION_NEXT
→ VERIFY_READY
→ OPEN_GATE
→ LOAD
```

Ожидающий КТЯ начинает движение во время отвода заполненного. Старый КТЯ и находившиеся в нём товары удаляются до окончательного позиционирования следующей тары.

### Проверка непрерывности

Во втором терминале:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash

chmod +x scripts/check_kty_runtime_v18.sh
bash ./scripts/check_kty_runtime_v18.sh
```

Критерий успешного теста — четыре разных цикла `LOAD` без состояния `ERROR`.

### Интерфейс машинного зрения

Основная симуляция должна быть уже запущена.

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

Сохраняемые результаты:

```text
~/.ros/kty_vision/polygons_latest.json
~/.ros/kty_vision/polygons.jsonl
```

Основные топики:

```text
/kty/flow/state
/kty/fill/state
/kty/perception/contours
/kty/vision/dashboard
/kty/mech/model_pose_registry_json
```

## Матрица сингуляризации

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/run_roller_demo.sh
```

Основные параметры:

- 14 продольных рядов;
- 4 поперечные колонки;
- 56 управляемых зон;
- активная поверхность ячейки `360 × 175 мм`;
- зазоры `20 мм`;
- габарит матрицы `5,30 × 0,76 м`;
- продольное трение `mu=0,8`;
- поперечное трение `mu2=0,2`.

## Инфид-сепаратор

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py
```

Сепаратор имеет рабочую ширину `2,5 м` и отделяет товары с минимальной опорной проекцией меньше `70 мм`.

## Структура пакетов

| Пакет | Назначение |
|---|---|
| `singulator_interfaces` | ROS-сообщения проекта |
| `singulator_description` | Геометрия линии |
| `singulator_gazebo` | Миры матрицы и сепаратора |
| `singulator_bringup` | Launch-файлы и bridge-конфигурации |
| `singulator_sim` | Сценарии и генераторы потока |
| `singulator_control` | Контроллеры линии |
| `singulator_perception` | Машинное зрение матрицы |
| `kty_conveyor_surface` | Gazebo-плагин контактных транспортных зон |
| `kty_station_sim` | Автомат КТЯ, RGB-D, уплотнение и dashboard |

## Документация

- [Команды запуска и тестирования КТЯ](docs/KTY_RUNTIME_COMMANDS.md)
- [Передача стабильного runtime v18](docs/KTY_RUNTIME_V18_HANDOFF.md)
- [Архитектура проекта](docs/ARCHITECTURE.md)
- [ROS-интерфейсы](docs/INTERFACES.md)
- [Инфид-сепаратор](docs/INFEED_SIZE_SEPARATOR.md)
- [Интеграция машинного зрения](docs/VISION_INTERFACE.md)
- [Диагностика](docs/TROUBLESHOOTING.md)

## Статические проверки

```bash
python3 tools/validate_project.py
python3 tools/validate_separator_demo.py
python3 tools/validate_kty_runtime_v18.py
```

Полная сборка рабочего КТЯ-контура запускает все необходимые валидаторы автоматически:

```bash
bash ./scripts/build_kty_perception_3d.sh
```

## Статус

Runtime v18 подтверждён локальным многократным запуском: КТЯ загружается, виброуплотняется, покидает активную зону, удаляется, а следующая тара занимает его место. Перед слиянием PR требуется повторить четырёхцикловый тест после изменения параметров заполнения и RGB-D камеры.
