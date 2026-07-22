# Singulator Digital Twin

Цифровой двойник зоны сингуляризации товарного потока на базе **ROS 2 Jazzy** и **Gazebo Harmonic**.

Главная модель содержит:

- матрицу `14 × 4` — 56 независимо управляемых конвейерных ячеек;
- входной конвейер и роликовое горлышко `760 → 600 мм`;
- генератор волн из 1–4 коробок;
- удаление коробок после выхода;
- потоковое машинное зрение;
- замкнутый forward-only алгоритм управления матрицей;
- ROS-интерфейсы для зрения, управления и диагностики.

## Поддерживаемая среда

- Ubuntu 24.04 LTS;
- ROS 2 Jazzy;
- Gazebo Harmonic / Gazebo Sim 8;
- Python 3.12;
- пакеты `ros_gz_sim` и `ros_gz_bridge`.

## Геометрия текущей симуляции

| Параметр | Значение |
|---|---:|
| Число продольных рядов | 14 |
| Число поперечных колонок | 4 |
| Активная поверхность ячейки | 360 × 175 мм |
| Продольный зазор | 20 мм |
| Поперечный зазор | 20 мм |
| Шаг центров по X | 380 мм |
| Шаг центров по Y | 195 мм |
| Общий габарит матрицы | 5,30 × 0,76 м |
| Продольное трение `mu` | 0,8 |
| Поперечное трение `mu2` | 0,8 |
| Диапазон привода матрицы | −3…3 м/с |
| Рабочий диапазон алгоритма | 0,35…3 м/с |
| Ограничение ускорения | 3 м/с² |

Направление движения — вдоль глобальной оси `+X`. Строка `r00` находится у входа, строка `r13` — у выхода. Колонки нумеруются от отрицательных координат `Y` к положительным.

## Быстрый запуск

<!-- cluster-docs:quick-start:start -->
### Основной рабочий сценарий

После первой установки зависимостей и сборки:

```bash
cd ~/singulator_digital_twin
./scripts/setup_dependencies.sh
./scripts/build.sh
source install/setup.bash
./scripts/run_roller_demo.sh
```

При последующих запусках достаточно:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./scripts/run_roller_demo.sh
```

Параметризованный запуск:

```bash
INFEED_SPEED_MPS=2.0 \
TARGET_RATE_BOXES_PER_SEC=2.0 \
SEED=42 \
./scripts/run_roller_demo.sh
```

Остановка всех процессов рабочего сценария:

```bash
./scripts/stop_roller_demo.sh
```

### Интерфейс машинного зрения

Проверка ROS-топиков:

```bash
./scripts/check_vision.sh
ros2 topic hz /singulator/camera/image_raw
ros2 topic hz /singulator/boxes
ros2 topic echo /singulator/boxes --once
```

Графический интерфейс отладки зрения:

```bash
ros2 run rqt_image_view rqt_image_view \
  /singulator/perception/debug_image
```

Основной контракт:

```text
/singulator/camera/image_raw       sensor_msgs/msg/Image
/singulator/boxes                  singulator_interfaces/msg/BoxObservationArray
/singulator/perception/debug_image sensor_msgs/msg/Image
```

### Смена позиции камеры Gazebo

Перенос GUI-камеры к выходу матрицы и роликовому горлышку:

```bash
./scripts/view_throat.sh
```

Ручной эквивалент:

```bash
gz service \
  -s /gui/move_to \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  -r 'data: "roller_throat"' \
  --timeout 5000
```
<!-- cluster-docs:quick-start:end -->


### 1. Установка зависимостей

```bash
cd ~/singulator_digital_twin
chmod +x scripts/*.sh tools/*.py
./scripts/setup_dependencies.sh
```

ROS 2 Jazzy должен быть установлен заранее по официальной инструкции для Ubuntu 24.04.

### 2. Сборка

```bash
./scripts/build.sh
source install/setup.bash
```

### 3. Демонстрационный режим

```bash
./scripts/run_demo.sh
```

Запускаются:

- Gazebo с полной линией `matrix_14x4_stream`;
- пять процессов `ros_gz_bridge`;
- `matrix_command_fanout`;
- входной и выходной конвейеры по 2 м/с;
- равномерное движение всех 56 ячеек со скоростью 0,5 м/с;
- генератор коробок со средней интенсивностью 4 коробки/с;
- автоматическое удаление прошедших коробок.

Остановка выполняется `Ctrl+C`. Для принудительной очистки процессов:

```bash
./scripts/stop_demo.sh
```

## Режимы запуска

### Только инфраструктура для интеграции алгоритма

```bash
./scripts/run_integration.sh
```

В этом режиме коробки не создаются, а команды матрицы не генерируются. Второй программист может самостоятельно публиковать `MatrixCommand`.

### Инфраструктура и поток коробок, но без тестового контроллера

```bash
./scripts/run_scenario.sh
```

Это основной режим интеграции алгоритма сингуляризации: симулятор создаёт коробки, а внешний контроллер должен публиковать скорости матрицы.

### Прямой вызов launch-файла

```bash
ros2 launch singulator_bringup matrix_stream.launch.py \
  start_spawner:=true \
  start_demo_controller:=false \
  infeed_speed_mps:=2.0 \
  outfeed_speed_mps:=2.0 \
  target_rate_boxes_per_sec:=4.0 \
  seed:=42
```

Все аргументы launch-файла:

```bash
ros2 launch singulator_bringup matrix_stream.launch.py --show-args
```

## Контракт для алгоритма управления

Алгоритм должен публиковать:

```text
/singulator/matrix/command
singulator_interfaces/msg/MatrixCommand
```

Структура сообщения:

```text
std_msgs/Header header
uint16 rows
uint16 cols
float32[] target_speed_mps
```

Для матрицы 14×4:

```text
rows = 14
cols = 4
len(target_speed_mps) = 56
```

Массив разворачивается **построчно**:

```text
index = row * cols + col
```

Первые и последние элементы:

```text
0  -> r00_c00
1  -> r00_c01
2  -> r00_c02
3  -> r00_c03
4  -> r01_c00
...
55 -> r13_c03
```

Пример публикации находится в [`examples/matrix_command_publisher.py`](examples/matrix_command_publisher.py).

`matrix_command_fanout` проверяет размеры сообщения и раскладывает массив на 56 топиков:

```text
/singulator/cell/rXX_cYY/cmd_vel
std_msgs/msg/Float64
```

Алгоритм не должен напрямую зависеть от внутренних имён Gazebo и транспортных топиков отдельных ячеек.

## Структура пакетов

| Пакет | Назначение |
|---|---|
| `singulator_interfaces` | ROS-сообщения и сервисы — стабильный контракт между модулями |
| `singulator_description` | Геометрические параметры и модели |
| `singulator_gazebo` | Миры Gazebo и генераторы SDF |
| `singulator_bringup` | Launch-файлы и конфигурации ROS–Gazebo bridge |
| `singulator_sim` | Fan-out команд, спавнер коробок и очистка мира |
| `singulator_control` | Тестовые контроллеры и управление входным/выходным конвейерами |
| `singulator_perception` | Заготовка будущего модуля машинного зрения |
| `singulator_metrics` | Заготовка будущего модуля метрик |

## Документация

- [Архитектура](docs/ARCHITECTURE.md)
- [ROS-интерфейсы и система координат](docs/INTERFACES.md)
- [Полная инструкция запуска](docs/RUNBOOK.md)
- [Сценарии появления коробок](docs/SPAWN_SCENARIOS.md)
- [Интеграция алгоритма второго программиста](docs/ALGORITHM_INTEGRATION.md)
- [Интеграция машинного зрения](docs/VISION_INTERFACE.md)
- [Проект кластерного алгоритма сингуляризации](docs/CLUSTER_SINGULATION.md)
- [Диагностика проблем](docs/TROUBLESHOOTING.md)
- [Чек-лист передачи проекта](docs/HANDOFF_CHECKLIST.md)
- [Подготовка GitHub и материалов хакатона](docs/GITHUB_AND_SUBMISSION.md)
- [Аудит исходного архива и внесённые исправления](docs/AUDIT.md)

## Проверка проекта без запуска Gazebo

```bash
python3 tools/validate_project.py
```

Проверяются:

- наличие всех 56 ячеек;
- уникальность имён и управляющих топиков;
- параметры геометрии и трения;
- полнота bridge-конфигураций;
- структура `MatrixCommand`;
- синтаксис Python-файлов;
- отсутствие `build/`, `install/`, `log/` и резервных копий.

## Текущее состояние и ограничения

Работает:

- физическая модель полной матрицы 14×4;
- индивидуальное управление 56 ячейками;
- входной и выходной конвейеры;
- случайные размеры и углы коробок;
- паттерны одиночной коробки, неодинаковой пары и пары «крупная + маленькая»;
- средняя интенсивность около 4 коробок/с симуляционного времени;
- автоматическое удаление коробок после выхода.

Пока не реализовано полностью:

- публикация `/singulator/boxes` из Gazebo или камеры;
- устойчивое сопровождение идентификатора товара между кадрами;
- промышленный алгоритм сингуляризации;
- вычисление фактических скоростей в `MatrixState`;
- автоматические метрики качества сингуляризации.

Сейчас `MatrixState.actual_speed_mps` копирует заданные скорости и не является реальным измерением одометрии. Топики Gazebo `.../odometry` в основном launch-файле не мостятся в ROS, чтобы не создавать лишнюю нагрузку.

<!-- cluster-docs:limitation:start -->
### Ограничение текущего контроллера

В сложных поперечных волнах около 40% товаров могут достигать выхода без достаточного продольного зазора. Следующая версия контроллера должна использовать устойчивые кластеры по координате `X`, фиксировать порядок товаров внутри кластера по убыванию `Y` и назначать каждому рангу отдельную целевую продольную позицию.

Подробный проект: [`docs/CLUSTER_SINGULATION.md`](docs/CLUSTER_SINGULATION.md).
<!-- cluster-docs:limitation:end -->

## Публикация на GitHub

Сборочные каталоги и резервные копии не входят в репозиторий. После проверки:

```bash
git init
git branch -M main
git add .
git status
git commit -m "Initial handoff of 14x4 singulator digital twin"
git remote add origin <repository-url>
git push -u origin main

git tag -a v0.1-handoff -m "Stable integration baseline"
git push origin v0.1-handoff
```

Крупные CAD-файлы, видео и исполняемые бинарники размещаются отдельно. Их назначение, версия, контрольная сумма и ссылка фиксируются в [`external_artifacts/README.md`](external_artifacts/README.md).
