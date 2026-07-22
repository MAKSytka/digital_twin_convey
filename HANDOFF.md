# Передача проекта цифрового двойника сингулятора

Этот документ — стартовая точка для второго программиста. Он описывает текущую архитектуру, порядок запуска, ключевые ROS-топики, физические и геометрические параметры, правила настройки алгоритма и публикации изменений.

## Текущее состояние

Реализованы:

- матрица конвейеров `14×4` — 56 независимо управляемых ячеек;
- RGB-камера Gazebo и потоковая обработка кадров;
- публикация наблюдений коробок в `/singulator/boxes`;
- замкнутый алгоритм сингуляризации;
- forward-only управление: алгоритм не выдаёт отрицательные скорости;
- роликовое горлышко, сужающее поток с `760` до `600 мм`;
- входной конвейер и ролики со скоростью `2 м/с`;
- физический диапазон приводов матрицы `-3…+3 м/с`;
- `mu=0.8`, `mu2=0.8`;
- переходная пластина между матрицей и роликами;
- скрипты запуска, остановки и диагностики.

## Основная архитектура

```text
Gazebo RGB camera
  -> /singulator/camera/image_raw
  -> vision_stream_node
  -> /singulator/boxes
  -> singulation_controller
  -> /singulator/matrix/command
  -> matrix_command_fanout
  -> 56 топиков /singulator/cell/rXX_cYY/cmd_vel
```

Роликовое горлышко управляется отдельно:

```text
roller_throat_controller
  -> /singulator/throat/left/cmd_vel
  -> /singulator/throat/right/cmd_vel
```

## Быстрый запуск

### Первый запуск после клонирования

```bash
cd ~/singulator_digital_twin
./scripts/setup_dependencies.sh
./scripts/build.sh
source install/setup.bash
./scripts/run_roller_demo.sh
```

### Обычный повторный запуск

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./scripts/run_roller_demo.sh
```

Параметры сценария передаются через переменные окружения:

```bash
INFEED_SPEED_MPS=2.0 \
TARGET_RATE_BOXES_PER_SEC=2.0 \
SEED=42 \
./scripts/run_roller_demo.sh
```

Не включать `uniform_matrix_controller`: рабочий roller-launch самостоятельно запускает `singulation_controller`.

### Остановка

```bash
./scripts/stop_roller_demo.sh
```

## Интерфейс машинного зрения

Поток данных:

```text
/singulator/camera/image_raw
  -> vision_stream_node
  -> /singulator/boxes
  -> singulation_controller
```

Проверка:

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

В `BoxObservationArray` контроллер использует `id`, координаты центра, длину, ширину, yaw и confidence. Скорость товара оценивается по последовательным кадрам.

## Смена позиции камеры Gazebo

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

## Обязательные проверки

```bash
./scripts/check_vision.sh
./scripts/check_singulation.sh
./scripts/check_roller_upgrade.sh
./scripts/check_positive_flow.sh
```

Критически важно:

- `/singulator/matrix/command` должен иметь ровно одного издателя;
- издатель должен называться `singulation_controller`;
- `uniform_matrix_controller` одновременно с рабочим алгоритмом запускать нельзя;
- все команды скоростей в рабочем режиме должны быть строго положительными;
- минимальная командная скорость должна быть не ниже `minimum_speed_mps`;
- `max_lag` желательно удерживать ниже `maximum_longitudinal_lag_m`.

## Документация

- [`docs/COMMAND_REFERENCE.md`](docs/COMMAND_REFERENCE.md) — все команды и сценарии запуска;
- [`docs/PARAMETER_REFERENCE.md`](docs/PARAMETER_REFERENCE.md) — физические, геометрические и алгоритмические параметры;
- [`docs/TUNING_GUIDE.md`](docs/TUNING_GUIDE.md) — порядок настройки алгоритма;
- [`docs/CLUSTER_SINGULATION.md`](docs/CLUSTER_SINGULATION.md) — проект следующей версии кластерного алгоритма;
- [`docs/CHANGE_HISTORY_CURRENT.md`](docs/CHANGE_HISTORY_CURRENT.md) — что было добавлено по этапам.

## Текущее ограничение алгоритма

В сложных волнах около 40% товаров могут достигать выхода без достаточного продольного зазора. Причина — глобальный порядок пересчитывается в основном по `X`, а для поперечной волны не сохраняется устойчивый порядок разделения.

Запланирован кластерный режим:

1. объединение товаров в кластер при близких координатах `X`;
2. однократная нумерация внутри кластера по убыванию `Y`;
3. сохранение рангов до полного разделения;
4. назначение каждому рангу целевой продольной позиции и положительной скорости;
5. отдельная защита расстояния между соседними кластерами.

## Правило совместной разработки

1. Не запускать два контроллера матрицы одновременно.
2. Перед изменением создать ветку `feature/...` или `fix/...`.
3. После изменения Python-кода выполнить `python3 -m py_compile` или полную сборку.
4. После изменения SDF/launch выполнить чистую сборку затронутых пакетов.
5. Перед коммитом выполнить:

```bash
python3 tools/validate_project.py
git diff --check
git status
```

6. В описании коммита или PR указывать:
   - что изменено;
   - какие параметры изменены;
   - как запускалось;
   - какие проверки выполнены;
   - известные ограничения.
