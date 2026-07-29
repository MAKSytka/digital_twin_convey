# Передача проекта цифрового двойника сортировочной линии

Этот документ — стартовая точка для второго программиста. Он описывает текущую архитектуру, порядок запуска, ключевые ROS-топики, физические и геометрические параметры, правила настройки алгоритмов и публикации изменений.

Проект содержит три независимо запускаемых контура:

1. матрицу сингуляризации и роликовое горлышко;
2. полноширинный инфид-сепаратор мелких и узких товаров;
3. станцию операций с КТЯ.

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
- `mu=0.8`, `mu2=0.8` в рабочем roller-сценарии;
- переходная пластина между матрицей и роликами;
- полноширинный инфид-сепаратор шириной `2,5 м` с верхней и нижней ветвями;
- 11 физических валов с 25 дисками на каждом;
- размерный порог `70 мм` по минимальной опорной проекции;
- регулируемый генератор потока, 10 фиксированных точек появления и 11 профилей товаров;
- контроль фактического маршрута и автоматическое удаление коробок на выходах;
- станция КТЯ с автоматом состояний, виброуплотнением, RGB-D контролем и метриками;
- скрипты запуска, остановки и диагностики основных контуров.

## Архитектура матрицы сингуляризации

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

## Архитектура инфид-сепаратора

```text
infeed_size_separator_demo.launch.py
  -> Gazebo world: infeed_size_separator_demo.sdf
  -> separator_demo_bridge
  -> separator_demo_controller
       -> входная лента
       -> 11 валов роликового экрана
       -> верхняя выходная лента
       -> нижняя приёмная лента
       -> нижняя выходная лента
  -> separator_demo_spawner
       -> /world/infeed_size_separator_demo/create_multiple
  -> separator_demo_cleanup
       -> /world/infeed_size_separator_demo/pose/info
       -> /world/infeed_size_separator_demo/remove
```

Основные команды движения:

```text
/singulator/separator/infeed/cmd_vel
/singulator/separator/screen/cmd_vel
/singulator/separator/accepted/cmd_vel
/singulator/separator/reject_transfer/cmd_vel
/singulator/separator/reject/cmd_vel
```

`separator_demo_controller` публикует линейную скорость лент и рассчитывает угловую скорость валов. `separator_demo_spawner` создаёт товары через Gazebo transport service. `separator_demo_cleanup` читает поток поз, определяет фактическую ветвь, ведёт статистику и вызывает сервис удаления модели.

## Быстрый запуск матрицы

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

Остановка:

```bash
./scripts/stop_roller_demo.sh
```

## Быстрый запуск инфид-сепаратора

### Обычный запуск

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py
```

Значения по умолчанию:

```text
spawn_mode                         = continuous
target_rate_boxes_per_sec          = 4.0
maximum_items                      = 100
small_item_probability             = 0.50
conveyor_speed_mps                 = 2.0
screen_surface_speed_mps           = 2.0
spawn_clearance_m                  = 0.002
box_restitution                    = 0.02
bounce_capture_velocity_mps        = 0.35
linear_velocity_decay              = 0.05
angular_velocity_decay             = 0.30
contact_max_correcting_velocity_mps = 0.05
seed                               = 42
```

В режиме `continuous` значение `maximum_items` игнорируется.

### Конечный тест

```bash
ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py \
  spawn_mode:=finite \
  maximum_items:=200 \
  target_rate_boxes_per_sec:=4.0 \
  small_item_probability:=0.50 \
  conveyor_speed_mps:=2.0 \
  screen_surface_speed_mps:=2.0 \
  seed:=42
```

### Медленный визуальный тест

```bash
ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py \
  target_rate_boxes_per_sec:=1.0
```

### Чистая сборка после изменения SDF

Обычной инкрементальной сборки может быть недостаточно: старые модели могут остаться в `install`.

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash

rm -rf \
  build/singulator_description \
  build/singulator_gazebo \
  build/singulator_bringup \
  build/singulator_sim

rm -rf \
  install/singulator_description \
  install/singulator_gazebo \
  install/singulator_bringup \
  install/singulator_sim

colcon build --symlink-install
source install/setup.bash
python3 tools/validate_separator_demo.py
```

### Позиционирование камеры

Проверить наличие GUI-сервисов:

```bash
gz service -l | grep -E '/gui/(follow|move_to|track)'
```

Переместить камеру к сепаратору:

```bash
gz service -s /gui/move_to \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'data: "infeed_size_separator"'
```

## Технические характеристики инфид-сепаратора

| Параметр | Значение |
|---|---:|
| Рабочая ширина | 2,5 м |
| Входная лента | 3,0 × 2,5 м |
| Верхняя выходная лента | 3,0 × 2,5 м |
| Нижняя выходная лента | 3,0 × 2,5 м |
| Скорость движущихся поверхностей | 2,0 м/с |
| Число валов | 11 |
| Шаг валов по X | 120 мм |
| Дисков на валу | 25 |
| Шаг дисков по Y | 100 мм |
| Контактный радиус диска | 25 мм |
| Контактный диаметр | 50 мм |
| Толщина диска | 30 мм |
| Открытая щель по X | 70 мм |
| Открытая щель по Y | 70 мм |
| Радиус серой поперечной оси | 8 мм |
| Длина оси | 2,480 м |
| Угловая скорость валов | 80 рад/с |
| Частота вращения | ≈763,9 об/мин |
| Зазор на верхнем и нижнем выходах | 1 мм |
| Ступень принимающей поверхности | 4 мм вниз |
| Координата удаления верхней ветви | `x = 3,45 м` |
| Координата удаления нижней ветви | `x = 3,52 м` |
| Максимальное время жизни модели | 30 с |

Угловая скорость рассчитывается по упрощённой контактной коллизии радиусом `25 мм`:

```text
omega = v / r = 2 / 0.025 = 80 rad/s
n = omega * 60 / (2*pi) = 763.9 rpm
```

Визуальный зуб имеет наружный размер `64 мм`, но он не используется как расчётный радиус контакта.

## Правило размерного разделения

Для основания `L × W`, повёрнутого на угол `yaw`, вычисляются:

```text
projection_x = |L cos(yaw)| + |W sin(yaw)|
projection_y = |L sin(yaw)| + |W cos(yaw)|
```

Правило ожидаемого маршрута:

```text
min(projection_x, projection_y) < 0.070 м  -> LOWER
иначе                                      -> UPPER
```

Это правило специально учитывает длинные узкие товары. Коробка длиной `300 мм` и шириной `40 мм` должна попасть на нижнюю ветвь, даже если её продольная проекция перекрывает несколько валов.

Физическая классификация выполняется самой геометрией. Вычисленный класс используется для генерации тестового набора и последующего сравнения с фактической ветвью.

## Профили коробок инфид-сепаратора

### Ожидаемая нижняя ветвь

- `micro_parcel` — компактный минимальный товар;
- `long_narrow` — длинная коробка с шириной `25–55 мм`;
- `flat_strip` — длинная плоская упаковка;
- `tall_slender` — высокая узкая коробка;
- `near_cutoff` — малая проекция `55–69 мм`.

### Ожидаемая верхняя ветвь

- `medium_carton`;
- `large_carton`;
- `long_parcel`;
- `flat_panel`;
- `tall_carton`;
- `square_carton`.

Центр масс выбирается из десяти фиксированных координат `Y` на ширине 2,5 м. Размеры, высота, масса и yaw случайны внутри профиля. Имя модели содержит ожидаемый маршрут, профиль и индекс позиции:

```text
box_separator_123_n0000001_exp_lower_long_narrow_spot04
box_separator_123_n0000002_exp_upper_large_carton_spot07
```

Не менять формат имени без одновременного обновления регулярного выражения в `separator_demo_cleanup.py` и статического валидатора.

## Масса и контактная динамика

Масса товара оценивается как:

```text
mass = bulk_density * volume
     + cardboard_areal_density * surface_area
```

После расчёта применяется минимальная масса профиля. Это не позволяет мелким товарам становиться почти безынерционными телами.

Для уменьшения отскоков и численных импульсов используются:

```text
spawn_clearance_m                    = 0.002
box_restitution                      = 0.02
bounce_capture_velocity_mps          = 0.35
linear_velocity_decay                = 0.05
angular_velocity_decay               = 0.30
contact_max_correcting_velocity_mps  = 0.05
```

При калибровке сначала менять `box_restitution`, `angular_velocity_decay` и `contact_max_correcting_velocity_mps`. Не добавлять постоянную внешнюю силу к коробкам: движение должно возникать от контакта с лентами и физически вращающимися валами.

## Деспавнер и статистика

`separator_demo_cleanup`:

- подписывается через `gz topic` на `/world/infeed_size_separator_demo/pose/info`;
- подтверждает верхнюю или нижнюю ветвь после `x = 0,85 м`;
- удаляет товар после достижения конца соответствующего выхода;
- удаляет товар при падении ниже допустимой высоты, выходе за боковую границу или превышении времени жизни;
- автоматически перезапускает монитор поз;
- повторяет удаление до трёх раз.

Статистика:

```text
seen
upper
lower
mismatches
removed
active
avg_upper
avg_lower
remove_failures
monitor_restarts
```

Нормальный результат длительного теста:

```text
removed растёт
remove_failures = 0
monitor_restarts = 0 или редкие единичные перезапуски
```

При ненулевом `mismatches` сначала проверить товары `near_cutoff`, фактические щели, угол коробки и контакт с осью. Ошибка около порога не обязательно означает ошибку Python-классификатора: физический исход зависит от ориентации, контактов и динамики.

## Диагностика инфид-сепаратора

Проверить узлы:

```bash
ros2 node list | grep separator
```

Ожидаются:

```text
/separator_demo_bridge
/separator_demo_controller
/separator_demo_spawner
/separator_demo_cleanup
```

Проверить команды:

```bash
ros2 topic echo /singulator/separator/infeed/cmd_vel --once
ros2 topic echo /singulator/separator/screen/cmd_vel --once
ros2 topic echo /singulator/separator/accepted/cmd_vel --once
ros2 topic echo /singulator/separator/reject/cmd_vel --once
```

Для стандартных параметров контроллер должен логировать примерно:

```text
belts=2.000 m/s
disc_contact_radius=25.0 mm
screen_surface=2.000 m/s
shaft=80.000 rad/s (763.9 rpm)
```

Проверить сервисы мира:

```bash
gz service -l | grep '/world/infeed_size_separator_demo'
```

Статическая проверка:

```bash
python3 tools/validate_separator_demo.py
```

Проверка Python-синтаксиса:

```bash
python3 -m py_compile \
  src/singulator_control/singulator_control/separator_demo_controller.py \
  src/singulator_sim/singulator_sim/separator_demo_spawner.py \
  src/singulator_sim/singulator_sim/separator_demo_cleanup.py \
  src/singulator_bringup/launch/infeed_size_separator_demo.launch.py \
  tools/validate_separator_demo.py
```

## Ключевые файлы инфид-сепаратора

| Файл | Назначение |
|---|---|
| `src/singulator_description/models/infeed_size_separator/model.sdf` | Корпус, входная, верхняя и нижняя ленты, ограждения |
| `src/singulator_description/models/separator_star_shaft/model.sdf` | Один физический вал с осью, дисками, joint и контроллером |
| `src/singulator_gazebo/worlds/infeed_size_separator_demo.sdf` | Отдельный мир и расстановка 11 валов |
| `src/singulator_bringup/launch/infeed_size_separator_demo.launch.py` | Полный запуск Gazebo, bridge, controller, spawner и cleanup |
| `src/singulator_bringup/config/bridge_separator_demo.yaml` | ROS–Gazebo bridge для пяти команд скорости |
| `src/singulator_control/singulator_control/separator_demo_controller.py` | Расчёт и публикация скоростей |
| `src/singulator_sim/singulator_sim/separator_demo_spawner.py` | Профили, масса, параметры контакта и регулируемый поток |
| `src/singulator_sim/singulator_sim/separator_demo_cleanup.py` | Маршруты, статистика, safety cleanup и деспавн |
| `tools/validate_separator_demo.py` | Статическая проверка геометрии и конфигурации |
| `docs/INFEED_SIZE_SEPARATOR.md` | Подробное описание узла |

## Интерфейс машинного зрения матрицы

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

## Смена позиции камеры Gazebo для матрицы

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

## Обязательные проверки матрицы

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

## Текущий алгоритм V7

V7 один раз формирует порядок входной волны по убыванию `Y`, добавляет товары в неизменяемую глобальную очередь и создаёт продольные интервалы прямым регулятором соседних зазоров. Raw-ID зрения может перепривязываться к постоянному логическому `uid`; объединённые контуры сопровождаются как отдельные ghost tracks.

Основные проверки:

```bash
python3 tools/test_v7_logic.py
./scripts/check_v7_control.sh
```

Ключевые метрики в `control_v7`: `inversions`, `unresolved_exit`, `merged`, `ghosts`, `uncontrollable`, `allocation_error`.

## Известные ограничения инфид-сепаратора

- Коллизия зубчатого диска упрощена цилиндром радиусом `25 мм`; визуальный профиль зуба не моделируется точной составной коллизией.
- Значение `50%` для нижней ветви является демонстрационной настройкой, а не оценкой реального ассортимента.
- Порог `70 мм` необходимо калибровать сериями прогонов, особенно для `near_cutoff` и повёрнутых коробок.
- При `4 товарах/с` необходимо следить за real-time factor: 11 валов и большое число контактов создают заметную нагрузку.
- Геометрия переходов настроена на зазор `1 мм` и ступень `4 мм вниз`; изменение высоты валов или толщины лент требует повторной проверки переходов.
- Деспавнер зависит от потока поз Gazebo, сервиса `/remove` и соглашения об именах моделей.
- Физические параметры массы, трения и контактного демпфирования являются стартовой калибровкой и требуют сравнения с реальными упаковками.

## Документация

- [`README.md`](README.md) — обзор всех контуров и быстрый старт;
- [`docs/INFEED_SIZE_SEPARATOR.md`](docs/INFEED_SIZE_SEPARATOR.md) — подробная документация инфид-сепаратора;
- [`docs/COMMAND_REFERENCE.md`](docs/COMMAND_REFERENCE.md) — команды и сценарии запуска матрицы;
- [`docs/PARAMETER_REFERENCE.md`](docs/PARAMETER_REFERENCE.md) — физические, геометрические и алгоритмические параметры матрицы;
- [`docs/TUNING_GUIDE.md`](docs/TUNING_GUIDE.md) — порядок настройки алгоритма;
- [`docs/V7_GLOBAL_QUEUE_CONTROL.md`](docs/V7_GLOBAL_QUEUE_CONTROL.md) — текущая логика неизменяемой глобальной очереди;
- [`docs/KTY_STATION.md`](docs/KTY_STATION.md) — станция операций с КТЯ;
- [`docs/CHANGE_HISTORY_CURRENT.md`](docs/CHANGE_HISTORY_CURRENT.md) — что было добавлено по этапам.

## Правило совместной разработки

1. Не запускать два контроллера матрицы одновременно.
2. Перед изменением создать ветку `feature/...`, `fix/...` или `docs/...`.
3. После изменения Python-кода выполнить `python3 -m py_compile` или полную сборку.
4. После изменения SDF/launch выполнить чистую сборку затронутых пакетов.
5. При изменении геометрии инфид-сепаратора обновить `tools/validate_separator_demo.py` и документацию.
6. При изменении формата имени коробки обновить спавнер, деспавнер и валидатор одновременно.
7. Перед коммитом выполнить:

```bash
python3 tools/validate_project.py
python3 tools/validate_separator_demo.py
git diff --check
git status
```

8. В описании коммита или PR указывать:
   - что изменено;
   - какие параметры изменены;
   - как запускалось;
   - какие проверки выполнены;
   - известные ограничения.

## Экспериментальная V7: immutable global queue

Текущий roller-launch использует диапазон `1.00–3.00 м/с`, ускорение `6 м/с²`, неизменяемый глобальный порядок и прямое управление зазорами. Подробности: [`docs/V7_GLOBAL_QUEUE_CONTROL.md`](docs/V7_GLOBAL_QUEUE_CONTROL.md).
