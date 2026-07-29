# Digital Twin Convey

Цифровой двойник узлов автоматизированной сортировочной линии на базе **ROS 2 Jazzy** и **Gazebo Harmonic**.

Репозиторий содержит три независимо запускаемых контура:

1. матрицу сингуляризации `14 × 4` с 56 независимо управляемыми ячейками;
2. полноширинный инфид-сепаратор для предварительного отделения мелких и узких товаров;
3. станцию операций с картонной тарной ячейкой — подвод КТЯ, загрузку, виброуплотнение, RGB-D контроль заполнения и отвод.

## Поддерживаемая среда

- Ubuntu 24.04 LTS;
- ROS 2 Jazzy;
- Gazebo Harmonic / Gazebo Sim 8;
- Python 3.12;
- `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`;
- OpenCV и `cv_bridge` для контуров машинного зрения.

## Быстрый старт

```bash
cd ~/singulator_digital_twin
chmod +x scripts/*.sh tools/*.py
./scripts/setup_dependencies.sh
./scripts/build.sh
source install/setup.bash
```

### Матрица сингуляризации

Рабочий сценарий с машинным зрением и контроллером:

```bash
./scripts/run_roller_demo.sh
```

Базовый демонстрационный сценарий:

```bash
./scripts/run_demo.sh
```

Сценарий для подключения внешнего алгоритма управления:

```bash
./scripts/run_scenario.sh
```

Алгоритм публикует:

```text
/singulator/matrix/command
singulator_interfaces/msg/MatrixCommand
```

Порядок массива скоростей — построчный:

```python
index = row * cols + col
```

### Инфид-сепаратор

Базовый непрерывный сценарий:

```bash
ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py
```

По умолчанию создаётся поток `4 товара/с`, а вероятность товара, который должен уйти на нижнюю ветвь, равна `0.50`.

Конечный воспроизводимый тест:

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

Пример более медленного потока для визуальной проверки:

```bash
ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py \
  target_rate_boxes_per_sec:=1.0
```

После изменения SDF-моделей сепаратора требуется чистая пересборка затронутых пакетов:

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
```

Статическая проверка узла:

```bash
python3 tools/validate_separator_demo.py
```

Полное описание: [`docs/INFEED_SIZE_SEPARATOR.md`](docs/INFEED_SIZE_SEPARATOR.md).

### Станция операций с КТЯ

```bash
./scripts/run_kty_station.sh
```

Параметризованный запуск:

```bash
./scripts/run_kty_station.sh \
  vibration_frequency_hz:=25.0 \
  vibration_amplitude_m:=0.001 \
  product_rate_products_per_s:=1.0 \
  seed:=42
```

Остановка:

```bash
./scripts/stop_kty_station.sh
```

Полное описание: [`docs/KTY_STATION.md`](docs/KTY_STATION.md).

## Основные параметры матрицы

| Параметр | Значение |
|---|---:|
| Число продольных рядов | 14 |
| Число поперечных колонок | 4 |
| Число управляемых зон | 56 |
| Активная поверхность ячейки | 360 × 175 мм |
| Продольный зазор текущей модели | 20 мм |
| Поперечный зазор текущей модели | 20 мм |
| Габарит матрицы | 5,30 × 0,76 м |
| Продольное трение `mu` | 0,8 |
| Поперечное трение `mu2` | 0,2 |
| Направление движения | глобальная ось `+X` |

## Основные параметры инфид-сепаратора

| Параметр | Значение текущей модели |
|---|---:|
| Рабочая ширина | 2,5 м |
| Длина входной ленты | 3,0 м |
| Длина верхней выходной ленты | 3,0 м |
| Длина нижней выходной ленты | 3,0 м |
| Скорость лент | 2,0 м/с |
| Число поперечных валов | 11 |
| Шаг валов по X | 120 мм |
| Число контактных дисков на валу | 25 |
| Шаг дисков по Y | 100 мм |
| Радиус контактной коллизии диска | 25 мм |
| Толщина контактного диска | 30 мм |
| Чистое отверстие по X и Y | 70 × 70 мм |
| Радиус коллизии поперечного вала | 8 мм |
| Длина поперечного вала | 2,480 м |
| Угловая скорость для поверхности 2 м/с | 80 рад/с |
| Частота вращения | ≈763,9 об/мин |
| Зазор перед выходной лентой | 1 мм |
| Ступень принимающей ленты | 4 мм вниз |
| Демонстрационный входной поток | 4 товара/с |
| Вероятность нижнего класса | 50% |
| Число фиксированных точек спавна | 10 |
| Число профилей коробок | 11 |
| Верхняя координата деспавна | `x = 3,45 м` |
| Нижняя координата деспавна | `x = 3,52 м` |

### Принцип разделения потока

Для основания коробки `L × W`, повёрнутого на угол `yaw`, спавнер вычисляет две опорные проекции:

```text
projection_x = |L cos(yaw)| + |W sin(yaw)|
projection_y = |L sin(yaw)| + |W cos(yaw)|
```

Маршрут назначается по правилу:

```text
min(projection_x, projection_y) < 0.070 м  -> LOWER
иначе                                      -> UPPER
```

Такой критерий отсеивает не только короткий товар, но и длинную узкую упаковку, которая может попасть в технологические зазоры основной матрицы. Верхний класс в демонстрационном генераторе имеет страховую минимальную проекцию не менее `90 мм`.

### Генерируемые профили товаров

Нижний класс:

- `micro_parcel`;
- `long_narrow`;
- `flat_strip`;
- `tall_slender`;
- `near_cutoff`.

Верхний класс:

- `medium_carton`;
- `large_carton`;
- `long_parcel`;
- `flat_panel`;
- `tall_carton`;
- `square_carton`.

Масса рассчитывается по объёму содержимого и площади картонной оболочки, после чего применяется минимальная масса конкретного профиля. Для подавления нереалистичных отскоков используются низкая упругость, затухание линейной и угловой скорости и ограничение скорости коррекции контакта.

### Управление и диагностика инфид-сепаратора

Контроллер публикует команды движущимся поверхностям:

```text
/singulator/separator/infeed/cmd_vel
/singulator/separator/screen/cmd_vel
/singulator/separator/accepted/cmd_vel
/singulator/separator/reject_transfer/cmd_vel
/singulator/separator/reject/cmd_vel
```

Три основных узла сценария:

- `separator_demo_controller` — задаёт скорость лент и угловую скорость валов;
- `separator_demo_spawner` — формирует регулируемый поток из десяти фиксированных поперечных позиций и одиннадцати размерных профилей;
- `separator_demo_cleanup` — определяет фактическую ветвь, ведёт статистику и удаляет коробки на выходах.

В логах очистки контролируются:

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

Нормальный длительный тест должен показывать рост `removed` при значениях `remove_failures=0` и, как правило, `monitor_restarts=0`.

## Основные параметры станции КТЯ

| Параметр | Значение первой версии |
|---|---:|
| Внутренний размер КТЯ | 600 × 400 × 400 мм |
| Толщина стенки | 3 мм |
| Масса пустого КТЯ | 1,6 кг |
| Время подвода | 2 с |
| Время отвода | 1 с |
| Поток товара | 1 ед/с |
| Лоток | 1000 × 600 мм, 32° |
| Высота края лотка над дном КТЯ | 420 мм |
| Базовый коэффициент трения | 0,75 |
| Стартовый режим вибрации | 25 Гц, ±1 мм |
| Порог высоты заполнения | 0,34 м |
| Максимальная расчётная масса | 35 кг |

Геометрия станции упрощена: рольганги и выталкивание представлены управляемыми контактными зонами. КТЯ, товары и элементы станции моделируются жёсткими телами.

## Цикл станции КТЯ

```text
WAIT_EMPTY_KTY
  -> POSITION_KTY
  -> CLAMP
  -> LOAD
  -> VIBRATE
  -> SETTLE
  -> SCAN
       -> VIBRATE, если высота ниже порога
       -> EJECT_PREP, если достигнут порог
  -> EJECT
  -> WAIT_EMPTY_KTY
```

В `FAULT` станция закрывает шторку, останавливает контактные зоны, подачу товара и вибрацию. Сброс:

```bash
ros2 service call /kty/station/reset std_srvs/srv/Trigger '{}'
```

## Машинное зрение станции КТЯ

Виртуальная RGB-D камера расположена над центром КТЯ. После микропаузы узел выделяет видимые верхние области товаров и публикует полигональные контуры с устойчивыми идентификаторами и оценкой доступности четырёх боковых направлений.

```text
/kty/camera/image              sensor_msgs/msg/Image
/kty/camera/depth_image        sensor_msgs/msg/Image
/kty/perception/contours       KtyProductContourArray
/kty/ground_truth/registry     KtyGroundTruthArray
```

Ground truth используется только для оценки качества и метрик, но не поступает на вход детектора.

## Метрики станции КТЯ

Результаты циклов записываются в:

```text
/tmp/kty_station_metrics/cycle_XXXXXX/
```

Формируются:

- `summary.json` — коэффициент заполнения, максимальная высота, пустоты, время успокоения, precision/recall/F1;
- `timeseries.csv` — состояние и динамические показатели;
- `product_displacements.csv` — перемещение каждого товара.

## Структура пакетов

| Пакет | Назначение |
|---|---|
| `singulator_interfaces` | Общие ROS-сообщения матрицы и станции КТЯ |
| `singulator_description` | Геометрия матрицы, корпуса сепаратора и валов |
| `singulator_gazebo` | Миры матрицы и инфид-сепаратора |
| `singulator_bringup` | Launch-файлы и конфигурации ROS–Gazebo bridge |
| `singulator_sim` | Спавнеры, fan-out команд, очистка и сценарии |
| `singulator_control` | Контроллеры конвейеров, валов и матрицы |
| `singulator_perception` | Машинное зрение матрицы |
| `singulator_metrics` | Метрики основного контура |
| `kty_station_sim` | Мир, автомат, RGB-D обработка, безопасность и метрики КТЯ |

## Проверка без запуска Gazebo

```bash
python3 tools/validate_project.py
python3 tools/validate_separator_demo.py
python3 tools/validate_kty_station.py
```

После изменения сообщений необходимо пересобрать интерфейсы и зависимые пакеты:

```bash
colcon build --symlink-install \
  --packages-select \
  singulator_interfaces \
  kty_station_sim \
  singulator_sim \
  singulator_control \
  singulator_perception
```

## Документация

- [Архитектура](docs/ARCHITECTURE.md)
- [ROS-интерфейсы и системы координат](docs/INTERFACES.md)
- [Установка и запуск](docs/RUNBOOK.md)
- [Инфид-сепаратор](docs/INFEED_SIZE_SEPARATOR.md)
- [Станция операций с КТЯ](docs/KTY_STATION.md)
- [Интеграция машинного зрения](docs/VISION_INTERFACE.md)
- [Интеграция алгоритма сингуляризации](docs/ALGORITHM_INTEGRATION.md)
- [Чек-лист передачи проекта](docs/HANDOFF_CHECKLIST.md)
- [Диагностика](docs/TROUBLESHOOTING.md)

## Текущий статус и ограничения

Статически проверены структура проекта, Python-синтаксис и XML/SDF основных модулей. Инфид-сепаратор уже прошёл итерации исправления коллизии поперечных валов, выходных переходов и деспавнера, однако требует дальнейшей runtime-калибровки на целевой установке: необходимо измерять долю ошибочных маршрутов около порога 70 мм, устойчивость переходов, real-time factor при потоке 4 товара/с и чувствительность к трению и массе упаковок.

Первая версия станции КТЯ также требует runtime-проверки и последующей калибровки трения, контактов, режима вибрации и порога заполнения.

Критическое инженерное замечание для КТЯ: при `mu = 0,75` и угле лотка `32°` полностью успокоившийся плоский товар может не продолжить скольжение, поскольку `tan(32°) < 0,75`. Зависание поэтому включено в контролируемые аварии, а коэффициент трения должен быть уточнён экспериментально.
