# Digital Twin Convey

Цифровой двойник трёх ключевых узлов автоматизированной сортировочной линии на базе **ROS 2 Jazzy**, **Gazebo Harmonic / Gazebo Sim 8** и классического машинного зрения OpenCV.

Репозиторий подготовлен как самостоятельная точка входа для экспертной проверки. Три демонстрации запускаются независимо:

1. полноширинный инфид-сепаратор;
2. матрица сингуляризации 18×4 с роликовым горлышком;
3. станция операций с КТЯ, виброуплотнением и RGB-D контролем.

<p align="center">
  <a href="docs/media/matrix_18x4_overview.png">
    <img src="docs/media/matrix_18x4_overview.png" alt="Матрица сингуляризации 18×4" width="49%">
  </a>
  <a href="docs/media/kty_rgbd_dashboard.png">
    <img src="docs/media/kty_rgbd_dashboard.png" alt="RGB-D dashboard станции КТЯ" width="49%">
  </a>
</p>

## Быстрая навигация

| Что проверить | Команда | Документация |
|---|---|---|
| Матрица сингуляризации | `bash ./scripts/run_roller_demo.sh` | [DEMO_SCENARIOS.md](docs/DEMO_SCENARIOS.md) |
| GUI зрения матрицы | `ros2 run rqt_image_view rqt_image_view /singulator/perception/debug_image` | [Раздел ниже](#gui-машинного-зрения-матрицы) |
| Инфид-сепаратор | `ros2 launch singulator_bringup infeed_size_separator_demo.launch.py` | [INFEED_SIZE_SEPARATOR.md](docs/INFEED_SIZE_SEPARATOR.md) |
| Станция КТЯ | `bash ./scripts/run_kty_perception_3d.sh` | [KTY_RUNTIME_COMMANDS.md](docs/KTY_RUNTIME_COMMANDS.md) |
| Полный статический набор | `bash ./scripts/run_release_checks.sh` | [TROUBLESHOOTING.md](docs/TROUBLESHOOTING.md) |
| Все скриншоты | — | [docs/media](docs/media/README.md) |

## Демонстрационные материалы

Все изображения хранятся непосредственно в репозитории и открываются в полном разрешении 1920×1080:

- [общий вид матрицы 18×4](docs/media/matrix_18x4_overview.png);
- [GUI машинного зрения матрицы](docs/media/singulation_vision_gui.png);
- [работа полноширинного инфид-сепаратора](docs/media/infeed_separator.png);
- [общий вид станции КТЯ](docs/media/kty_station_scene.png);
- [RGB-D dashboard станции КТЯ](docs/media/kty_rgbd_dashboard.png).

Полная галерея с пояснениями: [docs/media/README.md](docs/media/README.md).

## 1. Назначение решения

Целевая система должна обрабатывать суммарный поток до **100 000 товаров/ч**, поддерживать 400 логических направлений сортировки и принимать товары размером от **15×35×10 мм** до **400×320×280 мм**, массой от **10 г** до **5 кг**.

Репозиторий демонстрирует три последовательных этапа:

- предварительный размерный отсев мелких товаров;
- формирование продольных интервалов и уменьшение ошибки ориентации;
- загрузка товаров в КТЯ, оценка заполнения, уплотнение и смена тары.

Целевые 100 000 товаров/ч относятся к системе из нескольких параллельных линий. Одна матрица не заявляется как универсальная линия на весь поток независимо от ассортимента.

## 2. Поддерживаемая среда

- Ubuntu 24.04 LTS;
- ROS 2 Jazzy;
- Gazebo Harmonic / Gazebo Sim 8;
- Python 3.12;
- `colcon`, `rosdep`;
- `ros_gz_sim`, `ros_gz_bridge`, `ros_gz_image`;
- OpenCV, NumPy, PyYAML, `cv_bridge`;
- 4 ГБ ОЗУ минимум, 8 ГБ рекомендуется;
- дискретный GPU рекомендуется для Gazebo GUI.

Типичный RTF сложных сцен находится в диапазоне **0,2–0,8** и зависит от CPU/GPU, открытого GUI и числа динамических моделей.

## 3. Установка и сборка

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-cv-bridge \
  ros-jazzy-rqt-image-view \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-opencv \
  python3-yaml
```

Сборка workspace:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash ./scripts/build.sh
source install/setup.bash
```

Чистая пересборка после изменения SDF, моделей или Gazebo-плагинов:

```bash
rm -rf build install log
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
bash ./scripts/build.sh
source install/setup.bash
```

## 4. Матрица сингуляризации

Запуск:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/run_roller_demo.sh
```

[![Матрица сингуляризации 18×4](docs/media/matrix_18x4_overview.png)](docs/media/matrix_18x4_overview.png)

### Принятая конфигурация

| Параметр | Значение |
|---|---:|
| Матрица | 18×4 |
| Независимые зоны | 72 |
| Активная поверхность ячейки | 360×175 мм |
| Продольный/поперечный зазор | 20/20 мм |
| Базовая скорость | 2,0–2,5 м/с |
| Диапазон команд ячеек | 1,0–3,0 м/с |
| Ограничение ускорения | 6,0 м/с² |
| Продольное трение `mu` | 0,8 |
| Поперечное трение `mu2` | 0,2 |
| Целевой межтоварный зазор | 0,18 м |
| Межволновой зазор | 0,28 м |
| Частота управления | 30 Гц |
| Seed | 42 |

`mu2=0,2` сохраняется как проверенная рабочая калибровка. Значение `2g ≈ 19,6 м/с²` не используется как штатное ограничение: для лёгких и высоких упаковок оно слишком агрессивно. В release-сценарии применяется 6,0 м/с².

### Контур управления

1. Машинное зрение публикует ID, центр, размеры, yaw и оценку движения товара.
2. Контроллер сохраняет глобальный порядок товаров и рассчитывает ошибки зазоров соседних пар.
3. Формируются целевые скорости лидера и следующего товара.
4. Пересечение товара с сеткой преобразуется в веса контактирующих ячеек.
5. Cell-aware allocator распределяет команды, когда одну ячейку делят несколько товаров.
6. Deadline recovery усиливает разделение ближе к выходу.
7. Поперечная разность скоростей создаёт момент для коррекции yaw.
8. Fan-out публикует 72 индивидуальные команды приводов.

Подробно: [SINGULATION_CONTROL.md](docs/SINGULATION_CONTROL.md).

### GUI машинного зрения матрицы

Контур зрения запускается вместе с `run_roller_demo.sh`. Камера публикует исходный кадр в `/singulator/camera/image_raw`, а узел `vision_stream_node` формирует:

- массив наблюдений `/singulator/boxes`;
- отладочное изображение `/singulator/perception/debug_image`;
- контуры, ID, центры и оценку геометрии товаров;
- бинарную маску сегментации в нижней части debug-кадра.

Открой GUI во втором терминале:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 run rqt_image_view rqt_image_view \
  /singulator/perception/debug_image
```

Если поток не выбран автоматически, выбери в выпадающем списке:

```text
/singulator/perception/debug_image
```

Проверка частоты и данных:

```bash
ros2 topic hz /singulator/perception/debug_image
ros2 topic echo /singulator/boxes --once
bash ./scripts/check_vision.sh
```

[![GUI машинного зрения матрицы](docs/media/singulation_vision_gui.png)](docs/media/singulation_vision_gui.png)

Общая проверка управления:

```bash
bash ./scripts/check_v7_control.sh
bash ./scripts/check_vision.sh
```

## 5. Инфид-сепаратор

Запуск:

```bash
ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py \
  seed:=42
```

Конечный приёмочный сценарий:

```bash
ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py \
  spawn_mode:=finite \
  maximum_items:=200 \
  target_rate_boxes_per_sec:=4.0 \
  small_item_probability:=0.70 \
  seed:=42
```

[![Полноширинный инфид-сепаратор](docs/media/infeed_separator.png)](docs/media/infeed_separator.png)

### Принятая конфигурация

| Параметр | Значение |
|---|---:|
| Рабочая ширина | 2,5 м |
| Скорость | 2,0 м/с |
| Число роликов | 11 |
| Конструкция ряда | один сплошной поперечный ролик |
| Длина ролика | 2,480 м |
| Радиус | 25 мм |
| Межосевой шаг | 150 мм |
| Чистый продольный промежуток | 100 мм |
| Нижний класс | `projection_x < 70 мм` |
| Безопасный верхний класс | `projection_x >= 110 мм` |
| Вероятность нижней ветви | 70% |
| Скорость ролика | 80 рад/с, около 763,9 об/мин |
| Гравитация demo-мира | 12,0 м/с² вниз |
| Коэффициент восстановления товаров | 0,00 |
| Линейное/угловое затухание | 0,12 / 0,60 |

Пограничный диапазон 70–110 мм исключён из release-потока. Отверстие 100 мм создаёт запас 30 мм для нижнего класса и 10 мм до безопасного верхнего класса.

Увеличенная гравитация является демонстрационной калибровкой, используемой для подавления многократных неестественных отскоков лёгких товаров.

Статическая проверка:

```bash
python3 tools/validate_separator_demo.py
```

Подробно: [INFEED_SIZE_SEPARATOR.md](docs/INFEED_SIZE_SEPARATOR.md).

## 6. Станция КТЯ — runtime v18

Сборка и запуск:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
bash ./scripts/build_kty_perception_3d.sh
source install/setup.bash
bash ./scripts/run_kty_perception_3d.sh
```

Проверка во втором терминале:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/check_kty_runtime_v18.sh
```

Успешный тест: минимум четыре различных цикла `LOAD` без `ERROR` и с `position_recovery_failures=0`.

[![Станция операций с КТЯ](docs/media/kty_station_scene.png)](docs/media/kty_station_scene.png)

Рабочий цикл:

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

### Принятые параметры КТЯ

| Параметр | Значение |
|---|---:|
| Внутренний размер | 600×400×400 мм |
| Товары runtime v18 | 35×15×10 … 280×190×145 мм |
| Порог заполнения | 82% |
| Порог максимальной высоты | 340 мм |
| Камера до дна КТЯ | 1,10 м |
| Слабая вибрация | 5 Гц, ±1,8 мм |
| Основное уплотнение | 6,5–9 Гц, ±8 мм, 15 с |
| Скорость контактных зон | 0,80 м/с |
| Интервал спавна | 1,90 с |
| Центры спавна по Y | ±0,090 м |
| Допустимая полная поперечная проекция | ±0,170 м |

### Направляющие лотка

На наклонном лотке добавлены две высокие сходящиеся направляющие с физическими коллизиями:

- длина 1,010 м;
- толщина 30 мм;
- высота 240 мм;
- внутренний коридор сужается примерно с 565 до 400 мм;
- коэффициент восстановления равен 0,0.

Направляющие удерживают товары в пределах лотка и сводят поток к внутренней ширине КТЯ. Координата спавна дополнительно ограничивается с учётом yaw-зависимой поперечной проекции коробки.

### RGB-D машинное зрение

Dashboard обычно запускается основным сценарием. Для ручного открытия:

```bash
ros2 run kty_station_sim vision_dashboard_3d \
  --ros-args \
  -r __node:=kty_vision_dashboard_window \
  -p show_window:=true \
  -p refresh_hz:=3.0
```

Основные топики:

```text
/kty/vision/image
/kty/vision/depth_image
/kty/perception/contours
/kty/vision/dashboard
/kty/fill/state
/kty/flow/state
/kty/mech/model_pose_registry_json
```

Результат perception содержит постоянный ID, верхний полигон, OBB, размеры XYZ, центроид, yaw, нормаль поверхности, состояние `VISIBLE`/`OCCLUDED` и кандидатов безопасного захвата.

[![RGB-D dashboard станции КТЯ](docs/media/kty_rgbd_dashboard.png)](docs/media/kty_rgbd_dashboard.png)

Подробные материалы:

- [KTY runtime v18 — handoff и критерии приёмки](docs/KTY_RUNTIME_V18_HANDOFF.md)
- [Команды сборки, запуска и диагностики](docs/KTY_RUNTIME_COMMANDS.md)

## 7. Архитектура

```text
Gazebo worlds/models
    ↓ camera, depth, poses, contacts
ros_gz_bridge
    ↓ ROS topics
Perception nodes
    ↓ typed observations / contours / fill state
Control nodes
    ↓ matrix, separator and KTY commands
Gazebo plugins
    ↓ physical actuation
Diagnostics, validators and dashboards
```

Основная документация:

- [Архитектура](docs/ARCHITECTURE.md)
- [ROS-интерфейсы](docs/INTERFACES.md)
- [Параметры симуляции](docs/SIMULATION_PARAMETERS.md)
- [Сценарии демонстрации](docs/DEMO_SCENARIOS.md)
- [Алгоритм сингуляризации](docs/SINGULATION_CONTROL.md)
- [Runtime-приёмка](docs/RUNTIME_ACCEPTANCE.md)
- [Диагностика](docs/TROUBLESHOOTING.md)
- [Галерея скриншотов](docs/media/README.md)
- [Очистка репозитория](docs/REPOSITORY_CLEANUP.md)

## 8. Структура пакетов

| Пакет | Назначение |
|---|---|
| `singulator_interfaces` | Пользовательские ROS-сообщения |
| `singulator_description` | Геометрия и модели |
| `singulator_gazebo` | Миры и генераторы SDF |
| `singulator_bringup` | Launch и bridge-конфигурации |
| `singulator_sim` | Генераторы потока и служебные узлы |
| `singulator_control` | Контроллеры матрицы и сепаратора |
| `singulator_perception` | Классическое машинное зрение матрицы |
| `kty_conveyor_surface` | Gazebo-плагин транспортных зон КТЯ |
| `kty_station_sim` | Автомат КТЯ, RGB-D, уплотнение и dashboard |

## 9. Проверки перед демонстрацией

Единый статический набор:

```bash
bash ./scripts/run_release_checks.sh
```

Отдельные проверки:

```bash
python3 tools/validate_project.py
python3 tools/validate_release.py
python3 tools/validate_separator_demo.py
python3 tools/validate_kty_runtime_v18.py
python3 tools/test_v7_logic.py
bash ./scripts/check_v7_control.sh
bash ./scripts/check_vision.sh
```

CI и статические валидаторы проверяют Python, XML/SDF, shell-скрипты и межфайловые контракты, но не заменяют физический runtime-прогон Gazebo.

## 10. Принятый статус

На целевой машине подтверждены:

- матрица 18×4 и роликовое горлышко;
- машинное зрение матрицы с публикацией ID, координат и debug-изображения;
- инфид-сепаратор с шагом 150 мм и демпфированной контактной моделью;
- станция КТЯ runtime v18 с четырьмя непрерывными циклами, сходящимися направляющими и суженной зоной спавна;
- RGB-D dashboard станции КТЯ.

Подробная фиксация: [RUNTIME_ACCEPTANCE.md](docs/RUNTIME_ACCEPTANCE.md).

## 11. Ограничения модели

- проект проверен на одном основном ПК;
- RTF зависит от оборудования и числа активных моделей;
- коэффициенты трения и ограничения приводов являются калибровочными параметрами;
- деформация картона и упаковки не моделируется;
- одна верхняя RGB-D камера не гарантирует разделение одинаковых соприкасающихся коробок без видимого шва;
- интеграция с внешней WMS имитируется;
- Docker-образ пока не поддерживается.

## 12. Воспроизводимость релиза

Стабильный runtime зафиксирован тегом `v1.0.0`. Тег остаётся на принятом release-коммите; последующие изменения документации и медиа оформляются отдельными PR.

Для воспроизводимости сохраняются:

- commit SHA и release tag;
- `seed=42`;
- вывод `bash ./scripts/run_release_checks.sh`;
- версии Ubuntu, ROS 2 и Gazebo;
- результат `check_kty_runtime_v18.sh`;
- RTF и скриншоты демонстраций из [docs/media](docs/media/README.md).
