# Демонстрация входного размерного сепаратора

## Назначение

Стенд показывает предварительный механический отсев мелких товаров перед
основной матрицей сингуляризации. Входная линия, верхний выход и нижний выход
имеют ширину 2,5 м. Скорость всех лент по умолчанию равна 2 м/с.

После роликового экрана поток разделяется:

- товар, уверенно перекрывающий отверстия экрана, продолжает движение по
  верхнему трёхметровому конвейеру;
- мелкий товар проваливается на нижнюю приёмную ленту и затем движется по
  отдельному трёхметровому конвейеру.

Коробки автоматически удаляются после достижения конца соответствующей ветви.

## Геометрия экрана

Экран состоит из 11 физических поперечных валов с шагом 120 мм. Каждый вал
является одной динамической моделью и содержит:

- один неподвижный корпус с опорами;
- один вращающийся link `rotor`;
- одно `revolute`-соединение вокруг оси Y;
- один `gz::sim::systems::JointController`;
- 25 дисковых коллизий на общем вращающемся link.

Это важно для производительности. В ранней версии каждый диск был отдельной
динамической моделью со своим joint и контроллером. При 11 валах получалось
275 отдельных моделей и контроллеров, что могло перегружать GUI и приводить к
белому окну. В текущей версии количество управляемых вращающихся моделей
снижено до 11, при этом дисковые контакты сохранены.

Параметры дисков:

- шаг дисков по ширине: 100 мм;
- упрощённая цилиндрическая коллизия: диаметр 50 мм, толщина 30 мм;
- чистое отверстие по X: `120 - 50 = 70 мм`;
- чистое отверстие по Y: `100 - 30 = 70 мм`.

Визуальные зубья детализированы сильнее коллизий. Это сохраняет понятный вид
механизма, но не создаёт отдельную сложную коллизию для каждого зуба.

## Правило размерного класса

Для основания `L × W`, повёрнутого на угол `yaw`, вычисляются две проекции:

```text
projection_x = |L cos(yaw)| + |W sin(yaw)|
projection_y = |L sin(yaw)| + |W cos(yaw)|
```

Ожидаемый нижний маршрут назначается по правилу:

```text
min(projection_x, projection_y) < 0.070 м  -> LOWER
иначе                                      -> UPPER
```

Так учитывается не только короткий товар, но и длинная узкая упаковка, которая
опасна для 20-миллиметровых зазоров основной матрицы.

Для чистой демонстрации верхний класс генерируется с минимальной проекцией не
менее 90 мм. Область непосредственно около 70 мм следует отдельно калибровать
по результатам физической симуляции.

## Входящий поток

Центр масс каждой новой коробки выбирается из десяти фиксированных координат Y
на ширине 2,5 м. Размер, высота, масса и угол коробки меняются случайно.

Параметры по умолчанию:

```text
spawn_mode                  = continuous
target_rate_boxes_per_sec   = 4.0
small_item_probability      = 0.20
cutoff_m                    = 0.070
conveyor_speed_mps          = 2.0
screen_surface_speed_mps    = 2.0
seed                        = 42
```

Искусственная постоянная сила к коробкам не прикладывается. Продольное движение
должно возникать от контакта с лентами и вращающимися дисками.

## Запуск

```bash
cd ~/singulator_digital_twin
git fetch origin
git switch feature-realistic-separator-flow
git pull --ff-only

source /opt/ros/jazzy/setup.bash
rm -rf build/singulator_description build/singulator_gazebo \
  build/singulator_bringup build/singulator_control build/singulator_sim
rm -rf install/singulator_description install/singulator_gazebo \
  install/singulator_bringup install/singulator_control install/singulator_sim

colcon build --symlink-install
source install/setup.bash

ros2 launch singulator_bringup infeed_size_separator_demo.launch.py
```

Очистка выбранных пакетов перед сборкой обязательна после изменения структуры
моделей. Иначе в `install` могут остаться старые модели отдельных дисков.

### Регулирование непрерывного потока

```bash
ros2 launch singulator_bringup infeed_size_separator_demo.launch.py \
  spawn_mode:=continuous \
  target_rate_boxes_per_sec:=4.0 \
  small_item_probability:=0.20 \
  conveyor_speed_mps:=2.0 \
  screen_surface_speed_mps:=2.0 \
  seed:=42
```

### Конечный тест

```bash
ros2 launch singulator_bringup infeed_size_separator_demo.launch.py \
  spawn_mode:=finite \
  maximum_items:=100 \
  target_rate_boxes_per_sec:=4.0 \
  small_item_probability:=0.20
```

После создания последней коробки спавнер останавливается, но оставшиеся товары
продолжают движение до выхода и деспавна.

## Камера и белое окно

В world-файл добавлены `GzSceneManager`, `InteractiveViewControl` и
`CameraTracking`. Поэтому команды `move_to`, `follow` и `track` должны быть
доступны. Для совместимости стенд по умолчанию использует Ogre 1 вместо Ogre 2.

Проверка доступности камеры:

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

Следовать за конкретной коробкой можно только после её появления и с точным
именем из лога спавнера:

```bash
gz service -s /gui/follow \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'data: "box_separator_<точное_имя>"'
```

Если окно остаётся белым, сначала проверь простой мир:

```bash
gz sim -v 4 shapes.sdf --render-engine ogre
```

Затем посмотри ошибки клиента:

```bash
grep -iE 'error|exception|render|ogre' ~/.gz/rendering/ogre*.log | tail -n 80
```

Для проблем Qt / Wayland можно проверить запуск в X11-режиме:

```bash
QT_QPA_PLATFORM=xcb ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py
```

## Логи

`separator_demo_spawner` выводит:

- размеры и угол каждой коробки;
- обе опорные проекции;
- выбранный фиксированный spot;
- ожидаемый маршрут;
- фактическую частоту успешного создания;
- количество ожидаемых верхних и нижних товаров.

`separator_demo_cleanup` выводит:

- подтверждённый фактический маршрут;
- ошибки `expected/actual`;
- число товаров на каждой ветви;
- среднее время до определения маршрута;
- число удалённых и активных коробок;
- ошибки удаления.

Имена коробок содержат ожидаемый маршрут:

```text
box_separator_<session>_n0000123_exp_upper_spot07
box_separator_<session>_n0000124_exp_lower_spot02
```

## Статическая проверка

```bash
python3 tools/validate_separator_demo.py

python3 -m py_compile \
  src/singulator_control/singulator_control/separator_demo_controller.py \
  src/singulator_sim/singulator_sim/separator_demo_spawner.py \
  src/singulator_sim/singulator_sim/separator_demo_cleanup.py \
  src/singulator_bringup/launch/infeed_size_separator_demo.launch.py \
  tools/validate_separator_demo.py
```

Валидатор проверяет ширину и длину лент, 11 физических валов, 25 дисковых
коллизий на каждом валу, отверстия 70 мм, по одному `revolute`-соединению и
контроллеру на вал, плагины камеры, управляющие топики, параметры потока и
отсутствие искусственного `transport_assist`.

## Ограничения текущей модели

Статическая проверка не подтверждает фактическое прохождение коробок. После
сборки необходимо проверить в Gazebo:

1. направление вращения валов и движение поверхности по +X;
2. отсутствие зависания на переходах лента–экран и экран–верхняя лента;
3. попадание уверенно мелких товаров на нижнюю ветвь;
4. прохождение уверенно крупных товаров сверху;
5. нагрузку симулятора при 4 товарах/с;
6. долю ошибок в пограничной области около 70 мм.

После калибровки модуль можно встроить перед основной матрицей 18×4. Верхний
выход подключается к основной матрице, нижний — к отдельной мелкоячеистой зоне
сингуляризации.
