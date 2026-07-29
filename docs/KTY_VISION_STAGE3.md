# KTY vision stage 3

Этот этап добавляет к принятому циклу движения КТЯ отдельный контур машинного
зрения. Он не использует ground truth для распознавания товаров: геометрия
получается из изображения глубины фиксированной RGB-D камеры.

## Архитектура

```text
Gazebo RGB-D camera
  /kty/vision/image
  /kty/vision/depth_image
        |
        v
kty_depth_perception
  - depth ROI внутри КТЯ
  - маска высоты над дном
  - morphology + connected components
  - watershed для соприкасающихся товаров
  - polygon approximation
  - persistent greedy tracking
        |
        +--> /kty/perception/contours
        +--> /kty/perception/debug_image
                 |
                 +--> kty_contour_recorder
                 |      - polygons.jsonl
                 |      - polygons_latest.json
                 |      - /kty/vision/polygons_json
                 |
                 +--> kty_vision_dashboard
                        - RGB + контуры
                        - depth heatmap
                        - вид сверху
                        - ID, высота, заполнение, доступные стороны
                        - /kty/vision/dashboard
```

## RGB-D камера

Камера установлена над центром активной зоны:

- высота оптического центра: `1,75 м`;
- расстояние до дна установленного КТЯ: `1,25 м`;
- разрешение: `800 × 600`;
- частота: `15 Гц`;
- горизонтальное поле зрения: `1,05 рад`;
- диапазон глубины: `0,20–3,0 м`;
- рендер: Ogre 2.

Камера неподвижна. Полезные данные формируются, когда КТЯ находится в центре
платформы. При подводе и отводе контуры могут временно исчезать — это ожидаемо.

## Выходной интерфейс полигонов

Основной типизированный топик:

```text
/kty/perception/contours
singulator_interfaces/msg/KtyProductContourArray
```

Для каждого объекта передаются:

- устойчивый `track_id`;
- центроид в системе `kty_inner`;
- полигон верхней видимой проекции в метрах;
- верхняя высота;
- видимая площадь;
- confidence;
- доступность подхода с четырёх сторон;
- свободный зазор с четырёх сторон.

Эти данные предназначены для следующего этапа — выбора точки захвата и стороны
подхода робота-манипулятора.

Диагностическое JSON-зеркало:

```text
/kty/vision/polygons_json
std_msgs/msg/String
```

Файлы сохраняются в:

```text
~/.ros/kty_vision/polygons.jsonl
~/.ros/kty_vision/polygons_latest.json
```

`polygons.jsonl` содержит историю кадров с обнаруженными объектами.
`polygons_latest.json` атомарно заменяется последним непустым кадром.

## Dashboard

Окно `KTY RGB-D Vision Dashboard` показывает:

1. RGB-изображение с контурами и ID;
2. цветную карту глубины;
3. вид сверху на внутреннюю область КТЯ;
4. текущее состояние flow-цикла;
5. состояние камеры и долю валидной глубины;
6. число отслеживаемых товаров;
7. максимальную высоту и коэффициент заполнения сверху;
8. количество точек каждого полигона;
9. доступные стороны подхода.

Одновременно готовое изображение dashboard публикуется в:

```text
/kty/vision/dashboard
sensor_msgs/msg/Image
```

Поэтому UI можно наблюдать и через стандартные ROS-инструменты, даже если
OpenCV-окно отключено.

## Сборка

```bash
cd ~/singulator_digital_twin
chmod +x scripts/*kty_vision.sh tools/validate_kty_vision.py
python3 tools/validate_kty_vision.py
bash ./scripts/build_kty_vision.sh
```

## Запуск

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/run_kty_vision.sh
```

По умолчанию цикл повторяется непрерывно, чтобы dashboard не закрывался после
первой КТЯ.

Один цикл без повторения:

```bash
bash ./scripts/run_kty_vision.sh auto_repeat:=false
```

Запуск без отдельного OpenCV-окна:

```bash
bash ./scripts/run_kty_vision.sh show_dashboard:=false
```

Изображение при этом остаётся доступно в `/kty/vision/dashboard`.

Другой каталог записи:

```bash
bash ./scripts/run_kty_vision.sh \
  polygon_output_directory:=/tmp/kty_polygons
```

## Диагностика

Во втором терминале:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/check_kty_vision.sh
```

Ручные проверки:

```bash
ros2 topic echo /kty/perception/contours --once
ros2 topic echo /kty/vision/polygons_json --once
ros2 topic hz /kty/vision/image
ros2 topic hz /kty/vision/depth_image
ros2 topic hz /kty/vision/dashboard
```

## Критерий приёмки

Этап принят, когда на целевой машине подтверждено:

1. видны RGB и depth-потоки;
2. dashboard обновляется без зависания;
3. после падения товаров появляются зелёные контуры;
4. устойчивые ID не меняются при небольшом движении товаров;
5. для каждого объекта полигон содержит минимум три точки;
6. `polygons_latest.json` и `polygons.jsonl` создаются;
7. `check_kty_vision.sh` завершается строкой `KTY vision diagnostics: OK`;
8. плавность движения и пятсекундная вибрация из обновлённого PR #40 визуально
   подтверждены.

## Ограничения текущего этапа

- координаты полигонов заданы в плоскости КТЯ, а не в базе будущего робота;
- нет калибровки hand-eye;
- не выбирается точка захвата;
- частично перекрытые товары разделяются watershed-алгоритмом, поэтому сложные
  стопки ещё потребуют 3D-сегментации;
- UI является операторским наблюдением, а не системой управления безопасностью;
- runtime RGB-D и качество контуров должны быть проверены на Ubuntu 24.04,
  ROS 2 Jazzy и Gazebo Harmonic.
