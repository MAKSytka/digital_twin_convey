# singulator_perception

Пакет переводит исходный однокадровый алгоритм `BoxDetector.exe` в
непрерывный ROS 2-конвейер обработки кадров:

```text
Gazebo RGB camera
  -> ros_gz_image
  -> /singulator/camera/image_raw
  -> vision_stream_node
  -> /singulator/boxes (BoxObservationArray)
  -> алгоритм сингуляризации
```

Узел не читает положения моделей из Gazebo. Единственный источник положения
товаров — RGB-кадр верхней камеры.

## Что изменено относительно приложения

- калибровка поля выполняется один раз по пустой сцене;
- пустой кадр сохраняется как фон, поэтому синие ячейки матрицы не принимаются
  за товары;
- контуры, геометрия и опциональная ORB-классификация выполняются для каждого
  нового кадра;
- ближайшие положения связываются между кадрами, поэтому `id` остаётся
  стабильным;
- координаты переводятся из системы изображения в мировую систему симуляции;
- публикуется отладочное изображение с контурами и бинарной маской.

## Топики

| Назначение | Топик | Тип |
|---|---|---|
| Входной поток | `/singulator/camera/image_raw` | `sensor_msgs/msg/Image` |
| Наблюдения | `/singulator/boxes` | `singulator_interfaces/msg/BoxObservationArray` |
| Отладка | `/singulator/perception/debug_image` | `sensor_msgs/msg/Image` |

## Запуск

Камера и машинное зрение включены в `matrix_stream.launch.py` через параметр
`start_perception`.

```bash
cd ~/singulator_digital_twin
./scripts/setup_dependencies.sh
./scripts/build.sh
source install/setup.bash

ros2 launch singulator_bringup matrix_stream.launch.py \
  start_perception:=true \
  start_spawner:=true \
  start_demo_controller:=true
```

Спавнер запускается через 6 секунд. За это время камера собирает пустые кадры,
находит границы поля и формирует эталон фона.

## Проверка

```bash
ros2 topic hz /singulator/camera/image_raw
ros2 topic echo /singulator/boxes --once
ros2 run rqt_image_view rqt_image_view \
  /singulator/perception/debug_image
```

## Система координат

Калибровочное поле имеет размер `7.70 x 0.90 м`. Его левый верхний угол на
изображении соответствует мировой точке `X=-3.95 м, Y=+0.45 м`.

```text
world_x = -3.95 + image_field_x
world_y = +0.45 - image_field_y
world_yaw = -image_angle
```

Поэтому алгоритм управления получает метры и радианы в той же мировой системе,
в которой расположена матрица, но не получает истинные позы Gazebo.

## Ограничение высоты

Одна RGB-камера над плоскостью надёжно оценивает `X`, `Y`, длину, ширину и yaw.
Высота пока задаётся параметром `default_box_height_m=0.12`. Для реальной
оценки высоты потребуется depth-камера, стереопара или отдельный профильный
датчик.

## Реальная камера

Для физической камеры меняется только источник `/singulator/camera/image_raw`.
При необходимости можно отключить фоновую сегментацию параметром
`use_background_subtraction:=false`; тогда используется HSV-сегментация,
восстановленная из исходного приложения. Путь к датасету ORB задаётся параметром
`dataset_folder`.
