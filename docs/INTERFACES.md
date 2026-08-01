# ROS 2-интерфейсы и системы координат

Документ описывает публичные контракты, которые нужны для запуска, диагностики и интеграции трёх демонстрационных стендов.

## 1. Матрица сингуляризации

### Система координат

- `+X` — движение от входа к выходу;
- `+Y` — поперёк матрицы;
- `+Z` — вверх;
- строки идут от `r00` на входе до `r17` на выходе;
- колонки `c00...c03` идут от отрицательного `Y` к положительному.

### Наблюдения товаров

```text
/singulator/boxes
singulator_interfaces/msg/BoxObservationArray
```

Один объект содержит логический ID, имя модели, центр, размеры, yaw, confidence и поля движения, определённые в актуальном `.msg`-контракте. Наблюдения формируются классическим машинным зрением; Gazebo ground truth не используется как вход контроллера.

### Общая команда матрицы

```text
/singulator/matrix/command
singulator_interfaces/msg/MatrixCommand
```

Ключевые условия актуального roller-сценария:

```text
rows = 18
cols = 4
len(target_speed_mps) = 72
index = row * cols + col
speed unit = m/s
```

Команда формируется `singulation_controller`, а `matrix_command_fanout` преобразует её в индивидуальные команды зон.

### Индивидуальные команды

Шаблон имён:

```text
/singulator/cell/r00_c00/cmd_vel
...
/singulator/cell/r17_c03/cmd_vel
```

Тип команды на стороне ROS задаётся конфигурацией bridge/fan-out и передаётся в Gazebo TrackController соответствующей зоны.

### Состояние и диагностика

```text
/singulator/matrix/state
singulator_interfaces/msg/MatrixState
```

`actual_speed_mps` не следует трактовать как промышленную обратную связь энкодера каждого физического привода. Это состояние цифровой модели и программного контура.

Полезные проверки:

```bash
ros2 topic echo /singulator/boxes --once
ros2 topic echo /singulator/matrix/command --once
ros2 topic hz /singulator/matrix/command
```

## 2. Инфид-сепаратор

Стенд использует ROS-команды скоростей входной, верхней, нижней лент и вращающегося экрана. Точные Gazebo-топики задаются bridge-конфигурацией `singulator_bringup`.

Основные логические узлы:

```text
separator_demo_controller
separator_demo_spawner
separator_demo_cleanup
```

Спавнер принимает launch-параметры:

```text
spawn_mode
maximum_items
target_rate_boxes_per_sec
small_item_probability
seed
conveyor_speed_mps
screen_surface_speed_mps
```

Cleanup публикует диагностическую статистику в логах: фактические верхние/нижние маршруты, удаления, ошибки удаления и перезапуски монитора.

## 3. Станция операций с КТЯ

### Системы координат

`kty_station`:

- `+X` — от входной транспортной зоны к выходной;
- `+Y` — поперёк КТЯ;
- `+Z` — вверх.

`kty_inner`:

- локальная система внутреннего объёма активного КТЯ;
- геометрия товаров публикуется в метрах;
- высота задаётся относительно внутреннего дна.

### Камера

```text
/kty/vision/image          sensor_msgs/msg/Image
/kty/vision/depth_image    sensor_msgs/msg/Image
```

Камера работает с частотой 15 Гц в принятой конфигурации. Графический dashboard может обновляться реже, не изменяя частоту perception-контура.

### Результат классического 3-D perception

```text
/kty/perception/contours
singulator_interfaces/msg/KtyProductContourArray
```

Для видимого товара публикуются:

- постоянный `track_id`;
- верхний полигон;
- ориентированный прямоугольник;
- 3-D центроид;
- нормаль поверхности;
- оценка размеров XYZ;
- yaw;
- confidence;
- оценка окклюзии;
- состояние `VISIBLE` или `OCCLUDED`;
- признак доступности сверху;
- кандидаты захвата.

Для `OCCLUDED`-объекта ID может временно сохраняться, но безопасные кандидаты захвата не публикуются.

### Dashboard

```text
/kty/vision/dashboard
sensor_msgs/msg/Image
```

Запуск отдельного окна:

```bash
ros2 run kty_station_sim vision_dashboard_3d \
  --ros-args \
  -r __node:=kty_vision_dashboard_window \
  -p show_window:=true \
  -p refresh_hz:=3.0
```

### Оценка заполнения

```text
/kty/fill/state
```

Состояние содержит оценку заполнения и максимальной высоты, используемые автоматом для перехода к закрытию створки и уплотнению. Точный тип следует проверять командой:

```bash
ros2 topic type /kty/fill/state
```

### Состояние рабочего цикла

```text
/kty/flow/state
```

Принятая последовательность runtime v18:

```text
LOAD
CLOSE_GATE
COMPACT
EJECT_ACTIVE
DESPAWN_ACTIVE
POSITION_NEXT
VERIFY_READY
OPEN_GATE
```

### Реестр поз

```text
/kty/mech/model_pose_registry_json
```

Реестр используется автоматом жизненного цикла и диагностикой для подтверждения создания, положения и удаления КТЯ, створки и товаров. Он не является входом алгоритма сегментации RGB-D.

### Сохраняемые данные

```text
~/.ros/kty_vision/polygons_latest.json
~/.ros/kty_vision/polygons.jsonl
```

Схема включает геометрию, состояние трека, OBB, нормали, размеры и кандидатов захвата.

## 4. Sim time и QoS

- физические сценарии используют `/clock` Gazebo;
- узлы, управляющие динамикой, запускаются с `use_sim_time=true`, если иное явно не указано в принятом runtime;
- состояние, необходимое поздно подключившимся узлам, может использовать transient-local QoS;
- перед диагностикой следует убедиться, что `sim_time` увеличивается.

Проверка:

```bash
ros2 topic echo /clock --once
ros2 node list
ros2 topic list
```

## 5. Правило изменения контрактов

Изменение `.msg`, имени топика, frame convention или порядка массива команд считается изменением межмодульного API. Такое изменение должно сопровождаться одновременным обновлением:

1. издателя;
2. подписчика;
3. launch/bridge-конфигурации;
4. валидатора;
5. этого документа.

Для точного состава любого пользовательского сообщения источником истины является установленный интерфейс:

```bash
ros2 interface show singulator_interfaces/msg/MatrixCommand
ros2 interface show singulator_interfaces/msg/BoxObservation
ros2 interface show singulator_interfaces/msg/KtyProductContour
ros2 interface show singulator_interfaces/msg/KtyProductContourArray
```
