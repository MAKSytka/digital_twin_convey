# ROS-интерфейсы и системы координат

## Матрица сингуляризации

### Система координат

- `+X` — движение от входа к выходу;
- `+Y` — поперёк матрицы;
- `+Z` — вверх;
- начало — центр матрицы.

Строки идут от `r00` на входе до `r13` на выходе. Колонки нумеруются от отрицательного `Y` к положительному.

### Общая команда

```text
/singulator/matrix/command
singulator_interfaces/msg/MatrixCommand
```

```text
std_msgs/Header header
uint16 rows
uint16 cols
float32[] target_speed_mps
```

Обязательные условия:

- `rows == 14`;
- `cols == 4`;
- `len(target_speed_mps) == 56`;
- индекс: `row * cols + col`;
- единица скорости: м/с.

### Состояние

```text
/singulator/matrix/state
singulator_interfaces/msg/MatrixState
```

`actual_speed_mps` в текущей реализации не является полноценной обратной связью от каждого физического привода.

### Наблюдения товаров

```text
/singulator/boxes
singulator_interfaces/msg/BoxObservationArray
```

```text
uint32 id
string model_name
geometry_msgs/Point center
float32 length_m
float32 width_m
float32 height_m
float32 yaw_rad
float32 confidence
```

## Станция операций с КТЯ

### Системы координат

`kty_station`:

- `+X` — от входной контактной зоны к выходной;
- `+Y` — поперёк КТЯ;
- `+Z` — вверх;
- центр виброплатформы находится около `X=0`, `Y=0`.

`kty_inner`:

- локальная система внутреннего объёма установленного КТЯ;
- полигон машинного зрения публикуется в плоскости дна;
- высота задаётся относительно внутреннего дна КТЯ.

### Команды контактных зон и механизмов

```text
/kty/infeed/cmd_vel       std_msgs/msg/Float64
/kty/platform/cmd_vel     std_msgs/msg/Float64
/kty/outfeed/cmd_vel      std_msgs/msg/Float64
/kty/platform/cmd_pos     std_msgs/msg/Float64
/kty/shutter/cmd_pos      std_msgs/msg/Float64
```

`cmd_vel` задаётся в м/с. `platform/cmd_pos` задаёт вертикальное отклонение платформы в метрах. `shutter/cmd_pos` задаёт положение вертикального призматического соединения шторки.

### Состояние станции

```text
/kty/station/state
singulator_interfaces/msg/KtyStationState
```

```text
std_msgs/Header header
uint32 cycle_id
uint8 state
string state_name
string reason
bool kty_expected
bool shutter_closed
bool vibration_enabled
bool product_feed_enabled
float32 vibration_frequency_hz
float32 vibration_amplitude_m
float32 measured_maximum_height_m
float32 fill_height_threshold_m
float32 estimated_mass_kg
```

Состояния:

```text
WAIT_EMPTY_KTY, POSITION_KTY, CLAMP, LOAD, VIBRATE,
SETTLE, SCAN, EJECT_PREP, EJECT, FAULT
```

### RGB-D сенсор

```text
/kty/camera/image          sensor_msgs/msg/Image
/kty/camera/depth_image    sensor_msgs/msg/Image
/kty/camera/camera_info    sensor_msgs/msg/CameraInfo
```

### Полигональные контуры

```text
/kty/perception/contours
singulator_interfaces/msg/KtyProductContourArray
```

Массив содержит:

```text
std_msgs/Header header
uint32 frame_sequence
bool camera_ok
float32 valid_depth_fraction
float32 maximum_height_m
float32 top_fill_ratio
KtyProductContour[] products
```

Один товар:

```text
uint32 track_id
geometry_msgs/Polygon polygon
geometry_msgs/Point32 centroid
float32 top_height_m
float32 visible_area_m2
float32 confidence
bool side_neg_x_accessible
bool side_pos_x_accessible
bool side_neg_y_accessible
bool side_pos_y_accessible
float32 clearance_neg_x_m
float32 clearance_pos_x_m
float32 clearance_neg_y_m
float32 clearance_pos_y_m
```

`track_id` предназначен для сохранения идентичности между последовательными микропаузами. Полигон описывает видимую верхнюю область, а доступность сторон — предварительный интерфейс для будущего манипулятора.

### Ground truth

```text
/kty/ground_truth/registry
singulator_interfaces/msg/KtyGroundTruthArray
```

Реестр содержит размер, массу, профиль и имя Gazebo-модели. Он используется safety/metrics узлами и не должен подключаться к входу `depth_perception`.

### Аварии

```text
/kty/fault
singulator_interfaces/msg/KtyFault
```

Коды:

```text
product_outside_kty
product_jammed_on_chute
kty_not_installed
mass_exceeded
camera_lost_view
product_still_moving
```

Критическое активное сообщение переводит автомат в `FAULT`.

### Управление сценарием

```text
/kty/cycle_id                    std_msgs/msg/UInt32
/kty/product_spawner/enabled     std_msgs/msg/Bool
/kty/product_spawner/clear       std_msgs/msg/Bool
/kty/station/reset               std_srvs/srv/Trigger
```

### Позиции Gazebo

```text
/kty/world/poses
tf2_msgs/msg/TFMessage
```

Топик мостит `/world/kty_station/pose/info` и используется только для safety, ground-truth метрик и проверки качества зрения.

## Полезные команды

```bash
ros2 interface show singulator_interfaces/msg/MatrixCommand
ros2 interface show singulator_interfaces/msg/KtyStationState
ros2 interface show singulator_interfaces/msg/KtyProductContourArray
ros2 interface show singulator_interfaces/msg/KtyFault
ros2 topic echo /kty/station/state
ros2 topic echo /kty/fault
```
