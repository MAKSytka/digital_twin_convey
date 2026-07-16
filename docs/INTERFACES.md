# ROS-интерфейсы и система координат

## Система координат

- `+X` — направление движения товара от входа к выходу;
- `+Y` — поперёк матрицы;
- `+Z` — вверх;
- начало координат находится в центре матрицы.

Строки:

```text
r00: X = -2.470 м
r01: X = -2.090 м
...
r13: X = +2.470 м
```

Колонки:

```text
c00: Y = -0.2925 м
c01: Y = -0.0975 м
c02: Y = +0.0975 м
c03: Y = +0.2925 м
```

Положительная скорость TrackController двигает товар вдоль `+X`.

## Общая команда матрицы

### Топик

```text
/singulator/matrix/command
```

### Тип

```text
singulator_interfaces/msg/MatrixCommand
```

```text
std_msgs/Header header
uint16 rows
uint16 cols
float32[] target_speed_mps
```

### Обязательные условия

- `rows == 14`;
- `cols == 4`;
- размер массива равен `rows * cols == 56`;
- скорость указывается в м/с;
- допустимый физической моделью диапазон: `−2.0…2.0 м/с`.

### Порядок массива

```python
index = row * cols + col
```

То есть сначала идут четыре колонки строки `r00`, затем четыре колонки `r01` и так далее.

```text
[ r00c00, r00c01, r00c02, r00c03,
  r01c00, r01c01, r01c02, r01c03,
  ...,
  r13c00, r13c01, r13c02, r13c03 ]
```

## Состояние матрицы

### Топик

```text
/singulator/matrix/state
```

### Тип

```text
singulator_interfaces/msg/MatrixState
```

```text
std_msgs/Header header
uint16 rows
uint16 cols
float32[] target_speed_mps
float32[] actual_speed_mps
bool[] fault
```

Текущее ограничение: `actual_speed_mps` копирует целевые значения, а `fault` всегда содержит `false`. Это подтверждение приёма команды, а не реальное измерение приводов.

## Команды отдельных приводов

```text
/singulator/cell/rXX_cYY/cmd_vel
std_msgs/msg/Float64
```

Эти топики являются внутренним интерфейсом симулятора. Внешнему алгоритму следует публиковать одну общую `MatrixCommand`.

## Входной и выходной конвейеры

```text
/singulator/infeed/cmd_vel
/singulator/outfeed/cmd_vel
std_msgs/msg/Float64
```

Основной launch-файл поддерживает команды с частотой 10 Гц через `aux_conveyor_controller`.

## Gazebo Transport odometry

Каждый TrackController создаёт транспортный топик:

```text
/singulator/cell/rXX_cYY/odometry
/singulator/infeed/odometry
/singulator/outfeed/odometry
```

В основном режиме они не мостятся в ROS. Для диагностики:

```bash
gz topic -e -t /singulator/cell/r00_c00/odometry
```

## Наблюдение коробки

```text
singulator_interfaces/msg/BoxObservation
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

Массив наблюдений:

```text
/singulator/boxes
singulator_interfaces/msg/BoxObservationArray
```

В текущей версии издатель этого топика отсутствует. Тип подготовлен для будущего адаптера Gazebo или машинного зрения.

## Полезные команды

```bash
ros2 interface show singulator_interfaces/msg/MatrixCommand
ros2 interface show singulator_interfaces/msg/BoxObservationArray
ros2 topic info /singulator/matrix/command
ros2 topic echo /singulator/matrix/state
```
