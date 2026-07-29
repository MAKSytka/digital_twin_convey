# Цифровой двойник станции операций с КТЯ

## Назначение

Пакет `kty_station_sim` моделирует полный цикл одной станции:

1. спавн пустого КТЯ на входной зоне;
2. подвод КТЯ к виброплатформе за `2 с`;
3. боковое ограничение направляющими;
4. хаотичную подачу товара по лотку со средней интенсивностью `1 ед/с`;
5. вертикальную регулируемую вибрацию;
6. микропаузу, RGB-D измерение и решение о заполненности;
7. остановку вибрации за `0,5 с` до отвода;
8. отвод КТЯ за `1 с` и удаление модели из Gazebo.

КТЯ имеет внутренний размер `600 × 400 × 400 мм`, стенку `3 мм`, открытый верх
и массу пустой тары `1,6 кг`.

## Принятые параметры

- лоток: `1000 × 600 мм`, угол `32°`;
- нижний край лотка: `420 мм` над внутренним дном КТЯ;
- коэффициент трения основных пар: `0,75`;
- верхняя прокладка платформы: полиуретан TPU-95A;
- движение виброплатформы: вертикальное;
- четыре пружины показаны визуально, физическая степень свободы задаётся
  призматическим соединением;
- рабочий диапазон: `20–50 Гц`, амплитуда до `3 мм` как отклонение от среднего
  положения;
- стартовая конфигурация: `25 Гц`, `1 мм`.

При `mu=0,75` и угле `32°` полностью успокоившийся плоский товар может не
продолжить скольжение. Это параметр для последующей экспериментальной
калибровки.

## Архитектура runtime v3

Runtime v3 устраняет зависание в `POSITION_KTY`. Подвод и отвод выполняются
через сервис Gazebo UserCommands:

```text
/world/kty_station/set_pose
gz.msgs.Pose -> gz.msgs.Boolean
```

Контроллер интерполирует положение КТЯ по симуляционному времени:

```text
x=-1.30 м -> x=0.00 м за 2 с
x=0.00 м  -> выходная позиция за 1 с
```

Это движение является детерминированным и не зависит от коэффициента трения,
направления виртуальной гусеницы или ROS-моста `Twist`. Команды контактных зон
`TrackController` сохранены для визуального и физического движения поверхностей.

Постоянный `VelocityControl` из модели КТЯ удалён. Между фазами подвода и отвода
тара остаётся обычным динамическим телом и реагирует на:

- гравитацию;
- контакт с виброплатформой;
- вертикальное движение платформы;
- падение и столкновение товаров.

Параметры транспорта:

```yaml
transport_update_period_s: 0.05
transport_position_tolerance_m: 0.005
transport_failure_limit: 8
world_reset_jump_threshold_s: 0.10
```

Высокочастотный драйвер вибрации работает отдельно:

```text
/station_controller
    -> /kty/station/state

/kty_vibration_driver (500 Гц)
    -> /kty/platform/cmd_pos_filtered

ros_gz_bridge
    -> Gazebo /kty/platform/cmd_pos
```

КТЯ не получает принудительную вертикальную скорость: колебания передаются через
контакт с платформой.

## Автомат состояний

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
       -> повтор проверки при единичной потере кадра
  -> EJECT
  -> WAIT_EMPTY_KTY
```

В состояниях `LOAD` и `VIBRATE` контроллер непрерывно публикует:

```text
/kty/product_spawner/enabled = true
```

Спавнер после этого создаёт модели вида:

```text
kty_product_c000001_p000001
```

## Обзорная камера Gazebo

GUI мира загружает четыре взаимосвязанных плагина:

```text
MinimalScene
GzSceneManager
InteractiveViewControl
CameraTracking
```

`InteractiveViewControl` обрабатывает мышь. Фиксированная RGB-D камера машинного
зрения при этом не перемещается.

Управление обзором:

- левая кнопка и перемещение — панорамирование;
- средняя кнопка и перемещение — вращение;
- `Shift` + левая кнопка — альтернативное вращение;
- колесо или правая кнопка и перемещение — масштабирование.

## Play, Pause, Step и Reset

Панель `WorldControl` обращается непосредственно к сервису:

```text
/world/kty_station/control
```

Режим `use_event=true` удалён, поскольку он не гарантировал управление сервером
мира в этой конфигурации GUI.

Ручные команды:

```bash
# Пауза
gz service -s /world/kty_station/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'pause: true'

# Продолжить
gz service -s /world/kty_station/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'pause: false'

# Сброс
gz service -s /world/kty_station/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'reset { all: true }'
```

При сбросе мира симуляционное время возвращается назад. Контроллер распознаёт
этот переход, очищает локальное состояние и запускает новый цикл.

## Чистая сборка

После изменения сообщений, entry points или SDF выполняйте:

```bash
cd ~/singulator_digital_twin
bash ./scripts/build_kty_station.sh
```

Скрипт очищает и пересобирает вместе:

```text
singulator_interfaces
kty_station_sim
```

## Запуск

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/run_kty_station.sh
```

Параметризованный запуск:

```bash
bash ./scripts/run_kty_station.sh \
  vibration_frequency_hz:=30.0 \
  vibration_amplitude_m:=0.001 \
  product_rate_products_per_s:=1.0 \
  seed:=42
```

Окно `rqt_image_view` включается отдельно:

```bash
bash ./scripts/run_kty_station.sh vision_gui:=true
```

Остановка:

```bash
bash ./scripts/stop_kty_station.sh
```

## Диагностика

В новом терминале:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/check_kty_station.sh
```

Скрипт проверяет:

- типы `KtyGroundTruthArray` и `KtyStationState`;
- наличие контроллера, спавнера и драйвера вибрации;
- `/world/kty_station/control`;
- `/world/kty_station/set_pose`;
- `/gui/camera/view_control`;
- `/kty/product_spawner/enabled`;
- состояние автомата и реестр товаров;
- модели `kty_XXXXXX` и `kty_product_*` в Gazebo.

Проверка состояния:

```bash
ros2 topic echo /kty/station/state
```

Проверка команды спавнеру:

```bash
ros2 topic echo /kty/product_spawner/enabled --once
```

Проверка пользовательского типа и реестра:

```bash
ros2 interface show singulator_interfaces/msg/KtyGroundTruthArray
ros2 topic echo /kty/ground_truth/registry --once
ros2 topic echo /kty/ground_truth/registry_json --once
```

Полный runbook: [`KTY_RUNTIME_V3.md`](KTY_RUNTIME_V3.md).

## Машинное зрение

Входы:

```text
/kty/camera/image         sensor_msgs/msg/Image
/kty/camera/depth_image   sensor_msgs/msg/Image
/kty/camera/camera_info   sensor_msgs/msg/CameraInfo
```

Выход:

```text
/kty/perception/contours
singulator_interfaces/msg/KtyProductContourArray
```

Для каждого видимого товара публикуются устойчивый `track_id`, полигон верхней
области, высота, площадь, confidence и доступность четырёх боковых направлений.
Ground truth используется для верификации и метрик, но не является входом
детектора.

## Контролируемые неисправности

`kty_safety_monitor` обрабатывает:

- товар выпал из КТЯ;
- товар завис на лотке;
- КТЯ отсутствует или смещён при доступной диагностике поз;
- превышена масса `35 кг`;
- потерян RGB-D обзор;
- товар продолжает двигаться после остановки вибрации.

Единичная потеря RGB-D кадра является предупреждением. Контроллер переводит
станцию в `FAULT` после трёх последовательных неудачных проверок.

## Метрики

Результаты цикла сохраняются в:

```text
/tmp/kty_station_metrics/cycle_XXXXXX/
```

Файлы:

- `summary.json`;
- `timeseries.csv`;
- `product_displacements.csv`.

Оцениваются коэффициент заполнения, максимальная высота, пустоты, время
успокоения, перемещения товаров и точность машинного зрения.

## Ограничения

- картон, полиуретан и товары моделируются жёсткими телами;
- деформация стенок КТЯ и повреждение товара не моделируются;
- коэффициенты трения и параметры вибратора требуют экспериментальной
  калибровки;
- статический GitHub CI не заменяет запуск контактной физики и GUI на Ubuntu
  24.04 с ROS 2 Jazzy и Gazebo Harmonic.
