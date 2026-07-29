# KTY runtime v3: транспорт, GUI и диагностика

## Что исправлено

Runtime v3 устраняет общую причину трёх наблюдавшихся симптомов:

- КТЯ появлялся, но не перемещался к платформе;
- автомат не переходил в `LOAD`, поэтому товары не спавнились;
- управление миром и обзорной камерой Gazebo было неполным.

В предыдущем runtime движение КТЯ зависело от динамически добавленного
`VelocityControl` и ROS–Gazebo-моста `Twist`. При живом потоке поз неподвижный
КТЯ не проходил проверку центрирования, поэтому автомат оставался в
`POSITION_KTY` до аварии.

## Транспорт КТЯ

В runtime v3 подвод и отвод выполняются короткими траекториями через сервис
Gazebo:

```text
/world/kty_station/set_pose
gz.msgs.Pose -> gz.msgs.Boolean
```

Подвод интерполируется от `x=-1.30 м` до `x=0` за `2 с`. Отвод выполняется от
`x=0` до выходной позиции за `1 с`. Команды формируются по симуляционному
времени, поэтому при паузе мира движение также останавливается.

Между транспортными фазами КТЯ остаётся обычным динамическим телом. Постоянный
`VelocityControl` удалён: тара реагирует на гравитацию, контакт с платформой,
вибрацию и падающие товары.

Основные параметры:

```yaml
transport_update_period_s: 0.05
transport_position_tolerance_m: 0.005
transport_failure_limit: 8
world_reset_jump_threshold_s: 0.10
```

## Спавн товаров

После успешного подвода автомат проходит состояния:

```text
POSITION_KTY -> CLAMP -> LOAD -> VIBRATE
```

В `LOAD` и `VIBRATE` публикуется:

```text
/kty/product_spawner/enabled = true
```

После этого с частотой около `1 товар/с` создаются модели вида:

```text
kty_product_c000001_p000001
```

## Обзорная камера

В GUI мира явно загружаются:

```text
MinimalScene
GzSceneManager
InteractiveViewControl
CameraTracking
```

`InteractiveViewControl` обеспечивает мышью панорамирование, вращение и
масштабирование. Фиксированная RGB-D камера машинного зрения от обзорной камеры
не зависит.

Управление:

- левая кнопка мыши и перемещение — панорамирование;
- средняя кнопка мыши и перемещение — вращение;
- `Shift` + левая кнопка — альтернативное вращение;
- колесо или правая кнопка и перемещение — масштабирование.

## Play, Pause, Step и Reset

`WorldControl` обращается непосредственно к:

```text
/world/kty_station/control
```

Режим `use_event=true` удалён. Поэтому панель должна управлять сервером мира, а
не только локальными GUI-событиями.

Ручные эквиваленты:

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

# Сброс мира
gz service -s /world/kty_station/control \
  --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'reset { all: true }'
```

При сбросе симуляционное время перескакивает назад. Контроллер распознаёт это,
очищает локальное состояние и начинает новый цикл КТЯ.

## Сборка и запуск

```bash
cd ~/singulator_digital_twin
bash ./scripts/build_kty_station.sh
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/run_kty_station.sh
```

Во втором терминале:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/check_kty_station.sh
```

Диагностический скрипт проверяет:

- пользовательские ROS-сообщения;
- контроллер, спавнер и драйвер вибрации;
- `/world/kty_station/control`;
- `/world/kty_station/set_pose`;
- `/gui/camera/view_control`;
- команду включения спавнера;
- состояние автомата и модели Gazebo.

## Ограничение

GitHub CI проверяет Python-синтаксис, SDF/XML, shell-скрипты и согласованность
файлов. Полный тест физики, GUI-плагинов и бинарных систем Gazebo выполняется на
целевой Ubuntu 24.04 с ROS 2 Jazzy и Gazebo Harmonic.
