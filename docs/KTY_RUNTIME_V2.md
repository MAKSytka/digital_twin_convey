# KTY runtime v2: сборка, запуск и диагностика

## Какие неисправности устранены

Предыдущая реализация могла зависнуть в `POSITION_KTY`: переход к загрузке
требовал сообщения `/kty/world/poses`, а при отсутствии или несовместимости
моста `gz.msgs.Pose_V -> tf2_msgs/msg/TFMessage` КТЯ никогда не считался
установленным. Поскольку спавнер товаров включается только в `LOAD` и
`VIBRATE`, товары при этом тоже не создавались.

В runtime v2:

- в модель КТЯ встроен `VelocityControl` с темой `/kty/carrier/cmd_vel`;
- подвод за 2 секунды и отвод за 1 секунду имеют прямой привод модели;
- координата из `/kty/world/poses` используется для проверки, но отсутствие
  этой диагностики больше не блокирует цикл;
- safety-monitor отключает только проверки, которым действительно нужны позы,
  вместо немедленного аварийного останова всей станции;
- единичный пропуск кадра RGB-D является предупреждением, а контроллер выполняет
  до трёх проверок перед переходом в `FAULT`;
- отдельный узел `kty_vibration_driver` формирует команды с периодом 2 мс
  (500 Гц), поэтому частоты 20–50 Гц не дискретизируются 50-герцовым автоматом;
- добавлен диагностический JSON-топик стандартного типа
  `/kty/ground_truth/registry_json`.

## Почему ROS пишет `message type ... is invalid`

Сообщения КТЯ определены в пакете `singulator_interfaces`. Ошибка CLI означает,
что текущий терминал использует старую сборку пакета или в нём не выполнено:

```bash
source /opt/ros/jazzy/setup.bash
source ~/singulator_digital_twin/install/setup.bash
```

Важно: `scripts/run_kty_station.sh` загружает overlay только внутри своего
процесса. Это не изменяет окружение другого уже открытого терминала.

Для исключения смешивания старого Python-пакета и старого rosidl type support
добавлена целевая чистая сборка обоих пакетов:

```bash
cd ~/singulator_digital_twin
bash ./scripts/build_kty_station.sh
```

Скрипт удаляет только:

```text
build/singulator_interfaces
install/singulator_interfaces
build/kty_station_sim
install/kty_station_sim
```

После сборки он проверяет все шесть сообщений КТЯ командой
`ros2 interface show`.

### Важное замечание об `ament_python`

`ament_python` в этом пакете является значением тега
`<build_type>` внутри секции `<export>`. Это не отдельная зависимость `rosdep`
и не обязательный пакет `ros-jazzy-ament-python`. В `package.xml` не должно быть
`<buildtool_depend>ament_python</buildtool_depend>`, иначе `rosdep` пытается
разрешить несуществующий ключ.

Если старый вариант ветки выводит предложение установить
`ros-jazzy-ament-python`, обновите ветку:

```bash
git fetch origin
git switch fix/kty-ament-python-metadata
git pull --ff-only
```

После обновления повторите `bash ./scripts/build_kty_station.sh` без установки
дополнительного пакета.

## Запуск

Терминал 1:

```bash
cd ~/singulator_digital_twin
bash ./scripts/run_kty_station.sh
```

Для отдельного окна машинного зрения:

```bash
bash ./scripts/run_kty_station.sh vision_gui:=true
```

Терминал 2:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/check_kty_station.sh
```

## Ручные проверки

Проверка установленного пользовательского типа:

```bash
ros2 interface show singulator_interfaces/msg/KtyGroundTruthArray
```

Канонический типизированный реестр:

```bash
ros2 topic echo /kty/ground_truth/registry --once
```

Диагностический реестр стандартного типа:

```bash
ros2 topic echo /kty/ground_truth/registry_json --once
```

Состояние автомата:

```bash
ros2 topic echo /kty/station/state
```

Команда прямого привода КТЯ:

```bash
ros2 topic echo /kty/carrier/cmd_vel_filtered --once
```

Частота высокоскоростного драйвера:

```bash
ros2 topic hz /kty/carrier/cmd_vel_filtered
```

Ожидаемая частота — около 500 Гц при нормальной загрузке компьютера.

Список моделей:

```bash
gz model --list | grep -E 'kty_[0-9]{6}|kty_product_'
```

Во время загрузки должны появляться имена вида:

```text
kty_000001
kty_product_c000001_p000001
kty_product_c000001_p000002
```

## Ожидаемая последовательность

```text
WAIT_EMPTY_KTY
  -> POSITION_KTY
  -> CLAMP
  -> LOAD
  -> VIBRATE
  -> SETTLE
  -> SCAN
  -> VIBRATE или EJECT_PREP
  -> EJECT
  -> WAIT_EMPTY_KTY
```

После появления `LOAD` шторка открывается, а `product_spawner` получает
`enabled=true` и создаёт товар со средней интенсивностью 1 единица/с.

## Ограничение проверки PR

GitHub CI выполняет Python compile, XML/SDF-проверки, согласованность entry
points, bridge-конфигурации и shell syntax. Полная контактная физика и наличие
конкретных бинарных плагинов Gazebo проверяются только на целевой Ubuntu 24.04
с ROS 2 Jazzy и Gazebo Harmonic.
