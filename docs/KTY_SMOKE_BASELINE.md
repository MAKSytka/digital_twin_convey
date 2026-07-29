# KTY smoke baseline v1

Этот сценарий является первым проверяемым этапом повторной разработки станции
КТЯ. Он намеренно не запускает старый автомат состояний, машинное зрение,
случайный спавнер, safety-monitor и метрики.

## Что проверяется

- Gazebo Harmonic открывает отдельный мир `kty_station_smoke`;
- в центре платформы всегда виден статический открытый КТЯ;
- модель называется `kty_smoke_container`;
- КТЯ имеет внутренний размер 600×400×400 мм и стенку 3 мм;
- Play, Pause, Step и Reset работают через стандартный `WorldControl`;
- обзорная камера поддерживает pan, orbit и zoom мышью;
- ROS-пакет запускает независимый wall-clock heartbeat;
- сценарий не зависит от `/clock` и пользовательских сообщений КТЯ.

## Почему КТЯ пока статический

На этом этапе нужно отделить ошибки загрузки мира и GUI от ошибок транспорта,
контактной физики и автомата. Статическая модель гарантированно существует после
старта и после Reset. Динамический create / set_pose / remove будет добавлен в
следующем PR только после приёмки этого сценария.

## Сборка

```bash
cd ~/singulator_digital_twin
chmod +x scripts/*kty_smoke.sh tools/validate_kty_smoke.py
bash ./scripts/build_kty_smoke.sh
```

Статическая проверка без Gazebo:

```bash
python3 tools/validate_kty_smoke.py
```

## Запуск

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/run_kty_smoke.sh
```

Остановка:

```bash
bash ./scripts/stop_kty_smoke.sh
```

## Проверка во втором терминале

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/check_kty_smoke.sh
```

Ожидается:

```text
OK node: /kty_smoke_heartbeat
OK Gazebo service: /world/kty_station_smoke/control
OK Gazebo service: /world/kty_station_smoke/create
OK Gazebo service: /world/kty_station_smoke/remove
OK Gazebo service: /world/kty_station_smoke/set_pose
OK Gazebo service: /gui/camera/view_control
OK model: kty_smoke_container
KTY smoke diagnostics: OK
```

Heartbeat можно прочитать отдельно:

```bash
ros2 topic echo /kty/smoke/heartbeat --once
```

Пример:

```text
data: '{"status":"alive","sequence":3,"wall_uptime_s":3.002,"expected_world":"kty_station_smoke","expected_model":"kty_smoke_container"}'
```

## Управление камерой

- средняя кнопка мыши и перемещение — вращение;
- левая кнопка мыши и перемещение — панорамирование;
- колесо или правая кнопка и перемещение — масштабирование.

## Критерий приёмки этапа

Этап считается пройденным только после подтверждения на целевой Ubuntu:

1. КТЯ виден сразу после запуска;
2. `gz model --list` содержит `kty_smoke_container`;
3. heartbeat приходит не менее 30 секунд;
4. камера вращается мышью;
5. Play/Pause/Step/Reset работают из GUI;
6. после Reset статический КТЯ остаётся в мире;
7. `scripts/check_kty_smoke.sh` завершается без ошибок.

После этого следующий PR реализует только ручной жизненный цикл одной
динамической модели: `create -> set_pose -> remove`.
