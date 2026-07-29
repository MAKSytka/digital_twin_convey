# KTY flow stage 2: движение, загрузка и despawn

Этот сценарий является вторым проверяемым этапом повторной разработки станции
КТЯ. Он строится поверх принятого smoke baseline и добавляет только жизненный
цикл тары и товаров. Машинное зрение, вибрация, safety и метрики пока не
запускаются.

## Цикл

```text
WAIT_SERVICES
  -> SPAWN_KTY
  -> APPROACH
  -> LOAD
  -> SETTLE
  -> OUTFEED
  -> OUTFEED_HOLD
  -> DESPAWN
  -> COMPLETE
```

По умолчанию выполняется один цикл:

1. создаётся динамический открытый КТЯ `kty_flow_container`;
2. КТЯ перемещается от `x=-1,25 м` до центра платформы за `3 с`;
3. на верхней части лотка последовательно создаются шесть фиксированных товаров;
4. товары скользят по лотку с пониженным трением и падают внутрь КТЯ;
5. контроллер читает `/world/kty_flow/pose/info` и ждёт, пока центры всех товаров
   окажутся внутри объёма тары;
6. положение КТЯ и фактические положения товаров фиксируются;
7. загруженная группа плавно перемещается до `x=1,35 м`;
8. после короткой визуальной паузы товары и КТЯ удаляются;
9. `/kty/flow/state` сохраняет состояние `COMPLETE`.

## Почему движение выполняется через set_pose

На этом этапе проверяется жизненный цикл и геометрия, а не калибровка
контактного привода. Сервис Gazebo UserCommands
`/world/kty_flow/set_pose` уже доступен в принятом smoke baseline.

Во время подвода перемещается один КТЯ. После загрузки контроллер читает
фактические позы упавших товаров и перемещает КТЯ вместе с ними с одинаковым
смещением по X. Благодаря этому содержимое визуально остаётся в таре во время
отвода, а транспорт не зависит от неподтверждённого TrackController.

Контактный привод конвейеров будет возвращён отдельным этапом после успешной
приёмки этого детерминированного цикла.

## Сборка

```bash
cd ~/singulator_digital_twin
chmod +x scripts/*kty_flow.sh tools/validate_kty_flow.py
python3 tools/validate_kty_flow.py
bash ./scripts/build_kty_flow.sh
```

## Запуск

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/run_kty_flow.sh
```

Непрерывное повторение:

```bash
bash ./scripts/run_kty_flow.sh auto_repeat:=true
```

Более медленная визуальная проверка:

```bash
bash ./scripts/run_kty_flow.sh \
  approach_duration_s:=5.0 \
  product_spawn_interval_s:=1.5 \
  outfeed_duration_s:=5.0
```

## Диагностика

Во втором терминале:

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/check_kty_flow.sh
```

Скрипт ждёт до 70 секунд и требует:

- существование `/kty_flow_cycle`;
- доступность `create`, `set_pose` и `remove`;
- создание заданного числа товаров;
- нахождение всех товаров внутри КТЯ;
- успешный отвод;
- удаление `product_count + 1` моделей;
- отсутствие КТЯ и товаров после `COMPLETE`.

Текущее состояние:

```bash
ros2 topic echo /kty/flow/state --once
```

Heartbeat:

```bash
ros2 topic echo /kty/flow/heartbeat --once
```

Повтор одного цикла после `COMPLETE` или `ERROR`:

```bash
ros2 service call /kty/flow/restart std_srvs/srv/Trigger '{}'
```

## Имена динамических моделей

```text
kty_flow_container
kty_flow_product_01
kty_flow_product_02
...
kty_flow_product_06
```

Во время `LOAD`, `SETTLE`, `OUTFEED` и `OUTFEED_HOLD` их можно проверить:

```bash
gz model --list | grep -E 'kty_flow_container|kty_flow_product_'
```

После `COMPLETE` команда не должна ничего возвращать.

## Ограничения этапа

- транспорт задаётся позами, а не трением ленты;
- во время отвода товары перемещаются как зарегистрированная группа;
- используется фиксированный набор из шести небольших товаров;
- контроллер работает по wall time и не зависит от `/clock`;
- вибрация, RGB-D, случайные размеры, safety и метрики отключены.

## Критерии приёмки

1. пустой КТЯ виден на входе и плавно приходит на активную позицию;
2. все товары появляются на верхней части лотка;
3. товары скользят и падают внутрь КТЯ;
4. `inside_products` достигает `product_count`;
5. загруженный КТЯ перемещается на выход вместе с товарами;
6. после `DESPAWN` все динамические модели исчезают;
7. `scripts/check_kty_flow.sh` завершается строкой
   `KTY flow diagnostics: OK`.
