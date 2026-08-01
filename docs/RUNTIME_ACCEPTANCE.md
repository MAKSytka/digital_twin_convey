# Runtime-приёмка финальной ветки

Этот документ фиксирует проверки, которые нельзя подтвердить только статическими валидаторами. Все прогоны выполняются на целевой Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic после чистой сборки ветки `chore/final-project-packaging`.

## Сводный статус

| Узел | Статические проверки | Runtime на финальной ветке | Статус |
|---|---|---|---|
| Матрица 18×4 и роликовое горлышко | пройдены | подтверждён 2026-08-01 | принято |
| Полноширинный инфид-сепаратор | пройдены | требуется финальный прогон | ожидает |
| Станция КТЯ, runtime v18 | пройдены | ранее принят runtime v18; требуется повторный прогон финальной ветки | ожидает |

## 1. Матрица сингуляризации — принято

Подтверждено на целевой машине:

- чистая сборка создаёт устанавливаемый мир 18×4;
- коробки переходят с инфид-ленты на первую строку матрицы;
- коробки проходят всю матрицу, роликовое горлышко и выходную ленту;
- камера Gazebo поддерживает orbit, pan и zoom;
- `singulation_controller` публикует `MatrixCommand` размером 72;
- `matrix_command_fanout` публикует индивидуальные команды ячеек;
- значения команд находятся в диапазоне 1,00–3,00 м/с;
- рабочая калибровка трения: `mu=0.8`, `mu2=0.2`;
- зазор между инфидом и первой строкой равен 20 мм.

Проверка интерфейса:

```bash
bash scripts/check_v7_control.sh
ros2 topic info /singulator/matrix/command -v
ros2 topic echo /singulator/matrix/command --once
```

## 2. Инфид-сепаратор — финальный прогон

Запуск:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py \
  spawn_mode:=finite \
  maximum_items:=200 \
  target_rate_boxes_per_sec:=4.0 \
  small_item_probability:=0.50 \
  seed:=42
```

Критерии приёмки:

- верхняя и нижняя ветви получают товары;
- мелкие товары проходят через отверстия 70×70 мм;
- крупные товары не проходят сквозь физические оси;
- на переходах 1 мм / 4 мм вниз нет устойчивого затора;
- деспавн работает на обеих ветвях;
- `remove_failures=0`;
- после конечной серии в мире не остаётся растущего хвоста моделей.

Статическая проверка:

```bash
python3 tools/validate_separator_demo.py
```

## 3. Станция КТЯ runtime v18 — финальный прогон

Запуск:

```bash
bash scripts/build_kty_perception_3d.sh
source install/setup.bash
bash scripts/run_kty_perception_3d.sh
```

Диагностика во втором терминале:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/check_kty_runtime_v18.sh
```

Критерии приёмки:

- не менее четырёх различных входов в `LOAD`;
- отсутствует состояние `ERROR`;
- `position_recovery_failures=0`;
- проходят состояния `CLOSE_GATE`, `COMPACT`, `EJECT_ACTIVE`, `DESPAWN_ACTIVE`, `POSITION_NEXT`, `VERIFY_READY`, `OPEN_GATE`;
- RGB и depth публикуются;
- `/kty/perception/contours`, `/kty/fill/state` и `/kty/flow/state` активны;
- dashboard обновляется;
- загруженная тара удаляется, следующая тара занимает активную позицию.

## 4. Финальный критерий merge

Перед слиянием в `main` должны одновременно выполняться:

```bash
bash scripts/run_release_checks.sh
```

и все три runtime-раздела этого документа должны иметь статус `принято`.

После merge выполняется удаление только тех исторических ветвей, которые отмечены в `docs/REPOSITORY_CLEANUP.md` как `safe-ancestor` или `safe-tree-duplicate`.
