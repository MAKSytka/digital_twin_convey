# План удаления исторических runtime-файлов

Цель очистки — оставить в основной ветке только три публичных демонстрации и
один поддерживаемый runtime станции КТЯ. Удаление выполняется не по дате файла,
а только после переноса всех рабочих зависимостей в актуальные точки входа.

## Что сохраняется

Публичные сценарии:

```text
scripts/run_roller_demo.sh
src/singulator_bringup/launch/infeed_size_separator_demo.launch.py
scripts/run_kty_perception_3d.sh
```

Актуальное ядро КТЯ:

```text
mechatronics_cycle_v18.py
world_patch_v4.py
fill_estimator_v2.py
depth_perception_3d_v2.py
contour_recorder_3d.py
vision_dashboard_3d.py
```

Итоговая документация:

```text
README.md
docs/TROUBLESHOOTING.md
docs/KTY_RUNTIME_COMMANDS.md
docs/KTY_RUNTIME_V18_HANDOFF.md
docs/RUNTIME_ACCEPTANCE.md
```

## Почему нельзя начать с `git rm`

Текущий release исторически строился последовательными слоями. Публичный
KTY-запуск проходит через несколько launch-обёрток, а `setup.py`, build-скрипты,
валидаторы и GitHub Actions всё ещё требуют промежуточные версии. Удаление
одного старого файла без миграции ссылки может привести к одному из эффектов:

- пакет собирается, но launch падает только во время запуска;
- `ros2 run` больше не находит executable;
- статический release-validator падает из-за отсутствующего маркера;
- GitHub Actions остаётся привязанным к удалённому валидатору;
- документация продолжает отправлять эксперта к несуществующей команде.

Поэтому очистка разбита на четыре независимых коммита.

## Коммит 1. Восстановить диагностику

Восстановить `docs/TROUBLESHOOTING.md`, но не возвращать устаревшие параметры
14×4, 56 зон и прежние KTY-пороги. README уже содержит ссылку на этот файл, а
release-validator проверяет её наличие.

Проверка:

```bash
python3 tools/validate_release.py
```

## Коммит 2. Консолидировать текущий KTY runtime

Создать один неверсированный launch-файл:

```text
src/kty_station_sim/launch/kty_runtime.launch.py
```

В него должны быть перенесены:

- Gazebo contact-surface world;
- `GZ_SIM_SYSTEM_PLUGIN_PATH`;
- command bridge;
- JSON pose-registry bridge;
- RGB и depth bridge;
- `fill_estimator_v2`;
- `depth_perception_3d_v2`;
- `contour_recorder_3d`;
- `vision_dashboard_3d`;
- прямой запуск `mechatronics_cycle_v18`;
- release-пороги заполнения `0.82` и высоты `0.340 м`.

После этого:

1. `scripts/run_kty_perception_3d.sh` должен запускать только
   `kty_runtime.launch.py`.
2. В `setup.py` нужно удалить entry points старых `mechatronics_cycle_v*` и
   оставить один публичный alias актуального контроллера.
3. `scripts/build_kty_perception_3d.sh` должен проверять только актуальные
   executable и валидаторы.
4. `tools/validate_kty_runtime_v18.py` нужно перевести на новый launch и удалить
   проверки совместимости со старыми обёртками.
5. Документы должны использовать только публичные release-команды.

Рекомендуемый публичный executable:

```text
mechatronics_cycle = kty_station_sim.mechatronics_cycle_v18:main
```

Имя Python-модуля `mechatronics_cycle_v18.py` можно сохранить до отдельного
переименования: это текущая принятая версия, а не неподдерживаемый прототип.

## Коммит 3. Консолидировать проверки

Оставить обязательный набор:

```text
.github/workflows/release-static.yml
.github/workflows/kty-runtime-v18-static.yml
scripts/run_release_checks.sh
tools/validate_release.py
tools/validate_project.py
tools/validate_separator_demo.py
tools/validate_kty_runtime_v18.py
tools/validate_kty_classical_3d.py
tools/validate_kty_contact_surface.py
```

Перед удалением component-workflow их полезные проверки следует вызвать из
актуального KTY workflow. Старые workflow нельзя удалять раньше этого переноса.

## Коммит 4. Удалить исторические файлы

Сначала режим проверки:

```bash
bash scripts/prune_legacy_kty_runtime.sh --check
```

Скрипт показывает:

- какие кандидаты ещё отслеживаются Git;
- какие ссылки на их имена остаются в сохраняемых файлах;
- какие зависимости нужно перенести до удаления.

`--apply` разрешён только в отдельной ветке, при чистом рабочем дереве и при
отсутствии surviving references:

```bash
git switch main
git pull --ff-only
git switch -c chore/prune-legacy-kty-runtime

bash scripts/prune_legacy_kty_runtime.sh --check
bash scripts/prune_legacy_kty_runtime.sh --apply
```

Перед `git rm` скрипт создаёт и проверяет Git bundle в `~/git_backups`. После
удаления он запускает release checks. При ошибке удалённые файлы автоматически
восстанавливаются из текущего `HEAD`.

## Категории удаления

Сценарий удаляет:

- документацию промежуточных KTY-стадий;
- контроллеры до runtime v18;
- старые генераторы мира;
- первое поколение RGB-D узлов, заменённое `v2`/`3d` реализациями;
- launch-цепочку промежуточных версий;
- старые build/run/check-скрипты;
- валидаторы runtime v7 и v13–v17;
- GitHub Actions для неподдерживаемых runtime.

Полный машинно-читаемый список находится в массиве `LEGACY_PATHS` внутри
`scripts/prune_legacy_kty_runtime.sh`.

## Файлы, которые не удаляются только из-за старого имени

Следующие файлы нельзя удалять до отдельного переименования, потому что они
участвуют в актуальном release-контракте:

```text
src/singulator_gazebo/scripts/generate_matrix_14x4_stream_v2.py
src/singulator_gazebo/worlds/matrix_14x4_stream_v2.sdf
scripts/check_v7_control.sh
tools/test_v7_logic.py
```

Несмотря на исторические строки в имени, генератор создаёт текущую матрицу 18×4,
а `check_v7_control.sh` и `test_v7_logic.py` входят в release checks. Их следует
переименовывать отдельным коммитом с синхронным обновлением CMake, launch,
валидаторов и документации.

## Финальная приёмка

После применения очистки:

```bash
bash scripts/run_release_checks.sh

rm -rf build install log
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash scripts/build.sh
source install/setup.bash
```

Обязательные runtime-тесты:

```bash
bash scripts/run_roller_demo.sh

ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py \
  seed:=42

bash scripts/run_kty_perception_3d.sh
bash scripts/check_kty_runtime_v18.sh
```

Очистка считается безопасной только после прохождения статических проверок,
чистой ROS-сборки и ручного запуска всех трёх Gazebo-демонстраций на целевой
Ubuntu-машине.
