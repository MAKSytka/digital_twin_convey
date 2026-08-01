# Аудит и очистка репозитория перед сдачей

Этот документ задаёт безопасный порядок финализации. История разработки содержит много stacked-, fix-, experiment- и Codex-ветвей, поэтому удалять ветви только по названию нельзя.

## Принятый источник истины

- базовая ветка: `main`;
- ветка финального оформления: `chore/final-project-packaging`;
- матрица: 18×4, 72 команды;
- рабочее трение матрицы: `mu=0.8`, `mu2=0.2`;
- ограничение ускорения: 6 м/с²;
- инфид-сепаратор: 11 сплошных роликов, шаг 150 мм, отверстие 100 мм;
- станция КТЯ: runtime v18 со сходящимися направляющими лотка;
- воспроизводимый seed: 42.

Все три runtime-модуля приняты на целевой машине. Подробности зафиксированы в `docs/RUNTIME_ACCEPTANCE.md`.

## Текущее состояние финализации

Ветка `chore/final-project-packaging` должна быть слита в `main` одним финальным PR. До merge запрещено удалять:

```text
main
chore/final-project-packaging
```

После squash merge источником истины становится `main`, затем создаётся release tag и только после этого удаляются исторические ветви.

## Read-only аудит

```bash
git fetch --all --prune
chmod +x scripts/audit_repository.sh
bash scripts/audit_repository.sh
```

Скрипт ничего не удаляет и создаёт локальный каталог `release_audit/`:

```text
branches.tsv
tracked_candidates.txt
historical_branch_references.txt
outdated_matrix_references.txt
```

`branches.tsv` содержит число уникальных коммитов, равенство деревьев и предварительную подсказку. После squash merge значение `unique_commits > 0` само по себе не запрещает удаление ветви.

## Проверка отдельной ветви

```bash
git log --oneline main..origin/<branch>
git diff --stat main...origin/<branch>
git diff --name-status main...origin/<branch>
```

Для большой squash-ветви дополнительно сравнить итоговое содержимое:

```bash
git diff main origin/<branch> -- \
  README.md \
  docs \
  scripts \
  tools \
  src
```

Ветвь допустимо удалить, когда её полезная функциональность уже присутствует в `main`, даже если исходные SHA не стали предками `main` из-за squash merge.

## Группа A — ветви слитых PR

После финального merge и повторного аудита являются кандидатами на удаление:

```text
docs/cluster-singulation
docs/infeed-separator-readme-handoff
feature/v7-global-queue
feature-realistic-separator-flow
codex-infeed-size-separator-demo
fix-separator-transfers-despawn
fix-separator-white-screen-v2
feat/kty-station-simulation-v1
fix/kty-ament-python-metadata
fix/kty-runtime-and-camera-controls
feat/kty-smoke-baseline-v1
feat/kty-flow-cycle-v2
feat/kty-flow-smooth-v4
feat/kty-vision-dashboard-v4
fix/kty-mechatronics-runtime-v7
```

## Группа B — закрытые или заменённые stacked-ветви

```text
chore/kty-vision-mainline-v5
feat/kty-classical-3d-perception-v6
feat/kty-physical-mechatronics-v5
feat/kty-vision-dashboard-v3
fix/kty-clock-gate-v4
fix/kty-runtime-v2
archive/kty-runtime-v3-broken
```

Их функциональность либо вошла в runtime v18, либо была заменена более поздней реализацией.

## Группа C — автоматические Codex-ветви

```text
codex
codex-02xg29
codex-2zl945
codex-6brl8t
codex-6t6gog
codex-7ato9f
codex-dz8aaj
codex-gvbimc
codex-hycl95
9eldel-codex/-pr-24-26
dpztn1-codex/-pr-24-26
jqny7y-codex/-pr-24-26
labxqy-codex/-pr-24-26
uv3d8q-codex/-pr-24-26
w3xd7f-codex/-pr-24-26
```

Удалять группой только после подтверждения, что связанные PR закрыты и их итоговые изменения присутствуют в `main`.

## Группа D — эксперименты сепаратора

```text
fix-separator-white-screen
fix-separator-white-screen-actual
fix-separator-white-screen-final
fix-separator-white-screen-pr
fix-separator-white-screen-ready
fix-separator-white-screen-x
fix-separator-white-screen-y
fix-separator-product-motion
feature-separator-box-realism
```

Сохранять их для экспертной проверки не требуется. Перед удалением проверить отсутствие уникальных файлов, которых нет в финальном `main`.

## Прочие временные ветви

Ветвь `tmp-ignore` не является частью принятой архитектуры. Удалять её только после проверки через `scripts/audit_repository.sh` и сравнения с `main`.

## Файлы-кандидаты на удаление

```text
*.before_*
*.backup
*.bak
src_before_*/
scripts_before_*/
backup_src/
old_workspace/
build/
install/
log/
__pycache__/
```

Проверка уже отслеживаемых файлов:

```bash
git ls-files | grep -E \
  '(^|/)(build|install|log|__pycache__)(/|$)|\.before_|\.backup$|\.bak$|^src_before_|^scripts_before_'
```

## Обязательные проверки перед PR/merge

```bash
cd ~/singulator_digital_twin
rm -rf build install log
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y
bash ./scripts/build.sh
source install/setup.bash

bash ./scripts/run_release_checks.sh
```

Runtime уже принят, но перед merge нужно убедиться, что актуальный commit по-прежнему запускает:

```bash
bash ./scripts/run_roller_demo.sh
ros2 launch singulator_bringup infeed_size_separator_demo.launch.py seed:=42
bash ./scripts/run_kty_perception_3d.sh
```

## Создание финального PR

PR должен иметь:

```text
base: main
head: chore/final-project-packaging
```

В описании зафиксировать:

- матрицу 18×4 и 72 команды;
- `mu=0.8`, `mu2=0.2`;
- инфид-сепаратор с шагом 150 мм и отверстием 100 мм;
- KTY runtime v18 со сходящимися направляющими;
- результаты `run_release_checks.sh`;
- подтверждение трёх runtime-сценариев;
- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic;
- seed 42.

Рекомендуемый способ объединения — squash merge.

## Release tag

После merge:

```bash
git switch main
git pull --ff-only origin main
git tag -a v1.0.0 -m "Digital Twin Convey final demo release"
git push origin v1.0.0
```

Перед созданием тега сохранить:

```bash
git rev-parse HEAD
ros2 --version
gz sim --version
lsb_release -a
```

Также сохранить вывод валидаторов, результат четырёхциклового KTY-теста, RTF и скриншоты трёх демонстраций.

## Удаление ветвей

Локальная ветвь:

```bash
git branch -d <branch>
```

После ручного подтверждения squash-ветви:

```bash
git branch -D <branch>
```

Удалённая ветвь:

```bash
git push origin --delete <branch>
```

## Финальный порядок

1. Выполнить `scripts/audit_repository.sh`.
2. Выполнить чистую сборку и `scripts/run_release_checks.sh`.
3. Открыть финальный PR в `main`.
4. Выполнить squash merge.
5. Создать tag `v1.0.0`.
6. Удалить только подтверждённые исторические ветви.
7. Выполнить `git fetch --prune` и повторный аудит.
8. Проверить чистый клон из тега на новой директории.
