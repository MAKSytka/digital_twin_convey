# Аудит и очистка репозитория перед сдачей

Этот документ фиксирует безопасный порядок очистки. История разработки содержит много stacked-, fix- и Codex-ветвей; удалять их только по названию нельзя.

## Принятый источник истины

- базовая ветка: `main`;
- ветка финального оформления: `chore/final-project-packaging`;
- принятый КТЯ runtime: v18, слит PR #47;
- принятый инфид-сепаратор: изменения PR #18–#23 и документация PR #29;
- актуальный roller-сценарий: 18×4, 72 команды;
- рабочее трение матрицы: `mu=0.8`, `mu2=0.2`;
- штатное ограничение ускорения: 6 м/с²;
- воспроизводимый seed: 42.

## Что уже установлено по истории PR

На момент аудита открытых PR нет. PR #47 объединяет принятый KTY runtime v18 и был слит в `main`. Из-за squash/merge большая feature-ветвь может продолжать показывать множество «уникальных» коммитов, хотя её итоговое дерево уже перенесено в `main`. Поэтому одного `git rev-list main..branch` недостаточно: нужно также сравнивать дерево и изменённые файлы.

### Группа A — ветви слитых PR, кандидаты на удаление

После успешного финального runtime-теста и merge этой ветки можно удалить ветви, связанные со слитыми PR:

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

Перед удалением каждой ветви всё равно выполняется `scripts/audit_repository.sh`.

### Группа B — закрытые неслитые или заменённые stacked-ветви

Эти ветви не являются источником актуального runtime. Они сохранялись как промежуточные этапы и после проверки могут быть удалены:

```text
chore/kty-vision-mainline-v5
feat/kty-classical-3d-perception-v6
feat/kty-physical-mechatronics-v5
feat/kty-vision-dashboard-v3
fix/kty-clock-gate-v4
fix/kty-runtime-v2
archive/kty-runtime-v3-broken
```

Их функциональность либо вошла в PR #47, либо была заменена более поздней реализацией.

### Группа C — автоматические и дублирующие Codex-ветви

Многочисленные ветви вида ниже относятся к повторным PR одной и той же доработки:

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

После подтверждения, что связанные PR закрыты и итоговые изменения присутствуют в `main`, эти ветви можно удалить одной группой.

### Группа D — separator GUI эксперименты

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

Сохранять их для жюри не нужно. Перед удалением проверить только отсутствие уникальных файлов, которых нет в принятом PR #23.

## Автоматический read-only аудит

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

## Проверка содержимого ветви

Для спорной ветви выполнить:

```bash
git log --oneline main..origin/<branch>
git diff --stat main...origin/<branch>
git diff --name-status main...origin/<branch>
```

Для большой squash-ветви дополнительно сравнить конечное содержимое ключевых файлов:

```bash
git diff main origin/<branch> -- \
  README.md \
  docs \
  scripts \
  tools \
  src
```

Ветвь допустимо удалить, когда её принятый функционал уже есть в `main`, даже если SHA-коммиты не являются предками `main` из-за squash merge.

## Удаление ветвей

Сначала удалить локальную ветвь:

```bash
git branch -d <branch>
```

Если Git отказывается из-за squash merge, после ручного сравнения:

```bash
git branch -D <branch>
```

Удаление удалённой ветви:

```bash
git push origin --delete <branch>
```

Не удалять до завершения работы:

```text
main
chore/final-project-packaging
```

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

`.gitignore` предотвращает их повторное добавление. Наличие уже отслеживаемого файла проверяется командой:

```bash
git ls-files | grep -E '(^|/)(build|install|log|__pycache__)(/|$)|\.before_|\.backup$|\.bak$|^src_before_|^scripts_before_'
```

## Обязательные проверки перед merge

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash ./scripts/build.sh
source install/setup.bash

python3 tools/validate_project.py
python3 tools/validate_release.py
python3 tools/validate_separator_demo.py
python3 tools/validate_kty_runtime_v18.py
python3 tools/test_v7_logic.py
bash ./scripts/check_v7_control.sh
bash ./scripts/check_kty_runtime_v18.sh
```

## Порядок финализации

1. Выполнить read-only аудит и сохранить отчёт локально.
2. Завершить документацию и валидаторы.
3. Выполнить чистую сборку и runtime-тесты на целевом ПК.
4. Открыть один финальный PR в `main`.
5. Выполнить squash merge.
6. Создать release tag и сохранить SHA, версии среды, seed и логи.
7. Удалить подтверждённые устаревшие ветви.
8. Выполнить `git fetch --prune` и повторный аудит.
