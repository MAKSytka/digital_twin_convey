# Аудит и очистка репозитория после релиза

Этот документ фиксирует завершённую финализацию репозитория и порядок
безопасного обслуживания после выпуска `v1.0.0`.

## Зафиксированный релиз

- основная ветка: `main`;
- release tag: `v1.0.0`;
- release commit: `cbe7acb92ad3ce1819398db91f3eb60abfebe2b5`;
- матрица: 18×4, 72 команды;
- рабочее трение матрицы: `mu=0.8`, `mu2=0.2`;
- ограничение ускорения: 6 м/с²;
- инфид-сепаратор: 11 сплошных роликов, шаг 150 мм, отверстие 100 мм;
- станция КТЯ: runtime v18 со сходящимися направляющими лотка;
- воспроизводимый seed: 42.

Все три runtime-модуля приняты на целевой машине. Подробности находятся в
[Runtime acceptance](RUNTIME_ACCEPTANCE.md).

## Выполненная очистка

После squash merge финального PR и создания `v1.0.0`:

- удалена release-ветка `chore/final-project-packaging`;
- удалены исторические `feature`, `fix`, `experiment`, `archive` и Codex-ветви;
- на GitHub оставлена только ветка `main`;
- проверен чистый клон непосредственно из тега `v1.0.0`;
- статический release-набор выполнен из чистого клона;
- история перед очисткой сохранена локально в Git bundle;
- build-, install-, log- и backup-артефакты не отслеживаются Git.

## Read-only аудит

```bash
git fetch --all --tags --prune
bash scripts/audit_repository.sh
```

Скрипт ничего не удаляет и создаёт локальный каталог `release_audit/`:

```text
branches.tsv
tracked_candidates.txt
historical_branch_references.txt
outdated_matrix_references.txt
```

Для очищенного репозитория ожидается:

```text
remote branches: origin/main
branches.tsv: только строка заголовка
tracked_candidates.txt: только заголовок
historical_branch_references.txt: только заголовок
outdated_matrix_references.txt: только заголовок
```

## Проверка релизного тега

Аннотированный тег имеет собственный объект SHA. Целевой commit проверяется с
оператором разыменования:

```bash
git show-ref --tags v1.0.0
git rev-parse 'v1.0.0^{}'
git describe --tags --exact-match 'v1.0.0^{}'
```

Ожидаемый commit:

```text
cbe7acb92ad3ce1819398db91f3eb60abfebe2b5
```

## Чистый клон релиза

```bash
cd ~
rm -rf digital_twin_convey_release_test

git clone \
  --branch v1.0.0 \
  --single-branch \
  https://github.com/MAKSytka/digital_twin_convey.git \
  digital_twin_convey_release_test

cd digital_twin_convey_release_test

git status --short
git describe --tags --exact-match HEAD
bash scripts/run_release_checks.sh
```

Для полной проверки ROS-сборки:

```bash
rm -rf build install log
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH
unset COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -r -y
bash scripts/build.sh
source install/setup.bash
```

## Резервный Git bundle

Перед удалением исторических ссылок создавался локальный архив:

```bash
mkdir -p ~/git_backups
bundle="$HOME/git_backups/digital_twin_before_final_branch_cleanup_$(date +%Y%m%d-%H%M%S).bundle"

git bundle create "$bundle" --all
git bundle verify "$bundle"
```

Восстановление:

```bash
git clone /path/to/archive.bundle restored_repository
```

Bundle не должен помещаться в этот репозиторий или публиковаться как release
asset без отдельной необходимости.

## Локальные stash

Stash не публикуется на GitHub и не влияет на чистоту удалённого репозитория.
Перед удалением старых stash рекомендуется сохранить каждый diff:

```bash
mkdir -p ~/git_backups/digital_twin_stashes

git stash list --format='%gd' |
while IFS= read -r ref; do
  safe_name="$(printf '%s' "$ref" | tr '@{}:/' '_')"
  git stash show --binary -p "$ref" \
    > "$HOME/git_backups/digital_twin_stashes/${safe_name}.patch"
done
```

Проверка созданных файлов:

```bash
find ~/git_backups/digital_twin_stashes \
  -maxdepth 1 -type f -name '*.patch' -size +0c -print
```

Только после проверки архива:

```bash
git stash clear
```

## Правила дальнейшей разработки

1. Новая работа начинается от актуального `main`.
2. Одна задача — одна короткоживущая ветка.
3. Изменения в `main` попадают через PR.
4. Перед merge выполняется `bash scripts/run_release_checks.sh`.
5. Runtime-изменения требуют обновления `docs/RUNTIME_ACCEPTANCE.md`.
6. После merge рабочая ветка удаляется.
7. Release-теги не перемещаются и не перезаписываются.
8. Исторические параметры должны быть явно помечены как исторические.

## Текущее ожидаемое состояние

```bash
git status --short
git branch --list
git branch -r
git tag -n
```

Ожидается:

```text
worktree: clean
local branches: main
remote branches: origin/main
tag: v1.0.0
```
