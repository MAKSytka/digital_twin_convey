# Аудит и очистка репозитория перед сдачей

Документ фиксирует безопасный порядок очистки. Удаление ветвей и исторических файлов выполняется только после подтверждения, что их содержимое уже присутствует в `main` или не относится к принятому runtime.

## Принятый источник истины

- базовая ветка: `main`;
- ветка финального оформления: `chore/final-project-packaging`;
- принятый КТЯ runtime: v18, слит PR #47;
- принятый инфид-сепаратор: исправления до PR #29 включительно;
- актуальный roller-сценарий сингулятора: 18×4, 72 команды;
- коэффициенты трения матрицы: `mu=0.8`, `mu2=0.2`.

## Ветки-кандидаты на удаление

После merge финального PR можно удалить удалённые feature/fix-ветки, связанные с уже слитыми PR, если сравнение с `main` не показывает уникальных коммитов.

Высокоприоритетные группы:

```text
fix-separator-white-screen*
fix-separator-transfers-despawn
feature-realistic-separator-flow
fix/kty-runtime-v2
fix/kty-runtime-and-camera-controls
fix/kty-clock-gate-v4
feat/kty-smoke-baseline-v1
feat/kty-flow-*
feat/kty-vision-*
feat/kty-physical-mechatronics-v5
feat/kty-classical-3d-perception-v6
fix/kty-mechatronics-runtime-v7
```

Последняя ветка не удаляется до подтверждения, что PR #47 полностью перенёс её содержимое в `main` и в документации больше нет команд переключения на неё.

## Старые PR

Закрытые неслитые PR сохраняются как история решений, но не должны использоваться README или runbook как источник команд. Особое внимание:

- ранние нерабочие KTY runtime;
- stacked PR, которые были заменены PR #47;
- повторные PR перехода 14×4 → 18×4;
- rollback и экспериментальные ветки separator GUI.

Закрывать уже закрытые PR повторно не требуется. Для жюри важнее убрать ссылки на них из актуальной документации.

## Файлы-кандидаты на удаление

Удаляются только при наличии в git tree:

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

`.gitignore` уже запрещает повторное добавление этих категорий.

## Проверка перед удалением файла

1. Найти все ссылки на путь в launch, setup, CMake, package manifests, документации и CI.
2. Проверить, не является ли файл генератором актуального SDF.
3. Проверить, установлен ли он через `setup.py`, `CMakeLists.txt` или `install(DIRECTORY ...)`.
4. Запустить статический валидатор.
5. Выполнить чистую сборку.
6. Запустить соответствующий runtime.

## Обязательные проверки перед merge

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
bash ./scripts/build.sh
source install/setup.bash

python3 tools/validate_project.py
python3 tools/validate_separator_demo.py
python3 tools/validate_kty_runtime_v18.py
python3 tools/test_v7_logic.py
bash ./scripts/check_v7_control.sh
bash ./scripts/check_kty_runtime_v18.sh
```

## Критерий готовности к удалению ветви

Ветвь можно удалить, когда одновременно выполнены условия:

- связанный PR слит или явно заменён более новым PR;
- `git log main..branch` не содержит нужных уникальных изменений;
- README и docs не ссылаются на ветвь;
- актуальные launch и scripts находятся в `main`;
- runtime проверен из `main` или финальной ветки.

## Порядок финализации

1. Завершить документацию и валидаторы в `chore/final-project-packaging`.
2. Выполнить runtime-тесты на целевом ПК.
3. Открыть один финальный PR в `main`.
4. Выполнить squash merge.
5. Создать release tag.
6. Удалить подтверждённые устаревшие ветки.
7. Сохранить commit SHA, версии среды, seed и логи приёмки.
