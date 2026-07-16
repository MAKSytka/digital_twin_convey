# GitHub и подготовка материалов

## Рекомендуемая модель работы

- `main` — только собирающаяся и проверенная версия;
- `develop` — объединение текущих изменений;
- `feature/singulation-algorithm` — алгоритм второго программиста;
- `feature/vision-adapter` — интеграция машинного зрения;
- `fix/...` — локальные исправления.

Перед объединением ветки:

```bash
python3 tools/validate_project.py
colcon build --symlink-install
```

## Первый push

```bash
cd ~/singulator_digital_twin
git init
git branch -M main
git add .
git status
git commit -m "Initial handoff of 14x4 singulator digital twin"
git remote add origin <repository-url>
git push -u origin main
```

Стабильная точка интеграции:

```bash
git tag -a v0.1-handoff -m "Stable integration baseline"
git push origin v0.1-handoff
```

## Что хранить в GitHub

- исходный код;
- launch- и YAML-файлы;
- SDF-миры;
- ROS-интерфейсы;
- документацию;
- небольшие примеры JSON;
- отчёт и презентацию в итоговой версии проекта.

## Что хранить в S3 или другом облаке

- STEP/CAD-файлы;
- полную видеодемонстрацию;
- большие датасеты;
- Windows `.exe`;
- тяжёлые инженерные проекты;
- архивы результатов.

Каждая ссылка описывается в `external_artifacts/README.md` вместе с версией и контрольной суммой.

## Рекомендуемый состав финального репозитория

```text
README.md
src/
scripts/
docs/
examples/
tools/
project_materials/
external_artifacts/
LICENSE
.gitignore
```

В `project_materials/` перед сдачей помещаются итоговый отчёт и презентация либо их явно названные версии. Корневой README должен содержать ссылки на них и на видеодемонстрацию.
