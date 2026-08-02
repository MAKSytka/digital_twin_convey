# Диагностика проблем

Этот файл относится к актуальной конфигурации репозитория: матрица 18×4,
инфид-сепаратор со сплошными роликами и станция КТЯ runtime v18.
Исторические runtime-файлы и промежуточные версии не следует запускать как
основные сценарии.

## Рекомендуемый порядок диагностики

Сначала выполните статические проверки:

```bash
cd ~/singulator_digital_twin
bash scripts/run_release_checks.sh
```

Затем выполните чистую сборку:

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

После этого запускайте только один демонстрационный сценарий за раз.

## `AMENT_TRACE_SETUP_FILES: unbound variable`

ROS 2 setup-скрипты могут обращаться к ещё не определённым переменным. Если
проектный Bash-скрипт работает с `set -u`, временно отключите `nounset`:

```bash
set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u
```

## `install/setup.bash` отсутствует

Каталоги `build/`, `install/` и `log/` создаются только после успешной сборки и
не хранятся в Git. Исправьте первую ошибку `colcon`, затем повторите:

```bash
source /opt/ros/jazzy/setup.bash
bash scripts/build.sh
source install/setup.bash
```

## `colcon` находит одинаковые пакеты

В workspace, вероятно, лежит резервная копия исходников с повторяющимися
`package.xml`.

```bash
find . -name package.xml -print | sort
```

В рабочем дереве не должно быть каталогов `src_before_*`, `scripts_before_*`,
а также копий пакетов внутри `build/` или случайных архивных директорий.

## ROS executable или пакет не найден

```bash
source /opt/ros/jazzy/setup.bash
source ~/singulator_digital_twin/install/setup.bash

ros2 pkg executables singulator_sim
ros2 pkg executables singulator_control
ros2 pkg executables kty_station_sim
```

После изменения `setup.py`, `CMakeLists.txt`, сообщений или launch-файлов
пересоберите затронутые пакеты. При сомнениях используйте чистую сборку.

## `invalid message type` для `MatrixCommand`

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select singulator_interfaces
source install/setup.bash
ros2 interface show singulator_interfaces/msg/MatrixCommand
```

Каждый новый терминал должен source-ить ROS 2 и текущий workspace.

## Матрица не запускается или коробки не создаются

Основной запуск:

```bash
bash scripts/run_roller_demo.sh
```

Текущая матрица содержит 18×4 = 72 ячейки. Некоторые внутренние имена файлов и
мира сохраняют историческую строку `matrix_14x4_stream`; это имя ресурса, а не
фактическое число строк.

Проверка `/clock`, массива наблюдений и команд:

```bash
ros2 topic hz /clock
ros2 topic echo /singulator/boxes --once
ros2 topic echo /singulator/matrix/command --once
```

Проверка сервиса создания моделей:

```bash
gz service -l | grep create_multiple
```

Если спавнер не работает, проверьте, что launch запущен со включённым спавнером
и что используется один и тот же `seed` в повторяемом тесте.

## Часть строк матрицы не получает команды

Проверьте крайние ячейки, включая строки 14–17:

```bash
ros2 topic info /singulator/cell/r00_c00/cmd_vel
ros2 topic info /singulator/cell/r17_c03/cmd_vel
```

Затем выполните структурную проверку текущего контура:

```bash
bash scripts/check_v7_control.sh
```

Название скрипта историческое; release-проверка использует его для актуальной
матрицы 18×4. Не удаляйте и не переименовывайте его без синхронного обновления
`run_release_checks.sh` и валидаторов.

## GUI машинного зрения матрицы пустой

```bash
ros2 topic hz /singulator/camera/image_raw
ros2 topic hz /singulator/perception/debug_image
ros2 topic echo /singulator/boxes --once
bash scripts/check_vision.sh
```

Открытие изображения:

```bash
ros2 run rqt_image_view rqt_image_view \
  /singulator/perception/debug_image
```

## Инфид-сепаратор не разделяет товары

Запуск:

```bash
ros2 launch singulator_bringup \
  infeed_size_separator_demo.launch.py \
  seed:=42
```

Статическая проверка геометрии и параметров:

```bash
python3 tools/validate_separator_demo.py
```

Для приёмочного прогона используйте конечную серию товаров и контролируйте
счётчики верхней ветви, нижней ветви и удаления моделей.

## Станция КТЯ не запускается

Используйте только публичные release-скрипты:

```bash
bash scripts/build_kty_perception_3d.sh
source install/setup.bash
bash scripts/run_kty_perception_3d.sh
```

Проверка текущего runtime:

```bash
bash scripts/check_kty_runtime_v18.sh
```

Не запускайте напрямую промежуточные `mechatronics_cycle_v*` и
`kty_mechatronics_v*.launch.py`: часть этих файлов сохранена только ради
совместимости старых валидаторов и должна быть удалена после консолидации.

## Команды заданы правильно, но движение визуально медленнее

Скорости задаются в единицах симуляционного времени. При Real Time Factor ниже
1.0 модель визуально движется медленнее относительно обычных часов, хотя
физическая скорость внутри симуляции остаётся заданной.

Не компенсируйте низкий RTF увеличением скорости конвейера. Сначала уменьшите
число динамических моделей, закройте лишние GUI-панели и проверьте нагрузку.

## Real Time Factor сильно падает

Основные причины:

- большое число динамических товаров;
- высокая частота контактов;
- одновременно открытые Gazebo GUI и окна машинного зрения;
- частые операции создания и удаления моделей;
- параллельно запущенные демонстрационные сценарии.

Практические меры:

```bash
pkill -f 'gz sim' || true
pkill -f 'ros2 launch' || true
pkill -f 'ros2 run' || true
```

После остановки старых процессов запустите только нужную демонстрацию. Для
повторяемых тестов используйте `seed:=42`.

## Проверка целостности репозитория

```bash
git status --short
bash scripts/audit_repository.sh
bash scripts/run_release_checks.sh
```

`audit_repository.sh` работает в режиме чтения и не удаляет файлы. Удаление
исторических runtime-файлов выполняется только в отдельной ветке после создания
Git bundle и после устранения всех ссылок из `setup.py`, launch-файлов,
скриптов сборки и GitHub Actions.
