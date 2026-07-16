
## v0.6-handoff

- Removed invalid `<buildtool_depend>ament_python</buildtool_depend>` entries from the three pure Python package manifests.
- Added a static manifest check so rosdep compatibility regressions are caught before handoff.

# Changelog

## v0.4.0 — uniform 2 m/s demo

- Fixed the demo matrix command from 0.5 m/s to 2.0 m/s.
- One `CONVEYOR_SPEED_MPS` value now drives infeed, all 56 matrix cells, and outfeed.
- Matrix controller starts before box spawning so belts reach the target speed first.
- Added `scripts/check_speeds.sh` and static validation of speed defaults.

## 0.2.0 — handoff package

- Added one-command full-stream launch `matrix_stream.launch.py`.
- Split ROS–Gazebo bridge into four matrix groups plus auxiliary topics.
- Added `/clock`, infeed and outfeed bridge configuration.
- Added `aux_conveyor_controller`.
- Added installed `cleanup_passed_boxes` executable.
- Documented the row-major 56-speed contract.
- Corrected `matrix.yaml` to the current 5.30 × 0.76 m simulation geometry.
- Added setup, build, run, validation and troubleshooting scripts.
- Removed generated directories, caches and historical backup copies.
- Marked `/singulator/boxes` and real measured `MatrixState` as future work.

## 0.1.0 — original working baseline

- Full 14×4 matrix with 56 TrackController cells.
- Calibrated conveyor friction `mu=0.8`, `mu2=0.2`.
- Random box-wave spawner and multiple size/orientation patterns.
- Infeed/outfeed conveyors and cleanup script.

## v5

- Исправлен аргумент `rosdep install`: заменён неподдерживаемый `--yes` на стандартный `-y`.
- Исправление внесено в `scripts/build.sh` и `scripts/setup_dependencies.sh`.
