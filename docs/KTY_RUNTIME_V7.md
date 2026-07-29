# KTY runtime v7

This stage corrects the first physical mechatronics runtime after target-machine testing.

## Confirmed stage-6 failures

- the hinged chute gate did not block products;
- roller links rotated around an incorrect joint frame and became skewed;
- a grouped `JointController` did not reliably drive all outfeed rollers;
- the loaded KTY failed with `did not clear active zone`;
- the 0.65 s feeder interval produced an uncontrolled pile;
- RGB-D plus dashboard load reduced RTF to about 0.15;
- the classical 3-D node disappeared after an unhandled image callback exception.

## Runtime-v7 changes

### Roller geometry

The source SDF is patched before Gazebo starts:

- complete roller links are no longer rotated;
- only cylinder collision and visual geometry is rotated onto the Y axle;
- each revolute joint is placed at the matching roller centre;
- each roller has its own `JointController` subscribing to the shared group topic;
- each clamp has its own position controller.

This removes orbital / diagonal roller motion and makes all contact surfaces receive a velocity command.

### Slide gate

The hinged mechanism is removed from the generated world.  The controller creates a static model named:

```text
kty_mech_chute_gate
```

when the chute must close and removes it when a verified empty KTY is ready.  The plate is 35 x 620 x 260 mm and overlaps the chute surface so small products cannot pass below it.

### Feeder and load reduction

```text
old interval: 0.65 s
new interval: 1.15 s
```

The default flow is 1.77 times slower.

Balanced sensor settings:

```text
RGB-D:       640 x 480 at 8 Hz
3-D process: 4 Hz
fill:        4 Hz
dashboard:   5 Hz
physics:     2 ms step / 500 Hz target
```

The dashboard window defaults to disabled for the first mechanics acceptance test.  Its ROS image topic remains available.

### Fill estimate

A 40 mm border is excluded from every KTY side.  Samples near the 400 mm wall top are rejected and the measured core volume is extrapolated to the complete 600 x 400 mm floor.  Output schema:

```text
kty_fill_state/v2
```

### 3-D process fault containment

`kty_classical_3d_perception_v2` processes at 4 Hz.  A bad frame no longer terminates the node.  Details are published in:

```text
/kty/perception/fault
```

Message fields are assigned explicitly for compatibility with Jazzy-generated Python interfaces.  OCCLUDED objects remain non-actionable.

## Build

```bash
cd ~/singulator_digital_twin
git fetch origin
git switch --track origin/fix/kty-mechatronics-runtime-v7
chmod +x scripts/check_kty_runtime_v7.sh
python3 tools/validate_kty_runtime_v7.py
bash ./scripts/build_kty_perception_3d.sh
```

## First launch

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/run_kty_perception_3d.sh
```

The first run is headless by default.  After transport works:

```bash
bash ./scripts/run_kty_perception_3d.sh show_dashboard:=true
```

## Diagnostics

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/check_kty_runtime_v7.sh
```

Acceptance requires:

1. all rollers remain parallel;
2. the KTY exits on active / outfeed rollers;
3. the static gate model appears during CLOSE_GATE / COMPACT;
4. products remain on the chute while the gate exists;
5. the gate disappears only after the next KTY passes readiness checks;
6. a second LOAD state is reached;
7. the classical 3-D node stays present and publishes contours;
8. RTF improves over the previous 0.15 baseline.
