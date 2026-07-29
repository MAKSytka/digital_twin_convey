# KTY roller-free contact-surface runtime

This runtime replaces the experimental roller mechanism with three flat physical
contact zones. The goal is a robust digital-twin transport abstraction while
preserving gravity, collisions, physical vibration and product motion inside the
KTY.

## Transport layout

```text
infeed contact surface
        -> vibrating active contact surface
        -> outfeed contact surface
```

The generated Gazebo world contains no `*_roller_*` links, joints or velocity
controllers.

A Gazebo system plugin named `KtyConveyorSurfaceSystem` subscribes to:

```text
/kty/mech/infeed_surface/cmd_vel
/kty/mech/active_surface/cmd_vel
/kty/mech/outfeed_surface/cmd_vel
```

Commands are target linear velocities in metres per second. The plugin measures
the current X velocity of every `kty_mech_container_*` model located above a
surface zone and applies a bounded X force to its canonical link. It does not set
world pose and does not overwrite vertical velocity, so vibration, gravity and
contact physics remain active.

The default transport speeds are:

```text
normal: 0.34 m/s
slow positioning: 0.12 m/s
```

A command of `0.0` is normal during `LOAD`, `COMPACT` and idle phases. The
outfeed command becomes positive only in `EJECT_ACTIVE`; the diagnostic script
observes the state and command together instead of treating an idle zero as a
failure.

## Active zone

The central contact plate is attached to `vibration_deck`. Therefore the same
KTY receives:

- longitudinal transport force from the contact-surface plugin;
- weak 8 Hz loading vibration at ±0.5 mm;
- strong 18 Hz compaction vibration at ±3 mm;
- side-clamp and locator interactions.

## Chute gate

The slide gate remains lifecycle-managed:

- closing creates static model `kty_mech_chute_gate`;
- opening removes that model;
- the plate blocks products while the loaded KTY exits and the queued KTY is
  positioned.

## Runtime load

Default test settings remain balanced:

```text
product spawn interval: 1.15 s
physics step:           2 ms
RGB-D:                  640 x 480 at 8 Hz
fill estimation:        4 Hz
3-D perception:         4 Hz
dashboard window:       disabled by default
```

## Build

```bash
cd ~/singulator_digital_twin
source /opt/ros/jazzy/setup.bash
bash ./scripts/build_kty_perception_3d.sh
```

The build compiles:

```text
singulator_interfaces
kty_conveyor_surface
kty_station_sim
```

The Gazebo plugin must be installed as:

```text
install/kty_conveyor_surface/lib/libKtyConveyorSurfaceSystem.so
```

## Run

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash ./scripts/run_kty_perception_3d.sh
```

The launch file sets `GZ_SIM_SYSTEM_PLUGIN_PATH` automatically.

## Diagnostics

```bash
bash ./scripts/check_kty_runtime_v7.sh
```

Expected acceptance sequence:

```text
LOAD
-> CLOSE_GATE
-> COMPACT
-> EJECT_ACTIVE
-> POSITION_NEXT
-> VERIFY_READY
-> OPEN_GATE
-> LOAD (cycle 2)
```

Manual command observation:

```bash
ros2 topic echo /kty/mech/outfeed_surface/cmd_vel
```

Expected values:

```text
0.0 during loading and compaction
approximately 0.34 during EJECT_ACTIVE
```

## Runtime acceptance

The stage is accepted only after target-machine testing confirms:

1. no roller models are visible;
2. the loaded KTY moves continuously on the flat active/outfeed surfaces;
3. products stay inside the KTY during transport;
4. the gate retains products during changeover;
5. a second KTY reaches `LOAD`;
6. perception remains alive;
7. measured RTF is recorded.
