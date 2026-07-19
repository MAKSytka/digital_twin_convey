# Forward-only singulation with bounded longitudinal lag

## Problem fixed

The previous emergency mode could command a follower at `-0.8 m/s`.  That
created reverse-moving parcels and collisions between consecutive spawn waves.
The controller now guarantees that every commanded cell speed is positive.

## Reference trajectory and lag budget

Each parcel receives a virtual reference trajectory when it first enters the
controlled matrix zone:

```text
x_ref(t) = x_entry + v_nominal * (t - t_entry)
lag(t)   = max(0, x_ref(t) - x_measured(t))
```

Default values:

```text
v_nominal = 2.00 m/s
maximum longitudinal lag = 0.30 m
minimum forward cell speed = 0.35 m/s
```

The speed floor is increased as the remaining lag budget is consumed:

```text
v_floor = v_nominal - (lag_max - lag) / guard_horizon
```

The result is clamped to `0.35 ... 3.00 m/s`.  If measured lag exceeds the
budget, the floor rises above `2.00 m/s`, so the parcel catches up instead of
being stopped or reversed.

With the default `guard_horizon = 0.20 s`, the nominal floor is approximately:

| measured lag | minimum permitted speed |
|---:|---:|
| 0.00 m | 0.50 m/s |
| 0.10 m | 1.00 m/s |
| 0.20 m | 1.50 m/s |
| 0.30 m | 2.00 m/s |

## Gap controller

The controller still separates a rear parcel from the parcel in front, but it
now does so only by:

1. accelerating the leading parcel up to `3.0 m/s`;
2. slowing the following parcel while keeping a positive speed;
3. increasing the target gap when measured closing speed is high;
4. respecting the remaining longitudinal lag budget.

The dynamic extra distance is estimated from closing speed and available
acceleration.  This starts braking before the geometric gap becomes critical.

## Conveyor settings

The upgraded demonstration uses:

```text
infeed conveyor = 2.00 m/s
free matrix cells = 2.00 m/s
roller throat = 2.00 m/s
matrix command range = 0.35 ... 3.00 m/s
mu = 0.8
mu2 = 0.8
```

The Gazebo actuator still supports the physical range `-3 ... +3 m/s`, but the
closed-loop controller deliberately uses only positive commands.

## Tuning

Start with `maximum_longitudinal_lag_m = 0.30`.  Reduce it to `0.20` if parcels
must stay closer to their nominal trajectory.  Increase it to `0.35 ... 0.40`
only if the matrix needs more time to separate dense waves.

A lower `lag_guard_horizon_s` permits a deeper short slowdown.  A higher value
makes the controller more conservative and keeps speeds closer to `2 m/s`.
