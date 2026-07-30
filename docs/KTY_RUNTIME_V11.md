# KTY runtime v11

Runtime v11 keeps the accepted roller-free contact-surface transport and changes
two behaviours observed during target-machine testing.

## Stronger compaction

Loading agitation:

- 5 Hz;
- +/-1.8 mm.

Compaction:

- frequency sweep 6.5..9.0 Hz;
- +/-8 mm commanded displacement;
- 15 s duration;
- 2 s smooth ramp in and out;
- 1.2 s settling interval.

The generated world gives the physical vibration joint +/-10 mm travel, 16 kN
effort and a stronger position controller. During COMPACT the joint position
should repeatedly approach approximately -0.008..+0.008 m.

## Despawn-first changeover

The repeatable lifecycle is:

```text
EJECT_ACTIVE
-> DESPAWN_ACTIVE
-> POSITION_NEXT
-> VERIFY_READY
-> OPEN_GATE
```

After the loaded KTY reaches the exit threshold, active and outfeed surfaces are
stopped. Products belonging to that KTY are removed first; the KTY model is then
removed. Every removal is retried and confirmed against the Gazebo world pose
stream. POSITION_NEXT is entered only after the old KTY is absent.

Telemetry in `/kty/flow/state` includes:

- `runtime_profile=kty_mechatronics_v11`;
- `changeover_order=eject_despawn_position_next`;
- `last_despawned_kty`;
- `despawned_cycles`;
- effective vibration frequency, amplitude and duration.

## Acceptance

```bash
bash ./scripts/check_kty_vibration.sh
bash ./scripts/check_kty_runtime_v7.sh
```

The first script validates the +/-8 mm command and frequency sweep. The second
requires DESPAWN_ACTIVE and verifies that the first KTY no longer exists before
the second KTY is positioned.
