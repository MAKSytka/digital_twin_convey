# KTY contact-surface runtime: anti-tip ejection correction

The recorded runtime showed the loaded KTY beginning to translate and then rotating forward at the active-zone exit. The front lower edge was still able to contact the locator stop while the active and outfeed surface commands were already enabled.

The corrected changeover is staged:

1. close the chute gate;
2. stop active and outfeed traction;
3. command clamps open and locator fully retracted;
4. wait 2.5 seconds of wall time for mechanical clearance;
5. enable active and outfeed surfaces at 0.65 m/s;
6. monitor the KTY until it clears x=1.25 m.

Generated geometry changes:

- locator centre: `(0.350, 0, 0.300) m`;
- locator blade: `20 x 520 x 180 mm`;
- retracted locator top: `390 mm`, giving 110 mm clearance below the 500 mm transport plane;
- raised locator top: `615 mm`, giving a 115 mm physical stop above the transport plane;
- active / bridge / outfeed tops: `500 / 498 / 496 mm`.

The force-based surface controller now works against low longitudinal friction:

- `mu_x = 0.08` using `fdir1 = 1 0 0`;
- `mu_y = 1.15` for transverse guidance;
- `velocity_gain = 80`;
- `max_force = 120 N`.

This avoids the previous large impulse while preserving gravity, vertical vibration and product motion inside the KTY.
