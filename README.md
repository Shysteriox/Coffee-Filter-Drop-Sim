# Coffee Filter Drop Simulator

A browser simulator of nested paper coffee filters falling through air, made for an AP Physics C drag lab. Open `index.html` to run it. There is nothing to install, but it needs internet access to load three.js.

## What it does

- Vertical motion uses Newton's second law with quadratic drag, F = 1/2 rho Cd A v^2, in 2 ms steps.
- Tilt uses a restoring torque that grows with speed, plus damping and a small random torque.
- Room air is a random sideways wind. Bendiness weakens the restoring torque.
- The airflow picture is only a visual. It does not change the forces.
- The drop panel runs many drops at once and reports landing tilt and sideways drift.

## Settings

The simulator uses g = 9.81 m/s^2, air density 1.2 kg/m^3, and 0.95 g per filter. Its area is the rim circle, 0.0241 m^2, so its Cd is not the same number as a Cd worked out with a different area. Only the product Cd times A matters for the fall.

The default Cd of 0.255 gives a terminal velocity of about 1.60 sqrt(n) m/s, the best fit to the measured drops.

The lab report's tables set Cd for each stack so that the terminal velocity matched that stack's measured value:

| Filters | Measured terminal velocity (m/s) | Cd |
|---|---|---|
| 4 | 3.20 | 0.255 |
| 8 | 4.58 | 0.249 |
| 12 | 5.44 | 0.265 |
| 16 | 6.30 | 0.264 |
| 20 | 7.68 | 0.222 |

## Limits

Fall times match the measurements because Cd was set from them, so that match is not an independent test. The torque, damping, room air and bendiness values are estimates, so the results show trends and not real values.

## License

MIT
