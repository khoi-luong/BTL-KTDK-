# Drone altitude control: BLDC actuator simulation

MATLAB code for Part II of the BLDC report. A 5 kg quadcopter with four
T-Motor U5 KV400 motors is held at a target altitude H = 50, 75 or 100 m by a PID
controller. An −8 N push from 70 s to 72 s tests how well it rejects a disturbance.

## Folder layout

| Path | What it is |
|---|---|
| `main_drone.m` | **Start here.** Runs everything, section by section. |
| `bldc_altitude_sim.m` | The simulation used in the report (plain MATLAB, no toolboxes). |
| `animate_drone_3d.m` | 3-D animation with X/Y/Z axes, using the SolidWorks drone. |
| `CAD\` | SolidWorks files, `stl\` (shapes for the animation), and `export_drone_stl.m`. |
| `simscape\` | Optional Simscape Multibody version of the same loop. |
| `results\` | Every saved figure, data file and video goes here. |

## How the code works

`bldc_altitude_sim.m` runs the loop in the report's flowchart, one step every
Δt = 0.005 s for 150 s:

1. **Controller:** a two-degree-of-freedom PID computes the required acceleration from the altitude error and the vertical speed. It has anti-windup.
2. **Thrust command:** the acceleration becomes a thrust per motor, T = m(g + a)/4, limited to 0 … T_max.
3. **Motor speed command:** the inverse thrust table A⁻¹(T) turns that thrust into a speed command n_c.
4. **BLDC actuator:** a first-order lag, τ·dn/dt = n_c − n with τ = 0.14 s, then the static thrust table T = A(n). The table is the U5 + 16×5.4 propeller test data at 22.2 V.
5. **Drone:** vertical dynamics m·dv/dt = 4T − mg + d(t), and dz/dt = v.

The PID gains come from pole placement:
D(s) = τ(s + p)³(s + λ), with p = 0.25 / 0.20 / 0.18 for 50 / 75 / 100 m.

The script then prints:
- the hover point,
- the gains,
- the response metrics (settling time, sag after the disturbance, peak throttle),
- the Bode stability margins.

It returns one result per altitude in `R(1..3)`, each with the time series
`t, z, v, n, F, thr`.

`animate_drone_3d.m` replays `R` in 3-D:
- The drone shape is read from `CAD\stl` and moves to the simulated altitude z(t).
- The propellers spin faster when the motor speed n is higher.
- The Z axis is compressed so a 50–100 m flight fits in the plot. The drone is stretched by the same factor, so it keeps its real shape.

## Saving the results for all 3 altitudes

Open `main_drone.m` and run it with **Run**, or section by section with **Ctrl+Enter**.

| Section | Saves to `results\` |
|---|---|
| 1. Simulate 50 / 75 / 100 m | `hinh_1.png` … `hinh_4.png` (actuator step, altitude, current/power, Bode) |
| 2. Save data | `altitude_results.mat` (all of `R`) and `altitude_H50.csv`, `altitude_H75.csv`, `altitude_H100.csv` (columns: t, z, v, n, F, throttle) |
| 4. Save videos | `drone_animation_H50.mp4`, `drone_animation_H75.mp4`, `drone_animation_H100.mp4` |

Single commands, if you only need one thing:

```matlab
R = bldc_altitude_sim(true, true);                    % figures -> results\hinh_1..4.png
animate_drone_3d(75, 'SaveVideo', true)               % video for one altitude
animate_drone_3d(100, 'Speed', 20)                    % just watch, 20x speed
```

Saving a video takes about 15–30 s per altitude.

## Optional

- **Simscape model:** `simscape\run_altitude_compare.m` runs `SimpleDrone_altitude.slx` at all 3 altitudes and compares it with `bldc_altitude_sim`. It saves `results\so_sanh_simscape.png`. This needs Simscape Multibody. To rebuild the model from scratch, run `build_drone_altitude_model`.
- **Changed the CAD?** Run `CAD\export_drone_stl.m`. It uses Autodesk Inventor to convert `SimpleDrone.SLDASM` to STL, and the animation picks up the new shape.
