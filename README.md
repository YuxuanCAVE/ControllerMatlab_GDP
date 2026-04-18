# ControllerMatlab

MATLAB closed-loop path-following framework for the Cranfield Kia Niro automated vehicle platform (GDP 2026).

ROS2 node repository:
`https://github.com/YuxuanCAVE/vehicle_controller`

## Overview

This repository simulates a decoupled lateral and longitudinal control stack on a coupled bicycle-model vehicle plant. The current baseline is:

- lateral controller: `mpc_kinematic`
- longitudinal controller: `pid`
- lateral plant: selectable `dynamic` or `bicycle_linear`
- longitudinal actuator model: 1D lookup inversion plus 1D forward execution
- outputs: run directory with metrics, plots, and optional simulation-vs-bag comparison

The active defaults are always defined in [config/default_config.m](/f:/Controller_GDP/config/default_config.m). If this README and the code ever diverge, trust the code.

## Current Default Setup

The current default selection is:

```matlab
cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "pid";
cfg.plant.lateral_model = "dynamic";   % "dynamic" | "bicycle_linear"
```

Other important active defaults:

```matlab
cfg.sim.dt = 0.1;
cfg.sim.T_end = 150;

cfg.speed.mode = "constant";
cfg.speed.constant_value = 3.0;
cfg.speed.profile_file = fullfile( ...
    'data', 'reference_velocity', 'referencePath_Velocity_peak_velocity_3.mat');

cfg.vehicle.max_steer = deg2rad(20);
cfg.vehicle.max_steer_rate = deg2rad(15);
cfg.vehicle.delay.steer_s = 0.1;
cfg.vehicle.delay.longitudinal_s = 0.1;
```

Current `mpc_kinematic` baseline:

```matlab
cfg.mpc_kinematic.N = 16;
cfg.mpc_kinematic.Q = diag([2.2, 0.8]);
cfg.mpc_kinematic.R = 25;
cfg.mpc_kinematic.Rd = 15.0;
cfg.mpc_kinematic.kappa_ff_gain = 0.1;
cfg.mpc_kinematic.fallback_k_e_y = 0.9;
cfg.mpc_kinematic.fallback_k_e_psi = 1.4;
```

Current PID baseline:

```matlab
cfg.lon_pid.kp = 1.6;
cfg.lon_pid.ki = 0.0;
cfg.lon_pid.kd = 0.0;
```

## Controllers And Plant Options

### Lateral Controllers

| Item | Type | Notes |
|---|---|---|
| `stanley` | Controller | Heading plus cross-track feedback |
| `pure_pursuit` | Controller | Geometric lookahead steering |
| `mpc` | Controller | Dynamic-state lateral MPC |
| `mpc_kinematic` | Controller | Kinematic bicycle MPC with curvature preview |
| `mpc_combined` | Controller | Combined steering and acceleration MPC |
| `fake_controller` | Controller | Debug mode for isolating subsystems |

### Longitudinal Controllers

| Item | Type | Notes |
|---|---|---|
| `pid` | Controller | PID on speed error, output is net `a_des` |
| `fake_controller` | Controller | Debug mode for isolating subsystems |

### Lateral Plant Models

| Item | Type | Notes |
|---|---|---|
| `dynamic` | Plant | Nonlinear tire-force model through `lateral_tire_model.m` |
| `bicycle_linear` | Plant | Linear bicycle tire-force model with force saturation |

Set the plant in [config/default_config.m](/f:/Controller_GDP/config/default_config.m):

```matlab
cfg.plant.lateral_model = "dynamic";
```

## Quick Start

Open MATLAB in the project root and run:

```matlab
main
```

This will:

- load the reference path and speed reference
- load vehicle and actuator map data
- run one closed-loop simulation
- save plots and `result.mat` under `run/single_run/<timestamp>_<controller>_<plant>/`

The run tag now includes the plant type, so `dynamic` and `bicycle_linear` results are no longer mixed together in names or plot titles.

## Control And Plant Signal Chain

### Lateral

The current lateral execution chain is:

```text
reference path
  -> lateral controller
  -> steering delay buffer
  -> steering rate limit
  -> lateral plant ("dynamic" or "bicycle_linear")
  -> coupled_bicycle_dynamics
```

`lateral_model.m` is now the plant wrapper. It selects:

- `dynamic` -> `lateral_tire_model.m`
- `bicycle_linear` -> linear tire model inside `lateral_model.m`

Both plant options then pass the computed tire forces into the same coupled vehicle dynamics.

Low-speed tire-force fade-in is applied in both plant variants to reduce unrealistic yaw oscillation near standstill.

### Longitudinal

The current longitudinal chain is:

```text
v_ref, v
  -> longitudinal controller
  -> a_des
  -> longitudinal delay buffer
  -> force-balance model
  -> actuator branch selection: drive / coast / brake
  -> 1D inverse lookup: force -> map command
  -> pedal publish scaling
  -> 1D forward lookup: map command -> actual force
  -> coupled_bicycle_dynamics
```

This logic is implemented in [model/longitudinal_model.m](/f:/Controller_GDP/model/longitudinal_model.m).

## Longitudinal Lookup Logic

The longitudinal lookup has been simplified to a fully 1D and self-consistent structure.

Teacher-provided mapping idea:

```matlab
ACC_req = interp1(Force_full, Acc_full, Ftractive);
ACC_pct = ACC_req / max(Acc_full) * 0.6;
```

The current implementation follows that same structure, but adds execution-side consistency and actuator gating:

1. Compute required total drive/brake force from force balance:

```matlab
F_required = M * a_des + F_resist;
```

2. Select actuator branch with hysteresis:

- `drive`
- `coast`
- `brake`

3. For `drive` or `brake`, invert force to map command with a 1D interpolation.

4. Convert the internal map command to published pedal percentage using the 0.6 publish cap.

5. Re-evaluate the achieved force with the matching 1D forward map.

This means inverse lookup and forward execution use the same reduced 1D map, which is the main reason the current actuator behavior is much easier to debug.

## Pedal Minimum Effective Logic

The current code no longer inserts an artificial continuous small-force segment at the low end.

Instead, [model/load_vehicle_params.m](/f:/Controller_GDP/model/load_vehicle_params.m) derives low-end thresholds directly from the lookup tables:

- `veh.acc.pedal_min_publish_from_map`
- `veh.brk.pedal_min_publish_from_map`
- `veh.acc.pedal_min_effective`
- `veh.brk.pedal_min_effective`
- `veh.acc.force_min_effective`
- `veh.brk.force_min_effective`

Current behavior:

- below the map-derived minimum effective force, inverse lookup returns `0`
- below the map-derived minimum effective command, forward lookup returns `0`
- coast is explicitly represented
- drive/coast/brake switching uses hysteresis through `force_enter` and `force_exit_coast`

So the current logic is:

- no fake tiny throttle below the real map minimum
- no fake tiny brake below the real map minimum
- explicit `0 / >= minimum effective pedal` actuator behavior

## PID Interface

`PID_controller.m` now expects:

```matlab
[a_des, lon] = PID_controller(v_ref, v, lon, dt)
```

It does not accept zero-argument calls.

The PID output is now a net acceleration correction only:

```matlab
a_des = kp * ev + ki * integral(ev) + kd * d(ev)/dt
```

Resistance compensation is not added inside the PID. Resistance is added later by the force-balance actuator model through:

```matlab
F_required = M * a_des + F_resist
```

This is important because the longitudinal controller and longitudinal plant are now intentionally separated:

- controller decides the desired net acceleration
- plant decides what force is needed once resistance is included

## Reference Speed Handling

The simulation smooths the speed reference in the time domain inside [simulation/run_closed_loop.m](/f:/Controller_GDP/simulation/run_closed_loop.m).

This means the raw speed profile is not applied as an instantaneous step at each waypoint. Instead, the reference speed is rate-limited using 85% of the available accel/decel limits.

That smoother is part of the simulation baseline and affects all controller comparisons.

## Controller Selection

Edit [config/default_config.m](/f:/Controller_GDP/config/default_config.m), for example:

```matlab
cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "pid";
cfg.plant.lateral_model = "dynamic";
```

Typical comparisons:

```matlab
cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "pid";
cfg.plant.lateral_model = "dynamic";
```

```matlab
cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "pid";
cfg.plant.lateral_model = "bicycle_linear";
```

```matlab
cfg.controller.lateral = "mpc";
cfg.controller.longitudinal = "pid";
cfg.plant.lateral_model = "dynamic";
```

```matlab
cfg.controller.lateral = "mpc_combined";
cfg.controller.longitudinal = "pid";   % ignored by combined MPC
cfg.plant.lateral_model = "dynamic";
```

## Outputs

Each `main` run produces:

| File | Description |
|---|---|
| `result.mat` | Full saved workspace for the run |
| `summary.txt` | Key metrics and pass/fail summary |
| `path_tracking.png` | Reference path vs simulated path |
| `tracking_errors.png` | CTE, heading error, longitudinal deviation, speed error |
| `speed_tracking.png` | Speed and longitudinal acceleration |
| `lateral_dynamics.png` | Commanded/executed steering plus `v_y` and `r` |
| `execution_timing.png` | Controller execution time against control period |
| `longitudinal_internal_diagnostics.png` | `F_required`, `F_drive`, lookup requests, and branch mode |
| `sim_vs_bag.png` | Simulation-vs-bag comparison if bag data exists |

## Simulation Vs Bag Comparison

The bag comparison plot uses:

- speed
- steering command
- throttle
- brake

Important implementation notes:

- the simulation steering is plotted with a sign flip to match the bag convention
- the title now includes the controller and plant label passed from `main.m`

This figure is mainly for checking:

- steering amplitude and oscillation
- speed bias
- throttle/brake pulse pattern
- overall mismatch between plant/controller settings and recorded behavior

## Run Modes

### Single Run

```matlab
main
```

Outputs are saved under:

```text
run/single_run/<timestamp>_<controller_tag>/
```

### Lateral MPC Comparison

```matlab
compare_lateral_mpc_variants
```

This is the quick script for comparing lateral MPC variants.

### Tuning Sweep

```matlab
run_tuning_sweep
```

Use this for broader parameter sweeps once a local baseline is stable.

## Practical Tuning Guidance

The current practical order is:

1. choose the lateral plant you want to trust for tuning
2. tune lateral controller smoothness and tracking first
3. lock lateral settings
4. tune longitudinal controller on top of that baseline

### Lateral MPC

Main parameters:

- `cfg.mpc_kinematic.Q`
- `cfg.mpc_kinematic.R`
- `cfg.mpc_kinematic.Rd`
- `cfg.mpc_kinematic.N`
- `cfg.mpc_kinematic.kappa_ff_gain`

Interpretation:

- larger `Q` pushes harder on tracking error
- larger `R` suppresses steering amplitude
- larger `Rd` suppresses steering variation between steps
- larger `N` increases preview horizon
- larger `kappa_ff_gain` makes curvature feedforward stronger

If the vehicle still tracks but steering amplitude is too large, the first parameters to revisit are usually `R`, then `Rd`, then the plant choice itself.

### Longitudinal PID

Main parameters:

- `cfg.lon_pid.kp`
- `cfg.lon_pid.ki`
- `cfg.lon_pid.kd`

Interpretation:

- increase `kp` first for response strength
- add `ki` only if there is a persistent steady-state speed bias
- add `kd` only if proportional control is too oscillatory

Because the actuator model already includes resistance and map deadzone behavior, do not retune PID as if it were directly commanding pedal.

## Project Structure

```text
Controller_GDP/
  main.m
  compare_lateral_mpc_variants.m
  run_tuning_sweep.m

  config/
    default_config.m

  controllers/
    lateral/
      mpc_lateral.m
      mpc_kinematic_lateral.m
      mpc_combined.m
      pure_pursuit_lateral.m
      stanley_lateral.m
    longitudinal/
      PID_controller.m

  model/
    coupled_bicycle_dynamics.m
    lateral_model.m
    lateral_tire_model.m
    load_vehicle_params.m
    longitudinal_model.m

  plotting/
    save_sim_vs_bag_plot.m
    plot_lookup_maps.m

  reference/
    load_reference_path.m
    load_reference_speed.m
    path_curvature.m
    path_yaw.m
    smooth_speed_profile.m

  simulation/
    run_closed_loop.m
    track_errors.m

  utils/
    angle_wrap.m
    bag_to_ref.m
    nearest_path_ref_point.m
    rate_limit.m
    read_rosbag_kia.m

  data/
    path_ref.mat
    bag_data_10hz.mat
    Acc_mapData_noSlope.mat
    brake_mapData_noSlope.mat
    reference_velocity/

  run/
    single_run/
    compare_run/
```

## Dependencies

- MATLAB R2020b or later recommended
- Optimization Toolbox for `quadprog`
- ROS Toolbox if you want to read ROS2 bags directly

## Review Notes

After checking the current control and plant chain, there is no obvious blocking logic bug in the updated lookup or plant-selection flow.

Two practical notes are worth keeping in mind:

- the README was previously outdated and did not reflect the current force-balance lookup logic or plant selection
- run labels previously did not clearly include the lateral plant type, which made dynamic vs bicycle-linear comparisons easy to misread

Both are now aligned with the current codebase.
