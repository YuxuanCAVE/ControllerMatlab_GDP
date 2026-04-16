# ControllerMatlab

MATLAB closed-loop path-following framework for the Cranfield Kia Niro automated vehicle platform (GDP 2026).

ROS2 node repository:
`https://github.com/YuxuanCAVE/vehicle_controller`

## Overview

This repository simulates a decoupled lateral and longitudinal control stack on a coupled bicycle-model plant. The current codebase is centred around:

- `mpc_kinematic` for lateral path tracking
- `pid` for longitudinal speed tracking
- a coupled planar bicycle plant
- 1D longitudinal actuator lookup maps for self-consistent force-command conversion
- ROS2 bag comparison plots for steering, speed, throttle, and brake

The main recent changes are:

- longitudinal actuator lookup has been simplified to a fully 1D mapping
- the longitudinal actuator branch now switches based on `F_required`, not directly on the sign of `a_des`
- the kinematic MPC cost now includes state, steering magnitude, and steering rate-change penalties
- the current default workflow is to tune lateral first, then tune longitudinal PID on top of the locked lateral baseline

## Current Default Setup

The current default selection in [config/default_config.m](/f:/Controller_GDP/config/default_config.m) is:

```matlab
cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "pid";
```

The current default tuning values in that file are the active baseline. Always treat that file as the ground truth if this README and local experiments diverge.

## Controllers

| Controller | Type | Description |
|---|---|---|
| `stanley` | Lateral | Heading + cross-track correction |
| `pure_pursuit` | Lateral | Lookahead geometric steering |
| `mpc` | Lateral | Dynamic bicycle model MPC |
| `mpc_kinematic` | Lateral | Kinematic bicycle MPC with curvature preview and steering delay compensation |
| `mpc_combined` | Lateral + Longitudinal | Combined steering/acceleration MPC baseline |
| `pid` | Longitudinal | Speed-error PID producing `a_des` |
| `lqr` | Longitudinal | Longitudinal LQR with anti-chatter shaping |
| `lqr_force_balance` | Longitudinal | Force-balance LQR around resistance equilibrium |

## Quick Start

Open MATLAB in the project root and run:

```matlab
main
```

This will:

- load the reference path and speed profile
- load vehicle and actuator map data
- run one closed-loop simulation
- save plots and summary files under `run/single_run/...`

## Current Signal Chain

### Lateral

The main lateral chain is:

```text
reference path
  -> mpc_kinematic_lateral
  -> steering delay buffer
  -> steering rate limit
  -> lateral_tire_model
  -> coupled_bicycle_dynamics
```

The current `mpc_kinematic` cost is:

```text
J = X'QX + U'RU + Rd * dU'dU
```

This is implemented in [controllers/lateral/mpc_kinematic_lateral.m](/f:/Controller_GDP/controllers/lateral/mpc_kinematic_lateral.m).

### Longitudinal

The main longitudinal chain is:

```text
v_ref, v
  -> PID_controller
  -> a_des
  -> longitudinal delay buffer
  -> longitudinal_model
  -> 1D inverse lookup: force -> cmd
  -> 1D forward lookup: cmd -> force
  -> coupled_bicycle_dynamics
```

This is intentionally self-consistent now: both inverse and forward actuator conversion are 1D. The current implementation lives in [model/longitudinal_model.m](/f:/Controller_GDP/model/longitudinal_model.m).

## Longitudinal Actuator Model

The current longitudinal actuator model no longer uses velocity-dependent 2D execution inside the plant. Instead:

- `force_target -> cmd` uses 1D interpolation on `force_full` / `cmd_full`
- `cmd -> force` uses the matching 1D interpolation on the same reduced map

This change was made to keep actuator inversion and actuator execution self-consistent during tuning and debugging.

The branch logic is now:

```matlab
F_required = M * a_des + F_resist;

if F_required >= 0
    % drive branch
else
    % brake branch
end
```

This avoids the earlier issue where small negative `a_des` values could still correspond to positive required drive force once resistance was included.

## Controller Selection

Edit [config/default_config.m](/f:/Controller_GDP/config/default_config.m):

```matlab
cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "pid";
```

Typical combinations:

```matlab
cfg.controller.lateral = "mpc";
cfg.controller.longitudinal = "pid";

cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "pid";

cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "lqr";

cfg.controller.lateral = "mpc_combined";
cfg.controller.longitudinal = "pid";   % ignored by combined MPC
```

## Speed Reference

```matlab
cfg.speed.mode = "constant";
cfg.speed.constant_value = 3.0;
```

or

```matlab
cfg.speed.mode = "profile";
cfg.speed.profile_file = fullfile( ...
    'data', 'reference_velocity', 'referencePath_Velocity_peak_velocity_5.mat');
```

Available example profiles are under `data/reference_velocity/`.

## Run Modes

### Single Run

```matlab
main
```

Outputs are saved under:

```text
run/single_run/<timestamp>_<controller_tag>/
```

### Compare Dynamic MPC vs Kinematic MPC

```matlab
compare_lateral_mpc_variants
```

This script compares:

- `mpc + lqr`
- `mpc_kinematic + lqr`

Outputs are saved under:

```text
run/compare_run/<timestamp>_lateral_mpc_compare/
```

### Broad Sweep

```matlab
run_tuning_sweep
```

This is a broader scenario sweep script. It is useful for compliance-style batch runs, but for current `mpc_kinematic + pid` tuning, a narrower custom sweep is usually more effective.

## Current Tuning Guidance

The current practical tuning order is:

1. tune lateral `mpc_kinematic` until steering command is smooth and `peak_cte` is acceptable
2. lock lateral parameters
3. tune longitudinal `pid`

### Lateral

Primary parameters:

- `cfg.mpc_kinematic.Q`
- `cfg.mpc_kinematic.R`
- `cfg.mpc_kinematic.Rd`
- `cfg.mpc_kinematic.kappa_ff_gain`
- `cfg.mpc_kinematic.N`

Interpretation:

- larger `Q` pushes harder on path and heading error
- larger `R` penalises steering magnitude
- larger `Rd` penalises steering changes and reduces chatter
- larger `kappa_ff_gain` increases curvature feedforward aggressiveness
- larger `N` increases prediction horizon

The fallback gains:

```matlab
cfg.mpc_kinematic.fallback_k_e_y
cfg.mpc_kinematic.fallback_k_e_psi
```

are only used if the QP solve fails and the controller falls back to a simple linear correction law.

### Longitudinal

Primary parameters:

- `cfg.lon_pid.kp`
- `cfg.lon_pid.ki`
- `cfg.lon_pid.kd`

Current guidance:

- increase `kp` first
- add small `ki` only after proportional response is acceptable
- use `kd` last, if needed

For current tuning work, `peak_lon_dev` is often the key longitudinal metric to reduce once lateral behaviour is already stable.

## Outputs

Each `main` run typically produces:

| File | Description |
|---|---|
| `result.mat` | Full simulation result struct |
| `summary.txt` | Key performance metrics |
| `path_tracking.png` | Reference path vs actual path |
| `tracking_errors.png` | Lateral and longitudinal tracking errors |
| `speed_tracking.png` | Speed and acceleration plots |
| `lateral_dynamics.png` | Steering and lateral-state plots |
| `execution_timing.png` | Controller execution timing |
| `sim_vs_bag.png` | Simulation vs bag comparison, if bag data exists |

## Simulation vs Bag Plot

The current bag comparison figure includes:

- Speed
- Steering command
- Throttle
- Brake

This is useful for quickly checking:

- steering chatter
- speed tracking bias
- throttle/brake pulse behaviour
- overall shape mismatch between simulation and recorded data

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
      stanley_lateral.m
      pure_pursuit_lateral.m
    longitudinal/
      PID_controller.m
      LQR_controller.m
      longitudinal_lqr_force_balance_controller.m

  model/
    load_vehicle_params.m
    longitudinal_model.m
    lateral_model.m
    lateral_tire_model.m
    coupled_bicycle_dynamics.m

  reference/
    load_reference_path.m
    load_reference_speed.m
    smooth_speed_profile.m
    path_yaw.m
    path_curvature.m

  simulation/
    run_closed_loop.m
    track_errors.m

  plotting/
    save_sim_vs_bag_plot.m

  utils/
    read_rosbag_kia.m
    bag_to_ref.m
    angle_wrap.m
    nearest_path_ref_point.m
    rate_limit.m

  data/
    path_ref.mat
    Acc_mapData_noSlope.mat
    brake_mapData_noSlope.mat
    bag_data_10hz.mat
    reference_velocity/

  run/
    single_run/
    compare_run/
```

## Dependencies

- MATLAB R2020b or later recommended
- Optimization Toolbox for `quadprog`
- ROS Toolbox for ROS2 bag import through `ros2bag`

## Notes

- the README reflects the current code state as of today's tuning/debugging updates
- the active defaults should always be checked in [config/default_config.m](/f:/Controller_GDP/config/default_config.m)
- if behaviour and README diverge, trust the code and generated `summary.txt` outputs first
