# ControllerMatlab

MATLAB closed-loop path-following framework for the Cranfield Kia Niro automated vehicle platform (GDP 2026).

ROS2 node repository:
`https://github.com/YuxuanCAVE/vehicle_controller`

## Overview

This project contains:

- Reference-path based closed-loop simulation for the Kia Niro platform
- Decoupled lateral and longitudinal controllers
- A combined MPC baseline
- Vehicle and actuator models based on measured map data
- ROS2 bag import and plotting utilities
- Single-run and controller-comparison workflows with auto-saved results

## Controllers

| Controller | Type | Description |
|---|---|---|
| `stanley` | Lateral | Heading + cross-track error correction |
| `pure_pursuit` | Lateral | Lookahead-based geometric steering |
| `mpc` | Lateral | Dynamic bicycle model MPC |
| `mpc_kinematic` | Lateral | Kinematic bicycle model MPC with curvature preview |
| `mpc_combined` | Lateral + Longitudinal | Single QP for steering and acceleration |
| `pid` | Longitudinal | Classical PID speed controller |
| `lqr` | Longitudinal | LQR speed controller with integral action |

## Quick Start

Open MATLAB in the project root and run:

```matlab
main
```

This executes a single simulation using the controller selection in `config/default_config.m`.

## Controller Selection

Edit [config/default_config.m](/f:/Controller_GDP/config/default_config.m):

```matlab
% Lateral controller
cfg.controller.lateral = "mpc_kinematic";   % "stanley" | "pure_pursuit" | "mpc" | "mpc_kinematic" | "mpc_combined"

% Longitudinal controller
cfg.controller.longitudinal = "lqr";        % "pid" | "lqr"
```

Typical combinations:

```matlab
% Dynamic MPC + LQR
cfg.controller.lateral = "mpc";
cfg.controller.longitudinal = "lqr";

% Kinematic MPC + LQR
cfg.controller.lateral = "mpc_kinematic";
cfg.controller.longitudinal = "lqr";

% Stanley + PID
cfg.controller.lateral = "stanley";
cfg.controller.longitudinal = "pid";

% Combined MPC
cfg.controller.lateral = "mpc_combined";
cfg.controller.longitudinal = "lqr";   % ignored by combined MPC
```

## Speed Reference

```matlab
% Constant speed
cfg.speed.mode = "constant";
cfg.speed.constant_value = 5.0;

% Profile from MAT file
cfg.speed.mode = "profile";
cfg.speed.profile_file = fullfile('data', 'reference_velocity', ...
    'referencePath_Velocity_peak_velocity_7.mat');
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

This runs:

- `mpc + lqr`
- `mpc_kinematic + lqr`

Outputs are saved under:

```text
run/compare_run/<timestamp>_lateral_mpc_compare/
```

Each controller has its own subfolder, and the compare folder also contains a comparison summary and comparison figure.

## New Updates in This Version

### 1. Kinematic Bicycle MPC Lateral Controller

New file:

- [controllers/lateral/mpc_kinematic_lateral.m](/f:/Controller_GDP/controllers/lateral/mpc_kinematic_lateral.m)

Key points:

- Uses a 2-state kinematic bicycle error model
- Uses path curvature preview
- Supports steering-delay compensation through the same interface as other lateral controllers
- Can be selected with `cfg.controller.lateral = "mpc_kinematic"`

### 2. Actuator Map Upgraded to 2D Scatter Interpolation

Updated files:

- [model/load_vehicle_params.m](/f:/Controller_GDP/model/load_vehicle_params.m)
- [model/longitudinal_model.m](/f:/Controller_GDP/model/longitudinal_model.m)

What changed:

- Actuator maps now use `(vel, cmd) -> force` scattered interpolation
- `Vel_Full` from the `.mat` map files is now loaded and used
- Force inversion is now done numerically from the 2D map instead of only using a 1D command-force lookup
- The old 1D arrays are still kept as fallback if 2D evaluation fails

This better matches the measured actuator data and avoids collapsing velocity-dependent map behaviour into a single line.

### 3. ROS2 Bag Import and Plotting

New utilities:

- [utils/read_rosbag_kia.m](/f:/Controller_GDP/utils/read_rosbag_kia.m)
- [utils/bag_to_ref.m](/f:/Controller_GDP/utils/bag_to_ref.m)

`read_rosbag_kia`:

- Reads a ROS2 bag folder
- Extracts steering, throttle, brake, odometry, and actual acceleration
- Normalises and resamples signals
- Saves processed output to `data/bag_data_10hz.mat`
- Saves a 2x2 white-background plot for quick inspection

Expected ROS2 bag topics:

- `/steering_feedback`
- `/brake_pedal_feedback`
- `/acceleration_feedback`
- `/ins/odometry`

Example:

```matlab
addpath('utils');
bag = read_rosbag_kia(fullfile(pwd, 'kianirobag'));
```

### 4. Simulation vs Bag Plotting

New plotting helper:

- [plotting/save_sim_vs_bag_plot.m](/f:/Controller_GDP/plotting/save_sim_vs_bag_plot.m)

For runs where `data/bag_data_10hz.mat` exists, the framework now generates:

- `sim_vs_bag.png`

The comparison figure is a 2x2 layout:

- Speed
- Steering command
- Throttle
- Brake

Plot conventions:

- Bag: red
- Simulation: green

The plotting logic has also been adjusted to reduce one trace fully covering the other by:

- Interpolating both onto a common time base
- Drawing simulation first
- Drawing bag second with stronger visual emphasis

### 5. Run Directory Reorganization

Run outputs are now separated by purpose:

```text
run/
  single_run/
    <timestamp>_<controller_tag>/
  compare_run/
    <timestamp>_lateral_mpc_compare/
```

This keeps normal runs and controller-comparison runs separate.

### 6. Longitudinal Anti-Chatter Shaping

Updated file:

- [controllers/longitudinal/LQR_controller.m](/f:/Controller_GDP/controllers/longitudinal/LQR_controller.m)

Added logic:

- Small acceleration-command deadband
- Drive/brake/coast switching hysteresis
- Optional low-pass filtering
- Optional rate limiting

Important note:

- These protections are now configured to be very mild by default
- They are only intended to suppress small command chatter near steady-state operation
- If path tracking degrades, the first place to check is whether longitudinal command shaping is changing the speed dynamics too much for the lateral MPC assumptions

Related tuning lives in:

- [config/default_config.m](/f:/Controller_GDP/config/default_config.m)

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

  plotting/
    save_sim_vs_bag_plot.m

  utils/
    read_rosbag_kia.m
    bag_to_ref.m
    angle_wrap.m
    nearest_path_ref_point.m
    rate_limit.m
    track_errors.m

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

## Output Files

### Single Run

Each `main` run typically produces:

| File | Description |
|---|---|
| `result.mat` | Full simulation output |
| `summary.txt` | Key performance metrics |
| `path_tracking.png` | Reference path vs actual path |
| `tracking_errors.png` | Tracking errors over time |
| `speed_tracking.png` | Speed and acceleration-related plots |
| `lateral_dynamics.png` | Steering and lateral-state plots |
| `execution_timing.png` | Controller execution timing |
| `sim_vs_bag.png` | Simulation vs bag comparison, if bag data exists |

### Compare Run

Each `compare_lateral_mpc_variants` run produces:

- One folder per controller case
- `comparison.mat`
- `comparison_summary.txt`
- `lateral_controller_comparison.png`

## Vehicle and Actuator Model

The simulation plant is a coupled bicycle model with:

- Steering delay
- Longitudinal delay
- Tire lateral force model
- Resistive longitudinal force model
- Pedal-to-force actuator mapping from measured data

The acceleration and brake maps are now treated as velocity-dependent 2D maps rather than a single 1D curve.

## MATLAB Dependencies

- MATLAB R2020b or later recommended
- Optimization Toolbox for `quadprog`
- ROS Toolbox for ROS2 bag import through `ros2bag`

## Notes

- `main` no longer includes the old longitudinal controller comparison interface
- The previous `cfg.run.compare_longitudinal` and `cfg.run.longitudinal_compare_set` options have been removed
- Use `compare_lateral_mpc_variants` for the current controller-comparison workflow

## Recommended Workflow

1. Generate or update bag data with `read_rosbag_kia`
2. Run a single controller case with `main`
3. Inspect `summary.txt`, run plots, and `sim_vs_bag.png`
4. Run `compare_lateral_mpc_variants`
5. Compare `mpc` vs `mpc_kinematic` under the same longitudinal `lqr` controller
