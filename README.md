<<<<<<< HEAD
# ControllerMatlab — Kia Niro Path-Tracking Controller Framework
=======
# ROS2 Node
You can find here https://github.com/YuxuanCAVE/vehicle_controller

# ControllerMatlab
>>>>>>> 4c7c847ed59c18e67016c0cec229ec8640e3f630

MATLAB closed-loop path-following framework for the Cranfield Kia Niro automated vehicle platform (GDP 2026).

## Controllers Implemented

| Controller | Type | Description |
|-----------|------|-------------|
| **Stanley** | Lateral (kinematic) | Heading + cross-track error correction at front axle |
| **Pure Pursuit** | Lateral (geometric) | Lookahead-based geometric steering |
| **MPC** | Lateral (dynamic) | Linear MPC with bicycle model, curvature preview, delay compensation |
| **MPC Combined** | Lateral + Longitudinal | Single QP optimises steering AND acceleration simultaneously |
| **PID** | Longitudinal | Classical proportional-integral-derivative speed controller |
| **LQR** | Longitudinal | Optimal controller with integral action via DARE |

## Quick Start

```matlab
% Open MATLAB in the project root, then:
main
```

All outputs are saved under `run/`.

## Changing the Controller

Edit `config/default_config.m`:

```matlab
% Option 1: MPC lateral + LQR longitudinal (decoupled)
cfg.controller.lateral = "mpc";
cfg.controller.longitudinal = "lqr";

% Option 2: MPC Combined (lateral + longitudinal in one QP)
cfg.controller.lateral = "mpc_combined";
cfg.controller.longitudinal = "lqr";   % ignored, MPC handles both

% Option 3: Stanley + PID
cfg.controller.lateral = "stanley";
cfg.controller.longitudinal = "pid";

% Option 4: Pure Pursuit + LQR
cfg.controller.lateral = "pure_pursuit";
cfg.controller.longitudinal = "lqr";
```

## Changing the Speed Reference

```matlab
% Constant speed (e.g. 5.0 m/s)
cfg.speed.mode = "constant";
cfg.speed.constant_value = 5.0;

% Speed profile from .mat file
cfg.speed.mode = "profile";
cfg.speed.profile_file = fullfile('data', 'reference_velocity', ...
    'referencePath_Velocity_peak_velocity_7.mat');
```

Available profiles: `peak_velocity_3.mat`, `peak_velocity_4.mat`, `peak_velocity_5.mat`, `peak_velocity_7.mat`.

## Comparing Controllers

```matlab
% In default_config.m:
cfg.controller.lateral = "mpc";
cfg.run.compare_longitudinal = true;
cfg.run.longitudinal_compare_set = ["pid", "lqr"];
```

This runs MPC+PID and MPC+LQR back-to-back with comparison plots.

## Running the Full Sweep (42 or 49 cases)

```matlab
run_tuning_sweep          % runs all test cases
run_tuning_sweep(true)    % dry-run: prints test matrix only
```

Tests 7 controller combinations × 7 speed conditions against GDP requirements:
- Peak lateral deviation < 0.6 m
- Peak longitudinal deviation < 0.8 m

## Project Structure

```
ControllerMatlab/
├── main.m                          Entry point (single run or comparison)
├── run_tuning_sweep.m              Automated 49-case GDP sweep
│
├── config/
│   └── default_config.m            All tuning parameters
│
├── reference/
│   ├── load_reference_path.m       Loads waypoints from path_ref.mat
│   ├── load_reference_speed.m      Loads speed profile or constant
│   ├── smooth_speed_profile.m      Rate-limits speed transitions
│   ├── path_yaw.m                  Computes heading from waypoints
│   └── path_curvature.m            Computes curvature from waypoints
│
├── controllers/
│   ├── lateral/
│   │   ├── mpc_lateral.m           MPC lateral-only (4 states, 1 input)
│   │   ├── mpc_combined.m          MPC lateral+longitudinal (6 states, 2 inputs)
│   │   ├── stanley_lateral.m       Stanley kinematic controller
│   │   └── pure_pursuit_lateral.m  Pure Pursuit geometric controller
│   └── longitudinal/
│       ├── PID_controller.m        PID speed controller
│       └── LQR_controller.m        LQR optimal speed controller
│
├── model/
│   ├── load_vehicle_params.m       Kia Niro parameters + actuator maps
│   ├── lateral_model.m             Plant wrapper (tires + dynamics)
│   ├── lateral_tire_model.m        Magic Formula tire forces
│   ├── longitudinal_model.m        Force balance + pedal-to-force maps
│   └── coupled_bicycle_dynamics.m  3-DOF coupled bicycle model
│
├── simulation/
│   └── run_closed_loop.m           Main simulation loop with timing
│
├── utils/
│   ├── angle_wrap.m                Wraps angle to [-pi, pi]
│   ├── nearest_path_ref_point.m    Segment projection + nearest point
│   ├── rate_limit.m                Signal rate limiter
│   └── track_errors.m              CTE and heading error computation
│
├── data/
│   ├── path_ref.mat                Reference path waypoints
│   ├── Acc_mapData_noSlope.mat     Throttle actuator map
│   ├── brake_mapData_noSlope.mat   Brake actuator map
│   └── reference_velocity/         Speed profiles (3/4/5/7 m/s)
│
└── run/                            Output directory (auto-created)
    └── <timestamp>_<controller>/
        ├── result.mat              Full simulation data
        ├── summary.txt             Performance metrics
        ├── path_tracking.png       Path plot
        ├── tracking_errors.png     CTE, heading, lon dev, speed error
        ├── speed_tracking.png      Speed + acceleration plots
        ├── lateral_dynamics.png    Steering + lateral states
        └── execution_timing.png    Controller computation time
```

## Vehicle Model

The simulation plant is a coupled 3-DOF planar bicycle model for the Kia Niro:

| Parameter | Value | Source |
|-----------|-------|--------|
| Mass | 1948 kg | Vehicle data |
| Wheelbase | 2.720 m | Vehicle data |
| Front axle to CG | 1.214 m | Vehicle data |
| Yaw inertia | 2500 kg·m² | Estimated |
| Resistance | 45 + 10v + 0.518v² N | Vehicle data |
| Max steering | ±35° at 40°/s | DBW spec |
| Steering delay | 0.1 s | Measured |
| Longitudinal delay | 0.1 s | Measured |
| Tire model | Pacejka Magic Formula | Estimated |
| Max accel (60% pedal) | 3.372 m/s² | From Acc_mapData |
| Max braking (60% pedal) | -7.357 m/s² | From brake_mapData |

**Pedal-to-force conversion:**
```
ACC_req = interp1(Force_full, Acc_full, F_tractive)
ACC_% = ACC_req / max(Acc_full) * 0.6
```
Same for braking. The 0.6 factor is a safety cap (60% max pedal).

**Steering output for DBW:** Internal computation uses radians. For the Sygnal drive-by-wire unit, normalise to [-1, +1]:
```
steer_normalized = delta_rad / max_steer_rad
% +1 = full left, -1 = full right
```

## GDP Requirements (Brief slide 13)

| REQ | Description | Status |
|-----|-------------|--------|
| REQ-1 | Peak lateral deviation < 0.6 m | MPC passes at 0.5–7.0 m/s |
| REQ-1 | Peak longitudinal deviation < 0.8 m | MPC+LQR passes at 0.5–2.0 m/s |
| REQ-2 | Controller loop ≥ 10 Hz | PASS (MPC combined: 10 Hz, others: 20 Hz) |
| REQ-3 | Model accounts for delays + dynamics | PASS (0.1s delays, 3DOF, Magic Formula) |

## Testing Scenarios (Brief slide 14)

| Scenario | Speed | Mode | Status |
|----------|-------|------|--------|
| GDP-S1 (Low) | 0.5 m/s | Constant | MPC+LQR: PASS |
| GDP-S2 (Medium) | 2.0 m/s | Constant | MPC+LQR: PASS |
| GDP-S3 (High) | 3/4/5/7 m/s | Profile | Lateral PASS, Longitudinal in progress |
| GDP-S3-EXT | 10.0 m/s | Constant | All FAIL (needs high-speed tuning) |

## Output Files Per Run

Each run produces these files in the `run/` directory:

| File | Contents |
|------|----------|
| `result.mat` | Full simulation data (log, metrics, config) |
| `summary.txt` | RMS/Peak for lateral + longitudinal + timing |
| `path_tracking.png` | Reference vs actual path |
| `tracking_errors.png` | 4 panels: CTE, heading error, lon deviation, speed error (with RMS/Peak annotations) |
| `speed_tracking.png` | Speed reference vs actual + acceleration commands |
| `lateral_dynamics.png` | Steering commands + lateral velocity/yaw rate |
| `execution_timing.png` | Controller computation time vs loop period |

## Expected Results to Show

For a complete presentation or report, show these for each controller tested:

1. **Path tracking plot** — reference vs actual trajectory
2. **Lateral error** — CTE time trace with RMS and peak values
3. **Heading error** — time trace with RMS
4. **Longitudinal deviation** — position error time trace with RMS and peak
5. **Speed tracking** — reference vs actual speed
6. **Speed error** — time trace with RMS and peak
7. **Acceleration commands** — commanded vs actual
8. **Steering commands** — commanded vs executed (shows delay + rate limit effect)
9. **Execution timing** — computation time vs loop period (proves real-time feasibility)
10. **Comparison table** — all controllers side-by-side with pass/fail
11. **Pass/fail heatmap** — controllers vs GDP scenarios

## Key Tuning Parameters

### MPC Combined (lateral + longitudinal)
```matlab
cfg.mpc_combined.N          % prediction horizon (steps)
cfg.mpc_combined.dt_ctrl    % control period (0.10 = 10 Hz)
cfg.mpc_combined.Q          % 6×6 state weights [ey, epsi, vy, r, ev, z_lon]
cfg.mpc_combined.R          % 2×2 input weights [delta, a_des]
cfg.mpc_combined.Rd         % 2×2 input rate weights
cfg.mpc_combined.a_min/max  % acceleration limits (from maps)
```

### MPC Lateral Only
```matlab
cfg.mpc.N, cfg.mpc.Q, cfg.mpc.R, cfg.mpc.Rd
```

### PID Longitudinal
```matlab
cfg.lon_pid.kp/ki/kd        % PID gains
cfg.lon_pid.a_min/a_max     % acceleration limits
```

### LQR Longitudinal
```matlab
cfg.lon_lqr.Q               % diag([speed_error, integral_error])
cfg.lon_lqr.R               % acceleration effort penalty
```

## Dependencies

- MATLAB R2020b or later
- Optimization Toolbox (for `quadprog` used by MPC)
- No external toolboxes required for PID/LQR/Stanley/Pure Pursuit