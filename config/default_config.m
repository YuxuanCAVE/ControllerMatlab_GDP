function cfg = default_config()
    cfg.controller.lateral = "nmpc_kbm";  % "stanley" | "pure_pursuit" | "mpc_kinematic" | "nmpc_kbm" | "fake_controller"
    cfg.controller.longitudinal = "pid";  % "pid" | "fake_controller"

    cfg.sim.dt = 0.1;
    cfg.sim.T_end = 300;
    cfg.sim.max_travel_time = cfg.sim.T_end;
    cfg.sim.progress_window = 5;

    cfg.ref.path_file = fullfile('data', 'path_ref.mat');

    cfg.vehicle.accel_map_file = fullfile('data', 'Acc_mapData_noSlope.mat');
    cfg.vehicle.brake_map_file = fullfile('data', 'brake_mapData_noSlope.mat');
    cfg.vehicle.max_steer = deg2rad(35);          % max steering angle (rad)
    cfg.vehicle.delay.steer_s = 0.;              % steering actuator delay (s)
    cfg.vehicle.delay.longitudinal_s = 0.;       % throttle/brake actuator delay (s)
    steer_rate_limit = deg2rad(35);               % controller steering-rate limit (rad/s)

    cfg.init.mode = "path_pose";                  % "path_pose" | "global_pose"
    cfg.init.anchor_mode = "index";               % "index"
    cfg.init.path_index = 20;
    cfg.init.ex0_m = 0.0;                         % tangent offset [m]
    cfg.init.ey0_m = -10.0;                         % normal offset [m]
    cfg.init.yaw_offset_deg = 0.0;
    cfg.init.v0_mps = 0;

    cfg.speed.mode = "profile";                  % "constant" | "profile"
    cfg.speed.constant_value = 3;                 % used when mode = "constant" (m/s)
    cfg.speed.profile_file = fullfile( ...
        'data', 'reference_velocity', 'referencePath_Velocity_peak_velocity_3.mat');

    cfg.accel_limits.a_max = 3.372;
    cfg.accel_limits.a_min = -7.357;

    cfg.stanley.k_cte = 1.8;
    cfg.stanley.k_soft = 2.0;
    cfg.stanley.k_yaw = 1.4;
    cfg.stanley.delta_ff_gain = 1.0;

    cfg.pure_pursuit.Ld_min = 4.0;
    cfg.pure_pursuit.Ld_gain = 0.70;
    cfg.pure_pursuit.Ld_max = 16.0;
    cfg.pure_pursuit.k_pp = 1.0;
    cfg.pure_pursuit.delta_ff_gain = 0.8;

    % Linear MPC lateral only (global-error + linearised full KBM)
    cfg.mpc_kinematic.Ts = cfg.sim.dt;
    cfg.mpc_kinematic.N = 5;
    cfg.mpc_kinematic.q_X = 20.0;
    cfg.mpc_kinematic.q_Y = 20.0;
    cfg.mpc_kinematic.q_psi = 1.0;
    cfg.mpc_kinematic.r_delta = 5.2;
    cfg.mpc_kinematic.r_u = 0.12;
    cfg.mpc_kinematic.r_du = 0.3;
    cfg.mpc_kinematic.delta_min = -cfg.vehicle.max_steer;
    cfg.mpc_kinematic.delta_max = cfg.vehicle.max_steer;
    cfg.mpc_kinematic.u_min = -steer_rate_limit;
    cfg.mpc_kinematic.u_max = steer_rate_limit;
    cfg.mpc_kinematic.use_ay_constraint = false;
    cfg.mpc_kinematic.ay_max = 4.0;

    % Nonlinear MPC lateral only (global-error + full nonlinear KBM)
    cfg.nmpc_kbm.Ts = cfg.sim.dt;
    cfg.nmpc_kbm.N = 5;
    cfg.nmpc_kbm.q_X = 32.0;
    cfg.nmpc_kbm.q_Y = 32.0;
    cfg.nmpc_kbm.q_psi = 4.0;
    cfg.nmpc_kbm.r_delta = 32;
    cfg.nmpc_kbm.r_du = 32;
    cfg.nmpc_kbm.delta_min = -cfg.vehicle.max_steer;
    cfg.nmpc_kbm.delta_max = cfg.vehicle.max_steer;
    cfg.nmpc_kbm.delta_rate_max = steer_rate_limit;
    cfg.nmpc_kbm.u_min = -steer_rate_limit;
    cfg.nmpc_kbm.u_max = steer_rate_limit;
    cfg.nmpc_kbm.use_ay_constraint = false;
    cfg.nmpc_kbm.ay_max = 4.0;
    cfg.nmpc_kbm.max_iterations = 100;
    cfg.nmpc_kbm.max_fun_evals = 4000;
    cfg.nmpc_kbm.u_init = 0.0;

    % PID longitudinal
    cfg.lon_pid.kp = 1.6;
    cfg.lon_pid.ki = 0.0;
    cfg.lon_pid.kd = 0.0;
    cfg.lon_pid.a_min = cfg.accel_limits.a_min;
    cfg.lon_pid.a_max = cfg.accel_limits.a_max;
    cfg.lon_pid.i_term = 0;
    cfg.lon_pid.prev_ev = 0;

    cfg.run.root_dir = 'run';
end
