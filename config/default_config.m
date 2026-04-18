function cfg = default_config()
    cfg.controller.lateral = "mpc_kbm";  % "stanley" | "pure_pursuit" | "mpc_kinematic" | "nmpc_kbm" | "fake_controller"
    cfg.controller.longitudinal = "pid";      % "pid" | "lqr" | "lqr_force_balance" | "fake_controller"

    cfg.sim.dt = 0.1;
    cfg.sim.T_end = 150;
    cfg.sim.max_travel_time = cfg.sim.T_end;
    cfg.sim.progress_window = 5;

    cfg.ref.path_file = fullfile('data', 'path_ref.mat');

    cfg.vehicle.accel_map_file = fullfile('data', 'Acc_mapData_noSlope.mat');
    cfg.vehicle.brake_map_file = fullfile('data', 'brake_mapData_noSlope.mat');
    cfg.vehicle.max_steer = deg2rad(35);       % max steering angle (rad)
    cfg.vehicle.max_steer_rate = deg2rad(35);   % max steering rate (rad/s)
    cfg.vehicle.delay.steer_s = 0.1;            % steering actuator delay (s)
    cfg.vehicle.delay.longitudinal_s = 0.1;     % throttle/brake actuator delay (s)

    % ── Initial vehicle pose ───────────────────────────────────────────
    cfg.init.mode = "path_pose";       % "path_pose" | "global_pose"
    cfg.init.anchor_mode = "index";    % "index"
    cfg.init.path_index = 1;
    cfg.init.ex0_m = 0.0;              % tangent offset [m]
    cfg.init.ey0_m = 0.0;              % normal offset [m]
    cfg.init.yaw_offset_deg = 0.0;
    cfg.init.v0_mps = 0.5;

    % ── Speed reference ───────────────────────────────────────────────
    % To use constant speed: set mode = "constant" and constant_value
    % To use speed profile:  set mode = "profile" and profile_file
    cfg.speed.mode = "constant";   % "constant" | "profile"
    cfg.speed.constant_value = 3;  % used when mode = "constant" (m/s)
    cfg.speed.profile_file = fullfile( ...
        'data', 'reference_velocity', 'referencePath_Velocity_peak_velocity_3.mat');
    % Available profiles: peak_velocity_3, 4, 5, 7 .mat

    % ── Acceleration limits from actuator maps ────────────────────────
    % Computed from Acc_mapData_noSlope.mat and brake_mapData_noSlope.mat
    % at 60% pedal cap (max_pedal_publish = 0.6)
    %   a_max = max(Force_full) / M = 3.372 m/s^2
    %   a_min = -max(brake Force_full) / M = -7.357 m/s^2
    cfg.accel_limits.a_max = 3.372;
    cfg.accel_limits.a_min = -7.357;

    % ── Stanley lateral ───────────────────────────────────────────────
    cfg.stanley.k_cte = 1.8;
    cfg.stanley.k_soft = 2.0;
    cfg.stanley.k_yaw = 1.4;
    cfg.stanley.delta_ff_gain = 1.0;

    % ── Pure Pursuit lateral ──────────────────────────────────────────
    cfg.pure_pursuit.Ld_min = 4.0;
    cfg.pure_pursuit.Ld_gain = 0.70;
    cfg.pure_pursuit.Ld_max = 16.0;
    cfg.pure_pursuit.k_pp = 1.0;
    cfg.pure_pursuit.delta_ff_gain = 0.8;

    % Linear MPC lateral only (global-error + linearised full KBM)
    cfg.mpc_kinematic.Ts = cfg.sim.dt;
    cfg.mpc_kinematic.N = 5;
    cfg.mpc_kinematic.q_X = 5.0;
    cfg.mpc_kinematic.q_Y = 5.0;
    cfg.mpc_kinematic.q_psi = 3.0;
    cfg.mpc_kinematic.r_delta = 0.2;
    cfg.mpc_kinematic.r_u = 0.12;
    cfg.mpc_kinematic.r_du = 0.3;
    cfg.mpc_kinematic.delta_min = -cfg.vehicle.max_steer;
    cfg.mpc_kinematic.delta_max = cfg.vehicle.max_steer;
    cfg.mpc_kinematic.u_min = -cfg.vehicle.max_steer_rate;
    cfg.mpc_kinematic.u_max = cfg.vehicle.max_steer_rate;
    cfg.mpc_kinematic.use_ay_constraint = false;
    cfg.mpc_kinematic.ay_max = 4.0;

    % Nonlinear MPC lateral only (global-error + full nonlinear KBM)
    cfg.nmpc_kbm.Ts = cfg.sim.dt;
    cfg.nmpc_kbm.N = 15;
    cfg.nmpc_kbm.q_X = 2.0;
    cfg.nmpc_kbm.q_Y = 8.0;
    cfg.nmpc_kbm.q_psi = 3.0;
    cfg.nmpc_kbm.r_delta = 0.2;
    cfg.nmpc_kbm.r_u = 0.05;
    cfg.nmpc_kbm.delta_min = -cfg.vehicle.max_steer;
    cfg.nmpc_kbm.delta_max = cfg.vehicle.max_steer;
    cfg.nmpc_kbm.u_min = -cfg.vehicle.max_steer_rate;
    cfg.nmpc_kbm.u_max = cfg.vehicle.max_steer_rate;
    cfg.nmpc_kbm.use_ay_constraint = false;
    cfg.nmpc_kbm.ay_max = 4.0;
    cfg.nmpc_kbm.max_iterations = 100;
    cfg.nmpc_kbm.max_fun_evals = 4000;
    cfg.nmpc_kbm.u_init = 0.0;

    % ── PID longitudinal ──────────────────────────────────────────────
    cfg.lon_pid.kp = 1.6;
    cfg.lon_pid.ki = 0.0;
    cfg.lon_pid.kd = 0.0;
    cfg.lon_pid.a_min = cfg.accel_limits.a_min;
    cfg.lon_pid.a_max = cfg.accel_limits.a_max;
    cfg.lon_pid.i_term = 0;
    cfg.lon_pid.prev_ev = 0;

    % ── LQR longitudinal ─────────────────────────────────────────────
    cfg.lon_lqr.Q = diag([10, 3]);
    cfg.lon_lqr.R = 1.5;
    cfg.lon_lqr.a_min = cfg.accel_limits.a_min;
    cfg.lon_lqr.a_max = cfg.accel_limits.a_max;
    cfg.lon_lqr.int_error = 0.0;
    cfg.lon_lqr.prev_a_des = 0.0;
    cfg.lon_lqr.drive_mode = "coast";
    cfg.lon_lqr.ev_gate = 0.05;
    cfg.lon_lqr.a_gate = 0.12;
    cfg.lon_lqr.a_deadband = 0.005;
    cfg.lon_lqr.a_hyst_enter_drive = 0.05;
    cfg.lon_lqr.a_hyst_exit_drive = 0.02;
    cfg.lon_lqr.a_hyst_enter_brake = 0.05;
    cfg.lon_lqr.a_hyst_exit_brake = 0.02;
    cfg.lon_lqr.cmd_lowpass_tau_s = 0.0;
    cfg.lon_lqr.a_rate_up_max = inf;
    cfg.lon_lqr.a_rate_down_max = inf;

    % Force-balance LQR longitudinal
    cfg.lon_lqr_force.Q = diag([120, 8]);
    cfg.lon_lqr_force.R = 2e-6;
    cfg.lon_lqr_force.a_min = cfg.accel_limits.a_min;
    cfg.lon_lqr_force.a_max = cfg.accel_limits.a_max;
    cfg.lon_lqr_force.int_error = 0.0;
    cfg.lon_lqr_force.prev_F_cmd = 0.0;
    cfg.lon_lqr_force.prev_a_des = 0.0;
    cfg.lon_lqr_force.last_v_lin = 0.0;
    cfg.lon_lqr_force.last_delta_F = 0.0;
    cfg.lon_lqr_force.v_lin_min = 0.5;

    % ── Run settings ──────────────────────────────────────────────────
    cfg.run.root_dir = 'run';
end
