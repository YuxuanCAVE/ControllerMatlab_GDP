function cfg = default_config()
    cfg.controller.lateral = "mpc";         % "stanley" | "pure_pursuit" | "mpc"
    cfg.controller.longitudinal = "lqr";    % "pid" | "lqr"

    cfg.sim.dt = 0.05;
    cfg.sim.T_end = 150;
    cfg.sim.max_travel_time = cfg.sim.T_end;
    cfg.sim.progress_window = 80;

    cfg.ref.path_file = fullfile('data', 'path_ref.mat');

    cfg.vehicle.accel_map_file = fullfile('data', 'Acc_mapData_noSlope.mat');
    cfg.vehicle.brake_map_file = fullfile('data', 'brake_mapData_noSlope.mat');
    cfg.vehicle.max_steer = deg2rad(35);
    cfg.vehicle.max_steer_rate = deg2rad(70);
    cfg.vehicle.delay.steer_s = 0.1;
    cfg.vehicle.delay.longitudinal_s = 0.1;

    cfg.speed.mode = "profile";   % "constant" | "profile"
    cfg.speed.constant_value = 7.0;
    cfg.speed.profile_file = fullfile( ...
        'data', 'reference_velocity', 'referencePath_Velocity_peak_velocity_7.mat');

    cfg.stanley.k_cte = 1.8;
    cfg.stanley.k_soft = 2.0;
    cfg.stanley.k_yaw = 1.4;
    cfg.stanley.delta_ff_gain = 1.0;

    cfg.pure_pursuit.Ld_min = 4.0;
    cfg.pure_pursuit.Ld_gain = 0.70;
    cfg.pure_pursuit.Ld_max = 16.0;
    cfg.pure_pursuit.k_pp = 1.0;
    cfg.pure_pursuit.delta_ff_gain = 0.8;

    cfg.mpc.N = 25;
    cfg.mpc.Q = diag([15, 12, 8, 10]);
    cfg.mpc.R = 5;
    cfg.mpc.Rd = 15.0;
    cfg.mpc.kappa_ff_gain = 0.5;
    cfg.mpc.max_steer = cfg.vehicle.max_steer;

    cfg.lon_pid.kp = 0.8;
    cfg.lon_pid.ki = 0.30;
    cfg.lon_pid.kd = 0.02;
    cfg.lon_pid.a_min = -3.0;
    cfg.lon_pid.a_max = 2.0;
    cfg.lon_pid.i_term = 0;
    cfg.lon_pid.prev_ev = 0;

    % Longitudinal kinematic model for LQR:
    %   v(k+1) = v(k) + a_des(k) * dt
    % Error-state augmentation:
    %   e_v(k+1) = e_v(k) - dt * a_des(k)
    %   z(k+1)   = z(k) + dt * e_v(k)
    cfg.lon_lqr.Q = diag([8.0, 1.5]);   % [speed_error, integrated_speed_error]
    cfg.lon_lqr.R = 0.8;                % acceleration effort penalty
    cfg.lon_lqr.a_min = -3.0;
    cfg.lon_lqr.a_max = 2.0;
    cfg.lon_lqr.int_error = 0.0;

    cfg.run.root_dir = 'run';
    cfg.run.compare_longitudinal = true;
    cfg.run.longitudinal_compare_set = ["pid", "lqr"];
end
