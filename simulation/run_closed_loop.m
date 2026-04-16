function result = run_closed_loop(cfg, ref, veh)
    dt = cfg.sim.dt;
    if isfield(cfg.sim, 'max_travel_time') && ~isempty(cfg.sim.max_travel_time)
        max_travel_time = cfg.sim.max_travel_time;
    else
        max_travel_time = cfg.sim.T_end;
    end
    Nsim = round(max_travel_time / dt);
    steer_delay_steps = max(round(cfg.vehicle.delay.steer_s / dt), 0);
    lon_delay_steps = max(round(cfg.vehicle.delay.longitudinal_s / dt), 0);

    state.x = ref.x(1);
    state.y = ref.y(1);
    state.yaw = ref.yaw(1);
    state.v = 0.5;
    state.vx = state.v;
    state.vy = 0.0;
    state.r = 0.0;
    state.beta = 0.0;
    state.delta = 0.0;
    state.z_lon = 0.0;

    use_combined_mpc = (cfg.controller.lateral == "mpc_combined");
    use_fake_lateral = (~use_combined_mpc && cfg.controller.lateral == "fake_controller");
    use_fake_longitudinal = (~use_combined_mpc && cfg.controller.longitudinal == "fake_controller");

    if ~use_combined_mpc
        switch cfg.controller.longitudinal
            case "pid", lon = cfg.lon_pid;
            case "lqr", lon = cfg.lon_lqr;
            case "lqr_force_balance", lon = cfg.lon_lqr_force;
            case "fake_controller", lon = struct();
            otherwise, error('Unknown longitudinal controller: %s', cfg.controller.longitudinal);
        end
    else
        lon = struct();
    end

    if use_combined_mpc
        ctrl_period = max(1, round(cfg.mpc_combined.dt_ctrl / dt));
        ctrl_dt = cfg.mpc_combined.dt_ctrl;
        last_delta_cmd = 0;
        last_a_des_cmd = 0;
    else
        ctrl_period = 1;
        ctrl_dt = dt;
    end

    idx_progress = 1;
    steer_delay_buffer = state.delta * ones(steer_delay_steps + 1, 1);
    lon_delay_buffer = zeros(lon_delay_steps + 1, 1);

    % ── Time-domain speed reference smoother ──────────────────────────
    % This rate-limits the speed reference AS THE VEHICLE MOVES THROUGH
    % TIME, not per waypoint index. Uses 85% of actual vehicle limits
    % to leave headroom for the feedback controller.
    a_max_ref = cfg.accel_limits.a_max * 0.85;  % e.g. 3.372 * 0.85 = 2.87
    a_min_ref = cfg.accel_limits.a_min * 0.85;  % e.g. -7.357 * 0.85 = -6.25
    v_ref_smooth = 0.5;  % start from initial vehicle speed

    % Preallocate logs
    log.t = zeros(Nsim, 1);
    log.x = zeros(Nsim, 1);
    log.y = zeros(Nsim, 1);
    log.yaw = zeros(Nsim, 1);
    log.v = zeros(Nsim, 1);
    log.v_ref = zeros(Nsim, 1);
    log.v_ref_raw = zeros(Nsim, 1);
    log.delta = zeros(Nsim, 1);
    log.delta_cmd_raw = zeros(Nsim, 1);
    log.delta_cmd_exec = zeros(Nsim, 1);
    log.ax = zeros(Nsim, 1);
    log.ax_cmd = zeros(Nsim, 1);
    log.ax_cmd_exec = zeros(Nsim, 1);
    log.throttle_raw = zeros(Nsim, 1);
    log.brake_raw = zeros(Nsim, 1);
    log.throttle = zeros(Nsim, 1);
    log.brake = zeros(Nsim, 1);
    log.ACC_req = zeros(Nsim, 1);
    log.BRK_req = zeros(Nsim, 1);
    log.vy = zeros(Nsim, 1);
    log.r = zeros(Nsim, 1);
    log.beta = zeros(Nsim, 1);
    log.ay = zeros(Nsim, 1);
    log.alpha_f = zeros(Nsim, 1);
    log.alpha_r = zeros(Nsim, 1);
    log.Fy_f = zeros(Nsim, 1);
    log.Fy_r = zeros(Nsim, 1);
    log.F_resist = zeros(Nsim, 1);
    log.F_required = zeros(Nsim, 1);
    log.F_total = zeros(Nsim, 1);
    log.F_drive = zeros(Nsim, 1);
    log.branch_mode = zeros(Nsim, 1);
    log.branch_drive = zeros(Nsim, 1);
    log.branch_brake = zeros(Nsim, 1);
    log.branch_coast = zeros(Nsim, 1);
    log.idx = zeros(Nsim, 1);
    log.e_ct = zeros(Nsim, 1);
    log.e_psi = zeros(Nsim, 1);
    log.z_lon = zeros(Nsim, 1);
    log.ctrl_exec_time_s = zeros(Nsim, 1);

    stop_idx = Nsim;
    goal_reached = false;
    termination_reason = "max_travel_time";

    for k = 1:Nsim
        [idx_near, e_ct, e_psi] = track_errors( ...
            state.x, state.y, state.yaw, ref, idx_progress, cfg.sim.progress_window);
        idx_progress = max(idx_progress, idx_near);

        % Raw reference speed from the profile/constant
        v_ref_raw = ref.v_ref(max(min(idx_near, numel(ref.v_ref)), 1));

        % ── TIME-DOMAIN rate limiting of speed reference ──────────────
        % This is the key fix: clamp how fast the reference speed can
        % change per simulation timestep IN TIME, not per waypoint.
        dv_desired = v_ref_raw - v_ref_smooth;
        dv_max = a_max_ref * dt;   % max increase per timestep
        dv_min = a_min_ref * dt;   % max decrease per timestep (negative)
        dv_clamped = min(max(dv_desired, dv_min), dv_max);
        v_ref_smooth = max(v_ref_smooth + dv_clamped, 0);
        v_ref_now = v_ref_smooth;
        % ──────────────────────────────────────────────────────────────

        % Update longitudinal deviation (using smoothed reference)
        speed_error_now = v_ref_now - state.v;
        state.z_lon = state.z_lon + speed_error_now * dt;

        if use_fake_longitudinal
            state.v = v_ref_now;
            state.vx = v_ref_now;
        end

        compute_ctrl = (mod(k-1, ctrl_period) == 0);

        ctrl_tic = tic;

        if use_combined_mpc
            if compute_ctrl
                [delta_cmd, a_des_raw] = mpc_combined(state, ref, veh, dt, ...
                    cfg.mpc_combined, idx_progress, cfg.sim.progress_window, ...
                    steer_delay_buffer, lon_delay_buffer);
                last_delta_cmd = delta_cmd;
                last_a_des_cmd = a_des_raw;
            else
                delta_cmd = last_delta_cmd;
                a_des_raw = last_a_des_cmd;
            end
        else
            switch cfg.controller.lateral
                case "stanley"
                    delta_cmd = stanley_lateral(state.x, state.y, state.yaw, state.v, ...
                        ref, veh.L, cfg.stanley, idx_progress, cfg.sim.progress_window);
                case "pure_pursuit"
                    delta_cmd = pure_pursuit_lateral(state.x, state.y, state.yaw, state.v, ...
                        ref, veh.L, cfg.pure_pursuit, idx_progress, cfg.sim.progress_window);
                case "mpc"
                    delta_cmd = mpc_lateral(state, ref, veh, dt, cfg.mpc, ...
                        idx_progress, cfg.sim.progress_window, steer_delay_buffer);
                case "mpc_kinematic"
                    delta_cmd = mpc_kinematic_lateral(state, ref, veh, dt, cfg.mpc_kinematic, ...
                        idx_progress, cfg.sim.progress_window, steer_delay_buffer);
                case "fake_controller"
                    delta_cmd = 0.0;
                otherwise
                    error('Unknown lateral controller: %s', cfg.controller.lateral);
            end

            switch cfg.controller.longitudinal
                case "pid"
                    [a_des_raw, lon] = PID_controller(v_ref_now, state.v, lon, dt);
                case "lqr"
                    [a_des_raw, lon] = LQR_controller(v_ref_now, state.v, lon, dt);
                case "lqr_force_balance"
                    [a_des_raw, lon] = longitudinal_lqr_force_balance_controller(v_ref_now, state.v, lon, dt, veh);
                case "fake_controller"
                    a_des_raw = 0.0;
                otherwise
                    error('Unknown longitudinal controller: %s', cfg.controller.longitudinal);
            end
        end

        ctrl_exec_time = toc(ctrl_tic);
        log.ctrl_exec_time_s(k) = ctrl_exec_time;

        delta_cmd_raw = max(min(delta_cmd, veh.max_steer), -veh.max_steer);
        [delta_cmd_delayed, steer_delay_buffer] = apply_delay(delta_cmd_raw, steer_delay_buffer);
        delta_cmd_exec = rate_limit(delta_cmd_delayed, state.delta, veh.max_steer_rate, dt);

        if use_fake_longitudinal
            a_des_exec = 0.0;
            lon_model = fake_longitudinal_model(state.v, veh);
        else
            [a_des_exec, lon_delay_buffer] = apply_delay(a_des_raw, lon_delay_buffer);
            [lon_model, lon] = longitudinal_model(state.v, a_des_exec, veh, lon);
        end

        if use_fake_lateral
            [state, ax_actual, lat, idx_progress] = fake_lateral_step( ...
                state, lon_model, dt, veh, ref, idx_progress, cfg.sim.progress_window);
        else
            [state, ax_actual, lat] = lateral_model(state, delta_cmd_exec, lon_model, dt, veh, cfg);
        end

        if use_fake_longitudinal
            state.v = v_ref_now;
            state.vx = v_ref_now;
            ax_actual = 0.0;
            lon_model.a_des = 0.0;
            lon_model.a_actual = 0.0;
            lon_model.F_total = 0.0;
        end

        log.t(k) = (k-1) * dt;
        log.x(k) = state.x;
        log.y(k) = state.y;
        log.yaw(k) = state.yaw;
        log.v(k) = state.v;
        log.v_ref(k) = v_ref_now;        % smoothed reference
        log.v_ref_raw(k) = v_ref_raw;    % original raw reference
        log.vy(k) = state.vy;
        log.r(k) = state.r;
        log.beta(k) = state.beta;
        log.delta(k) = state.delta;
        log.delta_cmd_raw(k) = delta_cmd_raw;
        log.delta_cmd_exec(k) = delta_cmd_exec;
        log.ax(k) = ax_actual;
        log.ax_cmd(k) = a_des_raw;
        log.ax_cmd_exec(k) = a_des_exec;
        log.throttle_raw(k) = lon_model.throttle_pct_raw;
        log.brake_raw(k) = lon_model.brake_pct_raw;
        log.throttle(k) = lon_model.throttle_pct;
        log.brake(k) = lon_model.brake_pct;
        log.ACC_req(k) = lon_model.ACC_req;
        log.BRK_req(k) = lon_model.BRK_req;
        log.ay(k) = lat.ay;
        log.alpha_f(k) = lat.alpha_f;
        log.alpha_r(k) = lat.alpha_r;
        log.Fy_f(k) = lat.Fy_f;
        log.Fy_r(k) = lat.Fy_r;
        log.F_resist(k) = lon_model.F_resist;
        log.F_required(k) = lon_model.F_required;
        log.F_total(k) = lon_model.F_total;
        log.F_drive(k) = lon_model.F_drive_actual;
        log.branch_mode(k) = lon_model.branch_mode;
        log.branch_drive(k) = lon_model.branch_drive;
        log.branch_brake(k) = lon_model.branch_brake;
        log.branch_coast(k) = lon_model.branch_coast;
        log.idx(k) = idx_near;
        log.e_ct(k) = e_ct;
        log.e_psi(k) = e_psi;
        log.z_lon(k) = state.z_lon;

        if idx_progress >= numel(ref.x) - 2 && hypot(state.x - ref.x(end), state.y - ref.y(end)) < 1.5
            stop_idx = k;
            goal_reached = true;
            termination_reason = "goal_reached";
            break;
        end
    end

    fields = fieldnames(log);
    for ii = 1:numel(fields)
        log.(fields{ii}) = log.(fields{ii})(1:stop_idx);
    end

    result.log = log;
    result.final_state = state;
    result.metrics.rms_cte = sqrt(mean(log.e_ct.^2));
    result.metrics.rms_epsi_deg = sqrt(mean(rad2deg(log.e_psi).^2));
    result.metrics.peak_cte = max(abs(log.e_ct));
    sp_err = log.v_ref - log.v;
    result.metrics.rms_speed_error = sqrt(mean(sp_err.^2));
    result.metrics.peak_speed_error = max(abs(sp_err));
    result.metrics.final_speed = log.v(end);
    result.metrics.single_loop_time_s = stop_idx * dt;
    result.metrics.max_travel_time_s = max_travel_time;
    result.metrics.goal_reached = goal_reached;
    result.metrics.timeout = ~goal_reached;
    result.metrics.termination_reason = termination_reason;
    result.metrics.lateral_controller = cfg.controller.lateral;

    lon_dev = cumsum(sp_err) * dt;
    result.metrics.peak_lon_dev = max(abs(lon_dev));
    result.metrics.rms_lon_dev = sqrt(mean(lon_dev.^2));

    if use_combined_mpc
        result.metrics.longitudinal_controller = "mpc_combined";
        result.metrics.controller = "mpc_combined";
    else
        result.metrics.longitudinal_controller = cfg.controller.longitudinal;
        result.metrics.controller = sprintf('%s + %s', ...
            char(cfg.controller.lateral), char(cfg.controller.longitudinal));
    end

    if use_combined_mpc
        compute_mask = mod((0:stop_idx-1)', ctrl_period) == 0;
    else
        compute_mask = true(stop_idx, 1);
    end
    ctrl_times = log.ctrl_exec_time_s(compute_mask);
    result.metrics.ctrl_loop_dt = ctrl_dt;
    result.metrics.ctrl_freq_hz = 1 / ctrl_dt;
    result.metrics.ctrl_exec_mean_s = mean(ctrl_times);
    result.metrics.ctrl_exec_max_s = max(ctrl_times);
    result.metrics.ctrl_exec_std_s = std(ctrl_times);
    result.metrics.ctrl_realtime_ok = (max(ctrl_times) < ctrl_dt);
end

function [state_next, ax, lat, idx_progress] = fake_lateral_step(state, lon_force, dt, veh, ref, idx_hint, window)
    v_curr = max(state.v, 0.0);
    ax = lon_force.a_actual;
    v_next = max(0.0, v_curr + ax * dt);
    ds = max(0.0, 0.5 * (v_curr + v_next) * dt);

    [~, ~, ~, ~, ~, seg_idx, seg_t] = nearest_path_ref_point( ...
        state.x, state.y, ref.x, ref.y, idx_hint, window);
    [x_next, y_next, yaw_next, idx_progress] = advance_along_reference(ref, seg_idx, seg_t, ds);

    state_next = state;
    state_next.x = x_next;
    state_next.y = y_next;
    state_next.yaw = yaw_next;
    state_next.v = v_next;
    state_next.vx = v_next;
    state_next.vy = 0.0;
    state_next.r = 0.0;
    state_next.beta = 0.0;
    state_next.delta = 0.0;

    lat.vx = v_next;
    lat.vy = 0.0;
    lat.r = 0.0;
    lat.beta = 0.0;
    lat.ay = 0.0;
    lat.alpha_f = 0.0;
    lat.alpha_r = 0.0;
    lat.Fy_f = 0.0;
    lat.Fy_r = 0.0;
end

function [x, y, yaw, idx_out] = advance_along_reference(ref, seg_idx, seg_t, ds)
    n = numel(ref.x);
    if n < 2
        x = ref.x(1);
        y = ref.y(1);
        yaw = ref.yaw(1);
        idx_out = 1;
        return;
    end

    if isnan(seg_idx) || seg_idx < 1
        seg_idx = 1;
    end
    seg_idx = min(seg_idx, n - 1);
    seg_t = min(max(seg_t, 0.0), 1.0);

    idx = seg_idx;
    t = seg_t;
    remaining = ds;

    while idx < n
        Ax = ref.x(idx);
        Ay = ref.y(idx);
        Bx = ref.x(idx + 1);
        By = ref.y(idx + 1);
        seg_len = hypot(Bx - Ax, By - Ay);

        if seg_len < 1e-9
            idx = idx + 1;
            t = 0.0;
            continue;
        end

        seg_remaining = (1.0 - t) * seg_len;
        if remaining <= seg_remaining
            t = t + remaining / seg_len;
            x = Ax + t * (Bx - Ax);
            y = Ay + t * (By - Ay);
            yaw = atan2(By - Ay, Bx - Ax);
            idx_out = idx;
            return;
        end

        remaining = remaining - seg_remaining;
        idx = idx + 1;
        t = 0.0;
    end

    x = ref.x(end);
    y = ref.y(end);
    yaw = ref.yaw(end);
    idx_out = n;
end

function lon_force = fake_longitudinal_model(vx, veh)
    F_resist = veh.A + veh.B * vx + veh.C * vx^2;

    lon_force.a_des = 0.0;
    lon_force.a_actual = 0.0;
    lon_force.throttle_pct_raw = 0.0;
    lon_force.brake_pct_raw = 0.0;
    lon_force.throttle_pct = 0.0;
    lon_force.brake_pct = 0.0;
    lon_force.ACC_req = 0.0;
    lon_force.BRK_req = 0.0;
    lon_force.F_grad = 0.0;
    lon_force.F_aero_plus_road = F_resist;
    lon_force.F_resist = F_resist;
    lon_force.F_required = 0.0;
    lon_force.F_drive_actual = F_resist;
    lon_force.F_total = 0.0;
    lon_force.branch_mode = 0.0;
    lon_force.branch_drive = 0.0;
    lon_force.branch_brake = 0.0;
    lon_force.branch_coast = 1.0;
    lon_force.F_branch_eps = 1.0;
end


function [u_exec, buffer] = apply_delay(u_cmd, buffer)
    if numel(buffer) <= 1
        u_exec = u_cmd;
        buffer(1) = u_cmd;
        return;
    end
    u_exec = buffer(1);
    buffer(1:end-1) = buffer(2:end);
    buffer(end) = u_cmd;
end
