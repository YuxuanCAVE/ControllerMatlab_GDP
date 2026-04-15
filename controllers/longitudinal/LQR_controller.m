function [a_des, lon] = LQR_controller(v_ref, v, lon, dt)
    % Longitudinal kinematic model:
    %   v(k+1) = v(k) + a_des(k) * dt
    %
    % Augmented error-state model for tracking:
    %   x = [e_v; z]
    %   e_v = v_ref - v
    %   z   = integral(e_v)
    %
    % Discrete dynamics:
    %   x(k+1) = A x(k) + B u(k)
    % with
    %   A = [1  0;
    %        dt 1]
    %   B = [-dt;
    %         0]
    % and control input u = a_des.

    lon = ensure_lqr_state(lon);

    ev = v_ref - v;
    int_error_prev = lon.int_error;
    lon.int_error = lon.int_error + ev * dt;

    x = [ev; lon.int_error];
    A = [1.0, 0.0;
         dt, 1.0];
    B = [-dt;
          0.0];

    K = solve_dlqr_gain(A, B, lon.Q, lon.R);
    a_des_raw = -K * x;
    a_des_raw = min(max(a_des_raw, lon.a_min), lon.a_max);

    % Only apply anti-chatter logic when the vehicle is already close to
    % the target speed. Large tracking errors should not be throttled.
    anti_chatter_active = abs(ev) < lon.ev_gate && abs(a_des_raw) < lon.a_gate;

    if anti_chatter_active
        a_des_shaped = apply_accel_deadband(a_des_raw, lon);
        [a_des_shaped, lon] = apply_mode_hysteresis(a_des_shaped, lon);

        % Avoid integrator wind-up while the command is intentionally held.
        if abs(a_des_shaped) < 1e-9
            lon.int_error = int_error_prev;
        end

        if lon.cmd_lowpass_tau_s > 0
            alpha = dt / (lon.cmd_lowpass_tau_s + dt);
            a_des_filt = lon.prev_a_des + alpha * (a_des_shaped - lon.prev_a_des);
        else
            a_des_filt = a_des_shaped;
        end

        max_step_up = lon.a_rate_up_max * dt;
        max_step_down = lon.a_rate_down_max * dt;
        if isfinite(max_step_up) && isfinite(max_step_down)
            da = a_des_filt - lon.prev_a_des;
            da = min(max(da, -max_step_down), max_step_up);
            a_des = lon.prev_a_des + da;
        else
            a_des = a_des_filt;
        end
    else
        % Outside the near-cruise region, keep the original LQR behaviour.
        if a_des_raw > 0
            lon.drive_mode = "drive";
        elseif a_des_raw < 0
            lon.drive_mode = "brake";
        else
            lon.drive_mode = "coast";
        end
        a_des = a_des_raw;
    end

    a_des = min(max(a_des, lon.a_min), lon.a_max);

    lon.prev_a_des = a_des;
end

function K = solve_dlqr_gain(A, B, Q, R)
    P = Q;

    for k = 1:200
        BtPB = B' * P * B;
        G = R + BtPB;
        P_next = A' * P * A - (A' * P * B) * (G \ (B' * P * A)) + Q;

        if norm(P_next - P, 'fro') < 1e-9
            P = P_next;
            break;
        end

        P = P_next;
    end

    K = (R + B' * P * B) \ (B' * P * A);
end

function lon = ensure_lqr_state(lon)
    if ~isfield(lon, 'int_error') || isempty(lon.int_error)
        lon.int_error = 0.0;
    end
    if ~isfield(lon, 'prev_a_des') || isempty(lon.prev_a_des)
        lon.prev_a_des = 0.0;
    end
    if ~isfield(lon, 'drive_mode') || isempty(lon.drive_mode)
        lon.drive_mode = "coast";
    end
end

function a_cmd = apply_accel_deadband(a_cmd, lon)
    if abs(a_cmd) < lon.a_deadband
        a_cmd = 0.0;
    end
end

function [a_cmd, lon] = apply_mode_hysteresis(a_cmd, lon)
    switch lon.drive_mode
        case "drive"
            if a_cmd < -lon.a_hyst_enter_brake
                lon.drive_mode = "brake";
            elseif a_cmd < lon.a_hyst_exit_drive
                lon.drive_mode = "coast";
                a_cmd = 0.0;
            end
        case "brake"
            if a_cmd > lon.a_hyst_enter_drive
                lon.drive_mode = "drive";
            elseif a_cmd > -lon.a_hyst_exit_brake
                lon.drive_mode = "coast";
                a_cmd = 0.0;
            end
        otherwise
            if a_cmd > lon.a_hyst_enter_drive
                lon.drive_mode = "drive";
            elseif a_cmd < -lon.a_hyst_enter_brake
                lon.drive_mode = "brake";
            else
                a_cmd = 0.0;
            end
    end

    switch lon.drive_mode
        case "drive"
            a_cmd = max(a_cmd, 0.0);
        case "brake"
            a_cmd = min(a_cmd, 0.0);
        otherwise
            a_cmd = 0.0;
    end
end
