function [a_des, lon] = longitudinal_lqr_force_balance_controller(v_ref, v, lon, dt, veh)
    % Force-balance longitudinal LQR around the current reference speed.
    %
    % Plant used for controller design:
    %   M * v_dot = F_drive - F_resist(v)
    %   F_resist(v) = A + B*v + C*v^2
    %
    % Linearized around v_lin = max(v_ref, v_lin_min):
    %   delta_v_dot = a_v * delta_v + b_v * delta_F
    %   a_v = -(dF_resist/dv) / M
    %   b_v = 1 / M
    %
    % Using tracking error e_v = v_ref - v and integral state z:
    %   e_v_dot = a_v * e_v - b_v * delta_F
    %   z_dot   = e_v
    %
    % The controller solves for delta_F around the equilibrium force needed
    % to balance resistance at v_lin. The final output is still a_des so it
    % can drop into the existing actuator and plant chain.

    lon = ensure_force_lqr_state(lon);

    v_lin = max(v_ref, lon.v_lin_min);
    ev = v_ref - v;
    lon.int_error = lon.int_error + ev * dt;

    dFdv = veh.B + 2 * veh.C * v_lin;
    a_v = -dFdv / veh.M;
    b_v = 1 / veh.M;

    A = [1 + a_v * dt, 0.0;
         dt,           1.0];
    B = [-b_v * dt;
          0.0];

    x = [ev; lon.int_error];
    K = solve_dlqr_gain(A, B, lon.Q, lon.R);
    delta_F = -K * x;

    F_eq = veh.A + veh.B * v_lin + veh.C * v_lin^2;
    F_cmd = F_eq + delta_F;

    F_min = veh.M * lon.a_min + resistance_force(veh, v);
    F_max = veh.M * lon.a_max + resistance_force(veh, v);
    F_cmd = min(max(F_cmd, F_min), F_max);

    a_des = (F_cmd - resistance_force(veh, v)) / veh.M;
    a_des = min(max(a_des, lon.a_min), lon.a_max);

    lon.prev_F_cmd = F_cmd;
    lon.prev_a_des = a_des;
    lon.last_v_lin = v_lin;
    lon.last_delta_F = delta_F;
end

function F_resist = resistance_force(veh, v)
    F_resist = veh.A + veh.B * v + veh.C * v.^2;
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

function lon = ensure_force_lqr_state(lon)
    if ~isfield(lon, 'int_error') || isempty(lon.int_error)
        lon.int_error = 0.0;
    end
    if ~isfield(lon, 'prev_F_cmd') || isempty(lon.prev_F_cmd)
        lon.prev_F_cmd = 0.0;
    end
    if ~isfield(lon, 'prev_a_des') || isempty(lon.prev_a_des)
        lon.prev_a_des = 0.0;
    end
    if ~isfield(lon, 'last_v_lin') || isempty(lon.last_v_lin)
        lon.last_v_lin = 0.0;
    end
    if ~isfield(lon, 'last_delta_F') || isempty(lon.last_delta_F)
        lon.last_delta_F = 0.0;
    end
    if ~isfield(lon, 'v_lin_min') || isempty(lon.v_lin_min)
        lon.v_lin_min = 0.5;
    end
end
