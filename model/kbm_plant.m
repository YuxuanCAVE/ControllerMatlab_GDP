% Main lateral plant used by the project.
% Standard kinematic bicycle model (KBM) at the vehicle CG.
%
% Steering command convention in this project:
%   left turn  -> negative delta_cmd
%   right turn -> positive delta_cmd
%
%   X_dot   = V * cos(psi + beta(delta))
%   Y_dot   = V * sin(psi + beta(delta))
%   V_dot   = a_x
%   psi_dot = V / l_r * sin(beta(delta))
%
% where
%   beta(delta) = atan((l_r / (l_f + l_r)) * tan(delta))
%
% Input:
%   state, delta_cmd, lon_model, dt, veh
%
% Output:
%   updated state, longitudinal acceleration ax, and lateral debug struct
function [state, ax, lat] = kbm_plant(state, delta_cmd, lon_model, dt, veh)
    v = max(get_speed(state), 0.0);
    yaw = state.yaw;
    lf = veh.lf;
    lr = veh.lr;
    L = lf + lr;

    ax = lon_model.a_actual;
    v_next = max(0.0, v + ax * dt);
    v_mid = 0.5 * (v + v_next);

    beta_mid = kbm_beta(delta_cmd, lr, L);
    yaw_rate_mid = v_mid / max(lr, 1e-6) * sin(beta_mid);

    x_dot = v_mid * cos(yaw + beta_mid);
    y_dot = v_mid * sin(yaw + beta_mid);
    yaw_next = angle_wrap(yaw + yaw_rate_mid * dt);

    beta_next = kbm_beta(delta_cmd, lr, L);
    yaw_rate_next = v_next / max(lr, 1e-6) * sin(beta_next);

    state.x = state.x + x_dot * dt;
    state.y = state.y + y_dot * dt;
    state.yaw = yaw_next;
    state.v = v_next;
    state.vx = v_next * cos(beta_next);
    state.vy = v_next * sin(beta_next);
    state.r = yaw_rate_next;
    state.beta = beta_next;
    state.delta = delta_cmd;

    lat.vx = state.vx;
    lat.vy = state.vy;
    lat.r = state.r;
    lat.beta = state.beta;
    lat.ay = v_mid * yaw_rate_mid;
    lat.alpha_f = 0.0;
    lat.alpha_r = 0.0;
    lat.Fy_f = 0.0;
    lat.Fy_r = 0.0;
end

function beta = kbm_beta(delta_cmd, lr, L)
    % The KBM itself uses the standard mathematical steering sign
    % (left-positive). Convert from the project command convention
    % (left-negative, right-positive) here.
    beta = -atan((lr / max(L, 1e-6)) * tan(delta_cmd));
end

function v = get_speed(state)
    if isfield(state, 'v')
        v = state.v;
    elseif isfield(state, 'vx')
        v = state.vx;
    else
        v = 0.0;
    end
end
