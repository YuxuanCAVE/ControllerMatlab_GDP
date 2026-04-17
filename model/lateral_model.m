% Main lateral plant used by the project.
% This is a single kinematic bicycle model updated at the rear axle.
%
% Input:
%   state, delta_cmd, lon_model, dt, veh
%
% Output:
%   updated state, longitudinal acceleration ax, and lateral debug struct
function [state, ax, lat] = lateral_model(state, delta_cmd, lon_model, dt, veh)
    vx = max(get_vx(state), 0.0);
    yaw = state.yaw;
    L = veh.L;

    ax = lon_model.a_actual;
    vx_next = max(0.0, vx + ax * dt);
    vx_mid = 0.5 * (vx + vx_next);

    yaw_rate = vx_mid / max(L, 1e-6) * tan(delta_cmd);
    yaw_next = angle_wrap(yaw + yaw_rate * dt);

    x_dot = vx_mid * cos(yaw);
    y_dot = vx_mid * sin(yaw);

    state.x = state.x + x_dot * dt;
    state.y = state.y + y_dot * dt;
    state.yaw = yaw_next;
    state.v = vx_next;
    state.vx = vx_next;
    state.vy = 0.0;
    state.r = yaw_rate;
    state.beta = 0.0;
    state.delta = delta_cmd;

    lat.vx = vx_next;
    lat.vy = 0.0;
    lat.r = yaw_rate;
    lat.beta = 0.0;
    lat.ay = vx_mid * yaw_rate;
    lat.alpha_f = 0.0;
    lat.alpha_r = 0.0;
    lat.Fy_f = 0.0;
    lat.Fy_r = 0.0;
end

function vx = get_vx(state)
    if isfield(state, 'vx')
        vx = state.vx;
    else
        vx = state.v;
    end
end
