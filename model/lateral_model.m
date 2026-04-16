% Main lateral plant wrapper used by the project.
% Supported plant choices:
%   1) dynamic        -> nonlinear tire-force model
%   2) bicycle_linear -> linear bicycle tire-force model
%
% Input:
%   state, delta_cmd, lon_model, dt, veh, cfg
%
% Output:
%   updated state, longitudinal acceleration ax, and lateral debug struct
function [state, ax, lat] = lateral_model(state, delta_cmd, lon_model, dt, veh, cfg)
    vx = get_vx(state);

    lateral_model_type = "dynamic";
    if nargin >= 6 && isfield(cfg, 'plant') && isfield(cfg.plant, 'lateral_model')
        lateral_model_type = string(cfg.plant.lateral_model);
    end

    switch lateral_model_type
        case "bicycle_linear"
            lat_force = bicycle_linear_lateral_tire_model(vx, state.vy, state.r, delta_cmd, veh);
        otherwise
            lat_force = lateral_tire_model(vx, state.vy, state.r, delta_cmd, veh);
    end

    [state, dbg] = coupled_bicycle_dynamics(state, delta_cmd, lon_model, lat_force, veh, dt);

    ax = dbg.ax;
    lat.vx = get_vx(state);
    lat.vy = state.vy;
    lat.r = state.r;
    lat.beta = state.beta;
    lat.ay = dbg.ay;
    lat.alpha_f = dbg.alpha_f;
    lat.alpha_r = dbg.alpha_r;
    lat.Fy_f = dbg.Fy_f;
    lat.Fy_r = dbg.Fy_r;
end

function vx = get_vx(state)
    if isfield(state, 'vx')
        vx = state.vx;
    else
        vx = state.v;
    end
end

function lat_force = bicycle_linear_lateral_tire_model(vx, vy, r, delta_cmd, veh)
    vx = max(vx, 0.5);

    alpha_f = atan2(vy + veh.lf * r, vx) - delta_cmd;
    alpha_r = atan2(vy - veh.lr * r, vx);

    Fy_f = -veh.tire.front.Calpha * alpha_f;
    Fy_r = -veh.tire.rear.Calpha * alpha_r;

    % Low-speed fade-in to avoid overly aggressive yaw response near standstill.
    low_speed_force_scale = smoothstep_local(vx, 0.5, 2.0);
    Fy_f = low_speed_force_scale * Fy_f;
    Fy_r = low_speed_force_scale * Fy_r;

    % Keep the linear model comparable to the nonlinear plant by enforcing
    % the same peak lateral-force ceilings.
    Fy_f = min(max(Fy_f, -veh.tire.front.D), veh.tire.front.D);
    Fy_r = min(max(Fy_r, -veh.tire.rear.D), veh.tire.rear.D);

    lat_force.alpha_f = alpha_f;
    lat_force.alpha_r = alpha_r;
    lat_force.Fy_f = Fy_f;
    lat_force.Fy_r = Fy_r;
end

function y = smoothstep_local(x, x0, x1)
    if x <= x0
        y = 0.0;
        return;
    end
    if x >= x1
        y = 1.0;
        return;
    end

    t = (x - x0) / (x1 - x0);
    y = t * t * (3.0 - 2.0 * t);
end
