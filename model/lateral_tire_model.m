function lat_force = lateral_tire_model(vx, vy, r, delta_cmd, veh)
    % Standalone lateral tire force model.
    % This module only computes slip angles and tire lateral forces.

    vx = max(vx, 0.5);

    alpha_f = atan2(vy + veh.lf * r, vx) - delta_cmd;
    alpha_r = atan2(vy - veh.lr * r, vx);

    Fy_f = magic_formula_lat(-alpha_f, veh.tire.front);
    Fy_r = magic_formula_lat(-alpha_r, veh.tire.rear);

    % At very low speed the slip-angle-based tire model can generate
    % unrealistically large lateral forces and yaw-rate oscillations.
    % Fade the tire forces in smoothly from standstill to nominal speed.
    low_speed_force_scale = smoothstep(vx, 0.5, 2.0);
    Fy_f = low_speed_force_scale * Fy_f;
    Fy_r = low_speed_force_scale * Fy_r;

    lat_force.alpha_f = alpha_f;
    lat_force.alpha_r = alpha_r;
    lat_force.Fy_f = Fy_f;
    lat_force.Fy_r = Fy_r;
end

function Fy = magic_formula_lat(alpha, tire)
    Fy = tire.D * sin(tire.C * atan(tire.B * alpha));
end

function y = smoothstep(x, x0, x1)
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
