function lon_force = longitudinal_model(vx, a_des, veh)
    % Main longitudinal model wrapper used by the current project.
    % Internally it delegates the force generation to a standalone module.
    Velocity = vx;

    A = veh.A;
    B = veh.B;
    C = veh.C;
    M = veh.M;

    F_grad = 0;
    F_aero_plus_road = A + B * Velocity + C * Velocity^2;
    F_resist = F_aero_plus_road + F_grad;

    throttle_pct = 0;
    brake_pct = 0;
    ACC_req = 0;
    BRK_req = 0;
    F_drive_actual = 0;

    if a_des >= 0
        F_required = M * a_des + F_resist;
        F_tractive_required = F_required;

        ACC_req = invert_force_map_2d(F_tractive_required, Velocity, veh.acc, true);
        if isnan(ACC_req)
            ACC_req = interp1( ...
                veh.acc.force_full, ...
                veh.acc.acc_full, ...
                F_tractive_required, ...
                'linear', 'extrap');
        end

        throttle_pct = (ACC_req / max(veh.acc.acc_full)) * veh.max_pedal_publish;
        throttle_pct = min(max(throttle_pct, 0), veh.max_pedal_publish);

        ACC_internal = (throttle_pct / veh.max_pedal_publish) * max(veh.acc.acc_full);
        F_drive_actual = eval_force_map_2d(Velocity, ACC_internal, veh.acc, true);
        if isnan(F_drive_actual)
            F_drive_actual = interp1( ...
                veh.acc.acc_full, ...
                veh.acc.force_full, ...
                ACC_internal, ...
                'linear', 'extrap');
        end
    else
        F_required = M * a_des + F_resist;
        F_brake_required = max(-F_required, 0);

        BRK_req = invert_force_map_2d(F_brake_required, Velocity, veh.brk, false);
        if isnan(BRK_req)
            BRK_req = interp1( ...
                veh.brk.force_full, ...
                veh.brk.brake_full, ...
                F_brake_required, ...
                'linear', 'extrap');
        end

        brake_pct = (BRK_req / max(veh.brk.brake_full)) * veh.max_pedal_publish;
        brake_pct = min(max(brake_pct, 0), veh.max_pedal_publish);

        BRK_internal = (brake_pct / veh.max_pedal_publish) * max(veh.brk.brake_full);
        F_brake_actual = eval_force_map_2d(Velocity, BRK_internal, veh.brk, false);
        if isnan(F_brake_actual)
            F_brake_actual = interp1( ...
                veh.brk.brake_full, ...
                veh.brk.force_full, ...
                BRK_internal, ...
                'linear', 'extrap');
        end

        F_drive_actual = -F_brake_actual;
    end

    F_total = F_drive_actual - F_resist;
    a_actual = F_total / M;

    lon_force.a_des = a_des;
    lon_force.a_actual = a_actual;
    lon_force.throttle_pct = throttle_pct;
    lon_force.brake_pct = brake_pct;
    lon_force.ACC_req = ACC_req;
    lon_force.BRK_req = BRK_req;
    lon_force.F_grad = F_grad;
    lon_force.F_aero_plus_road = F_aero_plus_road;
    lon_force.F_resist = F_resist;
    lon_force.F_required = F_required;
    lon_force.F_drive_actual = F_drive_actual;
    lon_force.F_total = F_total;
end

function force = eval_force_map_2d(vx, cmd, map, is_accel)
    persistent acc_interp brk_interp

    vx_q = min(max(vx, map.vel_min), map.vel_max);
    cmd_q = min(max(cmd, map.cmd_min), map.cmd_max);

    try
        if is_accel
            if isempty(acc_interp)
                acc_interp = scatteredInterpolant(map.vel_samples, map.cmd_samples, map.force_samples, ...
                    'natural', 'nearest');
            end
            force = acc_interp(vx_q, cmd_q);
        else
            if isempty(brk_interp)
                brk_interp = scatteredInterpolant(map.vel_samples, map.cmd_samples, map.force_samples, ...
                    'natural', 'nearest');
            end
            force = brk_interp(vx_q, cmd_q);
        end
    catch
        force = NaN;
    end
end

function cmd = invert_force_map_2d(force_target, vx, map, is_accel)
    cmd_lo = map.cmd_min;
    cmd_hi = map.cmd_max;

    try
        f_lo = eval_force_map_2d(vx, cmd_lo, map, is_accel);
        f_hi = eval_force_map_2d(vx, cmd_hi, map, is_accel);

        if any(isnan([f_lo, f_hi]))
            cmd = NaN;
            return;
        end

        if force_target <= f_lo
            cmd = cmd_lo;
            return;
        end
        if force_target >= f_hi
            cmd = cmd_hi;
            return;
        end

        obj = @(u) (eval_force_map_2d(vx, u, map, is_accel) - force_target).^2;
        opts = optimset('TolX', 1e-4, 'Display', 'off');
        cmd = fminbnd(obj, cmd_lo, cmd_hi, opts);
    catch
        cmd = NaN;
    end
end
