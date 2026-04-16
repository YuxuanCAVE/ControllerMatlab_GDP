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

    throttle_pct_raw = 0;
    brake_pct_raw = 0;
    throttle_pct = 0;
    brake_pct = 0;
    ACC_req = 0;
    BRK_req = 0;
    F_drive_actual = 0;

    F_required = M * a_des + F_resist;

    if F_required >= 0
        F_tractive_required = F_required;

        ACC_req = invert_force_map_1d(F_tractive_required, veh.acc);
        if isnan(ACC_req)
            ACC_req = interp1( ...
                veh.acc.force_full, ...
                veh.acc.acc_full, ...
                F_tractive_required, ...
                'linear', 'extrap');
        end

        throttle_pct_raw = (ACC_req / max(veh.acc.acc_full)) * veh.max_pedal_publish;
        throttle_pct_raw = min(max(throttle_pct_raw, 0), veh.max_pedal_publish);
        throttle_pct = throttle_pct_raw;
        brake_pct = brake_pct_raw;

        ACC_internal = (throttle_pct / veh.max_pedal_publish) * max(veh.acc.acc_full);
        F_drive_actual = eval_force_map_1d(ACC_internal, veh.acc);
        if isnan(F_drive_actual)
            F_drive_actual = interp1( ...
                veh.acc.acc_full, ...
                veh.acc.force_full, ...
                ACC_internal, ...
                'linear', 'extrap');
        end
    else
        F_brake_required = max(-F_required, 0);

        BRK_req = invert_force_map_1d(F_brake_required, veh.brk);
        if isnan(BRK_req)
            BRK_req = interp1( ...
                veh.brk.force_full, ...
                veh.brk.brake_full, ...
                F_brake_required, ...
                'linear', 'extrap');
        end

        brake_pct_raw = (BRK_req / max(veh.brk.brake_full)) * veh.max_pedal_publish;
        brake_pct_raw = min(max(brake_pct_raw, 0), veh.max_pedal_publish);
        throttle_pct = throttle_pct_raw;
        brake_pct = brake_pct_raw;

        BRK_internal = (brake_pct / veh.max_pedal_publish) * max(veh.brk.brake_full);
        F_brake_actual = eval_force_map_1d(BRK_internal, veh.brk);
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
    lon_force.throttle_pct_raw = throttle_pct_raw;
    lon_force.brake_pct_raw = brake_pct_raw;
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

function force = eval_force_map_1d(cmd, map)
    cmd_q = min(max(cmd, map.cmd_min), map.cmd_max);

    try
        [cmd_axis, idx] = sort(map.cmd_full);
        force_axis = map.force_full(idx);
        [cmd_axis, iu] = unique(cmd_axis, 'stable');
        force_axis = force_axis(iu);

        if numel(cmd_axis) < 2
            force = NaN;
            return;
        end

        force = interp1(cmd_axis, force_axis, cmd_q, 'linear', 'extrap');
    catch
        force = NaN;
    end
end

function cmd = invert_force_map_1d(force_target, map)
    cmd_lo = map.cmd_min;
    cmd_hi = map.cmd_max;

    try
        [force_axis, idx] = sort(map.force_full);
        cmd_axis = map.cmd_full(idx);
        [force_axis, iu] = unique(force_axis, 'stable');
        cmd_axis = cmd_axis(iu);

        if numel(force_axis) < 2
            cmd = NaN;
            return;
        end

        cmd = interp1(force_axis, cmd_axis, force_target, 'linear', 'extrap');
        cmd = min(max(cmd, cmd_lo), cmd_hi);
    catch
        cmd = NaN;
    end
end
