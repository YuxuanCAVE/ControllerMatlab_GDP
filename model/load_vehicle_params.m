function veh = load_vehicle_params(accel_map_file, brake_map_file)
    % Vehicle parameters retained by the current project:
    %   - simple longitudinal resistance model
    %   - vehicle mass
    %   - wheelbase / axle geometry for kinematic bicycle relations
    %   - 1D actuator lookup tables for throttle and brake

    veh.A = 45;
    veh.B = 10;
    veh.C = 0.518;
    veh.M = 1948;
    veh.L = 2.720;

    % Front / rear axle distances are kept for the standard KBM beta(delta)
    % relation even though the current plant only uses veh.L directly.
    veh.lf = 1.2140;
    veh.lr = veh.L - veh.lf;

    % ── Load actuator maps ────────────────────────────────────────────
    Sacc = load(accel_map_file);
    acc_cmd_raw = make_col(get_first_existing_field(Sacc, {'Acc_Full', 'Acc_full'}));
    acc_force_raw = make_col(get_first_existing_field(Sacc, {'Force_full', 'Force_Full'}));
    acc_vel_raw = make_col(get_first_existing_field(Sacc, {'Vel_Full', 'Vel_full'}));

    veh.acc.cmd_samples = acc_cmd_raw;
    veh.acc.force_samples = acc_force_raw;
    veh.acc.vel_samples = acc_vel_raw;

    acc_cmd = acc_cmd_raw;
    acc_force = acc_force_raw;
    [acc_cmd, ia] = unique(acc_cmd, 'stable');
    acc_force = acc_force(ia);
    [acc_force_sorted, i1] = sort(acc_force);
    acc_cmd_sorted = acc_cmd(i1);

    veh.acc.acc_full = acc_cmd_sorted;
    veh.acc.cmd_full = acc_cmd_sorted;
    veh.acc.force_full = acc_force_sorted;
    veh.acc.cmd_max = max(acc_cmd_sorted);

    Sbrk = load(brake_map_file);
    brk_cmd_raw = make_col(get_first_existing_field(Sbrk, {'Break_Full', 'Brake_Full', 'Brake_full'}));
    brk_force_raw = make_col(get_first_existing_field(Sbrk, {'Force_full', 'Force_Full'}));
    brk_vel_raw = make_col(get_first_existing_field(Sbrk, {'Vel_Full', 'Vel_full'}));

    brk_cmd_mag = abs(brk_cmd_raw);
    brk_force_mag = abs(brk_force_raw);
    brk_vel_mag = abs(brk_vel_raw);

    veh.brk.cmd_samples = brk_cmd_mag;
    veh.brk.force_samples = brk_force_mag;
    veh.brk.vel_samples = brk_vel_mag;

    [brk_cmd_mag, ib] = unique(brk_cmd_mag, 'stable');
    brk_force_mag = brk_force_mag(ib);
    [brk_cmd_sorted, i2] = sort(brk_cmd_mag);
    brk_force_sorted = brk_force_mag(i2);

    veh.brk.brake_full = brk_cmd_sorted;
    veh.brk.cmd_full = brk_cmd_sorted;
    veh.brk.force_full = brk_force_sorted;
    veh.brk.cmd_max = max(brk_cmd_sorted);

    veh.max_pedal_publish = 0.60;

    % Keep the raw map threshold for diagnostics, but use a zero-anchored
    % lookup so small commands/forces are interpolated linearly from zero.
    acc_cmd_min = min(acc_cmd_raw);
    brk_cmd_min = min(brk_cmd_mag);

    veh.acc.pedal_min_publish_from_map = ...
        (acc_cmd_min / max(veh.acc.acc_full)) * veh.max_pedal_publish;
    veh.brk.pedal_min_publish_from_map = ...
        (brk_cmd_min / max(veh.brk.brake_full)) * veh.max_pedal_publish;

    veh.acc.cmd_min_from_map = ...
        (veh.acc.pedal_min_publish_from_map / veh.max_pedal_publish) * max(veh.acc.acc_full);
    veh.brk.cmd_min_from_map = ...
        (veh.brk.pedal_min_publish_from_map / veh.max_pedal_publish) * max(veh.brk.brake_full);

    veh.acc.pedal_min_effective = 0;
    veh.brk.pedal_min_effective = 0;
    veh.acc.cmd_min_effective = 0;
    veh.brk.cmd_min_effective = 0;

    veh.acc.force_min_from_map = interp1( ...
        veh.acc.acc_full, veh.acc.force_full, veh.acc.cmd_min_from_map, ...
        'linear', 'extrap');
    veh.brk.force_min_from_map = interp1( ...
        veh.brk.brake_full, veh.brk.force_full, veh.brk.cmd_min_from_map, ...
        'linear', 'extrap');

    [veh.acc.cmd_lookup, veh.acc.force_lookup] = add_zero_anchor( ...
        veh.acc.cmd_full, veh.acc.force_full);
    [veh.brk.cmd_lookup, veh.brk.force_lookup] = add_zero_anchor( ...
        veh.brk.cmd_full, veh.brk.force_full);

    veh.acc.cmd_max = max(veh.acc.cmd_lookup);
    veh.brk.cmd_max = max(veh.brk.cmd_lookup);

    veh.acc.force_min_effective = 0;
    veh.brk.force_min_effective = 0;

    veh.acc.force_exit_coast = 0;
    veh.brk.force_exit_coast = 0;

    % ── Compute acceleration limits from actuator maps ────────────────
    % These are the ACTUAL physical limits of the vehicle at 60% pedal cap.
    %
    % For throttle:
    %   Max pedal at 60% → max internal command → max tractive force
    %   a_max = (F_trac_max - F_resist) / M
    %   Use F_resist at v=0 for the most optimistic (highest) a_max
    %
    % For braking:
    %   Max brake at 60% → max brake force
    %   a_min = -(F_brake_max + F_resist) / M
    %   Use F_resist at v=0 for the most conservative (least negative) a_min
    %
    % The longitudinal_model.m handles the detailed force balance at each
    % timestep; these limits are for the controller to know its bounds.

    % Max tractive force at 60% pedal:
    % the publish cap maps back to the maximum internal command.
    F_trac_max = max(veh.acc.force_full);

    % Max brake force at 60% pedal:
    % the publish cap maps back to the maximum internal command.
    F_brake_max = max(veh.brk.force_full);

    % Resistance at standstill (v=0)
    F_resist_v0 = veh.A;  % = 45 N (B*0 + C*0 = 0)

    % Compute acceleration limits
    % a_max: net forward acceleration = (F_trac - F_resist) / M
    veh.a_max_from_map = (F_trac_max - F_resist_v0) / veh.M;

    % a_min: net braking deceleration = -(F_brake + F_resist) / M
    % At v=0, F_resist is small so this gives the least aggressive braking
    % At higher speeds, braking is actually stronger (resistance helps)
    veh.a_min_from_map = -(F_brake_max + F_resist_v0) / veh.M;

    % Store the raw force values for reference
    veh.F_trac_max = F_trac_max;
    veh.F_brake_max = F_brake_max;

    % Print for verification
    fprintf('  Actuator limits from maps (at 60%% pedal):\n');
    fprintf('    Max tractive force:  %.1f N\n', F_trac_max);
    fprintf('    Max braking force:   %.1f N\n', F_brake_max);
    fprintf('    a_max (from map):    %.3f m/s^2\n', veh.a_max_from_map);
    fprintf('    a_min (from map):    %.3f m/s^2\n', veh.a_min_from_map);
    fprintf('    Min accel publish:   %.4f (map), %.4f (effective)\n', ...
        veh.acc.pedal_min_publish_from_map, veh.acc.pedal_min_effective);
    fprintf('    Min brake publish:   %.4f (map), %.4f (effective)\n', ...
        veh.brk.pedal_min_publish_from_map, veh.brk.pedal_min_effective);
end

function out = get_first_existing_field(S, names)
    out = [];
    for i = 1:numel(names)
        if isfield(S, names{i})
            out = S.(names{i});
            return;
        end
    end
    error('Required field not found in MAT file.');
end

function x = make_col(x)
    x = x(:);
end

function [cmd_axis_out, force_axis_out] = add_zero_anchor(cmd_axis_in, force_axis_in)
    cmd_axis = cmd_axis_in(:);
    force_axis = force_axis_in(:);

    [cmd_axis, idx] = sort(cmd_axis);
    force_axis = force_axis(idx);
    [cmd_axis, iu] = unique(cmd_axis, 'stable');
    force_axis = force_axis(iu);

    if isempty(cmd_axis)
        cmd_axis_out = 0;
        force_axis_out = 0;
        return;
    end

    if cmd_axis(1) > 0
        cmd_axis_out = [0; cmd_axis];
        force_axis_out = [0; force_axis];
    else
        cmd_axis_out = cmd_axis;
        force_axis_out = force_axis;
        cmd_axis_out(1) = 0;
        force_axis_out(1) = 0;
    end
end
