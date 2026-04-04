function veh = load_vehicle_params(accel_map_file, brake_map_file)
    veh.A = 45;
    veh.B = 10;
    veh.C = 0.518;
    veh.M = 1948;
    veh.L = 2.720;

    veh.lf = 1.2140;
    veh.lr = veh.L - veh.lf;
    veh.Iz = 2500;

    % Simplified Magic Formula tire parameters.
    % Fy = D * sin(C * atan(B * alpha))
    veh.tire.front.B = 9.5;
    veh.tire.front.C = 1.30;
    veh.tire.front.D = 8000;

    veh.tire.rear.B = 10.5;
    veh.tire.rear.C = 1.30;
    veh.tire.rear.D = 8500;

    % Linearized cornering stiffness around small slip angles.
    veh.tire.front.Calpha = veh.tire.front.B * veh.tire.front.C * veh.tire.front.D;
    veh.tire.rear.Calpha = veh.tire.rear.B * veh.tire.rear.C * veh.tire.rear.D;

    %all the data we're using for magic formula are the assuptions

    Sacc = load(accel_map_file);
    acc_cmd = make_col(get_first_existing_field(Sacc, {'Acc_Full', 'Acc_full'}));
    acc_force = make_col(get_first_existing_field(Sacc, {'Force_full', 'Force_Full'}));

    [acc_cmd, ia] = unique(acc_cmd, 'stable');
    acc_force = acc_force(ia);
    [acc_force_sorted, i1] = sort(acc_force);
    acc_cmd_sorted = acc_cmd(i1);

    veh.acc.acc_full = acc_cmd_sorted;
    veh.acc.force_full = acc_force_sorted;

    Sbrk = load(brake_map_file);
    brk_cmd_raw = make_col(get_first_existing_field(Sbrk, {'Break_Full', 'Brake_Full', 'Brake_full'}));
    brk_force_raw = make_col(get_first_existing_field(Sbrk, {'Force_full', 'Force_Full'}));

    brk_cmd_mag = abs(brk_cmd_raw);
    brk_force_mag = abs(brk_force_raw);

    [brk_cmd_mag, ib] = unique(brk_cmd_mag, 'stable');
    brk_force_mag = brk_force_mag(ib);
    [brk_cmd_sorted, i2] = sort(brk_cmd_mag);
    brk_force_sorted = brk_force_mag(i2);

    veh.brk.brake_full = brk_cmd_sorted;
    veh.brk.force_full = brk_force_sorted;

    veh.max_pedal_publish = 0.60;
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
