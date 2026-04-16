function [a_des, lon] = PID_controller(v_ref, v, lon, dt)
    % PID outputs net acceleration correction.
    % The force-balance actuator model adds resistance back via:
    %   F_required = M * a_des + F_resist(v)

    lon = ensure_pid_state(lon);

    ev = v_ref - v;
    dt_safe = max(dt, 1e-6);
    lon.i_term = lon.i_term + ev * dt;
    d_ev = (ev - lon.prev_ev) / dt_safe;
    lon.prev_ev = ev;

    a_des = lon.kp * ev + lon.ki * lon.i_term + lon.kd * d_ev;
    a_des = min(max(a_des, lon.a_min), lon.a_max);
end

function lon = ensure_pid_state(lon)
    if ~isfield(lon, 'i_term') || isempty(lon.i_term)
        lon.i_term = 0.0;
    end
    if ~isfield(lon, 'prev_ev') || isempty(lon.prev_ev)
        lon.prev_ev = 0.0;
    end
end
