function [a_des, lon] = PID_controller(v_ref, v, lon, dt)
    ev = v_ref - v;
    lon.i_term = lon.i_term + ev * dt;
    d_ev = (ev - lon.prev_ev) / max(dt, 1e-6);
    lon.prev_ev = ev;
    a_des = lon.kp * ev + lon.ki * lon.i_term + lon.kd * d_ev;
    a_des = min(max(a_des, lon.a_min), lon.a_max);
end
