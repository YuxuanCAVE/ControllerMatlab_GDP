function delta = mpc_kinematic_lateral(state, ref, veh, dt, p, idx_hint, window, steer_buffer)
% MPC_KINEMATIC_LATERAL  Lateral MPC based on a kinematic bicycle model.
%
%   delta = mpc_kinematic_lateral(state, ref, veh, dt, p, ...)
%
% State vector:
%   x = [e_y; e_psi]
%
% Control:
%   delta = delta_ff + delta_corr
%
% The QP optimises the steering correction delta_corr while a curvature
% feedforward term handles nominal path curvature.

    x = state.x;
    y = state.y;
    yaw = state.yaw;
    vx = max(get_vx(state), 0.5);

    if nargin < 6
        idx_hint = [];
    end
    if nargin < 7
        window = [];
    end
    if nargin < 8
        steer_buffer = [];
    end

    [idx, xr, yr, psi_ref, ~, seg_idx, seg_t] = nearest_path_ref_point( ...
        x, y, ref.x, ref.y, idx_hint, window);

    kappa0 = interpolate_projected_curvature(ref.kappa, idx, seg_idx, seg_t);

    dx = x - xr;
    dy = y - yr;
    e_y = -sin(psi_ref) * dx + cos(psi_ref) * dy;
    e_psi = angle_wrap(yaw - psi_ref);

    L = veh.L;
    nx = 2;
    nu = 1;
    N = p.N;

    % Small-angle kinematic bicycle error model:
    %   e_y_dot   = vx * e_psi
    %   e_psi_dot = vx / L * delta - vx * kappa
    x0 = [e_y; e_psi];

    A_c = [0, vx;
           0,  0];
    B_c = [0;
           vx / L];
    g_c_unit = [0;
               -vx];

    M_aug = zeros(nx + nu + 1, nx + nu + 1);
    M_aug(1:nx, 1:nx) = A_c;
    M_aug(1:nx, nx+1) = B_c;
    M_aug(1:nx, nx+2) = g_c_unit;

    E = expm(M_aug * dt);
    A = E(1:nx, 1:nx);
    B = E(1:nx, nx+1);
    g_unit = E(1:nx, nx+2);

    n_ref = numel(ref.kappa);
    i_lo = max(1, idx - 1);
    i_hi = min(n_ref, idx + 1);
    ds_local = hypot(ref.x(i_hi) - ref.x(i_lo), ref.y(i_hi) - ref.y(i_lo)) / max(i_hi - i_lo, 1);
    ds_local = max(ds_local, 0.01);
    idx_per_step = max(1, round(vx * dt / ds_local));

    n_delay = 0;
    if ~isempty(steer_buffer) && numel(steer_buffer) > 1
        n_delay = numel(steer_buffer) - 1;
    end

    kappa_horizon = zeros(n_delay + N, 1);
    delta_ff_horizon = zeros(n_delay + N, 1);
    for k = 1:(n_delay + N)
        future_idx = min(idx + k * idx_per_step, n_ref);
        kappa_horizon(k) = ref.kappa(future_idx);
        delta_ff_horizon(k) = p.kappa_ff_gain * atan(L * kappa_horizon(k));
    end

    delta_ff0 = p.kappa_ff_gain * atan(L * kappa0);
    delta_prev_corr = state.delta - delta_ff0;
    if n_delay > 0
        for d = 1:n_delay
            u_pending_corr = steer_buffer(d) - delta_ff_horizon(d);
            d_model = B * delta_ff_horizon(d) + g_unit * kappa_horizon(d);
            x0 = A * x0 + B * u_pending_corr + d_model;
        end
        delta_prev_corr = steer_buffer(n_delay) - delta_ff_horizon(n_delay);
    end

    kappa_pred = kappa_horizon(n_delay+1:n_delay+N);
    delta_ff_pred = delta_ff_horizon(n_delay+1:n_delay+N);

    Qbar = kron(eye(N), p.Q);
    Rbar = kron(eye(N), p.R);

    Sx = zeros(nx * N, nx);
    Su = zeros(nx * N, nu * N);
    Sd = zeros(nx * N, 1);
    A_power = eye(nx);

    for i = 1:N
        A_power = A_power * A;
        row = (i-1)*nx+1:i*nx;
        Sx(row, :) = A_power;

        d_sum = zeros(nx, 1);
        for j = 1:i
            A_ij = A^(i-j);
            Su(row, (j-1)*nu+1:j*nu) = A_ij * B;
            d_j = B * delta_ff_pred(j) + g_unit * kappa_pred(j);
            d_sum = d_sum + A_ij * d_j;
        end
        Sd(row, :) = d_sum;
    end

    x_free = Sx * x0 + Sd;

    D = eye(N);
    D = D - [zeros(1, N); eye(N-1, N)];
    d0 = zeros(N, 1);
    d0(1) = delta_prev_corr;

    % Cost function: J = X'QX + U'RU + Rd * dU'dU

    H = Su' * Qbar * Su + Rbar + p.Rd * (D' * D);
    H = 0.5 * (H + H');
    
    f = Su' * Qbar * x_free - p.Rd * (D' * d0);

    lb = -p.max_steer * ones(N, 1) - delta_ff_pred;
    ub =  p.max_steer * ones(N, 1) - delta_ff_pred;

    try
        opts = optimoptions('quadprog', 'Display', 'off');
        U = quadprog(2 * H, 2 * f, [], [], [], [], lb, ub, [], opts);
        if isempty(U)
            delta_corr = 0;
        else
            delta_corr = U(1);
        end
    catch
        delta_corr = -p.fallback_k_e_y * e_y - p.fallback_k_e_psi * e_psi;
    end

    delta = delta_corr + delta_ff0;
end

function kappa0 = interpolate_projected_curvature(kappa_ref, idx, seg_idx, seg_t)
    n = numel(kappa_ref);
    if ~isnan(seg_idx) && seg_idx >= 1 && seg_idx < n
        kappa0 = (1 - seg_t) * kappa_ref(seg_idx) + seg_t * kappa_ref(seg_idx + 1);
    else
        kappa0 = kappa_ref(idx);
    end
end

function vx = get_vx(state)
    if isfield(state, 'vx')
        vx = state.vx;
    else
        vx = state.v;
    end
end
