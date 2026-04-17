function delta = mpc_kinematic_lateral(state, ref, veh, dt, p, idx_hint, window, steer_buffer)
% MPC_KINEMATIC_LATERAL
% Lateral MPC based on a linearized kinematic bicycle model
% with path-frame error states:
%   x = [e_y; e_psi]
%
% e_y   : cross-track error in path frame
% e_psi : heading error relative to reference path
%
% Control:
%   delta = delta_ff + delta_corr

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

    [idx, ~, ~, psi_ref, e_y, seg_idx, seg_t] = nearest_path_ref_point( ...
        x, y, ref.x, ref.y, idx_hint, window);

    kappa0 = interpolate_projected_curvature(ref.kappa, idx, seg_idx, seg_t);

    % ------------------------------------------------------------------
    % Path-frame errors
    % ------------------------------------------------------------------
    e_psi = angle_wrap(yaw - psi_ref);

    x0 = [e_y; e_psi];

    L = veh.L;
    nx = 2;
    nu = 1;
    N = p.N;

    % ------------------------------------------------------------------
    % Continuous-time linearized path-frame error model
    %
    % e_y_dot   = vx * e_psi
    % e_psi_dot = vx/L * delta - vx * kappa
    %
    % delta = delta_ff + delta_corr
    % ------------------------------------------------------------------
    
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

    % ------------------------------------------------------------------
    % Reference preview spacing
    % ------------------------------------------------------------------
    n_ref = numel(ref.kappa);
    i_lo = max(1, idx - 1);
    i_hi = min(n_ref, idx + 1);
    ds_local = hypot(ref.x(i_hi) - ref.x(i_lo), ref.y(i_hi) - ref.y(i_lo)) / max(i_hi - i_lo, 1);
    ds_local = max(ds_local, 0.01);
    idx_per_step = max(1, round(vx * dt / ds_local));

    % ------------------------------------------------------------------
    % Steering delay buffer length
    % If steer_buffer contains pending commands in time order,
    % numel(buffer)-1 is treated as delayed pending horizon.
    % ------------------------------------------------------------------
    n_delay = 0;
    if ~isempty(steer_buffer) && numel(steer_buffer) > 1
        n_delay = numel(steer_buffer) - 1;
    end

    % ------------------------------------------------------------------
    % Build curvature / feedforward preview
    % ------------------------------------------------------------------
    kappa_horizon = zeros(n_delay + N, 1);
    delta_ff_horizon = zeros(n_delay + N, 1);

    for k = 1:(n_delay + N)
        future_idx = min(idx + k * idx_per_step, n_ref);
        kappa_horizon(k) = ref.kappa(future_idx);
        delta_ff_horizon(k) = p.kappa_ff_gain * atan(L * kappa_horizon(k));
    end

    delta_ff0 = p.kappa_ff_gain * atan(L * kappa0);
    delta_prev_corr = state.delta - delta_ff0;

    % ------------------------------------------------------------------
    % Propagate delayed commands already in steering pipeline
    % ------------------------------------------------------------------
    if n_delay > 0
        for d = 1:n_delay
            u_pending_corr = steer_buffer(d) - delta_ff_horizon(d);
            d_model = B * delta_ff_horizon(d) + g_unit * kappa_horizon(d);
            x0 = A * x0 + B * u_pending_corr + d_model;
        end
        delta_prev_corr = steer_buffer(n_delay) - delta_ff_horizon(n_delay);
    end

    % Prediction horizon references
    kappa_pred = kappa_horizon(n_delay+1:n_delay+N);
    delta_ff_pred = delta_ff_horizon(n_delay+1:n_delay+N);

    % ------------------------------------------------------------------
    % Cost matrices
    % p.Q should be 2x2, e.g. diag([q_ey, q_epsi])
    % p.R should be scalar
    % p.Rd should be scalar
    % ------------------------------------------------------------------
    Qbar = kron(eye(N), p.Q);
    Rbar = kron(eye(N), p.R);

    Sx = zeros(nx * N, nx);
    Su = zeros(nx * N, nu * N);
    Sd = zeros(nx * N, 1);

    % ------------------------------------------------------------------
    % Build lifted prediction matrices
    % x_{k+i} = Sx*x0 + Su*U + Sd
    % ------------------------------------------------------------------
    for i = 1:N
        row = (i-1)*nx+1:i*nx;

        A_power = eye(nx);
        for m = 1:i
            A_power = A * A_power;
        end
        Sx(row, :) = A_power;

        d_sum = zeros(nx, 1);

        for j = 1:i
            A_ij = eye(nx);
            for m = j+1:i
                A_ij = A * A_ij;
            end

            Su(row, (j-1)*nu+1:j*nu) = A_ij * B;

            d_j = B * delta_ff_pred(j) + g_unit * kappa_pred(j);
            d_sum = d_sum + A_ij * d_j;
        end

        Sd(row, :) = d_sum;
    end

    x_free = Sx * x0 + Sd;

    % ------------------------------------------------------------------
    % Steering increment penalty
    % D * U approximates dU, with first element referenced to previous input
    % ------------------------------------------------------------------
    D = eye(N);
    D = D - [zeros(1, N); eye(N-1, N)];

    d0 = zeros(N, 1);
    d0(1) = delta_prev_corr;

    % Cost:
    % J = X'QX + U'RU + Rd*(DU)'*(DU)
    H = Su' * Qbar * Su + Rbar + p.Rd * (D' * D);
    H = 0.5 * (H + H');

    f = Su' * Qbar * x_free - p.Rd * (D' * d0);

    % Steering bounds on total steering angle:
    % delta = delta_corr + delta_ff
    lb = -p.max_steer * ones(N, 1) - delta_ff_pred;
    ub =  p.max_steer * ones(N, 1) - delta_ff_pred;

    try
        opts = optimoptions('quadprog', 'Display', 'off');
        U = quadprog(2 * H, 2 * f, [], [], [], [], lb, ub, [], opts);

        if isempty(U)
            delta_corr = 0.0;
        else
            delta_corr = U(1);
        end
    catch
        % Simple linear fallback in path frame
        delta_corr = -p.fallback_k_e_y * e_y ...
                     -p.fallback_k_e_psi * e_psi;
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
