function delta_cmd = mpc_kbm_lateral(state, ref, veh, dt, p, idx_hint, window, steer_buffer)
% MPC_KINEMATIC_LATERAL
% Linear MPC based on a time-varying linearisation of the full nonlinear
% kinematic bicycle model (KBM).
%
% Steering command convention:
%   left turn  -> negative delta command
%   right turn -> positive delta command
%
% State:
%   x_k = [X_k; Y_k; psi_k; delta_k]
%
% Input:
%   u_k = delta_dot_k
%
% Continuous-time nonlinear KBM:
%   X_dot     = v * cos(psi + beta(delta))
%   Y_dot     = v * sin(psi + beta(delta))
%   psi_dot   = v / l_r * sin(beta(delta))
%   delta_dot = u
%
%   beta(delta_cmd) = -atan((l_r / L) * tan(delta_cmd))
%
% This controller keeps the MPC formulation linear by:
%   1) building a nominal trajectory with the full nonlinear KBM
%   2) linearising the full KBM along that nominal trajectory
%   3) solving one QP on the resulting affine LTV prediction model
%
% Tracking errors are defined in the same global form as the NMPC:
%   e_X   = X - X_r
%   e_Y   = Y - Y_r
%   e_psi = wrapToPi(psi - psi_r)

    if nargin < 6
        idx_hint = [];
    end
    if nargin < 7
        window = [];
    end
    if nargin < 8
        steer_buffer = [];
    end

    Ts = get_sample_time(p, dt);
    v_measured = max(get_speed(state), 0.0);
    x_meas = [state.x; state.y; state.yaw; state.delta];

    x0 = propagate_delay_state(x_meas, steer_buffer, v_measured, veh, Ts);
    preview = build_reference_preview_from_state(x0, v_measured, ref, Ts, p, idx_hint, window);
    nominal = build_nominal_trajectory(x0, preview.v, veh, Ts);
    model = linearise_nominal_trajectory(nominal, preview.v, veh, Ts);
    qp = build_linear_mpc_qp(x0, nominal, model, preview, p, veh);

    try
        opts = optimoptions('quadprog', 'Display', 'off');
        U = quadprog(2 * qp.H, 2 * qp.f, qp.Aineq, qp.bineq, [], [], qp.lb, qp.ub, [], opts);

        if isempty(U)
            U = zeros(p.N, 1);
        end
    catch
        U = zeros(p.N, 1);
    end

    u0 = U(1);
    delta_cmd = x0(4) + Ts * u0;
    delta_cmd = min(max(delta_cmd, p.delta_min), p.delta_max);
end

function Ts = get_sample_time(p, dt_default)
    if isfield(p, 'Ts') && ~isempty(p.Ts)
        Ts = p.Ts;
    else
        Ts = dt_default;
    end
end

function x0 = propagate_delay_state(x_meas, steer_buffer, v_measured, veh, Ts)
% Propagate the measured state through already-buffered steering commands.
% This makes the optimisation start from the state at which a newly issued
% steering command will first reach the plant.

    x0 = x_meas;

    if isempty(steer_buffer) || numel(steer_buffer) <= 1
        return;
    end

    n_delay = numel(steer_buffer) - 1;
    v_delay = v_measured * ones(n_delay, 1);

    for k = 1:n_delay
        x0 = nonlinear_kbm_step(x0, steer_buffer(k), v_delay(k), veh, Ts);
    end
end

function preview = build_reference_preview_from_state(x0, v_measured, ref, Ts, p, idx_hint, window)
% Build the global reference preview used by the MPC cost:
%   r_k = [X_r,k; Y_r,k; psi_r,k; 0]
%
% The known speed sequence uses the measured speed at every step, matching
% the current NMPC convention in this project.

    idx0 = nearest_path_ref_point( ...
        x0(1), x0(2), ref.x, ref.y, idx_hint, window);

    N = p.N;
    n_ref = numel(ref.x);
    i_lo = max(1, idx0 - 1);
    i_hi = min(n_ref, idx0 + 1);
    ds_local = hypot(ref.x(i_hi) - ref.x(i_lo), ref.y(i_hi) - ref.y(i_lo)) / max(i_hi - i_lo, 1);
    ds_local = max(ds_local, 0.01);

    idx_cursor = idx0;
    Xr = zeros(N, 1);
    Yr = zeros(N, 1);
    psir = zeros(N, 1);
    v_known = max(v_measured, 0.0) * ones(N, 1);

    for k = 1:N
        if k > 1
            idx_advance = max(1, round(v_known(k) * Ts / ds_local));
            idx_cursor = min(idx_cursor + idx_advance, n_ref);
        end

        Xr(k) = ref.x(idx_cursor);
        Yr(k) = ref.y(idx_cursor);
        psir(k) = get_ref_heading(ref, idx_cursor);
    end

    psir = unwrap_reference_yaw(psir, x0(3));

    preview.Xr = Xr;
    preview.Yr = Yr;
    preview.psir = psir;
    preview.v = v_known;
end

function psi_ref = get_ref_heading(ref, idx)
    n_ref = numel(ref.x);
    if isfield(ref, 'yaw') && ~isempty(ref.yaw)
        psi_ref = ref.yaw(idx);
    elseif idx <= 1
        psi_ref = atan2(ref.y(2) - ref.y(1), ref.x(2) - ref.x(1));
    elseif idx >= n_ref
        psi_ref = atan2(ref.y(n_ref) - ref.y(n_ref - 1), ref.x(n_ref) - ref.x(n_ref - 1));
    else
        psi_ref = atan2(ref.y(idx + 1) - ref.y(idx - 1), ref.x(idx + 1) - ref.x(idx - 1));
    end
end

function psi_seq = unwrap_reference_yaw(psi_seq, psi_anchor)
% Keep the reference heading sequence locally continuous so that the linear
% MPC can penalise psi - psi_r without a discontinuity at +/-pi.

    if isempty(psi_seq)
        return;
    end

    psi_seq(1) = psi_anchor + angle_wrap(psi_seq(1) - psi_anchor);
    for k = 2:numel(psi_seq)
        psi_seq(k) = psi_seq(k-1) + angle_wrap(psi_seq(k) - psi_seq(k-1));
    end
end

function nominal = build_nominal_trajectory(x0, v_known, veh, Ts)
% Build a nominal trajectory with the full nonlinear KBM using zero
% steering-rate input:
%   u_bar,k = 0
%
% This keeps the model "full KBM", while the MPC itself stays linear
% because we later linearise around this nominal rollout.

    N = numel(v_known);
    x_bar = zeros(4, N + 1);
    u_bar = zeros(N, 1);
    x_bar(:, 1) = x0;

    for k = 1:N
        delta_next = x_bar(4, k) + Ts * u_bar(k);
        x_bar(:, k+1) = nonlinear_kbm_step(x_bar(:, k), delta_next, v_known(k), veh, Ts);
    end

    nominal.x = x_bar;
    nominal.u = u_bar;
end

function model = linearise_nominal_trajectory(nominal, v_known, veh, Ts)
% Linearise the full nonlinear KBM around the nominal trajectory:
%
%   x_{k+1} = A_k x_k + B_k u_k + c_k

    N = numel(v_known);
    nx = 4;
    nu = 1;

    A = zeros(nx, nx, N);
    B = zeros(nx, nu, N);
    c = zeros(nx, N);

    lr = veh.lr;
    L = veh.L;
    alpha = lr / max(L, 1e-6);

    for k = 1:N
        xk = nominal.x(:, k);
        uk = nominal.u(k);
        vk = v_known(k);

        psi_k = xk(3);
        delta_k = xk(4);

        beta_k = kbm_beta(delta_k, lr, L);
        dbeta_ddelta = kbm_beta_derivative(delta_k, alpha);
        theta_k = psi_k + beta_k;

        f_k = [
            vk * cos(theta_k);
            vk * sin(theta_k);
            vk / max(lr, 1e-6) * sin(beta_k);
            uk
        ];

        A_c = zeros(nx, nx);
        A_c(1, 3) = -vk * sin(theta_k);
        A_c(1, 4) = -vk * sin(theta_k) * dbeta_ddelta;
        A_c(2, 3) =  vk * cos(theta_k);
        A_c(2, 4) =  vk * cos(theta_k) * dbeta_ddelta;
        A_c(3, 4) =  vk / max(lr, 1e-6) * cos(beta_k) * dbeta_ddelta;

        B_c = [0; 0; 0; 1];

        c_c = f_k - A_c * xk - B_c * uk;

        A(:, :, k) = eye(nx) + Ts * A_c;
        B(:, :, k) = Ts * B_c;
        c(:, k) = Ts * c_c;
    end

    model.A = A;
    model.B = B;
    model.c = c;
end

function qp = build_linear_mpc_qp(x0, nominal, model, preview, p, veh)
% Build the condensed QP for:
%   min sum (x_k - r_k)'Q(x_k - r_k) + u_k'R u_k [+ optional du penalty]
% subject to:
%   x_{k+1} = A_k x_k + B_k u_k + c_k
%   delta_min <= delta_k <= delta_max
%   u_min <= u_k <= u_max
%   optional linearised lateral acceleration constraints

    N = p.N;
    nx = 4;
    nu = 1;

    [Sx, Su, Sc] = build_lifted_prediction(model);
    x_free = Sx * x0 + Sc;

    Q_stage = diag([p.q_X, p.q_Y, p.q_psi, p.r_delta]);
    Qbar = kron(eye(N), Q_stage);
    Rbar = kron(eye(N), p.r_u);

    if isfield(p, 'r_du') && p.r_du > 0
        D = eye(N);
        D = D - [zeros(1, N); eye(N-1, N)];
        H_du = p.r_du * (D' * D);
        f_du = zeros(N, 1);
    else
        H_du = zeros(N);
        f_du = zeros(N, 1);
    end

    r_stack = zeros(nx * N, 1);
    for k = 1:N
        row = (k - 1) * nx + (1:nx);
        r_stack(row) = [preview.Xr(k); preview.Yr(k); preview.psir(k); 0.0];
    end

    H = Su' * Qbar * Su + Rbar + H_du;
    H = 0.5 * (H + H');
    f = Su' * Qbar * (x_free - r_stack) + f_du;

    [Aineq, bineq] = build_state_constraints(Sx, Su, x0, Sc, nominal, preview, p, veh);

    qp.H = H;
    qp.f = f;
    qp.Aineq = Aineq;
    qp.bineq = bineq;
    qp.lb = p.u_min * ones(N, 1);
    qp.ub = p.u_max * ones(N, 1);
end

function [Sx, Su, Sc] = build_lifted_prediction(model)
% Build the affine lifted prediction model:
%   X = Sx * x0 + Su * U + Sc
%
% where X stacks the predicted states x_1,...,x_N.

    [nx, ~, N] = size(model.A);
    nu = size(model.B, 2);

    Sx = zeros(nx * N, nx);
    Su = zeros(nx * N, nu * N);
    Sc = zeros(nx * N, 1);

    for i = 1:N
        row_i = (i - 1) * nx + (1:nx);

        Phi = eye(nx);
        for m = 1:i
            Phi = model.A(:, :, m) * Phi;
        end
        Sx(row_i, :) = Phi;

        c_sum = zeros(nx, 1);
        for j = 1:i
            Phi_ij = eye(nx);
            for m = (j + 1):i
                Phi_ij = model.A(:, :, m) * Phi_ij;
            end

            col_j = (j - 1) * nu + (1:nu);
            Su(row_i, col_j) = Phi_ij * model.B(:, :, j);
            c_sum = c_sum + Phi_ij * model.c(:, j);
        end

        Sc(row_i) = c_sum;
    end
end

function [Aineq, bineq] = build_state_constraints(Sx, Su, x0, Sc, nominal, preview, p, veh)
% Build linear inequality constraints on:
%   - predicted steering angle delta_k
%   - optional linearised lateral acceleration

    N = p.N;
    nx = 4;

    delta_rows = 4:nx:(nx * N);
    Su_delta = Su(delta_rows, :);
    xfree_delta = Sx(delta_rows, :) * x0 + Sc(delta_rows);

    Aineq = [
        Su_delta;
        -Su_delta
    ];
    bineq = [
        p.delta_max * ones(N, 1) - xfree_delta;
        -p.delta_min * ones(N, 1) + xfree_delta
    ];

    if isfield(p, 'use_ay_constraint') && p.use_ay_constraint
        L = veh.L;

        A_ay = zeros(N, N);
        b_ay_pos = zeros(N, 1);
        b_ay_neg = zeros(N, 1);

        for k = 1:N
            row = 4 * (k - 1) + 4;
            delta_bar = nominal.x(4, k);
            v_bar = preview.v(k);

            ay_bar = -v_bar^2 / max(L, 1e-6) * tan(delta_bar);
            day_ddelta = -v_bar^2 / max(L, 1e-6) * sec(delta_bar)^2;

            A_ay(k, :) = day_ddelta * Su(row, :);

            xfree_delta_k = Sx(row, :) * x0 + Sc(row);
            ay_free_k = ay_bar + day_ddelta * (xfree_delta_k - delta_bar);

            b_ay_pos(k) = p.ay_max - ay_free_k;
            b_ay_neg(k) = p.ay_max + ay_free_k;
        end

        Aineq = [
            Aineq;
            A_ay;
            -A_ay
        ];
        bineq = [
            bineq;
            b_ay_pos;
            b_ay_neg
        ];
    end
end

function x_next = nonlinear_kbm_step(xk, delta_cmd, v_k, veh, Ts)
% One-step forward-Euler rollout of the full nonlinear KBM using the
% steering angle command delta_cmd directly.

    X_k = xk(1);
    Y_k = xk(2);
    psi_k = xk(3);

    beta_k = kbm_beta(delta_cmd, veh.lr, veh.L);

    x_next = zeros(4, 1);
    x_next(1) = X_k + Ts * v_k * cos(psi_k + beta_k);
    x_next(2) = Y_k + Ts * v_k * sin(psi_k + beta_k);
    x_next(3) = angle_wrap(psi_k + Ts * (v_k / max(veh.lr, 1e-6)) * sin(beta_k));
    x_next(4) = delta_cmd;
end

function beta = kbm_beta(delta_cmd, lr, L)
    beta = -atan((lr / max(L, 1e-6)) * tan(delta_cmd));
end

function dbeta = kbm_beta_derivative(delta_cmd, alpha)
    tan_delta = tan(delta_cmd);
    sec_delta_sq = sec(delta_cmd)^2;
    dbeta = -alpha * sec_delta_sq / (1 + (alpha * tan_delta)^2);
end

function v = get_speed(state)
    if isfield(state, 'v')
        v = state.v;
    elseif isfield(state, 'vx')
        v = state.vx;
    else
        v = 0.0;
    end
end
