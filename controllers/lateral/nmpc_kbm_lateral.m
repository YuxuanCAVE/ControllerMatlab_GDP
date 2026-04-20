function [delta_cmd, nmpc] = nmpc_kbm_lateral(state, ref, veh, dt, p, idx_hint, window)
% NMPC_KBM_LATERAL  Lateral NMPC using a nonlinear kinematic bicycle model.
%
% Steering command convention:
%   left turn  -> negative delta command
%   right turn -> positive delta command
%
% State:
%   x_k = [X_k; Y_k; psi_k]
%
% Input:
%   u_k = delta_k
%
% Prediction model:
%   X_{k+1}   = X_k + Ts * v_k * cos(psi_k + beta(delta_k))
%   Y_{k+1}   = Y_k + Ts * v_k * sin(psi_k + beta(delta_k))
%   psi_{k+1} = psi_k + Ts * v_k / l_r * sin(beta(delta_k))
%
%   beta(delta_cmd) = -atan((l_r / L) * tan(delta_cmd))
%
% Cost:
%   sum_{k=0}^{N-1} q_X e_X^2 + q_Y e_Y^2 + q_psi e_psi^2
%                   + r_delta delta_k^2 + r_du (delta_k - delta_{k-1})^2
%
% Constraints:
%   delta_min <= delta_k <= delta_max
%   |delta_k - delta_{k-1}| <= delta_rate_max * Ts
%   optional: |a_y| <= a_y_max with a_y = v^2 / L * tan(delta)

    if nargin < 6
        idx_hint = [];
    end
    if nargin < 7
        window = [];
    end

    Ts = get_sample_time(p, dt);
    x0 = [state.x; state.y; state.yaw];

    [x_pred0, preview] = build_reference_preview(state, ref, veh, Ts, p, idx_hint, window);
    problem = build_nmpc_problem(x0, x_pred0, preview, veh, p, Ts, state.delta);
    nmpc = solve_nmpc_problem(problem, p);
    delta_cmd = nmpc.delta0;
end

function Ts = get_sample_time(p, dt_default)
    if isfield(p, 'Ts') && ~isempty(p.Ts)
        Ts = p.Ts;
    else
        Ts = dt_default;
    end
end

function [x_pred0, preview] = build_reference_preview(state, ref, veh, Ts, p, idx_hint, window)
% Build the reference preview used by the NMPC problem.

    idx = nearest_path_ref_point( ...
        state.x, state.y, ref.x, ref.y, idx_hint, window);

    N = p.N;
    n_ref = numel(ref.x);
    i_lo = max(1, idx - 1);
    i_hi = min(n_ref, idx + 1);
    ds_local = hypot(ref.x(i_hi) - ref.x(i_lo), ref.y(i_hi) - ref.y(i_lo)) / max(i_hi - i_lo, 1);
    ds_local = max(ds_local, 0.01);

    speed_measured = max(get_speed(state), 0.0);
    idx_cursor = idx;
    x_pred0 = [state.x; state.y; state.yaw];

    Xr = zeros(N, 1);
    Yr = zeros(N, 1);
    psir = zeros(N, 1);
    v_known = zeros(N, 1);

    for k = 1:N
        idx_advance = max(1, round(speed_measured * Ts / ds_local));
        idx_cursor = min(idx_cursor + idx_advance, n_ref);
        Xr(k) = ref.x(idx_cursor);
        Yr(k) = ref.y(idx_cursor);
        psir(k) = get_ref_heading(ref, idx_cursor);
        v_known(k) = speed_measured;
    end

    preview.Xr = Xr;
    preview.Yr = Yr;
    preview.psir = psir;
    preview.v = v_known;
    preview.idx0 = idx_cursor;
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

function problem = build_nmpc_problem(x0, x_pred0, preview, veh, p, Ts, delta_last)
% Gather everything needed by the solver.

    N = p.N;

    problem.x0 = x0;
    problem.x_pred0 = x_pred0;
    problem.preview = preview;
    problem.veh = veh;
    problem.p = p;
    problem.Ts = Ts;
    problem.delta_prev = delta_last;
    problem.u_init = build_initial_guess(problem);
    problem.lb = p.delta_min * ones(N, 1);
    problem.ub = p.delta_max * ones(N, 1);
    [problem.A, problem.b] = build_rate_constraints(problem);
end

function nmpc = solve_nmpc_problem(problem, p)
% Solve:
%   min_{delta_0,...,delta_{N-1}} J
% subject to:
%   - nonlinear KBM prediction model
%   - steering angle bounds
%   - steering-rate bounds
%   - optional lateral acceleration bound

    N = problem.p.N;
    cost_fun = @(U) nmpc_stage_cost(U, problem);
    nonlcon = @(U) nmpc_constraints(U, problem);

    try
        opts = optimoptions('fmincon', ...
            'Display', 'off', ...
            'Algorithm', 'sqp', ...
            'MaxFunctionEvaluations', p.max_fun_evals, ...
            'MaxIterations', p.max_iterations);

        [U_opt, J_opt, exitflag, output] = fmincon( ...
            cost_fun, ...
            problem.u_init, ...
            problem.A, problem.b, [], [], ...
            problem.lb, problem.ub, ...
            nonlcon, ...
            opts);

        [x_pred, ay_pred] = predict_kbm_trajectory(problem.x_pred0, U_opt, problem.preview.v, problem.veh, problem.Ts);
        status = "success";
    catch solver_err
        U_opt = problem.u_init;
        J_opt = NaN;
        exitflag = -999;
        output.message = solver_err.message;
        output.iterations = 0;
        [x_pred, ay_pred] = predict_kbm_trajectory(problem.x_pred0, U_opt, problem.preview.v, problem.veh, problem.Ts);
        status = "solver_error";
    end

    nmpc.delta0 = U_opt(1);
    nmpc.delta_seq = U_opt;
    nmpc.x_pred = x_pred;
    nmpc.ay_pred = ay_pred;
    nmpc.cost = J_opt;
    nmpc.exitflag = exitflag;
    nmpc.output = output;
    nmpc.status = status;
    nmpc.ref_preview = [problem.preview.Xr, problem.preview.Yr, problem.preview.psir];
    nmpc.v_preview = problem.preview.v;
    remember_last_delta_seq(U_opt);
end

function J = nmpc_stage_cost(U, problem)
% Stage cost:
%   J = sum_{k=0}^{N-1} q_X e_X^2 + q_Y e_Y^2 + q_psi e_psi^2
%                     + r_delta delta_k^2 + r_du (delta_k - delta_{k-1})^2

    [x_pred, ~] = predict_kbm_trajectory(problem.x_pred0, U, problem.preview.v, problem.veh, problem.Ts);

    q_X = problem.p.q_X;
    q_Y = problem.p.q_Y;
    q_psi = problem.p.q_psi;
    r_delta = problem.p.r_delta;
    r_du = get_r_du(problem.p);

    J = 0.0;
    delta_prev = problem.delta_prev;
    for k = 1:problem.p.N
        e_X = x_pred(1, k + 1) - problem.preview.Xr(k);
        e_Y = x_pred(2, k + 1) - problem.preview.Yr(k);
        e_psi = angle_wrap(x_pred(3, k + 1) - problem.preview.psir(k));
        delta_k = U(k);
        ddelta_k = delta_k - delta_prev;

        J = J ...
            + q_X * e_X^2 ...
            + q_Y * e_Y^2 ...
            + q_psi * e_psi^2 ...
            + r_delta * delta_k^2 ...
            + r_du * ddelta_k^2;
        delta_prev = delta_k;
    end
end

function [c, ceq] = nmpc_constraints(U, problem)
% Nonlinear constraints:
%   - optional lateral acceleration bound |a_y| <= a_y_max
%
% Steering angle bounds and steering-rate bounds are enforced by the box
% bounds and linear constraints assembled in build_nmpc_problem().

    [~, ay_pred] = predict_kbm_trajectory(problem.x_pred0, U, problem.preview.v, problem.veh, problem.Ts);

    c = [];

    if isfield(problem.p, 'use_ay_constraint') && problem.p.use_ay_constraint
        c = [
            c;
            ay_pred - problem.p.ay_max;
            -ay_pred - problem.p.ay_max
        ];
    end

    ceq = [];
end

function [x_pred, ay_pred] = predict_kbm_trajectory(x0, U, v_known, veh, Ts)
% Discrete-time nonlinear KBM prediction using forward Euler.

    N = numel(U);
    x_pred = zeros(3, N + 1);
    ay_pred = zeros(N, 1);
    x_pred(:, 1) = x0;

    lr = veh.lr;
    L = veh.L;

    for k = 1:N
        X_k = x_pred(1, k);
        Y_k = x_pred(2, k);
        psi_k = x_pred(3, k);
        v_k = v_known(k);
        delta_k = U(k);

        beta_k = kbm_beta_from_command(delta_k, lr, L);

        x_pred(1, k + 1) = X_k + Ts * v_k * cos(psi_k + beta_k);
        x_pred(2, k + 1) = Y_k + Ts * v_k * sin(psi_k + beta_k);
        x_pred(3, k + 1) = angle_wrap(psi_k + Ts * (v_k / max(lr, 1e-6)) * sin(beta_k));

        ay_pred(k) = -v_k^2 / max(L, 1e-6) * tan(delta_k);
    end
end

function beta = kbm_beta_from_command(delta_cmd, lr, L)
    beta = -atan((lr / max(L, 1e-6)) * tan(delta_cmd));
end

function u_init = build_initial_guess(problem)
    N = problem.p.N;
    delta_prev = problem.delta_prev;
    delta_rate_max = get_delta_rate_max(problem.p);
    ddelta_max = delta_rate_max * problem.Ts;
    last_delta_seq = delta_seq_memory('get');

    if isempty(last_delta_seq) || numel(last_delta_seq) ~= N
        u_init = delta_prev * ones(N, 1);
    else
        u_init = [last_delta_seq(2:end); last_delta_seq(end)];
    end

    u_init(1) = clamp(u_init(1), delta_prev - ddelta_max, delta_prev + ddelta_max);
    for k = 2:N
        u_init(k) = clamp(u_init(k), u_init(k - 1) - ddelta_max, u_init(k - 1) + ddelta_max);
    end
    u_init = min(max(u_init, problem.p.delta_min), problem.p.delta_max);

    delta_seq_memory('set', u_init);
end

function remember_last_delta_seq(U_opt)
    delta_seq_memory('set', U_opt);
end

function seq = delta_seq_memory(action, seq_in)
    persistent last_delta_seq

    if nargin < 1
        action = 'get';
    end

    switch action
        case 'set'
            last_delta_seq = seq_in;
        case 'clear'
            last_delta_seq = [];
    end

    if isempty(last_delta_seq)
        seq = [];
    else
        seq = last_delta_seq;
    end
end

function [A, b] = build_rate_constraints(problem)
    N = problem.p.N;
    delta_prev = problem.delta_prev;
    delta_rate_max = get_delta_rate_max(problem.p);
    ddelta_max = delta_rate_max * problem.Ts;
    slack = 1e-6;

    A = zeros(2 * N, N);
    b = zeros(2 * N, 1);
    A(1, 1) = 1;
    b(1) = delta_prev + ddelta_max + slack;
    A(2, 1) = -1;
    b(2) = -delta_prev + ddelta_max + slack;

    for k = 2:N
        r = 2 * k - 1;
        A(r, k) = 1;
        A(r, k - 1) = -1;
        b(r) = ddelta_max + slack;
        A(r + 1, k) = -1;
        A(r + 1, k - 1) = 1;
        b(r + 1) = ddelta_max + slack;
    end
end

function r_du = get_r_du(p)
    if isfield(p, 'r_du') && ~isempty(p.r_du)
        r_du = p.r_du;
    else
        r_du = 0.0;
    end
end

function delta_rate_max = get_delta_rate_max(p)
    if isfield(p, 'delta_rate_max') && ~isempty(p.delta_rate_max)
        delta_rate_max = p.delta_rate_max;
    elseif isfield(p, 'u_max') && ~isempty(p.u_max)
        delta_rate_max = p.u_max;
    else
        delta_rate_max = inf;
    end
end

function y = clamp(x, lo, hi)
    y = min(max(x, lo), hi);
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
