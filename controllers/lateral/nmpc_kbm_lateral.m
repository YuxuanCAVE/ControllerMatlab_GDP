function [delta_cmd, nmpc] = nmpc_kbm_lateral(state, ref, veh, dt, p, idx_hint, window)
% NMPC_KBM_LATERAL  Lateral NMPC using the full nonlinear kinematic bicycle model.
%
% State:
%   x_k = [X_k; Y_k; psi_k; delta_k]
%
% Input:
%   u_k = delta_dot_k
%
% Continuous-time model:
%   X_dot     = v * cos(psi + beta(delta))
%   Y_dot     = v * sin(psi + beta(delta))
%   psi_dot   = v / l_r * sin(beta(delta))
%   delta_dot = u
%
%   beta(delta) = atan((l_r / L) * tan(delta))
%
% Discretisation:
%   Forward Euler with sample time Ts
%
% Cost:
%   sum_{k=0}^{N-1} q_X e_X^2 + q_Y e_Y^2 + q_psi e_psi^2 + r_delta delta^2 + r_u u^2
%
% Constraints:
%   delta_min <= delta_k <= delta_max
%   u_min     <= u_k     <= u_max
%   optional: |a_y| <= a_y_max with a_y = v^2 / L * tan(delta)

    if nargin < 6
        idx_hint = [];
    end
    if nargin < 7
        window = [];
    end

    Ts = get_sample_time(p, dt);
    x0 = [state.x; state.y; state.yaw; state.delta];

    preview = build_reference_preview(state, ref, Ts, p, idx_hint, window);
    problem = build_nmpc_problem(x0, preview, veh, p, Ts);
    nmpc = solve_nmpc_problem(problem, p);

    % Apply the first steering-rate input over one sample to obtain
    % the commanded steering angle that will later pass through the
    % existing delay and rate-limiting actuator chain.
    delta_cmd = state.delta + Ts * nmpc.u0;
end

function Ts = get_sample_time(p, dt_default)
    if isfield(p, 'Ts') && ~isempty(p.Ts)
        Ts = p.Ts;
    else
        Ts = dt_default;
    end
end

function preview = build_reference_preview(state, ref, Ts, p, idx_hint, window)
% Build the known parameter / reference data used by the NMPC problem:
%   - reference global pose [X_r,k, Y_r,k, psi_r,k]
%   - known speed parameter v_k at each prediction step
%
% For this lateral-only NMPC, the full prediction horizon uses the current
% measured speed as a known parameter sequence:
%   v_k = v_measured,  k = 0,...,N-1

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

    Xr = zeros(N, 1);
    Yr = zeros(N, 1);
    psir = zeros(N, 1);
    v_known = zeros(N, 1);

    for k = 1:N
        if k > 1
            idx_advance = max(1, round(speed_measured * Ts / ds_local));
            idx_cursor = min(idx_cursor + idx_advance, n_ref);
        end

        Xr(k) = ref.x(idx_cursor);
        Yr(k) = ref.y(idx_cursor);
        psir(k) = get_ref_heading(ref, idx_cursor);
        v_known(k) = speed_measured;
    end

    preview.Xr = Xr;
    preview.Yr = Yr;
    preview.psir = psir;
    preview.v = v_known;
    preview.idx0 = idx;
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

function problem = build_nmpc_problem(x0, preview, veh, p, Ts)
% Gather everything needed by the solver. This keeps the solver call
% separate from model prediction, cost construction, and constraints.

    N = p.N;

    problem.x0 = x0;
    problem.preview = preview;
    problem.veh = veh;
    problem.p = p;
    problem.Ts = Ts;

    if isfield(p, 'u_init') && ~isempty(p.u_init)
        u_init = p.u_init * ones(N, 1);
    else
        u_init = zeros(N, 1);
    end

    problem.u_init = u_init;
    problem.lb = p.u_min * ones(N, 1);
    problem.ub = p.u_max * ones(N, 1);
end

function nmpc = solve_nmpc_problem(problem, p)
% Solve:
%   min_{u_0,...,u_{N-1}} J
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
            [], [], [], [], ...
            problem.lb, problem.ub, ...
            nonlcon, ...
            opts);

        [x_pred, ay_pred] = predict_kbm_trajectory(problem.x0, U_opt, problem.preview.v, problem.veh, problem.Ts);
        status = "success";
    catch solver_err
        U_opt = zeros(N, 1);
        J_opt = NaN;
        exitflag = -999;
        output.message = solver_err.message;
        output.iterations = 0;
        [x_pred, ay_pred] = predict_kbm_trajectory(problem.x0, U_opt, problem.preview.v, problem.veh, problem.Ts);
        status = "solver_error";
    end

    nmpc.u0 = U_opt(1);
    nmpc.u_seq = U_opt;
    nmpc.x_pred = x_pred;
    nmpc.ay_pred = ay_pred;
    nmpc.cost = J_opt;
    nmpc.exitflag = exitflag;
    nmpc.output = output;
    nmpc.status = status;
    nmpc.ref_preview = [problem.preview.Xr, problem.preview.Yr, problem.preview.psir];
    nmpc.v_preview = problem.preview.v;
end

function J = nmpc_stage_cost(U, problem)
% Implements exactly the requested stage cost:
%   J = sum_{k=0}^{N-1} q_X e_X^2 + q_Y e_Y^2 + q_psi e_psi^2
%                     + r_delta delta_k^2 + r_u u_k^2
% with no terminal cost.

    [x_pred, ~] = predict_kbm_trajectory(problem.x0, U, problem.preview.v, problem.veh, problem.Ts);

    q_X = problem.p.q_X;
    q_Y = problem.p.q_Y;
    q_psi = problem.p.q_psi;
    r_delta = problem.p.r_delta;
    r_u = problem.p.r_u;

    J = 0.0;
    for k = 1:problem.p.N
        e_X = x_pred(1, k) - problem.preview.Xr(k);
        e_Y = x_pred(2, k) - problem.preview.Yr(k);
        e_psi = angle_wrap(x_pred(3, k) - problem.preview.psir(k));
        delta_k = x_pred(4, k);
        u_k = U(k);

        J = J ...
            + q_X * e_X^2 ...
            + q_Y * e_Y^2 ...
            + q_psi * e_psi^2 ...
            + r_delta * delta_k^2 ...
            + r_u * u_k^2;
    end
end

function [c, ceq] = nmpc_constraints(U, problem)
% Nonlinear constraints for:
%   - steering angle bounds delta_min <= delta_k <= delta_max
%   - optional lateral acceleration bound |a_y| <= a_y_max
%
% Steering-rate bounds are enforced directly through lower/upper bounds on U.
% The initial state equality x_0 = x_measured is enforced by construction:
% prediction starts from the measured state problem.x0.

    [x_pred, ay_pred] = predict_kbm_trajectory(problem.x0, U, problem.preview.v, problem.veh, problem.Ts);

    delta_seq = x_pred(4, 2:end);
    c = [
        delta_seq - problem.p.delta_max;
        problem.p.delta_min - delta_seq
    ];

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
% Discrete-time nonlinear KBM prediction using forward Euler:
%
%   X_{k+1}     = X_k + Ts * v_k * cos(psi_k + beta_k)
%   Y_{k+1}     = Y_k + Ts * v_k * sin(psi_k + beta_k)
%   psi_{k+1}   = psi_k + Ts * v_k / l_r * sin(beta_k)
%   delta_{k+1} = delta_k + Ts * u_k
%
%   beta_k = atan((l_r / L) * tan(delta_k))

    N = numel(U);
    x_pred = zeros(4, N + 1);
    ay_pred = zeros(N, 1);
    x_pred(:, 1) = x0;

    lr = veh.lr;
    L = veh.L;

    for k = 1:N
        X_k = x_pred(1, k);
        Y_k = x_pred(2, k);
        psi_k = x_pred(3, k);
        delta_k = x_pred(4, k);
        v_k = v_known(k);
        u_k = U(k);

        beta_k = atan((lr / max(L, 1e-6)) * tan(delta_k));

        x_pred(1, k+1) = X_k + Ts * v_k * cos(psi_k + beta_k);
        x_pred(2, k+1) = Y_k + Ts * v_k * sin(psi_k + beta_k);
        x_pred(3, k+1) = angle_wrap(psi_k + Ts * (v_k / max(lr, 1e-6)) * sin(beta_k));
        x_pred(4, k+1) = delta_k + Ts * u_k;

        ay_pred(k) = v_k^2 / max(L, 1e-6) * tan(delta_k);
    end
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
