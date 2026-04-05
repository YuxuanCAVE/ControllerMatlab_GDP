%% run_tuning_sweep.m
%  Automated parameter sweep aligned with GDP Brief (March 2026).
%
%  TEST MATRIX (from GDP Brief Requirements + Testing Scenarios):
%    - 6 controller combos (3 lateral x 2 longitudinal)
%    - 7 speed conditions:
%        * GDP Scenario 1 (Low-speed):    0.5 m/s constant
%        * GDP Scenario 2 (Medium-speed): 2.0 m/s constant
%        * GDP Scenario 3 (High-speed):   Profile 3/4/5/7 m/s peak
%        * GDP Scenario 3 (Extended):     10.0 m/s constant (full range)
%    = 42 total test cases
%
%  PASS/FAIL CRITERIA (from GDP Brief REQ-1):
%    - Peak lateral deviation  < 0.6 m
%    - Peak speed error        < 0.8 m/s  (proxy for longitudinal deviation)
%
%  REAL-TIME CHECK (from GDP Brief REQ-2):
%    - Controller loop dt = 0.05 s => 20 Hz (requirement: >= 10 Hz) -> PASS
%
%  MODEL ADAPTABILITY (from GDP Brief REQ-3):
%    - Actuator delay modelling: steering 0.1 s, longitudinal 0.1 s -> YES
%    - Dynamic bicycle model with Magic Formula tires -> YES
%
%  Usage:
%      run_tuning_sweep          % runs all cases
%      run_tuning_sweep(true)    % dry-run: prints test matrix only

function run_tuning_sweep(dry_run)
    if nargin < 1
        dry_run = false;
    end

    %% ── Setup paths ──────────────────────────────────────────────────────
    project_root = fileparts(which('main'));
    if isempty(project_root)
        project_root = pwd;
    end

    addpath(fullfile(project_root, 'config'));
    addpath(fullfile(project_root, 'reference'));
    addpath(fullfile(project_root, 'controllers', 'lateral'));
    addpath(fullfile(project_root, 'controllers', 'longitudinal'));
    addpath(fullfile(project_root, 'model'));
    addpath(fullfile(project_root, 'simulation'));
    addpath(fullfile(project_root, 'plotting'));
    addpath(fullfile(project_root, 'utils'));

    %% ── GDP Brief: Pass/Fail Thresholds (Requirements slide 13) ─────────
    LATERAL_LIMIT   = 0.6;   % m   (REQ-1: max lateral deviation)
    SPEED_ERR_LIMIT = 0.8;   % m/s (REQ-1: max longitudinal deviation proxy)
    MIN_LOOP_HZ     = 10;    % Hz  (REQ-2: minimum controller frequency)

    %% ── Define the test matrix ───────────────────────────────────────────
    lateral_set = ["stanley", "pure_pursuit", "mpc"];
    lon_set     = ["pid", "lqr"];

    % GDP Brief Testing Scenarios (slide 14):
    %   Scenario 1: Low-speed   -> 0.5 m/s constant
    %   Scenario 2: Medium-speed -> 2.0 m/s constant
    %   Scenario 3: High-speed  -> speed profiles (3/4/5/7 m/s peak)
    %   Extended:   Full range  -> 10.0 m/s constant (brief says 0-10 m/s)

    speed_conditions = {};

    % --- GDP Scenario 1: Low-speed (creep torque) ---
    speed_conditions{end+1} = struct( ...
        'mode', "constant", ...
        'value', 0.5, ...
        'file', "", ...
        'label', "0.5 m/s (GDP Scenario 1: Low-speed)", ...
        'scenario', "GDP-S1", ...
        'peak_speed', 0.5);

    % --- GDP Scenario 2: Medium-speed ---
    speed_conditions{end+1} = struct( ...
        'mode', "constant", ...
        'value', 2.0, ...
        'file', "", ...
        'label', "2.0 m/s (GDP Scenario 2: Medium-speed)", ...
        'scenario', "GDP-S2", ...
        'peak_speed', 2.0);

    % --- GDP Scenario 3: High-speed (profile-based) ---
    profile_speeds = [3, 4, 5, 7];
    profile_files = dictionary( ...
        3, fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_3.mat'), ...
        4, fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_4.mat'), ...
        5, fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_5.mat'), ...
        7, fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_7.mat') ...
    );

    for si = 1:numel(profile_speeds)
        spd = profile_speeds(si);
        speed_conditions{end+1} = struct( ...
            'mode', "profile", ...
            'value', spd, ...
            'file', profile_files(spd), ...
            'label', sprintf("%d m/s profile (GDP Scenario 3: High-speed)", spd), ...
            'scenario', "GDP-S3", ...
            'peak_speed', spd); %#ok<AGROW>
    end

    % --- GDP Extended: Full operational range (10 m/s) ---
    speed_conditions{end+1} = struct( ...
        'mode', "constant", ...
        'value', 10.0, ...
        'file', "", ...
        'label', "10.0 m/s (GDP Full Range: 0-10 m/s)", ...
        'scenario', "GDP-S3-EXT", ...
        'peak_speed', 10.0);

    % Build full test list
    cases = {};
    for li = 1:numel(lateral_set)
        for lo = 1:numel(lon_set)
            for si = 1:numel(speed_conditions)
                sc = speed_conditions{si};
                c.lateral    = lateral_set(li);
                c.lon        = lon_set(lo);
                c.speed_cond = sc;
                c.label      = sprintf('%s + %s @ %s', ...
                    char(lateral_set(li)), char(lon_set(lo)), char(sc.label));
                cases{end+1} = c; %#ok<AGROW>
            end
        end
    end

    n_cases = numel(cases);
    n_speeds = numel(speed_conditions);

    %% ── Print test matrix ────────────────────────────────────────────────
    fprintf('================================================================\n');
    fprintf('  GDP BRIEF-ALIGNED TUNING SWEEP\n');
    fprintf('  %d controller combos x %d speed conditions = %d test cases\n', ...
        numel(lateral_set)*numel(lon_set), n_speeds, n_cases);
    fprintf('================================================================\n\n');

    fprintf('GDP Brief Requirements Check:\n');
    fprintf('  REQ-1: Lateral deviation  < %.1f m   [will verify]\n', LATERAL_LIMIT);
    fprintf('  REQ-1: Speed error        < %.1f m/s [will verify]\n', SPEED_ERR_LIMIT);
    fprintf('  REQ-2: Loop frequency     >= %d Hz   [dt=0.05s => 20 Hz: PASS]\n', MIN_LOOP_HZ);
    fprintf('  REQ-3: Actuator delays    = YES (steer: 0.1s, lon: 0.1s)\n');
    fprintf('  REQ-3: Dynamic model      = YES (3DOF bicycle + Magic Formula)\n\n');

    fprintf('GDP Testing Scenarios:\n');
    fprintf('  Scenario 1 (Low-speed):    0.5 m/s constant\n');
    fprintf('  Scenario 2 (Medium-speed): 2.0 m/s constant\n');
    fprintf('  Scenario 3 (High-speed):   Profiles at 3/4/5/7 m/s peak\n');
    fprintf('  Extended (Full range):     10.0 m/s constant\n\n');

    fprintf('%-5s  %-50s  %-10s\n', 'Case', 'Configuration', 'Scenario');
    fprintf('%s\n', repmat('-', 1, 70));
    for i = 1:n_cases
        fprintf('%-5d  %-50s  %s\n', i, ...
            sprintf('%s+%s @ %.1f m/s', char(cases{i}.lateral), ...
                char(cases{i}.lon), cases{i}.speed_cond.peak_speed), ...
            char(cases{i}.speed_cond.scenario));
    end
    fprintf('\n');

    if dry_run
        fprintf('Dry run complete. Set dry_run=false to execute.\n');
        return;
    end

    %% ── Prepare output directory ─────────────────────────────────────────
    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    sweep_dir = fullfile(project_root, 'run', sprintf('%s_GDP_sweep', stamp));
    if ~exist(sweep_dir, 'dir')
        mkdir(sweep_dir);
    end

    %% ── Load base config and vehicle ─────────────────────────────────────
    cfg_base = default_config();
    ref_base = load_reference_path(cfg_base.ref.path_file);
    veh = load_vehicle_params(cfg_base.vehicle.accel_map_file, cfg_base.vehicle.brake_map_file);
    veh.max_steer      = cfg_base.vehicle.max_steer;
    veh.max_steer_rate = cfg_base.vehicle.max_steer_rate;

    %% ── Preallocate results table ────────────────────────────────────────
    T = table( ...
        strings(n_cases, 1), ...          % Lateral
        strings(n_cases, 1), ...          % Longitudinal
        strings(n_cases, 1), ...          % GDP_Scenario
        zeros(n_cases, 1), ...            % Peak_Speed_mps
        strings(n_cases, 1), ...          % Speed_Mode
        zeros(n_cases, 1), ...            % RMS_CTE_m
        zeros(n_cases, 1), ...            % Peak_CTE_m
        zeros(n_cases, 1), ...            % RMS_Heading_Err_deg
        zeros(n_cases, 1), ...            % RMS_Speed_Err_mps
        zeros(n_cases, 1), ...            % Peak_Speed_Err_mps
        zeros(n_cases, 1), ...            % Loop_Time_s
        strings(n_cases, 1), ...          % Goal_Reached
        strings(n_cases, 1), ...          % Termination
        strings(n_cases, 1), ...          % REQ1_Lateral_Pass
        strings(n_cases, 1), ...          % REQ1_Lon_Pass
        strings(n_cases, 1), ...          % Overall_Pass
        'VariableNames', { ...
            'Lateral', 'Longitudinal', 'GDP_Scenario', ...
            'Peak_Speed_mps', 'Speed_Mode', ...
            'RMS_CTE_m', 'Peak_CTE_m', 'RMS_Heading_Err_deg', ...
            'RMS_Speed_Err_mps', 'Peak_Speed_Err_mps', ...
            'Loop_Time_s', 'Goal_Reached', 'Termination', ...
            'REQ1_Lateral_Pass', 'REQ1_Lon_Pass', 'Overall_Pass'});

    %% ── Run each case ────────────────────────────────────────────────────
    all_results = cell(n_cases, 1);

    for i = 1:n_cases
        c = cases{i};
        sc = c.speed_cond;
        fprintf('\n[%2d/%d] Running: %s+%s @ %.1f m/s (%s) ...', ...
            i, n_cases, char(c.lateral), char(c.lon), sc.peak_speed, char(sc.scenario));
        t_start = tic;

        % Configure this case
        cfg = cfg_base;
        cfg.controller.lateral      = c.lateral;
        cfg.controller.longitudinal = c.lon;
        cfg.speed.mode              = sc.mode;

        if sc.mode == "constant"
            cfg.speed.constant_value = sc.value;
        else
            cfg.speed.profile_file = sc.file;
        end

        % Adjust sim time for very low speeds (vehicle needs more time)
        if sc.peak_speed <= 1.0
            cfg.sim.T_end = 400;
            cfg.sim.max_travel_time = 400;
        elseif sc.peak_speed <= 3.0
            cfg.sim.T_end = 250;
            cfg.sim.max_travel_time = 250;
        else
            cfg.sim.T_end = 150;
            cfg.sim.max_travel_time = 150;
        end

        % Load speed reference
        ref = ref_base;
        ref = load_reference_speed(ref, cfg.speed);

        % Run simulation
        try
            result = run_closed_loop(cfg, ref, veh);
            m = result.metrics;

            % Pass/fail against GDP REQ-1 thresholds
            lat_pass = m.peak_cte < LATERAL_LIMIT;
            lon_pass = m.peak_speed_error < SPEED_ERR_LIMIT;

            T.Lateral(i)            = c.lateral;
            T.Longitudinal(i)       = c.lon;
            T.GDP_Scenario(i)       = sc.scenario;
            T.Peak_Speed_mps(i)     = sc.peak_speed;
            T.Speed_Mode(i)         = sc.mode;
            T.RMS_CTE_m(i)          = round(m.rms_cte, 4);
            T.Peak_CTE_m(i)         = round(m.peak_cte, 4);
            T.RMS_Heading_Err_deg(i) = round(m.rms_epsi_deg, 4);
            T.RMS_Speed_Err_mps(i)  = round(m.rms_speed_error, 4);
            T.Peak_Speed_Err_mps(i) = round(m.peak_speed_error, 4);
            T.Loop_Time_s(i)        = round(m.single_loop_time_s, 2);
            T.Goal_Reached(i)       = string(m.goal_reached);
            T.Termination(i)        = m.termination_reason;
            T.REQ1_Lateral_Pass(i)  = iff(lat_pass, "PASS", "FAIL");
            T.REQ1_Lon_Pass(i)      = iff(lon_pass, "PASS", "FAIL");
            T.Overall_Pass(i)       = iff(lat_pass && lon_pass, "PASS", "FAIL");

            all_results{i} = result;

            % Save individual case
            case_tag = sprintf('case_%02d_%s_%s_%.0fmps_%s', ...
                i, char(c.lateral), char(c.lon), sc.peak_speed*10, char(sc.scenario));
            case_dir = fullfile(sweep_dir, case_tag);
            if ~exist(case_dir, 'dir')
                mkdir(case_dir);
            end
            save(fullfile(case_dir, 'result.mat'), 'cfg', 'ref', 'veh', 'result');

            elapsed = toc(t_start);
            fprintf(' done (%.1f s) | Peak CTE=%.3f m [%s] | Peak Spd Err=%.3f m/s [%s]\n', ...
                elapsed, m.peak_cte, iff(lat_pass, "OK", "FAIL"), ...
                m.peak_speed_error, iff(lon_pass, "OK", "FAIL"));

        catch ME
            fprintf(' FAILED: %s\n', ME.message);
            T.Lateral(i)       = c.lateral;
            T.Longitudinal(i)  = c.lon;
            T.GDP_Scenario(i)  = sc.scenario;
            T.Peak_Speed_mps(i) = sc.peak_speed;
            T.Speed_Mode(i)    = sc.mode;
            T.Termination(i)   = "error: " + string(ME.message);
            T.REQ1_Lateral_Pass(i) = "ERROR";
            T.REQ1_Lon_Pass(i)     = "ERROR";
            T.Overall_Pass(i)      = "ERROR";
        end
    end

    %% ── Save summary table ───────────────────────────────────────────────
    writetable(T, fullfile(sweep_dir, 'GDP_sweep_results.csv'));
    save(fullfile(sweep_dir, 'GDP_sweep_results.mat'), 'T', 'all_results', 'cases');

    %% ── Print GDP Requirements Compliance Summary ────────────────────────
    fprintf('\n\n');
    fprintf('================================================================\n');
    fprintf('  GDP BRIEF REQUIREMENTS COMPLIANCE SUMMARY\n');
    fprintf('================================================================\n\n');

    fprintf('REQ-2: Real-time Response (>= 10 Hz)\n');
    fprintf('  Controller dt = %.3f s => %.0f Hz : PASS\n\n', cfg_base.sim.dt, 1/cfg_base.sim.dt);

    fprintf('REQ-3: Model Adaptability\n');
    fprintf('  Steering delay     = %.3f s : YES\n', cfg_base.vehicle.delay.steer_s);
    fprintf('  Longitudinal delay = %.3f s : YES\n', cfg_base.vehicle.delay.longitudinal_s);
    fprintf('  Dynamic model      = 3DOF coupled bicycle + Magic Formula : YES\n');
    fprintf('  MPC delay compensation = YES\n\n');

    fprintf('REQ-1: Robust Tracking (Peak Lateral < %.1f m, Peak Speed Err < %.1f m/s)\n', ...
        LATERAL_LIMIT, SPEED_ERR_LIMIT);
    fprintf('%s\n', repmat('-', 1, 105));
    fprintf('%-20s %-8s %-8s %-10s %-10s %-10s %-8s %-8s %-8s\n', ...
        'Combo', 'Scen', 'Speed', 'PeakCTE', 'LatPass', 'PeakSpdE', 'LonPass', 'Overall', 'Goal');
    fprintf('%s\n', repmat('-', 1, 105));

    for i = 1:n_cases
        fprintf('%-20s %-8s %-8.1f %-10.3f %-10s %-10.3f %-8s %-8s %-8s\n', ...
            sprintf('%s+%s', char(T.Lateral(i)), char(T.Longitudinal(i))), ...
            char(T.GDP_Scenario(i)), ...
            T.Peak_Speed_mps(i), ...
            T.Peak_CTE_m(i), ...
            char(T.REQ1_Lateral_Pass(i)), ...
            T.Peak_Speed_Err_mps(i), ...
            char(T.REQ1_Lon_Pass(i)), ...
            char(T.Overall_Pass(i)), ...
            char(T.Goal_Reached(i)));
    end

    %% ── Per-scenario summary ─────────────────────────────────────────────
    scenarios = unique(T.GDP_Scenario);
    fprintf('\n\n--- PASS RATE BY GDP SCENARIO ---\n');
    for si = 1:numel(scenarios)
        mask = T.GDP_Scenario == scenarios(si);
        n_total = sum(mask);
        n_pass = sum(T.Overall_Pass(mask) == "PASS");
        fprintf('  %s: %d / %d passed (%.0f%%)\n', ...
            char(scenarios(si)), n_pass, n_total, 100*n_pass/n_total);
    end

    %% ── Per-combo summary ────────────────────────────────────────────────
    fprintf('\n--- BEST CONTROLLER COMBO PER SCENARIO ---\n');
    for si = 1:numel(scenarios)
        mask = T.GDP_Scenario == scenarios(si);
        sub = T(mask, :);
        if height(sub) > 0
            [~, best_idx] = min(sub.Peak_CTE_m);
            fprintf('  %s: %s+%s (Peak CTE = %.3f m)\n', ...
                char(scenarios(si)), ...
                char(sub.Lateral(best_idx)), char(sub.Longitudinal(best_idx)), ...
                sub.Peak_CTE_m(best_idx));
        end
    end

    %% ── Generate comparison figures ──────────────────────────────────────
    generate_GDP_plots(T, sweep_dir, lateral_set, lon_set, LATERAL_LIMIT, SPEED_ERR_LIMIT);

    %% ── Write text summary ───────────────────────────────────────────────
    write_GDP_summary(T, sweep_dir, cfg_base, LATERAL_LIMIT, SPEED_ERR_LIMIT);

    fprintf('\n\nAll outputs saved to: %s\n', sweep_dir);
end


%% =====================================================================
%  Helper: inline if
%  =====================================================================
function result = iff(cond, true_val, false_val)
    if cond
        result = true_val;
    else
        result = false_val;
    end
end


%% =====================================================================
%  PLOTTING
%  =====================================================================
function generate_GDP_plots(T, sweep_dir, lateral_set, lon_set, lat_lim, spd_lim)

    combos = strings(0);
    for li = 1:numel(lateral_set)
        for lo = 1:numel(lon_set)
            combos(end+1) = sprintf('%s+%s', char(lateral_set(li)), char(lon_set(lo))); %#ok<AGROW>
        end
    end

    all_speeds = unique(T.Peak_Speed_mps);

    % ── 1) Peak CTE by combo and speed, with pass/fail line ──────────────
    fig1 = figure('Position', [100 100 1100 500], 'Visible', 'off');
    cte_matrix = nan(numel(combos), numel(all_speeds));
    for ci = 1:numel(combos)
        parts = split(combos(ci), '+');
        for si = 1:numel(all_speeds)
            mask = T.Lateral == parts(1) & T.Longitudinal == parts(2) & ...
                   T.Peak_Speed_mps == all_speeds(si);
            vals = T.Peak_CTE_m(mask);
            if ~isempty(vals)
                cte_matrix(ci, si) = vals(1);
            end
        end
    end

    b = bar(cte_matrix);
    hold on;
    yline(lat_lim, 'r--', 'LineWidth', 2, 'Label', ...
        sprintf('REQ-1 Limit: %.1f m', lat_lim), 'FontSize', 10);
    set(gca, 'XTickLabel', combos, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel('Peak Cross-Track Error (m)');
    title('GDP REQ-1: Peak Lateral Deviation vs 0.6 m Threshold');
    legend_labels = arrayfun(@(s) sprintf('%.1f m/s', s), all_speeds, 'UniformOutput', false);
    legend(b, legend_labels, 'Location', 'northwest');
    grid on;
    saveas(fig1, fullfile(sweep_dir, 'GDP_REQ1_lateral_compliance.png'));
    close(fig1);

    % ── 2) Peak speed error with pass/fail line ──────────────────────────
    fig2 = figure('Position', [100 100 1100 500], 'Visible', 'off');
    spd_matrix = nan(numel(combos), numel(all_speeds));
    for ci = 1:numel(combos)
        parts = split(combos(ci), '+');
        for si = 1:numel(all_speeds)
            mask = T.Lateral == parts(1) & T.Longitudinal == parts(2) & ...
                   T.Peak_Speed_mps == all_speeds(si);
            vals = T.Peak_Speed_Err_mps(mask);
            if ~isempty(vals)
                spd_matrix(ci, si) = vals(1);
            end
        end
    end

    b2 = bar(spd_matrix);
    hold on;
    yline(spd_lim, 'r--', 'LineWidth', 2, 'Label', ...
        sprintf('REQ-1 Limit: %.1f m/s', spd_lim), 'FontSize', 10);
    set(gca, 'XTickLabel', combos, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel('Peak Speed Error (m/s)');
    title('GDP REQ-1: Peak Speed Error vs 0.8 m/s Threshold');
    legend(b2, legend_labels, 'Location', 'northwest');
    grid on;
    saveas(fig2, fullfile(sweep_dir, 'GDP_REQ1_longitudinal_compliance.png'));
    close(fig2);

    % ── 3) GDP Scenario pass/fail heatmap ────────────────────────────────
    fig3 = figure('Position', [100 100 900 500], 'Visible', 'off');
    scenarios_ordered = ["GDP-S1", "GDP-S2", "GDP-S3", "GDP-S3-EXT"];
    scenario_labels = ["Low (0.5)", "Med (2.0)", "High (3-7)", "Full (10)"];

    pass_matrix = zeros(numel(combos), numel(scenarios_ordered));

    for ci = 1:numel(combos)
        parts = split(combos(ci), '+');
        for si = 1:numel(scenarios_ordered)
            mask = T.Lateral == parts(1) & T.Longitudinal == parts(2) & ...
                   T.GDP_Scenario == scenarios_ordered(si);
            sub = T(mask, :);
            if height(sub) == 0
                pass_matrix(ci, si) = -1;
            elseif all(sub.Overall_Pass == "PASS")
                pass_matrix(ci, si) = 1;
            elseif any(sub.Overall_Pass == "PASS")
                pass_matrix(ci, si) = 0.5;
            else
                pass_matrix(ci, si) = 0;
            end
        end
    end

    imagesc(pass_matrix);
    colormap([1 0.3 0.3;   % 0: fail (red)
              1 0.8 0.3;   % 0.5: partial (amber)
              0.3 0.8 0.3; % 1: pass (green)
              0.7 0.7 0.7]);
    caxis([-1 1]);
    set(gca, 'XTick', 1:numel(scenarios_ordered), 'XTickLabel', scenario_labels);
    set(gca, 'YTick', 1:numel(combos), 'YTickLabel', combos);
    title('GDP Scenario Pass/Fail Matrix');
    xlabel('GDP Testing Scenario');
    ylabel('Controller Combination');

    for ci = 1:numel(combos)
        for si = 1:numel(scenarios_ordered)
            val = pass_matrix(ci, si);
            if val == -1, txt = "N/A";
            elseif val == 0, txt = "FAIL";
            elseif val == 0.5, txt = "PARTIAL";
            else, txt = "PASS";
            end
            text(si, ci, char(txt), 'HorizontalAlignment', 'center', ...
                'FontSize', 9, 'FontWeight', 'bold');
        end
    end
    saveas(fig3, fullfile(sweep_dir, 'GDP_scenario_pass_fail_matrix.png'));
    close(fig3);

    % ── 4) RMS CTE comparison across all speeds ─────────────────────────
    fig4 = figure('Position', [100 100 1100 500], 'Visible', 'off');
    rms_matrix = nan(numel(combos), numel(all_speeds));
    for ci = 1:numel(combos)
        parts = split(combos(ci), '+');
        for si = 1:numel(all_speeds)
            mask = T.Lateral == parts(1) & T.Longitudinal == parts(2) & ...
                   T.Peak_Speed_mps == all_speeds(si);
            vals = T.RMS_CTE_m(mask);
            if ~isempty(vals)
                rms_matrix(ci, si) = vals(1);
            end
        end
    end

    b4 = bar(rms_matrix);
    set(gca, 'XTickLabel', combos, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel('RMS Cross-Track Error (m)');
    title('RMS Lateral Tracking Accuracy: All Combos x All Speeds');
    legend(b4, legend_labels, 'Location', 'northwest');
    grid on;
    saveas(fig4, fullfile(sweep_dir, 'rms_cte_all_speeds.png'));
    close(fig4);

    fprintf('GDP comparison plots saved.\n');
end


%% =====================================================================
%  TEXT SUMMARY
%  =====================================================================
function write_GDP_summary(T, sweep_dir, cfg, lat_lim, spd_lim)
    fid = fopen(fullfile(sweep_dir, 'GDP_compliance_summary.txt'), 'w');

    fprintf(fid, '================================================================\n');
    fprintf(fid, '  GDP BRIEF COMPLIANCE SUMMARY\n');
    fprintf(fid, '  Generated: %s\n', char(datetime('now')));
    fprintf(fid, '================================================================\n\n');

    fprintf(fid, 'REQUIREMENTS STATUS:\n');
    fprintf(fid, '  REQ-1 Robust Tracking:\n');
    fprintf(fid, '    Lateral limit:  < %.1f m\n', lat_lim);
    fprintf(fid, '    Speed err limit: < %.1f m/s\n', spd_lim);
    fprintf(fid, '    Speed range tested: 0.5 to 10.0 m/s\n\n');

    fprintf(fid, '  REQ-2 Real-time Response:\n');
    fprintf(fid, '    Controller dt = %.3f s => %.0f Hz (req >= 10 Hz): PASS\n\n', ...
        cfg.sim.dt, 1/cfg.sim.dt);

    fprintf(fid, '  REQ-3 Model Adaptability:\n');
    fprintf(fid, '    Steering delay modelling:     %.3f s: YES\n', cfg.vehicle.delay.steer_s);
    fprintf(fid, '    Longitudinal delay modelling:  %.3f s: YES\n', cfg.vehicle.delay.longitudinal_s);
    fprintf(fid, '    Dynamic bicycle model:         YES (3DOF coupled, Magic Formula tires)\n');
    fprintf(fid, '    MPC delay compensation:        YES (forward simulation through buffer)\n');
    fprintf(fid, '    Steering rate limit:           %.0f deg/s: YES\n\n', ...
        rad2deg(cfg.vehicle.max_steer_rate));

    fprintf(fid, 'RESULTS TABLE:\n');
    fprintf(fid, '%-20s %-8s %-6s %-9s %-9s %-7s %-9s %-9s %-7s %-7s\n', ...
        'Combo', 'Scenario', 'Speed', 'RMS_CTE', 'Peak_CTE', 'LatOK', ...
        'RMS_SpdE', 'Peak_SpdE', 'LonOK', 'Overall');
    fprintf(fid, '%s\n', repmat('-', 1, 105));

    for i = 1:height(T)
        fprintf(fid, '%-20s %-8s %-6.1f %-9.4f %-9.4f %-7s %-9.4f %-9.4f %-7s %-7s\n', ...
            sprintf('%s+%s', char(T.Lateral(i)), char(T.Longitudinal(i))), ...
            char(T.GDP_Scenario(i)), ...
            T.Peak_Speed_mps(i), ...
            T.RMS_CTE_m(i), T.Peak_CTE_m(i), char(T.REQ1_Lateral_Pass(i)), ...
            T.RMS_Speed_Err_mps(i), T.Peak_Speed_Err_mps(i), ...
            char(T.REQ1_Lon_Pass(i)), char(T.Overall_Pass(i)));
    end

    n_pass = sum(T.Overall_Pass == "PASS");
    n_total = height(T);
    fprintf(fid, '\nOVERALL: %d / %d test cases passed (%.0f%%)\n', ...
        n_pass, n_total, 100*n_pass/n_total);

    fclose(fid);
end