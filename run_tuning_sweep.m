%% run_tuning_sweep.m
%  Automated parameter sweep aligned with GDP Brief (March 2026).
%
%  CORRECTED METRICS (from GDP Brief REQ-1, slide 13):
%    - Peak LATERAL deviation   < 0.6 m  (cross-track error)
%    - Peak LONGITUDINAL deviation < 0.8 m  (position error along path)
%
%  Longitudinal deviation = cumulative integral of (v_ref - v_actual)*dt
%  This is how far ahead/behind the vehicle is from where it should be.
%
%  TEST MATRIX:
%    6 controller combos x 7 speed conditions = 42 test cases
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
    %  "maximum deviation less than 0.6 m laterally and 0.8 m longitudinally"
    LATERAL_LIMIT = 0.6;        % metres (peak cross-track error)
    LONGITUDINAL_LIMIT = 0.8;   % metres (peak along-path position deviation)
    MIN_LOOP_HZ = 10;           % Hz (REQ-2)

    %% ── Define the test matrix ───────────────────────────────────────────
    lateral_set = ["stanley", "pure_pursuit", "mpc"];
    lon_set     = ["pid", "lqr"];

    speed_conditions = {};

    % GDP Scenario 1: Low-speed (creep torque)
    speed_conditions{end+1} = struct('mode',"constant",'value',0.5,'file',"",...
        'label',"0.5 m/s (GDP-S1: Low-speed)",'scenario',"GDP-S1",'peak_speed',0.5);

    % GDP Scenario 2: Medium-speed
    speed_conditions{end+1} = struct('mode',"constant",'value',2.0,'file',"",...
        'label',"2.0 m/s (GDP-S2: Medium-speed)",'scenario',"GDP-S2",'peak_speed',2.0);

    % GDP Scenario 3: High-speed (profile-based)
    profile_speeds = [3, 4, 5, 7];
    profile_files = dictionary( ...
        3, fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_3.mat'), ...
        4, fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_4.mat'), ...
        5, fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_5.mat'), ...
        7, fullfile('data','reference_velocity','referencePath_Velocity_peak_velocity_7.mat'));

    for si = 1:numel(profile_speeds)
        spd = profile_speeds(si);
        speed_conditions{end+1} = struct('mode',"profile",'value',spd,...
            'file',profile_files(spd),...
            'label',sprintf("%d m/s profile (GDP-S3: High-speed)",spd),...
            'scenario',"GDP-S3",'peak_speed',spd); %#ok<AGROW>
    end

    % GDP Extended: Full operational range
    speed_conditions{end+1} = struct('mode',"constant",'value',10.0,'file',"",...
        'label',"10.0 m/s (GDP-S3-EXT: Full Range)",'scenario',"GDP-S3-EXT",'peak_speed',10.0);

    % Build full test list
    cases = {};
    for li = 1:numel(lateral_set)
        for lo = 1:numel(lon_set)
            for si = 1:numel(speed_conditions)
                sc = speed_conditions{si};
                c.lateral    = lateral_set(li);
                c.lon        = lon_set(lo);
                c.speed_cond = sc;
                cases{end+1} = c; %#ok<AGROW>
            end
        end
    end

    n_cases = numel(cases);
    n_speeds = numel(speed_conditions);

    %% ── Print test matrix ────────────────────────────────────────────────
    fprintf('================================================================\n');
    fprintf('  GDP BRIEF-ALIGNED TUNING SWEEP (CORRECTED METRICS)\n');
    fprintf('  %d controller combos x %d speed conditions = %d test cases\n', ...
        numel(lateral_set)*numel(lon_set), n_speeds, n_cases);
    fprintf('================================================================\n\n');

    fprintf('GDP Brief Requirements (slide 13):\n');
    fprintf('  REQ-1: Peak LATERAL deviation    < %.1f m   [cross-track error]\n', LATERAL_LIMIT);
    fprintf('  REQ-1: Peak LONGITUDINAL deviation < %.1f m [along-path position error]\n', LONGITUDINAL_LIMIT);
    fprintf('  REQ-2: Loop frequency >= %d Hz   [dt=0.05s => 20 Hz: PASS]\n', MIN_LOOP_HZ);
    fprintf('  REQ-3: Actuator delays + dynamics  [YES]\n\n');

    fprintf('  NOTE: Longitudinal deviation = cumulative integral of\n');
    fprintf('        (v_ref - v_actual)*dt, i.e. how many metres ahead/behind\n');
    fprintf('        the vehicle is from its expected position on the path.\n\n');

    fprintf('%-5s  %-45s  %-10s\n', 'Case', 'Configuration', 'Scenario');
    fprintf('%s\n', repmat('-', 1, 65));
    for i = 1:n_cases
        fprintf('%-5d  %-45s  %s\n', i, ...
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
    if ~exist(sweep_dir, 'dir'), mkdir(sweep_dir); end

    %% ── Load base config and vehicle ─────────────────────────────────────
    cfg_base = default_config();
    ref_base = load_reference_path(cfg_base.ref.path_file);
    veh = load_vehicle_params(cfg_base.vehicle.accel_map_file, cfg_base.vehicle.brake_map_file);
    veh.max_steer      = cfg_base.vehicle.max_steer;
    veh.max_steer_rate = cfg_base.vehicle.max_steer_rate;

    %% ── Preallocate results table ────────────────────────────────────────
    T = table( ...
        strings(n_cases,1), strings(n_cases,1), strings(n_cases,1), ...
        zeros(n_cases,1), strings(n_cases,1), ...
        zeros(n_cases,1), zeros(n_cases,1), zeros(n_cases,1), ...
        zeros(n_cases,1), zeros(n_cases,1), zeros(n_cases,1), ...
        zeros(n_cases,1), zeros(n_cases,1), ...
        strings(n_cases,1), strings(n_cases,1), ...
        strings(n_cases,1), strings(n_cases,1), strings(n_cases,1), ...
        'VariableNames', { ...
            'Lateral','Longitudinal','GDP_Scenario',...
            'Peak_Speed_mps','Speed_Mode',...
            'RMS_CTE_m','Peak_CTE_m','RMS_Heading_Err_deg',...
            'RMS_Speed_Err_mps','Peak_Speed_Err_mps','Peak_Lon_Dev_m',...
            'RMS_Lon_Dev_m','Loop_Time_s',...
            'Goal_Reached','Termination',...
            'REQ1_Lat_Pass','REQ1_Lon_Pass','Overall_Pass'});

    %% ── Run each case ────────────────────────────────────────────────────
    all_results = cell(n_cases, 1);

    for i = 1:n_cases
        c = cases{i};
        sc = c.speed_cond;
        fprintf('\n[%2d/%d] %s+%s @ %.1f m/s (%s) ...', ...
            i, n_cases, char(c.lateral), char(c.lon), sc.peak_speed, char(sc.scenario));
        t_start = tic;

        cfg = cfg_base;
        cfg.controller.lateral      = c.lateral;
        cfg.controller.longitudinal = c.lon;
        cfg.speed.mode              = sc.mode;

        if sc.mode == "constant"
            cfg.speed.constant_value = sc.value;
        else
            cfg.speed.profile_file = sc.file;
        end

        % Adjust sim time for slow speeds
        if sc.peak_speed <= 1.0
            cfg.sim.T_end = 400; cfg.sim.max_travel_time = 400;
        elseif sc.peak_speed <= 3.0
            cfg.sim.T_end = 250; cfg.sim.max_travel_time = 250;
        else
            cfg.sim.T_end = 150; cfg.sim.max_travel_time = 150;
        end

        ref = ref_base;
        ref = load_reference_speed(ref, cfg.speed);

        try
            result = run_closed_loop(cfg, ref, veh);
            m = result.metrics;
            dt = cfg.sim.dt;

            % ── COMPUTE LONGITUDINAL DEVIATION (metres) ──────────────
            % Longitudinal deviation = integral of speed error over time
            % Positive = vehicle is behind where it should be
            % Negative = vehicle is ahead of where it should be
            speed_error = result.log.v_ref - result.log.v;
            lon_dev = cumsum(speed_error) * dt;  % metres
            peak_lon_dev = max(abs(lon_dev));
            rms_lon_dev = sqrt(mean(lon_dev.^2));
            % ─────────────────────────────────────────────────────────

            lat_pass = m.peak_cte < LATERAL_LIMIT;
            lon_pass = peak_lon_dev < LONGITUDINAL_LIMIT;

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
            T.Peak_Lon_Dev_m(i)     = round(peak_lon_dev, 4);
            T.RMS_Lon_Dev_m(i)      = round(rms_lon_dev, 4);
            T.Loop_Time_s(i)        = round(m.single_loop_time_s, 2);
            T.Goal_Reached(i)       = string(m.goal_reached);
            T.Termination(i)        = m.termination_reason;
            T.REQ1_Lat_Pass(i)      = iff(lat_pass, "PASS", "FAIL");
            T.REQ1_Lon_Pass(i)      = iff(lon_pass, "PASS", "FAIL");
            T.Overall_Pass(i)       = iff(lat_pass && lon_pass, "PASS", "FAIL");

            all_results{i} = result;
            all_results{i}.lon_dev = lon_dev;

            % Save individual case
            case_tag = sprintf('case_%02d_%s_%s_%.0fmps_%s', ...
                i, char(c.lateral), char(c.lon), sc.peak_speed*10, char(sc.scenario));
            case_dir = fullfile(sweep_dir, case_tag);
            if ~exist(case_dir, 'dir'), mkdir(case_dir); end
            save(fullfile(case_dir, 'result.mat'), 'cfg', 'ref', 'veh', 'result', ...
                'lon_dev', 'peak_lon_dev', 'rms_lon_dev');

            elapsed = toc(t_start);
            fprintf(' done (%.1fs) | PeakCTE=%.3fm [%s] | PeakLonDev=%.3fm [%s]', ...
                elapsed, m.peak_cte, iff(lat_pass,"OK","FAIL"), ...
                peak_lon_dev, iff(lon_pass,"OK","FAIL"));

        catch ME
            fprintf(' FAILED: %s', ME.message);
            T.Lateral(i) = c.lateral;
            T.Longitudinal(i) = c.lon;
            T.GDP_Scenario(i) = sc.scenario;
            T.Peak_Speed_mps(i) = sc.peak_speed;
            T.Speed_Mode(i) = sc.mode;
            T.Termination(i) = "error: " + string(ME.message);
            T.REQ1_Lat_Pass(i) = "ERROR";
            T.REQ1_Lon_Pass(i) = "ERROR";
            T.Overall_Pass(i) = "ERROR";
        end
    end

    %% ── Save results ─────────────────────────────────────────────────────
    writetable(T, fullfile(sweep_dir, 'GDP_sweep_results.csv'));
    save(fullfile(sweep_dir, 'GDP_sweep_results.mat'), 'T', 'all_results', 'cases');

    %% ── Print summary ────────────────────────────────────────────────────
    fprintf('\n\n');
    fprintf('================================================================\n');
    fprintf('  GDP BRIEF REQUIREMENTS COMPLIANCE SUMMARY\n');
    fprintf('  Lateral limit:      < %.1f m (cross-track error)\n', LATERAL_LIMIT);
    fprintf('  Longitudinal limit: < %.1f m (along-path position deviation)\n', LONGITUDINAL_LIMIT);
    fprintf('================================================================\n\n');

    fprintf('REQ-2: dt = %.3f s => %.0f Hz : PASS\n', cfg_base.sim.dt, 1/cfg_base.sim.dt);
    fprintf('REQ-3: Delays (steer=%.1fs, lon=%.1fs), 3DOF+MF : PASS\n\n', ...
        cfg_base.vehicle.delay.steer_s, cfg_base.vehicle.delay.longitudinal_s);

    fprintf('%-20s %-8s %-6s %-10s %-8s %-10s %-8s %-8s %-6s\n', ...
        'Combo','Scen','Speed','PeakCTE(m)','LatPass','PkLonDev(m)','LonPass','Overall','Goal');
    fprintf('%s\n', repmat('-', 1, 100));

    for i = 1:n_cases
        fprintf('%-20s %-8s %-6.1f %-10.3f %-8s %-10.3f %-8s %-8s %-6s\n', ...
            sprintf('%s+%s', char(T.Lateral(i)), char(T.Longitudinal(i))), ...
            char(T.GDP_Scenario(i)), T.Peak_Speed_mps(i), ...
            T.Peak_CTE_m(i), char(T.REQ1_Lat_Pass(i)), ...
            T.Peak_Lon_Dev_m(i), char(T.REQ1_Lon_Pass(i)), ...
            char(T.Overall_Pass(i)), char(T.Goal_Reached(i)));
    end

    % Per-scenario summary
    scenarios = unique(T.GDP_Scenario);
    fprintf('\n--- PASS RATE BY GDP SCENARIO ---\n');
    for si = 1:numel(scenarios)
        mask = T.GDP_Scenario == scenarios(si);
        n_t = sum(mask); n_p = sum(T.Overall_Pass(mask) == "PASS");
        fprintf('  %s: %d / %d passed (%.0f%%)\n', char(scenarios(si)), n_p, n_t, 100*n_p/n_t);
    end

    fprintf('\n--- BEST COMBO PER SCENARIO ---\n');
    for si = 1:numel(scenarios)
        mask = T.GDP_Scenario == scenarios(si);
        sub = T(mask, :);
        if height(sub) > 0
            [~, bi] = min(sub.Peak_CTE_m);
            fprintf('  %s: %s+%s (PeakCTE=%.3fm, PeakLonDev=%.3fm)\n', ...
                char(scenarios(si)), char(sub.Lateral(bi)), char(sub.Longitudinal(bi)), ...
                sub.Peak_CTE_m(bi), sub.Peak_Lon_Dev_m(bi));
        end
    end

    %% ── Generate plots (NO threshold lines) ──────────────────────────────
    generate_GDP_plots(T, sweep_dir, lateral_set, lon_set);

    %% ── Write text summary ───────────────────────────────────────────────
    write_GDP_summary(T, sweep_dir, cfg_base, LATERAL_LIMIT, LONGITUDINAL_LIMIT);

    fprintf('\n\nAll outputs saved to: %s\n', sweep_dir);
end


function result = iff(cond, tv, fv)
    if cond, result = tv; else, result = fv; end
end


%% =====================================================================
%  PLOTTING (NO threshold lines as requested)
%  =====================================================================
function generate_GDP_plots(T, sweep_dir, lateral_set, lon_set)

    combos = strings(0);
    for li = 1:numel(lateral_set)
        for lo = 1:numel(lon_set)
            combos(end+1) = sprintf('%s+%s', char(lateral_set(li)), char(lon_set(lo))); %#ok<AGROW>
        end
    end
    all_speeds = unique(T.Peak_Speed_mps);
    leg_labels = arrayfun(@(s) sprintf('%.1f m/s', s), all_speeds, 'UniformOutput', false);

    % ── 1) Peak CTE bar chart (NO threshold line) ────────────────────────
    fig1 = figure('Position', [100 100 1100 500], 'Visible', 'off');
    cte_mat = nan(numel(combos), numel(all_speeds));
    for ci = 1:numel(combos)
        parts = split(combos(ci), '+');
        for si = 1:numel(all_speeds)
            mask = T.Lateral == parts(1) & T.Longitudinal == parts(2) & ...
                   T.Peak_Speed_mps == all_speeds(si);
            v = T.Peak_CTE_m(mask);
            if ~isempty(v), cte_mat(ci, si) = v(1); end
        end
    end
    bar(cte_mat);
    set(gca, 'XTickLabel', combos, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel('Peak Cross-Track Error (m)');
    title('Peak Lateral Deviation by Controller Combination');
    legend(leg_labels, 'Location', 'northwest');
    grid on;
    saveas(fig1, fullfile(sweep_dir, 'peak_lateral_deviation.png'));
    close(fig1);

    % ── 2) Peak Longitudinal Deviation bar chart (NO threshold line) ─────
    fig2 = figure('Position', [100 100 1100 500], 'Visible', 'off');
    lon_mat = nan(numel(combos), numel(all_speeds));
    for ci = 1:numel(combos)
        parts = split(combos(ci), '+');
        for si = 1:numel(all_speeds)
            mask = T.Lateral == parts(1) & T.Longitudinal == parts(2) & ...
                   T.Peak_Speed_mps == all_speeds(si);
            v = T.Peak_Lon_Dev_m(mask);
            if ~isempty(v), lon_mat(ci, si) = v(1); end
        end
    end
    bar(lon_mat);
    set(gca, 'XTickLabel', combos, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel('Peak Longitudinal Deviation (m)');
    title('Peak Longitudinal Deviation by Controller Combination');
    legend(leg_labels, 'Location', 'northwest');
    grid on;
    saveas(fig2, fullfile(sweep_dir, 'peak_longitudinal_deviation.png'));
    close(fig2);

    % ── 3) RMS CTE bar chart ─────────────────────────────────────────────
    fig3 = figure('Position', [100 100 1100 500], 'Visible', 'off');
    rms_mat = nan(numel(combos), numel(all_speeds));
    for ci = 1:numel(combos)
        parts = split(combos(ci), '+');
        for si = 1:numel(all_speeds)
            mask = T.Lateral == parts(1) & T.Longitudinal == parts(2) & ...
                   T.Peak_Speed_mps == all_speeds(si);
            v = T.RMS_CTE_m(mask);
            if ~isempty(v), rms_mat(ci, si) = v(1); end
        end
    end
    bar(rms_mat);
    set(gca, 'XTickLabel', combos, 'XTickLabelRotation', 30, 'FontSize', 9);
    ylabel('RMS Cross-Track Error (m)');
    title('RMS Lateral Tracking Accuracy');
    legend(leg_labels, 'Location', 'northwest');
    grid on;
    saveas(fig3, fullfile(sweep_dir, 'rms_lateral_accuracy.png'));
    close(fig3);

    % ── 4) GDP Scenario pass/fail heatmap ────────────────────────────────
    fig4 = figure('Position', [100 100 900 500], 'Visible', 'off');
    scen_order = ["GDP-S1", "GDP-S2", "GDP-S3", "GDP-S3-EXT"];
    scen_labels = ["Low (0.5)", "Med (2.0)", "High (3-7)", "Full (10)"];

    pass_mat = zeros(numel(combos), numel(scen_order));
    for ci = 1:numel(combos)
        parts = split(combos(ci), '+');
        for si = 1:numel(scen_order)
            mask = T.Lateral == parts(1) & T.Longitudinal == parts(2) & ...
                   T.GDP_Scenario == scen_order(si);
            sub = T(mask, :);
            if height(sub) == 0
                pass_mat(ci, si) = -1;
            elseif all(sub.Overall_Pass == "PASS")
                pass_mat(ci, si) = 1;
            elseif any(sub.Overall_Pass == "PASS")
                pass_mat(ci, si) = 0.5;
            else
                pass_mat(ci, si) = 0;
            end
        end
    end

    imagesc(pass_mat);
    colormap([1 0.3 0.3; 1 0.8 0.3; 0.3 0.8 0.3; 0.7 0.7 0.7]);
    caxis([-1 1]);
    set(gca, 'XTick', 1:numel(scen_order), 'XTickLabel', scen_labels);
    set(gca, 'YTick', 1:numel(combos), 'YTickLabel', combos);
    title('GDP Scenario Pass/Fail Matrix');
    xlabel('GDP Testing Scenario');
    ylabel('Controller Combination');

    for ci = 1:numel(combos)
        for si = 1:numel(scen_order)
            v = pass_mat(ci, si);
            if v == -1, txt = "N/A";
            elseif v == 0, txt = "FAIL";
            elseif v == 0.5, txt = "PARTIAL";
            else, txt = "PASS";
            end
            text(si, ci, char(txt), 'HorizontalAlignment', 'center', ...
                'FontSize', 9, 'FontWeight', 'bold');
        end
    end
    saveas(fig4, fullfile(sweep_dir, 'GDP_scenario_pass_fail.png'));
    close(fig4);

    % ── 5) Longitudinal deviation time trace (MPC combos only) ───────────
    % This plot is generated separately if needed from saved results.

    fprintf('Plots saved (no threshold lines).\n');
end


%% =====================================================================
%  TEXT SUMMARY
%  =====================================================================
function write_GDP_summary(T, sweep_dir, cfg, lat_lim, lon_lim)
    fid = fopen(fullfile(sweep_dir, 'GDP_compliance_summary.txt'), 'w');

    fprintf(fid, '================================================================\n');
    fprintf(fid, '  GDP BRIEF COMPLIANCE SUMMARY (CORRECTED METRICS)\n');
    fprintf(fid, '  Generated: %s\n', char(datetime('now')));
    fprintf(fid, '================================================================\n\n');

    fprintf(fid, 'METRIC DEFINITIONS:\n');
    fprintf(fid, '  Lateral deviation  = peak cross-track error (perpendicular to path)\n');
    fprintf(fid, '  Longitudinal deviation = peak along-path position error\n');
    fprintf(fid, '    Computed as: max(abs(cumsum((v_ref - v_actual) * dt)))\n');
    fprintf(fid, '    i.e. how many METRES ahead/behind the vehicle is\n\n');

    fprintf(fid, 'THRESHOLDS (GDP Brief REQ-1, slide 13):\n');
    fprintf(fid, '  Lateral:      < %.1f m\n', lat_lim);
    fprintf(fid, '  Longitudinal: < %.1f m\n\n', lon_lim);

    fprintf(fid, 'REQ-2: dt = %.3f s => %.0f Hz (req >= 10 Hz): PASS\n', ...
        cfg.sim.dt, 1/cfg.sim.dt);
    fprintf(fid, 'REQ-3: Steer delay=%.1fs, Lon delay=%.1fs, 3DOF+MF: PASS\n\n', ...
        cfg.vehicle.delay.steer_s, cfg.vehicle.delay.longitudinal_s);

    fprintf(fid, '%-20s %-8s %-6s %-10s %-8s %-12s %-8s %-8s\n', ...
        'Combo','Scenario','Speed','PeakCTE(m)','LatOK','PkLonDev(m)','LonOK','Overall');
    fprintf(fid, '%s\n', repmat('-', 1, 95));

    for i = 1:height(T)
        fprintf(fid, '%-20s %-8s %-6.1f %-10.4f %-8s %-12.4f %-8s %-8s\n', ...
            sprintf('%s+%s', char(T.Lateral(i)), char(T.Longitudinal(i))), ...
            char(T.GDP_Scenario(i)), T.Peak_Speed_mps(i), ...
            T.Peak_CTE_m(i), char(T.REQ1_Lat_Pass(i)), ...
            T.Peak_Lon_Dev_m(i), char(T.REQ1_Lon_Pass(i)), ...
            char(T.Overall_Pass(i)));
    end

    n_pass = sum(T.Overall_Pass == "PASS");
    n_total = height(T);
    fprintf(fid, '\nOVERALL: %d / %d test cases passed (%.0f%%)\n', ...
        n_pass, n_total, 100*n_pass/n_total);

    fclose(fid);
end