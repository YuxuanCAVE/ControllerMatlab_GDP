function comparison = compare_lateral_mpc_variants()
% COMPARE_LATERAL_MPC_VARIANTS  Compare mpc_kinematic vs mpc with LQR longitudinal.
%
% This script runs two cases:
%   1) mpc_kinematic + lqr
%   2) mpc + lqr
%
% Outputs:
%   run/compare_run/<timestamp>_lateral_mpc_compare/
%       mpc_kinematic_lqr/
%       mpc_lqr/
%       comparison.mat
%       comparison_summary.txt
%       lateral_controller_comparison.png

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

    cfg_base = default_config();
    cfg_base.controller.longitudinal = "lqr";

    ref = load_reference_path(cfg_base.ref.path_file);
    ref = load_reference_speed(ref, cfg_base.speed);

    veh = load_vehicle_params(cfg_base.vehicle.accel_map_file, cfg_base.vehicle.brake_map_file);
    veh.max_steer = cfg_base.vehicle.max_steer;
    veh.max_steer_rate = cfg_base.vehicle.max_steer_rate;

    cases = {
        struct('lateral', "mpc_kinematic", 'label', "MPC Kinematic + LQR", 'tag', "mpc_kinematic_lqr")
        struct('lateral', "mpc", 'label', "MPC Dynamic + LQR", 'tag', "mpc_lqr")
    };

    stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));
    compare_root = fullfile(project_root, cfg_base.run.root_dir, 'compare_run');
    if ~exist(compare_root, 'dir')
        mkdir(compare_root);
    end
    run_dir = fullfile(compare_root, sprintf('%s_lateral_mpc_compare', stamp));
    if ~exist(run_dir, 'dir')
        mkdir(run_dir);
    end

    fprintf('\n============================================================\n');
    fprintf('  Comparing lateral MPC controllers with LQR longitudinal\n');
    fprintf('============================================================\n');

    comparison.cfg = cfg_base;
    comparison.ref = ref;
    comparison.veh = veh;
    comparison.results = struct();
    comparison.case_labels = strings(numel(cases), 1);

    for i = 1:numel(cases)
        cfg_case = cfg_base;
        cfg_case.controller.lateral = cases{i}.lateral;
        cfg_case.controller.longitudinal = "lqr";

        case_dir = fullfile(run_dir, cases{i}.tag);
        if ~exist(case_dir, 'dir')
            mkdir(case_dir);
        end

        fprintf('Running: %s ...\n', cases{i}.label);
        result = run_closed_loop(cfg_case, ref, veh);

        save_run_plots(cfg_case, ref, result, case_dir);
        save_sim_vs_bag_plot(result.log, case_dir, fullfile('data', 'bag_data_10hz.mat'), cases{i}.label);
        save(fullfile(case_dir, 'result.mat'), 'cfg_case', 'ref', 'veh', 'result');
        write_case_summary(result, case_dir, cases{i}.label);
        print_case_summary(result, cases{i}.label);

        comparison.results.(cases{i}.tag) = result;
        comparison.case_labels(i) = string(cases{i}.label);
    end

    save(fullfile(run_dir, 'comparison.mat'), 'comparison');
    write_comparison_summary(comparison, run_dir, cases);
    save_lateral_comparison_plot(comparison, run_dir, cases);

    fprintf('\nSaved comparison to: %s\n', run_dir);
end

function print_case_summary(result, ctrl_name)
    m = result.metrics;
    fprintf('  Peak CTE: %.4f m | RMS CTE: %.4f m | RMS heading: %.3f deg | Peak lon dev: %.4f m\n', ...
        m.peak_cte, m.rms_cte, m.rms_epsi_deg, m.peak_lon_dev);
end

function write_case_summary(result, run_dir, ctrl_name)
    m = result.metrics;
    fid = fopen(fullfile(run_dir, 'summary.txt'), 'w');
    fprintf(fid, 'Controller: %s\n\n', ctrl_name);
    fprintf(fid, 'LATERAL\n');
    fprintf(fid, '  RMS CTE            : %.4f m\n', m.rms_cte);
    fprintf(fid, '  Peak CTE           : %.4f m\n', m.peak_cte);
    fprintf(fid, '  RMS heading error  : %.3f deg\n', m.rms_epsi_deg);
    fprintf(fid, '  REQ-1 lateral      : %s\n\n', iff(m.peak_cte < 0.6, 'PASS', 'FAIL'));
    fprintf(fid, 'LONGITUDINAL\n');
    fprintf(fid, '  RMS speed error    : %.4f m/s\n', m.rms_speed_error);
    fprintf(fid, '  Peak speed error   : %.4f m/s\n', m.peak_speed_error);
    fprintf(fid, '  Peak lon deviation : %.4f m\n', m.peak_lon_dev);
    fprintf(fid, '  RMS lon deviation  : %.4f m\n', m.rms_lon_dev);
    fprintf(fid, '  REQ-1 longit       : %s\n\n', iff(m.peak_lon_dev < 0.8, 'PASS', 'FAIL'));
    fprintf(fid, 'TIMING\n');
    fprintf(fid, '  Control loop       : %.3f s (%.0f Hz)\n', m.ctrl_loop_dt, m.ctrl_freq_hz);
    fprintf(fid, '  Mean exec time     : %.6f s\n', m.ctrl_exec_mean_s);
    fprintf(fid, '  Max exec time      : %.6f s\n', m.ctrl_exec_max_s);
    fprintf(fid, '  Real-time OK       : %s\n\n', iff(m.ctrl_realtime_ok, 'YES', 'NO'));
    fprintf(fid, 'GENERAL\n');
    fprintf(fid, '  Loop time          : %.2f s\n', m.single_loop_time_s);
    fprintf(fid, '  Goal reached       : %d\n', m.goal_reached);
    fprintf(fid, '  Termination        : %s\n', char(m.termination_reason));
    fclose(fid);
end

function write_comparison_summary(comparison, run_dir, cases)
    fid = fopen(fullfile(run_dir, 'comparison_summary.txt'), 'w');
    fprintf(fid, 'Lateral MPC comparison with LQR longitudinal\n\n');
    fprintf(fid, '%-24s %-10s %-10s %-12s %-12s %-12s %-10s %-10s\n', ...
        'Controller', 'RMS_CTE', 'Peak_CTE', 'RMS_Epsi', 'RMS_SpdE', 'PkLonDev', 'Mean_ms', 'Max_ms');

    for i = 1:numel(cases)
        m = comparison.results.(cases{i}.tag).metrics;
        fprintf(fid, '%-24s %-10.4f %-10.4f %-12.3f %-12.4f %-12.4f %-10.3f %-10.3f\n', ...
            cases{i}.label, ...
            m.rms_cte, m.peak_cte, m.rms_epsi_deg, m.rms_speed_error, m.peak_lon_dev, ...
            m.ctrl_exec_mean_s * 1000, m.ctrl_exec_max_s * 1000);
    end

    fprintf(fid, '\nWinner by metric (lower is better)\n');
    report_winner(fid, comparison, cases, 'rms_cte', 'RMS CTE');
    report_winner(fid, comparison, cases, 'peak_cte', 'Peak CTE');
    report_winner(fid, comparison, cases, 'rms_epsi_deg', 'RMS heading error');
    report_winner(fid, comparison, cases, 'rms_speed_error', 'RMS speed error');
    report_winner(fid, comparison, cases, 'peak_lon_dev', 'Peak longitudinal deviation');
    report_winner(fid, comparison, cases, 'ctrl_exec_mean_s', 'Mean execution time');
    report_winner(fid, comparison, cases, 'ctrl_exec_max_s', 'Max execution time');
    fclose(fid);
end

function report_winner(fid, comparison, cases, field_name, label)
    vals = zeros(numel(cases), 1);
    for i = 1:numel(cases)
        vals(i) = comparison.results.(cases{i}.tag).metrics.(field_name);
    end
    [best, idx] = min(vals);
    fprintf(fid, '  %-24s : %s (%.6f)\n', label, cases{idx}.label, best);
end

function save_lateral_comparison_plot(comparison, run_dir, cases)
    labels = strings(numel(cases), 1);
    rms_cte = zeros(numel(cases), 1);
    peak_cte = zeros(numel(cases), 1);
    rms_epsi = zeros(numel(cases), 1);
    peak_lon = zeros(numel(cases), 1);

    for i = 1:numel(cases)
        m = comparison.results.(cases{i}.tag).metrics;
        labels(i) = string(cases{i}.label);
        rms_cte(i) = m.rms_cte;
        peak_cte(i) = m.peak_cte;
        rms_epsi(i) = m.rms_epsi_deg;
        peak_lon(i) = m.peak_lon_dev;
    end

    fig = figure('Color', 'w', 'Position', [100 100 1100 750], 'Visible', 'off');

    subplot(2, 2, 1);
    bar(categorical(labels), rms_cte);
    ylabel('m');
    title('RMS CTE');
    grid on;

    subplot(2, 2, 2);
    bar(categorical(labels), peak_cte);
    ylabel('m');
    title('Peak CTE');
    yline(0.6, 'r--', 'REQ-1 0.6 m', 'LineWidth', 1.2);
    grid on;

    subplot(2, 2, 3);
    bar(categorical(labels), rms_epsi);
    ylabel('deg');
    title('RMS Heading Error');
    grid on;

    subplot(2, 2, 4);
    bar(categorical(labels), peak_lon);
    ylabel('m');
    title('Peak Longitudinal Deviation');
    yline(0.8, 'r--', 'REQ-1 0.8 m', 'LineWidth', 1.2);
    grid on;

    sgtitle('Lateral MPC Controller Comparison (Longitudinal = LQR)');
    exportgraphics(fig, fullfile(run_dir, 'lateral_controller_comparison.png'), 'Resolution', 150);
    close(fig);
end

function r = iff(c, t, f)
    if c
        r = t;
    else
        r = f;
    end
end
