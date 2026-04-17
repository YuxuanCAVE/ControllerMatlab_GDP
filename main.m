clear; clc; close all;

project_root = fileparts(which('main'));
if isempty(project_root), project_root = pwd; end

addpath(fullfile(project_root, 'config'));
addpath(fullfile(project_root, 'reference'));
addpath(fullfile(project_root, 'controllers', 'lateral'));
addpath(fullfile(project_root, 'controllers', 'longitudinal'));
addpath(fullfile(project_root, 'model'));
addpath(fullfile(project_root, 'simulation'));
addpath(fullfile(project_root, 'plotting'));
addpath(fullfile(project_root, 'utils'));

cfg = default_config();
ref = load_reference_path(cfg.ref.path_file);
ref = load_reference_speed(ref, cfg.speed);
veh = load_vehicle_params(cfg.vehicle.accel_map_file, cfg.vehicle.brake_map_file);
veh.max_steer = cfg.vehicle.max_steer;
veh.max_steer_rate = cfg.vehicle.max_steer_rate;

ctrl_name = sprintf('%s + %s', upper(char(cfg.controller.lateral)), ...
                               upper(char(cfg.controller.longitudinal)));
ctrl_tag = sprintf('%s_%s', char(cfg.controller.lateral), ...
                          char(cfg.controller.longitudinal));

if cfg.speed.mode == "constant"
    speed_info = sprintf('Constant %.1f m/s', cfg.speed.constant_value);
else
    speed_info = sprintf('Profile (from %s)', cfg.speed.profile_file);
end

stamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss'));

fprintf('\n============================================================\n');
fprintf('  Running: %s\n', ctrl_name);
fprintf('  Speed:   %s\n', speed_info);
fprintf('============================================================\n\n');

single_run_root = fullfile(project_root, cfg.run.root_dir, 'single_run');
if ~exist(single_run_root, 'dir'), mkdir(single_run_root); end

run_name = sprintf('%s_%s', stamp, ctrl_tag);
run_dir = fullfile(single_run_root, run_name);
if ~exist(run_dir, 'dir'), mkdir(run_dir); end

result = run_closed_loop(cfg, ref, veh);
save_result_plots(cfg, ref, result, run_dir, ctrl_name);
save(fullfile(run_dir, 'result.mat'), 'cfg', 'ref', 'veh', 'result');
write_summary_file(result, run_dir, ctrl_name);
print_result(result, ctrl_name);

fprintf('\nSaved to: %s\n', run_dir);


function print_result(result, ctrl_name)
    m = result.metrics;

    fprintf('\n------------------------------------------------------------\n');
    fprintf('  Controller: %s\n', ctrl_name);
    fprintf('------------------------------------------------------------\n');

    fprintf('  LATERAL:\n');
    fprintf('    RMS CTE              : %.4f m\n', m.rms_cte);
    fprintf('    Peak CTE             : %.4f m\n', m.peak_cte);
    fprintf('    RMS heading error    : %.3f deg\n', m.rms_epsi_deg);
    fprintf('    REQ-1 lateral (<0.6m): %s\n', iff(m.peak_cte < 0.6, 'PASS', 'FAIL'));

    fprintf('  LONGITUDINAL:\n');
    fprintf('    RMS speed error      : %.4f m/s\n', m.rms_speed_error);
    fprintf('    Peak speed error     : %.4f m/s\n', m.peak_speed_error);
    fprintf('    Peak lon deviation   : %.4f m\n', m.peak_lon_dev);
    fprintf('    RMS lon deviation    : %.4f m\n', m.rms_lon_dev);
    fprintf('    REQ-1 longit (<0.8m) : %s\n', iff(m.peak_lon_dev < 0.8, 'PASS', 'FAIL'));

    fprintf('  TIMING:\n');
    fprintf('    Control loop period  : %.3f s (%.0f Hz)\n', m.ctrl_loop_dt, m.ctrl_freq_hz);
    fprintf('    Mean exec time       : %.6f s (%.3f ms)\n', m.ctrl_exec_mean_s, m.ctrl_exec_mean_s*1000);
    fprintf('    Max exec time        : %.6f s (%.3f ms)\n', m.ctrl_exec_max_s, m.ctrl_exec_max_s*1000);
    fprintf('    Std exec time        : %.6f s\n', m.ctrl_exec_std_s);
    fprintf('    Real-time feasible   : %s (exec %.3f ms < loop %.1f ms)\n', ...
        iff(m.ctrl_realtime_ok, 'YES', 'NO'), m.ctrl_exec_max_s*1000, m.ctrl_loop_dt*1000);

    fprintf('  GENERAL:\n');
    fprintf('    Loop time            : %.2f s\n', m.single_loop_time_s);
    fprintf('    Goal reached         : %d\n', m.goal_reached);
    fprintf('    Termination          : %s\n', char(m.termination_reason));

    lat_ok = m.peak_cte < 0.6;
    lon_ok = m.peak_lon_dev < 0.8;
    if lat_ok && lon_ok
        fprintf('  >>> OVERALL: PASS <<<\n');
    else
        reasons = {};
        if ~lat_ok, reasons{end+1} = sprintf('lateral %.3f>0.6m', m.peak_cte); end
        if ~lon_ok, reasons{end+1} = sprintf('longitudinal %.3f>0.8m', m.peak_lon_dev); end
        fprintf('  >>> OVERALL: FAIL (%s) <<<\n', strjoin(reasons, ', '));
    end
    fprintf('------------------------------------------------------------\n');
end

function save_result_plots(cfg, ref, result, run_dir, ctrl_name)
    log = result.log;
    m = result.metrics;
    dt = cfg.sim.dt;

    sp_err = log.v_ref - log.v;
    lon_dev = cumsum(sp_err) * dt;

    fig1 = figure('Position', [100 100 800 600], 'Visible', 'off');
    plot(ref.x, ref.y, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(log.x, log.y, 'b-', 'LineWidth', 1.2, 'DisplayName', char(ctrl_name));
    xlabel('X [m]'); ylabel('Y [m]');
    title(sprintf('Path Tracking (%s)', ctrl_name));
    legend('Location', 'best'); grid on; axis equal;
    saveas(fig1, fullfile(run_dir, 'path_tracking.png')); close(fig1);

    fig2 = figure('Position', [100 100 1000 700], 'Visible', 'off');

    subplot(2,2,1);
    plot(log.t, log.e_ct, 'b-', 'DisplayName', char(ctrl_name));
    xlabel('Time [s]'); ylabel('CTE [m]');
    title('Lateral Error'); legend; grid on;
    ann(gca, sprintf('RMS: %.4f m\nPeak: %.4f m', m.rms_cte, m.peak_cte));

    subplot(2,2,2);
    plot(log.t, rad2deg(log.e_psi), 'b-', 'DisplayName', char(ctrl_name));
    xlabel('Time [s]'); ylabel('[deg]');
    title('Heading Error'); legend; grid on;
    ann(gca, sprintf('RMS: %.3f deg', m.rms_epsi_deg));

    subplot(2,2,3);
    plot(log.t, lon_dev, 'b-', 'DisplayName', char(ctrl_name));
    xlabel('Time [s]'); ylabel('[m]');
    title('Longitudinal Deviation'); legend; grid on;
    ann(gca, sprintf('RMS: %.4f m\nPeak: %.4f m', m.rms_lon_dev, m.peak_lon_dev));

    subplot(2,2,4);
    plot(log.t, sp_err, 'b-', 'DisplayName', char(ctrl_name));
    xlabel('Time [s]'); ylabel('[m/s]');
    title('Speed Error'); legend; grid on;
    ann(gca, sprintf('RMS: %.4f m/s\nPeak: %.4f m/s', m.rms_speed_error, m.peak_speed_error));

    sgtitle(sprintf('Tracking Errors (%s)', ctrl_name));
    saveas(fig2, fullfile(run_dir, 'tracking_errors.png')); close(fig2);

    fig3 = figure('Position', [100 100 1000 500], 'Visible', 'off');

    subplot(1,2,1);
    plot(log.t, log.v_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(log.t, log.v, 'b-', 'LineWidth', 1.2, 'DisplayName', char(ctrl_name));
    xlabel('Time [s]'); ylabel('Speed [m/s]');
    title('Speed Tracking'); legend('Location', 'best'); grid on;

    subplot(1,2,2);
    plot(log.t, log.ax_cmd, 'b-', 'DisplayName', sprintf('%s Cmd', char(ctrl_name)));
    hold on;
    plot(log.t, log.ax, 'r-', 'LineWidth', 0.8, 'DisplayName', 'Actual');
    xlabel('Time [s]'); ylabel('[m/s^2]');
    title('Acceleration'); legend; grid on;

    sgtitle(sprintf('Longitudinal (%s)', ctrl_name));
    saveas(fig3, fullfile(run_dir, 'speed_tracking.png')); close(fig3);

    fig4 = figure('Position', [100 100 1000 500], 'Visible', 'off');

    subplot(1,2,1);
    plot(log.t, rad2deg(log.delta_cmd_raw), 'b-', 'DisplayName', 'Commanded');
    hold on;
    plot(log.t, rad2deg(log.delta_cmd_exec), 'r-', 'DisplayName', 'Executed');
    xlabel('Time [s]'); ylabel('[deg]');
    title('Steering Angle'); legend; grid on;

    subplot(1,2,2);
    plot(log.t, log.vy, 'b-', 'DisplayName', 'v_y [m/s]');
    hold on;
    plot(log.t, log.r, 'r-', 'DisplayName', 'r [rad/s]');
    xlabel('Time [s]');
    title('Lateral States'); legend; grid on;

    sgtitle(sprintf('Lateral Dynamics (%s)', ctrl_name));
    saveas(fig4, fullfile(run_dir, 'lateral_dynamics.png')); close(fig4);

    fig5 = figure('Position', [100 100 800 400], 'Visible', 'off');
    plot(log.t, log.ctrl_exec_time_s * 1000, 'b-', 'DisplayName', char(ctrl_name));
    hold on;
    yline(m.ctrl_loop_dt * 1000, 'r--', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Loop period (%.0f ms)', m.ctrl_loop_dt*1000));
    xlabel('Time [s]'); ylabel('Execution Time [ms]');
    title(sprintf('Controller Execution Time (%s) - mean %.3f ms, max %.3f ms', ...
        ctrl_name, m.ctrl_exec_mean_s*1000, m.ctrl_exec_max_s*1000));
    legend('Location', 'best'); grid on;
    saveas(fig5, fullfile(run_dir, 'execution_timing.png')); close(fig5);

    fig6 = figure('Position', [100 100 1200 900], 'Visible', 'off');

    subplot(4,1,1);
    plot(log.t, log.F_required, 'k-', 'LineWidth', 1.2, 'DisplayName', 'F_{required}');
    hold on;
    plot(log.t, log.F_drive, 'b-', 'LineWidth', 1.0, 'DisplayName', 'F_{drive,actual}');
    yline(0, 'k--');
    xlabel('Time [s]'); ylabel('[N]');
    title('Force Balance Internals');
    legend('Location', 'best'); grid on;

    subplot(4,1,2);
    plot(log.t, log.throttle, 'g-', 'LineWidth', 1.1, 'DisplayName', 'throttle\_pct');
    hold on;
    plot(log.t, log.brake, 'r-', 'LineWidth', 1.1, 'DisplayName', 'brake\_pct');
    xlabel('Time [s]'); ylabel('[0-1]');
    title('Pedal Commands');
    legend('Location', 'best'); grid on;

    subplot(4,1,3);
    plot(log.t, log.ACC_req, 'g-', 'LineWidth', 1.1, 'DisplayName', 'ACC\_req');
    hold on;
    plot(log.t, log.BRK_req, 'r-', 'LineWidth', 1.1, 'DisplayName', 'BRK\_req');
    xlabel('Time [s]'); ylabel('Map cmd');
    title('Lookup Requests');
    legend('Location', 'best'); grid on;

    subplot(4,1,4);
    stairs(log.t, log.branch_mode, 'k-', 'LineWidth', 1.2, 'DisplayName', 'branch\_mode');
    hold on;
    stairs(log.t, log.branch_drive, 'g--', 'LineWidth', 1.0, 'DisplayName', 'drive');
    stairs(log.t, log.branch_coast, 'b--', 'LineWidth', 1.0, 'DisplayName', 'coast');
    stairs(log.t, log.branch_brake, 'r--', 'LineWidth', 1.0, 'DisplayName', 'brake');
    yline(0, 'k:');
    ylim([-1.2, 1.2]);
    yticks([-1 0 1]);
    yticklabels({'brake', 'coast', 'drive'});
    xlabel('Time [s]'); ylabel('Branch');
    title('Actuator Branch Selection');
    legend('Location', 'best'); grid on;

    sgtitle(sprintf('Longitudinal Internal Diagnostics (%s)', ctrl_name));
    saveas(fig6, fullfile(run_dir, 'longitudinal_internal_diagnostics.png')); close(fig6);

    save_sim_vs_bag_plot(log, run_dir, fullfile('data', 'bag_data_10hz.mat'), ctrl_name);
end

function write_summary_file(result, run_dir, ctrl_name)
    m = result.metrics;
    fid = fopen(fullfile(run_dir, 'summary.txt'), 'w');

    fprintf(fid, 'Controller: %s\n\n', ctrl_name);
    fprintf(fid, 'LATERAL:\n');
    fprintf(fid, '  RMS CTE            : %.4f m\n', m.rms_cte);
    fprintf(fid, '  Peak CTE           : %.4f m\n', m.peak_cte);
    fprintf(fid, '  RMS heading error  : %.3f deg\n', m.rms_epsi_deg);
    fprintf(fid, '  REQ-1 (<0.6m)      : %s\n\n', iff(m.peak_cte<0.6,'PASS','FAIL'));

    fprintf(fid, 'LONGITUDINAL:\n');
    fprintf(fid, '  RMS speed error    : %.4f m/s\n', m.rms_speed_error);
    fprintf(fid, '  Peak speed error   : %.4f m/s\n', m.peak_speed_error);
    fprintf(fid, '  Peak lon deviation : %.4f m\n', m.peak_lon_dev);
    fprintf(fid, '  RMS lon deviation  : %.4f m\n', m.rms_lon_dev);
    fprintf(fid, '  REQ-1 (<0.8m)      : %s\n\n', iff(m.peak_lon_dev<0.8,'PASS','FAIL'));

    fprintf(fid, 'TIMING:\n');
    fprintf(fid, '  Control loop       : %.3f s (%.0f Hz)\n', m.ctrl_loop_dt, m.ctrl_freq_hz);
    fprintf(fid, '  Mean exec time     : %.6f s (%.3f ms)\n', m.ctrl_exec_mean_s, m.ctrl_exec_mean_s*1000);
    fprintf(fid, '  Max exec time      : %.6f s (%.3f ms)\n', m.ctrl_exec_max_s, m.ctrl_exec_max_s*1000);
    fprintf(fid, '  Real-time OK       : %s\n\n', iff(m.ctrl_realtime_ok,'YES','NO'));

    fprintf(fid, 'GENERAL:\n');
    fprintf(fid, '  Loop time          : %.2f s\n', m.single_loop_time_s);
    fprintf(fid, '  Goal reached       : %d\n', m.goal_reached);
    fprintf(fid, '  Termination        : %s\n', char(m.termination_reason));
    fprintf(fid, '  Run directory      : %s\n', run_dir);
    fclose(fid);
end

function r = iff(c, t, f)
    if c, r = t; else, r = f; end
end

function ann(ax, txt)
    text(ax, 0.02, 0.95, txt, 'Units', 'normalized', ...
        'VerticalAlignment', 'top', 'FontSize', 9, ...
        'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.7 0.7 0.7]);
end
