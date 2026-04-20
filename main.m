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
    fprintf('    Mean CTE             : %.4f m\n', m.rms_cte);
    fprintf('    Max CTE              : %.4f m\n', m.peak_cte);

    fprintf('  LONGITUDINAL:\n');
    fprintf('    Mean lon deviation   : %.4f m\n', m.rms_lon_dev);
    fprintf('    Max lon deviation    : %.4f m\n', m.peak_lon_dev);

    fprintf('  TIMING:\n');
    fprintf('    Control loop period  : %.3f s (%.0f Hz)\n', m.ctrl_loop_dt, m.ctrl_freq_hz);
    fprintf('    Mean exec time       : %.6f s (%.3f ms)\n', m.ctrl_exec_mean_s, m.ctrl_exec_mean_s*1000);
    fprintf('    Max exec time        : %.6f s (%.3f ms)\n', m.ctrl_exec_max_s, m.ctrl_exec_max_s*1000);
    fprintf('    Std exec time        : %.6f s\n', m.ctrl_exec_std_s);
    fprintf('    Loop time            : %.2f s\n', m.single_loop_time_s);
    fprintf('------------------------------------------------------------\n');
end

function save_result_plots(cfg, ref, result, run_dir, ctrl_name)
    log = result.log;
    m = result.metrics;
    dt = cfg.sim.dt;
    t_end_plot = get_plot_end_time(log, m);

    sp_err = log.v_ref - log.v;
    lon_dev = cumsum(sp_err) * dt;

    fig2 = figure('Position', [100 100 1000 700], 'Visible', 'off');

    subplot(2,2,1);
    plot(log.t, log.e_ct, 'b-', 'DisplayName', char(ctrl_name));
    xlabel('Time [s]'); ylabel('CTE [m]');
    title('Lateral Error'); legend; grid on;
    xlim([0, t_end_plot]);
    ann(gca, sprintf('RMS: %.4f m\nPeak: %.4f m', m.rms_cte, m.peak_cte));

    subplot(2,2,2);
    plot(log.t, rad2deg(log.e_psi), 'b-', 'DisplayName', char(ctrl_name));
    xlabel('Time [s]'); ylabel('[deg]');
    title('Heading Error'); legend; grid on;
    xlim([0, t_end_plot]);
    ann(gca, sprintf('RMS: %.3f deg', m.rms_epsi_deg));

    subplot(2,2,3);
    plot(log.t, lon_dev, 'b-', 'DisplayName', char(ctrl_name));
    xlabel('Time [s]'); ylabel('[m]');
    title('Longitudinal Deviation'); legend; grid on;
    xlim([0, t_end_plot]);
    ann(gca, sprintf('RMS: %.4f m\nPeak: %.4f m', m.rms_lon_dev, m.peak_lon_dev));

    subplot(2,2,4);
    plot(log.t, sp_err, 'b-', 'DisplayName', char(ctrl_name));
    xlabel('Time [s]'); ylabel('[m/s]');
    title('Speed Error'); legend; grid on;
    xlim([0, t_end_plot]);
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
    xlim([0, t_end_plot]);

    subplot(1,2,2);
    plot(log.t, log.ax_cmd, 'b-', 'DisplayName', sprintf('%s Cmd', char(ctrl_name)));
    hold on;
    plot(log.t, log.ax, 'r-', 'LineWidth', 0.8, 'DisplayName', 'Actual');
    xlabel('Time [s]'); ylabel('[m/s^2]');
    title('Acceleration'); legend; grid on;
    xlim([0, t_end_plot]);

    sgtitle(sprintf('Longitudinal (%s)', ctrl_name));
    saveas(fig3, fullfile(run_dir, 'speed_tracking.png')); close(fig3);

    

    fig4 = figure('Position', [100 100 1000 500], 'Visible', 'off');

    subplot(1,2,1);
    plot(log.t, rad2deg(log.delta_cmd_raw), 'b-', 'DisplayName', 'Commanded');
    hold on;
    plot(log.t, rad2deg(log.delta_cmd_exec), 'r-', 'DisplayName', 'Executed');
    xlabel('Time [s]'); ylabel('[deg]');
    title('Steering Angle'); legend; grid on;
    xlim([0, t_end_plot]);

    subplot(1,2,2);
    plot(log.t, log.vy, 'b-', 'DisplayName', 'v_y [m/s]');
    hold on;
    plot(log.t, log.r, 'r-', 'DisplayName', 'r [rad/s]');
    xlabel('Time [s]');
    title('Lateral States'); legend; grid on;
    xlim([0, t_end_plot]);

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
    xlim([0, t_end_plot]);
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
    xlim([0, t_end_plot]);

    subplot(4,1,2);
    plot(log.t, log.throttle, 'g-', 'LineWidth', 1.1, 'DisplayName', 'throttle\_pct');
    hold on;
    plot(log.t, log.brake, 'r-', 'LineWidth', 1.1, 'DisplayName', 'brake\_pct');
    xlabel('Time [s]'); ylabel('[0-1]');
    title('Pedal Commands');
    legend('Location', 'best'); grid on;
    xlim([0, t_end_plot]);

    subplot(4,1,3);
    plot(log.t, log.ACC_req, 'g-', 'LineWidth', 1.1, 'DisplayName', 'ACC\_req');
    hold on;
    plot(log.t, log.BRK_req, 'r-', 'LineWidth', 1.1, 'DisplayName', 'BRK\_req');
    xlabel('Time [s]'); ylabel('Map cmd');
    title('Lookup Requests');
    legend('Location', 'best'); grid on;
    xlim([0, t_end_plot]);

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
    xlim([0, t_end_plot]);

    sgtitle(sprintf('Longitudinal Internal Diagnostics (%s)', ctrl_name));
    saveas(fig6, fullfile(run_dir, 'longitudinal_internal_diagnostics.png')); close(fig6);

    fig7 = figure('Position', [100 100 1200 900], 'Visible', 'off');
    plot(log.t, log.v_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    hold on;
    plot(log.t, log.v, 'b-', 'LineWidth', 1.2, 'DisplayName', char(ctrl_name));
    xlabel('Time [s]'); ylabel('Speed [m/s]');
    title('Speed Tracking'); legend('Location', 'best'); grid on;
    xlim([0, t_end_plot]);
    saveas(fig7, fullfile(run_dir, 'onlyspeed.png')); close(fig7);


    save_poster_overview_plot(ref, log, run_dir, ctrl_name, t_end_plot);
    save_sim_vs_bag_plot(log, run_dir, fullfile('data', 'bag_data_10hz.mat'), ctrl_name);
end

function save_poster_overview_plot(ref, log, run_dir, ctrl_name, t_end_plot)
    c_ref = [0.08, 0.08, 0.08];
    c_path = [0.00, 0.36, 0.84];
    c_speed = [0.95, 0.45, 0.10];
    c_throttle = [0.03, 0.55, 0.24];
    c_brake = [0.78, 0.16, 0.18];
    c_delta_cmd = [0.31, 0.20, 0.60];
    grid_c = [0.88, 0.89, 0.91];

    fig = figure('Position', [80 80 1400 900], 'Visible', 'off', 'Color', 'w');
    tl = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

    ax1 = nexttile(tl, 1);

    hold(ax1, 'on');

    plot(ax1, ref.x, ref.y, '--', 'Color', c_ref, 'LineWidth', 2.2, ...
        'DisplayName', 'Reference');
    plot(ax1, log.x, log.y, '-', 'Color', c_path, 'LineWidth', 2.8, ...
        'DisplayName', 'NMPC + PID');
    plot(ax1, log.x(1), log.y(1), 'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', [0.00, 0.65, 0.20], ...
        'MarkerEdgeColor', [0.00, 0.45, 0.15], ...
        'LineWidth', 0.8, ...
        'DisplayName', 'Start');
    plot(ax1, log.x(end), log.y(end), 'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', [0.82, 0.14, 0.16], ...
        'MarkerEdgeColor', [0.60, 0.08, 0.10], ...
        'LineWidth', 0.8, ...
        'DisplayName', 'End');

    % plot(ax1, ref.x(1), ref.y(1), 'o', 'MarkerSize', 7, 'MarkerFaceColor', c_ref, ...
    %     'MarkerEdgeColor', c_ref, 'HandleVisibility', 'off');
    % plot(ax1, ref.x(end), ref.y(end), 's', 'MarkerSize', 7, 'MarkerFaceColor', c_path, ...
    %     'MarkerEdgeColor', c_path, 'HandleVisibility', 'off');

    axis(ax1, 'equal');
    grid(ax1, 'on');
    box(ax1, 'on');
    ax1.GridColor = grid_c;
    ax1.GridAlpha = 0.9;
    ax1.FontSize = 12;
    xlabel(ax1, 'X [m]', 'FontWeight', 'bold');
    ylabel(ax1, 'Y [m]', 'FontWeight', 'bold');
    title(ax1, 'Path Tracking', 'FontWeight', 'bold', 'FontSize', 16);
    legend(ax1, 'Location', 'best', 'Box', 'off', 'FontSize', 11);

    ax2 = nexttile(tl, 2);
    plot(ax2, log.t, log.v_ref, '--', 'Color', c_ref, 'LineWidth', 2.0, ...
        'DisplayName', 'Reference speed');
    hold(ax2, 'on');
    plot(ax2, log.t, log.v, '-', 'Color', c_speed, 'LineWidth', 2.8, ...
        'DisplayName', 'Vehicle speed');
    grid(ax2, 'on');
    box(ax2, 'on');
    ax2.GridColor = grid_c;
    ax2.GridAlpha = 0.9;
    ax2.FontSize = 12;
    xlabel(ax2, 'Time [s]', 'FontWeight', 'bold');
    ylabel(ax2, 'Speed [m/s]', 'FontWeight', 'bold');
    title(ax2, 'Speed Tracking', 'FontWeight', 'bold', 'FontSize', 16);
    legend(ax2, 'Location', 'best', 'Box', 'off', 'FontSize', 11);
    xlim(ax2, [0, t_end_plot]);

    ax3 = nexttile(tl, 3);
    plot(ax3, log.t, 100 * log.throttle, '-', 'Color', c_throttle, 'LineWidth', 2.6, ...
        'DisplayName', 'Throttle');
    hold(ax3, 'on');
    plot(ax3, log.t, 100 * log.brake, '-', 'Color', c_brake, 'LineWidth', 2.6, ...
        'DisplayName', 'Brake');
    ylim(ax3, [0, max(5, 1.08 * max([100 * log.throttle; 100 * log.brake; 1]))]);
    grid(ax3, 'on');
    box(ax3, 'on');
    ax3.GridColor = grid_c;
    ax3.GridAlpha = 0.9;
    ax3.FontSize = 12;
    xlabel(ax3, 'Time [s]', 'FontWeight', 'bold');
    ylabel(ax3, 'Pedal Command [%]', 'FontWeight', 'bold');
    title(ax3, 'Throttle / Brake Command', 'FontWeight', 'bold', 'FontSize', 16);
    legend(ax3, 'Location', 'best', 'Box', 'off', 'FontSize', 11);
    xlim(ax3, [0, t_end_plot]);

    ax4 = nexttile(tl, 4);
    plot(ax4, log.t, rad2deg(log.delta_cmd_raw), '-', 'Color', c_delta_cmd, 'LineWidth', 2.2, ...
        'DisplayName', 'Steering cmd');
    yline(ax4, 0, ':', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.0, 'HandleVisibility', 'off');
    grid(ax4, 'on');
    box(ax4, 'on');
    ax4.GridColor = grid_c;
    ax4.GridAlpha = 0.9;
    ax4.FontSize = 12;
    xlabel(ax4, 'Time [s]', 'FontWeight', 'bold');
    ylabel(ax4, 'Steering [deg]', 'FontWeight', 'bold');
    title(ax4, 'Steering Command', 'FontWeight', 'bold', 'FontSize', 16);
    legend(ax4, 'Location', 'best', 'Box', 'off', 'FontSize', 11);
    xlim(ax4, [0, t_end_plot]);

    title(tl, sprintf('Controller Overview'), ...
        'FontWeight', 'bold', 'FontSize', 20);

    exportgraphics(fig, fullfile(run_dir, 'poster_overview.png'), 'Resolution', 220);
    close(fig);
end

function t_end_plot = get_plot_end_time(log, m)
    t_end_plot = 0.0;

    if isfield(log, 't') && ~isempty(log.t)
        t_end_plot = log.t(end);
    end

    if isfield(m, 'single_loop_time_s') && ~isempty(m.single_loop_time_s)
        t_end_plot = min(max(t_end_plot, 1e-6), m.single_loop_time_s);
    else
        t_end_plot = max(t_end_plot, 1e-6);
    end
end

function write_summary_file(result, run_dir, ctrl_name)
    m = result.metrics;
    fid = fopen(fullfile(run_dir, 'summary.txt'), 'w');

    fprintf(fid, 'Controller: %s\n\n', ctrl_name);
    fprintf(fid, 'LATERAL:\n');
    fprintf(fid, '  Mean CTE           : %.4f m\n', m.rms_cte);
    fprintf(fid, '  Max CTE            : %.4f m\n\n', m.peak_cte);

    fprintf(fid, 'LONGITUDINAL:\n');
    fprintf(fid, '  Mean lon deviation : %.4f m\n', m.rms_lon_dev);
    fprintf(fid, '  Max lon deviation  : %.4f m\n\n', m.peak_lon_dev);

    fprintf(fid, 'TIMING:\n');
    fprintf(fid, '  Control loop       : %.3f s (%.0f Hz)\n', m.ctrl_loop_dt, m.ctrl_freq_hz);
    fprintf(fid, '  Mean exec time     : %.6f s (%.3f ms)\n', m.ctrl_exec_mean_s, m.ctrl_exec_mean_s*1000);
    fprintf(fid, '  Max exec time      : %.6f s (%.3f ms)\n', m.ctrl_exec_max_s, m.ctrl_exec_max_s*1000);
    fprintf(fid, '  Std exec time      : %.6f s\n', m.ctrl_exec_std_s);
    fprintf(fid, '  Loop time          : %.2f s\n', m.single_loop_time_s);
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
