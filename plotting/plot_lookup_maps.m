function plot_lookup_maps()
    project_root = fileparts(fileparts(mfilename('fullpath')));
    addpath(fullfile(project_root, 'model'));

    data_dir = fullfile(project_root, 'data');
    accel_map_file = fullfile(data_dir, 'Acc_mapData_noSlope.mat');
    brake_map_file = fullfile(data_dir, 'brake_mapData_noSlope.mat');

    veh = load_vehicle_params(accel_map_file, brake_map_file);

    acc_raw_cmd = veh.acc.cmd_samples(:);
    acc_raw_force = veh.acc.force_samples(:);
    brk_raw_cmd = veh.brk.cmd_samples(:);
    brk_raw_force = veh.brk.force_samples(:);

    acc_cmd_1d = veh.acc.cmd_full(:);
    acc_force_1d = veh.acc.force_full(:);
    brk_cmd_1d = veh.brk.cmd_full(:);
    brk_force_1d = veh.brk.force_full(:);

    acc_pedal = (acc_cmd_1d / max(acc_cmd_1d)) * veh.max_pedal_publish;
    brk_pedal = (brk_cmd_1d / max(brk_cmd_1d)) * veh.max_pedal_publish;

    fig1 = figure('Position', [100 100 1200 900], 'Visible', 'off');

    subplot(2,2,1);
    scatter(acc_raw_cmd, acc_raw_force, 12, [0.75 0.75 0.75], 'filled', ...
        'DisplayName', 'Raw samples');
    hold on;
    plot(acc_cmd_1d, acc_force_1d, 'g-', 'LineWidth', 1.5, ...
        'DisplayName', '1D lookup');
    xlabel('ACC command');
    ylabel('Force [N]');
    title('Acceleration Map');
    legend('Location', 'best');
    grid on;

    subplot(2,2,2);
    scatter(brk_raw_cmd, brk_raw_force, 12, [0.75 0.75 0.75], 'filled', ...
        'DisplayName', 'Raw samples');
    hold on;
    plot(brk_cmd_1d, brk_force_1d, 'r-', 'LineWidth', 1.5, ...
        'DisplayName', '1D lookup');
    xlabel('Brake command');
    ylabel('Force [N]');
    title('Brake Map');
    legend('Location', 'best');
    grid on;

    subplot(2,2,3);
    plot(acc_pedal, acc_force_1d, 'g-', 'LineWidth', 1.5, ...
        'DisplayName', 'Throttle publish -> force');
    hold on;
    plot(brk_pedal, brk_force_1d, 'r-', 'LineWidth', 1.5, ...
        'DisplayName', 'Brake publish -> force');
    xlabel('Published pedal [0-0.6]');
    ylabel('Force magnitude [N]');
    title('Published Pedal to Force');
    legend('Location', 'best');
    grid on;

    subplot(2,2,4);
    stem(acc_pedal, acc_force_1d, 'g', 'filled', 'DisplayName', 'Accel low end');
    hold on;
    stem(brk_pedal, -brk_force_1d, 'r', 'filled', 'DisplayName', 'Brake low end');
    yline(0, 'k--');
    xlabel('Published pedal [0-0.6]');
    ylabel('Signed force [N]');
    title('Discrete Force Levels');
    legend('Location', 'best');
    grid on;

    sgtitle('Longitudinal Lookup Maps');
    exportgraphics(fig1, fullfile(data_dir, 'lookup_map_overview.png'), 'Resolution', 150);
    close(fig1);

    fig2 = figure('Position', [100 100 1100 500], 'Visible', 'off');

    subplot(1,2,1);
    stem(acc_pedal, acc_force_1d, 'g', 'filled', 'DisplayName', 'Accel 1D lookup');
    hold on;
    yline(0, 'k--');
    ylim([0 1200]);
    xlabel('Published throttle [0-0.6]');
    ylabel('Force [N]');
    title('Acceleration Map Low-Force Zoom');
    legend('Location', 'best');
    grid on;

    subplot(1,2,2);
    stem(brk_pedal, brk_force_1d, 'r', 'filled', 'DisplayName', 'Brake 1D lookup');
    hold on;
    yline(0, 'k--');
    ylim([0 1200]);
    xlabel('Published brake [0-0.6]');
    ylabel('Force [N]');
    title('Brake Map Low-Force Zoom');
    legend('Location', 'best');
    grid on;

    sgtitle('Lookup Map Low-End Resolution');
    exportgraphics(fig2, fullfile(data_dir, 'lookup_map_low_force_zoom.png'), 'Resolution', 150);
    close(fig2);

    save(fullfile(data_dir, 'lookup_map_processed_1d.mat'), ...
        'acc_raw_cmd', 'acc_raw_force', 'brk_raw_cmd', 'brk_raw_force', ...
        'acc_cmd_1d', 'acc_force_1d', 'brk_cmd_1d', 'brk_force_1d', ...
        'acc_pedal', 'brk_pedal');

    fprintf('Saved lookup map plots to:\n');
    fprintf('  %s\n', fullfile(data_dir, 'lookup_map_overview.png'));
    fprintf('  %s\n', fullfile(data_dir, 'lookup_map_low_force_zoom.png'));
    fprintf('  %s\n', fullfile(data_dir, 'lookup_map_processed_1d.mat'));
end
