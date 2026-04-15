function saved = save_sim_vs_bag_plot(log, run_dir, bag_mat_path, title_suffix)
% SAVE_SIM_VS_BAG_PLOT  Save a 2x2 simulation-vs-bag comparison figure.
%
%   saved = save_sim_vs_bag_plot(log, run_dir)
%   saved = save_sim_vs_bag_plot(log, run_dir, bag_mat_path, title_suffix)
%
% The plot compares:
%   - speed
%   - steering command
%   - throttle
%   - brake
%
% bag: red
% simulation: green

    if nargin < 3 || isempty(bag_mat_path)
        bag_mat_path = fullfile('data', 'bag_data_10hz.mat');
    end
    if nargin < 4
        title_suffix = '';
    end

    saved = false;
    if ~exist(bag_mat_path, 'file')
        fprintf('Bag comparison skipped: %s not found\n', bag_mat_path);
        return;
    end

    B = load(bag_mat_path);
    required = {'t', 'speed_mps', 'steer_road_rad', 'accel_pct', 'brake_pct'};
    for i = 1:numel(required)
        if ~isfield(B, required{i})
            fprintf('Bag comparison skipped: missing field %s in %s\n', required{i}, bag_mat_path);
            return;
        end
    end

    t_bag = B.t(:);
    t_sim = log.t(:);
    if isempty(t_bag) || isempty(t_sim)
        fprintf('Bag comparison skipped: empty time series\n');
        return;
    end

    t_start = max(t_bag(1), t_sim(1));
    t_end = min(t_bag(end), t_sim(end));
    if t_end <= t_start
        fprintf('Bag comparison skipped: no overlapping sim/bag time range\n');
        return;
    end

    if isfield(B, 'dt') && ~isempty(B.dt)
        dt_plot = double(B.dt);
    else
        dt_plot = median(diff(t_bag));
    end
    dt_plot = max(dt_plot, 1e-3);

    t_c = (t_start:dt_plot:t_end).';
    if numel(t_c) < 2
        fprintf('Bag comparison skipped: not enough overlapping samples\n');
        return;
    end

    speed_bag = interp1(t_bag, B.speed_mps(:), t_c, 'linear', 'extrap');
    steer_bag = interp1(t_bag, B.steer_road_rad(:), t_c, 'linear', 'extrap');
    throttle_bag = interp1(t_bag, B.accel_pct(:) / 100, t_c, 'linear', 'extrap');
    brake_bag = interp1(t_bag, B.brake_pct(:) / 100, t_c, 'linear', 'extrap');

    speed_sim = interp1(t_sim, log.v(:), t_c, 'linear', 'extrap');
    steer_sim = interp1(t_sim, log.delta_cmd_exec(:), t_c, 'linear', 'extrap');
    throttle_sim = interp1(t_sim, log.throttle(:), t_c, 'linear', 'extrap');
    brake_sim = interp1(t_sim, log.brake(:), t_c, 'linear', 'extrap');

    fig = figure('Color', 'w', 'Position', [100 100 1100 760], 'Visible', 'off');
    sim_color = [0.45 0.75 0.45];
    bag_color = [0.86 0.18 0.18];
    bag_marker_idx = 1:5:numel(t_c);

    subplot(2, 2, 1);
    plot(t_c, speed_sim, 'Color', sim_color, 'LineWidth', 1.0, 'DisplayName', 'Simulation');
    hold on;
    plot(t_c, speed_bag, 'Color', bag_color, 'LineWidth', 1.8, 'DisplayName', 'Bag');
    plot(t_c(bag_marker_idx), speed_bag(bag_marker_idx), '.', 'Color', bag_color, 'HandleVisibility', 'off');
    title('Speed');
    xlabel('Time [s]');
    ylabel('[m/s]');
    legend('Location', 'best');
    grid on;

    subplot(2, 2, 2);
    plot(t_c, -steer_sim, 'Color', sim_color, 'LineWidth', 1.0, 'DisplayName', 'Simulation');
    hold on;
    plot(t_c, steer_bag, 'Color', bag_color, 'LineWidth', 1.8, 'DisplayName', 'Bag');
    plot(t_c(bag_marker_idx), steer_bag(bag_marker_idx), '.', 'Color', bag_color, 'HandleVisibility', 'off');
    title('Steering Command');
    xlabel('Time [s]');
    ylabel('[rad]');
    legend('Location', 'best');
    grid on;

    subplot(2, 2, 3);
    plot(t_c, throttle_sim, 'Color', sim_color, 'LineWidth', 1.0, 'DisplayName', 'Simulation');
    hold on;
    plot(t_c, throttle_bag, 'Color', bag_color, 'LineWidth', 1.8, 'DisplayName', 'Bag');
    plot(t_c(bag_marker_idx), throttle_bag(bag_marker_idx), '.', 'Color', bag_color, 'HandleVisibility', 'off');
    title('Throttle');
    xlabel('Time [s]');
    ylabel('[0-1]');
    legend('Location', 'best');
    grid on;

    subplot(2, 2, 4);
    plot(t_c, brake_sim, 'Color', sim_color, 'LineWidth', 1.0, 'DisplayName', 'Simulation');
    hold on;
    plot(t_c, brake_bag, 'Color', bag_color, 'LineWidth', 1.8, 'DisplayName', 'Bag');
    plot(t_c(bag_marker_idx), brake_bag(bag_marker_idx), '.', 'Color', bag_color, 'HandleVisibility', 'off');
    title('Brake');
    xlabel('Time [s]');
    ylabel('[0-1]');
    legend('Location', 'best');
    grid on;

    if strlength(string(title_suffix)) > 0
        sgtitle(sprintf('Simulation vs Bag Comparison'));
    else
        sgtitle('Simulation vs Bag Comparison');
    end

    exportgraphics(fig, fullfile(run_dir, 'sim_vs_bag.png'), 'Resolution', 150);
    close(fig);
    saved = true;
end
