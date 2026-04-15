function bag = read_rosbag_kia(bag_path, out_mat, out_png)
% READ_ROSBAG_KIA  Read and plot Kia Niro ROS2 bag signals.
%
%   bag = read_rosbag_kia()
%   bag = read_rosbag_kia(bag_path)
%   bag = read_rosbag_kia(bag_path, out_mat, out_png)
%
% Extracted signals:
%   /steering_feedback     -> steering wheel angle [deg], road wheel [rad]
%   /brake_pedal_feedback  -> brake pedal [%]
%   /acceleration_feedback -> throttle pedal [%]
%   /ins/odometry          -> x, y, yaw, speed
%
% The function resamples all signals to a common 10 Hz timebase, saves a
% MAT file under data/, and exports a summary plot.

    if nargin < 1 || isempty(bag_path)
        bag_path = default_bag_path();
    end
    if nargin < 2 || isempty(out_mat)
        out_mat = fullfile('data', 'bag_data_10hz.mat');
    end
    if nargin < 3 || isempty(out_png)
        out_png = fullfile('data', 'bag_plots.png');
    end

    topic_steer = '/steering_feedback';
    topic_brake = '/brake_pedal_feedback';
    topic_accel = '/acceleration_feedback';
    topic_odom = '/ins/odometry';

    steer_ratio = 14.5;
    steer_min_raw = -481;
    steer_max_raw = 486;
    brake_raw_max = 244;
    accel_raw_max = 255;
    dt_resamp = 0.1;

    if ~has_ros2_toolbox()
        error(['ROS Toolbox is required for direct bag reading in this workspace. ', ...
               'MATLAB does not provide ros2bag here.']);
    end

    bag_path = normalize_bag_path(bag_path);
    fprintf('Reading ROS2 bag: %s\n', bag_path);
    [t_steer, steer_raw] = extract_f32(bag_path, topic_steer);
    [t_brake, brake_raw] = extract_f32(bag_path, topic_brake);
    [t_accel, accel_raw] = extract_f32(bag_path, topic_accel);
    [t_odom, px, py, yaw_rad, speed_mps] = extract_odometry(bag_path, topic_odom);

    steer_wheel_deg = double(steer_raw);
    steer_wheel_deg = min(max(steer_wheel_deg, steer_min_raw), steer_max_raw);
    steer_road_deg = steer_wheel_deg / steer_ratio;
    steer_road_rad = deg2rad(steer_road_deg);

    brake_pct = min(max(double(brake_raw), 0), brake_raw_max) / brake_raw_max * 100;
    accel_pct = min(max(double(accel_raw), 0), accel_raw_max) / accel_raw_max * 100;

    t0 = min([t_steer(1), t_brake(1), t_accel(1), t_odom(1)]);
    t_steer = t_steer - t0;
    t_brake = t_brake - t0;
    t_accel = t_accel - t0;
    t_odom = t_odom - t0;

    t_start = max([t_steer(1), t_brake(1), t_accel(1), t_odom(1)]);
    t_end = min([t_steer(end), t_brake(end), t_accel(end), t_odom(end)]) - 1.0;
    if t_end <= t_start
        error(['No overlapping time range across bag topics.\n', ...
               'steer: [%s, %s]\n', ...
               'brake: [%s, %s]\n', ...
               'accel: [%s, %s]\n', ...
               'odom : [%s, %s]'], ...
               scalar_to_text(t_steer(1)), scalar_to_text(t_steer(end)), ...
               scalar_to_text(t_brake(1)), scalar_to_text(t_brake(end)), ...
               scalar_to_text(t_accel(1)), scalar_to_text(t_accel(end)), ...
               scalar_to_text(t_odom(1)), scalar_to_text(t_odom(end)));
    end

    t = (t_start:dt_resamp:t_end).';
    t_rel = t - t(1);

    bag.t = t_rel;
    bag.dt = dt_resamp;
    bag.steer_wheel_deg = resamp(t_steer, steer_wheel_deg, t);
    bag.steer_road_deg = resamp(t_steer, steer_road_deg, t);
    bag.steer_road_rad = resamp(t_steer, steer_road_rad, t);
    bag.brake_pct = resamp(t_brake, brake_pct, t);
    bag.accel_pct = resamp(t_accel, accel_pct, t);
    bag.speed_mps = resamp(t_odom, speed_mps, t);
    bag.px = resamp(t_odom, px, t);
    bag.py = resamp(t_odom, py, t);
    bag.yaw_rad = resamp(t_odom, yaw_rad, t);
    bag.a_actual_mps2 = gradient(bag.speed_mps, dt_resamp);
    bag.steer_ratio = steer_ratio;

    print_summary(bag);
    make_plots(bag, out_png);

    save(out_mat, '-struct', 'bag');
    fprintf('Saved MAT: %s\n', out_mat);
    fprintf('Saved plot: %s\n', out_png);
end

function print_summary(bag)
    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('  ROS Bag Summary\n');
    fprintf('============================================================\n');
    fprintf('  Duration           : %.1f s\n', bag.t(end) - bag.t(1));
    fprintf('  Samples            : %d\n', numel(bag.t));
    fprintf('  Sample time        : %.3f s\n', bag.dt);
    fprintf('  Steering road [rad]: %+.3f .. %+.3f\n', min(bag.steer_road_rad), max(bag.steer_road_rad));
    fprintf('  Throttle [%%]       : %.1f .. %.1f\n', min(bag.accel_pct), max(bag.accel_pct));
    fprintf('  Brake [%%]          : %.1f .. %.1f\n', min(bag.brake_pct), max(bag.brake_pct));
    fprintf('  Speed [m/s]        : %.3f .. %.3f\n', min(bag.speed_mps), max(bag.speed_mps));
    fprintf('  Accel [m/s^2]      : %.3f .. %.3f\n', min(bag.a_actual_mps2), max(bag.a_actual_mps2));
    fprintf('============================================================\n');
end

function make_plots(bag, out_png)
    fig = figure('Name', 'Kia Niro ROS Bag', 'Position', [80 80 1200 850], ...
        'Color', 'w');

    c.blue = [0.31 0.76 0.97];
    c.green = [0.51 0.78 0.52];
    c.amber = [1.00 0.72 0.30];
    c.red = [0.94 0.33 0.31];
    c.purple = [0.81 0.58 0.85];

    function ax = setup_ax(pos, ttl)
        ax = subplot(2, 2, pos);
        set(ax, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', ...
            'GridColor', [0.85 0.85 0.85], 'GridAlpha', 1, 'FontSize', 10);
        title(ax, ttl, 'Color', 'k', 'FontSize', 11, 'FontWeight', 'bold');
        grid(ax, 'on');
        hold(ax, 'on');
        box(ax, 'on');
    end

    ax1 = setup_ax(1, 'Vehicle Path');
    plot(ax1, bag.px, bag.py, 'Color', c.green, 'LineWidth', 1.3);
    plot(ax1, bag.px(1), bag.py(1), 'o', 'Color', c.green, 'MarkerFaceColor', c.green, 'MarkerSize', 7);
    plot(ax1, bag.px(end), bag.py(end), 's', 'Color', c.amber, 'MarkerFaceColor', c.amber, 'MarkerSize', 7);
    legend(ax1, 'Path', 'Start', 'End', 'TextColor', 'k', ...
        'Color', 'w', 'EdgeColor', [0.8 0.8 0.8], 'FontSize', 9, 'Location', 'best');
    xlabel(ax1, 'X (m)');
    ylabel(ax1, 'Y (m)');
    axis(ax1, 'equal');

    ax2 = setup_ax(2, 'Actual Acceleration');
    area(ax2, bag.t, max(bag.a_actual_mps2, 0), 'FaceColor', c.green, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    area(ax2, bag.t, min(bag.a_actual_mps2, 0), 'FaceColor', c.red, 'FaceAlpha', 0.25, 'EdgeColor', 'none');
    plot(ax2, bag.t, bag.a_actual_mps2, 'Color', c.purple, 'LineWidth', 1.2);
    yline(ax2, 0, 'Color', [0.5 0.5 0.5], 'LineWidth', 0.9);
    xlabel(ax2, 'Time (s)');
    ylabel(ax2, 'm/s^2');

    ax3 = setup_ax(3, 'Steering Angle');
    plot(ax3, bag.t, bag.steer_wheel_deg, 'Color', [0.85 0.85 0.85], 'LineWidth', 0.9);
    plot(ax3, bag.t, bag.steer_road_deg, 'Color', c.amber, 'LineWidth', 1.4);
    yline(ax3, 0, 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8);
    legend(ax3, 'Wheel Angle (deg)', 'Road Wheel Angle (deg)', 'TextColor', 'k', ...
        'Color', 'w', 'EdgeColor', [0.8 0.8 0.8], 'FontSize', 9, 'Location', 'best');
    xlabel(ax3, 'Time (s)');
    ylabel(ax3, 'deg');

    ax4 = setup_ax(4, 'Throttle & Brake');
    yyaxis(ax4, 'left');
    area(ax4, bag.t, bag.accel_pct, 'FaceColor', c.green, 'FaceAlpha', 0.20, 'EdgeColor', 'none');
    plot(ax4, bag.t, bag.accel_pct, 'Color', c.green, 'LineWidth', 1.2, 'DisplayName', 'Throttle (%)');
    area(ax4, bag.t, bag.brake_pct, 'FaceColor', c.red, 'FaceAlpha', 0.20, 'EdgeColor', 'none');
    plot(ax4, bag.t, bag.brake_pct, 'Color', c.red, 'LineWidth', 1.2, 'DisplayName', 'Brake (%)');
    ylabel(ax4, 'Pedal (%)');
    ylim(ax4, [-5 105]);
    set(ax4, 'YColor', 'k');
    yyaxis(ax4, 'right');
    plot(ax4, bag.t, bag.speed_mps, 'Color', c.blue, 'LineWidth', 1.1, 'DisplayName', 'Speed (m/s)');
    ylabel(ax4, 'Speed (m/s)');
    set(ax4, 'YColor', c.blue);
    xlabel(ax4, 'Time (s)');
    legend(ax4, 'TextColor', 'k', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8], ...
        'FontSize', 9, 'Location', 'best');

    sgtitle(fig, sprintf('Kia Niro DBW Log | %.0f s | %.2f m/s peak | 10 Hz resampled', ...
        bag.t(end) - bag.t(1), max(bag.speed_mps)), 'Color', 'k', 'FontSize', 13, 'FontWeight', 'bold');

    exportgraphics(fig, out_png, 'Resolution', 150);
    fprintf('Plot saved: %s\n', out_png);
end

function y_new = resamp(t_old, y_old, t_new)
    y_new = interp1(t_old, y_old, t_new, 'linear', 'extrap');
end

function ok = has_ros2_toolbox()
    ok = ~isempty(which('ros2bag'));
end

function bag_path = default_bag_path()
    candidates = { ...
        fullfile(pwd, 'kianirobag'), ...
        fullfile(pwd, 'data', 'kianirobag'), ...
        'kianirobag' ...
    };

    for i = 1:numel(candidates)
        if is_valid_bag_dir(candidates{i})
            bag_path = candidates{i};
            return;
        end
    end

    bag_path = 'kianirobag';
end

function bag_path = normalize_bag_path(bag_path)
    if ~isfolder(bag_path)
        error(['Bag folder not found: %s\n', ...
               'Current working directory: %s\n', ...
               'Expected a folder containing metadata.yaml and one or more .db3 files.'], ...
               bag_path, pwd);
    end

    if ~is_valid_bag_dir(bag_path)
        error(['Invalid ROS2 bag folder: %s\n', ...
               'The folder must contain metadata.yaml and at least one .db3 file.'], ...
               bag_path);
    end
end

function ok = is_valid_bag_dir(bag_path)
    ok = isfolder(bag_path) && ...
         isfile(fullfile(bag_path, 'metadata.yaml')) && ...
         ~isempty(dir(fullfile(bag_path, '*.db3')));
end

function [t, data] = extract_f32(bag_path, topic)
    bag = ros2bag(bag_path);
    sel = select(bag, 'Topic', topic);
    if sel.NumMessages == 0
        error('Topic %s has 0 messages in %s', topic, bag_path);
    end

    msgs = readMessages(sel);
    data = cellfun(@(m) double(m.data), msgs);
    stamps = sel.MessageList.Time;
    t = time_values_to_seconds(stamps);
end

function [t, px, py, yaw, speed] = extract_odometry(bag_path, topic)
    bag = ros2bag(bag_path);
    sel = select(bag, 'Topic', topic);
    if sel.NumMessages == 0
        error('Topic %s has 0 messages in %s', topic, bag_path);
    end

    msgs = readMessages(sel);
    n = numel(msgs);
    px = zeros(n, 1);
    py = zeros(n, 1);
    yaw = zeros(n, 1);
    speed = zeros(n, 1);

    for i = 1:n
        m = msgs{i};
        px(i) = m.pose.pose.position.x;
        py(i) = m.pose.pose.position.y;
        q = m.pose.pose.orientation;
        yaw(i) = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y^2 + q.z^2));
        speed(i) = m.twist.twist.linear.x;
    end

    stamps = sel.MessageList.Time;
    t = time_values_to_seconds(stamps);
end

function t = time_values_to_seconds(stamps)
    if isdatetime(stamps)
        t = posixtime(stamps);
    elseif isduration(stamps)
        t = stamps ./ seconds(1);
    else
        try
            t = posixtime(stamps);
        catch
            try
                if isduration(stamps)
                    t = stamps ./ seconds(1);
                else
                    t = seconds(stamps);
                end
            catch
                t = double(stamps);
            end
        end
    end
    if isduration(t)
        t = t ./ seconds(1);
    end
    t = double(t(:));
end

function txt = scalar_to_text(x)
    if isduration(x) || isdatetime(x)
        txt = char(string(x));
        return;
    end
    if isnumeric(x)
        txt = sprintf('%.6f s', x);
        return;
    end
    txt = char(string(x));
end
