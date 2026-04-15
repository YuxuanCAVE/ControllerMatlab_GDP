function ref = bag_to_ref(bag_mat_path, out_path)
% BAG_TO_REF  Convert resampled bag data into a simulation reference path.
%
%   ref = bag_to_ref()
%   ref = bag_to_ref(bag_mat_path)
%   ref = bag_to_ref(bag_mat_path, out_path)
%
% Input MAT is expected to come from read_rosbag_kia.m and contain:
%   px, py, speed_mps, t
%
% Output MAT is compatible with the current workspace:
%   x_opt, y_opt
% and also includes:
%   yaw, kappa, v_ref, ds_out

    if nargin < 1 || isempty(bag_mat_path)
        bag_mat_path = fullfile('data', 'bag_data_10hz.mat');
    end
    if nargin < 2 || isempty(out_path)
        out_path = fullfile('data', 'bag_path_ref.mat');
    end

    fprintf('Loading bag MAT: %s\n', bag_mat_path);
    D = load(bag_mat_path);

    px_raw = D.px(:);
    py_raw = D.py(:);
    speed_raw = D.speed_mps(:);
    t_raw = D.t(:);

    if numel(px_raw) < 10
        error('Bag path is too short to build a reference.');
    end

    fprintf('  Raw points: %d  duration: %.1f s\n', numel(px_raw), t_raw(end) - t_raw(1));

    ds_raw = hypot(diff(px_raw), diff(py_raw));
    keep = [true; ds_raw > 0.02];
    px_raw = px_raw(keep);
    py_raw = py_raw(keep);
    speed_raw = speed_raw(keep);

    px_sm = moving_average(px_raw, 7);
    py_sm = moving_average(py_raw, 7);

    ds_sm = hypot(diff(px_sm), diff(py_sm));
    arc_sm = [0; cumsum(ds_sm)];
    total_length = arc_sm(end);
    ds_target = 0.3;
    arc_new = (0:ds_target:total_length).';

    px = interp1(arc_sm, px_sm, arc_new, 'pchip');
    py = interp1(arc_sm, py_sm, arc_new, 'pchip');
    v_ref = interp1(arc_sm, speed_raw, arc_new, 'pchip');
    v_ref = max(v_ref, 0.1);

    yaw = compute_path_yaw(px, py);
    kappa = compute_path_curvature(px, py, yaw);

    [px, py, yaw, kappa, v_ref] = align_path_direction(px, py, yaw, kappa, v_ref, px_raw, py_raw);
    [px, py, yaw, kappa, v_ref] = repair_path_seam(px, py, yaw, kappa, v_ref);

    ref.x_opt = px(:);
    ref.y_opt = py(:);
    ref.yaw = yaw(:);
    ref.kappa = kappa(:);
    ref.v_ref = v_ref(:);
    ref.ds_out = ds_target * ones(size(ref.x_opt));
    ref.total_length = total_length;

    x_opt = ref.x_opt; %#ok<NASGU>
    y_opt = ref.y_opt; %#ok<NASGU>
    yaw = ref.yaw; %#ok<NASGU>
    kappa = ref.kappa; %#ok<NASGU>
    v_ref = ref.v_ref; %#ok<NASGU>
    ds_out = ref.ds_out; %#ok<NASGU>
    total_length = ref.total_length; %#ok<NASGU>

    save(out_path, 'x_opt', 'y_opt', 'yaw', 'kappa', 'v_ref', 'ds_out', 'total_length');
    fprintf('Saved reference: %s\n', out_path);

    plot_bag_reference(px_raw, py_raw, ref, out_path);
end

function y = moving_average(x, w)
    y = x;
    half = floor(w / 2);
    n = numel(x);
    for i = 1:n
        i0 = max(1, i - half);
        i1 = min(n, i + half);
        y(i) = mean(x(i0:i1));
    end
end

function yaw = compute_path_yaw(x, y)
    n = numel(x);
    yaw = zeros(n, 1);
    for i = 1:n-1
        yaw(i) = atan2(y(i+1) - y(i), x(i+1) - x(i));
    end
    yaw(end) = yaw(end-1);
    yaw = unwrap(yaw);
    yaw = moving_average(yaw, 5);
    yaw = angle_wrap_vec(yaw);
end

function kappa = compute_path_curvature(x, y, yaw)
    n = numel(x);
    kappa = zeros(n, 1);
    for i = 2:n-1
        dyaw = angle_wrap_scalar(yaw(i+1) - yaw(i-1));
        ds_i = 0.5 * (hypot(x(i+1) - x(i), y(i+1) - y(i)) + ...
            hypot(x(i) - x(i-1), y(i) - y(i-1)));
        kappa(i) = dyaw / max(ds_i * 2, 1e-6);
    end
    kappa(1) = kappa(2);
    kappa(end) = kappa(end-1);
    kappa = moving_average(kappa, 9);
end

function [x, y, yaw, kappa, v_ref] = align_path_direction(x, y, yaw, kappa, v_ref, x_raw, y_raw)
    cx = x - mean(x);
    cy = y - mean(y);
    winding = sum(cx(1:end-1) .* diff(cy) - cy(1:end-1) .* diff(cx));

    driver_yaw0 = atan2(mean(diff(y_raw(1:10))), mean(diff(x_raw(1:10))));
    if winding < 0
        fprintf('  Reversing waypoint order to match driver direction\n');
        x = flipud(x);
        y = flipud(y);
        yaw = flipud(angle_wrap_vec(yaw + pi));
        kappa = flipud(-kappa);
        v_ref = flipud(v_ref);
    end

    dist_pos = hypot(x - x_raw(1), y - y_raw(1));
    dist_yaw = abs(angle_wrap_vec(yaw - driver_yaw0));
    score = dist_pos / max(max(dist_pos), 1e-9) + 0.5 * dist_yaw / pi;
    [~, i0] = min(score);

    idx = [i0:numel(x), 1:i0-1].';
    x = x(idx);
    y = y(idx);
    yaw = yaw(idx);
    kappa = kappa(idx);
    v_ref = v_ref(idx);
end

function [x, y, yaw, kappa, v_ref] = repair_path_seam(x, y, yaw, kappa, v_ref)
    kappa_max = 0.08;
    spike_k = abs(kappa) > kappa_max;
    if any(spike_k)
        idx_ok = find(~spike_k);
        kappa(spike_k) = interp1(idx_ok, kappa(idx_ok), find(spike_k), 'linear', 'extrap');
        kappa = moving_average(kappa, 9);
        fprintf('  Removed %d curvature spike points\n', sum(spike_k));
    end

    spike_v = v_ref < 1.5;
    if any(spike_v)
        idx_ok = find(~spike_v);
        v_ref(spike_v) = interp1(idx_ok, v_ref(idx_ok), find(spike_v), 'linear', 'extrap');
        fprintf('  Interpolated %d low-speed points\n', sum(spike_v));
    end
    v_ref = moving_average(v_ref, 21);
    v_ref = max(v_ref, 1.5);

    yaw = angle_wrap_vec(yaw);
end

function plot_bag_reference(px_raw, py_raw, ref, out_path)
    fig = figure('Name', 'Bag Path Reference', 'Position', [100 100 1200 500], ...
        'Color', [0.08 0.08 0.12]);

    bg = [0.05 0.05 0.08];

    subplot(1, 3, 1);
    set(gca, 'Color', bg, 'XColor', [0.6 0.6 0.6], 'YColor', [0.6 0.6 0.6], ...
        'GridColor', [0.14 0.14 0.14], 'GridAlpha', 1);
    grid on;
    hold on;
    plot(px_raw, py_raw, '-', 'Color', [0.5 0.5 0.6], 'LineWidth', 0.8, 'DisplayName', 'Raw INS');
    plot(ref.x_opt, ref.y_opt, '-', 'Color', [0.5 0.9 0.5], 'LineWidth', 1.5, 'DisplayName', 'Reference');
    legend('TextColor', [0.8 0.8 0.8], 'Color', bg, 'EdgeColor', [0.2 0.2 0.2]);
    title('Path', 'Color', [0.88 0.88 0.88]);
    xlabel('X (m)');
    ylabel('Y (m)');
    axis equal;

    subplot(1, 3, 2);
    set(gca, 'Color', bg, 'XColor', [0.6 0.6 0.6], 'YColor', [0.6 0.6 0.6], ...
        'GridColor', [0.14 0.14 0.14], 'GridAlpha', 1);
    grid on;
    hold on;
    plot(ref.kappa, 'Color', [1.0 0.72 0.3], 'LineWidth', 1.0);
    yline(0, 'Color', [0.4 0.4 0.4], 'LineWidth', 0.8);
    title('Curvature (1/m)', 'Color', [0.88 0.88 0.88]);
    xlabel('Waypoint index');
    ylabel('1/m');

    subplot(1, 3, 3);
    set(gca, 'Color', bg, 'XColor', [0.6 0.6 0.6], 'YColor', [0.6 0.6 0.6], ...
        'GridColor', [0.14 0.14 0.14], 'GridAlpha', 1);
    grid on;
    hold on;
    plot(ref.v_ref, 'Color', [0.31 0.76 0.97], 'LineWidth', 1.0);
    title('Speed Profile (m/s)', 'Color', [0.88 0.88 0.88]);
    xlabel('Waypoint index');
    ylabel('m/s');

    sgtitle('bag_to_ref output', 'Color', 'white', 'FontSize', 10);
    out_png = strrep(out_path, '.mat', '.png');
    exportgraphics(fig, out_png, 'Resolution', 150);
    close(fig);
    fprintf('Saved reference plot: %s\n', out_png);
end

function y = angle_wrap_vec(x)
    y = mod(x + pi, 2 * pi) - pi;
end

function y = angle_wrap_scalar(x)
    y = mod(x + pi, 2 * pi) - pi;
end
