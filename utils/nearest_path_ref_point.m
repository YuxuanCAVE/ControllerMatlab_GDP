function [idx, xr, yr, psi_r, seg_idx, seg_t] = nearest_path_ref_point(x, y, x_ref, y_ref, idx_hint, window)
% Find the nearest reference waypoint on a path.
%
% Inputs:
%   x, y       - vehicle rear axle center position
%   x_ref      - reference path x coordinates
%   y_ref      - reference path y coordinates
%   idx_hint   - previous nearest index (optional)
%   window     - local search window size (optional)
%
% Outputs:
%   idx        - nearest waypoint index
%   xr, yr     - nearest waypoint coordinates
%   psi_r      - waypoint heading estimated from neighboring waypoints
%   seg_idx    - compatibility output; equal to idx
%   seg_t      - compatibility output; always 0 for waypoint mode

    n = numel(x_ref);
    if n < 2
        error('Path must contain at least 2 waypoints.');
    end

    if nargin < 5 || isempty(idx_hint)
        d2 = (x_ref - x).^2 + (y_ref - y).^2;
        [~, idx] = min(d2);
    else
        if nargin < 6 || isempty(window)
            window = 40;
        end

        i0 = max(1, idx_hint - window);
        i1 = min(n, idx_hint + window);

        d2 = (x_ref(i0:i1) - x).^2 + (y_ref(i0:i1) - y).^2;
        [~, k] = min(d2);
        idx = i0 + k - 1;
    end

    xr = x_ref(idx);
    yr = y_ref(idx);
    seg_idx = idx;
    seg_t = 0.0;

    if idx == 1
        psi_r = atan2(y_ref(2) - y_ref(1), x_ref(2) - x_ref(1));
    elseif idx == n
        psi_r = atan2(y_ref(n) - y_ref(n-1), x_ref(n) - x_ref(n-1));
    else
        psi_r = atan2(y_ref(idx+1) - y_ref(idx-1), x_ref(idx+1) - x_ref(idx-1));
    end
end
